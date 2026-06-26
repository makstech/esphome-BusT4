# Nice Bus-T4 — packet types carrying BlueBus events

Reverse-engineering notes for the "BlueBus events" feature (photocells + keypad).
Sources are cited per row. Confidence levels:

- **VERIFIED** — byte layout cross-checked across ≥2 independent forks and consistent
  with the makstech `T4Packet` struct; safe to implement.
- **SUSPECTED** — described in one fork or in the gashtaan firmware string tables, but
  the exact byte offset/encoding has not been confirmed against a real capture.
- **GUESS** — inferred from naming/structure only; MUST be confirmed with a capture
  from Petr's hardware before being trusted. Never silently relied on in code.

> The whole point of the `debug_unknown_packets` logger (see below) is to turn
> SUSPECTED/GUESS rows into VERIFIED ones using real captures.

---

## 1. Wire framing (VERIFIED)

A frame on the wire is:

```
[BREAK] 0x55 SIZE  <DATA[SIZE]>  SIZE
         sync  len      body     len(repeated)
```

The makstech RX state machine (`bus_t4_component.cpp`) strips the BREAK, `0x55` sync
and the leading/trailing `SIZE` bytes, so **`T4Packet::data[]` begins at the first body
byte** (`header.to`). All offsets in this document are indices into that `data[]` array.

### Canonical `data[]` layout

| idx | field | notes |
|-----|-------|-------|
| 0 | `header.to.address`   | "series"/row of destination |
| 1 | `header.to.endpoint`  | address of destination |
| 2 | `header.from.address` | source series |
| 3 | `header.from.endpoint`| source address |
| 4 | `header.protocol`     | `DEP=0x01` (command) or `DMP=0x08` (info) |
| 5 | `header.messageSize`  | payload length + 1 |
| 6 | `header.checksum`     | XOR of data[0..5] (CRC1) |
| 7 | `message.device`      | target sub-device (`FOR_ALL=0x00`, `FOR_CU=0x04`, `FOR_OXI=0x0A`) |
| 8 | `message.command`     | command / INF id |
| 9 | DMP: `flags` / DEP: first data byte | see below |
| 10 | DMP: `sequence` | offset/length |
| 11 | DMP: `status` | `0x00 = NOERR`, `0xFD = unsupported` |
| 12+ | DMP: payload | |
| last | CRC2 | XOR of data[7..last-1] |

> **Index mapping to the original forks (pruwait/xdanik/karol27):** those forks keep
> the leading `0x55` and `SIZE` bytes in their buffer, so **`fork_data[i] == makstech_data[i-2]`**.
> e.g. the fork's `data[9]` ("whose"/device) is our `data[7]`; fork `data[11]` is our `data[9]`.
> This offset is the #1 source of confusion when porting fork code — keep it in mind.

### Flags byte (`data[9]` on DMP) — decoded from gashtaan `T4Flags` (VERIFIED)

gashtaan/firmware/t4.h defines the bitfield that explains makstech's "magic" constants:

```
REQ = 0x80   EVT = 0x40   SET = 0x20   GET = 0x10   ACK = 0x08   FIN = 0x01
```

| value | = bits | meaning | makstech name |
|-------|--------|---------|---------------|
| 0x99 | REQ\|GET\|ACK\|FIN | our outgoing GET request | `REQ_GET` |
| 0xA9 | REQ\|SET\|ACK\|FIN | our outgoing SET request | `REQ_SET` |
| 0x89 | REQ\|ACK\|FIN | info/"get supported" request | `REQ_GET_SUPP` |
| 0x19 | GET\|ACK\|FIN | GET response, final | `RSP_GET_COMPLETE` |
| 0x18 | GET\|ACK | GET response, more fragments | `RSP_GET_INCOMPLETE` |
| 0x29 | SET\|ACK\|FIN | SET response, final | `RSP_SET_COMPLETE` |

The **`EVT` (0x40)** bit marks a spontaneous event/notification rather than a
request/response. (SUSPECTED that controller-pushed BlueBus events use it; not yet
observed in a capture — see §4.)

---

## 2. Keypad / command events (VERIFIED mechanism)

When a command is **executed** by the control unit — whatever the source (wired keypad
on BlueBus, hardwired input, OXI remote relayed onto the bus, or our own ESP) — the
controller emits a **DEP (`protocol=0x01`) "RUN" packet** onto Bus-T4. This is how
OView/other bus devices stay in sync, and it is what we passively listen for.

Layout of a RUN packet (makstech `data[]` indices):

| idx | value | meaning |
|-----|-------|---------|
| 4 | `0x01` | `DEP` protocol |
| 7 | `0x04` | `message.device` = `FOR_CU` / `CONTROL` |
| 8 | `0x02` | `message.command` = `RUN - 0x80` (pruwait: *"for CMD packets only RUN was seen"*) |
| 9 | **command echo or status** | **≥ 0x80 → a command was issued; < 0x80 → movement status** |

**Command echo (`data[9] >= 0x80`)** → the command that was executed is `data[9] - 0x80`:

| `data[9]` | cmd value | command |
|-----------|-----------|---------|
| 0x81 | 0x01 `CMD_STEP`          | Step-by-step |
| 0x82 | 0x02 `CMD_STOP`          | Stop |
| 0x83 | 0x03 `CMD_OPEN`          | Open |
| 0x84 | 0x04 `CMD_CLOSE`         | Close |
| 0x85 | 0x05 `CMD_OPEN_PARTIAL_1`| Partial 1 |
| 0x86 | 0x06 `CMD_OPEN_PARTIAL_2`| Partial 2 |
| 0x87 | 0x07 `CMD_OPEN_PARTIAL_3`| Partial 3 |

**Movement status (`data[9] < 0x80`)** → already handled by the cover:
`STA_OPENING=0x02, STA_CLOSING=0x03, STA_OPENED=0x04, STA_CLOSED=0x05, STA_ENDTIME=0x06,
STA_STOPPED=0x08, STA_PART_OPENED=0x10`.

- **Sources:** xdanik `parse_status_packet` "RSP package received" branch; karol27
  `nice-bust4.cpp` ~L665–751 (clean `if (data[11] >= 0x80)` split); pruwait L150 comment;
  command enum cross-checked in all three headers (`SBS=1..P_OPN3=7`).
- **makstech gap:** `parse_dep_packet()` only switches on the `< 0x80` statuses; it
  never decodes the `>= 0x80` command echoes. That is exactly the keypad-event signal
  we add.

### Caveat — source attribution is NOT possible
The RUN packet carries *which* command, not *who* issued it. A keypad "Open", a remote
"Open", and our own `cover.open` all look identical on the bus. Per the brief, user/source
identification is out of scope (BlueBus carries only ~4 bits and the controller does not
forward the origin). The keypad `event` entity therefore reports the **command**, and may
also fire for our own ESP-issued commands. Whether the controller actually echoes our own
commands as a `>= 0x80` command echo (vs only as a `< 0x80` status) is itself unconfirmed —
so echo-suppression is intentionally NOT implemented yet to avoid speculative code; a
capture (open the gate from HA, watch for a duplicate `[pkt] COMMAND open`) will tell us
whether it is needed.

---

## 3. Photocells

There is **no fork that decodes per-photocell beam state**, so this is the hard part and
the main reason the debug logger ships first. Three signals exist, in decreasing confidence:

### 3a. Photo intervention during a maneuver (SUSPECTED)
gashtaan `T4ManoeuvreStatusStrings` (t4.h L201) gives the *reason a maneuver ended*:

```
0 OK              1 ERROR_ON_BLUEBUS      2 PHOTO_INTERVENTION   3 OBSTACLE_DETECTED
4 HALT_DETECTED   5 INTERNAL_PARAMS_ERR   6 MAX_MANEUVERS_HOUR   7 ELECTRIC_ANOMALY
8 BLOCKING_CMD    9 BLOCKED_AUTOMATION    10 DETECTED_OBSTACLE_BY_ENCODER
```

This tells us a photocell *intervened*, but only at maneuver end and **not which cell**.
The exact packet/byte carrying this index is **not pinned down** in any fork (gashtaan
defines the strings but the sample doesn't show the read site). → capture needed.
Useful as a coarse "photo blocked the gate" indication, not for idle real-time state.

### 3b. I/O diagnostics `INF_IO = 0xD1` (PARTIAL — VERIFIED layout, mapping config-dependent)
makstech already polls `0xD1` (only reading `data[16]` = limit switch). gashtaan
`web.cpp` (L420+) decodes the full response. Payload starts at our `data[12]`
(gashtaan `data[0]` = our `data[12]`):

| payload byte | bits |
|--------------|------|
| 0 | b0 Input halt, b1 **Input 1 PP** (step), b2 **Input 2 AP** (open), b3 **Input 3 CH** (close), b4 Loop1, b5 Loop2 |
| 1 | b0 Button1, b1 Button2, b2 Button3 |
| 2 | b0 Fca M1 (open limit), b1 Fcc M1 (close limit), b2 Fca M2, b3 Fcc M2, … |
| 4 | outputs M1/M2, out1/2/3, fan, green/red light |
| 10 | b0 Error positions, **b1 Error BlueBus**, b2 Error halt, b3 Error function, … |

The photocells are wired into the logical **AP/CH/PP** inputs (which one depends on the
installer's command-mapping menu), and a BlueBus fault raises **Error BlueBus** (byte 10 b1).
This is real, but "input 2 AP active" ≠ "photocell N beam broken" without knowing the
config. Reliable as a **"BlueBus fault"** diagnostic; weak as a per-cell beam sensor.

### 3c. BlueBus device diagnostics `DIAG_BB = 0xD0` (GUESS — layout unknown)
`CTRL_DIAGNOSTICS_BLUEBUS_DEVICES = 0xD0` (gashtaan t4.h L474; karol27/pruwait `DIAG_BB`).
This is the controller's per-device view of everything on the BlueBus (photocells FT,
keypads MOTB/EDSB, etc.) and is the **right source for real-time per-cell beam state**.
**No fork decodes its payload** and gashtaan's web UI leaves it generic. The byte layout
is a GUESS until captured. Plan: poll `0xD0` periodically when photocell sensors are
configured, log the raw payload via the debug logger, and have Petr capture
beam-clear vs beam-blocked to map the offset.

### Photocell naming (reference)
Nice command/menu names: `PHOTO` (FOTO, the main closing safety), `PHOTO1` (FOTO1,
opening safety), `PHOTO2`, `PHOTO3`, plus `FOTOTEST`. Command codes
`CMD_PHOTO=35, CMD_PHOTO1=36, CMD_PHOTO2=37, CMD_PHOTO3=38` and menu
`COMM_PHOTO=0x68, COMM_PHOTO2=0x69, COMM_PHOTO3=0x6A` (gashtaan/karol27). We expose a
configurable `address: 1|2|3` → PHOTO / PHOTO2 / PHOTO3.

---

## 4. Does the controller PUSH events, or must we poll?

- **Status / RUN packets (§2):** broadcast by the controller in real time whenever the
  gate state changes, including external operation. **Passive listening works.** (This is
  how makstech already tracks a gate opened by remote.) VERIFIED.
- **Idle photocell state (§3c):** likely **not** broadcast while the gate is idle (the
  controller only reacts to photos during a maneuver / photo-test). Real-time idle
  monitoring therefore needs **periodic polling of 0xD0/0xD1 over Bus-T4**. That is a
  Bus-T4 request to the controller (allowed) — *not* probing the BlueBus wire (forbidden:
  a second master on BlueBus breaks the gate). SUSPECTED; confirm cadence with captures.

---

## 5. OXI receiver packets (context; makstech bug noted)

Remote-control events arrive as DMP packets from the OXI/RADIO device (our `data[7]==0x0A`):
- `0x25` remote list, `0x26` button read (verified offsets in xdanik L360–368 /
  karol27 L652–656).

⚠️ **makstech `parse_oxi_packet()` is off by two bytes**: it triggers on `data[9]==FOR_OXI`
and reads the command at `data[10]`, but per the canonical layout the OXI device byte is
`data[7]` and the command is `data[8]` (its sibling `parse_dmp_packet()` uses the correct
struct fields). The OXI handler therefore likely never fires correctly. We do **not** rely
on it; new event parsing uses the verified offsets. (Left untouched — out of scope to fix.)

---

## 6. Open questions to resolve with captures (for Petr)

1. **0xD0 payload layout** — block/unblock each photocell, capture the delta. (§3c, GUESS)
2. **PHOTO_INTERVENTION read site** — which packet/byte carries the manoeuvre-end reason? (§3a)
3. **Keypad vs remote distinction** — confirm a wired-keypad press produces the §2 RUN echo
   and whether any byte differs from a remote-issued command. (§2)
4. **EVT (0x40) push** — do any spontaneous EVT-flagged packets appear on the bus during
   photocell/keypad activity? (§1, §4)

See `debug_yaml`/`example.yaml` and the README "Capturing packets" section for the exact
steps and the format to send captures back.
