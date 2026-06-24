# STATE — current cursor

**Branch:** `claude/quizzical-williamson-f97fe1` (fork of makstech/esphome-BusT4)

**Phase:** Investigation complete → implementation.

## What is true right now
- makstech architecture mapped. Dispatch: `BusT4Component::loop()` pulls from `rxQueue_`
  and calls `device->on_packet(packet)` on every registered `BusT4Device`. New entity
  platforms subclass `BusT4Device` + `Component` and get every packet — additive, no core
  changes needed for dispatch.
- Canonical packet layout + flags decode documented in `.agent/PROTOCOL.md`.
- Keypad/command mechanism is VERIFIED (DEP RUN packet, `data[9] >= 0x80` ⇒ command echo).
- Photocell per-cell beam layout (0xD0) is UNKNOWN → debug logger ships first.
- ESPHome 2026.6.2 installed in `%TEMP%/esphome-venv` for local `esphome config` validation.
  Forks cloned at `%TEMP%/bust4_forks/{pruwait,xdanik,karol27,gashtaan}`.

## Next action
Implement Task #2: shared `T4Packet` accessor helpers in `t4_packet.h` + the
`debug_unknown_packets` logger in the component.

See `PLAN.md` for the ordered steps and `LEDGER.md` for completed work.
