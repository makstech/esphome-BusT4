# PLAN — next steps (ordered)

Each step lists its verify check (CLAUDE.md §4).

1. **Shared packet helpers + debug logger** (Task #2)
   - Add `const`/`constexpr` accessors to `T4Packet` in `t4_packet.h`
     (`is_dmp/is_dep`, `device()`, `command()`, `flags()`, `payload()`, `payload_len()`,
     `is_event_push()` for EVT). Header-only, no behaviour change.
   - `bus_t4: debug_unknown_packets: true` config flag. When set, the component logs every
     RX packet, classified (known DEP-RUN / DMP-response / OXI / diagnostics / unknown) with
     hex + a guessed label. Never throws.
   - Verify: `esphome config debug_yaml` passes; logger code compiles (CI build).

2. **Keypad event platform** (Task #3) — `components/bus_t4/event/`
   - `BusT4KeypadEvent : event::Event, BusT4Device, Component`. On RUN packet with
     `data[9] >= 0x80`, map cmd→event_type and `trigger()`.
   - Optional self-command suppression window.
   - Verify: unit test feeds crafted RUN bytes → expected event_type; `esphome config`.

3. **Photocell binary_sensor platform** (Task #4) — `components/bus_t4/binary_sensor/`
   - `BusT4Photocell : binary_sensor::BinarySensor, BusT4Device, Component`, config
     `address: 1|2|3`, `name`.
   - Component gains optional periodic `0xD0` poll when ≥1 photocell is registered
     (configurable interval, default off-until-registered).
   - Parse: isolated, well-commented `decode_*` with the uncertain offset flagged
     (PROTOCOL §3c). Also set "blocked" transiently on PHOTO_INTERVENTION.
   - Verify: unit test for the decode against a documented sample; `esphome config`.

4. **Device-list text_sensor + docs + tests + example** (Task #5)
   - `components/bus_t4/text_sensor/` diagnostic list of recognized bus devices.
   - `example.yaml` extended (commented) + new `debug_yaml` for captures.
   - README "BlueBus events" section: config reference + caveats (Robus, what we can/can't see).
   - `tests/` host-compilable parser unit tests.
   - Verify: `esphome config example.yaml` + `esphome config debug_yaml` pass; tests pass.

5. **CI**: add `esphome config debug_yaml` to ci.yml validate job (cheap). Optionally a
   tiny host build for `tests/`.

6. Open PR against `makstech/esphome-BusT4:main` once green; include capture format.

## Guardrails (from brief + CLAUDE.md)
- Backward compatible: existing cover/button/switch/lock/number/text configs unchanged.
- No new external libraries. Robust on unknown packets (DEBUG log + continue).
- Read-only on BlueBus; only poll the controller over Bus-T4. Don't regress Robus handling.
- Don't guess in code — uncertain offsets are isolated + documented in PROTOCOL.md.
