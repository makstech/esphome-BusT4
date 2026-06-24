# LEDGER — completed work

## 2026-06-24

- **Investigation phase complete.**
  - Read all makstech source (`bus_t4.{h,cpp}`, `bus_t4_component.{h,cpp}`, `t4_packet.h`,
    `cover/`). Mapped packet dispatch and the OXI handler (and found it is off-by-2 / buggy).
  - Cloned + cross-referenced forks: pruwait, xdanik (English), karol27 (closest existing,
    has `DIAG_BB`/`send_inf_cmd`), gashtaan (BiDi-WiFi firmware, richest string tables).
  - Decoded the flags byte from gashtaan `T4Flags`; reconciled makstech's magic constants.
  - Verified the keypad/command mechanism (DEP RUN packet, command echo `data[9] >= 0x80`).
  - Established that per-photocell beam layout (0xD0) is undocumented in every fork → debug
    logger must ship first to capture it.
  - Wrote `.agent/PROTOCOL.md` (packet tables, byte layouts, confidence levels, open
    questions for Petr).
  - Set up local toolchain: ESPHome 2026.6.2 venv for `esphome config` validation.

- **Implementation complete.**
  - `t4_packet.h`: shared bounds-aware accessors (is_dmp/is_dep, device/command,
    dmp_flags/sequence/status, is_event_push, is_run_packet/run_command_echo), T4Flag bit
    enum, INF_DIAG_BB (0xD0). Moved T4CommandPacket above the struct.
  - Component: `debug_unknown_packets` classifier/logger; shared `controller_address_`.
  - `event/`: BusT4KeypadEvent — fires on RUN command echoes (verified).
  - `binary_sensor/`: BusT4Photocell — polls 0xD0, isolated EXPERIMENTAL decode + raw logging.
  - `text_sensor/`: BusT4DeviceList — passive diagnostic list of Bus-T4 endpoints.
  - `example.yaml` extended; `debug_yaml` capture config; README "BlueBus events" section.
  - `tests/test_t4_packet.cpp` — host unit tests for the parser; **pass locally** (zig c++)
    and wired into CI (new `unit-tests` job + `esphome config debug_yaml`).
  - Every config validated with `esphome config`. C++ integration compile is left to CI
    (`esphome compile example.yaml`), which now exercises all four platforms.
