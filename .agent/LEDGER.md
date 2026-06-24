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
