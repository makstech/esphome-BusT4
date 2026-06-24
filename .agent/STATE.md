# STATE — current cursor

**Branch:** `claude/quizzical-williamson-f97fe1` (fork of makstech/esphome-BusT4)

**Phase:** Implementation complete → PR + real-hardware capture loop with Petr.

## What is true right now
- All five planned deliverables are implemented and committed (see LEDGER.md):
  PROTOCOL.md, packet helpers + debug logger, keypad `event`, photocell `binary_sensor`
  (EXPERIMENTAL), device-list `text_sensor`, docs, example/debug YAML, unit tests + CI.
- Local verification: every YAML passes `esphome config`; the host unit test passes
  (`zig c++`). Full firmware C++ compile is delegated to CI (`esphome compile example.yaml`),
  which now includes all four platforms — not yet run/confirmed green here.
- Backward compatible: existing cover/button/switch/lock/number/text configs untouched.

## Known unknowns (need Petr's captures — tracked in PROTOCOL.md §6)
1. 0xD0 BlueBus diagnostics payload layout → photocell decode constant (currently a GUESS).
2. PHOTO_INTERVENTION read site.
3. Whether the controller echoes our own commands (keypad event self-fire) → echo suppression.

## Verification status (important)
- `esphome config` (example.yaml + debug_yaml): **pass**.
- Host unit test of the packet parser (`tests/test_t4_packet.cpp`): **pass** (zig c++).
- ESPHome **codegen**: validated end-to-end — all four entities are constructed and wired in
  the generated `main.cpp` (`set_event_types({...})` matches the emitted strings, set_parent/
  set_address/etc. correct).
- Full firmware **C++ compile**: NOT verified locally. The esp-idf build on this Czech-locale,
  Python‑3.14 Windows box dies in platformio/esptool setup (UnicodeDecodeError on cp1250 bytes;
  tool-esptoolpy packaging error) BEFORE reaching the bus_t4 sources — an environment bug, not a
  code issue. CI (Ubuntu, Python 3.12, UTF‑8) is the designated compile gate and `esphome
  compile example.yaml` there exercises all four platforms.

## Next action
1. Decision point: push branch to the user's fork + open PR vs. hand off (awaiting user go —
   the local compile gate I set could not run for environment reasons).
2. CI confirms the compile on push/PR.
3. Iterate photocell 0xD0 decode once Petr sends captures (PROTOCOL.md §6).
