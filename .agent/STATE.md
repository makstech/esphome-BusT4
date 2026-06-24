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

## Next action
1. Push branch, open CI, confirm green (config + unit-tests + compile).
2. Open PR against makstech/esphome-BusT4:main (additive feature; include capture format).
3. Iterate photocell decode once Petr sends 0xD0 captures.
