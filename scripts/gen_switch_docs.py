#!/usr/bin/env python3
"""Generate the control-unit type tables in README.md from the registries in
components/bus_t4_control_unit/__init__.py (CONFIG_TYPES, NUMBER_TYPES,
SELECT_TYPES, SENSOR_TYPES, BUTTON_TYPES).

Usage:
    python scripts/gen_switch_docs.py          # print all tables to stdout
    python scripts/gen_switch_docs.py --patch   # update README.md in-place
"""
import argparse
import ast
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
INIT = ROOT / "components" / "bus_t4_control_unit" / "__init__.py"
README = ROOT / "README.md"


def _const(node):
    return node.value if isinstance(node, ast.Constant) else None


def _entries(dict_name):
    """Yield (key, [constant-or-None per tuple element]) for a top-level dict
    literal. Non-constant elements (e.g. _out_opts(...) calls) read as None."""
    tree = ast.parse(INIT.read_text())
    for node in ast.iter_child_nodes(tree):
        if isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id == dict_name for t in node.targets
        ):
            for k, v in zip(node.value.keys, node.value.values):
                elts = [_const(e) for e in v.elts] if isinstance(v, ast.Tuple) else []
                yield _const(k), elts
            return
    print(f"ERROR: {dict_name} not found in {INIT.name}", file=sys.stderr)
    sys.exit(1)


def _md_table(header, rows):
    out = ["| " + " | ".join(header) + " |", "|" + "|".join("---" for _ in header) + "|"]
    for r in rows:
        out.append("| " + " | ".join(str(c) for c in r) + " |")
    return "\n".join(out)


def build_flags():
    rows = [[f"`{k}`", e[1]] for k, e in _entries("CONFIG_TYPES")]
    return _md_table(["`type`", "Description"], rows)


def build_numbers():
    rows = []
    for k, e in _entries("NUMBER_TYPES"):
        lo, hi, name, unit = e[1], e[2], e[5], e[4] or ""
        rows.append([f"`{k}`", name, f"{lo:g}–{hi:g} {unit}".strip()])
    return _md_table(["`type`", "Description", "Range"], rows)


def build_selects():
    rows = [[f"`{k}`", e[1]] for k, e in _entries("SELECT_TYPES")]
    return _md_table(["`type`", "Description"], rows)


def build_sensors():
    rows = [[f"`{k}`", e[1]] for k, e in _entries("SENSOR_TYPES")]
    return _md_table(["`type`", "Description"], rows)


def build_buttons():
    rows = [[f"`{k}`", e[2]] for k, e in _entries("BUTTON_TYPES")]
    return _md_table(["`type`", "Description"], rows)


TABLES = [
    ("SWITCH_TYPES", build_flags),
    ("NUMBER_TYPES", build_numbers),
    ("SELECT_TYPES", build_selects),
    ("SENSOR_TYPES", build_sensors),
    ("BUTTON_TYPES", build_buttons),
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--patch", action="store_true", help="update README.md in-place")
    args = parser.parse_args()

    if not args.patch:
        for marker, build in TABLES:
            print(f"# {marker}\n{build()}\n")
        return

    text = README.read_text()
    for marker, build in TABLES:
        start, end = f"<!-- BEGIN {marker} -->", f"<!-- END {marker} -->"
        pattern = re.compile(re.escape(start) + r".*?" + re.escape(end), re.DOTALL)
        if not pattern.search(text):
            print(f"ERROR: missing {start} / {end} markers in README.md", file=sys.stderr)
            sys.exit(1)
        block = f"{start}\n{build()}\n{end}"
        text = pattern.sub(lambda m: block, text)
    README.write_text(text)
    print("README.md updated.")


if __name__ == "__main__":
    main()
