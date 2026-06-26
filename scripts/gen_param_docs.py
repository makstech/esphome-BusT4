#!/usr/bin/env python3
"""Generate the control-unit parameter tables in README.md from the PARAMS
registry in components/bus_t4_control_unit/__init__.py, grouped by domain
(switch / number / select / sensor / button).

Usage:
    python scripts/gen_param_docs.py           # print all tables to stdout
    python scripts/gen_param_docs.py --patch    # update README.md in-place
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


def _params(domain):
    """Yield (name, {field: constant-or-None}) for PARAMS rows of `domain`.
    Non-constant field values (e.g. options=_out_opts(...)) read as None."""
    tree = ast.parse(INIT.read_text())
    for node in ast.iter_child_nodes(tree):
        if isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id == "PARAMS" for t in node.targets
        ):
            for k, v in zip(node.value.keys, node.value.values):
                if not isinstance(v, ast.Dict):
                    continue
                row = {_const(kk): _const(vv) for kk, vv in zip(v.keys, v.values)}
                if row.get("domain") == domain:
                    yield _const(k), row
            return
    print(f"ERROR: PARAMS not found in {INIT.name}", file=sys.stderr)
    sys.exit(1)


def _md_table(header, rows):
    out = ["| " + " | ".join(header) + " |", "|" + "|".join("---" for _ in header) + "|"]
    for r in rows:
        out.append("| " + " | ".join(str(c) for c in r) + " |")
    return "\n".join(out)


def build_switches():
    rows = [[f"`{k}`", r["name"]] for k, r in _params("switch")]
    return _md_table(["`param`", "Description"], rows)


def build_numbers():
    rows = []
    for k, r in _params("number"):
        unit = r.get("unit") or ""
        rows.append([f"`{k}`", r["name"], f"{r['min']:g}–{r['max']:g} {unit}".strip()])
    return _md_table(["`param`", "Description", "Range"], rows)


def build_selects():
    rows = [[f"`{k}`", r["name"]] for k, r in _params("select")]
    return _md_table(["`param`", "Description"], rows)


def build_sensors():
    rows = [[f"`{k}`", r["name"]] for k, r in _params("sensor")]
    return _md_table(["`param`", "Description"], rows)


def build_buttons():
    rows = [[f"`{k}`", r["name"]] for k, r in _params("button")]
    return _md_table(["`param`", "Description"], rows)


TABLES = [
    ("SWITCH_TYPES", build_switches),
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
