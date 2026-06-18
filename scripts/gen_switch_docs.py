#!/usr/bin/env python3
"""Generate the switch-type markdown table from CONFIG_TYPES.

Usage:
    python scripts/gen_switch_docs.py          # print to stdout
    python scripts/gen_switch_docs.py --patch   # update README.md in-place
"""
import argparse
import ast
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SWITCH_INIT = ROOT / "components" / "bus_t4" / "switch" / "__init__.py"

START = "<!-- BEGIN SWITCH_TYPES -->"
END = "<!-- END SWITCH_TYPES -->"


def load_config_types() -> dict:
    """Extract CONFIG_TYPES from __init__.py using ast (no esphome import needed)."""
    tree = ast.parse(SWITCH_INIT.read_text())
    for node in ast.iter_child_nodes(tree):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == "CONFIG_TYPES":
                    return ast.literal_eval(node.value)
    print("ERROR: CONFIG_TYPES not found in __init__.py", file=sys.stderr)
    sys.exit(1)


def build_table(config_types: dict) -> str:
    lines = [
        "| `type` | Description |",
        "|--------|-------------|",
    ]
    for name, (_, desc) in config_types.items():
        lines.append(f"| `{name}` | {desc} |")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--patch", action="store_true", help="update README.md in-place")
    args = parser.parse_args()

    table = build_table(load_config_types())

    if not args.patch:
        print(table)
        return

    readme = ROOT / "README.md"
    text = readme.read_text()
    pattern = re.compile(rf"({re.escape(START)})\n.*?\n({re.escape(END)})", re.DOTALL)
    if not pattern.search(text):
        print(f"ERROR: missing {START} / {END} markers in README.md", file=sys.stderr)
        sys.exit(1)
    text = pattern.sub(rf"\1\n{table}\n\2", text)
    readme.write_text(text)
    print("README.md updated.")


if __name__ == "__main__":
    main()
