#!/usr/bin/python3
"""Batch driver for kicad_to_orp: one process converts a whole dataset.

Reads a JSON manifest ``[{"pcb": "<in.kicad_pcb>", "orp": "<out.orp>",
"pro": "<optional .kicad_pro>"}...]`` and converts each board in a single
process, so the pcbnew import is paid once instead of per board.

Runs in any interpreter where ``import pcbnew`` works — the engine's own build
(``PYTHONPATH=build_rl/pcbnew``) or a host KiCad's python. ``kicad_to_orp`` is
looked up next to this file's parent in ``OrthoRoute/``.

Usage:
  PYTHONPATH=build_rl/pcbnew python batch_kicad_to_orp.py <manifest.json>
"""
import json
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
# kicad_to_orp.py is NOT shipped in this tree (a leftover of the original prep tree):
# this driver fails at import until it is restored. The rule-based pipeline normally
# skips it entirely by reading the pre-converted mirrors (--orp-root / *_dsn datasets).
sys.path.insert(0, str(_HERE.parent / "OrthoRoute"))
import kicad_to_orp  # noqa: E402


def main():
    if len(sys.argv) < 2:
        print("usage: batch_kicad_to_orp.py <manifest.json>", file=sys.stderr)
        sys.exit(2)
    manifest = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    ok = 0
    fail = 0
    for i, item in enumerate(manifest):
        pcb = Path(item["pcb"])
        orp = Path(item["orp"])
        pro = Path(item["pro"]) if item.get("pro") else None
        tag = item.get("board_id", pcb.stem)
        try:
            kicad_to_orp.convert(pcb, orp, pro)
            ok += 1
            print(f"OK [{i+1}/{len(manifest)}] {tag} -> {orp}", flush=True)
        except Exception as exc:  # noqa: BLE001
            fail += 1
            print(f"FAIL [{i+1}/{len(manifest)}] {tag}: {exc}", flush=True)
    print(f"BATCH_DONE ok={ok} fail={fail}", flush=True)
    sys.exit(1 if fail and ok == 0 else 0)


if __name__ == "__main__":
    main()
