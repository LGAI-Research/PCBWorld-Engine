#!/usr/bin/python3
"""Batch driver for kicad_to_orp inside the pcbnew container.

Reads a JSON manifest ``[{"pcb": "<in.kicad_pcb>", "orp": "<out.orp>",
"pro": "<optional .kicad_pro>"}...]`` and converts each board in a single
process, so one ``docker run`` covers a whole dataset instead of paying the
container-startup cost per board.

Runs *inside* ``pcbnew-repro:9.0`` (or any env where ``import pcbnew`` works).
``kicad_to_orp`` lives next to this file in ``baselines/OrthoRoute/`` — we add
that directory to ``sys.path`` so no PYTHONPATH is needed.

Usage (inside container):
  python3 batch_kicad_to_orp.py /out/_manifest.json
"""
import json
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
# kicad_to_orp.py is under baselines/OrthoRoute/
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
