#!/usr/bin/python3
"""
synth_2L_v2/{split}/board_XXXXX.kicad_pcb + .kicad_pro →
synth_2L_v2_dsn/{split}/board_XXXXX_unrouted.dsn + board_XXXXX.orp

split: test / train / val

Usage:
  # Test with 5 samples
  /usr/bin/python3 make_dsn_orp_synth.py --limit 5

  # Specific splits only
  /usr/bin/python3 make_dsn_orp_synth.py --splits test val

  # Full run
  /usr/bin/python3 make_dsn_orp_synth.py

  # Save result JSON
  /usr/bin/python3 make_dsn_orp_synth.py --report /tmp/synth_result.json
"""

import argparse
import gzip
import json
import shutil
import sys
import tempfile
from datetime import datetime
from pathlib import Path

sys.path.append('/usr/lib/python3/dist-packages')  # fallback: apt KiCad's pcbnew; PYTHONPATH (e.g. build_rl/pcbnew) outranks it
import pcbnew

sys.path.insert(0, str(Path(__file__).resolve().parent))
from setup_drvzero import remove_routing
from dsn_classes import (effective_class_params, fix_dsn_classes, parse_pro_classes,
                         print_pro_warnings, resolve_net_to_class, verify_dsn_constraints)

# ─────────────────────────────────────────────
# Path constants (env override; no machine-specific fallback)
#   SYNTH2L_PCB_ROOT  — raw kicad_pcb directory (with test/train/val splits)
#   SYNTH2L_DSN_ROOT  — output directory for DSN+ORP
# ─────────────────────────────────────────────
import os as _os
_SRC = _os.environ.get('SYNTH2L_PCB_ROOT')
_DST = _os.environ.get('SYNTH2L_DSN_ROOT')
if not _SRC or not _DST:
    raise SystemExit(
        'SYNTH2L_PCB_ROOT and SYNTH2L_DSN_ROOT must be set (paths to the '
        'synth_2L_v2 source tree and DSN+ORP output tree).'
    )
SRC_ROOT = Path(_SRC)
DST_ROOT = Path(_DST)
SYNTH_ROOT = SRC_ROOT.parent  # kept for backward-compat in any downstream prints

SPLITS = ['test', 'train', 'val']



# ─────────────────────────────────────────────
# Board data extraction via pcbnew
# ─────────────────────────────────────────────

PAD_SHAPE_MAP = {
    pcbnew.PAD_SHAPE_CIRCLE: 'circle',
    pcbnew.PAD_SHAPE_RECT: 'rect',
    pcbnew.PAD_SHAPE_OVAL: 'oval',
    pcbnew.PAD_SHAPE_TRAPEZOID: 'trapezoid',
    pcbnew.PAD_SHAPE_ROUNDRECT: 'roundrect',
    pcbnew.PAD_SHAPE_CHAMFERED_RECT: 'chamfered_rect',
    pcbnew.PAD_SHAPE_CUSTOM: 'custom',
}


def _to_mm(iu):
    return pcbnew.ToMM(iu)


def extract_pads(board):
    pads_data = []
    pad_id_set = set()

    for fp in board.GetFootprints():
        ref = fp.GetReference()
        for pad in fp.Pads():
            pad_name = pad.GetName()
            net_name = pad.GetNetname() or ''

            base_id = f'{ref}@{pad_name}' if ref and pad_name else f'PAD{len(pads_data):05d}'
            pad_id = base_id
            suffix = 2
            while pad_id in pad_id_set:
                pad_id = f'{base_id}_v{suffix}'
                suffix += 1
            pad_id_set.add(pad_id)

            ls = pad.GetLayerSet()
            on_f = ls.Contains(pcbnew.F_Cu)
            on_b = ls.Contains(pcbnew.B_Cu)
            if on_f and on_b:
                layer = 'THRU'
            elif on_b:
                layer = 'B.Cu'
            else:
                layer = 'F.Cu'

            size = pad.GetSize()
            drill_x = pad.GetDrillSizeX()
            drill_y = pad.GetDrillSizeY()
            drill_mm = _to_mm(max(drill_x, drill_y)) if (drill_x > 0 or drill_y > 0) else None

            shape_str = PAD_SHAPE_MAP.get(pad.GetShape(), 'circle')

            pads_data.append({
                'id': pad_id,
                'component_id': ref,
                'component_ref': ref,
                'net_id': net_name,
                'net_name': net_name,
                'position': {'x': _to_mm(pad.GetX()), 'y': _to_mm(pad.GetY())},
                'size': {'width': _to_mm(size.x), 'height': _to_mm(size.y)},
                'drill_size': drill_mm,
                'layer': layer,
                'shape': shape_str,
                'angle': pad.GetOrientationDegrees(),
            })
    return pads_data


def extract_nets(board, pads_data, net_to_class):
    nets_data = []
    net_info = board.GetNetInfo()

    for net_code in range(net_info.GetNetCount()):
        net = net_info.GetNetItem(net_code)
        if not net:
            continue
        net_name = net.GetNetname()
        if not net_name:
            continue

        terminals = []
        x_coords, y_coords = [], []
        for pad in pads_data:
            if pad['net_name'] == net_name:
                pos = pad['position']
                terminals.append({
                    'pad_id': pad['id'],
                    'position': {'x': pos['x'], 'y': pos['y']},
                    'layer': pad['layer'],
                })
                x_coords.append(pos['x'])
                y_coords.append(pos['y'])

        if not terminals:
            continue

        bounds = {
            'x_min': min(x_coords), 'y_min': min(y_coords),
            'x_max': max(x_coords), 'y_max': max(y_coords),
        }

        nets_data.append({
            'id': net_name,
            'name': net_name,
            'netclass': net_to_class.get(net_name, 'Default'),
            'pad_count': len(terminals),
            'is_routable': len(terminals) >= 2,
            'terminals': terminals,
            'bounds': bounds,
        })
    return nets_data


def extract_layers(board):
    layers = []
    cu_count = board.GetCopperLayerCount()

    if board.IsLayerEnabled(pcbnew.F_Cu):
        layers.append({
            'name': board.GetLayerName(pcbnew.F_Cu),
            'type': 'signal',
            'stackup_position': 0,
            'thickness': 0.035,
            'material': 'copper',
            'is_routing_layer': True,
        })

    pos = 1
    for layer_id in range(pcbnew.In1_Cu, pcbnew.In1_Cu + max(0, cu_count - 2)):
        if board.IsLayerEnabled(layer_id):
            layers.append({
                'name': board.GetLayerName(layer_id),
                'type': 'signal',
                'stackup_position': pos,
                'thickness': 0.035,
                'material': 'copper',
                'is_routing_layer': True,
            })
            pos += 1

    if board.IsLayerEnabled(pcbnew.B_Cu) and cu_count >= 2:
        layers.append({
            'name': board.GetLayerName(pcbnew.B_Cu),
            'type': 'signal',
            'stackup_position': pos,
            'thickness': 0.035,
            'material': 'copper',
            'is_routing_layer': True,
        })

    return layers


def build_drc_rules(board, class_params):
    ds = board.GetDesignSettings()
    min_track = _to_mm(ds.m_TrackMinWidth) or 0.1
    min_via = _to_mm(ds.m_ViasMinSize) or 0.2

    default_params = effective_class_params(class_params, 'Default')
    default_rules = {
        'track_width': default_params['trace_width'],
        'clearance': default_params['clearance'],
        'via_diameter': default_params['via_dia'],
        'via_drill': default_params['via_drill'],
        'microvia_diameter': default_params.get('uvia_dia', 0.3),
        'microvia_drill': default_params.get('uvia_drill', 0.1),
        'netclass': 'Default',
    }
    default_rules = {k: v for k, v in default_rules.items() if v is not None}

    netclasses = {}
    for cls_name, params in class_params.items():
        if cls_name == 'Default':
            continue
        nc = {
            'track_width': params.get('trace_width', default_rules['track_width']),
            'clearance': params.get('clearance', default_rules['clearance']),
            'via_diameter': params.get('via_dia', default_rules['via_diameter']),
            'via_drill': params.get('via_drill', default_rules['via_drill']),
        }
        if 'uvia_dia' in params:
            nc['microvia_diameter'] = params['uvia_dia']
        if 'uvia_drill' in params:
            nc['microvia_drill'] = params['uvia_drill']
        netclasses[cls_name] = nc

    return {
        'default': default_rules,
        'netclasses': netclasses,
        'min_values': {
            'track_width': min_track,
            'clearance': min_track,
            'via_diameter': min_via,
            'via_drill': 0.1,
        },
    }


def build_orp(board, pads_data, nets_data, layers_data, drc_rules, board_id):
    bb = board.GetBoardEdgesBoundingBox()
    x_min = _to_mm(bb.GetLeft())
    y_min = _to_mm(bb.GetTop())
    x_max = _to_mm(bb.GetRight())
    y_max = _to_mm(bb.GetBottom())

    return {
        'format_version': '1.0',
        'metadata': {
            'filename': board_id,
            'board_name': board_id,
            'board_id': board_id,
            'export_timestamp': datetime.utcnow().isoformat() + 'Z',
            'orthoroute_version': '0.1.0',
        },
        'board': {
            'bounds': {
                'x_min': x_min, 'y_min': y_min,
                'x_max': x_max, 'y_max': y_max,
                'width': x_max - x_min,
                'height': y_max - y_min,
            },
            'layer_count': board.GetCopperLayerCount(),
            'thickness': _to_mm(board.GetDesignSettings().GetBoardThickness()),
        },
        'pads': pads_data,
        'nets': nets_data,
        'layers': layers_data,
        'drc_rules': drc_rules,
        'grid_parameters': {
            'grid_pitch': 0.4,
            'expansion_margin': 3.0,
            'coordinate_system': 'PCB coordinates in millimeters',
            'board_bounds_used': {
                'x_min': x_min, 'y_min': y_min,
                'x_max': x_max, 'y_max': y_max,
            },
        },
    }


# ─────────────────────────────────────────────
# Single-board processing
# ─────────────────────────────────────────────

def process_board(pcb_path, pro_path, dst_dir, board_stem, verbose=False):
    """
    board_XXXXX.kicad_pcb + .kicad_pro →
      dst_dir/board_XXXXX_unrouted.dsn
      dst_dir/board_XXXXX.orp

    Returns result dict.
    """
    pcb_path = Path(pcb_path)
    pro_path = Path(pro_path)
    dst_dir = Path(dst_dir)

    res = {
        'board': board_stem,
        'status': 'fail', 'msg': '',
        'dsn_classes': 0, 'dsn_issues': [], 'warnings': [], 'orp_nets': 0, 'orp_classes': 0,
    }

    temporaries = []  # removed whatever happens
    outputs = []      # removed unless the board ends ok
    try:
        dst_dir.mkdir(parents=True, exist_ok=True)
        # A board that fails leaves no DSN or ORP, so a router never reads a stale one.
        dsn_path = dst_dir / f'{board_stem}_unrouted.dsn'
        outputs += [dsn_path, dst_dir / f'{board_stem}.orp']
        for out in outputs:
            out.unlink(missing_ok=True)

        # ── 1. Parse kicad_pro (required) ─────────────────────
        pro = parse_pro_classes(json.loads(pro_path.read_text(encoding='utf-8')),
                                source=str(pro_path))   # holds the net-class rules
        class_params, patterns = pro.class_params, pro.patterns
        res['warnings'] = list(pro.warnings)

        # ── 2. Unroute → temporary unrouted kicad_pcb ─────────
        pcb_content = pcb_path.read_text(encoding='utf-8')
        unrouted_content = remove_routing(pcb_content)

        # pcbnew must read from a file, so write a temporary copy in dst.
        tmp_pcb = dst_dir / f'{board_stem}_unrouted.kicad_pcb'
        tmp_pro = dst_dir / f'{board_stem}_unrouted.kicad_pro'
        temporaries += [tmp_pcb, tmp_pro]
        tmp_pcb.write_text(unrouted_content, encoding='utf-8')
        shutil.copy2(pro_path, tmp_pro)

        # ── 3. DSN export ──────────────────────────────────────
        board = pcbnew.LoadBoard(str(tmp_pcb))
        raw_dsn = dsn_path.with_name(dsn_path.stem + '_pcbnew.dsn')
        temporaries.append(raw_dsn)
        ok = bool(pcbnew.ExportSpecctraDSN(board, str(raw_dsn)))
        if not ok or not raw_dsn.exists():
            res['msg'] = 'pcbnew ExportSpecctraDSN failed'
            return res

        # ── 4. Patch DSN class block, verify (written once the board is done) ──
        dsn_content = raw_dsn.read_text(encoding='utf-8')
        new_dsn, cls_summary = fix_dsn_classes(dsn_content, pro)
        ok_verify, issues = verify_dsn_constraints(new_dsn, pro)
        res['dsn_classes'] = cls_summary['classes']
        res['dsn_issues'] = issues
        if not ok_verify:
            res['msg'] = f'DSN rules do not match the .kicad_pro ({len(issues)} issues)'
            return res

        # ── 5. Build ORP (based on the original pcb) ──────────
        board_orig = pcbnew.LoadBoard(str(pcb_path))

        all_net_names = [
            board_orig.GetNetInfo().GetNetItem(i).GetNetname()
            for i in range(board_orig.GetNetInfo().GetNetCount())
            if board_orig.GetNetInfo().GetNetItem(i) and
               board_orig.GetNetInfo().GetNetItem(i).GetNetname()
        ]
        net_to_class = resolve_net_to_class(
            all_net_names, class_params, patterns)

        pads_data = extract_pads(board_orig)
        nets_data = extract_nets(board_orig, pads_data, net_to_class)
        layers_data = extract_layers(board_orig)
        drc_rules = build_drc_rules(board_orig, class_params)

        orp_data = build_orp(
            board_orig, pads_data, nets_data, layers_data, drc_rules,
            board_id=board_stem,
        )

        orp_path = dst_dir / f'{board_stem}.orp'
        with gzip.open(orp_path, 'wt', encoding='utf-8') as f:
            json.dump(orp_data, f, indent=2, ensure_ascii=False)

        res['orp_nets'] = len(nets_data)
        res['orp_classes'] = len(drc_rules.get('netclasses', {})) + 1
        dsn_path.write_text(new_dsn, encoding='utf-8')  # last: a failed board has no DSN
        res['status'] = 'ok'
        res['msg'] = (f'DSN: {cls_summary["classes"]} classes, '
                      f'{cls_summary["nets"]} nets | '
                      f'ORP: {len(nets_data)} nets, {res["orp_classes"]} classes')
        if res['warnings']:
            res['msg'] += f' | {len(res["warnings"])} .kicad_pro warning(s)'
        return res

    except Exception as e:
        import traceback
        res['msg'] = f'{type(e).__name__}: {e}'
        if verbose:
            traceback.print_exc()
        return res
    finally:
        for path in temporaries + (outputs if res['status'] != 'ok' else []):
            path.unlink(missing_ok=True)


# ─────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────

def collect_boards(src_root, splits):
    """For each split, return a list of (split, stem, pcb_path, pro_path)."""
    boards = []
    for split in splits:
        split_dir = src_root / split
        if not split_dir.is_dir():
            continue
        for pcb_path in sorted(split_dir.glob('*.kicad_pcb')):
            stem = pcb_path.stem
            pro_path = pcb_path.with_suffix('.kicad_pro')
            boards.append((split, stem, pcb_path, pro_path))
    return boards


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter, allow_abbrev=False)
    ap.add_argument('--src', default=str(SRC_ROOT),
                    help=f'Input root (default: {SRC_ROOT}).')
    ap.add_argument('--dst', default=str(DST_ROOT),
                    help=f'Output root (default: {DST_ROOT}).')
    ap.add_argument('--splits', nargs='+', default=SPLITS,
                    help=f'Splits to process (default: {SPLITS}).')
    ap.add_argument('--limit', type=int, default=0,
                    help='Max number of boards to process (0 = all).')
    ap.add_argument('--verbose', '-v', action='store_true')
    ap.add_argument('--report', default='',
                    help='Path to save the result JSON.')
    args = ap.parse_args()

    src_root = Path(args.src)
    dst_root = Path(args.dst)

    boards = collect_boards(src_root, args.splits)
    if args.limit:
        boards = boards[:args.limit]

    total = len(boards)
    print(f'Targets: {total} | src={src_root} | dst={dst_root}')
    print(f'{"#":<6} {"Split":<6} {"Board":<20} {"Status":>6} {"DSNcls":>7} {"ORPnets":>8}  Msg')
    print('-' * 100)

    results = []
    ok_n = fail_n = 0

    for i, (split, stem, pcb_path, pro_path) in enumerate(boards, 1):
        dst_dir = dst_root / split
        r = process_board(pcb_path, pro_path, dst_dir, stem, verbose=args.verbose)
        r['split'] = split
        results.append(r)

        iss_tag = f' [{len(r["dsn_issues"])}issues]' if r['dsn_issues'] else ''
        if r['warnings'] and r['status'] != 'ok':   # an ok board's msg carries the count
            iss_tag += f' [{len(r["warnings"])} .kicad_pro warnings]'
        print(f"{i:<6} {split:<6} {stem:<20} {r['status']:>6} "
              f"{r['dsn_classes']:>7} {r['orp_nets']:>8}  {r['msg']}{iss_tag}",
              flush=True)

        if r['status'] == 'ok':
            ok_n += 1
        else:
            fail_n += 1

    print()
    print('=' * 100)
    print(f'Done: {ok_n}/{total} OK | {fail_n} FAIL')

    all_issues = [(r['board'], iss)
                  for r in results if r['dsn_issues']
                  for iss in r['dsn_issues']]
    if all_issues:
        print(f'\n[!] DSN constraint issues ({len(all_issues)}):')
        for board, iss in all_issues[:20]:
            print(f'  {board}: {iss}')
        if len(all_issues) > 20:
            print(f'  ... ({len(all_issues) - 20} more)')

    print_pro_warnings(results, args.report)

    if args.report:
        Path(args.report).write_text(
            json.dumps(results, indent=2, ensure_ascii=False), encoding='utf-8')
        print(f'\nReport saved: {args.report}')


    if fail_n:
        sys.exit(1)

if __name__ == '__main__':
    main()
