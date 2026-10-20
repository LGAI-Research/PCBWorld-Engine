#!/usr/bin/python3
"""
Convert exacad_sorted/<folder>/processed_v9_guide_v3.kicad_pcb + .kicad_pro
to DSN + ORP format under exacad_sorted_dsn/<folder>/.

DSN:
  - Unroute the board, then call pcbnew.ExportSpecctraDSN.
  - Patch the class block using kicad_pro netclass info (avoids missing width/clearance).

ORP:
  - OrthoRoute PCB JSON (gzip-compressed).
  - Includes DRC constraints for every netclass (track_width, clearance, via_diameter, via_drill).
  - Includes per-net netclass assignments.

Usage:
  # Test with 5 samples
  /usr/bin/python3 make_dsn_orp_v3.py --limit 5

  # Run on the full dataset
  /usr/bin/python3 make_dsn_orp_v3.py

  # Process specific folders only
  /usr/bin/python3 make_dsn_orp_v3.py --targets 0001_xxx 0002_yyy
"""

import argparse
import gzip
import json
import shutil
import sys
from datetime import datetime
from pathlib import Path

sys.path.append('/usr/lib/python3/dist-packages')  # fallback: apt KiCad's pcbnew; PYTHONPATH (e.g. build_rl/pcbnew) outranks it
import pcbnew

sys.path.insert(0, str(Path(__file__).resolve().parent))
from setup_drvzero import remove_routing
from dsn_classes import (effective_class_params, fix_dsn_classes, parse_pro_classes,
                         print_pro_warnings, resolve_net_to_class, verify_dsn_constraints)

# ─────────────────────────────────────────────
# Path constants (env override)
#   PCBENCH_PCB_ROOT  — raw kicad_pcb (exacad_sorted) directory
#   PCBENCH_DSN_ROOT  — output directory for DSN+ORP
# Both env vars are required; no built-in machine-specific fallback so the
# script is portable across hosts.
# ─────────────────────────────────────────────
import os as _os
_SRC = _os.environ.get('PCBENCH_PCB_ROOT')
_DST = _os.environ.get('PCBENCH_DSN_ROOT')
if not _SRC or not _DST:
    raise SystemExit(
        'PCBENCH_PCB_ROOT and PCBENCH_DSN_ROOT must be set (paths to the '
        'exacad_sorted source tree and the DSN+ORP output tree).'
    )
SRC_DIR = Path(_SRC)
DST_DIR = Path(_DST)
ROOT = SRC_DIR.parent  # kept for backward-compat in any downstream prints

INPUT_STEM = 'processed_v9_guide_v3'
DSN_NAME = f'{INPUT_STEM}_unrouted.dsn'
ORP_NAME = f'{INPUT_STEM}.orp'


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
    """pcbnew Board → pads list."""
    pads_data = []
    pad_id_set = set()

    for fp in board.GetFootprints():
        ref = fp.GetReference()
        for pad in fp.Pads():
            pad_name = pad.GetName()
            net_name = pad.GetNetname() or ''

            # Generate a unique pad ID.
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

            shape_int = pad.GetShape()
            shape_str = PAD_SHAPE_MAP.get(shape_int, 'circle')

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
    """Build nets list from pcbnew Board + pads_data, including netclass assignments."""
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
    """Return list of copper layers."""
    layers = []
    cu_count = board.GetCopperLayerCount()

    # F.Cu
    if board.IsLayerEnabled(pcbnew.F_Cu):
        layers.append({
            'name': board.GetLayerName(pcbnew.F_Cu),
            'type': 'signal',
            'stackup_position': 0,
            'thickness': 0.035,
            'material': 'copper',
            'is_routing_layer': True,
        })

    # Internal layers (In1.Cu ... InN.Cu)
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

    # B.Cu
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
    """
    Build the ORP drc_rules section from pcbnew design settings + class_params.
    Includes Default-class info plus per-class netclass info.
    """
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
    # Drop None values.
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
        # uvia
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
            'clearance': min_track,  # pcbnew does not expose a separate min_clearance
            'via_diameter': min_via,
            'via_drill': 0.1,
        },
    }


# ─────────────────────────────────────────────
# ORP build
# ─────────────────────────────────────────────

def build_orp(board, pads_data, nets_data, layers_data, drc_rules, filename):
    """Build the ORP dict."""
    bb = board.GetBoardEdgesBoundingBox()
    x_min = _to_mm(bb.GetLeft())
    y_min = _to_mm(bb.GetTop())
    x_max = _to_mm(bb.GetRight())
    y_max = _to_mm(bb.GetBottom())

    return {
        'format_version': '1.0',
        'metadata': {
            'filename': Path(filename).name,
            'board_name': filename,
            'board_id': filename,
            'export_timestamp': datetime.utcnow().isoformat() + 'Z',
            'orthoroute_version': '0.1.0',
        },
        'board': {
            'bounds': {
                'x_min': x_min,
                'y_min': y_min,
                'x_max': x_max,
                'y_max': y_max,
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
# Single-folder processing
# ─────────────────────────────────────────────

def process_folder(src_folder, dst_root, verbose=False):
    """
    Returns dict:
      {'folder', 'status': 'ok'|'skip'|'fail', 'msg',
       'dsn_classes', 'dsn_issues', 'orp_nets', 'orp_classes'}
    """
    src_folder = Path(src_folder)
    folder_name = src_folder.name

    pcb_path = src_folder / f'{INPUT_STEM}.kicad_pcb'
    pro_path = src_folder / f'{INPUT_STEM}.kicad_pro'

    res = {'folder': folder_name, 'status': 'fail', 'msg': '',
           'dsn_classes': 0, 'dsn_issues': [], 'warnings': [], 'orp_nets': 0, 'orp_classes': 0}

    if not pcb_path.exists():
        res['status'] = 'skip'
        res['msg'] = f'no {INPUT_STEM}.kicad_pcb'
        return res

    temporaries = []  # removed whatever happens
    outputs = []      # removed unless the board ends ok
    try:
        dst_folder = dst_root / folder_name
        dst_folder.mkdir(parents=True, exist_ok=True)
        # A board that fails leaves no DSN or ORP, so a router never reads a stale one.
        dsn_path = dst_folder / DSN_NAME
        outputs += [dsn_path, dst_folder / ORP_NAME]
        for out in outputs:
            out.unlink(missing_ok=True)

        # ── 1. Parse kicad_pro (required) ─────────────────────
        pro = parse_pro_classes(json.loads(pro_path.read_text(encoding='utf-8')),
                                source=str(pro_path))   # holds the net-class rules
        class_params, patterns = pro.class_params, pro.patterns
        res['warnings'] = list(pro.warnings)

        # ── 2. unrouted pcb + pro → dst_folder ────────────────
        pcb_content = pcb_path.read_text(encoding='utf-8')
        unrouted_content = remove_routing(pcb_content)

        unrouted_pcb = dst_folder / f'{INPUT_STEM}_unrouted.kicad_pcb'
        unrouted_pro = dst_folder / f'{INPUT_STEM}_unrouted.kicad_pro'
        unrouted_pcb.write_text(unrouted_content, encoding='utf-8')
        shutil.copy2(pro_path, unrouted_pro)

        # ── 3. Load via pcbnew → DSN export ──────────────────
        board = pcbnew.LoadBoard(str(unrouted_pcb))
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

        # ── 5. Build ORP (load original guide_v3 pcb) ────────
        # ORP is built from the original guide pcb, not the unrouted one
        # (pad info is identical).
        board_orig = pcbnew.LoadBoard(str(pcb_path))

        # net → class mapping
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
            filename=f'{folder_name}/{INPUT_STEM}.kicad_pcb'
        )

        orp_path = dst_folder / ORP_NAME
        with gzip.open(orp_path, 'wt', encoding='utf-8') as f:
            json.dump(orp_data, f, indent=2, ensure_ascii=False)

        res['orp_nets'] = len(nets_data)
        res['orp_classes'] = len(drc_rules.get('netclasses', {})) + 1  # +1 for Default

        # ── 6. Cleanup temporaries (keep unrouted pcb/pro) ───
        # The unrouted .kicad_pcb/.kicad_pro files are the source for the
        # DSN export, so we preserve them.

        dsn_path.write_text(new_dsn, encoding='utf-8')  # last: a failed board has no DSN
        res['status'] = 'ok'
        res['msg'] = (f'DSN: {cls_summary["classes"]} classes, '
                      f'{cls_summary["nets"]} nets | '
                      f'ORP: {len(nets_data)} nets, '
                      f'{res["orp_classes"]} classes')
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

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter, allow_abbrev=False)
    ap.add_argument('--src', default=str(SRC_DIR),
                    help='Input directory (default: exacad_sorted).')
    ap.add_argument('--dst', default=str(DST_DIR),
                    help='Output directory (default: exacad_sorted_dsn).')
    ap.add_argument('--limit', type=int, default=0,
                    help='Max number of folders to process (0 = all).')
    ap.add_argument('--targets', nargs='*', default=None,
                    help='Specific folder names to process.')
    ap.add_argument('--verbose', '-v', action='store_true')
    ap.add_argument('--report', default='',
                    help='Path to save the result JSON (default: not saved).')
    args = ap.parse_args()

    src = Path(args.src)
    dst = Path(args.dst)
    dst.mkdir(parents=True, exist_ok=True)

    if args.targets:
        folders = [src / t for t in args.targets if (src / t).is_dir()]
    else:
        folders = sorted(f for f in src.iterdir()
                         if f.is_dir() and (f / f'{INPUT_STEM}.kicad_pcb').exists())

    if args.limit:
        folders = folders[:args.limit]

    total = len(folders)
    print(f'Targets: {total} | src={src} | dst={dst}')
    print(f'{"#":<5} {"Folder":<55} {"Status":>6} {"DSNcls":>7} {"ORPnets":>8} {"Msg"}')
    print('-' * 110)

    results = []
    ok_n = fail_n = skip_n = 0

    for i, folder in enumerate(folders, 1):
        r = process_folder(folder, dst, verbose=args.verbose)
        results.append(r)

        status = r['status']
        dsn_iss_tag = f' [{len(r["dsn_issues"])}issues]' if r['dsn_issues'] else ''
        if r['warnings'] and r['status'] != 'ok':   # an ok board's msg carries the count
            dsn_iss_tag += f' [{len(r["warnings"])} .kicad_pro warnings]'
        print(f"{i:<5} {r['folder']:<55} {status:>6} "
              f"{r['dsn_classes']:>7} {r['orp_nets']:>8}  {r['msg']}{dsn_iss_tag}",
              flush=True)

        if status == 'ok':
            ok_n += 1
        elif status == 'skip':
            skip_n += 1
        else:
            fail_n += 1

    print()
    print('=' * 110)
    print(f'Done: {ok_n}/{total} OK | {fail_n} FAIL | {skip_n} SKIP')

    # Report any DSN constraint issues.
    all_issues = [(r['folder'], iss)
                  for r in results if r['dsn_issues']
                  for iss in r['dsn_issues']]
    if all_issues:
        print(f'\n[!] DSN constraint issues ({len(all_issues)}):')
        for folder, iss in all_issues[:20]:
            print(f'  {folder}: {iss}')
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
