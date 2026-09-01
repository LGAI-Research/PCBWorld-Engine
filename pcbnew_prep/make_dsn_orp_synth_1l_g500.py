#!/usr/bin/python3
"""
synth_1L_grid500_5net_v02/{split}/board_XXXXX.kicad_pcb + .kicad_pro →
synth_1L_grid500_5net_v02_dsn/{split}/board_XXXXX_unrouted.dsn + board_XXXXX.orp

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
import fnmatch
import gzip
import json
import re
import shutil
import sys
import tempfile
from collections import defaultdict
from datetime import datetime
from pathlib import Path

sys.path.append('/usr/lib/python3/dist-packages')  # fallback: apt KiCad's pcbnew; PYTHONPATH (e.g. build_rl/pcbnew) outranks it
import pcbnew

sys.path.insert(0, str(Path(__file__).resolve().parent))
from setup_drvzero import remove_routing

# ─────────────────────────────────────────────
# Path constants
# ─────────────────────────────────────────────
# env override; no machine-specific fallback.
#   SYNTH1L_PCB_ROOT  — raw kicad_pcb directory for THIS variant
#   SYNTH1L_DSN_ROOT  — output directory for DSN+ORP for THIS variant
import os as _os
_SRC = _os.environ.get('SYNTH1L_PCB_ROOT')
_DST = _os.environ.get('SYNTH1L_DSN_ROOT')
if not _SRC or not _DST:
    raise SystemExit(
        'SYNTH1L_PCB_ROOT and SYNTH1L_DSN_ROOT must be set (this script '
        'targets variant synth_1L_grid500_5net_v02; the env vars should match that variant).'
    )
SRC_ROOT = Path(_SRC)
DST_ROOT = Path(_DST)
SYNTH_ROOT = SRC_ROOT.parent

SPLITS = ['test', 'train', 'val']

MM2DSN = 1000  # mm → µm (DSN unit = 1 µm)


# ─────────────────────────────────────────────
# kicad_pro parsing (netclass extraction)
# ─────────────────────────────────────────────

PRO_CLASS_KEYS = {
    'clearance': 'clearance',
    'track_width': 'trace_width',
    'via_diameter': 'via_dia',
    'via_drill': 'via_drill',
    'microvia_diameter': 'uvia_dia',
    'microvia_drill': 'uvia_drill',
}


def parse_pro_classes(pro_data):
    class_params = {}
    explicit_n2c = {}
    patterns = []

    classes = (pro_data.get('net_settings', {}) or {}).get('classes', []) or []
    for c in classes:
        name = c.get('name')
        if not name:
            continue
        params = {}
        for pro_k, internal_k in PRO_CLASS_KEYS.items():
            if pro_k in c and c[pro_k] is not None:
                params[internal_k] = float(c[pro_k])
        class_params[name] = params
        for net_name in c.get('nets', []) or []:
            if net_name:
                explicit_n2c[net_name] = name

    raw_pats = (pro_data.get('net_settings', {}) or {}).get('netclass_patterns', []) or []
    for p in raw_pats:
        if not isinstance(p, dict):
            continue
        pat = p.get('pattern')
        cls = p.get('netclass')
        if pat and cls and cls in class_params:
            patterns.append((pat, cls))

    return class_params, explicit_n2c, patterns


def resolve_net_to_class(net_names, class_params, explicit_n2c, patterns):
    out = {}
    for name in net_names:
        if not name:
            continue
        if name in explicit_n2c:
            out[name] = explicit_n2c[name]
            continue
        matched = None
        for pat, cls in patterns:
            if fnmatch.fnmatchcase(name, pat):
                matched = cls
                break
        out[name] = matched if matched else 'Default'
    return out


# ─────────────────────────────────────────────
# DSN parsing utilities
# ─────────────────────────────────────────────

def extract_dsn_net_names(dsn_content):
    names = []
    for m in re.finditer(r'\(net\s+("(?:[^"\\]|\\.)*"|[^\s)]+)', dsn_content):
        raw = m.group(1)
        names.append(raw[1:-1] if raw.startswith('"') else raw)
    return names


def extract_dsn_via_name(dsn_content):
    m = re.search(r'\(via\s+("(?:[^"\\]|\\.)*"|[^\s)]+)\s*\)', dsn_content)
    if m:
        return m.group(1).strip('"')
    return 'Via[0-1]_600:400_um'


def _dsn_net_token(name):
    if re.search(r'[() /+\-]', name):
        return f'"{name}"'
    return name


# ─────────────────────────────────────────────
# DSN class block patching
# ─────────────────────────────────────────────

def _remove_dsn_class_blocks(dsn_content):
    lines = dsn_content.split('\n')
    result = []
    in_class = False
    depth = 0
    for line in lines:
        stripped = line.strip()
        if not in_class:
            if re.match(r'\(class\b', stripped):
                in_class = True
                depth = stripped.count('(') - stripped.count(')')
                if depth <= 0:
                    in_class = False
            else:
                result.append(line)
        else:
            depth += stripped.count('(') - stripped.count(')')
            if depth <= 0:
                in_class = False
    return '\n'.join(result)


def build_dsn_classes(net_to_class, class_params, via_name):
    class_nets = defaultdict(list)
    for net_name, cls in net_to_class.items():
        class_nets[cls].append(net_name)

    lines = []
    for cls_name in sorted(class_nets.keys()):
        nets = class_nets[cls_name]
        if not nets:
            continue
        params = class_params.get(cls_name) or class_params.get('Default') or {}
        trace_w = params.get('trace_width', 0.25)
        clearance = params.get('clearance', 0.2)

        width_dsn = int(round(trace_w * MM2DSN))
        clear_dsn = int(round(clearance * MM2DSN))

        net_tokens = ' '.join(_dsn_net_token(n) for n in sorted(nets))
        lines.append(f'    (class {cls_name} {net_tokens}')
        lines.append(f'      (circuit')
        lines.append(f'        (use_via "{via_name}")')
        lines.append(f'      )')
        lines.append(f'      (rule')
        lines.append(f'        (width {width_dsn})')
        lines.append(f'        (clearance {clear_dsn})')
        lines.append(f'      )')
        lines.append(f'    )')
    return '\n'.join(lines)


def insert_classes_before_wiring(dsn_no_class, class_text):
    lines = dsn_no_class.split('\n')
    final = []
    inserted = False
    for line in lines:
        if not inserted and line.strip().startswith('(wiring'):
            for i in range(len(final) - 1, -1, -1):
                if final[i].strip() == ')':
                    network_closer = final.pop(i)
                    break
            else:
                network_closer = '  )'
            final.append(class_text)
            final.append(network_closer)
            inserted = True
        final.append(line)
    if not inserted:
        for i in range(len(final) - 1, -1, -1):
            if final[i].strip() == ')':
                network_closer = final.pop(i)
                final.append(class_text)
                final.append(network_closer)
                break
    return '\n'.join(final)


def fix_dsn_classes(dsn_content, pro_data):
    class_params, explicit_n2c, patterns = parse_pro_classes(pro_data)

    if not class_params:
        return dsn_content, {'classes': 1, 'nets': 0, 'note': 'no_pro_classes'}

    net_names = extract_dsn_net_names(dsn_content)
    via_name = extract_dsn_via_name(dsn_content)

    net_to_class = resolve_net_to_class(net_names, class_params, explicit_n2c, patterns)
    class_text = build_dsn_classes(net_to_class, class_params, via_name)

    dsn_no_class = _remove_dsn_class_blocks(dsn_content)
    new_dsn = insert_classes_before_wiring(dsn_no_class, class_text)

    class_nets = defaultdict(list)
    for n, c in net_to_class.items():
        class_nets[c].append(n)
    summary = {
        'classes': len(class_nets),
        'nets': len(net_names),
        'via': via_name,
    }
    return new_dsn, summary


# ─────────────────────────────────────────────
# DSN verification
# ─────────────────────────────────────────────

def _dsn_balanced_end(content, start):
    depth = 0
    for j in range(start, len(content)):
        c = content[j]
        if c == '(':
            depth += 1
        elif c == ')':
            depth -= 1
            if depth == 0:
                return j + 1
    return len(content)


def verify_dsn_constraints(dsn_content, pro_data):
    class_params, _, _ = parse_pro_classes(pro_data)
    if not class_params:
        return True, []

    dsn_classes = {}
    idx = 0
    while True:
        start = dsn_content.find('(class ', idx)
        if start == -1:
            break
        end = _dsn_balanced_end(dsn_content, start)
        block = dsn_content[start:end]

        m = re.match(r'\(class\s+(\S+)', block)
        cls_name = m.group(1) if m else 'unknown'

        wm = re.search(r'\(width\s+(\d+)\)', block)
        cm = re.search(r'\(clearance\s+(\d+)\)', block)
        width_mm = int(wm.group(1)) / MM2DSN if wm else None
        clear_mm = int(cm.group(1)) / MM2DSN if cm else None
        dsn_classes[cls_name] = {'width_mm': width_mm, 'clearance_mm': clear_mm}
        idx = end

    issues = []
    for cls_name, dsn_vals in dsn_classes.items():
        params = class_params.get(cls_name) or class_params.get('Default') or {}
        expected_w = params.get('trace_width', 0.25)
        expected_c = params.get('clearance', 0.2)

        if dsn_vals['width_mm'] is None:
            issues.append(f'[{cls_name}] missing width')
        elif abs(dsn_vals['width_mm'] - expected_w) > 0.001:
            issues.append(f'[{cls_name}] width mismatch: DSN={dsn_vals["width_mm"]}mm, pro={expected_w}mm')

        if dsn_vals['clearance_mm'] is None:
            issues.append(f'[{cls_name}] missing clearance')
        elif abs(dsn_vals['clearance_mm'] - expected_c) > 0.001:
            issues.append(f'[{cls_name}] clearance mismatch: DSN={dsn_vals["clearance_mm"]}mm, pro={expected_c}mm')

    return len(issues) == 0, issues


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

    default_params = class_params.get('Default', {})
    default_rules = {
        'track_width': default_params.get('trace_width', 0.25),
        'clearance': default_params.get('clearance', 0.2),
        'via_diameter': default_params.get('via_dia', 0.6),
        'via_drill': default_params.get('via_drill', 0.3),
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
            'via_diameter': params.get('via_dia', default_rules.get('via_diameter', 0.6)),
            'via_drill': params.get('via_drill', default_rules.get('via_drill', 0.3)),
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
        'dsn_classes': 0, 'dsn_issues': [], 'orp_nets': 0, 'orp_classes': 0,
    }

    try:
        dst_dir.mkdir(parents=True, exist_ok=True)

        # ── 1. Parse kicad_pro ────────────────────────────────
        pro_data = {}
        if pro_path.exists():
            try:
                pro_data = json.loads(pro_path.read_text(encoding='utf-8'))
            except Exception as e:
                if verbose:
                    print(f'  [WARN] failed to parse kicad_pro: {e}')

        class_params, explicit_n2c, patterns = parse_pro_classes(pro_data)

        # ── 2. Unroute → temporary unrouted kicad_pcb ─────────
        pcb_content = pcb_path.read_text(encoding='utf-8')
        unrouted_content = remove_routing(pcb_content)

        # pcbnew must read from a file, so write a temporary copy in dst.
        tmp_pcb = dst_dir / f'{board_stem}_unrouted.kicad_pcb'
        tmp_pro = dst_dir / f'{board_stem}_unrouted.kicad_pro'
        tmp_pcb.write_text(unrouted_content, encoding='utf-8')
        if pro_path.exists():
            shutil.copy2(pro_path, tmp_pro)

        # ── 3. DSN export ──────────────────────────────────────
        board = pcbnew.LoadBoard(str(tmp_pcb))
        dsn_path = dst_dir / f'{board_stem}_unrouted.dsn'
        ok = bool(pcbnew.ExportSpecctraDSN(board, str(dsn_path)))
        if not ok or not dsn_path.exists():
            res['msg'] = 'pcbnew ExportSpecctraDSN failed'
            return res

        # ── 4. Patch DSN class block ──────────────────────────
        dsn_content = dsn_path.read_text(encoding='utf-8')
        new_dsn, cls_summary = fix_dsn_classes(dsn_content, pro_data)
        dsn_path.write_text(new_dsn, encoding='utf-8')

        ok_verify, issues = verify_dsn_constraints(new_dsn, pro_data)
        res['dsn_classes'] = cls_summary.get('classes', 1)
        res['dsn_issues'] = issues
        if not ok_verify and verbose:
            for iss in issues:
                print(f'  [DSN ISSUE] {iss}')

        # ── 5. Build ORP (based on the original pcb) ──────────
        board_orig = pcbnew.LoadBoard(str(pcb_path))

        all_net_names = [
            board_orig.GetNetInfo().GetNetItem(i).GetNetname()
            for i in range(board_orig.GetNetInfo().GetNetCount())
            if board_orig.GetNetInfo().GetNetItem(i) and
               board_orig.GetNetInfo().GetNetItem(i).GetNetname()
        ]
        net_to_class = resolve_net_to_class(
            all_net_names, class_params, explicit_n2c, patterns)

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

        # ── 6. Cleanup temporary unrouted kicad_pcb/pro ──────
        tmp_pcb.unlink(missing_ok=True)
        tmp_pro.unlink(missing_ok=True)

        res['orp_nets'] = len(nets_data)
        res['orp_classes'] = len(drc_rules.get('netclasses', {})) + 1
        res['status'] = 'ok'
        res['msg'] = (f'DSN: {cls_summary.get("classes", 1)} classes, '
                      f'{cls_summary.get("nets", 0)} nets | '
                      f'ORP: {len(nets_data)} nets, {res["orp_classes"]} classes')
        return res

    except Exception as e:
        import traceback
        res['msg'] = f'{type(e).__name__}: {e}'
        if verbose:
            traceback.print_exc()
        return res


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
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
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

    if args.report:
        Path(args.report).write_text(
            json.dumps(results, indent=2, ensure_ascii=False), encoding='utf-8')
        print(f'\nReport saved: {args.report}')


if __name__ == '__main__':
    main()
