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
import fnmatch
import gzip
import json
import re
import shutil
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path

sys.path.insert(0, '/usr/lib/python3/dist-packages')
import pcbnew

sys.path.insert(0, str(Path(__file__).resolve().parent))
from setup_drvzero import remove_routing

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

# DSN unit: 1 unit = 1 µm (per pcbnew ExportSpecctraDSN).
MM2DSN = 1000  # mm → µm


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
    """
    Parse kicad_pro.net_settings.classes.
    Returns:
      class_params: {class_name: {trace_width, clearance, via_dia, via_drill, ...}}
      explicit_n2c: {net_name: class_name}  (from classes[i].nets)
      patterns:     [(pattern, class_name), ...]
    """
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
    """Map net_name → class_name."""
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
    """Return net names from (network (net NAME ...)) in DSN content."""
    names = []
    for m in re.finditer(r'\(net\s+("(?:[^"\\]|\\.)*"|[^\s)]+)', dsn_content):
        raw = m.group(1)
        if raw.startswith('"'):
            names.append(raw[1:-1])
        else:
            names.append(raw)
    return names


def extract_dsn_via_name(dsn_content):
    """Extract the via name from (via "name") in the DSN structure section."""
    m = re.search(r'\(via\s+("(?:[^"\\]|\\.)*"|[^\s)]+)\s*\)', dsn_content)
    if m:
        raw = m.group(1)
        return raw.strip('"')
    return 'Via[0-1]_600:400_um'


def _dsn_net_token(name):
    if re.search(r'[() /+\-]', name):
        return f'"{name}"'
    return name


# ─────────────────────────────────────────────
# DSN class block patching
# ─────────────────────────────────────────────

def _remove_dsn_class_blocks(dsn_content):
    """Remove existing (class ...) blocks."""
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
    """
    Build DSN class block text from net_to_class + class_params.
    Includes per-class track_width / clearance.
    """
    # Nets grouped by class
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
    """
    Insert the class block just before the ')' that closes the network
    section, immediately preceding (wiring ...).
    """
    lines = dsn_no_class.split('\n')
    final = []
    inserted = False
    for line in lines:
        if not inserted and line.strip().startswith('(wiring'):
            # The preceding ')' closes the network section.
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
        # No wiring section — insert before the final ')'.
        for i in range(len(final) - 1, -1, -1):
            if final[i].strip() == ')':
                network_closer = final.pop(i)
                final.append(class_text)
                final.append(network_closer)
                break
    return '\n'.join(final)


def fix_dsn_classes(dsn_content, pro_data):
    """
    Replace pcbnew's single kicad_default class with per-class blocks
    derived from kicad_pro netclass info.
    Returns: (new_dsn, class_summary)
    """
    class_params, explicit_n2c, patterns = parse_pro_classes(pro_data)

    if not class_params:
        # No kicad_pro info — return the DSN unchanged.
        return dsn_content, {'classes': 1, 'nets': 0, 'note': 'no_pro_classes'}

    net_names = extract_dsn_net_names(dsn_content)
    via_name = extract_dsn_via_name(dsn_content)

    net_to_class = resolve_net_to_class(net_names, class_params, explicit_n2c, patterns)
    class_text = build_dsn_classes(net_to_class, class_params, via_name)

    dsn_no_class = _remove_dsn_class_blocks(dsn_content)
    new_dsn = insert_classes_before_wiring(dsn_no_class, class_text)

    # Summary
    class_nets = defaultdict(list)
    for n, c in net_to_class.items():
        class_nets[c].append(n)
    summary = {
        'classes': len(class_nets),
        'nets': len(net_names),
        'via': via_name,
        'class_detail': {
            c: {'count': len(ns),
                'width_mm': (class_params.get(c) or class_params.get('Default', {})).get('trace_width', 0.25),
                'clearance_mm': (class_params.get(c) or class_params.get('Default', {})).get('clearance', 0.2)}
            for c, ns in class_nets.items()
        },
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
    """
    Verify that the width/clearance values in DSN class blocks match the
    kicad_pro netclass info.
    Returns: (ok: bool, issues: [str])
    """
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
    # Drop None values.
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
           'dsn_classes': 0, 'dsn_issues': [], 'orp_nets': 0, 'orp_classes': 0}

    if not pcb_path.exists():
        res['status'] = 'skip'
        res['msg'] = f'no {INPUT_STEM}.kicad_pcb'
        return res

    try:
        dst_folder = dst_root / folder_name
        dst_folder.mkdir(parents=True, exist_ok=True)

        # ── 1. Parse kicad_pro ────────────────────────────────
        pro_data = {}
        if pro_path.exists():
            try:
                pro_data = json.loads(pro_path.read_text(encoding='utf-8'))
            except Exception as e:
                if verbose:
                    print(f'  [WARN] failed to parse kicad_pro: {e}')

        class_params, explicit_n2c, patterns = parse_pro_classes(pro_data)

        # ── 2. unrouted pcb + pro → dst_folder ────────────────
        pcb_content = pcb_path.read_text(encoding='utf-8')
        unrouted_content = remove_routing(pcb_content)

        unrouted_pcb = dst_folder / f'{INPUT_STEM}_unrouted.kicad_pcb'
        unrouted_pro = dst_folder / f'{INPUT_STEM}_unrouted.kicad_pro'
        unrouted_pcb.write_text(unrouted_content, encoding='utf-8')
        if pro_path.exists():
            shutil.copy2(pro_path, unrouted_pro)

        # ── 3. Load via pcbnew → DSN export ──────────────────
        board = pcbnew.LoadBoard(str(unrouted_pcb))
        dsn_path = dst_folder / DSN_NAME
        ok = bool(pcbnew.ExportSpecctraDSN(board, str(dsn_path)))
        if not ok or not dsn_path.exists():
            res['msg'] = 'pcbnew ExportSpecctraDSN failed'
            return res

        # ── 4. Patch DSN class block ──────────────────────────
        dsn_content = dsn_path.read_text(encoding='utf-8')
        new_dsn, cls_summary = fix_dsn_classes(dsn_content, pro_data)
        dsn_path.write_text(new_dsn, encoding='utf-8')

        # Verify
        ok_verify, issues = verify_dsn_constraints(new_dsn, pro_data)
        res['dsn_classes'] = cls_summary.get('classes', 1)
        res['dsn_issues'] = issues
        if not ok_verify and verbose:
            for iss in issues:
                print(f'  [DSN ISSUE] {iss}')

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
            all_net_names, class_params, explicit_n2c, patterns)

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

        res['status'] = 'ok'
        res['msg'] = (f'DSN: {cls_summary.get("classes",1)} classes, '
                      f'{cls_summary.get("nets",0)} nets | '
                      f'ORP: {len(nets_data)} nets, '
                      f'{res["orp_classes"]} classes')
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

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
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

    if args.report:
        Path(args.report).write_text(
            json.dumps(results, indent=2, ensure_ascii=False), encoding='utf-8')
        print(f'\nReport saved: {args.report}')


if __name__ == '__main__':
    main()
