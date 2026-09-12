#!/usr/bin/env python3
"""
1. Copy the folders under PCBench/PCBs that match pcbbench_drvzero_filtered
   into drvzero_filtered/, numbered.
2. Remove routing (segment, via, zone) from processed.kicad_pcb -> processed_unrouted.kicad_pcb
3. processed_unrouted.kicad_pcb -> processed_unrouted.dsn (Specctra DSN)

Set DRVZERO_DATA_DIR to the directory that holds ``pcbbench_drvzero_filtered/``
and ``PCBench/PCBs/`` (output goes to ``<dir>/drvzero_filtered/``). Env-var
only — the dataset root is machine-specific and never hardcoded here.
"""

import os
import re
import math
import shutil
from pathlib import Path

_data_dir = os.environ.get("DRVZERO_DATA_DIR")
if not _data_dir:
    raise SystemExit("DRVZERO_DATA_DIR is not set — see this script's docstring.")
DATA_DIR = Path(_data_dir)
DRVZERO_SRC = DATA_DIR / "pcbbench_drvzero_filtered"
PCBS_DIR = DATA_DIR / "PCBench" / "PCBs"
DEST_DIR = DATA_DIR / "drvzero_filtered"

# DSN unit: KiCad mm * 10000 -> um/10 (Specctra resolution um 10)
MM2DSN = 10000.0


# ─────────────────────────────────────────────
# S-expression tokenizer
# ─────────────────────────────────────────────

def tokenize(text):
    tokens = []
    i = 0
    while i < len(text):
        c = text[i]
        if c in ' \t\r\n':
            i += 1
        elif c == '(':
            tokens.append('(')
            i += 1
        elif c == ')':
            tokens.append(')')
            i += 1
        elif c == '"':
            j = i + 1
            while j < len(text) and text[j] != '"':
                if text[j] == '\\':
                    j += 1
                j += 1
            tokens.append(text[i:j+1])
            i = j + 1
        else:
            j = i
            while j < len(text) and text[j] not in ' \t\r\n()':
                j += 1
            tokens.append(text[i:j])
            i = j
    return tokens


def parse_sexp(tokens, pos=0):
    if tokens[pos] != '(':
        val = tokens[pos]
        return val, pos + 1
    pos += 1  # skip '('
    items = []
    while tokens[pos] != ')':
        item, pos = parse_sexp(tokens, pos)
        items.append(item)
    pos += 1  # skip ')'
    return items, pos


def load_sexp(text):
    tokens = tokenize(text)
    result, _ = parse_sexp(tokens, 0)
    return result


def find_all(sexp, key):
    results = []
    if not isinstance(sexp, list):
        return results
    for item in sexp:
        if isinstance(item, list) and len(item) > 0 and item[0] == key:
            results.append(item)
        results.extend(find_all(item, key))
    return results


def find_first(sexp, key):
    if not isinstance(sexp, list):
        return None
    for item in sexp:
        if isinstance(item, list) and len(item) > 0 and item[0] == key:
            return item
        r = find_first(item, key)
        if r is not None:
            return r
    return None


def get_val(sexp, key, default=None):
    for item in sexp:
        if isinstance(item, list) and len(item) >= 2 and item[0] == key:
            return item[1]
    return default


def strip_quotes(s):
    if isinstance(s, str) and len(s) >= 2 and s[0] == '"' and s[-1] == '"':
        return s[1:-1]
    return s


def to_float(s):
    try:
        return float(strip_quotes(s))
    except (ValueError, TypeError):
        return 0.0


# ─────────────────────────────────────────────
# Step 2: Remove routing from kicad_pcb
# ─────────────────────────────────────────────

def remove_routing(content):
    """Remove segment, via, zone blocks from kicad_pcb text content."""
    lines = content.split('\n')
    result = []
    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()

        # Remove segment and via blocks (single-line or multi-line)
        if (stripped.startswith('(segment ') or stripped == '(segment'
                or stripped.startswith('(via ') or stripped == '(via'):
            depth = stripped.count('(') - stripped.count(')')
            i += 1
            while i < len(lines) and depth > 0:
                depth += lines[i].count('(') - lines[i].count(')')
                i += 1
            if depth < 0:
                result.append(')' * (-depth))
            continue

        # Remove multi-line zone blocks
        if stripped.startswith('(zone '):
            depth = stripped.count('(') - stripped.count(')')
            i += 1
            while i < len(lines) and depth > 0:
                depth += lines[i].count('(') - lines[i].count(')')
                i += 1
            if depth < 0:
                result.append(')' * (-depth))
            continue

        result.append(line)
        i += 1

    output = '\n'.join(result)
    output = re.sub(r'\(tracks \d+\)', '(tracks 0)', output)
    output = re.sub(r'\(zones \d+\)', '(zones 0)', output)
    return output


# ─────────────────────────────────────────────
# Step 3: Export Specctra DSN
# ─────────────────────────────────────────────

def mm(v):
    """Convert mm string/float to DSN integer unit."""
    return int(round(to_float(v) * MM2DSN))


def xy_dsn(x, y):
    return f"{mm(x)} {-mm(y)}"  # KiCad Y is flipped for DSN


def get_copper_layers(pcb_sexp):
    layers = find_first(pcb_sexp, 'layers')
    cu = []
    if layers:
        for item in layers:
            if isinstance(item, list) and len(item) >= 3:
                layer_name = strip_quotes(item[1])
                layer_type = strip_quotes(item[2])
                if layer_type == 'signal':
                    cu.append(layer_name)
    return cu


def get_board_outline(pcb_sexp):
    """Extract Edge.Cuts lines and arcs for board boundary."""
    segments = []
    for gr_line in find_all(pcb_sexp, 'gr_line'):
        layer = strip_quotes(get_val(gr_line, 'layer', ''))
        if layer == 'Edge.Cuts':
            start = find_first(gr_line, 'start')
            end = find_first(gr_line, 'end')
            if start and end:
                segments.append(('line',
                                  to_float(start[1]), to_float(start[2]),
                                  to_float(end[1]), to_float(end[2])))
    for gr_arc in find_all(pcb_sexp, 'gr_arc'):
        layer = strip_quotes(get_val(gr_arc, 'layer', ''))
        if layer == 'Edge.Cuts':
            start = find_first(gr_arc, 'start')   # center in v5
            end = find_first(gr_arc, 'end')       # start point in v5
            angle_item = find_first(gr_arc, 'angle')
            if start and end and angle_item:
                cx, cy = to_float(start[1]), to_float(start[2])
                ex, ey = to_float(end[1]), to_float(end[2])
                angle_deg = to_float(angle_item[1])
                segments.append(('arc', cx, cy, ex, ey, angle_deg))
    return segments


def outline_to_path(segments):
    """Convert outline segments/arcs to a sequence of (x,y) polygon points."""
    # Build adjacency and chain into a polygon
    EPSILON = 1e-4
    edges = []  # list of (x1,y1,x2,y2) after arc interpolation
    for seg in segments:
        if seg[0] == 'line':
            _, x1, y1, x2, y2 = seg
            edges.append((x1, y1, x2, y2))
        elif seg[0] == 'arc':
            _, cx, cy, ex, ey, angle_deg = seg
            # Interpolate arc
            rx = ex - cx
            ry = ey - cy
            radius = math.sqrt(rx*rx + ry*ry)
            start_angle = math.atan2(ry, rx)
            end_angle = start_angle + math.radians(angle_deg)
            steps = max(4, int(abs(angle_deg) / 5))
            pts = []
            for k in range(steps + 1):
                t = k / steps
                a = start_angle + t * math.radians(angle_deg)
                pts.append((cx + radius * math.cos(a),
                             cy + radius * math.sin(a)))
            for k in range(len(pts) - 1):
                edges.append((pts[k][0], pts[k][1], pts[k+1][0], pts[k+1][1]))

    if not edges:
        return []

    # Chain edges into a polygon
    def close(a, b):
        return abs(a[0]-b[0]) < EPSILON and abs(a[1]-b[1]) < EPSILON

    remaining = list(edges)
    chain = [remaining.pop(0)]
    while remaining:
        last = (chain[-1][2], chain[-1][3])
        found = False
        for k, e in enumerate(remaining):
            if close(last, (e[0], e[1])):
                chain.append(remaining.pop(k))
                found = True
                break
            elif close(last, (e[2], e[3])):
                chain.append((e[2], e[3], e[0], e[1]))
                remaining.pop(k)
                found = True
                break
        if not found:
            break

    pts = [(e[0], e[1]) for e in chain]
    if chain:
        pts.append((chain[-1][2], chain[-1][3]))
    return pts


def pad_layers_to_dsn(pad_layers_raw, cu_layers):
    """Return which copper layers this pad uses."""
    pad_layers = [strip_quotes(l) for l in pad_layers_raw]
    result = []
    for l in cu_layers:
        if l in pad_layers or '*.Cu' in pad_layers or 'F&B.Cu' in pad_layers:
            result.append(l)
    return result if result else pad_layers[:1]


def pad_shape_dsn(shape, size_x, size_y, drill_x, drill_y, layers, pad_type):
    """Return DSN shape string(s) for one layer."""
    shapes = []
    for layer in layers:
        if shape == 'circle':
            d = mm(max(size_x, size_y))
            shapes.append(f'(circle "{layer}" {d})')
        elif shape == 'rect':
            w = mm(size_x)
            h = mm(size_y)
            shapes.append(f'(rect "{layer}" {-w//2} {-h//2} {w//2} {h//2})')
        elif shape == 'oval':
            w = mm(size_x)
            h = mm(size_y)
            if w == h:
                shapes.append(f'(circle "{layer}" {w})')
            elif w > h:
                shapes.append(f'(path "{layer}" {h} {-(w-h)//2} 0 {(w-h)//2} 0)')
            else:
                shapes.append(f'(path "{layer}" {w} 0 {-(h-w)//2} 0 {(h-w)//2})')
        else:
            # default rect
            w = mm(size_x)
            h = mm(size_y)
            shapes.append(f'(rect "{layer}" {-w//2} {-h//2} {w//2} {h//2})')
    return shapes


def export_dsn(pcb_sexp, out_path, pcb_name):
    cu_layers = get_copper_layers(pcb_sexp)
    if not cu_layers:
        cu_layers = ['F.Cu', 'B.Cu']

    outline_segs = get_board_outline(pcb_sexp)
    outline_pts = outline_to_path(outline_segs)

    # Setup rule values (trace_width/clearance in DSN units, via in both um and DSN units)
    setup = find_first(pcb_sexp, 'setup')
    trace_width = 250   # DSN units (0.1um), default 0.025mm
    clearance = 200
    via_size_dsn = 6000   # DSN units
    via_drill_dsn = 4000
    via_size_um = 600     # um (for padstack name)
    via_drill_um = 400
    if setup:
        tw = get_val(setup, 'last_trace_width')
        if tw:
            trace_width = int(round(to_float(tw) * MM2DSN))
        tc = get_val(setup, 'trace_clearance')
        if tc:
            clearance = int(round(to_float(tc) * MM2DSN))
        vs = get_val(setup, 'via_size')
        if vs:
            via_size_dsn = int(round(to_float(vs) * MM2DSN))
            via_size_um = int(round(to_float(vs) * 1000))
        vd = get_val(setup, 'via_drill')
        if vd:
            via_drill_dsn = int(round(to_float(vd) * MM2DSN))
            via_drill_um = int(round(to_float(vd) * 1000))

    # Collect modules/footprints
    modules = find_all(pcb_sexp, 'module') + find_all(pcb_sexp, 'footprint')

    # Build net id -> name map
    net_id_to_name = {'0': ''}
    for net_item in find_all(pcb_sexp, 'net'):
        if isinstance(net_item, list) and len(net_item) >= 3:
            net_id_to_name[str(strip_quotes(net_item[1]))] = strip_quotes(net_item[2])

    # Parse modules -> components
    components = []     # (footprint_name, ref, x, y, side, angle, pads)
    padstack_defs = {}  # name -> {shapes}

    def make_padstack_name(shape, sx, sy, layers, pad_type):
        layer_str = '_'.join(layers)
        return f"{pad_type}_{shape}_{mm(sx)}x{mm(sy)}_{layer_str}"

    for mod in modules:
        fp_name = strip_quotes(mod[1]) if len(mod) > 1 else 'unknown'
        fp_name = fp_name.replace(' ', '_')

        at = find_first(mod, 'at')
        if not at:
            continue
        mx = to_float(at[1])
        my = to_float(at[2])
        mangle = to_float(at[3]) if len(at) > 3 else 0.0

        mod_layer = strip_quotes(get_val(mod, 'layer', 'F.Cu'))
        side = 'front' if 'F.Cu' in mod_layer else 'back'

        # Reference
        ref = None
        for fp_text in find_all(mod, 'fp_text'):
            if len(fp_text) > 1 and strip_quotes(fp_text[1]) == 'reference':
                ref = strip_quotes(fp_text[2]) if len(fp_text) > 2 else 'U?'
                break
        if ref is None:
            ref = 'U?'
        ref = ref.replace(' ', '_').replace('"', '')

        # Pads
        pads = []
        for pad in find_all(mod, 'pad'):
            if len(pad) < 4:
                continue
            pad_num = strip_quotes(pad[1])
            pad_type = strip_quotes(pad[2])   # thru_hole, smd, np_thru_hole
            pad_shape = strip_quotes(pad[3])  # circle, rect, oval, roundrect

            pad_at = find_first(pad, 'at')
            if not pad_at:
                continue
            px = to_float(pad_at[1])
            py = to_float(pad_at[2])
            pangle = to_float(pad_at[3]) if len(pad_at) > 3 else 0.0

            size = find_first(pad, 'size')
            if not size:
                continue
            sx = to_float(size[1])
            sy = to_float(size[2])

            drill = find_first(pad, 'drill')
            dx = to_float(drill[1]) if drill and len(drill) > 1 else 0.0
            dy = to_float(drill[2]) if drill and len(drill) > 2 else dx

            layers_item = find_first(pad, 'layers')
            raw_layers = layers_item[1:] if layers_item else ['F.Cu']
            pad_cu_layers = pad_layers_to_dsn(raw_layers, cu_layers)

            net_item = find_first(pad, 'net')
            net_id = strip_quotes(net_item[1]) if net_item and len(net_item) > 1 else '0'
            net_name = net_id_to_name.get(str(net_id), '')

            if pad_type == 'np_thru_hole':
                continue  # skip mechanical pads

            ps_name = make_padstack_name(pad_shape, sx, sy, pad_cu_layers, pad_type)
            if ps_name not in padstack_defs:
                shape_strs = pad_shape_dsn(pad_shape, sx, sy, dx, dy, pad_cu_layers, pad_type)
                if pad_type == 'thru_hole':
                    # Add all copper layers
                    all_shapes = []
                    for l in cu_layers:
                        all_shapes.extend(pad_shape_dsn(pad_shape, sx, sy, dx, dy, [l], pad_type))
                    shape_strs = all_shapes
                padstack_defs[ps_name] = {
                    'shapes': shape_strs,
                    'drill': dx if pad_type == 'thru_hole' else None
                }

            # Absolute position in PCB coordinates
            a_rad = math.radians(mangle)
            abs_x = mx + px * math.cos(a_rad) - py * math.sin(a_rad)
            abs_y = my + px * math.sin(a_rad) + py * math.cos(a_rad)

            pads.append({
                'num': pad_num,
                'padstack': ps_name,
                'x': abs_x,
                'y': abs_y,
                'net': net_name,
                'net_id': net_id,
            })

        components.append({
            'fp_name': fp_name,
            'ref': ref,
            'x': mx,
            'y': my,
            'angle': mangle,
            'side': side,
            'pads': pads,
        })

    # Build nets: net_name -> list of (ref, pad_num)
    nets = {}
    for comp in components:
        for pad in comp['pads']:
            net_name = pad['net']
            if not net_name:
                continue
            if net_name not in nets:
                nets[net_name] = []
            nets[net_name].append(f'"{comp["ref"]}"-"{pad["num"]}"')

    # Build via padstack (name uses um values, shapes use DSN units)
    via_ps_name = f"Via[0-1]_{via_size_um}:{via_drill_um}_um"
    via_shapes = []
    for l in cu_layers:
        via_shapes.append(f'(circle "{l}" {via_size_dsn})')
    padstack_defs[via_ps_name] = {
        'shapes': via_shapes,
        'drill': via_drill_um / 1000.0  # back to mm for hole calc
    }

    # ── Write DSN ──
    lines = []
    lines.append(f'(pcb {pcb_name}.dsn')
    lines.append('  (parser')
    lines.append('    (string_quote ")')
    lines.append('    (space_in_quoted_tokens on)')
    lines.append('    (host_cad "KiCad\'s Pcbnew")')
    lines.append('    (host_version "5.1.12-1")')
    lines.append('  )')
    lines.append('  (resolution um 10)')
    lines.append('  (unit um)')

    # Structure
    lines.append('  (structure')
    for idx, layer in enumerate(cu_layers):
        lines.append(f'    (layer "{layer}"')
        lines.append(f'      (type signal)')
        lines.append(f'      (property')
        lines.append(f'        (index {idx})')
        lines.append(f'      )')
        lines.append(f'    )')

    # Boundary: unquoted 'pcb', coordinates split 4 pairs per line
    if outline_pts:
        coord_pairs = [
            f'{int(round(x*MM2DSN))} {int(round(-y*MM2DSN))}'
            for x, y in outline_pts
        ]
        lines.append('    (boundary')
        lines.append('      (path pcb 0')
        for i in range(0, len(coord_pairs), 4):
            chunk = '  '.join(coord_pairs[i:i+4])
            lines.append(f'        {chunk}')
        lines.append('      )')
        lines.append('    )')
    else:
        # fallback: bounding box from padstack positions
        if components:
            all_x = [int(round(c['x'] * MM2DSN)) for c in components]
            all_y = [int(round(-c['y'] * MM2DSN)) for c in components]
            margin = int(5 * MM2DSN)
            x0, y0 = min(all_x) - margin, min(all_y) - margin
            x1, y1 = max(all_x) + margin, max(all_y) + margin
            lines.append(f'    (boundary')
            lines.append(f'      (rect pcb {x0} {y0} {x1} {y1})')
            lines.append(f'    )')

    lines.append(f'    (via "{via_ps_name}" "{via_ps_name}")')
    lines.append(f'    (rule')
    lines.append(f'      (width {trace_width})')
    lines.append(f'      (clearance {clearance} (type default_smd))')
    lines.append(f'      (clearance {clearance} (type smd_via_same_net))')
    lines.append(f'      (clearance {clearance} (type exposed_copper))')
    lines.append(f'    )')
    lines.append('  )')

    # Placement - group by footprint name
    from collections import defaultdict
    fp_groups = defaultdict(list)
    for comp in components:
        fp_groups[comp['fp_name']].append(comp)

    lines.append('  (placement')
    for fp_name, comps in fp_groups.items():
        lines.append(f'    (component "{fp_name}"')
        for comp in comps:
            x = int(round(comp['x'] * MM2DSN))
            y = int(round(-comp['y'] * MM2DSN))
            angle = comp['angle']
            side_str = '"F.Cu"' if comp['side'] == 'front' else '"B.Cu"'
            lines.append(f'      (place "{comp["ref"]}" {x} {y} {side_str} {angle})')
        lines.append('    )')
    lines.append('  )')

    # Library (padstacks)
    lines.append('  (library')
    for ps_name, ps_def in padstack_defs.items():
        lines.append(f'    (padstack "{ps_name}"')
        for shape_str in ps_def['shapes']:
            lines.append(f'      (shape {shape_str})')
        if ps_def.get('drill') and ps_def['drill'] > 0:
            d = int(round(ps_def['drill'] * MM2DSN))
            lines.append(f'      (hole (circle "drill" {d}))')
        lines.append(f'      (attach off)')
        lines.append(f'    )')
    lines.append('  )')

    # Network
    lines.append('  (network')
    for net_name, pins in nets.items():
        safe_name = net_name.replace('"', '\\"')
        lines.append(f'    (net "{safe_name}"')
        lines.append(f'      (pins {" ".join(pins)})')
        lines.append(f'    )')
    lines.append('  )')

    # Wiring (empty - unrouted)
    lines.append('  (wiring')
    lines.append('  )')
    lines.append(')')

    with open(out_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')


# ─────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────

def main():
    DEST_DIR.mkdir(parents=True, exist_ok=True)

    entries = sorted(os.listdir(DRVZERO_SRC))
    total = len([e for e in entries if os.path.isdir(DRVZERO_SRC / e)])
    ok = 0
    fail = 0

    for entry in entries:
        if not os.path.isdir(DRVZERO_SRC / entry):
            continue

        # Strip leading number: "001_name" -> "name"
        match = re.match(r'^(\d+)_(.*)', entry)
        if not match:
            continue
        num = match.group(1)
        pcb_name = match.group(2)

        src_pcb_dir = PCBS_DIR / pcb_name
        dst_dir = DEST_DIR / entry

        # ── Step 1: Copy ──
        if not src_pcb_dir.exists():
            print(f"[SKIP] {entry}: source not found at {src_pcb_dir}")
            fail += 1
            continue

        if dst_dir.exists():
            shutil.rmtree(dst_dir)
        shutil.copytree(src_pcb_dir, dst_dir)

        # ── Step 2: Create unrouted kicad_pcb ──
        processed_pcb = dst_dir / "processed.kicad_pcb"
        unrouted_pcb = dst_dir / "processed_unrouted.kicad_pcb"

        if not processed_pcb.exists():
            print(f"[WARN] {entry}: no processed.kicad_pcb, skipping step 2/3")
            ok += 1
            continue

        with open(processed_pcb, 'r', encoding='utf-8') as f:
            content = f.read()
        unrouted_content = remove_routing(content)
        with open(unrouted_pcb, 'w', encoding='utf-8') as f:
            f.write(unrouted_content)

        # ── Step 3: Export DSN ──
        dsn_path = dst_dir / "processed_unrouted.dsn"
        try:
            tokens = tokenize(unrouted_content)
            pcb_sexp, _ = parse_sexp(tokens, 0)
            export_dsn(pcb_sexp, dsn_path, "processed_unrouted")
        except Exception as e:
            print(f"[WARN] {entry}: DSN export failed: {e}")

        print(f"[OK] {entry}")
        ok += 1

    print(f"\nDone: {ok}/{total} OK, {fail} skipped")


if __name__ == '__main__':
    main()
