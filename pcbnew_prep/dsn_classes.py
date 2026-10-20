"""Net-class blocks of a pcbnew Specctra DSN, rewritten from the .kicad_pro.

Shared by every make_dsn_orp_* script: pcbnew's ExportSpecctraDSN output gets its
(class ...) blocks replaced by one block per .kicad_pro net class in use, each with
that class's track width, clearance and via padstack. Anything the rewrite cannot
state exactly is an error, never a default value.

No pcbnew import here, so the rewrite is testable without a KiCad build.
"""

import json
import re
from collections import defaultdict, namedtuple
from pathlib import Path

from dsn_units import dsn_resolution, dsn_to_mm, kicad_nm, mm_to_dsn, rule_matches


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

# The rules the DSN class blocks carry.
RULE_KEYS = ('trace_width', 'clearance', 'via_dia', 'via_drill')


# The one .kicad_pro format accepted: the one KiCad 9 writes (shared with the rule-based runner).
KICAD9_PRO_FORMAT = json.loads((Path(__file__).resolve().parent / 'kicad9_pro_format.json').read_text())

# A parsed .kicad_pro: its net classes, KiCad's netclass patterns, and warnings that name it.
ProClasses = namedtuple('ProClasses', 'source class_params patterns warnings')


def check_pro_format(pro_data, source):
    """Refuse a .kicad_pro that is not in the format KiCad 9 writes (KICAD9_PRO_FORMAT)."""
    found = ((pro_data.get('meta') or {}).get('version'),
             ((pro_data.get('net_settings') or {}).get('meta') or {}).get('version'))
    want = (KICAD9_PRO_FORMAT['project_meta_version'], KICAD9_PRO_FORMAT['net_settings_meta_version'])
    if found != want:
        raise ValueError(f'{source}: .kicad_pro format (meta.version, net_settings.meta.version) = '
                         f'{found}, only the KiCad 9 format {want} is supported — re-save the '
                         'project with KiCad 9')


def parse_pro_classes(pro_data, source='.kicad_pro'):
    """
    Parse a KiCad 9 .kicad_pro (``source`` names it in every error and warning).
    Returns ProClasses:
      class_params: {class_name: {trace_width, clearance, via_dia, via_drill, ...}}
                    (only the fields the class sets; see effective_class_params)
      patterns:     [(pattern, class_name), ...] — KiCad 9 assigns classes by
                    ``netclass_patterns`` only (net_settings.cpp); a class's ``nets`` list is
                    not read, and each listed net no pattern repeats is a warning
      warnings:     [str]
    """
    check_pro_format(pro_data, source)
    net_settings = pro_data.get('net_settings', {}) or {}
    if net_settings.get('netclass_assignments'):
        raise ValueError(f'{source}: .kicad_pro has netclass_assignments, which this converter does '
                         'not resolve — the DSN would put those nets in the wrong class')

    entries = net_settings.get('netclass_patterns', []) or []
    repeated = {(e.get('pattern'), e.get('netclass')) for e in entries if isinstance(e, dict)}
    patterned = {e.get('pattern') for e in entries if isinstance(e, dict)}
    warnings = []
    class_params = {}
    for c in net_settings.get('classes', []) or []:
        name = c.get('name')
        if not name:
            continue
        params = {}
        for pro_k, internal_k in PRO_CLASS_KEYS.items():
            if pro_k in c and c[pro_k] is not None:
                params[internal_k] = float(c[pro_k])
        class_params[name] = params
        # A Default entry no pattern names changes nothing by being ignored (the net is in Default).
        ignored = [n for n in c.get('nets') or [] if (n, name) not in repeated
                   and not (name == 'Default' and n not in patterned)]
        if ignored:
            warnings.append(f'{source}: class {name!r} `nets` lists {ignored[:5]}'
                            f'{" ..." if len(ignored) > 5 else ""} ({len(ignored)}), which KiCad 9 '
                            'does not read (only netclass_patterns assign classes); not assigned')

    if 'Default' not in class_params:
        raise ValueError(f'{source}: .kicad_pro has no Default net class')

    patterns = []
    for p in entries:
        if not isinstance(p, dict):
            continue        # KiCad skips a non-object entry too
        pat = p.get('pattern')
        cls = p.get('netclass')
        if not (pat and cls):
            continue
        if cls not in class_params:
            raise ValueError(f'{source}: netclass pattern {pat!r} names undefined net class {cls!r}')
        patterns.append((pat, cls))

    return ProClasses(source, class_params, patterns, warnings)


def print_pro_warnings(results, report=None):
    """The run summary of the boards' .kicad_pro warnings (each names its .kicad_pro and class):
    the first and last 20, and where all of them are (each board's result, saved by --report)."""
    warnings = [w for r in results for w in r.get('warnings', [])]
    if not warnings:
        return
    where = f'all of them in {report}' if report else 'pass --report <file> to keep all of them'
    print(f'\n[!] .kicad_pro warnings ({len(warnings)}; {where}):')
    shown = warnings if len(warnings) <= 40 else [
        *warnings[:20], f'... {len(warnings) - 40} more ...', *warnings[-20:]]
    for w in shown:
        print(f'  {w}')


def _pro_classes(pro):
    """``pro`` as ProClasses: parsed already, or a .kicad_pro dict parsed here."""
    return pro if isinstance(pro, ProClasses) else parse_pro_classes(pro)


def effective_class_params(class_params, name):
    """A net class's rules as KiCad applies them: a field the class leaves unset comes from Default."""
    if name not in class_params:
        raise ValueError(f'net class {name!r} is not in the .kicad_pro')
    params = {**class_params['Default'], **class_params[name]}
    missing = [k for k in RULE_KEYS if k not in params]
    if missing:
        inherited = '' if name == 'Default' else ' (nor does Default)'
        raise ValueError(f'net class {name!r} has no {", ".join(missing)} in the .kicad_pro{inherited}')
    return params


# Characters KiCad's anchored wildcard escapes (eda_pattern_match.cpp); '*' and '?' are its wildcards.
_WILDCARD_LITERAL = set('.+^${}()|[]/\\')
# KiCad expands bus patterns (net_settings.cpp ForEachBusMember): a vector NAME[0..7] or a group
# {A B}; a brace after ~ ^ _ is overbar / super / subscript markup, not a group.
_BUS_PATTERN = re.compile(r'\[\d+\.\.\d+\]|(?<![~^_])\{')


def _kicad_pattern_readings(pattern):
    """The anchored regex and anchored wildcard KiCad matches a netclass pattern with (CTX_NETCLASS).

    The regex reading is compiled with Python's re as a stand-in for wxRegEx; a pattern re
    cannot compile gets only the wildcard reading.
    """
    readings = []
    rx = pattern if pattern.startswith('^') else '^' + pattern
    rx = rx if rx.endswith('$') else rx + '$'
    try:
        readings.append(re.compile(rx))
    except re.error:
        pass
    wc = ''.join('.' if c == '?' else '.*' if c == '*'
                 else '\\' + c if c in _WILDCARD_LITERAL else re.escape(c) for c in pattern)
    readings.append(re.compile('^' + wc + '$'))
    return readings


def resolve_net_to_class(net_names, class_params, patterns):
    """Map net_name → class_name.

    Only exact assignments are supported: a netclass pattern that, on this board, KiCad
    would match to the one net of that name. A
    pattern KiCad would also match to other nets (as a wildcard or a regex), a bus pattern,
    or a net that lands in more than one class (KiCad merges them) is an error.
    """
    names = [n for n in net_names if n]
    for pat, cls in patterns:
        if _BUS_PATTERN.search(pat):
            raise ValueError(f'netclass pattern {pat!r} ({cls}) is a bus pattern — not supported')
        readings = _kicad_pattern_readings(pat)
        wider = sorted(n for n in names if n != pat and any(r.match(n) for r in readings))
        if wider:
            raise ValueError(f'netclass pattern {pat!r} ({cls}) also matches {wider[:5]} as KiCad '
                             'reads it (wildcard / regex) — only exact net names are supported')
    out = {}
    for name in names:
        found = {cls for pat, cls in patterns if pat == name}
        if len(found) > 1:
            raise ValueError(f'net {name!r} is in net classes {sorted(found)} — KiCad merges them; '
                             'not supported')
        out[name] = found.pop() if found else 'Default'
    return out


# ─────────────────────────────────────────────
# DSN parsing utilities
# ─────────────────────────────────────────────

# A DSN token: a quoted string, a paren, or a bare word.
# pcbnew quotes without escaping (OUTPUTFORMATTER), so a quoted name runs to the next quote.
_DSN_TOKEN = re.compile(r'"[^"]*"|[()]|[^\s()"]+')


def dsn_section(dsn_content, head):
    """The first ``(head ...)`` block as nested lists of tokens, quotes stripped.

    Starts at the block itself, so pcbnew's ``(string_quote ")`` header never reaches the tokenizer.
    """
    m = re.search(r'\(' + head + r'(?=[\s()])', dsn_content)
    if not m:
        raise ValueError(f'DSN has no ({head} ...) block')
    stack = [[]]
    for t in _DSN_TOKEN.finditer(dsn_content, m.start()):
        tok = t.group()
        if tok == '(':
            stack.append([])
        elif tok == ')':
            done = stack.pop()
            stack[-1].append(done)
            if len(stack) == 1:
                return done
        else:
            stack[-1].append(tok[1:-1] if tok.startswith('"') else tok)
    raise ValueError(f'DSN ({head} ...) block is not closed')


def _children(block, head):
    return [c for c in block if isinstance(c, list) and c and c[0] == head]


def extract_dsn_net_names(dsn_content):
    """Net names of the DSN's (network (net NAME ...)) entries."""
    return [n[1] for n in _children(dsn_section(dsn_content, 'network'), 'net')
            if len(n) > 1 and isinstance(n[1], str)]


def dsn_via_padstacks(dsn_content):
    """The via padstack names the export declares in ``(structure (via "A" "B" ...))``."""
    names = []
    for via in _children(dsn_section(dsn_content, 'structure'), 'via'):
        names.extend(t for t in via[1:] if isinstance(t, str))
    if not names:
        raise ValueError('DSN structure declares no via padstack')
    return names


_VIA_NAME = re.compile(r'Via\[(\d+-\d+)\]_')


def dsn_via_layer_span(padstacks):
    """The ``<top>-<bot>`` layer span pcbnew gives every netclass via of the board."""
    spans = set()
    for name in padstacks:
        m = _VIA_NAME.match(name)
        if not m:
            raise ValueError(f'DSN via padstack {name!r} is not named Via[<top>-<bot>]_...')
        spans.add(m.group(1))
    if len(spans) != 1:
        raise ValueError(f'DSN via padstacks span several layer pairs: {sorted(spans)}')
    return spans.pop()


def via_padstack_name(dia_mm, drill_mm, layer_span):
    """pcbnew's padstack name for a netclass via (specctra_export.cpp makeVia):
    ``Via[<top>-<bot>]_<diameter µm, %.6g>:<drill µm, %.6g>_um``, with pcbnew's own
    nm → µm arithmetic (``scale()`` for the diameter, ``IU2um()`` for the drill)."""
    dia_um = kicad_nm(dia_mm) / (1e6 / 1000.0)
    drill_um = kicad_nm(drill_mm) * (1000.0 / 1e6)
    return f'Via[{layer_span}]_{dia_um:.6g}:{drill_um:.6g}_um'


def _dsn_net_token(name):
    """A name as pcbnew writes it (OUTPUTFORMATTER::GetQuoteChar): quoted when empty, starting
    with '#', holding a tab, space, ( ) % { }, or a '-' after the first character."""
    if (not name or name.startswith('#') or any(c in '\t ()%{}' for c in name)
            or '-' in name[1:]):
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


def class_via_name(class_params, cls_name, padstacks):
    """The via padstack of ``cls_name``; it must be one the export declared."""
    params = effective_class_params(class_params, cls_name)
    name = via_padstack_name(params['via_dia'], params['via_drill'], dsn_via_layer_span(padstacks))
    if name not in padstacks:
        raise ValueError(f'net class {cls_name!r}: via padstack {name!r} is not declared in the DSN '
                         f'(it declares {padstacks})')
    return name


def build_dsn_classes(net_to_class, class_params, padstacks, steps_per_um):
    """
    Build DSN class block text from net_to_class + class_params:
    per-class track width, clearance and via padstack.
    """
    class_nets = defaultdict(list)
    for net_name, cls in net_to_class.items():
        class_nets[cls].append(net_name)

    lines = []
    for cls_name in sorted(class_nets.keys()):
        nets = class_nets[cls_name]
        if not nets:
            continue
        params = effective_class_params(class_params, cls_name)
        width_dsn = mm_to_dsn(params['trace_width'], steps_per_um)
        clear_dsn = mm_to_dsn(params['clearance'], steps_per_um)
        via_name = class_via_name(class_params, cls_name, padstacks)

        net_tokens = ' '.join(_dsn_net_token(n) for n in sorted(nets))
        lines.append(f'    (class {_dsn_net_token(cls_name)} {net_tokens}')
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


def fix_dsn_classes(dsn_content, pro):
    """
    Replace pcbnew's class blocks with per-class blocks derived from the
    kicad_pro netclass info (``pro``: ProClasses, or the .kicad_pro dict).
    Returns: (new_dsn, class_summary)
    """
    pro = _pro_classes(pro)
    class_params, patterns = pro.class_params, pro.patterns

    net_names = extract_dsn_net_names(dsn_content)
    padstacks = dsn_via_padstacks(dsn_content)

    net_to_class = resolve_net_to_class(net_names, class_params, patterns)
    class_text = build_dsn_classes(net_to_class, class_params, padstacks,
                                   dsn_resolution(dsn_content))

    dsn_no_class = _remove_dsn_class_blocks(dsn_content)
    new_dsn = insert_classes_before_wiring(dsn_no_class, class_text)

    class_nets = defaultdict(list)
    for n, c in net_to_class.items():
        class_nets[c].append(n)
    summary = {
        'classes': len(class_nets),
        'nets': len(net_names),
        'warnings': list(pro.warnings),
        'class_detail': {
            c: {'count': len(ns),
                'width_mm': effective_class_params(class_params, c)['trace_width'],
                'clearance_mm': effective_class_params(class_params, c)['clearance'],
                'via': class_via_name(class_params, c, padstacks)}
            for c, ns in class_nets.items()
        },
    }
    return new_dsn, summary


# ─────────────────────────────────────────────
# DSN verification
# ─────────────────────────────────────────────

def verify_dsn_constraints(dsn_content, pro):
    """
    Verify that every DSN class block carries its kicad_pro net class's width
    and clearance (on the DSN grid: not below, less than one step above) and
    the via padstack of that class, declared in the structure.
    ``pro``: ProClasses, or the .kicad_pro dict.
    Returns: (ok: bool, issues: [str])
    """
    class_params = _pro_classes(pro).class_params
    steps_per_um = dsn_resolution(dsn_content)
    padstacks = dsn_via_padstacks(dsn_content)

    issues = []
    classes = _children(dsn_section(dsn_content, 'network'), 'class')
    if not classes:
        issues.append('DSN has no class block')
    for block in classes:
        cls_name = block[1] if len(block) > 1 and isinstance(block[1], str) else 'unknown'
        if cls_name not in class_params:
            issues.append(f'[{cls_name}] not a net class of the .kicad_pro')
            continue
        params = effective_class_params(class_params, cls_name)
        rules = {}
        for rule in _children(block, 'rule'):
            for r in rule[1:]:
                if isinstance(r, list) and len(r) == 2 and r[0] in ('width', 'clearance'):
                    rules[r[0]] = dsn_to_mm(r[1])
        for key, pro_key in (('width', 'trace_width'), ('clearance', 'clearance')):
            if key not in rules:
                issues.append(f'[{cls_name}] missing {key}')
            elif not rule_matches(rules[key], params[pro_key], steps_per_um):
                issues.append(f'[{cls_name}] {key} mismatch: DSN={rules[key]}mm, '
                              f'pro={params[pro_key]}mm')
        use_via = [u[1] for c in _children(block, 'circuit') for u in _children(c, 'use_via')
                   if len(u) > 1]
        expected_via = via_padstack_name(params['via_dia'], params['via_drill'],
                                         dsn_via_layer_span(padstacks))
        if use_via != [expected_via]:
            issues.append(f'[{cls_name}] use_via {use_via} is not {expected_via!r}')
        elif expected_via not in padstacks:
            issues.append(f'[{cls_name}] via padstack {expected_via!r} is not declared')

    return len(issues) == 0, issues
