"""Length units of the net-class rules the make_dsn_orp_* scripts write into a DSN.

A Specctra DSN declares its length grid in the header, ``(resolution um N)``:
values are in µm and the finest step is 1/N µm (pcbnew's ExportSpecctraDSN
writes N = 10). A rule is written on that grid, rounded UP, so the router never
gets a track narrower or a clearance smaller than the KiCad rule it came from —
writing whole µm turned 0.2032 mm (8 mil) into ``(width 203)``, below the board
minimum.

No pcbnew import here, so the conversion is testable without a KiCad build.
"""

import math
import re

MM2UM = 1000


def dsn_resolution(dsn_content):
    """Steps per µm from the DSN header ``(resolution um N)``."""
    m = re.search(r'\(resolution\s+(\S+)\s+(\d+)\s*\)', dsn_content)
    if not m:
        raise ValueError('DSN has no `(resolution um N)` header — cannot size the class rules')
    unit, steps = m.group(1), int(m.group(2))
    if unit != 'um':
        raise ValueError(f'DSN `(resolution {unit} {steps})` is not in um — cannot size the class rules')
    # A rule is written as a decimal µm token; only a power-of-ten grid has an exact one.
    if str(steps).rstrip('0') != '1':
        raise ValueError(f'DSN `(resolution um {steps})` is not a power of ten — '
                         'a class rule on that grid has no exact decimal µm form')
    return steps


def kicad_nm(mm):
    """``mm`` in KiCad's internal unit, whole nm, rounded as KiCad's KiRound does (half away from zero)."""
    return int(math.floor(mm * 1e6 + 0.5)) if mm >= 0 else -int(math.floor(-mm * 1e6 + 0.5))


def mm_to_dsn(mm, steps_per_um):
    """``mm`` as a DSN µm token on the file's grid, rounded up to the next step."""
    if not (math.isfinite(mm) and kicad_nm(mm) > 0):
        raise ValueError(f'a net-class rule must be a positive length (whole nm), got {mm!r} mm')
    # KiCad holds the rule in whole nm; ceil that onto the grid in integers (no float noise).
    steps = -(-kicad_nm(mm) * steps_per_um // 1000)
    whole, frac = divmod(steps, steps_per_um)
    if frac == 0:
        return str(whole)
    # steps_per_um is a power of ten (dsn_resolution): the fraction is exact in decimal.
    digits = len(str(steps_per_um)) - 1
    return f'{whole}.{frac:0{digits}d}'.rstrip('0')


def dsn_to_mm(token):
    """A DSN µm token back to mm."""
    return float(token) / MM2UM


def rule_matches(dsn_mm, rule_mm, steps_per_um):
    """True when ``dsn_mm`` is ``rule_mm`` (as KiCad holds it, whole nm) on the DSN grid:
    not below it, less than one step above."""
    diff_nm = dsn_mm * 1e6 - kicad_nm(rule_mm)
    return -1e-6 <= diff_nm < 1000.0 / steps_per_um - 1e-6
