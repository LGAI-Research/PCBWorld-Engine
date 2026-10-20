# `pcbnew_prep/` — Pre-conversion (kicad_pcb → DSN + ORP)

These scripts produce the `.dsn` and `.orp` files that the environment repository's
rule-based runner (`methods/baselines/rule_based/run_rule_based_routers.py`) expects to
find next to each raw `.kicad_pcb`. They are the original
`data_convert/make_dsn_orp_*.py` prep scripts with the hardcoded absolute
constants replaced by required env vars; all three share the DSN net-class
rewrite in `dsn_classes.py` and its units in `dsn_units.py`.

The unified runner does **not** call these at runtime — they must be executed
once per dataset on a host where the system Python's `pcbnew` module is
available (typical KiCad install on Ubuntu / a CI image with KiCad 9).

## Scripts

| Script | Target | Env vars |
|---|---|---|
| `make_dsn_orp_v3.py` | pcbench (`exacad_sorted`) | `PCBENCH_PCB_ROOT`, `PCBENCH_DSN_ROOT` |
| `make_dsn_orp_synth.py` | synth_2L_v2 | `SYNTH2L_PCB_ROOT`, `SYNTH2L_DSN_ROOT` |
| `make_dsn_orp_synth_1l.py` | synth_1L grid10/50/100/200/500 | `SYNTH1L_PCB_ROOT`, `SYNTH1L_DSN_ROOT` |

`make_dsn_orp_synth_1l.py` serves every grid: point the env vars at the matching
`synth_1L_grid<G>_5net_v02{,_dsn}` directories. The boards' stackup declares F.Cu + B.Cu but they
are routed on F.Cu alone (the RL side masks `make_via`), so the script exports the DSN with one
copper layer; the `.orp` comes from the unmodified board and lists both. The shipped
`synth_1L_grid<G>_5net_v02_dsn` mirrors are 2-layer DSNs (the runner sets FreeRouting's vias off on
every `synthetic_1l` board); a FreeRouting result on a regenerated mirror is not shown to equal one
on the shipped mirror.

## Net-class rules in the DSN

Each script rewrites the DSN class blocks from the `.kicad_pro` (KiCad 9's format only,
[`kicad9_pro_format.json`](kicad9_pro_format.json); an unset field comes from `Default`):
width and clearance on the DSN grid, rounded up from KiCad's whole nm (0.2032 mm is
`(width 203.2)`); `use_via` = the padstack pcbnew exported for the class's via; names quoted as
pcbnew quotes them; nets by `netclass_patterns` only, by exact name (a pattern KiCad matches to
that one net). A class `nets` entry no pattern repeats (KiCad 9 does not read it; a Default one
no pattern names is skipped) is a warning naming the `.kicad_pro` and class, in the board's
result and the run summary (first and last 20). A board fails — status `fail`, no `.dsn` or
`.orp`, non-zero exit — on anything else: a header not `um` with a power-of-ten N, a missing
`.kicad_pro` or `Default`, another `.kicad_pro` format, `netclass_assignments`, a pattern KiCad
matches to other nets or expands as a bus, a net in two classes, a rule missing or not
positive, an unexported padstack, or a `verify_dsn_constraints` issue.

## Why each script needs pcbnew

The kicad_pcb → DSN conversion uses `pcbnew.ExportSpecctraDSN()` (the same
function KiCad's UI emits for export-to-Specctra). The ORP build path reads
pad / net / layer / DRC information through the same `pcbnew` bindings.

No replacement is available in pure Python. The engine's own build provides it:
`BUILD_CLI=1 BUILD_PCBNEW=1 bash build_rl_router.sh` puts `pcbnew.py` + `_pcbnew.so`
in `build_rl/pcbnew` (see the root README, "Build"), importable by the interpreter
the build was configured with. A host KiCad install
(`/usr/lib/python3/dist-packages/pcbnew`) works as well.

## Typical invocation

```bash
# pcbench
export PCBENCH_PCB_ROOT=$PCBWORLD_DATA_ROOT/pcbench/exacad_sorted
export PCBENCH_DSN_ROOT=$PCBWORLD_DATA_ROOT/pcbench/exacad_sorted_dsn
/usr/bin/python3 pcbnew_prep/make_dsn_orp_v3.py            # all 679 folders
/usr/bin/python3 pcbnew_prep/make_dsn_orp_v3.py --limit 5  # smoke

# synth_2L_v2
export SYNTH2L_PCB_ROOT=$PCBWORLD_DATA_ROOT/synthetic/synth_2L_v2
export SYNTH2L_DSN_ROOT=$PCBWORLD_DATA_ROOT/synthetic/synth_2L_v2_dsn
/usr/bin/python3 pcbnew_prep/make_dsn_orp_synth.py

# synth_1L grid50
export SYNTH1L_PCB_ROOT=$PCBWORLD_DATA_ROOT/synthetic/synth_1L_grid50_5net_v02
export SYNTH1L_DSN_ROOT=$PCBWORLD_DATA_ROOT/synthetic/synth_1L_grid50_5net_v02_dsn
/usr/bin/python3 pcbnew_prep/make_dsn_orp_synth_1l.py
```

> `pcbnew_env/` holds the container recipe the shipped mirrors were originally built
> with. No script or setup step invokes it — it is a manual historical record.
>
> Any interpreter that imports `pcbnew` works. The engine's source build serves the
> conda env's python via `PYTHONPATH=build_rl/pcbnew python …`; `/usr/bin/python3`
> is the interpreter an apt-installed KiCad extends.
>
> Checked 2026-10-12 with the source-built pcbnew 9.0.8 on 3 `synth_1L_grid50_5net_v02/test` boards
> against the shipped mirror: `.orp` identical apart from `export_timestamp`; `.dsn` identical apart from
> line 1 (the export's file name), `host_version` and the shipped DSN's B.Cu references (the B.Cu layer,
> the via padstack's B.Cu shape, `Via[0-1]` for `Via[0-0]`).

## Expected pre-converted layout under `$PCBWORLD_DATA_ROOT`

| dataset | pre-converted DSN+ORP | location |
|---|---|---|
| pcbench (679 folders) | required by the runner | `pcbench/exacad_sorted_dsn/` |
| synth_2L_v2/test (128 boards) | required by the runner | `synthetic/synth_2L_v2_dsn/test/` |
| synth_1L_grid{10,50,100,200,500}/test | optional | `synthetic/synth_1L_grid<G>_5net_v02_dsn/` |
| synth_1L_grid1000 | run prep script first | — |

The environment repository's `tools/quickstart/prepare_pcbench.sh` runs `make_dsn_orp_v3.py`
as the last step of its PCBench chain, so the pcbench row is filled there. If new datasets are
added, re-run the appropriate script with env vars pointing at the new dataset's directories.
