# `pcbnew_prep/` — Pre-conversion (kicad_pcb → DSN + ORP)

These scripts produce the `.dsn` and `.orp` files that the environment repository's
rule-based runner (`methods/baselines/rule_based/run_rule_based_routers.py`) expects to
find next to each raw `.kicad_pcb`. They are **verbatim copies** of
the original `data_convert/make_dsn_orp_*.py` prep scripts with the hardcoded
absolute constants replaced by required env vars.

The unified runner does **not** call these at runtime — they must be executed
once per dataset on a host where the system Python's `pcbnew` module is
available (typical KiCad install on Ubuntu / a CI image with KiCad 9).

## Scripts

| Script | Target | Env vars |
|---|---|---|
| `make_dsn_orp_v3.py` | pcbench (`exacad_sorted`) | `PCBENCH_PCB_ROOT`, `PCBENCH_DSN_ROOT` |
| `make_dsn_orp_synth.py` | synth_2L_v2 | `SYNTH2L_PCB_ROOT`, `SYNTH2L_DSN_ROOT` |
| `make_dsn_orp_synth_1l.py` | synth_1L grid50 | `SYNTH1L_PCB_ROOT`, `SYNTH1L_DSN_ROOT` |
| `make_dsn_orp_synth_1l_g10.py` | synth_1L grid10 | (same) |
| `make_dsn_orp_synth_1l_g50.py` | synth_1L grid50 | (same) |
| `make_dsn_orp_synth_1l_g100.py` | synth_1L grid100 | (same) |
| `make_dsn_orp_synth_1l_g200.py` | synth_1L grid200 | (same) |
| `make_dsn_orp_synth_1l_g500.py` | synth_1L grid500 | (same) |

The `make_dsn_orp_synth_1l*.py` family is six near-identical scripts, one per
grid variant (the original convention from `data_convert/`). Set the
env vars to point at the matching `synth_1L_grid<G>_5net_v02{,_dsn}` directories
before running each.

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
export PCBENCH_PCB_ROOT=$CADAGENT_DATA_ROOT/pcbench/exacad_sorted
export PCBENCH_DSN_ROOT=$CADAGENT_DATA_ROOT/pcbench/exacad_sorted_dsn
/usr/bin/python3 pcbnew_prep/make_dsn_orp_v3.py            # all 679 folders
/usr/bin/python3 pcbnew_prep/make_dsn_orp_v3.py --limit 5  # smoke

# synth_2L_v2
export SYNTH2L_PCB_ROOT=$CADAGENT_DATA_ROOT/synthetic/synth_2L_v2
export SYNTH2L_DSN_ROOT=$CADAGENT_DATA_ROOT/synthetic/synth_2L_v2_dsn
/usr/bin/python3 pcbnew_prep/make_dsn_orp_synth.py

# synth_1L grid50
export SYNTH1L_PCB_ROOT=$CADAGENT_DATA_ROOT/synthetic/synth_1L_grid50_5net_v02
export SYNTH1L_DSN_ROOT=$CADAGENT_DATA_ROOT/synthetic/synth_1L_grid50_5net_v02_dsn
/usr/bin/python3 pcbnew_prep/make_dsn_orp_synth_1l_g50.py
```

> `pcbnew_env/` holds the container recipe the shipped mirrors were originally built
> with. No script or setup step invokes it — it is a manual historical record.
>
> Any interpreter that imports `pcbnew` works. The engine's source build serves the
> conda env's python via `PYTHONPATH=build_rl/pcbnew python …`; `/usr/bin/python3`
> is the interpreter an apt-installed KiCad extends.
>
> Verified 2026-09-01 with the source-built pcbnew 9.0.8 on
> `synth_1L_grid10_5net_v02/test` (5 boards) against the shipped DSN/ORP mirror:
> `.orp` byte-identical (export timestamp aside); `.dsn` byte-identical (output
> path + `host_version` lines aside) once the script's `SetCopperLayerCount(1)`
> line is accounted for — that line postdates the mirror (added 2026-05-31, the
> mirror was generated before it), so the shipped mirror is the 2-copper-layer
> variant while today's script emits the 1-layer variant, locally and in Docker
> alike.

## Expected pre-converted layout under `$CADAGENT_DATA_ROOT`

| dataset | pre-converted DSN+ORP | location |
|---|---|---|
| pcbench (679 folders) | required by the runner | `pcbench/exacad_sorted_dsn/` |
| synth_2L_v2/test (128 boards) | required by the runner | `synthetic/synth_2L_v2_dsn/test/` |
| synth_1L_grid{10,50,100,200,500}/test | optional | `synthetic/synth_1L_grid<G>_5net_v02_dsn/` |
| synth_1L_grid1000 | run prep script first | — |

If new datasets are added, re-run the appropriate script with env vars
pointing at the new dataset's directories.
