# PCBWorld Engine — GPLv3

This repository is a **separate program** from the PCBWorld environment, which lives in
its own repository. It is licensed under the [GNU General Public License v3](LICENSE),
because it links KiCad.

The environment does not link this program and does not import any module from it. It
starts `engine_server/rl_engine_server.py` as a child process and exchanges plain
scalars, strings and lists over a unix socket. **The two are never combined into one
distributable** — no wheel, no container image, no installer contains both.

The environment repository, [LGAI-Research/PCBWorld](https://github.com/LGAI-Research/PCBWorld),
pins this one as its `engine/` submodule: a URL and a commit hash, and nothing of this
tree is distributed from there.

## Contents

| Path | What |
|---|---|
| [`kicad-python/`](kicad-python) | git submodule, pinned to upstream KiCad 9.0.8. Not redistributed by this repository — `git submodule update --init --recursive` fetches it from upstream |
| [`kicad-patches/kicad/`](kicad-patches/kicad) | our modified copies of the upstream KiCad files the build replaces, one full file per upstream path |
| [`kicad-patches/rl/`](kicad-patches/rl) | our C++ sources: the headless RL router, its DRC fork, and the pybind11 module `kicad_rl_router`. GPLv3 because they link KiCad |
| [`CHANGELOG.md`](CHANGELOG.md) | the release notes, newest first — one `## vX.Y.Z` section per published release |
| [`kicad-patches/ENGINE_VERSION`](kicad-patches/ENGINE_VERSION) | the engine version, stamped next to the built module as a build-provenance marker |
| [`engine_server/`](engine_server) | the server process. The only process in the project that imports `kicad_rl_router` |
| [`pcbnew_prep/`](pcbnew_prep) | dataset pre-conversion scripts (`kicad_pcb` → DSN/ORP). They import KiCad's `pcbnew` Python module, so they live on this side of the boundary — the module built here with `BUILD_PCBNEW=1` (or any host KiCad's python) |
| [`build_rl_router.sh`](build_rl_router.sh) | rsync the submodule into `build_rl/kicad_src`, drop in `kicad-patches/`, run cmake + ninja (`BUILD_CLI=1 BUILD_PCBNEW=1` add `kicad-cli` and the `pcbnew` module, see Build) |
| [`docs/upstream-diff/`](docs/upstream-diff) | **generated, read-only**: one unified diff per modified upstream file — exactly what this engine changes in KiCad. Not a build input; regenerate with `tools/make_upstream_diff.sh` |
| [`tools/`](tools) | `make_upstream_diff.sh` (regenerates the view above and proves it round-trips) · `diff_patches.sh` (ad-hoc diff summary against the pinned upstream) |

## Build

```bash
git submodule update --init --recursive       # from the repository root
bash build_rl_router.sh
python -c 'import sys; sys.path.insert(0, "build_rl/pcbnew/python/rl"); import kicad_rl_router; print("OK")'
```

Output: `build_rl/pcbnew/python/rl/kicad_rl_router.so`, stamped with a copy of
`kicad-patches/ENGINE_VERSION` for build provenance.

`BUILD_DIR` overrides where the build lands; the environment sets it to its own tree so
the module is built once, next to the code that spawns this server.

### Optional targets: `kicad-cli` and the `pcbnew` Python module

The default build is the RL module alone — all the environment needs, and fast. Two
switches add the headless KiCad tools that the environment's data-preparation chain runs
as child processes, built from the same pinned, patched source:

```bash
BUILD_CLI=1 BUILD_PCBNEW=1 bash build_rl_router.sh
KICAD_RUN_FROM_BUILD_DIR=1 build_rl/kicad/kicad-cli --version                          # 9.0.8
PYTHONPATH=build_rl/pcbnew python -c 'import pcbnew; print(pcbnew.GetBuildVersion())'   # 9.0.8
```

| switch | ninja targets | output |
|---|---|---|
| `BUILD_CLI=1` | `kicad-cli` | `build_rl/kicad/kicad-cli` |
| `BUILD_PCBNEW=1` | `pcbnew_kiface pcbnew_python_module` | `build_rl/pcbnew/_pcbnew.kiface` and, beside it, `pcbnew.py` + `_pcbnew.so` (a symlink to the kiface): `build_rl/pcbnew` on `PYTHONPATH` is the whole install, for the interpreter the build was configured with (`PYTHON_EXECUTABLE`) |

- `kicad-cli pcb …` loads `_pcbnew.kiface`, so `BUILD_CLI=1` is useful only together with
  `BUILD_PCBNEW=1`. Run from the build tree, `kicad-cli` finds the kiface (one directory up,
  in `pcbnew/`) only with `KICAD_RUN_FROM_BUILD_DIR=1` in its environment; without it every
  `pcb` command fails with "Failed to load kiface library".
- `BUILD_PCBNEW=1` needs SWIG (`swig` on `PATH`) and OpenCASCADE: the kiface carries the
  STEP exporter. The script hands the Homebrew or the active conda env's OCC to CMake
  whenever one is present — for engine-only builds too, so the configuration does not flip
  between the two invocations of one build tree. The RL module itself never touches OCC.
- Cost (64-core host): the three targets together are 1 352 ninja steps from scratch, about
  4.5 minutes. A tree configured before OCC was available recompiles everything once when
  OCC first lands (the compile flags change). Re-running the script with unchanged sources
  is a link-only no-op — KiCad regenerates its version header on every build, so 7–13 steps
  and about 15 seconds, with either switch setting.
- The DRC these tools run is this engine's: the `drc_engine.cpp` patch lifts the per-type
  report caps (199 / 499 in stock KiCad), so `kicad-cli pcb drc` reports every violation.

## Test

The tests that exercise the binding directly live in the environment repository
(`tests/test_engine_api/`, `tests/stress/`): they run against the built `.so`, so they
belong wherever the build is driven from. This repository's own gate is the build plus
the import smoke above.

## Protocol

`engine_server/wire.py` defines the protocol the two programs speak: plain mirror
types for every value the binding returns, the field-order registry `KRL_FIELDS`, the
module constants exchanged in the handshake, and the encode/decode pair. An identical
copy sits at `pcb_world/engine/wire.py` in the environment repository, so neither
program imports a module from the other; the environment's
`tools/check_separation.py` verifies the two copies match byte for byte.

The server validates `KRL_FIELDS` against the live binding at startup and sends it in
the handshake, which the client compares against its own copy. A binding whose fields
drifted therefore fails loudly on both sides instead of silently mis-decoding.

Requests are length-prefixed pickles: `construct`, `call`, `batch`, `module_call`,
`close_router`, `ping`. Nothing but primitives, lists, tuples and dicts crosses the
socket — no KiCad type ever leaves this process. There are no per-call timeouts: a
single shove can legitimately take minutes.

## Contact

Questions and bug reports about the engine: open an issue here, or write to
hyungseok.song@lgresearch.ai. Issues about the environment, training or
evaluation belong in the environment repository.
