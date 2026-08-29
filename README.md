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
| [`kicad-patches/ENGINE_VERSION`](kicad-patches/ENGINE_VERSION) | the engine version, stamped next to the built module as a build-provenance marker |
| [`engine_server/`](engine_server) | the server process. The only process in the project that imports `kicad_rl_router` |
| [`pcbnew_prep/`](pcbnew_prep) | dataset pre-conversion scripts (`kicad_pcb` → DSN/ORP). They import KiCad's `pcbnew` Python module, so they live on this side of the boundary |
| [`build_rl_router.sh`](build_rl_router.sh) | rsync the submodule into `build_rl/kicad_src`, drop in `kicad-patches/`, run cmake + ninja |

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
