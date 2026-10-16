# Changelog

Release notes for the PCBWorld Engine, newest first. This is a separate program from the
PCBWorld environment, which pins one engine commit per environment release.

## v1.0.2 — KiCad 10.0.7 port

- Pin the KiCad source at tag `10.0.7`, commit `69cafb907ebc09e7722a2a464b1d466fc2e3608d`; `build_rl_router.sh` enforces the commit because the nested gitlink is owned by the upstream `kicad-python` submodule.
- Remove property-grid, DRC R-tree, and CLI-render overlays whose changes are now upstream; retain the Engine DRC report-cap change and bounded `Finish(int)` adaptation.
- Preserve RL API behavior and wire protocol version 2, including the handshake, field order, and constants.
- On macOS, resolve Python symbols from the embedding interpreter and report the actual app-bundle `kicad-cli` path.
- Bump `kicad-patches/ENGINE_VERSION` from 1.4 to 1.5.

## v1.0.1 — 2026-09-16

Build-provenance release. Router sources unchanged since v1.0.0 — no patch, CMake or
`kicad-python` change, wire protocol still 2 — either release routes identically.

- `build_rl_router.sh` — stamps the module with `ENGINE_CPP_HASH`, the C++ patch-tree content hash,
  taken before the source copy; a hash failure stops the build
- `tools/cpp_content_hash.sh` (new) — the one implementation: sha256 over each source file's path
  and bytes under `kicad-patches/`, 8 hex chars. PCBWorld refuses a mismatched stamp; a v1.0.0
  build warns until rebuilt
- `kicad-patches/ENGINE_VERSION` 1.3 → 1.4

## v1.0.0 — 2026-09-11

First public release: KiCad 9.0.8's PNS push-and-shove router, patched and wrapped as the
RL routing engine of [PCBWorld](https://github.com/LGAI-Research/PCBWorld).

- `pcbnew/router` patches expose the interactive router to a Python binding (`kicad_rl_router`)
  with an RL-facing API: board load, net selection, start / make_line / make_via / finish
  primitives, walkaround and shove modes, incremental DRC, and JSON observations of the board.
- Deterministic routing: PNS items are ordered by geometry (kind → layer → anchor coordinates)
  with UUIDs only as a tie-break, and every routing session canonicalises the world order on
  start — the same geometry and action give the same result across re-saved boards and reused
  servers. Shove and walkaround loops are bounded by iteration limits rather than wall-clock time.
- The engine server (`engine_server/`) is the only process that imports the GPL module; the
  environment talks to it over a unix socket (wire protocol version 2). Startup is hardened:
  the socket is published only after `listen`, an orphaned server exits, the client retries
  within a startup deadline.
- DRC: the per-type report cap of stock KiCad is removed (violation counts are exact), the
  R-tree entry leak of `DRC_RTREE::clear()` in 9.0.8 is fixed, and board outlines include
  footprint-owned Edge.Cuts items (boards whose outline is drawn inside a footprint load).
- `build_rl_router.sh` builds the module from the pinned `kicad-python` submodule with the
  patches applied; `BUILD_CLI=1 BUILD_PCBNEW=1` add `kicad-cli` and the `pcbnew` Python module
  from the same source, so no distribution KiCad is needed to prepare board data. The exact
  upstream diff is generated into `docs/upstream-diff/`, and every modified region carries a
  `PCBWorld-mod begin/end` marker.
