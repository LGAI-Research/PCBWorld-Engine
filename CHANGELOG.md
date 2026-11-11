# Changelog

Release notes for the PCBWorld Engine, newest first. This is a separate program from the
PCBWorld environment, which pins one engine commit per environment release.

## v1.2.0 — 2026-11-11

Determinism and build release. Wire protocol still 2, routing calls unchanged. The ratsnest behind
`get_ratsnest` and `get_routing_target` is now ordered by board content: where one net has several
clusters at one position (a front and a back pad at the same spot) it no longer varies run to run
and differs from v1.1.0; on any board, which of several equal-length edges is reported can differ,
while the number of edges per net is the same.

### Added
- macOS (Apple Silicon) build — `build_rl_router.sh` builds inside a conda env that has clang, from
  the env alone; package list in `README.md`, optional `BUILD_PCBNEW` / `BUILD_CLI` targets not yet verified there

### Changed
- `build_rl_router.sh` — refuses a `BUILD_DIR` that is a symlink and names its target
- `kicad-patches/ENGINE_VERSION` 1.7 → 1.9 — the router source hash changed, rebuild before use

### Fixed
- Ratsnest order — MST anchors sort by content (position, layer range, parent type and UUID, zone
  sub-polygon, anchor index) instead of heap address and edges sort stably, so coincident anchors no longer follow allocation order
- `PCBWorld-mod` markers — the single-position anchor branch of `ratsnest_data.cpp`, modified before this release, now carries one (comments only)

## v1.1.0 — 2026-10-20

Routing release. Wire protocol still 2. Each route now takes its sizes from the start net's class,
so boards with more than one net class route differently from v1.0.1; single-class boards route
identically.

### Added
- `trim_at_contact` (on `fix_route` / `finish`) and `optimize_all_lines` — opt-in copper clean-up
  and whole-board re-optimization, off unless called

### Changed
- Route sizes per net class — `start_route` imports the start net's track width and via size
  (KiCad `ImportSizes`, floored at board minima)
- Size setters — `set_track_width` / `set_via_diameter` / `set_via_drill` override the open session
  only and raise on ≤ 0; `set_via_placement(on)` replaces `toggle_via` / `reset_via_mode`

### Fixed
- Locked tracks the router splits stay locked; a failed rules load, router DRC build or restore
  re-open raises instead of continuing silently

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
