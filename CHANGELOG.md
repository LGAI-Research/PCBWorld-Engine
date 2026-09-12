# Changelog

Release notes for the PCBWorld Engine, newest first. This is a separate program from the
PCBWorld environment, which pins one engine commit per environment release.

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
