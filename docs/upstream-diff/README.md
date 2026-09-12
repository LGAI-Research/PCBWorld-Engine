# upstream-diff — what this engine changes in KiCad (read-only view)

Every file here is **generated** by `tools/make_upstream_diff.sh` from the full copies in
`kicad-patches/kicad/` (our modified upstream files, the only source) and the pristine
upstream checkout in `kicad-python/` (the pinned submodule). One unified diff per modified
file, in `patch -p1` form, so a reader sees exactly which lines this engine changes.

- **Do not edit** anything in this directory; edit the full copy under `kicad-patches/kicad/`
  and re-run the script. The environment repository's doc checker regenerates this view and
  fails when it differs from the tracked files.
- **The build never reads this directory.** `build_rl_router.sh` overlays the full copies onto
  the upstream sources; there is no patch-apply path and no fallback.
- The view is proved on every regeneration: applying every diff to the pristine upstream
  files must reproduce the full copies byte for byte.
- Files under `kicad-patches/rl/` are this engine's own sources (not modified upstream files)
  and are therefore not listed here.
