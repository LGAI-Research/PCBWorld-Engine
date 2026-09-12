# pcbnew environment for reproducing the synth DSN/ORP (historical recipe)

> **Nothing in this project runs this image.** The `pcbnew` every script uses is the
> engine's own build of the pinned 9.0.8 source — `BUILD_CLI=1 BUILD_PCBNEW=1 bash
> build_rl_router.sh`, then `PYTHONPATH=build_rl/pcbnew python …` (root README, "Build") —
> the same KiCad the router runs, with no apt/PPA/container in any code path or setup
> step. This directory is kept only as the manual record of how the shipped DSN/ORP
> mirrors were originally produced, and to reconstruct the historic apt build's
> `host_version` string by hand if anyone ever needs to.
>
> The source-built module's output is verified against those mirrors (2026-09-01, see
> `../README.md`): `.orp` byte-identical (timestamp aside); `.dsn` byte-identical
> (path + `host_version` aside) up to the script's own later `SetCopperLayerCount(1)`
> change, which no build reproduces the mirror through.

The synthetic-board Freerouting input (`*_unrouted.dsn`) and OrthoRoute
(`*.orp`) files are produced by **KiCad's `pcbnew` Python module** through
`pcbnew_prep/make_dsn_orp_synth_1l_g10.py`. This documents the exact
generator build, how to stand up a fresh environment, and the byte-level
reproduction results.

> Verified 2026-05-29 against the original generator environment. All
> comparisons below were taken with sentinel-guarded shell verdicts (not
> trusting raw stdout), because the result channel was intermittently garbling
> output during that session.

## The exact generator

Every reference DSN header records:

```
(host_cad "KiCad's Pcbnew")
(host_version "9.0.8-9.0.8~ubuntu22.04.1")
```

Target build: **KiCad `9.0.8-9.0.8~ubuntu22.04.1`** (Ubuntu 22.04 / jammy build
from the official KiCad stable PPA). `pcbnew` ships *inside* the apt `kicad`
package at `/usr/lib/python3/dist-packages/pcbnew.py`; there is no pip/conda-only
package that yields this build. The engine's source build (`BUILD_PCBNEW=1`) is the
same 9.0.8 code; only the `host_version` string differs (`9.0.8` vs
`9.0.8-9.0.8~ubuntu22.04.1`).

## The original generator environment

The reference DSN/ORP files were produced inside a Docker container running the
apt `kicad` 9.0.8 build, with the datasets bind-mounted under `/workspace/...`
— which is why the reference DSN headers embed `/workspace/...` paths. Note
that `pcbnew` lives in the **system** python's `/usr/lib/python3/dist-packages`
there, not in any conda python; probe with
`python3 -c "import pcbnew; print(pcbnew.GetBuildVersion())"`.

## Build a fresh env (Docker)

```bash
cd pcbnew_prep/pcbnew_env
docker build -t pcbnew-repro:9.0 .
docker run --rm pcbnew-repro:9.0 \
  python3 -c "import pcbnew; print(pcbnew.GetBuildVersion())"
```

Two gotchas the Dockerfile already handles:
1. The minimal `ubuntu:22.04` base lacks `gnupg`, so `add-apt-repository`'s key
   import fails — `gnupg` is installed first.
2. **The KiCad PPA keeps only the latest 9.0.x.** As of 2026-05-29 the candidate
   is `9.0.9-9.0.9~ubuntu22.04.1`; `9.0.8` has been superseded and is no longer
   `apt install`-able. The Dockerfile therefore installs plain `kicad` (→ 9.0.9).
   See the reproduction results for why this is still correct for the content.

To reproduce the **exact** `9.0.8` build (e.g. to match the `host_version`
string byte-for-byte), fetch the archived 9.0.8 debs from Launchpad
(`https://launchpad.net/~kicad/+archive/ubuntu/kicad-9.0-releases/+packages`,
filter version `9.0.8~ubuntu22.04.1`) and `dpkg -i` them. The routing-relevant
content is identical either way (below).

## Run the converter

Reads `SYNTH1L_PCB_ROOT/<split>/board_*.kicad_pcb` (+`.kicad_pro`), strips
routing, exports DSN, then patches netclass rules from the `.kicad_pro`. It
imports `remove_routing` from `setup_drvzero.py`, which sits **next to the
converter** in `pcbnew_prep/` (a self-contained copy; the converter
auto-adds its own directory to `sys.path`), so **no `PYTHONPATH` is needed**
and nothing outside this repo is required.

The converter and `setup_drvzero.py` are both in this repo, so the only
machine-specific inputs are (a) where you checked out the repo and (b) where your
`board_*.kicad_pcb` dataset lives. Parameterize both and mount them read-only:

```bash
# adjust these two to your machine; everything else is portable
REPO=$(git rev-parse --show-toplevel)         # this repo's root
DATA=/path/to/synth_1L_grid10_5net_v15        # dir containing <split>/board_*.kicad_pcb
OUT=/tmp/out_dsn                              # where to write DSN+ORP

docker run --rm \
  -v "$REPO":/repo:ro -v "$DATA":/data:ro -v "$OUT":/out \
  -e SYNTH1L_PCB_ROOT=/data \
  -e SYNTH1L_DSN_ROOT=/out \
  pcbnew-repro:9.0 \
  python3 /repo/pcbnew_prep/make_dsn_orp_synth_1l_g10.py \
  --splits test --limit 5
```

> Self-containment verified 2026-05-29: mounting only the repo (read-only),
> **without** `PYTHONPATH` and **without** the original prep tree, converts
> cleanly and reproduces the reference DSN body byte-for-byte (3/3 boards). The
> copied `setup_drvzero.py` is md5-identical to the original.

Files written by the container are owned by `root` (it runs as root); `chown`
them back, or clean up with
`docker run --rm -v "$OUT":/out pcbnew-repro:9.0 rm -rf /out/<...>`.

Pipeline (`process_board`): read `.kicad_pcb` → `remove_routing()` (text removal
of `(segment`/`(via`/`(zone`, sets `(tracks 0)`/`(zones 0)`) → write a temp
`_unrouted.kicad_pcb` → `pcbnew.LoadBoard` → `pcbnew.ExportSpecctraDSN` →
`fix_dsn_classes()` rewrites `(class ...)` clearance/width/via from `.kicad_pro`.

> A bare `ExportSpecctraDSN` is **not** enough: without `remove_routing` first,
> leftover vias/segments stay in the DSN; without `fix_dsn_classes` after, the
> netclass rules differ from the reference. (The converter does **no** header
> canonicalization — the `(pcb "…")` path and `host_version` come straight from
> pcbnew.)

## Byte-identity results (2026-05-29, grid10 v15 test)

Generated vs reference, line-by-line. Three fields legitimately vary per run and
are **not** produced by the conversion of board content:

| field | what it is | varies by |
|-------|-----------|-----------|
| DSN line 1 `(pcb "<path>")` | the output-filename argument you pass (`SYNTH1L_DSN_ROOT`) | output path |
| DSN `(host_version "…")` | the installed KiCad build string | KiCad version |
| ORP `"export_timestamp"` | wall-clock time the `.orp` was written | run time |

Result after accounting for those (5 boards each env):

| env | KiCad | DSN content (excl. path+host_version) | ORP content (excl. timestamp) | `host_version` |
|-----|-------|---------------------------------------|-------------------------------|----------------|
| original 9.0.8 env | 9.0.8 | **byte-identical** | **byte-identical** | matches (9.0.8) |
| **pcbnew-repro:9.0** (fresh) | 9.0.9 | **byte-identical** | **byte-identical** | 9.0.9 (differs) |

Concretely, the *only* lines that ever differ are:

```
# DSN (9.0.9 fresh vs 9.0.8 reference)
1c1   (pcb "<your output path>")      <-- SYNTH1L_DSN_ROOT argument
6c6   (host_version "9.0.9-…")        <-- 9.0.8 in the original env

# ORP (uncompressed)
7c7   "export_timestamp": "2026-05-29T…"   <-- generation time
```

**Conclusion.** Everything `pcbnew` actually computes — geometry, padstacks,
placement, netclasses, rules, and all ORP net/pad/class data — is
**byte-identical** between the original 9.0.8 and a fresh 9.0.9 install. The
residual differences are purely environment metadata (output path, KiCad version
string, timestamp). The original 9.0.8 build additionally
reproduces the `host_version` line; a fresh PPA build reproduces all content but
its `host_version`/timestamp naturally reflect the new run.

Compare commands:

```bash
# DSN: neutralize the path-arg (line 1) and the host_version string, then diff
norm(){ sed -e '1s#(pcb .*#(pcb X#' -e 's#host_version "[^"]*"#host_version "V"#' "$1"; }
diff <(norm gen.dsn) <(norm ref.dsn)              # empty => content identical

# ORP is gzip: compare UNCOMPRESSED (gzip headers embed their own mtime/name).
# The only line that differs is "export_timestamp".
diff <(zcat gen.orp) <(zcat ref.orp)
```
