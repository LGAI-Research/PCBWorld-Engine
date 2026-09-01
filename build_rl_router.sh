#!/usr/bin/env bash
set -euo pipefail

BUNDLE_DIR="$(cd "$(dirname "$0")" && pwd)"        # the engine's root (this directory)
# Where the build lands. Embedded in the PCBWorld environment — the engine sits at
# <env>/engine as a subdirectory or a submodule — the output goes next to the
# environment, where its loader (pcb_world/engine, build_rl/) looks for it; a
# standalone engine checkout builds inside itself. BUILD_DIR / VENV_DIR override.
if [ -f "$BUNDLE_DIR/../pcb_world/engine/router_client.py" ]; then
    HOST_ROOT="$(cd "$BUNDLE_DIR/.." && pwd)"
else
    HOST_ROOT="$BUNDLE_DIR"
fi
BUILD_DIR="${BUILD_DIR:-${HOST_ROOT}/build_rl}"
KICAD_SRC_ORIG="${BUNDLE_DIR}/kicad-python/kicad"
KICAD_SRC="${BUILD_DIR}/kicad_src"
PATCHES_DIR="${BUNDLE_DIR}/kicad-patches"
VENV_DIR="${VENV_DIR:-${HOST_ROOT}/.venv}"

# ──────────────────────────────────────────────
# Build options (overridable via environment variables)
# Default: minimal build (kicad_rl_router only)
# ──────────────────────────────────────────────
BUILD_PCBNEW="${BUILD_PCBNEW:-0}"         # 1=include (kiface + python module), 0=exclude (default: exclude)
BUILD_CLI="${BUILD_CLI:-0}"               # 1=include kicad-cli (headless DRC for the data-prep chain, SVG/PDF rendering), 0=exclude (default: exclude)
USE_CONDA=0

# Parse arguments
for arg in "$@"; do
    case "$arg" in
        --conda) USE_CONDA=1 ;;
    esac
done

echo "=== Build Options ==="
echo "  BUILD_PCBNEW:    $BUILD_PCBNEW"
echo "  BUILD_CLI:       $BUILD_CLI"
echo "  USE_CONDA:       $USE_CONDA"

# ──────────────────────────────────────────────
# Dependency check
# ──────────────────────────────────────────────
echo "=== Dependency Check ==="

# cmake (prefer Homebrew path)
CMAKE="${CMAKE:-}"
if [ -z "$CMAKE" ]; then
    if command -v cmake &>/dev/null; then
        CMAKE="cmake"
    elif [ -x /opt/homebrew/opt/cmake/bin/cmake ]; then
        CMAKE="/opt/homebrew/opt/cmake/bin/cmake"
    else
        echo "ERROR: cmake not found"; exit 1
    fi
fi
echo "  cmake: $CMAKE ($($CMAKE --version | head -1))"

# ninja
if ! command -v ninja &>/dev/null; then
    echo "ERROR: ninja not found"; exit 1
fi
echo "  ninja: $(ninja --version)"

# Python (.venv preferred)
if [ -x "${VENV_DIR}/bin/python3" ]; then
    PYTHON="${VENV_DIR}/bin/python3"
else
    PYTHON="$(command -v python3)"
fi
echo "  python: $PYTHON ($($PYTHON --version))"

# Locate Python include/lib paths
PYTHON_INCLUDE=$($PYTHON -c "import sysconfig; print(sysconfig.get_path('include'))")
PYTHON_LIB_DIR=$($PYTHON -c "import sysconfig; print(sysconfig.get_config_var('LIBDIR'))")
PYTHON_LDLIBRARY=$($PYTHON -c "import sysconfig; print(sysconfig.get_config_var('LDLIBRARY'))")
PYTHON_LIBRARY="${PYTHON_LIB_DIR}/${PYTHON_LDLIBRARY}"

if [ ! -f "$PYTHON_LIBRARY" ]; then
    # Fallback: search for a Linux .so or macOS .dylib (a conda env's
    # LDLIBRARY may point to libpython*.a, so the actual shared library is
    # located separately)
    PYTHON_LIBRARY=$(find "$PYTHON_LIB_DIR" -maxdepth 1 -name "libpython3*.so*" -not -name "*.a" 2>/dev/null | head -1)
    if [ -z "$PYTHON_LIBRARY" ]; then
        PYTHON_LIBRARY=$(find "$PYTHON_LIB_DIR" -maxdepth 1 -name "libpython3*.dylib" 2>/dev/null | head -1)
    fi
fi
echo "  python include: $PYTHON_INCLUDE"
echo "  python library: $PYTHON_LIBRARY"

# Check for pybind11
if ! $PYTHON -c "import pybind11" 2>/dev/null; then
    echo "ERROR: pybind11 not installed in $PYTHON"
    echo "  Install: $PYTHON -m pip install pybind11"
    exit 1
fi
echo "  pybind11: $($PYTHON -c 'import pybind11; print(pybind11.__version__)')"

# wxWidgets
if ! command -v wx-config &>/dev/null; then
    echo "ERROR: wxWidgets not found (wx-config)"; exit 1
fi
echo "  wxWidgets: $(wx-config --version)"

# SWIG (for pcbnew Python bindings — needed only when BUILD_PCBNEW=1)
if [ "$BUILD_PCBNEW" = "1" ]; then
    if ! command -v swig &>/dev/null; then
        echo "ERROR: swig not found (required for pcbnew Python bindings)"
        echo "  Install: brew install swig"
        exit 1
    fi
    SWIG_VER=$(swig -version 2>&1 | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1)
    echo "  swig: $SWIG_VER"
fi

# Check the kicad-python submodule
if [ ! -f "$KICAD_SRC_ORIG/CMakeLists.txt" ]; then
    echo "ERROR: kicad-python submodule not initialized"
    echo "  Run: git submodule update --init --recursive"
    exit 1
fi

# ──────────────────────────────────────────────
# Step 1: rsync source -> build directory (kicad-python/ is left unmodified)
# ──────────────────────────────────────────────
echo ""
echo "=== Prepare patched source ==="
mkdir -p "$BUILD_DIR"

if [ ! -d "$KICAD_SRC" ]; then
    echo "  First build: copying kicad source to build dir..."
    rsync -a "$KICAD_SRC_ORIG/" "$KICAD_SRC/"
else
    echo "  Incremental sync..."
    rsync -a --delete "$KICAD_SRC_ORIG/" "$KICAD_SRC/"
fi

# Step 2: Apply patches (only to the copy in the build directory). `cp -p` keeps the
# patch files' mtimes: a plain cp re-stamps ~40 sources (kiid.h among them, included
# nearly everywhere) on every run and makes ninja redo ~700 steps for nothing — with
# unchanged patches a re-run of this script is a no-op build.
# Caveat (rsync -a shares it for upstream files): mtimes are restored, so if you
# hand-edited files inside build_rl/kicad_src and built, a re-run restores content
# with OLD stamps and ninja may keep the stale objects — hand-edits there are out of
# contract; after one, delete build_rl/kicad_src (this script recreates it).
echo "  Applying patches..."

# Modified kicad files (overwritten in place, same path structure as upstream)
cp -p "$PATCHES_DIR/kicad/CMakeLists.txt"                       "$KICAD_SRC/CMakeLists.txt"
cp -p "$PATCHES_DIR/kicad/cmake/FindOCC.cmake"                  "$KICAD_SRC/cmake/FindOCC.cmake"
cp -p "$PATCHES_DIR/kicad/common/build_version.cpp"             "$KICAD_SRC/common/build_version.cpp"
cp -p "$PATCHES_DIR/kicad/common/properties/pg_properties.cpp"  "$KICAD_SRC/common/properties/pg_properties.cpp"
cp -p "$PATCHES_DIR/kicad/include/properties/pg_properties.h"   "$KICAD_SRC/include/properties/pg_properties.h"
# kiid: expose KIID::Get/SetGeneratorState so the RL checkpoint can snapshot and the
# restore can rewind the process-global UUID generator — makes routing replayed after
# a checkpoint restore mint the same UUID stream a fresh run does.
cp -p "$PATCHES_DIR/kicad/include/kiid.h"                       "$KICAD_SRC/include/kiid.h"
cp -p "$PATCHES_DIR/kicad/common/kiid.cpp"                      "$KICAD_SRC/common/kiid.cpp"
cp -p "$PATCHES_DIR/kicad/kicad/CMakeLists.txt"                 "$KICAD_SRC/kicad/CMakeLists.txt"
cp -p "$PATCHES_DIR/kicad/eeschema/CMakeLists.txt"              "$KICAD_SRC/eeschema/CMakeLists.txt"
cp -p "$PATCHES_DIR/kicad/pcbnew/CMakeLists.txt"                "$KICAD_SRC/pcbnew/CMakeLists.txt"
cp -p "$PATCHES_DIR/kicad/pcbnew/router/pns_kicad_iface.cpp"    "$KICAD_SRC/pcbnew/router/pns_kicad_iface.cpp"
cp -p "$PATCHES_DIR/kicad/pcbnew/router/pns_kicad_iface.h"      "$KICAD_SRC/pcbnew/router/pns_kicad_iface.h"
# MITERED_90 consistency fix (3 hunks, see kicad-patches/kicad/pcbnew/router/):
#   line_placer: forward cornerMode through reduceTail, OBTUSE→forbidden in mergeHead
#   shove:       hull→AABB substitution in shoveLineToHullSet (mirror of walkaround:169)
#   optimizer:   CornerCost penalises ANG_OBTUSE (45° miter) prohibitively in 90° mode
cp -p "$PATCHES_DIR/kicad/pcbnew/router/pns_line_placer.cpp"    "$KICAD_SRC/pcbnew/router/pns_line_placer.cpp"
# pns_walkaround: removes processCluster's wall-clock timeout, so routing
# stays deterministic regardless of run-to-run variance in how many
# clusters get processed.
cp -p "$PATCHES_DIR/kicad/pcbnew/router/pns_walkaround.cpp"     "$KICAD_SRC/pcbnew/router/pns_walkaround.cpp"
# pns_routing_settings.h: exposes a shove iter-limit setter and a
# followBranch iter-limit field, so the iteration bound left by removing
# the wall clock can be configured from engine init.
cp -p "$PATCHES_DIR/kicad/pcbnew/router/pns_routing_settings.h" "$KICAD_SRC/pcbnew/router/pns_routing_settings.h"
cp -p "$PATCHES_DIR/kicad/pcbnew/router/pns_shove.cpp"          "$KICAD_SRC/pcbnew/router/pns_shove.cpp"
cp -p "$PATCHES_DIR/kicad/pcbnew/router/pns_optimizer.cpp"      "$KICAD_SRC/pcbnew/router/pns_optimizer.cpp"
# pns_node: determinism fix (UUID-ordered obstacle set, reproducible routes across
# router instances) + NODE::ReleaseGarbage() public wrapper used by the RL incremental
# restore (restoreIncremental drains m_garbageItems without a full ClearWorld+SyncWorld).
cp -p "$PATCHES_DIR/kicad/pcbnew/router/pns_node.h"             "$KICAD_SRC/pcbnew/router/pns_node.h"
cp -p "$PATCHES_DIR/kicad/pcbnew/router/pns_node.cpp"           "$KICAD_SRC/pcbnew/router/pns_node.cpp"
# pns_item: ITEM::Serial() — an allocation-order sequence number (a
# process-global monotonic counter, including clones). Every remaining
# address-based fallback (geometric exact-tie temporary items, OBSTACLE head
# comparison, nodeItemOrder ties) is replaced with this sequence number,
# closing the last path by which ASLR/allocator history could leak into
# routing results.
cp -p "$PATCHES_DIR/kicad/pcbnew/router/pns_item.h"             "$KICAD_SRC/pcbnew/router/pns_item.h"
# pns_item.cpp: captures the head's Serial() by value on OBSTACLE insertion
# (m_headSerial) — NearestObstacle accumulates loop-local temporary heads
# into one set, so by comparison time the head object may already be dead.
cp -p "$PATCHES_DIR/kicad/pcbnew/router/pns_item.cpp"           "$KICAD_SRC/pcbnew/router/pns_item.cpp"
# pns_router: ROUTER::Finish() takes aMaxAttempts (was hardcoded triesLeft=5) so the
# RL finish() can bound the internal Move-to-convergence loop from Python.
cp -p "$PATCHES_DIR/kicad/pcbnew/router/pns_router.h"           "$KICAD_SRC/pcbnew/router/pns_router.h"
cp -p "$PATCHES_DIR/kicad/pcbnew/router/pns_router.cpp"         "$KICAD_SRC/pcbnew/router/pns_router.cpp"
# pns_mouse_trail_tracer: if the ROUTER::GetInstance() singleton is null (a
# destruction-order violation after two routers coexisted), prints a
# diagnostic message and aborts (loud-fail) instead of segfaulting randomly.
# Paired with the destructor guard in pns_router.cpp (if theRouter==this).
cp -p "$PATCHES_DIR/kicad/pcbnew/router/pns_mouse_trail_tracer.cpp" "$KICAD_SRC/pcbnew/router/pns_mouse_trail_tracer.cpp"
# pns_topology: sorts CLUSTER::m_items by UUID (std::set<ITEM*,ItemCmp>) so
# the processCluster accumulation order in walkaround/shove is
# deterministic, removing ASLR-based rollout nondeterminism.
# .cpp: replaces followBranch's wall-clock timeout (m_FollowBranchTimeout)
# with a deterministic pop counter, so a time cutoff no longer interrupts
# DFS at a different point on every run.
cp -p "$PATCHES_DIR/kicad/pcbnew/router/pns_topology.h"         "$KICAD_SRC/pcbnew/router/pns_topology.h"
cp -p "$PATCHES_DIR/kicad/pcbnew/router/pns_topology.cpp"       "$KICAD_SRC/pcbnew/router/pns_topology.cpp"
# ratsnest_data: removes Triangulate()'s same-coordinate-anchor special
# branch (anchors.size()==1), which could swallow an edge via the Parent
# LayerSet overlap heuristic instead of cluster identity — the branch is
# emptied and falls through to the anchorChains cluster-chain loop below.
cp -p "$PATCHES_DIR/kicad/pcbnew/ratsnest/ratsnest_data.cpp"    "$KICAD_SRC/pcbnew/ratsnest/ratsnest_data.cpp"
# drc_engine.cpp: lift the per-type DRC report caps (ERROR_LIMIT 199 /
#     EXTENDED_ERROR_LIMIT 499 -> INT_MAX). Upstream caps the marker list for
#     GUI responsiveness; headless callers (reward, eval metrics, incremental
#     DRC baseline) need every violation, and the cap also misfiles shorts as
#     clearance violations once the shorting budget is spent.
cp -p "$PATCHES_DIR/kicad/pcbnew/drc/drc_engine.cpp"           "$KICAD_SRC/pcbnew/drc/drc_engine.cpp"
# Two related layer fixes:
#   pns_joint.h: JOINT::operator== now includes the layer range (removes a
#     latent stacked-joint confusion trap).
#   tracks_cleaner: merge-collinear candidates now require a same-layer
#     guard — an upstream bug could silently merge-delete segments on a
#     different layer across a via.
cp -p "$PATCHES_DIR/kicad/pcbnew/router/pns_joint.h"            "$KICAD_SRC/pcbnew/router/pns_joint.h"
cp -p "$PATCHES_DIR/kicad/pcbnew/tracks_cleaner.cpp"            "$KICAD_SRC/pcbnew/tracks_cleaner.cpp"
mkdir -p "$KICAD_SRC/kicad/cli"
cp -p "$PATCHES_DIR/kicad/kicad/cli/command_pcb_render.cpp"     "$KICAD_SRC/kicad/cli/command_pcb_render.cpp"

# New RL files
cp -p "$PATCHES_DIR/rl/pns_rl_router.cpp"    "$KICAD_SRC/pcbnew/router/"
cp -p "$PATCHES_DIR/rl/pns_rl_router.h"      "$KICAD_SRC/pcbnew/router/"
cp -p "$PATCHES_DIR/rl/pns_rl_iface.cpp"     "$KICAD_SRC/pcbnew/router/"
cp -p "$PATCHES_DIR/rl/pns_rl_iface.h"       "$KICAD_SRC/pcbnew/router/"
# RL DRC (tag: _rl_): the stock DRC files are left PRISTINE. Our copper-clearance fork
# (event-driven wait + incremental clearance scope) is dropped in alongside them and
# compiled INSTEAD of the original by kicad-patches/rl/CMakeLists.txt's DRC_SRCS. The
# scope plumbing lives in a thread_local (drc_rl_scope.h), so no core DRC file is touched.
cp -p "$PATCHES_DIR/rl/drc_test_provider_rl_copper_clearance.cpp" "$KICAD_SRC/pcbnew/drc/"
cp -p "$PATCHES_DIR/rl/drc_rl_scope.cpp"     "$KICAD_SRC/pcbnew/drc/"
cp -p "$PATCHES_DIR/rl/drc_rl_scope.h"       "$KICAD_SRC/pcbnew/drc/"
mkdir -p "$KICAD_SRC/pcbnew/python/rl"
cp -p "$PATCHES_DIR/rl/pns_rl_bindings.cpp"  "$KICAD_SRC/pcbnew/python/rl/"
cp -p "$PATCHES_DIR/rl/stackup_stub.cpp"     "$KICAD_SRC/pcbnew/python/rl/"
cp -p "$PATCHES_DIR/rl/CMakeLists.txt"       "$KICAD_SRC/pcbnew/python/rl/"
# RL fork of tracks_cleaner.cpp (tag: _rl), dropped in beside the stock file — which
# stays pristine for the KiCad app build. Compiled into kicad_rl_router.so only, by
# kicad-patches/rl/CMakeLists.txt's PNS_RL_SRCS.
cp -p "$PATCHES_DIR/rl/tracks_cleaner_rl.cpp" "$KICAD_SRC/pcbnew/"
cp -p "$PATCHES_DIR/rl/tracks_cleaner_rl.h"   "$KICAD_SRC/pcbnew/"

echo "  Done (24 files patched)"

# ──────────────────────────────────────────────
# Step 3: CMake Configure
# ──────────────────────────────────────────────
echo ""
echo "=== CMake Configure ==="
cd "$BUILD_DIR"

CMAKE_ARGS=(
    "$KICAD_SRC"
    -G Ninja
    -DCMAKE_BUILD_TYPE=Release
    -DKICAD_SCRIPTING_WXPYTHON=OFF
    -DKICAD_BUILD_I18N=OFF
    -DKICAD_BUILD_QA_TESTS=OFF
    "-DPYTHON_EXECUTABLE=$PYTHON"
    "-DPYTHON_INCLUDE_DIR=$PYTHON_INCLUDE"
    "-DPYTHON_LIBRARY=$PYTHON_LIBRARY"
    -DCMAKE_IGNORE_PREFIX_PATH=/opt/anaconda3
    -Wno-dev
)
if [ "$BUILD_PCBNEW" = "1" ]; then
    CMAKE_ARGS+=("-DSWIG_EXECUTABLE=$(command -v swig)")
fi

# Prefer the conda environment
if [ "$USE_CONDA" = "1" ]; then
    if [ -z "${CONDA_PREFIX:-}" ]; then
        echo "ERROR: --conda requires an active conda environment (conda activate <env>)"
        exit 1
    fi
    CMAKE_ARGS+=("-DCMAKE_PREFIX_PATH=$CONDA_PREFIX")
    echo "  conda prefix: $CONDA_PREFIX"
fi

# protobuf/abseil/utf8_range: since the conda env's headers are included
# first, using conda's cmake config keeps protoc/headers/lib consistent.
# If a conda env is active and has a cmake config, try conda first;
# otherwise fall back to brew.
if [ -n "${CONDA_PREFIX:-}" ] && [ -d "$CONDA_PREFIX/lib/cmake/absl" ]; then
    CMAKE_ARGS+=("-Dabsl_DIR=$CONDA_PREFIX/lib/cmake/absl")
elif [ -d /opt/homebrew/lib/cmake/absl ]; then
    CMAKE_ARGS+=("-Dabsl_DIR=/opt/homebrew/lib/cmake/absl")
fi
if [ -n "${CONDA_PREFIX:-}" ] && [ -d "$CONDA_PREFIX/lib/cmake/protobuf" ]; then
    CMAKE_ARGS+=("-DProtobuf_DIR=$CONDA_PREFIX/lib/cmake/protobuf")
elif [ -d /opt/homebrew/lib/cmake/protobuf ]; then
    CMAKE_ARGS+=("-DProtobuf_DIR=/opt/homebrew/lib/cmake/protobuf")
fi
if [ -n "${CONDA_PREFIX:-}" ] && [ -d "$CONDA_PREFIX/lib/cmake/utf8_range" ]; then
    CMAKE_ARGS+=("-Dutf8_range_DIR=$CONDA_PREFIX/lib/cmake/utf8_range")
elif [ -d /opt/homebrew/lib/cmake/utf8_range ]; then
    CMAKE_ARGS+=("-Dutf8_range_DIR=/opt/homebrew/lib/cmake/utf8_range")
fi
# OpenCASCADE: optional for the engine (the RL router never touches it) but required by
# pcbnew_kiface (STEP exporter), i.e. by BUILD_PCBNEW=1. FindOCC.cmake searches fixed
# system prefixes only, so a Homebrew or conda OCC (environment.yml ships `occt`) has to
# be pointed at explicitly. Passed whenever present so the configuration does not flip
# between engine-only and BUILD_PCBNEW=1 runs of the same build tree.
if [ -f /opt/homebrew/include/opencascade/Standard_Handle.hxx ]; then
    CMAKE_ARGS+=("-DOCC_INCLUDE_DIR=/opt/homebrew/include/opencascade")
    CMAKE_ARGS+=("-DOCC_LIBRARY_DIR=/opt/homebrew/lib")
elif [ -n "${CONDA_PREFIX:-}" ] && [ -f "$CONDA_PREFIX/include/opencascade/Standard_Handle.hxx" ]; then
    CMAKE_ARGS+=("-DOCC_INCLUDE_DIR=$CONDA_PREFIX/include/opencascade")
    CMAKE_ARGS+=("-DOCC_LIBRARY_DIR=$CONDA_PREFIX/lib")
fi

$CMAKE "${CMAKE_ARGS[@]}"

# ──────────────────────────────────────────────
# Step 4: Build
# ──────────────────────────────────────────────
echo ""
# kicad_rl_router is the only target the core RL/DRC pipeline needs.
# kicad-cli is optional: only envs/rendering (SVG/PDF/3D) shells out to it.
# Gate with BUILD_CLI=1.
NINJA_TARGETS="kicad_rl_router"
[ "$BUILD_CLI"       = "1" ] && NINJA_TARGETS="$NINJA_TARGETS kicad-cli"
[ "$BUILD_PCBNEW"    = "1" ] && NINJA_TARGETS="$NINJA_TARGETS pcbnew_kiface pcbnew_python_module"
echo "=== Build targets: $NINJA_TARGETS ==="
ninja $NINJA_TARGETS

# ──────────────────────────────────────────────
# Step 4b: Place kicad-cli into the bundle so rpath (@executable_path/../
# Frameworks) resolves to the already-built KiCad.app/Contents/Frameworks.
# Only runs when BUILD_CLI=1; the -x check also gracefully no-ops on an
# already-installed CLI-free build tree.
# ──────────────────────────────────────────────
CLI_BUILT="${BUILD_DIR}/kicad/kicad-cli"
CLI_DEST_DIR="${BUILD_DIR}/kicad/KiCad.app/Contents/MacOS"
if [ "$BUILD_CLI" = "1" ] && [ -x "$CLI_BUILT" ]; then
    mkdir -p "$CLI_DEST_DIR"
    cp -f "$CLI_BUILT" "$CLI_DEST_DIR/kicad-cli"
fi

# ──────────────────────────────────────────────
# Step 5: Print results
# ──────────────────────────────────────────────
echo ""
echo "=== Build Complete ==="
MODULE_PATH=$(find "$BUILD_DIR/pcbnew/python/rl" -name "kicad_rl_router*.so" 2>/dev/null | head -1)
FRAMEWORKS_DIR="${BUILD_DIR}/kicad/KiCad.app/Contents/Frameworks"

[ -n "$MODULE_PATH" ] && echo "RL module: $MODULE_PATH"
[ -x "$CLI_BUILT" ] && echo "kicad-cli: $CLI_BUILT"

if [ "$BUILD_PCBNEW" = "1" ]; then
    # Linux: pcbnew.py + _pcbnew.so (-> _pcbnew.kiface) sit in build_rl/pcbnew/ — that
    # directory on PYTHONPATH is the whole install. macOS: inside the app bundle.
    PCBNEW_SO=$(find "$BUILD_DIR/pcbnew" -maxdepth 1 -name "_pcbnew.so" 2>/dev/null | head -1)
    PCBNEW_PY=$(find "$BUILD_DIR/pcbnew" -maxdepth 1 -name "pcbnew.py" 2>/dev/null | head -1)
    [ -n "$PCBNEW_PY" ] || PCBNEW_PY=$(find "$BUILD_DIR/kicad" -name "pcbnew.py" 2>/dev/null | head -1)
    PCBNEW_SITE=$(dirname "$PCBNEW_PY" 2>/dev/null)
    [ -n "$PCBNEW_SO"   ] && echo "pcbnew so: $PCBNEW_SO"
    [ -n "$PCBNEW_SITE" ] && echo "pcbnew py: $PCBNEW_SITE"
fi

if [ -z "$MODULE_PATH" ]; then
    echo "WARNING: kicad_rl_router not found"
    exit 1
fi

# Stamp the build with the engine (patch) version so a stale .so can be detected: copy
# the source engine-version marker next to the freshly-built module. It is a provenance
# marker ONLY — no runtime code reads it. tests/test_engine_api/test_engine_build_version.py
# compares this stamp against kicad-patches/ENGINE_VERSION and fails (asking for a
# rebuild) when they diverge, which happens after a minor bump lands without a re-run here.
if [ -f "$PATCHES_DIR/ENGINE_VERSION" ]; then
    cp "$PATCHES_DIR/ENGINE_VERSION" "$(dirname "$MODULE_PATH")/ENGINE_VERSION"
    echo "engine version: $(tr -d '[:space:]' < "$PATCHES_DIR/ENGINE_VERSION") (stamped next to module)"
fi

echo ""
echo "Usage:"
echo "  export DYLD_LIBRARY_PATH='${FRAMEWORKS_DIR}'"
PYTHONPATH_HINT="${BUILD_DIR}/pcbnew/python/rl"
if [ "$BUILD_PCBNEW" = "1" ] && [ -n "${PCBNEW_SITE:-}" ]; then
    PYTHONPATH_HINT="${PYTHONPATH_HINT}:${PCBNEW_SITE}"
fi
echo "  export PYTHONPATH='${PYTHONPATH_HINT}'"
echo "  python3 -c 'import kicad_rl_router; print(\"OK\")'"
if [ "$BUILD_PCBNEW" = "1" ]; then
    echo "  python3 -c 'import pcbnew; print(pcbnew.GetBuildVersion())'"
fi
if [ "$BUILD_CLI" = "1" ]; then
    # A kicad-cli run from the build tree finds _pcbnew.kiface (one directory up, in
    # pcbnew/) only with this variable set; `pcb drc` and every other pcb command need it.
    echo "  KICAD_RUN_FROM_BUILD_DIR=1 '${CLI_BUILT}' --version"
fi
echo ""
echo "Test:"
echo "  DYLD_LIBRARY_PATH='${FRAMEWORKS_DIR}' pytest tests/ -v"
