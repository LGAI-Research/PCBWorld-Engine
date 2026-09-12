# CMakeLists.txt for the kicad_rl_router Python extension module
#
# This module wraps PNS_RL_ROUTER (pcbnew/router/pns_rl_router.h) via
# pybind11 so that Python RL agents can call the full PNS router engine.

include_directories( BEFORE ${INC_BEFORE} )

include_directories(
    ${CMAKE_SOURCE_DIR}/pcbnew
    ${CMAKE_SOURCE_DIR}/pcbnew/router
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/common
    ${CMAKE_SOURCE_DIR}/libs/kimath/include
    ${CMAKE_SOURCE_DIR}/libs/core/include
    ${CMAKE_SOURCE_DIR}/thirdparty/clipper2/Clipper2Lib/include
    ${CMAKE_SOURCE_DIR}/thirdparty/dynamic_bitset
    ${INC_AFTER}
    ${PYBIND11_INCLUDE_DIR}
    ${PYTHON_INCLUDE_DIRS}
)

# RL-specific wrapper sources.
# pns_kicad_iface.cpp is compiled here WITH PNS_HEADLESS_BUILD so only
# PNS_KICAD_IFACE_BASE (no GUI PCB_TOOL_BASE dependency) is compiled in.
set( PNS_RL_SRCS
    ${CMAKE_SOURCE_DIR}/pcbnew/router/pns_kicad_iface.cpp
    ${CMAKE_SOURCE_DIR}/pcbnew/router/pns_rl_iface.cpp
    ${CMAKE_SOURCE_DIR}/pcbnew/router/pns_rl_router.cpp
    # RL incremental-DRC clearance scope (thread_local). Always compiled, independent of
    # which clearance provider is in DRC_SRCS, so the stock provider can be swapped back
    # for comparison without an undefined-symbol link error.
    ${CMAKE_SOURCE_DIR}/pcbnew/drc/drc_rl_scope.cpp
    # RL fork of pcbnew/tracks_cleaner.cpp. The stock class takes a BOARD_COMMIT&, and
    # board_commit.cpp drags pcb_tool_base / pcb_actions / zone_filler_tool / view /
    # teardrop into a headless link; the fork drops the commit (checkpoint/restore is
    # the undo mechanism here) and hardens the pointer-ordered passes. The stock file
    # is left untouched for the KiCad app build.
    ${CMAKE_SOURCE_DIR}/pcbnew/tracks_cleaner_rl.cpp
    pns_rl_bindings.cpp
    stackup_stub.cpp
)

# DRC test providers: each file registers itself via a static dummy object.
# Without these compiled into kicad_rl_router.so the registry stays empty
# and DRC_ENGINE::RunTests() finds nothing to run.
set( DRC_SRCS
    ${CMAKE_SOURCE_DIR}/pcbnew/drc/drc_cache_generator.cpp
    ${CMAKE_SOURCE_DIR}/pcbnew/drc/drc_creepage_utils.cpp
    ${CMAKE_SOURCE_DIR}/pcbnew/drc/drc_test_provider.cpp
    ${CMAKE_SOURCE_DIR}/pcbnew/drc/drc_test_provider_clearance_base.cpp
    # RL fork of the copper-clearance provider (event-driven wait + incremental scope).
    # Compiled INSTEAD of the stock drc_test_provider_copper_clearance.cpp, which is left
    # pristine for comparison — swap these two lines to build the original DRC instead.
    ${CMAKE_SOURCE_DIR}/pcbnew/drc/drc_test_provider_rl_copper_clearance.cpp
    # ${CMAKE_SOURCE_DIR}/pcbnew/drc/drc_test_provider_copper_clearance.cpp
    ${CMAKE_SOURCE_DIR}/pcbnew/drc/drc_test_provider_connectivity.cpp
    ${CMAKE_SOURCE_DIR}/pcbnew/drc/drc_test_provider_physical_clearance.cpp
    ${CMAKE_SOURCE_DIR}/pcbnew/drc/drc_test_provider_track_width.cpp
    ${CMAKE_SOURCE_DIR}/pcbnew/drc/drc_test_provider_via_diameter.cpp
    ${CMAKE_SOURCE_DIR}/pcbnew/drc/drc_test_provider_annular_width.cpp
    ${CMAKE_SOURCE_DIR}/pcbnew/drc/drc_test_provider_hole_size.cpp
    ${CMAKE_SOURCE_DIR}/pcbnew/drc/drc_test_provider_hole_to_hole.cpp
)

# pnsrouter.a also contains pns_kicad_iface.cpp.o (compiled without the headless
# guard).  Strip it out so the linker sees only our headless version.
set( PNSROUTER_HEADLESS "${CMAKE_CURRENT_BINARY_DIR}/libpnsrouter_headless.a" )

add_custom_command(
    OUTPUT  "${PNSROUTER_HEADLESS}"
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:pnsrouter>" "${PNSROUTER_HEADLESS}"
    COMMAND ar d "${PNSROUTER_HEADLESS}" pns_kicad_iface.cpp.o
    DEPENDS pnsrouter
    COMMENT "Creating headless pnsrouter (stripping GUI iface object)"
)

add_custom_target( pnsrouter_headless_target DEPENDS "${PNSROUTER_HEADLESS}" )

# Build a Python extension module (shared lib with .so / .pyd extension)
add_library( kicad_rl_router MODULE ${PNS_RL_SRCS} ${DRC_SRCS} )
add_dependencies( kicad_rl_router pnsrouter_headless_target )

# No "lib" prefix for Python extension modules
set_target_properties( kicad_rl_router PROPERTIES
    PREFIX ""
    CXX_STANDARD 20
)

# Propagate include directories from pcbcommon/kicommon/common targets
# (include-only — we do NOT link the static library code for those targets)
target_include_directories( kicad_rl_router PRIVATE
    $<TARGET_PROPERTY:pcbcommon,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:kicommon,INTERFACE_INCLUDE_DIRECTORIES>
    $<TARGET_PROPERTY:common,INTERFACE_INCLUDE_DIRECTORIES>
)

# Link against pnsrouter (headless) + all required KiCad static libs.
# KiCad uses -fvisibility=hidden so BOARD_ITEM typeinfo and other symbols
# are private in _pcbnew.so.  By statically linking the same libs here,
# kicad_rl_router.so gets its own copies of those private symbols.
# Only wxWidgets / GUI symbols (not pulled in here) are left for
# -undefined dynamic_lookup (resolved from _pcbnew.so's flat namespace).
target_link_libraries( kicad_rl_router
    PRIVATE
        # PNS router (headless version)
        "${PNSROUTER_HEADLESS}"

        # PCB-specific libraries
        pcbcommon
        connectivity
        3d-viewer

        # Framework libraries
        kicommon
        kiplatform
        common
        gal
        scripting

        # Utility libraries
        core
        kimath
        clipper2
        idf3
        markdown_lib

        # IO plugins (all of them)
        # Note: ipc2581 and odbpp require GetStandardColors, provided by stackup_stub.cpp
        pcad2kicadpcb
        altium2pcbnew
        cadstar2pcbnew
        easyeda
        easyedapro
        fabmaster
        ipc2581
        odbpp

        # System libraries
        ${PYTHON_LIBRARIES}
)

# On macOS, defer undefined symbol resolution to load-time
if( APPLE )
    target_link_options( kicad_rl_router PRIVATE
        -undefined dynamic_lookup
        -flat_namespace
    )
    set_target_properties( kicad_rl_router PROPERTIES
        BUILD_RPATH "/usr/lib;${CMAKE_BINARY_DIR}/kicad/KiCad.app/Contents/Frameworks"
    )
endif()

# pybind11 compile definitions
target_compile_definitions( kicad_rl_router PRIVATE
    PYBIND11_PYTHON_VERSION="${PYTHON_VERSION_STRING}"
    PNS_HEADLESS_BUILD
)

# Installation: put next to pcbnew.so / pcbnew.py so the user can
#   import kicad_rl_router
install( TARGETS kicad_rl_router
    DESTINATION ${PYTHON_DEST}
)
