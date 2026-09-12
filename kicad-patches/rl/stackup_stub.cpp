/*
 * stackup_stub.cpp
 *
 * Stub implementations for headless build.
 * These functions are only called by GUI/plugin code that we don't use
 * in the RL environment (we only use .kicad_pcb format and headless routing).
 */

#include <board_stackup_manager/stackup_predefined_prms.h>
#include <fp_lib_table.h>
#include <kiface_base.h>
#include <vector>

// Stub for GetStandardColors - used by odbpp/ipc2581 IO plugins
static std::vector<FAB_LAYER_COLOR> dummy_colors;

const std::vector<FAB_LAYER_COLOR>& GetStandardColors( BOARD_STACKUP_ITEM_TYPE aType )
{
    // Return empty vector - this is fine because we never use odbpp/ipc2581 formats
    return dummy_colors;
}

// Stub for GFootprintTable - global footprint library table (not needed for headless routing)
FP_LIB_TABLE GFootprintTable;

// Stub for Kiface - KiCad plugin interface (not needed for headless RL router)
class KIFACE_STUB : public KIFACE_BASE
{
public:
    KIFACE_STUB() : KIFACE_BASE( "kicad_rl_router", KIWAY::KIWAY_FACE_COUNT ) {}

    bool OnKifaceStart( PGM_BASE* aProgram, int aCtlBits, KIWAY* aKiway ) override { return true; }
    void OnKifaceEnd() override {}
    wxWindow* CreateKiWindow( wxWindow* aParent, int aClassId, KIWAY* aKiway, int aCtlBits = 0 ) override { return nullptr; }
    void* IfaceOrAddress( int aDataId ) override { return nullptr; }
};

static KIFACE_STUB kiface_stub;

KIFACE_BASE& Kiface()
{
    return kiface_stub;
}
