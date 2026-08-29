/*
 * RL incremental-DRC clearance scope — definition.
 *
 * Kept in its own translation unit (not in the RL copper-clearance provider) so the
 * scope symbols exist regardless of which clearance provider is compiled. That lets the
 * stock provider be swapped back in (kicad-patches/rl/CMakeLists.txt DRC_SRCS) to build
 * and compare the original DRC without a link error from PNS_RL_ROUTER::runDRCEngine,
 * which always calls DRC_RL::SetClearanceScope(). See drc/drc_rl_scope.h.
 */
#include <drc/drc_rl_scope.h>

namespace DRC_RL
{
    // Thread-local: set/read only on the thread that drives DRC_ENGINE::RunTests()
    // (the outer track loop is built before work is dispatched to the pool), so
    // concurrent engines on different threads do not interfere.
    static thread_local const std::vector<BOARD_ITEM*>* g_clearanceScope = nullptr;

    void SetClearanceScope( const std::vector<BOARD_ITEM*>* aItems ) { g_clearanceScope = aItems; }
    const std::vector<BOARD_ITEM*>* GetClearanceScope() { return g_clearanceScope; }
}
