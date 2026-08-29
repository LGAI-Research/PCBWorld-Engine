/*
 * RL incremental-DRC clearance scope.
 *
 * This is an RL-specific add-on (tag: _rl_), deliberately kept OUT of the stock KiCad
 * DRC engine so the upstream DRC files stay pristine and the original DRC can be kept
 * and compared. It holds the "scope" for an incremental copper-clearance pass: when set,
 * DRC_TEST_PROVIDER_RL_COPPER_CLEARANCE restricts its outer track loop to these items
 * (each still tested against the full RTree, so any pair involving a scoped item is
 * identical to a full pass) and skips the pad/graphic/zone sub-tests (retained by the
 * caller). This is the "recompute only the changed items" half of an incremental DRC.
 *
 * Lifetime: PNS_RL_ROUTER::runDRCEngine() sets the scope immediately before
 * engine->RunTests() and clears it (nullptr) immediately after, so the pointed-to vector
 * always outlives the run. The provider only reads it on the thread that drives
 * RunTests() (the outer track loop is built before the work is dispatched to the pool),
 * so a thread_local keeps concurrent engines on different threads independent.
 */
#ifndef DRC_RL_SCOPE_H
#define DRC_RL_SCOPE_H

#include <vector>

class BOARD_ITEM;

namespace DRC_RL
{
    /// Set the clearance scope for the next DRC run (nullptr = full board).
    void SetClearanceScope( const std::vector<BOARD_ITEM*>* aItems );

    /// Current clearance scope (nullptr = no scope, test all tracks).
    const std::vector<BOARD_ITEM*>* GetClearanceScope();
}

#endif // DRC_RL_SCOPE_H
