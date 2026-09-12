/*
 * KiCad PNS RL Router Interface
 * Headless ROUTER_IFACE for reinforcement-learning usage.
 *
 * Extends PNS_KICAD_IFACE_BASE (which already provides board syncing and
 * no-op display methods) and overrides only the three methods needed to
 * apply routing changes back to the BOARD without a GUI tool/commit stack.
 */

#ifndef PNS_RL_IFACE_H
#define PNS_RL_IFACE_H

#include "pns_kicad_iface.h"

#include <vector>

class BOARD_ITEM;
class BOARD_CONNECTED_ITEM;

namespace PNS { class NODE; }

/**
 * Headless PNS router interface for RL environments.
 *
 * Key differences from PNS_KICAD_IFACE:
 *   - No PCB_TOOL_BASE / BOARD_COMMIT required.
 *   - AddItem / RemoveItem / UpdateItem accumulate pending changes.
 *   - Commit() applies those changes directly to the BOARD and rebuilds
 *     connectivity (no undo stack).
 *   - All display / view methods are inherited as no-ops from
 *     PNS_KICAD_IFACE_BASE.
 */
class PNS_RL_IFACE : public PNS_KICAD_IFACE_BASE
{
public:
    PNS_RL_IFACE();
    ~PNS_RL_IFACE() override;

    // -----------------------------------------------------------------------
    // Net information (same logic as PNS_KICAD_IFACE)
    // -----------------------------------------------------------------------
    int      GetNetCode( PNS::NET_HANDLE aNet ) const override;
    wxString GetNetName( PNS::NET_HANDLE aNet ) const override;

    // -----------------------------------------------------------------------
    // Board-change callbacks called by ROUTER::CommitRouting()
    // -----------------------------------------------------------------------
    void RemoveItem( PNS::ITEM* aItem ) override;
    void AddItem( PNS::ITEM* aItem ) override;
    void UpdateItem( PNS::ITEM* aItem ) override;

    /**
     * Apply all pending Add / Remove / Update changes to the BOARD and
     * rebuild connectivity.  Called automatically by ROUTER::CommitRouting().
     */
    void Commit() override;

    /**
     * Create the PNS item for a single board track/via/arc and add it directly
     * to @a aWorld. Used by the incremental checkpoint restore to update only
     * the changed tracks in the PNS world (instead of a full SyncWorld that
     * re-creates every invariant pad/footprint obstacle). The created PNS item
     * has its Parent set to @a aItem, so NODE::FindItemByParent locates it on a
     * later restore.
     */
    void addBoardItemToWorld( PNS::NODE* aWorld, BOARD_ITEM* aItem );

private:
    /// Create a new BOARD_CONNECTED_ITEM from a PNS item (SEGMENT / VIA / ARC).
    BOARD_CONNECTED_ITEM* createBoardItem( PNS::ITEM* aItem );

    /// Modify an existing BOARD_CONNECTED_ITEM in-place from PNS data.
    void modifyBoardItem( PNS::ITEM* aItem );

    std::vector<BOARD_ITEM*>          m_pendingRemove;
    std::vector<BOARD_CONNECTED_ITEM*> m_pendingAdd;
};

#endif // PNS_RL_IFACE_H
