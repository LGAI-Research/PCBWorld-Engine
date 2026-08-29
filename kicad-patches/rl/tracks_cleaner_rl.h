/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you may find one here:
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 * or you may search the http://www.gnu.org website for the version 2 license,
 * or you may write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 */

#ifndef RL_TRACKS_CLEANER_H
#define RL_TRACKS_CLEANER_H

#include <map>
#include <unordered_set>
#include <vector>

#include <board.h>
#include <pcb_track.h>

/**
 * RL fork of pcbnew/tracks_cleaner.{h,cpp} (TRACKS_CLEANER).  The geometry passes are
 * ported verbatim; four things differ, each of them required to make the cleaner usable
 * from the headless RL router:
 *
 *  1. No BOARD_COMMIT.  The stock cleaner records every change in a commit for the GUI
 *     undo stack; linking board_commit.cpp into kicad_rl_router.so drags in
 *     pcb_tool_base / pcb_actions / zone_filler_tool / view / teardrop.  The RL router
 *     has no undo stack — checkpoint/restore is the undo mechanism — so removed items
 *     are DETACHED (BOARD::Remove, no free) and handed back to the caller, which frees
 *     them only after resyncWorld() + RecalculateRatsnest() have dropped every
 *     reference (same deferred-free invariant as deleteRoutingOfNets()).
 *
 *  2. No REPORTER / wxSafeYield: nothing to report to and no UI to time-slice.
 *
 *  3. Deterministic ordering.  The stock cleaner orders work by pointer address in
 *     three places, which makes the surviving item heap-layout dependent — the exact
 *     failure mode that CLUSTER::m_items had (fixed by UUID ordering in pns_topology.h).
 *     Every ordering decision here is keyed on KIID instead, and the collinear-merge
 *     collection is serial (the stock parallel_loop chunks by thread-pool size, so the
 *     merge fixpoint could depend on the pool size).
 *
 *  4. Honest dry runs.  The stock dry run still mutates: it normalises via ends
 *     (SetEnd(GetStart())) and leaves IS_DELETED flags on board items after the
 *     dangling / tracks-in-pads passes.  Here a dry run touches nothing, and both modes
 *     clear the scratch flags before returning.
 */

/// Cleanup operation codes.  Mirrors CLEANUP_RC_CODE without the RC_ITEM / wxString
/// machinery (which pulls the GUI reporter stack into the link).
enum RL_CLEANUP_CODE
{
    RL_CLEANUP_SHORTING_TRACK = 0,
    RL_CLEANUP_SHORTING_VIA,
    RL_CLEANUP_REDUNDANT_VIA,
    RL_CLEANUP_DUPLICATE_TRACK,
    RL_CLEANUP_MERGE_TRACKS,
    RL_CLEANUP_DANGLING_TRACK,
    RL_CLEANUP_DANGLING_VIA,
    RL_CLEANUP_ZERO_LENGTH_TRACK,
    RL_CLEANUP_TRACK_IN_PAD,
};

/// Stable snake_case name of a cleanup code ("" for an unknown code).
const char* RLCleanupCodeName( int aCode );

/**
 * Which passes to run.  Unlike TRACKS_CLEANER::CleanupBoard's eight positional bools
 * (whose 3rd/4th parameter NAMES are swapped between the stock header and its .cpp),
 * every switch here is named exactly once.
 */
struct RL_CLEANUP_SPEC
{
    bool dry_run         = true;    ///< report only; the board is not touched at all
    bool merge_segments  = false;   ///< collinear merge + duplicate + zero-length tracks
    bool clean_vias      = false;   ///< superimposed vias + vias on all-layer THT pads
    bool remove_shorts   = false;   ///< segments connecting two different nets
    bool tracks_in_pads  = false;   ///< tracks fully buried inside a pad
    bool dangling_tracks = false;   ///< tracks not connected at both ends
    bool dangling_vias   = false;   ///< vias connected on fewer than two layers

    /// Nets the cleaner may touch.  Empty = all nets.  Anything outside the set is
    /// skipped by every pass (same role as TRACKS_CLEANER::SetFilter, but declarative
    /// so no Python callback is ever entered while the board is being mutated).
    std::vector<int> net_codes;
};

/// One recorded cleanup operation.  UUIDs, not pointers: the item may already be gone.
struct RL_CLEANUP_ITEM
{
    int  code;                  ///< RL_CLEANUP_CODE
    KIID item_a = niluuid;      ///< the removed / modified item
    KIID item_b = niluuid;      ///< the merge partner or shorting pad (niluuid if none)
};


class RL_TRACKS_CLEANER
{
public:
    explicit RL_TRACKS_CLEANER( BOARD* aPcb );

    /**
     * Run the requested passes.
     *
     * @param aSpec     which passes to run (see RL_CLEANUP_SPEC)
     * @param aItems    [out] one entry per operation, in execution order
     * @param aDetached [out] items removed from the board but NOT freed — the caller
     *                  owns them and must delete them only after the PNS world and the
     *                  connectivity/ratsnest have stopped referencing them.  Always
     *                  empty for a dry run.
     * @param aModified [out] UUIDs of items whose geometry changed in place (merge
     *                  survivors).  Always empty for a dry run.
     */
    void CleanupBoard( const RL_CLEANUP_SPEC& aSpec, std::vector<RL_CLEANUP_ITEM>& aItems,
                       std::vector<PCB_TRACK*>& aDetached, std::vector<KIID>& aModified );

private:
    /// True when the item is out of scope (net filter) and must not be touched.
    bool filterItem( const BOARD_CONNECTED_ITEM* aItem ) const;

    void removeShortingTrackSegments();
    bool deleteDanglingTracks( bool aTrack, bool aVia );
    void deleteTracksInPads();

    /// Geometry-based cleanup: duplicate items, null items, collinear items.
    void cleanup( bool aDeleteDuplicateVias, bool aDeleteNullSegments,
                  bool aDeleteDuplicateSegments, bool aMergeSegments );

    bool mergeCollinearSegments( PCB_TRACK* aSeg1, PCB_TRACK* aSeg2 );
    bool testMergeCollinearSegments( PCB_TRACK* aSeg1, PCB_TRACK* aSeg2,
                                     PCB_TRACK* aDummySeg = nullptr );
    bool testTrackEndpointIsNode( PCB_TRACK* aTrack, bool aTstStart, bool aTstEnd );

    void record( int aCode, const BOARD_ITEM* aItemA, const BOARD_ITEM* aItemB = nullptr );

    /// BOARD::Remove + record for deferred free.  No-op on a dry run.
    void detach( PCB_TRACK* aTrack );

    /// detach() every item, in KIID order so the free sequence (and therefore the
    /// allocator's address recycling) does not depend on heap layout.
    void detachAll( std::vector<PCB_TRACK*>& aItems );

    const std::vector<BOARD_CONNECTED_ITEM*>& getConnectedItems( PCB_TRACK* aTrack );

    /// Clear the IS_DELETED / SKIP_STRUCT scratch flags off every remaining board track.
    void clearScratchFlags();

private:
    BOARD*                        m_brd;
    RL_CLEANUP_SPEC               m_spec;
    std::unordered_set<int>       m_netFilter;    ///< empty = no filtering
    std::vector<RL_CLEANUP_ITEM>* m_items    = nullptr;
    std::vector<PCB_TRACK*>*      m_detached = nullptr;
    std::vector<KIID>*            m_modified = nullptr;

    // Cache connections.  O(n^2) is awful, but it beats O(2n^3).  Pointer-keyed, but
    // read-only lookup — no ordering decision is derived from it.
    std::map<PCB_TRACK*, std::vector<BOARD_CONNECTED_ITEM*>> m_connectedItemsCache;
};

#endif // RL_TRACKS_CLEANER_H
