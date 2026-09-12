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

/*
 * RL fork of pcbnew/tracks_cleaner.cpp — see tracks_cleaner_rl.h for what differs and
 * why.  Keep the pass bodies structurally close to the stock file so upstream fixes
 * stay reviewable against it; every intentional divergence is marked "RL:".
 */

#include <algorithm>
#include <bit>
#include <deque>

#include <connectivity/connectivity_algo.h>
#include <connectivity/connectivity_data.h>
#include <drc/drc_rtree.h>
#include <lset.h>
#include <pad.h>

#include "tracks_cleaner_rl.h"


const char* RLCleanupCodeName( int aCode )
{
    switch( aCode )
    {
    case RL_CLEANUP_SHORTING_TRACK:    return "shorting_track";
    case RL_CLEANUP_SHORTING_VIA:      return "shorting_via";
    case RL_CLEANUP_REDUNDANT_VIA:     return "redundant_via";
    case RL_CLEANUP_DUPLICATE_TRACK:   return "duplicate_track";
    case RL_CLEANUP_MERGE_TRACKS:      return "merge_tracks";
    case RL_CLEANUP_DANGLING_TRACK:    return "dangling_track";
    case RL_CLEANUP_DANGLING_VIA:      return "dangling_via";
    case RL_CLEANUP_ZERO_LENGTH_TRACK: return "zero_length_track";
    case RL_CLEANUP_TRACK_IN_PAD:      return "track_in_pad";
    default:                           return "";
    }
}


RL_TRACKS_CLEANER::RL_TRACKS_CLEANER( BOARD* aPcb ) :
        m_brd( aPcb )
{
}


void RL_TRACKS_CLEANER::CleanupBoard( const RL_CLEANUP_SPEC& aSpec,
                                      std::vector<RL_CLEANUP_ITEM>& aItems,
                                      std::vector<PCB_TRACK*>&      aDetached,
                                      std::vector<KIID>&            aModified )
{
    m_spec     = aSpec;
    m_items    = &aItems;
    m_detached = &aDetached;
    m_modified = &aModified;

    m_netFilter.clear();
    m_netFilter.insert( aSpec.net_codes.begin(), aSpec.net_codes.end() );
    m_connectedItemsCache.clear();

    bool has_deleted = false;

    // Zero-length segments come along with either merge or shorts (stock behaviour).
    const bool removeNullSegments = aSpec.merge_segments || aSpec.remove_shorts;

    cleanup( aSpec.clean_vias, removeNullSegments, aSpec.merge_segments,
             aSpec.merge_segments );

    // If we didn't remove duplicates above, do it now
    if( !aSpec.merge_segments )
        cleanup( false, false, true, false );

    if( aSpec.remove_shorts )
        removeShortingTrackSegments();

    if( aSpec.tracks_in_pads )
        deleteTracksInPads();

    if( aSpec.dangling_tracks || aSpec.dangling_vias )
        has_deleted = deleteDanglingTracks( aSpec.dangling_tracks, aSpec.dangling_vias );

    if( has_deleted && aSpec.merge_segments )
        cleanup( false, false, false, true );

    // RL: the stock passes leave IS_DELETED set on board items after a dry run (and
    // after the dangling pass in either mode).  A stale IS_DELETED flag makes the NEXT
    // cleanup skip that item silently, so clear the scratch flags unconditionally.
    clearScratchFlags();

    m_items    = nullptr;
    m_detached = nullptr;
    m_modified = nullptr;
    m_connectedItemsCache.clear();
}


bool RL_TRACKS_CLEANER::filterItem( const BOARD_CONNECTED_ITEM* aItem ) const
{
    if( m_netFilter.empty() )
        return false;

    return !m_netFilter.count( aItem->GetNetCode() );
}


void RL_TRACKS_CLEANER::record( int aCode, const BOARD_ITEM* aItemA, const BOARD_ITEM* aItemB )
{
    RL_CLEANUP_ITEM item;
    item.code   = aCode;
    item.item_a = aItemA ? aItemA->m_Uuid : niluuid;
    item.item_b = aItemB ? aItemB->m_Uuid : niluuid;
    m_items->push_back( item );
}


void RL_TRACKS_CLEANER::detach( PCB_TRACK* aTrack )
{
    if( m_spec.dry_run )
        return;

    m_brd->Remove( aTrack );
    m_detached->push_back( aTrack );

    // The cache keys removed items; drop it so no later pass reads a detached pointer.
    m_connectedItemsCache.clear();
}


void RL_TRACKS_CLEANER::detachAll( std::vector<PCB_TRACK*>& aItems )
{
    // RL: KIID order, not the pointer order a std::set<BOARD_ITEM*> would give.  The
    // resulting board is the same either way, but the free sequence (and with it the
    // allocator's address recycling) becomes reproducible run to run.
    std::sort( aItems.begin(), aItems.end(),
               []( const PCB_TRACK* a, const PCB_TRACK* b )
               {
                   return a->m_Uuid < b->m_Uuid;
               } );
    aItems.erase( std::unique( aItems.begin(), aItems.end() ), aItems.end() );

    for( PCB_TRACK* track : aItems )
        detach( track );
}


void RL_TRACKS_CLEANER::clearScratchFlags()
{
    for( PCB_TRACK* track : m_brd->Tracks() )
        track->ClearFlags( IS_DELETED | SKIP_STRUCT );
}


void RL_TRACKS_CLEANER::removeShortingTrackSegments()
{
    std::shared_ptr<CONNECTIVITY_DATA> connectivity = m_brd->GetConnectivity();

    std::vector<PCB_TRACK*> toRemove;

    for( PCB_TRACK* segment : m_brd->Tracks() )
    {
        if( segment->IsLocked() || filterItem( segment ) )
            continue;

        for( PAD* testedPad : connectivity->GetConnectedPads( segment ) )
        {
            if( segment->GetNetCode() != testedPad->GetNetCode() )
            {
                record( segment->Type() == PCB_VIA_T ? RL_CLEANUP_SHORTING_VIA
                                                     : RL_CLEANUP_SHORTING_TRACK,
                        segment, testedPad );
                toRemove.push_back( segment );
            }
        }

        for( PCB_TRACK* testedTrack : connectivity->GetConnectedTracks( segment ) )
        {
            if( segment->GetNetCode() != testedTrack->GetNetCode() )
            {
                record( segment->Type() == PCB_VIA_T ? RL_CLEANUP_SHORTING_VIA
                                                     : RL_CLEANUP_SHORTING_TRACK,
                        segment, testedTrack );
                toRemove.push_back( segment );
            }
        }
    }

    detachAll( toRemove );
}


bool RL_TRACKS_CLEANER::testTrackEndpointIsNode( PCB_TRACK* aTrack, bool aTstStart, bool aTstEnd )
{
    if( !( aTstStart && aTstEnd ) )
        return false;

    // A node is a point where more than 2 items are connected.  However, we elide tracks
    // that are collinear with the track being tested.
    const std::list<CN_ITEM*>& items =
            m_brd->GetConnectivity()->GetConnectivityAlgo()->ItemEntry( aTrack ).GetItems();

    if( items.empty() )
        return false;

    int itemcount = 0;

    for( CN_ITEM* item : items )
    {
        if( !item->Valid() || item->Parent() == aTrack || item->Parent()->HasFlag( IS_DELETED ) )
            continue;

        if( item->Parent()->Type() == PCB_TRACE_T
            && static_cast<PCB_TRACK*>( item->Parent() )->ApproxCollinear( aTrack ) )
        {
            continue;
        }

        for( const std::shared_ptr<CN_ANCHOR>& anchor : item->Anchors() )
        {
            if( ( aTstStart && anchor->Pos() == aTrack->GetStart() )
                && ( aTstEnd && anchor->Pos() == aTrack->GetEnd() ) )
            {
                itemcount++;
                break;
            }
        }
    }

    return itemcount > 1;
}


bool RL_TRACKS_CLEANER::deleteDanglingTracks( bool aTrack, bool aVia )
{
    bool item_erased = false;
    bool modified    = false;

    if( !aTrack && !aVia )
        return false;

    do // Iterate when at least one track is deleted
    {
        item_erased = false;
        // Ensure the connectivity is up to date, especially after removing a dangling
        // segment
        m_brd->BuildConnectivity();

        // Keep a duplicate list to allow deleting in the primary.  RL: KIID order, so
        // the deletion sequence (which drives the next connectivity rebuild, and
        // therefore which further items become dangling) is reproducible.
        std::vector<PCB_TRACK*> ordered( m_brd->Tracks().begin(), m_brd->Tracks().end() );
        std::sort( ordered.begin(), ordered.end(),
                   []( const PCB_TRACK* a, const PCB_TRACK* b )
                   {
                       return a->m_Uuid < b->m_Uuid;
                   } );

        for( PCB_TRACK* track : ordered )
        {
            if( track->HasFlag( IS_DELETED ) || track->IsLocked() || filterItem( track ) )
                continue;

            if( !aVia && track->Type() == PCB_VIA_T )
                continue;

            if( !aTrack && ( track->Type() == PCB_TRACE_T || track->Type() == PCB_ARC_T ) )
                continue;

            // Test if a track (or a via) endpoint is not connected to another track or
            // zone.
            if( m_brd->GetConnectivity()->TestTrackEndpointDangling( track, false ) )
            {
                record( track->Type() == PCB_VIA_T ? RL_CLEANUP_DANGLING_VIA
                                                   : RL_CLEANUP_DANGLING_TRACK,
                        track );
                track->SetFlags( IS_DELETED );

                // keep iterating, because a track connected to the deleted track now
                // perhaps is not connected and should be deleted
                item_erased = true;

                if( !m_spec.dry_run )
                {
                    detach( track );
                    modified = true;
                }
            }
        }
    } while( item_erased && !m_spec.dry_run );
    // RL: a dry run never removes anything, so the stock re-iteration would spin on the
    // same items forever; IS_DELETED marking already reports the full cascade's first
    // wave, and the loop above re-runs only when the board actually shrank.

    return modified;
}


void RL_TRACKS_CLEANER::deleteTracksInPads()
{
    std::vector<PCB_TRACK*> toRemove;

    // Delete tracks that start and end on the same pad
    std::shared_ptr<CONNECTIVITY_DATA> connectivity = m_brd->GetConnectivity();

    for( PCB_TRACK* track : m_brd->Tracks() )
    {
        if( track->IsLocked() || filterItem( track ) )
            continue;

        if( track->Type() == PCB_VIA_T )
            continue;

        // Mark track if connected to pads
        for( PAD* pad : connectivity->GetConnectedPads( track ) )
        {
            if( pad->HitTest( track->GetStart() ) && pad->HitTest( track->GetEnd() ) )
            {
                SHAPE_POLY_SET poly;
                track->TransformShapeToPolygon( poly, track->GetLayer(), 0, ARC_HIGH_DEF,
                                                ERROR_INSIDE );

                poly.BooleanSubtract( *pad->GetEffectivePolygon( track->GetLayer(),
                                                                ERROR_INSIDE ) );

                if( poly.IsEmpty() )
                {
                    record( RL_CLEANUP_TRACK_IN_PAD, track, pad );
                    toRemove.push_back( track );
                    track->SetFlags( IS_DELETED );
                }
            }
        }
    }

    detachAll( toRemove );
}


void RL_TRACKS_CLEANER::cleanup( bool aDeleteDuplicateVias, bool aDeleteNullSegments,
                                 bool aDeleteDuplicateSegments, bool aMergeSegments )
{
    DRC_RTREE rtree;

    for( PCB_TRACK* track : m_brd->Tracks() )
    {
        track->ClearFlags( IS_DELETED | SKIP_STRUCT );
        rtree.Insert( track, track->GetLayer() );
    }

    std::vector<PCB_TRACK*> toRemove;

    for( PCB_TRACK* track : m_brd->Tracks() )
    {
        if( track->HasFlag( IS_DELETED ) || track->IsLocked() || filterItem( track ) )
            continue;

        if( aDeleteDuplicateVias && track->Type() == PCB_VIA_T )
        {
            PCB_VIA* via = static_cast<PCB_VIA*>( track );

            // RL: the stock cleaner normalises the via end even on a dry run — that is
            // a board mutation, so it is gated here.
            if( !m_spec.dry_run && via->GetStart() != via->GetEnd() )
                via->SetEnd( via->GetStart() );

            rtree.QueryColliding( via, via->GetLayer(), via->GetLayer(),
                    // Filter:
                    [&]( BOARD_ITEM* aItem ) -> bool
                    {
                        return aItem->Type() == PCB_VIA_T
                                  && !aItem->HasFlag( SKIP_STRUCT )
                                  && !aItem->HasFlag( IS_DELETED );
                    },
                    // Visitor:
                    [&]( BOARD_ITEM* aItem ) -> bool
                    {
                        PCB_VIA* other = static_cast<PCB_VIA*>( aItem );

                        if( via->GetPosition() == other->GetPosition()
                                && via->GetViaType() == other->GetViaType()
                                && via->GetLayerSet() == other->GetLayerSet() )
                        {
                            record( RL_CLEANUP_REDUNDANT_VIA, via );
                            via->SetFlags( IS_DELETED );
                            toRemove.push_back( via );
                        }

                        return true;
                    } );

            // To delete through Via on THT pads at same location.  Examine the list of
            // connected pads: if a through pad is found, the via is redundant.
            for( PAD* pad : m_brd->GetConnectivity()->GetConnectedPads( via ) )
            {
                const LSET all_cu = LSET::AllCuMask( m_brd->GetCopperLayerCount() );

                if( ( pad->GetLayerSet() & all_cu ) == all_cu )
                {
                    record( RL_CLEANUP_REDUNDANT_VIA, via, pad );
                    via->SetFlags( IS_DELETED );
                    toRemove.push_back( via );
                    break;
                }
            }

            via->SetFlags( SKIP_STRUCT );
        }

        if( aDeleteNullSegments && track->Type() != PCB_VIA_T )
        {
            if( track->IsNull() )
            {
                record( RL_CLEANUP_ZERO_LENGTH_TRACK, track );
                track->SetFlags( IS_DELETED );
                toRemove.push_back( track );
            }
        }

        if( aDeleteDuplicateSegments && track->Type() == PCB_TRACE_T && !track->IsNull() )
        {
            rtree.QueryColliding( track, track->GetLayer(), track->GetLayer(),
                    // Filter:
                    [&]( BOARD_ITEM* aItem ) -> bool
                    {
                        return aItem->Type() == PCB_TRACE_T
                                  && !aItem->HasFlag( SKIP_STRUCT )
                                  && !aItem->HasFlag( IS_DELETED )
                                  && !static_cast<PCB_TRACK*>( aItem )->IsNull();
                    },
                    // Visitor:
                    [&]( BOARD_ITEM* aItem ) -> bool
                    {
                        PCB_TRACK* other = static_cast<PCB_TRACK*>( aItem );

                        if( track->IsPointOnEnds( other->GetStart() )
                                && track->IsPointOnEnds( other->GetEnd() )
                                && track->GetWidth() == other->GetWidth()
                                && track->GetLayer() == other->GetLayer() )
                        {
                            record( RL_CLEANUP_DUPLICATE_TRACK, track );
                            track->SetFlags( IS_DELETED );
                            toRemove.push_back( track );
                        }

                        return true;
                    } );

            track->SetFlags( SKIP_STRUCT );
        }
    }

    detachAll( toRemove );

    // RL: serial collection (the stock version parallelises this loop, chunked by
    // thread-pool size, so the merge fixpoint could depend on how many threads the
    // process happens to run with) + KIID-ordered application.
    auto mergeSegments = [&]( std::shared_ptr<CN_CONNECTIVITY_ALGO> connectivity ) -> bool
    {
        std::vector<std::pair<PCB_TRACK*, PCB_TRACK*>> pairs;

        for( PCB_TRACK* segment : m_brd->Tracks() )
        {
            // one can merge only collinear segments, not vias or arcs.
            if( segment->Type() != PCB_TRACE_T )
                continue;

            if( segment->HasFlag( IS_DELETED ) ) // already taken into account
                continue;

            if( filterItem( segment ) )
                continue;

            // for each end of the segment:
            for( CN_ITEM* citem : connectivity->ItemEntry( segment ).GetItems() )
            {
                // Do not merge an end which has different width tracks attached -- it's
                // a common use-case for necking-down a track between pads.
                std::vector<PCB_TRACK*> sameWidthCandidates;
                bool                    differentWidth = false;

                for( CN_ITEM* connected : citem->ConnectedItems() )
                {
                    if( !connected->Valid() )
                        continue;

                    BOARD_CONNECTED_ITEM* candidate = connected->Parent();

                    if( candidate->Type() == PCB_TRACE_T && !candidate->HasFlag( IS_DELETED )
                        && !filterItem( candidate ) )
                    {
                        PCB_TRACK* candidateSegment = static_cast<PCB_TRACK*>( candidate );

                        if( candidateSegment->GetWidth() == segment->GetWidth() )
                        {
                            sameWidthCandidates.push_back( candidateSegment );
                        }
                        else
                        {
                            differentWidth = true;
                            break;
                        }
                    }
                }

                if( differentWidth )
                    continue;

                // RL: KIID order, not pointer order.  The stock `candidate < segment`
                // deduplicates the symmetric pair by heap address, which decides which
                // of the two survives the merge — so the surviving UUID varied run to
                // run.  Ordering by UUID makes the lower-UUID segment the survivor,
                // always.
                std::sort( sameWidthCandidates.begin(), sameWidthCandidates.end(),
                           []( const PCB_TRACK* a, const PCB_TRACK* b )
                           {
                               return a->m_Uuid < b->m_Uuid;
                           } );

                for( PCB_TRACK* candidate : sameWidthCandidates )
                {
                    if( candidate->m_Uuid < segment->m_Uuid ) // avoid duplicate merges
                        continue;

                    // Candidates arrive via connectivity, which crosses layers through
                    // vias/thru-pads — without this gate two collinear same-net
                    // same-width tracks on DIFFERENT layers joined by a via passed every
                    // check and mergeCollinearSegments silently deleted the other
                    // layer's segment.  (Same fix as the stock-file overlay.)
                    if( candidate->GetLayer() != segment->GetLayer() )
                        continue;

                    if( segment->ApproxCollinear( *candidate )
                        && testMergeCollinearSegments( segment, candidate ) )
                    {
                        pairs.emplace_back( segment, candidate );
                        break;
                    }
                }
            }
        }

        std::sort( pairs.begin(), pairs.end(),
                   []( const std::pair<PCB_TRACK*, PCB_TRACK*>& a,
                       const std::pair<PCB_TRACK*, PCB_TRACK*>& b )
                   {
                       if( a.first->m_Uuid == b.first->m_Uuid )
                           return a.second->m_Uuid < b.second->m_Uuid;

                       return a.first->m_Uuid < b.first->m_Uuid;
                   } );

        bool retval = false;

        for( auto& [seg1, seg2] : pairs )
        {
            retval = true;

            if( seg1->HasFlag( IS_DELETED ) || seg2->HasFlag( IS_DELETED ) )
                continue;

            mergeCollinearSegments( seg1, seg2 );
        }

        return retval;
    };

    if( aMergeSegments )
    {
        // RL: a dry run reports the mergeable pairs once — without a board mutation the
        // stock do/while would never reach its fixpoint.
        if( m_spec.dry_run )
        {
            m_brd->BuildConnectivity();
            m_connectedItemsCache.clear();
            mergeSegments( m_brd->GetConnectivity()->GetConnectivityAlgo() );
        }
        else
        {
            do
            {
                m_brd->BuildConnectivity();
                m_connectedItemsCache.clear();
            } while( mergeSegments( m_brd->GetConnectivity()->GetConnectivityAlgo() ) );
        }
    }

    clearScratchFlags();
}


const std::vector<BOARD_CONNECTED_ITEM*>& RL_TRACKS_CLEANER::getConnectedItems( PCB_TRACK* aTrack )
{
    static const std::vector<KICAD_T> connectedTypes = { PCB_TRACE_T,
                                                         PCB_ARC_T,
                                                         PCB_VIA_T,
                                                         PCB_PAD_T,
                                                         PCB_ZONE_T };

    const std::shared_ptr<CONNECTIVITY_DATA>& connectivity = m_brd->GetConnectivity();

    if( !m_connectedItemsCache.contains( aTrack ) )
        m_connectedItemsCache[aTrack] = connectivity->GetConnectedItems( aTrack, connectedTypes );

    return m_connectedItemsCache.at( aTrack );
}


bool RL_TRACKS_CLEANER::testMergeCollinearSegments( PCB_TRACK* aSeg1, PCB_TRACK* aSeg2,
                                                    PCB_TRACK* aDummySeg )
{
    if( aSeg1->IsLocked() || aSeg2->IsLocked() )
        return false;

    // Collect the unique points where the two tracks are connected to other items
    const unsigned p1s = 1 << 0;
    const unsigned p1e = 1 << 1;
    const unsigned p2s = 1 << 2;
    const unsigned p2e = 1 << 3;

    std::vector<VECTOR2I> pts = { aSeg1->GetStart(), aSeg1->GetEnd(),
                                  aSeg2->GetStart(), aSeg2->GetEnd() };
    unsigned              flags = 0;

    auto collectPts =
            [&]( BOARD_CONNECTED_ITEM* citem, PCB_TRACK* seg, unsigned startBit, unsigned endBit )
            {
                if( std::popcount( flags ) > 2 )
                    return;

                if( citem->Type() == PCB_TRACE_T || citem->Type() == PCB_ARC_T
                        || citem->Type() == PCB_VIA_T )
                {
                    PCB_TRACK* track = static_cast<PCB_TRACK*>( citem );

                    if( track->IsPointOnEnds( seg->GetStart() ) )
                        flags |= startBit;

                    if( track->IsPointOnEnds( seg->GetEnd() ) )
                        flags |= endBit;
                }
                else
                {
                    if( !( flags & startBit )
                        && citem->HitTest( seg->GetStart(), ( seg->GetWidth() + 1 ) / 2 ) )
                        flags |= startBit;

                    if( !( flags & endBit )
                        && citem->HitTest( seg->GetEnd(), ( seg->GetWidth() + 1 ) / 2 ) )
                        flags |= endBit;
                }
            };

    for( BOARD_CONNECTED_ITEM* item : getConnectedItems( aSeg1 ) )
    {
        if( item->HasFlag( IS_DELETED ) )
            continue;

        if( item != aSeg1 && item != aSeg2 )
            collectPts( item, aSeg1, p1s, p1e );
    }

    for( BOARD_CONNECTED_ITEM* item : getConnectedItems( aSeg2 ) )
    {
        if( item->HasFlag( IS_DELETED ) )
            continue;

        if( item != aSeg1 && item != aSeg2 )
            collectPts( item, aSeg2, p2s, p2e );
    }

    // This means there is a node in the center
    if( std::popcount( flags ) > 2 )
        return false;

    // Verify the removed point after merging is not a node.  If it is a node (i.e. if
    // more than one other item is connected), the segments cannot be merged.
    PCB_TRACK dummy_seg( *aSeg1 );

    if( !aDummySeg )
        aDummySeg = &dummy_seg;

    // Do not copy the parent group to the dummy segment
    dummy_seg.SetParentGroup( nullptr );

    // Calculate the new ends of the segment to merge, and store them to dummy_seg:
    int min_x = std::min( aSeg1->GetStart().x,
            std::min( aSeg1->GetEnd().x, std::min( aSeg2->GetStart().x, aSeg2->GetEnd().x ) ) );
    int min_y = std::min( aSeg1->GetStart().y,
            std::min( aSeg1->GetEnd().y, std::min( aSeg2->GetStart().y, aSeg2->GetEnd().y ) ) );
    int max_x = std::max( aSeg1->GetStart().x,
            std::max( aSeg1->GetEnd().x, std::max( aSeg2->GetStart().x, aSeg2->GetEnd().x ) ) );
    int max_y = std::max( aSeg1->GetStart().y,
            std::max( aSeg1->GetEnd().y, std::max( aSeg2->GetStart().y, aSeg2->GetEnd().y ) ) );

    if( ( aSeg1->GetStart().x > aSeg1->GetEnd().x )
            == ( aSeg1->GetStart().y > aSeg1->GetEnd().y ) )
    {
        aDummySeg->SetStart( VECTOR2I( min_x, min_y ) );
        aDummySeg->SetEnd( VECTOR2I( max_x, max_y ) );
    }
    else
    {
        aDummySeg->SetStart( VECTOR2I( min_x, max_y ) );
        aDummySeg->SetEnd( VECTOR2I( max_x, min_y ) );
    }

    // The new ends of the segment must be connected to all of the same points as the
    // original segments.  If not, the segments cannot be merged.
    for( unsigned i = 0; i < 4; ++i )
    {
        if( ( flags & ( 1 << i ) ) && !aDummySeg->IsPointOnEnds( pts[i] ) )
            return false;
    }

    // Now find the removed end(s) and stop merging if it is a node:
    return !testTrackEndpointIsNode( aSeg1, aDummySeg->IsPointOnEnds( aSeg1->GetStart() ),
                                     aDummySeg->IsPointOnEnds( aSeg1->GetEnd() ) );
}


bool RL_TRACKS_CLEANER::mergeCollinearSegments( PCB_TRACK* aSeg1, PCB_TRACK* aSeg2 )
{
    PCB_TRACK dummy_seg( *aSeg1 );

    dummy_seg.SetParentGroup( nullptr );

    if( !testMergeCollinearSegments( aSeg1, aSeg2, &dummy_seg ) )
        return false;

    record( RL_CLEANUP_MERGE_TRACKS, aSeg1, aSeg2 );

    aSeg2->SetFlags( IS_DELETED );

    if( !m_spec.dry_run )
    {
        PCB_GROUP* group = aSeg1->GetParentGroup();
        *aSeg1 = dummy_seg;
        aSeg1->SetParentGroup( group );

        m_brd->GetConnectivity()->Update( aSeg1 );
        m_modified->push_back( aSeg1->m_Uuid );

        // Merge successful, seg2 has to go away
        detach( aSeg2 );
    }

    return true;
}
