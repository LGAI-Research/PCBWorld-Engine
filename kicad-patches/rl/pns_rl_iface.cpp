/*
 * KiCad PNS RL Router Interface – implementation
 */

#include "pns_rl_iface.h"

#include "pns_arc.h"
#include "pns_item.h"
#include "pns_node.h"
#include "pns_segment.h"
#include "pns_via.h"
#include "pns_solid.h"

#include <board.h>
#include <board_connected_item.h>
#include <netinfo.h>
#include <pcb_track.h>          // PCB_TRACK, PCB_VIA, PCB_ARC
#include <geometry/shape_arc.h>
#include <padstack.h>
#include <connectivity/connectivity_data.h>
#include <wx/log.h>


// ---------------------------------------------------------------------------
// Ctor / dtor
// ---------------------------------------------------------------------------

PNS_RL_IFACE::PNS_RL_IFACE()
{
}


PNS_RL_IFACE::~PNS_RL_IFACE()
{
}


// ---------------------------------------------------------------------------
// Net info
// ---------------------------------------------------------------------------

int PNS_RL_IFACE::GetNetCode( PNS::NET_HANDLE aNet ) const
{
    if( aNet )
        return static_cast<NETINFO_ITEM*>( aNet )->GetNetCode();

    return -1;
}


wxString PNS_RL_IFACE::GetNetName( PNS::NET_HANDLE aNet ) const
{
    if( aNet )
        return static_cast<NETINFO_ITEM*>( aNet )->GetNetname();

    return wxEmptyString;
}


// ---------------------------------------------------------------------------
// Board-item creation helper (stripped-down createBoardItem from full iface)
// ---------------------------------------------------------------------------

BOARD_CONNECTED_ITEM* PNS_RL_IFACE::createBoardItem( PNS::ITEM* aItem )
{
    NETINFO_ITEM* net = static_cast<NETINFO_ITEM*>( aItem->Net() );

    if( !net )
        net = NETINFO_LIST::OrphanedItem();

    switch( aItem->Kind() )
    {
    case PNS::ITEM::SEGMENT_T:
    {
        PNS::SEGMENT* seg   = static_cast<PNS::SEGMENT*>( aItem );
        PCB_TRACK*    track = new PCB_TRACK( m_board );
        const SEG&    s     = seg->Seg();

        track->SetStart( VECTOR2I( s.A.x, s.A.y ) );
        track->SetEnd( VECTOR2I( s.B.x, s.B.y ) );
        track->SetWidth( seg->Width() );
        track->SetLayer( GetBoardLayerFromPNSLayer( seg->Layers().Start() ) );
        track->SetNet( net );
        return track;
    }

    case PNS::ITEM::ARC_T:
    {
        PNS::ARC*        arc       = static_cast<PNS::ARC*>( aItem );
        const SHAPE_ARC* arcShape  = static_cast<const SHAPE_ARC*>( arc->Shape( -1 ) );
        PCB_ARC*         new_arc   = new PCB_ARC( m_board, arcShape );

        new_arc->SetWidth( arc->Width() );
        new_arc->SetLayer( GetBoardLayerFromPNSLayer( arc->Layers().Start() ) );
        new_arc->SetNet( net );
        return new_arc;
    }

    case PNS::ITEM::VIA_T:
    {
        PNS::VIA* via       = static_cast<PNS::VIA*>( aItem );
        PCB_VIA*  via_board = new PCB_VIA( m_board );

        via_board->SetPosition( VECTOR2I( via->Pos().x, via->Pos().y ) );
        via_board->SetWidth( PADSTACK::ALL_LAYERS, via->Diameter( 0 ) );
        via_board->SetDrill( via->Drill() );
        via_board->SetNet( net );
        via_board->SetViaType( via->ViaType() );
        via_board->SetLayerPair( GetBoardLayerFromPNSLayer( via->Layers().Start() ),
                                 GetBoardLayerFromPNSLayer( via->Layers().End() ) );
        return via_board;
    }

    default:
        return nullptr;
    }
}


// ---------------------------------------------------------------------------
// In-place board-item modification helper
// ---------------------------------------------------------------------------

void PNS_RL_IFACE::modifyBoardItem( PNS::ITEM* aItem )
{
    BOARD_ITEM* boardItem = aItem->Parent();

    if( !boardItem )
        return;

    switch( aItem->Kind() )
    {
    case PNS::ITEM::SEGMENT_T:
    {
        PNS::SEGMENT* seg   = static_cast<PNS::SEGMENT*>( aItem );
        PCB_TRACK*    track = static_cast<PCB_TRACK*>( boardItem );
        const SEG&    s     = seg->Seg();

        track->SetStart( VECTOR2I( s.A.x, s.A.y ) );
        track->SetEnd( VECTOR2I( s.B.x, s.B.y ) );
        track->SetWidth( seg->Width() );
        break;
    }

    case PNS::ITEM::ARC_T:
    {
        PNS::ARC*        arc      = static_cast<PNS::ARC*>( aItem );
        PCB_ARC*         arc_b    = static_cast<PCB_ARC*>( boardItem );
        const SHAPE_ARC* arcShape = static_cast<const SHAPE_ARC*>( arc->Shape( -1 ) );

        arc_b->SetStart( VECTOR2I( arcShape->GetP0() ) );
        arc_b->SetEnd( VECTOR2I( arcShape->GetP1() ) );
        arc_b->SetMid( VECTOR2I( arcShape->GetArcMid() ) );
        arc_b->SetWidth( arc->Width() );
        break;
    }

    case PNS::ITEM::VIA_T:
    {
        PNS::VIA* via      = static_cast<PNS::VIA*>( aItem );
        PCB_VIA*  via_b    = static_cast<PCB_VIA*>( boardItem );

        via_b->SetPosition( VECTOR2I( via->Pos().x, via->Pos().y ) );
        via_b->SetWidth( PADSTACK::ALL_LAYERS, via->Diameter( 0 ) );
        via_b->SetDrill( via->Drill() );
        via_b->SetLayerPair( GetBoardLayerFromPNSLayer( via->Layers().Start() ),
                             GetBoardLayerFromPNSLayer( via->Layers().End() ) );
        break;
    }

    default:
        break;
    }
}


// ---------------------------------------------------------------------------
// Board-change callbacks (accumulate, applied in Commit())
// ---------------------------------------------------------------------------

void PNS_RL_IFACE::RemoveItem( PNS::ITEM* aItem )
{
    // Skip pads / solids – we do not move footprints in RL routing.
    if( aItem->OfKind( PNS::ITEM::SOLID_T ) )
        return;

    BOARD_ITEM* parent = aItem->Parent();

    if( parent )
        m_pendingRemove.push_back( parent );
}


void PNS_RL_IFACE::AddItem( PNS::ITEM* aItem )
{
    BOARD_CONNECTED_ITEM* boardItem = createBoardItem( aItem );

    if( boardItem )
    {
        aItem->SetParent( boardItem );
        m_pendingAdd.push_back( boardItem );
    }
}


void PNS_RL_IFACE::UpdateItem( PNS::ITEM* aItem )
{
    modifyBoardItem( aItem );
}


// ---------------------------------------------------------------------------
// Commit: apply all pending changes to the BOARD
// ---------------------------------------------------------------------------

void PNS_RL_IFACE::Commit()
{
    for( BOARD_ITEM* item : m_pendingRemove )
    {
        m_board->Remove( item );
        delete item;
    }
    m_pendingRemove.clear();

    for( BOARD_CONNECTED_ITEM* item : m_pendingAdd )
    {
        item->ClearFlags();
        m_board->Add( item );
    }
    m_pendingAdd.clear();

    // Rebuild ratsnest / connectivity so the board state is consistent.
    m_board->GetConnectivity()->Build( m_board );
}


// ---------------------------------------------------------------------------
// Incremental world update (checkpoint restore): add one board item's PNS
// item to a world node, dispatching to the per-type sync helpers.
// ---------------------------------------------------------------------------

void PNS_RL_IFACE::addBoardItemToWorld( PNS::NODE* aWorld, BOARD_ITEM* aItem )
{
    switch( aItem->Type() )
    {
    case PCB_VIA_T:
        if( auto p = syncVia( static_cast<PCB_VIA*>( aItem ) ) )
            aWorld->Add( std::move( p ) );
        break;
    case PCB_ARC_T:
        if( auto p = syncArc( static_cast<PCB_ARC*>( aItem ) ) )
            aWorld->Add( std::move( p ) );
        break;
    case PCB_TRACE_T:
        if( auto p = syncTrack( static_cast<PCB_TRACK*>( aItem ) ) )
            aWorld->Add( std::move( p ) );
        break;
    default:
        break;
    }
}
