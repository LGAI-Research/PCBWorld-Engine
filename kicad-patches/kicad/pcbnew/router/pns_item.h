/*
 * KiRouter - a push-and-(sometimes-)shove PCB router
 *
 * Copyright (C) 2013-2017 CERN
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
 *
 * @author Tomasz Wlostowski <tomasz.wlostowski@cern.ch>
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PNS_ITEM_H
#define PNS_ITEM_H

#include <atomic>
#include <cstdint>
#include <memory>
#include <set>
#include <unordered_set>
#include <kiid.h>
#include <math/vector2d.h>

#include <geometry/shape.h>
#include <geometry/shape_line_chain.h>

#include "pns_layerset.h"

class BOARD_ITEM;

namespace PNS {

class NODE;

enum LineMarker {
    MK_HEAD         = ( 1 << 0 ),
    MK_VIOLATION    = ( 1 << 3 ),
    MK_LOCKED       = ( 1 << 4 ),
    MK_DP_COUPLED   = ( 1 << 5 )
};


class ITEM;
class HOLE;
struct COLLISION_SEARCH_CONTEXT;

// An opaque net identifier.  The internal workings are owned by the ROUTER_IFACE.
typedef void* NET_HANDLE;

class ITEM_OWNER {
public:
    virtual ~ITEM_OWNER() {};
};

class OWNABLE_ITEM
{
public:
    OWNABLE_ITEM() :
        m_owner( nullptr )
    {}

    /**
     * Return the owner of this item, or NULL if there's none.
     */
    const ITEM_OWNER* Owner() const { return m_owner; }

    /**
     * Set the node that owns this item. An item can belong to a single NODE or be unowned.
     */
    void SetOwner( const ITEM_OWNER* aOwner ) { m_owner = aOwner; }

    /**
     * @return true if the item is owned by the node aNode.
     */
    bool BelongsTo( const ITEM_OWNER* aNode ) const
    {
        return m_owner == aNode;
    }

protected:
    const ITEM_OWNER *m_owner;
};

/**
 * Base class for PNS router board items.
 *
 * Implements the shared properties of all PCB items  net, spanned layers, geometric shape and
 * reference to owning model.
 */
class ITEM : public OWNABLE_ITEM, public ITEM_OWNER
{
public:
    ///< Supported item types
    enum PnsKind
    {
        INVALID_T   =    0,
        SOLID_T     =    1,
        LINE_T      =    2,
        JOINT_T     =    4,
        SEGMENT_T   =    8,
        ARC_T       =   16,
        VIA_T       =   32,
        DIFF_PAIR_T =   64,
        HOLE_T      =   128,
        ANY_T       =   0xffff,
        LINKED_ITEM_MASK_T = SOLID_T | SEGMENT_T | ARC_T | VIA_T | HOLE_T
    };

    ITEM( PnsKind aKind )
    {
        m_net = nullptr;
        m_movable = true;
        m_kind = aKind;
        m_parent = nullptr;
        m_sourceItem = nullptr;
        m_owner = nullptr;
        m_marker = 0;
        m_rank = -1;
        m_routable = true;
        m_isVirtual = false;
        m_isFreePad = false;
        m_isCompoundShapePrimitive = false;
        m_serial = nextSerial();
    }

    ITEM( const ITEM& aOther )
    {
        m_layers = aOther.m_layers;
        m_net = aOther.m_net;
        m_movable = aOther.m_movable;
        m_kind = aOther.m_kind;
        m_parent = aOther.m_parent;
        m_parentUuid = aOther.m_parentUuid;   // travels with m_parent, never re-derived
        m_sourceItem = aOther.m_sourceItem;
        m_owner = nullptr;
        m_marker = aOther.m_marker;
        m_rank = aOther.m_rank;
        m_routable = aOther.m_routable;
        m_isVirtual = aOther.m_isVirtual;
        m_isFreePad = aOther.m_isFreePad;
        m_isCompoundShapePrimitive = aOther.m_isCompoundShapePrimitive;
        m_serial = nextSerial();   // a clone is a NEW object: fresh creation serial
    }

    // Explicit copy-assignment, copying every member EXCEPT m_serial, which is
    // object identity (creation order) and must never be duplicated onto
    // another live object: two items sharing a serial would compare equal in
    // the serial tie-break and collapse in ordered sets. Base
    // OWNABLE_ITEM::operator= copies m_owner as usual.
    ITEM& operator=( const ITEM& aOther )
    {
        OWNABLE_ITEM::operator=( aOther );
        m_layers = aOther.m_layers;
        m_net = aOther.m_net;
        m_movable = aOther.m_movable;
        m_kind = aOther.m_kind;
        m_parent = aOther.m_parent;
        m_parentUuid = aOther.m_parentUuid;   // travels with m_parent, never re-derived
        m_sourceItem = aOther.m_sourceItem;
        m_marker = aOther.m_marker;
        m_rank = aOther.m_rank;
        m_routable = aOther.m_routable;
        m_isVirtual = aOther.m_isVirtual;
        m_isFreePad = aOther.m_isFreePad;
        m_isCompoundShapePrimitive = aOther.m_isCompoundShapePrimitive;
        // m_serial: intentionally NOT copied — the target keeps its own identity.
        return *this;
    }

    /**
     * Determinism: process-wide allocation-order serial (1, 2, 3, ...) minted
     * at construction — clones included. Replaces the heap address as the final
     * ordering tie-break for items that have no board UUID (uncommitted
     * temporaries) and compare equal geometrically: unlike the address, the
     * creation order is a pure function of the (deterministic) routing execution,
     * so it is reproducible across processes/ASLR/allocator history.
     */
    uint64_t Serial() const { return m_serial; }

    virtual ~ITEM();

    /**
     * Return a deep copy of the item.
     */
    virtual ITEM* Clone() const = 0;

    /*
     * Returns a convex polygon "hull" of a the item, that is used as the walk-around path.
     *
     * @param aClearance defines how far from the body of the item the hull should be,
     * @param aWalkaroundThickness is the width of the line that walks around this hull.
     * @param aLayer is the layer to build a hull for (the item may have different shapes on each
     *               layer).  If aLayer is -1, the hull will be a merged hull from all layers.
     */
    virtual const SHAPE_LINE_CHAIN Hull( int aClearance = 0, int aWalkaroundThickness = 0,
                                         int aLayer = -1 ) const
    {
        return SHAPE_LINE_CHAIN();
    }

    /**
     * Return the type (kind) of the item.
     */
    PnsKind Kind() const
    {
        return m_kind;
    }

    /**
     * @return true if the item's type matches the mask \a aKindMask.
     */
    bool OfKind( int aKindMask ) const
    {
        return ( aKindMask & m_kind ) != 0;
    }

    /**
     * @return the kind of the item, as string
     */
    std::string KindStr() const;

    // Defined out-of-line (pns_item.cpp): it caches the parent's KIID, which needs
    // BOARD_ITEM's definition, and this header only forward-declares it.
    void SetParent( BOARD_ITEM* aParent );

    BOARD_ITEM* Parent() const { return m_parent; }

    /**
     * The parent BOARD_ITEM's UUID, copied at SetParent() time — nil for an item with
     * no board parent (a temporary, or a segment not yet committed).
     *
     * Read this instead of `Parent()->m_Uuid` in any ordering: NODE::Commit sorts its
     * `removed` set AFTER ROUTER::CommitRouting has called m_iface->Commit(), which
     * already deleted those items' board parents, so going through the pointer there
     * is a use-after-free. Caching the value is what lets ONE comparator
     * (canonicalItemCompare) serve every call site.
     */
    const KIID& ParentUuid() const { return m_parentUuid; }

    void SetSourceItem( BOARD_ITEM* aSourceItem ) { m_sourceItem = aSourceItem; }
    BOARD_ITEM* GetSourceItem() const { return m_sourceItem; }

    /**
     * @return the BOARD_ITEM, even if it's not the direct parent.
     */
    virtual BOARD_ITEM* BoardItem() const { return m_parent; }

    void SetNet( NET_HANDLE aNet ) { m_net = aNet; }
    virtual NET_HANDLE Net() const { return m_net;  }

    const PNS_LAYER_RANGE& Layers() const { return m_layers; }
    void SetLayers( const PNS_LAYER_RANGE& aLayers ) { m_layers = aLayers; }

    void SetLayer( int aLayer ) { m_layers = PNS_LAYER_RANGE( aLayer, aLayer ); }
    virtual int Layer() const { return Layers().Start(); }

    /**
     * Return true if the set of layers spanned by aOther overlaps our layers.
     */
    bool LayersOverlap( const ITEM* aOther ) const
    {
        return Layers().Overlaps( aOther->Layers() );
    }

    /**
     * Check for a collision (clearance violation) with between us and item \a aOther.
     *
     * Collision checking takes all PCB stuff into account (layers, nets, DRC rules).
     * Optionally returns a minimum translation vector for force propagation algorithm.
     *
     * @param aOther is the item to check collision against.
     * @return true, if a collision was found.
     */
    bool Collide( const ITEM* aHead, const NODE* aNode, int aLayer,
                  COLLISION_SEARCH_CONTEXT* aCtx = nullptr ) const;

    /**
     * Return the geometrical shape of the item. Used for collision detection and spatial indexing.
     * @param aLayer is the layer to query shape for (items may have different shapes on different layers)
     */
    virtual const SHAPE* Shape( int aLayer ) const
    {
        return nullptr;
    }

    /**
     * Return a list of layers that have unique (potentially different) shapes
     */
    virtual std::vector<int> UniqueShapeLayers() const { return { -1 }; }

    virtual bool HasUniqueShapeLayers() const { return false; }

    /**
     * Returns the set of layers on which either this or the other item can have a unique shape.
     * Use this to loop over layers when hit-testing objects that can have different shapes on
     * each layer (currently only VIA)
     */
    std::set<int> RelevantShapeLayers( const ITEM* aOther ) const;

    virtual void Mark( int aMarker ) const { m_marker = aMarker; }
    virtual void Unmark( int aMarker = -1 ) const { m_marker &= ~aMarker; }
    virtual int Marker() const { return m_marker; }

    virtual void SetRank( int aRank ) { m_rank = aRank; }
    virtual int Rank() const { return m_rank; }

    virtual VECTOR2I Anchor( int n ) const
    {
        return VECTOR2I();
    }

    virtual int AnchorCount() const
    {
        return 0;
    }

    bool IsLocked() const
    {
        return Marker() & MK_LOCKED;
    }

    void SetRoutable( bool aRoutable ) { m_routable = aRoutable; }
    bool IsRoutable() const { return m_routable; }

    void SetIsFreePad( bool aIsFreePad = true ) { m_isFreePad = aIsFreePad; }

    bool IsFreePad() const
    {
        return m_isFreePad || ( ParentPadVia() && ParentPadVia()->m_isFreePad );
    }

    virtual ITEM* ParentPadVia() const { return nullptr; }

    bool IsVirtual() const
    {
        return m_isVirtual;
    }

    void SetIsCompoundShapePrimitive() { m_isCompoundShapePrimitive = true; }
    bool IsCompoundShapePrimitive() const { return m_isCompoundShapePrimitive; }

    virtual bool HasHole() const { return false; }
    virtual HOLE *Hole() const { return nullptr; }
    virtual void SetHole( HOLE* aHole ) {};

    virtual const std::string Format() const;

    virtual const NODE* OwningNode() const;

private:
    bool collideSimple( const ITEM* aHead, const NODE* aNode, int aLayer,
                        COLLISION_SEARCH_CONTEXT* aCtx ) const;

    // Monotonic process-wide creation counter behind Serial(). Atomic for safety
    // (routing itself is single-threaded, so the observed order — the only thing
    // the tie-break consumes — is the deterministic execution order).
    static uint64_t nextSerial()
    {
        static std::atomic<uint64_t> s_counter{ 0 };
        return ++s_counter;
    }

    uint64_t        m_serial;

    // The parent's UUID, cached by SetParent(). NOTE the explicit nil initialiser:
    // KIID's DEFAULT constructor mints a fresh UUID, which would draw from the seeded
    // process-wide KIID generator and desynchronise the UUID stream our checkpoints
    // save/restore. Never default-construct this.
    KIID            m_parentUuid{ 0 };

protected:
    PnsKind         m_kind;
    BOARD_ITEM*     m_parent;       // The parent BOARD_ITEM, used when there is a 1:1 map
                                    //   between the PNS::ITEM and the BOARD_ITEM.
    BOARD_ITEM*     m_sourceItem;   // The progenator BOARD_ITEM for when there is NOT a 1:1 map.
                                    //   For instance, dragging a track might produce multiple
                                    //   segments, none of which can be directly mapped to the
                                    //   track.
    PNS_LAYER_RANGE m_layers;

    bool            m_movable;
    NET_HANDLE      m_net;
    mutable int     m_marker;
    int             m_rank;
    bool            m_routable;
    bool            m_isVirtual;
    bool            m_isFreePad;
    bool            m_isCompoundShapePrimitive;
};


/**
 * The ONE canonical ordering of PNS items (defined in pns_node.cpp).
 *
 * Every place that would otherwise fall back on a heap address must route its
 * tie-break through this: OBSTACLE::operator< (pns_node.h),
 * TOPOLOGY::CLUSTER::ItemCmp and TOPOLOGY::NearestUnconnectedItem (pns_topology.*),
 * NODE::ItemOrder (Branch's child index, CanonicalizeOrder's joint links,
 * GetUpdatedItems / Commit emission) and the RL layer's obstacle pick.
 *
 * Keys, in order — all CONTENT-derived, so two worlds holding the same items order
 * them identically however they were built:
 *   parent UUID (cached, see ITEM::ParentUuid — nil, i.e. parentless, sorts first)
 *   → kind → layer start → layer end → anchors → anchor count → Serial().
 *
 * Returns <0 / 0 / >0. Zero is reported only for the very same object; distinct items
 * always differ somewhere, so ordered sets never collapse two colliding fragments.
 * Serial() is the last resort for exact-duplicate temporaries and is the one key that
 * is creation-order rather than content — see the note at its definition.
 */
int canonicalItemCompare( const ITEM* aA, const ITEM* aB );


template<typename T, typename S>
std::unique_ptr<T> ItemCast( std::unique_ptr<S> aPtr )
{
    static_assert( std::is_base_of<ITEM, S>::value, "Need to be handed a ITEM!" );
    static_assert( std::is_base_of<ITEM, T>::value, "Need to cast to an ITEM!" );
    return std::unique_ptr<T>( static_cast<T*>( aPtr.release() ) );
}

template<typename T>
std::unique_ptr< typename std::remove_const<T>::type > Clone( const T& aItem )
{
    static_assert( std::is_base_of<ITEM, T>::value, "Need to be handed an ITEM!" );
    return std::unique_ptr<typename std::remove_const<T>::type>( aItem.Clone() );
}

}

#endif    // PNS_ITEM_H
