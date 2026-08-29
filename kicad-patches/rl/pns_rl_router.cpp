/*
 * PNS_RL_ROUTER – implementation
 */

#include "pns_rl_router.h"
#include "pns_rl_iface.h"
#include "tracks_cleaner_rl.h"

#include "pns_router.h"
#include "pns_routing_settings.h"
#include "pns_sizes_settings.h"
#include <geometry/direction45.h>
#include "pns_node.h"
#include "pns_item.h"
#include "pns_itemset.h"
#include "pns_line.h"
#include "pns_segment.h"
#include "pns_placement_algo.h"

#include <board.h>
#include <board_design_settings.h>
#include <project/net_settings.h>
#include <netclass.h>
#include <pcb_io/kicad_sexpr/pcb_io_kicad_sexpr.h>
#include <pcb_io/pcb_io.h>
#include <io/io_mgr.h>
#include <pcb_track.h>
#include <netinfo.h>
#include <kiid.h>   // KIID::Get/SetGeneratorState — checkpoint/restore UUID-stream rewind
#include <math/vector2d.h>

#include <pad.h>
#include <footprint.h>
#include <zone.h>
#include <layer_range.h>
#include <padstack.h>
#include <pcb_shape.h>
#include <eda_shape.h>
#include <geometry/shape_poly_set.h>
#include <geometry/shape_arc.h>
#include <geometry/shape_line_chain.h>
#include <connectivity/connectivity_data.h>
#include <connectivity/connectivity_algo.h>
#include <connectivity/connectivity_items.h>
#include <ratsnest/ratsnest_data.h>

#include <drc/drc_engine.h>
#include <drc/drc_rl_scope.h>   // RL incremental-DRC clearance scope (no core DRC patch)
#include <drc/drc_item.h>
#include <board_connected_item.h>
#include <eda_units.h>
#include <base_units.h>   // pcbIUScale (mm→IU rounding; see nmFromMm)
#include <math/box2.h>

#include <netclass.h>
#include <project/net_settings.h>
#include <project.h>
#include <settings/settings_manager.h>

#include <wx/filename.h>
#include <wx/log.h>

#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <random>
#include <chrono>
#include <cstdint>

// ---------------------------------------------------------------------------
// Helper: mm ↔ nm
//
// mm→nm delegates to pcbIUScale.mmToIU (round-half-away), matching KiCad
// proper. Rounding (not truncating) matters: a caller-side double sitting a
// hair below a clean value (float arithmetic noise, e.g. 146.1+3.7 =
// 149.79999999999998) would otherwise land a full 1 nm short, seeding
// …999999-coordinate junk at the API boundary before any hull math. Every
// coordinate-input path funnels through this helper.
// ---------------------------------------------------------------------------
static inline int nmFromMm( double mm ) { return pcbIUScale.mmToIU( mm ); }
static inline double mmFromNm( int nm )  { return pcbIUScale.IUTomm( nm ); }


// ---------------------------------------------------------------------------
// Ctor / dtor
// ---------------------------------------------------------------------------

// Resolve the .kicad_pro path to associate with @a board_path.
// Explicit @a pro_path wins; otherwise <board_stem>.kicad_pro sibling is used.
// Returns the chosen path (which may point to a non-existent file — that is
// handled as the "in-memory fallback" case by the caller).
static std::string resolveProjectPath( const std::string& board_path,
                                       const std::string& pro_path )
{
    if( !pro_path.empty() )
        return pro_path;

    wxFileName fn( wxString::FromUTF8( board_path ) );
    fn.SetExt( wxT( "kicad_pro" ) );
    return fn.GetFullPath().ToStdString();
}


PNS_RL_ROUTER::PNS_RL_ROUTER( const std::string& board_path,
                              const std::string& project_path,
                              int engineSeed,
                              int shoveIterLimit,
                              int followBranchIterLimit )
    : m_shoveIterLimit( shoveIterLimit ),
      m_followBranchIterLimit( followBranchIterLimit )
{
    // Optionally make UUID generation deterministic BEFORE loading the board, so
    // both the load and all subsequent routing draw a reproducible KIID stream. The
    // PNS obstacle set is ordered by UUID, so a fixed seed makes routing — and
    // UUID-keyed DRC — reproducible across runs/processes for a fixed action sequence.
    // Decided once here, never re-seeded. (Global generator; with multiple engines per
    // process the last constructor's seed wins — use one engine per process for clean
    // determinism.)
    if( engineSeed >= 0 )
        KIID::SeedGenerator( static_cast<unsigned int>( engineSeed ) );

    // Always attach a PROJECT (see header comment). This drives the legacy
    // design-setting migration: parseSetup() populates BDS from legacy
    // tokens and marks m_LegacyDesignSettingsLoaded=true; BOARD::SetProject
    // then preserves those values by calling SetParent(&project, false).
    m_projectPath = resolveProjectPath( board_path, project_path );

    m_settingsMgr = std::make_unique<SETTINGS_MANAGER>( /*aHeadless=*/true );

    // LoadProject still registers a fresh in-memory PROJECT under
    // m_projectPath even when the file is missing/unreadable, so Prj() is
    // always safe to call afterwards. The return value tells us whether
    // the on-disk JSON was actually read — exposed via
    // wasProjectLoadedFromFile() so callers can distinguish authoritative
    // rules from the default fallback.
    const wxString wxPath = wxString::FromUTF8( m_projectPath );
    {
        wxLogNull suppress; // silence "file not found" noise for the fallback case
        m_projectLoadedFromFile = m_settingsMgr->LoadProject( wxPath );
    }
    m_project = &m_settingsMgr->Prj();

    try
    {
        IO_RELEASER<PCB_IO> pi( new PCB_IO_KICAD_SEXPR );
        m_board = pi->LoadBoard( wxString::FromUTF8( board_path ),
                                 /*aAppendToMe=*/nullptr,
                                 /*aProperties=*/nullptr,
                                 /*aProject=*/m_project );
    }
    catch( const IO_ERROR& ioe )
    {
        throw std::runtime_error( "PNS_RL_ROUTER: failed to load board: "
                                  + board_path + " – "
                                  + ioe.Problem().ToStdString() );
    }

    if( !m_board )
        throw std::runtime_error( "PNS_RL_ROUTER: failed to load board: " + board_path );

    // Bidirectional link: if the project JSON was read, BDS is overwritten
    // from it; otherwise SetParent(..., aLoadFromFile=!LegacyLoaded)
    // preserves whatever parseSetup() already wrote (legacy path) or takes
    // KiCad defaults (modern pcb w/o pro file).
    m_board->SetProject( m_project );

    // Net-to-netclass matching reads NET_SETTINGS.netclass_patterns; without
    // this call all nets would fall into "Default" and per-class clearance
    // would not apply. pcbnew does the same thing right after load.
    m_board->SynchronizeNetsAndNetClasses( /*aResetTrackAndViaSizes=*/true );

    initRouter();

    // Capture the UUID-generator position now — post board-load, post initial
    // connectivity, before any routing — so rewindKIIDToEpisodeStart() can return here
    // at every episode reset and hand each episode an identical UUID stream. Only
    // meaningful under deterministic seeding; entropy mode leaves this empty so the
    // rewind is a no-op and per-episode UUIDs stay non-reproducible as before.
    if( engineSeed >= 0 )
        m_episodeStartKiidState = KIID::GetGeneratorState();
}


PNS_RL_ROUTER::~PNS_RL_ROUTER()
{
    // m_router must be destroyed before m_iface (it holds a raw pointer to it)
    m_router.reset();
    m_iface.reset();

    // m_board holds a raw PROJECT* into m_settingsMgr — destroy it first so
    // the project outlives any BOARD cleanup that dereferences the pointer.
    delete m_board;
    m_board = nullptr;

    m_project = nullptr;
    m_settingsMgr.reset();
}


bool PNS_RL_ROUTER::wasLegacyDesignSettingsLoaded() const
{
    return m_board ? m_board->m_LegacyDesignSettingsLoaded : false;
}


// ---------------------------------------------------------------------------
// Internal: initialise / reinitialise the PNS router
// ---------------------------------------------------------------------------

void PNS_RL_ROUTER::initRouter()
{
    reseedCheckpointEpoch();   // unique handle epoch for this router instance

    m_iface   = std::make_unique<PNS_RL_IFACE>();
    m_router  = std::make_unique<PNS::ROUTER>();
    m_settings = std::make_unique<PNS::ROUTING_SETTINGS>( nullptr, std::string() );

    m_board->BuildConnectivity();

    m_iface->SetBoard( m_board );
    m_router->SetInterface( m_iface.get() );
    m_router->ClearWorld();
    m_router->SyncWorld();
    m_router->LoadSettings( m_settings.get() );

    // Determinism: shove/followBranch are bounded by these iteration counts only
    // (no wallclock timeouts), so behaviour is reproducible. Configurable at
    // engine construction.
    m_settings->SetShoveIterationLimit( m_shoveIterLimit );
    m_settings->SetFollowBranchIterLimit( m_followBranchIterLimit );

    // Defaults: walkaround mode, width from board minimum
    m_settings->SetMode( PNS::RM_Walkaround );

    BOARD_DESIGN_SETTINGS& bds = m_board->GetDesignSettings();
    m_router->Sizes().SetTrackWidth( bds.GetCurrentTrackWidth() );

    // m_MinClearance is board-level minimum; it is loaded from .kicad_pro and
    // defaults to 0 when only a .kicad_pcb is available (as in our synthetic
    // dataset).  The *netclass* clearance from the board file is authoritative
    // for routing — use it when present and fall back to m_MinClearance.
    int clearance = bds.m_MinClearance;
    if( bds.m_NetSettings )
    {
        auto defaultNC = bds.m_NetSettings->GetDefaultNetclass();
        if( defaultNC && defaultNC->HasClearance() )
            clearance = std::max( clearance, defaultNC->GetClearance() );
    }
    m_router->Sizes().SetClearance( clearance );

    // Several board-level design-rule fields in BOARD_DESIGN_SETTINGS default
    // to hardcoded non-zero values when no design rules are loaded (see
    // board_design_settings.h DEFAULT_* macros):
    //
    //   CopperEdgeClearance  (default 0.5 mm): copper ↔ Edge.Cuts
    //   HoleClearance        (default 0.25 mm): copper ↔ drilled holes
    //   HoleToHoleMin        (default 0.25 mm): drill ↔ drill
    //
    // On bare .kicad_pcb boards (our synthetic dataset + most of pcb_dataset/),
    // where neither a .kicad_pro nor a legacy setup block supplied these,
    // those defaults are stricter than the netclass allows and block pad/trace
    // placement near the edges or near holes. Align them down to the netclass
    // clearance so the router's keepouts match what the board actually declares.
    //
    // But when a project (.kicad_pro) or legacy setup DID supply these values,
    // they are authoritative: clamping them would silently shrink an intended
    // margin (e.g. a 0.5 mm copper-to-edge keepout down to the copper spacing,
    // leaving traces hugging the board outline). Honor them as-is in that case.
    const bool haveAuthoritativeRules =
            m_projectLoadedFromFile || wasLegacyDesignSettingsLoaded();

    if( !haveAuthoritativeRules )
    {
        if( bds.m_CopperEdgeClearance > clearance )
            bds.m_CopperEdgeClearance = clearance;
        if( bds.m_HoleClearance > clearance )
            bds.m_HoleClearance = clearance;
        if( bds.m_HoleToHoleMin > clearance )
            bds.m_HoleToHoleMin = clearance;
    }

    // The PNS router queries clearance per-pair via m_DRCEngine. When no
    // project file (.kicad_pro) was loaded alongside the board, m_DRCEngine
    // is null and every QueryConstraint() returns false → effective clearance
    // during routing becomes 0. Create and attach a DRC_ENGINE here so the
    // router's rule resolver picks up the netclass clearance we just set.
    if( !bds.m_DRCEngine )
    {
        auto drcEngine = std::make_shared<DRC_ENGINE>( m_board, &bds );
        try
        {
            drcEngine->InitEngine( wxFileName() );  // default rules: netclass-based
            bds.m_DRCEngine = drcEngine;
        }
        catch( ... )
        {
            // Leave m_DRCEngine null on failure; router falls back to 0.
        }
    }
}


void PNS_RL_ROUTER::resyncWorld()
{
    m_router->ClearWorld();
    m_router->SyncWorld();
}


std::map<std::string, long long> PNS_RL_ROUTER::worldStats()
{
    std::map<std::string, long long> s;
    PNS::NODE* world = m_router->GetWorld();

    if( !world )
        return s;

    s["joint_count"] = world->JointCount();
    s["depth"]       = world->Depth();

    // Enumerate every joint: QueryJoints over a box generously covering the board.
    BOX2I bbox = m_board->GetBoundingBox();
    bbox.Inflate( nmFromMm( 100.0 ) );   // fixed margin (max(w,h)+margin could overflow int)

    std::vector<PNS::JOINT*> joints;
    world->QueryJoints( bbox, joints, PNS_LAYER_RANGE::All(), PNS::ITEM::ANY_T );

    long long linksTotal = 0, linksMax = 0, lockedJoints = 0;
    long long segs = 0, vias = 0, vvias = 0, solids = 0, arcs = 0, others = 0;
    std::unordered_set<PNS::ITEM*> uniq;

    for( PNS::JOINT* j : joints )
    {
        long long lc = j->LinkCount();
        linksTotal += lc;
        linksMax = std::max( linksMax, lc );

        if( j->IsLocked() )
            ++lockedJoints;

        for( PNS::ITEM* item : j->LinkList() )
        {
            if( !uniq.insert( item ).second )
                continue;

            switch( item->Kind() )
            {
            case PNS::ITEM::SEGMENT_T: ++segs;   break;
            case PNS::ITEM::VIA_T:     item->IsVirtual() ? ++vvias : ++vias; break;
            case PNS::ITEM::SOLID_T:   ++solids; break;
            case PNS::ITEM::ARC_T:     ++arcs;   break;
            default:                   ++others; break;
            }
        }
    }

    s["joints_queried"] = static_cast<long long>( joints.size() );
    s["links_total"]    = linksTotal;
    s["links_max"]      = linksMax;
    s["joints_locked"]  = lockedJoints;
    s["items_linked"]   = static_cast<long long>( uniq.size() );
    s["segments"]       = segs;
    s["vias"]           = vias;
    s["virtual_vias"]   = vvias;
    s["solids"]         = solids;
    s["arcs"]           = arcs;
    s["others"]         = others;
    return s;
}


void PNS_RL_ROUTER::setShoveIterationLimit( int limit )
{
    m_shoveIterLimit = limit;
    m_settings->SetShoveIterationLimit( limit );
}


int PNS_RL_ROUTER::getShoveIterationLimit() const
{
    return m_settings->ShoveIterationLimit();
}


// ---------------------------------------------------------------------------
// Checkpoint / Restore (in-memory, MCTS tree search) — full-swap baseline
// ---------------------------------------------------------------------------

int64_t PNS_RL_ROUTER::checkpoint()
{
    RLCheckpoint ckpt;

    // A — clone every committed track/via/arc (read-only; no session cancel).
    //     Clone() deep-copies all fields incl. via drill/layer-pair, arc mid,
    //     net pointer, and the UUID.
    ckpt.tracks.reserve( m_board->Tracks().size() );

    for( BOARD_ITEM* item : m_board->Tracks() )
        ckpt.tracks.emplace_back( static_cast<BOARD_ITEM*>( item->Clone() ) );

    // B — engine config: read the actual current values straight from the PNS
    //     objects (m_settings / Sizes / placer), not re-derived from net_id.
    ckpt.config.routingMode = static_cast<int>( m_settings->Mode() );
    ckpt.config.cornerMode  = static_cast<int>( m_settings->GetCornerMode() );
    ckpt.config.trackWidth  = m_router->Sizes().TrackWidth();
    ckpt.config.shoveIterLimit = m_settings->ShoveIterationLimit();
    ckpt.config.viaDiameter = m_router->Sizes().ViaDiameter();
    ckpt.config.viaDrill    = m_router->Sizes().ViaDrill();
    ckpt.config.placingVia  = m_router->IsPlacingVia();

    // C — active routing session. Between actions a session is only open right
    //     after start_route, with the head at the start anchor (forceFinish
    //     grammar), so route_head fully reproduces it. Layer is board-native.
    std::array<double, 3> head = getRouteHead();
    ckpt.session.routing        = m_routing;
    ckpt.session.headX_mm       = head[0];
    ckpt.session.headY_mm       = head[1];
    ckpt.session.headBoardLayer = static_cast<int>( head[2] );
    ckpt.session.netCode        = getCurrentNetCode();

    // D — DRC state (violations + per-track signature snapshot) as of now, so an
    //     incremental DRC after a restore to this checkpoint can retain the
    //     unchanged violations instead of recomputing the full board.
    ckpt.drcViolations = m_drcViolations;
    ckpt.drcItemSig    = m_drcItemSig;

    // E — UUID-generator position. The KIID generator is seeded once at construction
    //     and never rewound, so without this a routing action replayed after a restore
    //     would draw a different UUID stream than a fresh run and the UUID obstacle
    //     tie-break would diverge. Snapshot it here; restore() rewinds to it.
    ckpt.kiidGenState = KIID::GetGeneratorState();

    // Globally-unique handle: epoch (entropy+time, re-seeded on reset) in the high
    // 32 bits + a monotonic per-router sequence in the low 32 bits. The sequence
    // guarantees no collision within this run; the epoch guarantees a handle minted
    // before a reset (or in another engine instance/process) never aliases a live
    // one, so hasCheckpoint()/restore() can reliably reject a stale handle.
    int64_t id = ( static_cast<int64_t>( m_checkpointEpoch ) << 32 )
                 | static_cast<int64_t>( m_nextCheckpointSeq++ );
    m_checkpoints.emplace( id, std::move( ckpt ) );
    return id;
}


bool PNS_RL_ROUTER::restore( int64_t handle )
{
    auto it = m_checkpoints.find( handle );

    if( it == m_checkpoints.end() )
        return false;   // unknown / released / reset handle — board left unchanged

    const RLCheckpoint& ckpt = it->second;

    // 1. Cancel any active session UNCONDITIONALLY. resyncWorld() does not
    //    reset m_state / m_dragger; a live state pointing at the freed world
    //    would crash on the next action. cancelRoute()/cancelDrag() force IDLE.
    if( m_routing )
        cancelRoute();
    if( m_dragging )
        cancelDrag();

    // 1a. Drop the rule resolver's clearance caches. They are keyed on PNS::ITEM*
    //     (CLEARANCE_CACHE_KEY) and are only invalidated by ClearCacheForItems /
    //     ROUTER::StartRouting. This restore is about to free every PNS item in the
    //     world (ClearWorld + SyncWorld below) and allocate replacements, which the
    //     allocator hands back at the SAME addresses — so a surviving entry aliases
    //     a different item and returns its clearance. Only owner-less items are
    //     excluded from caching (see PNS_PCBNEW_RULE_RESOLVER::Clearance); board
    //     items are cached and would alias. StartRouting clears it for us, but only
    //     when the checkpoint had a session open — an idle checkpoint would leave
    //     the stale entries in place, so clear unconditionally, here.
    m_router->GetRuleResolver()->ClearCaches();
    m_router->GetRuleResolver()->ClearTemporaryCaches();

    // 2. Collect-then-remove all current tracks. Tracks() is a std::deque and
    //    Remove() invalidates iterators, so snapshot the pointers first. Every
    //    item must be removed BEFORE any clone is re-added: clones preserve the
    //    original UUIDs and BOARD::Add does not reject a duplicate KIID, so a
    //    live same-UUID item would corrupt the board's id cache.
    std::vector<BOARD_ITEM*> current;
    current.reserve( m_board->Tracks().size() );

    for( BOARD_ITEM* item : m_board->Tracks() )
        current.push_back( item );

    for( BOARD_ITEM* item : current )
    {
        m_board->Remove( item );
        delete item;
    }

    // 3. Re-clone the stored items and append. The checkpoint keeps ownership
    //    of its own clones (re-Clone) so the same handle can be restored again.
    for( const std::unique_ptr<BOARD_ITEM>& stored : ckpt.tracks )
        m_board->Add( static_cast<BOARD_ITEM*>( stored->Clone() ), ADD_MODE::APPEND );

    // 4. Engine config (board → config → session order). Sizes are set in
    //    nm directly to avoid the mm round-trip of the public setters.
    setRoutingMode( ckpt.config.routingMode );
    setCornerMode( ckpt.config.cornerMode );
    setShoveIterationLimit( ckpt.config.shoveIterLimit );
    m_router->Sizes().SetTrackWidth( ckpt.config.trackWidth );
    m_router->Sizes().SetViaDiameter( ckpt.config.viaDiameter );
    m_router->Sizes().SetViaDrill( ckpt.config.viaDrill );

    // 5. Rebuild PNS world + connectivity from the restored board, and restore the
    //    checkpoint's DRC state (violations + signature snapshot) as a consistent
    //    pair so a following run_drc_incremental() can retain unchanged violations.
    resyncWorld();
    // Same canonicalisation as restoreIncremental() step 5c. This path rebuilds the
    // world from ckpt.tracks so its order is already checkpoint-derived rather than
    // start-board-derived, but SyncWorld's own emission order is not content-sorted
    // — normalise here too so both restore paths hand routing an identically shaped
    // world (they are supposed to be interchangeable, and today are not: 355 tracks
    // vs 359 on a mavbridge rollout).
    m_router->GetWorld()->CanonicalizeOrder();
    buildConnectivity();
    m_drcViolations = ckpt.drcViolations;
    m_drcItemSig    = ckpt.drcItemSig;

    // 6. Re-open the routing session if one was active at checkpoint time.
    //    startRoute re-derives the net from the anchor at the head position.
    if( ckpt.session.routing )
    {
        startRoute( ckpt.session.headX_mm, ckpt.session.headY_mm,
                    ckpt.session.headBoardLayer );

        if( ckpt.config.placingVia && !m_router->IsPlacingVia() )
            toggleVia();
    }

    // 7. Rewind the UUID generator to the checkpoint's position (done last, so any
    //    KIID drawn during the rebuild above does not leak into the post-restore
    //    stream). New items routed after this restore now get the same UUIDs a fresh
    //    run would — the UUID obstacle tie-break is reproducible.
    KIID::SetGeneratorState( ckpt.kiidGenState );

    return true;
}


void PNS_RL_ROUTER::releaseCheckpoint( int64_t handle )
{
    m_checkpoints.erase( handle );   // idempotent: no-op if already absent
}


void PNS_RL_ROUTER::reseedCheckpointEpoch()
{
    // Entropy + monotonic clock → a fresh 32-bit epoch. Mixed so that neither a
    // weak random_device nor a coarse clock alone can repeat the previous epoch.
    std::random_device rd;
    uint64_t t = static_cast<uint64_t>(
            std::chrono::steady_clock::now().time_since_epoch().count() );
    m_checkpointEpoch = static_cast<uint32_t>( rd() ) ^ static_cast<uint32_t>( t )
                        ^ static_cast<uint32_t>( t >> 32 );
}


void PNS_RL_ROUTER::resetCheckpoints()
{
    // Drop every checkpoint at once (frees all cloned tracks + DRC state) and mint a
    // new epoch, so every handle issued before this reset becomes permanently invalid
    // (it can never alias a post-reset handle). The sequence counter keeps counting.
    m_checkpoints.clear();
    reseedCheckpointEpoch();
}


void PNS_RL_ROUTER::rewindKIIDToEpisodeStart()
{
    // Return the process-global UUID generator to the construction-time position so the
    // NEXT episode's routing mints the same UUID stream as every other episode. Meant to
    // run as the LAST engine mutation in env.reset(), mirroring restore()'s ordering
    // (step 7): any KIID drawn by this reset's connectivity/DRC rebuild is discarded
    // here, and the routed tracks that consumed the stream last episode were already
    // deleted, so no live item aliases the re-issued UUIDs. Empty = entropy-seeded, no-op.
    if( !m_episodeStartKiidState.empty() )
        KIID::SetGeneratorState( m_episodeStartKiidState );
}


// Field-level equality for two board items sharing a UUID. PNS shove can modify
// an existing track in place (same UUID, new geometry), so the incremental diff
// compares fields, not just UUIDs.
static bool boardItemsEqual( const BOARD_ITEM* a, const BOARD_ITEM* b )
{
    if( a->Type() != b->Type() )
        return false;

    const PCB_TRACK* ta = static_cast<const PCB_TRACK*>( a );
    const PCB_TRACK* tb = static_cast<const PCB_TRACK*>( b );

    if( ta->GetStart() != tb->GetStart() || ta->GetEnd() != tb->GetEnd() )
        return false;
    if( ta->GetWidth() != tb->GetWidth() || ta->GetNetCode() != tb->GetNetCode() )
        return false;

    if( a->Type() == PCB_VIA_T )
    {
        const PCB_VIA* va = static_cast<const PCB_VIA*>( a );
        const PCB_VIA* vb = static_cast<const PCB_VIA*>( b );
        return va->GetViaType() == vb->GetViaType()
            && va->TopLayer() == vb->TopLayer()
            && va->BottomLayer() == vb->BottomLayer()
            && va->GetDrillValue() == vb->GetDrillValue();
    }

    if( ta->GetLayer() != tb->GetLayer() )
        return false;

    if( a->Type() == PCB_ARC_T )
        return static_cast<const PCB_ARC*>( a )->GetMid()
            == static_cast<const PCB_ARC*>( b )->GetMid();

    return true;
}


bool PNS_RL_ROUTER::restoreIncremental( int64_t handle )
{
    auto it = m_checkpoints.find( handle );

    if( it == m_checkpoints.end() )
        return false;   // unknown / released / reset handle — board left unchanged

    const RLCheckpoint& ckpt = it->second;

    // 1. Cancel any active session (same as restore() step 1).
    if( m_routing )
        cancelRoute();
    if( m_dragging )
        cancelDrag();

    // 1a. Same pointer-keyed clearance-cache invalidation as restore() step 1a:
    //     the diff below removes PNS items and ReleaseGarbage() frees them, so the
    //     replacements land on recycled addresses and any surviving cache entry
    //     aliases them.
    m_router->GetRuleResolver()->ClearCaches();
    m_router->GetRuleResolver()->ClearTemporaryCaches();

    PNS::NODE* world = m_router->GetWorld();

    // 2. Index live board tracks + checkpoint clones by UUID.
    std::unordered_map<KIID, PCB_TRACK*> live;
    for( PCB_TRACK* t : m_board->Tracks() )
        live[t->m_Uuid] = t;

    std::unordered_map<KIID, BOARD_ITEM*> target;
    for( const std::unique_ptr<BOARD_ITEM>& c : ckpt.tracks )
        target[c->m_Uuid] = c.get();

    // 3. Diff (computed BEFORE any deletion to avoid dangling-pointer reads):
    //    unchanged = same UUID + equal fields (left in place, world item kept);
    //    everything else live is removed, everything else target is added.
    std::unordered_set<KIID> unchanged;
    std::vector<PCB_TRACK*>  toRemove;

    for( auto& [uuid, t] : live )
    {
        auto tg = target.find( uuid );
        if( tg != target.end() && boardItemsEqual( t, tg->second ) )
            unchanged.insert( uuid );
        else
            toRemove.push_back( t );
    }

    // 4. Remove from the PNS world FIRST (FindItemByParent needs the live track
    //    pointer), then from the board. DEFER the C++ delete: BOARD::Remove only
    //    MARKS the track's connectivity CN_ITEM invalid (CN_CONNECTIVITY_ALGO::
    //    Remove); that CN_ITEM, still holding a pointer to the track, is purged
    //    later by recalculateRatsnest()'s searchConnections(). Deleting the track
    //    now would leave that CN_ITEM with a dangling parent → SIGBUS in the
    //    cluster/ratsnest walk on dense boards. Keep the tracks alive until after
    //    step 7 (mirrors KiCad's BOARD_COMMIT, which never frees an item before
    //    the connectivity has settled).
    std::vector<PCB_TRACK*> toFree;
    toFree.reserve( toRemove.size() );

    for( PCB_TRACK* t : toRemove )
    {
        if( PNS::ITEM* w = world->FindItemByParent( t ) )
            world->Remove( w );
        m_board->Remove( t );
        toFree.push_back( t );
    }

    // 5. Add the checkpoint items that are not already present unchanged
    //    (re-clone for re-use), mirroring each into the PNS world incrementally.
    for( auto& [uuid, c] : target )
    {
        if( unchanged.count( uuid ) )
            continue;
        BOARD_ITEM* clone = static_cast<BOARD_ITEM*>( c->Clone() );
        m_board->Add( clone, ADD_MODE::APPEND );
        m_iface->addBoardItemToWorld( world, clone );
    }

    // 5a. Regenerate virtual vias for the restored joint topology. VVIAs are a
    //     derived projection of the joints (width-change / T / locked), NOT board
    //     items, so the track diff above never touches them: a VVIA minted for the
    //     pre-restore state lingers as a phantom obstacle and deflects the first
    //     post-restore route (full restore()'s ClearWorld+SyncWorld gets a fresh
    //     set for free; the incremental path must reproduce it). FixupVirtualVias
    //     is idempotent (drops existing VVIAs before re-deriving), so this yields
    //     the same VVIA set a full SyncWorld would. Run before ReleaseGarbage so
    //     the dropped stale VVIAs are drained together with the track removals.
    world->FixupVirtualVias();

    // 5b. Drain the world's garbage: step 4's world->Remove() parks the removed
    //     PNS items in NODE::m_garbageItems (released only by NODE::Commit(),
    //     which this resync-less path never calls), so drain explicitly here —
    //     mirroring the releaseGarbage() at the tail of NODE::Commit(). The PNS
    //     items' board parents are still alive (freed in step 7b), so this is a
    //     clean drain with no dangling reference.
    world->ReleaseGarbage();

    // 5c. Canonicalise the world's insertion-order state. Steps 3-5 only touch what
    //     the diff reported as changed, so the R-tree shape and the joint link order
    //     are left describing the board this restore STARTED from, not the one it
    //     restored TO — and first-hit collision queries plus followLine/AssembleLine
    //     traversal both read that order. Without this, restoring a checkpoint
    //     directly and restoring it via another board route the next action
    //     differently (repro: sandbox/shove_investigation/session_restore_repro.py
    //     --via, at 222 tracks on 0344_mavbridge). Re-derives both from item
    //     content, which is far cheaper than a full ClearWorld+SyncWorld.
    world->CanonicalizeOrder();

    // 6. Engine config (board → config → session order).
    setRoutingMode( ckpt.config.routingMode );
    setCornerMode( ckpt.config.cornerMode );
    setShoveIterationLimit( ckpt.config.shoveIterLimit );
    m_router->Sizes().SetTrackWidth( ckpt.config.trackWidth );
    m_router->Sizes().SetViaDiameter( ckpt.config.viaDiameter );
    m_router->Sizes().SetViaDrill( ckpt.config.viaDrill );

    // 7. BOARD::Add/Remove already maintained the connectivity graph
    //    incrementally (board.cpp m_connectivity->Add/Remove), so recompute only
    //    the ratsnest — no full BuildConnectivity rebuild. This pass also purges
    //    the CN_ITEMs invalidated in step 4 (searchConnections() ->
    //    RemoveInvalidItems), which is why the track deletes were deferred.
    recalculateRatsnest();

    // Restore the checkpoint's DRC state as a consistent pair (instead of clearing),
    // so a following run_drc_incremental() diffs from this checkpoint and retains the
    // unchanged violations rather than recomputing the whole board.
    m_drcViolations = ckpt.drcViolations;
    m_drcItemSig    = ckpt.drcItemSig;

    // 7b. recalculateRatsnest() has purged the invalidated connectivity items, so
    //     nothing references the removed tracks any more — free them now (deferred
    //     from step 4 to avoid a dangling CN_ITEM parent during the ratsnest walk).
    for( PCB_TRACK* t : toFree )
        delete t;

    // 8. Re-open the routing session if one was active at checkpoint time.
    if( ckpt.session.routing )
    {
        startRoute( ckpt.session.headX_mm, ckpt.session.headY_mm,
                    ckpt.session.headBoardLayer );

        if( ckpt.config.placingVia && !m_router->IsPlacingVia() )
            toggleVia();
    }

    // 9. Rewind the UUID generator to the checkpoint's position (see restore() step 7):
    //    post-restore routing then mints the same UUID stream a fresh run would.
    KIID::SetGeneratorState( ckpt.kiidGenState );

    return true;
}


// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

void PNS_RL_ROUTER::setRoutingMode( int mode )
{
    m_settings->SetMode( static_cast<PNS::PNS_MODE>( mode ) );
}


void PNS_RL_ROUTER::setCornerMode( int mode )
{
    m_settings->SetCornerMode( static_cast<DIRECTION_45::CORNER_MODE>( mode ) );
}


void PNS_RL_ROUTER::setTrackWidth( double width_mm )
{
    int width_nm = nmFromMm( width_mm );

    if( width_nm > 0 )
    {
        m_router->Sizes().SetTrackWidth( width_nm );
        // Stock ROUTER::UpdateSizes also forwards to the active placer, so a
        // mid-session change takes effect this session.
        m_router->UpdateSizes( m_router->Sizes() );
    }
}


// ---------------------------------------------------------------------------
// Board manipulation
// ---------------------------------------------------------------------------

bool PNS_RL_ROUTER::deleteTrackNear( double x1_mm, double y1_mm,
                                     double x2_mm, double y2_mm,
                                     int layer, int net_code,
                                     double tol_mm )
{
    const int tol = nmFromMm( tol_mm );
    VECTOR2I  a( nmFromMm( x1_mm ), nmFromMm( y1_mm ) );
    VECTOR2I  b( nmFromMm( x2_mm ), nmFromMm( y2_mm ) );

    for( BOARD_ITEM* item : m_board->Tracks() )
    {
        PCB_TRACK* trk = dynamic_cast<PCB_TRACK*>( item );

        if( !trk || trk->GetClass() == "PCB_VIA" )
            continue;

        if( static_cast<int>( trk->GetLayer() ) != layer
                || trk->GetNetCode() != net_code )
            continue;

        VECTOR2I s = trk->GetStart();
        VECTOR2I e = trk->GetEnd();

        bool matchFwd = ( ( s - a ).EuclideanNorm() < tol && ( e - b ).EuclideanNorm() < tol );
        bool matchRev = ( ( s - b ).EuclideanNorm() < tol && ( e - a ).EuclideanNorm() < tol );

        if( matchFwd || matchRev )
        {
            m_board->Remove( trk );
            resyncWorld();
            // Deferred free (same invariant as restoreIncremental step 4/7b):
            // BOARD::Remove only MARKS the CN_ITEM invalid; recalculateRatsnest's
            // searchConnections() purges it — free the track only after that,
            // or a ratsnest walk dereferences the dangling parent (SIGBUS).
            recalculateRatsnest();
            delete trk;
            return true;
        }
    }

    return false;
}


bool PNS_RL_ROUTER::deleteTrackByIndex( int index )
{
    int idx = 0;

    for( BOARD_ITEM* item : m_board->Tracks() )
    {
        PCB_TRACK* trk = dynamic_cast<PCB_TRACK*>( item );

        if( !trk || trk->GetClass() == "PCB_VIA" )
            continue;

        if( idx == index )
        {
            m_board->Remove( trk );
            resyncWorld();
            // Deferred free (same invariant as restoreIncremental step 4/7b):
            // BOARD::Remove only MARKS the CN_ITEM invalid; recalculateRatsnest's
            // searchConnections() purges it — free the track only after that,
            // or a ratsnest walk dereferences the dangling parent (SIGBUS).
            recalculateRatsnest();
            delete trk;
            return true;
        }

        ++idx;
    }

    return false;
}


bool PNS_RL_ROUTER::deleteViaNear( double x_mm, double y_mm, int net_code,
                                   double tol_mm )
{
    const int tol = nmFromMm( tol_mm );
    VECTOR2I  p( nmFromMm( x_mm ), nmFromMm( y_mm ) );

    for( BOARD_ITEM* item : m_board->Tracks() )
    {
        PCB_VIA* via = dynamic_cast<PCB_VIA*>( item );

        if( !via || via->GetNetCode() != net_code )
            continue;

        if( ( via->GetStart() - p ).EuclideanNorm() < tol )
        {
            m_board->Remove( via );
            resyncWorld();
            recalculateRatsnest();   // deferred free — see deleteTrackNear
            delete via;
            return true;
        }
    }

    return false;
}


bool PNS_RL_ROUTER::deleteViaByIndex( int index )
{
    int idx = 0;

    for( BOARD_ITEM* item : m_board->Tracks() )
    {
        PCB_VIA* via = dynamic_cast<PCB_VIA*>( item );

        if( !via )
            continue;

        if( idx == index )
        {
            m_board->Remove( via );
            resyncWorld();
            recalculateRatsnest();   // deferred free — see deleteTrackNear
            delete via;
            return true;
        }

        ++idx;
    }

    return false;
}


int PNS_RL_ROUTER::getViaCount() const
{
    int count = 0;

    for( BOARD_ITEM* item : m_board->Tracks() )
    {
        if( dynamic_cast<PCB_VIA*>( item ) )
            ++count;
    }

    return count;
}


int PNS_RL_ROUTER::lockNet( int netCode, bool locked )
{
    // Set the BOARD lock flag on every track/via/arc of this net. m_board->
    // Tracks() spans PCB_TRACK/PCB_VIA/PCB_ARC (all BOARD_CONNECTED_ITEM), so
    // GetNetCode() is available on each. No geometry changes — only the lock
    // marker — so no connectivity/ratsnest churn; a single resyncWorld() below
    // re-marks the PNS world (SEGMENT/VIA/ARC → MK_LOCKED via syncTrack et al.).
    int changed = 0;

    for( BOARD_ITEM* item : m_board->Tracks() )
    {
        BOARD_CONNECTED_ITEM* bci = dynamic_cast<BOARD_CONNECTED_ITEM*>( item );

        if( !bci || bci->GetNetCode() != netCode )
            continue;

        item->SetLocked( locked );
        ++changed;
    }

    // Rebuild the PNS world so the new lock markers take effect immediately for
    // the next shove. resyncWorld() cancels any active session (locking is a
    // between-routes operation), matching the other board-mutation methods.
    if( changed )
        resyncWorld();

    return changed;
}


int PNS_RL_ROUTER::deleteRoutingOfNets( const std::vector<int>& netCodes )
{
    if( netCodes.empty() )
        return 0;

    const std::unordered_set<int> targets( netCodes.begin(), netCodes.end() );

    // Snapshot first: BOARD::Remove invalidates the Tracks() iterator, so collect
    // the matching items before mutating. Deletion is keyed purely on the net
    // (the re-route set); lock is an orthogonal shove property handled elsewhere
    // — the caller keeps a net out of the strip set to preserve it.
    std::vector<PCB_TRACK*> toRemove;

    for( BOARD_ITEM* item : m_board->Tracks() )
    {
        BOARD_CONNECTED_ITEM* bci = dynamic_cast<BOARD_CONNECTED_ITEM*>( item );

        if( bci && targets.count( bci->GetNetCode() ) )
            toRemove.push_back( static_cast<PCB_TRACK*>( item ) );
    }

    if( toRemove.empty() )
        return 0;

    // Deferred free (same invariant as deleteTrackByIndex / restoreIncremental):
    // BOARD::Remove only MARKS each CN_ITEM invalid; the item is dereferenced by
    // recalculateRatsnest()'s searchConnections() purge, so free only after that —
    // otherwise a ratsnest walk hits a dangling parent (SIGBUS on dense boards).
    for( PCB_TRACK* t : toRemove )
        m_board->Remove( t );

    resyncWorld();          // ClearWorld + SyncWorld rebuilds the world from the
                            // now-reduced board (no per-item world removal needed).
    recalculateRatsnest();  // purges the invalidated CN_ITEMs

    for( PCB_TRACK* t : toRemove )
        delete t;

    return static_cast<int>( toRemove.size() );
}


// ---------------------------------------------------------------------------
// Track cleaner
// ---------------------------------------------------------------------------

// The only translation points between the binding-facing structs in pns_rl_router.h
// and the cleaner's own (see the note on RLCleanupSpec).
static RL_CLEANUP_SPEC toCleanupSpec( const RLCleanupSpec& aSpec )
{
    RL_CLEANUP_SPEC out;
    out.dry_run         = aSpec.dry_run;
    out.merge_segments  = aSpec.merge_segments;
    out.clean_vias      = aSpec.clean_vias;
    out.remove_shorts   = aSpec.remove_shorts;
    out.tracks_in_pads  = aSpec.tracks_in_pads;
    out.dangling_tracks = aSpec.dangling_tracks;
    out.dangling_vias   = aSpec.dangling_vias;
    out.net_codes       = aSpec.net_codes;
    return out;
}


static RLCleanupItem fromCleanupItem( const RL_CLEANUP_ITEM& aItem )
{
    RLCleanupItem out;
    out.code      = aItem.code;
    out.code_name = RLCleanupCodeName( aItem.code );
    out.item_a    = aItem.item_a;
    out.item_b    = aItem.item_b;
    return out;
}


RLCleanupResult PNS_RL_ROUTER::cleanupTracks( const RLCleanupSpec& spec )
{
    RLCleanupResult res;

    // Quiescent-only. Cancelling the caller's session here would mutate state the
    // caller still believes it owns (an MCTS simulation cannot see a silent cancel),
    // so refuse instead and let the caller decide.
    if( m_routing )
    {
        res.reject_reason = "routing_session_active";
        return res;
    }

    if( m_dragging )
    {
        res.reject_reason = "drag_session_active";
        return res;
    }

    // Pointer-keyed clearance caches would alias recycled addresses once the detached
    // items are freed below — same invalidation as restoreIncremental() step 1a.
    if( !spec.dry_run )
    {
        m_router->GetRuleResolver()->ClearCaches();
        m_router->GetRuleResolver()->ClearTemporaryCaches();
    }

    std::vector<RL_CLEANUP_ITEM> items;
    std::vector<PCB_TRACK*>      detached;
    std::vector<KIID>            modified;

    RL_TRACKS_CLEANER cleaner( m_board );
    cleaner.CleanupBoard( toCleanupSpec( spec ), items, detached, modified );

    if( !detached.empty() || !modified.empty() )
    {
        // Deferred free, exactly as in deleteRoutingOfNets(): BOARD::Remove only MARKS
        // each CN_ITEM invalid, so the items stay alive until the world rebuild and the
        // ratsnest purge have dropped every reference to them.
        resyncWorld();
        recalculateRatsnest();

        for( PCB_TRACK* t : detached )
        {
            res.removed.push_back( t->m_Uuid );
            delete t;
        }
    }

    res.items.reserve( items.size() );

    for( const RL_CLEANUP_ITEM& item : items )
        res.items.push_back( fromCleanupItem( item ) );

    // A survivor merged more than once appears once per merge — report it once.
    std::unordered_set<KIID> seen;

    for( const KIID& uuid : modified )
    {
        if( seen.insert( uuid ).second )
            res.modified.push_back( uuid );
    }

    res.ran = true;
    return res;
}


// ---------------------------------------------------------------------------
// Find nearest PNS item at a given board position
// ---------------------------------------------------------------------------

int PNS_RL_ROUTER::boardToPnsLayer( int boardLayer ) const
{
    return m_iface->GetPNSLayerFromBoardLayer(
        static_cast<PCB_LAYER_ID>( boardLayer ) );
}


int PNS_RL_ROUTER::pnsToBoardLayer( int pnsLayer ) const
{
    return static_cast<int>(
        m_iface->GetBoardLayerFromPNSLayer( pnsLayer ) );
}


PNS::ITEM* PNS_RL_ROUTER::itemAt( const VECTOR2I& pos, int boardLayer )
{
    PNS::NODE* world = m_router->GetWorld();

    if( !world )
        return nullptr;

    int pnsLayer = boardToPnsLayer( boardLayer );

    PNS::ITEM_SET hits = world->HitTest( pos );

    // Determinism: HitTest returns an unordered_set<ITEM*> whose iteration is
    // heap-address-ordered, so "first overlapping hit" would vary across runs
    // (ASLR / heap layout) and flip the chosen start item. Pick the
    // UUID-smallest overlapping item instead.
    PNS::ITEM* best = nullptr;

    for( PNS::ITEM* item : hits.Items() )
    {
        if( item->Layers().Overlaps( pnsLayer ) )
        {
            if( !best || PNS::canonicalItemCompare( item, best ) < 0 )
                best = item;
        }
    }

    return best;
}


// ---------------------------------------------------------------------------
// Routing API
// ---------------------------------------------------------------------------

bool PNS_RL_ROUTER::startRoute( double x_mm, double y_mm, int boardLayer )
{
    if( m_routing )
        cancelRoute();

    VECTOR2I  pos( nmFromMm( x_mm ), nmFromMm( y_mm ) );
    PNS::ITEM* item = itemAt( pos, boardLayer );

    int pnsLayer = boardToPnsLayer( boardLayer );

    m_router->SetMode( PNS::PNS_MODE_ROUTE_SINGLE );
    bool ok = m_router->StartRouting( pos, item, pnsLayer );
    m_routing = ok;
    return ok;
}


bool PNS_RL_ROUTER::move( double x_mm, double y_mm )
{
    if( !m_routing && !m_dragging )
        return false;

    VECTOR2I pos( nmFromMm( x_mm ), nmFromMm( y_mm ) );
    return m_router->Move( pos, nullptr );
}


bool PNS_RL_ROUTER::fixRoute( double x_mm, double y_mm, bool forceFinish,
                             bool rejectIfStuck, int expectedLayer,
                             double arriveTolMm, bool requireVia )
{
    if( !m_routing )
        return false;

    VECTOR2I pos( nmFromMm( x_mm ), nmFromMm( y_mm ) );

    m_router->Move( pos, nullptr );

    // Move() routes the head toward pos and computes reachesEnd = (head == pos)
    // internally, but LINE_PLACER::Move discards it (always returns true). The
    // reached point survives in Placer()->CurrentEnd(), so an exact compare
    // recovers the engine's own reached/stuck verdict. When rejectIfStuck is
    // set and the head did not reach pos, the walkaround got stuck at a wall or
    // existing copper: abort without committing so no partial dangling stub is
    // drawn (Move leaves the board untouched — only CommitRouting writes to it).
    // This mirrors ROUTER::Finish, which likewise commits only on exact arrival.
    // ``arriveTolMm`` > 0 relaxes the exact compare to a radius. Exact match is
    // stricter than PNS's own notion of arrival: LINE_PLACER accepts a point
    // within head width / 2, on the reasoning that the placed item's copper then
    // covers the target. The caller supplies the radius (make_via passes the via
    // radius) so the tolerance is derived from the net class, not a magic number.
    // 0 keeps the original exact-match behaviour.
    if( rejectIfStuck )
    {
        auto* placer = m_router->Placer();

        if( placer )
        {
            const VECTOR2I end = placer->CurrentEnd();
            const bool arrived = ( arriveTolMm > 0.0 )
                ? ( ( end - pos ).EuclideanNorm() <= nmFromMm( arriveTolMm ) )
                : ( end == pos );

            if( !arrived )
            {
                cancelRoute();   // StopRouting() + m_routing = false
                return false;
            }
        }
    }

    // Caller-declared layer intent: reaching (x, y) on the WRONG
    // copper layer is not arrival. A non-via action (make_line) must end on
    // the layer it was issued on — a mismatch means an unexpected layer
    // switch, e.g. stray via-placement state leaking from an earlier failed
    // make_via. Abort loudly instead of committing an unintended via.
    // -1 = no check (make_via legitimately changes layer via
    // ToggleViaPlacement, and its exit layer is PNS's layer-pair choice).
    if( expectedLayer >= 0 )
    {
        auto* placer = m_router->Placer();

        if( placer && pnsToBoardLayer( placer->CurrentLayer() ) != expectedLayer )
        {
            cancelRoute();
            return false;
        }
    }

    // Gated on the CALLER's intent (``requireVia``), not on
    // ``m_router->IsPlacingVia()``: via mode is reset only at episode reset
    // (PCBWorld.reset -> reset_via_mode) and cancelRoute does NOT clear it, so a
    // failed make_via leaves it on — the leak make_line already guards against
    // with expectedLayer. Reading the router's mode here would make a later
    // make_line fail spuriously whenever that leak is live.
    //
    // Via intent is ALL-OR-NOTHING. LINE_PLACER attaches the pending via to the
    // head only when its own viaOk test passes (rhWalkOnly / rhShoveOnly:
    // `if( m_placingVia && viaOk )`); when that test fails the head simply
    // carries no via and FixRoute commits the BARE ROUTE anyway, returning true.
    // The caller then sees via_count unchanged, reports failure — and the copper
    // stays on the board. Measured on d3b (every make_via candidate at every
    // routing state, 2 rollouts): routed-without-via was 3.5% of evaluations on
    // 0103_Lizard and 30.6% on 0232_ATtiny461, and 73-99% of those had actually
    // changed the board (up to 9 tracks / 23 mm of wire); some had even closed a
    // ratsnest edge (ΔΦ ≈ +1.1) while being labelled valid_dispatch_fail and
    // popped by the search.
    //
    // Move() has written nothing yet — only CommitRouting() below touches the
    // board — so the honest response is to refuse here, exactly like the
    // reject_if_stuck and expectedLayer guards above. This makes the
    // all-or-nothing contract that make_via's docstring claims actually hold,
    // and leaves the caller's post-hoc via_count check as a pure safety net.
    if( requireVia )
    {
        auto* placer = m_router->Placer();
        const PNS::LINE* line = nullptr;

        if( placer )
        {
            const PNS::ITEM_SET traces = placer->Traces();
            line = traces.Size()
                   ? dynamic_cast<const PNS::LINE*>( traces[0] ) : nullptr;
        }

        if( !line || !line->EndsWithVia() )
        {
            cancelRoute();
            return false;
        }
    }

    bool ok = m_router->FixRoute( pos, nullptr, forceFinish,
                                  /*aForceCommit=*/false );

    if( ok && forceFinish )
    {
        m_router->CommitRouting();
        m_routing = false;
        // Re-sync the PNS world from the committed board.
        resyncWorld();
    }

    return ok;
}


void PNS_RL_ROUTER::cancelRoute()
{
    if( m_routing )
    {
        m_router->StopRouting();
        m_routing = false;
    }
}


// ---------------------------------------------------------------------------
// Drag API
// ---------------------------------------------------------------------------

bool PNS_RL_ROUTER::startDrag( double x_mm, double y_mm, int boardLayer, int dragMode )
{
    if( m_routing )
        cancelRoute();

    if( m_dragging )
        cancelDrag();

    VECTOR2I pos( nmFromMm( x_mm ), nmFromMm( y_mm ) );
    PNS::ITEM* item = itemAt( pos, boardLayer );

    if( !item )
        return false;

    bool ok = m_router->StartDragging( pos, item, dragMode );
    m_dragging = ok;
    return ok;
}


bool PNS_RL_ROUTER::fixDrag( bool forceCommit )
{
    if( !m_dragging )
        return false;

    bool ok = m_router->FixRoute( VECTOR2I(), nullptr, false, forceCommit );

    if( ok )
    {
        m_router->CommitRouting();
        m_dragging = false;
        // Re-sync the PNS world from the committed board.
        resyncWorld();
    }

    return ok;
}


void PNS_RL_ROUTER::cancelDrag()
{
    if( m_dragging )
    {
        m_router->StopRouting();
        m_dragging = false;
    }
}


// ---------------------------------------------------------------------------
// DRC API
// ---------------------------------------------------------------------------

// Clearance-family DRC codes — produced by the copper-clearance provider, the
// pairwise-local part we scope incrementally. Everything else (connectivity,
// per-item) is recomputed in full each pass.
static bool isClearanceFamily( int aCode )
{
    return aCode == DRCE_CLEARANCE || aCode == DRCE_HOLE_CLEARANCE
        || aCode == DRCE_SHORTING_ITEMS || aCode == DRCE_TRACKS_CROSSING
        || aCode == DRCE_ZONES_INTERSECT;
}


// Geometry+net signature of a track/via/arc, used to diff the board between DRC
// passes (added / removed / shove-modified) for the incremental clearance scope.
static std::string drcSigOf( PCB_TRACK* t )
{
    std::string s = std::to_string( t->Type() ) + ":"
        + std::to_string( t->GetStart().x ) + "," + std::to_string( t->GetStart().y ) + ":"
        + std::to_string( t->GetEnd().x )   + "," + std::to_string( t->GetEnd().y )   + ":"
        + std::to_string( t->GetWidth() )   + ":"
        + std::to_string( static_cast<int>( t->GetLayer() ) ) + ":"
        + std::to_string( t->GetNetCode() );

    if( t->Type() == PCB_VIA_T )
    {
        PCB_VIA* v = static_cast<PCB_VIA*>( t );
        s += ":" + std::to_string( v->GetDrillValue() )
           + ":" + std::to_string( static_cast<int>( v->TopLayer() ) )
           + ":" + std::to_string( static_cast<int>( v->BottomLayer() ) );
    }
    else if( t->Type() == PCB_ARC_T )
    {
        PCB_ARC* a = static_cast<PCB_ARC*>( t );
        s += ":" + std::to_string( a->GetMid().x ) + "," + std::to_string( a->GetMid().y );
    }

    return s;
}


void PNS_RL_ROUTER::snapshotDrcItems()
{
    m_drcItemSig.clear();
    for( PCB_TRACK* t : m_board->Tracks() )
        m_drcItemSig[t->m_Uuid] = drcSigOf( t );
}


// Run the DRC engine into m_drcViolations. If clearanceScope is non-null, the
// copper-clearance provider tests only those items (each vs the full RTree) — the
// rest of the providers always run in full.
void PNS_RL_ROUTER::runDRCEngine( const std::string& rules_path,
                                  const std::vector<BOARD_ITEM*>* clearanceScope )
{
    m_drcViolations.clear();

    // Connectivity: the full DRC (clearanceScope == nullptr — the ground-truth path)
    // rebuilds the whole graph from scratch. The incremental path relies on the graph
    // already being maintained by the board mutations that preceded it (BOARD::Add/
    // Remove keep m_connectivity in sync, the same invariant restoreIncremental uses),
    // so it only needs to recompute the ratsnest.
    if( clearanceScope )
        recalculateRatsnest();
    else
        m_board->BuildConnectivity();

    BOARD_DESIGN_SETTINGS& bds = m_board->GetDesignSettings();
    auto engine = std::make_shared<DRC_ENGINE>( m_board, &bds );

    try
    {
        engine->InitEngine( wxFileName( wxString::FromUTF8( rules_path ) ) );
    }
    catch( ... )
    {
        // Best effort: board default rules only
    }

    engine->SetProgressReporter( nullptr );

    // Hand the incremental-clearance scope to the RL copper-clearance provider via the
    // RL-only thread_local (drc/drc_rl_scope.h), NOT through a DRC_ENGINE member — this
    // keeps the stock KiCad DRC engine untouched. Cleared again right after RunTests()
    // so the pointer (into the caller's local vector) never dangles past this call.
    DRC_RL::SetClearanceScope( clearanceScope );

    engine->SetViolationHandler(
        [&]( const std::shared_ptr<DRC_ITEM>& aItem, const VECTOR2I& aPos,
             int aLayer, DRC_CUSTOM_MARKER_HANDLER* )
        {
            RLDRCViolation v;
            v.error_code = aItem->GetErrorCode();
            v.message    = aItem->GetErrorMessage().ToStdString();
            v.error_type = aItem->GetErrorText().ToStdString();
            v.x_mm       = mmFromNm( aPos.x );
            v.y_mm       = mmFromNm( aPos.y );
            v.layer      = aLayer;
            v.severity   = static_cast<int>( bds.GetSeverity( aItem->GetErrorCode() ) );

            // Extract net names from the main item and aux item (if any)
            auto extractNet = [&]( const KIID& id ) -> std::string
            {
                if( id == niluuid )
                    return {};
                EDA_ITEM* ei = m_board->GetItem( id );
                if( !ei )
                    return {};
                BOARD_CONNECTED_ITEM* bci = dynamic_cast<BOARD_CONNECTED_ITEM*>( ei );
                if( !bci || !bci->GetNet() )
                    return {};
                return bci->GetNetname().ToStdString();
            };
            std::string n1 = extractNet( aItem->GetMainItemID() );
            std::string n2 = extractNet( aItem->GetAuxItemID() );
            if( !n1.empty() )
                v.net_names.push_back( n1 );
            if( !n2.empty() && n2 != n1 )
                v.net_names.push_back( n2 );

            // Stable UUID keys (values, not pointers) for incremental DRC
            // invalidation: compare against the removed/changed track set without
            // dereferencing anything. Resolve to a live item via
            // m_board->GetItem(kiid) only when needed (null-safe).
            v.item_a = aItem->GetMainItemID();
            v.item_b = aItem->GetAuxItemID();

            m_drcViolations.push_back( v );
        } );

    engine->RunTests( EDA_UNITS::MM, true, false );
    engine->ClearViolationHandler();

    DRC_RL::SetClearanceScope( nullptr );   // avoid dangling scope after this run
}


std::vector<RLDRCViolation> PNS_RL_ROUTER::runDRC( const std::string& rules_path )
{
    runDRCEngine( rules_path, nullptr );
    snapshotDrcItems();
    m_lastDrcRulesPath = rules_path;   // baseline provenance for the incremental diff
    return m_drcViolations;
}


// Incremental DRC. Clearance violations are pairwise-local, so retain the ones not
// involving any changed track/via and recompute only the changed ones via a scoped
// clearance pass; connectivity + per-item providers are recomputed in full (cheap,
// and connectivity is global so it must be). The result is identical to a full
// runDRC(), zones included: track-vs-zone is recomputed per scoped track
// (testItemAgainstZone runs inside the scoped track loop) and zone-vs-zone is
// retained (zones never change during routing). The DRC state (m_drcViolations +
// m_drcItemSig) travels with each checkpoint (see checkpoint()/restore*()), so this
// retains correctly after a restore too. Falls back to a full run only on the
// first call (no baseline snapshot to diff against) or when the rules file
// differs from the baseline's.
std::vector<RLDRCViolation> PNS_RL_ROUTER::runDRCIncremental( const std::string& rules_path )
{
    // No baseline, or the baseline was computed under a DIFFERENT rules file:
    // retained violations would mix rule regimes — recompute in full.
    if( m_drcItemSig.empty() || rules_path != m_lastDrcRulesPath )
        return runDRC( rules_path );

    // Diff the board against the last DRC snapshot.
    std::unordered_map<KIID, std::string> curr;
    std::unordered_set<KIID>              changed;
    std::vector<BOARD_ITEM*>              scope;     // added + modified (present on board)

    for( PCB_TRACK* t : m_board->Tracks() )
    {
        std::string sig = drcSigOf( t );
        curr[t->m_Uuid] = sig;

        auto it = m_drcItemSig.find( t->m_Uuid );
        if( it == m_drcItemSig.end() || it->second != sig )   // added or modified
        {
            changed.insert( t->m_Uuid );
            scope.push_back( t );
        }
    }
    for( const auto& [uuid, sig] : m_drcItemSig )
        if( !curr.count( uuid ) )                              // removed
            changed.insert( uuid );

    // Retain prior clearance-family violations not involving any changed item.
    std::vector<RLDRCViolation> retain;
    for( const RLDRCViolation& v : m_drcViolations )
        if( isClearanceFamily( v.error_code )
            && !changed.count( v.item_a ) && !changed.count( v.item_b ) )
            retain.push_back( v );

    // Scoped pass: connectivity + per-item run in full; clearance runs only over
    // the changed items (each still vs the full RTree, so identical results).
    runDRCEngine( rules_path, &scope );

    // Merge the retained clearance violations back in.
    for( const RLDRCViolation& v : retain )
        m_drcViolations.push_back( v );

    m_drcItemSig = std::move( curr );
    return m_drcViolations;
}


int PNS_RL_ROUTER::getDRCViolationCount() const
{
    return static_cast<int>( m_drcViolations.size() );
}


std::vector<RLDRCViolation> PNS_RL_ROUTER::getDRCViolations() const
{
    return m_drcViolations;
}


void PNS_RL_ROUTER::clearDRCCache()
{
    // Both halves of the incremental-DRC state must reset together: m_drcItemSig is
    // the baseline runDRCIncremental() diffs against, so leaving it stale while
    // emptying m_drcViolations makes the next incremental treat the board as
    // "unchanged" and silently drop clearance violations (env.reset → stale carry-
    // over). Clearing the signature forces a full DRC fallback on the next call.
    m_drcViolations.clear();
    m_drcItemSig.clear();
}


std::map<std::string, std::vector<std::string>>
PNS_RL_ROUTER::getDRCViolationsByNet() const
{
    std::map<std::string, std::vector<std::string>> result;

    for( const auto& v : m_drcViolations )
    {
        for( const auto& net : v.net_names )
        {
            auto& types = result[net];
            if( std::find( types.begin(), types.end(), v.error_type ) == types.end() )
                types.push_back( v.error_type );
        }
    }

    return result;
}


// ---------------------------------------------------------------------------
// Design rules (BOARD_DESIGN_SETTINGS + NET_SETTINGS snapshot / write)
// ---------------------------------------------------------------------------

// Convert a NETCLASS std::optional<int> field to millimetres, or -1.0 when unset.
static inline double ncOpt( std::optional<int> v )
{
    return v.has_value() ? mmFromNm( *v ) : -1.0;
}

static RLNetClassInfo toNetClassInfo( const NETCLASS& nc )
{
    RLNetClassInfo info;
    info.name             = nc.GetName().ToStdString();
    info.clearance_mm     = ncOpt( nc.GetClearanceOpt() );
    info.track_width_mm   = ncOpt( nc.GetTrackWidthOpt() );
    info.via_diameter_mm  = ncOpt( nc.GetViaDiameterOpt() );
    info.via_drill_mm     = ncOpt( nc.GetViaDrillOpt() );
    info.uvia_diameter_mm = ncOpt( nc.GetuViaDiameterOpt() );
    info.uvia_drill_mm    = ncOpt( nc.GetuViaDrillOpt() );
    return info;
}


RLDesignRules PNS_RL_ROUTER::getDesignRules() const
{
    const BOARD_DESIGN_SETTINGS& bds = m_board->GetDesignSettings();
    RLDesignRules r{};

    // Global minima
    r.min_clearance_mm         = mmFromNm( bds.m_MinClearance );
    r.min_track_width_mm       = mmFromNm( bds.m_TrackMinWidth );
    r.min_via_diameter_mm      = mmFromNm( bds.m_ViasMinSize );
    r.min_through_hole_mm      = mmFromNm( bds.m_MinThroughDrill );
    r.min_via_annular_width_mm = mmFromNm( bds.m_ViasMinAnnularWidth );
    r.min_hole_to_hole_mm      = mmFromNm( bds.m_HoleToHoleMin );
    r.min_uvia_diameter_mm     = mmFromNm( bds.m_MicroViasMinSize );
    r.min_uvia_drill_mm        = mmFromNm( bds.m_MicroViasMinDrill );
    r.copper_edge_clearance_mm = mmFromNm( bds.m_CopperEdgeClearance );

    // Track width presets — element [0] is reserved for the netclass default
    // (KiCad convention, see BDS::m_TrackWidthList docs). Skip it so callers
    // only see user-defined custom widths.
    for( size_t i = 1; i < bds.m_TrackWidthList.size(); ++i )
    {
        int w = bds.m_TrackWidthList[i];
        if( w > 0 )
            r.track_width_presets_mm.push_back( mmFromNm( w ) );
    }

    // Via size presets — same convention: element [0] is the netclass default.
    for( size_t i = 1; i < bds.m_ViasDimensionsList.size(); ++i )
    {
        const VIA_DIMENSION& v = bds.m_ViasDimensionsList[i];
        if( v.m_Diameter > 0 )
            r.via_presets_mm.emplace_back( mmFromNm( v.m_Diameter ),
                                           mmFromNm( v.m_Drill ) );
    }

    // Netclasses
    const auto& ns = bds.m_NetSettings;
    if( ns )
    {
        if( auto def = ns->GetDefaultNetclass() )
            r.default_netclass = toNetClassInfo( *def );

        for( const auto& [name, nc] : ns->GetNetclasses() )
        {
            if( nc )
                r.netclasses.push_back( toNetClassInfo( *nc ) );
        }
    }

    return r;
}


void PNS_RL_ROUTER::setDesignRules( const RLDesignRules& rules )
{
    BOARD_DESIGN_SETTINGS& bds = m_board->GetDesignSettings();

    // Negative sentinel = "leave unchanged" so partial updates are easy.
    auto apply = [&]( double mm, int& field )
    {
        if( mm >= 0.0 )
            field = nmFromMm( mm );
    };

    apply( rules.min_clearance_mm,         bds.m_MinClearance );
    apply( rules.min_track_width_mm,       bds.m_TrackMinWidth );
    apply( rules.min_via_diameter_mm,      bds.m_ViasMinSize );
    apply( rules.min_through_hole_mm,      bds.m_MinThroughDrill );
    apply( rules.min_via_annular_width_mm, bds.m_ViasMinAnnularWidth );
    apply( rules.min_hole_to_hole_mm,      bds.m_HoleToHoleMin );
    apply( rules.min_uvia_diameter_mm,     bds.m_MicroViasMinSize );
    apply( rules.min_uvia_drill_mm,        bds.m_MicroViasMinDrill );
    apply( rules.copper_edge_clearance_mm, bds.m_CopperEdgeClearance );

    // Resync PNS size cache — initRouter() copies these once at construction,
    // so without this the router keeps the old values until the next reload.
    m_router->Sizes().SetTrackWidth( bds.GetCurrentTrackWidth() );
    m_router->Sizes().SetClearance( bds.m_MinClearance );
    m_router->UpdateSizes( m_router->Sizes() );   // see setTrackWidth

    // Rule change invalidates the incremental-DRC baseline: retained violations
    // were computed under the OLD rules and must not be mixed with a scoped
    // re-check under the new ones — force a full DRC on the next call.
    clearDRCCache();
}


RLNetClassInfo PNS_RL_ROUTER::getNetClassForNet( int net_code ) const
{
    RLNetClassInfo empty{};   // name == "" signals "lookup failed"

    if( !m_board )
        return empty;

    NETINFO_ITEM* net = m_board->FindNet( net_code );
    if( !net )
        return empty;

    NETCLASS* nc = net->GetNetClass();
    if( !nc )
        return empty;

    return toNetClassInfo( *nc );
}


// ---------------------------------------------------------------------------
// Observation helpers
// ---------------------------------------------------------------------------

int PNS_RL_ROUTER::getTrackCount() const
{
    int count = 0;

    for( BOARD_ITEM* item : m_board->Tracks() )
    {
        if( item->GetClass() != "PCB_VIA" )
            ++count;
    }

    return count;
}


std::vector<RLTrackInfo> PNS_RL_ROUTER::getTracks() const
{
    std::vector<RLTrackInfo> result;

    for( BOARD_ITEM* item : m_board->Tracks() )
    {
        PCB_TRACK* trk = dynamic_cast<PCB_TRACK*>( item );

        if( !trk || trk->GetClass() == "PCB_VIA" )
            continue;

        RLTrackInfo info;
        info.x1_mm    = mmFromNm( trk->GetStart().x );
        info.y1_mm    = mmFromNm( trk->GetStart().y );
        info.x2_mm    = mmFromNm( trk->GetEnd().x );
        info.y2_mm    = mmFromNm( trk->GetEnd().y );
        info.width_mm = mmFromNm( trk->GetWidth() );
        info.layer    = static_cast<int>( trk->GetLayer() );
        info.uuid     = trk->m_Uuid;

        if( NETINFO_ITEM* net = trk->GetNet() )
        {
            info.net_code = net->GetNetCode();
            info.net_name = net->GetNetname().ToStdString();
        }
        else
        {
            info.net_code = -1;
        }

        result.push_back( info );
    }

    return result;
}


// ---------------------------------------------------------------------------
// Observation: pads, ratsnest, unrouted count
// ---------------------------------------------------------------------------

std::vector<RLViaInfo> PNS_RL_ROUTER::getVias() const
{
    std::vector<RLViaInfo> result;

    for( BOARD_ITEM* item : m_board->Tracks() )
    {
        PCB_VIA* via = dynamic_cast<PCB_VIA*>( item );

        if( !via )
            continue;

        RLViaInfo info;
        info.x_mm        = mmFromNm( via->GetStart().x );
        info.y_mm        = mmFromNm( via->GetStart().y );
        info.diameter_mm = mmFromNm( via->GetWidth() );
        info.drill_mm    = mmFromNm( via->GetDrillValue() );
        info.top_layer   = static_cast<int>( via->TopLayer() );
        info.bottom_layer = static_cast<int>( via->BottomLayer() );
        info.uuid        = via->m_Uuid;

        if( NETINFO_ITEM* net = via->GetNet() )
        {
            info.net_code = net->GetNetCode();
            info.net_name = net->GetNetname().ToStdString();
        }
        else
        {
            info.net_code = -1;
        }

        result.push_back( info );
    }

    return result;
}


std::vector<RLPadInfo> PNS_RL_ROUTER::getPads() const
{
    std::vector<RLPadInfo> result;

    for( PAD* pad : m_board->GetPads() )
    {
        RLPadInfo info;
        info.x_mm      = mmFromNm( pad->GetPosition().x );
        info.y_mm      = mmFromNm( pad->GetPosition().y );
        info.width_mm  = mmFromNm( pad->GetSizeX() );
        info.height_mm = mmFromNm( pad->GetSizeY() );

        // Bake cardinal pad rotation (90° / 270°) into width/height so
        // axis-aligned downstream consumers see world-x / world-y extents.
        // Non-cardinal angles are left for callers to handle. KiCad sexpr
        // writes absolute pad orientation, and ``GetOrientationDegrees()``
        // returns that absolute world-frame value.
        {
            double a = std::fmod( pad->GetOrientationDegrees(), 180.0 );
            if( a < 0 ) a += 180.0;
            if( std::abs( a - 90.0 ) < 1.0 )
                std::swap( info.width_mm, info.height_mm );
        }

        // A pad on exactly one copper layer reports that PCB_LAYER_ID; a pad
        // spanning several copper layers (PTH/NPTH, multi-layer SMD/edge
        // connectors) reports RL_LAYER_SPANS_COPPER — Python maps it to the
        // 0/thru sentinel.
        LSEQ cu = pad->GetLayerSet().CuStack();
        info.layer = ( cu.size() == 1 ) ? static_cast<int>( cu[0] )
                                        : RL_LAYER_SPANS_COPPER;

        info.pad_name  = pad->GetNumber().ToStdString();

        if( NETINFO_ITEM* net = pad->GetNet() )
        {
            info.net_code = net->GetNetCode();
            info.net_name = net->GetNetname().ToStdString();
        }
        else
        {
            info.net_code = -1;
        }

        if( FOOTPRINT* fp = pad->GetParentFootprint() )
            info.footprint_ref = fp->GetReference().ToStdString();

        switch( pad->GetAttribute() )
        {
        case PAD_ATTRIB::PTH:  info.pad_type = "thru_hole";    break;
        case PAD_ATTRIB::SMD:  info.pad_type = "smd";          break;
        case PAD_ATTRIB::CONN: info.pad_type = "connect";      break;
        case PAD_ATTRIB::NPTH: info.pad_type = "np_thru_hole"; break;
        default:               info.pad_type = "smd";          break;
        }

        switch( pad->GetShape( pad->GetLayer() ) )
        {
        case PAD_SHAPE::CIRCLE:         info.shape = "circle";         break;
        case PAD_SHAPE::RECTANGLE:      info.shape = "rect";           break;
        case PAD_SHAPE::OVAL:           info.shape = "oval";           break;
        case PAD_SHAPE::TRAPEZOID:      info.shape = "trapezoid";      break;
        case PAD_SHAPE::ROUNDRECT:      info.shape = "roundrect";      break;
        case PAD_SHAPE::CHAMFERED_RECT: info.shape = "chamfered_rect"; break;
        case PAD_SHAPE::CUSTOM:         info.shape = "custom";         break;
        default:                        info.shape = "";               break;
        }

        result.push_back( info );
    }

    return result;
}


std::vector<RLZoneInfo> PNS_RL_ROUTER::getKeepouts() const
{
    std::vector<RLZoneInfo> result;

    auto emitZone =
            [&]( ZONE* zone )
            {
                // Only rule areas that actually declare keepout parameters —
                // the same gate ``syncZone`` uses before adding PNS obstacles.
                if( !zone->GetIsRuleArea() || !zone->HasKeepoutParametersSet() )
                    return;

                const SHAPE_POLY_SET* poly = zone->Outline();

                if( !poly || poly->OutlineCount() == 0 )
                    return;

                // First contour of the first polygon; holes are not exported.
                const SHAPE_LINE_CHAIN& chain = poly->COutline( 0 );

                LSET layers = zone->GetLayerSet();

                for( PCB_LAYER_ID layer : LAYER_RANGE( F_Cu, B_Cu, m_board->GetCopperLayerCount() ) )
                {
                    if( !layers[layer] )
                        continue;

                    RLZoneInfo info;
                    info.layer          = static_cast<int>( layer );
                    info.keepout_tracks = zone->GetDoNotAllowTracks();
                    info.keepout_vias   = zone->GetDoNotAllowVias();
                    info.keepout_pads   = zone->GetDoNotAllowPads();
                    info.name           = zone->GetZoneName().ToStdString();

                    for( int i = 0; i < chain.PointCount(); ++i )
                    {
                        const VECTOR2I& p = chain.CPoint( i );
                        info.pts.emplace_back( mmFromNm( p.x ), mmFromNm( p.y ) );
                    }

                    result.push_back( std::move( info ) );
                }
            };

    for( ZONE* zone : m_board->Zones() )
        emitZone( zone );

    // Footprint-scoped rule areas (rare, but SyncWorld handles them too).
    for( FOOTPRINT* fp : m_board->Footprints() )
        for( ZONE* zone : fp->Zones() )
            emitZone( zone );

    return result;
}


std::vector<RLFootprintInfo> PNS_RL_ROUTER::getFootprints() const
{
    std::vector<RLFootprintInfo> result;

    for( FOOTPRINT* fp : m_board->Footprints() )
    {
        RLFootprintInfo info;
        info.ref             = fp->GetReference().ToStdString();
        info.value           = fp->GetValue().ToStdString();
        info.fpid            = fp->GetFPIDAsString().ToStdString();
        info.x_mm            = mmFromNm( fp->GetPosition().x );
        info.y_mm            = mmFromNm( fp->GetPosition().y );
        info.orientation_deg = fp->GetOrientationDegrees();
        info.flipped         = fp->IsFlipped();
        info.layer           = static_cast<int>( fp->GetLayer() );

        // Prefer the side the part is mounted on; a few libraries draw the
        // courtyard only on the other side, so fall back rather than report
        // "no courtyard". GetCourtyard() rebuilds the cache itself when the
        // graphics changed, so no explicit BuildCourtyardCaches() is needed.
        const PCB_LAYER_ID mountSide = info.flipped ? B_CrtYd : F_CrtYd;
        const PCB_LAYER_ID otherSide = info.flipped ? F_CrtYd : B_CrtYd;

        const SHAPE_POLY_SET* poly = &fp->GetCourtyard( mountSide );

        if( poly->OutlineCount() == 0 )
            poly = &fp->GetCourtyard( otherSide );

        for( int i = 0; i < poly->OutlineCount(); ++i )
        {
            const SHAPE_LINE_CHAIN& chain = poly->COutline( i );

            if( chain.PointCount() < 3 )
                continue;

            std::vector<std::pair<double, double>> contour;
            contour.reserve( chain.PointCount() );

            for( int k = 0; k < chain.PointCount(); ++k )
            {
                const VECTOR2I& p = chain.CPoint( k );
                contour.emplace_back( mmFromNm( p.x ), mmFromNm( p.y ) );
            }

            info.courtyard.push_back( std::move( contour ) );
        }

        result.push_back( std::move( info ) );
    }

    return result;
}


// Copper layer of a ratsnest endpoint: single copper layer -> its
// PCB_LAYER_ID, several -> RL_LAYER_SPANS_COPPER (Python maps it to the
// 0/thru sentinel). The RN edge itself carries no layer (KiCad computes it
// in 2D).
//
// Canonicalised over (position, cluster), NOT the anchor instance: several
// coincident anchors can exist at one point (via centre + track end + pad)
// and WHICH one the MST picked is a tie-break that varies across ratsnest
// rebuilds — reporting the picked anchor's parent alone would make obs layers
// nondeterministic. Union the copper of every same-cluster item anchored at
// this position instead; that depends only on board state.
static int anchorCopperLayer( const std::shared_ptr<const CN_ANCHOR>& aAnchor )
{
    if( !aAnchor || !aAnchor->Parent() )
        return RL_LAYER_SPANS_COPPER;

    LSET           copper;
    const VECTOR2I pos = aAnchor->Pos();

    if( const std::shared_ptr<CN_CLUSTER>& cluster = aAnchor->GetCluster() )
    {
        for( CN_ITEM* item : *cluster )
        {
            if( !item || !item->Parent() )
                continue;

            for( const std::shared_ptr<CN_ANCHOR>& a : item->Anchors() )
            {
                if( a && a->Pos() == pos )
                {
                    copper |= item->Parent()->GetLayerSet();
                    break;
                }
            }
        }
    }

    if( copper.none() )
        copper = aAnchor->Parent()->GetLayerSet();

    LSEQ cu = copper.CuStack();
    return cu.size() == 1 ? static_cast<int>( cu[0] ) : RL_LAYER_SPANS_COPPER;
}


std::vector<RLRatsnestEdge> PNS_RL_ROUTER::getRatsnest() const
{
    std::vector<RLRatsnestEdge> result;

    auto connectivity = m_board->GetConnectivity();

    if( !connectivity )
        return result;

    connectivity->RecalculateRatsnest();

    for( unsigned nc = 1; nc < m_board->GetNetCount(); ++nc )
    {
        RN_NET* rn = connectivity->GetRatsnestForNet( nc );

        if( !rn )
            continue;

        for( const CN_EDGE& edge : rn->GetEdges() )
        {
            RLRatsnestEdge re;
            re.x1_mm    = mmFromNm( edge.GetSourcePos().x );
            re.y1_mm    = mmFromNm( edge.GetSourcePos().y );
            re.x2_mm    = mmFromNm( edge.GetTargetPos().x );
            re.y2_mm    = mmFromNm( edge.GetTargetPos().y );
            re.net_code = static_cast<int>( nc );
            re.layer1   = anchorCopperLayer( edge.GetSourceNode() );
            re.layer2   = anchorCopperLayer( edge.GetTargetNode() );

            // Undirected edge — canonicalise endpoint order. Which end the
            // MST calls "source" is an implementation detail that varies
            // across ratsnest rebuild histories; without this, coincident
            // (zero-length) edges flip their (layer1, layer2) order between
            // otherwise identical runs.
            if( std::tie( re.x2_mm, re.y2_mm, re.layer2 )
                    < std::tie( re.x1_mm, re.y1_mm, re.layer1 ) )
            {
                std::swap( re.x1_mm, re.x2_mm );
                std::swap( re.y1_mm, re.y2_mm );
                std::swap( re.layer1, re.layer2 );
            }

            result.push_back( re );
        }
    }

    return result;
}


std::vector<std::pair<int, int>> PNS_RL_ROUTER::getPadClusters() const
{
    std::vector<std::pair<int, int>> result;

    auto connectivity = m_board->GetConnectivity();

    if( !connectivity )
        return result;

    connectivity->RecalculateRatsnest();

    for( const std::shared_ptr<CN_CLUSTER>& cluster :
         connectivity->GetConnectivityAlgo()->GetClusters() )
    {
        if( !cluster->HasValidNet() )
            continue;

        int padCount = 0;

        for( CN_ITEM* item : *cluster )
        {
            BOARD_CONNECTED_ITEM* parent = item->Parent();

            if( parent && parent->Type() == PCB_PAD_T )
                padCount++;
        }

        if( padCount > 0 )
            result.emplace_back( cluster->OriginNet(), padCount );
    }

    return result;
}


/**
 * Append one entry per (anchor point, copper layer) of @p aItem.
 *
 * Anchors are the same points the RL candidate pool can target: pad / via
 * centres and both track endpoints. Layers come from the item's own LSET, so a
 * thru-hole pad or a via naturally covers every layer it spans — which is why
 * the caller never has to do layer-blind coordinate matching.
 */
static void appendClusterPoints( const BOARD_CONNECTED_ITEM* aItem,
                                 std::vector<RLClusterPoint>& aOut )
{
    if( !aItem )
        return;

    LSET lset = aItem->GetLayerSet();

    auto emit =
            [&]( const VECTOR2I& aPos )
            {
                for( PCB_LAYER_ID layer : lset.CuStack() )
                {
                    RLClusterPoint p;
                    p.x_mm  = mmFromNm( aPos.x );
                    p.y_mm  = mmFromNm( aPos.y );
                    p.layer = static_cast<int>( layer );
                    aOut.push_back( p );
                }
            };

    switch( aItem->Type() )
    {
    case PCB_PAD_T:
        emit( static_cast<const PAD*>( aItem )->GetPosition() );
        break;

    case PCB_VIA_T:
        emit( static_cast<const PCB_VIA*>( aItem )->GetPosition() );
        break;

    case PCB_TRACE_T:
    case PCB_ARC_T:
    {
        const PCB_TRACK* trk = static_cast<const PCB_TRACK*>( aItem );
        emit( trk->GetStart() );
        emit( trk->GetEnd() );
        break;
    }

    default:
        // Zones and everything else carry no discrete anchor the router can
        // target as a candidate point; ignore them.
        break;
    }
}


std::vector<RLClusterPoint> PNS_RL_ROUTER::getConnectedPoints( double x_mm, double y_mm,
                                                               int boardLayer )
{
    std::vector<RLClusterPoint> result;

    if( !m_board || !m_router )
        return result;

    // Locate the copper under the query point. itemAt() is the same
    // deterministic (UUID-ordered) lookup startRoute uses, so the cluster we
    // report is the cluster of the item the router itself would grab.
    PNS::ITEM* hit = itemAt( VECTOR2I( nmFromMm( x_mm ), nmFromMm( y_mm ) ), boardLayer );

    if( !hit )
        return result;

    BOARD_CONNECTED_ITEM* seed = dynamic_cast<BOARD_CONNECTED_ITEM*>( hit->Parent() );

    if( !seed )
        return result;

    auto connectivity = m_board->GetConnectivity();

    if( !connectivity )
        return result;

    // GetConnectedItems() returns the seed's ENTIRE cluster (transitive, scoped
    // to its net), not just direct neighbours — connectivity_data.cpp resolves
    // it through SearchClusters + CN_CLUSTER::Contains.
    static const std::vector<KICAD_T> types = { PCB_TRACE_T, PCB_ARC_T, PCB_VIA_T, PCB_PAD_T };

    for( BOARD_CONNECTED_ITEM* item : connectivity->GetConnectedItems( seed, types ) )
        appendClusterPoints( item, result );

    return result;
}


std::vector<RLBoardEdge> PNS_RL_ROUTER::getBoardOutline() const
{
    std::vector<RLBoardEdge> result;
    constexpr int CIRCLE_SEGMENTS     = 32;
    constexpr double TWO_PI           = 2.0 * M_PI;

    auto pushSeg = [&]( const VECTOR2I& a, const VECTOR2I& b, double width )
    {
        RLBoardEdge e;
        e.x1_mm    = mmFromNm( a.x );
        e.y1_mm    = mmFromNm( a.y );
        e.x2_mm    = mmFromNm( b.x );
        e.y2_mm    = mmFromNm( b.y );
        e.width_mm = width;
        result.push_back( e );
    };
    auto pushSegMm = [&]( double x1, double y1, double x2, double y2, double width )
    {
        RLBoardEdge e;
        e.x1_mm    = x1;
        e.y1_mm    = y1;
        e.x2_mm    = x2;
        e.y2_mm    = y2;
        e.width_mm = width;
        result.push_back( e );
    };

    for( BOARD_ITEM* item : m_board->Drawings() )
    {
        PCB_SHAPE* shape = dynamic_cast<PCB_SHAPE*>( item );

        if( !shape )
            continue;

        if( shape->GetLayer() != Edge_Cuts )
            continue;

        const double width = mmFromNm( shape->GetWidth() );

        switch( shape->GetShape() )
        {
        case SHAPE_T::SEGMENT:
            pushSeg( shape->GetStart(), shape->GetEnd(), width );
            break;

        case SHAPE_T::RECTANGLE:
        {
            std::vector<VECTOR2I> corners = shape->GetRectCorners();

            if( corners.size() == 4 )
            {
                for( int i = 0; i < 4; ++i )
                    pushSeg( corners[i], corners[( i + 1 ) % 4], width );
            }

            break;
        }

        case SHAPE_T::CIRCLE:
        {
            VECTOR2I c   = shape->GetCenter();
            double cx_mm = mmFromNm( c.x );
            double cy_mm = mmFromNm( c.y );
            double r_mm  = mmFromNm( shape->GetRadius() );

            for( int i = 0; i < CIRCLE_SEGMENTS; ++i )
            {
                double a1 = TWO_PI * i / CIRCLE_SEGMENTS;
                double a2 = TWO_PI * ( i + 1 ) / CIRCLE_SEGMENTS;
                pushSegMm(
                    cx_mm + r_mm * std::cos( a1 ), cy_mm + r_mm * std::sin( a1 ),
                    cx_mm + r_mm * std::cos( a2 ), cy_mm + r_mm * std::sin( a2 ),
                    width );
            }

            break;
        }

        case SHAPE_T::ARC:
        {
            // Error-bounded tessellation via KiCad's own arc->polyline builder.
            // The segment count adapts to radius & sweep so the chord-to-arc
            // deviation stays within the board's m_MaxError (default 0.005 mm),
            // instead of a fixed 16-segments-per-90-degrees that under-resolves
            // large arcs and over-tessellates small fillets (see GetArcToSegmentCount).
            SHAPE_ARC        sarc( shape->GetStart(), shape->GetArcMid(),
                                   shape->GetEnd(), 0 );
            SHAPE_LINE_CHAIN chain;
            chain.Append( sarc, m_board->GetDesignSettings().m_MaxError );
            chain.ClearArcs();   // drop retained arc metadata; keep plain points

            for( int i = 0; i + 1 < chain.PointCount(); ++i )
                pushSeg( chain.CPoint( i ), chain.CPoint( i + 1 ), width );

            break;
        }

        case SHAPE_T::POLY:
        {
            const SHAPE_POLY_SET& poly = shape->GetPolyShape();

            for( int oi = 0; oi < poly.OutlineCount(); ++oi )
            {
                const SHAPE_LINE_CHAIN& chain = poly.COutline( oi );
                int                     npts  = chain.PointCount();

                for( int i = 0; i < npts; ++i )
                {
                    VECTOR2I p1 = chain.CPoint( i );
                    VECTOR2I p2 = chain.CPoint( ( i + 1 ) % npts );
                    pushSeg( p1, p2, width );
                }
            }

            break;
        }

        case SHAPE_T::BEZIER:
        {
            const std::vector<VECTOR2I>& pts = shape->GetBezierPoints();

            for( size_t i = 0; i + 1 < pts.size(); ++i )
                pushSeg( pts[i], pts[i + 1], width );

            break;
        }

        default:
            break;
        }
    }

    return result;
}


std::vector<RLBoardOutlineShape> PNS_RL_ROUTER::getBoardOutlineShapes() const
{
    // Primitive-preserving counterpart of getBoardOutline(): arcs and circles
    // come back as single typed entries (KiCad's native 3-point form) instead
    // of tessellated polylines; everything else is emitted as segments.
    std::vector<RLBoardOutlineShape> result;

    auto push = [&]( int kind, double x1, double y1, double x2, double y2,
                     double x3, double y3, double width )
    {
        RLBoardOutlineShape s;
        s.kind     = kind;
        s.x1_mm    = x1;
        s.y1_mm    = y1;
        s.x2_mm    = x2;
        s.y2_mm    = y2;
        s.x3_mm    = x3;
        s.y3_mm    = y3;
        s.width_mm = width;
        result.push_back( s );
    };
    auto pushSeg = [&]( const VECTOR2I& a, const VECTOR2I& b, double width )
    {
        push( 0, mmFromNm( a.x ), mmFromNm( a.y ),
                 mmFromNm( b.x ), mmFromNm( b.y ), 0.0, 0.0, width );
    };

    for( BOARD_ITEM* item : m_board->Drawings() )
    {
        PCB_SHAPE* shape = dynamic_cast<PCB_SHAPE*>( item );

        if( !shape )
            continue;

        if( shape->GetLayer() != Edge_Cuts )
            continue;

        const double width = mmFromNm( shape->GetWidth() );

        switch( shape->GetShape() )
        {
        case SHAPE_T::SEGMENT:
            pushSeg( shape->GetStart(), shape->GetEnd(), width );
            break;

        case SHAPE_T::RECTANGLE:
        {
            std::vector<VECTOR2I> corners = shape->GetRectCorners();

            if( corners.size() == 4 )
            {
                for( int i = 0; i < 4; ++i )
                    pushSeg( corners[i], corners[( i + 1 ) % 4], width );
            }

            break;
        }

        case SHAPE_T::CIRCLE:
        {
            VECTOR2I c   = shape->GetCenter();
            double cx_mm = mmFromNm( c.x );
            double cy_mm = mmFromNm( c.y );
            double r_mm  = mmFromNm( shape->GetRadius() );

            push( 2, cx_mm + r_mm, cy_mm, cx_mm + r_mm, cy_mm,
                     cx_mm - r_mm, cy_mm, width );
            break;
        }

        case SHAPE_T::ARC:
        {
            VECTOR2I a = shape->GetStart();
            VECTOR2I m = shape->GetArcMid();
            VECTOR2I b = shape->GetEnd();

            push( 1, mmFromNm( a.x ), mmFromNm( a.y ),
                     mmFromNm( b.x ), mmFromNm( b.y ),
                     mmFromNm( m.x ), mmFromNm( m.y ), width );
            break;
        }

        case SHAPE_T::POLY:
        {
            const SHAPE_POLY_SET& poly = shape->GetPolyShape();

            for( int oi = 0; oi < poly.OutlineCount(); ++oi )
            {
                const SHAPE_LINE_CHAIN& chain = poly.COutline( oi );
                int                     npts  = chain.PointCount();

                for( int i = 0; i < npts; ++i )
                {
                    VECTOR2I p1 = chain.CPoint( i );
                    VECTOR2I p2 = chain.CPoint( ( i + 1 ) % npts );
                    pushSeg( p1, p2, width );
                }
            }

            break;
        }

        case SHAPE_T::BEZIER:
        {
            const std::vector<VECTOR2I>& pts = shape->GetBezierPoints();

            for( size_t i = 0; i + 1 < pts.size(); ++i )
                pushSeg( pts[i], pts[i + 1], width );

            break;
        }

        default:
            break;
        }
    }

    return result;
}


int PNS_RL_ROUTER::getUnroutedCount() const
{
    auto connectivity = m_board->GetConnectivity();

    if( !connectivity )
        return 0;

    connectivity->RecalculateRatsnest();
    return static_cast<int>( connectivity->GetUnconnectedCount( false ) );
}


// ---------------------------------------------------------------------------
// Connectivity & Board Query
// ---------------------------------------------------------------------------

void PNS_RL_ROUTER::buildConnectivity()
{
    m_board->BuildConnectivity();
}


void PNS_RL_ROUTER::recalculateRatsnest()
{
    auto connectivity = m_board->GetConnectivity();

    if( connectivity )
        connectivity->RecalculateRatsnest();
}


int PNS_RL_ROUTER::getNetCount() const
{
    auto connectivity = m_board->GetConnectivity();

    if( !connectivity )
        return 0;

    return connectivity->GetNetCount();
}


int PNS_RL_ROUTER::getBoardNetCount() const
{
    return static_cast<int>( m_board->GetNetCount() );
}


RLBoundingBox PNS_RL_ROUTER::getBoardBBox() const
{
    // Edge.Cuts-only bbox = the physical board size (selected by LAYER, so
    // lines/arcs/circles/polys and footprint-embedded edges all count; tracks,
    // zones and off-board silk/text never do). The all-items GetBoundingBox()
    // would let decorations and agent copper inflate the obs/wirelength scale —
    // on real boards by up to 1.6x.
    const BOX2I bbox = m_board->GetBoardEdgesBoundingBox();

    if( bbox.GetWidth() <= 0 || bbox.GetHeight() <= 0 )
        throw std::runtime_error(
                "PNS_RL_ROUTER::getBoardBBox: board has no Edge.Cuts outline "
                "(degenerate bbox) — obs/reward normalisation needs the "
                "physical board size; the source file must carry an outline" );

    RLBoundingBox result;
    result.x_mm      = mmFromNm( bbox.GetOrigin().x );
    result.y_mm      = mmFromNm( bbox.GetOrigin().y );
    result.width_mm  = mmFromNm( bbox.GetWidth() );
    result.height_mm = mmFromNm( bbox.GetHeight() );
    return result;
}


int PNS_RL_ROUTER::getCopperLayerCount() const
{
    return m_board->GetCopperLayerCount();
}


// ---------------------------------------------------------------------------
// State & Failure Query
// ---------------------------------------------------------------------------

int PNS_RL_ROUTER::getRouterState() const
{
    return static_cast<int>( m_router->GetState() );
}


std::string PNS_RL_ROUTER::getFailureReason() const
{
    return m_router->FailureReason().ToStdString();
}


// ---------------------------------------------------------------------------
// Routing Control
// ---------------------------------------------------------------------------

bool PNS_RL_ROUTER::finish( int maxAttempts )
{
    if( !m_routing )
        return false;

    // maxAttempts is forwarded to ROUTER::Finish(), where it bounds the internal
    // Move-to-convergence loop (triesLeft). No outer retry here.
    bool ok = m_router->Finish( maxAttempts );

    if( ok )
    {
        m_router->CommitRouting();
        m_routing = false;
        // Re-sync the PNS world from the committed board.
        resyncWorld();
    }

    return ok;
}


bool PNS_RL_ROUTER::undoLastSegment()
{
    if( !m_routing )
        return false;

    auto result = m_router->UndoLastSegment();
    return result.has_value();
}


void PNS_RL_ROUTER::flipPosture()
{
    if( m_routing )
        m_router->FlipPosture();
}


// ---------------------------------------------------------------------------
// Via & Layer Control
// ---------------------------------------------------------------------------

void PNS_RL_ROUTER::toggleVia()
{
    m_router->ToggleViaPlacement();
}


bool PNS_RL_ROUTER::switchLayer( int boardLayer )
{
    if( !m_routing )
        return false;

    return m_router->SwitchLayer( boardToPnsLayer( boardLayer ) );
}


bool PNS_RL_ROUTER::isPlacingVia() const
{
    return m_router->IsPlacingVia();
}


int PNS_RL_ROUTER::getCurrentLayer() const
{
    int pnsLayer = m_router->GetCurrentLayer();

    if( pnsLayer < 0 )
        return -1;

    return pnsToBoardLayer( pnsLayer );
}


void PNS_RL_ROUTER::setViaDiameter( double diameter_mm )
{
    int diameter_nm = nmFromMm( diameter_mm );

    if( diameter_nm > 0 )
    {
        m_router->Sizes().SetViaDiameter( diameter_nm );
        m_router->UpdateSizes( m_router->Sizes() );   // see setTrackWidth
    }
}


void PNS_RL_ROUTER::setViaDrill( double drill_mm )
{
    int drill_nm = nmFromMm( drill_mm );

    if( drill_nm > 0 )
    {
        m_router->Sizes().SetViaDrill( drill_nm );
        m_router->UpdateSizes( m_router->Sizes() );   // see setTrackWidth
    }
}


void PNS_RL_ROUTER::resetViaMode()
{
    if( m_router->IsPlacingVia() )
        m_router->ToggleViaPlacement();
}


// ---------------------------------------------------------------------------
// Markov-property observation helpers (route head, active net, target)
// ---------------------------------------------------------------------------

std::array<double, 3> PNS_RL_ROUTER::getRouteHead() const
{
    if( !m_routing || !m_router->Placer() )
        return { 0.0, 0.0, -1.0 };

    const VECTOR2I& end = m_router->Placer()->CurrentEnd();
    int pnsLayer = m_router->Placer()->CurrentLayer();
    int boardLayer = pnsToBoardLayer( pnsLayer );

    return { mmFromNm( end.x ), mmFromNm( end.y ), static_cast<double>( boardLayer ) };
}


int PNS_RL_ROUTER::getCurrentNetCode() const
{
    if( !m_routing )
        return -1;

    auto nets = m_router->GetCurrentNets();

    if( nets.empty() || !nets[0] )
        return -1;

    return m_iface->GetNetCode( nets[0] );
}


std::array<double, 3> PNS_RL_ROUTER::getRoutingTarget() const
{
    if( !m_routing || !m_router->Placer() )
        return { 0.0, 0.0, -1.0 };

    int netCode = getCurrentNetCode();

    if( netCode <= 0 )
        return { 0.0, 0.0, -1.0 };

    auto connectivity = m_board->GetConnectivity();

    if( !connectivity )
        return { 0.0, 0.0, -1.0 };

    connectivity->RecalculateRatsnest();
    RN_NET* rn = connectivity->GetRatsnestForNet( netCode );

    if( !rn || rn->GetEdges().empty() )
        return { 0.0, 0.0, -1.0 };

    // Find the ratsnest edge whose one endpoint is nearest to route start,
    // return the other endpoint as the routing target.
    VECTOR2I start = m_router->Placer()->CurrentStart();

    typedef VECTOR2I::extended_type ecoord;
    ecoord   bestDist = std::numeric_limits<ecoord>::max();
    VECTOR2I target   = m_router->Placer()->CurrentEnd();
    std::shared_ptr<const CN_ANCHOR> targetAnchor;

    for( const CN_EDGE& edge : rn->GetEdges() )
    {
        VECTOR2I src = edge.GetSourcePos();
        VECTOR2I tgt = edge.GetTargetPos();

        ecoord dSrc = ( src - start ).SquaredEuclideanNorm();
        ecoord dTgt = ( tgt - start ).SquaredEuclideanNorm();

        if( dSrc < bestDist )
        {
            bestDist     = dSrc;
            target       = tgt;   // far anchor = routing target
            targetAnchor = edge.GetTargetNode();
        }

        if( dTgt < bestDist )
        {
            bestDist     = dTgt;
            target       = src;
            targetAnchor = edge.GetSourceNode();
        }
    }

    // Layer of the TARGET anchor itself. 3rd element encoding:
    // >=0 = target's PCB_LAYER_ID, RL_LAYER_SPANS_COPPER(-2) = target's parent
    // spans copper layers (thru), RL_LAYER_NONE(-1) = no target (the early
    // returns above).
    int layerOut = anchorCopperLayer( targetAnchor );

    return { mmFromNm( target.x ), mmFromNm( target.y ), static_cast<double>( layerOut ) };
}


std::vector<RLTrackInfo> PNS_RL_ROUTER::getWipSegments() const
{
    std::vector<RLTrackInfo> result;

    if( !m_routing || !m_router->Placer() )
        return result;

    PNS::ITEM_SET traces = m_router->Placer()->Traces();

    int wipNet = getCurrentNetCode();

    for( PNS::ITEM* item : traces.Items() )
    {
        if( item->Kind() != PNS::ITEM::SEGMENT_T )
            continue;

        PNS::SEGMENT* seg = static_cast<PNS::SEGMENT*>( item );

        RLTrackInfo info;
        info.x1_mm    = mmFromNm( seg->Seg().A.x );
        info.y1_mm    = mmFromNm( seg->Seg().A.y );
        info.x2_mm    = mmFromNm( seg->Seg().B.x );
        info.y2_mm    = mmFromNm( seg->Seg().B.y );
        info.width_mm = mmFromNm( seg->Width() );
        info.layer    = pnsToBoardLayer( seg->Layer() );
        info.net_code = wipNet;

        result.push_back( info );
    }

    return result;
}


// ---------------------------------------------------------------------------
// Save
// ---------------------------------------------------------------------------

void PNS_RL_ROUTER::save( const std::string& output_path,
                          const std::string& project_output_path ) const
{
    try
    {
        IO_RELEASER<PCB_IO> pi( new PCB_IO_KICAD_SEXPR );
        pi->SaveBoard( wxString::FromUTF8( output_path ), m_board, nullptr );
    }
    catch( const IO_ERROR& ioe )
    {
        throw std::runtime_error( "PNS_RL_ROUTER: failed to save board: "
                                  + output_path + " – "
                                  + ioe.Problem().ToStdString() );
    }

    // Always emit the companion .kicad_pro so BDS + NetSettings survive
    // the round-trip. Without this the modern .kicad_pcb writer would
    // silently drop the design rules.
    if( !m_settingsMgr || !m_project )
        return;

    std::string proOut = project_output_path;
    if( proOut.empty() )
    {
        wxFileName fn( wxString::FromUTF8( output_path ) );
        fn.SetExt( wxT( "kicad_pro" ) );
        proOut = fn.GetFullPath().ToStdString();
    }

    try
    {
        // The source project may have been opened READ-ONLY: LoadProject sets
        // that flag when the project's lock file (~<stem>.kicad_pro.lck) is
        // held by another process — or left stale by a crash — and
        // SaveProjectAs copies it onto the output PROJECT_FILE, whose
        // SaveToFile then silently no-ops: the routed .kicad_pcb would land
        // without its rules sidecar (a parallel-eval hazard). This
        // engine never writes back to the SOURCE project — this save-as to a
        // new path is its only project write — so clearing the flag here,
        // scoped to the save rather than at load, loses no protection: the
        // KiCad-side guard on the source stays intact for any future
        // write-back path.
        m_project->SetReadOnly( false );

        // SaveProjectAs retargets the project's filename and writes it,
        // so subsequent saves with a different output_path work correctly.
        m_settingsMgr->SaveProjectAs( wxString::FromUTF8( proOut ), m_project );
    }
    catch( const std::exception& e )
    {
        throw std::runtime_error( "PNS_RL_ROUTER: failed to save project: "
                                  + proOut + " – " + e.what() );
    }

    // SaveProjectAs ignores its internal SaveToFile results — verify the
    // sidecar actually landed and fail loudly instead of shipping a pcb whose
    // design rules would silently fall back to KiCad defaults on reload.
    if( !wxFileName::FileExists( wxString::FromUTF8( proOut ) ) )
    {
        throw std::runtime_error( "PNS_RL_ROUTER: project file was not written: "
                                  + proOut );
    }
}


// ---------------------------------------------------------------------------
// Board-level graphics read/replace (outline-simplify ingest support)
// ---------------------------------------------------------------------------

std::vector<RLGraphicShape> PNS_RL_ROUTER::getGraphicShapes( int aLayer ) const
{
    std::vector<RLGraphicShape> result;
    int idx = -1;

    for( BOARD_ITEM* item : m_board->Drawings() )
    {
        ++idx;
        PCB_SHAPE* shape = dynamic_cast<PCB_SHAPE*>( item );

        if( !shape || shape->GetLayer() != static_cast<PCB_LAYER_ID>( aLayer ) )
            continue;

        RLGraphicShape g;
        g.index    = idx;
        g.width_nm = shape->GetWidth();

        switch( shape->GetShape() )
        {
        case SHAPE_T::SEGMENT:
            g.kind  = 0;
            g.x1_nm = shape->GetStart().x;
            g.y1_nm = shape->GetStart().y;
            g.x2_nm = shape->GetEnd().x;
            g.y2_nm = shape->GetEnd().y;
            break;

        case SHAPE_T::ARC:
            g.kind  = 1;
            g.x1_nm = shape->GetStart().x;
            g.y1_nm = shape->GetStart().y;
            g.xm_nm = shape->GetArcMid().x;
            g.ym_nm = shape->GetArcMid().y;
            g.x2_nm = shape->GetEnd().x;
            g.y2_nm = shape->GetEnd().y;
            break;

        default:
            g.kind = 2;
            break;
        }

        result.push_back( g );
    }

    return result;
}


void PNS_RL_ROUTER::replaceGraphicShapes( int aLayer,
                                          const std::vector<int>& aRemoveIndices,
                                          const std::vector<std::array<long long, 5>>& aNewSegments,
                                          const std::vector<std::array<long long, 7>>& aNewArcs )
{
    if( m_routing || m_dragging )
        throw std::runtime_error( "replaceGraphicShapes: refusing to mutate the board "
                                  "during an active routing/dragging session" );

    const PCB_LAYER_ID layer = static_cast<PCB_LAYER_ID>( aLayer );

    // Resolve indices against the same enumeration getGraphicShapes() used.
    // Any mismatch (out of range / not a PCB_SHAPE / wrong layer) is a hard
    // error: the caller's plan is stale, and silently skipping would desync
    // the planned and the applied geometry.
    std::vector<BOARD_ITEM*> drawings( m_board->Drawings().begin(),
                                       m_board->Drawings().end() );
    std::vector<PCB_SHAPE*> doomed;
    doomed.reserve( aRemoveIndices.size() );

    for( int i : aRemoveIndices )
    {
        if( i < 0 || static_cast<size_t>( i ) >= drawings.size() )
            throw std::runtime_error( "replaceGraphicShapes: index out of range: "
                                      + std::to_string( i ) );

        PCB_SHAPE* shape = dynamic_cast<PCB_SHAPE*>( drawings[i] );

        if( !shape || shape->GetLayer() != layer )
            throw std::runtime_error( "replaceGraphicShapes: index " + std::to_string( i )
                                      + " is not a PCB_SHAPE on the target layer" );

        doomed.push_back( shape );
    }

    for( PCB_SHAPE* shape : doomed )
    {
        m_board->Remove( shape, REMOVE_MODE::NORMAL );
        delete shape;
    }

    for( const auto& s : aNewSegments )
    {
        PCB_SHAPE* shape = new PCB_SHAPE( m_board, SHAPE_T::SEGMENT );
        shape->SetStart( VECTOR2I( static_cast<int>( s[0] ), static_cast<int>( s[1] ) ) );
        shape->SetEnd( VECTOR2I( static_cast<int>( s[2] ), static_cast<int>( s[3] ) ) );
        shape->SetLayer( layer );
        shape->SetWidth( static_cast<int>( s[4] ) );
        m_board->Add( shape, ADD_MODE::APPEND );
    }

    for( const auto& a : aNewArcs )
    {
        PCB_SHAPE* shape = new PCB_SHAPE( m_board, SHAPE_T::ARC );
        shape->SetArcGeometry( VECTOR2I( static_cast<int>( a[0] ), static_cast<int>( a[1] ) ),
                               VECTOR2I( static_cast<int>( a[2] ), static_cast<int>( a[3] ) ),
                               VECTOR2I( static_cast<int>( a[4] ), static_cast<int>( a[5] ) ) );
        shape->SetLayer( layer );
        shape->SetWidth( static_cast<int>( a[6] ) );
        m_board->Add( shape, ADD_MODE::APPEND );
    }

    // The board's item set changed: rebuild connectivity + the PNS world
    // exactly like the initial load.
    initRouter();

    // Move the episode-start KIID rewind point past the UUIDs the new shapes
    // just consumed (see header comment — collision guard under seeding).
    if( !m_episodeStartKiidState.empty() )
        m_episodeStartKiidState = KIID::GetGeneratorState();
}
