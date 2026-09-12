/*
 * PNS_RL_ROUTER  –  high-level C++ wrapper for RL-based PCB routing.
 *
 * Exposes a minimal Python-friendly API:
 *   1. load a board
 *   2. delete / inspect existing tracks
 *   3. run PNS routing (shove / walkaround / mark-obstacles) step by step
 *   4. save the modified board
 *
 * All coordinates in the public API use millimetres (double).
 * Internally coordinates are in KiCad nanometres (int).
 */

#ifndef PNS_RL_ROUTER_H
#define PNS_RL_ROUTER_H

#include <memory>
#include <map>
#include <unordered_map>
#include <string>
#include <utility>
#include <vector>
#include <array>
#include <limits>
#include <cstdint>
#include <math/vector2d.h>
#include <kiid.h>

// Forward declarations – keep includes out of the header
class BOARD;
class BOARD_ITEM;
class PROJECT;
class SETTINGS_MANAGER;

namespace PNS
{
class ROUTER;
class ROUTING_SETTINGS;
class ITEM;
}

class PNS_RL_IFACE;

/**
 * Info struct returned by getPads().
 */
struct RLPadInfo
{
    double      x_mm, y_mm;              ///< pad centre (mm)
    double      width_mm, height_mm;     ///< pad size   (mm)
    int         layer;                   ///< PCB_LAYER_ID
    int         net_code;                ///< net code (-1 = unconnected)
    std::string net_name;
    std::string pad_name;                ///< pad number (e.g. "1")
    std::string footprint_ref;           ///< parent footprint reference (e.g. "P1")
    std::string pad_type;                ///< "smd" | "thru_hole" | "np_thru_hole" | "connect"
    std::string shape;                   ///< KiCad pad shape token: "circle" | "rect" | "oval" | "trapezoid" | "roundrect" | "chamfered_rect" | "custom"
};

/**
 * Info struct returned by getKeepouts().
 * One entry per rule-area keepout zone per copper layer it applies to. The
 * outline is the zone's first contour as a polygon (mm); holes are not
 * represented. This mirrors what ``syncZone`` feeds the PNS world, so obs /
 * viz can show exactly the region the router treats as a keepout obstacle.
 */
struct RLZoneInfo
{
    std::vector<std::pair<double, double>> pts;  ///< outline polygon vertices (x_mm, y_mm)
    int         layer;               ///< PCB_LAYER_ID copper layer this entry applies to
    bool        keepout_tracks;      ///< GetDoNotAllowTracks(): tracks disallowed
    bool        keepout_vias;        ///< GetDoNotAllowVias(): vias disallowed
    bool        keepout_pads;        ///< GetDoNotAllowPads(): pads disallowed
    std::string name;                ///< zone name (may be empty)
};

/**
 * Info struct returned by getFootprints() — one entry per board component.
 *
 * ``courtyard`` is the component's placement keep-out (F.CrtYd / B.CrtYd),
 * already in BOARD coordinates: KiCad builds the courtyard cache from the
 * footprint's placed PCB_SHAPEs, so rotation and side-flip are baked in and
 * callers need no transform. One entry per closed contour; interior holes are
 * dropped (they are vanishingly rare on courtyards) — same convention as
 * RLZoneInfo.
 *
 * The courtyard is chosen from the side the footprint is mounted on, falling
 * back to the opposite side when only that one is drawn. Many older library
 * footprints declare no courtyard at all; ``courtyard`` is then empty. It is
 * deliberately NOT replaced by a bounding box here — FOOTPRINT::GetBoundingHull
 * envelopes the pads and convex-hulls the result, so it carries no component
 * shape. Callers that want a fallback should synthesize it themselves.
 */
struct RLFootprintInfo
{
    std::string ref;                 ///< reference designator (e.g. "U3")
    std::string value;               ///< value field (e.g. "STM32F103")
    std::string fpid;                ///< library identifier ("lib:footprint")
    double      x_mm, y_mm;          ///< footprint origin (mm, board coords)
    double      orientation_deg;     ///< absolute orientation, degrees
    bool        flipped;             ///< true = mounted on the back side
    int         layer;               ///< PCB_LAYER_ID the footprint sits on
    /// Closed courtyard contours, each a list of (x_mm, y_mm) vertices.
    std::vector<std::vector<std::pair<double, double>>> courtyard;
};

/**
 * One board outline (Edge.Cuts) segment after C++-side discretization
 * of arcs / circles / polygons / beziers into straight-line polylines.
 */
struct RLBoardEdge
{
    double x1_mm, y1_mm;
    double x2_mm, y2_mm;
    double width_mm;
};

/**
 * One board outline (Edge.Cuts) primitive returned by getBoardOutlineShapes(),
 * with arcs/circles kept intact instead of pre-tessellated.
 * kind = 0: straight segment p1->p2 (p3 unused, 0). Rectangles, polygons and
 *           beziers are emitted as their constituent segments.
 * kind = 1: arc — p1 = start, p2 = end, p3 = the on-arc midpoint (KiCad's
 *           native 3-point representation; centre/radius are derived values).
 * kind = 2: full circle — p1 == p2 = a point on the circle, p3 = its antipode
 *           (the two points span a diameter, so the circle is fully determined).
 */
struct RLBoardOutlineShape
{
    int    kind;
    double x1_mm, y1_mm;
    double x2_mm, y2_mm;
    double x3_mm, y3_mm;
    double width_mm;
};

/**
 * Copper-layer sentinels shared by the RL info structs and getRoutingTarget().
 * Values >= 0 are PCB_LAYER_IDs. Python's wrapper translates them to its
 * human-layer space (1..N; 0 = "spans the copper stack" a.k.a. thru sentinel):
 *   RL_LAYER_NONE          — no such layer (e.g. no routing target)
 *   RL_LAYER_SPANS_COPPER  — parent occupies several copper layers
 *                            (through pad/via) → Python human layer 0
 */
constexpr int RL_LAYER_NONE         = -1;
constexpr int RL_LAYER_SPANS_COPPER = -2;

/**
 * Info struct returned by getRatsnest().
 * Each edge represents an unrouted connection between two anchors.
 */
struct RLRatsnestEdge
{
    double x1_mm, y1_mm;   ///< source anchor (mm)
    double x2_mm, y2_mm;   ///< target anchor (mm)
    int    net_code;        ///< net code
    int    layer1;          ///< source anchor: PCB_LAYER_ID or RL_LAYER_SPANS_COPPER
    int    layer2;          ///< target anchor: PCB_LAYER_ID or RL_LAYER_SPANS_COPPER
};

/**
 * Info struct returned by getConnectedPoints().
 *
 * One anchor point of a board item that shares a connectivity cluster with the
 * queried position — i.e. copper that is ALREADY electrically joined to it, so
 * routing to it would only close a redundant loop. One entry per copper layer
 * the item occupies (a thru-hole pad emits its centre once per copper layer, a
 * via once per layer of its span, a track its two endpoints on its own layer),
 * mirroring how the Python candidate pool expands layers.
 */
struct RLClusterPoint
{
    double x_mm, y_mm;   ///< anchor position (mm)
    int    layer;        ///< PCB_LAYER_ID
};

/**
 * Info struct returned by getVias().
 */
struct RLViaInfo
{
    double x_mm, y_mm;       ///< via centre (mm)
    double diameter_mm;      ///< outer copper diameter (mm)
    double drill_mm;         ///< drill hole diameter (mm)
    int    top_layer;        ///< topmost connected layer (PCB_LAYER_ID)
    int    bottom_layer;     ///< bottommost connected layer (PCB_LAYER_ID)
    int    net_code;         ///< net code (-1 = unconnected)
    std::string net_name;
    KIID   uuid = niluuid;   ///< stable via identity (value key, not a pointer)
};

/**
 * Info struct returned by getTracks().
 */
struct RLTrackInfo
{
    double x1_mm, y1_mm;   ///< start point (mm)
    double x2_mm, y2_mm;   ///< end  point  (mm)
    double width_mm;        ///< track width (mm)
    int    layer;           ///< KiCad PCB_LAYER_ID (F_Cu=0, B_Cu=2, ...)
    int    net_code;        ///< net code (-1 = unconnected)
    std::string net_name;
    KIID   uuid = niluuid;  ///< stable track identity (value key, not a pointer)
};


/**
 * Info struct returned by getDRCViolations().
 */
struct RLDRCViolation
{
    int         error_code;                  ///< DRCE_CLEARANCE, DRCE_TRACK_WIDTH, etc.
    std::string error_type;                  ///< human-readable type name
    std::string message;                     ///< violation description
    double      x_mm, y_mm;                 ///< violation position (mm)
    int         layer;                       ///< PCB_LAYER_ID
    std::vector<std::string> net_names;      ///< nets involved in this violation (0–2 entries)
    int         severity;                    ///< RPT_SEVERITY_* (0x10=warning, 0x20=error, 0x40=ignore)
    KIID        item_a = niluuid;            ///< main item UUID — stable VALUE key (not a pointer)
    KIID        item_b = niluuid;            ///< aux item UUID (niluuid if none); for incremental DRC invalidation
};

/**
 * Bounding box returned by getBoardBBox().
 */
struct RLBoundingBox
{
    double x_mm, y_mm;          ///< origin (top-left) in mm
    double width_mm, height_mm; ///< size in mm
};

/**
 * Track-cleaner request / report.  Field-for-field mirrors of RL_CLEANUP_SPEC and
 * RL_CLEANUP_ITEM in tracks_cleaner_rl.h — mirrored rather than reused because this
 * header stays free of KiCad includes (see the forward declarations above).
 * toCleanupSpec() / fromCleanupItem() in pns_rl_router.cpp are the only translation
 * points; keep all three in step.
 */
struct RLCleanupSpec
{
    bool dry_run         = true;    ///< report only; board geometry untouched
    bool merge_segments  = false;   ///< collinear merge + duplicate + zero-length tracks
    bool clean_vias      = false;   ///< superimposed vias + vias on all-layer THT pads
    bool remove_shorts   = false;   ///< segments connecting two different nets
    bool tracks_in_pads  = false;   ///< tracks fully buried inside a pad
    bool dangling_tracks = false;   ///< tracks not connected at both ends
    bool dangling_vias   = false;   ///< vias connected on fewer than two layers
    std::vector<int> net_codes;     ///< nets the cleaner may touch (empty = all)
};

/// One cleanup operation.  UUIDs, not pointers: the item may already be freed.
struct RLCleanupItem
{
    int         code;               ///< RL_CLEANUP_CODE
    std::string code_name;          ///< stable snake_case name ("merge_tracks", …)
    KIID        item_a = niluuid;   ///< the removed / modified item
    KIID        item_b = niluuid;   ///< merge partner / shorting counterpart (may be nil)
};

struct RLCleanupResult
{
    bool                       ran = false;   ///< false ⇒ a precondition rejected the call
    std::string                reject_reason; ///< "routing_session_active" | "drag_session_active"
    std::vector<RLCleanupItem> items;         ///< one entry per operation, execution order
    std::vector<KIID>          removed;       ///< items gone from the board
    std::vector<KIID>          modified;      ///< items whose geometry changed in place
};


/**
 * One board-level graphic drawing (PCB_SHAPE) in exact integer nanometres, as
 * stored in the BOARD — arcs analytic (start/mid/end), nothing tessellated.
 * Returned by getGraphicShapes(); consumed by the load-time outline-simplify
 * pass (pcb_world/engine/outline_simplify.py), which plans a micro-segment →
 * arc/line rewrite and applies it via replaceGraphicShapes().
 *
 *   kind = 0: straight segment start→end (mid fields unused, 0)
 *   kind = 1: arc — start/mid/end, KiCad's native 3-point representation
 *   kind = 2: any other PCB_SHAPE (rect/circle/poly/bezier/…) — reported for
 *             completeness (coordinates zeroed); never eligible for rewrite.
 *
 * ``index`` is the item's position in BOARD::Drawings() at enumeration time;
 * it stays valid for replaceGraphicShapes() as long as the board is not
 * mutated in between.
 */
struct RLGraphicShape
{
    int       index;
    int       kind;
    long long x1_nm = 0, y1_nm = 0;   ///< start
    long long xm_nm = 0, ym_nm = 0;   ///< arc mid-point (kind==1 only)
    long long x2_nm = 0, y2_nm = 0;   ///< end
    long long width_nm = 0;           ///< stroke width
};


/**
 * One netclass entry exposed via getDesignRules().
 *
 * Values use -1.0 to mean "unset / inherit from Default" (KiCad stores these
 * as std::optional<int>; a missing optional maps to -1.0 here).
 */
struct RLNetClassInfo
{
    std::string name;               ///< netclass name (e.g. "Default", "phat")
    double      clearance_mm;       ///< min copper clearance for this class
    double      track_width_mm;     ///< default track width for this class
    double      via_diameter_mm;    ///< default via outer diameter
    double      via_drill_mm;       ///< default via drill diameter
    double      uvia_diameter_mm;   ///< default microvia outer diameter
    double      uvia_drill_mm;      ///< default microvia drill diameter
};


/**
 * Snapshot of the board's design rules exposed via getDesignRules().
 *
 * Structure mirrors KiCad's BOARD_DESIGN_SETTINGS + NET_SETTINGS:
 *   - Global minima (m_Min*, m_TrackMinWidth, m_ViasMin*, ...)
 *   - Track width / via size preset lists (custom widths the user can select)
 *   - Default netclass + any additional netclasses
 *
 * setDesignRules() only writes the **global minima** fields; preset lists
 * and netclass entries are ignored by the setter (treated as read-only).
 * This matches the common RL/LLM use case of tuning DRC thresholds without
 * reshaping the netclass hierarchy.
 */
struct RLDesignRules
{
    // ------------------ Global minima (writable) ----------------------
    double min_clearance_mm;            ///< BDS::m_MinClearance
    double min_track_width_mm;          ///< BDS::m_TrackMinWidth
    double min_via_diameter_mm;         ///< BDS::m_ViasMinSize
    double min_through_hole_mm;         ///< BDS::m_MinThroughDrill (via drill min)
    double min_via_annular_width_mm;    ///< BDS::m_ViasMinAnnularWidth
    double min_hole_to_hole_mm;         ///< BDS::m_HoleToHoleMin
    double min_uvia_diameter_mm;        ///< BDS::m_MicroViasMinSize
    double min_uvia_drill_mm;           ///< BDS::m_MicroViasMinDrill
    double copper_edge_clearance_mm;    ///< BDS::m_CopperEdgeClearance

    // ------------------ Presets (read-only) ---------------------------
    /// User-selectable track widths (BDS::m_TrackWidthList, index 0 excluded
    /// because it is the netclass-derived default, not a user preset).
    std::vector<double> track_width_presets_mm;

    /// User-selectable via (diameter, drill) pairs (BDS::m_ViasDimensionsList,
    /// index 0 excluded for the same reason).
    std::vector<std::pair<double, double>> via_presets_mm;

    // ------------------ Netclasses (read-only) ------------------------
    RLNetClassInfo              default_netclass;   ///< always present
    std::vector<RLNetClassInfo> netclasses;         ///< non-Default classes
};


/**
 * In-memory checkpoint of the router's full mutable state: committed board
 * items (tracks/vias/arcs) + engine config + active routing session. Stored
 * C++-side in PNS_RL_ROUTER and referenced from Python by an opaque integer
 * handle. Used for MCTS tree-search checkpoint/restore.
 */
struct RLCheckpointConfig
{
    int  routingMode;    ///< m_settings->Mode()             (PNS_MODE)
    int  cornerMode;     ///< m_settings->GetCornerMode()
    int  trackWidth;     ///< m_router->Sizes().TrackWidth()  (nm)
    int  viaDiameter;    ///< m_router->Sizes().ViaDiameter() (nm)
    int  viaDrill;       ///< m_router->Sizes().ViaDrill()    (nm)
    int  shoveIterLimit; ///< m_settings->ShoveIterationLimit() — runtime-mutable,
                         ///<   so it must travel with the checkpoint
    bool placingVia;     ///< m_router->IsPlacingVia()
};

struct RLCheckpointSession
{
    bool   routing;            ///< m_routing at checkpoint time
    double headX_mm, headY_mm; ///< route head position (mm)
    int    headBoardLayer;     ///< board layer id (native, not human)
    int    netCode;            ///< net being routed (-1 if idle)
};

struct RLCheckpoint
{
    std::vector<std::unique_ptr<BOARD_ITEM>> tracks;  ///< clones of every Tracks() item
    RLCheckpointConfig  config;
    RLCheckpointSession session;

    // DRC state as of checkpoint time, travelling with the board snapshot so an
    // incremental DRC after restore can diff/retain instead of recomputing from
    // scratch. Restored as a consistent pair (drcItemSig describes the board state
    // that produced drcViolations); see restore()/restoreIncremental().
    std::vector<RLDRCViolation>           drcViolations;
    std::unordered_map<KIID, std::string> drcItemSig;

    // Opaque snapshot of the process-global KIID (UUID) generator position at
    // checkpoint time. restore()/restoreIncremental() rewind the generator to this,
    // so a routing action replayed after a restore mints the same UUID stream a fresh
    // run would — without it, post-restore items get history-dependent UUIDs and the
    // UUID obstacle tie-break diverges (fresh routing is already deterministic).
    std::string kiidGenState;
};


class PNS_RL_ROUTER
{
public:
    // ------------------------------------------------------------------
    // Lifecycle
    // ------------------------------------------------------------------

    /**
     * Load a board from @a board_path and initialise the PNS router.
     *
     * If @a project_path is supplied, that .kicad_pro file is loaded and
     * attached to the BOARD so design rules (BDS + NetSettings) come from
     * the project. An empty @a project_path triggers auto-discovery:
     *   <board_stem>.kicad_pro is used if it exists; otherwise a blank
     *   in-memory PROJECT is created so the BOARD always has one attached.
     *
     * This "always attach a PROJECT" invariant is what makes legacy
     * design-setting migration work: the KiCad sexpr parser populates BDS
     * from legacy `(setup ...)` tokens, and BOARD::SetProject (called from
     * this ctor) preserves those values when no JSON project overrides them.
     * Without a PROJECT the BDS data would be silently dropped on save.
     *
     * Throws std::runtime_error on board load failure. Failure to read an
     * existing project file is non-fatal — the blank in-memory fallback is
     * used and a warning may be logged via wxLog.
     */
    /**
     * @param engineSeed if >= 0, seed KiCad's (process-global) KIID/UUID generator
     *        with this value at construction, BEFORE the board is loaded — making
     *        new-item UUIDs (routed tracks) deterministic. Routed-track UUIDs drive
     *        the PNS obstacle ordering, so a fixed seed makes routing — and
     *        UUID-keyed DRC — reproducible across runs/processes for a fixed action
     *        sequence. Decided once here and never re-seeded (re-seeding mid-run
     *        would replay the same UUID stream and risk collisions). -1 = default
     *        entropy seeding (non-reproducible across processes). Global, not
     *        per-router: with multiple engines per process, the last seed wins.
     */
    /**
     * @param shoveIterLimit   max shove iterations; shove is bounded by this count
     *        only (no wallclock timeout). Default 250.
     * @param followBranchIterLimit  max TOPOLOGY::followBranch DFS pops; bounded by
     *        this count only (no wallclock timeout). Default 1,000,000.
     */
    explicit PNS_RL_ROUTER( const std::string& board_path,
                            const std::string& project_path = "",
                            int engineSeed = -1,
                            int shoveIterLimit = 250,
                            int followBranchIterLimit = 1000000 );
    ~PNS_RL_ROUTER();

    // ------------------------------------------------------------------
    // Configuration (call before startRoute)
    // ------------------------------------------------------------------

    /**
     * Set the routing strategy.
     *   0 = RM_MarkObstacles  (ignore DRC, route over everything)
     *   1 = RM_Shove          (push existing traces out of the way)
     *   2 = RM_Walkaround     (hug around existing traces)  [default]
     */
    void setRoutingMode( int mode );

    /**
     * Set the corner mode (controls allowed track angles).
     *   0 = MITERED_45 (H/V/45, default)
     *   1 = ROUNDED_45
     *   2 = MITERED_90 (H/V only — 90-degree corners, no diagonals)
     *   3 = ROUNDED_90
     */
    void setCornerMode( int mode );

    /**
     * Set the track width for new segments (millimetres).
     * Pass 0 to let PNS pick the width from design rules.
     */
    void setTrackWidth( double width_mm );

    // ------------------------------------------------------------------
    // Board manipulation
    // ------------------------------------------------------------------

    /**
     * Remove the track segment on @a layer / @a net_code whose start/end
     * points match the given coordinates within @a tol_mm tolerance.
     * layer/net are REQUIRED: coordinate-only matching could remove the wrong
     * segment when same-shape routing exists on F/B, or another net's track
     * lies within tolerance.
     * Returns true if a track was found and removed.
     * Resyncs the PNS world automatically.
     */
    bool deleteTrackNear( double x1_mm, double y1_mm,
                          double x2_mm, double y2_mm,
                          int layer, int net_code,
                          double tol_mm = 0.1 );

    /**
     * Remove the track at the given list index (0-based, skipping VIAs).
     * Resyncs the PNS world automatically.
     */
    bool deleteTrackByIndex( int index );

    /**
     * Remove the via of @a net_code whose centre is closest to (x_mm, y_mm)
     * within @a tol_mm millimetres tolerance. net_code is REQUIRED:
     * coordinate-only matching could remove another net's via inside tol.
     * (No layer filter — vias span layers; blind/buried disambiguation is
     * not needed on this corpus.)
     * Returns true if a via was found and removed.
     * Resyncs the PNS world automatically.
     */
    bool deleteViaNear( double x_mm, double y_mm, int net_code,
                        double tol_mm = 0.1 );

    /**
     * Remove the via at the given list index (0-based, tracks excluded).
     * Resyncs the PNS world automatically.
     */
    bool deleteViaByIndex( int index );

    /**
     * Return the number of vias on the board.
     */
    int getViaCount() const;

    /**
     * Lock (or unlock) every track / via / arc belonging to @a netCode by
     * setting its BOARD_ITEM lock flag, then resync the PNS world so
     * SyncWorld re-marks the items PNS::MK_LOCKED. Locked copper is treated
     * as immovable by the shove engine (HasLockedSegments() → walk-around),
     * so a subsequent route of another net cannot push it — used to fix an
     * already-routed net in net-subset / staged routing. The board copper is
     * otherwise unchanged (still a physical obstacle). Meaningful only in
     * Shove mode; Walkaround never moves existing copper regardless.
     * Returns the number of items whose lock flag was set.
     */
    int lockNet( int netCode, bool locked = true );

    /**
     * Remove every track / via / arc whose net code is in @a netCodes, leaving
     * all other routing (and every pad) intact. Used by the env's net-aware
     * reset: only the nets being re-routed (the target subset) are wiped to a
     * clean slate; pre-routed nets that are NOT being re-routed are kept —
     * independently of whether they are locked (lock only decides shove
     * movability, not deletion). Resyncs the PNS world + ratsnest afterwards.
     * Returns the number of items removed. Empty @a netCodes removes nothing.
     */
    int deleteRoutingOfNets( const std::vector<int>& netCodes );

    /**
     * Run KiCad's track cleaner (RL fork, see tracks_cleaner_rl.h) over the board.
     *
     * QUIESCENT ONLY: rejected outright while a routing or drag session is open —
     * the cleaner mutates the BOARD behind the router's back, and silently
     * cancelling a caller's session would be worse than refusing. Check
     * RLCleanupResult::ran / reject_reason.
     *
     * A live run detaches the removed items, rebuilds the PNS world in full
     * (resyncWorld) + the ratsnest, and only then frees them — the deferred-free
     * invariant deleteRoutingOfNets() uses. The incremental-DRC baseline needs no
     * surgery: runDRCIncremental() diffs the board against m_drcItemSig by UUID, so
     * removed and geometry-changed items fall out of that diff on their own.
     *
     * Cleanup is UNDOABLE through checkpoint/restore: no pass mints a new UUID (a
     * merge edits the lower-UUID survivor in place and removes its partner), so a
     * checkpoint taken beforehand restores the exact pre-cleanup board.
     */
    RLCleanupResult cleanupTracks( const RLCleanupSpec& spec );

    // ------------------------------------------------------------------
    // Routing API  (StartRouting → Move* → FixRoute)
    // ------------------------------------------------------------------

    /**
     * Begin a new route starting at (x_mm, y_mm) on @a layer.
     * PNS will snap to the nearest pad / segment endpoint automatically.
     * Returns false if no suitable anchor was found at that position.
     */
    bool startRoute( double x_mm, double y_mm, int layer );

    /**
     * Move the route head to (x_mm, y_mm).
     * Call this repeatedly to "draw" the route.
     * Returns false if the router is not in an active routing state.
     */
    bool move( double x_mm, double y_mm );

    /**
     * Fix (commit) the current route at (x_mm, y_mm).
     * On success the board is updated and the world is resynced.
     * Returns true if the route was successfully committed.
     */
    /**
     * @param forceFinish  true  → complete route to nearest pad, commit to board
     *                     false → fix waypoint, keep routing from here
     */
    /**
     * @param arriveTolMm  arrival tolerance for the rejectIfStuck check, in mm.
     *                     0 (default) = exact coordinate match. A positive value
     *                     accepts a head that stopped within that distance of the
     *                     requested point — the same convention PNS itself uses
     *                     internally (LINE_PLACER compares against head width / 2),
     *                     where copper of the placed item still covers the target.
     */
    bool fixRoute( double x_mm, double y_mm, bool forceFinish = true,
                   bool rejectIfStuck = false, int expectedLayer = -1,
                   double arriveTolMm = 0.0,
                   bool requireVia = false );

    /**
     * Abort the current route without modifying the board.
     */
    void cancelRoute();

    // ------------------------------------------------------------------
    // Drag API  (StartDragging → Move* → FixDrag)
    // ------------------------------------------------------------------

    /**
     * Begin dragging the track item nearest to (x_mm, y_mm) on @a layer.
     * @param dragMode  bitmask of PNS::DRAG_MODE (default DM_ANY = 0x17)
     * Returns false if no suitable item was found.
     */
    bool startDrag( double x_mm, double y_mm, int layer, int dragMode = 0x17 );

    /** True while an interactive drag session is active. */
    bool isDragging() const { return m_dragging; }

    /**
     * Fix (commit) the current drag.
     * On success the board is updated and the world is resynced.
     */
    bool fixDrag( bool forceCommit = true );

    /** Abort the current drag without modifying the board. */
    void cancelDrag();

    // ------------------------------------------------------------------
    // Project / design-rule provenance
    // ------------------------------------------------------------------

    /**
     * Absolute path of the .kicad_pro associated with this router.
     *
     * This is always non-empty: either the path supplied to the constructor
     * or <board_stem>.kicad_pro derived automatically. It is the path that
     * save(pcb, pro) defaults to when the caller wants to round-trip.
     * The file at this path may not exist on disk (see
     * wasProjectLoadedFromFile()).
     */
    std::string getProjectPath() const { return m_projectPath; }

    /**
     * True when the .kicad_pro at getProjectPath() was successfully read
     * from disk. False means the attached PROJECT is a blank in-memory
     * fallback (the file was missing or failed to parse).
     *
     * Combined with wasLegacyDesignSettingsLoaded() this lets callers
     * tell whether design rules are authoritative:
     *   (true, *)        → project JSON is ground truth
     *   (false, true)    → legacy (setup ...) tokens filled BDS/NetSettings
     *   (false, false)   → BDS holds KiCad defaults, rules are meaningless
     */
    bool wasProjectLoadedFromFile() const { return m_projectLoadedFromFile; }

    /**
     * True when the .kicad_pcb contained legacy (pre-KiCad 6) setup tokens
     * that populated BDS/NetSettings during parsing. Proxies
     * BOARD::m_LegacyDesignSettingsLoaded.
     */
    bool wasLegacyDesignSettingsLoaded() const;

    // ------------------------------------------------------------------
    // Design rules (global minima + presets + netclasses)
    // ------------------------------------------------------------------

    /**
     * Snapshot the board's current design rules (BOARD_DESIGN_SETTINGS +
     * NET_SETTINGS) as an RLDesignRules struct.
     *
     * All values are in millimetres. Netclass fields that are unset in KiCad
     * (std::optional without a value) are reported as -1.0.
     */
    RLDesignRules getDesignRules() const;

    /**
     * Apply the **global minima** fields of @a rules to BOARD_DESIGN_SETTINGS
     * and resync the PNS router's size cache so subsequent routing honours
     * the new thresholds. Preset lists and netclass entries inside @a rules
     * are ignored (they remain read-only at this layer).
     *
     * Negative values are treated as "leave unchanged" to let callers update
     * a subset of fields without reading back the full struct first.
     */
    void setDesignRules( const RLDesignRules& rules );

    /**
     * Return the NetClass assigned to @a net_code, mirroring KiCad's own
     * resolution. Unassigned nets report the Default class. The returned
     * struct carries the class name plus per-class clearance / track width /
     * via sizes; fields the class leaves unset (inherit from Default) are
     * reported as -1.0, identical to @ref getDesignRules.
     *
     * Returns an empty info (@c name == "") if the lookup fails (no board
     * loaded, unknown net code, or null netclass pointer). Callers should
     * treat an empty info as "use the Default netclass" for the purposes of
     * parameter resolution.
     */
    RLNetClassInfo getNetClassForNet( int net_code ) const;

    // ------------------------------------------------------------------
    // DRC API
    // ------------------------------------------------------------------

    /**
     * Run a full DRC check on the current board state.
     * Returns the number of violations found.
     */
    std::vector<RLDRCViolation> runDRC( const std::string& rules_path = "" );

    /**
     * Incremental DRC: same result as runDRC() but recomputes only the clearance
     * violations involving tracks/vias changed since the last DRC (retaining the
     * rest); connectivity + per-item providers run in full. Falls back to a full
     * run on the first call (no baseline) or on boards with zones.
     */
    std::vector<RLDRCViolation> runDRCIncremental( const std::string& rules_path = "" );

    /** Number of violations from the last runDRC() call. */
    int getDRCViolationCount() const;

    /** All violations from the last runDRC() call. */
    std::vector<RLDRCViolation> getDRCViolations() const;

    /** Clear cached DRC violations from previous episode. */
    void clearDRCCache();

    /** DRC violations grouped by net name: {net_name → [error_type, ...]} (unique, preserves insertion order). */
    std::map<std::string, std::vector<std::string>> getDRCViolationsByNet() const;

    // ------------------------------------------------------------------
    // Observation / board state
    // ------------------------------------------------------------------

    /** Number of track segments (VIAs excluded). */
    int getTrackCount() const;

    /** All track segments as a vector of RLTrackInfo structs. */
    std::vector<RLTrackInfo> getTracks() const;

    /** All vias as a vector of RLViaInfo structs. */
    std::vector<RLViaInfo> getVias() const;

    /** All pads as a vector of RLPadInfo structs. */
    std::vector<RLPadInfo> getPads() const;

    /** Rule-area keepout zones (one entry per zone per copper layer). */
    std::vector<RLZoneInfo> getKeepouts() const;

    /** All footprints (components) with their courtyard outlines. */
    std::vector<RLFootprintInfo> getFootprints() const;

    /** Ratsnest (unrouted) edges across all nets. */
    std::vector<RLRatsnestEdge> getRatsnest() const;

    /** Pad-bearing connectivity clusters as {net_code, pad_count} pairs.
     *
     *  One entry per copper cluster that contains at least one pad; clusters
     *  made only of dangling track/via copper are omitted.  Grouping by
     *  net_code therefore gives both the net's pad count (sum of pad_count)
     *  and its number of distinct pad groups (number of entries) — the two
     *  quantities the routability metric is defined on.  Reflects COMMITTED
     *  board copper, same liveness contract as getRatsnest(). */
    std::vector<std::pair<int, int>> getPadClusters() const;

    /**
     * Anchor points of every board item electrically connected to the item at
     * (x_mm, y_mm, boardLayer) — the item's whole connectivity cluster,
     * including itself. Empty when no copper is at that position.
     *
     * This answers "what is already joined to me?" exactly, which the RL
     * candidate filter uses to drop redundant-loop targets. Reflects COMMITTED
     * board copper (BOARD::GetConnectivity()), so callers must have run
     * buildConnectivity() after their last mutation — the same liveness
     * contract as getRatsnest().
     */
    std::vector<RLClusterPoint> getConnectedPoints( double x_mm, double y_mm, int boardLayer );

    /** Board outline (Edge.Cuts) discretized to line segments. */
    std::vector<RLBoardEdge> getBoardOutline() const;
    std::vector<RLBoardOutlineShape> getBoardOutlineShapes() const;

    /** Number of unrouted connections (ratsnest edges). */
    int getUnroutedCount() const;

    /** True if the router is currently in an active routing session. */
    bool isRouting() const { return m_routing; }

    /** Route head position during active routing: {x_mm, y_mm, layer_id}.
     *  Returns {0, 0, -1} if not routing. */
    std::array<double, 3> getRouteHead() const;

    /** Net code of the net currently being routed. Returns -1 if not routing. */
    int getCurrentNetCode() const;

    /** Nearest ratsnest target for the active net: {x_mm, y_mm, layer_id}.
     *  Returns {0, 0, -1} if unavailable. */
    std::array<double, 3> getRoutingTarget() const;

    /** Work-in-progress trace segments from the active placer. */
    std::vector<RLTrackInfo> getWipSegments() const;

    // ------------------------------------------------------------------
    // Connectivity & Board Query
    // ------------------------------------------------------------------

    /** Rebuild board connectivity graph. Must call after track changes. */
    void buildConnectivity();

    /** Recalculate ratsnest (unrouted) edges. */
    void recalculateRatsnest();

    /** Number of nets in the connectivity graph. */
    int getNetCount() const;

    /** Number of nets on the board (from BOARD::GetNetCount). */
    int getBoardNetCount() const;

    /** Board bounding box in mm. */
    RLBoundingBox getBoardBBox() const;

    // ------------------------------------------------------------------
    // Board-level graphics read/replace (outline-simplify ingest support)
    // ------------------------------------------------------------------

    /** Board-level PCB_SHAPE drawings on aLayer — exact nm ints, arcs analytic. */
    std::vector<RLGraphicShape> getGraphicShapes( int aLayer ) const;

    /** Atomically delete drawings (by getGraphicShapes() index) and add new
     *  segments/arcs on aLayer, then rebuild connectivity + the PNS world like
     *  the initial load. Call right after construction, before configuring the
     *  router (the internal re-init resets ROUTING_SETTINGS to defaults).
     *  aNewSegments entries: {x1, y1, x2, y2, width} nm;
     *  aNewArcs entries: {x1, y1, xm, ym, x2, y2, width} nm (3-point arc).
     *  Also re-captures the episode-start KIID rewind point: the added shapes
     *  consume seeded UUIDs, and without the re-capture every episode reset
     *  would rewind to before the conversion and re-mint those same UUIDs for
     *  tracks/vias — colliding with the graphics added here. */
    void replaceGraphicShapes( int aLayer,
                               const std::vector<int>& aRemoveIndices,
                               const std::vector<std::array<long long, 5>>& aNewSegments,
                               const std::vector<std::array<long long, 7>>& aNewArcs );

    /** Number of copper layers (2 = two-sided, 4/6/8 = multi-layer). */
    int getCopperLayerCount() const;

    // ------------------------------------------------------------------
    // State & Failure Query
    // ------------------------------------------------------------------

    /**
     * Router state enum: 0=IDLE, 1=DRAG_SEGMENT, 2=DRAG_COMPONENT, 3=ROUTE_TRACK.
     */
    int getRouterState() const;

    /** Failure reason string from the last routing attempt. */
    std::string getFailureReason() const;

    // ------------------------------------------------------------------
    // Routing Control
    // ------------------------------------------------------------------

    /**
     * Auto-complete the route to the nearest unconnected anchor.
     *
     * maxAttempts is forwarded to ROUTER::Finish() as the bound on its internal
     * Move-to-convergence loop (triesLeft): the more attempts, the more shove
     * iterations it may spend reaching the anchor; deterministic modes converge
     * and stop early regardless. Commits and resyncs on success. Returns true
     * on success.
     */
    bool finish( int maxAttempts = 5 );

    /**
     * Undo the last fixed segment and return to the previous waypoint.
     * Returns true if undo was successful.
     */
    bool undoLastSegment();

    /** Flip posture (horizontal-first vs vertical-first). */
    void flipPosture();

    // ------------------------------------------------------------------
    // Via & Layer Control
    // ------------------------------------------------------------------

    /** Toggle via placement mode. Next fix will insert a via. */
    void toggleVia();

    /** Switch routing layer (inserts via automatically). */
    bool switchLayer( int layer );

    /** True if via placement mode is active. */
    bool isPlacingVia() const;

    /** Reset via placement mode to OFF. */
    void resetViaMode();

    /** Current routing layer ID (F.Cu=0, B.Cu=2). */
    int getCurrentLayer() const;

    /** Set via outer diameter in millimetres. */
    void setViaDiameter( double diameter_mm );

    /** Set via drill diameter in millimetres. */
    void setViaDrill( double drill_mm );

    // ------------------------------------------------------------------
    // I/O
    // ------------------------------------------------------------------

    /**
     * Save the board to @a output_path and always emit the attached
     * PROJECT as a .kicad_pro.
     *
     * When @a project_output_path is empty (default), the pro path is
     * derived from @a output_path as <output_stem>.kicad_pro alongside
     * the pcb. Supply an explicit path to place the pro file elsewhere.
     *
     * Keeping pcb and pro together is what preserves BDS + NetSettings
     * across reloads — the modern .kicad_pcb format intentionally strips
     * those fields on save.
     */
    void save( const std::string& output_path,
               const std::string& project_output_path = "" ) const;

    // ------------------------------------------------------------------
    // Checkpoint / Restore  (in-memory, MCTS tree search)
    // ------------------------------------------------------------------

    /**
     * Capture the full mutable router state (committed tracks + engine config
     * + active routing session) into an internal store and return an opaque
     * integer handle. Read-only w.r.t. the board/router (does NOT cancel an
     * active session). Handles are worker/router-local and become invalid once
     * this router is destroyed.
     */
    int64_t checkpoint();

    /**
     * Restore the state captured by @a handle: swap the board's tracks back to
     * the checkpointed set, re-apply engine config, rebuild the PNS world and
     * connectivity, and re-open the routing session if one was active at
     * checkpoint time. Returns true if restored, false if @a handle is unknown /
     * already released / reset (in which case the board is left unchanged) — let
     * the caller detect a stale handle instead of silently acting on it.
     */
    bool restore( int64_t handle );

    /**
     * Incremental variant of restore(). Diffs against the checkpoint and updates
     * only the changed tracks in the PNS world — `NODE::FindItemByParent`/`Remove`
     * for removed tracks, `PNS_RL_IFACE::addBoardItemToWorld` for added ones —
     * instead of a full `ClearWorld()+SyncWorld()`, keeping the invariant
     * pad/footprint obstacles in place (the ~26ms dominant cost). Produces the
     * same board as restore(), against which it is validated as the oracle.
     * Returns true if restored, false if @a handle is invalid (see restore()).
     */
    bool restoreIncremental( int64_t handle );

    /** Release checkpoint @a handle and free its cloned items. Idempotent. */
    void releaseCheckpoint( int64_t handle );

    /** Release ALL checkpoints at once (frees every clone + DRC state). Re-seeds
     *  the handle epoch from entropy, so every handle minted before the reset
     *  becomes permanently invalid (never aliases a post-reset handle). Intended
     *  for episode boundaries to bound memory. */
    void resetCheckpoints();

    /** True if @a handle refers to a live checkpoint. Use this to validate a
     *  handle before restore() in long-lived RL loops. Reliable because handles
     *  are globally unique (epoch+sequence), so a stale handle never aliases. */
    bool hasCheckpoint( int64_t handle ) const { return m_checkpoints.count( handle ) > 0; }

    /** Number of live checkpoints held (diagnostic / memory monitoring). */
    int checkpointCount() const { return static_cast<int>( m_checkpoints.size() ); }

    /** Rewind the process-global KIID/UUID generator to the position captured at
     *  construction (post board-load, pre-routing), so every episode after reset()
     *  mints the SAME UUID stream — making per-episode routing + UUID-keyed DRC
     *  reproducible. No-op when the engine was constructed with entropy seeding
     *  (engineSeed < 0). Collision-safe the same way restore() is (step 7): pre-load
     *  items keep their file UUIDs (drawn before the captured position) and routed
     *  tracks are deleted before the rewind, so the re-issued stream never aliases a
     *  live item. Intended to be the LAST engine call in env.reset(). */
    void rewindKIIDToEpisodeStart();

    /** Rebuild the PNS world from the current board (ClearWorld + SyncWorld).
     *  RAW: does NOT touch an active routing/drag session — a live placer would
     *  keep pointers into the freed world, so callers with an open session must
     *  cancelRoute()/cancelDrag() first (the Python binding does this). */
    void resyncWorld();

    /** Introspection counters of the current PNS world (NODE): joint count,
     *  branch depth, per-kind unique item counts and joint-link stats gathered
     *  via QueryJoints over the whole board area. Diagnostic only. */
    std::map<std::string, long long> worldStats();

    /** Change the shove iteration bound at runtime. SHOVE reads the settings
     *  value live on each Run (pns_shove.cpp shoveMainLoop), so this takes
     *  effect mid-session — e.g. right before a single fixRoute call. */
    void setShoveIterationLimit( int limit );
    int  getShoveIterationLimit() const;

private:
    void initRouter();
    int boardToPnsLayer( int boardLayer ) const;
    int pnsToBoardLayer( int pnsLayer ) const;
    PNS::ITEM* itemAt( const VECTOR2I& pos, int boardLayer );

    // Declaration order matters for destruction: SETTINGS_MANAGER (declared
    // first) is destroyed LAST so it outlives m_board, which holds a raw
    // PROJECT* pointer into it. The dtor explicitly deletes m_board before
    // this unique_ptr is released to guarantee the invariant.
    std::unique_ptr<SETTINGS_MANAGER>      m_settingsMgr;
    PROJECT*                               m_project  = nullptr;  ///< owned by m_settingsMgr
    std::string                            m_projectPath;          ///< path associated with m_project
    bool                                   m_projectLoadedFromFile = false; ///< true if .kicad_pro was read from disk

    // DRC internals (incremental clearance).
    void runDRCEngine( const std::string& rules_path,
                       const std::vector<BOARD_ITEM*>* clearanceScope );
    void snapshotDrcItems();

    BOARD*                                m_board    = nullptr;
    std::unique_ptr<PNS_RL_IFACE>         m_iface;
    std::unique_ptr<PNS::ROUTER>          m_router;
    std::unique_ptr<PNS::ROUTING_SETTINGS> m_settings;
    int                                   m_shoveIterLimit = 250;        // applied in initRouter()
    int                                   m_followBranchIterLimit = 1000000;
    bool                                  m_routing  = false;
    bool                                  m_dragging = false;
    std::vector<RLDRCViolation>           m_drcViolations;
    std::unordered_map<KIID, std::string> m_drcItemSig;   // last-DRC item signatures (incremental diff)
    std::string                           m_lastDrcRulesPath;  // rules file the baseline was computed
                                                               // under — mismatch forces a full DRC

    // Checkpoint store (MCTS). Handles are keys into this map; the heavy cloned
    // BOARD_ITEMs live here, owned by the router. A handle is (epoch << 32 | seq):
    //   - seq  is monotonic within this router → no collision within one run;
    //   - epoch is entropy+time seeded at construction and re-seeded on reset →
    //     handles are globally unique across resets / engine instances / processes,
    //     so a stale handle never aliases a live one (hasCheckpoint stays reliable).
    std::unordered_map<int64_t, RLCheckpoint> m_checkpoints;
    uint32_t                                  m_nextCheckpointSeq = 1;
    uint32_t                                  m_checkpointEpoch   = 0;
    void     reseedCheckpointEpoch();   ///< new entropy+time epoch (ctor + reset)

    // KIID/UUID generator position captured at construction (post board-load,
    // pre-routing). Empty under entropy seeding (engineSeed < 0). Restored by
    // rewindKIIDToEpisodeStart() at every episode reset so routing draws an identical
    // UUID stream each episode. Opaque mt19937 blob — same encoding as RLCheckpoint.
    std::string                               m_episodeStartKiidState;
};

#endif // PNS_RL_ROUTER_H
