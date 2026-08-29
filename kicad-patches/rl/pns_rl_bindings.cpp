/*
 * pns_rl_bindings.cpp
 *
 * pybind11 Python module: kicad_rl_router
 *
 * Exposes PNS_RL_ROUTER to Python so that RL agents can:
 *   - load a KiCad board
 *   - delete / inspect existing tracks
 *   - run interactive PNS routing (shove / walkaround / mark-obstacles)
 *   - query connectivity, board state, and DRC
 *   - control via placement and layer switching
 *   - save the result
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <layer_ids.h>   // Edge_Cuts — exported as m.attr("LAYER_EDGE_CUTS")
#include <thread_pool.h>

#include <wx/debug.h>
#include <wx/string.h>

#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <unordered_map>

#include <kiid.h>   // module-level KIID generator-state get/set (diagnostics)

#include "../../router/pns_rl_router.h"

namespace py = pybind11;


#if wxDEBUG_LEVEL
// ------------------------------------------------------------------
// wx soft-assert dedup
//
// KiCad's wxASSERTs are non-fatal warnings that fire per inner-loop call in
// headless routing (PCB_VIA::GetWidth, pns_node AssembleLine, ...), flooding
// stderr at GB scale and costing a format+write each. This handler keeps the
// signal — the FIRST occurrence per (file, line) is printed, so a NEW assert
// kind is always visible — and suppresses the repeats to a counted hash-map
// lookup. Execution continues exactly as with the default handler.
// KICAD_WX_ASSERT=full restores the stock full-spam handler (debugging).
// Suppressed totals: kicad_rl_router.wx_assert_stats().
// ------------------------------------------------------------------
static std::mutex s_assertMutex;
static std::unordered_map<std::string, unsigned long long> s_assertCounts;

static void dedupAssertHandler( const wxString& file, int line, const wxString& func,
                                const wxString& cond, const wxString& msg )
{
    std::string key = std::string( file.utf8_str() ) + ":" + std::to_string( line );
    unsigned long long n;
    {
        std::lock_guard<std::mutex> lock( s_assertMutex );
        n = ++s_assertCounts[key];
    }
    if( n == 1 )
    {
        fprintf( stderr,
                 "[wx-assert] %s %s(): %s — %s (repeats suppressed; totals via "
                 "kicad_rl_router.wx_assert_stats())\n",
                 key.c_str(), (const char*) func.utf8_str(),
                 (const char*) cond.utf8_str(), (const char*) msg.utf8_str() );
        fflush( stderr );
    }
}
#endif // wxDEBUG_LEVEL


PYBIND11_MODULE( kicad_rl_router, m )
{
    m.doc() = "KiCad PNS router binding for RL environments";

#if wxDEBUG_LEVEL
    {
        const char* mode = std::getenv( "KICAD_WX_ASSERT" );

        if( !mode || std::strcmp( mode, "full" ) != 0 )
            wxSetAssertHandler( &dedupAssertHandler );
    }

    m.def( "wx_assert_stats",
           []()
           {
               std::lock_guard<std::mutex> lock( s_assertMutex );
               return s_assertCounts;
           },
           "Suppressed wx-assert counts per file:line (dedup handler; "
           "empty when KICAD_WX_ASSERT=full)." );
#endif

    // ------------------------------------------------------------------
    // KiCad thread pool control
    //
    // DRC / build_connectivity bursts run on the process-global
    // GetKiCadThreadPool(), which defaults to hardware_concurrency()
    // threads. With N parallel env workers that oversubscribes the host
    // (N workers x nproc threads), so callers cap it explicitly — see
    // pcb_world/engine/utils.py (apply_thread_pool_cap). The
    // kicad_advanced MaximumThreads config knob is NOT usable here:
    // ADVANCED_CFG::loadFromConfigFile() bails out when !wxTheApp, i.e.
    // headless bindings never read that file — hence this explicit API.
    // ------------------------------------------------------------------
    m.def( "set_thread_pool_size",
           []( int n )
           {
               if( n < 1 )
                   throw py::value_error( "thread pool size must be >= 1" );

               GetKiCadThreadPool().reset( static_cast<unsigned int>( n ) );
           },
           py::arg( "n" ),
           "Resize the process-global KiCad thread pool (DRC/connectivity). "
           "Default size is std::thread::hardware_concurrency()." );

    m.def( "get_thread_pool_size",
           []()
           {
               return static_cast<int>( GetKiCadThreadPool().get_thread_count() );
           },
           "Current KiCad thread pool size." );

    // ------------------------------------------------------------------
    // RLPadInfo
    // ------------------------------------------------------------------
    py::class_<RLPadInfo>( m, "PadInfo" )
        .def_readonly( "x_mm",          &RLPadInfo::x_mm )
        .def_readonly( "y_mm",          &RLPadInfo::y_mm )
        .def_readonly( "width_mm",      &RLPadInfo::width_mm )
        .def_readonly( "height_mm",     &RLPadInfo::height_mm )
        .def_readonly( "layer",         &RLPadInfo::layer )
        .def_readonly( "net_code",      &RLPadInfo::net_code )
        .def_readonly( "net_name",      &RLPadInfo::net_name )
        .def_readonly( "pad_name",      &RLPadInfo::pad_name )
        .def_readonly( "footprint_ref", &RLPadInfo::footprint_ref )
        .def_readonly( "pad_type",      &RLPadInfo::pad_type )
        .def_readonly( "shape",         &RLPadInfo::shape )
        .def( "__repr__", []( const RLPadInfo& p )
        {
            return "<PadInfo " + p.footprint_ref + ":" + p.pad_name
                + " (" + std::to_string( p.x_mm ) + "," + std::to_string( p.y_mm )
                + ") net=" + p.net_name + ">";
        } );

    // ------------------------------------------------------------------
    // RLZoneInfo
    // ------------------------------------------------------------------
    py::class_<RLZoneInfo>( m, "ZoneInfo" )
        .def_readonly( "pts",            &RLZoneInfo::pts )   // list[(x_mm, y_mm)]
        .def_readonly( "layer",          &RLZoneInfo::layer )
        .def_readonly( "keepout_tracks", &RLZoneInfo::keepout_tracks )
        .def_readonly( "keepout_vias",   &RLZoneInfo::keepout_vias )
        .def_readonly( "keepout_pads",   &RLZoneInfo::keepout_pads )
        .def_readonly( "name",           &RLZoneInfo::name )
        .def( "__repr__", []( const RLZoneInfo& z )
        {
            return "<ZoneInfo name=" + z.name + " layer=" + std::to_string( z.layer )
                + " pts=" + std::to_string( z.pts.size() )
                + " tracks=" + ( z.keepout_tracks ? "no" : "ok" ) + ">";
        } );

    // ------------------------------------------------------------------
    // RLFootprintInfo
    // ------------------------------------------------------------------
    py::class_<RLFootprintInfo>( m, "FootprintInfo" )
        .def_readonly( "ref",             &RLFootprintInfo::ref )
        .def_readonly( "value",           &RLFootprintInfo::value )
        .def_readonly( "fpid",            &RLFootprintInfo::fpid )
        .def_readonly( "x_mm",            &RLFootprintInfo::x_mm )
        .def_readonly( "y_mm",            &RLFootprintInfo::y_mm )
        .def_readonly( "orientation_deg", &RLFootprintInfo::orientation_deg )
        .def_readonly( "flipped",         &RLFootprintInfo::flipped )
        .def_readonly( "layer",           &RLFootprintInfo::layer )
        // list[list[(x_mm, y_mm)]] — one closed contour per entry, empty when
        // the footprint declares no courtyard.
        .def_readonly( "courtyard",       &RLFootprintInfo::courtyard )
        .def( "__repr__", []( const RLFootprintInfo& f )
        {
            return "<FootprintInfo ref=" + f.ref + " value=" + f.value
                + " courtyard=" + std::to_string( f.courtyard.size() ) + ">";
        } );

    // ------------------------------------------------------------------
    // RLBoardEdge
    // ------------------------------------------------------------------
    py::class_<RLBoardEdge>( m, "BoardEdge" )
        .def_readonly( "x1_mm",    &RLBoardEdge::x1_mm )
        .def_readonly( "y1_mm",    &RLBoardEdge::y1_mm )
        .def_readonly( "x2_mm",    &RLBoardEdge::x2_mm )
        .def_readonly( "y2_mm",    &RLBoardEdge::y2_mm )
        .def_readonly( "width_mm", &RLBoardEdge::width_mm )
        .def( "__repr__", []( const RLBoardEdge& e )
        {
            return "<BoardEdge ("
                + std::to_string( e.x1_mm ) + "," + std::to_string( e.y1_mm )
                + ")->(" + std::to_string( e.x2_mm ) + "," + std::to_string( e.y2_mm )
                + ")>";
        } );

    // ------------------------------------------------------------------
    // RLBoardOutlineShape
    // ------------------------------------------------------------------
    py::class_<RLBoardOutlineShape>( m, "BoardOutlineShape" )
        .def_readonly( "kind",     &RLBoardOutlineShape::kind )
        .def_readonly( "x1_mm",    &RLBoardOutlineShape::x1_mm )
        .def_readonly( "y1_mm",    &RLBoardOutlineShape::y1_mm )
        .def_readonly( "x2_mm",    &RLBoardOutlineShape::x2_mm )
        .def_readonly( "y2_mm",    &RLBoardOutlineShape::y2_mm )
        .def_readonly( "x3_mm",    &RLBoardOutlineShape::x3_mm )
        .def_readonly( "y3_mm",    &RLBoardOutlineShape::y3_mm )
        .def_readonly( "width_mm", &RLBoardOutlineShape::width_mm )
        .def( "__repr__", []( const RLBoardOutlineShape& s )
        {
            static const char* kinds[] = { "seg", "arc", "circle" };
            const char* k = ( s.kind >= 0 && s.kind <= 2 ) ? kinds[s.kind] : "?";
            return "<BoardOutlineShape " + std::string( k ) + " ("
                + std::to_string( s.x1_mm ) + "," + std::to_string( s.y1_mm )
                + ")->(" + std::to_string( s.x2_mm ) + "," + std::to_string( s.y2_mm )
                + ")>";
        } );

    // ------------------------------------------------------------------
    // RLRatsnestEdge
    // ------------------------------------------------------------------
    py::class_<RLRatsnestEdge>( m, "RatsnestEdge" )
        .def_readonly( "x1_mm",    &RLRatsnestEdge::x1_mm )
        .def_readonly( "y1_mm",    &RLRatsnestEdge::y1_mm )
        .def_readonly( "x2_mm",    &RLRatsnestEdge::x2_mm )
        .def_readonly( "y2_mm",    &RLRatsnestEdge::y2_mm )
        .def_readonly( "net_code", &RLRatsnestEdge::net_code )
        .def_readonly( "layer1",   &RLRatsnestEdge::layer1,
                       "source anchor copper layer (PCB_LAYER_ID; -2 = spans layers/thru)" )
        .def_readonly( "layer2",   &RLRatsnestEdge::layer2,
                       "target anchor copper layer (PCB_LAYER_ID; -2 = spans layers/thru)" )
        .def( "__repr__", []( const RLRatsnestEdge& e )
        {
            return "<RatsnestEdge ("
                + std::to_string( e.x1_mm ) + "," + std::to_string( e.y1_mm )
                + ")->(" + std::to_string( e.x2_mm ) + "," + std::to_string( e.y2_mm )
                + ") net=" + std::to_string( e.net_code ) + ">";
        } );

    // ------------------------------------------------------------------
    // RLClusterPoint
    // ------------------------------------------------------------------
    py::class_<RLClusterPoint>( m, "ClusterPoint" )
        .def_readonly( "x_mm",  &RLClusterPoint::x_mm )
        .def_readonly( "y_mm",  &RLClusterPoint::y_mm )
        .def_readonly( "layer", &RLClusterPoint::layer )
        .def( "__repr__", []( const RLClusterPoint& p )
        {
            return "<ClusterPoint ("
                + std::to_string( p.x_mm ) + "," + std::to_string( p.y_mm )
                + ") layer=" + std::to_string( p.layer ) + ">";
        } );

    // ------------------------------------------------------------------
    // RLViaInfo
    // ------------------------------------------------------------------
    py::class_<RLViaInfo>( m, "ViaInfo" )
        .def_readonly( "x_mm",          &RLViaInfo::x_mm )
        .def_readonly( "y_mm",          &RLViaInfo::y_mm )
        .def_readonly( "diameter_mm",   &RLViaInfo::diameter_mm )
        .def_readonly( "drill_mm",      &RLViaInfo::drill_mm )
        .def_readonly( "top_layer",     &RLViaInfo::top_layer )
        .def_readonly( "bottom_layer",  &RLViaInfo::bottom_layer )
        .def_readonly( "net_code",      &RLViaInfo::net_code )
        .def_readonly( "net_name",      &RLViaInfo::net_name )
        .def_property_readonly( "uuid", []( const RLViaInfo& v )
            { return v.uuid.AsStdString(); } )
        .def( "__repr__", []( const RLViaInfo& v )
        {
            return "<ViaInfo ("
                + std::to_string( v.x_mm ) + "," + std::to_string( v.y_mm )
                + ") layers=" + std::to_string( v.top_layer )
                + "-" + std::to_string( v.bottom_layer )
                + " net=" + v.net_name + ">";
        } );

    // ------------------------------------------------------------------
    // RLTrackInfo
    // ------------------------------------------------------------------
    py::class_<RLTrackInfo>( m, "TrackInfo" )
        .def_readonly( "x1_mm",    &RLTrackInfo::x1_mm )
        .def_readonly( "y1_mm",    &RLTrackInfo::y1_mm )
        .def_readonly( "x2_mm",    &RLTrackInfo::x2_mm )
        .def_readonly( "y2_mm",    &RLTrackInfo::y2_mm )
        .def_readonly( "width_mm", &RLTrackInfo::width_mm )
        .def_readonly( "layer",    &RLTrackInfo::layer )
        .def_readonly( "net_code", &RLTrackInfo::net_code )
        .def_readonly( "net_name", &RLTrackInfo::net_name )
        .def_property_readonly( "uuid", []( const RLTrackInfo& t )
            { return t.uuid.AsStdString(); } )
        .def( "__repr__", []( const RLTrackInfo& t )
        {
            return "<TrackInfo ("
                + std::to_string( t.x1_mm ) + "," + std::to_string( t.y1_mm )
                + ")->("
                + std::to_string( t.x2_mm ) + "," + std::to_string( t.y2_mm )
                + ") net=" + t.net_name + ">";
        } );

    // ------------------------------------------------------------------
    // RLDRCViolation
    // ------------------------------------------------------------------
    py::class_<RLDRCViolation>( m, "DRCViolation" )
        .def_readonly( "error_code", &RLDRCViolation::error_code )
        .def_readonly( "error_type", &RLDRCViolation::error_type )
        .def_readonly( "message",    &RLDRCViolation::message )
        .def_readonly( "x_mm",       &RLDRCViolation::x_mm )
        .def_readonly( "y_mm",       &RLDRCViolation::y_mm )
        .def_readonly( "layer",      &RLDRCViolation::layer )
        .def_readonly( "net_names",  &RLDRCViolation::net_names )
        .def_readonly( "severity",   &RLDRCViolation::severity )
        .def_property_readonly( "item_a", []( const RLDRCViolation& v )
            { return v.item_a.AsStdString(); } )
        .def_property_readonly( "item_b", []( const RLDRCViolation& v )
            { return v.item_b.AsStdString(); } )
        .def( "__repr__", []( const RLDRCViolation& v )
        {
            std::string nets;
            for( size_t i = 0; i < v.net_names.size(); ++i )
            {
                if( i ) nets += ",";
                nets += v.net_names[i];
            }
            return "<DRCViolation [" + v.error_type + "] sev=" + std::to_string( v.severity )
                + " " + v.message + " at ("
                + std::to_string( v.x_mm ) + "," + std::to_string( v.y_mm )
                + ") layer=" + std::to_string( v.layer )
                + " nets=[" + nets + "]>";
        } );

    // ------------------------------------------------------------------
    // Track cleaner (RLCleanupItem / RLCleanupResult)
    // ------------------------------------------------------------------
    py::class_<RLCleanupItem>( m, "CleanupItem" )
        .def_readonly( "code",      &RLCleanupItem::code )
        .def_readonly( "code_name", &RLCleanupItem::code_name )
        .def_property_readonly( "item_a", []( const RLCleanupItem& i )
            { return i.item_a.AsStdString(); } )
        .def_property_readonly( "item_b", []( const RLCleanupItem& i )
            { return i.item_b.AsStdString(); } )
        .def( "__repr__", []( const RLCleanupItem& i )
        {
            return "<CleanupItem " + i.code_name + " " + i.item_a.AsStdString() + ">";
        } );

    py::class_<RLCleanupResult>( m, "CleanupResult" )
        .def_readonly( "ran",           &RLCleanupResult::ran )
        .def_readonly( "reject_reason", &RLCleanupResult::reject_reason )
        .def_readonly( "items",         &RLCleanupResult::items )
        .def_property_readonly( "removed", []( const RLCleanupResult& r )
        {
            std::vector<std::string> out;
            for( const KIID& u : r.removed )
                out.push_back( u.AsStdString() );
            return out;
        } )
        .def_property_readonly( "modified", []( const RLCleanupResult& r )
        {
            std::vector<std::string> out;
            for( const KIID& u : r.modified )
                out.push_back( u.AsStdString() );
            return out;
        } )
        .def( "__repr__", []( const RLCleanupResult& r )
        {
            if( !r.ran )
                return std::string( "<CleanupResult rejected: " ) + r.reject_reason + ">";

            return "<CleanupResult items=" + std::to_string( r.items.size() )
                + " removed=" + std::to_string( r.removed.size() )
                + " modified=" + std::to_string( r.modified.size() ) + ">";
        } );

    // ------------------------------------------------------------------
    // RLNetClassInfo
    // ------------------------------------------------------------------
    // Read/write: the getter fills this from KiCad's NETCLASS. The setter on
    // RLRouter currently ignores netclass updates (global minima only), so
    // writable fields here are for symmetry with RLDesignRules round-tripping.
    py::class_<RLNetClassInfo>( m, "NetClassInfo" )
        .def( py::init<>() )
        .def_readwrite( "name",             &RLNetClassInfo::name )
        .def_readwrite( "clearance_mm",     &RLNetClassInfo::clearance_mm )
        .def_readwrite( "track_width_mm",   &RLNetClassInfo::track_width_mm )
        .def_readwrite( "via_diameter_mm",  &RLNetClassInfo::via_diameter_mm )
        .def_readwrite( "via_drill_mm",     &RLNetClassInfo::via_drill_mm )
        .def_readwrite( "uvia_diameter_mm", &RLNetClassInfo::uvia_diameter_mm )
        .def_readwrite( "uvia_drill_mm",    &RLNetClassInfo::uvia_drill_mm )
        .def( "__repr__", []( const RLNetClassInfo& n )
        {
            return "<NetClassInfo '" + n.name + "' clr="
                + std::to_string( n.clearance_mm ) + " tw="
                + std::to_string( n.track_width_mm ) + " via="
                + std::to_string( n.via_diameter_mm ) + "/"
                + std::to_string( n.via_drill_mm ) + ">";
        } );

    // ------------------------------------------------------------------
    // RLDesignRules
    // ------------------------------------------------------------------
    // Round-trip struct for get_design_rules / set_design_rules.
    // Only the `min_*` + `copper_edge_clearance_mm` fields are written by the
    // setter; preset lists and netclass entries are read-only snapshots.
    py::class_<RLDesignRules>( m, "DesignRules" )
        .def( py::init<>() )
        // Global minima (writable via set_design_rules)
        .def_readwrite( "min_clearance_mm",         &RLDesignRules::min_clearance_mm )
        .def_readwrite( "min_track_width_mm",       &RLDesignRules::min_track_width_mm )
        .def_readwrite( "min_via_diameter_mm",      &RLDesignRules::min_via_diameter_mm )
        .def_readwrite( "min_through_hole_mm",      &RLDesignRules::min_through_hole_mm )
        .def_readwrite( "min_via_annular_width_mm", &RLDesignRules::min_via_annular_width_mm )
        .def_readwrite( "min_hole_to_hole_mm",      &RLDesignRules::min_hole_to_hole_mm )
        .def_readwrite( "min_uvia_diameter_mm",     &RLDesignRules::min_uvia_diameter_mm )
        .def_readwrite( "min_uvia_drill_mm",        &RLDesignRules::min_uvia_drill_mm )
        .def_readwrite( "copper_edge_clearance_mm", &RLDesignRules::copper_edge_clearance_mm )
        // Presets + netclasses (read-only snapshot; setter ignores)
        .def_readwrite( "track_width_presets_mm",   &RLDesignRules::track_width_presets_mm )
        .def_readwrite( "via_presets_mm",           &RLDesignRules::via_presets_mm )
        .def_readwrite( "default_netclass",         &RLDesignRules::default_netclass )
        .def_readwrite( "netclasses",               &RLDesignRules::netclasses )
        .def( "__repr__", []( const RLDesignRules& r )
        {
            return "<DesignRules min_clr=" + std::to_string( r.min_clearance_mm )
                + " min_tw=" + std::to_string( r.min_track_width_mm )
                + " min_via=" + std::to_string( r.min_via_diameter_mm )
                + " netclasses=" + std::to_string( r.netclasses.size() + 1 ) + ">";
        } );

    // ------------------------------------------------------------------
    // RLBoundingBox
    // ------------------------------------------------------------------
    py::class_<RLBoundingBox>( m, "BoundingBox" )
        .def_readonly( "x_mm",      &RLBoundingBox::x_mm )
        .def_readonly( "y_mm",      &RLBoundingBox::y_mm )
        .def_readonly( "width_mm",  &RLBoundingBox::width_mm )
        .def_readonly( "height_mm", &RLBoundingBox::height_mm )
        .def( "__repr__", []( const RLBoundingBox& b )
        {
            return "<BoundingBox (" + std::to_string( b.x_mm ) + ","
                + std::to_string( b.y_mm ) + ") "
                + std::to_string( b.width_mm ) + "x"
                + std::to_string( b.height_mm ) + " mm>";
        } );

    // Authoritative PCB_LAYER_ID values for the two graphic layers the PNS
    // world syncs as full-stack unroutable obstacles (syncGraphicalItem).
    // The enum value and the obs token space disagree — Edge_Cuts is 25 in
    // PCB_LAYER_ID, while the obs token space uses 44 — so Python callers must
    // read these instead of hardcoding either number.
    m.attr( "LAYER_EDGE_CUTS" ) = static_cast<int>( Edge_Cuts );
    m.attr( "LAYER_MARGIN" )    = static_cast<int>( Margin );

    // ------------------------------------------------------------------
    // RLGraphicShape
    // ------------------------------------------------------------------
    py::class_<RLGraphicShape>( m, "GraphicShape" )
        .def_readonly( "index",    &RLGraphicShape::index )
        .def_readonly( "kind",     &RLGraphicShape::kind )
        .def_readonly( "x1_nm",    &RLGraphicShape::x1_nm )
        .def_readonly( "y1_nm",    &RLGraphicShape::y1_nm )
        .def_readonly( "xm_nm",    &RLGraphicShape::xm_nm )
        .def_readonly( "ym_nm",    &RLGraphicShape::ym_nm )
        .def_readonly( "x2_nm",    &RLGraphicShape::x2_nm )
        .def_readonly( "y2_nm",    &RLGraphicShape::y2_nm )
        .def_readonly( "width_nm", &RLGraphicShape::width_nm )
        .def( "__repr__", []( const RLGraphicShape& g )
        {
            static const char* kinds[] = { "seg", "arc", "other" };
            const char* k = ( g.kind >= 0 && g.kind <= 2 ) ? kinds[g.kind] : "?";
            return "<GraphicShape #" + std::to_string( g.index ) + " "
                + std::string( k ) + " ("
                + std::to_string( g.x1_nm ) + "," + std::to_string( g.y1_nm )
                + ")->(" + std::to_string( g.x2_nm ) + "," + std::to_string( g.y2_nm )
                + ") nm>";
        } );

    // ------------------------------------------------------------------
    // PNS_RL_ROUTER  (exposed as kicad_rl_router.RLRouter)
    // ------------------------------------------------------------------
    py::class_<PNS_RL_ROUTER>( m, "RLRouter" )
        .def( py::init<const std::string&, const std::string&, int, int, int>(),
              py::arg( "board_path" ), py::arg( "project_path" ) = "",
              py::arg( "engine_seed" ) = -1,
              py::arg( "shove_iter_limit" ) = 250,
              py::arg( "followbranch_iter_limit" ) = 1000000,
              "Load a KiCad PCB file and initialise the PNS router. "
              "An empty project_path auto-discovers <stem>.kicad_pro; if the "
              "file is missing a blank in-memory PROJECT is attached so design "
              "rules are always consistent. engine_seed >= 0 seeds the (global) KIID "
              "generator at construction for reproducible routing / UUID-keyed DRC "
              "(decided once at init; -1 = default entropy seeding). "
              "shove_iter_limit (250) / followbranch_iter_limit (1,000,000) bound the "
              "shove loop and TOPOLOGY::followBranch DFS. They are iteration counts, "
              "not wallclock timeouts, so a given board truncates at the same point "
              "on every run." )

        // -- Project / design-rule provenance --
        .def( "get_project_path", &PNS_RL_ROUTER::getProjectPath,
              "Absolute path of the .kicad_pro associated with this router "
              "(whether or not the file exists on disk)." )
        .def( "was_project_loaded_from_file", &PNS_RL_ROUTER::wasProjectLoadedFromFile,
              "True if the .kicad_pro at get_project_path() was read from disk; "
              "False means an in-memory blank project was used as fallback." )
        .def( "was_legacy_design_settings_loaded", &PNS_RL_ROUTER::wasLegacyDesignSettingsLoaded,
              "True if the .kicad_pcb contained legacy (pre-KiCad 6) setup "
              "tokens that populated BDS/NetSettings during parsing." )

        // -- Configuration --
        .def( "set_routing_mode", &PNS_RL_ROUTER::setRoutingMode, py::arg( "mode" ),
              "Routing strategy: 0=MarkObstacles, 1=Shove, 2=Walkaround (default)." )
        .def( "set_corner_mode", &PNS_RL_ROUTER::setCornerMode, py::arg( "mode" ),
              "Corner mode: 0=MITERED_45 (default), 1=ROUNDED_45, "
              "2=MITERED_90 (no diagonals), 3=ROUNDED_90." )
        .def( "set_track_width", &PNS_RL_ROUTER::setTrackWidth, py::arg( "width_mm" ),
              "Track width in millimetres (0 = use design rules)." )

        // -- Board manipulation --
        .def( "delete_track_near", &PNS_RL_ROUTER::deleteTrackNear,
              py::arg( "x1_mm" ), py::arg( "y1_mm" ),
              py::arg( "x2_mm" ), py::arg( "y2_mm" ),
              py::arg( "layer" ), py::arg( "net_code" ),
              py::arg( "tol_mm" ) = 0.1,
              "Remove the track segment on layer (PCB_LAYER_ID) / net_code whose "
              "endpoints match the given coords within tol." )
        .def( "delete_track_by_index", &PNS_RL_ROUTER::deleteTrackByIndex,
              py::arg( "index" ),
              "Remove the track at list index (0-based, VIAs excluded)." )
        .def( "delete_via_near", &PNS_RL_ROUTER::deleteViaNear,
              py::arg( "x_mm" ), py::arg( "y_mm" ), py::arg( "net_code" ),
              py::arg( "tol_mm" ) = 0.1,
              "Remove the via of net_code whose centre is closest to the given "
              "coords within tolerance." )
        .def( "delete_via_by_index", &PNS_RL_ROUTER::deleteViaByIndex,
              py::arg( "index" ),
              "Remove the via at list index (0-based)." )
        .def( "get_via_count", &PNS_RL_ROUTER::getViaCount,
              "Number of vias on the board." )
        .def( "lock_net", &PNS_RL_ROUTER::lockNet,
              py::arg( "net_code" ), py::arg( "locked" ) = true,
              "Lock/unlock a net's tracks/vias/arcs (BOARD lock flag + PNS "
              "MK_LOCKED) so shove treats them as immovable. Returns #items "
              "changed. Resyncs the world; cancels any active session." )
        .def( "delete_routing_of_nets", &PNS_RL_ROUTER::deleteRoutingOfNets,
              py::arg( "net_codes" ),
              "Remove tracks/vias/arcs of the given net codes only; keep all "
              "other routing (regardless of lock). Returns #items removed. "
              "Resyncs the world + ratsnest." )
        .def( "cleanup_tracks",
              []( PNS_RL_ROUTER& r, bool dry_run, bool merge_segments, bool clean_vias,
                  bool remove_shorts, bool tracks_in_pads, bool dangling_tracks,
                  bool dangling_vias, const std::vector<int>& net_codes )
              {
                  RLCleanupSpec spec;
                  spec.dry_run         = dry_run;
                  spec.merge_segments  = merge_segments;
                  spec.clean_vias      = clean_vias;
                  spec.remove_shorts   = remove_shorts;
                  spec.tracks_in_pads  = tracks_in_pads;
                  spec.dangling_tracks = dangling_tracks;
                  spec.dangling_vias   = dangling_vias;
                  spec.net_codes       = net_codes;
                  return r.cleanupTracks( spec );
              },
              py::arg( "dry_run" )         = true,
              py::arg( "merge_segments" )  = false,
              py::arg( "clean_vias" )      = false,
              py::arg( "remove_shorts" )   = false,
              py::arg( "tracks_in_pads" )  = false,
              py::arg( "dangling_tracks" ) = false,
              py::arg( "dangling_vias" )   = false,
              py::arg( "net_codes" )       = std::vector<int>{},
              "Run KiCad's track cleaner (RL fork) over the board. Quiescent only: "
              "returns CleanupResult(ran=False, reject_reason=...) while a routing or "
              "drag session is open. dry_run=True (default) reports without touching "
              "board geometry. A live run resyncs the world + ratsnest and frees the "
              "removed items; it is undoable via checkpoint/restore (no new UUIDs). "
              "net_codes limits every pass to those nets (empty = all)." )

        // -- Routing --
        .def( "start_route", &PNS_RL_ROUTER::startRoute,
              py::arg( "x_mm" ), py::arg( "y_mm" ), py::arg( "layer" ),
              "Begin a route from (x,y) on the given layer." )
        .def( "move", &PNS_RL_ROUTER::move,
              py::arg( "x_mm" ), py::arg( "y_mm" ),
              "Move the route head to (x,y). Call repeatedly to draw the path." )
        .def( "fix_route", &PNS_RL_ROUTER::fixRoute,
              py::arg( "x_mm" ), py::arg( "y_mm" ), py::arg( "force_finish" ) = true,
              py::arg( "reject_if_stuck" ) = false,
              py::arg( "expected_layer" ) = -1,
              py::arg( "arrive_tol_mm" ) = 0.0,
              py::arg( "require_via" ) = false,
              "Fix the route at (x,y). force_finish=True commits to board; "
              "False fixes a waypoint and continues routing. "
              "reject_if_stuck=True aborts without committing (returns False) when "
              "the head could not reach (x,y) — no partial stub is drawn. "
              "expected_layer (PCB_LAYER_ID, -1=skip): arrival on any other copper "
              "layer is treated as stuck — guards non-via actions against stray "
              "via-placement state. "
              "arrive_tol_mm (0 = exact): accept a head that stopped within this "
              "distance of (x,y) — PNS itself treats a point within head width/2 "
              "as reached, since the item's copper covers it." )
        .def( "cancel_route", &PNS_RL_ROUTER::cancelRoute,
              "Abort the current route without modifying the board." )

        // -- Routing control (Step 4) --
        .def( "finish", &PNS_RL_ROUTER::finish, py::arg( "max_attempts" ) = 5,
              "Auto-complete route to target pad, retrying ROUTER::Finish up to "
              "max_attempts times. Returns True on success." )
        .def( "undo_last_segment", &PNS_RL_ROUTER::undoLastSegment,
              "Undo the last fixed segment. Returns True if successful." )
        .def( "flip_posture", &PNS_RL_ROUTER::flipPosture,
              "Flip posture (horizontal-first vs vertical-first)." )

        // -- Via & Layer (Step 2.5) --
        .def( "toggle_via", &PNS_RL_ROUTER::toggleVia,
              "Toggle via placement mode. Next fix will insert a via." )
        .def( "switch_layer", &PNS_RL_ROUTER::switchLayer, py::arg( "layer" ),
              "Switch routing layer (inserts via automatically). Returns True on success." )
        .def( "is_placing_via", &PNS_RL_ROUTER::isPlacingVia,
              "True if via placement mode is active." )
        .def( "get_current_layer", &PNS_RL_ROUTER::getCurrentLayer,
              "Current routing layer ID (F.Cu=0, B.Cu=2)." )
        .def( "set_via_diameter", &PNS_RL_ROUTER::setViaDiameter, py::arg( "diameter_mm" ),
              "Set via outer diameter in millimetres." )
        .def( "set_via_drill", &PNS_RL_ROUTER::setViaDrill, py::arg( "drill_mm" ),
              "Set via drill diameter in millimetres." )
        .def( "reset_via_mode", &PNS_RL_ROUTER::resetViaMode,
              "Reset via placement mode to OFF." )

        // -- Dragging --
        .def( "start_drag", &PNS_RL_ROUTER::startDrag,
              py::arg( "x_mm" ), py::arg( "y_mm" ), py::arg( "layer" ),
              py::arg( "drag_mode" ) = 0x17,
              "Begin dragging the track item nearest to (x,y). "
              "drag_mode: DM_CORNER=0x1, DM_SEGMENT=0x2, DM_VIA=0x4, DM_ANY=0x17, etc." )
        .def( "fix_drag", &PNS_RL_ROUTER::fixDrag,
              py::arg( "force_commit" ) = true,
              "Commit the current drag to the board." )
        .def( "cancel_drag", &PNS_RL_ROUTER::cancelDrag,
              "Abort the current drag without modifying the board." )
        .def( "is_dragging", &PNS_RL_ROUTER::isDragging,
              "True while an interactive drag session is active." )

        // -- DRC --
        .def( "run_drc", &PNS_RL_ROUTER::runDRC,
              py::arg("rules_path") = "",
              "Run a full DRC check. Pass optional .kicad_dru path to apply custom rules. Returns list[DRCViolation]." )
        .def( "run_drc_incremental", &PNS_RL_ROUTER::runDRCIncremental,
              py::arg("rules_path") = "",
              "Incremental DRC: same result as run_drc() but rechecks only clearance "
              "for tracks/vias changed since the last DRC. Falls back to full on first "
              "call or zoned boards. Returns list[DRCViolation]." )
        .def( "get_drc_violation_count", &PNS_RL_ROUTER::getDRCViolationCount,
              "Number of violations from the last run_drc() call." )
        .def( "get_drc_violations", &PNS_RL_ROUTER::getDRCViolations,
              "List of DRCViolation objects from the last run_drc() call." )
        .def( "clear_drc_cache", &PNS_RL_ROUTER::clearDRCCache,
              "Clear cached DRC violations." )
        .def( "get_drc_violations_by_net", &PNS_RL_ROUTER::getDRCViolationsByNet,
              "DRC violations grouped by net: dict[str, list[str]] (net_name → [error_type, ...])." )

        // -- Design rules --
        .def( "get_design_rules", &PNS_RL_ROUTER::getDesignRules,
              "Snapshot the board's design rules (BDS + NetSettings) as a DesignRules object." )
        .def( "set_design_rules", &PNS_RL_ROUTER::setDesignRules, py::arg( "rules" ),
              "Apply the global minima fields of DesignRules (negative values = leave unchanged). "
              "Preset lists and netclass entries are ignored." )
        .def( "get_netclass_for_net",
              &PNS_RL_ROUTER::getNetClassForNet, py::arg( "net_code" ),
              "Return the NetClassInfo KiCad assigns to the given net code "
              "(Default if the net has no explicit class). If the net code "
              "is unknown the returned info has empty ``name``; treat that "
              "as a signal to fall back to ``default_netclass``." )

        // -- Observation --
        .def( "get_track_count", &PNS_RL_ROUTER::getTrackCount,
              "Number of track segments (VIAs excluded)." )
        .def( "get_tracks", &PNS_RL_ROUTER::getTracks,
              "Return a list of TrackInfo objects for all track segments." )
        .def( "get_vias", &PNS_RL_ROUTER::getVias,
              "Return a list of ViaInfo objects for all vias." )
        .def( "get_pads", &PNS_RL_ROUTER::getPads,
              "Return a list of PadInfo objects for all pads." )
        .def( "get_keepouts", &PNS_RL_ROUTER::getKeepouts,
              "Return a list of ZoneInfo objects for rule-area keepout zones "
              "(one entry per zone per copper layer)." )
        .def( "get_footprints", &PNS_RL_ROUTER::getFootprints,
              "Return a list of FootprintInfo objects for every component, each "
              "carrying its courtyard outline in board coordinates (empty when "
              "the footprint declares no courtyard)." )
        .def( "get_ratsnest", &PNS_RL_ROUTER::getRatsnest,
              "Return a list of RatsnestEdge objects (unrouted connections)." )
        .def( "get_pad_clusters", &PNS_RL_ROUTER::getPadClusters,
              "Return [(net_code, pad_count)] for every connectivity cluster "
              "holding at least one pad (pad-free copper islands omitted)." )
        .def( "get_connected_points", &PNS_RL_ROUTER::getConnectedPoints,
              py::arg( "x_mm" ), py::arg( "y_mm" ), py::arg( "board_layer" ),
              "Return ClusterPoint objects for every board item already connected "
              "to the copper at (x_mm, y_mm, board_layer) — its whole connectivity "
              "cluster, one entry per anchor point per copper layer. Empty when no "
              "copper is at that position. Reflects committed board state, so run "
              "build_connectivity() after mutations first." )
        .def( "get_board_outline", &PNS_RL_ROUTER::getBoardOutline,
              "Return a list of BoardEdge objects (Edge.Cuts polylines)." )
        .def( "get_board_outline_shapes", &PNS_RL_ROUTER::getBoardOutlineShapes,
              "Return a list of BoardOutlineShape objects (Edge.Cuts primitives; "
              "arcs/circles kept as typed 3-point entries, not tessellated)." )
        .def( "get_unrouted_count", &PNS_RL_ROUTER::getUnroutedCount,
              "Number of unrouted connections (ratsnest edges)." )
        .def( "is_routing", &PNS_RL_ROUTER::isRouting,
              "True while an interactive routing session is active." )

        // -- Markov-property observation (route head, active net, target) --
        .def( "get_route_head", &PNS_RL_ROUTER::getRouteHead,
              "Route head {x_mm, y_mm, layer} during routing, {0,0,-1} if idle." )
        .def( "get_current_net_code", &PNS_RL_ROUTER::getCurrentNetCode,
              "Net code of the net being routed, -1 if idle." )
        .def( "get_routing_target", &PNS_RL_ROUTER::getRoutingTarget,
              "Nearest ratsnest target {x_mm, y_mm, layer} for active net." )
        .def( "get_wip_segments", &PNS_RL_ROUTER::getWipSegments,
              "Work-in-progress trace segments from the active placer." )

        // -- Connectivity & Board Query (Step 3) --
        .def( "build_connectivity", &PNS_RL_ROUTER::buildConnectivity,
              "Rebuild board connectivity graph. Must call after track changes." )
        .def( "recalculate_ratsnest", &PNS_RL_ROUTER::recalculateRatsnest,
              "Recalculate ratsnest (unrouted) edges." )
        .def( "get_net_count", &PNS_RL_ROUTER::getNetCount,
              "Number of nets in the connectivity graph." )
        .def( "get_board_net_count", &PNS_RL_ROUTER::getBoardNetCount,
              "Number of nets on the board." )
        .def( "get_board_bbox", &PNS_RL_ROUTER::getBoardBBox,
              "Board bounding box as BoundingBox object." )
        .def( "get_graphic_shapes", &PNS_RL_ROUTER::getGraphicShapes,
              py::arg( "layer" ),
              "Board-level PCB_SHAPE drawings on the given layer as GraphicShape "
              "objects — exact integer-nm coordinates, arcs analytic (3-point)." )
        .def( "replace_graphic_shapes", &PNS_RL_ROUTER::replaceGraphicShapes,
              py::arg( "layer" ), py::arg( "remove_indices" ),
              py::arg( "new_segments" ), py::arg( "new_arcs" ),
              "Delete drawings by GraphicShape.index and add new segments "
              "({x1,y1,x2,y2,width} nm) / 3-point arcs ({x1,y1,xm,ym,x2,y2,width} nm) "
              "on the layer, then rebuild connectivity + the PNS world. Load-time "
              "outline-simplify support — call before configuring the router." )
        .def( "get_copper_layer_count", &PNS_RL_ROUTER::getCopperLayerCount,
              "Number of copper layers (2=two-sided, 4/6/8=multi-layer)." )

        // -- State Query (Step 3) --
        .def( "get_router_state", &PNS_RL_ROUTER::getRouterState,
              "Router state: 0=IDLE, 1=DRAG_SEGMENT, 2=DRAG_COMPONENT, 3=ROUTE_TRACK." )
        .def( "get_failure_reason", &PNS_RL_ROUTER::getFailureReason,
              "Failure reason string from the last routing attempt." )

        // -- Checkpoint / Restore (MCTS tree search) --
        .def( "checkpoint", &PNS_RL_ROUTER::checkpoint,
              "Capture board + engine config + routing session into an internal "
              "store; return an opaque integer handle (worker/router-local)." )
        .def( "restore", &PNS_RL_ROUTER::restore,
              py::arg( "handle" ),
              "Restore the state captured by handle. No-op if handle is unknown "
              "or already released." )
        .def( "restore_incremental", &PNS_RL_ROUTER::restoreIncremental,
              py::arg( "handle" ),
              "Incremental restore (diff-at-restore): updates only changed tracks "
              "in the PNS world instead of a full rebuild. Same result as restore()." )
        .def( "release_checkpoint", &PNS_RL_ROUTER::releaseCheckpoint,
              py::arg( "handle" ),
              "Release a checkpoint handle and free its cloned items. Idempotent." )
        .def( "reset_checkpoints", &PNS_RL_ROUTER::resetCheckpoints,
              "Release ALL checkpoints at once (frees every clone + DRC state). Use at "
              "episode boundaries to bound memory. Re-seeds the handle epoch so every "
              "handle from before the reset becomes permanently invalid." )
        .def( "has_checkpoint", &PNS_RL_ROUTER::hasCheckpoint,
              py::arg( "handle" ),
              "True if handle refers to a live checkpoint. Reliable across resets / "
              "instances (handles are globally unique), so use it to reject stale handles." )
        .def( "get_checkpoint_count", &PNS_RL_ROUTER::checkpointCount,
              "Number of live checkpoints currently held (diagnostic)." )
        .def( "rewind_kiid_to_episode_start", &PNS_RL_ROUTER::rewindKIIDToEpisodeStart,
              "Rewind the global KIID/UUID generator to the position captured at "
              "construction (post board-load), so every episode after reset() draws the "
              "same UUID stream. No-op when entropy-seeded (engine_seed < 0). Call it "
              "LAST in env.reset(); collision-safe like restore()." )

        // -- World diagnostics (state-dependency probes) --
        .def( "resync_world",
              []( PNS_RL_ROUTER& r )
              {
                  // A live placer/dragger holds pointers into the world being
                  // freed — cancel any open session first (both are no-ops when
                  // idle), mirroring restore() step 1.
                  r.cancelRoute();
                  r.cancelDrag();
                  r.resyncWorld();
              },
              "Rebuild the PNS world from the current board (ClearWorld + SyncWorld). "
              "Cancels any active routing/drag session first (a live placer would "
              "dangle into the freed world) — re-open with start_route() afterwards." )
        .def( "world_stats", &PNS_RL_ROUTER::worldStats,
              "Dict of PNS world (NODE) introspection counters: joint_count, depth, "
              "joints_queried/links_total/links_max/joints_locked and unique per-kind "
              "item counts (segments/vias/virtual_vias/solids/arcs/others) gathered "
              "via QueryJoints over the whole board area. Diagnostic only." )
        .def( "set_shove_iter_limit", &PNS_RL_ROUTER::setShoveIterationLimit,
              py::arg( "limit" ),
              "Change the shove iteration bound at runtime. SHOVE reads it live on "
              "each Run, so it takes effect mid-session (e.g. right before one "
              "fix_route call) without re-creating the engine." )
        .def( "get_shove_iter_limit", &PNS_RL_ROUTER::getShoveIterationLimit,
              "Current shove iteration bound (see set_shove_iter_limit)." )

        // -- I/O --
        .def( "save", &PNS_RL_ROUTER::save,
              py::arg( "output_path" ),
              py::arg( "project_output_path" ) = "",
              "Save the board and its companion .kicad_pro. "
              "An empty project_output_path auto-derives <output_stem>.kicad_pro "
              "next to the pcb." );

    // Process-global KIID/UUID generator state (opaque bytes) — lets diagnostics
    // transplant the exact UUID-stream position between engines/processes (the
    // obstacle ordering is UUID-keyed, so the stream position is routing-
    // relevant state that is NOT stored in the board file).
    m.def( "kiid_get_generator_state",
           []() { return py::bytes( KIID::GetGeneratorState() ); },
           "Opaque byte blob of the process-global KIID generator position." );
    m.def( "kiid_set_generator_state",
           []( const py::bytes& state )
           { KIID::SetGeneratorState( static_cast<std::string>( state ) ); },
           py::arg( "state" ),
           "Restore a KIID generator position captured by kiid_get_generator_state(). "
           "Diagnostic use: mid-episode rewinds can re-issue UUIDs already held by "
           "live board items." );

    // Routing mode constants
    m.attr( "MODE_MARK_OBSTACLES" ) = 0;
    m.attr( "MODE_SHOVE"          ) = 1;
    m.attr( "MODE_WALKAROUND"     ) = 2;

    // Corner mode constants (DIRECTION_45::CORNER_MODE)
    m.attr( "CORNER_MITERED_45" ) = 0;
    m.attr( "CORNER_ROUNDED_45" ) = 1;
    m.attr( "CORNER_MITERED_90" ) = 2;
    m.attr( "CORNER_ROUNDED_90" ) = 3;

    // Drag mode constants (PNS::DRAG_MODE bitmask values)
    m.attr( "DM_CORNER" )     = 0x1;
    m.attr( "DM_SEGMENT" )    = 0x2;
    m.attr( "DM_VIA" )        = 0x4;
    m.attr( "DM_FREE_ANGLE" ) = 0x8;
    m.attr( "DM_ARC" )        = 0x10;
    m.attr( "DM_ANY" )        = 0x17;
    m.attr( "DM_COMPONENT" )  = 0x20;

    // Router state constants
    m.attr( "STATE_IDLE" )           = 0;
    m.attr( "STATE_DRAG_SEGMENT" )   = 1;
    m.attr( "STATE_DRAG_COMPONENT" ) = 2;
    m.attr( "STATE_ROUTE_TRACK" )    = 3;

    // Layer constants
    m.attr( "F_Cu" ) = 0;
    m.attr( "B_Cu" ) = 2;
}
