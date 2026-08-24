#define _CRT_SECURE_NO_WARNINGS

// tools/rx_lite.cpp -- checker-oriented lite entrypoint.
//
// LITE0 proves the DATA PATH parses/builds/reports cleanly. R0 adds the first
// checker-oriented routing baseline: one shortest legal route per demanded
// pair, no chunks, no ensemble, no present/history pricing, no negotiation.
//
// Two modes:
//   1) q43 normal:   rx_lite <case.csv> <placement.cfg>
//        Parser -> PlacementReader -> ChannelBuilder -> RxGraph -> demandedPairs,
//        then a key=value report. Channels here are ChannelBuilder-DERIVED (the
//        corpus_q43 .cfg carries placement only).
//   2) calibration:  rx_lite --declared <submission.cfg>
//        Reads the .cfg's DECLARED OUTLINE + CHANNEL sections (PlacementReader
//        ignores CHANNEL on purpose, and the official checker scores the DECLARED
//        channels) and reports outlineArea + channelCapacityTotal = sum(25*(h+w)),
//        so it can be checked to the decimal against a submission's
//        case_result.json (outline_area / channel_total_capacity).
//   3) R0 route:     rx_lite --r0 <case.csv> <placement.cfg>
//        Same build path, then route each demanded pair once: direct adjacency,
//        channel-only shortest path, then SOFT feedthrough fallback. The ledger
//        is raw usage only; scorer tradeoffs begin later.
//   4) calibration:  rx_lite --calib <case.csv> <submission.cfg>
//        Reads declared BLOCK/CHANNEL/PATH and replays those routes through the
//        R1 ledger. This is official-artifact calibration, isolated from R0's
//        own routed numbers.
//   5) R2b1 surface: rx_lite --r2b1 <case.csv> <placement.cfg>
//        Generates the lite candidate surface around each R0 route, projects each
//        candidate as a replacement for that pair's R0 route, and reports whether
//        there is material wire-saving topology. It does not change routing.
//   6) R2b2 selector: rx_lite --r2b2 <selector> <case.csv> <placement.cfg>
//        Runs the lite candidate factory + official ledger + selector loop.
//
// R2w0: --r0/--r2b1/--r2b2 default to checker-style contact-guiding-point WL
// in RxSearch. Set RX_LITE_SEARCH_COST=edge_center to reproduce the legacy
// edge-center proxy baseline.
//
// RxGraph route-space sync: --r0/--r2b1/--r2b2 default to official_full:
// block<->SOFT endpoint access plus SOFT<->SOFT edges. Set
// RX_LITE_ROUTE_SPACE=legacy to reproduce the older graph, or use
// soft_endpoint / soft_soft to isolate each legal route-space half.
//
// AccessAltUnified defaults to the full legal endpoint-access universe. Set
// RX_LITE_ACCESS_ALT_UNIVERSE=capped_channel to reproduce the old
// target-directed channel-only surface.
//
// Boundary: RxGraph / RxSearch / RxEmit / RxPairOrder / RxScore + Parser /
// PlacementReader / ChannelBuilder / DataModel / Utility. Does NOT touch
// RouterX.cpp, ensembles, candidate probe, negotiation, chunking, or any P2
// harness.

#include "RouterLiteEngine.hpp"
#include "DataModel.hpp"
#include "ChannelBuilder.hpp"
#include "RouterX/RxGraph.hpp"
#include "RouterX/RxPairOrder.hpp"
#include "RouterX/RxScore.hpp"   // kChannelDensity (single source) + oracle
#include "RouterX/RxSearch.hpp"
#include "RouterX/RxCandidate.hpp"
#include "RouterX/RxCandidateFactoryLite.hpp"
#include "RouterX/RxDelta.hpp"
#include "RouterX/RxEmit.hpp"
#include "RouterX/RxRouteLedger.hpp"
#include "RouterX/RxResourceLedgerLite.hpp"
#include "Utility.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace {

// The standalone tool reports thousands of key/value diagnostics.  The
// embedded adapter keeps the exact routing decisions but leaves application
// stdout under main.cpp's control.
void emit(const char*, double) {}
void emit(const char*, long long) {}
void emit(const char*, const std::string&) {}

const char* liteSearchCostModeName(routerx::SearchCostMode m) {
    switch (m) {
        case routerx::SearchCostMode::EdgeCenter: return "edge_center";
        case routerx::SearchCostMode::TrueWire:   return "truewire";
    }
    return "unknown";
}

bool parseLiteSearchCostMode(routerx::SearchCostMode& out) {
    const char* env = std::getenv("RX_LITE_SEARCH_COST");
    if (!env || std::string(env).empty()) {
        out = routerx::SearchCostMode::TrueWire;
        return true;
    }
    const std::string v(env);
    if (v == "edge_center" || v == "edgecenter" || v == "edge") {
        out = routerx::SearchCostMode::EdgeCenter;
        return true;
    }
    if (v == "truewire" || v == "true_wire" || v == "official_wire") {
        out = routerx::SearchCostMode::TrueWire;
        return true;
    }
    std::cerr << "[rx_lite] Unknown RX_LITE_SEARCH_COST=" << v
              << " (want edge_center|truewire)\n";
    return false;
}

const char* liteRouteSpaceName(const routerx::SoftRoutingPolicy& p) {
    if (p.endpointAccess && p.softSoftEdges) return "official_full";
    if (p.endpointAccess) return "soft_endpoint";
    if (p.softSoftEdges) return "soft_soft";
    return "legacy";
}

bool parseLiteRouteSpace(routerx::SoftRoutingPolicy& out) {
    const char* env = std::getenv("RX_LITE_ROUTE_SPACE");
    if (!env || std::string(env).empty()) {
        out = routerx::SoftRoutingPolicy{/*endpointAccess=*/true,
                                         /*softSoftEdges=*/true};
        return true;
    }
    const std::string v(env);
    if (v == "legacy" || v == "off" || v == "channel_soft") {
        out = routerx::SoftRoutingPolicy{};
        return true;
    }
    if (v == "official_full" || v == "official" || v == "full") {
        out = routerx::SoftRoutingPolicy{/*endpointAccess=*/true,
                                         /*softSoftEdges=*/true};
        return true;
    }
    if (v == "soft_endpoint" || v == "endpoint") {
        out = routerx::SoftRoutingPolicy{/*endpointAccess=*/true,
                                         /*softSoftEdges=*/false};
        return true;
    }
    if (v == "soft_soft" || v == "softsoft") {
        out = routerx::SoftRoutingPolicy{/*endpointAccess=*/false,
                                         /*softSoftEdges=*/true};
        return true;
    }
    std::cerr << "[rx_lite] Unknown RX_LITE_ROUTE_SPACE=" << v
              << " (want legacy|soft_endpoint|soft_soft|official_full)\n";
    return false;
}

enum class AccessAltUniverseMode {
    CappedChannel,
    FullUnified,
};

enum class AvoidHotEligibilityMode {
    WinnerTouchesHot,
    CandidateTouchesHot,
};

const char* accessAltUniverseModeName(AccessAltUniverseMode m) {
    switch (m) {
        case AccessAltUniverseMode::CappedChannel: return "capped_channel";
        case AccessAltUniverseMode::FullUnified:   return "full_unified";
    }
    return "unknown";
}

bool parseAccessAltUniverseMode(AccessAltUniverseMode& out) {
    const char* env = std::getenv("RX_LITE_ACCESS_ALT_UNIVERSE");
    if (!env || std::string(env).empty()) {
        out = AccessAltUniverseMode::FullUnified;
        return true;
    }
    const std::string v(env);
    if (v == "full" || v == "full_unified" || v == "all") {
        out = AccessAltUniverseMode::FullUnified;
        return true;
    }
    if (v == "capped" || v == "capped_channel" || v == "legacy") {
        out = AccessAltUniverseMode::CappedChannel;
        return true;
    }
    std::cerr << "[rx_lite] Unknown RX_LITE_ACCESS_ALT_UNIVERSE=" << v
              << " (want capped_channel|full_unified)\n";
    return false;
}

const char* avoidHotEligibilityModeName(AvoidHotEligibilityMode m) {
    switch (m) {
        case AvoidHotEligibilityMode::WinnerTouchesHot:
            return "winner_touches_hot";
        case AvoidHotEligibilityMode::CandidateTouchesHot:
            return "candidate_touches_hot";
    }
    return "unknown";
}

bool parseAvoidHotEligibilityMode(AvoidHotEligibilityMode& out) {
    const char* env = std::getenv("RX_LITE_AVOID_HOT_ELIGIBILITY");
    if (!env || std::string(env).empty()) {
        out = AvoidHotEligibilityMode::CandidateTouchesHot;
        return true;
    }
    const std::string v(env);
    if (v == "winner" || v == "winner_touches_hot") {
        out = AvoidHotEligibilityMode::WinnerTouchesHot;
        return true;
    }
    if (v == "candidate" || v == "candidate_touch_hot" ||
        v == "candidate_touches_hot") {
        out = AvoidHotEligibilityMode::CandidateTouchesHot;
        return true;
    }
    std::cerr << "[rx_lite] Unknown RX_LITE_AVOID_HOT_ELIGIBILITY=" << v
              << " (want winner_touches_hot|candidate_touches_hot)\n";
    return false;
}

bool parseEnvBoolDefault(const char* name, bool defaultValue, bool& out) {
    const char* env = std::getenv(name);
    if (!env || std::string(env).empty()) {
        out = defaultValue;
        return true;
    }
    const std::string v(env);
    if (v == "1" || v == "on" || v == "true" || v == "yes") {
        out = true;
        return true;
    }
    if (v == "0" || v == "off" || v == "false" || v == "no") {
        out = false;
        return true;
    }
    std::cerr << "[rx_lite] Unknown " << name << "=" << v
              << " (want on|off)\n";
    return false;
}

bool parseEnvDouble(const char* name, double& out) {
    const char* env = std::getenv(name);
    if (!env || std::string(env).empty()) return true;
    char* end = nullptr;
    const double v = std::strtod(env, &end);
    if (end && *end == '\0') {
        out = v;
        return true;
    }
    std::cerr << "[rx_lite] Invalid " << name << "=" << env << "\n";
    return false;
}

struct CapacityAwareConfig {
    bool enabled = true;
    routerx::CapacityAwareStressMode stressMode =
        routerx::CapacityAwareStressMode::LogUtil;
    double weight = 750.0;
    double capFloor = 1.0;
    double targetUtil = 0.30;
    double stressCap = 4.0;
};

const char* capacityAwareStressModeName(routerx::CapacityAwareStressMode m) {
    switch (m) {
        case routerx::CapacityAwareStressMode::CappedLinear:
            return "capped_linear";
        case routerx::CapacityAwareStressMode::LogUtil:
            return "log_util";
    }
    return "unknown";
}

bool parseCapacityAwareConfig(CapacityAwareConfig& out) {
    if (!parseEnvBoolDefault("RX_LITE_CAPACITY_AWARE", true, out.enabled))
        return false;
    const char* stress = std::getenv("RX_LITE_CAPACITY_AWARE_STRESS");
    if (stress && *stress) {
        const std::string s(stress);
        if (s == "capped_linear" || s == "linear" || s == "cap") {
            out.stressMode = routerx::CapacityAwareStressMode::CappedLinear;
        } else if (s == "log_util" || s == "log") {
            out.stressMode = routerx::CapacityAwareStressMode::LogUtil;
        } else {
            std::cerr << "[rx_lite] Unknown RX_LITE_CAPACITY_AWARE_STRESS="
                      << s << " (want capped_linear|log_util)\n";
            return false;
        }
    }
    if (!parseEnvDouble("RX_LITE_CAPACITY_AWARE_WEIGHT", out.weight)) return false;
    if (!parseEnvDouble("RX_LITE_CAPACITY_AWARE_CAP_FLOOR", out.capFloor)) return false;
    if (!parseEnvDouble("RX_LITE_CAPACITY_AWARE_TARGET_UTIL", out.targetUtil)) return false;
    if (!parseEnvDouble("RX_LITE_CAPACITY_AWARE_STRESS_CAP", out.stressCap)) return false;
    if (out.enabled && out.weight <= 0.0) {
        std::cerr << "[rx_lite] CapacityAware baseline requires positive weight\n";
        return false;
    }
    return true;
}

void emitCapacityAwareConfig(const CapacityAwareConfig& c) {
    emit("lite_capacityAware", std::string(c.enabled ? "on" : "off"));
    emit("lite_capacityAwareStress",
         std::string(capacityAwareStressModeName(c.stressMode)));
    emit("lite_capacityAwareWeight", c.weight);
    emit("lite_capacityAwareCapFloor", c.capFloor);
    emit("lite_capacityAwareTargetUtil", c.targetUtil);
    emit("lite_capacityAwareStressCap", c.stressCap);
}

int avoidHotPasses() {
    const char* env = std::getenv("RX_LITE_AVOID_HOT_PASSES");
    if (!env || std::string(env).empty()) return 1;
    char* end = nullptr;
    const long v = std::strtol(env, &end, 10);
    if (end && *end == '\0' && (v == 1 || v == 2)) return static_cast<int>(v);
    std::cerr << "[rx_lite] Unknown RX_LITE_AVOID_HOT_PASSES=" << env
              << " (expected 1|2)\n";
    return -1;
}

int reselectMaxPasses() {
    const char* env = std::getenv("RX_LITE_RESELECT_MAX_PASSES");
    if (!env || std::string(env).empty()) return 5;
    char* end = nullptr;
    const long v = std::strtol(env, &end, 10);
    if (end && *end == '\0' && v >= 0 && v <= 100) return static_cast<int>(v);
    std::cerr << "[rx_lite] Unknown RX_LITE_RESELECT_MAX_PASSES=" << env
              << " (expected integer 0..100)\n";
    return -1;
}

bool hotReportEnabled() {
    const char* env = std::getenv("RX_LITE_HOT_REPORT");
    if (!env) return false;
    const std::string v(env);
    return v == "1" || v == "on" || v == "true" || v == "yes";
}

std::string fmt3(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.3f", v);
    return std::string(buf);
}

void emitLiteGraphReport(const routerx::RxGraph& graph) {
    emit("lite_routeSpace", std::string(liteRouteSpaceName(
        routerx::SoftRoutingPolicy{graph.softEndpointAccessEnabled(),
                                   graph.softSoftEdgesEnabled()})));
    emit("lite_graphSoftEndpointAccess", std::string(
        graph.softEndpointAccessEnabled() ? "on" : "off"));
    emit("lite_graphSoftSoftEdges", std::string(
        graph.softSoftEdgesEnabled() ? "on" : "off"));
    emit("lite_graphSoftEndpointAccessCount",
         static_cast<long long>(graph.softEndpointAccessCount()));
    emit("lite_graphSoftSoftEdgeCount",
         static_cast<long long>(graph.softSoftEdgeCount()));
}

routerx::SearchResult findLitePathInMode(const Design& design,
                                         const routerx::RxGraph& graph,
                                         int srcBlock,
                                         int dstBlock,
                                         routerx::Mode mode,
                                         routerx::SearchCostMode costMode) {
    routerx::SearchOptions opt;
    opt.mode = mode;
    opt.costMode = costMode;
    return routerx::RxSearch::findPathWith(design, graph, srcBlock, dstBlock, opt);
}

struct ParsedPlacement {
    Design design;
    bool ok = false;
};

#if 0 // The embedded engine receives an already parsed/floorplanned Design.
ParsedPlacement loadQ43Design(const std::string& csvPath, const std::string& cfgPath) {
    ParsedPlacement p;
    Parser parser;
    if (!parser.read(csvPath, p.design)) {
        std::cerr << "[rx_lite] Parser failed on " << csvPath << "\n";
        return p;
    }
    PlacementReader reader;
    if (!reader.load(cfgPath, p.design)) {
        std::cerr << "[rx_lite] PlacementReader failed on " << cfgPath << "\n";
        return p;
    }
    ChannelBuilder channelBuilder;
    channelBuilder.build(p.design);
    p.ok = true;
    return p;
}
#endif

#if 0
ParsedPlacement loadDeclaredDesignShell(const std::string& csvPath,
                                        const std::string& cfgPath) {
    ParsedPlacement p;
    Parser parser;
    if (!parser.read(csvPath, p.design)) {
        std::cerr << "[rx_lite] Parser failed on " << csvPath << "\n";
        return p;
    }
    PlacementReader reader;
    if (!reader.load(cfgPath, p.design)) {
        std::cerr << "[rx_lite] PlacementReader failed on " << cfgPath << "\n";
        return p;
    }
    p.ok = true;
    return p;
}
#endif

double channelCapacityTotal(const Design& design) {
    double total = 0.0;
    for (const Channel& c : design.channels) total += c.lrCapacity + c.tbCapacity;
    return total;
}

double blockBaseAreaSum(const Design& design, bool softOnly) {
    double total = 0.0;
    for (const BlockInst& b : design.blocks) {
        if (!softOnly || b.spec.type == BlockType::SOFT) total += b.spec.area;
    }
    return total;
}

int directAdjacencyEdgeLite(const Design& design, int srcBlock, int dstBlock) {
    // Mirrors RxCandidates::directAdjacencyEdge without linking the full
    // candidate factory into the lite binary.
    if (srcBlock < 0 || dstBlock < 0 ||
        srcBlock >= static_cast<int>(design.blocks.size()) ||
        dstBlock >= static_cast<int>(design.blocks.size())) {
        return 0;
    }
    const BlockInst& s = design.blocks[srcBlock];
    const BlockInst& d = design.blocks[dstBlock];
    const int pA = routerx::facingEdge(s.rect, d.rect);
    if (pA == 0) return 0;
    const int pB = routerx::oppositeEdge(pA);
    if (!isPortEdgeAllowed(s, pA, design.outlineW, design.outlineH)) return 0;
    if (!isPortEdgeAllowed(d, pB, design.outlineW, design.outlineH)) return 0;
    return pA;
}

struct LiteRoutedPair {
    routerx::DemandedPair demand;
    RoutePath route;
    bool found = false;
    bool emitValid = false;
    bool usedFeedthrough = false;
    bool direct = false;
    bool channelOnly = false;
    long long searchCalls = 0;
    std::string rejectReason = "none";
};

std::vector<RouteStep> directStepsForPair(const Design& design,
                                          const routerx::DemandedPair& p,
                                          int directEdge) {
    return {
        RouteStep{design.blocks[p.src].spec.name, directEdge},
        RouteStep{design.blocks[p.dst].spec.name, routerx::oppositeEdge(directEdge)}
    };
}

RoutePath routePathFromSteps(const Design& design,
                             const routerx::DemandedPair& p,
                             const std::vector<RouteStep>& steps) {
    RoutePath r;
    r.netCount = p.nets;
    r.srcBlock = design.blocks[p.src].spec.name;
    r.dstBlock = design.blocks[p.dst].spec.name;
    r.steps = steps;
    r.open = false;
    return r;
}

std::vector<RouteStep> stepsFromSearch(const Design& design,
                                       const routerx::RxGraph& graph,
                                       const routerx::DemandedPair& p,
                                       const routerx::SearchResult& r) {
    routerx::RouteCandidate c;
    c.family = r.usedFeedthrough ? routerx::CandidateFamily::FtForced
                                 : routerx::CandidateFamily::ChannelShortest;
    c.srcPortEdge = r.srcPortEdge;
    c.dstPortEdge = r.dstPortEdge;
    c.hops = r.hops;
    c.usedFeedthrough = r.usedFeedthrough;
    return routerx::candidateSteps(design, graph, c,
                                   design.blocks[p.src].spec.name,
                                   design.blocks[p.dst].spec.name);
}

bool stepsUseFeedthrough(const Design& design, const std::vector<RouteStep>& steps) {
    for (size_t i = 1; i + 1 < steps.size(); i += 2) {
        const std::string& name = steps[i].rectName;
        auto it = std::find_if(design.blocks.begin(), design.blocks.end(),
            [&](const BlockInst& b) { return b.spec.name == name; });
        if (it != design.blocks.end() && it->spec.type == BlockType::SOFT)
            return true;
    }
    return false;
}

LiteRoutedPair routeR0Fallback(const Design& design,
                               const routerx::RxGraph& graph,
                               const routerx::DemandedPair& p,
                               routerx::SearchCostMode costMode) {
    LiteRoutedPair out;
    out.demand = p;
    const std::string& srcName = design.blocks[p.src].spec.name;
    const std::string& dstName = design.blocks[p.dst].spec.name;

    const int directEdge = directAdjacencyEdgeLite(design, p.src, p.dst);
    if (directEdge != 0) {
        out.route = routePathFromSteps(design, p, directStepsForPair(design, p, directEdge));
        out.direct = true;
    } else {
        ++out.searchCalls;
        routerx::SearchResult r = findLitePathInMode(
            design, graph, p.src, p.dst, routerx::Mode::ChannelOnly, costMode);
        if (!r.found) {
            ++out.searchCalls;
            r = findLitePathInMode(
                design, graph, p.src, p.dst, routerx::Mode::AllowFeedthrough, costMode);
        }
        if (!r.found) return out;
        out.route = routePathFromSteps(design, p, stepsFromSearch(design, graph, p, r));
        out.usedFeedthrough = r.usedFeedthrough || stepsUseFeedthrough(design, out.route.steps);
        out.channelOnly = !out.usedFeedthrough;
    }

    routerx::EmitReject reject = routerx::EmitReject::None;
    out.emitValid = routerx::RxEmit::validate(out.route.steps, srcName, dstName, reject);
    out.rejectReason = routerx::emitRejectName(reject);
    out.found = out.emitValid;
    return out;
}

struct AccessChoice {
    int pos = -1;          // position in graph.accessesOf(block)
    double score = 0.0;
};

int targetPortEdge(const Rect& from, const Rect& to) {
    const double dx = rectCx(to) - rectCx(from);
    const double dy = rectCy(to) - rectCy(from);
    if (std::fabs(dx) >= std::fabs(dy)) return dx >= 0.0 ? 3 : 1;
    return dy >= 0.0 ? 2 : 4;
}

double accessTargetScore(const Design& design,
                         const routerx::RxGraph& graph,
                         int blockIndex,
                         int targetBlock,
                         int accessIndex) {
    const routerx::Access& a = graph.accesses()[accessIndex];
    const int preferred = targetPortEdge(design.blocks[blockIndex].rect,
                                         design.blocks[targetBlock].rect);
    const double edgePenalty = (a.portEdge == preferred) ? 0.0 : 1000000000.0;
    const routerx::Node& n = graph.nodes()[a.node];
    const Rect& nodeRect = (n.kind == routerx::NodeKind::Channel)
        ? design.channels[n.designIndex].rect
        : design.blocks[n.designIndex].rect;
    const Rect& target = design.blocks[targetBlock].rect;
    return edgePenalty + manhattan(rectCx(nodeRect), rectCy(nodeRect),
                                   rectCx(target), rectCy(target));
}

std::vector<AccessChoice> rankedChannelAccessChoices(const Design& design,
                                                     const routerx::RxGraph& graph,
                                                     int blockIndex,
                                                     int targetBlock,
                                                     int maxChoices) {
    std::vector<AccessChoice> out;
    const std::vector<int>& acc = graph.accessesOf(blockIndex);
    for (size_t pos = 0; pos < acc.size(); ++pos) {
        if (!graph.accessIsChannel(acc[pos])) continue;
        out.push_back(AccessChoice{
            static_cast<int>(pos),
            accessTargetScore(design, graph, blockIndex, targetBlock, acc[pos])
        });
    }
    std::sort(out.begin(), out.end(), [](const AccessChoice& a, const AccessChoice& b) {
        if (a.score != b.score) return a.score < b.score;
        return a.pos < b.pos;
    });
    if (static_cast<int>(out.size()) > maxChoices) out.resize(maxChoices);
    return out;
}

routerx::LiteCandidate makeCandidate(const Design& design,
                                     const routerx::DemandedPair& p,
                                     routerx::LiteCandidateFamily family,
                                     const std::vector<RouteStep>& steps,
                                     bool isR0Fallback = false) {
    routerx::LiteCandidate c;
    c.family = family;
    c.route = routePathFromSteps(design, p, steps);
    c.signature = routerx::makeLiteSignature(c.route.steps);
    c.usedFeedthrough = stepsUseFeedthrough(design, c.route.steps);
    c.isR0Fallback = isR0Fallback;

    routerx::EmitReject reject = routerx::EmitReject::None;
    c.emitValid = routerx::RxEmit::validate(
        c.route.steps,
        design.blocks[p.src].spec.name,
        design.blocks[p.dst].spec.name,
        reject);
    c.rejectReason = routerx::emitRejectName(reject);
    return c;
}

enum class PerimeterSide {
    Left,
    Right,
    Bottom,
    Top,
};

const char* perimeterSideName(PerimeterSide side) {
    switch (side) {
        case PerimeterSide::Left:   return "L";
        case PerimeterSide::Right:  return "R";
        case PerimeterSide::Bottom: return "B";
        case PerimeterSide::Top:    return "T";
    }
    return "?";
}

std::vector<char> perimeterChannelMask(const Design& design,
                                       PerimeterSide side) {
    std::vector<char> mask(design.channels.size(), 0);
    if (!design.hasRoutingCore || design.routingCoreW <= EPS ||
        design.routingCoreH <= EPS) {
        return {};
    }
    const double coreRight = design.routingCoreX + design.routingCoreW;
    const double coreTop = design.routingCoreY + design.routingCoreH;
    bool sideHasHalo = false;
    switch (side) {
        case PerimeterSide::Left:
            sideHasHalo = design.routingCoreX > EPS;
            break;
        case PerimeterSide::Right:
            sideHasHalo = design.outlineW - coreRight > EPS;
            break;
        case PerimeterSide::Bottom:
            sideHasHalo = design.routingCoreY > EPS;
            break;
        case PerimeterSide::Top:
            sideHasHalo = design.outlineH - coreTop > EPS;
            break;
    }
    if (!sideHasHalo) return {};

    for (size_t ci = 0; ci < design.channels.size(); ++ci) {
        const Rect& r = design.channels[ci].rect;
        bool onSide = false;
        switch (side) {
            case PerimeterSide::Left:
                onSide = r.x <= EPS && rectRight(r) > 0.0 &&
                         rectRight(r) <= design.routingCoreX + EPS;
                break;
            case PerimeterSide::Right:
                onSide = std::fabs(rectRight(r) - design.outlineW) <= EPS &&
                         r.x >= coreRight - EPS;
                break;
            case PerimeterSide::Bottom:
                onSide = r.y <= EPS && rectTop(r) > 0.0 &&
                         rectTop(r) <= design.routingCoreY + EPS;
                break;
            case PerimeterSide::Top:
                onSide = std::fabs(rectTop(r) - design.outlineH) <= EPS &&
                         r.y >= coreTop - EPS;
                break;
        }
        if (onSide) mask[ci] = 1;
    }
    if (std::find(mask.begin(), mask.end(), static_cast<char>(1)) == mask.end()) {
        return {};
    }
    return mask;
}

void appendPerimeterCandidates(
        const Design& design,
        const routerx::RxGraph& graph,
        const routerx::DemandedPair& p,
        routerx::SearchCostMode costMode,
        std::vector<routerx::LiteCandidate>& raw,
        long long& searchCalls) {
    if (!design.hasRoutingCore) return;
    static constexpr PerimeterSide kSides[] = {
        PerimeterSide::Left, PerimeterSide::Right,
        PerimeterSide::Bottom, PerimeterSide::Top
    };
    const bool report = std::getenv("RX_LITE_PERIMETER_REPORT") != nullptr;
    std::ostringstream reportLine;
    if (report) {
        reportLine << "[RouterLite/Perimeter] pair="
                   << design.blocks[p.src].spec.name << "->"
                   << design.blocks[p.dst].spec.name;
    }
    for (PerimeterSide side : kSides) {
        std::vector<char> mask = perimeterChannelMask(design, side);
        if (mask.empty()) {
            if (report) reportLine << " " << perimeterSideName(side) << "=no_mask";
            continue;
        }
        routerx::SearchOptions opt;
        opt.mode = routerx::Mode::AllowFeedthrough;
        opt.costMode = costMode;
        opt.requiredChannelMask = std::move(mask);
        ++searchCalls;
        const routerx::SearchResult r = routerx::RxSearch::findPathWith(
            design, graph, p.src, p.dst, opt);
        if (report) {
            reportLine << " " << perimeterSideName(side)
                       << (r.found ? "=path" : "=unreachable");
        }
        if (!r.found) continue;
        raw.push_back(makeCandidate(
            design, p, routerx::LiteCandidateFamily::PerimeterDetourUnified,
            stepsFromSearch(design, graph, p, r)));
    }
    if (report) std::cerr << reportLine.str() << "\n";
}

std::vector<routerx::LiteCandidate> generateR2b1Candidates(
        const Design& design,
        const routerx::RxGraph& graph,
        const routerx::DemandedPair& p,
        const LiteRoutedPair& r0,
        routerx::SearchCostMode costMode,
        AccessAltUniverseMode accessMode,
        const CapacityAwareConfig& capacityConfig,
        long long& searchCalls,
        long long& altAccessSearchCalls) {
    std::vector<routerx::LiteCandidate> raw;

    if (r0.found) {
        raw.push_back(makeCandidate(design, p, routerx::LiteCandidateFamily::R0Fallback,
                                    r0.route.steps, true));
    }

    const int directEdge = directAdjacencyEdgeLite(design, p.src, p.dst);
    if (directEdge != 0) {
        raw.push_back(makeCandidate(design, p, routerx::LiteCandidateFamily::Direct,
                                    directStepsForPair(design, p, directEdge)));
        return raw; // direct has zero wire; other families cannot improve it.
    }

    ++searchCalls;
    routerx::SearchResult unified = findLitePathInMode(
        design, graph, p.src, p.dst, routerx::Mode::AllowFeedthrough, costMode);
    if (unified.found) {
        raw.push_back(makeCandidate(
            design, p, routerx::LiteCandidateFamily::BaseUnifiedShortest,
            stepsFromSearch(design, graph, p, unified)));
    }

    if (capacityConfig.enabled) {
        routerx::SearchOptions opt;
        opt.mode = routerx::Mode::AllowFeedthrough;
        opt.costMode = costMode;
        opt.capacityAwareStressMode = capacityConfig.stressMode;
        opt.capacityAwareWeight = capacityConfig.weight;
        opt.capacityAwareNets = p.nets;
        opt.capacityAwareCapFloor = capacityConfig.capFloor;
        opt.capacityAwareTargetUtil = capacityConfig.targetUtil;
        opt.capacityAwareStressCap = capacityConfig.stressCap;
        ++searchCalls;
        const routerx::SearchResult r =
            routerx::RxSearch::findPathWith(design, graph, p.src, p.dst, opt);
        if (r.found) {
            raw.push_back(makeCandidate(
                design, p, routerx::LiteCandidateFamily::CapacityAwareUnified,
                stepsFromSearch(design, graph, p, r)));
        }
    }

    if (accessMode == AccessAltUniverseMode::FullUnified) {
        const std::vector<int>& srcAccesses = graph.accessesOf(p.src);
        const std::vector<int>& dstAccesses = graph.accessesOf(p.dst);
        for (size_t si = 0; si < srcAccesses.size(); ++si) {
            for (size_t di = 0; di < dstAccesses.size(); ++di) {
                routerx::SearchOptions opt;
                opt.mode = routerx::Mode::AllowFeedthrough;
                opt.onlySrcAccess = static_cast<int>(si);
                opt.onlyDstAccess = static_cast<int>(di);
                opt.costMode = costMode;
                ++searchCalls;
                ++altAccessSearchCalls;
                const routerx::SearchResult r =
                    routerx::RxSearch::findPathWith(design, graph, p.src, p.dst, opt);
                if (!r.found) continue;
                raw.push_back(makeCandidate(
                    design, p, routerx::LiteCandidateFamily::AccessAltUnified,
                    stepsFromSearch(design, graph, p, r)));
            }
        }
        appendPerimeterCandidates(
            design, graph, p, costMode, raw, searchCalls);
        return raw;
    }

    constexpr int kAccessChoicesPerSide = 3;
    constexpr int kAltAccessSearchCap = 8;
    std::vector<AccessChoice> srcChoices =
        rankedChannelAccessChoices(design, graph, p.src, p.dst, kAccessChoicesPerSide);
    std::vector<AccessChoice> dstChoices =
        rankedChannelAccessChoices(design, graph, p.dst, p.src, kAccessChoicesPerSide);
    struct PairChoice { int src = -1; int dst = -1; double score = 0.0; };
    std::vector<PairChoice> pairs;
    for (const AccessChoice& s : srcChoices) {
        for (const AccessChoice& d : dstChoices) {
            pairs.push_back(PairChoice{s.pos, d.pos, s.score + d.score});
        }
    }
    std::sort(pairs.begin(), pairs.end(), [](const PairChoice& a, const PairChoice& b) {
        if (a.score != b.score) return a.score < b.score;
        if (a.src != b.src) return a.src < b.src;
        return a.dst < b.dst;
    });
    if (static_cast<int>(pairs.size()) > kAltAccessSearchCap) {
        pairs.resize(kAltAccessSearchCap);
    }
    for (const PairChoice& pc : pairs) {
        routerx::SearchOptions opt;
        opt.mode = routerx::Mode::ChannelOnly;
        opt.onlySrcAccess = pc.src;
        opt.onlyDstAccess = pc.dst;
        opt.costMode = costMode;
        ++searchCalls;
        ++altAccessSearchCalls;
        const routerx::SearchResult r =
            routerx::RxSearch::findPathWith(design, graph, p.src, p.dst, opt);
        if (!r.found) continue;
        raw.push_back(makeCandidate(design, p, routerx::LiteCandidateFamily::AccessAltUnified,
                                    stepsFromSearch(design, graph, p, r)));
    }

    appendPerimeterCandidates(
        design, graph, p, costMode, raw, searchCalls);
    return raw;
}

std::vector<double> avoidHotWeights() {
    const char* env = std::getenv("RX_LITE_AVOID_HOT_WEIGHTS");
    const std::string spec = (env && *env) ? std::string(env)
                                          : std::string("20000");
    std::vector<double> out;
    std::stringstream ss(spec);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (item.empty()) continue;
        char* end = nullptr;
        const double v = std::strtod(item.c_str(), &end);
        if (end && *end == '\0' && v > 0.0) out.push_back(v);
    }
    if (out.empty()) out = {20000.0};
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

std::string weightsString(const std::vector<double>& weights) {
    std::ostringstream oss;
    for (size_t i = 0; i < weights.size(); ++i) {
        if (i) oss << ",";
        oss << weights[i];
    }
    return oss.str();
}

double avoidHotThreshold() {
    const char* env = std::getenv("RX_LITE_HOT_THRESHOLD");
    if (!env || std::string(env).empty()) return 1.0;
    char* end = nullptr;
    const double v = std::strtod(env, &end);
    if (end && *end == '\0' && v >= 0.0) return v;
    std::cerr << "[rx_lite] Unknown RX_LITE_HOT_THRESHOLD=" << env
              << " (expected non-negative number, e.g. 1.0|0.95|0.9)\n";
    return -1.0;
}

struct HotAxisRatios {
    std::vector<double> lr;
    std::vector<double> tb;
};

std::vector<routerx::LiteCandidate> generateAvoidHotCandidates(
        const Design& design,
        const routerx::RxGraph& graph,
        const routerx::DemandedPair& p,
        routerx::SearchCostMode costMode,
        const HotAxisRatios& hot,
        const std::vector<double>& weights,
        long long& searchCalls,
        long long& avoidHotSearchCalls) {
    std::vector<routerx::LiteCandidate> raw;
    for (double w : weights) {
        routerx::SearchOptions opt;
        opt.mode = routerx::Mode::AllowFeedthrough;
        opt.costMode = costMode;
        opt.hotChannelLrRatio = hot.lr;
        opt.hotChannelTbRatio = hot.tb;
        opt.hotChannelWeight = w;
        opt.hotChannelNets = p.nets;
        ++searchCalls;
        ++avoidHotSearchCalls;
        const routerx::SearchResult r =
            routerx::RxSearch::findPathWith(design, graph, p.src, p.dst, opt);
        if (!r.found) continue;
        raw.push_back(makeCandidate(design, p,
                                    routerx::LiteCandidateFamily::AvoidHotUnified,
                                    stepsFromSearch(design, graph, p, r)));
    }
    return raw;
}

long long accessAltUnifiedLegalViewCount(const routerx::RxGraph& graph,
                                         const routerx::DemandedPair& p) {
    const long long src = static_cast<long long>(graph.accessesOf(p.src).size());
    const long long dst = static_cast<long long>(graph.accessesOf(p.dst).size());
    return src * dst;
}

struct HotChannelAxis {
    std::string key;
    std::string name;
    std::string axis;
    int channelIndex = -1;
    double overflow = 0.0;
    double used = 0.0;
    double cap = 0.0;
    double usageRate = 0.0;
    double hotness = 0.0;
};

std::vector<HotChannelAxis> hotChannelAxes(
        const Design& design,
        const routerx::RxResourceLedgerLite& ledger,
        double threshold = 1.0) {
    std::vector<HotChannelAxis> out;
    for (int i = 0; i < ledger.channelCount() &&
                    i < static_cast<int>(design.channels.size()); ++i) {
        const routerx::LiteChannelUse& u = ledger.channel(i);
        const std::string& name = design.channels[i].name;
        const double ofLR = std::max(0.0, u.usedLR - u.capLR);
        const double ofTB = std::max(0.0, u.usedTB - u.capTB);
        const double rateLR = u.capLR > 0.0 ? u.usedLR / u.capLR : 0.0;
        const double rateTB = u.capTB > 0.0 ? u.usedTB / u.capTB : 0.0;
        const double hotLR = rateLR - threshold;
        const double hotTB = rateTB - threshold;
        if (hotLR > 1.0e-9) {
            out.push_back(HotChannelAxis{name + ":LR", name, "LR", i,
                                         ofLR, u.usedLR, u.capLR,
                                         rateLR, hotLR});
        }
        if (hotTB > 1.0e-9) {
            out.push_back(HotChannelAxis{name + ":TB", name, "TB", i,
                                         ofTB, u.usedTB, u.capTB,
                                         rateTB, hotTB});
        }
    }
    std::sort(out.begin(), out.end(), [](const HotChannelAxis& a,
                                         const HotChannelAxis& b) {
        if (a.hotness != b.hotness) return a.hotness > b.hotness;
        if (a.overflow != b.overflow) return a.overflow > b.overflow;
        return a.key < b.key;
    });
    return out;
}

HotAxisRatios hotAxisRatios(const Design& design,
                            const routerx::RxResourceLedgerLite& ledger,
                            double threshold = 1.0) {
    HotAxisRatios out;
    out.lr.assign(design.channels.size(), 0.0);
    out.tb.assign(design.channels.size(), 0.0);
    for (int i = 0; i < ledger.channelCount() &&
                    i < static_cast<int>(design.channels.size()); ++i) {
        const routerx::LiteChannelUse& u = ledger.channel(i);
        if (u.capLR > 0.0) {
            out.lr[i] = std::max(0.0, u.usedLR / u.capLR - threshold);
        }
        if (u.capTB > 0.0) {
            out.tb[i] = std::max(0.0, u.usedTB / u.capTB - threshold);
        }
    }
    return out;
}

int channelIndexByName(const Design& design, const std::string& name) {
    for (int i = 0; i < static_cast<int>(design.channels.size()); ++i) {
        if (design.channels[i].name == name) return i;
    }
    return -1;
}

std::string hotChannelAxisString(const HotChannelAxis& h);
void emitHotWorld(const std::string& prefix,
                  const Design& design,
                  const routerx::RxResourceLedgerLite& ledger,
                  double threshold);

bool selectableCandidate(const routerx::LiteCandidate& c);

bool routeTouchesHotAxis(const Design& design,
                         const std::vector<RouteStep>& steps,
                         const HotAxisRatios& hot) {
    for (size_t i = 0; i + 1 < steps.size(); ++i) {
        const RouteStep& a = steps[i];
        const RouteStep& b = steps[i + 1];
        if (a.rectName != b.rectName) continue;
        const int ci = channelIndexByName(design, a.rectName);
        if (ci < 0) continue;

        bool lr = false;
        bool tb = false;
        routerx::RxResource::traversalAxes(a.edge, b.edge, lr, tb);
        if (lr && ci < static_cast<int>(hot.lr.size()) && hot.lr[ci] > 0.0) {
            return true;
        }
        if (tb && ci < static_cast<int>(hot.tb.size()) && hot.tb[ci] > 0.0) {
            return true;
        }
    }
    return false;
}

struct RouteAxisDemand {
    std::vector<double> lr;
    std::vector<double> tb;
};

RouteAxisDemand routeAxisDemand(const Design& design,
                                const std::vector<RouteStep>& steps,
                                int nets) {
    RouteAxisDemand out;
    out.lr.assign(design.channels.size(), 0.0);
    out.tb.assign(design.channels.size(), 0.0);
    for (size_t i = 0; i + 1 < steps.size(); ++i) {
        const RouteStep& a = steps[i];
        const RouteStep& b = steps[i + 1];
        if (a.rectName != b.rectName) continue;
        const int ci = channelIndexByName(design, a.rectName);
        if (ci < 0) continue;

        bool lr = false;
        bool tb = false;
        routerx::RxResource::traversalAxes(a.edge, b.edge, lr, tb);
        if (lr && ci < static_cast<int>(out.lr.size())) out.lr[ci] += nets;
        if (tb && ci < static_cast<int>(out.tb.size())) out.tb[ci] += nets;
    }
    return out;
}

struct SegmentTargetAxis {
    int channelIndex = -1;
    bool lr = false;

    bool operator<(const SegmentTargetAxis& o) const {
        if (channelIndex != o.channelIndex) return channelIndex < o.channelIndex;
        return lr < o.lr;
    }
    bool operator==(const SegmentTargetAxis& o) const {
        return channelIndex == o.channelIndex && lr == o.lr;
    }
};

struct SegmentAnchor {
    std::string objectName;
    int routeStepIndex = -1;
    bool isEndpoint = false;
    int fixedOutsideEdge = 0; // A: prefix enters here; B: suffix exits here.
    int graphNode = -1;       // channels/SOFT only; HARD/EDGE endpoints are -1.
};

struct SegmentWindow {
    int hotPairStart = -1;    // odd RouteStep index of first hot traversal pair
    int hotPairEnd = -1;      // odd RouteStep index of last hot traversal pair
    int replaceBegin = -1;    // first replaced step, exclusive of anchor A
    int replaceEnd = -1;      // last replaced step, exclusive of anchor B
    SegmentAnchor a;
    SegmentAnchor b;
    std::vector<SegmentTargetAxis> targets;
    double originalTargetDemand = 0.0;
};

bool targetAxisContains(const std::vector<SegmentTargetAxis>& targets,
                        int channelIndex,
                        bool lr) {
    return std::find_if(targets.begin(), targets.end(),
        [&](const SegmentTargetAxis& t) {
            return t.channelIndex == channelIndex && t.lr == lr;
        }) != targets.end();
}

double targetAxisDemandForSteps(const Design& design,
                                const std::vector<RouteStep>& steps,
                                int nets,
                                const std::vector<SegmentTargetAxis>& targets) {
    if (targets.empty()) return 0.0;
    const RouteAxisDemand d = routeAxisDemand(design, steps, nets);
    double total = 0.0;
    for (const SegmentTargetAxis& t : targets) {
        if (t.channelIndex < 0) continue;
        if (t.lr) {
            if (t.channelIndex < static_cast<int>(d.lr.size())) {
                total += d.lr[t.channelIndex];
            }
        } else {
            if (t.channelIndex < static_cast<int>(d.tb.size())) {
                total += d.tb[t.channelIndex];
            }
        }
    }
    return total;
}

double nonTargetHotAxisDemandForSteps(
        const Design& design,
        const std::vector<RouteStep>& steps,
        int nets,
        const HotAxisRatios& hot,
        const std::vector<SegmentTargetAxis>& targets) {
    const RouteAxisDemand d = routeAxisDemand(design, steps, nets);
    double total = 0.0;
    const int n = std::min(static_cast<int>(d.lr.size()),
                           static_cast<int>(hot.lr.size()));
    for (int i = 0; i < n; ++i) {
        if (hot.lr[i] > 0.0 && !targetAxisContains(targets, i, true)) {
            total += d.lr[i];
        }
    }
    const int m = std::min(static_cast<int>(d.tb.size()),
                           static_cast<int>(hot.tb.size()));
    for (int i = 0; i < m; ++i) {
        if (hot.tb[i] > 0.0 && !targetAxisContains(targets, i, false)) {
            total += d.tb[i];
        }
    }
    return total;
}

int graphNodeForStepObject(const Design& design,
                           const routerx::RxGraph& graph,
                           const std::string& name);

std::vector<SegmentWindow> segmentWindowsForRoute(
        const Design& design,
        const routerx::RxGraph& graph,
        const std::vector<RouteStep>& steps,
        int nets,
        const HotAxisRatios& hot,
        int anchorShiftDepth) {
    std::vector<SegmentWindow> out;
    SegmentWindow current;
    anchorShiftDepth = std::max(0, anchorShiftDepth);

    auto addWindow = [&](const SegmentWindow& base,
                         int aIndex,
                         int bIndex) {
        SegmentWindow w = base;
        w.a.routeStepIndex = aIndex;
        w.b.routeStepIndex = bIndex;
        w.replaceBegin = w.a.routeStepIndex + 1;
        w.replaceEnd = w.b.routeStepIndex - 1;
        if (w.a.routeStepIndex < 0 ||
            w.b.routeStepIndex >= static_cast<int>(steps.size()) ||
            w.replaceBegin > w.replaceEnd ||
            w.targets.empty()) {
            return;
        }
        w.a.objectName = steps[w.a.routeStepIndex].rectName;
        w.b.objectName = steps[w.b.routeStepIndex].rectName;
        w.a.isEndpoint = (w.a.routeStepIndex == 0);
        w.b.isEndpoint =
            (w.b.routeStepIndex + 1 == static_cast<int>(steps.size()));
        w.a.fixedOutsideEdge = w.a.isEndpoint
            ? 0
            : steps[w.a.routeStepIndex - 1].edge;
        w.b.fixedOutsideEdge = w.b.isEndpoint
            ? 0
            : steps[w.b.routeStepIndex + 1].edge;
        w.a.graphNode = graphNodeForStepObject(design, graph, w.a.objectName);
        w.b.graphNode = graphNodeForStepObject(design, graph, w.b.objectName);
        w.originalTargetDemand =
            targetAxisDemandForSteps(design, steps, nets, w.targets);
        out.push_back(w);
    };

    auto closeRun = [&]() {
        if (current.hotPairStart < 0) return;
        std::sort(current.targets.begin(), current.targets.end());
        current.targets.erase(std::unique(current.targets.begin(), current.targets.end()),
                              current.targets.end());
        const int baseA = current.hotPairStart - 1;
        const int baseB = current.hotPairEnd + 2;
        std::vector<int> aChoices{baseA};
        std::vector<int> bChoices{baseB};
        for (int d = 1; d <= anchorShiftDepth; ++d) {
            const int aIndex = baseA - 2 * d;
            if (aIndex >= 0) aChoices.push_back(aIndex);
            const int bIndex = baseB + 2 * d;
            if (bIndex < static_cast<int>(steps.size())) {
                bChoices.push_back(bIndex);
            }
        }
        std::sort(aChoices.begin(), aChoices.end());
        aChoices.erase(std::unique(aChoices.begin(), aChoices.end()),
                       aChoices.end());
        std::sort(bChoices.begin(), bChoices.end());
        bChoices.erase(std::unique(bChoices.begin(), bChoices.end()),
                       bChoices.end());
        for (int aIndex : aChoices) {
            for (int bIndex : bChoices) {
                addWindow(current, aIndex, bIndex);
            }
        }
        current = SegmentWindow{};
    };

    for (int i = 1; i + 1 < static_cast<int>(steps.size()); i += 2) {
        const RouteStep& a = steps[i];
        const RouteStep& b = steps[i + 1];
        bool hotTraversal = false;
        std::vector<SegmentTargetAxis> pairTargets;
        if (a.rectName == b.rectName) {
            const int ci = channelIndexByName(design, a.rectName);
            if (ci >= 0) {
                bool lr = false;
                bool tb = false;
                routerx::RxResource::traversalAxes(a.edge, b.edge, lr, tb);
                if (lr && ci < static_cast<int>(hot.lr.size()) && hot.lr[ci] > 0.0) {
                    hotTraversal = true;
                    pairTargets.push_back(SegmentTargetAxis{ci, true});
                }
                if (tb && ci < static_cast<int>(hot.tb.size()) && hot.tb[ci] > 0.0) {
                    hotTraversal = true;
                    pairTargets.push_back(SegmentTargetAxis{ci, false});
                }
            }
        }

        if (!hotTraversal) {
            closeRun();
            continue;
        }
        if (current.hotPairStart < 0) {
            current.hotPairStart = i;
            current.hotPairEnd = i;
        } else {
            current.hotPairEnd = i;
        }
        current.targets.insert(current.targets.end(), pairTargets.begin(), pairTargets.end());
    }
    closeRun();
    return out;
}

int blockIndexByName(const Design& design, const std::string& name) {
    for (int i = 0; i < static_cast<int>(design.blocks.size()); ++i) {
        if (design.blocks[i].spec.name == name) return i;
    }
    return -1;
}

int graphNodeForStepObject(const Design& design,
                           const routerx::RxGraph& graph,
                           const std::string& name) {
    const int ci = channelIndexByName(design, name);
    if (ci >= 0) return graph.nodeForChannel(ci);
    const int bi = blockIndexByName(design, name);
    if (bi >= 0) return graph.nodeForSoftBlock(bi);
    return -1;
}

const Rect* objectRectByNameLite(const Design& design, const std::string& name) {
    const int bi = blockIndexByName(design, name);
    if (bi >= 0) return &design.blocks[bi].rect;
    const int ci = channelIndexByName(design, name);
    if (ci >= 0) return &design.channels[ci].rect;
    return nullptr;
}

const Rect& graphNodeRectLite(const Design& design,
                              const routerx::RxGraph& graph,
                              int node) {
    const routerx::Node& n = graph.nodes()[node];
    return n.kind == routerx::NodeKind::Channel
        ? design.channels[n.designIndex].rect
        : design.blocks[n.designIndex].rect;
}

std::string graphNodeNameLite(const Design& design,
                              const routerx::RxGraph& graph,
                              int node) {
    const routerx::Node& n = graph.nodes()[node];
    return n.kind == routerx::NodeKind::Channel
        ? design.channels[n.designIndex].name
        : design.blocks[n.designIndex].spec.name;
}

struct AnchorContact {
    int node = -1;
    int edgeOnAnchor = 0;
    int edgeOnNode = 0;
};

std::vector<AnchorContact> departureContactsForAnchor(
        const Design& design,
        const routerx::RxGraph& graph,
        const SegmentAnchor& anchor) {
    std::vector<AnchorContact> out;
    if (anchor.routeStepIndex < 0) return out;
    if (anchor.isEndpoint) {
        const int bi = blockIndexByName(design, anchor.objectName);
        if (bi < 0) return out;
        const std::vector<int>& accesses = graph.accessesOf(bi);
        for (int ai : accesses) {
            const routerx::Access& a = graph.accesses()[ai];
            const int enterOnNode = routerx::oppositeEdge(a.portEdge);
            if (enterOnNode == 0) continue;
            out.push_back(AnchorContact{a.node, a.portEdge, enterOnNode});
        }
        return out;
    }

    const int anchorNode = anchor.graphNode;
    if (anchorNode < 0) return out;
    std::vector<int> incident = graph.incident(anchorNode);
    std::sort(incident.begin(), incident.end());
    for (int eid : incident) {
        const routerx::Edge& e = graph.edges()[eid];
        const bool anchorIsA = e.a == anchorNode;
        const int edgeOnAnchor = anchorIsA ? e.edgeOnA : e.edgeOnB;
        const int nbr = anchorIsA ? e.b : e.a;
        const int edgeOnNbr = anchorIsA ? e.edgeOnB : e.edgeOnA;
        if (edgeOnAnchor == anchor.fixedOutsideEdge) continue;
        out.push_back(AnchorContact{nbr, edgeOnAnchor, edgeOnNbr});
    }
    return out;
}

std::vector<AnchorContact> arrivalContactsForAnchor(
        const Design& design,
        const routerx::RxGraph& graph,
        const SegmentAnchor& anchor) {
    std::vector<AnchorContact> out;
    if (anchor.routeStepIndex < 0) return out;
    if (anchor.isEndpoint) {
        const int bi = blockIndexByName(design, anchor.objectName);
        if (bi < 0) return out;
        const std::vector<int>& accesses = graph.accessesOf(bi);
        for (int ai : accesses) {
            const routerx::Access& a = graph.accesses()[ai];
            const int exitOnNode = routerx::oppositeEdge(a.portEdge);
            if (exitOnNode == 0) continue;
            out.push_back(AnchorContact{a.node, a.portEdge, exitOnNode});
        }
        return out;
    }

    const int anchorNode = anchor.graphNode;
    if (anchorNode < 0) return out;
    std::vector<int> incident = graph.incident(anchorNode);
    std::sort(incident.begin(), incident.end());
    for (int eid : incident) {
        const routerx::Edge& e = graph.edges()[eid];
        const bool anchorIsA = e.a == anchorNode;
        const int edgeOnAnchor = anchorIsA ? e.edgeOnA : e.edgeOnB;
        const int nbr = anchorIsA ? e.b : e.a;
        const int edgeOnNbr = anchorIsA ? e.edgeOnB : e.edgeOnA;
        if (edgeOnAnchor == anchor.fixedOutsideEdge) continue;
        out.push_back(AnchorContact{nbr, edgeOnAnchor, edgeOnNbr});
    }
    return out;
}

struct SegmentLocalResult {
    bool found = false;
    std::vector<routerx::Hop> hops;
    double targetDemand = 0.0;
    double otherHotDemand = 0.0;
    double overlapDemand = 0.0;
    double softOverlapDemand = 0.0;
    double wire = 0.0;
    bool usedFeedthrough = false;
    int feature = 0;
    int rank = 1;
};

enum class SegmentSearchFeature {
    TargetHotRelief = 0,
    GlobalHotSafe = 1,
    ResourceDiverse = 2,
};

constexpr int kSegmentSearchFeatureCount = 3;

int segmentFeatureIndex(SegmentSearchFeature f) {
    return static_cast<int>(f);
}

const char* segmentFeatureName(SegmentSearchFeature f) {
    switch (f) {
        case SegmentSearchFeature::TargetHotRelief: return "TargetHotRelief";
        case SegmentSearchFeature::GlobalHotSafe: return "GlobalHotSafe";
        case SegmentSearchFeature::ResourceDiverse: return "ResourceDiverse";
    }
    return "Unknown";
}

struct SegmentSearchCost {
    double target = 0.0;
    double otherHot = 0.0;
    double overlap = 0.0;
    double softOverlap = 0.0;
    double wire = 0.0;
};

struct SegmentSearchPolicy {
    SegmentSearchFeature feature = SegmentSearchFeature::TargetHotRelief;
    bool weighted = false;
    double targetWeight = 0.0;
};

struct SegmentSearchConfig {
    std::vector<SegmentSearchPolicy> policies;
    int topK = 1;
    int anchorShiftDepth = 0;
    std::string name;
};

int segmentTopKFromEnv() {
    const char* env = std::getenv("RX_LITE_SEGMENT_TOPK");
    if (!env || std::string(env).empty()) return 1;
    char* end = nullptr;
    const long v = std::strtol(env, &end, 10);
    if (end && *end == '\0' && v >= 1 && v <= 8) {
        return static_cast<int>(v);
    }
    std::cerr << "[rx_lite] Ignoring invalid RX_LITE_SEGMENT_TOPK="
              << env << " (expected integer 1..8)\n";
    return 1;
}

SegmentSearchConfig segmentSearchConfigFromEnv() {
    const int topK = segmentTopKFromEnv();
    const char* abShift = std::getenv("RX_LITE_SEGMENT_AB_SHIFT");
    int anchorShiftDepth = 2;  // adopted SegmentBypass v0 sweep point: 3x3.
    if (abShift) {
        const std::string v(abShift);
        if (v == "on") anchorShiftDepth = 1;   // legacy 2x2 shorthand.
        if (v == "off") anchorShiftDepth = 0;
    }
    const char* shiftDepthEnv = std::getenv("RX_LITE_SEGMENT_AB_SHIFT_DEPTH");
    if (shiftDepthEnv && !std::string(shiftDepthEnv).empty()) {
        char* end = nullptr;
        const long v = std::strtol(shiftDepthEnv, &end, 10);
        if (end && *end == '\0' && v >= 0 && v <= 8) {
            anchorShiftDepth = static_cast<int>(v);
        } else {
            std::cerr << "[rx_lite] Ignoring invalid "
                      << "RX_LITE_SEGMENT_AB_SHIFT_DEPTH=" << shiftDepthEnv
                      << " (expected integer 0..8)\n";
        }
    }
    const char* multi = std::getenv("RX_LITE_SEGMENT_MULTI_FEATURE");
    const bool multiOn = multi && std::string(multi) == "on";
    if (multiOn) {
        SegmentSearchConfig cfg;
        cfg.topK = topK;
        cfg.anchorShiftDepth = anchorShiftDepth;
        cfg.policies.push_back(
            SegmentSearchPolicy{SegmentSearchFeature::TargetHotRelief, false, 0.0});
        cfg.policies.push_back(
            SegmentSearchPolicy{SegmentSearchFeature::GlobalHotSafe, false, 0.0});
        cfg.policies.push_back(
            SegmentSearchPolicy{SegmentSearchFeature::ResourceDiverse, false, 0.0});
        cfg.name = "multi_feature_top" + std::to_string(topK);
        if (cfg.anchorShiftDepth > 0) {
            cfg.name += "_ab_shift" + std::to_string(cfg.anchorShiftDepth);
        }
        return cfg;
    }

    const char* env = std::getenv("RX_LITE_SEGMENT_TARGET_WEIGHT");
    if (env && !std::string(env).empty()) {
        char* end = nullptr;
        const double v = std::strtod(env, &end);
        if (end && *end == '\0' && v >= 0.0) {
            SegmentSearchConfig cfg;
            cfg.topK = topK;
            cfg.anchorShiftDepth = anchorShiftDepth;
            cfg.policies.push_back(
                SegmentSearchPolicy{SegmentSearchFeature::TargetHotRelief, true, v});
            cfg.name = "truewire_plus_" + std::to_string(v) + "_target_hot";
            if (cfg.anchorShiftDepth > 0) {
                cfg.name += "_ab_shift" + std::to_string(cfg.anchorShiftDepth);
            }
            return cfg;
        }
        std::cerr << "[rx_lite] Ignoring invalid RX_LITE_SEGMENT_TARGET_WEIGHT="
                  << env << " (expected non-negative number)\n";
    }

    SegmentSearchConfig cfg;
    cfg.topK = topK;
    cfg.anchorShiftDepth = anchorShiftDepth;
    cfg.policies.push_back(
        SegmentSearchPolicy{SegmentSearchFeature::TargetHotRelief, false, 0.0});
    cfg.name = "lexicographic_target_hot_axis_then_truewire";
    if (cfg.anchorShiftDepth > 0) {
        cfg.name += "_ab_shift" + std::to_string(cfg.anchorShiftDepth);
    }
    return cfg;
}

double segmentWeightedScalar(const SegmentSearchCost& c,
                             const SegmentSearchPolicy& p) {
    return c.wire + p.targetWeight * c.target;
}

bool segmentCostLess(const SegmentSearchCost& a,
                     const SegmentSearchCost& b,
                     const SegmentSearchPolicy& policy,
                     double eps = 1.0e-12) {
    if (policy.weighted) {
        const double ka = segmentWeightedScalar(a, policy);
        const double kb = segmentWeightedScalar(b, policy);
        if (ka < kb - eps) return true;
        if (ka > kb + eps) return false;
        if (a.target < b.target - eps) return true;
        if (a.target > b.target + eps) return false;
        return a.wire < b.wire - eps;
    }
    auto cmp = [&](double av, double bv) {
        if (av < bv - eps) return -1;
        if (av > bv + eps) return 1;
        return 0;
    };
    int c = cmp(a.target, b.target);
    if (c != 0) return c < 0;
    switch (policy.feature) {
        case SegmentSearchFeature::TargetHotRelief:
            break;
        case SegmentSearchFeature::GlobalHotSafe:
            c = cmp(a.otherHot, b.otherHot);
            if (c != 0) return c < 0;
            break;
        case SegmentSearchFeature::ResourceDiverse:
            c = cmp(a.overlap, b.overlap);
            if (c != 0) return c < 0;
            c = cmp(a.softOverlap, b.softOverlap);
            if (c != 0) return c < 0;
            break;
    }
    return a.wire < b.wire - eps;
}

double targetAxisTraversalCost(const Design& design,
                               const routerx::RxGraph& graph,
                               const std::vector<SegmentTargetAxis>& targets,
                               int node,
                               int inEdge,
                               int outEdge) {
    const routerx::Node& n = graph.nodes()[node];
    if (n.kind != routerx::NodeKind::Channel) return 0.0;
    bool lr = false;
    bool tb = false;
    routerx::RxResource::traversalAxes(inEdge, outEdge, lr, tb);
    double c = 0.0;
    if (lr && targetAxisContains(targets, n.designIndex, true)) c += 1.0;
    if (tb && targetAxisContains(targets, n.designIndex, false)) c += 1.0;
    (void)design;
    return c;
}

struct SegmentResourceMask {
    std::set<SegmentTargetAxis> channelAxes;
    std::set<int> softNodes;
};

SegmentResourceMask segmentResourceMaskForWindow(
        const Design& design,
        const routerx::RxGraph& graph,
        const std::vector<RouteStep>& steps,
        const SegmentWindow& window) {
    SegmentResourceMask mask;
    for (int i = window.replaceBegin;
         i + 1 <= window.replaceEnd && i + 1 < static_cast<int>(steps.size());
         i += 2) {
        const RouteStep& a = steps[i];
        const RouteStep& b = steps[i + 1];
        if (a.rectName != b.rectName) continue;
        const int ci = channelIndexByName(design, a.rectName);
        if (ci >= 0) {
            bool lr = false;
            bool tb = false;
            routerx::RxResource::traversalAxes(a.edge, b.edge, lr, tb);
            if (lr) mask.channelAxes.insert(SegmentTargetAxis{ci, true});
            if (tb) mask.channelAxes.insert(SegmentTargetAxis{ci, false});
            continue;
        }
        const int node = graphNodeForStepObject(design, graph, a.rectName);
        if (node >= 0 &&
            graph.nodes()[node].kind == routerx::NodeKind::SoftFeedthrough) {
            mask.softNodes.insert(node);
        }
    }
    return mask;
}

double otherHotTraversalCost(const Design& design,
                             const routerx::RxGraph& graph,
                             const HotAxisRatios& hot,
                             const std::vector<SegmentTargetAxis>& targets,
                             int node,
                             int inEdge,
                             int outEdge) {
    const routerx::Node& n = graph.nodes()[node];
    if (n.kind != routerx::NodeKind::Channel) return 0.0;
    bool lr = false;
    bool tb = false;
    routerx::RxResource::traversalAxes(inEdge, outEdge, lr, tb);
    double c = 0.0;
    if (lr && n.designIndex < static_cast<int>(hot.lr.size()) &&
        hot.lr[n.designIndex] > 0.0 &&
        !targetAxisContains(targets, n.designIndex, true)) {
        c += 1.0;
    }
    if (tb && n.designIndex < static_cast<int>(hot.tb.size()) &&
        hot.tb[n.designIndex] > 0.0 &&
        !targetAxisContains(targets, n.designIndex, false)) {
        c += 1.0;
    }
    (void)design;
    return c;
}

double resourceOverlapTraversalCost(const routerx::RxGraph& graph,
                                    const SegmentResourceMask& mask,
                                    int node,
                                    int inEdge,
                                    int outEdge) {
    const routerx::Node& n = graph.nodes()[node];
    if (n.kind != routerx::NodeKind::Channel) return 0.0;
    bool lr = false;
    bool tb = false;
    routerx::RxResource::traversalAxes(inEdge, outEdge, lr, tb);
    double c = 0.0;
    if (lr && mask.channelAxes.find(SegmentTargetAxis{n.designIndex, true}) !=
                  mask.channelAxes.end()) {
        c += 1.0;
    }
    if (tb && mask.channelAxes.find(SegmentTargetAxis{n.designIndex, false}) !=
                  mask.channelAxes.end()) {
        c += 1.0;
    }
    return c;
}

double softResourceOverlapTraversalCost(const routerx::RxGraph& graph,
                                        const SegmentResourceMask& mask,
                                        int node) {
    if (graph.nodes()[node].kind != routerx::NodeKind::SoftFeedthrough) {
        return 0.0;
    }
    return mask.softNodes.find(node) != mask.softNodes.end() ? 1.0 : 0.0;
}

std::set<int> segmentBannedGraphNodes(
        const Design& design,
        const routerx::RxGraph& graph,
        const std::vector<RouteStep>& steps,
        const SegmentWindow& window) {
    std::set<int> banned;
    for (const RouteStep& s : steps) {
        const int node = graphNodeForStepObject(design, graph, s.rectName);
        if (node >= 0) banned.insert(node);
    }
    if (window.a.graphNode >= 0) banned.erase(window.a.graphNode);
    if (window.b.graphNode >= 0) banned.erase(window.b.graphNode);
    return banned;
}

std::vector<SegmentLocalResult> findSegmentsBetweenContacts(
        const Design& design,
        const routerx::RxGraph& graph,
        int pairSrcBlock,
        int pairDstBlock,
        const SegmentAnchor& anchorA,
        const AnchorContact& dep,
        const SegmentAnchor& anchorB,
        const AnchorContact& arr,
        const std::vector<SegmentTargetAxis>& targets,
        const HotAxisRatios& hot,
        const SegmentResourceMask& resourceMask,
        const std::set<int>& bannedNodes,
        const SegmentSearchPolicy& policy,
        int maxResults) {
    std::vector<SegmentLocalResult> out;
    maxResults = std::max(1, maxResults);
    if (dep.node < 0 || arr.node < 0 ||
        dep.node >= static_cast<int>(graph.nodes().size()) ||
        arr.node >= static_cast<int>(graph.nodes().size())) {
        return out;
    }

    const Rect* anchorARect = objectRectByNameLite(design, anchorA.objectName);
    const Rect* anchorBRect = objectRectByNameLite(design, anchorB.objectName);
    if (!anchorARect || !anchorBRect) return out;

    if (anchorA.graphNode >= 0 && anchorB.graphNode >= 0 &&
        dep.node == anchorB.graphNode && arr.node == anchorA.graphNode &&
        dep.edgeOnAnchor == arr.edgeOnNode &&
        dep.edgeOnNode == arr.edgeOnAnchor) {
        routerx::GuidingPt directGp;
        if (!routerx::contactGuidingPoint(
                *anchorARect, dep.edgeOnAnchor,
                *anchorBRect, arr.edgeOnAnchor, directGp)) {
            return out;
        }
        SegmentLocalResult direct;
        direct.found = true;
        direct.targetDemand = 0.0;
        direct.otherHotDemand = 0.0;
        direct.overlapDemand = 0.0;
        direct.softOverlapDemand = 0.0;
        direct.wire = 0.0;
        direct.usedFeedthrough = false;
        direct.feature = segmentFeatureIndex(policy.feature);
        direct.rank = 1;
        return std::vector<SegmentLocalResult>{direct};
    }

    const int srcSoftNode = graph.nodeForSoftBlock(pairSrcBlock);
    const int dstSoftNode = graph.nodeForSoftBlock(pairDstBlock);
    auto traversable = [&](int node) {
        if (node == srcSoftNode || node == dstSoftNode) return false;
        if (!anchorA.isEndpoint && node == anchorA.graphNode) return false;
        if (!anchorB.isEndpoint && node == anchorB.graphNode) return false;
        if (bannedNodes.find(node) != bannedNodes.end()) return false;
        const routerx::NodeKind k = graph.nodes()[node].kind;
        if (k == routerx::NodeKind::Channel) return true;
        return true;
    };
    if (!traversable(dep.node) || !traversable(arr.node)) return out;

    const int N = static_cast<int>(graph.nodes().size());
    if (N <= 0) return out;
    auto stateId = [N](int prevSlot, int cur) -> long long {
        return static_cast<long long>(prevSlot) * N + cur;
    };

    routerx::GuidingPt depGp;
    if (!routerx::contactGuidingPoint(*anchorARect, dep.edgeOnAnchor,
                                      graphNodeRectLite(design, graph, dep.node),
                                      dep.edgeOnNode, depGp)) {
        return out;
    }

    const long long stateCount = static_cast<long long>(N + 1) * N;

    struct SearchLabel {
        SegmentSearchCost cost;
        long long state = -1;
        int parent = -1;
        routerx::GuidingPt entryGp;
        int enterEdge = 0;
    };
    std::vector<SearchLabel> labels;
    std::vector<std::vector<int>> bestIds(static_cast<size_t>(stateCount));

    struct QItem {
        SegmentSearchCost cost;
        int label = -1;
    };
    struct QGreater {
        SegmentSearchPolicy policy;
        bool operator()(const QItem& a, const QItem& b) const {
            if (segmentCostLess(a.cost, b.cost, policy)) return false;
            if (segmentCostLess(b.cost, a.cost, policy)) return true;
            return a.label > b.label;
        }
    };
    std::priority_queue<QItem, std::vector<QItem>, QGreater> pq(QGreater{policy});

    auto labelLess = [&](int lhs, int rhs) {
        const SearchLabel& a = labels[lhs];
        const SearchLabel& b = labels[rhs];
        if (segmentCostLess(a.cost, b.cost, policy)) return true;
        if (segmentCostLess(b.cost, a.cost, policy)) return false;
        return lhs < rhs;
    };
    auto isActiveLabel = [&](int id) {
        if (id < 0 || id >= static_cast<int>(labels.size())) return false;
        const long long state = labels[id].state;
        if (state < 0 || state >= stateCount) return false;
        const std::vector<int>& ids = bestIds[static_cast<size_t>(state)];
        return std::find(ids.begin(), ids.end(), id) != ids.end();
    };
    auto insertLabel = [&](int id) {
        SearchLabel& lab = labels[id];
        std::vector<int>& ids = bestIds[static_cast<size_t>(lab.state)];
        ids.push_back(id);
        std::sort(ids.begin(), ids.end(), labelLess);
        if (static_cast<int>(ids.size()) > maxResults) {
            const int removed = ids.back();
            ids.pop_back();
            return removed != id;
        }
        return true;
    };

    const long long seed = stateId(N, dep.node);
    labels.push_back(SearchLabel{SegmentSearchCost{0.0, 0.0, 0.0, 0.0, 0.0},
                                 seed, -1, depGp, dep.edgeOnNode});
    insertLabel(0);
    pq.push(QItem{labels[0].cost, 0});

    struct SegmentGoal {
        SegmentSearchCost cost;
        int label = -1;
    };
    std::vector<SegmentGoal> goals;
    auto goalLess = [&](const SegmentGoal& a, const SegmentGoal& b) {
        if (segmentCostLess(a.cost, b.cost, policy)) return true;
        if (segmentCostLess(b.cost, a.cost, policy)) return false;
        return a.label < b.label;
    };
    auto insertGoal = [&](const SegmentGoal& g) {
        goals.push_back(g);
        std::sort(goals.begin(), goals.end(), goalLess);
        if (static_cast<int>(goals.size()) > maxResults) goals.pop_back();
    };

    while (!pq.empty()) {
        const QItem qi = pq.top();
        pq.pop();
        if (!isActiveLabel(qi.label)) continue;
        const SearchLabel& lab = labels[qi.label];
        const long long s = lab.state;

        const int cur = static_cast<int>(s % N);
        const int curEnter = lab.enterEdge;
        const routerx::GuidingPt curGp = lab.entryGp;

        if (cur == arr.node && arr.edgeOnNode != curEnter) {
            routerx::GuidingPt arrGp;
            if (routerx::contactGuidingPoint(graphNodeRectLite(design, graph, cur),
                                             arr.edgeOnNode,
                                             *anchorBRect, arr.edgeOnAnchor,
                                             arrGp)) {
                SegmentSearchCost total = qi.cost;
                total.target += targetAxisTraversalCost(
                    design, graph, targets, cur, curEnter, arr.edgeOnNode);
                total.otherHot += otherHotTraversalCost(
                    design, graph, hot, targets, cur, curEnter, arr.edgeOnNode);
                total.overlap += resourceOverlapTraversalCost(
                    graph, resourceMask, cur, curEnter, arr.edgeOnNode);
                total.softOverlap += softResourceOverlapTraversalCost(
                    graph, resourceMask, cur);
                total.wire += std::fabs(curGp.x - arrGp.x) +
                              std::fabs(curGp.y - arrGp.y);
                insertGoal(SegmentGoal{total, qi.label});
            }
        }

        std::vector<int> incident = graph.incident(cur);
        std::sort(incident.begin(), incident.end());
        for (int eid : incident) {
            const routerx::Edge& e = graph.edges()[eid];
            const bool curIsA = e.a == cur;
            const int outEdge = curIsA ? e.edgeOnA : e.edgeOnB;
            const int nbr = curIsA ? e.b : e.a;
            const int nbrEnter = curIsA ? e.edgeOnB : e.edgeOnA;
            if (!traversable(nbr)) continue;
            if (outEdge == curEnter) continue;

            routerx::GuidingPt nextGp;
            if (!routerx::contactGuidingPoint(
                    graphNodeRectLite(design, graph, cur), outEdge,
                    graphNodeRectLite(design, graph, nbr), nbrEnter,
                    nextGp)) {
                continue;
            }

            SegmentSearchCost nd = qi.cost;
            nd.target += targetAxisTraversalCost(
                design, graph, targets, cur, curEnter, outEdge);
            nd.otherHot += otherHotTraversalCost(
                design, graph, hot, targets, cur, curEnter, outEdge);
            nd.overlap += resourceOverlapTraversalCost(
                graph, resourceMask, cur, curEnter, outEdge);
            nd.softOverlap += softResourceOverlapTraversalCost(
                graph, resourceMask, cur);
            nd.wire += std::fabs(curGp.x - nextGp.x) +
                       std::fabs(curGp.y - nextGp.y);
            const long long ns = stateId(cur, nbr);
            const int id = static_cast<int>(labels.size());
            labels.push_back(SearchLabel{nd, ns, qi.label, nextGp, nbrEnter});
            if (!insertLabel(id)) continue;
            pq.push(QItem{nd, id});
        }
    }

    if (goals.empty()) return out;

    for (int rank = 0; rank < static_cast<int>(goals.size()); ++rank) {
        const SegmentGoal& goal = goals[rank];
        std::vector<int> chain;
        for (int id = goal.label; id >= 0; id = labels[id].parent) {
            chain.push_back(id);
            if (labels[id].parent < 0) break;
        }
        std::reverse(chain.begin(), chain.end());
        SegmentLocalResult result;
        for (size_t i = 0; i < chain.size(); ++i) {
            const SearchLabel& cl = labels[chain[i]];
            const int cur = static_cast<int>(cl.state % N);
            const int exitE = (i + 1 < chain.size())
                ? routerx::oppositeEdge(labels[chain[i + 1]].enterEdge)
                : arr.edgeOnNode;
            result.hops.push_back(routerx::Hop{cur, cl.enterEdge, exitE});
        }
        result.found = true;
        result.targetDemand = goal.cost.target;
        result.otherHotDemand = goal.cost.otherHot;
        result.overlapDemand = goal.cost.overlap;
        result.softOverlapDemand = goal.cost.softOverlap;
        result.wire = goal.cost.wire;
        result.feature = segmentFeatureIndex(policy.feature);
        result.rank = rank + 1;
        for (const routerx::Hop& h : result.hops) {
            if (graph.nodes()[h.node].kind == routerx::NodeKind::SoftFeedthrough) {
                result.usedFeedthrough = true;
                break;
            }
        }
        out.push_back(result);
    }
    return out;
}

std::vector<RouteStep> spliceSegmentSteps(
        const Design& design,
        const routerx::RxGraph& graph,
        const std::vector<RouteStep>& base,
        const SegmentWindow& window,
        const AnchorContact& dep,
        const AnchorContact& arr,
        const SegmentLocalResult& seg) {
    std::vector<RouteStep> out;
    if (window.a.routeStepIndex < 0 ||
        window.b.routeStepIndex >= static_cast<int>(base.size()) ||
        window.a.routeStepIndex >= window.b.routeStepIndex) {
        return out;
    }
    out.reserve(base.size() + seg.hops.size() * 2);
    for (int i = 0; i < window.a.routeStepIndex; ++i) out.push_back(base[i]);
    out.push_back(RouteStep{window.a.objectName, dep.edgeOnAnchor});
    for (const routerx::Hop& h : seg.hops) {
        const std::string name = graphNodeNameLite(design, graph, h.node);
        out.push_back(RouteStep{name, h.inEdge});
        out.push_back(RouteStep{name, h.outEdge});
    }
    out.push_back(RouteStep{window.b.objectName, arr.edgeOnAnchor});
    for (int i = window.b.routeStepIndex + 1; i < static_cast<int>(base.size()); ++i) {
        out.push_back(base[i]);
    }
    return out;
}

bool routeHasRepeatedIntermediateObject(const std::vector<RouteStep>& steps) {
    std::set<std::string> seen;
    for (size_t i = 1; i + 1 < steps.size(); i += 2) {
        if (steps[i].rectName != steps[i + 1].rectName) return true;
        if (!seen.insert(steps[i].rectName).second) return true;
    }
    return false;
}

struct SegmentBypassStats {
    long long eligiblePairs = 0;
    long long hotRuns = 0;
    long long accessPairs = 0;
    long long bannedExistingNodeAccessPairs = 0;
    long long searchCalls = 0;
    long long dijkstraFoundCandidates = 0;
    long long noReliefCandidates = 0;
    long long rawCandidates = 0;
    long long distinctCandidates = 0;
    long long duplicateCandidates = 0;
    long long reliefCandidates = 0;
    long long reliefStillTargetHotCandidates = 0;
    long long reliefTouchesOtherHotCandidates = 0;
    long long reliefTouchesAnyHotCandidates = 0;
    long long loopCandidates = 0;
    long long invalidCandidates = 0;
    long long dirtyCandidates = 0;
    long long illegalFtCandidates = 0;
    long long selectorVisibleSegmentSignatures = 0;
    long long selectorLabeledSegmentCandidates = 0;
    long long selectorHiddenSegmentSignatures = 0;
    long long selectedWins = 0;
    long long selectedHiddenSegment = 0;
    long long changedPairs = 0;
    long long changedNonSegment = 0;
    long long selectedUsedFeedthrough = 0;
    long long selectedInvalid = 0;
    long long selectedIllegalFt = 0;
    double selectedTargetDemandDrop = 0.0;
    double localOfficialGainSum = 0.0;
    double localWireDeltaSum = 0.0;
    std::array<long long, kSegmentSearchFeatureCount> searchCallsByFeature{};
    std::array<long long, kSegmentSearchFeatureCount> dijkstraFoundByFeature{};
    std::array<long long, kSegmentSearchFeatureCount> loopByFeature{};
    std::array<long long, kSegmentSearchFeatureCount> rawByFeature{};
    std::array<long long, kSegmentSearchFeatureCount> distinctByFeature{};
    std::array<long long, kSegmentSearchFeatureCount> selectedByFeature{};
    std::array<long long, 4> selectedByRank{}; // indexes 1..3 are reported.
};

struct SegmentCandidateOrigin {
    int feature = 0;
    int rank = 1;
};

void rememberSegmentOrigin(std::map<std::string, SegmentCandidateOrigin>& origins,
                           const std::string& signature,
                           int feature,
                           int rank) {
    auto it = origins.find(signature);
    if (it == origins.end() ||
        feature < it->second.feature ||
        (feature == it->second.feature && rank < it->second.rank)) {
        origins[signature] = SegmentCandidateOrigin{feature, rank};
    }
}

std::vector<routerx::LiteCandidate> generateSegmentBypassCandidates(
        const Design& design,
        const routerx::RxGraph& graph,
        const routerx::DemandedPair& p,
        const routerx::LiteCandidate& current,
        const HotAxisRatios& hot,
        const SegmentSearchConfig& config,
        std::map<std::string, SegmentCandidateOrigin>& origins,
        SegmentBypassStats& stats) {
    std::vector<routerx::LiteCandidate> raw;
    const std::vector<SegmentWindow> windows =
        segmentWindowsForRoute(design, graph, current.route.steps, p.nets,
                               hot, config.anchorShiftDepth);
    if (windows.empty()) return raw;

    ++stats.eligiblePairs;
    stats.hotRuns += static_cast<long long>(windows.size());
    for (const SegmentWindow& window : windows) {
        const std::set<int> bannedNodes =
            segmentBannedGraphNodes(design, graph, current.route.steps, window);
        const SegmentResourceMask resourceMask =
            segmentResourceMaskForWindow(design, graph, current.route.steps, window);
        const std::vector<AnchorContact> deps =
            departureContactsForAnchor(design, graph, window.a);
        const std::vector<AnchorContact> arrs =
            arrivalContactsForAnchor(design, graph, window.b);
        stats.accessPairs += static_cast<long long>(deps.size()) *
                             static_cast<long long>(arrs.size());
        for (const AnchorContact& dep : deps) {
            for (const AnchorContact& arr : arrs) {
                if (bannedNodes.find(dep.node) != bannedNodes.end() ||
                    bannedNodes.find(arr.node) != bannedNodes.end()) {
                    ++stats.bannedExistingNodeAccessPairs;
                    continue;
                }
                for (const SegmentSearchPolicy& policy : config.policies) {
                    const int fidx = segmentFeatureIndex(policy.feature);
                    ++stats.searchCalls;
                    if (fidx >= 0 && fidx < kSegmentSearchFeatureCount) {
                        ++stats.searchCallsByFeature[fidx];
                    }
                    const std::vector<SegmentLocalResult> segs =
                        findSegmentsBetweenContacts(
                            design, graph, p.src, p.dst, window.a, dep,
                            window.b, arr, window.targets, hot, resourceMask,
                            bannedNodes, policy, config.topK);
                    for (const SegmentLocalResult& seg : segs) {
                        if (!seg.found) continue;
                        ++stats.dijkstraFoundCandidates;
                        if (fidx >= 0 && fidx < kSegmentSearchFeatureCount) {
                            ++stats.dijkstraFoundByFeature[fidx];
                        }
                        const std::vector<RouteStep> steps = spliceSegmentSteps(
                            design, graph, current.route.steps, window, dep, arr, seg);
                        if (steps.empty()) continue;
                        if (routeHasRepeatedIntermediateObject(steps)) {
                            ++stats.loopCandidates;
                            if (fidx >= 0 && fidx < kSegmentSearchFeatureCount) {
                                ++stats.loopByFeature[fidx];
                            }
                            continue;
                        }
                        const double targetDemand = targetAxisDemandForSteps(
                            design, steps, p.nets, window.targets);
                        if (!(targetDemand < window.originalTargetDemand - 1.0e-9)) {
                            ++stats.noReliefCandidates;
                            continue;
                        }
                        const double otherHotDemand = nonTargetHotAxisDemandForSteps(
                            design, steps, p.nets, hot, window.targets);
                        if (targetDemand > 1.0e-9) {
                            ++stats.reliefStillTargetHotCandidates;
                        }
                        if (otherHotDemand > 1.0e-9) {
                            ++stats.reliefTouchesOtherHotCandidates;
                        }
                        if (targetDemand > 1.0e-9 || otherHotDemand > 1.0e-9) {
                            ++stats.reliefTouchesAnyHotCandidates;
                        }
                        ++stats.reliefCandidates;
                        routerx::LiteCandidate cand = makeCandidate(
                            design, p,
                            routerx::LiteCandidateFamily::SegmentBypassUnified,
                            steps);
                        if (!cand.emitValid) ++stats.invalidCandidates;
                        rememberSegmentOrigin(origins, cand.signature, fidx, seg.rank);
                        if (fidx >= 0 && fidx < kSegmentSearchFeatureCount) {
                            ++stats.rawByFeature[fidx];
                        }
                        raw.push_back(cand);
                    }
                }
            }
        }
    }
    stats.rawCandidates += static_cast<long long>(raw.size());
    routerx::LiteDedupResult dedup = routerx::dedupLiteCandidates(raw);
    stats.distinctCandidates += static_cast<long long>(dedup.candidates.size());
    stats.duplicateCandidates += dedup.duplicateCandidates;
    for (const routerx::LiteCandidate& c : dedup.candidates) {
        const auto it = origins.find(c.signature);
        if (it == origins.end()) continue;
        const int fidx = it->second.feature;
        if (fidx >= 0 && fidx < kSegmentSearchFeatureCount) {
            ++stats.distinctByFeature[fidx];
        }
    }
    return dedup.candidates;
}

double hotAxisDemandTotal(const RouteAxisDemand& demand,
                          const HotAxisRatios& hot) {
    double total = 0.0;
    const int n = std::min(static_cast<int>(demand.lr.size()),
                           static_cast<int>(hot.lr.size()));
    for (int i = 0; i < n; ++i) {
        if (hot.lr[i] > 0.0) total += demand.lr[i];
    }
    const int m = std::min(static_cast<int>(demand.tb.size()),
                           static_cast<int>(hot.tb.size()));
    for (int i = 0; i < m; ++i) {
        if (hot.tb[i] > 0.0) total += demand.tb[i];
    }
    return total;
}

double usefulHotReliefTotal(const Design& design,
                            const routerx::RxResourceLedgerLite& ledger,
                            const HotAxisRatios& hot,
                            const RouteAxisDemand& current,
                            const RouteAxisDemand& alt) {
    double total = 0.0;
    for (int i = 0; i < ledger.channelCount() &&
                    i < static_cast<int>(design.channels.size()); ++i) {
        const routerx::LiteChannelUse& u = ledger.channel(i);
        if (i < static_cast<int>(hot.lr.size()) && hot.lr[i] > 0.0) {
            const double overflow = std::max(0.0, u.usedLR - u.capLR);
            total += std::min(overflow, std::max(0.0, current.lr[i] - alt.lr[i]));
        }
        if (i < static_cast<int>(hot.tb.size()) && hot.tb[i] > 0.0) {
            const double overflow = std::max(0.0, u.usedTB - u.capTB);
            total += std::min(overflow, std::max(0.0, current.tb[i] - alt.tb[i]));
        }
    }
    return total;
}

double rawHotReliefTotal(const HotAxisRatios& hot,
                         const RouteAxisDemand& current,
                         const RouteAxisDemand& alt) {
    double total = 0.0;
    const int n = std::min(static_cast<int>(current.lr.size()),
                           static_cast<int>(hot.lr.size()));
    for (int i = 0; i < n; ++i) {
        if (hot.lr[i] > 0.0) {
            total += std::max(0.0, current.lr[i] - alt.lr[i]);
        }
    }
    const int m = std::min(static_cast<int>(current.tb.size()),
                           static_cast<int>(hot.tb.size()));
    for (int i = 0; i < m; ++i) {
        if (hot.tb[i] > 0.0) {
            total += std::max(0.0, current.tb[i] - alt.tb[i]);
        }
    }
    return total;
}

struct SplitTriggerSummary {
    long long winnerTouchesHot = 0;
    long long hasReliefAlternative = 0;
    long long usefulSplittable = 0;
    double hotDemandWinnerSum = 0.0;
    double maxUsefulReliefSum = 0.0;
};

struct SplitPairPlan {
    size_t pairIndex = 0;
    routerx::LiteCandidate current;
    routerx::LiteCandidate alternative;
    double hotDemand = 0.0;
    double usefulRelief = 0.0;
    double wireOverhead = 0.0;
    int movedNets = 0;
};

SplitTriggerSummary summarizeSplitTriggers(
        const Design& design,
        const std::vector<routerx::DemandedPair>& pairs,
        const std::vector<routerx::LiteCandidate>& winners,
        const std::vector<std::vector<routerx::LiteCandidate>>& candidateSets,
        const routerx::RxResourceLedgerLite& ledger,
        const HotAxisRatios& hot) {
    SplitTriggerSummary out;
    for (size_t i = 0; i < pairs.size() && i < winners.size() &&
                       i < candidateSets.size(); ++i) {
        const routerx::DemandedPair& p = pairs[i];
        const routerx::LiteCandidate& current = winners[i];
        if (current.route.steps.empty() || !selectableCandidate(current)) continue;

        const RouteAxisDemand currentDemand =
            routeAxisDemand(design, current.route.steps, p.nets);
        const double hotDemand = hotAxisDemandTotal(currentDemand, hot);
        if (hotDemand <= 1.0e-9) continue;
        ++out.winnerTouchesHot;
        out.hotDemandWinnerSum += hotDemand;

        bool hasRelief = false;
        double bestUseful = 0.0;
        for (const routerx::LiteCandidate& c : candidateSets[i]) {
            if (!selectableCandidate(c)) continue;
            if (c.signature == current.signature) continue;
            const RouteAxisDemand altDemand =
                routeAxisDemand(design, c.route.steps, p.nets);
            const double hotRelief = rawHotReliefTotal(hot, currentDemand, altDemand);
            if (hotRelief > 1.0e-9) hasRelief = true;
            bestUseful = std::max(bestUseful, usefulHotReliefTotal(
                design, ledger, hot, currentDemand, altDemand));
        }
        if (!hasRelief) continue;
        ++out.hasReliefAlternative;
        out.maxUsefulReliefSum += bestUseful;
        if (p.nets >= 2 && bestUseful > 1.0e-9) ++out.usefulSplittable;
    }
    return out;
}

std::vector<SplitPairPlan> buildHalfSplitPlans(
        const Design& design,
        const std::vector<routerx::DemandedPair>& pairs,
        const std::vector<routerx::LiteCandidate>& winners,
        const std::vector<std::vector<routerx::LiteCandidate>>& candidateSets,
        const routerx::RxResourceLedgerLite& ledger,
        const HotAxisRatios& hot) {
    std::vector<SplitPairPlan> out;
    for (size_t i = 0; i < pairs.size() && i < winners.size() &&
                       i < candidateSets.size(); ++i) {
        const routerx::DemandedPair& p = pairs[i];
        const routerx::LiteCandidate& current = winners[i];
        if (p.nets < 2 || current.route.steps.empty() ||
            !selectableCandidate(current)) {
            continue;
        }

        const RouteAxisDemand currentDemand =
            routeAxisDemand(design, current.route.steps, p.nets);
        const double hotDemand = hotAxisDemandTotal(currentDemand, hot);
        if (hotDemand <= 1.0e-9) continue;

        int bestIdx = -1;
        double bestUseful = 0.0;
        double bestWireOverhead = 0.0;
        for (size_t j = 0; j < candidateSets[i].size(); ++j) {
            const routerx::LiteCandidate& c = candidateSets[i][j];
            if (!selectableCandidate(c)) continue;
            if (c.signature == current.signature) continue;
            const RouteAxisDemand altDemand =
                routeAxisDemand(design, c.route.steps, p.nets);
            const double useful = usefulHotReliefTotal(
                design, ledger, hot, currentDemand, altDemand);
            if (useful <= 1.0e-9) continue;
            const double wireOverhead = c.impact.wire - current.impact.wire;
            if (bestIdx < 0 || useful > bestUseful + 1.0e-9 ||
                (std::fabs(useful - bestUseful) <= 1.0e-9 &&
                 (wireOverhead < bestWireOverhead - 1.0e-9 ||
                  (std::fabs(wireOverhead - bestWireOverhead) <= 1.0e-9 &&
                   c.signature < candidateSets[i][bestIdx].signature)))) {
                bestIdx = static_cast<int>(j);
                bestUseful = useful;
                bestWireOverhead = wireOverhead;
            }
        }
        if (bestIdx < 0) continue;

        SplitPairPlan plan;
        plan.pairIndex = i;
        plan.current = current;
        plan.alternative = candidateSets[i][bestIdx];
        plan.hotDemand = hotDemand;
        plan.usefulRelief = bestUseful;
        plan.wireOverhead = bestWireOverhead;
        plan.movedNets = p.nets / 2;
        if (plan.movedNets <= 0 || plan.movedNets >= p.nets) continue;
        out.push_back(std::move(plan));
    }

    std::sort(out.begin(), out.end(), [&](const SplitPairPlan& a,
                                          const SplitPairPlan& b) {
        if (a.usefulRelief != b.usefulRelief) return a.usefulRelief > b.usefulRelief;
        const int an = pairs[a.pairIndex].nets;
        const int bn = pairs[b.pairIndex].nets;
        if (an != bn) return an > bn;
        return a.pairIndex < b.pairIndex;
    });
    return out;
}

struct AllocationPart {
    const routerx::LiteCandidate* candidate = nullptr;
    int nets = 0;
};

struct AllocationEval {
    bool valid = true;
    double key = 0.0;
    double wire = 0.0;
    double dChannelPenalty = 0.0;
    double dFtPenalty = 0.0;
    double dIllegalFtPenalty = 0.0;
};

AllocationEval evaluateAllocation(
        const Design& design,
        const routerx::RxResourceLedgerLite& base,
        const std::vector<AllocationPart>& parts,
        double alpha) {
    AllocationEval out;
    const double outlineArea = design.outlineW * design.outlineH;
    const double beforeChPenalty = base.channelPenalty(outlineArea);
    const rxscore::FtResult beforeFt = base.ftResult(design);

    routerx::RxResourceLedgerLite after = base;
    for (const AllocationPart& part : parts) {
        if (!part.candidate || part.nets <= 0) continue;
        if (!part.candidate->emitValid) out.valid = false;
        const routerx::LiteRouteImpact impact =
            after.projectSteps(design, part.candidate->route.steps, part.nets);
        if (!impact.clean()) out.valid = false;
        out.wire += impact.wire;
        after.commitSteps(design, part.candidate->route.steps, part.nets);
    }

    const double afterChPenalty = after.channelPenalty(outlineArea);
    const rxscore::FtResult afterFt = after.ftResult(design);
    out.dChannelPenalty = afterChPenalty - beforeChPenalty;
    out.dFtPenalty = afterFt.penalty - beforeFt.penalty;
    out.dIllegalFtPenalty = afterFt.illegalPenalty - beforeFt.illegalPenalty;
    if (afterFt.illegalBlocks > 0 || out.dIllegalFtPenalty > 0.0) {
        out.valid = false;
    }
    out.key = alpha * out.wire + out.dChannelPenalty +
              out.dFtPenalty + out.dIllegalFtPenalty;
    return out;
}

void commitAllocation(const Design& design,
                      routerx::RxResourceLedgerLite& ledger,
                      const std::vector<AllocationPart>& parts) {
    for (const AllocationPart& part : parts) {
        if (!part.candidate || part.nets <= 0) continue;
        ledger.commitSteps(design, part.candidate->route.steps, part.nets);
    }
}

enum class SplitChoiceKind {
    Keep,
    FullSwitch,
    HalfSplit,
};

enum class SplitRatioKind {
    ClearSmallestHot = 0,
    ClearLargestHot,
    Quarter,
    Half,
};

static constexpr int kSplitRatioKindCount = 4;

const char* splitRatioKindName(SplitRatioKind kind) {
    switch (kind) {
        case SplitRatioKind::ClearSmallestHot: return "clearSmallestHot";
        case SplitRatioKind::ClearLargestHot:  return "clearLargestHot";
        case SplitRatioKind::Quarter:          return "quarter";
        case SplitRatioKind::Half:             return "half";
    }
    return "unknown";
}

std::string splitRatioPrefix(SplitRatioKind kind) {
    return std::string("r2d2_") + splitRatioKindName(kind);
}

struct SplitPassStats {
    long long eligiblePairs = 0;
    long long allocationCandidates = 0;
    long long keepWins = 0;
    long long fullSwitchWins = 0;
    long long halfSplitWins = 0;
    long long changedPairs = 0;
    long long selectedUsedFeedthrough = 0;
    double localOfficialGainSum = 0.0;
    double localWireDeltaSum = 0.0;
};

struct SplitRatioOracleStats {
    long long winnerTouchesHot = 0;
    long long pairsWithReliefCandidate = 0;
    long long reliefCandidates = 0;
    long long allocationCandidates = 0;
    long long bestPositivePairs = 0;
    long long bestFullSwitchWins = 0;
    long long bestPartialSplitWins = 0;
    std::array<long long, kSplitRatioKindCount> kindCandidates{};
    std::array<long long, kSplitRatioKindCount> kindPositiveCandidates{};
    std::array<long long, kSplitRatioKindCount> kindPositivePairs{};
    std::array<long long, kSplitRatioKindCount> kindBestFullSwitchWins{};
    std::array<long long, kSplitRatioKindCount> kindBestPartialSplitWins{};
    std::array<long long, kSplitRatioKindCount> bestRatioWins{};
    std::array<double, kSplitRatioKindCount> kindBestGainSum{};
    std::array<double, kSplitRatioKindCount> kindBestWireDeltaSum{};
    std::array<double, kSplitRatioKindCount> kindMovedNetsSum{};
    double bestGainSum = 0.0;
    double bestWireDeltaSum = 0.0;
    double bestMovedNetsSum = 0.0;
};

struct Split3WayOracleStats {
    long long winnerTouchesHot = 0;
    long long pairsWithTwoReliefCandidates = 0;
    long long reliefCandidates = 0;
    long long allocationCandidates = 0;
    long long bestPositivePairs = 0;
    long long bestUsesBothAlternatives = 0;
    long long bestLeavesOnCurrent = 0;
    std::array<long long, kSplitRatioKindCount> kindCandidates{};
    std::array<long long, kSplitRatioKindCount> kindPositiveCandidates{};
    std::array<long long, kSplitRatioKindCount> kindPositivePairs{};
    std::array<long long, kSplitRatioKindCount> kindBestUsesBothAlternatives{};
    std::array<long long, kSplitRatioKindCount> bestRatioWins{};
    std::array<double, kSplitRatioKindCount> kindBestGainSum{};
    std::array<double, kSplitRatioKindCount> kindBestWireDeltaSum{};
    std::array<double, kSplitRatioKindCount> kindMovedBNetsSum{};
    std::array<double, kSplitRatioKindCount> kindMovedCNetsSum{};
    double bestGainSum = 0.0;
    double bestWireDeltaSum = 0.0;
    double bestMovedBNetsSum = 0.0;
    double bestMovedCNetsSum = 0.0;
};

enum class AllocationSplitPolicy {
    ClearSmallestHot = 0,
    ClearLargestHot,
    FivePercent,
    TenPercent,
    FifteenPercent,
    TwentyPercent,
    Quarter,
    ThirtyThreePercent,
    FiftyPercent,
};

static constexpr int kAllocationSplitPolicyCount = 9;

const char* allocationSplitPolicyName(AllocationSplitPolicy policy) {
    switch (policy) {
        case AllocationSplitPolicy::ClearSmallestHot: return "clearSmallestHot";
        case AllocationSplitPolicy::ClearLargestHot:  return "clearLargestHot";
        case AllocationSplitPolicy::FivePercent:      return "fivePercent";
        case AllocationSplitPolicy::TenPercent:       return "tenPercent";
        case AllocationSplitPolicy::FifteenPercent:   return "fifteenPercent";
        case AllocationSplitPolicy::TwentyPercent:    return "twentyPercent";
        case AllocationSplitPolicy::Quarter:          return "quarter";
        case AllocationSplitPolicy::ThirtyThreePercent:return "thirtyThreePercent";
        case AllocationSplitPolicy::FiftyPercent:     return "fiftyPercent";
    }
    return "unknown";
}

bool allocationSplitPolicyWideOnly(AllocationSplitPolicy policy) {
    switch (policy) {
        case AllocationSplitPolicy::FivePercent:
        case AllocationSplitPolicy::FifteenPercent:
        case AllocationSplitPolicy::TwentyPercent:
        case AllocationSplitPolicy::ThirtyThreePercent:
        case AllocationSplitPolicy::FiftyPercent:
            return true;
        case AllocationSplitPolicy::ClearSmallestHot:
        case AllocationSplitPolicy::ClearLargestHot:
        case AllocationSplitPolicy::TenPercent:
        case AllocationSplitPolicy::Quarter:
            return false;
    }
    return false;
}

bool allocationSplitPolicySlimSet(AllocationSplitPolicy policy) {
    switch (policy) {
        case AllocationSplitPolicy::ClearSmallestHot:
        case AllocationSplitPolicy::FivePercent:
        case AllocationSplitPolicy::FifteenPercent:
        case AllocationSplitPolicy::Quarter:
        case AllocationSplitPolicy::ThirtyThreePercent:
        case AllocationSplitPolicy::FiftyPercent:
            return true;
        case AllocationSplitPolicy::ClearLargestHot:
        case AllocationSplitPolicy::TenPercent:
        case AllocationSplitPolicy::TwentyPercent:
            return false;
    }
    return false;
}

struct AllocationPoolSelectorStats {
    long long eligiblePairs = 0;
    long long loserCandidates = 0;
    long long reliefLoserCandidates = 0;
    long long allocationCandidates = 0;
    long long keepWins = 0;
    long long fullSwitchWins = 0;
    long long splitWins = 0;
    long long changedPairs = 0;
    long long selectedUsedFeedthrough = 0;
    std::array<long long, kAllocationSplitPolicyCount> splitPolicyCandidates{};
    std::array<long long, kAllocationSplitPolicyCount> splitPolicyWins{};
    double localOfficialGainSum = 0.0;
    double localWireDeltaSum = 0.0;
    double fullSwitchOfficialGainSum = 0.0;
    double fullSwitchWireDeltaSum = 0.0;
    double splitOfficialGainSum = 0.0;
    double splitWireDeltaSum = 0.0;
    double movedNetsSum = 0.0;
};

struct AllocationReselectPassStats {
    int pass = 0;
    long long changedPairs = 0;
    long long keepWins = 0;
    long long baseSingleWins = 0;
    long long splitWins = 0;
    long long allocationCandidates = 0;
    double officialGain = 0.0;
    double wireGain = 0.0;
    double movedNetsSum = 0.0;
    double channelOverflowBefore = 0.0;
    double channelOverflowAfter = 0.0;
    double ftRequiredBefore = 0.0;
    double ftRequiredAfter = 0.0;
};

struct AllocationReselectResult {
    std::vector<AllocationReselectPassStats> passes;
    long long selectedInvalid = 0;
    long long selectedIllegalFt = 0;
    long long selectedUsedFeedthrough = 0;
    std::array<long long, routerx::kLiteCandidateFamilyCount> familyWins{};
};

enum class AllocationPoolChoiceKind {
    Keep,
    BaseSingle,
    FullSwitch,
    Split,
};

struct AllocationPoolChoice {
    AllocationPoolChoiceKind kind = AllocationPoolChoiceKind::Keep;
    const routerx::LiteCandidate* alternative = nullptr;
    int movedNets = 0;
    int splitPolicyIndex = -1;
    AllocationEval eval;
};

struct DynamicSplitMove {
    bool found = false;
    SplitPairPlan plan;
    SplitChoiceKind choice = SplitChoiceKind::Keep;
    AllocationEval keepEval;
    AllocationEval bestEval;
    double gain = 0.0;
};

struct ReliefAlt {
    const routerx::LiteCandidate* candidate = nullptr;
    RouteAxisDemand demand;
    double useful = 0.0;
    double wireOverhead = 0.0;
};

struct Split3Move {
    bool valid = false;
    int movedB = 0;
    int movedC = 0;
    AllocationEval eval;
};

int clampMovedNets(long long moved, int nets) {
    if (nets <= 0) return 0;
    if (moved < 1) return 1;
    if (moved > nets) return nets;
    return static_cast<int>(moved);
}

double demandOnAxis(const RouteAxisDemand& demand, bool lr, int channelIndex) {
    const std::vector<double>& v = lr ? demand.lr : demand.tb;
    if (channelIndex < 0 || channelIndex >= static_cast<int>(v.size())) return 0.0;
    return v[channelIndex];
}

long long movedNetsToClear(double overflow,
                           double currentFullDemand,
                           double altFullDemand,
                           int nets) {
    if (nets <= 0 || overflow <= 1.0e-9) return 0;
    const double reliefPerNet = (currentFullDemand - altFullDemand) /
                                static_cast<double>(nets);
    if (reliefPerNet <= 1.0e-9) return 0;
    return static_cast<long long>(std::ceil(overflow / reliefPerNet - 1.0e-9));
}

std::vector<AllocationPart> splitAllocationParts3(
        const routerx::LiteCandidate& current,
        const routerx::LiteCandidate& b,
        const routerx::LiteCandidate& c,
        int nets,
        int movedB,
        int movedC) {
    movedB = movedB > 0 ? clampMovedNets(movedB, nets) : 0;
    movedC = movedC > 0 ? clampMovedNets(movedC, nets) : 0;
    if (movedB + movedC > nets) {
        const double scale = static_cast<double>(nets) /
                             static_cast<double>(movedB + movedC);
        movedB = std::max(0, static_cast<int>(std::floor(movedB * scale)));
        movedC = std::max(0, nets - movedB);
    }
    std::vector<AllocationPart> parts;
    const int stay = nets - movedB - movedC;
    if (stay > 0) parts.push_back(AllocationPart{&current, stay});
    if (movedB > 0) parts.push_back(AllocationPart{&b, movedB});
    if (movedC > 0) parts.push_back(AllocationPart{&c, movedC});
    return parts;
}

int clearHotMovedNets(const Design& design,
                      const routerx::RxResourceLedgerLite& ledger,
                      const HotAxisRatios& hot,
                      const RouteAxisDemand& current,
                      const RouteAxisDemand& alt,
                      int nets,
                      bool largestOverflow) {
    bool found = false;
    long long selectedMoved = 0;
    double selectedOverflow = 0.0;

    auto consider = [&](double hotness, double overflow,
                        double currentDemand, double altDemand) {
        if (hotness <= 0.0 || overflow <= 1.0e-9) return;
        const long long moved = movedNetsToClear(
            overflow, currentDemand, altDemand, nets);
        if (moved <= 0) return;
        if (!found) {
            found = true;
            selectedMoved = moved;
            selectedOverflow = overflow;
            return;
        }
        if (largestOverflow) {
            if (overflow > selectedOverflow + 1.0e-9 ||
                (std::fabs(overflow - selectedOverflow) <= 1.0e-9 &&
                 moved < selectedMoved)) {
                selectedMoved = moved;
                selectedOverflow = overflow;
            }
        } else {
            if (moved < selectedMoved ||
                (moved == selectedMoved &&
                 overflow < selectedOverflow - 1.0e-9)) {
                selectedMoved = moved;
                selectedOverflow = overflow;
            }
        }
    };

    for (int i = 0; i < ledger.channelCount() &&
                    i < static_cast<int>(design.channels.size()); ++i) {
        const routerx::LiteChannelUse& u = ledger.channel(i);
        if (i < static_cast<int>(hot.lr.size()) && hot.lr[i] > 0.0) {
            consider(hot.lr[i], std::max(0.0, u.usedLR - u.capLR),
                     i < static_cast<int>(current.lr.size()) ? current.lr[i] : 0.0,
                     i < static_cast<int>(alt.lr.size()) ? alt.lr[i] : 0.0);
        }
        if (i < static_cast<int>(hot.tb.size()) && hot.tb[i] > 0.0) {
            consider(hot.tb[i], std::max(0.0, u.usedTB - u.capTB),
                     i < static_cast<int>(current.tb.size()) ? current.tb[i] : 0.0,
                     i < static_cast<int>(alt.tb.size()) ? alt.tb[i] : 0.0);
        }
    }

    return found ? clampMovedNets(selectedMoved, nets) : 0;
}

Split3Move bestThreeWayClearMoveForAxis(
        const Design& design,
        const routerx::RxResourceLedgerLite& replaceLedger,
        const AllocationEval& keepEval,
        const routerx::LiteCandidate& current,
        const ReliefAlt& b,
        const ReliefAlt& c,
        int nets,
        double overflow,
        double currentDemand,
        double bDemand,
        double cDemand) {
    Split3Move best;
    if (nets <= 0 || overflow <= 1.0e-9) return best;

    const double rB = (currentDemand - bDemand) / static_cast<double>(nets);
    const double rC = (currentDemand - cDemand) / static_cast<double>(nets);
    std::vector<std::pair<int, int>> combos;
    auto addCombo = [&](long long rawB, long long rawC) {
        int movedB = rawB > 0 ? clampMovedNets(rawB, nets) : 0;
        int movedC = rawC > 0 ? clampMovedNets(rawC, nets) : 0;
        if (movedB + movedC <= 0) return;
        if (movedB + movedC > nets) {
            const double scale = static_cast<double>(nets) /
                                 static_cast<double>(movedB + movedC);
            movedB = std::max(0, static_cast<int>(std::floor(movedB * scale)));
            movedC = std::max(0, nets - movedB);
        }
        if (movedB + movedC <= 0) return;
        combos.push_back({movedB, movedC});
    };

    if (rB > 1.0e-9) addCombo(movedNetsToClear(overflow, currentDemand, bDemand, nets), 0);
    if (rC > 1.0e-9) addCombo(0, movedNetsToClear(overflow, currentDemand, cDemand, nets));
    if (rB > 1.0e-9 && rC > 1.0e-9) {
        const long long halfB = static_cast<long long>(
            std::ceil((overflow * 0.5) / rB - 1.0e-9));
        const double remAfterB = std::max(0.0, overflow - halfB * rB);
        addCombo(halfB, movedNetsToClear(remAfterB, currentDemand, cDemand, nets));

        const long long halfC = static_cast<long long>(
            std::ceil((overflow * 0.5) / rC - 1.0e-9));
        const double remAfterC = std::max(0.0, overflow - halfC * rC);
        addCombo(movedNetsToClear(remAfterC, currentDemand, bDemand, nets), halfC);

        const long long both = static_cast<long long>(
            std::ceil(overflow / (rB + rC) - 1.0e-9));
        addCombo(both, both);
    }

    std::sort(combos.begin(), combos.end());
    combos.erase(std::unique(combos.begin(), combos.end()), combos.end());
    for (const auto& combo : combos) {
        const std::vector<AllocationPart> parts = splitAllocationParts3(
            current, *b.candidate, *c.candidate, nets, combo.first, combo.second);
        const AllocationEval scored = evaluateAllocation(
            design, replaceLedger, parts, design.alpha);
        if (!scored.valid) continue;
        const double gain = keepEval.key - scored.key;
        if (gain <= 1.0e-6) continue;
        if (!best.valid || gain > keepEval.key - best.eval.key + 1.0e-6 ||
            (std::fabs(gain - (keepEval.key - best.eval.key)) <= 1.0e-6 &&
             scored.wire < best.eval.wire)) {
            best.valid = true;
            best.movedB = combo.first;
            best.movedC = combo.second;
            best.eval = scored;
        }
    }
    return best;
}

int proportionalMovedNets(int nets, SplitRatioKind kind);

Split3Move evaluateThreeWayRatio(
        const Design& design,
        const routerx::RxResourceLedgerLite& ledger,
        const HotAxisRatios& hot,
        const AllocationEval& keepEval,
        const routerx::LiteCandidate& current,
        const RouteAxisDemand& currentDemand,
        const ReliefAlt& b,
        const ReliefAlt& c,
        int nets,
        SplitRatioKind kind) {
    Split3Move out;
    if (nets <= 0 || !b.candidate || !c.candidate) return out;

    routerx::RxResourceLedgerLite replaceLedger = ledger;
    replaceLedger.ripupSteps(design, current.route.steps, nets);

    if (kind == SplitRatioKind::Quarter || kind == SplitRatioKind::Half) {
        const int movedTotal = proportionalMovedNets(nets, kind);
        if (movedTotal <= 0) return out;
        const double denom = std::max(1.0e-9, b.useful + c.useful);
        int movedB = static_cast<int>(std::llround(
            movedTotal * (b.useful / denom)));
        movedB = std::max(1, std::min(movedTotal - 1, movedB));
        const int movedC = movedTotal - movedB;
        if (movedC <= 0) return out;
        const std::vector<AllocationPart> parts = splitAllocationParts3(
            current, *b.candidate, *c.candidate, nets, movedB, movedC);
        const AllocationEval eval =
            evaluateAllocation(design, replaceLedger, parts, design.alpha);
        if (!eval.valid || keepEval.key - eval.key <= 1.0e-6) return out;
        out.valid = true;
        out.movedB = movedB;
        out.movedC = movedC;
        out.eval = eval;
        return out;
    }

    bool haveTarget = false;
    Split3Move selected;
    long long selectedMoved = 0;
    double selectedOverflow = 0.0;

    auto considerAxis = [&](bool lr, int ci, double hotness, double overflow) {
        if (hotness <= 0.0 || overflow <= 1.0e-9) return;
        const double cur = demandOnAxis(currentDemand, lr, ci);
        const double bd = demandOnAxis(b.demand, lr, ci);
        const double cd = demandOnAxis(c.demand, lr, ci);
        const Split3Move move = bestThreeWayClearMoveForAxis(
            design, replaceLedger, keepEval, current, b, c, nets,
            overflow, cur, bd, cd);
        if (!move.valid) return;
        const long long moved = move.movedB + move.movedC;
        if (!haveTarget) {
            haveTarget = true;
            selected = move;
            selectedMoved = moved;
            selectedOverflow = overflow;
            return;
        }
        if (kind == SplitRatioKind::ClearLargestHot) {
            if (overflow > selectedOverflow + 1.0e-9 ||
                (std::fabs(overflow - selectedOverflow) <= 1.0e-9 &&
                 moved < selectedMoved)) {
                selected = move;
                selectedMoved = moved;
                selectedOverflow = overflow;
            }
        } else {
            if (moved < selectedMoved ||
                (moved == selectedMoved &&
                 overflow < selectedOverflow - 1.0e-9)) {
                selected = move;
                selectedMoved = moved;
                selectedOverflow = overflow;
            }
        }
    };

    for (int i = 0; i < ledger.channelCount() &&
                    i < static_cast<int>(design.channels.size()); ++i) {
        const routerx::LiteChannelUse& u = ledger.channel(i);
        if (i < static_cast<int>(hot.lr.size()) && hot.lr[i] > 0.0) {
            considerAxis(true, i, hot.lr[i], std::max(0.0, u.usedLR - u.capLR));
        }
        if (i < static_cast<int>(hot.tb.size()) && hot.tb[i] > 0.0) {
            considerAxis(false, i, hot.tb[i], std::max(0.0, u.usedTB - u.capTB));
        }
    }
    return selected;
}

int proportionalMovedNets(int nets, SplitRatioKind kind) {
    if (nets <= 0) return 0;
    switch (kind) {
        case SplitRatioKind::Quarter:
            return clampMovedNets(static_cast<long long>(
                std::ceil(0.25 * static_cast<double>(nets) - 1.0e-9)), nets);
        case SplitRatioKind::Half:
            return clampMovedNets(nets / 2, nets);
        case SplitRatioKind::ClearSmallestHot:
        case SplitRatioKind::ClearLargestHot:
            return 0;
    }
    return 0;
}

Split3WayOracleStats runSplit3WayOracle(
        const Design& design,
        const std::vector<routerx::DemandedPair>& pairs,
        const std::vector<routerx::LiteCandidate>& winners,
        const std::vector<std::vector<routerx::LiteCandidate>>& candidateSets,
        const routerx::RxResourceLedgerLite& ledger,
        const HotAxisRatios& hot) {
    Split3WayOracleStats stats;

    for (size_t i = 0; i < pairs.size() && i < winners.size() &&
                       i < candidateSets.size(); ++i) {
        const routerx::DemandedPair& p = pairs[i];
        const routerx::LiteCandidate& current = winners[i];
        if (p.nets < 3 || current.route.steps.empty() ||
            !current.emitValid) {
            continue;
        }

        const RouteAxisDemand currentDemand =
            routeAxisDemand(design, current.route.steps, p.nets);
        const double hotDemand = hotAxisDemandTotal(currentDemand, hot);
        if (hotDemand <= 1.0e-9) continue;
        ++stats.winnerTouchesHot;

        std::vector<ReliefAlt> reliefs;
        for (const routerx::LiteCandidate& alt : candidateSets[i]) {
            if (!alt.emitValid) continue;
            if (alt.signature == current.signature) continue;
            const RouteAxisDemand altDemand =
                routeAxisDemand(design, alt.route.steps, p.nets);
            const double useful = usefulHotReliefTotal(
                design, ledger, hot, currentDemand, altDemand);
            if (useful <= 1.0e-9) continue;
            reliefs.push_back(ReliefAlt{
                &alt,
                altDemand,
                useful,
                alt.impact.wire - current.impact.wire
            });
        }
        stats.reliefCandidates += static_cast<long long>(reliefs.size());
        if (reliefs.size() < 2) continue;
        ++stats.pairsWithTwoReliefCandidates;
        std::sort(reliefs.begin(), reliefs.end(),
                  [](const ReliefAlt& a, const ReliefAlt& b) {
            if (a.useful != b.useful) return a.useful > b.useful;
            if (a.wireOverhead != b.wireOverhead) return a.wireOverhead < b.wireOverhead;
            return a.candidate->signature < b.candidate->signature;
        });
        const ReliefAlt& b = reliefs[0];
        const ReliefAlt& c = reliefs[1];

        routerx::RxResourceLedgerLite replaceLedger = ledger;
        replaceLedger.ripupSteps(design, current.route.steps, p.nets);
        const std::vector<AllocationPart> keepParts = {
            AllocationPart{&current, p.nets}
        };
        const AllocationEval keepEval =
            evaluateAllocation(design, replaceLedger, keepParts, design.alpha);

        std::array<double, kSplitRatioKindCount> pairBestGain{};
        std::array<double, kSplitRatioKindCount> pairBestWireDelta{};
        std::array<int, kSplitRatioKindCount> pairBestMovedB{};
        std::array<int, kSplitRatioKindCount> pairBestMovedC{};
        double pairOverallBestGain = 0.0;
        double pairOverallBestWireDelta = 0.0;
        int pairOverallBestKind = -1;
        int pairOverallMovedB = 0;
        int pairOverallMovedC = 0;

        for (int k = 0; k < kSplitRatioKindCount; ++k) {
            const SplitRatioKind kind = static_cast<SplitRatioKind>(k);
            ++stats.allocationCandidates;
            ++stats.kindCandidates[k];
            const Split3Move move = evaluateThreeWayRatio(
                design, ledger, hot, keepEval, current, currentDemand,
                b, c, p.nets, kind);
            if (!move.valid) continue;
            const double gain = keepEval.key - move.eval.key;
            if (gain <= 1.0e-6) continue;
            ++stats.kindPositiveCandidates[k];
            const double wireDelta = move.eval.wire - keepEval.wire;
            pairBestGain[k] = gain;
            pairBestWireDelta[k] = wireDelta;
            pairBestMovedB[k] = move.movedB;
            pairBestMovedC[k] = move.movedC;
            if (gain > pairOverallBestGain + 1.0e-6 ||
                (std::fabs(gain - pairOverallBestGain) <= 1.0e-6 &&
                 wireDelta < pairOverallBestWireDelta)) {
                pairOverallBestGain = gain;
                pairOverallBestWireDelta = wireDelta;
                pairOverallBestKind = k;
                pairOverallMovedB = move.movedB;
                pairOverallMovedC = move.movedC;
            }
        }

        for (int k = 0; k < kSplitRatioKindCount; ++k) {
            if (pairBestGain[k] <= 1.0e-6) continue;
            ++stats.kindPositivePairs[k];
            stats.kindBestGainSum[k] += pairBestGain[k];
            stats.kindBestWireDeltaSum[k] += pairBestWireDelta[k];
            stats.kindMovedBNetsSum[k] += pairBestMovedB[k];
            stats.kindMovedCNetsSum[k] += pairBestMovedC[k];
            if (pairBestMovedB[k] > 0 && pairBestMovedC[k] > 0) {
                ++stats.kindBestUsesBothAlternatives[k];
            }
        }
        if (pairOverallBestGain > 1.0e-6 && pairOverallBestKind >= 0) {
            ++stats.bestPositivePairs;
            stats.bestGainSum += pairOverallBestGain;
            stats.bestWireDeltaSum += pairOverallBestWireDelta;
            stats.bestMovedBNetsSum += pairOverallMovedB;
            stats.bestMovedCNetsSum += pairOverallMovedC;
            ++stats.bestRatioWins[pairOverallBestKind];
            if (pairOverallMovedB > 0 && pairOverallMovedC > 0) {
                ++stats.bestUsesBothAlternatives;
            }
            if (pairOverallMovedB + pairOverallMovedC < p.nets) {
                ++stats.bestLeavesOnCurrent;
            }
        }
    }

    return stats;
}

std::vector<AllocationPart> splitAllocationParts(
        const routerx::LiteCandidate& current,
        const routerx::LiteCandidate& alternative,
        int nets,
        int moved) {
    std::vector<AllocationPart> parts;
    const int clamped = clampMovedNets(moved, nets);
    const int stay = nets - clamped;
    if (stay > 0) parts.push_back(AllocationPart{&current, stay});
    if (clamped > 0) parts.push_back(AllocationPart{&alternative, clamped});
    return parts;
}

SplitRatioOracleStats runSplitRatioOracle(
        const Design& design,
        const std::vector<routerx::DemandedPair>& pairs,
        const std::vector<routerx::LiteCandidate>& winners,
        const std::vector<std::vector<routerx::LiteCandidate>>& candidateSets,
        const routerx::RxResourceLedgerLite& ledger,
        const HotAxisRatios& hot) {
    SplitRatioOracleStats stats;

    for (size_t i = 0; i < pairs.size() && i < winners.size() &&
                       i < candidateSets.size(); ++i) {
        const routerx::DemandedPair& p = pairs[i];
        const routerx::LiteCandidate& current = winners[i];
        if (p.nets < 2 || current.route.steps.empty() ||
            !current.emitValid) {
            continue;
        }

        const RouteAxisDemand currentDemand =
            routeAxisDemand(design, current.route.steps, p.nets);
        const double hotDemand = hotAxisDemandTotal(currentDemand, hot);
        if (hotDemand <= 1.0e-9) continue;
        ++stats.winnerTouchesHot;

        routerx::RxResourceLedgerLite replaceLedger = ledger;
        replaceLedger.ripupSteps(design, current.route.steps, p.nets);
        const std::vector<AllocationPart> keepParts = {
            AllocationPart{&current, p.nets}
        };
        const AllocationEval keepEval =
            evaluateAllocation(design, replaceLedger, keepParts, design.alpha);

        bool pairHasRelief = false;
        std::array<double, kSplitRatioKindCount> pairBestGain{};
        std::array<double, kSplitRatioKindCount> pairBestWireDelta{};
        std::array<int, kSplitRatioKindCount> pairBestMoved{};
        double pairOverallBestGain = 0.0;
        double pairOverallBestWireDelta = 0.0;
        int pairOverallBestMoved = 0;
        int pairOverallBestKind = -1;

        for (const routerx::LiteCandidate& alt : candidateSets[i]) {
            if (!alt.emitValid) continue;
            if (alt.signature == current.signature) continue;

            const RouteAxisDemand altDemand =
                routeAxisDemand(design, alt.route.steps, p.nets);
            const double useful = usefulHotReliefTotal(
                design, ledger, hot, currentDemand, altDemand);
            if (useful <= 1.0e-9) continue;
            pairHasRelief = true;
            ++stats.reliefCandidates;

            for (int k = 0; k < kSplitRatioKindCount; ++k) {
                const SplitRatioKind kind = static_cast<SplitRatioKind>(k);
                int moved = 0;
                if (kind == SplitRatioKind::ClearSmallestHot) {
                    moved = clearHotMovedNets(design, ledger, hot, currentDemand,
                                              altDemand, p.nets, false);
                } else if (kind == SplitRatioKind::ClearLargestHot) {
                    moved = clearHotMovedNets(design, ledger, hot, currentDemand,
                                              altDemand, p.nets, true);
                } else {
                    moved = proportionalMovedNets(p.nets, kind);
                }
                if (moved <= 0) continue;

                ++stats.allocationCandidates;
                ++stats.kindCandidates[k];
                const std::vector<AllocationPart> parts =
                    splitAllocationParts(current, alt, p.nets, moved);
                const AllocationEval eval =
                    evaluateAllocation(design, replaceLedger, parts, design.alpha);
                if (!eval.valid) continue;
                const double gain = keepEval.key - eval.key;
                if (gain <= 1.0e-6) continue;

                ++stats.kindPositiveCandidates[k];
                const double wireDelta = eval.wire - keepEval.wire;
                if (gain > pairBestGain[k] + 1.0e-6 ||
                    (std::fabs(gain - pairBestGain[k]) <= 1.0e-6 &&
                     wireDelta < pairBestWireDelta[k])) {
                    pairBestGain[k] = gain;
                    pairBestWireDelta[k] = wireDelta;
                    pairBestMoved[k] = moved;
                }
                if (gain > pairOverallBestGain + 1.0e-6 ||
                    (std::fabs(gain - pairOverallBestGain) <= 1.0e-6 &&
                     wireDelta < pairOverallBestWireDelta)) {
                    pairOverallBestGain = gain;
                    pairOverallBestWireDelta = wireDelta;
                    pairOverallBestMoved = moved;
                    pairOverallBestKind = k;
                }
            }
        }

        if (!pairHasRelief) continue;
        ++stats.pairsWithReliefCandidate;
        for (int k = 0; k < kSplitRatioKindCount; ++k) {
            if (pairBestGain[k] <= 1.0e-6) continue;
            ++stats.kindPositivePairs[k];
            stats.kindBestGainSum[k] += pairBestGain[k];
            stats.kindBestWireDeltaSum[k] += pairBestWireDelta[k];
            stats.kindMovedNetsSum[k] += pairBestMoved[k];
            if (pairBestMoved[k] >= p.nets) ++stats.kindBestFullSwitchWins[k];
            else ++stats.kindBestPartialSplitWins[k];
        }
        if (pairOverallBestGain > 1.0e-6 && pairOverallBestKind >= 0) {
            ++stats.bestPositivePairs;
            stats.bestGainSum += pairOverallBestGain;
            stats.bestWireDeltaSum += pairOverallBestWireDelta;
            stats.bestMovedNetsSum += pairOverallBestMoved;
            ++stats.bestRatioWins[pairOverallBestKind];
            if (pairOverallBestMoved >= p.nets) ++stats.bestFullSwitchWins;
            else ++stats.bestPartialSplitWins;
        }
    }

    return stats;
}

int allocationSplitMovedNets(const Design& design,
                             const routerx::RxResourceLedgerLite& ledger,
                             const HotAxisRatios& hot,
                             const RouteAxisDemand& currentDemand,
                             const RouteAxisDemand& altDemand,
                             int nets,
                             AllocationSplitPolicy policy) {
    if (nets <= 0) return 0;
    switch (policy) {
        case AllocationSplitPolicy::ClearSmallestHot:
            return clearHotMovedNets(design, ledger, hot, currentDemand,
                                     altDemand, nets, false);
        case AllocationSplitPolicy::ClearLargestHot:
            return clearHotMovedNets(design, ledger, hot, currentDemand,
                                     altDemand, nets, true);
        case AllocationSplitPolicy::FivePercent:
            return clampMovedNets(static_cast<long long>(
                std::ceil(0.05 * static_cast<double>(nets) - 1.0e-9)), nets);
        case AllocationSplitPolicy::TenPercent:
            return clampMovedNets(static_cast<long long>(
                std::ceil(0.10 * static_cast<double>(nets) - 1.0e-9)), nets);
        case AllocationSplitPolicy::FifteenPercent:
            return clampMovedNets(static_cast<long long>(
                std::ceil(0.15 * static_cast<double>(nets) - 1.0e-9)), nets);
        case AllocationSplitPolicy::TwentyPercent:
            return clampMovedNets(static_cast<long long>(
                std::ceil(0.20 * static_cast<double>(nets) - 1.0e-9)), nets);
        case AllocationSplitPolicy::Quarter:
            return clampMovedNets(static_cast<long long>(
                std::ceil(0.25 * static_cast<double>(nets) - 1.0e-9)), nets);
        case AllocationSplitPolicy::ThirtyThreePercent:
            return clampMovedNets(static_cast<long long>(
                std::ceil(0.333333333333 * static_cast<double>(nets) - 1.0e-9)),
                nets);
        case AllocationSplitPolicy::FiftyPercent:
            return clampMovedNets(nets / 2, nets);
    }
    return 0;
}

void addAdaptiveMovedPoint(std::set<int>& points,
                           double raw,
                           int nets,
                           int radius = 0) {
    if (!std::isfinite(raw) || nets < 2) return;
    const long long center = static_cast<long long>(std::floor(raw + 1.0e-9));
    for (int d = -radius; d <= radius + 1; ++d) {
        const long long moved = center + d;
        if (moved > 0 && moved < nets) points.insert(static_cast<int>(moved));
    }
}

std::vector<int> routeFtTraversalCounts(
        const Design& design,
        const std::vector<RouteStep>& steps) {
    std::vector<int> counts(design.blocks.size(), 0);
    for (size_t i = 0; i + 1 < steps.size(); ++i) {
        const RouteStep& a = steps[i];
        const RouteStep& b = steps[i + 1];
        if (a.rectName != b.rectName || a.edge == b.edge) continue;
        const auto it = design.blockNameToIndex.find(a.rectName);
        if (it == design.blockNameToIndex.end()) continue;
        const int bi = it->second;
        if (bi >= 0 && bi < static_cast<int>(counts.size())) ++counts[bi];
    }
    return counts;
}

// Return the integer breakpoints at which a two-path allocation can change the
// official marginal.  Unlike the former fixed 5/15/25/33/50 percent menu, this
// is derived from the live world: channel capacity crossings, the official 5%
// aggregate-overflow knee, and per-block FT tier crossings.  A small eighth-grid
// remains as a guard for smooth FT/wire trade-offs between those breakpoints.
std::vector<int> adaptiveSplitMovedNets(
        const Design& design,
        const routerx::RxResourceLedgerLite& baseWithoutPair,
        const routerx::LiteCandidate& a,
        const routerx::LiteCandidate& b,
        int nets) {
    std::set<int> points;
    if (nets < 2 || !selectableCandidate(a) || !selectableCandidate(b) ||
        a.signature == b.signature) {
        return {};
    }

    addAdaptiveMovedPoint(points, 1.0, nets, 0);
    addAdaptiveMovedPoint(points, static_cast<double>(nets - 1), nets, 0);
    for (int q = 1; q < 8; ++q) {
        addAdaptiveMovedPoint(points,
            static_cast<double>(nets) * static_cast<double>(q) / 8.0,
            nets, 0);
    }
    static constexpr double kSeedRatios[] = {0.05, 0.15, 0.25, 1.0 / 3.0, 0.50};
    for (double ratio : kSeedRatios) {
        addAdaptiveMovedPoint(points, static_cast<double>(nets) * ratio, nets, 0);
    }

    const RouteAxisDemand da = routeAxisDemand(design, a.route.steps, nets);
    const RouteAxisDemand db = routeAxisDemand(design, b.route.steps, nets);
    for (int ci = 0; ci < baseWithoutPair.channelCount(); ++ci) {
        const routerx::LiteChannelUse& u = baseWithoutPair.channel(ci);
        auto addAxisBoundary = [&](double baseUsed, double cap,
                                   const std::vector<double>& av,
                                   const std::vector<double>& bv) {
            const double fullA = ci < static_cast<int>(av.size()) ? av[ci] : 0.0;
            const double fullB = ci < static_cast<int>(bv.size()) ? bv[ci] : 0.0;
            const double delta = (fullB - fullA) / static_cast<double>(nets);
            if (std::fabs(delta) <= 1.0e-12) return;
            const double usedAtA = baseUsed + fullA;
            addAdaptiveMovedPoint(points, (cap - usedAtA) / delta, nets);
        };
        addAxisBoundary(u.usedLR, u.capLR, da.lr, db.lr);
        addAxisBoundary(u.usedTB, u.capTB, da.tb, db.tb);
    }

    const routerx::LiteResourceSnapshot snap = baseWithoutPair.snapshot();
    const std::vector<int> ftA = routeFtTraversalCounts(design, a.route.steps);
    const std::vector<int> ftB = routeFtTraversalCounts(design, b.route.steps);
    static constexpr double kRawFtTierBoundaries[] = {0.0, 1500.0, 3000.0, 4500.0};
    for (size_t bi = 0; bi < design.blocks.size(); ++bi) {
        const double usedAtA =
            (bi < snap.ftUsedRawByBlock.size() ? snap.ftUsedRawByBlock[bi] : 0.0) +
            static_cast<double>(ftA[bi] * nets);
        const double delta = static_cast<double>(ftB[bi] - ftA[bi]);
        if (std::fabs(delta) <= 1.0e-12) continue;
        for (double boundary : kRawFtTierBoundaries) {
            addAdaptiveMovedPoint(points, (boundary - usedAtA) / delta, nets);
        }
    }

    // The channel penalty changes formula when aggregate overflow reaches 5%
    // of capacity.  Capacity crossings above partition overflow into linear
    // pieces; interpolate any knee crossing inside each resulting interval.
    std::vector<int> partition(points.begin(), points.end());
    partition.insert(partition.begin(), 0);
    partition.push_back(nets);
    std::sort(partition.begin(), partition.end());
    partition.erase(std::unique(partition.begin(), partition.end()), partition.end());
    auto overflowAt = [&](double moved) {
        double total = 0.0;
        for (int ci = 0; ci < baseWithoutPair.channelCount(); ++ci) {
            const routerx::LiteChannelUse& u = baseWithoutPair.channel(ci);
            const double aLR = ci < static_cast<int>(da.lr.size()) ? da.lr[ci] : 0.0;
            const double bLR = ci < static_cast<int>(db.lr.size()) ? db.lr[ci] : 0.0;
            const double aTB = ci < static_cast<int>(da.tb.size()) ? da.tb[ci] : 0.0;
            const double bTB = ci < static_cast<int>(db.tb.size()) ? db.tb[ci] : 0.0;
            const double usedLR = u.usedLR + aLR +
                moved * (bLR - aLR) / static_cast<double>(nets);
            const double usedTB = u.usedTB + aTB +
                moved * (bTB - aTB) / static_cast<double>(nets);
            total += std::max(0.0, usedLR - u.capLR);
            total += std::max(0.0, usedTB - u.capTB);
        }
        return total;
    };
    const double knee = rxscore::kOverflowThresh *
                        baseWithoutPair.totalChannelCapacity();
    for (size_t i = 0; i + 1 < partition.size(); ++i) {
        const double x0 = static_cast<double>(partition[i]);
        const double x1 = static_cast<double>(partition[i + 1]);
        if (x1 <= x0 + 1.0) continue;
        const double y0 = overflowAt(x0) - knee;
        const double y1 = overflowAt(x1) - knee;
        if (y0 * y1 > 0.0 || std::fabs(y1 - y0) <= 1.0e-12) continue;
        addAdaptiveMovedPoint(points, x0 - y0 * (x1 - x0) / (y1 - y0), nets);
    }

    return std::vector<int>(points.begin(), points.end());
}

bool allocationChoiceBetter(const AllocationPoolChoice& cand,
                            const AllocationPoolChoice& best) {
    if (!cand.eval.valid) return false;
    if (!best.eval.valid) return true;
    if (cand.eval.key < best.eval.key - 1.0e-6) return true;
    if (std::fabs(cand.eval.key - best.eval.key) > 1.0e-6) return false;
    if (cand.eval.wire < best.eval.wire - 1.0e-6) return true;
    if (std::fabs(cand.eval.wire - best.eval.wire) > 1.0e-6) return false;
    const int ck = static_cast<int>(cand.kind);
    const int bk = static_cast<int>(best.kind);
    if (ck != bk) return ck < bk;
    const std::string cs = cand.alternative ? cand.alternative->signature : "";
    const std::string bs = best.alternative ? best.alternative->signature : "";
    if (cs != bs) return cs < bs;
    return cand.movedNets < best.movedNets;
}

void noteSelectedFeedthrough(AllocationPoolSelectorStats& stats,
                             const std::vector<AllocationPart>& parts) {
    bool any = false;
    for (const AllocationPart& part : parts) {
        if (part.candidate && part.nets > 0 && part.candidate->usedFeedthrough) {
            any = true;
            break;
        }
    }
    if (any) ++stats.selectedUsedFeedthrough;
}

AllocationPoolSelectorStats runAllocationPoolSelector(
        const Design& design,
        const std::vector<routerx::DemandedPair>& pairs,
        const std::vector<routerx::LiteCandidate>& winners,
        const std::vector<std::vector<routerx::LiteCandidate>>& candidateSets,
        routerx::RxResourceLedgerLite& ledger,
        double& totalWire,
        double hotThreshold,
        std::array<long long, routerx::kLiteCandidateFamilyCount>& familyWins,
        bool allPairs,
        bool splitAllLosers,
        bool wideFixedRatios,
        bool allowFullSwitch,
        bool allowSplit,
        bool includeFiftyPercent = false,
        bool slimRatios = false,
        std::vector<std::vector<AllocationPart>>* selectedAllocations = nullptr) {
    AllocationPoolSelectorStats stats;
    if (selectedAllocations) {
        selectedAllocations->assign(pairs.size(), {});
    }

    for (size_t i = 0; i < pairs.size() && i < winners.size() &&
                       i < candidateSets.size(); ++i) {
        const routerx::DemandedPair& p = pairs[i];
        const routerx::LiteCandidate& current = winners[i];
        if (p.nets <= 0 || current.route.steps.empty() || !current.emitValid) {
            continue;
        }

        const HotAxisRatios hot = hotAxisRatios(design, ledger, hotThreshold);
        const RouteAxisDemand currentDemand =
            routeAxisDemand(design, current.route.steps, p.nets);
        const double hotDemand = hotAxisDemandTotal(currentDemand, hot);
        if (!allPairs && hotDemand <= 1.0e-9) {
            noteSelectedFeedthrough(stats, {AllocationPart{&current, p.nets}});
            const int fidx = static_cast<int>(current.family);
            if (fidx >= 0 && fidx < routerx::kLiteCandidateFamilyCount) {
                ++familyWins[fidx];
            }
            continue;
        }
        ++stats.eligiblePairs;

        const routerx::RxResourceLedgerLite fullLedger = ledger;
        ledger.ripupSteps(design, current.route.steps, p.nets);
        const std::vector<AllocationPart> keepParts = {
            AllocationPart{&current, p.nets}
        };
        const AllocationEval keepEval =
            evaluateAllocation(design, ledger, keepParts, design.alpha);
        AllocationPoolChoice best;
        best.kind = AllocationPoolChoiceKind::Keep;
        best.eval = keepEval;
        ++stats.allocationCandidates;

        for (const routerx::LiteCandidate& alt : candidateSets[i]) {
            if (!selectableCandidate(alt)) continue;
            if (alt.signature == current.signature) continue;
            ++stats.loserCandidates;

            if (allowFullSwitch) {
                const std::vector<AllocationPart> fullParts = {
                    AllocationPart{&alt, p.nets}
                };
                AllocationPoolChoice full;
                full.kind = AllocationPoolChoiceKind::FullSwitch;
                full.alternative = &alt;
                full.movedNets = p.nets;
                full.eval = evaluateAllocation(design, ledger, fullParts, design.alpha);
                ++stats.allocationCandidates;
                if (allocationChoiceBetter(full, best)) best = full;
            }

            if (!allowSplit) continue;

            const RouteAxisDemand altDemand =
                routeAxisDemand(design, alt.route.steps, p.nets);
            const double useful = usefulHotReliefTotal(
                design, fullLedger, hot, currentDemand, altDemand);
            const bool hasUsefulRelief = useful > 1.0e-9;
            if (hasUsefulRelief) ++stats.reliefLoserCandidates;
            if (!hasUsefulRelief && !splitAllLosers) continue;

            std::vector<std::pair<int, int>> movedChoices;
            std::set<int> seenMoved;
            for (int pi = 0; pi < kAllocationSplitPolicyCount; ++pi) {
                const AllocationSplitPolicy policy =
                    static_cast<AllocationSplitPolicy>(pi);
                if (slimRatios) {
                    // Keep the proven slim selector as the deterministic seed.
                    // The adaptive any-two-path search runs in the fixed-point
                    // reselect below, where the whole seed remains available as
                    // Keep and every accepted move is transactionally monotone.
                    if (!allocationSplitPolicySlimSet(policy)) continue;
                } else {
                    if (!wideFixedRatios && allocationSplitPolicyWideOnly(policy)) {
                        continue;
                    }
                    if (policy == AllocationSplitPolicy::FiftyPercent &&
                        !includeFiftyPercent) {
                        continue;
                    }
                }
                if (!hasUsefulRelief &&
                    (policy == AllocationSplitPolicy::ClearSmallestHot ||
                     policy == AllocationSplitPolicy::ClearLargestHot)) {
                    continue;
                }
                const int moved = allocationSplitMovedNets(
                    design, fullLedger, hot, currentDemand, altDemand,
                    p.nets, policy);
                if (moved <= 0 || moved >= p.nets ||
                    !seenMoved.insert(moved).second) {
                    continue;
                }
                movedChoices.push_back({moved, pi});
            }
            for (const auto& movedChoice : movedChoices) {
                const int moved = movedChoice.first;
                const int pi = movedChoice.second;
                ++stats.allocationCandidates;
                if (pi >= 0 && pi < kAllocationSplitPolicyCount) {
                    ++stats.splitPolicyCandidates[pi];
                }
                const std::vector<AllocationPart> splitParts =
                    splitAllocationParts(current, alt, p.nets, moved);
                AllocationPoolChoice split;
                split.kind = AllocationPoolChoiceKind::Split;
                split.alternative = &alt;
                split.movedNets = moved;
                split.splitPolicyIndex = pi;
                split.eval = evaluateAllocation(design, ledger, splitParts, design.alpha);
                if (allocationChoiceBetter(split, best)) best = split;
            }
        }

        if (!best.eval.valid || best.eval.key > keepEval.key + 1.0e-6) {
            best.kind = AllocationPoolChoiceKind::Keep;
            best.alternative = nullptr;
            best.movedNets = 0;
            best.splitPolicyIndex = -1;
            best.eval = keepEval;
        }

        std::vector<AllocationPart> selectedParts = keepParts;
        const routerx::LiteCandidate* selectedFamilySource = &current;
        switch (best.kind) {
            case AllocationPoolChoiceKind::Keep:
                ++stats.keepWins;
                break;
            case AllocationPoolChoiceKind::BaseSingle:
                ++stats.keepWins;
                break;
            case AllocationPoolChoiceKind::FullSwitch:
                if (best.alternative) {
                    selectedParts = {AllocationPart{best.alternative, p.nets}};
                    selectedFamilySource = best.alternative;
                    ++stats.fullSwitchWins;
                    ++stats.changedPairs;
                    stats.localOfficialGainSum += keepEval.key - best.eval.key;
                    stats.localWireDeltaSum += best.eval.wire - keepEval.wire;
                    stats.fullSwitchOfficialGainSum += keepEval.key - best.eval.key;
                    stats.fullSwitchWireDeltaSum += best.eval.wire - keepEval.wire;
                    totalWire += best.eval.wire - keepEval.wire;
                }
                break;
            case AllocationPoolChoiceKind::Split:
                if (best.alternative && best.movedNets > 0) {
                    selectedParts = splitAllocationParts(
                        current, *best.alternative, p.nets, best.movedNets);
                    ++stats.splitWins;
                    ++stats.changedPairs;
                    if (best.splitPolicyIndex >= 0 &&
                        best.splitPolicyIndex < kAllocationSplitPolicyCount) {
                        ++stats.splitPolicyWins[best.splitPolicyIndex];
                    }
                    stats.localOfficialGainSum += keepEval.key - best.eval.key;
                    stats.localWireDeltaSum += best.eval.wire - keepEval.wire;
                    stats.splitOfficialGainSum += keepEval.key - best.eval.key;
                    stats.splitWireDeltaSum += best.eval.wire - keepEval.wire;
                    stats.movedNetsSum += best.movedNets;
                    totalWire += best.eval.wire - keepEval.wire;
                }
                break;
        }

        if (selectedAllocations && i < selectedAllocations->size()) {
            (*selectedAllocations)[i] = selectedParts;
        }
        commitAllocation(design, ledger, selectedParts);
        noteSelectedFeedthrough(stats, selectedParts);
        if (selectedFamilySource) {
            const int fidx = static_cast<int>(selectedFamilySource->family);
            if (fidx >= 0 && fidx < routerx::kLiteCandidateFamilyCount) {
                ++familyWins[fidx];
            }
        }
    }

    return stats;
}

void ripupAllocation(const Design& design,
                     routerx::RxResourceLedgerLite& ledger,
                     const std::vector<AllocationPart>& parts) {
    for (const AllocationPart& part : parts) {
        if (!part.candidate || part.nets <= 0) continue;
        ledger.ripupSteps(design, part.candidate->route.steps, part.nets);
    }
}

std::string allocationSignature(const std::vector<AllocationPart>& parts) {
    std::vector<std::string> tokens;
    tokens.reserve(parts.size());
    for (const AllocationPart& part : parts) {
        if (!part.candidate || part.nets <= 0) continue;
        tokens.push_back(std::to_string(part.nets) + ":" + part.candidate->signature);
    }
    std::sort(tokens.begin(), tokens.end());
    std::string out;
    for (const std::string& token : tokens) {
        if (!out.empty()) out += "|";
        out += token;
    }
    return out;
}

int activeAllocationPartCount(const std::vector<AllocationPart>& parts) {
    int count = 0;
    for (const AllocationPart& part : parts) {
        if (part.candidate && part.nets > 0) ++count;
    }
    return count;
}

bool allocationDemandIsExact(const std::vector<AllocationPart>& parts,
                             int requiredNets,
                             int maxParts = 2) {
    long long delivered = 0;
    int active = 0;
    std::set<std::string> signatures;
    for (const AllocationPart& part : parts) {
        if (!part.candidate || part.nets <= 0) return false;
        delivered += part.nets;
        ++active;
        if (!signatures.insert(part.candidate->signature).second) return false;
    }
    return delivered == requiredNets && active >= 1 && active <= maxParts;
}

enum class AdaptiveAllocationKind {
    Keep,
    Single,
    Split,
};

struct AdaptiveAllocationChoice {
    AdaptiveAllocationKind kind = AdaptiveAllocationKind::Keep;
    std::vector<AllocationPart> parts;
    AllocationEval eval;
    int movedNets = 0;
};

bool adaptiveAllocationChoiceBetter(const AdaptiveAllocationChoice& cand,
                                    const AdaptiveAllocationChoice& best) {
    if (!cand.eval.valid) return false;
    if (!best.eval.valid) return true;
    if (cand.eval.key < best.eval.key - 1.0e-6) return true;
    if (std::fabs(cand.eval.key - best.eval.key) > 1.0e-6) return false;
    const int candParts = activeAllocationPartCount(cand.parts);
    const int bestParts = activeAllocationPartCount(best.parts);
    if (candParts != bestParts) return candParts < bestParts;
    if (cand.eval.wire < best.eval.wire - 1.0e-6) return true;
    if (std::fabs(cand.eval.wire - best.eval.wire) > 1.0e-6) return false;
    return allocationSignature(cand.parts) < allocationSignature(best.parts);
}

// Preserve the former fixed-point selector as a stable seed.  The redesigned
// adaptive pass starts from this completed world, so it can add capacity-aware
// splits without trading away a result that the previous router already found.
void runLegacyAllocationReselectFixedPoint(
        const Design& design,
        const std::vector<routerx::DemandedPair>& pairs,
        const std::vector<routerx::LiteCandidate>& baseWinners,
        const std::vector<std::vector<routerx::LiteCandidate>>& candidateSets,
        std::vector<std::vector<AllocationPart>>& allocations,
        routerx::RxResourceLedgerLite& ledger,
        double& totalWire,
        double hotThreshold,
        int maxPasses,
        std::vector<AllocationReselectPassStats>& passStats) {
    const double outlineArea = design.outlineW * design.outlineH;
    auto scoreNoRuntime = [&](const routerx::RxResourceLedgerLite& l,
                              double wire) {
        rxscore::Inputs in;
        in.alpha = design.alpha;
        in.outlineArea = outlineArea;
        in.totalWireLength = wire;
        in.channelOverflowTotal = l.totalChannelOverflow();
        in.channelCapacityTotal = l.totalChannelCapacity();
        in.blocks = l.blockFtInputs(design);
        return rxscore::score(in).totalNoRuntime;
    };

    for (int pass = 1; pass <= maxPasses; ++pass) {
        const double beforeCost = scoreNoRuntime(ledger, totalWire);
        const routerx::LiteResourceSnapshot beforeLedger = ledger.snapshot();
        const std::vector<std::vector<AllocationPart>> beforeAllocations = allocations;
        const double beforeWire = totalWire;
        AllocationReselectPassStats ps;
        ps.pass = pass;
        ps.channelOverflowBefore = ledger.totalChannelOverflow();
        ps.ftRequiredBefore = ledger.ftResult(design).sumRb;

        for (size_t i = 0; i < pairs.size() && i < baseWinners.size() &&
                           i < candidateSets.size(); ++i) {
            const routerx::DemandedPair& p = pairs[i];
            const routerx::LiteCandidate& base = baseWinners[i];
            if (p.nets <= 0 || base.route.steps.empty() || !base.emitValid) continue;
            if (allocations[i].empty() ||
                !allocationDemandIsExact(allocations[i], p.nets)) {
                allocations[i] = {AllocationPart{&base, p.nets}};
            }

            const std::vector<AllocationPart> currentParts = allocations[i];
            const std::string currentSig = allocationSignature(currentParts);
            const HotAxisRatios hot = hotAxisRatios(design, ledger, hotThreshold);
            const routerx::RxResourceLedgerLite fullLedger = ledger;
            ripupAllocation(design, ledger, currentParts);

            const AllocationEval keepEval =
                evaluateAllocation(design, ledger, currentParts, design.alpha);
            AllocationPoolChoice best;
            best.kind = AllocationPoolChoiceKind::Keep;
            best.eval = keepEval;
            ++ps.allocationCandidates;

            const std::vector<AllocationPart> baseParts = {
                AllocationPart{&base, p.nets}
            };
            AllocationPoolChoice baseSingle;
            baseSingle.kind = AllocationPoolChoiceKind::BaseSingle;
            baseSingle.alternative = &base;
            baseSingle.eval = evaluateAllocation(
                design, ledger, baseParts, design.alpha);
            ++ps.allocationCandidates;
            if (allocationChoiceBetter(baseSingle, best)) best = baseSingle;

            const RouteAxisDemand baseDemand =
                routeAxisDemand(design, base.route.steps, p.nets);
            for (const routerx::LiteCandidate& alt : candidateSets[i]) {
                if (!selectableCandidate(alt) || alt.signature == base.signature) continue;
                const RouteAxisDemand altDemand =
                    routeAxisDemand(design, alt.route.steps, p.nets);
                const double useful = usefulHotReliefTotal(
                    design, fullLedger, hot, baseDemand, altDemand);
                const bool hasUsefulRelief = useful > 1.0e-9;
                std::set<int> seenMoved;
                for (int pi = 0; pi < kAllocationSplitPolicyCount; ++pi) {
                    const AllocationSplitPolicy policy =
                        static_cast<AllocationSplitPolicy>(pi);
                    if (allocationSplitPolicyWideOnly(policy)) continue;
                    if (!hasUsefulRelief &&
                        (policy == AllocationSplitPolicy::ClearSmallestHot ||
                         policy == AllocationSplitPolicy::ClearLargestHot)) {
                        continue;
                    }
                    const int moved = allocationSplitMovedNets(
                        design, fullLedger, hot, baseDemand, altDemand,
                        p.nets, policy);
                    if (moved <= 0 || moved >= p.nets ||
                        !seenMoved.insert(moved).second) {
                        continue;
                    }
                    const std::vector<AllocationPart> splitParts =
                        splitAllocationParts(base, alt, p.nets, moved);
                    AllocationPoolChoice split;
                    split.kind = AllocationPoolChoiceKind::Split;
                    split.alternative = &alt;
                    split.movedNets = moved;
                    split.splitPolicyIndex = pi;
                    split.eval = evaluateAllocation(
                        design, ledger, splitParts, design.alpha);
                    ++ps.allocationCandidates;
                    if (allocationChoiceBetter(split, best)) best = split;
                }
            }

            if (!best.eval.valid || best.eval.key >= keepEval.key - 1.0e-6) {
                best.kind = AllocationPoolChoiceKind::Keep;
                best.alternative = nullptr;
                best.movedNets = 0;
                best.eval = keepEval;
            }

            std::vector<AllocationPart> selectedParts = currentParts;
            if (best.kind == AllocationPoolChoiceKind::BaseSingle) {
                selectedParts = baseParts;
                ++ps.baseSingleWins;
            } else if (best.kind == AllocationPoolChoiceKind::Split &&
                       best.alternative && best.movedNets > 0) {
                selectedParts = splitAllocationParts(
                    base, *best.alternative, p.nets, best.movedNets);
                ++ps.splitWins;
                ps.movedNetsSum += best.movedNets;
            } else {
                ++ps.keepWins;
            }
            if (!allocationDemandIsExact(selectedParts, p.nets)) {
                selectedParts = currentParts;
                best.eval = keepEval;
            }
            if (allocationSignature(selectedParts) != currentSig) ++ps.changedPairs;
            ps.wireGain += keepEval.wire - best.eval.wire;
            totalWire += best.eval.wire - keepEval.wire;
            allocations[i] = selectedParts;
            commitAllocation(design, ledger, selectedParts);
        }

        double afterCost = scoreNoRuntime(ledger, totalWire);
        if (afterCost > beforeCost + 1.0e-6) {
            ledger.restore(beforeLedger);
            allocations = beforeAllocations;
            totalWire = beforeWire;
            afterCost = beforeCost;
            ps.changedPairs = 0;
            ps.wireGain = 0.0;
            ps.movedNetsSum = 0.0;
        }
        ps.channelOverflowAfter = ledger.totalChannelOverflow();
        ps.ftRequiredAfter = ledger.ftResult(design).sumRb;
        ps.officialGain = beforeCost - afterCost;
        passStats.push_back(ps);
        if (ps.changedPairs == 0) break;
    }
}

AllocationReselectResult runAllocationReselectFixedPoint(
        const Design& design,
        const std::vector<routerx::DemandedPair>& pairs,
        const std::vector<routerx::LiteCandidate>& baseWinners,
        const std::vector<std::vector<routerx::LiteCandidate>>& candidateSets,
        std::vector<std::vector<AllocationPart>>& allocations,
        routerx::RxResourceLedgerLite& ledger,
        double& totalWire,
        double hotThreshold,
        int maxPasses) {
    AllocationReselectResult result;
    if (maxPasses <= 0) return result;
    if (allocations.size() < pairs.size()) allocations.resize(pairs.size());

    const double outlineArea = design.outlineW * design.outlineH;
    auto scoreNoRuntime = [&](const routerx::RxResourceLedgerLite& l,
                              double wire) {
        rxscore::Inputs in;
        in.alpha = design.alpha;
        in.outlineArea = outlineArea;
        in.totalWireLength = wire;
        in.channelOverflowTotal = l.totalChannelOverflow();
        in.channelCapacityTotal = l.totalChannelCapacity();
        in.runtimeSec = 0.0;
        in.blocks = l.blockFtInputs(design);
        return rxscore::score(in).totalNoRuntime;
    };

    const size_t blockCount = design.blockSpecs.size();
    const bool previouslyDenseBypass =
        pairs.size() >= 20 &&
        (blockCount >= 20 || (blockCount >= 10 && blockCount <= 12));
    if (!previouslyDenseBypass) {
        runLegacyAllocationReselectFixedPoint(
            design, pairs, baseWinners, candidateSets, allocations,
            ledger, totalWire, hotThreshold, maxPasses, result.passes);
    }

    const rxscore::FtResult seededFt = ledger.ftResult(design);
    const bool hasResidualPressure =
        ledger.totalChannelOverflow() > 1.0e-9 ||
        seededFt.penalty > 1.0e-9 || seededFt.illegalPenalty > 1.0e-9;
    const int adaptiveMaxPasses = hasResidualPressure
        ? std::min(maxPasses, 3) : 0;
    const int adaptivePassOffset = static_cast<int>(result.passes.size());

    for (int pass = 1; pass <= adaptiveMaxPasses; ++pass) {
        const double beforeCost = scoreNoRuntime(ledger, totalWire);
        const routerx::LiteResourceSnapshot beforeLedger = ledger.snapshot();
        const std::vector<std::vector<AllocationPart>> beforeAllocations = allocations;
        const double beforeWire = totalWire;
        AllocationReselectPassStats ps;
        ps.pass = adaptivePassOffset + pass;
        ps.channelOverflowBefore = ledger.totalChannelOverflow();
        ps.ftRequiredBefore = ledger.ftResult(design).sumRb;

        for (size_t i = 0; i < pairs.size() && i < baseWinners.size() &&
                           i < candidateSets.size(); ++i) {
            const routerx::DemandedPair& p = pairs[i];
            const routerx::LiteCandidate& base = baseWinners[i];
            if (p.nets <= 0 || base.route.steps.empty() || !base.emitValid) {
                ++result.selectedInvalid;
                continue;
            }
            if (allocations[i].empty()) {
                allocations[i] = {AllocationPart{&base, p.nets}};
            }

            std::vector<AllocationPart> currentParts = allocations[i];
            if (!allocationDemandIsExact(currentParts, p.nets)) {
                currentParts = {AllocationPart{&base, p.nets}};
            }
            const std::string currentSig = allocationSignature(currentParts);
            const routerx::RxResourceLedgerLite fullLedger = ledger;
            ripupAllocation(design, ledger, currentParts);

            const AllocationEval keepEval =
                evaluateAllocation(design, ledger, currentParts, design.alpha);
            AdaptiveAllocationChoice best;
            best.kind = AdaptiveAllocationKind::Keep;
            best.parts = currentParts;
            best.eval = keepEval;
            ++ps.allocationCandidates;

            std::vector<const routerx::LiteCandidate*> candidatePool;
            std::set<std::string> candidateSignatures;
            auto addCandidate = [&](const routerx::LiteCandidate* c) {
                if (!c || !selectableCandidate(*c) || c->route.steps.empty()) return;
                if (candidateSignatures.insert(c->signature).second) {
                    candidatePool.push_back(c);
                }
            };
            addCandidate(&base);
            for (const AllocationPart& part : currentParts) addCandidate(part.candidate);
            for (const routerx::LiteCandidate& c : candidateSets[i]) addCandidate(&c);

            RouteAxisDemand currentDemand;
            currentDemand.lr.assign(design.channels.size(), 0.0);
            currentDemand.tb.assign(design.channels.size(), 0.0);
            for (const AllocationPart& part : currentParts) {
                const RouteAxisDemand d = routeAxisDemand(
                    design, part.candidate->route.steps, part.nets);
                for (size_t ci = 0; ci < currentDemand.lr.size(); ++ci) {
                    if (ci < d.lr.size()) currentDemand.lr[ci] += d.lr[ci];
                    if (ci < d.tb.size()) currentDemand.tb[ci] += d.tb[ci];
                }
            }

            struct RankedCandidate {
                const routerx::LiteCandidate* candidate = nullptr;
                AllocationEval fullEval;
                double hotRelief = 0.0;
            };
            std::vector<RankedCandidate> ranked;
            ranked.reserve(candidatePool.size());
            for (const routerx::LiteCandidate* c : candidatePool) {
                const std::vector<AllocationPart> singleParts = {
                    AllocationPart{c, p.nets}
                };
                const AllocationEval singleEval =
                    evaluateAllocation(design, ledger, singleParts, design.alpha);
                ++ps.allocationCandidates;
                AdaptiveAllocationChoice single;
                single.kind = AdaptiveAllocationKind::Single;
                single.parts = singleParts;
                single.eval = singleEval;
                if (adaptiveAllocationChoiceBetter(single, best)) best = single;

                const RouteAxisDemand cd =
                    routeAxisDemand(design, c->route.steps, p.nets);
                double relief = 0.0;
                for (int ci = 0; ci < fullLedger.channelCount(); ++ci) {
                    const routerx::LiteChannelUse& u = fullLedger.channel(ci);
                    const double curLR = ci < static_cast<int>(currentDemand.lr.size())
                        ? currentDemand.lr[ci] : 0.0;
                    const double curTB = ci < static_cast<int>(currentDemand.tb.size())
                        ? currentDemand.tb[ci] : 0.0;
                    const double newLR = ci < static_cast<int>(cd.lr.size()) ? cd.lr[ci] : 0.0;
                    const double newTB = ci < static_cast<int>(cd.tb.size()) ? cd.tb[ci] : 0.0;
                    relief += std::min(std::max(0.0, u.usedLR - u.capLR),
                                       std::max(0.0, curLR - newLR));
                    relief += std::min(std::max(0.0, u.usedTB - u.capTB),
                                       std::max(0.0, curTB - newTB));
                }
                ranked.push_back(RankedCandidate{c, singleEval, relief});
            }

            std::vector<const routerx::LiteCandidate*> shortlist;
            std::set<std::string> shortlistSignatures;
            auto addShortlist = [&](const routerx::LiteCandidate* c) {
                if (c && shortlistSignatures.insert(c->signature).second) {
                    shortlist.push_back(c);
                }
            };
            addShortlist(&base);
            for (const AllocationPart& part : currentParts) addShortlist(part.candidate);

            std::vector<RankedCandidate> byFull = ranked;
            std::sort(byFull.begin(), byFull.end(), [](const RankedCandidate& x,
                                                       const RankedCandidate& y) {
                if (x.fullEval.valid != y.fullEval.valid) return x.fullEval.valid;
                if (x.fullEval.key != y.fullEval.key) return x.fullEval.key < y.fullEval.key;
                return x.candidate->signature < y.candidate->signature;
            });
            for (size_t k = 0; k < byFull.size() && k < 2; ++k) {
                if (byFull[k].fullEval.valid) addShortlist(byFull[k].candidate);
            }
            std::vector<RankedCandidate> byRelief = ranked;
            std::sort(byRelief.begin(), byRelief.end(), [](const RankedCandidate& x,
                                                           const RankedCandidate& y) {
                if (x.hotRelief != y.hotRelief) return x.hotRelief > y.hotRelief;
                if (x.fullEval.key != y.fullEval.key) return x.fullEval.key < y.fullEval.key;
                return x.candidate->signature < y.candidate->signature;
            });
            for (size_t k = 0; k < byRelief.size() && k < 2; ++k) {
                if (byRelief[k].hotRelief > 1.0e-9) addShortlist(byRelief[k].candidate);
            }

            bool splitRelevant = false;
            for (int ci = 0; ci < fullLedger.channelCount() && !splitRelevant; ++ci) {
                const routerx::LiteChannelUse& u = fullLedger.channel(ci);
                const double curLR = ci < static_cast<int>(currentDemand.lr.size())
                    ? currentDemand.lr[ci] : 0.0;
                const double curTB = ci < static_cast<int>(currentDemand.tb.size())
                    ? currentDemand.tb[ci] : 0.0;
                if ((curLR > 1.0e-9 && u.usedLR + 1.0e-9 >= hotThreshold * u.capLR) ||
                    (curTB > 1.0e-9 && u.usedTB + 1.0e-9 >= hotThreshold * u.capTB)) {
                    splitRelevant = true;
                }
            }
            const rxscore::FtResult fullFt = fullLedger.ftResult(design);
            if (!splitRelevant &&
                (fullFt.penalty > 1.0e-9 ||
                 fullFt.sumRb + 1.0e-9 >= outlineArea)) {
                for (const AllocationPart& part : currentParts) {
                    if (part.candidate && part.candidate->usedFeedthrough) {
                        splitRelevant = true;
                        break;
                    }
                }
            }

            for (size_t ai = 0; splitRelevant && ai < shortlist.size(); ++ai) {
                for (size_t bi = ai + 1; bi < shortlist.size(); ++bi) {
                    const routerx::LiteCandidate& a = *shortlist[ai];
                    const routerx::LiteCandidate& b = *shortlist[bi];
                    const std::vector<int> movedChoices = adaptiveSplitMovedNets(
                        design, ledger, a, b, p.nets);
                    for (int moved : movedChoices) {
                        std::vector<AllocationPart> splitParts = {
                            AllocationPart{&a, p.nets - moved},
                            AllocationPart{&b, moved}
                        };
                        if (!allocationDemandIsExact(splitParts, p.nets)) continue;
                        AdaptiveAllocationChoice split;
                        split.kind = AdaptiveAllocationKind::Split;
                        split.parts = std::move(splitParts);
                        split.movedNets = std::min(moved, p.nets - moved);
                        split.eval = evaluateAllocation(
                            design, ledger, split.parts, design.alpha);
                        ++ps.allocationCandidates;
                        if (adaptiveAllocationChoiceBetter(split, best)) best = std::move(split);
                    }
                }
            }

            if (!best.eval.valid || best.eval.key >= keepEval.key - 1.0e-6 ||
                !allocationDemandIsExact(best.parts, p.nets)) {
                best.kind = AdaptiveAllocationKind::Keep;
                best.parts = currentParts;
                best.eval = keepEval;
                best.movedNets = 0;
            }

            const std::vector<AllocationPart>& selectedParts = best.parts;
            if (best.kind == AdaptiveAllocationKind::Keep) {
                ++ps.keepWins;
            } else if (best.kind == AdaptiveAllocationKind::Single) {
                ++ps.baseSingleWins;
            } else {
                ++ps.splitWins;
                ps.movedNetsSum += best.movedNets;
            }
            const std::string selectedSig = allocationSignature(selectedParts);
            if (selectedSig != currentSig) ++ps.changedPairs;
            ps.wireGain += keepEval.wire - best.eval.wire;
            totalWire += best.eval.wire - keepEval.wire;
            allocations[i] = selectedParts;
            commitAllocation(design, ledger, selectedParts);
        }

        double afterCost = scoreNoRuntime(ledger, totalWire);
        if (afterCost > beforeCost + 1.0e-6) {
            // The local score is exact, so this should be unreachable.  Keep the
            // pass transactional anyway: a future ledger/scorer change cannot
            // silently make split reselect regress the official objective.
            ledger.restore(beforeLedger);
            allocations = beforeAllocations;
            totalWire = beforeWire;
            afterCost = beforeCost;
            ps.changedPairs = 0;
            ps.wireGain = 0.0;
            ps.movedNetsSum = 0.0;
        }
        ps.channelOverflowAfter = ledger.totalChannelOverflow();
        ps.ftRequiredAfter = ledger.ftResult(design).sumRb;
        ps.officialGain = beforeCost - afterCost;
        result.passes.push_back(ps);
        if (ps.changedPairs == 0) break;
    }

    for (size_t i = 0; i < allocations.size(); ++i) {
        bool anyFt = false;
        for (const AllocationPart& part : allocations[i]) {
            if (!part.candidate || part.nets <= 0) continue;
            if (!part.candidate->emitValid || !selectableCandidate(*part.candidate)) {
                ++result.selectedInvalid;
            }
            if (part.candidate->impact.dIllegalFtPenalty > 0.0 ||
                part.candidate->impact.illegalFtBlocksAfter > 0) {
                ++result.selectedIllegalFt;
            }
            if (part.candidate->usedFeedthrough) anyFt = true;
            const int fidx = static_cast<int>(part.candidate->family);
            if (fidx >= 0 && fidx < routerx::kLiteCandidateFamilyCount) {
                ++result.familyWins[fidx];
            }
        }
        if (anyFt) ++result.selectedUsedFeedthrough;
    }
    return result;
}

std::vector<AllocationPart> allocationOrBaseParts(
        size_t pairIndex,
        const std::vector<std::vector<AllocationPart>>& allocations,
        const std::vector<routerx::LiteCandidate>& baseWinners,
        const std::vector<routerx::DemandedPair>& pairs) {
    if (pairIndex < allocations.size() && !allocations[pairIndex].empty()) {
        return allocations[pairIndex];
    }
    if (pairIndex < baseWinners.size() && pairIndex < pairs.size()) {
        return {AllocationPart{&baseWinners[pairIndex], pairs[pairIndex].nets}};
    }
    return {};
}

bool materializeRoutes(
        Design& design,
        const std::vector<routerx::DemandedPair>& pairs,
        const std::vector<std::vector<AllocationPart>>& allocations,
        const std::vector<routerx::LiteCandidate>& baseWinners) {
    design.routes.clear();
    bool allRouted = true;

    for (size_t i = 0; i < pairs.size(); ++i) {
        const routerx::DemandedPair& pair = pairs[i];
        const std::string& srcName = design.blocks[pair.src].spec.name;
        const std::string& dstName = design.blocks[pair.dst].spec.name;
        int delivered = 0;

        std::vector<AllocationPart> parts =
            allocationOrBaseParts(i, allocations, baseWinners, pairs);
        if (!allocationDemandIsExact(parts, pair.nets)) {
            parts.clear();
            if (i < baseWinners.size() && selectableCandidate(baseWinners[i])) {
                parts.push_back(AllocationPart{&baseWinners[i], pair.nets});
            }
        }
        for (const AllocationPart& part : parts) {
            if (!part.candidate || part.nets <= 0) continue;
            routerx::EmitReject reject = routerx::EmitReject::None;
            if (routerx::RxEmit::emitPath(part.candidate->route.steps,
                                          part.nets,
                                          srcName,
                                          dstName,
                                          design,
                                          reject)) {
                delivered += part.nets;
            }
        }

        if (delivered != pair.nets) {
            allRouted = false;
            RoutePath open;
            open.netCount = std::max(0, pair.nets - delivered);
            open.srcBlock = srcName;
            open.dstBlock = dstName;
            open.open = true;
            int srcEdge = routerx::facingEdge(design.blocks[pair.src].rect,
                                              design.blocks[pair.dst].rect);
            if (srcEdge < 1 || srcEdge > 4) srcEdge = 1;
            open.steps.push_back({srcName, srcEdge});
            open.steps.push_back({dstName, routerx::oppositeEdge(srcEdge)});
            design.routes.push_back(std::move(open));
        }
    }
    return allRouted;
}

double allocationAxisDemand(const Design& design,
                            const std::vector<AllocationPart>& parts,
                            bool lr,
                            int channelIndex) {
    double total = 0.0;
    for (const AllocationPart& part : parts) {
        if (!part.candidate || part.nets <= 0) continue;
        const RouteAxisDemand d =
            routeAxisDemand(design, part.candidate->route.steps, part.nets);
        total += demandOnAxis(d, lr, channelIndex);
    }
    return total;
}

long long allocationPathPartsOnAxis(const Design& design,
                                    const std::vector<AllocationPart>& parts,
                                    bool lr,
                                    int channelIndex) {
    long long n = 0;
    for (const AllocationPart& part : parts) {
        if (!part.candidate || part.nets <= 0) continue;
        const RouteAxisDemand d =
            routeAxisDemand(design, part.candidate->route.steps, part.nets);
        if (demandOnAxis(d, lr, channelIndex) > 1.0e-9) ++n;
    }
    return n;
}

std::string connLabel(const Design& design, const routerx::DemandedPair& p) {
    const std::string src = (p.src >= 0 && p.src < static_cast<int>(design.blocks.size()))
        ? design.blocks[p.src].spec.name : std::string("src?");
    const std::string dst = (p.dst >= 0 && p.dst < static_cast<int>(design.blocks.size()))
        ? design.blocks[p.dst].spec.name : std::string("dst?");
    return src + "->" + dst;
}

std::string allocationPartsLabel(const std::vector<AllocationPart>& parts) {
    std::string out;
    for (const AllocationPart& part : parts) {
        if (!part.candidate || part.nets <= 0) continue;
        if (!out.empty()) out += "|";
        out += std::string(routerx::liteFamilyName(part.candidate->family)) +
               ":" + std::to_string(part.nets);
    }
    return out.empty() ? std::string("none") : out;
}

bool allocationHasSplit(const std::vector<AllocationPart>& parts) {
    int active = 0;
    for (const AllocationPart& part : parts) {
        if (part.candidate && part.nets > 0) ++active;
    }
    return active > 1;
}

struct ResidualHotContributor {
    size_t pairIndex = 0;
    double demand = 0.0;
    long long pathParts = 0;
    bool selectedSplit = false;
    bool hasReducingAlt = false;
    double bestRelief = 0.0;
    double bestFullGain = 0.0;
    double bestSplitGain = 0.0;
    std::string parts;
};

struct ResidualHotAxisAudit {
    long long contributorPairs = 0;
    long long contributorPathParts = 0;
    double contributorDemand = 0.0;
    long long splitContributorPairs = 0;
    long long pairsWithReducingAlt = 0;
    long long reducingAltCandidates = 0;
    long long fullImprovingPairs = 0;
    long long splitImprovingPairs = 0;
    double bestReliefSum = 0.0;
    double bestFullGainSum = 0.0;
    double bestSplitGainSum = 0.0;
    std::vector<ResidualHotContributor> contributors;
};

std::string residualHotClass(const ResidualHotAxisAudit& a,
                             const HotChannelAxis& h) {
    if (a.contributorPairs == 0) return "no_contributor_accounting_bug";
    if (a.pairsWithReducingAlt == 0) return "candidate_starvation";
    if (a.bestReliefSum + 1.0e-9 < h.overflow) {
        return a.contributorPairs >= 3 ? "capacity_or_placement_bound"
                                       : "candidate_pool_bound";
    }
    if (a.splitContributorPairs > 0 &&
        a.fullImprovingPairs == 0 &&
        a.splitImprovingPairs == 0) {
        return "split_not_enough";
    }
    if (a.fullImprovingPairs > 0 && a.splitImprovingPairs == 0) {
        return "full_switch_policy_gap";
    }
    if (a.fullImprovingPairs == 0 && a.splitImprovingPairs == 0) {
        return "selector_not_worth";
    }
    return "residual_interaction";
}

ResidualHotAxisAudit auditOneResidualHotAxis(
        const Design& design,
        const std::vector<routerx::DemandedPair>& pairs,
        const std::vector<routerx::LiteCandidate>& baseWinners,
        const std::vector<std::vector<routerx::LiteCandidate>>& candidateSets,
        const std::vector<std::vector<AllocationPart>>& allocations,
        const routerx::RxResourceLedgerLite& finalLedger,
        const HotChannelAxis& h) {
    ResidualHotAxisAudit out;
    const bool lr = (h.axis == "LR");
    const HotAxisRatios finalHot = hotAxisRatios(design, finalLedger, 1.0);

    for (size_t i = 0; i < pairs.size() && i < baseWinners.size() &&
                       i < candidateSets.size(); ++i) {
        const routerx::DemandedPair& p = pairs[i];
        if (p.nets <= 0) continue;
        const std::vector<AllocationPart> currentParts =
            allocationOrBaseParts(i, allocations, baseWinners, pairs);
        const double currentDemand =
            allocationAxisDemand(design, currentParts, lr, h.channelIndex);
        if (currentDemand <= 1.0e-9) continue;

        ResidualHotContributor c;
        c.pairIndex = i;
        c.demand = currentDemand;
        c.pathParts = allocationPathPartsOnAxis(
            design, currentParts, lr, h.channelIndex);
        c.selectedSplit = allocationHasSplit(currentParts);
        c.parts = allocationPartsLabel(currentParts);

        routerx::RxResourceLedgerLite replaceLedger = finalLedger;
        ripupAllocation(design, replaceLedger, currentParts);
        const AllocationEval keepEval =
            evaluateAllocation(design, replaceLedger, currentParts, design.alpha);

        for (const routerx::LiteCandidate& alt : candidateSets[i]) {
            if (!selectableCandidate(alt)) continue;
            const RouteAxisDemand altDemand =
                routeAxisDemand(design, alt.route.steps, p.nets);
            const double altAxisDemand =
                demandOnAxis(altDemand, lr, h.channelIndex);
            const double relief = currentDemand - altAxisDemand;
            if (relief <= 1.0e-9) continue;

            c.hasReducingAlt = true;
            c.bestRelief = std::max(c.bestRelief, relief);
            ++out.reducingAltCandidates;

            const std::vector<AllocationPart> fullParts = {
                AllocationPart{&alt, p.nets}
            };
            const AllocationEval fullEval =
                evaluateAllocation(design, replaceLedger, fullParts, design.alpha);
            if (fullEval.valid && fullEval.key < keepEval.key - 1.0e-6) {
                c.bestFullGain = std::max(c.bestFullGain,
                                          keepEval.key - fullEval.key);
            }

            if (p.nets >= 2) {
                const RouteAxisDemand baseDemand =
                    routeAxisDemand(design, baseWinners[i].route.steps, p.nets);
                std::vector<int> seenMoved;
                for (int pi = 0; pi < kAllocationSplitPolicyCount; ++pi) {
                    const AllocationSplitPolicy policy =
                        static_cast<AllocationSplitPolicy>(pi);
                    if (!allocationSplitPolicySlimSet(policy)) continue;
                    const int moved = allocationSplitMovedNets(
                        design, finalLedger, finalHot, baseDemand, altDemand,
                        p.nets, policy);
                    if (moved <= 0 || moved >= p.nets) continue;
                    if (std::find(seenMoved.begin(), seenMoved.end(), moved) !=
                        seenMoved.end()) {
                        continue;
                    }
                    seenMoved.push_back(moved);
                    const std::vector<AllocationPart> splitParts =
                        splitAllocationParts(baseWinners[i], alt, p.nets, moved);
                    const double splitAxisDemand =
                        allocationAxisDemand(design, splitParts, lr, h.channelIndex);
                    if (splitAxisDemand >= currentDemand - 1.0e-9) continue;
                    const AllocationEval splitEval =
                        evaluateAllocation(design, replaceLedger, splitParts,
                                           design.alpha);
                    if (splitEval.valid && splitEval.key < keepEval.key - 1.0e-6) {
                        c.bestSplitGain = std::max(c.bestSplitGain,
                                                   keepEval.key - splitEval.key);
                    }
                }
            }
        }

        ++out.contributorPairs;
        out.contributorPathParts += c.pathParts;
        out.contributorDemand += c.demand;
        if (c.selectedSplit) ++out.splitContributorPairs;
        if (c.hasReducingAlt) {
            ++out.pairsWithReducingAlt;
            out.bestReliefSum += c.bestRelief;
        }
        if (c.bestFullGain > 1.0e-6) {
            ++out.fullImprovingPairs;
            out.bestFullGainSum += c.bestFullGain;
        }
        if (c.bestSplitGain > 1.0e-6) {
            ++out.splitImprovingPairs;
            out.bestSplitGainSum += c.bestSplitGain;
        }
        out.contributors.push_back(std::move(c));
    }

    std::sort(out.contributors.begin(), out.contributors.end(),
        [](const ResidualHotContributor& a,
           const ResidualHotContributor& b) {
            if (a.demand != b.demand) return a.demand > b.demand;
            return a.pairIndex < b.pairIndex;
        });
    return out;
}

std::string residualContributorString(
        const Design& design,
        const std::vector<routerx::DemandedPair>& pairs,
        const ResidualHotContributor& c) {
    const routerx::DemandedPair& p = pairs[c.pairIndex];
    return connLabel(design, p) +
           ":nets=" + std::to_string(p.nets) +
           ":demand=" + fmt3(c.demand) +
           ":pathParts=" + std::to_string(c.pathParts) +
           ":split=" + std::string(c.selectedSplit ? "yes" : "no") +
           ":reducingAlt=" + std::string(c.hasReducingAlt ? "yes" : "no") +
           ":bestRelief=" + fmt3(c.bestRelief) +
           ":bestFullGain=" + fmt3(c.bestFullGain) +
           ":bestSplitGain=" + fmt3(c.bestSplitGain) +
           ":parts=" + c.parts;
}

void emitResidualOpportunityAudit(
        const std::string& prefix,
        const Design& design,
        const std::vector<routerx::DemandedPair>& pairs,
        const std::vector<routerx::LiteCandidate>& baseWinners,
        const std::vector<std::vector<routerx::LiteCandidate>>& candidateSets,
        const std::vector<std::vector<AllocationPart>>& allocations,
        const routerx::RxResourceLedgerLite& finalLedger) {
    const std::vector<HotChannelAxis> residualAxes =
        hotChannelAxes(design, finalLedger, 1.0);
    emit((prefix + "_scope").c_str(),
         std::string("residual_opportunity_audit_after_r2s2_final"));
    emit((prefix + "_routingMutation").c_str(), std::string("off"));
    emit((prefix + "_fullSwitchSemantics").c_str(),
         std::string("report_only_not_enabled_in_current_r2s2_allocation_policy"));
    emitHotWorld(prefix + "_final", design, finalLedger, 1.0);
    emit((prefix + "_residualHotThreshold").c_str(), 1.0);
    emit((prefix + "_residualHotAxisCount").c_str(),
         static_cast<long long>(residualAxes.size()));

    long long totalContributorPairs = 0;
    long long totalPathParts = 0;
    long long totalPairsWithReducingAlt = 0;
    long long totalFullImprovingPairs = 0;
    long long totalSplitImprovingPairs = 0;
    double totalBestRelief = 0.0;
    double residualOverflowTotal = 0.0;
    std::set<size_t> uniqueContributorPairs;
    for (size_t ai = 0; ai < residualAxes.size(); ++ai) {
        const HotChannelAxis& h = residualAxes[ai];
        residualOverflowTotal += h.overflow;
        const ResidualHotAxisAudit a = auditOneResidualHotAxis(
            design, pairs, baseWinners, candidateSets, allocations,
            finalLedger, h);
        totalContributorPairs += a.contributorPairs;
        totalPathParts += a.contributorPathParts;
        totalPairsWithReducingAlt += a.pairsWithReducingAlt;
        totalFullImprovingPairs += a.fullImprovingPairs;
        totalSplitImprovingPairs += a.splitImprovingPairs;
        totalBestRelief += a.bestReliefSum;
        for (const ResidualHotContributor& c : a.contributors) {
            uniqueContributorPairs.insert(c.pairIndex);
        }

        const std::string pfx = prefix + "_hotAxis" + std::to_string(ai);
        emit((pfx + "Id").c_str(), hotChannelAxisString(h));
        emit((pfx + "Class").c_str(), residualHotClass(a, h));
        emit((pfx + "ContributorPairs").c_str(), a.contributorPairs);
        emit((pfx + "ContributorPathParts").c_str(), a.contributorPathParts);
        emit((pfx + "ContributorDemand").c_str(), a.contributorDemand);
        emit((pfx + "Overflow").c_str(), h.overflow);
        emit((pfx + "PairsWithReducingAlt").c_str(), a.pairsWithReducingAlt);
        emit((pfx + "ReducingAltCandidates").c_str(), a.reducingAltCandidates);
        emit((pfx + "BestReliefSum").c_str(), a.bestReliefSum);
        emit((pfx + "FullImprovingPairs").c_str(), a.fullImprovingPairs);
        emit((pfx + "SplitImprovingPairs").c_str(), a.splitImprovingPairs);
        emit((pfx + "SplitContributorPairs").c_str(), a.splitContributorPairs);
        emit((pfx + "BestFullGainSum").c_str(), a.bestFullGainSum);
        emit((pfx + "BestSplitGainSum").c_str(), a.bestSplitGainSum);
        for (int ti = 0; ti < 5; ++ti) {
            const std::string key = pfx + "TopContributor" + std::to_string(ti);
            emit(key.c_str(), ti < static_cast<int>(a.contributors.size())
                ? residualContributorString(design, pairs, a.contributors[ti])
                : std::string("none"));
        }
    }

    emit((prefix + "_residualHotOverflowTotal").c_str(), residualOverflowTotal);
    emit((prefix + "_residualHotContributorPairMentions").c_str(),
         totalContributorPairs);
    emit((prefix + "_residualHotUniqueContributorPairs").c_str(),
         static_cast<long long>(uniqueContributorPairs.size()));
    emit((prefix + "_residualHotContributorPathParts").c_str(), totalPathParts);
    emit((prefix + "_residualHotPairsWithReducingAlt").c_str(),
         totalPairsWithReducingAlt);
    emit((prefix + "_residualHotFullImprovingPairs").c_str(),
         totalFullImprovingPairs);
    emit((prefix + "_residualHotSplitImprovingPairs").c_str(),
         totalSplitImprovingPairs);
    emit((prefix + "_residualHotBestReliefSum").c_str(), totalBestRelief);
}

SplitPassStats runHalfSplitPass(
        const Design& design,
        const std::vector<routerx::DemandedPair>& pairs,
        const std::vector<SplitPairPlan>& plans,
        routerx::RxResourceLedgerLite& ledger,
        double& totalWire,
        long long baseSelectedUsedFeedthrough) {
    SplitPassStats stats;
    stats.eligiblePairs = static_cast<long long>(plans.size());
    stats.selectedUsedFeedthrough = baseSelectedUsedFeedthrough;
    for (const SplitPairPlan& plan : plans) {
        const routerx::DemandedPair& p = pairs[plan.pairIndex];
        const int moved = plan.movedNets;
        const int stay = p.nets - moved;
        if (moved <= 0 || stay <= 0) {
            ++stats.keepWins;
            continue;
        }

        ledger.ripupSteps(design, plan.current.route.steps, p.nets);
        const std::vector<AllocationPart> keepParts = {
            AllocationPart{&plan.current, p.nets}
        };
        const std::vector<AllocationPart> fullParts = {
            AllocationPart{&plan.alternative, p.nets}
        };
        const std::vector<AllocationPart> splitParts = {
            AllocationPart{&plan.current, stay},
            AllocationPart{&plan.alternative, moved}
        };
        stats.allocationCandidates += 3;

        const AllocationEval keepEval =
            evaluateAllocation(design, ledger, keepParts, design.alpha);
        const AllocationEval fullEval =
            evaluateAllocation(design, ledger, fullParts, design.alpha);
        const AllocationEval splitEval =
            evaluateAllocation(design, ledger, splitParts, design.alpha);

        SplitChoiceKind choice = SplitChoiceKind::Keep;
        AllocationEval best = keepEval;
        if (fullEval.valid &&
            (fullEval.key < best.key - 1.0e-6 ||
             (std::fabs(fullEval.key - best.key) <= 1.0e-6 &&
              plan.alternative.signature < plan.current.signature))) {
            choice = SplitChoiceKind::FullSwitch;
            best = fullEval;
        }
        if (splitEval.valid && splitEval.key < best.key - 1.0e-6) {
            choice = SplitChoiceKind::HalfSplit;
            best = splitEval;
        }
        if (best.key > keepEval.key + 1.0e-6 || !best.valid) {
            choice = SplitChoiceKind::Keep;
            best = keepEval;
        }

        switch (choice) {
            case SplitChoiceKind::Keep:
                ++stats.keepWins;
                commitAllocation(design, ledger, keepParts);
                break;
            case SplitChoiceKind::FullSwitch:
                ++stats.fullSwitchWins;
                ++stats.changedPairs;
                stats.localOfficialGainSum += keepEval.key - best.key;
                stats.localWireDeltaSum += best.wire - keepEval.wire;
                totalWire += best.wire - keepEval.wire;
                if (plan.current.usedFeedthrough != plan.alternative.usedFeedthrough) {
                    stats.selectedUsedFeedthrough += plan.alternative.usedFeedthrough ? 1 : -1;
                }
                commitAllocation(design, ledger, fullParts);
                break;
            case SplitChoiceKind::HalfSplit:
                ++stats.halfSplitWins;
                ++stats.changedPairs;
                stats.localOfficialGainSum += keepEval.key - best.key;
                stats.localWireDeltaSum += best.wire - keepEval.wire;
                totalWire += best.wire - keepEval.wire;
                if (!plan.current.usedFeedthrough && plan.alternative.usedFeedthrough) {
                    ++stats.selectedUsedFeedthrough;
                }
                commitAllocation(design, ledger, splitParts);
                break;
        }
    }
    return stats;
}

DynamicSplitMove bestDynamicSplitMoveForPair(
        const Design& design,
        const std::vector<routerx::DemandedPair>& pairs,
        size_t pairIndex,
        const routerx::LiteCandidate& current,
        const std::vector<routerx::LiteCandidate>& candidates,
        const routerx::RxResourceLedgerLite& ledger,
        const HotAxisRatios& hot) {
    DynamicSplitMove move;
    if (pairIndex >= pairs.size()) return move;
    const routerx::DemandedPair& p = pairs[pairIndex];
    if (p.nets < 2 || current.route.steps.empty() ||
        !selectableCandidate(current)) {
        return move;
    }

    const RouteAxisDemand currentDemand =
        routeAxisDemand(design, current.route.steps, p.nets);
    const double hotDemand = hotAxisDemandTotal(currentDemand, hot);
    if (hotDemand <= 1.0e-9) return move;

    int bestAltIdx = -1;
    double bestUseful = 0.0;
    double bestWireOverhead = 0.0;
    for (size_t j = 0; j < candidates.size(); ++j) {
        const routerx::LiteCandidate& c = candidates[j];
        if (!selectableCandidate(c)) continue;
        if (c.signature == current.signature) continue;
        const RouteAxisDemand altDemand =
            routeAxisDemand(design, c.route.steps, p.nets);
        const double useful = usefulHotReliefTotal(
            design, ledger, hot, currentDemand, altDemand);
        if (useful <= 1.0e-9) continue;
        const double wireOverhead = c.impact.wire - current.impact.wire;
        if (bestAltIdx < 0 || useful > bestUseful + 1.0e-9 ||
            (std::fabs(useful - bestUseful) <= 1.0e-9 &&
             (wireOverhead < bestWireOverhead - 1.0e-9 ||
              (std::fabs(wireOverhead - bestWireOverhead) <= 1.0e-9 &&
               c.signature < candidates[bestAltIdx].signature)))) {
            bestAltIdx = static_cast<int>(j);
            bestUseful = useful;
            bestWireOverhead = wireOverhead;
        }
    }
    if (bestAltIdx < 0) return move;

    const routerx::LiteCandidate& alt = candidates[bestAltIdx];
    const int moved = p.nets / 2;
    const int stay = p.nets - moved;
    if (moved <= 0 || stay <= 0) return move;

    const std::vector<AllocationPart> keepParts = {
        AllocationPart{&current, p.nets}
    };
    const std::vector<AllocationPart> fullParts = {
        AllocationPart{&alt, p.nets}
    };
    const std::vector<AllocationPart> splitParts = {
        AllocationPart{&current, stay},
        AllocationPart{&alt, moved}
    };

    routerx::RxResourceLedgerLite replaceLedger = ledger;
    replaceLedger.ripupSteps(design, current.route.steps, p.nets);
    const AllocationEval keepEval =
        evaluateAllocation(design, replaceLedger, keepParts, design.alpha);
    const AllocationEval fullEval =
        evaluateAllocation(design, replaceLedger, fullParts, design.alpha);
    const AllocationEval splitEval =
        evaluateAllocation(design, replaceLedger, splitParts, design.alpha);

    SplitChoiceKind choice = SplitChoiceKind::Keep;
    AllocationEval best = keepEval;
    if (fullEval.valid &&
        (fullEval.key < best.key - 1.0e-6 ||
         (std::fabs(fullEval.key - best.key) <= 1.0e-6 &&
          alt.signature < current.signature))) {
        choice = SplitChoiceKind::FullSwitch;
        best = fullEval;
    }
    if (splitEval.valid && splitEval.key < best.key - 1.0e-6) {
        choice = SplitChoiceKind::HalfSplit;
        best = splitEval;
    }
    const double gain = keepEval.key - best.key;
    if (choice == SplitChoiceKind::Keep || gain <= 1.0e-6) return move;

    move.found = true;
    move.choice = choice;
    move.keepEval = keepEval;
    move.bestEval = best;
    move.gain = gain;
    move.plan.pairIndex = pairIndex;
    move.plan.current = current;
    move.plan.alternative = alt;
    move.plan.hotDemand = hotDemand;
    move.plan.usefulRelief = bestUseful;
    move.plan.wireOverhead = bestWireOverhead;
    move.plan.movedNets = moved;
    return move;
}

int splitMaxIterations() {
    const char* env = std::getenv("RX_LITE_SPLIT_MAX_ITERS");
    if (!env || std::string(env).empty()) return 20;
    char* end = nullptr;
    const long v = std::strtol(env, &end, 10);
    if (end && *end == '\0' && v >= 0 && v <= 1000) return static_cast<int>(v);
    std::cerr << "[rx_lite] Unknown RX_LITE_SPLIT_MAX_ITERS=" << env
              << " (expected integer 0..1000)\n";
    return -1;
}

SplitPassStats runDynamicHalfSplitPass(
        const Design& design,
        const std::vector<routerx::DemandedPair>& pairs,
        const std::vector<routerx::LiteCandidate>& world2Winners,
        const std::vector<std::vector<routerx::LiteCandidate>>& candidateSets,
        routerx::RxResourceLedgerLite& ledger,
        double& totalWire,
        long long baseSelectedUsedFeedthrough,
        double hotThreshold,
        int maxIterations) {
    SplitPassStats stats;
    stats.selectedUsedFeedthrough = baseSelectedUsedFeedthrough;
    std::vector<routerx::LiteCandidate> current = world2Winners;
    std::vector<bool> consumed(pairs.size(), false);

    for (int iter = 0; iter < maxIterations; ++iter) {
        const HotAxisRatios hot = hotAxisRatios(design, ledger, hotThreshold);
        DynamicSplitMove bestMove;
        long long consideredThisIter = 0;

        for (size_t i = 0; i < pairs.size() && i < current.size() &&
                           i < candidateSets.size(); ++i) {
            if (consumed[i]) continue;
            if (current[i].route.steps.empty()) continue;

            DynamicSplitMove move = bestDynamicSplitMoveForPair(
                design, pairs, i, current[i], candidateSets[i], ledger, hot);
            if (!move.found) continue;

            ++consideredThisIter;
            if (!bestMove.found || move.gain > bestMove.gain + 1.0e-6 ||
                (std::fabs(move.gain - bestMove.gain) <= 1.0e-6 &&
                 move.plan.pairIndex < bestMove.plan.pairIndex)) {
                bestMove = std::move(move);
            }
        }

        stats.eligiblePairs += consideredThisIter;
        stats.allocationCandidates += consideredThisIter * 3;
        if (!bestMove.found) break;

        const size_t i = bestMove.plan.pairIndex;
        const routerx::DemandedPair& p = pairs[i];
        const int moved = bestMove.plan.movedNets;
        const int stay = p.nets - moved;
        ledger.ripupSteps(design, current[i].route.steps, p.nets);

        stats.localOfficialGainSum += bestMove.gain;
        stats.localWireDeltaSum += bestMove.bestEval.wire - bestMove.keepEval.wire;
        totalWire += bestMove.bestEval.wire - bestMove.keepEval.wire;
        ++stats.changedPairs;
        consumed[i] = true;

        if (bestMove.choice == SplitChoiceKind::FullSwitch) {
            ++stats.fullSwitchWins;
            if (current[i].usedFeedthrough != bestMove.plan.alternative.usedFeedthrough) {
                stats.selectedUsedFeedthrough +=
                    bestMove.plan.alternative.usedFeedthrough ? 1 : -1;
            }
            const std::vector<AllocationPart> fullParts = {
                AllocationPart{&bestMove.plan.alternative, p.nets}
            };
            commitAllocation(design, ledger, fullParts);
            current[i] = bestMove.plan.alternative;
            continue;
        }

        ++stats.halfSplitWins;
        if (!current[i].usedFeedthrough && bestMove.plan.alternative.usedFeedthrough) {
            ++stats.selectedUsedFeedthrough;
        }
        const std::vector<AllocationPart> splitParts = {
            AllocationPart{&bestMove.plan.current, stay},
            AllocationPart{&bestMove.plan.alternative, moved}
        };
        commitAllocation(design, ledger, splitParts);
    }

    stats.keepWins = stats.eligiblePairs - stats.fullSwitchWins - stats.halfSplitWins;
    if (stats.keepWins < 0) stats.keepWins = 0;
    return stats;
}

bool avoidHotEligible(const Design& design,
                      const routerx::LiteCandidate& current,
                      const std::vector<routerx::LiteCandidate>& candidates,
                      const HotAxisRatios& hot,
                      AvoidHotEligibilityMode mode) {
    if (routeTouchesHotAxis(design, current.route.steps, hot)) return true;
    if (mode == AvoidHotEligibilityMode::WinnerTouchesHot) return false;
    for (const routerx::LiteCandidate& c : candidates) {
        if (!c.emitValid) continue;
        if (c.signature == current.signature) continue;
        if (routeTouchesHotAxis(design, c.route.steps, hot)) return true;
    }
    return false;
}

struct ActiveSoftUse {
    std::string name;
    double ftRaw = 0.0;
    double baseArea = 0.0;
    double requiredArea = 0.0;
    double deltaArea = 0.0;
};

std::vector<ActiveSoftUse> activeSoftUses(
        const Design& design,
        const routerx::RxResourceLedgerLite& ledger) {
    std::vector<ActiveSoftUse> out;
    const std::vector<rxscore::BlockFt> blocks = ledger.blockFtInputs(design);
    for (int i = 0; i < static_cast<int>(blocks.size()) &&
                    i < static_cast<int>(design.blocks.size()); ++i) {
        if (design.blocks[i].spec.type != BlockType::SOFT) continue;
        if (blocks[i].ftUsedRaw <= 0) continue;
        const double req = rxscore::requiredAreaFt(blocks[i]);
        out.push_back(ActiveSoftUse{
            design.blocks[i].spec.name,
            static_cast<double>(blocks[i].ftUsedRaw),
            blocks[i].baseArea,
            req,
            std::max(0.0, req - blocks[i].baseArea)
        });
    }
    std::sort(out.begin(), out.end(), [](const ActiveSoftUse& a,
                                         const ActiveSoftUse& b) {
        if (a.deltaArea != b.deltaArea) return a.deltaArea > b.deltaArea;
        return a.name < b.name;
    });
    return out;
}

std::string hotChannelAxisString(const HotChannelAxis& h) {
    return h.name + ":" + h.axis +
           ":of=" + fmt3(h.overflow) +
           ":used=" + fmt3(h.used) +
           ":cap=" + fmt3(h.cap) +
           ":util=" + fmt3(h.usageRate) +
           ":hot=" + fmt3(h.hotness);
}

std::string activeSoftString(const ActiveSoftUse& s) {
    return s.name +
           ":raw=" + fmt3(s.ftRaw) +
           ":deltaArea=" + fmt3(s.deltaArea) +
           ":requiredArea=" + fmt3(s.requiredArea) +
           ":baseArea=" + fmt3(s.baseArea);
}

void emitHotWorld(const std::string& prefix,
                  const Design& design,
                  const routerx::RxResourceLedgerLite& ledger,
                  double threshold = 1.0) {
    const std::vector<HotChannelAxis> hotAxes =
        hotChannelAxes(design, ledger, threshold);
    std::set<std::string> hotObjects;
    double hotAxisOverflow = 0.0;
    for (const HotChannelAxis& h : hotAxes) {
        hotObjects.insert(h.name);
        hotAxisOverflow += h.overflow;
    }
    emit((prefix + "HotChannelAxisCount").c_str(),
         static_cast<long long>(hotAxes.size()));
    emit((prefix + "HotChannelObjectCount").c_str(),
         static_cast<long long>(hotObjects.size()));
    emit((prefix + "HotChannelOverflowTotal").c_str(), hotAxisOverflow);
    for (int i = 0; i < 5; ++i) {
        const std::string key = prefix + "HotChannelTop" + std::to_string(i);
        emit(key.c_str(), i < static_cast<int>(hotAxes.size())
            ? hotChannelAxisString(hotAxes[i])
            : std::string("none"));
    }

    const std::vector<ActiveSoftUse> activeSoft = activeSoftUses(design, ledger);
    const rxscore::FtResult ft = ledger.ftResult(design);
    emit((prefix + "ActiveSoftCount").c_str(),
         static_cast<long long>(activeSoft.size()));
    emit((prefix + "FtUsedRawTotal").c_str(), ledger.totalFtUsedRaw());
    emit((prefix + "FtRequiredAreaSum").c_str(), ft.sumRb);
    emit((prefix + "FtExcess").c_str(), ft.areaExcess);
    emit((prefix + "FtPenalty").c_str(), ft.penalty);
    const double outlineArea = design.outlineW * design.outlineH;
    emit((prefix + "FtUsageRate").c_str(),
         outlineArea > 0.0 ? ft.sumRb / outlineArea : 0.0);
    emit((prefix + "FtSlack").c_str(), outlineArea - ft.sumRb);
    emit((prefix + "FtSlackRate").c_str(),
         outlineArea > 0.0 ? (outlineArea - ft.sumRb) / outlineArea : 0.0);
    for (int i = 0; i < 5; ++i) {
        const std::string key = prefix + "ActiveSoftTop" + std::to_string(i);
        emit(key.c_str(), i < static_cast<int>(activeSoft.size())
            ? activeSoftString(activeSoft[i])
            : std::string("none"));
    }
}

std::set<std::string> hotChannelAxisKeySet(
        const Design& design,
        const routerx::RxResourceLedgerLite& ledger,
        double threshold = 1.0) {
    std::set<std::string> out;
    for (const HotChannelAxis& h : hotChannelAxes(design, ledger, threshold)) {
        out.insert(h.key);
    }
    return out;
}

std::set<std::string> activeSoftKeySet(
        const Design& design,
        const routerx::RxResourceLedgerLite& ledger) {
    std::set<std::string> out;
    for (const ActiveSoftUse& s : activeSoftUses(design, ledger)) {
        out.insert(s.name);
    }
    return out;
}

long long intersectionCount(const std::set<std::string>& a,
                            const std::set<std::string>& b) {
    long long n = 0;
    for (const std::string& k : a) {
        if (b.find(k) != b.end()) ++n;
    }
    return n;
}

void emitHotWorldOverlap(const std::string& prefix,
                         const Design& design,
                         const routerx::RxResourceLedgerLite& before,
                         const routerx::RxResourceLedgerLite& after,
                         double threshold = 1.0) {
    const std::set<std::string> beforeHot =
        hotChannelAxisKeySet(design, before, threshold);
    const std::set<std::string> afterHot =
        hotChannelAxisKeySet(design, after, threshold);
    const long long hotOverlap = intersectionCount(beforeHot, afterHot);
    emit((prefix + "HotChannelAxisOverlap").c_str(), hotOverlap);
    emit((prefix + "HotChannelAxisBeforeOnly").c_str(),
         static_cast<long long>(beforeHot.size()) - hotOverlap);
    emit((prefix + "HotChannelAxisAfterOnly").c_str(),
         static_cast<long long>(afterHot.size()) - hotOverlap);

    const std::set<std::string> beforeSoft = activeSoftKeySet(design, before);
    const std::set<std::string> afterSoft = activeSoftKeySet(design, after);
    const long long softOverlap = intersectionCount(beforeSoft, afterSoft);
    emit((prefix + "ActiveSoftOverlap").c_str(), softOverlap);
    emit((prefix + "ActiveSoftBeforeOnly").c_str(),
         static_cast<long long>(beforeSoft.size()) - softOverlap);
    emit((prefix + "ActiveSoftAfterOnly").c_str(),
         static_cast<long long>(afterSoft.size()) - softOverlap);
}

enum class LiteSelectorMode {
    R0Only,
    GuardedFreeWire,
    MinWire,
    MinOfficialImmediate,
    FamilyPriorityThenWire,
};

const char* liteSelectorModeName(LiteSelectorMode m) {
    switch (m) {
        case LiteSelectorMode::R0Only:                 return "r0_only";
        case LiteSelectorMode::GuardedFreeWire:        return "guarded_free_wire";
        case LiteSelectorMode::MinWire:                return "min_wire";
        case LiteSelectorMode::MinOfficialImmediate:   return "min_official_immediate";
        case LiteSelectorMode::FamilyPriorityThenWire: return "family_priority_then_wire";
    }
    return "unknown";
}

bool parseLiteSelectorMode(const std::string& s, LiteSelectorMode& out) {
    if (s == "r0_only")                 { out = LiteSelectorMode::R0Only; return true; }
    if (s == "guarded_free_wire")       { out = LiteSelectorMode::GuardedFreeWire; return true; }
    if (s == "min_wire")                { out = LiteSelectorMode::MinWire; return true; }
    if (s == "min_official_immediate")  { out = LiteSelectorMode::MinOfficialImmediate; return true; }
    if (s == "family_priority_then_wire"){ out = LiteSelectorMode::FamilyPriorityThenWire; return true; }
    return false;
}

bool selectableCandidate(const routerx::LiteCandidate& c) {
    return c.emitValid && c.impact.clean() &&
           c.impact.dIllegalFtPenalty <= 0.0 &&
           c.impact.illegalFtBlocksAfter == 0;
}

double officialImmediateKey(const routerx::LiteCandidate& c, double alpha) {
    return alpha * c.impact.wire +
           c.impact.dChannelPenalty +
           c.impact.dFtPenalty +
           c.impact.dIllegalFtPenalty;
}

bool candidateTieLess(const routerx::LiteCandidate& a,
                      const routerx::LiteCandidate& b) {
    const int fa = routerx::liteFamilyPriority(a.family);
    const int fb = routerx::liteFamilyPriority(b.family);
    if (fa != fb) return fa < fb;
    return a.signature < b.signature;
}

int findR0FallbackCandidate(const std::vector<routerx::LiteCandidate>& candidates) {
    for (size_t i = 0; i < candidates.size(); ++i) {
        const routerx::LiteCandidate& c = candidates[i];
        if (c.isR0Fallback || c.family == routerx::LiteCandidateFamily::R0Fallback) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

int chooseLiteCandidate(const std::vector<routerx::LiteCandidate>& candidates,
                        LiteSelectorMode mode,
                        double r0Wire,
                        double alpha) {
    const int fallback = findR0FallbackCandidate(candidates);
    if (fallback < 0) return -1;
    if (mode == LiteSelectorMode::R0Only) return fallback;

    int best = fallback;
    auto betterWire = [&](int i, int j) {
        const routerx::LiteCandidate& a = candidates[i];
        const routerx::LiteCandidate& b = candidates[j];
        if (a.impact.wire != b.impact.wire) return a.impact.wire < b.impact.wire;
        return candidateTieLess(a, b);
    };

    switch (mode) {
        case LiteSelectorMode::GuardedFreeWire:
            for (size_t i = 0; i < candidates.size(); ++i) {
                const routerx::LiteCandidate& c = candidates[i];
                if (!selectableCandidate(c)) continue;
                if (routerx::classifyAgainstR0(c, r0Wire) !=
                    routerx::LiteSavingClass::WireFreeSaving) continue;
                if (best == fallback ||
                    candidates[best].isR0Fallback ||
                    betterWire(static_cast<int>(i), best)) {
                    best = static_cast<int>(i);
                }
            }
            return best;

        case LiteSelectorMode::MinWire:
            for (size_t i = 0; i < candidates.size(); ++i) {
                if (!selectableCandidate(candidates[i])) continue;
                if (betterWire(static_cast<int>(i), best)) best = static_cast<int>(i);
            }
            return best;

        case LiteSelectorMode::MinOfficialImmediate:
            for (size_t i = 0; i < candidates.size(); ++i) {
                if (!selectableCandidate(candidates[i])) continue;
                const double ki = officialImmediateKey(candidates[i], alpha);
                const double kb = officialImmediateKey(candidates[best], alpha);
                if (ki < kb || (ki == kb && candidateTieLess(candidates[i], candidates[best]))) {
                    best = static_cast<int>(i);
                }
            }
            return best;

        case LiteSelectorMode::FamilyPriorityThenWire:
            // Policy experiment: among legal, penalty-free improvements, prefer
            // earlier non-R0 families before comparing wire. This is intentionally
            // a selector policy, not a ledger rule.
            best = fallback;
            for (size_t i = 0; i < candidates.size(); ++i) {
                const routerx::LiteCandidate& c = candidates[i];
                if (!selectableCandidate(c)) continue;
                if (c.isR0Fallback || c.family == routerx::LiteCandidateFamily::R0Fallback) continue;
                if (routerx::classifyAgainstR0(c, r0Wire) !=
                    routerx::LiteSavingClass::WireFreeSaving) continue;
                if (best == fallback ||
                    routerx::liteFamilyPriority(c.family) <
                        routerx::liteFamilyPriority(candidates[best].family) ||
                    (routerx::liteFamilyPriority(c.family) ==
                        routerx::liteFamilyPriority(candidates[best].family) &&
                     betterWire(static_cast<int>(i), best))) {
                    best = static_cast<int>(i);
                }
            }
            return best;

        case LiteSelectorMode::R0Only:
            return fallback;
    }
    return best;
}

SegmentBypassStats runSegmentBypassPass(
        const Design& design,
        const routerx::RxGraph& graph,
        const std::vector<routerx::DemandedPair>& pairs,
        const std::vector<std::vector<routerx::LiteCandidate>>& candidateSets,
        std::vector<routerx::LiteCandidate>& winners,
        routerx::RxResourceLedgerLite& ledger,
        double& totalWire,
        double hotThreshold,
        const SegmentSearchConfig& config,
        std::array<long long, routerx::kLiteCandidateFamilyCount>& familyWins,
        std::vector<std::vector<routerx::LiteCandidate>>* fixedCandidateSets = nullptr) {
    SegmentBypassStats stats;

    for (size_t i = 0; i < pairs.size() && i < winners.size() &&
                       i < candidateSets.size(); ++i) {
        const routerx::DemandedPair& p = pairs[i];
        routerx::LiteCandidate current = winners[i];
        if (p.nets <= 0 || current.route.steps.empty() || !current.emitValid) {
            ++stats.selectedInvalid;
            continue;
        }

        const HotAxisRatios hot = hotAxisRatios(design, ledger, hotThreshold);
        std::vector<routerx::LiteCandidate> raw;
        raw.reserve(candidateSets[i].size() + 8);
        raw.push_back(current);
        raw.insert(raw.end(), candidateSets[i].begin(), candidateSets[i].end());
        std::map<std::string, SegmentCandidateOrigin> segOrigins;
        std::vector<routerx::LiteCandidate> segRaw =
            generateSegmentBypassCandidates(design, graph, p, current, hot,
                                            config, segOrigins, stats);
        std::set<std::string> segSignatures;
        for (const routerx::LiteCandidate& c : segRaw) {
            segSignatures.insert(c.signature);
        }
        raw.insert(raw.end(), segRaw.begin(), segRaw.end());

        ledger.ripupSteps(design, current.route.steps, p.nets);
        for (routerx::LiteCandidate& c : raw) {
            if (c.emitValid) {
                c.impact = ledger.projectSteps(design, c.route.steps, p.nets);
                if (c.family == routerx::LiteCandidateFamily::SegmentBypassUnified &&
                    !c.impact.clean()) {
                    ++stats.dirtyCandidates;
                }
                if (c.family == routerx::LiteCandidateFamily::SegmentBypassUnified) {
                    if (c.impact.dIllegalFtPenalty > 0.0 ||
                        c.impact.illegalFtBlocksAfter > 0) {
                        ++stats.illegalFtCandidates;
                    }
                }
            }
        }
        routerx::LiteDedupResult dedup = routerx::dedupLiteCandidates(raw);
        for (routerx::LiteCandidate& c : dedup.candidates) {
            if (c.emitValid) c.impact = ledger.projectSteps(design, c.route.steps, p.nets);
        }
        if (fixedCandidateSets && i < fixedCandidateSets->size()) {
            (*fixedCandidateSets)[i] = dedup.candidates;
        }
        for (const routerx::LiteCandidate& c : dedup.candidates) {
            if (segSignatures.find(c.signature) == segSignatures.end()) continue;
            ++stats.selectorVisibleSegmentSignatures;
            if (c.family == routerx::LiteCandidateFamily::SegmentBypassUnified) {
                ++stats.selectorLabeledSegmentCandidates;
            } else {
                ++stats.selectorHiddenSegmentSignatures;
            }
        }

        int currentIdx = -1;
        int bestIdx = -1;
        for (size_t j = 0; j < dedup.candidates.size(); ++j) {
            const routerx::LiteCandidate& c = dedup.candidates[j];
            if (c.signature == current.signature && currentIdx < 0) {
                currentIdx = static_cast<int>(j);
            }
            const bool candidateFromCurrent = (c.signature == current.signature);
            const bool candidateFromSegment =
                (segSignatures.find(c.signature) != segSignatures.end());
            if (!candidateFromCurrent && !candidateFromSegment) continue;
            if (!selectableCandidate(c)) continue;
            if (bestIdx < 0) {
                bestIdx = static_cast<int>(j);
                continue;
            }
            const double kc = officialImmediateKey(c, design.alpha);
            const double kb = officialImmediateKey(dedup.candidates[bestIdx], design.alpha);
            if (kc < kb - 1.0e-9 ||
                (std::fabs(kc - kb) <= 1.0e-9 &&
                 candidateTieLess(c, dedup.candidates[bestIdx]))) {
                bestIdx = static_cast<int>(j);
            }
        }

        if (currentIdx < 0 || bestIdx < 0) {
            ++stats.selectedInvalid;
            ledger.commitSteps(design, current.route.steps, p.nets);
            continue;
        }

        const double currentKey =
            officialImmediateKey(dedup.candidates[currentIdx], design.alpha);
        const double bestKey =
            officialImmediateKey(dedup.candidates[bestIdx], design.alpha);
        if (!(bestKey < currentKey - 1.0e-6)) bestIdx = currentIdx;

        const routerx::LiteCandidate& chosen = dedup.candidates[bestIdx];
        const routerx::LiteCandidate& currentProjected = dedup.candidates[currentIdx];
        if (!selectableCandidate(chosen)) ++stats.selectedInvalid;
        if (chosen.impact.dIllegalFtPenalty > 0.0 ||
            chosen.impact.illegalFtBlocksAfter > 0) {
            ++stats.selectedIllegalFt;
        }
        if (chosen.usedFeedthrough) ++stats.selectedUsedFeedthrough;
        if (chosen.signature != currentProjected.signature) {
            ++stats.changedPairs;
            stats.localOfficialGainSum += currentKey -
                officialImmediateKey(chosen, design.alpha);
            stats.localWireDeltaSum += chosen.impact.wire -
                currentProjected.impact.wire;
            const bool chosenFromSegment =
                segSignatures.find(chosen.signature) != segSignatures.end();
            if (chosenFromSegment) {
                ++stats.selectedWins;
                if (chosen.family != routerx::LiteCandidateFamily::SegmentBypassUnified) {
                    ++stats.selectedHiddenSegment;
                }
                const auto oit = segOrigins.find(chosen.signature);
                if (oit != segOrigins.end()) {
                    const int fidx = oit->second.feature;
                    if (fidx >= 0 && fidx < kSegmentSearchFeatureCount) {
                        ++stats.selectedByFeature[fidx];
                    }
                    if (oit->second.rank >= 1 &&
                        oit->second.rank < static_cast<int>(stats.selectedByRank.size())) {
                        ++stats.selectedByRank[oit->second.rank];
                    }
                }
                const double beforeTarget =
                    hotAxisDemandTotal(routeAxisDemand(design,
                        currentProjected.route.steps, p.nets), hot);
                const double afterTarget =
                    hotAxisDemandTotal(routeAxisDemand(design,
                        chosen.route.steps, p.nets), hot);
                stats.selectedTargetDemandDrop +=
                    std::max(0.0, beforeTarget - afterTarget);
            } else {
                ++stats.changedNonSegment;
            }
        }
        totalWire += chosen.impact.wire - currentProjected.impact.wire;
        ledger.commitSteps(design, chosen.route.steps, p.nets);
        winners[i] = chosen;

        const int fidx = static_cast<int>(chosen.family);
        if (fidx >= 0 && fidx < routerx::kLiteCandidateFamilyCount) {
            ++familyWins[fidx];
        }
    }

    return stats;
}

struct LiteWorldScore {
    double totalWire = 0.0;
    double channelOverflow = 0.0;
    double channelCapacity = 0.0;
    rxscore::FtResult ft;
    rxscore::Breakdown score;
};

LiteWorldScore scoreLiteWorld(const Design& design,
                              const routerx::RxResourceLedgerLite& ledger,
                              double totalWire) {
    LiteWorldScore out;
    out.totalWire = totalWire;
    out.channelOverflow = ledger.totalChannelOverflow();
    out.channelCapacity = ledger.totalChannelCapacity();
    out.ft = ledger.ftResult(design);

    rxscore::Inputs in;
    in.alpha = design.alpha;
    in.outlineArea = design.outlineW * design.outlineH;
    in.totalWireLength = totalWire;
    in.channelOverflowTotal = out.channelOverflow;
    in.channelCapacityTotal = out.channelCapacity;
    in.runtimeSec = 0.0;
    in.blocks = ledger.blockFtInputs(design);
    out.score = rxscore::score(in);
    return out;
}

void emitLiteWorldScore(const std::string& prefix, const LiteWorldScore& s) {
    emit((prefix + "Wire").c_str(), s.totalWire);
    emit((prefix + "PartialCostNoEdgeNoRt").c_str(), s.score.totalNoRuntime);
    emit((prefix + "ChannelOverflowTotal").c_str(), s.channelOverflow);
    emit((prefix + "ChannelOverflowRate").c_str(),
         s.channelCapacity > 0.0 ? s.channelOverflow / s.channelCapacity : 0.0);
    emit((prefix + "ChannelPenalty").c_str(), s.score.channelPenalty);
    emit((prefix + "FtRequiredAreaSum").c_str(), s.ft.sumRb);
    emit((prefix + "FtExcess").c_str(), s.ft.areaExcess);
    emit((prefix + "FtPenalty").c_str(), s.ft.penalty);
}

struct StaticReselectPassStats {
    int pass = 0;
    long long changedPairs = 0;
    long long selectedInvalid = 0;
    long long selectedIllegalFt = 0;
    double officialGain = 0.0;
    double wireGain = 0.0;
    double channelOverflowBefore = 0.0;
    double channelOverflowAfter = 0.0;
    double ftRequiredBefore = 0.0;
    double ftRequiredAfter = 0.0;
};

struct StaticReselectResult {
    std::vector<StaticReselectPassStats> passes;
    long long selectedInvalid = 0;
    long long selectedIllegalFt = 0;
    long long selectedUsedFeedthrough = 0;
    std::array<long long, routerx::kLiteCandidateFamilyCount> familyWins{};
};

StaticReselectResult runStaticReselectFixedPoint(
        const Design& design,
        const std::vector<routerx::DemandedPair>& pairs,
        const std::vector<std::vector<routerx::LiteCandidate>>& candidateSets,
        std::vector<routerx::LiteCandidate>& winners,
        routerx::RxResourceLedgerLite& ledger,
        double& totalWire,
        int maxPasses) {
    StaticReselectResult result;
    if (maxPasses <= 0) return result;

    for (int pass = 1; pass <= maxPasses; ++pass) {
        const LiteWorldScore before = scoreLiteWorld(design, ledger, totalWire);
        StaticReselectPassStats ps;
        ps.pass = pass;
        ps.channelOverflowBefore = before.channelOverflow;
        ps.ftRequiredBefore = before.ft.sumRb;

        for (size_t i = 0; i < pairs.size() && i < winners.size() &&
                           i < candidateSets.size(); ++i) {
            const routerx::DemandedPair& p = pairs[i];
            const routerx::LiteCandidate current = winners[i];
            if (p.nets <= 0 || current.route.steps.empty() || !current.emitValid) {
                ++ps.selectedInvalid;
                continue;
            }

            ledger.ripupSteps(design, current.route.steps, p.nets);
            std::vector<routerx::LiteCandidate> candidates = candidateSets[i];
            bool hasCurrent = false;
            for (const routerx::LiteCandidate& c : candidates) {
                if (c.signature == current.signature) {
                    hasCurrent = true;
                    break;
                }
            }
            if (!hasCurrent) candidates.push_back(current);

            int currentIdx = -1;
            int bestIdx = -1;
            for (size_t j = 0; j < candidates.size(); ++j) {
                routerx::LiteCandidate& c = candidates[j];
                if (c.emitValid) {
                    c.impact = ledger.projectSteps(design, c.route.steps, p.nets);
                }
                if (c.signature == current.signature && currentIdx < 0) {
                    currentIdx = static_cast<int>(j);
                }
                if (!selectableCandidate(c)) continue;
                if (bestIdx < 0) {
                    bestIdx = static_cast<int>(j);
                    continue;
                }
                const double kc = officialImmediateKey(c, design.alpha);
                const double kb = officialImmediateKey(candidates[bestIdx], design.alpha);
                if (kc < kb - 1.0e-9 ||
                    (std::fabs(kc - kb) <= 1.0e-9 &&
                     candidateTieLess(c, candidates[bestIdx]))) {
                    bestIdx = static_cast<int>(j);
                }
            }

            if (currentIdx < 0 || bestIdx < 0) {
                ++ps.selectedInvalid;
                ledger.commitSteps(design, current.route.steps, p.nets);
                continue;
            }

            const double currentKey =
                officialImmediateKey(candidates[currentIdx], design.alpha);
            const double bestKey =
                officialImmediateKey(candidates[bestIdx], design.alpha);
            // Reselect is a coordinate-descent refinement: only strict
            // official-cost improvements may change the current assignment.
            // Equal-cost tie flips make changedPairs noisy and weaken the
            // finite-state convergence argument.
            if (!(bestKey < currentKey - 1.0e-6)) bestIdx = currentIdx;

            const routerx::LiteCandidate& chosen = candidates[bestIdx];
            const routerx::LiteCandidate& currentProjected = candidates[currentIdx];
            if (!selectableCandidate(chosen)) ++ps.selectedInvalid;
            if (chosen.impact.dIllegalFtPenalty > 0.0 ||
                chosen.impact.illegalFtBlocksAfter > 0) {
                ++ps.selectedIllegalFt;
            }
            if (chosen.signature != currentProjected.signature) {
                ++ps.changedPairs;
            }
            totalWire += chosen.impact.wire - currentProjected.impact.wire;
            ledger.commitSteps(design, chosen.route.steps, p.nets);
            winners[i] = chosen;
        }

        const LiteWorldScore after = scoreLiteWorld(design, ledger, totalWire);
        ps.channelOverflowAfter = after.channelOverflow;
        ps.ftRequiredAfter = after.ft.sumRb;
        ps.officialGain = before.score.totalNoRuntime - after.score.totalNoRuntime;
        ps.wireGain = before.totalWire - after.totalWire;
        result.passes.push_back(ps);
        if (ps.changedPairs == 0) break;
    }

    for (const routerx::LiteCandidate& c : winners) {
        if (c.route.steps.empty() || !c.emitValid) {
            ++result.selectedInvalid;
            continue;
        }
        if (!selectableCandidate(c)) ++result.selectedInvalid;
        if (c.impact.dIllegalFtPenalty > 0.0 ||
            c.impact.illegalFtBlocksAfter > 0) {
            ++result.selectedIllegalFt;
        }
        if (c.usedFeedthrough) ++result.selectedUsedFeedthrough;
        const int fidx = static_cast<int>(c.family);
        if (fidx >= 0 && fidx < routerx::kLiteCandidateFamilyCount) {
            ++result.familyWins[fidx];
        }
    }
    for (const StaticReselectPassStats& ps : result.passes) {
        result.selectedInvalid += ps.selectedInvalid;
        result.selectedIllegalFt += ps.selectedIllegalFt;
    }
    return result;
}

// --- mode 2: declared-channel .cfg reader (OUTLINE + CHANNEL only) ----------
// A tiny local parser, NOT a route-reader: PlacementReader deliberately drops the
// CHANNEL section, but the official checker computes capacity from the DECLARED
// channels, so calibration must read them here rather than rebuild via
// ChannelBuilder (which would produce a different, non-comparable set).
#if 0 // Standalone rx_lite CLI/reporting modes are not part of the embedded adapter.
struct DeclaredCfg {
    bool   ok           = false;
    double outlineW     = 0.0;
    double outlineH     = 0.0;
    int    channelCount = 0;
    double capacityTotal = 0.0;   // sum over channels of 25*h (LR) + 25*w (TB)
};

DeclaredCfg readDeclaredCfg(const std::string& path) {
    DeclaredCfg d;
    std::ifstream in(path);
    if (!in) { std::cerr << "[rx_lite] cannot open " << path << "\n"; return d; }
    std::string line;
    bool haveOutline = false;
    while (std::getline(in, line)) {
        std::istringstream ss(line);
        std::string tok;
        if (!(ss >> tok)) continue;
        if (tok == "Outline") {
            if (ss >> d.outlineW >> d.outlineH) haveOutline = true;
        } else if (tok == "CHANNEL") {
            // "CHANNEL <name> <x> <y> <w> <h>". The bare "CHANNEL" section header
            // has no further tokens, so the 5-field read below simply fails and
            // the header line is skipped.
            std::string name; double x = 0, y = 0, w = 0, h = 0;
            if (ss >> name >> x >> y >> w >> h) {
                d.channelCount += 1;
                d.capacityTotal += rxscore::kChannelDensity * (h + w);
            }
        }
    }
    if (!haveOutline) {
        std::cerr << "[rx_lite] --declared: no Outline line in " << path << "\n";
        return d;
    }
    if (d.channelCount == 0) {
        // Calibration mode, NOT an outline-only parser: a .cfg with no declared
        // CHANNEL section (e.g. a placement-only corpus_q43 .cfg) cannot be
        // calibrated against channel_total_capacity, so fail rather than silently
        // report capacity 0.
        std::cerr << "[rx_lite] --declared: no CHANNEL section in " << path
                  << " (use a submission .cfg with declared channels, not a"
                     " placement-only corpus_q43 .cfg)\n";
        return d;
    }
    d.ok = true;
    return d;
}

int runDeclared(const std::string& cfgPath) {
    DeclaredCfg d = readDeclaredCfg(cfgPath);
    if (!d.ok) return 1;
    emit("lite_mode", std::string("declared"));
    emit("lite_channelSource", std::string("declared"));
    emit("lite_cfg", cfgPath);
    emit("lite_outlineArea", d.outlineW * d.outlineH);
    emit("lite_channelCount", static_cast<long long>(d.channelCount));
    emit("lite_channelCapacityTotal", d.capacityTotal);
    return 0;
}

bool readDeclaredChannelsAndRoutes(const std::string& cfgPath, Design& design) {
    std::ifstream in(cfgPath);
    if (!in) {
        std::cerr << "[rx_lite] --calib: cannot open " << cfgPath << "\n";
        return false;
    }
    design.channels.clear();
    design.routes.clear();

    std::string line;
    while (std::getline(in, line)) {
        std::istringstream ss(line);
        std::string tok;
        if (!(ss >> tok)) continue;

        if (tok == "CHANNEL") {
            std::string name;
            double x = 0.0, y = 0.0, w = 0.0, h = 0.0;
            if (!(ss >> name >> x >> y >> w >> h)) continue; // section header
            Channel ch;
            ch.name = name;
            ch.rect = Rect{x, y, w, h};
            ch.lrCapacity = std::max(0.0, h) * rxscore::kChannelDensity;
            ch.tbCapacity = std::max(0.0, w) * rxscore::kChannelDensity;
            design.channels.push_back(ch);
        } else if (tok == "PATH") {
            int nets = 0;
            if (!(ss >> nets)) continue; // section header
            RoutePath r;
            r.netCount = nets;
            std::string obj;
            int edge = 0;
            while (ss >> obj >> edge) r.steps.push_back(RouteStep{obj, edge});
            if (r.steps.size() < 2) {
                std::cerr << "[rx_lite] --calib: malformed PATH with fewer than 2 steps\n";
                return false;
            }
            r.srcBlock = r.steps.front().rectName;
            r.dstBlock = r.steps.back().rectName;
            design.routes.push_back(r);
        }
    }

    if (design.channels.empty()) {
        std::cerr << "[rx_lite] --calib: no declared CHANNEL entries in " << cfgPath << "\n";
        return false;
    }
    if (design.routes.empty()) {
        std::cerr << "[rx_lite] --calib: no declared PATH entries in " << cfgPath << "\n";
        return false;
    }
    return true;
}

int runCalib(const std::string& csvPath, const std::string& cfgPath) {
    ParsedPlacement parsed = loadDeclaredDesignShell(csvPath, cfgPath);
    if (!parsed.ok) return 1;
    Design& design = parsed.design;
    if (!readDeclaredChannelsAndRoutes(cfgPath, design)) return 1;

    const routerx::RouteLedgerBreakdown b = routerx::RxRouteLedger::replay(design);

    emit("lite_mode", std::string("calib"));
    emit("lite_channelSource", std::string("declared"));
    emit("lite_csv", csvPath);
    emit("lite_cfg", cfgPath);
    emit("calib_outlineArea", design.outlineW * design.outlineH);
    emit("calib_blockCount", static_cast<long long>(design.blocks.size()));
    emit("calib_channelCount", static_cast<long long>(design.channels.size()));
    emit("calib_routeCount", static_cast<long long>(b.routeCount));
    emit("calib_stepCount", static_cast<long long>(b.stepCount));
    emit("calib_totalWireLength", b.totalWireLength);
    emit("calib_channelCapacityTotal", b.channelCapacityTotal);
    emit("calib_channelOverflowTotal", b.channelOverflowTotal);
    emit("calib_ftUsedRawTotal", b.ftUsedRawTotal);
    emit("calib_ftRequiredAreaSum", b.ft.sumRb);
    emit("calib_ftExcess", b.ft.areaExcess);
    emit("calib_ftPenalty", b.ft.penalty);
    emit("calib_illegalFtBlocks", static_cast<long long>(b.ft.illegalBlocks));
    emit("calib_illegalFtPenalty", b.ft.illegalPenalty);
    emit("calib_contactFailedRoutes", static_cast<long long>(b.contactFailedRoutes));
    emit("calib_unknownTraversalObjects", static_cast<long long>(b.unknownTraversalObjects));
    emit("calib_sameObjectSameEdgeTraversals", static_cast<long long>(b.sameObjectSameEdgeTraversals));
    for (size_t i = 0; i < b.routeWireLengths.size(); ++i) {
        const std::string prefix = "calib_route." + std::to_string(i);
        emit((prefix + ".wireLength").c_str(), b.routeWireLengths[i]);
        const int contact = i < b.routeContactFailed.size() ? b.routeContactFailed[i] : 1;
        emit((prefix + ".contactFailed").c_str(), static_cast<long long>(contact));
    }
    return (b.contactFailedRoutes == 0 &&
            b.unknownTraversalObjects == 0 &&
            b.sameObjectSameEdgeTraversals == 0) ? 0 : 2;
}

// --- mode 1: q43 normal path ------------------------------------------------
int runQ43(const std::string& csvPath, const std::string& cfgPath) {
    routerx::SoftRoutingPolicy routeSpace;
    if (!parseLiteRouteSpace(routeSpace)) return 1;

    ParsedPlacement parsed = loadQ43Design(csvPath, cfgPath);
    if (!parsed.ok) return 1;
    Design& design = parsed.design;

    routerx::RxGraph graph;
    graph.build(design, routeSpace);

    const std::vector<routerx::DemandedPair> pairs = routerx::demandedPairs(design);

    long long totalDemandNets = 0;
    for (const routerx::DemandedPair& p : pairs) totalDemandNets += p.nets;

    emit("lite_mode", std::string("q43"));
    emit("lite_channelSource", std::string("built"));
    emit("lite_csv", csvPath);
    emit("lite_cfg", cfgPath);
    emit("lite_alpha", design.alpha);
    emit("lite_outlineArea", design.outlineW * design.outlineH);
    emit("lite_blockCount", static_cast<long long>(design.blocks.size()));
    emit("lite_channelCount", static_cast<long long>(design.channels.size()));
    emit("lite_demandedPairCount", static_cast<long long>(pairs.size()));
    emit("lite_totalDemandedNets", totalDemandNets);
    emit("lite_channelCapacityTotal", channelCapacityTotal(design));
    emit("lite_blockBaseAreaSum", blockBaseAreaSum(design, false));
    emit("lite_softBaseAreaSum", blockBaseAreaSum(design, true));
    emit("lite_graphNodeCount", static_cast<long long>(graph.nodes().size()));
    emit("lite_graphEdgeCount", static_cast<long long>(graph.edges().size()));
    emit("lite_graphAccessCount", static_cast<long long>(graph.accesses().size()));
    emitLiteGraphReport(graph);
    return 0;
}

// --- mode 3: R0 shortest-legal routing -------------------------------------
int runR0(const std::string& csvPath, const std::string& cfgPath) {
    routerx::SearchCostMode searchCostMode;
    if (!parseLiteSearchCostMode(searchCostMode)) return 1;
    routerx::SoftRoutingPolicy routeSpace;
    if (!parseLiteRouteSpace(routeSpace)) return 1;

    ParsedPlacement parsed = loadQ43Design(csvPath, cfgPath);
    if (!parsed.ok) return 1;
    Design& design = parsed.design;

    routerx::RxGraph graph;
    graph.build(design, routeSpace);

    routerx::RxResourceLedgerLite ledger;
    ledger.build(design);

    const std::vector<routerx::DemandedPair> pairs = routerx::demandedPairs(design);

    long long totalDemandNets = 0;
    long long routedPairs = 0;
    long long openPairs = 0;
    long long shortageNets = 0;
    long long directRoutes = 0;
    long long channelOnlyRoutes = 0;
    long long ftRoutes = 0;
    long long invalidEmit = 0;
    long long searchCalls = 0;
    long long r1ContactFailedRoutes = 0;
    double r1TotalWireLength = 0.0;

    for (const routerx::DemandedPair& p : pairs) {
        totalDemandNets += p.nets;
        LiteRoutedPair r0 = routeR0Fallback(design, graph, p, searchCostMode);
        searchCalls += r0.searchCalls;
        if (!r0.found) {
            ++openPairs;
            shortageNets += p.nets;
            continue;
        }

        routerx::EmitReject reject = routerx::EmitReject::None;
        if (!routerx::RxEmit::emitPath(r0.route.steps, p.nets,
                                       r0.route.srcBlock, r0.route.dstBlock,
                                       design, reject)) {
            ++invalidEmit;
            ++openPairs;
            shortageNets += p.nets;
            continue;
        }

        const routerx::LiteRouteImpact impact =
            ledger.projectSteps(design, r0.route.steps, p.nets);
        r1TotalWireLength += impact.wire;
        if (impact.contactFailed) ++r1ContactFailedRoutes;
        ledger.commitSteps(design, r0.route.steps, p.nets);
        ++routedPairs;
        if (r0.direct) ++directRoutes;
        else if (r0.usedFeedthrough) ++ftRoutes;
        else ++channelOnlyRoutes;
    }

    const double outlineArea = design.outlineW * design.outlineH;
    const double chOverflow = ledger.totalChannelOverflow();
    const double chCapacity = ledger.totalChannelCapacity();
    const double builtCapacity = channelCapacityTotal(design);
    const rxscore::FtResult ft = ledger.ftResult(design);
    rxscore::Inputs scoreIn;
    scoreIn.alpha = design.alpha;
    scoreIn.outlineArea = outlineArea;
    scoreIn.totalWireLength = r1TotalWireLength;
    scoreIn.channelOverflowTotal = chOverflow;
    scoreIn.channelCapacityTotal = chCapacity;
    scoreIn.runtimeSec = 0.0;
    scoreIn.blocks = ledger.blockFtInputs(design);
    const rxscore::Breakdown score = rxscore::score(scoreIn);

    emit("lite_mode", std::string("r0"));
    emit("lite_searchCostMode", std::string(liteSearchCostModeName(searchCostMode)));
    emit("lite_channelSource", std::string("built"));
    emit("lite_csv", csvPath);
    emit("lite_cfg", cfgPath);
    emit("lite_alpha", design.alpha);
    emit("lite_outlineArea", outlineArea);
    emit("lite_blockCount", static_cast<long long>(design.blocks.size()));
    emit("lite_channelCount", static_cast<long long>(design.channels.size()));
    emit("lite_channelCapacityTotal", builtCapacity);
    emit("lite_demandedPairCount", static_cast<long long>(pairs.size()));
    emit("lite_totalDemandedNets", totalDemandNets);
    emit("lite_graphNodeCount", static_cast<long long>(graph.nodes().size()));
    emit("lite_graphEdgeCount", static_cast<long long>(graph.edges().size()));
    emit("lite_graphAccessCount", static_cast<long long>(graph.accesses().size()));
    emitLiteGraphReport(graph);
    emit("r0_softEndpointAccess", std::string(graph.softEndpointAccessEnabled() ? "on" : "off"));
    emit("r0_softSoftEdges", std::string(graph.softSoftEdgesEnabled() ? "on" : "off"));
    emit("r0_softEndpointAccessCount", static_cast<long long>(graph.softEndpointAccessCount()));
    emit("r0_softSoftEdgeCount", static_cast<long long>(graph.softSoftEdgeCount()));
    emit("r0_pairs", static_cast<long long>(pairs.size()));
    emit("r0_routedPairs", routedPairs);
    emit("r0_openPairs", openPairs);
    emit("r0_shortage", shortageNets);
    emit("r0_routes", static_cast<long long>(design.routes.size()));
    emit("r0_directRoutes", directRoutes);
    emit("r0_channelOnlyRoutes", channelOnlyRoutes);
    emit("r0_ftRoutes", ftRoutes);
    emit("r0_invalidEmit", invalidEmit);
    emit("r0_searchCalls", searchCalls);
    emit("r0_channelCapacityTotal", chCapacity);
    emit("r0_channelCapacityDiff", chCapacity - builtCapacity);
    emit("r0_channelOverflowTotal", chOverflow);
    emit("r0_channelOverflowRate", chCapacity > 0.0 ? chOverflow / chCapacity : 0.0);
    emit("r0_channelPenalty", rxscore::channelOverflowPenalty(chOverflow, chCapacity, outlineArea));
    emit("r0_ftUsedRawTotal", ledger.totalFtUsedRaw());
    emit("r0_ftRequiredAreaSum", ft.sumRb);
    emit("r0_ftExcess", ft.areaExcess);
    emit("r0_ftPenalty", ft.penalty);
    emit("r0_illegalFtBlocks", static_cast<long long>(ft.illegalBlocks));
    emit("r0_illegalFtPenalty", ft.illegalPenalty);
    emit("r1_calibrated", std::string("no"));
    emit("r1_calibrationSource", std::string("none"));
    emit("r1_scoreComplete", std::string("no"));
    emit("r1_missingCostTerms", std::string("edge,runtime"));
    emit("r1_totalWireLength", r1TotalWireLength);
    emit("r1_contactFailedRoutes", r1ContactFailedRoutes);
    emit("r1_scoreLegal", std::string(score.legal ? "yes" : "no"));
    emit("r1_scoreFirstError", score.firstError.empty() ? std::string("none") : score.firstError);
    emit("r1_outlineArea", score.outlineArea);
    emit("r1_wirePenalty", score.wirePenalty);
    emit("r1_channelPenalty", score.channelPenalty);
    emit("r1_channelRate", score.channelRate);
    emit("r1_ftRequiredAreaSum", score.ftRequiredAreaSum);
    emit("r1_ftExcess", score.ftAreaExcess);
    emit("r1_ftPenalty", score.ftPenalty);
    emit("r1_illegalFtBlocks", static_cast<long long>(score.illegalFtBlocks));
    emit("r1_illegalFtPenalty", score.illegalFtPenalty);
    emit("r1_runtimePenalty", score.runtimePenaltyVal);
    emit("r1_partialCostNoEdgeNoRt", score.totalNoRuntime);
    return (openPairs == 0 && shortageNets == 0 && invalidEmit == 0) ? 0 : 2;
}

int runR2b1(const std::string& csvPath, const std::string& cfgPath) {
    routerx::SearchCostMode searchCostMode;
    if (!parseLiteSearchCostMode(searchCostMode)) return 1;
    routerx::SoftRoutingPolicy routeSpace;
    if (!parseLiteRouteSpace(routeSpace)) return 1;
    AccessAltUniverseMode accessMode;
    if (!parseAccessAltUniverseMode(accessMode)) return 1;
    CapacityAwareConfig capacityConfig;
    if (!parseCapacityAwareConfig(capacityConfig)) return 1;

    ParsedPlacement parsed = loadQ43Design(csvPath, cfgPath);
    if (!parsed.ok) return 1;
    Design& design = parsed.design;

    routerx::RxGraph graph;
    graph.build(design, routeSpace);

    const std::vector<routerx::DemandedPair> pairs = routerx::demandedPairs(design);
    std::vector<LiteRoutedPair> r0Routes;
    r0Routes.reserve(pairs.size());

    routerx::RxResourceLedgerLite r0Ledger;
    r0Ledger.build(design);

    long long r0SearchCalls = 0;
    long long r0OpenPairs = 0;
    long long r0InvalidEmit = 0;
    long long totalDemandNets = 0;
    double r0TotalWire = 0.0;

    // Build the full R0 world first. Candidate-surface projection then removes
    // one pair at a time so replacement deltas are measured against a fully
    // occupied design, not an empty ledger.
    for (const routerx::DemandedPair& p : pairs) {
        totalDemandNets += p.nets;
        LiteRoutedPair r0 = routeR0Fallback(design, graph, p, searchCostMode);
        r0SearchCalls += r0.searchCalls;
        if (!r0.found || !r0.emitValid) {
            if (!r0.found) ++r0OpenPairs;
            if (!r0.emitValid) ++r0InvalidEmit;
            r0Routes.push_back(std::move(r0));
            continue;
        }

        const routerx::LiteRouteImpact impact =
            r0Ledger.projectSteps(design, r0.route.steps, p.nets);
        r0TotalWire += impact.wire;
        r0Ledger.commitSteps(design, r0.route.steps, p.nets);
        r0Routes.push_back(std::move(r0));
    }

    const double outlineArea = design.outlineW * design.outlineH;
    const double r0ChOverflow = r0Ledger.totalChannelOverflow();
    const double r0ChCapacity = r0Ledger.totalChannelCapacity();
    rxscore::Inputs scoreIn;
    scoreIn.alpha = design.alpha;
    scoreIn.outlineArea = outlineArea;
    scoreIn.totalWireLength = r0TotalWire;
    scoreIn.channelOverflowTotal = r0ChOverflow;
    scoreIn.channelCapacityTotal = r0ChCapacity;
    scoreIn.runtimeSec = 0.0;
    scoreIn.blocks = r0Ledger.blockFtInputs(design);
    const rxscore::Breakdown score = rxscore::score(scoreIn);

    long long candidateSearchCalls = 0;
    long long altAccessSearchCalls = 0;
    long long rawCandidates = 0;
    long long distinctCandidates = 0;
    long long duplicateCandidates = 0;
    long long missingR0Fallback = 0;
    long long invalidEmitCandidates = 0;
    long long illegalFtCandidates = 0;
    long long usedFeedthroughCandidates = 0;
    long long freeSavingPairs = 0;
    long long riskSavingPairs = 0;
    long long maxRawCandidatesForPair = 0;
    long long maxDistinctCandidatesForPair = 0;
    long long accessAltUnifiedLegalViews = 0;
    long long accessAltUnifiedUniverseViews = 0;
    long long accessAltUnifiedPairsWithUniverse = 0;
    long long accessAltUnifiedMaxUniverseForPair = 0;
    long long accessAltUnifiedBaseViewsRemoved = 0;
    std::array<long long, routerx::kLiteCandidateFamilyCount> familyRaw{};
    std::array<long long, routerx::kLiteCandidateFamilyCount> familyDistinct{};
    double bestFreeWireSaving = 0.0;
    double bestRiskWireSaving = 0.0;
    std::vector<double> freeSavingByPair;
    freeSavingByPair.reserve(pairs.size());

    for (size_t i = 0; i < pairs.size(); ++i) {
        const routerx::DemandedPair& p = pairs[i];
        const LiteRoutedPair& r0 = r0Routes[i];
        if (!r0.found || !r0.emitValid) {
            ++missingR0Fallback;
            freeSavingByPair.push_back(0.0);
            continue;
        }

        r0Ledger.ripupSteps(design, r0.route.steps, p.nets);
        std::vector<routerx::LiteCandidate> raw =
            generateR2b1Candidates(design, graph, p, r0, searchCostMode, accessMode,
                                   capacityConfig,
                                   candidateSearchCalls, altAccessSearchCalls);
        const long long legalViews = accessAltUnifiedLegalViewCount(graph, p);
        bool hasBaseUnified = false;
        for (const routerx::LiteCandidate& c : raw) {
            if (c.family == routerx::LiteCandidateFamily::BaseUnifiedShortest &&
                c.emitValid) {
                hasBaseUnified = true;
                break;
            }
        }
        const long long baseViewsRemoved =
            (hasBaseUnified && legalViews > 0) ? 1 : 0;
        const long long universeViews = legalViews - baseViewsRemoved;
        accessAltUnifiedLegalViews += legalViews;
        accessAltUnifiedUniverseViews += universeViews;
        accessAltUnifiedBaseViewsRemoved += baseViewsRemoved;
        if (universeViews > 0) ++accessAltUnifiedPairsWithUniverse;
        accessAltUnifiedMaxUniverseForPair = std::max(
            accessAltUnifiedMaxUniverseForPair, universeViews);
        const double r0Wire =
            r0Ledger.projectSteps(design, r0.route.steps, p.nets).wire;
        for (routerx::LiteCandidate& c : raw) {
            if (c.emitValid) {
                c.impact = r0Ledger.projectSteps(design, c.route.steps, p.nets);
            }
        }
        r0Ledger.commitSteps(design, r0.route.steps, p.nets);

        const routerx::LitePairCandidateReport report =
            routerx::summarizeLiteCandidateSurface(raw, r0Wire);

        rawCandidates += report.rawCandidates;
        distinctCandidates += report.distinctCandidates;
        duplicateCandidates += report.duplicateCandidates;
        missingR0Fallback += report.missingR0Fallback;
        invalidEmitCandidates += report.invalidEmitCandidates;
        illegalFtCandidates += report.illegalFtCandidates;
        usedFeedthroughCandidates += report.usedFeedthroughCandidates;
        maxRawCandidatesForPair = std::max<long long>(maxRawCandidatesForPair,
                                                      report.rawCandidates);
        maxDistinctCandidatesForPair = std::max<long long>(
            maxDistinctCandidatesForPair, report.distinctCandidates);
        for (int f = 0; f < routerx::kLiteCandidateFamilyCount; ++f) {
            familyRaw[f] += report.familyRaw[f];
            familyDistinct[f] += report.familyDistinct[f];
        }
        if (report.wireFreeSavingBest > 0.0) {
            ++freeSavingPairs;
            bestFreeWireSaving += report.wireFreeSavingBest;
        }
        if (report.wireRiskSavingBest > 0.0) {
            ++riskSavingPairs;
            bestRiskWireSaving += report.wireRiskSavingBest;
        }
        freeSavingByPair.push_back(report.wireFreeSavingBest);
    }

    std::sort(freeSavingByPair.begin(), freeSavingByPair.end(), std::greater<double>());
    const double top1Free = freeSavingByPair.empty() ? 0.0 : freeSavingByPair.front();
    double top5Free = 0.0;
    for (size_t i = 0; i < freeSavingByPair.size() && i < 5; ++i) {
        top5Free += freeSavingByPair[i];
    }
    const double routingMateriality =
        r0TotalWire > 0.0 ? bestFreeWireSaving / r0TotalWire : 0.0;
    const double scoreMateriality =
        score.totalNoRuntime > 0.0 ? design.alpha * bestFreeWireSaving / score.totalNoRuntime
                                   : 0.0;
    const double top1Share =
        bestFreeWireSaving > 0.0 ? top1Free / bestFreeWireSaving : 0.0;
    const double top5Share =
        bestFreeWireSaving > 0.0 ? top5Free / bestFreeWireSaving : 0.0;
    const bool gateOk = (r0OpenPairs == 0 && r0InvalidEmit == 0 &&
                         missingR0Fallback == 0 && invalidEmitCandidates == 0 &&
                         illegalFtCandidates == 0);
    const bool material = routingMateriality >= 0.005;

    emit("lite_mode", std::string("r2b1"));
    emit("lite_searchCostMode", std::string(liteSearchCostModeName(searchCostMode)));
    emit("r2b1_accessAltUniverseMode",
         std::string(accessAltUniverseModeName(accessMode)));
    emitCapacityAwareConfig(capacityConfig);
    emit("lite_csv", csvPath);
    emit("lite_cfg", cfgPath);
    emit("lite_alpha", design.alpha);
    emit("lite_outlineArea", outlineArea);
    emit("lite_blockCount", static_cast<long long>(design.blocks.size()));
    emit("lite_channelCount", static_cast<long long>(design.channels.size()));
    emit("lite_demandedPairCount", static_cast<long long>(pairs.size()));
    emit("lite_totalDemandedNets", totalDemandNets);
    emitLiteGraphReport(graph);
    emit("r2b1_projectionSemantics", std::string("replace_r0_pair"));
    emit("r2b1_materialityDenominator", std::string("r0_total_wire"));
    emit("r2b1_r0OpenPairs", r0OpenPairs);
    emit("r2b1_r0InvalidEmit", r0InvalidEmit);
    emit("r2b1_r0SearchCalls", r0SearchCalls);
    emit("r2b1_candidateSearchCalls", candidateSearchCalls);
    emit("r2b1_altAccessSearchCalls", altAccessSearchCalls);
    emit("r2b1_rawCandidates", rawCandidates);
    emit("r2b1_distinctCandidates", distinctCandidates);
    emit("r2b1_duplicateCandidates", duplicateCandidates);
    emit("r2b1_maxRawCandidatesForPair", maxRawCandidatesForPair);
    emit("r2b1_maxDistinctCandidatesForPair", maxDistinctCandidatesForPair);
    emit("r2b1_accessAltUnifiedLegalViews", accessAltUnifiedLegalViews);
    emit("r2b1_accessAltUnifiedUniverseViews", accessAltUnifiedUniverseViews);
    emit("r2b1_accessAltUnifiedPairsWithUniverse",
         accessAltUnifiedPairsWithUniverse);
    emit("r2b1_accessAltUnifiedMaxUniverseForPair",
         accessAltUnifiedMaxUniverseForPair);
    emit("r2b1_accessAltUnifiedBaseViewsRemoved",
         accessAltUnifiedBaseViewsRemoved);
    emit("r2b1_missingR0Fallback", missingR0Fallback);
    emit("r2b1_invalidEmitCandidates", invalidEmitCandidates);
    emit("r2b1_illegalFtCandidates", illegalFtCandidates);
    emit("r2b1_usedFeedthroughCandidates", usedFeedthroughCandidates);
    emit("r2b1_freeSavingPairs", freeSavingPairs);
    emit("r2b1_bestFreeWireSaving", bestFreeWireSaving);
    emit("r2b1_routingMateriality", routingMateriality);
    emit("r2b1_scoreMateriality", scoreMateriality);
    emit("r2b1_top1FreeSavingShare", top1Share);
    emit("r2b1_top5FreeSavingShare", top5Share);
    emit("r2b1_riskSavingPairs", riskSavingPairs);
    emit("r2b1_bestRiskWireSaving", bestRiskWireSaving);
    emit("r2b1_r0TotalWireLength", r0TotalWire);
    emit("r2b1_r0PartialCostNoEdgeNoRt", score.totalNoRuntime);
    emit("r2b1_r0ChannelOverflowTotal", r0ChOverflow);
    emit("r2b1_r0ChannelOverflowRate",
         r0ChCapacity > 0.0 ? r0ChOverflow / r0ChCapacity : 0.0);
    emit("r2b1_familyRaw.R0Fallback", familyRaw[0]);
    emit("r2b1_familyRaw.Direct", familyRaw[1]);
    emit("r2b1_familyRaw.BaseUnifiedShortest", familyRaw[2]);
    emit("r2b1_familyRaw.AccessAltUnified", familyRaw[3]);
    emit("r2b1_familyRaw.AvoidHotUnified", familyRaw[4]);
    emit("r2b1_familyRaw.CapacityAwareUnified", familyRaw[6]);
    emit("r2b1_familyDistinct.R0Fallback", familyDistinct[0]);
    emit("r2b1_familyDistinct.Direct", familyDistinct[1]);
    emit("r2b1_familyDistinct.BaseUnifiedShortest", familyDistinct[2]);
    emit("r2b1_familyDistinct.AccessAltUnified", familyDistinct[3]);
    emit("r2b1_familyDistinct.AvoidHotUnified", familyDistinct[4]);
    emit("r2b1_familyDistinct.CapacityAwareUnified", familyDistinct[6]);
    emit("r2b1_gate", std::string(gateOk ? "ok" : "FAIL"));
    emit("r2b1_verdict", std::string(material ? "material" : "no_material_signal"));

    return gateOk ? 0 : 2;
}

#endif

int runR2c(Design& design,
           bool splitV0 = false,
           bool dynamicSplitV0 = false,
           bool splitRatioOracleV0 = false,
           bool split3WayOracleV0 = false,
           bool allocationPoolSelectorV0 = false,
           bool allLoserAllocationPoolV0 = false,
           bool wideSplitRatiosV0 = false,
           bool fullSwitchOnlyV0 = false,
           bool splitOnlyV0 = false,
           bool staticReselectV0 = false,
           bool staticReselectThenHotV0 = false,
           bool staticReselectThenHotReselectV0 = false,
           bool r2e2ThenD5AllocationV0 = false,
           bool r2e2ThenSplitOnlyAllocationV0 = false,
           bool r2e2ThenSplitOnlyAllocationReselectV0 = false,
           bool r2e2ThenWideSplitOnlyAllocationV0 = false,
           bool r2e2ThenWidePlusFiftySplitOnlyAllocationV0 = false,
           bool r2e2ThenSlimSplitOnlyAllocationV0 = false,
           bool r2e2ThenSlimSplitOnlyAllocationReselectV0 = false,
           bool r2e2ThenSegmentBypassV0 = false,
           bool r2e2ThenSegmentBypassReselectV0 = false,
           bool r2e2ThenSegmentThenSlimSplitReselectV0 = false,
           bool r2g0ResidualOpportunityAuditV0 = false) {
    const auto t0 = std::chrono::steady_clock::now();
    const std::string csvPath = "<in-memory>";
    const std::string cfgPath = "<floorplanner>";

    routerx::SearchCostMode searchCostMode;
    if (!parseLiteSearchCostMode(searchCostMode)) return 1;
    AccessAltUniverseMode accessMode;
    if (!parseAccessAltUniverseMode(accessMode)) return 1;
    CapacityAwareConfig capacityConfig;
    if (!parseCapacityAwareConfig(capacityConfig)) return 1;
    AvoidHotEligibilityMode eligibilityMode;
    if (!parseAvoidHotEligibilityMode(eligibilityMode)) return 1;
    const std::vector<double> weights = avoidHotWeights();
    const double hotThreshold = avoidHotThreshold();
    if (hotThreshold < 0.0) return 1;
    const int hotPasses = avoidHotPasses();
    if (hotPasses < 0) return 1;
    const int maxSplitIterations = splitMaxIterations();
    if (maxSplitIterations < 0) return 1;
    const int maxReselectPasses = reselectMaxPasses();
    if (maxReselectPasses < 0) return 1;
    if (r2g0ResidualOpportunityAuditV0) {
        r2e2ThenSegmentThenSlimSplitReselectV0 = true;
    }
    if (r2e2ThenSegmentThenSlimSplitReselectV0) {
        r2e2ThenSegmentBypassReselectV0 = true;
    }
    if (r2e2ThenSegmentBypassReselectV0) {
        r2e2ThenSegmentBypassV0 = true;
    }

    // R2c is explicitly a full-graph AvoidHot experiment. Do not let a stale
    // RX_LITE_ROUTE_SPACE env var turn SOFT access off and answer a different
    // question.
    const routerx::SoftRoutingPolicy routeSpace{/*endpointAccess=*/true,
                                                /*softSoftEdges=*/true};
    routerx::RxGraph graph;
    graph.build(design, routeSpace);

    const std::vector<routerx::DemandedPair> pairs = routerx::demandedPairs(design);
    std::vector<LiteRoutedPair> r0Routes;
    r0Routes.reserve(pairs.size());

    routerx::RxResourceLedgerLite r0Ledger;
    r0Ledger.build(design);

    long long totalDemandNets = 0;
    long long r0SearchCalls = 0;
    long long r0OpenPairs = 0;
    long long r0InvalidEmit = 0;
    double r0TotalWire = 0.0;

    for (const routerx::DemandedPair& p : pairs) {
        totalDemandNets += p.nets;
        LiteRoutedPair r0 = routeR0Fallback(design, graph, p, searchCostMode);
        r0SearchCalls += r0.searchCalls;
        if (!r0.found || !r0.emitValid) {
            if (!r0.found) ++r0OpenPairs;
            if (!r0.emitValid) ++r0InvalidEmit;
            r0Routes.push_back(std::move(r0));
            continue;
        }
        const routerx::LiteRouteImpact impact =
            r0Ledger.projectSteps(design, r0.route.steps, p.nets);
        r0TotalWire += impact.wire;
        r0Ledger.commitSteps(design, r0.route.steps, p.nets);
        r0Routes.push_back(std::move(r0));
    }
    const LiteWorldScore r0Score = scoreLiteWorld(design, r0Ledger, r0TotalWire);

    routerx::RxResourceLedgerLite world1Ledger = r0Ledger;
    double world1Wire = r0TotalWire;
    std::vector<routerx::LiteCandidate> world1Winners(pairs.size());
    std::vector<std::vector<routerx::LiteCandidate>> staticCandidateSets(pairs.size());

    long long staticSearchCalls = 0;
    long long altAccessSearchCalls = 0;
    long long world1RawCandidates = 0;
    long long world1DistinctCandidates = 0;
    long long world1DuplicateCandidates = 0;
    long long missingR0Fallback = 0;
    long long invalidEmitCandidates = 0;
    long long illegalFtCandidates = 0;
    long long world1ChangedPairs = 0;
    long long world1SelectedInvalid = 0;
    long long world1SelectedIllegalFt = 0;
    long long world1SelectedUsedFeedthrough = 0;
    std::array<long long, routerx::kLiteCandidateFamilyCount> world1FamilyWins{};
    std::array<long long, routerx::kLiteCandidateFamilyCount> world1FamilyRaw{};
    std::array<long long, routerx::kLiteCandidateFamilyCount> world1FamilyDistinct{};

    for (size_t i = 0; i < pairs.size(); ++i) {
        const routerx::DemandedPair& p = pairs[i];
        const LiteRoutedPair& r0 = r0Routes[i];
        if (!r0.found || !r0.emitValid) {
            ++missingR0Fallback;
            continue;
        }

        world1Ledger.ripupSteps(design, r0.route.steps, p.nets);
        std::vector<routerx::LiteCandidate> raw =
            generateR2b1Candidates(design, graph, p, r0, searchCostMode, accessMode,
                                   capacityConfig,
                                   staticSearchCalls, altAccessSearchCalls);
        const double r0Wire = world1Ledger.projectSteps(design, r0.route.steps, p.nets).wire;
        for (routerx::LiteCandidate& c : raw) {
            if (c.emitValid) c.impact = world1Ledger.projectSteps(design, c.route.steps, p.nets);
        }

        const routerx::LitePairCandidateReport report =
            routerx::summarizeLiteCandidateSurface(raw, r0Wire);
        world1RawCandidates += report.rawCandidates;
        world1DistinctCandidates += report.distinctCandidates;
        world1DuplicateCandidates += report.duplicateCandidates;
        missingR0Fallback += report.missingR0Fallback;
        invalidEmitCandidates += report.invalidEmitCandidates;
        illegalFtCandidates += report.illegalFtCandidates;
        for (int f = 0; f < routerx::kLiteCandidateFamilyCount; ++f) {
            world1FamilyRaw[f] += report.familyRaw[f];
            world1FamilyDistinct[f] += report.familyDistinct[f];
        }

        routerx::LiteDedupResult dedup = routerx::dedupLiteCandidates(raw);
        for (routerx::LiteCandidate& c : dedup.candidates) {
            if (c.emitValid) c.impact = world1Ledger.projectSteps(design, c.route.steps, p.nets);
        }
        staticCandidateSets[i] = dedup.candidates;

        const int r0Idx = findR0FallbackCandidate(dedup.candidates);
        const int chosenIdx = chooseLiteCandidate(
            dedup.candidates, LiteSelectorMode::MinOfficialImmediate,
            r0Wire, design.alpha);
        if (r0Idx < 0 || chosenIdx < 0) {
            ++missingR0Fallback;
            world1Ledger.commitSteps(design, r0.route.steps, p.nets);
            continue;
        }

        const routerx::LiteCandidate& r0Cand = dedup.candidates[r0Idx];
        const routerx::LiteCandidate& winner = dedup.candidates[chosenIdx];
        if (!selectableCandidate(winner)) ++world1SelectedInvalid;
        if (winner.impact.dIllegalFtPenalty > 0.0 ||
            winner.impact.illegalFtBlocksAfter > 0) {
            ++world1SelectedIllegalFt;
        }
        if (winner.usedFeedthrough) ++world1SelectedUsedFeedthrough;
        if (winner.signature != r0Cand.signature) ++world1ChangedPairs;
        const int fidx = static_cast<int>(winner.family);
        if (fidx >= 0 && fidx < routerx::kLiteCandidateFamilyCount) {
            ++world1FamilyWins[fidx];
        }
        world1Wire += winner.impact.wire - r0Cand.impact.wire;
        world1Ledger.commitSteps(design, winner.route.steps, p.nets);
        world1Winners[i] = winner;
    }

    const LiteWorldScore world1Score = scoreLiteWorld(design, world1Ledger, world1Wire);
    const HotAxisRatios world1Hot =
        hotAxisRatios(design, world1Ledger, hotThreshold);

    if (staticReselectV0) {
        routerx::RxResourceLedgerLite reselectLedger = world1Ledger;
        double reselectWire = world1Wire;
        std::vector<routerx::LiteCandidate> reselectWinners = world1Winners;
        const StaticReselectResult reselect = runStaticReselectFixedPoint(
            design, pairs, staticCandidateSets, reselectWinners,
            reselectLedger, reselectWire, maxReselectPasses);
        const LiteWorldScore finalScore =
            scoreLiteWorld(design, reselectLedger, reselectWire);
        const auto t1 = std::chrono::steady_clock::now();
        const double runtimeSec =
            std::chrono::duration<double>(t1 - t0).count();

        bool passMonotone = true;
        long long totalChanged = 0;
        double totalPassGain = 0.0;
        for (const StaticReselectPassStats& ps : reselect.passes) {
            if (ps.officialGain < -1.0e-6) passMonotone = false;
            totalChanged += ps.changedPairs;
            totalPassGain += ps.officialGain;
        }
        const bool gateOk = (r0OpenPairs == 0 && r0InvalidEmit == 0 &&
                             missingR0Fallback == 0 &&
                             invalidEmitCandidates == 0 &&
                             illegalFtCandidates == 0 &&
                             world1SelectedInvalid == 0 &&
                             world1SelectedIllegalFt == 0 &&
                             reselect.selectedInvalid == 0 &&
                             reselect.selectedIllegalFt == 0 &&
                             passMonotone && finalScore.score.legal);

        emit("lite_mode", std::string("r2e0"));
        emit("lite_searchCostMode", std::string(liteSearchCostModeName(searchCostMode)));
        emitCapacityAwareConfig(capacityConfig);
        emit("lite_routeSpace", std::string(liteRouteSpaceName(routeSpace)));
        emit("r2e0_scope", std::string("static_only_fixed_point_reselect"));
        emit("r2e0_research", std::string("off"));
        emit("r2e0_split", std::string("off"));
        emit("r2e0_avoidHot", std::string("off"));
        emit("r2e0_maxPasses", static_cast<long long>(maxReselectPasses));
        emit("lite_csv", csvPath);
        emit("lite_cfg", cfgPath);
        emit("lite_alpha", design.alpha);
        emit("lite_demandedPairCount", static_cast<long long>(pairs.size()));
        emit("lite_totalDemandedNets", totalDemandNets);
        emitLiteGraphReport(graph);
        emit("r2e0_r0OpenPairs", r0OpenPairs);
        emit("r2e0_missingFallback", missingR0Fallback);
        emit("r2e0_invalidEmitCandidates", invalidEmitCandidates);
        emit("r2e0_illegalFtCandidates", illegalFtCandidates);
        emit("r2e0_world1ChangedPairsVsR0", world1ChangedPairs);
        emit("r2e0_world1SelectedUsedFeedthrough", world1SelectedUsedFeedthrough);
        emitLiteWorldScore("r2e0_r0", r0Score);
        emitLiteWorldScore("r2e0_world1", world1Score);
        emitLiteWorldScore("r2e0_final", finalScore);
        emit("r2e0_passesRun", static_cast<long long>(reselect.passes.size()));
        emit("r2e0_totalChangedPairs", totalChanged);
        emit("r2e0_totalPassOfficialGain", totalPassGain);
        emit("r2e0_finalOfficialGainVsWorld1",
             world1Score.score.totalNoRuntime - finalScore.score.totalNoRuntime);
        emit("r2e0_finalOfficialGainVsR0",
             r0Score.score.totalNoRuntime - finalScore.score.totalNoRuntime);
        emit("r2e0_finalWireGainVsWorld1",
             world1Score.totalWire - finalScore.totalWire);
        emit("r2e0_finalChOverflowDropVsWorld1",
             world1Score.channelOverflow - finalScore.channelOverflow);
        emit("r2e0_finalFtRequiredDeltaVsWorld1",
             finalScore.ft.sumRb - world1Score.ft.sumRb);
        emit("r2e0_selectedUsedFeedthrough", reselect.selectedUsedFeedthrough);
        emit("r2e0_familyWins.R0Fallback", reselect.familyWins[0]);
        emit("r2e0_familyWins.Direct", reselect.familyWins[1]);
        emit("r2e0_familyWins.BaseUnifiedShortest", reselect.familyWins[2]);
        emit("r2e0_familyWins.AccessAltUnified", reselect.familyWins[3]);
        emit("r2e0_familyWins.AvoidHotUnified", reselect.familyWins[4]);
        emit("r2e0_familyWins.CapacityAwareUnified", reselect.familyWins[6]);
        for (size_t i = 0; i < reselect.passes.size(); ++i) {
            const StaticReselectPassStats& ps = reselect.passes[i];
            const std::string pfx = "r2e0_pass" + std::to_string(i + 1);
            emit((pfx + "ChangedPairs").c_str(), ps.changedPairs);
            emit((pfx + "OfficialGain").c_str(), ps.officialGain);
            emit((pfx + "WireGain").c_str(), ps.wireGain);
            emit((pfx + "ChannelOverflowBefore").c_str(),
                 ps.channelOverflowBefore);
            emit((pfx + "ChannelOverflowAfter").c_str(),
                 ps.channelOverflowAfter);
            emit((pfx + "FtRequiredBefore").c_str(), ps.ftRequiredBefore);
            emit((pfx + "FtRequiredAfter").c_str(), ps.ftRequiredAfter);
            emit((pfx + "SelectedInvalid").c_str(), ps.selectedInvalid);
            emit((pfx + "SelectedIllegalFt").c_str(), ps.selectedIllegalFt);
        }
        emit("r2e0_passMonotone", std::string(passMonotone ? "yes" : "no"));
        emit("r2e0_gate", std::string(gateOk ? "ok" : "FAIL"));
        emit("r2e0_runtimeSec", runtimeSec);
        return gateOk ? 0 : 2;
    }

    if (staticReselectThenHotV0 || staticReselectThenHotReselectV0 ||
        r2e2ThenD5AllocationV0 || r2e2ThenSplitOnlyAllocationV0 ||
        r2e2ThenSplitOnlyAllocationReselectV0 ||
        r2e2ThenWideSplitOnlyAllocationV0 ||
        r2e2ThenWidePlusFiftySplitOnlyAllocationV0 ||
        r2e2ThenSlimSplitOnlyAllocationV0 ||
        r2e2ThenSlimSplitOnlyAllocationReselectV0 ||
        r2e2ThenSegmentBypassV0) {
        routerx::RxResourceLedgerLite r1Ledger = world1Ledger;
        double r1Wire = world1Wire;
        std::vector<routerx::LiteCandidate> r1Winners = world1Winners;
        const StaticReselectResult reselect = runStaticReselectFixedPoint(
            design, pairs, staticCandidateSets, r1Winners,
            r1Ledger, r1Wire, maxReselectPasses);
        const LiteWorldScore r1Score = scoreLiteWorld(design, r1Ledger, r1Wire);
        const HotAxisRatios r1Hot = hotAxisRatios(design, r1Ledger, hotThreshold);

        routerx::RxResourceLedgerLite r2Ledger = r1Ledger;
        double r2Wire = r1Wire;
        std::vector<routerx::LiteCandidate> r2Winners(pairs.size());
        std::vector<std::vector<routerx::LiteCandidate>> r2CandidateSets(pairs.size());
        long long avoidHotEligiblePairs = 0;
        long long avoidHotSearchCalls = 0;
        long long avoidHotRawCandidates = 0;
        long long avoidHotDistinctCandidates = 0;
        long long r2RawCandidates = 0;
        long long r2DistinctCandidates = 0;
        long long r2DuplicateCandidates = 0;
        long long r2ChangedPairs = 0;
        long long r2SelectedInvalid = 0;
        long long r2SelectedIllegalFt = 0;
        long long r2SelectedUsedFeedthrough = 0;
        double r2LocalOfficialGainSum = 0.0;
        std::array<long long, routerx::kLiteCandidateFamilyCount> r2FamilyWins{};
        std::array<long long, routerx::kLiteCandidateFamilyCount> r2FamilyRaw{};
        std::array<long long, routerx::kLiteCandidateFamilyCount> r2FamilyDistinct{};

        for (size_t i = 0; i < pairs.size(); ++i) {
            const routerx::DemandedPair& p = pairs[i];
            routerx::LiteCandidate current = r1Winners[i];
            if (current.route.steps.empty() || !current.emitValid) {
                ++missingR0Fallback;
                continue;
            }

            const bool touchesHot = avoidHotEligible(
                design, current, staticCandidateSets[i], r1Hot, eligibilityMode);
            if (touchesHot) ++avoidHotEligiblePairs;
            if (!touchesHot) {
                if (current.usedFeedthrough) ++r2SelectedUsedFeedthrough;
                const int fidx = static_cast<int>(current.family);
                if (fidx >= 0 && fidx < routerx::kLiteCandidateFamilyCount) {
                    ++r2FamilyWins[fidx];
                }
                r2Winners[i] = current;
                r2CandidateSets[i] = staticCandidateSets[i];
                continue;
            }

            r2Ledger.ripupSteps(design, current.route.steps, p.nets);
            std::vector<routerx::LiteCandidate> raw;
            raw.reserve(staticCandidateSets[i].size() + weights.size() + 1);
            raw.push_back(current);
            raw.insert(raw.end(), staticCandidateSets[i].begin(), staticCandidateSets[i].end());

            long long localSearchCalls = 0;
            std::vector<routerx::LiteCandidate> hotRaw = generateAvoidHotCandidates(
                design, graph, p, searchCostMode, r1Hot, weights,
                localSearchCalls, avoidHotSearchCalls);
            staticSearchCalls += localSearchCalls;
            avoidHotRawCandidates += static_cast<long long>(hotRaw.size());
            raw.insert(raw.end(), hotRaw.begin(), hotRaw.end());

            for (routerx::LiteCandidate& c : raw) {
                if (c.emitValid) c.impact = r2Ledger.projectSteps(design, c.route.steps, p.nets);
            }
            const routerx::LitePairCandidateReport report =
                routerx::summarizeLiteCandidateSurface(raw, current.impact.wire);
            r2RawCandidates += report.rawCandidates;
            r2DistinctCandidates += report.distinctCandidates;
            r2DuplicateCandidates += report.duplicateCandidates;
            invalidEmitCandidates += report.invalidEmitCandidates;
            illegalFtCandidates += report.illegalFtCandidates;
            for (int f = 0; f < routerx::kLiteCandidateFamilyCount; ++f) {
                r2FamilyRaw[f] += report.familyRaw[f];
                r2FamilyDistinct[f] += report.familyDistinct[f];
            }

            routerx::LiteDedupResult dedup = routerx::dedupLiteCandidates(raw);
            for (routerx::LiteCandidate& c : dedup.candidates) {
                if (c.emitValid) c.impact = r2Ledger.projectSteps(design, c.route.steps, p.nets);
            }
            r2CandidateSets[i] = dedup.candidates;

            int currentIdx = -1;
            int bestIdx = -1;
            for (size_t j = 0; j < dedup.candidates.size(); ++j) {
                const routerx::LiteCandidate& c = dedup.candidates[j];
                if (c.family == routerx::LiteCandidateFamily::AvoidHotUnified) {
                    ++avoidHotDistinctCandidates;
                }
                if (c.signature == current.signature && currentIdx < 0) {
                    currentIdx = static_cast<int>(j);
                }
                if (!selectableCandidate(c)) continue;
                if (bestIdx < 0) {
                    bestIdx = static_cast<int>(j);
                    continue;
                }
                const double kc = officialImmediateKey(c, design.alpha);
                const double kb = officialImmediateKey(dedup.candidates[bestIdx], design.alpha);
                if (kc < kb - 1.0e-9 ||
                    (std::fabs(kc - kb) <= 1.0e-9 &&
                     candidateTieLess(c, dedup.candidates[bestIdx]))) {
                    bestIdx = static_cast<int>(j);
                }
            }
            if (currentIdx < 0) {
                ++missingR0Fallback;
                r2Ledger.commitSteps(design, current.route.steps, p.nets);
                r2Winners[i] = current;
                continue;
            }
            if (bestIdx < 0) bestIdx = currentIdx;

            const double currentKey =
                officialImmediateKey(dedup.candidates[currentIdx], design.alpha);
            const double bestKey =
                officialImmediateKey(dedup.candidates[bestIdx], design.alpha);
            if (bestKey > currentKey + 1.0e-6) bestIdx = currentIdx;

            const routerx::LiteCandidate& winner = dedup.candidates[bestIdx];
            const routerx::LiteCandidate& currentCand = dedup.candidates[currentIdx];
            if (!selectableCandidate(winner)) ++r2SelectedInvalid;
            if (winner.impact.dIllegalFtPenalty > 0.0 ||
                winner.impact.illegalFtBlocksAfter > 0) {
                ++r2SelectedIllegalFt;
            }
            if (winner.usedFeedthrough) ++r2SelectedUsedFeedthrough;
            if (winner.signature != currentCand.signature) ++r2ChangedPairs;
            const int fidx = static_cast<int>(winner.family);
            if (fidx >= 0 && fidx < routerx::kLiteCandidateFamilyCount) {
                ++r2FamilyWins[fidx];
            }
            r2LocalOfficialGainSum += currentKey - officialImmediateKey(winner, design.alpha);
            r2Wire += winner.impact.wire - currentCand.impact.wire;
            r2Ledger.commitSteps(design, winner.route.steps, p.nets);
            r2Winners[i] = winner;
        }

        const LiteWorldScore r2Score = scoreLiteWorld(design, r2Ledger, r2Wire);
        routerx::RxResourceLedgerLite r3Ledger = r2Ledger;
        double r3Wire = r2Wire;
        std::vector<routerx::LiteCandidate> r3Winners = r2Winners;
        StaticReselectResult hotReselect;
        LiteWorldScore r3Score = r2Score;
        if (staticReselectThenHotReselectV0 || r2e2ThenD5AllocationV0 ||
            r2e2ThenSplitOnlyAllocationV0 ||
            r2e2ThenSplitOnlyAllocationReselectV0 ||
            r2e2ThenWideSplitOnlyAllocationV0 ||
            r2e2ThenWidePlusFiftySplitOnlyAllocationV0 ||
            r2e2ThenSlimSplitOnlyAllocationV0 ||
            r2e2ThenSlimSplitOnlyAllocationReselectV0 ||
            r2e2ThenSegmentBypassV0) {
            hotReselect = runStaticReselectFixedPoint(
                design, pairs, r2CandidateSets, r3Winners,
                r3Ledger, r3Wire, maxReselectPasses);
            r3Score = scoreLiteWorld(design, r3Ledger, r3Wire);
        }
        routerx::RxResourceLedgerLite segmentLedger = r3Ledger;
        double segmentWire = r3Wire;
        std::vector<routerx::LiteCandidate> segmentWinners = r3Winners;
        SegmentBypassStats segmentStats;
        const SegmentSearchConfig segmentConfig = segmentSearchConfigFromEnv();
        std::array<long long, routerx::kLiteCandidateFamilyCount> segmentFamilyWins{};
        std::vector<std::vector<routerx::LiteCandidate>> segmentCandidateSets =
            r2CandidateSets;
        LiteWorldScore segmentScore = r3Score;
        if (r2e2ThenSegmentBypassV0) {
            segmentStats = runSegmentBypassPass(
                design, graph, pairs, r2CandidateSets, segmentWinners,
                segmentLedger, segmentWire, hotThreshold, segmentConfig,
                segmentFamilyWins, &segmentCandidateSets);
            segmentScore = scoreLiteWorld(design, segmentLedger, segmentWire);
        }
        routerx::RxResourceLedgerLite segmentReselectLedger = segmentLedger;
        double segmentReselectWire = segmentWire;
        std::vector<routerx::LiteCandidate> segmentReselectWinners = segmentWinners;
        StaticReselectResult segmentReselect;
        LiteWorldScore segmentReselectScore = segmentScore;
        if (r2e2ThenSegmentBypassReselectV0) {
            segmentReselect = runStaticReselectFixedPoint(
                design, pairs, segmentCandidateSets, segmentReselectWinners,
                segmentReselectLedger, segmentReselectWire, maxReselectPasses);
            segmentReselectScore =
                scoreLiteWorld(design, segmentReselectLedger, segmentReselectWire);
        }
        const bool segmentSplitChain =
            r2e2ThenSegmentThenSlimSplitReselectV0;
        const bool allocationPoolEnabled =
            r2e2ThenD5AllocationV0 || r2e2ThenSplitOnlyAllocationV0 ||
            r2e2ThenSplitOnlyAllocationReselectV0 ||
            r2e2ThenWideSplitOnlyAllocationV0 ||
            r2e2ThenWidePlusFiftySplitOnlyAllocationV0 ||
            r2e2ThenSlimSplitOnlyAllocationV0 ||
            r2e2ThenSlimSplitOnlyAllocationReselectV0 ||
            segmentSplitChain;
        const auto& allocationBaseWinners =
            segmentSplitChain ? segmentReselectWinners : r3Winners;
        const auto& allocationCandidateSets =
            segmentSplitChain ? segmentCandidateSets : r2CandidateSets;
        const LiteWorldScore allocationBaseScore =
            segmentSplitChain ? segmentReselectScore : r3Score;

        routerx::RxResourceLedgerLite r4Ledger =
            segmentSplitChain ? segmentReselectLedger : r3Ledger;
        double r4Wire = segmentSplitChain ? segmentReselectWire : r3Wire;
        AllocationPoolSelectorStats r2f0AllocationStats;
        std::vector<std::vector<AllocationPart>> r4Allocations;
        std::array<long long, routerx::kLiteCandidateFamilyCount> r4FamilyWins{};
        LiteWorldScore r4Score = allocationBaseScore;
        if (allocationPoolEnabled) {
            r2f0AllocationStats = runAllocationPoolSelector(
                design, pairs, allocationBaseWinners, allocationCandidateSets,
                r4Ledger,
                r4Wire, hotThreshold, r4FamilyWins,
                /*allPairs=*/true,
                /*splitAllLosers=*/true,
                /*wideFixedRatios=*/r2e2ThenWideSplitOnlyAllocationV0 ||
                    r2e2ThenWidePlusFiftySplitOnlyAllocationV0,
                /*allowFullSwitch=*/r2e2ThenD5AllocationV0,
                /*allowSplit=*/true,
                /*includeFiftyPercent=*/r2e2ThenWidePlusFiftySplitOnlyAllocationV0,
                /*slimRatios=*/r2e2ThenSlimSplitOnlyAllocationV0 ||
                    r2e2ThenSlimSplitOnlyAllocationReselectV0 ||
                    segmentSplitChain,
                &r4Allocations);
            r4Score = scoreLiteWorld(design, r4Ledger, r4Wire);
        }
        routerx::RxResourceLedgerLite r5Ledger = r4Ledger;
        double r5Wire = r4Wire;
        AllocationReselectResult r2f2Reselect;
        LiteWorldScore r5Score = r4Score;
        if (r2e2ThenSplitOnlyAllocationReselectV0 ||
            r2e2ThenSlimSplitOnlyAllocationReselectV0 ||
            segmentSplitChain) {
            r2f2Reselect = runAllocationReselectFixedPoint(
                design, pairs, allocationBaseWinners, allocationCandidateSets,
                r4Allocations, r5Ledger, r5Wire, hotThreshold,
                maxReselectPasses);
            r5Score = scoreLiteWorld(design, r5Ledger, r5Wire);
        }
        const auto t1 = std::chrono::steady_clock::now();
        const double runtimeSec =
            std::chrono::duration<double>(t1 - t0).count();

        bool passMonotone = true;
        long long totalChanged = 0;
        double totalPassGain = 0.0;
        for (const StaticReselectPassStats& ps : reselect.passes) {
            if (ps.officialGain < -1.0e-6) passMonotone = false;
            totalChanged += ps.changedPairs;
            totalPassGain += ps.officialGain;
        }
        bool hotReselectPassMonotone = true;
        long long hotReselectChanged = 0;
        double hotReselectPassGain = 0.0;
        for (const StaticReselectPassStats& ps : hotReselect.passes) {
            if (ps.officialGain < -1.0e-6) hotReselectPassMonotone = false;
            hotReselectChanged += ps.changedPairs;
            hotReselectPassGain += ps.officialGain;
        }
        const bool r1Monotone =
            r1Score.score.totalNoRuntime <= world1Score.score.totalNoRuntime + 1.0e-6;
        const bool r2Monotone =
            r2Score.score.totalNoRuntime <= r1Score.score.totalNoRuntime + 1.0e-6;
        const bool r3Monotone =
            !(staticReselectThenHotReselectV0 || r2e2ThenD5AllocationV0 ||
              r2e2ThenSplitOnlyAllocationV0 ||
              r2e2ThenSplitOnlyAllocationReselectV0 ||
              r2e2ThenWideSplitOnlyAllocationV0 ||
              r2e2ThenWidePlusFiftySplitOnlyAllocationV0 ||
              r2e2ThenSlimSplitOnlyAllocationV0 ||
              r2e2ThenSlimSplitOnlyAllocationReselectV0 ||
              r2e2ThenSegmentBypassV0) ||
            r3Score.score.totalNoRuntime <= r2Score.score.totalNoRuntime + 1.0e-6;
        const bool segmentMonotone =
            !r2e2ThenSegmentBypassV0 ||
            segmentScore.score.totalNoRuntime <= r3Score.score.totalNoRuntime + 1.0e-6;
        bool segmentReselectPassMonotone = true;
        long long segmentReselectChanged = 0;
        double segmentReselectPassGain = 0.0;
        for (const StaticReselectPassStats& ps : segmentReselect.passes) {
            if (ps.officialGain < -1.0e-6) segmentReselectPassMonotone = false;
            segmentReselectChanged += ps.changedPairs;
            segmentReselectPassGain += ps.officialGain;
        }
        const bool segmentReselectMonotone =
            !r2e2ThenSegmentBypassReselectV0 ||
            segmentReselectScore.score.totalNoRuntime <=
                segmentScore.score.totalNoRuntime + 1.0e-6;
        const bool r4Monotone =
            !allocationPoolEnabled ||
            r4Score.score.totalNoRuntime <=
                allocationBaseScore.score.totalNoRuntime + 1.0e-6;
        bool allocationReselectPassMonotone = true;
        long long allocationReselectChanged = 0;
        double allocationReselectPassGain = 0.0;
        for (const AllocationReselectPassStats& ps : r2f2Reselect.passes) {
            if (ps.officialGain < -1.0e-6) allocationReselectPassMonotone = false;
            allocationReselectChanged += ps.changedPairs;
            allocationReselectPassGain += ps.officialGain;
        }
        const bool allocationReselectEnabled =
            r2e2ThenSplitOnlyAllocationReselectV0 ||
            r2e2ThenSlimSplitOnlyAllocationReselectV0 ||
            segmentSplitChain;
        const bool r5Monotone =
            !allocationReselectEnabled ||
            r5Score.score.totalNoRuntime <= r4Score.score.totalNoRuntime + 1.0e-6;
        const bool gateOk = (r0OpenPairs == 0 && r0InvalidEmit == 0 &&
                             missingR0Fallback == 0 &&
                             invalidEmitCandidates == 0 &&
                             illegalFtCandidates == 0 &&
                             world1SelectedInvalid == 0 &&
                             world1SelectedIllegalFt == 0 &&
                             reselect.selectedInvalid == 0 &&
                             reselect.selectedIllegalFt == 0 &&
                             r2SelectedInvalid == 0 &&
                             r2SelectedIllegalFt == 0 &&
                             (!staticReselectThenHotReselectV0 ||
                              (hotReselect.selectedInvalid == 0 &&
                               hotReselect.selectedIllegalFt == 0 &&
                               hotReselectPassMonotone)) &&
                             (!r2e2ThenSegmentBypassV0 ||
                             (segmentStats.selectedInvalid == 0 &&
                              segmentStats.invalidCandidates == 0 &&
                              segmentStats.dirtyCandidates == 0 &&
                              segmentStats.illegalFtCandidates == 0 &&
                              segmentStats.selectedIllegalFt == 0 &&
                              segmentStats.changedNonSegment == 0 &&
                              segmentMonotone)) &&
                             (!r2e2ThenSegmentBypassReselectV0 ||
                              (segmentReselect.selectedInvalid == 0 &&
                               segmentReselect.selectedIllegalFt == 0 &&
                               segmentReselectPassMonotone &&
                               segmentReselectMonotone)) &&
                             (!allocationReselectEnabled ||
                              (r2f2Reselect.selectedInvalid == 0 &&
                               r2f2Reselect.selectedIllegalFt == 0 &&
                               allocationReselectPassMonotone)) &&
                             passMonotone && r1Monotone && r2Monotone &&
                             r3Monotone && segmentMonotone &&
                             segmentReselectMonotone &&
                             r4Monotone && r5Monotone &&
                             r2Score.score.legal && r3Score.score.legal &&
                             segmentScore.score.legal &&
                             segmentReselectScore.score.legal &&
                             r4Score.score.legal && r5Score.score.legal);

        const std::string liteModeName =
            r2g0ResidualOpportunityAuditV0 ? "r2g0" :
            r2e2ThenSegmentThenSlimSplitReselectV0 ? "r2s2" :
            r2e2ThenSegmentBypassReselectV0 ? "r2s1" :
            r2e2ThenSegmentBypassV0 ? "r2s0" :
            r2e2ThenSlimSplitOnlyAllocationReselectV0 ? "r2f6" :
            r2e2ThenSlimSplitOnlyAllocationV0 ? "r2f5" :
            r2e2ThenWidePlusFiftySplitOnlyAllocationV0 ? "r2f4" :
            r2e2ThenWideSplitOnlyAllocationV0 ? "r2f3" :
            r2e2ThenSplitOnlyAllocationReselectV0 ? "r2f2" :
            (r2e2ThenSplitOnlyAllocationV0 ? "r2f1" :
            (r2e2ThenD5AllocationV0 ? "r2f0" :
             (staticReselectThenHotReselectV0 ? "r2e2" : "r2e1")));
        emit("lite_mode", liteModeName);
        emit("lite_searchCostMode", std::string(liteSearchCostModeName(searchCostMode)));
        emitCapacityAwareConfig(capacityConfig);
        emit("lite_routeSpace", std::string(liteRouteSpaceName(routeSpace)));
        emit("r2e1_scope", std::string("static_fixed_point_then_avoid_hot_one_pass"));
        emit("r2e1_research", std::string("avoid_hot_only"));
        emit("r2e1_split", std::string("off"));
        emit("r2e1_secondReselect", std::string("off"));
        emit("r2e1_maxStaticPasses", static_cast<long long>(maxReselectPasses));
        emit("lite_csv", csvPath);
        emit("lite_cfg", cfgPath);
        emit("lite_alpha", design.alpha);
        emit("lite_demandedPairCount", static_cast<long long>(pairs.size()));
        emit("lite_totalDemandedNets", totalDemandNets);
        emitLiteGraphReport(graph);
        emit("r2e1_r0OpenPairs", r0OpenPairs);
        emit("r2e1_missingFallback", missingR0Fallback);
        emit("r2e1_invalidEmitCandidates", invalidEmitCandidates);
        emit("r2e1_illegalFtCandidates", illegalFtCandidates);
        emit("r2e1_staticGreedyChangedPairsVsR0", world1ChangedPairs);
        emit("r2e1_staticGreedySelectedUsedFeedthrough", world1SelectedUsedFeedthrough);
        emitLiteWorldScore("r2e1_r0", r0Score);
        emitLiteWorldScore("r2e1_staticGreedy", world1Score);
        emitLiteWorldScore("r2e1_r1", r1Score);
        emitLiteWorldScore("r2e1_r2", r2Score);
        emit("r2e1_staticPassesRun", static_cast<long long>(reselect.passes.size()));
        emit("r2e1_staticTotalChangedPairs", totalChanged);
        emit("r2e1_staticTotalPassOfficialGain", totalPassGain);
        emit("r2e1_r1OfficialGainVsStaticGreedy",
             world1Score.score.totalNoRuntime - r1Score.score.totalNoRuntime);
        emit("r2e1_r1OfficialGainVsR0",
             r0Score.score.totalNoRuntime - r1Score.score.totalNoRuntime);
        emit("r2e1_r1WireGainVsStaticGreedy",
             world1Score.totalWire - r1Score.totalWire);
        emit("r2e1_r1ChOverflowDropVsStaticGreedy",
             world1Score.channelOverflow - r1Score.channelOverflow);
        emit("r2e1_r1FtRequiredDeltaVsStaticGreedy",
             r1Score.ft.sumRb - world1Score.ft.sumRb);
        emit("r2e1_r2OfficialGainVsR1",
             r1Score.score.totalNoRuntime - r2Score.score.totalNoRuntime);
        emit("r2e1_r2OfficialGainVsStaticGreedy",
             world1Score.score.totalNoRuntime - r2Score.score.totalNoRuntime);
        emit("r2e1_r2OfficialGainVsR0",
             r0Score.score.totalNoRuntime - r2Score.score.totalNoRuntime);
        emit("r2e1_r2WireGainVsR1", r1Score.totalWire - r2Score.totalWire);
        emit("r2e1_r2ChOverflowDropVsR1",
             r1Score.channelOverflow - r2Score.channelOverflow);
        emit("r2e1_r2FtRequiredDeltaVsR1", r2Score.ft.sumRb - r1Score.ft.sumRb);
        emit("r2e1_r2LocalOfficialGainSum", r2LocalOfficialGainSum);
        emit("r2e1_avoidHotEligibility",
             std::string(avoidHotEligibilityModeName(eligibilityMode)));
        emit("r2e1_hotThreshold", hotThreshold);
        emit("r2e1_avoidHotWeights", weightsString(weights));
        emit("r2e1_avoidHotEligiblePairs", avoidHotEligiblePairs);
        emit("r2e1_avoidHotSearchCalls", avoidHotSearchCalls);
        emit("r2e1_avoidHotRawCandidates", avoidHotRawCandidates);
        emit("r2e1_avoidHotDistinctCandidates", avoidHotDistinctCandidates);
        emit("r2e1_r2RawCandidates", r2RawCandidates);
        emit("r2e1_r2DistinctCandidates", r2DistinctCandidates);
        emit("r2e1_r2DuplicateCandidates", r2DuplicateCandidates);
        emit("r2e1_r1SelectedUsedFeedthrough", reselect.selectedUsedFeedthrough);
        emit("r2e1_r2SelectedUsedFeedthrough", r2SelectedUsedFeedthrough);
        emit("r2e1_r1FamilyWins.R0Fallback", reselect.familyWins[0]);
        emit("r2e1_r1FamilyWins.Direct", reselect.familyWins[1]);
        emit("r2e1_r1FamilyWins.BaseUnifiedShortest", reselect.familyWins[2]);
        emit("r2e1_r1FamilyWins.AccessAltUnified", reselect.familyWins[3]);
        emit("r2e1_r1FamilyWins.AvoidHotUnified", reselect.familyWins[4]);
        emit("r2e1_r1FamilyWins.CapacityAwareUnified", reselect.familyWins[6]);
        emit("r2e1_r2FamilyWins.R0Fallback", r2FamilyWins[0]);
        emit("r2e1_r2FamilyWins.Direct", r2FamilyWins[1]);
        emit("r2e1_r2FamilyWins.BaseUnifiedShortest", r2FamilyWins[2]);
        emit("r2e1_r2FamilyWins.AccessAltUnified", r2FamilyWins[3]);
        emit("r2e1_r2FamilyWins.AvoidHotUnified", r2FamilyWins[4]);
        emit("r2e1_r2FamilyWins.CapacityAwareUnified", r2FamilyWins[6]);
        emit("r2e1_r2FamilyRaw.AvoidHotUnified", r2FamilyRaw[4]);
        emit("r2e1_r2FamilyDistinct.AvoidHotUnified", r2FamilyDistinct[4]);
        for (size_t i = 0; i < reselect.passes.size(); ++i) {
            const StaticReselectPassStats& ps = reselect.passes[i];
            const std::string pfx = "r2e1_staticPass" + std::to_string(i + 1);
            emit((pfx + "ChangedPairs").c_str(), ps.changedPairs);
            emit((pfx + "OfficialGain").c_str(), ps.officialGain);
            emit((pfx + "WireGain").c_str(), ps.wireGain);
            emit((pfx + "ChannelOverflowBefore").c_str(),
                 ps.channelOverflowBefore);
            emit((pfx + "ChannelOverflowAfter").c_str(),
                 ps.channelOverflowAfter);
            emit((pfx + "FtRequiredBefore").c_str(), ps.ftRequiredBefore);
            emit((pfx + "FtRequiredAfter").c_str(), ps.ftRequiredAfter);
        }
        emitHotWorld("r2e1_staticGreedy", design, world1Ledger, hotThreshold);
        emitHotWorld("r2e1_r1", design, r1Ledger, hotThreshold);
        emitHotWorld("r2e1_r2", design, r2Ledger, hotThreshold);
        emitHotWorldOverlap("r2e1_staticGreedyToR1", design, world1Ledger,
                            r1Ledger, hotThreshold);
        emitHotWorldOverlap("r2e1_r1ToR2", design, r1Ledger,
                            r2Ledger, hotThreshold);
        emit("r2e1_staticPassMonotone", std::string(passMonotone ? "yes" : "no"));
        emit("r2e1_r1MonotoneVsStaticGreedy", std::string(r1Monotone ? "yes" : "no"));
        emit("r2e1_r2MonotoneVsR1", std::string(r2Monotone ? "yes" : "no"));
        emit("r2e1_runtimeSec", runtimeSec);
        emit("r2e1_gate", std::string(gateOk ? "ok" : "FAIL"));
        if (staticReselectThenHotReselectV0 || r2e2ThenD5AllocationV0 ||
            r2e2ThenSplitOnlyAllocationV0 ||
            r2e2ThenSplitOnlyAllocationReselectV0 ||
            r2e2ThenWideSplitOnlyAllocationV0 ||
            r2e2ThenWidePlusFiftySplitOnlyAllocationV0 ||
            r2e2ThenSlimSplitOnlyAllocationV0 ||
            r2e2ThenSlimSplitOnlyAllocationReselectV0 ||
            r2e2ThenSegmentBypassV0) {
            emit("r2e2_scope", std::string("static_fixed_point_then_avoid_hot_then_fixed_point"));
            emit("r2e2_split", std::string("off"));
            emit("r2e2_secondReselect", std::string("on"));
            emit("r2e2_maxHotReselectPasses", static_cast<long long>(maxReselectPasses));
            emitLiteWorldScore("r2e2_r3", r3Score);
            emit("r2e2_r3OfficialGainVsR2",
                 r2Score.score.totalNoRuntime - r3Score.score.totalNoRuntime);
            emit("r2e2_r3OfficialGainVsR1",
                 r1Score.score.totalNoRuntime - r3Score.score.totalNoRuntime);
            emit("r2e2_r3OfficialGainVsStaticGreedy",
                 world1Score.score.totalNoRuntime - r3Score.score.totalNoRuntime);
            emit("r2e2_r3OfficialGainVsR0",
                 r0Score.score.totalNoRuntime - r3Score.score.totalNoRuntime);
            emit("r2e2_r3WireGainVsR2", r2Score.totalWire - r3Score.totalWire);
            emit("r2e2_r3ChOverflowDropVsR2",
                 r2Score.channelOverflow - r3Score.channelOverflow);
            emit("r2e2_r3FtRequiredDeltaVsR2", r3Score.ft.sumRb - r2Score.ft.sumRb);
            emit("r2e2_hotReselectPassesRun",
                 static_cast<long long>(hotReselect.passes.size()));
            emit("r2e2_hotReselectTotalChangedPairs", hotReselectChanged);
            emit("r2e2_hotReselectTotalPassOfficialGain", hotReselectPassGain);
            emit("r2e2_r3SelectedUsedFeedthrough",
                 hotReselect.selectedUsedFeedthrough);
            emit("r2e2_r3FamilyWins.R0Fallback", hotReselect.familyWins[0]);
            emit("r2e2_r3FamilyWins.Direct", hotReselect.familyWins[1]);
            emit("r2e2_r3FamilyWins.BaseUnifiedShortest", hotReselect.familyWins[2]);
            emit("r2e2_r3FamilyWins.AccessAltUnified", hotReselect.familyWins[3]);
            emit("r2e2_r3FamilyWins.AvoidHotUnified", hotReselect.familyWins[4]);
            emit("r2e2_r3FamilyWins.SegmentBypassUnified", hotReselect.familyWins[5]);
            emit("r2e2_r3FamilyWins.CapacityAwareUnified", hotReselect.familyWins[6]);
            for (size_t i = 0; i < hotReselect.passes.size(); ++i) {
                const StaticReselectPassStats& ps = hotReselect.passes[i];
                const std::string pfx = "r2e2_hotReselectPass" + std::to_string(i + 1);
                emit((pfx + "ChangedPairs").c_str(), ps.changedPairs);
                emit((pfx + "OfficialGain").c_str(), ps.officialGain);
                emit((pfx + "WireGain").c_str(), ps.wireGain);
                emit((pfx + "ChannelOverflowBefore").c_str(),
                     ps.channelOverflowBefore);
                emit((pfx + "ChannelOverflowAfter").c_str(),
                     ps.channelOverflowAfter);
                emit((pfx + "FtRequiredBefore").c_str(), ps.ftRequiredBefore);
                emit((pfx + "FtRequiredAfter").c_str(), ps.ftRequiredAfter);
            }
            emitHotWorld("r2e2_r3", design, r3Ledger, hotThreshold);
            emitHotWorldOverlap("r2e2_r2ToR3", design, r2Ledger,
                                r3Ledger, hotThreshold);
            emit("r2e2_hotReselectPassMonotone",
                 std::string(hotReselectPassMonotone ? "yes" : "no"));
            emit("r2e2_r3MonotoneVsR2", std::string(r3Monotone ? "yes" : "no"));
            emit("r2e2_runtimeSec", runtimeSec);
            emit("r2e2_gate", std::string(gateOk ? "ok" : "FAIL"));
        }
        if (r2e2ThenSegmentBypassV0) {
            emit("r2s0_scope", std::string("segment_bypass_one_pass_on_r2e2"));
            emit("r2s0_baseline", std::string("r2e2_r3"));
            emit("r2s0_selector", std::string("min_official_immediate_strict"));
            emit("r2s0_dijkstraCost", segmentConfig.name);
            emit("r2s0_topK", static_cast<long long>(segmentConfig.topK));
            emit("r2s0_searchPolicies",
                 static_cast<long long>(segmentConfig.policies.size()));
            emit("r2s0_abShiftDepth",
                 static_cast<long long>(segmentConfig.anchorShiftDepth));
            emitLiteWorldScore("r2s0_final", segmentScore);
            emit("r2s0_officialGainVsR2e2",
                 r3Score.score.totalNoRuntime - segmentScore.score.totalNoRuntime);
            emit("r2s0_wireGainVsR2e2", r3Score.totalWire - segmentScore.totalWire);
            emit("r2s0_chOverflowDropVsR2e2",
                 r3Score.channelOverflow - segmentScore.channelOverflow);
            emit("r2s0_ftRequiredDeltaVsR2e2",
                 segmentScore.ft.sumRb - r3Score.ft.sumRb);
            emit("r2s0_eligiblePairs", segmentStats.eligiblePairs);
            emit("r2s0_hotRuns", segmentStats.hotRuns);
            emit("r2s0_accessPairs", segmentStats.accessPairs);
            emit("r2s0_bannedExistingNodeAccessPairs",
                 segmentStats.bannedExistingNodeAccessPairs);
            emit("r2s0_searchCalls", segmentStats.searchCalls);
            emit("r2s0_dijkstraFoundCandidates",
                 segmentStats.dijkstraFoundCandidates);
            emit("r2s0_noReliefCandidates", segmentStats.noReliefCandidates);
            emit("r2s0_rawCandidates", segmentStats.rawCandidates);
            emit("r2s0_distinctCandidates", segmentStats.distinctCandidates);
            emit("r2s0_duplicateCandidates", segmentStats.duplicateCandidates);
            emit("r2s0_reliefCandidates", segmentStats.reliefCandidates);
            emit("r2s0_reliefStillTargetHotCandidates",
                 segmentStats.reliefStillTargetHotCandidates);
            emit("r2s0_reliefTouchesOtherHotCandidates",
                 segmentStats.reliefTouchesOtherHotCandidates);
            emit("r2s0_reliefTouchesAnyHotCandidates",
                 segmentStats.reliefTouchesAnyHotCandidates);
            emit("r2s0_loopCandidates", segmentStats.loopCandidates);
            emit("r2s0_invalidCandidates", segmentStats.invalidCandidates);
            emit("r2s0_dirtyCandidates", segmentStats.dirtyCandidates);
            emit("r2s0_illegalFtCandidates", segmentStats.illegalFtCandidates);
            emit("r2s0_selectorVisibleSegmentSignatures",
                 segmentStats.selectorVisibleSegmentSignatures);
            emit("r2s0_selectorLabeledSegmentCandidates",
                 segmentStats.selectorLabeledSegmentCandidates);
            emit("r2s0_selectorHiddenSegmentSignatures",
                 segmentStats.selectorHiddenSegmentSignatures);
            emit("r2s0_selectedWins", segmentStats.selectedWins);
            emit("r2s0_selectedHiddenSegment", segmentStats.selectedHiddenSegment);
            emit("r2s0_changedPairs", segmentStats.changedPairs);
            emit("r2s0_changedNonSegment", segmentStats.changedNonSegment);
            emit("r2s0_selectedInvalid", segmentStats.selectedInvalid);
            emit("r2s0_selectedIllegalFt", segmentStats.selectedIllegalFt);
            emit("r2s0_selectedTargetDemandDrop",
                 segmentStats.selectedTargetDemandDrop);
            emit("r2s0_selectedUsedFeedthrough",
                 segmentStats.selectedUsedFeedthrough);
            emit("r2s0_localOfficialGainSum",
                 segmentStats.localOfficialGainSum);
            emit("r2s0_localWireDeltaSum", segmentStats.localWireDeltaSum);
            for (int fi = 0; fi < kSegmentSearchFeatureCount; ++fi) {
                const std::string name =
                    segmentFeatureName(static_cast<SegmentSearchFeature>(fi));
                emit(("r2s0_featureSearchCalls." + name).c_str(),
                     segmentStats.searchCallsByFeature[fi]);
                emit(("r2s0_featureDijkstraFound." + name).c_str(),
                     segmentStats.dijkstraFoundByFeature[fi]);
                emit(("r2s0_featureLoopPruned." + name).c_str(),
                     segmentStats.loopByFeature[fi]);
                emit(("r2s0_featureRaw." + name).c_str(),
                     segmentStats.rawByFeature[fi]);
                emit(("r2s0_featureDistinct." + name).c_str(),
                     segmentStats.distinctByFeature[fi]);
                emit(("r2s0_featureSelected." + name).c_str(),
                     segmentStats.selectedByFeature[fi]);
            }
            emit("r2s0_selectedRank.1", segmentStats.selectedByRank[1]);
            emit("r2s0_selectedRank.2", segmentStats.selectedByRank[2]);
            emit("r2s0_selectedRank.3", segmentStats.selectedByRank[3]);
            emit("r2s0_familyWins.R0Fallback", segmentFamilyWins[0]);
            emit("r2s0_familyWins.Direct", segmentFamilyWins[1]);
            emit("r2s0_familyWins.BaseUnifiedShortest", segmentFamilyWins[2]);
            emit("r2s0_familyWins.AccessAltUnified", segmentFamilyWins[3]);
            emit("r2s0_familyWins.AvoidHotUnified", segmentFamilyWins[4]);
            emit("r2s0_familyWins.SegmentBypassUnified", segmentFamilyWins[5]);
            emit("r2s0_familyWins.CapacityAwareUnified", segmentFamilyWins[6]);
            emitHotWorld("r2s0_final", design, segmentLedger, hotThreshold);
            emitHotWorldOverlap("r2s0_r2e2ToFinal", design, r3Ledger,
                                segmentLedger, hotThreshold);
            emit("r2s0_monotoneVsR2e2",
                 std::string(segmentMonotone ? "yes" : "no"));
            emit("r2s0_runtimeSec", runtimeSec);
            emit("r2s0_gate", std::string(gateOk ? "ok" : "FAIL"));
        }
        if (r2e2ThenSegmentBypassReselectV0) {
            emit("r2s1_scope",
                 std::string("segment_bypass_then_fixed_pool_reselect_on_r2e2"));
            emit("r2s1_baseline", std::string("r2s0_final"));
            emit("r2s1_reselect", std::string("fixed_pool"));
            emit("r2s1_maxPasses", static_cast<long long>(maxReselectPasses));
            emitLiteWorldScore("r2s1_final", segmentReselectScore);
            emit("r2s1_officialGainVsR2e2",
                 r3Score.score.totalNoRuntime -
                     segmentReselectScore.score.totalNoRuntime);
            emit("r2s1_officialGainVsR2s0",
                 segmentScore.score.totalNoRuntime -
                     segmentReselectScore.score.totalNoRuntime);
            emit("r2s1_wireGainVsR2s0",
                 segmentScore.totalWire - segmentReselectScore.totalWire);
            emit("r2s1_chOverflowDropVsR2s0",
                 segmentScore.channelOverflow -
                     segmentReselectScore.channelOverflow);
            emit("r2s1_ftRequiredDeltaVsR2s0",
                 segmentReselectScore.ft.sumRb - segmentScore.ft.sumRb);
            emit("r2s1_passesRun",
                 static_cast<long long>(segmentReselect.passes.size()));
            emit("r2s1_totalChangedPairs", segmentReselectChanged);
            emit("r2s1_totalPassOfficialGain", segmentReselectPassGain);
            emit("r2s1_selectedUsedFeedthrough",
                 segmentReselect.selectedUsedFeedthrough);
            emit("r2s1_familyWins.R0Fallback", segmentReselect.familyWins[0]);
            emit("r2s1_familyWins.Direct", segmentReselect.familyWins[1]);
            emit("r2s1_familyWins.BaseUnifiedShortest",
                 segmentReselect.familyWins[2]);
            emit("r2s1_familyWins.AccessAltUnified",
                 segmentReselect.familyWins[3]);
            emit("r2s1_familyWins.AvoidHotUnified",
                 segmentReselect.familyWins[4]);
            emit("r2s1_familyWins.SegmentBypassUnified",
                 segmentReselect.familyWins[5]);
            emit("r2s1_familyWins.CapacityAwareUnified",
                 segmentReselect.familyWins[6]);
            for (size_t i = 0; i < segmentReselect.passes.size(); ++i) {
                const StaticReselectPassStats& ps = segmentReselect.passes[i];
                const std::string pfx =
                    "r2s1_pass" + std::to_string(i + 1);
                emit((pfx + "ChangedPairs").c_str(), ps.changedPairs);
                emit((pfx + "OfficialGain").c_str(), ps.officialGain);
                emit((pfx + "WireGain").c_str(), ps.wireGain);
                emit((pfx + "ChannelOverflowBefore").c_str(),
                     ps.channelOverflowBefore);
                emit((pfx + "ChannelOverflowAfter").c_str(),
                     ps.channelOverflowAfter);
                emit((pfx + "FtRequiredBefore").c_str(), ps.ftRequiredBefore);
                emit((pfx + "FtRequiredAfter").c_str(), ps.ftRequiredAfter);
            }
            emitHotWorld("r2s1_final", design, segmentReselectLedger,
                         hotThreshold);
            emitHotWorldOverlap("r2s1_r2s0ToFinal", design, segmentLedger,
                                segmentReselectLedger, hotThreshold);
            emit("r2s1_passMonotone",
                 std::string(segmentReselectPassMonotone ? "yes" : "no"));
            emit("r2s1_monotoneVsR2s0",
                 std::string(segmentReselectMonotone ? "yes" : "no"));
            emit("r2s1_runtimeSec", runtimeSec);
            emit("r2s1_gate", std::string(gateOk ? "ok" : "FAIL"));
        }
        if (allocationPoolEnabled) {
            const std::string fPrefix =
                segmentSplitChain ? "r2s2" :
                r2e2ThenSlimSplitOnlyAllocationReselectV0 ? "r2f6" :
                r2e2ThenSlimSplitOnlyAllocationV0 ? "r2f5" :
                r2e2ThenWidePlusFiftySplitOnlyAllocationV0 ? "r2f4" :
                r2e2ThenWideSplitOnlyAllocationV0 ? "r2f3" :
                r2e2ThenSplitOnlyAllocationReselectV0 ? "r2f2" :
                (r2e2ThenSplitOnlyAllocationV0 ? "r2f1" : "r2f0");
            std::string allocationScope;
            if (segmentSplitChain) {
                allocationScope =
                    "segment_bypass_fixed_pool_reselect_then_slim_split_allocation_reselect";
            } else if (r2e2ThenSlimSplitOnlyAllocationReselectV0) {
                allocationScope =
                    "slim_split_only_all_pair_all_loser_allocation_then_allocation_reselect_on_r2e2_baseline";
            } else if (r2e2ThenSlimSplitOnlyAllocationV0) {
                allocationScope =
                    "slim_split_only_all_pair_all_loser_allocation_on_r2e2_baseline";
            } else if (r2e2ThenWidePlusFiftySplitOnlyAllocationV0) {
                allocationScope =
                    "wide_plus_50_split_only_all_pair_all_loser_allocation_on_r2e2_baseline";
            } else if (r2e2ThenWideSplitOnlyAllocationV0) {
                allocationScope =
                    "wide_split_only_all_pair_all_loser_allocation_on_r2e2_baseline";
            } else if (r2e2ThenSplitOnlyAllocationReselectV0) {
                allocationScope =
                    "split_only_all_pair_all_loser_allocation_then_allocation_reselect_on_r2e2_baseline";
            } else if (r2e2ThenSplitOnlyAllocationV0) {
                allocationScope =
                    "split_only_all_pair_all_loser_allocation_on_r2e2_baseline";
            } else {
                allocationScope =
                    "r2d5_all_pair_all_loser_allocation_on_r2e2_baseline";
            }
            emit((fPrefix + "_scope").c_str(), allocationScope);
            emit((fPrefix + "_baseline").c_str(),
                 std::string(segmentSplitChain ? "r2s1_final" : "r2e2_r3"));
            std::string allocationPolicy;
            if (segmentSplitChain ||
                r2e2ThenSlimSplitOnlyAllocationV0 ||
                r2e2ThenSlimSplitOnlyAllocationReselectV0) {
                allocationPolicy =
                    "all_pairs:all_loser_splits:clear_smallest_hot,5,15,25,33,50_percent";
            } else if (r2e2ThenWidePlusFiftySplitOnlyAllocationV0) {
                allocationPolicy =
                    "all_pairs:all_loser_splits:clear_smallest_hot,clear_largest_hot,5,10,15,20,25,33,50_percent";
            } else if (r2e2ThenWideSplitOnlyAllocationV0) {
                allocationPolicy =
                    "all_pairs:all_loser_splits:clear_smallest_hot,clear_largest_hot,5,10,15,20,25,33_percent";
            } else if (r2e2ThenSplitOnlyAllocationV0 ||
                       r2e2ThenSplitOnlyAllocationReselectV0) {
                allocationPolicy =
                    "all_pairs:all_loser_splits:clear_smallest_hot,clear_largest_hot,ten_percent,quarter";
            } else {
                allocationPolicy =
                    "all_pairs:all_loser_full_switch_plus_all_loser_splits:clear_smallest_hot,clear_largest_hot,ten_percent,quarter";
            }
            emit((fPrefix + "_allocationPolicy").c_str(), allocationPolicy);
            emitLiteWorldScore((fPrefix + "_r4").c_str(), r4Score);
            emit((fPrefix + "_officialGainVsR2e2").c_str(),
                 r3Score.score.totalNoRuntime - r4Score.score.totalNoRuntime);
            emit((fPrefix + "_officialGainVsBaseline").c_str(),
                 allocationBaseScore.score.totalNoRuntime -
                     r4Score.score.totalNoRuntime);
            if (segmentSplitChain) {
                emit((fPrefix + "_officialGainVsR2s1").c_str(),
                     segmentReselectScore.score.totalNoRuntime -
                         r4Score.score.totalNoRuntime);
                emit((fPrefix + "_officialGainVsR2s0").c_str(),
                     segmentScore.score.totalNoRuntime -
                         r4Score.score.totalNoRuntime);
            }
            emit((fPrefix + "_officialGainVsStaticGreedy").c_str(),
                 world1Score.score.totalNoRuntime - r4Score.score.totalNoRuntime);
            emit((fPrefix + "_officialGainVsR0").c_str(),
                 r0Score.score.totalNoRuntime - r4Score.score.totalNoRuntime);
            emit((fPrefix + "_wireGainVsR2e2").c_str(),
                 r3Score.totalWire - r4Score.totalWire);
            emit((fPrefix + "_wireGainVsBaseline").c_str(),
                 allocationBaseScore.totalWire - r4Score.totalWire);
            emit((fPrefix + "_chOverflowDropVsR2e2").c_str(),
                 r3Score.channelOverflow - r4Score.channelOverflow);
            emit((fPrefix + "_chOverflowDropVsBaseline").c_str(),
                 allocationBaseScore.channelOverflow - r4Score.channelOverflow);
            emit((fPrefix + "_ftRequiredDeltaVsR2e2").c_str(),
                 r4Score.ft.sumRb - r3Score.ft.sumRb);
            emit((fPrefix + "_ftRequiredDeltaVsBaseline").c_str(),
                 r4Score.ft.sumRb - allocationBaseScore.ft.sumRb);
            emit((fPrefix + "_allocationEligiblePairs").c_str(),
                 r2f0AllocationStats.eligiblePairs);
            emit((fPrefix + "_loserCandidates").c_str(),
                 r2f0AllocationStats.loserCandidates);
            emit((fPrefix + "_reliefLoserCandidates").c_str(),
                 r2f0AllocationStats.reliefLoserCandidates);
            emit((fPrefix + "_allocationCandidates").c_str(),
                 r2f0AllocationStats.allocationCandidates);
            emit((fPrefix + "_keepWins").c_str(), r2f0AllocationStats.keepWins);
            emit((fPrefix + "_fullSwitchWins").c_str(),
                 r2f0AllocationStats.fullSwitchWins);
            emit((fPrefix + "_splitWins").c_str(), r2f0AllocationStats.splitWins);
            emit((fPrefix + "_changedPairs").c_str(),
                 r2f0AllocationStats.changedPairs);
            emit((fPrefix + "_selectedUsedFeedthrough").c_str(),
                 r2f0AllocationStats.selectedUsedFeedthrough);
            emit((fPrefix + "_localOfficialGainSum").c_str(),
                 r2f0AllocationStats.localOfficialGainSum);
            emit((fPrefix + "_localWireDeltaSum").c_str(),
                 r2f0AllocationStats.localWireDeltaSum);
            emit((fPrefix + "_fullSwitchOfficialGainSum").c_str(),
                 r2f0AllocationStats.fullSwitchOfficialGainSum);
            emit((fPrefix + "_fullSwitchWireDeltaSum").c_str(),
                 r2f0AllocationStats.fullSwitchWireDeltaSum);
            emit((fPrefix + "_splitOfficialGainSum").c_str(),
                 r2f0AllocationStats.splitOfficialGainSum);
            emit((fPrefix + "_splitWireDeltaSum").c_str(),
                 r2f0AllocationStats.splitWireDeltaSum);
            emit((fPrefix + "_movedNetsSum").c_str(), r2f0AllocationStats.movedNetsSum);
            for (int k = 0; k < kAllocationSplitPolicyCount; ++k) {
                const AllocationSplitPolicy policy =
                    static_cast<AllocationSplitPolicy>(k);
                const std::string pfx = fPrefix + "_" +
                                        allocationSplitPolicyName(policy);
                emit((pfx + "Candidates").c_str(),
                     r2f0AllocationStats.splitPolicyCandidates[k]);
                emit((pfx + "Wins").c_str(),
                     r2f0AllocationStats.splitPolicyWins[k]);
            }
            emit((fPrefix + "_r4FamilyWins.R0Fallback").c_str(), r4FamilyWins[0]);
            emit((fPrefix + "_r4FamilyWins.Direct").c_str(), r4FamilyWins[1]);
            emit((fPrefix + "_r4FamilyWins.BaseUnifiedShortest").c_str(), r4FamilyWins[2]);
            emit((fPrefix + "_r4FamilyWins.AccessAltUnified").c_str(), r4FamilyWins[3]);
            emit((fPrefix + "_r4FamilyWins.AvoidHotUnified").c_str(), r4FamilyWins[4]);
            emit((fPrefix + "_r4FamilyWins.SegmentBypassUnified").c_str(), r4FamilyWins[5]);
            emit((fPrefix + "_r4FamilyWins.CapacityAwareUnified").c_str(), r4FamilyWins[6]);
            emitHotWorld(fPrefix + "_r4", design, r4Ledger, hotThreshold);
            if (segmentSplitChain) {
                emitHotWorldOverlap(fPrefix + "_baselineToR4", design,
                                    segmentReselectLedger, r4Ledger,
                                    hotThreshold);
            } else {
                emitHotWorldOverlap(fPrefix + "_r3ToR4", design, r3Ledger,
                                    r4Ledger, hotThreshold);
            }
            emit((fPrefix + "_r4MonotoneVsBaseline").c_str(),
                 std::string(r4Monotone ? "yes" : "no"));
            if (!segmentSplitChain) {
                emit((fPrefix + "_r4MonotoneVsR2e2").c_str(),
                     std::string(r4Monotone ? "yes" : "no"));
            }
            emit((fPrefix + "_runtimeSec").c_str(), runtimeSec);
            emit((fPrefix + "_gate").c_str(), std::string(gateOk ? "ok" : "FAIL"));
            if (allocationReselectEnabled) {
                emitLiteWorldScore((fPrefix + "_r5").c_str(), r5Score);
                emit((fPrefix + "_r5OfficialGainVsR4").c_str(),
                     r4Score.score.totalNoRuntime - r5Score.score.totalNoRuntime);
                emit((fPrefix + "_r5OfficialGainVsR2e2").c_str(),
                     r3Score.score.totalNoRuntime - r5Score.score.totalNoRuntime);
                emit((fPrefix + "_r5OfficialGainVsBaseline").c_str(),
                     allocationBaseScore.score.totalNoRuntime -
                         r5Score.score.totalNoRuntime);
                if (segmentSplitChain) {
                    emit((fPrefix + "_r5OfficialGainVsR2s1").c_str(),
                         segmentReselectScore.score.totalNoRuntime -
                             r5Score.score.totalNoRuntime);
                    emit((fPrefix + "_r5OfficialGainVsR2s0").c_str(),
                         segmentScore.score.totalNoRuntime -
                             r5Score.score.totalNoRuntime);
                }
                emit((fPrefix + "_r5OfficialGainVsR0").c_str(),
                     r0Score.score.totalNoRuntime - r5Score.score.totalNoRuntime);
                emit((fPrefix + "_r5WireGainVsR4").c_str(),
                     r4Score.totalWire - r5Score.totalWire);
                emit((fPrefix + "_r5WireGainVsBaseline").c_str(),
                     allocationBaseScore.totalWire - r5Score.totalWire);
                emit((fPrefix + "_r5ChOverflowDropVsR4").c_str(),
                     r4Score.channelOverflow - r5Score.channelOverflow);
                emit((fPrefix + "_r5ChOverflowDropVsBaseline").c_str(),
                     allocationBaseScore.channelOverflow - r5Score.channelOverflow);
                emit((fPrefix + "_r5FtRequiredDeltaVsR4").c_str(),
                     r5Score.ft.sumRb - r4Score.ft.sumRb);
                emit((fPrefix + "_r5FtRequiredDeltaVsBaseline").c_str(),
                     r5Score.ft.sumRb - allocationBaseScore.ft.sumRb);
                emit((fPrefix + "_allocationReselectPassesRun").c_str(),
                     static_cast<long long>(r2f2Reselect.passes.size()));
                emit((fPrefix + "_allocationReselectTotalChangedPairs").c_str(),
                     allocationReselectChanged);
                emit((fPrefix + "_allocationReselectTotalPassOfficialGain").c_str(),
                     allocationReselectPassGain);
                emit((fPrefix + "_allocationReselectSelectedUsedFeedthrough").c_str(),
                     r2f2Reselect.selectedUsedFeedthrough);
                emit((fPrefix + "_allocationReselectFamilyWins.R0Fallback").c_str(),
                     r2f2Reselect.familyWins[0]);
                emit((fPrefix + "_allocationReselectFamilyWins.Direct").c_str(),
                     r2f2Reselect.familyWins[1]);
                emit((fPrefix + "_allocationReselectFamilyWins.BaseUnifiedShortest").c_str(),
                     r2f2Reselect.familyWins[2]);
                emit((fPrefix + "_allocationReselectFamilyWins.AccessAltUnified").c_str(),
                     r2f2Reselect.familyWins[3]);
                emit((fPrefix + "_allocationReselectFamilyWins.AvoidHotUnified").c_str(),
                     r2f2Reselect.familyWins[4]);
                emit((fPrefix + "_allocationReselectFamilyWins.SegmentBypassUnified").c_str(),
                     r2f2Reselect.familyWins[5]);
                emit((fPrefix + "_allocationReselectFamilyWins.CapacityAwareUnified").c_str(),
                     r2f2Reselect.familyWins[6]);
                for (size_t i = 0; i < r2f2Reselect.passes.size(); ++i) {
                    const AllocationReselectPassStats& ps = r2f2Reselect.passes[i];
                    const std::string pfx = fPrefix + "_allocationReselectPass" +
                                            std::to_string(i + 1);
                    emit((pfx + "ChangedPairs").c_str(), ps.changedPairs);
                    emit((pfx + "KeepWins").c_str(), ps.keepWins);
                    emit((pfx + "BaseSingleWins").c_str(), ps.baseSingleWins);
                    emit((pfx + "SplitWins").c_str(), ps.splitWins);
                    emit((pfx + "AllocationCandidates").c_str(),
                         ps.allocationCandidates);
                    emit((pfx + "OfficialGain").c_str(), ps.officialGain);
                    emit((pfx + "WireGain").c_str(), ps.wireGain);
                    emit((pfx + "MovedNetsSum").c_str(), ps.movedNetsSum);
                    emit((pfx + "ChannelOverflowBefore").c_str(),
                         ps.channelOverflowBefore);
                    emit((pfx + "ChannelOverflowAfter").c_str(),
                         ps.channelOverflowAfter);
                    emit((pfx + "FtRequiredBefore").c_str(), ps.ftRequiredBefore);
                    emit((pfx + "FtRequiredAfter").c_str(), ps.ftRequiredAfter);
                }
                emitHotWorld(fPrefix + "_r5", design, r5Ledger, hotThreshold);
                emitHotWorldOverlap(fPrefix + "_r4ToR5", design, r4Ledger,
                                    r5Ledger, hotThreshold);
                emit((fPrefix + "_allocationReselectPassMonotone").c_str(),
                     std::string(allocationReselectPassMonotone ? "yes" : "no"));
                emit((fPrefix + "_r5MonotoneVsR4").c_str(),
                     std::string(r5Monotone ? "yes" : "no"));
            }
            if (r2g0ResidualOpportunityAuditV0 && allocationReselectEnabled) {
                emitResidualOpportunityAudit(
                    "r2g0", design, pairs, allocationBaseWinners,
                    allocationCandidateSets, r4Allocations, r5Ledger);
            }
        }
        const bool routesComplete = materializeRoutes(
            design, pairs, r4Allocations, allocationBaseWinners);
        return (gateOk && routesComplete) ? 0 : 2;
    }

    routerx::RxResourceLedgerLite world2Ledger = world1Ledger;
    double world2Wire = world1Wire;
    long long avoidHotEligiblePairs = 0;
    long long avoidHotSearchCalls = 0;
    long long avoidHotRawCandidates = 0;
    long long avoidHotDistinctCandidates = 0;
    long long world2RawCandidates = 0;
    long long world2DistinctCandidates = 0;
    long long world2DuplicateCandidates = 0;
    long long world2ChangedPairs = 0;
    long long world2SelectedInvalid = 0;
    long long world2SelectedIllegalFt = 0;
    long long world2SelectedUsedFeedthrough = 0;
    double world2LocalOfficialGainSum = 0.0;
    std::array<long long, routerx::kLiteCandidateFamilyCount> world2FamilyWins{};
    std::array<long long, routerx::kLiteCandidateFamilyCount> world2FamilyRaw{};
    std::array<long long, routerx::kLiteCandidateFamilyCount> world2FamilyDistinct{};
    std::vector<routerx::LiteCandidate> world2Winners(pairs.size());
    std::vector<std::vector<routerx::LiteCandidate>> world2CandidateSets(pairs.size());

    for (size_t i = 0; i < pairs.size(); ++i) {
        const routerx::DemandedPair& p = pairs[i];
        const LiteRoutedPair& r0 = r0Routes[i];
        if (!r0.found || !r0.emitValid) {
            ++missingR0Fallback;
            continue;
        }
        routerx::LiteCandidate current = world1Winners[i];
        if (current.route.steps.empty()) {
            ++missingR0Fallback;
            continue;
        }

        const bool touchesHot = avoidHotEligible(
            design, current, staticCandidateSets[i], world1Hot, eligibilityMode);
        if (touchesHot) ++avoidHotEligiblePairs;
        if (!touchesHot) {
            if (current.usedFeedthrough) ++world2SelectedUsedFeedthrough;
            const int fidx = static_cast<int>(current.family);
            if (fidx >= 0 && fidx < routerx::kLiteCandidateFamilyCount) {
                ++world2FamilyWins[fidx];
            }
            world2Winners[i] = current;
            world2CandidateSets[i] = staticCandidateSets[i];
            continue;
        }

        world2Ledger.ripupSteps(design, current.route.steps, p.nets);
        std::vector<routerx::LiteCandidate> raw;
        raw.reserve(staticCandidateSets[i].size() + weights.size() + 1);
        raw.push_back(current);
        raw.insert(raw.end(), staticCandidateSets[i].begin(), staticCandidateSets[i].end());

        long long localSearchCalls = 0;
        std::vector<routerx::LiteCandidate> hotRaw = generateAvoidHotCandidates(
            design, graph, p, searchCostMode, world1Hot, weights,
            localSearchCalls, avoidHotSearchCalls);
        staticSearchCalls += localSearchCalls;
        avoidHotRawCandidates += static_cast<long long>(hotRaw.size());
        raw.insert(raw.end(), hotRaw.begin(), hotRaw.end());

        for (routerx::LiteCandidate& c : raw) {
            if (c.emitValid) c.impact = world2Ledger.projectSteps(design, c.route.steps, p.nets);
        }
        const routerx::LitePairCandidateReport report =
            routerx::summarizeLiteCandidateSurface(raw, current.impact.wire);
        world2RawCandidates += report.rawCandidates;
        world2DistinctCandidates += report.distinctCandidates;
        world2DuplicateCandidates += report.duplicateCandidates;
        invalidEmitCandidates += report.invalidEmitCandidates;
        illegalFtCandidates += report.illegalFtCandidates;
        for (int f = 0; f < routerx::kLiteCandidateFamilyCount; ++f) {
            world2FamilyRaw[f] += report.familyRaw[f];
            world2FamilyDistinct[f] += report.familyDistinct[f];
        }

        routerx::LiteDedupResult dedup = routerx::dedupLiteCandidates(raw);
        for (routerx::LiteCandidate& c : dedup.candidates) {
            if (c.emitValid) c.impact = world2Ledger.projectSteps(design, c.route.steps, p.nets);
        }
        world2CandidateSets[i] = dedup.candidates;

        int currentIdx = -1;
        int bestIdx = -1;
        for (size_t j = 0; j < dedup.candidates.size(); ++j) {
            const routerx::LiteCandidate& c = dedup.candidates[j];
            if (c.family == routerx::LiteCandidateFamily::AvoidHotUnified) {
                ++avoidHotDistinctCandidates;
            }
            if (c.signature == current.signature && currentIdx < 0) {
                currentIdx = static_cast<int>(j);
            }
            if (!selectableCandidate(c)) continue;
            if (bestIdx < 0) {
                bestIdx = static_cast<int>(j);
                continue;
            }
            const double kc = officialImmediateKey(c, design.alpha);
            const double kb = officialImmediateKey(dedup.candidates[bestIdx], design.alpha);
            if (kc < kb || (kc == kb && candidateTieLess(c, dedup.candidates[bestIdx]))) {
                bestIdx = static_cast<int>(j);
            }
        }
        if (currentIdx < 0) {
            ++missingR0Fallback;
            world2Ledger.commitSteps(design, current.route.steps, p.nets);
            world2Winners[i] = current;
            continue;
        }
        if (bestIdx < 0) bestIdx = currentIdx;

        const double currentKey = officialImmediateKey(dedup.candidates[currentIdx], design.alpha);
        const double bestKey = officialImmediateKey(dedup.candidates[bestIdx], design.alpha);
        if (bestKey > currentKey + 1.0e-6) bestIdx = currentIdx;

        const routerx::LiteCandidate& winner = dedup.candidates[bestIdx];
        const routerx::LiteCandidate& currentCand = dedup.candidates[currentIdx];
        if (!selectableCandidate(winner)) ++world2SelectedInvalid;
        if (winner.impact.dIllegalFtPenalty > 0.0 ||
            winner.impact.illegalFtBlocksAfter > 0) {
            ++world2SelectedIllegalFt;
        }
        if (winner.usedFeedthrough) ++world2SelectedUsedFeedthrough;
        if (winner.signature != currentCand.signature) ++world2ChangedPairs;
        const int fidx = static_cast<int>(winner.family);
        if (fidx >= 0 && fidx < routerx::kLiteCandidateFamilyCount) {
            ++world2FamilyWins[fidx];
        }
        world2LocalOfficialGainSum += currentKey - officialImmediateKey(winner, design.alpha);
        world2Wire += winner.impact.wire - currentCand.impact.wire;
        world2Ledger.commitSteps(design, winner.route.steps, p.nets);
        world2Winners[i] = winner;
    }

    const LiteWorldScore world2Score = scoreLiteWorld(design, world2Ledger, world2Wire);
    const HotAxisRatios world2Hot =
        hotAxisRatios(design, world2Ledger, hotThreshold);
    const SplitTriggerSummary splitTriggers = summarizeSplitTriggers(
        design, pairs, world2Winners, world2CandidateSets, world2Ledger, world2Hot);
    const SplitRatioOracleStats splitRatioStats = splitRatioOracleV0
        ? runSplitRatioOracle(design, pairs, world2Winners, world2CandidateSets,
                              world2Ledger, world2Hot)
        : SplitRatioOracleStats{};
    const Split3WayOracleStats split3WayStats = split3WayOracleV0
        ? runSplit3WayOracle(design, pairs, world2Winners, world2CandidateSets,
                             world2Ledger, world2Hot)
        : Split3WayOracleStats{};

    routerx::RxResourceLedgerLite world3Ledger = world2Ledger;
    double world3Wire = world2Wire;
    const std::vector<SplitPairPlan> splitPlans = (splitV0 && !dynamicSplitV0)
        ? buildHalfSplitPlans(design, pairs, world2Winners, world2CandidateSets,
                              world2Ledger, world2Hot)
        : std::vector<SplitPairPlan>{};
    SplitPassStats splitStats;
    AllocationPoolSelectorStats allocationPoolStats;
    long long avoidHot2EligiblePairs = 0;
    long long avoidHot2SearchCalls = 0;
    long long avoidHot2RawCandidates = 0;
    long long avoidHot2DistinctCandidates = 0;
    long long world3RawCandidates = 0;
    long long world3DistinctCandidates = 0;
    long long world3DuplicateCandidates = 0;
    long long world3ChangedPairs = 0;
    long long world3SelectedInvalid = 0;
    long long world3SelectedIllegalFt = 0;
    long long world3SelectedUsedFeedthrough = 0;
    double world3LocalOfficialGainSum = 0.0;
    std::array<long long, routerx::kLiteCandidateFamilyCount> world3FamilyWins{};
    std::array<long long, routerx::kLiteCandidateFamilyCount> world3FamilyRaw{};
    std::array<long long, routerx::kLiteCandidateFamilyCount> world3FamilyDistinct{};
    if (allocationPoolSelectorV0 || allLoserAllocationPoolV0 ||
        wideSplitRatiosV0 || fullSwitchOnlyV0 || splitOnlyV0) {
        allocationPoolStats = runAllocationPoolSelector(
            design, pairs, world2Winners, world2CandidateSets, world3Ledger,
            world3Wire, hotThreshold, world3FamilyWins,
            /*allPairs=*/allLoserAllocationPoolV0 || wideSplitRatiosV0 ||
                         fullSwitchOnlyV0 || splitOnlyV0,
            /*splitAllLosers=*/allLoserAllocationPoolV0 || wideSplitRatiosV0 ||
                                splitOnlyV0,
            /*wideFixedRatios=*/wideSplitRatiosV0,
            /*allowFullSwitch=*/!splitOnlyV0,
            /*allowSplit=*/!fullSwitchOnlyV0);
        world3ChangedPairs = allocationPoolStats.changedPairs;
        world3SelectedUsedFeedthrough = allocationPoolStats.selectedUsedFeedthrough;
        world3LocalOfficialGainSum = allocationPoolStats.localOfficialGainSum;
    } else if (dynamicSplitV0) {
        world3FamilyWins = world2FamilyWins;
        splitStats = runDynamicHalfSplitPass(
            design, pairs, world2Winners, world2CandidateSets, world3Ledger,
            world3Wire, world2SelectedUsedFeedthrough, hotThreshold,
            maxSplitIterations);
        world3ChangedPairs = splitStats.changedPairs;
        world3SelectedUsedFeedthrough = splitStats.selectedUsedFeedthrough;
        world3LocalOfficialGainSum = splitStats.localOfficialGainSum;
    } else if (splitV0) {
        world3FamilyWins = world2FamilyWins;
        splitStats = runHalfSplitPass(
            design, pairs, splitPlans, world3Ledger, world3Wire,
            world2SelectedUsedFeedthrough);
        world3ChangedPairs = splitStats.changedPairs;
        world3SelectedUsedFeedthrough = splitStats.selectedUsedFeedthrough;
        world3LocalOfficialGainSum = splitStats.localOfficialGainSum;
    } else if (hotPasses >= 2) {
        for (size_t i = 0; i < pairs.size(); ++i) {
            const routerx::DemandedPair& p = pairs[i];
            routerx::LiteCandidate current = world2Winners[i];
            if (current.route.steps.empty()) {
                ++missingR0Fallback;
                continue;
            }

            const bool touchesHot = avoidHotEligible(
                design, current, staticCandidateSets[i], world2Hot, eligibilityMode);
            if (touchesHot) ++avoidHot2EligiblePairs;
            if (!touchesHot) {
                if (current.usedFeedthrough) ++world3SelectedUsedFeedthrough;
                const int fidx = static_cast<int>(current.family);
                if (fidx >= 0 && fidx < routerx::kLiteCandidateFamilyCount) {
                    ++world3FamilyWins[fidx];
                }
                continue;
            }

            world3Ledger.ripupSteps(design, current.route.steps, p.nets);
            std::vector<routerx::LiteCandidate> raw;
            raw.reserve(staticCandidateSets[i].size() + weights.size() + 1);
            raw.push_back(current);
            raw.insert(raw.end(), staticCandidateSets[i].begin(), staticCandidateSets[i].end());

            long long localSearchCalls = 0;
            std::vector<routerx::LiteCandidate> hotRaw = generateAvoidHotCandidates(
                design, graph, p, searchCostMode, world2Hot, weights,
                localSearchCalls, avoidHot2SearchCalls);
            avoidHot2RawCandidates += static_cast<long long>(hotRaw.size());
            raw.insert(raw.end(), hotRaw.begin(), hotRaw.end());

            for (routerx::LiteCandidate& c : raw) {
                if (c.emitValid) c.impact = world3Ledger.projectSteps(design, c.route.steps, p.nets);
            }
            const routerx::LitePairCandidateReport report =
                routerx::summarizeLiteCandidateSurface(raw, current.impact.wire);
            world3RawCandidates += report.rawCandidates;
            world3DistinctCandidates += report.distinctCandidates;
            world3DuplicateCandidates += report.duplicateCandidates;
            invalidEmitCandidates += report.invalidEmitCandidates;
            illegalFtCandidates += report.illegalFtCandidates;
            for (int f = 0; f < routerx::kLiteCandidateFamilyCount; ++f) {
                world3FamilyRaw[f] += report.familyRaw[f];
                world3FamilyDistinct[f] += report.familyDistinct[f];
            }

            routerx::LiteDedupResult dedup = routerx::dedupLiteCandidates(raw);
            for (routerx::LiteCandidate& c : dedup.candidates) {
                if (c.emitValid) c.impact = world3Ledger.projectSteps(design, c.route.steps, p.nets);
            }

            int currentIdx = -1;
            int bestIdx = -1;
            for (size_t j = 0; j < dedup.candidates.size(); ++j) {
                const routerx::LiteCandidate& c = dedup.candidates[j];
                if (c.family == routerx::LiteCandidateFamily::AvoidHotUnified &&
                    c.signature != current.signature) {
                    ++avoidHot2DistinctCandidates;
                }
                if (c.signature == current.signature && currentIdx < 0) {
                    currentIdx = static_cast<int>(j);
                }
                if (!selectableCandidate(c)) continue;
                if (bestIdx < 0) {
                    bestIdx = static_cast<int>(j);
                    continue;
                }
                const double kc = officialImmediateKey(c, design.alpha);
                const double kb = officialImmediateKey(dedup.candidates[bestIdx], design.alpha);
                if (kc < kb || (kc == kb && candidateTieLess(c, dedup.candidates[bestIdx]))) {
                    bestIdx = static_cast<int>(j);
                }
            }
            if (currentIdx < 0) {
                ++missingR0Fallback;
                world3Ledger.commitSteps(design, current.route.steps, p.nets);
                continue;
            }
            if (bestIdx < 0) bestIdx = currentIdx;

            const double currentKey = officialImmediateKey(dedup.candidates[currentIdx], design.alpha);
            const double bestKey = officialImmediateKey(dedup.candidates[bestIdx], design.alpha);
            if (bestKey > currentKey + 1.0e-6) bestIdx = currentIdx;

            const routerx::LiteCandidate& winner = dedup.candidates[bestIdx];
            const routerx::LiteCandidate& currentCand = dedup.candidates[currentIdx];
            if (!selectableCandidate(winner)) ++world3SelectedInvalid;
            if (winner.impact.dIllegalFtPenalty > 0.0 ||
                winner.impact.illegalFtBlocksAfter > 0) {
                ++world3SelectedIllegalFt;
            }
            if (winner.usedFeedthrough) ++world3SelectedUsedFeedthrough;
            if (winner.signature != currentCand.signature) ++world3ChangedPairs;
            const int fidx = static_cast<int>(winner.family);
            if (fidx >= 0 && fidx < routerx::kLiteCandidateFamilyCount) {
                ++world3FamilyWins[fidx];
            }
            world3LocalOfficialGainSum += currentKey - officialImmediateKey(winner, design.alpha);
            world3Wire += winner.impact.wire - currentCand.impact.wire;
            world3Ledger.commitSteps(design, winner.route.steps, p.nets);
        }
    } else {
        world3FamilyWins = world2FamilyWins;
        world3SelectedUsedFeedthrough = world2SelectedUsedFeedthrough;
    }

    const LiteWorldScore world3Score = scoreLiteWorld(design, world3Ledger, world3Wire);
    const auto t1 = std::chrono::steady_clock::now();
    const double runtimeSec =
        std::chrono::duration<double>(t1 - t0).count();

    const bool world1Monotone =
        world1Score.score.totalNoRuntime <= r0Score.score.totalNoRuntime + 1.0e-6;
    const bool world2Monotone =
        world2Score.score.totalNoRuntime <= world1Score.score.totalNoRuntime + 1.0e-6;
    const bool world3Monotone =
        world3Score.score.totalNoRuntime <= world2Score.score.totalNoRuntime + 1.0e-6;
    const bool gateOk = (r0OpenPairs == 0 && r0InvalidEmit == 0 &&
                         missingR0Fallback == 0 && invalidEmitCandidates == 0 &&
                         illegalFtCandidates == 0 && world1SelectedInvalid == 0 &&
                         world1SelectedIllegalFt == 0 && world2SelectedInvalid == 0 &&
                         world2SelectedIllegalFt == 0 && world3SelectedInvalid == 0 &&
                         world3SelectedIllegalFt == 0 && world1Monotone &&
                         world2Monotone && world3Monotone &&
                         world3Score.score.legal);

    emit("lite_mode", std::string(splitOnlyV0 ? "r2d8" :
                                  (fullSwitchOnlyV0 ? "r2d7" :
                                  (wideSplitRatiosV0 ? "r2d6" :
                                  (allLoserAllocationPoolV0 ? "r2d5" :
                                  (allocationPoolSelectorV0 ? "r2d4" :
                                  (split3WayOracleV0 ? "r2d3" :
                                  (splitRatioOracleV0 ? "r2d2" :
                                  (dynamicSplitV0 ? "r2d1" :
                                  (splitV0 ? "r2d0" : "r2c"))))))))));
    emit("lite_searchCostMode", std::string(liteSearchCostModeName(searchCostMode)));
    emitCapacityAwareConfig(capacityConfig);
    emit("lite_routeSpace", std::string(liteRouteSpaceName(routeSpace)));
    emit("r2c_routeSpacePolicy", std::string("force_official_full"));
    emit("r2c_accessAltUniverseMode", std::string(accessAltUniverseModeName(accessMode)));
    emit("r2c_avoidHotEligibility",
         std::string(avoidHotEligibilityModeName(eligibilityMode)));
    emit("r2c_hotThreshold", hotThreshold);
    emit("lite_csv", csvPath);
    emit("lite_cfg", cfgPath);
    emit("lite_alpha", design.alpha);
    emit("lite_outlineArea", design.outlineW * design.outlineH);
    emit("lite_demandedPairCount", static_cast<long long>(pairs.size()));
    emit("lite_totalDemandedNets", totalDemandNets);
    emitLiteGraphReport(graph);
    emit("r2c_selector", std::string("min_official_immediate"));
    emit("r2c_world1", std::string("static_families"));
    emit("r2c_world2", std::string("static_plus_avoid_hot"));
    const std::string world3Name =
        splitOnlyV0 ? "split_only_all_pair_all_loser_allocation_pool_selector_v0" :
        fullSwitchOnlyV0 ? "full_switch_only_all_pair_all_loser_selector_v0" :
        wideSplitRatiosV0 ? "all_pair_all_loser_wide_ratio_allocation_pool_selector_v0" :
        allLoserAllocationPoolV0 ? "all_pair_all_loser_allocation_pool_selector_v0" :
        allocationPoolSelectorV0 ? "allocation_pool_selector_v0" :
        split3WayOracleV0 ? "split_3way_oracle_no_commit" :
        splitRatioOracleV0 ? "split_ratio_oracle_no_commit" :
        dynamicSplitV0 ? "dynamic_half_split_v0" :
        (splitV0 ? "half_split_v0" :
         (hotPasses >= 2 ? "second_avoid_hot_pass" : "same_as_world2"));
    emit("r2c_world3", world3Name);
    emit("r2c_avoidHotWeights", weightsString(weights));
    emit("r2c_avoidHotPasses", static_cast<long long>(hotPasses));
    emit("r2c_r0SearchCalls", r0SearchCalls);
    emit("r2c_staticSearchCalls", staticSearchCalls - avoidHotSearchCalls);
    emit("r2c_altAccessSearchCalls", altAccessSearchCalls);
    emit("r2c_avoidHotEligiblePairs", avoidHotEligiblePairs);
    emit("r2c_avoidHotSearchCalls", avoidHotSearchCalls);
    emit("r2c_avoidHotRawCandidates", avoidHotRawCandidates);
    emit("r2c_avoidHotDistinctCandidates", avoidHotDistinctCandidates);
    emit("r2c_avoidHotPass2EligiblePairs", avoidHot2EligiblePairs);
    emit("r2c_avoidHotPass2SearchCalls", avoidHot2SearchCalls);
    emit("r2c_avoidHotPass2RawCandidates", avoidHot2RawCandidates);
    emit("r2c_avoidHotPass2DistinctCandidates", avoidHot2DistinctCandidates);
    emit("r2c_world1RawCandidates", world1RawCandidates);
    emit("r2c_world1DistinctCandidates", world1DistinctCandidates);
    emit("r2c_world1DuplicateCandidates", world1DuplicateCandidates);
    emit("r2c_world2RawCandidates", world2RawCandidates);
    emit("r2c_world2DistinctCandidates", world2DistinctCandidates);
    emit("r2c_world2DuplicateCandidates", world2DuplicateCandidates);
    emit("r2c_world3RawCandidates", world3RawCandidates);
    emit("r2c_world3DistinctCandidates", world3DistinctCandidates);
    emit("r2c_world3DuplicateCandidates", world3DuplicateCandidates);
    emit("r2c_missingFallback", missingR0Fallback);
    emit("r2c_invalidEmitCandidates", invalidEmitCandidates);
    emit("r2c_illegalFtCandidates", illegalFtCandidates);
    emit("r2c_world1ChangedPairsVsR0", world1ChangedPairs);
    emit("r2c_world2ChangedPairsVsWorld1", world2ChangedPairs);
    emit("r2c_world3ChangedPairsVsWorld2", world3ChangedPairs);
    emit("r2c_world1SelectedUsedFeedthrough", world1SelectedUsedFeedthrough);
    emit("r2c_world2SelectedUsedFeedthrough", world2SelectedUsedFeedthrough);
    emit("r2c_world3SelectedUsedFeedthrough", world3SelectedUsedFeedthrough);
    emitLiteWorldScore("r2c_r0", r0Score);
    emitLiteWorldScore("r2c_world1", world1Score);
    emitLiteWorldScore("r2c_world2", world2Score);
    emitLiteWorldScore("r2c_world3", world3Score);
    emit("r2c_world1OfficialGainVsR0",
         r0Score.score.totalNoRuntime - world1Score.score.totalNoRuntime);
    emit("r2c_world2OfficialGainVsR0",
         r0Score.score.totalNoRuntime - world2Score.score.totalNoRuntime);
    emit("r2c_world2OfficialGainVsWorld1",
         world1Score.score.totalNoRuntime - world2Score.score.totalNoRuntime);
    emit("r2c_world3OfficialGainVsWorld2",
         world2Score.score.totalNoRuntime - world3Score.score.totalNoRuntime);
    emit("r2c_world3OfficialGainVsWorld1",
         world1Score.score.totalNoRuntime - world3Score.score.totalNoRuntime);
    emit("r2c_world1WireGainVsR0", r0Score.totalWire - world1Score.totalWire);
    emit("r2c_world2WireGainVsWorld1", world1Score.totalWire - world2Score.totalWire);
    emit("r2c_world3WireGainVsWorld2", world2Score.totalWire - world3Score.totalWire);
    emit("r2c_world2LocalOfficialGainSum", world2LocalOfficialGainSum);
    emit("r2c_world3LocalOfficialGainSum", world3LocalOfficialGainSum);
    emit("r2c_world1FamilyWins.R0Fallback", world1FamilyWins[0]);
    emit("r2c_world1FamilyWins.Direct", world1FamilyWins[1]);
    emit("r2c_world1FamilyWins.BaseUnifiedShortest", world1FamilyWins[2]);
    emit("r2c_world1FamilyWins.AccessAltUnified", world1FamilyWins[3]);
    emit("r2c_world1FamilyWins.AvoidHotUnified", world1FamilyWins[4]);
    emit("r2c_world1FamilyWins.CapacityAwareUnified", world1FamilyWins[6]);
    emit("r2c_world2FamilyWins.R0Fallback", world2FamilyWins[0]);
    emit("r2c_world2FamilyWins.Direct", world2FamilyWins[1]);
    emit("r2c_world2FamilyWins.BaseUnifiedShortest", world2FamilyWins[2]);
    emit("r2c_world2FamilyWins.AccessAltUnified", world2FamilyWins[3]);
    emit("r2c_world2FamilyWins.AvoidHotUnified", world2FamilyWins[4]);
    emit("r2c_world2FamilyWins.CapacityAwareUnified", world2FamilyWins[6]);
    emit("r2c_world3FamilyWins.R0Fallback", world3FamilyWins[0]);
    emit("r2c_world3FamilyWins.Direct", world3FamilyWins[1]);
    emit("r2c_world3FamilyWins.BaseUnifiedShortest", world3FamilyWins[2]);
    emit("r2c_world3FamilyWins.AccessAltUnified", world3FamilyWins[3]);
    emit("r2c_world3FamilyWins.AvoidHotUnified", world3FamilyWins[4]);
    emit("r2c_world3FamilyWins.CapacityAwareUnified", world3FamilyWins[6]);
    emit("r2c_world2FamilyRaw.AvoidHotUnified", world2FamilyRaw[4]);
    emit("r2c_world2FamilyDistinct.AvoidHotUnified", world2FamilyDistinct[4]);
    emit("r2c_world3FamilyRaw.AvoidHotUnified", world3FamilyRaw[4]);
    emit("r2c_world3FamilyDistinct.AvoidHotUnified", world3FamilyDistinct[4]);
    emit("r2c_splitTrigger1WinnerTouchesHot", splitTriggers.winnerTouchesHot);
    emit("r2c_splitTrigger2HasReliefAlternative",
         splitTriggers.hasReliefAlternative);
    emit("r2c_splitTrigger3UsefulSplittable", splitTriggers.usefulSplittable);
    emit("r2c_splitTriggerHotDemandWinnerSum",
         splitTriggers.hotDemandWinnerSum);
    emit("r2c_splitTriggerMaxUsefulReliefSum",
         splitTriggers.maxUsefulReliefSum);
    emit("r2d0_splitPolicy", std::string(
        dynamicSplitV0 ? "dynamic_best_gain_winner_bestB_half" :
        (splitV0 ? "winner_bestB_half" : "off")));
    emit("r2d0_splitMaxIterations", static_cast<long long>(maxSplitIterations));
    emit("r2d0_splitEligiblePairs", splitStats.eligiblePairs);
    emit("r2d0_splitAllocationCandidates", splitStats.allocationCandidates);
    emit("r2d0_splitKeepWins", splitStats.keepWins);
    emit("r2d0_splitFullSwitchWins", splitStats.fullSwitchWins);
    emit("r2d0_splitHalfWins", splitStats.halfSplitWins);
    emit("r2d0_splitChangedPairs", splitStats.changedPairs);
    emit("r2d0_splitLocalOfficialGainSum", splitStats.localOfficialGainSum);
    emit("r2d0_splitLocalWireDeltaSum", splitStats.localWireDeltaSum);
    emit("r2d2_splitOraclePolicy",
         std::string("clear_smallest_hot,clear_largest_hot,quarter,half"));
    emit("r2d2_splitOracleWinnerTouchesHot", splitRatioStats.winnerTouchesHot);
    emit("r2d2_splitOraclePairsWithReliefCandidate",
         splitRatioStats.pairsWithReliefCandidate);
    emit("r2d2_splitOracleReliefCandidates", splitRatioStats.reliefCandidates);
    emit("r2d2_splitOracleAllocationCandidates",
         splitRatioStats.allocationCandidates);
    emit("r2d2_splitOracleBestPositivePairs",
         splitRatioStats.bestPositivePairs);
    emit("r2d2_splitOracleBestGainSum", splitRatioStats.bestGainSum);
    emit("r2d2_splitOracleBestWireDeltaSum",
         splitRatioStats.bestWireDeltaSum);
    emit("r2d2_splitOracleBestMovedNetsSum",
         splitRatioStats.bestMovedNetsSum);
    emit("r2d2_splitOracleBestFullSwitchWins",
         splitRatioStats.bestFullSwitchWins);
    emit("r2d2_splitOracleBestPartialSplitWins",
         splitRatioStats.bestPartialSplitWins);
    for (int k = 0; k < kSplitRatioKindCount; ++k) {
        const SplitRatioKind kind = static_cast<SplitRatioKind>(k);
        const std::string pfx = splitRatioPrefix(kind);
        emit((pfx + "Candidates").c_str(), splitRatioStats.kindCandidates[k]);
        emit((pfx + "PositiveCandidates").c_str(),
             splitRatioStats.kindPositiveCandidates[k]);
        emit((pfx + "PositivePairs").c_str(),
             splitRatioStats.kindPositivePairs[k]);
        emit((pfx + "BestGainSum").c_str(),
             splitRatioStats.kindBestGainSum[k]);
        emit((pfx + "BestWireDeltaSum").c_str(),
             splitRatioStats.kindBestWireDeltaSum[k]);
        emit((pfx + "BestMovedNetsSum").c_str(),
             splitRatioStats.kindMovedNetsSum[k]);
        emit((pfx + "BestFullSwitchWins").c_str(),
             splitRatioStats.kindBestFullSwitchWins[k]);
        emit((pfx + "BestPartialSplitWins").c_str(),
             splitRatioStats.kindBestPartialSplitWins[k]);
        emit((pfx + "OracleBestWins").c_str(),
             splitRatioStats.bestRatioWins[k]);
    }
    emit("r2d3_splitOraclePolicy",
         std::string("top2_relief_paths:clear_smallest_hot,clear_largest_hot,quarter,half"));
    emit("r2d3_splitOracleWinnerTouchesHot", split3WayStats.winnerTouchesHot);
    emit("r2d3_splitOraclePairsWithTwoReliefCandidates",
         split3WayStats.pairsWithTwoReliefCandidates);
    emit("r2d3_splitOracleReliefCandidates", split3WayStats.reliefCandidates);
    emit("r2d3_splitOracleAllocationCandidates",
         split3WayStats.allocationCandidates);
    emit("r2d3_splitOracleBestPositivePairs",
         split3WayStats.bestPositivePairs);
    emit("r2d3_splitOracleBestGainSum", split3WayStats.bestGainSum);
    emit("r2d3_splitOracleBestWireDeltaSum",
         split3WayStats.bestWireDeltaSum);
    emit("r2d3_splitOracleBestMovedBNetsSum",
         split3WayStats.bestMovedBNetsSum);
    emit("r2d3_splitOracleBestMovedCNetsSum",
         split3WayStats.bestMovedCNetsSum);
    emit("r2d3_splitOracleBestUsesBothAlternatives",
         split3WayStats.bestUsesBothAlternatives);
    emit("r2d3_splitOracleBestLeavesOnCurrent",
         split3WayStats.bestLeavesOnCurrent);
    for (int k = 0; k < kSplitRatioKindCount; ++k) {
        const SplitRatioKind kind = static_cast<SplitRatioKind>(k);
        const std::string pfx = std::string("r2d3_") + splitRatioKindName(kind);
        emit((pfx + "Candidates").c_str(), split3WayStats.kindCandidates[k]);
        emit((pfx + "PositiveCandidates").c_str(),
             split3WayStats.kindPositiveCandidates[k]);
        emit((pfx + "PositivePairs").c_str(),
             split3WayStats.kindPositivePairs[k]);
        emit((pfx + "BestGainSum").c_str(),
             split3WayStats.kindBestGainSum[k]);
        emit((pfx + "BestWireDeltaSum").c_str(),
             split3WayStats.kindBestWireDeltaSum[k]);
        emit((pfx + "MovedBNetsSum").c_str(),
             split3WayStats.kindMovedBNetsSum[k]);
        emit((pfx + "MovedCNetsSum").c_str(),
             split3WayStats.kindMovedCNetsSum[k]);
        emit((pfx + "BestUsesBothAlternatives").c_str(),
             split3WayStats.kindBestUsesBothAlternatives[k]);
        emit((pfx + "OracleBestWins").c_str(),
             split3WayStats.bestRatioWins[k]);
    }
    const std::string allocPrefix =
        splitOnlyV0 ? "r2d8" :
        fullSwitchOnlyV0 ? "r2d7" :
        wideSplitRatiosV0 ? "r2d6" :
        (allLoserAllocationPoolV0 ? "r2d5" : "r2d4");
    emit((allocPrefix + "_allocationPolicy").c_str(),
         splitOnlyV0
             ? std::string("all_pairs:split_only_all_loser_splits:clear_smallest_hot,clear_largest_hot,ten_percent,quarter")
             : fullSwitchOnlyV0
             ? std::string("all_pairs:full_switch_only_all_losers")
             : wideSplitRatiosV0
             ? std::string("all_pairs:all_loser_full_switch_plus_all_loser_splits:clear_smallest_hot,clear_largest_hot,5,10,15,20,25,33_percent")
             : allLoserAllocationPoolV0
             ? std::string("all_pairs:all_loser_full_switch_plus_all_loser_splits:clear_smallest_hot,clear_largest_hot,ten_percent,quarter")
             : std::string("hot_winner_pairs:all_loser_full_switch_plus_relief_splits:clear_smallest_hot,clear_largest_hot,ten_percent,quarter"));
    emit((allocPrefix + "_allocationEligiblePairs").c_str(),
         allocationPoolStats.eligiblePairs);
    emit((allocPrefix + "_loserCandidates").c_str(),
         allocationPoolStats.loserCandidates);
    emit((allocPrefix + "_reliefLoserCandidates").c_str(),
         allocationPoolStats.reliefLoserCandidates);
    emit((allocPrefix + "_allocationCandidates").c_str(),
         allocationPoolStats.allocationCandidates);
    emit((allocPrefix + "_keepWins").c_str(), allocationPoolStats.keepWins);
    emit((allocPrefix + "_fullSwitchWins").c_str(),
         allocationPoolStats.fullSwitchWins);
    emit((allocPrefix + "_splitWins").c_str(), allocationPoolStats.splitWins);
    emit((allocPrefix + "_changedPairs").c_str(),
         allocationPoolStats.changedPairs);
    emit((allocPrefix + "_selectedUsedFeedthrough").c_str(),
         allocationPoolStats.selectedUsedFeedthrough);
    emit((allocPrefix + "_localOfficialGainSum").c_str(),
         allocationPoolStats.localOfficialGainSum);
    emit((allocPrefix + "_localWireDeltaSum").c_str(),
         allocationPoolStats.localWireDeltaSum);
    emit((allocPrefix + "_fullSwitchOfficialGainSum").c_str(),
         allocationPoolStats.fullSwitchOfficialGainSum);
    emit((allocPrefix + "_fullSwitchWireDeltaSum").c_str(),
         allocationPoolStats.fullSwitchWireDeltaSum);
    emit((allocPrefix + "_splitOfficialGainSum").c_str(),
         allocationPoolStats.splitOfficialGainSum);
    emit((allocPrefix + "_splitWireDeltaSum").c_str(),
         allocationPoolStats.splitWireDeltaSum);
    emit((allocPrefix + "_movedNetsSum").c_str(),
         allocationPoolStats.movedNetsSum);
    for (int k = 0; k < kAllocationSplitPolicyCount; ++k) {
        const AllocationSplitPolicy policy =
            static_cast<AllocationSplitPolicy>(k);
        const std::string pfx = allocPrefix + "_" +
                                allocationSplitPolicyName(policy);
        emit((pfx + "Candidates").c_str(),
             allocationPoolStats.splitPolicyCandidates[k]);
        emit((pfx + "Wins").c_str(),
             allocationPoolStats.splitPolicyWins[k]);
    }
    emitHotWorld("r2c_r0", design, r0Ledger, hotThreshold);
    emitHotWorld("r2c_world1", design, world1Ledger, hotThreshold);
    emitHotWorld("r2c_world2", design, world2Ledger, hotThreshold);
    emitHotWorld("r2c_world3", design, world3Ledger, hotThreshold);
    emitHotWorldOverlap("r2c_r0ToWorld1", design, r0Ledger,
                        world1Ledger, hotThreshold);
    emitHotWorldOverlap("r2c_world1ToWorld2", design, world1Ledger,
                        world2Ledger, hotThreshold);
    emitHotWorldOverlap("r2c_world2ToWorld3", design, world2Ledger,
                        world3Ledger, hotThreshold);
    emit("r2c_world1MonotoneVsR0", std::string(world1Monotone ? "yes" : "no"));
    emit("r2c_world2MonotoneVsWorld1", std::string(world2Monotone ? "yes" : "no"));
    emit("r2c_world3MonotoneVsWorld2", std::string(world3Monotone ? "yes" : "no"));
    emit("r2c_runtimeSec", runtimeSec);
    emit("r2c_gate", std::string(gateOk ? "ok" : "FAIL"));
    return gateOk ? 0 : 2;
}

#if 0
int runR2b2(const std::string& selectorName,
            const std::string& csvPath,
            const std::string& cfgPath) {
    routerx::SearchCostMode searchCostMode;
    if (!parseLiteSearchCostMode(searchCostMode)) return 1;
    routerx::SoftRoutingPolicy routeSpace;
    if (!parseLiteRouteSpace(routeSpace)) return 1;
    AccessAltUniverseMode accessMode;
    if (!parseAccessAltUniverseMode(accessMode)) return 1;
    CapacityAwareConfig capacityConfig;
    if (!parseCapacityAwareConfig(capacityConfig)) return 1;

    LiteSelectorMode mode;
    if (!parseLiteSelectorMode(selectorName, mode)) {
        std::cerr << "[rx_lite] unknown R2b2 selector '" << selectorName << "'\n";
        return 1;
    }

    ParsedPlacement parsed = loadQ43Design(csvPath, cfgPath);
    if (!parsed.ok) return 1;
    Design& design = parsed.design;

    routerx::RxGraph graph;
    graph.build(design, routeSpace);

    const std::vector<routerx::DemandedPair> pairs = routerx::demandedPairs(design);
    std::vector<LiteRoutedPair> r0Routes;
    r0Routes.reserve(pairs.size());

    routerx::RxResourceLedgerLite ledger;
    ledger.build(design);

    long long totalDemandNets = 0;
    long long r0SearchCalls = 0;
    long long r0OpenPairs = 0;
    long long r0InvalidEmit = 0;
    double r0TotalWire = 0.0;

    // Build the full R0 world first. Selector policies then replace one pair at
    // a time against a fully occupied design instead of routing from an empty
    // ledger, which keeps the marginal penalty context honest.
    for (const routerx::DemandedPair& p : pairs) {
        totalDemandNets += p.nets;
        LiteRoutedPair r0 = routeR0Fallback(design, graph, p, searchCostMode);
        r0SearchCalls += r0.searchCalls;
        if (!r0.found || !r0.emitValid) {
            if (!r0.found) ++r0OpenPairs;
            if (!r0.emitValid) ++r0InvalidEmit;
            r0Routes.push_back(std::move(r0));
            continue;
        }
        const routerx::LiteRouteImpact impact =
            ledger.projectSteps(design, r0.route.steps, p.nets);
        r0TotalWire += impact.wire;
        ledger.commitSteps(design, r0.route.steps, p.nets);
        r0Routes.push_back(std::move(r0));
    }

    const double outlineArea = design.outlineW * design.outlineH;
    const double r0ChOverflow = ledger.totalChannelOverflow();
    const double r0ChCapacity = ledger.totalChannelCapacity();
    const rxscore::FtResult r0Ft = ledger.ftResult(design);
    rxscore::Inputs r0ScoreIn;
    r0ScoreIn.alpha = design.alpha;
    r0ScoreIn.outlineArea = outlineArea;
    r0ScoreIn.totalWireLength = r0TotalWire;
    r0ScoreIn.channelOverflowTotal = r0ChOverflow;
    r0ScoreIn.channelCapacityTotal = r0ChCapacity;
    r0ScoreIn.runtimeSec = 0.0;
    r0ScoreIn.blocks = ledger.blockFtInputs(design);
    const rxscore::Breakdown r0Score = rxscore::score(r0ScoreIn);
    const routerx::RxResourceLedgerLite r0WorldLedger = ledger;

    long long candidateSearchCalls = 0;
    long long altAccessSearchCalls = 0;
    long long rawCandidates = 0;
    long long distinctCandidates = 0;
    long long duplicateCandidates = 0;
    long long missingR0Fallback = 0;
    long long invalidEmitCandidates = 0;
    long long illegalFtCandidates = 0;
    long long changedPairs = 0;
    long long selectedInvalid = 0;
    long long selectedIllegalFt = 0;
    long long selectedUsedFeedthrough = 0;
    std::array<long long, routerx::kLiteCandidateFamilyCount> familyWins{};
    std::array<long long, routerx::kLiteCandidateFamilyCount> familyRaw{};
    std::array<long long, routerx::kLiteCandidateFamilyCount> familyDistinct{};
    double selectedTotalWire = r0TotalWire;
    double localOfficialGainSum = 0.0;
    double localWireGainSum = 0.0;

    for (size_t i = 0; i < pairs.size(); ++i) {
        const routerx::DemandedPair& p = pairs[i];
        const LiteRoutedPair& r0 = r0Routes[i];
        if (!r0.found || !r0.emitValid) {
            ++missingR0Fallback;
            continue;
        }

        // Replace semantics: remove this pair's R0 route, score alternatives in
        // the current full-world context, then commit exactly one winner.
        ledger.ripupSteps(design, r0.route.steps, p.nets);
        std::vector<routerx::LiteCandidate> raw =
            generateR2b1Candidates(design, graph, p, r0, searchCostMode, accessMode,
                                   capacityConfig,
                                   candidateSearchCalls, altAccessSearchCalls);
        const double r0Wire = ledger.projectSteps(design, r0.route.steps, p.nets).wire;
        for (routerx::LiteCandidate& c : raw) {
            if (c.emitValid) c.impact = ledger.projectSteps(design, c.route.steps, p.nets);
        }

        const routerx::LitePairCandidateReport report =
            routerx::summarizeLiteCandidateSurface(raw, r0Wire);
        rawCandidates += report.rawCandidates;
        distinctCandidates += report.distinctCandidates;
        duplicateCandidates += report.duplicateCandidates;
        missingR0Fallback += report.missingR0Fallback;
        invalidEmitCandidates += report.invalidEmitCandidates;
        illegalFtCandidates += report.illegalFtCandidates;
        for (int f = 0; f < routerx::kLiteCandidateFamilyCount; ++f) {
            familyRaw[f] += report.familyRaw[f];
            familyDistinct[f] += report.familyDistinct[f];
        }

        routerx::LiteDedupResult dedup = routerx::dedupLiteCandidates(raw);
        for (routerx::LiteCandidate& c : dedup.candidates) {
            if (c.emitValid) c.impact = ledger.projectSteps(design, c.route.steps, p.nets);
        }

        const int r0Idx = findR0FallbackCandidate(dedup.candidates);
        const int chosenIdx = chooseLiteCandidate(dedup.candidates, mode, r0Wire, design.alpha);
        if (r0Idx < 0 || chosenIdx < 0) {
            ++missingR0Fallback;
            ledger.commitSteps(design, r0.route.steps, p.nets);
            continue;
        }

        const routerx::LiteCandidate& r0Cand = dedup.candidates[r0Idx];
        const routerx::LiteCandidate& winner = dedup.candidates[chosenIdx];
        if (!selectableCandidate(winner)) ++selectedInvalid;
        if (winner.impact.dIllegalFtPenalty > 0.0 ||
            winner.impact.illegalFtBlocksAfter > 0) {
            ++selectedIllegalFt;
        }
        if (winner.usedFeedthrough) ++selectedUsedFeedthrough;

        selectedTotalWire += winner.impact.wire - r0Cand.impact.wire;
        localWireGainSum += r0Cand.impact.wire - winner.impact.wire;
        localOfficialGainSum +=
            officialImmediateKey(r0Cand, design.alpha) -
            officialImmediateKey(winner, design.alpha);

        if (winner.signature != r0Cand.signature) ++changedPairs;
        const int fidx = static_cast<int>(winner.family);
        if (fidx >= 0 && fidx < routerx::kLiteCandidateFamilyCount) ++familyWins[fidx];
        ledger.commitSteps(design, winner.route.steps, p.nets);
    }

    const double finalChOverflow = ledger.totalChannelOverflow();
    const double finalChCapacity = ledger.totalChannelCapacity();
    const rxscore::FtResult finalFt = ledger.ftResult(design);
    rxscore::Inputs finalScoreIn;
    finalScoreIn.alpha = design.alpha;
    finalScoreIn.outlineArea = outlineArea;
    finalScoreIn.totalWireLength = selectedTotalWire;
    finalScoreIn.channelOverflowTotal = finalChOverflow;
    finalScoreIn.channelCapacityTotal = finalChCapacity;
    finalScoreIn.runtimeSec = 0.0;
    finalScoreIn.blocks = ledger.blockFtInputs(design);
    const rxscore::Breakdown finalScore = rxscore::score(finalScoreIn);

    const double officialGain = r0Score.totalNoRuntime - finalScore.totalNoRuntime;
    const double wireGain = r0TotalWire - selectedTotalWire;
    const bool rollbackToR0 = finalScore.totalNoRuntime > r0Score.totalNoRuntime + 1e-6;
    const bool minOfficialMonotone =
        mode != LiteSelectorMode::MinOfficialImmediate ||
        finalScore.totalNoRuntime <= r0Score.totalNoRuntime + 1e-6;
    const bool gateOk = (r0OpenPairs == 0 && r0InvalidEmit == 0 &&
                         missingR0Fallback == 0 && invalidEmitCandidates == 0 &&
                         illegalFtCandidates == 0 && selectedInvalid == 0 &&
                         selectedIllegalFt == 0 && minOfficialMonotone &&
                         finalScore.legal);

    emit("lite_mode", std::string("r2b2"));
    emit("lite_searchCostMode", std::string(liteSearchCostModeName(searchCostMode)));
    emit("r2b2_accessAltUniverseMode",
         std::string(accessAltUniverseModeName(accessMode)));
    emitCapacityAwareConfig(capacityConfig);
    emit("lite_csv", csvPath);
    emit("lite_cfg", cfgPath);
    emit("lite_alpha", design.alpha);
    emit("lite_outlineArea", outlineArea);
    emit("lite_demandedPairCount", static_cast<long long>(pairs.size()));
    emit("lite_totalDemandedNets", totalDemandNets);
    emitLiteGraphReport(graph);
    emit("r2b2_selector", std::string(liteSelectorModeName(mode)));
    emit("r2b2_startState", std::string("r0_world"));
    emit("r2b2_projectionSemantics", std::string("ripup_replace_commit"));
    emit("r2b2_r0OpenPairs", r0OpenPairs);
    emit("r2b2_r0InvalidEmit", r0InvalidEmit);
    emit("r2b2_r0SearchCalls", r0SearchCalls);
    emit("r2b2_candidateSearchCalls", candidateSearchCalls);
    emit("r2b2_altAccessSearchCalls", altAccessSearchCalls);
    emit("r2b2_rawCandidates", rawCandidates);
    emit("r2b2_distinctCandidates", distinctCandidates);
    emit("r2b2_duplicateCandidates", duplicateCandidates);
    emit("r2b2_missingR0Fallback", missingR0Fallback);
    emit("r2b2_invalidEmitCandidates", invalidEmitCandidates);
    emit("r2b2_illegalFtCandidates", illegalFtCandidates);
    emit("r2b2_selectedInvalid", selectedInvalid);
    emit("r2b2_selectedIllegalFt", selectedIllegalFt);
    emit("r2b2_changedPairs", changedPairs);
    emit("r2b2_selectedUsedFeedthrough", selectedUsedFeedthrough);
    emit("r2b2_localWireGainSum", localWireGainSum);
    emit("r2b2_localOfficialGainSum", localOfficialGainSum);
    emit("r2b2_r0Wire", r0TotalWire);
    emit("r2b2_finalWire", selectedTotalWire);
    emit("r2b2_wireGain", wireGain);
    emit("r2b2_r0PartialCostNoEdgeNoRt", r0Score.totalNoRuntime);
    emit("r2b2_finalPartialCostNoEdgeNoRt", finalScore.totalNoRuntime);
    emit("r2b2_officialGain", officialGain);
    emit("r2b2_r0ChannelOverflowTotal", r0ChOverflow);
    emit("r2b2_finalChannelOverflowTotal", finalChOverflow);
    emit("r2b2_r0ChannelOverflowRate",
         r0ChCapacity > 0.0 ? r0ChOverflow / r0ChCapacity : 0.0);
    emit("r2b2_finalChannelOverflowRate",
         finalChCapacity > 0.0 ? finalChOverflow / finalChCapacity : 0.0);
    emit("r2b2_r0FtPenalty", r0Ft.penalty);
    emit("r2b2_finalFtPenalty", finalFt.penalty);
    emit("r2b2_familyWins.R0Fallback", familyWins[0]);
    emit("r2b2_familyWins.Direct", familyWins[1]);
    emit("r2b2_familyWins.BaseUnifiedShortest", familyWins[2]);
    emit("r2b2_familyWins.AccessAltUnified", familyWins[3]);
    emit("r2b2_familyWins.AvoidHotUnified", familyWins[4]);
    emit("r2b2_familyWins.CapacityAwareUnified", familyWins[6]);
    emit("r2b2_familyRaw.R0Fallback", familyRaw[0]);
    emit("r2b2_familyRaw.Direct", familyRaw[1]);
    emit("r2b2_familyRaw.BaseUnifiedShortest", familyRaw[2]);
    emit("r2b2_familyRaw.AccessAltUnified", familyRaw[3]);
    emit("r2b2_familyRaw.AvoidHotUnified", familyRaw[4]);
    emit("r2b2_familyRaw.CapacityAwareUnified", familyRaw[6]);
    emit("r2b2_familyDistinct.R0Fallback", familyDistinct[0]);
    emit("r2b2_familyDistinct.Direct", familyDistinct[1]);
    emit("r2b2_familyDistinct.BaseUnifiedShortest", familyDistinct[2]);
    emit("r2b2_familyDistinct.AccessAltUnified", familyDistinct[3]);
    emit("r2b2_familyDistinct.AvoidHotUnified", familyDistinct[4]);
    emit("r2b2_familyDistinct.CapacityAwareUnified", familyDistinct[6]);
    emit("r2b2_minOfficialMonotone", std::string(minOfficialMonotone ? "yes" : "no"));
    emit("r2b2_rollbackToR0", std::string(rollbackToR0 ? "yes" : "no"));
    emit("r2b2_gate", std::string(gateOk ? "ok" : "FAIL"));
    if (hotReportEnabled()) {
        emitHotWorld("r2hot_r0", design, r0WorldLedger);
        emitHotWorld("r2hot_final", design, ledger);
        const std::set<std::string> r0Hot = hotChannelAxisKeySet(design, r0WorldLedger);
        const std::set<std::string> finalHot = hotChannelAxisKeySet(design, ledger);
        const long long hotOverlap = intersectionCount(r0Hot, finalHot);
        emit("r2hot_hotChannelAxisOverlap", hotOverlap);
        emit("r2hot_hotChannelAxisR0Only",
             static_cast<long long>(r0Hot.size()) - hotOverlap);
        emit("r2hot_hotChannelAxisFinalOnly",
             static_cast<long long>(finalHot.size()) - hotOverlap);

        const std::set<std::string> r0Soft = activeSoftKeySet(design, r0WorldLedger);
        const std::set<std::string> finalSoft = activeSoftKeySet(design, ledger);
        const long long softOverlap = intersectionCount(r0Soft, finalSoft);
        emit("r2hot_activeSoftOverlap", softOverlap);
        emit("r2hot_activeSoftR0Only",
             static_cast<long long>(r0Soft.size()) - softOverlap);
        emit("r2hot_activeSoftFinalOnly",
             static_cast<long long>(finalSoft.size()) - softOverlap);
    }
    return gateOk ? 0 : 2;
}

} // namespace

int main(int argc, char** argv) {
    std::vector<std::string> args(argv + 1, argv + argc);
    const auto runR2f6Mode = [&]() {
        return runR2c(args[1], args[2],
                      false, false, false, false, false,
                      false, false, false, false, false,
                      false, false, false, false, false,
                      false, false, false, true);
    };
    const auto runR2s2Baseline = [&]() {
        return runR2c(args[1], args[2],
                      false, false, false, false, false,
                      false, false, false, false, false,
                      false, false, false, false, false,
                      false, false, false, false, false,
                      false, true);
    };
    const auto runR2g0Audit = [&]() {
        return runR2c(args[1], args[2],
                      false, false, false, false, false,
                      false, false, false, false, false,
                      false, false, false, false, false,
                      false, false, false, false, false,
                      false, true, true);
    };
    if (args.size() == 2 && args[0] == "--declared") return runDeclared(args[1]);
    if (args.size() == 3 && args[0] == "--calib")     return runCalib(args[1], args[2]);
    if (args.size() == 3 && args[0] == "--r0")        return runR0(args[1], args[2]);
    if (args.size() == 3 && args[0] == "--r2b1")      return runR2b1(args[1], args[2]);
    if (args.size() == 3 && args[0] == "--r2c")       return runR2c(args[1], args[2]);
    if (args.size() == 3 && args[0] == "--r2d0")      return runR2c(args[1], args[2], true);
    if (args.size() == 3 && args[0] == "--r2d1")      return runR2c(args[1], args[2], true, true);
    if (args.size() == 3 && args[0] == "--r2d2")      return runR2c(args[1], args[2], false, false, true);
    if (args.size() == 3 && args[0] == "--r2d3")      return runR2c(args[1], args[2], false, false, false, true);
    if (args.size() == 3 && args[0] == "--r2d4")      return runR2c(args[1], args[2], false, false, false, false, true);
    if (args.size() == 3 && args[0] == "--r2d5")      return runR2c(args[1], args[2], false, false, false, false, false, true);
    if (args.size() == 3 && args[0] == "--r2d6")      return runR2c(args[1], args[2], false, false, false, false, false, false, true);
    if (args.size() == 3 && args[0] == "--r2d7")      return runR2c(args[1], args[2], false, false, false, false, false, false, false, true);
    if (args.size() == 3 && args[0] == "--r2d8")      return runR2c(args[1], args[2], false, false, false, false, false, false, false, false, true);
    if (args.size() == 3 && args[0] == "--r2e0")      return runR2c(args[1], args[2], false, false, false, false, false, false, false, false, false, true);
    if (args.size() == 3 && args[0] == "--r2e1")      return runR2c(args[1], args[2], false, false, false, false, false, false, false, false, false, false, true, false);
    if (args.size() == 3 && args[0] == "--r2e2")      return runR2c(args[1], args[2], false, false, false, false, false, false, false, false, false, false, false, true);
    if (args.size() == 3 && args[0] == "--r2f0")      return runR2c(args[1], args[2], false, false, false, false, false, false, false, false, false, false, false, false, true);
    if (args.size() == 3 && args[0] == "--r2f1")      return runR2c(args[1], args[2], false, false, false, false, false, false, false, false, false, false, false, false, false, true);
    if (args.size() == 3 && args[0] == "--r2f2")      return runR2c(args[1], args[2], false, false, false, false, false, false, false, false, false, false, false, false, false, false, true);
    if (args.size() == 3 && args[0] == "--r2f3")      return runR2c(args[1], args[2], false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, true);
    if (args.size() == 3 && args[0] == "--r2f4")      return runR2c(args[1], args[2], false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, true);
    if (args.size() == 3 && args[0] == "--r2f5")      return runR2c(args[1], args[2], false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, true);
    if (args.size() == 3 && args[0] == "--r2f6") {
        return runR2f6Mode();
    }
    if (args.size() == 3 && (args[0] == "--r2baseline" ||
                             args[0] == "--baseline")) {
        return runR2s2Baseline();
    }
    if (args.size() == 3 && args[0] == "--r2s0") {
        return runR2c(args[1], args[2],
                      false, false, false, false, false,
                      false, false, false, false, false,
                      false, false, false, false, false,
                      false, false, false, false, true);
    }
    if (args.size() == 3 && args[0] == "--r2s1") {
        return runR2c(args[1], args[2],
                      false, false, false, false, false,
                      false, false, false, false, false,
                      false, false, false, false, false,
                      false, false, false, false, false, true);
    }
    if (args.size() == 3 && args[0] == "--r2s2") {
        return runR2s2Baseline();
    }
    if (args.size() == 3 && args[0] == "--r2g0") {
        return runR2g0Audit();
    }
    if (args.size() == 4 && args[0] == "--r2b2")      return runR2b2(args[1], args[2], args[3]);
    if (args.size() == 2)                             return runQ43(args[0], args[1]);
    std::cerr << "usage:\n"
              << "  rx_lite <case.csv> <placement.cfg>   # q43 report\n"
              << "  rx_lite --declared <submission.cfg>  # outline+capacity calibration\n"
              << "  rx_lite --calib <case.csv> <submission.cfg> # PATH ledger calibration\n"
              << "  rx_lite --r0 <case.csv> <placement.cfg> # shortest-legal R0 route\n"
              << "  rx_lite --r2b1 <case.csv> <placement.cfg> # candidate surface smoke\n"
              << "  rx_lite --r2b2 <selector> <case.csv> <placement.cfg> # selector smoke\n"
              << "  rx_lite --r2c <case.csv> <placement.cfg> # AvoidHot world1->world2 experiment\n"
              << "  rx_lite --r2d0 <case.csv> <placement.cfg> # split allocation v0 experiment\n"
              << "  rx_lite --r2d1 <case.csv> <placement.cfg> # dynamic split allocation v0 experiment\n"
              << "  rx_lite --r2d2 <case.csv> <placement.cfg> # split ratio oracle experiment\n"
              << "  rx_lite --r2d3 <case.csv> <placement.cfg> # 3-way split ratio oracle experiment\n"
              << "  rx_lite --r2d4 <case.csv> <placement.cfg> # loser-B allocation-pool selector experiment\n"
              << "  rx_lite --r2d5 <case.csv> <placement.cfg> # all-pair loser-B allocation-pool selector experiment\n"
              << "  rx_lite --r2d6 <case.csv> <placement.cfg> # all-pair loser-B wide-k allocation experiment\n"
              << "  rx_lite --r2d7 <case.csv> <placement.cfg> # full-switch-only allocation attribution experiment\n"
              << "  rx_lite --r2d8 <case.csv> <placement.cfg> # split-only allocation attribution experiment\n"
              << "  rx_lite --r2e0 <case.csv> <placement.cfg> # static-only fixed-point reselect experiment\n"
              << "  rx_lite --r2e1 <case.csv> <placement.cfg> # static fixed-point then one AvoidHot pass\n"
              << "  rx_lite --r2e2 <case.csv> <placement.cfg> # static fixed-point, AvoidHot, then fixed-point\n"
              << "  rx_lite --r2f0 <case.csv> <placement.cfg> # R2d5 allocation pool measured on R2e2 baseline\n"
              << "  rx_lite --r2f1 <case.csv> <placement.cfg> # split-only allocation pool measured on R2e2 baseline\n"
              << "  rx_lite --r2f2 <case.csv> <placement.cfg> # split-only allocation then allocation-aware reselect\n"
              << "  rx_lite --r2f3 <case.csv> <placement.cfg> # wide split-only allocation pool measured on R2e2 baseline\n"
              << "  rx_lite --r2f4 <case.csv> <placement.cfg> # wide+50 split-only allocation pool measured on R2e2 baseline\n"
              << "  rx_lite --r2f5 <case.csv> <placement.cfg> # slim split-only allocation pool measured on R2e2 baseline\n"
              << "  rx_lite --r2f6 <case.csv> <placement.cfg> # slim split-only allocation then allocation-aware reselect\n"
              << "  rx_lite --r2baseline <case.csv> <placement.cfg> # current router-lite baseline (R2s2)\n"
              << "  rx_lite --baseline <case.csv> <placement.cfg> # alias for --r2baseline\n"
              << "  rx_lite --r2s0 <case.csv> <placement.cfg> # SegmentBypass v0 on R2e2 baseline\n"
              << "  rx_lite --r2s1 <case.csv> <placement.cfg> # SegmentBypass then fixed-pool reselect\n"
              << "  rx_lite --r2s2 <case.csv> <placement.cfg> # SegmentBypass reselect then slim split allocation reselect\n"
              << "  rx_lite --r2g0 <case.csv> <placement.cfg> # residual opportunity audit after R2s2 final\n"
              << "env:\n"
              << "  RX_LITE_SEARCH_COST=edge_center|truewire\n"
              << "  RX_LITE_ROUTE_SPACE=legacy|soft_endpoint|soft_soft|official_full\n"
              << "  RX_LITE_ACCESS_ALT_UNIVERSE=capped_channel|full_unified\n"
              << "  RX_LITE_AVOID_HOT_ELIGIBILITY=winner_touches_hot|candidate_touches_hot\n"
              << "  RX_LITE_AVOID_HOT_WEIGHTS=20000|100,500,2000\n"
              << "  RX_LITE_AVOID_HOT_PASSES=1|2\n"
              << "  RX_LITE_RESELECT_MAX_PASSES=5\n"
              << "  RX_LITE_HOT_THRESHOLD=1.0|0.95|0.9\n";
    return 2;
}
#endif

} // namespace

namespace routerlite {

bool runBaseline(Design& design) {
    const size_t blockCount = design.blockSpecs.size();
    const size_t pairCount = routerx::demandedPairs(design).size();
    if (pairCount >= 200 && blockCount >= 20) {
        // The full R2s2 surface enumerates access alternatives, avoid-hot
        // candidates, segment bypasses and reselect passes for every pair.
        // On a dense 30-block matrix that is quadratic work on top of hundreds
        // of mandatory routes. Build R0 once/pair, then expose only a few
        // avoid-hot alternatives for pairs that actually touch an overloaded
        // axis. This keeps split repair bounded without abandoning it entirely.
        // Medium cases must not take this bypass: they need the adaptive split
        // pass to relieve capacity hot spots.
        const routerx::SoftRoutingPolicy routeSpace{/*endpointAccess=*/true,
                                                    /*softSoftEdges=*/true};
        routerx::RxGraph graph;
        graph.build(design, routeSpace);
        const std::vector<routerx::DemandedPair> pairs = routerx::demandedPairs(design);
        design.routes.clear();
        bool allRouted = true;
        int open = 0;
        long long searchCalls = 0;
        routerx::RxResourceLedgerLite ledger;
        ledger.build(design);
        double totalWire = 0.0;
        std::vector<routerx::LiteCandidate> baseWinners;
        baseWinners.reserve(pairs.size());
        for (const routerx::DemandedPair& pair : pairs) {
            LiteRoutedPair r0 = routeR0Fallback(
                design, graph, pair, routerx::SearchCostMode::TrueWire);
            searchCalls += r0.searchCalls;
            if (r0.found && r0.emitValid) {
                routerx::LiteCandidate current = makeCandidate(
                    design, pair, routerx::LiteCandidateFamily::R0Fallback,
                    r0.route.steps, true);
                current.impact = ledger.projectSteps(
                    design, current.route.steps, pair.nets);
                if (selectableCandidate(current)) {
                    totalWire += current.impact.wire;
                    ledger.commitSteps(design, current.route.steps, pair.nets);
                    baseWinners.push_back(std::move(current));
                    continue;
                }
            }
            allRouted = false;
            ++open;
            baseWinners.push_back(routerx::LiteCandidate{});
        }

        const double overflowBefore = ledger.totalChannelOverflow();
        long long hotPairs = 0;
        long long avoidHotSearchCalls = 0;
        long long avoidHotCandidates = 0;
        std::vector<std::vector<routerx::LiteCandidate>> candidateSets(pairs.size());
        std::vector<std::vector<AllocationPart>> allocations(pairs.size());
        if (allRouted) {
            // Include near-full axes while generating the bounded pool so a
            // repair does not simply move overflow onto the next bottleneck.
            const HotAxisRatios hot = hotAxisRatios(design, ledger, 0.90);
            const std::vector<double> denseWeights = {500.0, 2000.0, 10000.0, 50000.0};
            for (size_t i = 0; i < pairs.size(); ++i) {
                const routerx::DemandedPair& pair = pairs[i];
                const routerx::LiteCandidate& current = baseWinners[i];
                allocations[i] = {AllocationPart{&current, pair.nets}};
                std::vector<routerx::LiteCandidate> raw = {current};
                const RouteAxisDemand currentDemand = routeAxisDemand(
                    design, current.route.steps, pair.nets);
                if (hotAxisDemandTotal(currentDemand, hot) > 1.0e-9) {
                    ++hotPairs;
                    std::vector<routerx::LiteCandidate> alternatives =
                        generateAvoidHotCandidates(
                            design, graph, pair, routerx::SearchCostMode::TrueWire,
                            hot, denseWeights, searchCalls, avoidHotSearchCalls);
                    avoidHotCandidates += static_cast<long long>(alternatives.size());
                    raw.insert(raw.end(), alternatives.begin(), alternatives.end());
                    appendPerimeterCandidates(
                        design, graph, pair, routerx::SearchCostMode::TrueWire,
                        raw, searchCalls);
                }

                routerx::RxResourceLedgerLite replaceLedger = ledger;
                replaceLedger.ripupSteps(
                    design, current.route.steps, pair.nets);
                for (routerx::LiteCandidate& c : raw) {
                    if (c.emitValid) {
                        c.impact = replaceLedger.projectSteps(
                            design, c.route.steps, pair.nets);
                    }
                }
                candidateSets[i] = routerx::dedupLiteCandidates(raw).candidates;
            }

            const AllocationReselectResult denseReselect =
                runAllocationReselectFixedPoint(
                    design, pairs, baseWinners, candidateSets, allocations,
                    ledger, totalWire, 1.0, 3);
            (void)denseReselect;
        }

        const bool routesComplete = materializeRoutes(
            design, pairs, allocations, baseWinners);
        allRouted = allRouted && routesComplete;
        std::cerr << "[RouterLite/DenseAdaptive] pairs=" << pairs.size()
                  << " routed=" << (pairs.size() - static_cast<size_t>(open))
                  << " open=" << open
                  << " hotPairs=" << hotPairs
                  << " searches=" << searchCalls
                  << " avoidHotSearches=" << avoidHotSearchCalls
                  << " avoidHotCandidates=" << avoidHotCandidates
                  << " overflow=" << overflowBefore
                  << "->" << ledger.totalChannelOverflow() << "\n";
        return allRouted;
    }
    return runR2c(design,
                  false, false, false, false, false,
                  false, false, false, false, false,
                  false, false, false, false, false,
                  false, false, false, false, false,
                  false, true);
}

} // namespace routerlite
