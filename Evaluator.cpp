#include "Evaluator.hpp"
#include "Utility.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using namespace std;

namespace {

    static constexpr double CONTACT_EPS = 1.0e-3;
    static constexpr double CONTACT_OVERLAP_EPS = 1.0e-7;
    static constexpr double SHAPE_EPS = 1.0e-3;

    struct Point {
        double x = 0.0;
        double y = 0.0;
    };

    struct ChannelUse {
        double lrNets = 0.0;
        double tbNets = 0.0;
        int lrTraversalCount = 0;
        int tbTraversalCount = 0;
        int turnTraversalCount = 0;
    };

    enum class RouteInvalidReason {
        NONE,
        BAD_NETS_OR_TOO_SHORT,
        BAD_ITEM,
        UNKNOWN_OBJECT,
        BAD_TOPOLOGY,
        ILLEGAL_FEEDTHROUGH,
        CONTACT_FAIL,
        SAME_OBJECT_SAME_EDGE,
        EDGE_PORT_VIOLATION,
    };

    static const char* reasonName(RouteInvalidReason r) {
        switch (r) {
        case RouteInvalidReason::NONE: return "NONE";
        case RouteInvalidReason::BAD_NETS_OR_TOO_SHORT: return "BAD_NETS_OR_TOO_SHORT";
        case RouteInvalidReason::BAD_ITEM: return "BAD_ITEM";
        case RouteInvalidReason::UNKNOWN_OBJECT: return "UNKNOWN_OBJECT";
        case RouteInvalidReason::BAD_TOPOLOGY: return "BAD_TOPOLOGY";
        case RouteInvalidReason::ILLEGAL_FEEDTHROUGH: return "ILLEGAL_FEEDTHROUGH";
        case RouteInvalidReason::CONTACT_FAIL: return "CONTACT_FAIL";
        case RouteInvalidReason::SAME_OBJECT_SAME_EDGE: return "SAME_OBJECT_SAME_EDGE";
        case RouteInvalidReason::EDGE_PORT_VIOLATION: return "EDGE_PORT_VIOLATION";
        }
        return "UNKNOWN";
    }

    static bool validEdge(int e) {
        return e >= 1 && e <= 4;
    }

    static bool isOppositeLR(int a, int b) {
        return (a == 1 && b == 3) || (a == 3 && b == 1);
    }

    static bool isOppositeTB(int a, int b) {
        return (a == 2 && b == 4) || (a == 4 && b == 2);
    }

    static bool isTurn(int a, int b) {
        return validEdge(a) && validEdge(b) && a != b &&
            !isOppositeLR(a, b) && !isOppositeTB(a, b);
    }

    static double rectArea(const Rect& r) {
        return max(0.0, r.w) * max(0.0, r.h);
    }

    static bool rectOverlapAreaPositiveLocal(const Rect& a, const Rect& b) {
        return overlapLen(a.x, rectRight(a), b.x, rectRight(b)) > EPS &&
            overlapLen(a.y, rectTop(a), b.y, rectTop(b)) > EPS;
    }

    static Point edgeCenter(const Rect& r, int edge) {
        switch (edge) {
        case 1: return Point{ r.x, rectCy(r) };
        case 2: return Point{ rectCx(r), rectTop(r) };
        case 3: return Point{ rectRight(r), rectCy(r) };
        case 4: return Point{ rectCx(r), r.y };
        default: return Point{ rectCx(r), rectCy(r) };
        }
    }

    static bool contactGuidingPoint(const Rect& a, int ea, const Rect& b, int eb, Point& p) {
        if (!validEdge(ea) || !validEdge(eb)) return false;

        if (ea == 3 && eb == 1 && fabs(rectRight(a) - b.x) <= CONTACT_EPS) {
            double lo = max(a.y, b.y);
            double hi = min(rectTop(a), rectTop(b));
            if (hi <= lo + CONTACT_OVERLAP_EPS) return false;
            p = Point{ 0.5 * (rectRight(a) + b.x), 0.5 * (lo + hi) };
            return true;
        }
        if (ea == 1 && eb == 3 && fabs(a.x - rectRight(b)) <= CONTACT_EPS) {
            double lo = max(a.y, b.y);
            double hi = min(rectTop(a), rectTop(b));
            if (hi <= lo + CONTACT_OVERLAP_EPS) return false;
            p = Point{ 0.5 * (a.x + rectRight(b)), 0.5 * (lo + hi) };
            return true;
        }
        if (ea == 2 && eb == 4 && fabs(rectTop(a) - b.y) <= CONTACT_EPS) {
            double lo = max(a.x, b.x);
            double hi = min(rectRight(a), rectRight(b));
            if (hi <= lo + CONTACT_OVERLAP_EPS) return false;
            p = Point{ 0.5 * (lo + hi), 0.5 * (rectTop(a) + b.y) };
            return true;
        }
        if (ea == 4 && eb == 2 && fabs(a.y - rectTop(b)) <= CONTACT_EPS) {
            double lo = max(a.x, b.x);
            double hi = min(rectRight(a), rectRight(b));
            if (hi <= lo + CONTACT_OVERLAP_EPS) return false;
            p = Point{ 0.5 * (lo + hi), 0.5 * (a.y + rectTop(b)) };
            return true;
        }
        return false;
    }

    static double manhattanPoint(const Point& a, const Point& b) {
        return fabs(a.x - b.x) + fabs(a.y - b.y);
    }

    static vector<string> splitLocTokens(const vector<string>& rawLocs) {
        vector<string> out;
        for (string s : rawLocs) {
            string cur;
            for (char ch : s) {
                if (ch == ',' || ch == ';' || ch == '/' || ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n') {
                    if (!cur.empty()) {
                        out.push_back(upperStr(cur));
                        cur.clear();
                    }
                }
                else {
                    cur.push_back(ch);
                }
            }
            if (!cur.empty()) out.push_back(upperStr(cur));
        }
        return out;
    }

    static bool spanInsideZone(double loVal, double hiVal, int zone, double span) {
        double lo = zone * span / 3.0;
        double hi = (zone + 1) * span / 3.0;
        return loVal >= lo - SHAPE_EPS && hiVal <= hi + SHAPE_EPS;
    }

    static bool edgeBlockMatchesOneLocation(const Rect& r, const string& loc, double W, double H) {
        if (loc.size() < 2) return false;
        char a = loc[0];
        char b = loc[1];

        if (a == 'T' && (b == 'L' || b == 'M' || b == 'R')) {
            int zone = (b == 'L') ? 0 : (b == 'M' ? 1 : 2);
            return fabs(rectTop(r) - H) <= SHAPE_EPS && spanInsideZone(r.x, rectRight(r), zone, W);
        }
        if (a == 'B' && (b == 'L' || b == 'M' || b == 'R')) {
            int zone = (b == 'L') ? 0 : (b == 'M' ? 1 : 2);
            return fabs(r.y) <= SHAPE_EPS && spanInsideZone(r.x, rectRight(r), zone, W);
        }
        if (a == 'L' && (b == 'B' || b == 'M' || b == 'T')) {
            int zone = (b == 'B') ? 0 : (b == 'M' ? 1 : 2);
            return fabs(r.x) <= SHAPE_EPS && spanInsideZone(r.y, rectTop(r), zone, H);
        }
        if (a == 'R' && (b == 'B' || b == 'M' || b == 'T')) {
            int zone = (b == 'B') ? 0 : (b == 'M' ? 1 : 2);
            return fabs(rectRight(r) - W) <= SHAPE_EPS && spanInsideZone(r.y, rectTop(r), zone, H);
        }
        return false;
    }

    static bool edgeBlockLocationOK(const BlockInst& b, double W, double H) {
        if (b.spec.type != BlockType::EDGE) return true;
        vector<string> locs = splitLocTokens(b.spec.locations);
        if (locs.empty()) return true;
        for (const string& loc : locs) {
            if (edgeBlockMatchesOneLocation(b.rect, loc, W, H)) return true;
        }
        return false;
    }

    static bool blockShapeOK(const BlockInst& b) {
        const Rect& r = b.rect;
        if (r.w <= EPS || r.h <= EPS) return false;

        if (b.spec.type == BlockType::HARD || b.spec.type == BlockType::EDGE || b.spec.hasFixedSize) {
            if (b.spec.fixedW > EPS && fabs(r.w - b.spec.fixedW) > SHAPE_EPS) return false;
            if (b.spec.fixedH > EPS && fabs(r.h - b.spec.fixedH) > SHAPE_EPS) return false;
            return true;
        }

        if (b.spec.type == BlockType::SOFT) {
            if (b.spec.area > EPS && rectArea(r) + SHAPE_EPS < b.spec.area) return false;
            double ratio = r.w / max(EPS, r.h);
            double amin = max(0.0, b.spec.aspectMin);
            double amax = max(amin, b.spec.aspectMax);
            if (amin > EPS && ratio + 1.0e-6 < amin) return false;
            if (amax > EPS && ratio - 1.0e-6 > amax) return false;
        }
        return true;
    }

    static unordered_map<string, int> buildBlockMap(const Design& design) {
        unordered_map<string, int> mp;
        for (int i = 0; i < static_cast<int>(design.blocks.size()); ++i) {
            mp[design.blocks[i].spec.name] = i;
        }
        return mp;
    }

    static unordered_map<string, int> buildChannelMap(const Design& design) {
        unordered_map<string, int> mp;
        for (int i = 0; i < static_cast<int>(design.channels.size()); ++i) {
            mp[design.channels[i].name] = i;
        }
        return mp;
    }

    static bool getObjectRect(
        const string& name,
        const Design& design,
        const unordered_map<string, int>& blockMap,
        const unordered_map<string, int>& channelMap,
        Rect& out,
        bool& isBlock,
        bool& isChannel
    ) {
        auto bit = blockMap.find(name);
        if (bit != blockMap.end()) {
            out = design.blocks[bit->second].rect;
            isBlock = true;
            isChannel = false;
            return true;
        }
        auto cit = channelMap.find(name);
        if (cit != channelMap.end()) {
            out = design.channels[cit->second].rect;
            isBlock = false;
            isChannel = true;
            return true;
        }
        return false;
    }

    static double channelLRCapacity(const Rect& r) {
        return max(0.0, r.h) * CHANNEL_DENSITY;
    }

    static double channelTBCapacity(const Rect& r) {
        return max(0.0, r.w) * CHANNEL_DENSITY;
    }

    static void addChannelDirectionalUse(ChannelUse& use, int inEdge, int outEdge, double nets) {
        if (inEdge == outEdge) return;
        if (isOppositeLR(inEdge, outEdge)) {
            use.lrNets += nets;
            ++use.lrTraversalCount;
        }
        else if (isOppositeTB(inEdge, outEdge)) {
            use.tbNets += nets;
            ++use.tbTraversalCount;
        }
        else if (isTurn(inEdge, outEdge)) {
            use.lrNets += nets;
            use.tbNets += nets;
            ++use.turnTraversalCount;
        }
    }

    static double ftRateForNetsLocal(const BlockSpec& spec, double ftNets) {
        if (ftNets <= 3000.0) return spec.ftRate[0];
        if (ftNets <= 6000.0) return spec.ftRate[1];
        if (ftNets <= 9000.0) return spec.ftRate[2];
        return spec.ftRate[3];
    }

    static double ftSideDelta(const BlockSpec& spec, double ftNets) {
        if (ftNets <= EPS) return 0.0;
        return (ftNets / CHANNEL_DENSITY) * ftRateForNetsLocal(spec, ftNets) / 2.0;
    }

    static double requiredAreaWithFT(const BlockInst& b) {
        if (b.spec.type != BlockType::SOFT) return rectArea(b.rect);
        const double baseArea = max(0.0, b.spec.area);
        if (b.ftUsed <= EPS) return baseArea;
        const double baseSide = sqrt(baseArea);
        const double delta = ftSideDelta(b.spec, b.ftUsed);
        return (baseSide + delta) * (baseSide + delta);
    }

    static bool containsInt(const vector<int>& vals, int target) {
        return find(vals.begin(), vals.end(), target) != vals.end();
    }

    static vector<int> centerSideEdgesForEdgeBlock(const BlockInst& b, double W, double H) {
        vector<int> edges;
        const Rect& r = b.rect;
        if (fabs(rectTop(r) - H) <= SHAPE_EPS) edges.push_back(4);
        if (fabs(r.y) <= SHAPE_EPS) edges.push_back(2);
        if (fabs(r.x) <= SHAPE_EPS) edges.push_back(3);
        if (fabs(rectRight(r) - W) <= SHAPE_EPS) edges.push_back(1);
        return edges;
    }

    static bool edgeBlockPortOK(const BlockInst& b, int edge, double W, double H) {
        if (b.spec.type != BlockType::EDGE) return true;
        const vector<int> allowed = centerSideEdgesForEdgeBlock(b, W, H);
        if (allowed.empty()) return true;
        return containsInt(allowed, edge);
    }

    static Point contactOrObjectEdgeCenter(
        const Design& design,
        const unordered_map<string, int>& blockMap,
        const unordered_map<string, int>& channelMap,
        const RouteStep& a,
        const RouteStep& b,
        const string& fallbackObjectName,
        int fallbackEdge
    ) {
        Rect ra, rb, rf;
        bool ab = false, ac = false, bb = false, bc = false, fb = false, fc = false;
        Point gp;
        if (getObjectRect(a.rectName, design, blockMap, channelMap, ra, ab, ac) &&
            getObjectRect(b.rectName, design, blockMap, channelMap, rb, bb, bc) &&
            contactGuidingPoint(ra, a.edge, rb, b.edge, gp)) {
            return gp;
        }
        if (getObjectRect(fallbackObjectName, design, blockMap, channelMap, rf, fb, fc)) {
            return edgeCenter(rf, fallbackEdge);
        }
        return Point{};
    }

    static void incrementInvalidCounter(EvalReport& rpt, RouteInvalidReason reason) {
        switch (reason) {
        case RouteInvalidReason::BAD_NETS_OR_TOO_SHORT:
        case RouteInvalidReason::BAD_TOPOLOGY:
            ++rpt.badTopologyCount;
            break;
        case RouteInvalidReason::BAD_ITEM:
            ++rpt.badItemCount;
            break;
        case RouteInvalidReason::UNKNOWN_OBJECT:
            ++rpt.unknownObjectCount;
            break;
        case RouteInvalidReason::ILLEGAL_FEEDTHROUGH:
            ++rpt.illegalFeedthroughCount;
            break;
        case RouteInvalidReason::CONTACT_FAIL:
            ++rpt.contactFailCount;
            break;
        case RouteInvalidReason::SAME_OBJECT_SAME_EDGE:
            ++rpt.sameObjectSameEdgeCount;
            break;
        case RouteInvalidReason::EDGE_PORT_VIOLATION:
            ++rpt.edgePortViolationCount;
            break;
        case RouteInvalidReason::NONE:
            break;
        }
    }

    static void logInvalidRoute(const RouteTruth& truth) {
        static int printed = 0;
        static constexpr int MAX_PRINT = 200;
        if (printed >= MAX_PRINT) return;
        ++printed;

        cerr << "[EvalInvalidRoute] routeId=" << truth.routeId
            << " nets=" << truth.netCount
            << " reason=" << truth.invalidReason
            << " src=" << truth.srcBlock
            << " dst=" << truth.dstBlock;
        if (!truth.invalidDetail.empty()) cerr << " detail=" << truth.invalidDetail;
        cerr << "\n";
    }

    static RouteTruth validateAndAccumulateOneRoute(
        Design& design,
        RoutePath& route,
        int routeId,
        const unordered_map<string, int>& blockMap,
        const unordered_map<string, int>& channelMap,
        vector<ChannelUse>& channelUse,
        vector<ChannelSegmentTruth>& channelSegments,
        EvalReport& rpt
    ) {
        RouteTruth truth;
        truth.routeId = routeId;
        truth.netCount = route.netCount;
        truth.srcBlock = route.srcBlock;
        truth.dstBlock = route.dstBlock;
        truth.itemCount = static_cast<int>(route.steps.size());
        truth.explicitOpen = route.open;

        route.wireLength = 0.0;
        if (route.open) {
            return truth;
        }

        bool invalid = false;
        RouteInvalidReason firstReason = RouteInvalidReason::NONE;
        string firstDetail;

        auto markInvalid = [&](RouteInvalidReason reason, const string& detail) {
            if (!invalid) {
                invalid = true;
                firstReason = reason;
                firstDetail = detail;
            }
        };

        if (route.netCount <= 0 || route.steps.size() < 2) {
            markInvalid(RouteInvalidReason::BAD_NETS_OR_TOO_SHORT, "non-positive net count or fewer than two path items");
        }

        for (int i = 0; i < static_cast<int>(route.steps.size()); ++i) {
            const auto& step = route.steps[i];
            if (!validEdge(step.edge) || step.rectName.empty()) {
                ostringstream oss;
                oss << "bad item at index " << i << " name='" << step.rectName << "' edge=" << step.edge;
                markInvalid(RouteInvalidReason::BAD_ITEM, oss.str());
                continue;
            }

            Rect r;
            bool isBlock = false, isChannel = false;
            if (!getObjectRect(step.rectName, design, blockMap, channelMap, r, isBlock, isChannel)) {
                ostringstream oss;
                oss << "unknown object at index " << i << ": " << step.rectName;
                markInvalid(RouteInvalidReason::UNKNOWN_OBJECT, oss.str());
            }
        }

        if (!invalid) {
            if ((route.steps.size() % 2) != 0) {
                markInvalid(RouteInvalidReason::BAD_TOPOLOGY, "item count is odd; intermediate rectangles must be in/out pairs");
            }
            else if (!blockMap.count(route.steps.front().rectName)) {
                markInvalid(RouteInvalidReason::BAD_TOPOLOGY, "first item is not a block: " + route.steps.front().rectName);
            }
            else if (!blockMap.count(route.steps.back().rectName)) {
                markInvalid(RouteInvalidReason::BAD_TOPOLOGY, "last item is not a block: " + route.steps.back().rectName);
            }
        }

        if (!invalid && !route.steps.empty()) {
            const BlockInst& src = design.blocks[blockMap.at(route.steps.front().rectName)];
            const BlockInst& dst = design.blocks[blockMap.at(route.steps.back().rectName)];
            if (!edgeBlockPortOK(src, route.steps.front().edge, design.outlineW, design.outlineH)) {
                markInvalid(RouteInvalidReason::EDGE_PORT_VIOLATION, "edge block source uses non-center-side edge: " + src.spec.name);
            }
            if (!edgeBlockPortOK(dst, route.steps.back().edge, design.outlineW, design.outlineH)) {
                markInvalid(RouteInvalidReason::EDGE_PORT_VIOLATION, "edge block sink uses non-center-side edge: " + dst.spec.name);
            }
        }

        if (!invalid) {
            for (int i = 1; i + 1 < static_cast<int>(route.steps.size()); i += 2) {
                const auto& in = route.steps[i];
                const auto& out = route.steps[i + 1];

                if (in.rectName != out.rectName) {
                    ostringstream oss;
                    oss << "bad in/out pair at items[" << i << "," << (i + 1) << "]: "
                        << in.rectName << " vs " << out.rectName;
                    markInvalid(RouteInvalidReason::BAD_TOPOLOGY, oss.str());
                    break;
                }
                if (in.edge == out.edge) {
                    markInvalid(RouteInvalidReason::SAME_OBJECT_SAME_EDGE, "same in/out edge in intermediate pair: " + in.rectName);
                    break;
                }

                auto cit = channelMap.find(in.rectName);
                if (cit != channelMap.end()) continue;

                auto bit = blockMap.find(in.rectName);
                if (bit == blockMap.end()) {
                    markInvalid(RouteInvalidReason::UNKNOWN_OBJECT, "intermediate object is neither channel nor block: " + in.rectName);
                    break;
                }
                if (design.blocks[bit->second].spec.type != BlockType::SOFT) {
                    markInvalid(RouteInvalidReason::ILLEGAL_FEEDTHROUGH, "non-soft block used as intermediate feedthrough: " + in.rectName);
                    break;
                }
            }
        }

        vector<Point> guiding;
        for (int i = 0; i + 1 < static_cast<int>(route.steps.size()); ++i) {
            const auto& a = route.steps[i];
            const auto& b = route.steps[i + 1];

            Rect ra, rb;
            bool aBlock = false, aChannel = false, bBlock = false, bChannel = false;
            if (!getObjectRect(a.rectName, design, blockMap, channelMap, ra, aBlock, aChannel)) continue;
            if (!getObjectRect(b.rectName, design, blockMap, channelMap, rb, bBlock, bChannel)) continue;

            if (a.rectName == b.rectName) {
                if (a.edge == b.edge) {
                    markInvalid(RouteInvalidReason::SAME_OBJECT_SAME_EDGE, "same object traversal with identical in/out edge: " + a.rectName);
                }

                auto cit = channelMap.find(a.rectName);
                if (cit != channelMap.end()) {
                    ChannelUse& use = channelUse[cit->second];
                    addChannelDirectionalUse(use, a.edge, b.edge, static_cast<double>(max(0, route.netCount)));
                    ++truth.channelTraversalCount;

                    if (i > 0 && i + 2 < static_cast<int>(route.steps.size())) {
                        Point from = contactOrObjectEdgeCenter(design, blockMap, channelMap,
                            route.steps[i - 1], route.steps[i], a.rectName, a.edge);
                        Point to = contactOrObjectEdgeCenter(design, blockMap, channelMap,
                            route.steps[i + 1], route.steps[i + 2], b.rectName, b.edge);

                        ChannelSegmentTruth seg;
                        seg.routeId = routeId;
                        seg.channelName = a.rectName;
                        seg.netCount = route.netCount;
                        seg.inEdge = a.edge;
                        seg.outEdge = b.edge;
                        seg.fromX = from.x;
                        seg.fromY = from.y;
                        seg.toX = to.x;
                        seg.toY = to.y;

                        if (isOppositeLR(a.edge, b.edge)) {
                            seg.lrDemand = static_cast<double>(max(0, route.netCount));
                        }
                        else if (isOppositeTB(a.edge, b.edge)) {
                            seg.tbDemand = static_cast<double>(max(0, route.netCount));
                        }
                        else if (isTurn(a.edge, b.edge)) {
                            seg.lrDemand = static_cast<double>(max(0, route.netCount));
                            seg.tbDemand = static_cast<double>(max(0, route.netCount));
                        }

                        seg.lrSpanLo = min(from.y, to.y);
                        seg.lrSpanHi = max(from.y, to.y);
                        seg.tbSpanLo = min(from.x, to.x);
                        seg.tbSpanHi = max(from.x, to.x);
                        channelSegments.push_back(seg);
                    }
                }
                else {
                    auto bit = blockMap.find(a.rectName);
                    if (bit != blockMap.end()) {
                        BlockInst& blk = design.blocks[bit->second];
                        if (blk.spec.type == BlockType::SOFT) {
                            blk.ftUsed += static_cast<double>(max(0, route.netCount));
                            ++truth.feedthroughTraversalCount;
                        }
                        else {
                            markInvalid(RouteInvalidReason::ILLEGAL_FEEDTHROUGH, "non-soft block feedthrough: " + a.rectName);
                        }
                    }
                }
                continue;
            }

            Point gp;
            if (contactGuidingPoint(ra, a.edge, rb, b.edge, gp)) {
                guiding.push_back(gp);
            }
            else {
                if (!invalid) {
                    ostringstream oss;
                    oss << "contact fail at pair " << i << "->" << (i + 1)
                        << " " << a.rectName << ":e" << a.edge
                        << " to " << b.rectName << ":e" << b.edge;
                    markInvalid(RouteInvalidReason::CONTACT_FAIL, oss.str());
                }
                guiding.push_back(edgeCenter(ra, a.edge));
                guiding.push_back(edgeCenter(rb, b.edge));
            }
        }

        double wl = 0.0;
        for (int i = 0; i + 1 < static_cast<int>(guiding.size()); ++i) {
            wl += manhattanPoint(guiding[i], guiding[i + 1]) * static_cast<double>(max(0, route.netCount));
        }
        route.wireLength = wl;

        truth.guidingPointCount = static_cast<int>(guiding.size());
        truth.wireLength = wl;
        truth.invalid = invalid;
        if (invalid) {
            truth.invalidReason = reasonName(firstReason);
            truth.invalidDetail = firstDetail;
            ++rpt.invalidPathCount;
            rpt.pathInvalid = true;
            incrementInvalidCounter(rpt, firstReason);
            logInvalidRoute(truth);
        }

        return truth;
    }

    static void recomputeRoutingStatsFromPaths(Design& design, EvalReport& rpt) {
        auto blockMap = buildBlockMap(design);
        auto channelMap = buildChannelMap(design);

        vector<ChannelUse> channelUse(design.channels.size());
        rpt.routeTruth.clear();
        rpt.channelTruth.clear();
        rpt.channelSegments.clear();
        rpt.feedthroughTruth.clear();

        for (auto& ch : design.channels) {
            ch.usedNets = 0.0;
            ch.capacity = 0.0;
            ch.overflow = 0.0;
        }
        for (auto& b : design.blocks) {
            b.ftUsed = 0.0;
            b.ftOverflowArea = 0.0;
        }

        for (int rid = 0; rid < static_cast<int>(design.routes.size()); ++rid) {
            rpt.routeTruth.push_back(validateAndAccumulateOneRoute(
                design, design.routes[rid], rid, blockMap, channelMap, channelUse, rpt.channelSegments, rpt));
        }

        for (int i = 0; i < static_cast<int>(design.channels.size()); ++i) {
            auto& ch = design.channels[i];
            const ChannelUse& u = channelUse[i];

            const double capLR = channelLRCapacity(ch.rect);
            const double capTB = channelTBCapacity(ch.rect);
            const double ovLR = max(0.0, u.lrNets - capLR);
            const double ovTB = max(0.0, u.tbNets - capTB);
            const double utilLR = capLR > EPS ? u.lrNets / capLR : (u.lrNets > EPS ? numeric_limits<double>::infinity() : 0.0);
            const double utilTB = capTB > EPS ? u.tbNets / capTB : (u.tbNets > EPS ? numeric_limits<double>::infinity() : 0.0);

            if (utilLR >= utilTB) {
                ch.usedNets = u.lrNets;
                ch.capacity = capLR;
            }
            else {
                ch.usedNets = u.tbNets;
                ch.capacity = capTB;
            }
            ch.overflow = ovLR + ovTB;

            ChannelTruth ct;
            ct.name = ch.name;
            ct.rect = ch.rect;
            ct.lrUsed = u.lrNets;
            ct.tbUsed = u.tbNets;
            ct.lrCapacity = capLR;
            ct.tbCapacity = capTB;
            ct.lrUtilization = utilLR;
            ct.tbUtilization = utilTB;
            ct.lrOverflow = ovLR;
            ct.tbOverflow = ovTB;
            ct.lrTraversalCount = u.lrTraversalCount;
            ct.tbTraversalCount = u.tbTraversalCount;
            ct.turnTraversalCount = u.turnTraversalCount;
            rpt.channelTruth.push_back(ct);
        }

        for (auto& b : design.blocks) {
            if (b.spec.type != BlockType::SOFT) continue;

            FeedthroughTruth ft;
            ft.blockName = b.spec.name;
            ft.usedNets = b.ftUsed;
            ft.conversionRate = b.ftUsed > EPS ? ftRateForNetsLocal(b.spec, b.ftUsed) : 0.0;
            ft.sideDelta = ftSideDelta(b.spec, b.ftUsed);
            ft.baseArea = max(0.0, b.spec.area);
            ft.currentArea = rectArea(b.rect);
            ft.requiredArea = requiredAreaWithFT(b);
            ft.overflowArea = max(0.0, ft.requiredArea - ft.currentArea);
            b.ftOverflowArea = ft.overflowArea;
            rpt.feedthroughTruth.push_back(ft);
        }
    }

} // namespace

EvalReport Evaluator::evaluate(Design& design, double alpha) {
    EvalReport rpt;

    recomputeRoutingStatsFromPaths(design, rpt);

    rpt.outlineArea = design.outlineW * design.outlineH;
    rpt.totalWireLength = calcTotalWireLength(design);
    rpt.cost = rpt.outlineArea + alpha * rpt.totalWireLength;

    rpt.blockOverlap = checkBlockOverlap(design, rpt.overlapCount);
    rpt.outlineViolation = checkOutlineViolation(design, rpt.outlineViolationCount);
    rpt.routingOpen = checkRoutingOpen(design, rpt.openPathCount);

    calcChannelOverflow(design, rpt.totalChannelOverflow, rpt.maxChannelOverflow);
    calcFeedthroughOverflow(design, rpt.totalFeedthroughOverflow, rpt.maxFeedthroughOverflow);

    return rpt;
}

double Evaluator::calcTotalWireLength(const Design& design) const {
    double sum = 0.0;
    for (const auto& p : design.routes) sum += p.wireLength;
    return sum;
}

bool Evaluator::checkBlockOverlap(const Design& design, int& overlapCount) const {
    overlapCount = 0;
    for (int i = 0; i < static_cast<int>(design.blocks.size()); ++i) {
        for (int j = i + 1; j < static_cast<int>(design.blocks.size()); ++j) {
            if (rectOverlapAreaPositiveLocal(design.blocks[i].rect, design.blocks[j].rect)) ++overlapCount;
        }
    }
    return overlapCount > 0;
}

bool Evaluator::checkOutlineViolation(const Design& design, int& violationCount) const {
    violationCount = 0;

    if (design.outlineW <= EPS || design.outlineH <= EPS) ++violationCount;
    if (design.outlineW > design.maxOutlineW + EPS || design.outlineH > design.maxOutlineH + EPS) ++violationCount;

    for (const auto& b : design.blocks) {
        const Rect& r = b.rect;
        if (r.w <= EPS || r.h <= EPS) {
            ++violationCount;
            continue;
        }
        if (r.x < -EPS || r.y < -EPS || rectRight(r) > design.outlineW + EPS || rectTop(r) > design.outlineH + EPS) {
            ++violationCount;
        }
        if (!blockShapeOK(b)) ++violationCount;
        if (!edgeBlockLocationOK(b, design.outlineW, design.outlineH)) ++violationCount;
    }
    return violationCount > 0;
}

bool Evaluator::checkRoutingOpen(const Design& design, int& openPathCount) const {
    openPathCount = 0;
    for (const auto& p : design.routes) {
        if (p.open) ++openPathCount;
    }
    return openPathCount > 0;
}

void Evaluator::calcChannelOverflow(Design& design, double& totalOverflow, double& maxOverflow) const {
    totalOverflow = 0.0;
    maxOverflow = 0.0;
    for (auto& ch : design.channels) {
        ch.overflow = max(0.0, ch.overflow);
        totalOverflow += ch.overflow;
        maxOverflow = max(maxOverflow, ch.overflow);
    }
}

double Evaluator::ftRateForNets(const BlockSpec& spec, double ftNets) const {
    return ftRateForNetsLocal(spec, ftNets);
}

double Evaluator::estimateRequiredAreaWithFT(const BlockInst& b) const {
    return requiredAreaWithFT(b);
}

void Evaluator::calcFeedthroughOverflow(Design& design, double& totalOverflow, double& maxOverflow) const {
    totalOverflow = 0.0;
    maxOverflow = 0.0;
    for (auto& b : design.blocks) {
        b.ftOverflowArea = 0.0;
        if (b.spec.type != BlockType::SOFT) continue;
        b.ftOverflowArea = max(0.0, estimateRequiredAreaWithFT(b) - rectArea(b.rect));
        totalOverflow += b.ftOverflowArea;
        maxOverflow = max(maxOverflow, b.ftOverflowArea);
    }
}
