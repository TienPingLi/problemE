#include "Evaluator.hpp"
#include "Utility.hpp"
#include "RouterX/RxScore.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace std;

namespace {

    static constexpr double CHECKER_GEOM_EPS = 3.0;
    static constexpr double CHECKER_BETA_LIN = 20.0;
    static constexpr double CHECKER_BETA_TAIL = 1.5;
    static constexpr double CHECKER_BETA_FT = 3.0;
    static constexpr double CHECKER_LAMBDA_FT = 10.0;
    static constexpr double CHECKER_OVERFLOW_THRESHOLD = 0.05;
    static constexpr double CHECKER_GAMMA_EDGE = 1.0;
    static constexpr double CHECKER_DELTA_RUNTIME = 0.5;

    static constexpr double CONTACT_EPS = CHECKER_GEOM_EPS;
    // Rectangles may touch on a very small overlap segment after floating-point
    // placement/channel construction.  For contact validity, require positive
    // overlap, but do not require a large 1e-3 overlap.  This avoids false
    // routing-open reports for paths that the router built from touching geometry.
    static constexpr double CONTACT_OVERLAP_EPS = 1.0e-7;
    static constexpr double SHAPE_EPS = CHECKER_GEOM_EPS;

    struct Point {
        double x = 0.0;
        double y = 0.0;
    };

    struct RouteItemView {
        string name;
        int edge = 0;
    };

    struct ChannelUse {
        // Direction-aware channel demand.
        // Edge convention: 1=left, 2=top, 3=right, 4=bottom.
        //
        // A same-channel traversal 1<->3 consumes the LR component once.
        // A same-channel traversal 2<->4 consumes the TB component once.
        // A turn, e.g. 1->2 / 1->4 / 3->2 / 3->4, is L-shaped inside the channel
        // and consumes BOTH components once.
        double lrNets = 0.0;
        double tbNets = 0.0;
    };

    // -----------------------------------------------------------------------------
    // Small C++17 reflection helpers.
    // If your Route / RouteStep uses different field names, normally you only need to
    // edit stepName(), stepEdge(), routeNetCount(), getRouteItems(), setRouteOpen(),
    // and setRouteWireLength() below.
    // -----------------------------------------------------------------------------

    template <class, class = void> struct has_name : false_type {};
    template <class T> struct has_name<T, void_t<decltype(declval<T>().name)>> : true_type {};

    template <class, class = void> struct has_rectName : false_type {};
    template <class T> struct has_rectName<T, void_t<decltype(declval<T>().rectName)>> : true_type {};

    template <class, class = void> struct has_objectName : false_type {};
    template <class T> struct has_objectName<T, void_t<decltype(declval<T>().objectName)>> : true_type {};

    template <class, class = void> struct has_targetName : false_type {};
    template <class T> struct has_targetName<T, void_t<decltype(declval<T>().targetName)>> : true_type {};

    template <class, class = void> struct has_edge : false_type {};
    template <class T> struct has_edge<T, void_t<decltype(declval<T>().edge)>> : true_type {};

    template <class, class = void> struct has_edgeIdx : false_type {};
    template <class T> struct has_edgeIdx<T, void_t<decltype(declval<T>().edgeIdx)>> : true_type {};

    template <class, class = void> struct has_edgeId : false_type {};
    template <class T> struct has_edgeId<T, void_t<decltype(declval<T>().edgeId)>> : true_type {};

    template <class, class = void> struct has_edgeIndex : false_type {};
    template <class T> struct has_edgeIndex<T, void_t<decltype(declval<T>().edgeIndex)>> : true_type {};

    template <class, class = void> struct has_first : false_type {};
    template <class T> struct has_first<T, void_t<decltype(declval<T>().first)>> : true_type {};

    template <class, class = void> struct has_second : false_type {};
    template <class T> struct has_second<T, void_t<decltype(declval<T>().second)>> : true_type {};

    template <class, class = void> struct has_steps : false_type {};
    template <class T> struct has_steps<T, void_t<decltype(declval<T>().steps)>> : true_type {};

    template <class, class = void> struct has_nodes : false_type {};
    template <class T> struct has_nodes<T, void_t<decltype(declval<T>().nodes)>> : true_type {};

    template <class, class = void> struct has_items : false_type {};
    template <class T> struct has_items<T, void_t<decltype(declval<T>().items)>> : true_type {};

    template <class, class = void> struct has_pattern : false_type {};
    template <class T> struct has_pattern<T, void_t<decltype(declval<T>().pattern)>> : true_type {};

    template <class, class = void> struct has_path : false_type {};
    template <class T> struct has_path<T, void_t<decltype(declval<T>().path)>> : true_type {};

    template <class, class = void> struct has_netCount : false_type {};
    template <class T> struct has_netCount<T, void_t<decltype(declval<T>().netCount)>> : true_type {};

    template <class, class = void> struct has_nets : false_type {};
    template <class T> struct has_nets<T, void_t<decltype(declval<T>().nets)>> : true_type {};

    template <class, class = void> struct has_numNets : false_type {};
    template <class T> struct has_numNets<T, void_t<decltype(declval<T>().numNets)>> : true_type {};

    template <class, class = void> struct has_open : false_type {};
    template <class T> struct has_open<T, void_t<decltype(declval<T>().open)>> : true_type {};

    template <class, class = void> struct has_wireLength : false_type {};
    template <class T> struct has_wireLength<T, void_t<decltype(declval<T>().wireLength)>> : true_type {};

    template <class StepT>
    static string stepName(const StepT& s) {
        if constexpr (has_name<StepT>::value) return s.name;
        else if constexpr (has_rectName<StepT>::value) return s.rectName;
        else if constexpr (has_objectName<StepT>::value) return s.objectName;
        else if constexpr (has_targetName<StepT>::value) return s.targetName;
        else if constexpr (has_first<StepT>::value) return s.first;
        else return string();
    }

    template <class StepT>
    static int stepEdge(const StepT& s) {
        if constexpr (has_edge<StepT>::value) return static_cast<int>(s.edge);
        else if constexpr (has_edgeIdx<StepT>::value) return static_cast<int>(s.edgeIdx);
        else if constexpr (has_edgeId<StepT>::value) return static_cast<int>(s.edgeId);
        else if constexpr (has_edgeIndex<StepT>::value) return static_cast<int>(s.edgeIndex);
        else if constexpr (has_second<StepT>::value) return static_cast<int>(s.second);
        else return 0;
    }

    template <class RouteT>
    static int routeNetCount(const RouteT& r) {
        if constexpr (has_netCount<RouteT>::value) return static_cast<int>(r.netCount);
        else if constexpr (has_nets<RouteT>::value) return static_cast<int>(r.nets);
        else if constexpr (has_numNets<RouteT>::value) return static_cast<int>(r.numNets);
        else return 0;
    }

    template <class RouteT>
    static vector<RouteItemView> getRouteItems(const RouteT& r) {
        vector<RouteItemView> out;

        auto append = [&](const auto& seq) {
            out.reserve(seq.size());
            for (const auto& s : seq) {
                out.push_back(RouteItemView{ stepName(s), stepEdge(s) });
            }
            };

        if constexpr (has_steps<RouteT>::value) append(r.steps);
        else if constexpr (has_nodes<RouteT>::value) append(r.nodes);
        else if constexpr (has_items<RouteT>::value) append(r.items);
        else if constexpr (has_pattern<RouteT>::value) append(r.pattern);
        else if constexpr (has_path<RouteT>::value) append(r.path);

        return out;
    }

    template <class RouteT>
    static void setRouteOpen(RouteT& r, bool v) {
        if constexpr (has_open<RouteT>::value) r.open = v;
    }

    template <class RouteT>
    static bool routeOpen(const RouteT& r) {
        if constexpr (has_open<RouteT>::value) return r.open;
        else return false;
    }

    template <class RouteT>
    static void setRouteWireLength(RouteT& r, double v) {
        if constexpr (has_wireLength<RouteT>::value) r.wireLength = v;
    }

    template <class RouteT>
    static double routeWireLength(const RouteT& r) {
        if constexpr (has_wireLength<RouteT>::value) return r.wireLength;
        else return 0.0;
    }

    // -----------------------------------------------------------------------------
    // Geometry helpers.
    // Edge id follows the problem statement: 1=left, 2=top, 3=right, 4=bottom.
    // -----------------------------------------------------------------------------

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
        if (!validEdge(a) || !validEdge(b)) return false;
        return !isOppositeLR(a, b) && !isOppositeTB(a, b) && a != b;
    }

    static double rectArea(const Rect& r) {
        return max(0.0, r.w) * max(0.0, r.h);
    }

    static bool rectOverlapAreaPositiveLocal(const Rect& a, const Rect& b) {
        return overlapLen(a.x, rectRight(a), b.x, rectRight(b)) > CHECKER_GEOM_EPS &&
            overlapLen(a.y, rectTop(a), b.y, rectTop(b)) > CHECKER_GEOM_EPS;
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

        // a right -> b left
        if (ea == 3 && eb == 1 && fabs(rectRight(a) - b.x) <= CONTACT_EPS) {
            double lo = max(a.y, b.y);
            double hi = min(rectTop(a), rectTop(b));
            if (hi <= lo + CONTACT_OVERLAP_EPS) return false;
            p = Point{ 0.5 * (rectRight(a) + b.x), 0.5 * (lo + hi) };
            return true;
        }

        // a left -> b right
        if (ea == 1 && eb == 3 && fabs(a.x - rectRight(b)) <= CONTACT_EPS) {
            double lo = max(a.y, b.y);
            double hi = min(rectTop(a), rectTop(b));
            if (hi <= lo + CONTACT_OVERLAP_EPS) return false;
            p = Point{ 0.5 * (a.x + rectRight(b)), 0.5 * (lo + hi) };
            return true;
        }

        // a top -> b bottom
        if (ea == 2 && eb == 4 && fabs(rectTop(a) - b.y) <= CONTACT_EPS) {
            double lo = max(a.x, b.x);
            double hi = min(rectRight(a), rectRight(b));
            if (hi <= lo + CONTACT_OVERLAP_EPS) return false;
            p = Point{ 0.5 * (lo + hi), 0.5 * (rectTop(a) + b.y) };
            return true;
        }

        // a bottom -> b top
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

    // -----------------------------------------------------------------------------
    // Block constraint helpers.
    // -----------------------------------------------------------------------------

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
                else cur.push_back(ch);
            }
            if (!cur.empty()) out.push_back(upperStr(cur));
        }
        return out;
    }

    static bool edgeBlockMatchesLocationUnion(const Rect& r, const vector<string>& locs, double W, double H) {
        struct SideRange {
            vector<pair<double, double>> intervals;
        };

        SideRange ranges[4]; // T, B, L, R
        auto sideIndex = [](char side) {
            if (side == 'T') return 0;
            if (side == 'B') return 1;
            if (side == 'L') return 2;
            return 3;
            };
        auto addRange = [&](char side, int zone) {
            int idx = sideIndex(side);
            double span = (side == 'T' || side == 'B') ? W : H;
            double lo = static_cast<double>(zone) * span / 3.0;
            double hi = static_cast<double>(zone + 1) * span / 3.0;
            ranges[idx].intervals.push_back({ lo, hi });
            };

        for (string loc : locs) {
            loc = upperStr(trim(loc));
            if (loc.size() < 2) continue;
            char a = loc[0];
            char b = loc[1];
            if (a == 'T' && (b == 'L' || b == 'M' || b == 'R')) addRange('T', (b == 'L') ? 0 : (b == 'M' ? 1 : 2));
            else if (a == 'B' && (b == 'L' || b == 'M' || b == 'R')) addRange('B', (b == 'L') ? 0 : (b == 'M' ? 1 : 2));
            else if (a == 'L' && (b == 'B' || b == 'M' || b == 'T')) addRange('L', (b == 'B') ? 0 : (b == 'M' ? 1 : 2));
            else if (a == 'R' && (b == 'B' || b == 'M' || b == 'T')) addRange('R', (b == 'B') ? 0 : (b == 'M' ? 1 : 2));
        }

        auto overlapsAny = [](double loVal, double hiVal, const vector<pair<double, double>>& intervals) {
            if (intervals.empty()) return true;
            for (const auto& seg : intervals) {
                if (min(hiVal, seg.second) > max(loVal, seg.first) + CONTACT_OVERLAP_EPS) return true;
            }
            return false;
            };

        bool requiredAnySide = false;

        if (!ranges[0].intervals.empty()) {
            requiredAnySide = true;
            if (fabs(rectTop(r) - H) > SHAPE_EPS || !overlapsAny(r.x, rectRight(r), ranges[0].intervals)) return false;
        }
        if (!ranges[1].intervals.empty()) {
            requiredAnySide = true;
            if (fabs(r.y) > SHAPE_EPS || !overlapsAny(r.x, rectRight(r), ranges[1].intervals)) return false;
        }
        if (!ranges[2].intervals.empty()) {
            requiredAnySide = true;
            if (fabs(r.x) > SHAPE_EPS || !overlapsAny(r.y, rectTop(r), ranges[2].intervals)) return false;
        }
        if (!ranges[3].intervals.empty()) {
            requiredAnySide = true;
            if (fabs(rectRight(r) - W) > SHAPE_EPS || !overlapsAny(r.y, rectTop(r), ranges[3].intervals)) return false;
        }

        return requiredAnySide;
    }

    static bool decodeLocationToken(const string& raw, char& side, int& zone) {
        const string loc = upperStr(trim(raw));
        if (loc.size() < 2) return false;

        const char a = loc[0];
        const char b = loc[1];
        if (a == 'T' && (b == 'L' || b == 'M' || b == 'R')) {
            side = 'T';
            zone = (b == 'L') ? 0 : (b == 'M' ? 1 : 2);
            return true;
        }
        if (a == 'B' && (b == 'L' || b == 'M' || b == 'R')) {
            side = 'B';
            zone = (b == 'L') ? 0 : (b == 'M' ? 1 : 2);
            return true;
        }
        if (a == 'L' && (b == 'B' || b == 'M' || b == 'T')) {
            side = 'L';
            zone = (b == 'B') ? 0 : (b == 'M' ? 1 : 2);
            return true;
        }
        if (a == 'R' && (b == 'B' || b == 'M' || b == 'T')) {
            side = 'R';
            zone = (b == 'B') ? 0 : (b == 'M' ? 1 : 2);
            return true;
        }
        return false;
    }

    static double intervalGap(double loA, double hiA, double loB, double hiB) {
        if (min(hiA, hiB) > max(loA, loB) + CONTACT_OVERLAP_EPS) return 0.0;
        if (hiA < loB) return loB - hiA;
        if (hiB < loA) return loA - hiB;
        return 0.0;
    }

    static double edgeLocationOffsetToTarget(const Rect& r, char side, int zone, double W, double H) {
        const bool horizontalSide = (side == 'T' || side == 'B');
        const double span = horizontalSide ? W : H;
        const double targetLo = static_cast<double>(zone) * span / 3.0;
        const double targetHi = static_cast<double>(zone + 1) * span / 3.0;

        double sideOffset = 0.0;
        double segLo = 0.0;
        double segHi = 0.0;
        if (side == 'T') {
            sideOffset = fabs(rectTop(r) - H);
            segLo = r.x;
            segHi = rectRight(r);
        }
        else if (side == 'B') {
            sideOffset = fabs(r.y);
            segLo = r.x;
            segHi = rectRight(r);
        }
        else if (side == 'L') {
            sideOffset = fabs(r.x);
            segLo = r.y;
            segHi = rectTop(r);
        }
        else {
            sideOffset = fabs(rectRight(r) - W);
            segLo = r.y;
            segHi = rectTop(r);
        }

        return sideOffset + intervalGap(segLo, segHi, targetLo, targetHi);
    }

    static double edgeBlockLocationOffset(const BlockInst& b, double W, double H) {
        if (b.spec.type != BlockType::EDGE) return 0.0;
        const vector<string> locs = splitLocTokens(b.spec.locations);
        if (locs.empty()) return 0.0;

        // Multiple zones on the same outline side are alternatives; different
        // sides (for example RB/BR) are simultaneous corner requirements.
        unordered_map<char, double> bestBySide;
        for (const string& loc : locs) {
            char side = 0;
            int zone = -1;
            if (!decodeLocationToken(loc, side, zone)) continue;
            const double offset = edgeLocationOffsetToTarget(b.rect, side, zone, W, H);
            auto it = bestBySide.find(side);
            if (it == bestBySide.end()) bestBySide.emplace(side, offset);
            else it->second = min(it->second, offset);
        }
        double total = 0.0;
        if (bestBySide.size() == 2) {
            // The reference checker reports corner displacement as the
            // difference between the two adjacent-side offsets. This exactly
            // reproduces RB/BR=(0,8)->8, BR/RB=(0,32)->32 and
            // RT/TR=(6,6)->0 from the supplied official reports.
            auto it = bestBySide.begin();
            const double a = it->second;
            const double b = (++it)->second;
            total = fabs(a - b);
        }
        else {
            for (const auto& kv : bestBySide) {
                if (kv.second > CHECKER_GEOM_EPS) total += kv.second;
            }
        }
        if (total <= CHECKER_GEOM_EPS) total = 0.0;
        return total;
    }

    static bool edgeBlockLocationOK(const BlockInst& b, double W, double H) {
        if (b.spec.type != BlockType::EDGE) return true;
        return edgeBlockLocationOffset(b, W, H) <= 0.0;
    }

    static bool blockPortEdgeOK(const BlockInst& b, int edge) {
        if (!validEdge(edge) || b.spec.portEdges.empty()) return true;
        return find(b.spec.portEdges.begin(), b.spec.portEdges.end(), edge) != b.spec.portEdges.end();
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

    // Directional component capacities.
    // LR means internal traversal between edge 1 and edge 3. Horizontal wires
    // are packed along the channel height, so LR capacity uses rectangle height.
    // TB means internal traversal between edge 2 and edge 4. Vertical wires are
    // packed along the channel width, so TB capacity uses rectangle width.
    static double channelLRCapacity(const Rect& r) {
        return max(0.0, r.h) * CHANNEL_DENSITY;
    }

    static double channelTBCapacity(const Rect& r) {
        return max(0.0, r.w) * CHANNEL_DENSITY;
    }

    static void addChannelDirectionalUse(ChannelUse& use, int inEdge, int outEdge, double nets) {
        if (inEdge == outEdge) {
            if (inEdge == 1 || inEdge == 3) use.lrNets += nets;
            else if (inEdge == 2 || inEdge == 4) use.tbNets += nets;
            return;
        }

        if (isOppositeLR(inEdge, outEdge)) {
            use.lrNets += nets;
        }
        else if (isOppositeTB(inEdge, outEdge)) {
            use.tbNets += nets;
        }
        else if (isTurn(inEdge, outEdge)) {
            // L-shaped inside this CH*: count one horizontal/LR component and one
            // vertical/TB component.
            use.lrNets += nets;
            use.tbNets += nets;
        }
    }

    static double ftRateForNetsLocal(const BlockSpec& spec, double ftNets) {
        return feedthroughRateForNets(spec, ftNets);
    }

    static double requiredAreaWithFeedthroughForAnyBlock(const BlockInst& b, double ftNets) {
        const double baseArea = max(1.0, max(rectArea(b.rect), b.spec.area));
        if (ftNets <= EPS) return baseArea;

        const double rate = ftRateForNetsLocal(b.spec, ftNets);
        const double delta = (ftNets / CHANNEL_DENSITY) * rate / 2.0;
        const double side = sqrt(baseArea) + delta;
        return max(baseArea, side * side);
    }

    struct RoutingRecomputeSummary {
        vector<double> illegalFeedthroughUsed;
        double illegalFeedthroughDeltaArea = 0.0;
        int illegalFeedthroughCount = 0;
    };


    enum class RouteInvalidReason {
        NONE,
        EXPLICIT_OPEN,
        BAD_NETS_OR_TOO_SHORT,
        BAD_ITEM,
        UNKNOWN_OBJECT,
        BAD_TOPOLOGY,
        ILLEGAL_FEEDTHROUGH,
        CONTACT_FAIL,
        SAME_OBJECT_SAME_EDGE,
    };

    static const char* reasonName(RouteInvalidReason r) {
        switch (r) {
        case RouteInvalidReason::NONE: return "NONE";
        case RouteInvalidReason::EXPLICIT_OPEN: return "EXPLICIT_OPEN";
        case RouteInvalidReason::BAD_NETS_OR_TOO_SHORT: return "BAD_NETS_OR_TOO_SHORT";
        case RouteInvalidReason::BAD_ITEM: return "BAD_ITEM";
        case RouteInvalidReason::UNKNOWN_OBJECT: return "UNKNOWN_OBJECT";
        case RouteInvalidReason::BAD_TOPOLOGY: return "BAD_TOPOLOGY";
        case RouteInvalidReason::ILLEGAL_FEEDTHROUGH: return "ILLEGAL_FEEDTHROUGH";
        case RouteInvalidReason::CONTACT_FAIL: return "CONTACT_FAIL";
        case RouteInvalidReason::SAME_OBJECT_SAME_EDGE: return "SAME_OBJECT_SAME_EDGE";
        }
        return "UNKNOWN";
    }

    static void logInvalidRoute(
        int routeId,
        int nets,
        const vector<RouteItemView>& items,
        RouteInvalidReason reason,
        const string& detail
    ) {
        // Do not flood huge testcases.  The first 200 invalid-route diagnostics are
        // usually enough to identify whether the issue is topology, contact, or an
        // illegal feedthrough.
        static int printed = 0;
        static constexpr int MAX_PRINT = 200;
        if (printed >= MAX_PRINT) return;
        ++printed;

        cerr << "[EvalInvalidRoute] routeId=" << routeId
            << " nets=" << nets
            << " reason=" << reasonName(reason)
            << " items=" << items.size();
        if (!items.empty()) {
            cerr << " src=" << items.front().name
                << " dst=" << items.back().name;
        }
        if (!detail.empty()) cerr << " detail=" << detail;
        cerr << "\n";
    }

    static bool validatePathTopology(
        const vector<RouteItemView>& items,
        const unordered_map<string, int>& blockMap,
        const unordered_map<string, int>& channelMap,
        const Design& design
    ) {
        (void)design;
        if (items.size() < 2) return false;
        if ((items.size() % 2) != 0) return false;
        if (!blockMap.count(items.front().name)) return false;
        if (!blockMap.count(items.back().name)) return false;

        for (int i = 1; i + 1 < static_cast<int>(items.size()); i += 2) {
            const auto& in = items[i];
            const auto& out = items[i + 1];

            if (in.name != out.name) return false;
            if (!validEdge(in.edge) || !validEdge(out.edge)) return false;
            auto cit = channelMap.find(in.name);
            if (cit != channelMap.end()) continue;

            auto bit = blockMap.find(in.name);
            if (bit == blockMap.end()) return false;
        }

        return true;
    }


    static RouteInvalidReason validatePathTopologyReason(
        const vector<RouteItemView>& items,
        const unordered_map<string, int>& blockMap,
        const unordered_map<string, int>& channelMap,
        const Design& design,
        string& detail
    ) {
        (void)design;
        if (items.size() < 2) { detail = "path has fewer than 2 items"; return RouteInvalidReason::BAD_NETS_OR_TOO_SHORT; }
        if ((items.size() % 2) != 0) { detail = "item count is odd; intermediate rectangles must be in/out pairs"; return RouteInvalidReason::BAD_TOPOLOGY; }
        if (!blockMap.count(items.front().name)) { detail = "first item is not a block: " + items.front().name; return RouteInvalidReason::BAD_TOPOLOGY; }
        if (!blockMap.count(items.back().name)) { detail = "last item is not a block: " + items.back().name; return RouteInvalidReason::BAD_TOPOLOGY; }

        for (int i = 1; i + 1 < static_cast<int>(items.size()); i += 2) {
            const auto& in = items[i];
            const auto& out = items[i + 1];

            if (in.name != out.name) {
                ostringstream oss;
                oss << "bad in/out pair at items[" << i << "," << (i + 1) << "]: "
                    << in.name << " vs " << out.name;
                detail = oss.str();
                return RouteInvalidReason::BAD_TOPOLOGY;
            }
            if (!validEdge(in.edge) || !validEdge(out.edge)) {
                detail = "invalid edge in intermediate pair: " + in.name;
                return RouteInvalidReason::BAD_ITEM;
            }
            auto cit = channelMap.find(in.name);
            if (cit != channelMap.end()) continue;

            auto bit = blockMap.find(in.name);
            if (bit == blockMap.end()) {
                detail = "intermediate object is neither channel nor block: " + in.name;
                return RouteInvalidReason::UNKNOWN_OBJECT;
            }
        }

        detail.clear();
        return RouteInvalidReason::NONE;
    }

    static bool validConnectionMatrixShape(const Design& design, int n) {
        if (static_cast<int>(design.connMatrix.size()) != n) return false;
        for (const auto& row : design.connMatrix) {
            if (static_cast<int>(row.size()) != n) return false;
        }
        return true;
    }

    static void addPairDemand(vector<vector<long long>>& demand, int a, int b, int nets) {
        if (nets <= 0) return;
        if (a < 0 || b < 0) return;
        if (a >= static_cast<int>(demand.size()) || b >= static_cast<int>(demand.size())) return;

        // PATH geometry is undirected at this abstraction level; aggregate both
        // matrix directions into one physical block-pair requirement.
        if (b < a) swap(a, b);
        demand[a][b] += static_cast<long long>(nets);
    }

    static vector<vector<long long>> buildExpectedConnectionDemand(const Design& design) {
        const int n = static_cast<int>(design.blocks.size());
        vector<vector<long long>> expected(n, vector<long long>(n, 0));

        if (validConnectionMatrixShape(design, n)) {
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    addPairDemand(expected, i, j, design.connMatrix[i][j]);
                }
            }
            return expected;
        }

        for (const auto& c : design.connections) {
            addPairDemand(expected, c.src, c.dst, c.netCount);
        }

        return expected;
    }

    static vector<vector<long long>> buildActualRoutedDemand(
        const Design& design,
        const unordered_map<string, int>& blockMap
    ) {
        const int n = static_cast<int>(design.blocks.size());
        vector<vector<long long>> actual(n, vector<long long>(n, 0));

        for (const auto& route : design.routes) {
            if (routeOpen(route)) continue;
            const int nets = routeNetCount(route);
            if (nets <= 0) continue;

            const vector<RouteItemView> items = getRouteItems(route);
            if (items.size() < 2) continue;

            auto sit = blockMap.find(items.front().name);
            auto dit = blockMap.find(items.back().name);
            if (sit == blockMap.end() || dit == blockMap.end()) continue;

            addPairDemand(actual, sit->second, dit->second, nets);
        }

        return actual;
    }

    static int countConnectionCoverageIssues(const Design& design) {
        const int n = static_cast<int>(design.blocks.size());
        if (n <= 0) return 0;

        const auto blockMap = buildBlockMap(design);
        const auto expected = buildExpectedConnectionDemand(design);
        const auto actual = buildActualRoutedDemand(design, blockMap);

        int issues = 0;
        static int printed = 0;
        static constexpr int MAX_PRINT = 120;

        for (int i = 0; i < n; ++i) {
            for (int j = i; j < n; ++j) {
                if (actual[i][j] < expected[i][j]) {
                    ++issues;
                    if (printed < MAX_PRINT) {
                        ++printed;
                        cerr << "[EvalConnectionCoverage] missing_or_under_routed pair="
                            << design.blocks[i].spec.name << "-" << design.blocks[j].spec.name
                            << " expected=" << expected[i][j]
                            << " actual=" << actual[i][j] << "\n";
                    }
                }
                else if (actual[i][j] > expected[i][j]) {
                    ++issues;
                    if (printed < MAX_PRINT) {
                        ++printed;
                        cerr << "[EvalConnectionCoverage] extra_routed pair="
                            << design.blocks[i].spec.name << "-" << design.blocks[j].spec.name
                            << " expected=" << expected[i][j]
                            << " actual=" << actual[i][j] << "\n";
                    }
                }
            }
        }

        return issues;
    }

    static bool checkInternalOutputFormatLike(const Design& design) {
        if (design.outlineW <= EPS || design.outlineH <= EPS) return true;

        unordered_set<string> names;
        names.reserve(design.blocks.size() + design.channels.size());

        for (const auto& b : design.blocks) {
            if (b.spec.name.empty()) return true;
            if (!names.insert(b.spec.name).second) return true;
        }

        for (const auto& ch : design.channels) {
            if (ch.name.empty() || ch.name.size() > 30) return true;
            if (!names.insert(ch.name).second) return true;
        }

        const auto blockMap = buildBlockMap(design);
        const auto channelMap = buildChannelMap(design);

        for (const auto& route : design.routes) {
            if (routeNetCount(route) <= 0) return true;

            const vector<RouteItemView> items = getRouteItems(route);
            if (items.size() < 2 || (items.size() % 2) != 0) return true;

            for (const auto& item : items) {
                if (item.name.empty() || !validEdge(item.edge)) return true;
                if (!blockMap.count(item.name) && !channelMap.count(item.name)) return true;
            }

            if (!blockMap.count(items.front().name)) return true;
            if (!blockMap.count(items.back().name)) return true;
        }

        return false;
    }

    template <class RouteT>
    static void validateAndAccumulateOneRoute(
        Design& design,
        RouteT& route,
        int routeId,
        const unordered_map<string, int>& blockMap,
        const unordered_map<string, int>& channelMap,
        vector<ChannelUse>& channelUse,
        RoutingRecomputeSummary& summary
    ) {
        const bool explicitOpen = routeOpen(route);
        const int nets = routeNetCount(route);
        const vector<RouteItemView> items = getRouteItems(route);

        bool invalid = false;
        RouteInvalidReason firstReason = RouteInvalidReason::NONE;
        string firstDetail;

        auto markInvalid = [&](RouteInvalidReason r, const string& d) {
            if (!invalid) {
                invalid = true;
                firstReason = r;
                firstDetail = d;
            }
            };

        double wl = 0.0;
        vector<Point> guiding;

        if (explicitOpen) {
            // Preserve explicit router-open paths.  They remain routing-open.
            markInvalid(RouteInvalidReason::EXPLICIT_OPEN, "route.open was already true before evaluator validation");
        }

        if (nets <= 0 || items.size() < 2) {
            markInvalid(RouteInvalidReason::BAD_NETS_OR_TOO_SHORT, "non-positive nets or too few path items");
        }

        for (int idx = 0; idx < static_cast<int>(items.size()); ++idx) {
            const auto& it = items[idx];
            if (!validEdge(it.edge) || it.name.empty()) {
                ostringstream oss;
                oss << "bad item at index " << idx << " name='" << it.name << "' edge=" << it.edge;
                markInvalid(RouteInvalidReason::BAD_ITEM, oss.str());
                continue;
            }
            Rect rr;
            bool isBlock = false, isChannel = false;
            if (!getObjectRect(it.name, design, blockMap, channelMap, rr, isBlock, isChannel)) {
                ostringstream oss;
                oss << "unknown object at index " << idx << ": " << it.name;
                markInvalid(RouteInvalidReason::UNKNOWN_OBJECT, oss.str());
            }
        }

        if (!invalid) {
            auto firstBlockIt = blockMap.find(items.front().name);
            auto lastBlockIt = blockMap.find(items.back().name);
            if (firstBlockIt != blockMap.end() &&
                !blockPortEdgeOK(design.blocks[firstBlockIt->second], items.front().edge)) {
                ostringstream oss;
                oss << "source block " << items.front().name
                    << " uses disallowed port edge " << items.front().edge;
                markInvalid(RouteInvalidReason::BAD_ITEM, oss.str());
            }
            else if (lastBlockIt != blockMap.end() &&
                !blockPortEdgeOK(design.blocks[lastBlockIt->second], items.back().edge)) {
                ostringstream oss;
                oss << "destination block " << items.back().name
                    << " uses disallowed port edge " << items.back().edge;
                markInvalid(RouteInvalidReason::BAD_ITEM, oss.str());
            }
        }

        if (!invalid) {
            string topoDetail;
            RouteInvalidReason topo = validatePathTopologyReason(items, blockMap, channelMap, design, topoDetail);
            if (topo != RouteInvalidReason::NONE) {
                markInvalid(topo, topoDetail);
            }
        }

        // Still compute best-effort wirelength / resource usage for invalid routes
        // so diagnostics remain meaningful.  The route is marked open below; this
        // keeps local fail accounting conservative against the official checker.
        if (!items.empty()) {
            for (int i = 0; i + 1 < static_cast<int>(items.size()); ++i) {
                const RouteItemView& a = items[i];
                const RouteItemView& b = items[i + 1];

                Rect ra, rb;
                bool aBlock = false, aCh = false, bBlock = false, bCh = false;
                if (!getObjectRect(a.name, design, blockMap, channelMap, ra, aBlock, aCh)) {
                    continue;
                }
                if (!getObjectRect(b.name, design, blockMap, channelMap, rb, bBlock, bCh)) {
                    continue;
                }

                if (a.name == b.name) {
                    auto cit = channelMap.find(a.name);
                    if (cit != channelMap.end()) {
                        if (validEdge(a.edge) && validEdge(b.edge)) {
                            addChannelDirectionalUse(channelUse[cit->second], a.edge, b.edge, static_cast<double>(max(0, nets)));
                        }
                    }
                    else {
                        auto bit = blockMap.find(a.name);
                        if (bit != blockMap.end()) {
                            BlockInst& blk = design.blocks[bit->second];
                            if (blk.spec.type == BlockType::SOFT) {
                                blk.ftUsed += static_cast<double>(max(0, nets));
                            }
                            else {
                                if (bit->second >= 0 && bit->second < static_cast<int>(summary.illegalFeedthroughUsed.size())) {
                                    summary.illegalFeedthroughUsed[bit->second] += static_cast<double>(max(0, nets));
                                }
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
                            << " " << a.name << ":e" << a.edge
                            << " to " << b.name << ":e" << b.edge;
                        markInvalid(RouteInvalidReason::CONTACT_FAIL, oss.str());
                    }
                    // Keep WL finite.  This avoids treating numerical/geometry
                    // diagnosis as routing-open while still giving a comparable cost.
                    guiding.push_back(edgeCenter(ra, a.edge));
                    guiding.push_back(edgeCenter(rb, b.edge));
                }
            }
        }

        for (int i = 0; i + 1 < static_cast<int>(guiding.size()); ++i) {
            wl += manhattanPoint(guiding[i], guiding[i + 1]) * static_cast<double>(max(0, nets));
        }

        if (invalid && !explicitOpen) {
            logInvalidRoute(routeId, nets, items, firstReason, firstDetail);
        }

        // Treat evaluator-detected topology/contact/legality failures as routing
        // opens.  Otherwise local reports can pass paths that the official checker
        // is expected to reject.
        setRouteOpen(route, explicitOpen || invalid);
        setRouteWireLength(route, wl);
    }

    static RoutingRecomputeSummary recomputeRoutingStatsFromPaths(Design& design) {
        auto blockMap = buildBlockMap(design);
        auto channelMap = buildChannelMap(design);

        vector<ChannelUse> channelUse(design.channels.size());
        RoutingRecomputeSummary summary;
        summary.illegalFeedthroughUsed.assign(design.blocks.size(), 0.0);

        for (auto& ch : design.channels) {
            ch.lrUsed = 0.0;
            ch.tbUsed = 0.0;
            ch.lrCapacity = 0.0;
            ch.tbCapacity = 0.0;
            ch.lrOverflow = 0.0;
            ch.tbOverflow = 0.0;
            ch.usedNets = 0.0;
            ch.capacity = 0.0;
            ch.overflow = 0.0;
        }
        for (auto& b : design.blocks) {
            b.ftUsed = 0.0;
            b.ftOverflowArea = 0.0;
        }

        for (int rid = 0; rid < static_cast<int>(design.routes.size()); ++rid) {
            auto& route = design.routes[rid];
            validateAndAccumulateOneRoute(design, route, rid, blockMap, channelMap, channelUse, summary);
        }

        for (int i = 0; i < static_cast<int>(design.channels.size()); ++i) {
            auto& ch = design.channels[i];
            const ChannelUse& u = channelUse[i];

            const double capLR = channelLRCapacity(ch.rect);
            const double capTB = channelTBCapacity(ch.rect);
            const double ovLR = max(0.0, u.lrNets - capLR);
            const double ovTB = max(0.0, u.tbNets - capTB);

            ch.lrUsed = u.lrNets;
            ch.tbUsed = u.tbNets;
            ch.lrCapacity = capLR;
            ch.tbCapacity = capTB;
            ch.lrOverflow = ovLR;
            ch.tbOverflow = ovTB;

            // Legacy scalar fields:
            //   usedNets/capacity are set to the dominant-utilization component so
            //   Logger's one-number utilization still points to the bottleneck.
            //   overflow is the correct sum of independent LR/TB overflows.
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
        }

        for (int i = 0; i < static_cast<int>(design.blocks.size()); ++i) {
            const double illegalUsed = summary.illegalFeedthroughUsed[i];
            if (illegalUsed <= EPS) continue;
            const double baseArea = max(1.0, max(rectArea(design.blocks[i].rect), design.blocks[i].spec.area));
            const double requiredArea = requiredAreaWithFeedthroughForAnyBlock(design.blocks[i], illegalUsed);
            summary.illegalFeedthroughDeltaArea += max(0.0, requiredArea - baseArea);
            ++summary.illegalFeedthroughCount;
        }

        return summary;
    }

} // namespace

EvalReport Evaluator::evaluate(Design& design, double alpha, double runtimeSec) {
    EvalReport rpt;

    // Important: recompute routing statistics from PATH geometry.  Do not trust
    // stale p.wireLength, ch.usedNets, ch.capacity, or b.ftUsed left by the router.
    RoutingRecomputeSummary routingSummary = recomputeRoutingStatsFromPaths(design);
    rpt.formatFailed = checkInternalOutputFormatLike(design);

    rpt.outlineArea = design.outlineW * design.outlineH;
    rpt.totalWireLength = calcTotalWireLength(design);
    rpt.baseCost = rpt.outlineArea + alpha * rpt.totalWireLength;

    rpt.blockOverlap = checkBlockOverlap(design, rpt.overlapCount);
    rpt.outlineViolation = checkOutlineViolation(design, rpt.outlineViolationCount);
    rpt.routingOpen = checkRoutingOpen(design, rpt.openPathCount);

    calcChannelOverflow(design, rpt.totalChannelOverflow, rpt.maxChannelOverflow, rpt.totalChannelCapacity);
    calcFeedthroughOverflow(design, rpt.totalFeedthroughOverflow, rpt.maxFeedthroughOverflow);

    vector<rxscore::BlockFt> ftBlocks;
    ftBlocks.reserve(design.blocks.size());
    for (int i = 0; i < static_cast<int>(design.blocks.size()); ++i) {
        const BlockInst& b = design.blocks[i];
        rxscore::BlockFt ft;
        // The checker uses declared AREA for SOFT blocks, but the emitted
        // rectangle area for fixed HARD/EDGE blocks (which may differ slightly
        // within the 3um geometry tolerance).
        ft.baseArea = b.spec.type == BlockType::SOFT
            ? max(0.0, b.spec.area)
            : rectArea(b.rect);
        ft.kind = b.spec.type == BlockType::SOFT ? rxscore::BlockKind::Soft
            : (b.spec.type == BlockType::EDGE ? rxscore::BlockKind::Edge : rxscore::BlockKind::Hard);
        const double rawUsed = b.spec.type == BlockType::SOFT
            ? b.ftUsed
            : routingSummary.illegalFeedthroughUsed[i];
        ft.ftUsedRaw = max<long long>(0, llround(rawUsed));
        for (int tier = 0; tier < 4; ++tier) ft.ftRate[tier] = b.spec.ftRate[tier];
        ftBlocks.push_back(ft);
    }
    const rxscore::FtResult officialFt = rxscore::feedthroughPenalty(ftBlocks, rpt.outlineArea);
    rpt.totalFeedthroughOverflow = officialFt.areaExcess;
    rpt.maxFeedthroughOverflow = officialFt.areaExcess;
    rpt.illegalFeedthroughDeltaArea = officialFt.illegalDeltaArea;
    rpt.illegalFeedthroughCount = officialFt.illegalBlocks;

    for (const auto& b : design.blocks) {
        const double offset = edgeBlockLocationOffset(b, design.outlineW, design.outlineH);
        rpt.edgeLocationOffset += offset;
        if (b.spec.type == BlockType::EDGE &&
            !edgeBlockMatchesLocationUnion(
                b.rect, splitLocTokens(b.spec.locations), design.outlineW, design.outlineH)) {
            ++rpt.edgeLocationViolationCount;
        }
    }

    rpt.channelOverflowRate = rpt.totalChannelCapacity > EPS
        ? rpt.totalChannelOverflow / rpt.totalChannelCapacity
        : 0.0;

    if (rpt.channelOverflowRate <= CHECKER_OVERFLOW_THRESHOLD) {
        rpt.overflowPenalty = CHECKER_BETA_LIN * rpt.channelOverflowRate * rpt.outlineArea;
    }
    else {
        rpt.overflowPenalty =
            CHECKER_BETA_LIN * CHECKER_OVERFLOW_THRESHOLD * rpt.outlineArea +
            CHECKER_BETA_TAIL * sqrt(max(0.0, rpt.channelOverflowRate - CHECKER_OVERFLOW_THRESHOLD)) * rpt.outlineArea;
    }
    if (rpt.totalChannelOverflow <= EPS) rpt.overflowPenalty = 0.0;

    rpt.feedthroughPenalty = CHECKER_BETA_FT * rpt.totalFeedthroughOverflow;
    rpt.illegalFeedthroughPenalty = CHECKER_LAMBDA_FT * rpt.illegalFeedthroughDeltaArea;
    rpt.edgeLocationPenalty = CHECKER_GAMMA_EDGE * rpt.edgeLocationOffset;
    rpt.runtimePenalty = CHECKER_DELTA_RUNTIME * max(0.0, runtimeSec) * sqrt(max(0.0, rpt.outlineArea));
    rpt.warningPenaltyCost =
        rpt.overflowPenalty +
        rpt.feedthroughPenalty +
        rpt.illegalFeedthroughPenalty +
        rpt.edgeLocationPenalty;
    rpt.cost = rpt.baseCost + rpt.warningPenaltyCost + rpt.runtimePenalty;

    return rpt;
}

double Evaluator::calcTotalWireLength(const Design& design) const {
    double sum = 0.0;
    for (const auto& p : design.routes) sum += routeWireLength(p);
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
    if (design.outlineW > design.maxOutlineW + CHECKER_GEOM_EPS ||
        design.outlineH > design.maxOutlineH + CHECKER_GEOM_EPS) ++violationCount;

    for (const auto& b : design.blocks) {
        const Rect& r = b.rect;
        if (r.w <= EPS || r.h <= EPS) {
            ++violationCount;
            continue;
        }

        if (r.x < -CHECKER_GEOM_EPS ||
            r.y < -CHECKER_GEOM_EPS ||
            rectRight(r) > design.outlineW + CHECKER_GEOM_EPS ||
            rectTop(r) > design.outlineH + CHECKER_GEOM_EPS) {
            ++violationCount;
        }

        // EDGE location is a checker warning with penalty, not a hard fail.
        if (!blockShapeOK(b)) ++violationCount;
    }
    return violationCount > 0;
}

bool Evaluator::checkRoutingOpen(const Design& design, int& openPathCount) const {
    // Coverage catches normal shortages and over-routing. A malformed/open
    // redundant PATH must still fail even when another valid PATH happens to
    // satisfy the same pair demand; coverage alone would miss it.
    openPathCount = countConnectionCoverageIssues(design);
    int invalidRoutes = 0;
    for (const auto& route : design.routes) {
        if (routeOpen(route)) ++invalidRoutes;
    }
    if (openPathCount == 0) openPathCount = invalidRoutes;
    return openPathCount > 0 || invalidRoutes > 0;
}

void Evaluator::calcChannelOverflow(Design& design, double& totalOverflow, double& maxOverflow, double& totalCapacity) const {
    totalOverflow = 0.0;
    maxOverflow = 0.0;
    totalCapacity = 0.0;

    // evaluate() already calls recomputeRoutingStatsFromPaths().  This function
    // only sums the channel overflow stored on each channel, so it remains cheap.
    for (auto& ch : design.channels) {
        ch.overflow = max(0.0, ch.overflow);
        totalOverflow += ch.overflow;
        maxOverflow = max(maxOverflow, ch.overflow);
        totalCapacity += channelLRCapacity(ch.rect) + channelTBCapacity(ch.rect);
    }
}

double Evaluator::ftRateForNets(const BlockSpec& spec, double ftNets) const {
    return ftRateForNetsLocal(spec, ftNets);
}

double Evaluator::estimateRequiredAreaWithFT(const BlockInst& b) const {
    return requiredSoftAreaWithFeedthrough(b);
}
void Evaluator::calcFeedthroughOverflow(Design& design, double& totalOverflow, double& maxOverflow) const {
    totalOverflow = 0.0;
    maxOverflow = 0.0;

    // b.ftUsed has been recomputed from PATH same-block traversals in evaluate().
    for (auto& b : design.blocks) {
        b.ftOverflowArea = 0.0;
        if (b.spec.type != BlockType::SOFT || b.ftUsed <= EPS) continue;

        double currentArea = b.rect.w * b.rect.h;
        double requiredArea = estimateRequiredAreaWithFT(b);
        b.ftOverflowArea = max(0.0, requiredArea - currentArea);
        totalOverflow += b.ftOverflowArea;
        maxOverflow = max(maxOverflow, b.ftOverflowArea);
    }
}
