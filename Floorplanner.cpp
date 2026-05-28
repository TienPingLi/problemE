#include "Floorplanner.hpp"
#include "Utility.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <future>
#include <limits>
#include <sstream>
#include <numeric>
#include <random>
#include <unordered_map>
#include <vector>

using namespace std;

namespace {

    // ============================================================================
    //  Congestion-aware SA Floorplanner
    // ----------------------------------------------------------------------------
    //  This file intentionally does NOT use the old one-block-at-a-time greedy
    //  placement as the main algorithm.  The floorplan is a continuous-coordinate
    //  simulated annealing state.  Cost contains:
    //      1) legal placement terms: overlap / outline / edge constraint
    //      2) HPWL term from the testcase connection matrix
    //      3) congestion prediction term inspired by Sham-Young-Lu 3-step model:
    //            preliminary bbox density -> weighted diagonal/SMD-like spreading
    //            -> local redistribution from over-congested tiles
    //      4) soft-block feedthrough risk term
    //
    //  Design rules handled here:
    //      - EDGE block is always re-projected to its specified boundary region.
    //      - HARD/EDGE blocks are routing blockages in the estimator.
    //      - SOFT blocks allow feedthrough, but estimated FT demand is penalized.
    //      - Output rectangles remain legal rectangles.  ChannelBuilder will later
    //        generate contest-format vertical-strip channels from these rectangles.
    // ============================================================================

    //static constexpr double W_CHANNEL_SPACING = 2.0;
    //...
    //    cb.spacing = minGapPenalty(rects, 120.0);
    //...
    //    capByDeadspace = clampDouble(capByDeadspace, 0.0, 0.080);


    static constexpr double INF_COST = 1.0e100;
    static constexpr double TINY = 1.0e-9;

    // SA iteration controls.  For small case00 this is still fast; for larger cases
    // it scales linearly enough for a first contest-quality baseline.
    static constexpr int SA_BASE_ITERS = 4500;
    static constexpr int SA_ITERS_PER_BLOCK = 420;
    static constexpr int SA_RESTARTS = 16;

    // Parallel restarts are independent, so they are the safest way to use
    // multicore CPUs without changing the search behavior.  Keep this small to
    // avoid memory pressure from congestion buffers on very large cases.
    static constexpr bool ENABLE_PARALLEL_RESTARTS = true;
    static constexpr int MAX_PARALLEL_RESTARTS = 4;

    // Grid for congestion prediction.  Keep moderate: this is evaluated thousands
    // of times inside SA.
    static constexpr int GRID_X_MAX = 28;
    static constexpr int GRID_Y_MAX = 22;
    static constexpr int GRID_X_MIN = 10;
    static constexpr int GRID_Y_MIN = 8;

    // Cost weights.  These are deliberately separated so you can tune easily.
    static constexpr double W_AREA = 0.100;
    static constexpr double W_HPWL = 0.20;
    static constexpr double W_CONGESTION = 0.35;//////////////////////////////////0.35
    static constexpr double W_PEAK = 250.0;//25
    static constexpr double W_FT = 0.005;
    static constexpr double W_OVERLAP_AREA = 2.0e7;
    static constexpr double W_OVERLAP_COUNT = 2.0e11;
    static constexpr double W_OUTLINE = 2.0e10;
    static constexpr double W_EDGE = 2.0e10;
    static constexpr double W_CENTER = 1.0e-3;
    static constexpr double W_CHANNEL_SPACING = 6.0;//////////////////was 1.0; stronger preference for real channel whitespace
    // Direct channel capacity guard.  If a connection is routed directly through a
    // channel strip, the strip must be at least netCount / CHANNEL_DENSITY wide.
    // Example: 1500 nets need 1500 / 25 = 60um.  This prevents sliver channels
    // such as width=0.25 from being attractive to the router.
    static constexpr double W_DIRECT_CHANNEL_CAPACITY = 120.0;
    static constexpr double DIRECT_CHANNEL_SAFETY_MARGIN = 1.0;
    static constexpr double MIN_DIRECT_CHANNEL_GUARD_NETS = 25.0;

    // Global sliver guard.  Router must not treat a 0.x um whitespace as a
    // routable channel.  This term detects the actual vertical-strip channels
    // that ChannelBuilder will generate and strongly discourages tiny usable
    // dimensions even when the tiny strip is not between directly connected blocks.
    static constexpr double W_GLOBAL_CHANNEL_SLIVER = 180.0;
    static constexpr double SLIVER_SNAP_ABS = 2.0;       // snap purely numerical gaps closed
    static constexpr double SLIVER_SNAP_RATIO = 0.08;    // also snap gaps below 8% of max demand
    static constexpr double W_SOFT_BRIDGE = 80.0;//多容易走softblock 80
    static constexpr double W_COMMON_EDGE = 0.05;
    static constexpr double COMMON_EDGE_GAP = 80.0;

    struct FPState {
        vector<Rect> rects;
        vector<double> softRatio;   // same size as rects; valid for soft only

        // 1D edge floorplan state.  EDGE blocks are not independently projected
        // to a corner anymore.  Each edge block has:
        //   edgeRule[id] : which allowed boundary region is selected, e.g. TL vs LT
        //   edgePos[id]  : preferred 1D center position on that chosen boundary, [0,1]
        // A side-level 1D legalizer packs all edge blocks on the same boundary
        // without overlap.  SA mutates these fields like B*-tree perturbations:
        // move-position, swap-position, and change selected boundary rule.
        vector<double> edgePos;
        vector<int> edgeRule;

        // The outline is part of the SA state.  This is the key change that lets
        // outlineArea shrink.  EDGE blocks are re-projected to these dimensions
        // whenever the state is evaluated.
        double outlineW = 0.0;
        double outlineH = 0.0;
    };

    struct CostBreakdown {
        double total = INF_COST;
        double area = 0.0;
        double hpwl = 0.0;
        double congestion = 0.0;
        double peak = 0.0;
        double ft = 0.0;
        double overlap = 0.0;
        double outline = 0.0;
        double edge = 0.0;
        double spacing = 0.0;
        double channelCapacity = 0.0;
        double channelSliver = 0.0;
        double softBridge = 0.0;
        double commonEdgeReward = 0.0;

        // Fast legality summary filled by evaluateState().
        // This avoids calling isLegalState() again after every evaluated candidate.
        double rawOverlapArea = 0.0;
        int rawOverlapCount = 0;
        double rawOutlineViolation = 0.0;
    };

    static bool costPlacementLegal(const CostBreakdown& cb) {
        return cb.rawOverlapCount == 0 && cb.rawOverlapArea <= EPS && cb.rawOutlineViolation <= EPS;
    }

    struct TileInfo {
        double prelim = 0.0;
        double demand = 0.0;
        double capacity = 0.0;
        double blockage = 0.0;  // 0: open, 1: complete blockage
        int softBlock = -1;     // tile center is inside this soft block, if any
    };

    static double rand01(mt19937& rng) {
        return uniform_real_distribution<double>(0.0, 1.0)(rng);
    }

    static double randRange(mt19937& rng, double lo, double hi) {
        return uniform_real_distribution<double>(lo, hi)(rng);
    }

    static int randInt(mt19937& rng, int lo, int hiInclusive) {
        return uniform_int_distribution<int>(lo, hiInclusive)(rng);
    }

    static double sqr(double v) { return v * v; }

    static bool isMovableForSA(const BlockSpec& spec) {
        return spec.type != BlockType::EDGE;
    }

    static double clampDouble(double v, double lo, double hi) {
        if (hi < lo) return lo;
        return max(lo, min(v, hi));
    }

    static string primaryLocation(const BlockSpec& spec) {
        return spec.locations.empty() ? string("BL") : spec.locations[0];
    }

    static bool locContains(const string& loc, char c) {
        return loc.find(c) != string::npos;
    }

    static double aspectMid(const BlockSpec& spec) {
        double amin = max(0.05, spec.aspectMin);
        double amax = max(amin, spec.aspectMax);
        // geometric mean is more stable for ranges like 0.5~2.0
        return sqrt(amin * amax);
    }

    static double totalOriginalBlockArea(const Design& design) {
        // Hierarchy of memories: block area is invariant for one testcase.
        // Cache it so DSU / evaluation helpers do not rescan blockSpecs.
        static unordered_map<const Design*, double> cache;
        auto it = cache.find(&design);
        if (it != cache.end()) return it->second;

        double a = 0.0;
        for (const auto& spec : design.blockSpecs) {
            if (spec.hasFixedSize) a += spec.fixedW * spec.fixedH;
            else a += max(1.0, spec.area);
        }
        cache[&design] = a;
        return a;
    }

    // -------------------------------------------------------------------------
    // Dynamic congestion pressure
    // -------------------------------------------------------------------------
    // The congestion map already scales with each connection's netCount, because
    // netCount is added into the tile demand.  This factor additionally changes
    // how much SA cares about congestion when the testcase has more nets per
    // block.  Example: same block count but 2x total nets => larger pressure =>
    // stronger congestion / peak penalty and more conservative DSU compaction.
    static double totalNetDemand(const Design& design) {
        // Invariant per testcase; used by dynamic congestion pressure.
        static unordered_map<const Design*, double> cache;
        auto it = cache.find(&design);
        if (it != cache.end()) return it->second;

        double s = 0.0;
        for (const auto& c : design.connections) {
            s += max(0, c.netCount);
        }
        cache[&design] = s;
        return s;
    }

    static double congestionPressureFactor(const Design& design) {
        // Prediction + memory hierarchy: this factor is invariant during one run.
        static unordered_map<const Design*, double> cache;
        auto it = cache.find(&design);
        if (it != cache.end()) return it->second;

        const double nBlk = max(1.0, static_cast<double>(design.blockSpecs.size()));
        const double netsPerBlock = totalNetDemand(design) / nBlk;

        // case00 is around 1000 nets/block, so this is the neutral point.
        // Lower this value if you want congestion to become more sensitive.
        // Raise this value if outline shrink becomes too conservative.
        static constexpr double BASE_NETS_PER_BLOCK = 1000.0;

        double f = pow(max(0.05, netsPerBlock / BASE_NETS_PER_BLOCK), 0.75);
        f = clampDouble(f, 0.60, 3.00);
        cache[&design] = f;
        return f;
    }

    static Rect makeShapeFromSpec(const BlockSpec& spec, double ratioOverride = -1.0) {
        Rect r;
        r.x = r.y = 0.0;

        if (spec.hasFixedSize) {
            r.w = spec.fixedW;
            r.h = spec.fixedH;
            return r;
        }

        double area = max(1.0, spec.area);
        double ratio = ratioOverride > 0.0 ? ratioOverride : aspectMid(spec);
        ratio = clampDouble(ratio, max(0.05, spec.aspectMin), max(max(0.05, spec.aspectMin), spec.aspectMax));

        r.w = sqrt(area * ratio);
        r.h = area / max(TINY, r.w);

        if (r.w <= EPS || r.h <= EPS) {
            r.w = sqrt(area);
            r.h = sqrt(area);
        }
        return r;
    }

    static pair<double, double> minOutlineWH(const Design& design) {
        // Invariant lower bound.  This was called inside every normalize step.
        static unordered_map<const Design*, pair<double, double>> cache;
        auto it = cache.find(&design);
        if (it != cache.end()) return it->second;

        double maxW = 1.0, maxH = 1.0;
        for (const auto& spec : design.blockSpecs) {
            Rect r = makeShapeFromSpec(spec, aspectMid(spec));
            maxW = max(maxW, r.w);
            maxH = max(maxH, r.h);
        }
        pair<double, double> ans{ maxW * 1.02, maxH * 1.02 };
        cache[&design] = ans;
        return ans;
    }

    static void normalizeOutlineInState(const Design& design, FPState& st) {
        auto [minW0, minH0] = minOutlineWH(design);
        // Do not impose a strong lower bound by total area here; overlap penalties
        // decide legality.  These bounds only avoid numerically useless outlines.
        st.outlineW = clampDouble(st.outlineW > EPS ? st.outlineW : design.maxOutlineW, minW0, design.maxOutlineW);
        st.outlineH = clampDouble(st.outlineH > EPS ? st.outlineH : design.maxOutlineH, minH0, design.maxOutlineH);
    }

    static bool inside(const Rect& r, double W, double H) {
        return r.x >= -EPS && r.y >= -EPS && rectRight(r) <= W + EPS && rectTop(r) <= H + EPS;
    }

    static double rectOverlapArea(const Rect& a, const Rect& b) {
        return overlapLen(a.x, rectRight(a), b.x, rectRight(b)) *
            overlapLen(a.y, rectTop(a), b.y, rectTop(b));
    }

    static bool pointInsideRect(double x, double y, const Rect& r) {
        return x >= r.x - EPS && x <= rectRight(r) + EPS &&
            y >= r.y - EPS && y <= rectTop(r) + EPS;
    }

    struct EdgeLocRule {
        char side = 'B';   // T/B/L/R: first letter decides the boundary side.
        int zone = 0;      // top/bottom: L=0,M=1,R=2; left/right: B=0,M=1,T=2.
        bool valid = false;
        string raw;
    };

    static vector<string> splitLocationTokens(const string& s) {
        vector<string> out;
        string cur;
        for (char ch : s) {
            if (ch == ',' || ch == ';' || ch == '/' || ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n') {
                if (!cur.empty()) {
                    out.push_back(cur);
                    cur.clear();
                }
            }
            else {
                cur.push_back(ch);
            }
        }
        if (!cur.empty()) out.push_back(cur);
        return out;
    }

    static EdgeLocRule parseEdgeLocRule(const string& locRaw) {
        EdgeLocRule r;
        string L = upperStr(locRaw);
        r.raw = L;
        if (L.size() >= 2) {
            const char a = L[0];
            const char b = L[1];

            // Problem E defines 12 edge regions:
            //   top side    : TL / TM / TR
            //   bottom side : BL / BM / BR
            //   left side   : LT / LM / LB
            //   right side  : RT / RM / RB
            //
            // Important: TL and LT are NOT the same.
            // Old code used locContains('T') and locContains('L'), so TL and LT
            // were both forced into the top-left corner.  The same happened for
            // BR/RB and caused edge-edge overlap.
            if (a == 'T' && (b == 'L' || b == 'M' || b == 'R')) {
                r.side = 'T';
                r.zone = (b == 'L') ? 0 : (b == 'M' ? 1 : 2);
                r.valid = true;
                return r;
            }
            if (a == 'B' && (b == 'L' || b == 'M' || b == 'R')) {
                r.side = 'B';
                r.zone = (b == 'L') ? 0 : (b == 'M' ? 1 : 2);
                r.valid = true;
                return r;
            }
            if (a == 'L' && (b == 'B' || b == 'M' || b == 'T')) {
                r.side = 'L';
                r.zone = (b == 'B') ? 0 : (b == 'M' ? 1 : 2);
                r.valid = true;
                return r;
            }
            if (a == 'R' && (b == 'B' || b == 'M' || b == 'T')) {
                r.side = 'R';
                r.zone = (b == 'B') ? 0 : (b == 'M' ? 1 : 2);
                r.valid = true;
                return r;
            }
        }

        // Defensive fallback for imperfect input.
        if (locContains(L, 'T')) {
            r.side = 'T';
            r.zone = locContains(L, 'R') ? 2 : (locContains(L, 'M') ? 1 : 0);
            r.valid = true;
        }
        else if (locContains(L, 'B')) {
            r.side = 'B';
            r.zone = locContains(L, 'R') ? 2 : (locContains(L, 'M') ? 1 : 0);
            r.valid = true;
        }
        else if (locContains(L, 'R')) {
            r.side = 'R';
            r.zone = locContains(L, 'T') ? 2 : (locContains(L, 'M') ? 1 : 0);
            r.valid = true;
        }
        else if (locContains(L, 'L')) {
            r.side = 'L';
            r.zone = locContains(L, 'T') ? 2 : (locContains(L, 'M') ? 1 : 0);
            r.valid = true;
        }
        return r;
    }

    static vector<EdgeLocRule> collectEdgeLocRules(const BlockSpec& spec) {
        // Hierarchy of memories: parsing EDGE locations is invariant for a BlockSpec.
        // This function is called inside every edge projection / edge penalty eval,
        // so cache the parsed 12-region rule list by BlockSpec address.
        static unordered_map<const BlockSpec*, vector<EdgeLocRule>> cache;
        auto hit = cache.find(&spec);
        if (hit != cache.end()) return hit->second;

        vector<EdgeLocRule> rules;
        if (spec.locations.empty()) {
            EdgeLocRule r = parseEdgeLocRule("BL");
            if (r.valid) rules.push_back(r);
            cache[&spec] = rules;
            return rules;
        }

        for (const string& s : spec.locations) {
            vector<string> toks = splitLocationTokens(s);
            if (toks.empty()) toks.push_back(s);
            for (const string& t : toks) {
                EdgeLocRule r = parseEdgeLocRule(t);
                if (r.valid) rules.push_back(r);
            }
        }

        if (rules.empty()) {
            EdgeLocRule r = parseEdgeLocRule("BL");
            if (r.valid) rules.push_back(r);
        }
        cache[&spec] = rules;
        return rules;
    }

    static double edgeSideCapacity(char side, double W, double H) {
        return (side == 'T' || side == 'B') ? W : H;
    }

    static double edgeBlockLengthOnSide(const Rect& r, char side) {
        return (side == 'T' || side == 'B') ? r.w : r.h;
    }

    static double edgeZoneCenter(const EdgeLocRule& rule, double W, double H) {
        if (rule.side == 'T' || rule.side == 'B') {
            return (static_cast<double>(rule.zone) + 0.5) * W / 3.0;
        }
        return (static_cast<double>(rule.zone) + 0.5) * H / 3.0;
    }

    static pair<double, double> edgeZoneBounds(const EdgeLocRule& rule, double W, double H) {
        double span = edgeSideCapacity(rule.side, W, H);
        double lo = static_cast<double>(rule.zone) * span / 3.0;
        double hi = static_cast<double>(rule.zone + 1) * span / 3.0;
        return { lo, hi };
    }

    static Rect placeByLocationRule(const Rect& shape, const string& locRaw, double W, double H) {
        Rect r = shape;
        EdgeLocRule rule = parseEdgeLocRule(locRaw);
        if (!rule.valid) rule = parseEdgeLocRule("BL");

        auto [zoneLo, zoneHi] = edgeZoneBounds(rule, W, H);

        if (rule.side == 'T' || rule.side == 'B') {
            double pref = edgeZoneCenter(rule, W, H) - 0.5 * r.w;
            // The whole EDGE block must stay inside the selected one-third zone,
            // not only its center.
            r.x = clampDouble(pref, zoneLo, zoneHi - r.w);
            r.y = (rule.side == 'T') ? (H - r.h) : 0.0;
        }
        else {
            double pref = edgeZoneCenter(rule, W, H) - 0.5 * r.h;
            r.x = (rule.side == 'R') ? (W - r.w) : 0.0;
            r.y = clampDouble(pref, zoneLo, zoneHi - r.h);
        }

        r.x = clampDouble(r.x, 0.0, W - r.w);
        r.y = clampDouble(r.y, 0.0, H - r.h);
        return r;
    }

    static double edgeConstraintViolationForRule(const Rect& r, const EdgeLocRule& rule, double W, double H) {
        double v = 0.0;
        auto [zoneLo, zoneHi] = edgeZoneBounds(rule, W, H);

        // Strict 12-region constraint: the whole edge block rectangle must be
        // inside the selected one-third region.  Old code checked only the center,
        // which allowed a TL block to cross into TM, etc.
        if (rule.side == 'T') {
            v += fabs(rectTop(r) - H);
            v += max(0.0, zoneLo - r.x);
            v += max(0.0, rectRight(r) - zoneHi);
        }
        else if (rule.side == 'B') {
            v += fabs(r.y);
            v += max(0.0, zoneLo - r.x);
            v += max(0.0, rectRight(r) - zoneHi);
        }
        else if (rule.side == 'L') {
            v += fabs(r.x);
            v += max(0.0, zoneLo - r.y);
            v += max(0.0, rectTop(r) - zoneHi);
        }
        else if (rule.side == 'R') {
            v += fabs(rectRight(r) - W);
            v += max(0.0, zoneLo - r.y);
            v += max(0.0, rectTop(r) - zoneHi);
        }
        return v;
    }

    static double edgeConstraintViolation(const Rect& r, const BlockSpec& spec, double W, double H) {
        if (spec.type != BlockType::EDGE) return 0.0;

        vector<EdgeLocRule> rules = collectEdgeLocRules(spec);
        double best = INF_COST;
        for (const EdgeLocRule& rule : rules) {
            best = min(best, edgeConstraintViolationForRule(r, rule, W, H));
        }
        return best >= INF_COST * 0.5 ? 0.0 : best;
    }

    struct EdgePackItem {
        int id = -1;
        EdgeLocRule rule;
        double pref = 0.0;
        double pos = 0.0;
        double len = 0.0;
        double lo = 0.0;
        double hi = 0.0;
    };

    static int edgeSideIndex(char side) {
        if (side == 'T') return 0;
        if (side == 'B') return 1;
        if (side == 'L') return 2;
        return 3; // R
    }

    static const vector<int>& cachedMovableIds(const Design& design) {
        static unordered_map<const Design*, vector<int>> cache;
        auto it = cache.find(&design);
        if (it != cache.end()) return it->second;

        vector<int> ids;
        ids.reserve(design.blockSpecs.size());
        for (int i = 0; i < static_cast<int>(design.blockSpecs.size()); ++i) {
            if (isMovableForSA(design.blockSpecs[i])) ids.push_back(i);
        }
        auto res = cache.emplace(&design, std::move(ids));
        return res.first->second;
    }

    static const vector<int>& cachedSoftIds(const Design& design) {
        static unordered_map<const Design*, vector<int>> cache;
        auto it = cache.find(&design);
        if (it != cache.end()) return it->second;

        vector<int> ids;
        ids.reserve(design.blockSpecs.size());
        for (int i = 0; i < static_cast<int>(design.blockSpecs.size()); ++i) {
            if (design.blockSpecs[i].type == BlockType::SOFT) ids.push_back(i);
        }
        auto res = cache.emplace(&design, std::move(ids));
        return res.first->second;
    }

    static const vector<int>& cachedEdgeOrder(const Design& design) {
        // EDGE block order is invariant because EDGE blocks have fixed geometry.
        // Caching avoids sorting the same edge id list on every edge projection.
        static unordered_map<const Design*, vector<int>> cache;
        auto it = cache.find(&design);
        if (it != cache.end()) return it->second;

        vector<int> ids;
        ids.reserve(design.blockSpecs.size());
        for (int i = 0; i < static_cast<int>(design.blockSpecs.size()); ++i) {
            if (design.blockSpecs[i].type == BlockType::EDGE) ids.push_back(i);
        }
        sort(ids.begin(), ids.end(), [&](int a, int b) {
            const BlockSpec& A = design.blockSpecs[a];
            const BlockSpec& B = design.blockSpecs[b];
            double aa = A.hasFixedSize ? A.fixedW * A.fixedH : max(1.0, A.area);
            double bb = B.hasFixedSize ? B.fixedW * B.fixedH : max(1.0, B.area);
            if (fabs(aa - bb) > 1e-6) return aa > bb;
            return a < b;
            });
        auto res = cache.emplace(&design, std::move(ids));
        return res.first->second;
    }

    static int chooseEdgeBlock(const Design& design, mt19937& rng) {
        const vector<int>& ids = cachedEdgeOrder(design);
        if (ids.empty()) return -1;
        return ids[randInt(rng, 0, static_cast<int>(ids.size()) - 1)];
    }

    static void ensureEdge1DState(const Design& design, FPState& st, mt19937* rng = nullptr, int restartId = 0) {
        const int n = static_cast<int>(design.blockSpecs.size());
        if (static_cast<int>(st.edgePos.size()) != n) st.edgePos.assign(n, 0.5);
        if (static_cast<int>(st.edgeRule.size()) != n) st.edgeRule.assign(n, 0);

        const double W = max(1.0, st.outlineW > EPS ? st.outlineW : design.maxOutlineW);
        const double H = max(1.0, st.outlineH > EPS ? st.outlineH : design.maxOutlineH);

        for (int id : cachedEdgeOrder(design)) {
            const vector<EdgeLocRule> rules = collectEdgeLocRules(design.blockSpecs[id]);
            int rcount = max(1, static_cast<int>(rules.size()));
            st.edgeRule[id] = ((st.edgeRule[id] % rcount) + rcount) % rcount;
            const EdgeLocRule& rule = rules.empty() ? parseEdgeLocRule("BL") : rules[st.edgeRule[id]];
            double span = max(1.0, edgeSideCapacity(rule.side, W, H));
            double neutral = clampDouble(edgeZoneCenter(rule, W, H) / span, 0.0, 1.0);

            // When a state is just created, spread restarts over slightly different
            // 1D edge locations.  After that, SA owns edgePos and this function only
            // clamps / repairs size.
            if (st.edgePos[id] < -0.5 || st.edgePos[id] > 1.5 || (fabs(st.edgePos[id] - 0.5) < 1e-12 && restartId != 0)) {
                double jitter = rng ? randRange(*rng, -0.08, 0.08) : 0.0;
                st.edgePos[id] = clampDouble(neutral + jitter + 0.015 * (restartId % 5 - 2), 0.0, 1.0);
            }
            else {
                st.edgePos[id] = clampDouble(st.edgePos[id], 0.0, 1.0);
            }
        }
    }

    static EdgeLocRule selectedEdgeRule(const Design& design, const FPState& st, int id) {
        const vector<EdgeLocRule> rules = collectEdgeLocRules(design.blockSpecs[id]);
        if (rules.empty()) return parseEdgeLocRule("BL");
        int k = 0;
        if (id >= 0 && id < static_cast<int>(st.edgeRule.size())) {
            k = st.edgeRule[id];
        }
        k = ((k % static_cast<int>(rules.size())) + static_cast<int>(rules.size())) % static_cast<int>(rules.size());
        return rules[k];
    }

    static void mutateEdge1DState(const Design& design, FPState& st, mt19937& rng, double temp01) {
        ensureEdge1DState(design, st, nullptr, 0);
        int id = chooseEdgeBlock(design, rng);
        if (id < 0) return;

        int mt = randInt(rng, 0, 99);
        if (mt < 55) {
            // B*-tree-like move: delete/insert in 1D is approximated by relocating
            // the preferred coordinate.  The side packer will legalize the chain.
            double step = 0.015 + 0.22 * temp01;
            st.edgePos[id] = clampDouble(st.edgePos[id] + randRange(rng, -step, step), 0.0, 1.0);
        }
        else if (mt < 80) {
            // B*-tree-like swap: exchange the 1D preferred positions of two edge blocks.
            int id2 = chooseEdgeBlock(design, rng);
            if (id2 >= 0 && id2 != id) {
                swap(st.edgePos[id], st.edgePos[id2]);
            }
        }
        else {
            // Keep the boundary side selected by the input location order.  This is
            // deliberately conservative for Problem E: changing TL<->LT or RB<->BR
            // can make a floorplan look shorter in HPWL but hurt the current greedy
            // router badly.  We still keep a real 1D floorplan on that boundary by
            // relocating the preferred coordinate.
            st.edgePos[id] = clampDouble(randRange(rng, 0.0, 1.0), 0.0, 1.0);
        }
    }

    static void packOneEdgeSide(vector<EdgePackItem>& items, FPState& st, double W, double H) {
        if (items.empty()) return;

        const char side = items.front().rule.side;
        const double span = edgeSideCapacity(side, W, H);
        const double gap = 1.0e-3;

        sort(items.begin(), items.end(), [](const EdgePackItem& a, const EdgePackItem& b) {
            if (a.rule.zone != b.rule.zone) return a.rule.zone < b.rule.zone;
            if (fabs(a.pref - b.pref) > 1e-6) return a.pref < b.pref;
            return a.id < b.id;
            });

        // Pack along the chosen boundary, but never intentionally place a block
        // outside its selected one-third zone.  If blocks cannot physically fit in
        // that zone, overlap remains and the normal overlap/edge penalties expose
        // the infeasibility instead of silently sliding the block into another zone.
        double cur = 0.0;
        for (auto& it : items) {
            double lo = clampDouble(it.lo, 0.0, span);
            double hi = clampDouble(it.hi, lo, span);
            double maxPos = max(lo, hi - it.len);
            cur = max(cur, lo);
            it.pos = max(clampDouble(it.pref, lo, maxPos), cur);
            if (it.pos > maxPos) it.pos = maxPos;
            cur = it.pos + it.len + gap;
        }

        for (const auto& it : items) {
            Rect& r = st.rects[it.id];
            if (side == 'T') {
                r.x = it.pos;
                r.y = H - r.h;
            }
            else if (side == 'B') {
                r.x = it.pos;
                r.y = 0.0;
            }
            else if (side == 'L') {
                r.x = 0.0;
                r.y = it.pos;
            }
            else { // R
                r.x = W - r.w;
                r.y = it.pos;
            }
            r.x = clampDouble(r.x, 0.0, W - r.w);
            r.y = clampDouble(r.y, 0.0, H - r.h);
        }
    }


    static char placedEdgeSide(const Rect& r, double W, double H) {
        const double tol = 1.0e-4;
        if (fabs(r.y) <= tol) return 'B';
        if (fabs(rectTop(r) - H) <= tol) return 'T';
        if (fabs(r.x) <= tol) return 'L';
        if (fabs(rectRight(r) - W) <= tol) return 'R';
        return '?';
    }

    static void repairPerpendicularEdgeOverlaps(const Design& design, FPState& st, double W, double H) {
        // Young/Wong/Yang's boundary-constraint idea is: keep constrained modules
        // on their required boundary and fix violations by moving the closest legal
        // module/position.  In our coordinate floorplan, the only remaining hard
        // case after same-side 1D packing is a perpendicular corner conflict
        // (e.g. a TL top-side block and an LT left-side block).  This repair moves
        // only along the already chosen boundary, never into the chip interior.
        const vector<int>& edgeIds = cachedEdgeOrder(design);
        const double gap = 1.0e-3;

        for (int iter = 0; iter < 32; ++iter) {
            bool changed = false;
            for (int ai = 0; ai < static_cast<int>(edgeIds.size()); ++ai) {
                for (int bi = ai + 1; bi < static_cast<int>(edgeIds.size()); ++bi) {
                    int a = edgeIds[ai];
                    int b = edgeIds[bi];
                    Rect& ra = st.rects[a];
                    Rect& rb = st.rects[b];
                    if (rectOverlapArea(ra, rb) <= EPS) continue;

                    char sa = placedEdgeSide(ra, W, H);
                    char sb = placedEdgeSide(rb, W, H);
                    bool aHorizontal = (sa == 'T' || sa == 'B');
                    bool bHorizontal = (sb == 'T' || sb == 'B');
                    if (sa == '?' || sb == '?' || aHorizontal == bHorizontal) continue;

                    int hId = aHorizontal ? a : b;
                    int vId = aHorizontal ? b : a;
                    Rect& rh = st.rects[hId];
                    Rect& rv = st.rects[vId];
                    char hs = placedEdgeSide(rh, W, H);
                    char vs = placedEdgeSide(rv, W, H);

                    struct CandidateMove {
                        bool moveHorizontal = true;
                        double newPos = 0.0;
                        double cost = INF_COST;
                    };
                    vector<CandidateMove> candidates;

                    // Candidate 1: slide the top/bottom edge block horizontally
                    // away from the left/right edge block.
                    double hx = rh.x;
                    if (vs == 'L') hx = rectRight(rv) + gap;
                    else if (vs == 'R') hx = rv.x - rh.w - gap;
                    hx = clampDouble(hx, 0.0, W - rh.w);
                    Rect testH = rh;
                    testH.x = hx;
                    if (rectOverlapArea(testH, rv) <= EPS) {
                        double cost = fabs(hx - rh.x) + edgeConstraintViolation(testH, design.blockSpecs[hId], W, H);
                        candidates.push_back({ true, hx, cost });
                    }

                    // Candidate 2: slide the left/right edge block vertically away
                    // from the top/bottom edge block.
                    double vy = rv.y;
                    if (hs == 'B') vy = rectTop(rh) + gap;
                    else if (hs == 'T') vy = rh.y - rv.h - gap;
                    vy = clampDouble(vy, 0.0, H - rv.h);
                    Rect testV = rv;
                    testV.y = vy;
                    if (rectOverlapArea(rh, testV) <= EPS) {
                        double cost = fabs(vy - rv.y) + edgeConstraintViolation(testV, design.blockSpecs[vId], W, H);
                        candidates.push_back({ false, vy, cost });
                    }

                    if (candidates.empty()) continue;
                    sort(candidates.begin(), candidates.end(), [](const CandidateMove& x, const CandidateMove& y) {
                        return x.cost < y.cost;
                        });

                    if (candidates.front().moveHorizontal) rh.x = candidates.front().newPos;
                    else rv.y = candidates.front().newPos;
                    changed = true;
                }
            }
            if (!changed) break;
        }
    }



    static double totalOverlapInvolvingEdge(const Design& design, const FPState& st) {
        // Final safety score for EDGE-vs-EDGE overlap.  Non-edge overlaps are
        // already handled by spreadRepair() and the normal overlap penalty; keeping
        // this repair edge-only makes the common legal-candidate case fast.
        double s = 0.0;
        int n = static_cast<int>(st.rects.size());
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                if (design.blockSpecs[i].type != BlockType::EDGE ||
                    design.blockSpecs[j].type != BlockType::EDGE) {
                    continue;
                }
                s += rectOverlapArea(st.rects[i], st.rects[j]);
            }
        }
        return s;
    }

    static double edgeAxisCoord(const Rect& r, char side) {
        return (side == 'T' || side == 'B') ? r.x : r.y;
    }

    static void setEdgeAxisCoord(Rect& r, char side, double coord, double W, double H) {
        if (side == 'T') {
            r.x = coord;
            r.y = H - r.h;
        }
        else if (side == 'B') {
            r.x = coord;
            r.y = 0.0;
        }
        else if (side == 'L') {
            r.x = 0.0;
            r.y = coord;
        }
        else { // R
            r.x = W - r.w;
            r.y = coord;
        }
        r.x = clampDouble(r.x, 0.0, W - r.w);
        r.y = clampDouble(r.y, 0.0, H - r.h);
    }

    static double edgeRepairScore(const Design& design, const FPState& st, double W, double H) {
        double score = 0.0;
        int n = static_cast<int>(st.rects.size());
        for (int i = 0; i < n; ++i) {
            const Rect& r = st.rects[i];
            if (!inside(r, W, H)) {
                double viol = 0.0;
                viol += max(0.0, -r.x);
                viol += max(0.0, -r.y);
                viol += max(0.0, rectRight(r) - W);
                viol += max(0.0, rectTop(r) - H);
                score += 1.0e9 * (viol + sqr(viol));
            }
        }
        score += 1.0e9 * totalOverlapInvolvingEdge(design, st);
        for (int id : cachedEdgeOrder(design)) {
            score += 1.0e3 * edgeConstraintViolation(st.rects[id], design.blockSpecs[id], W, H);
        }
        return score;
    }

    static void strictRepairEdgeOverlaps(const Design& design, FPState& st, double W, double H) {
        // Final redundant safety pass for EDGE blocks.
        //
        // packOneEdgeSide() removes same-side overlaps, and
        // repairPerpendicularEdgeOverlaps() handles the common two-block corner
        // case.  However, moving one edge block away from a corner can create a
        // new conflict with a third edge block or with a non-edge block.  This pass
        // repeatedly tries legal 1D positions along the selected boundary and only
        // accepts a move that lowers the global EDGE-vs-EDGE overlap.
        const vector<int>& edgeIds = cachedEdgeOrder(design);
        if (edgeIds.empty()) return;

        // Make common case fast: most SA candidates already have no EDGE-vs-EDGE
        // overlap after the side 1D legalizer and perpendicular repair.  Do the
        // cheap overlap scan first and avoid edgeRepairScore() unless needed.
        if (totalOverlapInvolvingEdge(design, st) <= EPS) return;

        const double gap = 1.0e-3;
        double baseScore = edgeRepairScore(design, st, W, H);

        for (int iter = 0; iter < 96; ++iter) {
            bool improved = false;
            FPState bestState = st;
            double bestScore = baseScore;

            for (int id : edgeIds) {
                EdgeLocRule rule = selectedEdgeRule(design, st, id);
                char side = rule.side;
                double span = edgeSideCapacity(side, W, H);
                double len = edgeBlockLengthOnSide(st.rects[id], side);
                double maxPos = max(0.0, span - len);
                double curPos = edgeAxisCoord(st.rects[id], side);

                vector<double> candPos;
                candPos.reserve(2 * st.rects.size() + 8);
                candPos.push_back(curPos);
                candPos.push_back(0.0);
                candPos.push_back(maxPos);
                candPos.push_back(clampDouble(edgeZoneCenter(rule, W, H) - 0.5 * len, 0.0, maxPos));

                for (int j : edgeIds) {
                    if (j == id) continue;
                    const Rect& o = st.rects[j];
                    if (side == 'T' || side == 'B') {
                        candPos.push_back(o.x - st.rects[id].w - gap);
                        candPos.push_back(rectRight(o) + gap);
                    }
                    else {
                        candPos.push_back(o.y - st.rects[id].h - gap);
                        candPos.push_back(rectTop(o) + gap);
                    }
                }

                sort(candPos.begin(), candPos.end());
                candPos.erase(unique(candPos.begin(), candPos.end(), [](double a, double b) {
                    return fabs(a - b) < 1.0e-4;
                    }), candPos.end());

                for (double p : candPos) {
                    p = clampDouble(p, 0.0, maxPos);
                    FPState trial = st;
                    setEdgeAxisCoord(trial.rects[id], side, p, W, H);

                    // Keep edgePos synchronized with the repaired coordinate so the
                    // next applyEdgeAndClamp() does not undo this repair.
                    if (id >= 0 && id < static_cast<int>(trial.edgePos.size())) {
                        double center = p + 0.5 * len;
                        trial.edgePos[id] = clampDouble(center / max(1.0, span), 0.0, 1.0);
                    }

                    double sc = edgeRepairScore(design, trial, W, H);
                    // Tiny movement term keeps the repair stable when two positions
                    // have exactly the same overlap score.
                    sc += 1.0e-6 * fabs(p - curPos);
                    if (sc + 1.0e-6 < bestScore) {
                        bestScore = sc;
                        bestState = std::move(trial);
                        improved = true;
                    }
                }
            }

            if (!improved) break;
            st = std::move(bestState);
            baseScore = bestScore;
            if (totalOverlapInvolvingEdge(design, st) <= EPS) break;
        }
    }

    static void packEdgeBlocksNoOverlap(const Design& design, FPState& st, double W, double H) {
        ensureEdge1DState(design, st, nullptr, 0);

        vector<EdgePackItem> sideItems[4];
        const vector<int>& edgeIds = cachedEdgeOrder(design);

        for (int id : edgeIds) {
            EdgeLocRule rule = selectedEdgeRule(design, st, id);
            int si = edgeSideIndex(rule.side);
            double len = edgeBlockLengthOnSide(st.rects[id], rule.side);
            double span = edgeSideCapacity(rule.side, W, H);

            EdgePackItem item;
            item.id = id;
            item.rule = rule;
            item.len = len;
            auto [zoneLo, zoneHi] = edgeZoneBounds(rule, W, H);
            item.lo = zoneLo;
            item.hi = zoneHi;

            // The edge block is part of a true 1D floorplan.  edgePos[id] is its
            // preferred normalized center on the chosen boundary.  The preferred
            // coordinate is clipped to the selected one-third zone, so the full
            // rectangle stays inside TL/TM/TR/... instead of merely having its
            // center inside the region.
            double preferredCenter = clampDouble(st.edgePos[id], 0.0, 1.0) * max(1.0, span);
            item.pref = clampDouble(preferredCenter - 0.5 * len, zoneLo, zoneHi - len);
            sideItems[si].push_back(item);
        }

        for (int si = 0; si < 4; ++si) {
            packOneEdgeSide(sideItems[si], st, W, H);
        }

        // Same-side edge overlap is solved by the 1D side packer above.
        // This extra pass handles perpendicular corner overlap while preserving
        // the boundary side.
        repairPerpendicularEdgeOverlaps(design, st, W, H);

        // Redundant safety pass: after a perpendicular repair, a moved edge block
        // may conflict with a third edge block or a normal block.  This pass only
        // accepts moves that reduce global overlap involving EDGE blocks.
        strictRepairEdgeOverlaps(design, st, W, H);
    }

    static int totalConnBetween(const Design& design, int a, int b) {
        int n = 0;
        if (a >= 0 && b >= 0 && a < static_cast<int>(design.connMatrix.size()) &&
            b < static_cast<int>(design.connMatrix[a].size())) {
            n += design.connMatrix[a][b];
        }
        if (a >= 0 && b >= 0 && b < static_cast<int>(design.connMatrix.size()) &&
            a < static_cast<int>(design.connMatrix[b].size())) {
            n += design.connMatrix[b][a];
        }
        return n;
    }

    static double hpwlCost(const Design& design, const vector<Rect>& rects) {
        double hpwl = 0.0;
        for (const auto& c : design.connections) {
            if (c.src < 0 || c.dst < 0 || c.src >= static_cast<int>(rects.size()) || c.dst >= static_cast<int>(rects.size())) continue;
            hpwl += static_cast<double>(c.netCount) *
                manhattan(rectCx(rects[c.src]), rectCy(rects[c.src]), rectCx(rects[c.dst]), rectCy(rects[c.dst]));
        }
        return hpwl;
    }

    static double ftRateForNets(const BlockSpec& spec, double ftNets) {
        if (ftNets <= 3000.0) return spec.ftRate[0];
        if (ftNets <= 6000.0) return spec.ftRate[1];
        if (ftNets <= 9000.0) return spec.ftRate[2];
        return spec.ftRate[3];
    }

    static double estimatedFTAreaPenalty(const Design& design, const vector<Rect>& rects, const vector<double>& ftDemand) {
        double penalty = 0.0;
        for (int i = 0; i < static_cast<int>(rects.size()); ++i) {
            if (design.blockSpecs[i].type != BlockType::SOFT) continue;
            if (ftDemand[i] <= EPS) continue;

            double rate = ftRateForNets(design.blockSpecs[i], ftDemand[i]);
            double delta = (ftDemand[i] / CHANNEL_DENSITY) * rate / 2.0;
            double requiredArea = (rects[i].w + delta) * (rects[i].h + delta);
            double currentArea = rects[i].w * rects[i].h;
            penalty += max(0.0, requiredArea - currentArea);
        }
        return penalty;
    }

    static int tileIndex(int x, int y, int gx) { return y * gx + x; }

    static pair<int, int> pointToTile(double x, double y, double W, double H, int gx, int gy) {
        int ix = static_cast<int>(floor(clampDouble(x / max(TINY, W), 0.0, 0.999999) * gx));
        int iy = static_cast<int>(floor(clampDouble(y / max(TINY, H), 0.0, 0.999999) * gy));
        ix = clampDouble(ix, 0, gx - 1);
        iy = clampDouble(iy, 0, gy - 1);
        return { ix, iy };
    }

    static bool tileInBBox(int ix, int iy, int minX, int maxX, int minY, int maxY) {
        return ix >= minX && ix <= maxX && iy >= minY && iy <= maxY;
    }

    static void buildBlockageMap(
        const Design& design,
        const vector<Rect>& rects,
        vector<TileInfo>& tiles,
        int gx,
        int gy,
        double W,
        double H
    ) {
        const double tw = W / gx;
        const double th = H / gy;
        const double baseCap = CHANNEL_DENSITY * max(1.0, min(tw, th));

        // Hierarchy of memories + common case fast:
        // Old version visited every tile and then scanned every block to check
        // whether the tile center was inside the block: O(gx*gy*n).  This version
        // initializes tiles once, then visits only the tile range covered by each
        // block.  It preserves the same center-inside rule but avoids most checks.
        for (TileInfo& t : tiles) {
            t.prelim = 0.0;
            t.demand = 0.0;
            t.capacity = baseCap;
            t.blockage = 0.0;
            t.softBlock = -1;
        }

        for (int i = 0; i < static_cast<int>(rects.size()); ++i) {
            const Rect& r = rects[i];
            if (rectRight(r) < 0.0 || rectTop(r) < 0.0 || r.x > W || r.y > H) continue;

            int x0 = clampDouble(static_cast<int>(floor(r.x / max(TINY, tw))), 0, gx - 1);
            int x1 = clampDouble(static_cast<int>(floor(rectRight(r) / max(TINY, tw))), 0, gx - 1);
            int y0 = clampDouble(static_cast<int>(floor(r.y / max(TINY, th))), 0, gy - 1);
            int y1 = clampDouble(static_cast<int>(floor(rectTop(r) / max(TINY, th))), 0, gy - 1);

            for (int y = y0; y <= y1; ++y) {
                const double cy = (y + 0.5) * th;
                if (cy < r.y - EPS || cy > rectTop(r) + EPS) continue;
                for (int x = x0; x <= x1; ++x) {
                    const double cx = (x + 0.5) * tw;
                    if (cx < r.x - EPS || cx > rectRight(r) + EPS) continue;

                    TileInfo& t = tiles[tileIndex(x, y, gx)];
                    if (design.blockSpecs[i].type == BlockType::SOFT) {
                        t.blockage = max(t.blockage, 0.90);
                        t.softBlock = i;
                    }
                    else {
                        t.blockage = 1.0;
                        t.softBlock = -1;
                    }
                }
            }
        }

        for (TileInfo& t : tiles) {
            t.capacity *= max(0.03, 1.0 - t.blockage);
        }
    }

    static void preliminaryCongestion(
        const Design& design,
        const vector<Rect>& rects,
        vector<TileInfo>& tiles,
        int gx,
        int gy,
        double W,
        double H
    ) {
        // prelim was reset in buildBlockageMap().
        for (const auto& c : design.connections) {
            if (c.src < 0 || c.dst < 0 || c.src >= static_cast<int>(rects.size()) || c.dst >= static_cast<int>(rects.size())) continue;
            auto [sx, sy] = pointToTile(rectCx(rects[c.src]), rectCy(rects[c.src]), W, H, gx, gy);
            auto [tx, ty] = pointToTile(rectCx(rects[c.dst]), rectCy(rects[c.dst]), W, H, gx, gy);

            int minX = min(sx, tx), maxX = max(sx, tx);
            int minY = min(sy, ty), maxY = max(sy, ty);
            int dx = maxX - minX;
            int dy = maxY - minY;
            int cells = max(1, (dx + 1) * (dy + 1));

            // Equation-like rough bbox density: wirelength over bbox area.
            double contribution = c.netCount * static_cast<double>(dx + dy + 1) / static_cast<double>(cells);
            for (int y = minY; y <= maxY; ++y) {
                for (int x = minX; x <= maxX; ++x) {
                    tiles[tileIndex(x, y, gx)].prelim += contribution;
                }
            }
        }
    }

    static void detailedDiagonalCongestion(
        const Design& design,
        const vector<Rect>& rects,
        vector<TileInfo>& tiles,
        vector<double>& ftDemand,
        int gx,
        int gy,
        double W,
        double H
    ) {
        for (auto& t : tiles) t.demand = 0.0;
        fill(ftDemand.begin(), ftDemand.end(), 0.0);

        static thread_local vector<vector<int>> buckets;
        static thread_local vector<double> weights;
        if (static_cast<int>(buckets.size()) < gx + gy + 2) buckets.resize(gx + gy + 2);

        for (const auto& c : design.connections) {
            if (c.src < 0 || c.dst < 0 || c.src >= static_cast<int>(rects.size()) || c.dst >= static_cast<int>(rects.size())) continue;

            auto [sx0, sy0] = pointToTile(rectCx(rects[c.src]), rectCy(rects[c.src]), W, H, gx, gy);
            auto [tx0, ty0] = pointToTile(rectCx(rects[c.dst]), rectCy(rects[c.dst]), W, H, gx, gy);

            int minX = min(sx0, tx0), maxX = max(sx0, tx0);
            int minY = min(sy0, ty0), maxY = max(sy0, ty0);
            int dx = maxX - minX;
            int dy = maxY - minY;
            int dt = dx + dy;
            if (dt <= 0) continue;

            for (auto& b : buckets) b.clear();

            // Group tiles by Manhattan distance from the source, which is the
            // SMD/diagonal idea.  A route crosses one tile per division.
            for (int y = minY; y <= maxY; ++y) {
                for (int x = minX; x <= maxX; ++x) {
                    int d = abs(x - sx0) + abs(y - sy0);
                    if (d >= 0 && d <= dt) buckets[d].push_back(tileIndex(x, y, gx));
                }
            }

            for (int d = 0; d <= dt; ++d) {
                if (buckets[d].empty()) continue;

                double sumW = 0.0;
                weights.clear();
                weights.reserve(buckets[d].size());

                for (int idx : buckets[d]) {
                    const TileInfo& t = tiles[idx];
                    // Weighted detailed estimation: if prelim demand is above
                    // capacity, make that tile less attractive.  Blockages also
                    // reduce attractiveness.
                    double w = 1.0;
                    if (t.prelim > t.capacity + EPS) w = t.capacity / max(t.prelim, TINY);
                    w *= max(0.02, 1.0 - t.blockage);
                    weights.push_back(w);
                    sumW += w;
                }
                if (sumW <= TINY) sumW = static_cast<double>(buckets[d].size());

                for (int k = 0; k < static_cast<int>(buckets[d].size()); ++k) {
                    int idx = buckets[d][k];
                    double prob = (sumW <= TINY) ? (1.0 / buckets[d].size()) : (weights[k] / sumW);
                    double add = c.netCount * prob;
                    tiles[idx].demand += add;
                    if (tiles[idx].softBlock >= 0) ftDemand[tiles[idx].softBlock] += add * 0.5;
                }
            }
        }
    }

    static void redistributeCongestion(vector<TileInfo>& tiles, int gx, int gy) {
        // Simulate a cheap rip-up/reroute effect: overfull tiles push some demand
        // to the lowest-demand neighbor with spare resource.
        static thread_local vector<double> delta;
        for (int iter = 0; iter < 3; ++iter) {
            delta.assign(tiles.size(), 0.0);
            for (int y = 0; y < gy; ++y) {
                for (int x = 0; x < gx; ++x) {
                    int idx = tileIndex(x, y, gx);
                    double overflow = max(0.0, tiles[idx].demand - tiles[idx].capacity);
                    if (overflow <= EPS) continue;

                    int best = -1;
                    double bestSlack = 0.0;
                    const int dxs[4] = { 1, -1, 0, 0 };
                    const int dys[4] = { 0, 0, 1, -1 };
                    for (int k = 0; k < 4; ++k) {
                        int nx = x + dxs[k], ny = y + dys[k];
                        if (nx < 0 || nx >= gx || ny < 0 || ny >= gy) continue;
                        int nidx = tileIndex(nx, ny, gx);
                        double slack = tiles[nidx].capacity - tiles[nidx].demand;
                        if (slack > bestSlack) {
                            bestSlack = slack;
                            best = nidx;
                        }
                    }

                    if (best >= 0 && bestSlack > EPS) {
                        double mv = min({ overflow * 0.55, bestSlack * 0.90, tiles[idx].demand * 0.35 });
                        delta[idx] -= mv;
                        delta[best] += mv;
                    }
                }
            }
            for (int i = 0; i < static_cast<int>(tiles.size()); ++i) {
                tiles[i].demand = max(0.0, tiles[i].demand + delta[i]);
            }
        }
    }

    static pair<double, double> congestionCost(
        const Design& design,
        const vector<Rect>& rects,
        double W,
        double H,
        double& ftPenaltyOut
    ) {
        int n = static_cast<int>(rects.size());
        ftPenaltyOut = 0.0;
        if (n == 0 || design.connections.empty() || W <= EPS || H <= EPS) return { 0.0, 0.0 };

        int gx = clampDouble(static_cast<int>(sqrt(max(1, n)) * 6.0), GRID_X_MIN, GRID_X_MAX);
        int gy = clampDouble(static_cast<int>(sqrt(max(1, n)) * 5.0), GRID_Y_MIN, GRID_Y_MAX);

        // Memory hierarchy: keep these large evaluation buffers around and reuse
        // them across thousands of SA evaluations.  thread_local keeps this safe
        // if restarts are parallelized later.
        static thread_local vector<TileInfo> tiles;
        static thread_local vector<double> ftDemand;
        tiles.assign(gx * gy, TileInfo{});
        ftDemand.assign(rects.size(), 0.0);

        buildBlockageMap(design, rects, tiles, gx, gy, W, H);
        preliminaryCongestion(design, rects, tiles, gx, gy, W, H);
        detailedDiagonalCongestion(design, rects, tiles, ftDemand, gx, gy, W, H);
        redistributeCongestion(tiles, gx, gy);

        double overflowCost = 0.0;
        double peak = 0.0;
        for (const auto& t : tiles) {
            double ov = max(0.0, t.demand - t.capacity);
            if (ov <= EPS) continue;
            overflowCost += sqr(ov) / max(1.0, t.capacity);
            peak = max(peak, ov);
        }

        ftPenaltyOut = estimatedFTAreaPenalty(design, rects, ftDemand);
        return { overflowCost, peak };
    }


    static double softBridgePenalty(const Design& design, const vector<Rect>& rects) {
        double p = 0.0;
        for (const auto& c : design.connections) {
            if (c.src < 0 || c.dst < 0 || c.src >= static_cast<int>(rects.size()) || c.dst >= static_cast<int>(rects.size())) continue;
            double x1 = min(rectCx(rects[c.src]), rectCx(rects[c.dst]));
            double x2 = max(rectCx(rects[c.src]), rectCx(rects[c.dst]));
            double y1 = min(rectCy(rects[c.src]), rectCy(rects[c.dst]));
            double y2 = max(rectCy(rects[c.src]), rectCy(rects[c.dst]));
            // Give the corridor a little thickness so a nearby soft block bridge is discouraged.
            double margin = 60.0;
            Rect bbox{ max(0.0, x1 - margin), max(0.0, y1 - margin), (x2 - x1) + 2.0 * margin, (y2 - y1) + 2.0 * margin };
            for (int k : cachedSoftIds(design)) {
                if (k == c.src || k == c.dst) continue;
                double inter = rectOverlapArea(bbox, rects[k]);
                if (inter <= EPS) continue;
                double ratio = inter / max(1.0, rects[k].w * rects[k].h);
                p += static_cast<double>(c.netCount) * ratio;
            }
        }
        return p;
    }


    static double commonEdgeLikeScore(const Design& design, const vector<Rect>& rects) {
        // Feedthrough-aware idea: modules in the same high-net connection should
        // prefer adjacency.  Exact common edge is rare in continuous SA, so we give
        // credit for almost-adjacent rectangles with overlapping projection.  This
        // encourages source/destination blocks to share usable boundary length, and
        // tends to reduce unnecessary soft-block feedthrough bridges later.
        double score = 0.0;
        for (const auto& c : design.connections) {
            if (c.src < 0 || c.dst < 0 || c.src >= static_cast<int>(rects.size()) || c.dst >= static_cast<int>(rects.size())) continue;
            const Rect& a = rects[c.src];
            const Rect& b = rects[c.dst];
            double yOverlap = overlapLen(a.y, rectTop(a), b.y, rectTop(b));
            double xOverlap = overlapLen(a.x, rectRight(a), b.x, rectRight(b));

            double xGap = 0.0;
            if (rectRight(a) <= b.x) xGap = b.x - rectRight(a);
            else if (rectRight(b) <= a.x) xGap = a.x - rectRight(b);
            else xGap = 0.0;

            double yGap = 0.0;
            if (rectTop(a) <= b.y) yGap = b.y - rectTop(a);
            else if (rectTop(b) <= a.y) yGap = a.y - rectTop(b);
            else yGap = 0.0;

            if (yOverlap > EPS && xGap <= COMMON_EDGE_GAP) {
                score += static_cast<double>(c.netCount) * yOverlap * exp(-xGap / max(1.0, COMMON_EDGE_GAP));
            }
            if (xOverlap > EPS && yGap <= COMMON_EDGE_GAP) {
                score += static_cast<double>(c.netCount) * xOverlap * exp(-yGap / max(1.0, COMMON_EDGE_GAP));
            }
        }
        return score;
    }

    static double minGapPenalty(const vector<Rect>& rects, double targetGap) {
        // A mild preference for leaving channel-like gaps.  This is not a legality
        // term; overlap is handled separately.  It simply discourages nearly-abutted
        // block pairs when the congestion estimator says many connections exist.
        double p = 0.0;
        int n = static_cast<int>(rects.size());
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                double xGap = 0.0;
                if (rectRight(rects[i]) <= rects[j].x) xGap = rects[j].x - rectRight(rects[i]);
                else if (rectRight(rects[j]) <= rects[i].x) xGap = rects[i].x - rectRight(rects[j]);
                else xGap = 0.0;

                double yGap = 0.0;
                if (rectTop(rects[i]) <= rects[j].y) yGap = rects[j].y - rectTop(rects[i]);
                else if (rectTop(rects[j]) <= rects[i].y) yGap = rects[i].y - rectTop(rects[j]);
                else yGap = 0.0;

                bool xOverlap = overlapLen(rects[i].x, rectRight(rects[i]), rects[j].x, rectRight(rects[j])) > EPS;
                bool yOverlap = overlapLen(rects[i].y, rectTop(rects[i]), rects[j].y, rectTop(rects[j])) > EPS;
                if (xOverlap && yGap > EPS && yGap < targetGap) p += sqr(targetGap - yGap);
                if (yOverlap && xGap > EPS && xGap < targetGap) p += sqr(targetGap - xGap);
            }
        }
        return p;
    }


    static void applyEdgeAndClamp(const Design& design, FPState& st, double W, double H);
    static void spreadRepair(const Design& design, FPState& st, int maxIters);

    static double requiredChannelWidthForNets(int netCount) {
        // Problem statement: channel density is 25 nets / um.  Keep a tiny safety
        // margin so numerical noise does not create 59.999um for a 60um demand.
        if (netCount <= 0) return 0.0;
        return static_cast<double>(netCount) / CHANNEL_DENSITY + DIRECT_CHANNEL_SAFETY_MARGIN;
    }

    static double maxRequiredChannelWidth(const Design& design) {
        // Largest single routed bundle demand.  This is not used as a hard global
        // requirement for every channel; it is used to identify numerical slivers
        // that should not exist as routable rectangles at all.
        static unordered_map<const Design*, double> cache;
        auto it = cache.find(&design);
        if (it != cache.end()) return it->second;
        double req = 0.0;
        for (const auto& c : design.connections) {
            req = max(req, requiredChannelWidthForNets(c.netCount));
        }
        req = max(1.0, req);
        cache[&design] = req;
        return req;
    }

    static double sliverSnapThreshold(const Design& design) {
        return max(SLIVER_SNAP_ABS, maxRequiredChannelWidth(design) * SLIVER_SNAP_RATIO);
    }

    static void addMergedInterval(vector<pair<double, double>>& merged, double a, double b) {
        if (b <= a + EPS) return;
        if (merged.empty() || a > merged.back().second + EPS) {
            merged.push_back({ a, b });
        }
        else {
            merged.back().second = max(merged.back().second, b);
        }
    }

    static double actualChannelSliverPenalty(
        const Design& design,
        const vector<Rect>& rects,
        double W,
        double H
    ) {
        // Rebuild the same vertical-strip free rectangles as ChannelBuilder:
        // collect all block x / x+w edges, then for each strip find uncovered y
        // intervals.  Penalize tiny width/height channels because a later router
        // may otherwise route through them and create overflow.
        if (rects.empty() || W <= EPS || H <= EPS) return 0.0;

        const double reqMax = maxRequiredChannelWidth(design);
        const double snap = sliverSnapThreshold(design);
        double penalty = 0.0;

        vector<double> xs;
        xs.reserve(rects.size() * 2 + 2);
        xs.push_back(0.0);
        xs.push_back(W);
        for (const Rect& r : rects) {
            xs.push_back(clampDouble(r.x, 0.0, W));
            xs.push_back(clampDouble(rectRight(r), 0.0, W));
        }
        sort(xs.begin(), xs.end());
        xs.erase(unique(xs.begin(), xs.end(), [](double a, double b) { return fabs(a - b) < 1.0e-4; }), xs.end());

        for (int i = 0; i + 1 < static_cast<int>(xs.size()); ++i) {
            const double x1 = xs[i];
            const double x2 = xs[i + 1];
            const double cw = x2 - x1;
            if (cw <= EPS) continue;

            vector<pair<double, double>> covered;
            covered.reserve(rects.size());
            for (const Rect& r : rects) {
                if (rectRight(r) <= x1 + EPS || r.x >= x2 - EPS) continue;
                double a = clampDouble(r.y, 0.0, H);
                double b = clampDouble(rectTop(r), 0.0, H);
                if (b > a + EPS) covered.push_back({ a, b });
            }
            sort(covered.begin(), covered.end());
            vector<pair<double, double>> merged;
            for (auto [a, b] : covered) addMergedInterval(merged, a, b);

            double yPrev = 0.0;
            auto scoreChannel = [&](double ch, double y1, double y2) {
                if (ch <= EPS || y2 <= y1 + EPS) return;
                const double areaScale = max(1.0, min(cw * ch, reqMax * reqMax * 4.0) / max(1.0, reqMax));

                // Very tiny dimensions are the most dangerous: they are usually
                // numerical artifacts from almost-aligned block edges.  Penalize
                // them harder so SA either snaps them closed or opens real space.
                if (cw < snap) {
                    penalty += 8.0 * sqr(reqMax - cw) * areaScale;
                }
                else if (cw < reqMax) {
                    penalty += sqr(reqMax - cw) * 0.15 * areaScale;
                }
                if (ch < snap) {
                    penalty += 8.0 * sqr(reqMax - ch) * areaScale;
                }
                else if (ch < reqMax) {
                    penalty += sqr(reqMax - ch) * 0.15 * areaScale;
                }
                };

            for (const auto& seg : merged) {
                double a = max(0.0, seg.first);
                double b = min(H, seg.second);
                scoreChannel(a - yPrev, yPrev, a);
                yPrev = max(yPrev, b);
            }
            scoreChannel(H - yPrev, yPrev, H);
        }
        return penalty;
    }

    static bool closeXSliverGap(const Design& design, FPState& st, int aId, int bId, double snap) {
        Rect& a = st.rects[aId];
        Rect& b = st.rects[bId];
        if (overlapLen(a.y, rectTop(a), b.y, rectTop(b)) <= EPS) return false;

        bool aLeft = rectRight(a) <= b.x + EPS;
        bool bLeft = rectRight(b) <= a.x + EPS;
        if (!aLeft && !bLeft) return false;

        int leftId = aLeft ? aId : bId;
        int rightId = aLeft ? bId : aId;
        Rect& left = st.rects[leftId];
        Rect& right = st.rects[rightId];
        double gap = max(0.0, right.x - rectRight(left));
        if (gap <= EPS || gap >= snap) return false;

        bool leftMovable = isMovableForSA(design.blockSpecs[leftId]);
        bool rightMovable = isMovableForSA(design.blockSpecs[rightId]);

        // Prefer closing the numerical whitespace instead of widening it.  Closing
        // removes the generated channel, so the router cannot accidentally use it.
        if (rightMovable) {
            right.x -= gap;
        }
        else if (leftMovable) {
            left.x += gap;
        }
        else {
            return false;
        }
        return true;
    }

    static bool closeYSliverGap(const Design& design, FPState& st, int aId, int bId, double snap) {
        Rect& a = st.rects[aId];
        Rect& b = st.rects[bId];
        if (overlapLen(a.x, rectRight(a), b.x, rectRight(b)) <= EPS) return false;

        bool aBelow = rectTop(a) <= b.y + EPS;
        bool bBelow = rectTop(b) <= a.y + EPS;
        if (!aBelow && !bBelow) return false;

        int lowId = aBelow ? aId : bId;
        int highId = aBelow ? bId : aId;
        Rect& low = st.rects[lowId];
        Rect& high = st.rects[highId];
        double gap = max(0.0, high.y - rectTop(low));
        if (gap <= EPS || gap >= snap) return false;

        bool lowMovable = isMovableForSA(design.blockSpecs[lowId]);
        bool highMovable = isMovableForSA(design.blockSpecs[highId]);

        if (highMovable) {
            high.y -= gap;
        }
        else if (lowMovable) {
            low.y += gap;
        }
        else {
            return false;
        }
        return true;
    }

    static void closeNumericalSliverGaps(const Design& design, FPState& st, int maxPasses) {
        // Remove tiny gaps such as 0.093um or 0.036um before ChannelBuilder sees
        // them.  This is a final geometry cleanup; real channels above the snap
        // threshold are left for router capacity filtering.
        const double snap = sliverSnapThreshold(design);
        const int n = static_cast<int>(st.rects.size());
        for (int pass = 0; pass < maxPasses; ++pass) {
            normalizeOutlineInState(design, st);
            applyEdgeAndClamp(design, st, st.outlineW, st.outlineH);

            bool changed = false;
            for (int i = 0; i < n; ++i) {
                for (int j = i + 1; j < n; ++j) {
                    changed = closeXSliverGap(design, st, i, j, snap) || changed;
                    changed = closeYSliverGap(design, st, i, j, snap) || changed;
                }
            }
            normalizeOutlineInState(design, st);
            applyEdgeAndClamp(design, st, st.outlineW, st.outlineH);
            if (!changed) break;
            spreadRepair(design, st, 10);
        }
    }

    static double directChannelCapacityPenalty(const Design& design, const vector<Rect>& rects) {
        // Penalize connected block pairs that are separated by a tiny channel gap.
        // This specifically fixes cases like:
        //   PATH 1500 BLK03 3 CH16 1 CH16 3 BLK04 1
        // where the direct left-right channel must be at least 1500/25 = 60um, but
        // the generated vertical strip was only 0.25um wide.
        double p = 0.0;
        for (const auto& c : design.connections) {
            if (c.src < 0 || c.dst < 0 ||
                c.src >= static_cast<int>(rects.size()) ||
                c.dst >= static_cast<int>(rects.size())) {
                continue;
            }
            if (c.netCount < MIN_DIRECT_CHANNEL_GUARD_NETS) continue;

            const Rect& a = rects[c.src];
            const Rect& b = rects[c.dst];
            const double req = requiredChannelWidthForNets(c.netCount);
            const double demandWeight = max(1.0, static_cast<double>(c.netCount));

            // Left-right direct channel: checker/router will consume the x-width
            // of the channel rectangle when the path enters edge 1 and exits edge 3.
            double yOv = overlapLen(a.y, rectTop(a), b.y, rectTop(b));
            if (yOv > EPS) {
                double xGap = -1.0;
                if (rectRight(a) <= b.x + EPS) xGap = max(0.0, b.x - rectRight(a));
                else if (rectRight(b) <= a.x + EPS) xGap = max(0.0, a.x - rectRight(b));

                if (xGap >= 0.0 && xGap + EPS < req) {
                    double ovRatio = yOv / max(1.0, min(a.h, b.h));
                    p += demandWeight * sqr(req - xGap) * (1.0 + ovRatio);
                }
            }

            // Bottom-top direct channel/feed corridor.  This is symmetric and keeps
            // vertical direct routing from creating height slivers.
            double xOv = overlapLen(a.x, rectRight(a), b.x, rectRight(b));
            if (xOv > EPS) {
                double yGap = -1.0;
                if (rectTop(a) <= b.y + EPS) yGap = max(0.0, b.y - rectTop(a));
                else if (rectTop(b) <= a.y + EPS) yGap = max(0.0, a.y - rectTop(b));

                if (yGap >= 0.0 && yGap + EPS < req) {
                    double ovRatio = xOv / max(1.0, min(a.w, b.w));
                    p += demandWeight * sqr(req - yGap) * (1.0 + ovRatio);
                }
            }
        }
        return p;
    }

    static bool reserveXGapForPair(const Design& design, FPState& st, int aId, int bId, double req) {
        Rect& a = st.rects[aId];
        Rect& b = st.rects[bId];
        if (overlapLen(a.y, rectTop(a), b.y, rectTop(b)) <= EPS) return false;

        bool aLeft = rectRight(a) <= b.x + EPS;
        bool bLeft = rectRight(b) <= a.x + EPS;
        if (!aLeft && !bLeft) return false;

        int leftId = aLeft ? aId : bId;
        int rightId = aLeft ? bId : aId;
        Rect& left = st.rects[leftId];
        Rect& right = st.rects[rightId];
        double gap = max(0.0, right.x - rectRight(left));
        if (gap + EPS >= req) return false;

        double need = req - gap;
        bool leftMovable = isMovableForSA(design.blockSpecs[leftId]);
        bool rightMovable = isMovableForSA(design.blockSpecs[rightId]);

        // First try the least destructive fix: use remaining outline budget.  If
        // the right block is an EDGE block, applyEdgeAndClamp() will re-project it
        // to the new right boundary, directly creating more channel width.
        if (st.outlineW + need <= design.maxOutlineW + EPS) {
            st.outlineW = min(design.maxOutlineW, st.outlineW + need);
        }

        if (leftMovable && rightMovable) {
            left.x -= need * 0.5;
            right.x += need * 0.5;
        }
        else if (leftMovable) {
            left.x -= need;
        }
        else if (rightMovable) {
            right.x += need;
        }
        else {
            return false;
        }
        return true;
    }

    static bool reserveYGapForPair(const Design& design, FPState& st, int aId, int bId, double req) {
        Rect& a = st.rects[aId];
        Rect& b = st.rects[bId];
        if (overlapLen(a.x, rectRight(a), b.x, rectRight(b)) <= EPS) return false;

        bool aBelow = rectTop(a) <= b.y + EPS;
        bool bBelow = rectTop(b) <= a.y + EPS;
        if (!aBelow && !bBelow) return false;

        int lowId = aBelow ? aId : bId;
        int highId = aBelow ? bId : aId;
        Rect& low = st.rects[lowId];
        Rect& high = st.rects[highId];
        double gap = max(0.0, high.y - rectTop(low));
        if (gap + EPS >= req) return false;

        double need = req - gap;
        bool lowMovable = isMovableForSA(design.blockSpecs[lowId]);
        bool highMovable = isMovableForSA(design.blockSpecs[highId]);

        if (st.outlineH + need <= design.maxOutlineH + EPS) {
            st.outlineH = min(design.maxOutlineH, st.outlineH + need);
        }

        if (lowMovable && highMovable) {
            low.y -= need * 0.5;
            high.y += need * 0.5;
        }
        else if (lowMovable) {
            low.y -= need;
        }
        else if (highMovable) {
            high.y += need;
        }
        else {
            return false;
        }
        return true;
    }

    static void reserveDirectConnectionChannels(const Design& design, FPState& st, int maxPasses) {
        // Final hard-ish legalization pass for direct high-net channels.  Cost terms
        // guide SA toward these gaps; this pass catches remaining numerical slivers.
        if (design.connections.empty()) return;

        vector<int> order(design.connections.size());
        iota(order.begin(), order.end(), 0);
        sort(order.begin(), order.end(), [&](int ia, int ib) {
            return design.connections[ia].netCount > design.connections[ib].netCount;
            });

        for (int pass = 0; pass < maxPasses; ++pass) {
            normalizeOutlineInState(design, st);
            applyEdgeAndClamp(design, st, st.outlineW, st.outlineH);

            bool changed = false;
            for (int idx : order) {
                const auto& c = design.connections[idx];
                if (c.src < 0 || c.dst < 0 ||
                    c.src >= static_cast<int>(st.rects.size()) ||
                    c.dst >= static_cast<int>(st.rects.size())) {
                    continue;
                }
                if (c.netCount < MIN_DIRECT_CHANNEL_GUARD_NETS) continue;

                const double req = requiredChannelWidthForNets(c.netCount);
                changed = reserveXGapForPair(design, st, c.src, c.dst, req) || changed;
                changed = reserveYGapForPair(design, st, c.src, c.dst, req) || changed;
            }

            normalizeOutlineInState(design, st);
            applyEdgeAndClamp(design, st, st.outlineW, st.outlineH);
            if (changed) spreadRepair(design, st, 18);
            if (!changed) break;
        }
    }

    static void applyEdgeAndClamp(const Design& design, FPState& st, double W, double H) {
        // Clamp non-edge blocks first.  EDGE blocks are placed as a group below;
        // placing them one-by-one is wrong for TL/LT/BR/RB and can stack multiple
        // edge blocks at the same corner.
        for (int i = 0; i < static_cast<int>(st.rects.size()); ++i) {
            const BlockSpec& spec = design.blockSpecs[i];
            if (spec.type != BlockType::EDGE) {
                st.rects[i].x = clampDouble(st.rects[i].x, 0.0, W - st.rects[i].w);
                st.rects[i].y = clampDouble(st.rects[i].y, 0.0, H - st.rects[i].h);
            }
        }

        packEdgeBlocksNoOverlap(design, st, W, H);
    }

    static CostBreakdown evaluateState(const Design& design, const FPState& input) {
        CostBreakdown cb;
        if (input.rects.empty()) {
            cb.total = 0.0;
            return cb;
        }

        FPState st = input;
        normalizeOutlineInState(design, st);
        const double W = st.outlineW;
        const double H = st.outlineH;
        applyEdgeAndClamp(design, st, W, H);
        const vector<Rect>& rects = st.rects;

        // Outline violation.
        for (int i = 0; i < static_cast<int>(rects.size()); ++i) {
            const Rect& r = rects[i];
            double viol = 0.0;
            viol += max(0.0, -r.x);
            viol += max(0.0, -r.y);
            viol += max(0.0, rectRight(r) - W);
            viol += max(0.0, rectTop(r) - H);
            cb.rawOutlineViolation += viol;
            cb.outline += sqr(viol);
            cb.edge += sqr(edgeConstraintViolation(r, design.blockSpecs[i], W, H));
        }

        // Pairwise overlap penalty.
        int n = static_cast<int>(rects.size());
        int overlapCount = 0;
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                double area = rectOverlapArea(rects[i], rects[j]);
                if (area > EPS) {
                    cb.rawOverlapArea += area;
                    cb.overlap += area;
                    ++overlapCount;
                }
            }
        }
        cb.rawOverlapCount = overlapCount;
        cb.overlap = W_OVERLAP_AREA * cb.overlap + W_OVERLAP_COUNT * overlapCount;

        cb.hpwl = hpwlCost(design, rects);

        // Make the common illegal-candidate case fast.  When a candidate overlaps,
        // the huge overlap penalty dominates acceptance and best selection anyway,
        // so there is no benefit in running the expensive congestion estimator.
        const bool hasHardLegalViolation = (overlapCount > 0);
        if (hasHardLegalViolation) {
            cb.area = W * H;
            double centerCost = 0.0;
            for (const Rect& r : rects) {
                centerCost += manhattan(rectCx(r), rectCy(r), W * 0.5, H * 0.5);
            }
            cb.total =
                W_AREA * cb.area +
                W_HPWL * cb.hpwl +
                cb.overlap +
                W_OUTLINE * cb.outline +
                W_EDGE * cb.edge +
                W_CENTER * centerCost;
            return cb;
        }

        double ftPenalty = 0.0;
        auto [cong, peak] = congestionCost(design, rects, W, H, ftPenalty);
        cb.congestion = cong;
        cb.peak = peak;
        cb.ft = ftPenalty;
        cb.spacing = minGapPenalty(rects, 90.0);/////////////////////////////was 80.0
        cb.channelCapacity = directChannelCapacityPenalty(design, rects);
        cb.channelSliver = actualChannelSliverPenalty(design, rects, W, H);
        cb.softBridge = softBridgePenalty(design, rects);
        cb.commonEdgeReward = commonEdgeLikeScore(design, rects);

        // Slight center pull to avoid all blocks drifting to far edges when HPWL ties.
        double centerCost = 0.0;
        for (const Rect& r : rects) {
            centerCost += manhattan(rectCx(r), rectCy(r), W * 0.5, H * 0.5);
        }

        cb.area = W * H;

        const double congFactor = congestionPressureFactor(design);

        cb.total =
            W_AREA * cb.area +
            W_HPWL * cb.hpwl +
            (W_CONGESTION * congFactor) * cb.congestion +
            (W_PEAK * sqrt(congFactor)) * cb.peak +
            W_FT * cb.ft +
            cb.overlap +
            W_OUTLINE * cb.outline +
            W_EDGE * cb.edge +
            W_CHANNEL_SPACING * cb.spacing +
            W_DIRECT_CHANNEL_CAPACITY * cb.channelCapacity +
            W_GLOBAL_CHANNEL_SLIVER * cb.channelSliver +
            W_SOFT_BRIDGE * cb.softBridge -
            W_COMMON_EDGE * cb.commonEdgeReward +
            W_CENTER * centerCost;

        return cb;
    }


    static double floorplanProxyScore(const Design& design, const CostBreakdown& cb) {
        // Official contest cost is outlineArea + alpha * totalWireLength.
        // During floorplanning we do not have final routed WL yet, so HPWL is the
        // stable proxy.  Congestion / FT terms are still tie-breakers and safety
        // guards, but now their strength automatically follows nets/block.
        const double congFactor = congestionPressureFactor(design);
        return
            cb.area +
            0.20 * cb.hpwl +
            cb.overlap +
            W_OUTLINE * cb.outline +
            W_EDGE * cb.edge +
            (0.030 * congFactor) * cb.congestion +
            (3.000 * sqrt(congFactor)) * cb.peak +
            0.001 * cb.ft +
            0.030 * cb.spacing +
            25.000 * cb.channelCapacity +
            5.000 * cb.softBridge -
            0.003 * cb.commonEdgeReward;
    }

    static bool betterFloorplanProxy(const Design& design, const CostBreakdown& a, const CostBreakdown& b) {
        double pa = floorplanProxyScore(design, a);
        double pb = floorplanProxyScore(design, b);
        if (fabs(pa - pb) > 1e-6) return pa < pb;
        return a.total < b.total;
    }

    static bool isLegalState(const Design& design, const FPState& st) {
        FPState tmp = st;
        normalizeOutlineInState(design, tmp);
        applyEdgeAndClamp(design, tmp, tmp.outlineW, tmp.outlineH);
        const vector<Rect>& r = tmp.rects;
        for (const Rect& a : r) {
            if (!inside(a, tmp.outlineW, tmp.outlineH)) return false;
        }
        for (int i = 0; i < static_cast<int>(r.size()); ++i) {
            for (int j = i + 1; j < static_cast<int>(r.size()); ++j) {
                if (rectOverlapArea(r[i], r[j]) > EPS) return false;
            }
        }
        return true;
    }

    static vector<int> blockOrderByAreaAndDegree(const Design& design) {
        int n = static_cast<int>(design.blockSpecs.size());
        vector<int> ids(n);
        iota(ids.begin(), ids.end(), 0);

        // Prediction / memory hierarchy: degree is invariant during one testcase.
        // The previous sort comparator recomputed totalConnBetween() many times.
        vector<int> degree(n, 0);
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) degree[i] += totalConnBetween(design, i, j);
        }

        auto blockArea = [&](int id) {
            const BlockSpec& s = design.blockSpecs[id];
            return s.hasFixedSize ? s.fixedW * s.fixedH : max(1.0, s.area);
            };

        sort(ids.begin(), ids.end(), [&](int a, int b) {
            if (design.blockSpecs[a].type != design.blockSpecs[b].type) {
                if (design.blockSpecs[a].type == BlockType::EDGE) return true;
                if (design.blockSpecs[b].type == BlockType::EDGE) return false;
                if (design.blockSpecs[a].type == BlockType::HARD) return true;
                if (design.blockSpecs[b].type == BlockType::HARD) return false;
            }
            if (degree[a] != degree[b]) return degree[a] > degree[b];
            return blockArea(a) > blockArea(b);
            });
        return ids;
    }

    static void spreadRepair(const Design& design, FPState& st, int maxIters) {
        normalizeOutlineInState(design, st);
        const double W = st.outlineW;
        const double H = st.outlineH;
        int n = static_cast<int>(st.rects.size());

        applyEdgeAndClamp(design, st, W, H);

        for (int iter = 0; iter < maxIters; ++iter) {
            bool changed = false;
            for (int i = 0; i < n; ++i) {
                for (int j = i + 1; j < n; ++j) {
                    double ox = overlapLen(st.rects[i].x, rectRight(st.rects[i]), st.rects[j].x, rectRight(st.rects[j]));
                    double oy = overlapLen(st.rects[i].y, rectTop(st.rects[i]), st.rects[j].y, rectTop(st.rects[j]));
                    if (ox <= EPS || oy <= EPS) continue;

                    bool mi = isMovableForSA(design.blockSpecs[i]);
                    bool mj = isMovableForSA(design.blockSpecs[j]);
                    if (!mi && !mj) continue;

                    changed = true;
                    double pushX = ox * 0.55 + 2.0;
                    double pushY = oy * 0.55 + 2.0;
                    bool pushHorizontal = ox < oy;

                    if (pushHorizontal) {
                        double dir = (rectCx(st.rects[i]) <= rectCx(st.rects[j])) ? -1.0 : 1.0;
                        if (mi && mj) {
                            st.rects[i].x += dir * pushX * 0.5;
                            st.rects[j].x -= dir * pushX * 0.5;
                        }
                        else if (mi) {
                            st.rects[i].x += dir * pushX;
                        }
                        else if (mj) {
                            st.rects[j].x -= dir * pushX;
                        }
                    }
                    else {
                        double dir = (rectCy(st.rects[i]) <= rectCy(st.rects[j])) ? -1.0 : 1.0;
                        if (mi && mj) {
                            st.rects[i].y += dir * pushY * 0.5;
                            st.rects[j].y -= dir * pushY * 0.5;
                        }
                        else if (mi) {
                            st.rects[i].y += dir * pushY;
                        }
                        else if (mj) {
                            st.rects[j].y -= dir * pushY;
                        }
                    }

                    applyEdgeAndClamp(design, st, W, H);
                }
            }
            if (!changed) break;
        }
    }

    static FPState makeInitialState(const Design& design, mt19937& rng, int restartId) {
        int n = static_cast<int>(design.blockSpecs.size());
        FPState st;
        st.rects.assign(n, Rect{});
        st.softRatio.assign(n, 1.0);
        st.edgePos.assign(n, -1.0);
        st.edgeRule.assign(n, 0);
        st.outlineW = design.maxOutlineW;
        st.outlineH = design.maxOutlineH;

        for (int i = 0; i < n; ++i) {
            const BlockSpec& spec = design.blockSpecs[i];
            st.softRatio[i] = aspectMid(spec);
            st.rects[i] = makeShapeFromSpec(spec, st.softRatio[i]);
        }

        const double W = design.maxOutlineW;
        const double H = design.maxOutlineH;
        ensureEdge1DState(design, st, &rng, restartId);

        // Place edge blocks by contest location constraint first.
        for (int i = 0; i < n; ++i) {
            if (design.blockSpecs[i].type == BlockType::EDGE) {
                st.rects[i] = placeByLocationRule(st.rects[i], primaryLocation(design.blockSpecs[i]), W, H);
            }
        }

        vector<int> ids = blockOrderByAreaAndDegree(design);
        vector<int> movables;
        for (int id : ids) if (isMovableForSA(design.blockSpecs[id])) movables.push_back(id);

        // Several deterministic/randomized starts.  These are only starts; the SA is
        // what optimizes.  No old greedy candidate scoring is used here.
        if (restartId % 2 == 1) shuffle(movables.begin(), movables.end(), rng);
        if (restartId % 4 == 2) reverse(movables.begin(), movables.end());

        double totalArea = 0.0;
        for (int id : movables) totalArea += st.rects[id].w * st.rects[id].h;
        double targetRowW = clampDouble(sqrt(totalArea * W / max(1.0, H)) * (0.95 + 0.10 * restartId), W * 0.35, W * 0.92);

        double x = 0.0;
        double y = 0.0;
        double rowH = 0.0;
        double xJitter = (restartId % 3) * 17.0;

        for (int id : movables) {
            Rect& r = st.rects[id];
            if (x + r.w > targetRowW && x > EPS) {
                x = 0.0;
                y += rowH + 55.0;///////////////////55
                rowH = 0.0;
            }
            r.x = clampDouble(x + xJitter + randRange(rng, 0.0, 30.0), 0.0, W - r.w);
            r.y = clampDouble(y + randRange(rng, 0.0, 30.0), 0.0, H - r.h);
            x += r.w + 65.0;//////////////////65
            rowH = max(rowH, r.h);
        }

        // If shelves exceeded outline height, randomly scatter instead.
        double maxTop = 0.0;
        for (int id : movables) maxTop = max(maxTop, rectTop(st.rects[id]));
        if (maxTop > H + EPS) {
            for (int id : movables) {
                Rect& r = st.rects[id];
                r.x = randRange(rng, 0.0, max(0.0, W - r.w));
                r.y = randRange(rng, 0.0, max(0.0, H - r.h));
            }
        }

        applyEdgeAndClamp(design, st, W, H);
        spreadRepair(design, st, 160);
        return st;
    }

    static int chooseMovableBlock(const Design& design, mt19937& rng) {
        const vector<int>& ids = cachedMovableIds(design);
        if (ids.empty()) return -1;
        return ids[randInt(rng, 0, static_cast<int>(ids.size()) - 1)];
    }

    static int chooseSoftBlock(const Design& design, mt19937& rng) {
        const vector<int>& ids = cachedSoftIds(design);
        if (ids.empty()) return -1;
        return ids[randInt(rng, 0, static_cast<int>(ids.size()) - 1)];
    }

    static void mutateState(const Design& design, FPState& st, mt19937& rng, double temp01) {
        normalizeOutlineInState(design, st);
        double W = st.outlineW;
        double H = st.outlineH;
        int n = static_cast<int>(st.rects.size());
        if (n == 0) return;

        int moveType = randInt(rng, 0, 99);

        if (moveType < 13) {
            mutateEdge1DState(design, st, rng, temp01);
        }
        else if (moveType < 58) {
            int id = chooseMovableBlock(design, rng);
            if (id >= 0) {
                double stepX = (0.03 + 0.23 * temp01) * W;
                double stepY = (0.03 + 0.23 * temp01) * H;
                st.rects[id].x += randRange(rng, -stepX, stepX);
                st.rects[id].y += randRange(rng, -stepY, stepY);
            }
        }
        else if (moveType < 74) {
            int a = chooseMovableBlock(design, rng);
            int b = chooseMovableBlock(design, rng);
            if (a >= 0 && b >= 0 && a != b) {
                swap(st.rects[a].x, st.rects[b].x);
                swap(st.rects[a].y, st.rects[b].y);
            }
        }
        else if (moveType < 88) {
            // Soft aspect ratio mutation.
            int id = chooseSoftBlock(design, rng);
            if (id >= 0) {
                const BlockSpec& spec = design.blockSpecs[id];
                double oldCx = rectCx(st.rects[id]);
                double oldCy = rectCy(st.rects[id]);
                double logR = log(max(0.05, st.softRatio[id]));
                double range = (0.10 + 0.35 * temp01);
                logR += randRange(rng, -range, range);
                double nr = clampDouble(exp(logR), max(0.05, spec.aspectMin), max(max(0.05, spec.aspectMin), spec.aspectMax));
                st.softRatio[id] = nr;
                Rect shape = makeShapeFromSpec(spec, nr);
                st.rects[id].w = shape.w;
                st.rects[id].h = shape.h;
                st.rects[id].x = oldCx - shape.w * 0.5;
                st.rects[id].y = oldCy - shape.h * 0.5;
            }
        }
        else if (moveType < 95) {
            // Outline is part of the state.  Most outline moves try to shrink,
            // with a small chance to expand so SA can escape too-tight states.
            auto [minW0, minH0] = minOutlineWH(design);
            double shrinkMag = 0.002 + 0.012 * temp01;
            double expandMag = 0.001 + 0.006 * temp01;
            double fw = 1.0;
            double fh = 1.0;
            bool shrink = rand01(rng) < 0.78;////////////////0.78
            double mag = shrink ? randRange(rng, 0.0, shrinkMag) : -randRange(rng, 0.0, expandMag);
            int dim = randInt(rng, 0, 2);
            if (dim == 0 || dim == 2) fw = 1.0 - mag;
            if (dim == 1 || dim == 2) fh = 1.0 - mag;
            st.outlineW = clampDouble(st.outlineW * fw, minW0, design.maxOutlineW);
            st.outlineH = clampDouble(st.outlineH * fh, minH0, design.maxOutlineH);
            W = st.outlineW;
            H = st.outlineH;
        }
        else {
            // Small legalizing nudge away from one overlapping neighbor or from center.
            int id = chooseMovableBlock(design, rng);
            if (id >= 0) {
                double vx = rectCx(st.rects[id]) - W * 0.5;
                double vy = rectCy(st.rects[id]) - H * 0.5;
                double len = max(1.0, fabs(vx) + fabs(vy));
                st.rects[id].x += (vx / len) * (20.0 + 120.0 * temp01);
                st.rects[id].y += (vy / len) * (20.0 + 120.0 * temp01);
            }
        }

        normalizeOutlineInState(design, st);
        W = st.outlineW;
        H = st.outlineH;
        applyEdgeAndClamp(design, st, W, H);

        // Occasional local spreading helps the SA escape illegal overlap basins.
        if (rand01(rng) < 0.18) spreadRepair(design, st, 12);
    }



    static double estimateInitialTemperature(const Design& design, const FPState& start, mt19937& rng, int samples) {
        // Borrowed from the B*-tree SA style: sample random perturbations first,
        // estimate the average positive delta-cost, and choose T so the initial
        // uphill acceptance probability is around 0.90.  This is more stable than
        // using a fixed percentage of the absolute cost.
        constexpr double INITIAL_ACCEPT_PROB = 0.90;
        FPState cur = start;
        CostBreakdown curCost = evaluateState(design, cur);
        double sumPositive = 0.0;
        int cntPositive = 0;

        for (int i = 0; i < samples; ++i) {
            FPState nxt = cur;
            mutateState(design, nxt, rng, 1.0);
            CostBreakdown nxtCost = evaluateState(design, nxt);
            double d = nxtCost.total - curCost.total;
            if (d > 0.0 && std::isfinite(d)) {
                sumPositive += d;
                ++cntPositive;
            }
            cur = std::move(nxt);
            curCost = nxtCost;
        }

        double avg = (cntPositive > 0) ? (sumPositive / cntPositive) : max(1.0, evaluateState(design, start).total * 0.01);
        return max(1.0, avg / log(1.0 / INITIAL_ACCEPT_PROB));
    }

    // -----------------------------------------------------------------------------
    // DSU-inspired guarded outline shrink
    // -----------------------------------------------------------------------------
    // The previous shrink stage tried random outline scaling.  That can shrink area,
    // but it does not explicitly follow the deadspace-utilization idea: reclaim only
    // the unused whitespace and bound the compaction by congestion.  The helpers
    // below treat the current routing-aware solution as the "interconnect optimized
    // floorplan", estimate how much deadspace can safely be reclaimed, then apply
    // affine compaction to block centers while EDGE blocks are re-projected to the
    // new outline.  The quality guard keeps congestion / FT / HPWL close to the
    // anchor solution, similar to the congestion term G in DSU.

    static double dsuCompactionCapFromAnchor(const Design& design, const CostBreakdown& anchor) {
        if (anchor.area <= EPS) return 0.0;

        const double moduleArea = totalOriginalBlockArea(design);
        const double deadspaceRatio = clampDouble((anchor.area - moduleArea) / anchor.area, 0.0, 0.95);

        // If the floorplan is already dense, do not force outline shrink.
        // If there is large deadspace, allow at most about 10% area reduction by
        // default.  This mirrors the safer G=10 setting described in DSU: smaller
        // than aggressive 20%, but usually keeps routability stable.
        // Floorplan-first DSU cap:
        //   DSU says compaction should be bounded by a congestion term G.  Here the
        //   router is still a basic greedy baseline, so we do NOT let the estimator's
        //   absolute congestion dominate.  The cap is mainly decided by deadspace,
        //   while shrinkQualityGuard() later checks that congestion/FT do not explode
        //   relative to the anchor solution.
        double capByDeadspace = 0.0;
        if (deadspaceRatio > 0.035) {
            capByDeadspace = 0.55 * (deadspaceRatio - 0.035);
        }
        capByDeadspace = clampDouble(capByDeadspace, 0.0, 0.160);///////////////////////////0.160

        // A light G-like limiter, not an absolute veto.  This keeps the floorplan
        // from becoming corridor-free, but still allows meaningful area recovery.
        double capByCongestion = 0.160;
        if (anchor.congestion > 1.0e-6) capByCongestion = min(capByCongestion, 0.130);
        if (anchor.peak > 1.0e-6)       capByCongestion = min(capByCongestion, 0.120);
        if (anchor.ft > 1.0e5)          capByCongestion = min(capByCongestion, 0.125);

        // Dynamic G-like limit: more nets per block means compaction should be
        // more conservative because shrinking area removes channel/deadspace.
        const double pressure = congestionPressureFactor(design);
        double pressureCap = 0.160 / sqrt(max(1.0, pressure));
        pressureCap = clampDouble(pressureCap, 0.080, 0.160);
        capByCongestion = min(capByCongestion, pressureCap);

        double cap = min(capByDeadspace, capByCongestion);

        if (cap < 0.015 && deadspaceRatio > 0.10) cap = 0.015;
        return clampDouble(cap, 0.0, 0.160);
    }

    static void applyDSUAffineCompaction(
        const Design& design,
        FPState& st,
        double oldW,
        double oldH,
        double newW,
        double newH,
        bool compactX,
        bool compactY
    ) {
        oldW = max(oldW, 1.0);
        oldH = max(oldH, 1.0);
        double fx = compactX ? (newW / oldW) : 1.0;
        double fy = compactY ? (newH / oldH) : 1.0;

        st.outlineW = newW;
        st.outlineH = newH;

        for (int i = 0; i < static_cast<int>(st.rects.size()); ++i) {
            Rect& r = st.rects[i];
            const BlockSpec& spec = design.blockSpecs[i];

            if (spec.type == BlockType::EDGE) continue;

            // Scale the module center, not the module size.  This reclaims empty
            // gaps while keeping hard macros and the contest output rectangles
            // unchanged.  SOFT dimensions are only changed by explicit aspect-ratio
            // moves, not by this compaction step.
            double cx = rectCx(r);
            double cy = rectCy(r);
            if (compactX) r.x = cx * fx - 0.5 * r.w;
            if (compactY) r.y = cy * fy - 0.5 * r.h;
        }

        normalizeOutlineInState(design, st);
        applyEdgeAndClamp(design, st, st.outlineW, st.outlineH);
    }

    static bool shrinkQualityGuard(
        const CostBreakdown& cand,
        const CostBreakdown& anchor,
        double minAllowedArea
    ) {
        // Keep a good routing-aware solution: area may shrink, but do not allow
        // a dramatic degradation in congestion/FT/HPWL.  Legal violations are
        // filtered separately by isLegalState().
        if (cand.area + 1e-3 < minAllowedArea) return false;
        if (cand.area > anchor.area * 1.002) return false;

        // HPWL usually improves after compaction, but allow a small increase if it
        // buys significant outline reduction.
        if (cand.hpwl > anchor.hpwl * 1.120 + 8000.0) return false;

        // Congestion is the real guard.  If the anchor has no estimated overflow,
        // keep the candidate nearly overflow-free.  If it already has overflow,
        // allow only a limited relative degradation.
        if (anchor.congestion < 1.0e-6) {
            if (cand.congestion > 2500.0) return false;
        }
        else if (cand.congestion > anchor.congestion * 1.45 + 10000.0) {
            return false;
        }

        if (anchor.peak < 1.0e-6) {
            if (cand.peak > 50.0) return false;
        }
        else if (cand.peak > anchor.peak * 1.35 + 20.0) {
            return false;
        }

        // Feedthrough cost is allowed to move a little because some FT can replace
        // channel demand, but do not let shrink create a new FT-heavy solution.
        if (cand.ft > anchor.ft * 2.00 + 50000.0) return false;
        if (cand.spacing > anchor.spacing * 1.50 + 3000.0) return false;
        if (cand.channelCapacity > anchor.channelCapacity * 1.15 + 1000.0) return false;
        if (cand.softBridge > anchor.softBridge * 2.50 + 100.0) return false;
        return true;
    }

    static bool tryAcceptShrinkCandidate(
        const Design& design,
        const CostBreakdown& anchor,
        double minAllowedArea,
        FPState& best,
        CostBreakdown& bestCost,
        FPState cand,
        bool requireBetterTotal
    ) {
        normalizeOutlineInState(design, cand);
        applyEdgeAndClamp(design, cand, cand.outlineW, cand.outlineH);
        spreadRepair(design, cand, 70);

        CostBreakdown cc = evaluateState(design, cand);
        if (!costPlacementLegal(cc)) return false;
        if (!shrinkQualityGuard(cc, anchor, minAllowedArea)) return false;
        if (cc.area >= bestCost.area - 1e-3) return false;

        // In DSU mode, prefer smaller legal area even if the weighted cost is very
        // slightly worse, as long as the quality guard says routability is kept.
        if (requireBetterTotal && cc.total > bestCost.total + anchor.area * 0.0200) return false;
        if (!requireBetterTotal && cc.total > bestCost.total + anchor.area * 0.0400) return false;

        best = std::move(cand);
        bestCost = cc;
        return true;
    }

    static void deterministicDSUCompaction(
        const Design& design,
        const CostBreakdown& anchor,
        double minAllowedArea,
        FPState& best,
        CostBreakdown& bestCost
    ) {
        // A small line search before SA: compact W, H, and both dimensions.  This
        // is much more stable than pure random shrink and directly reclaims the
        // whitespace of the current solution.
        double step = 0.020;
        for (int pass = 0; pass < 16; ++pass) {
            bool improved = false;
            for (int dim = 0; dim < 3; ++dim) {
                FPState cand = best;
                double oldW = cand.outlineW;
                double oldH = cand.outlineH;
                double newW = oldW;
                double newH = oldH;
                bool cx = (dim == 0 || dim == 2);
                bool cy = (dim == 1 || dim == 2);

                if (cx) newW = oldW * (1.0 - step);
                if (cy) newH = oldH * (1.0 - step);

                auto [minW0, minH0] = minOutlineWH(design);
                newW = clampDouble(newW, minW0, design.maxOutlineW);
                newH = clampDouble(newH, minH0, design.maxOutlineH);
                if (newW * newH < minAllowedArea) continue;

                applyDSUAffineCompaction(design, cand, oldW, oldH, newW, newH, cx, cy);
                if (tryAcceptShrinkCandidate(design, anchor, minAllowedArea, best, bestCost, std::move(cand), true)) {
                    improved = true;
                }
            }
            if (!improved) step *= 0.58;
            if (step < 0.0012) break;
        }
    }

    static FPState guardedOutlineShrink(const Design& design, FPState bestIn, mt19937& rng) {
        normalizeOutlineInState(design, bestIn);
        applyEdgeAndClamp(design, bestIn, bestIn.outlineW, bestIn.outlineH);
        spreadRepair(design, bestIn, 300);

        CostBreakdown anchor = evaluateState(design, bestIn);
        if (!costPlacementLegal(anchor)) return bestIn;

        const double cap = dsuCompactionCapFromAnchor(design, anchor);
        const double minAllowedArea = anchor.area * (1.0 - cap);

        FPState best = bestIn;
        CostBreakdown bestCost = anchor;

        deterministicDSUCompaction(design, anchor, minAllowedArea, best, bestCost);

        FPState cur = best;
        CostBreakdown curCost = bestCost;

        const int n = static_cast<int>(design.blockSpecs.size());
        const int iters = 2800 + 220 * n;
        double T0 = max(1.0, anchor.area * 0.010 + anchor.hpwl * 0.0015);
        double Tend = max(1e-4, T0 * 1e-4);

        auto [minW0, minH0] = minOutlineWH(design);

        for (int iter = 0; iter < iters; ++iter) {
            double progress = static_cast<double>(iter) / max(1, iters - 1);
            double T = T0 * pow(Tend / T0, progress);
            double temp01 = max(0.0, 1.0 - progress);

            FPState cand = cur;
            double oldW = cand.outlineW;
            double oldH = cand.outlineH;

            int mt = randInt(rng, 0, 99);
            if (mt < 58) {
                // DSU-style compaction move.  Area shrink is bounded by G-like cap.
                double step = randRange(rng, 0.0010, 0.0080 + 0.0050 * temp01);
                int dim = randInt(rng, 0, 2);
                bool cx = (dim == 0 || dim == 2);
                bool cy = (dim == 1 || dim == 2);
                double newW = cx ? oldW * (1.0 - step) : oldW;
                double newH = cy ? oldH * (1.0 - step) : oldH;
                newW = clampDouble(newW, minW0, design.maxOutlineW);
                newH = clampDouble(newH, minH0, design.maxOutlineH);
                if (newW * newH < minAllowedArea) continue;
                applyDSUAffineCompaction(design, cand, oldW, oldH, newW, newH, cx, cy);
            }
            else if (mt < 78) {
                // Local whitespace utilization: move one module with its center
                // scaled slightly toward the current compacted outline.
                int id = chooseMovableBlock(design, rng);
                if (id >= 0) {
                    double sx = randRange(rng, 0.985, 1.005);
                    double sy = randRange(rng, 0.985, 1.005);
                    double cx = rectCx(cand.rects[id]) * sx;
                    double cy = rectCy(cand.rects[id]) * sy;
                    cand.rects[id].x = cx - 0.5 * cand.rects[id].w;
                    cand.rects[id].y = cy - 0.5 * cand.rects[id].h;
                }
            }
            else if (mt < 92) {
                // Soft aspect ratio tweak: useful when the current critical span is
                // limited by a soft block.  This follows the contest's soft-block
                // aspect-ratio flexibility without changing its area.
                int id = chooseSoftBlock(design, rng);
                if (id >= 0) {
                    const BlockSpec& spec = design.blockSpecs[id];
                    double oldCx = rectCx(cand.rects[id]);
                    double oldCy = rectCy(cand.rects[id]);
                    double nr = clampDouble(cand.softRatio[id] * exp(randRange(rng, -0.08, 0.08)),
                        max(0.05, spec.aspectMin), max(max(0.05, spec.aspectMin), spec.aspectMax));
                    cand.softRatio[id] = nr;
                    Rect shape = makeShapeFromSpec(spec, nr);
                    cand.rects[id].w = shape.w;
                    cand.rects[id].h = shape.h;
                    cand.rects[id].x = oldCx - 0.5 * shape.w;
                    cand.rects[id].y = oldCy - 0.5 * shape.h;
                }
            }
            else {
                // Rare recovery expansion, but never above the original max outline.
                double f = 1.0 + randRange(rng, 0.001, 0.0035);
                cand.outlineW = clampDouble(cand.outlineW * f, minW0, design.maxOutlineW);
                cand.outlineH = clampDouble(cand.outlineH * f, minH0, design.maxOutlineH);
                applyDSUAffineCompaction(design, cand, oldW, oldH, cand.outlineW, cand.outlineH, true, true);
            }

            normalizeOutlineInState(design, cand);
            applyEdgeAndClamp(design, cand, cand.outlineW, cand.outlineH);
            if (rand01(rng) < 0.58) spreadRepair(design, cand, 46);

            CostBreakdown cc = evaluateState(design, cand);
            if (!costPlacementLegal(cc)) continue;
            if (!shrinkQualityGuard(cc, anchor, minAllowedArea)) continue;

            // SA is still used, but the guard and minAllowedArea prevent the random
            // walk from destroying the interconnect-aware solution.
            double diff = cc.total - curCost.total;
            bool accept = diff <= 0.0 || rand01(rng) < exp(-diff / max(T, 1e-12));
            if (accept) {
                cur = std::move(cand);
                curCost = cc;
                if (cc.area < bestCost.area - 1e-3 && cc.total <= bestCost.total + anchor.area * 0.0030) {
                    best = cur;
                    bestCost = cc;
                }
                else if (cc.total < bestCost.total && cc.area <= bestCost.area * 1.002) {
                    best = cur;
                    bestCost = cc;
                }
            }
        }

        cerr << "[OutlineShrink/DSU] cap=" << cap
            << " pressure=" << congestionPressureFactor(design)
            << " area " << anchor.area << " -> " << bestCost.area
            << " W/H " << bestIn.outlineW << "x" << bestIn.outlineH
            << " -> " << best.outlineW << "x" << best.outlineH
            << " hpwl " << anchor.hpwl << " -> " << bestCost.hpwl
            << " cong " << anchor.congestion << " -> " << bestCost.congestion
            << " peak " << anchor.peak << " -> " << bestCost.peak
            << " ft " << anchor.ft << " -> " << bestCost.ft
            << "\n";
        return best;
    }



    static bool acceptProxyImprovement(
        const Design& design,
        const CostBreakdown& anchor,
        double minAllowedArea,
        FPState& best,
        CostBreakdown& bestCost,
        FPState cand
    ) {
        normalizeOutlineInState(design, cand);
        applyEdgeAndClamp(design, cand, cand.outlineW, cand.outlineH);
        spreadRepair(design, cand, 34);

        CostBreakdown cc = evaluateState(design, cand);
        if (!costPlacementLegal(cc)) return false;
        if (!shrinkQualityGuard(cc, anchor, minAllowedArea)) return false;

        double pc = floorplanProxyScore(design, cc);
        double pb = floorplanProxyScore(design, bestCost);
        if (pc + 1e-4 < pb) {
            best = std::move(cand);
            bestCost = cc;
            return true;
        }
        return false;
    }

    static FPState localProxyPolish(const Design& design, FPState start, mt19937& rng) {
        (void)rng;
        normalizeOutlineInState(design, start);
        applyEdgeAndClamp(design, start, start.outlineW, start.outlineH);
        spreadRepair(design, start, 120);

        CostBreakdown anchor = evaluateState(design, start);
        if (!costPlacementLegal(anchor)) return start;
        FPState best = start;
        CostBreakdown bestCost = anchor;

        // Conservative post-processing: only accept candidates that improve the
        // official-like proxy, so this pass should not worsen the floorplan result.
        double minAllowedArea = anchor.area * 0.90;
        vector<double> steps = {
            max(8.0, min(best.outlineW, best.outlineH) * 0.018),
            max(5.0, min(best.outlineW, best.outlineH) * 0.010),
            max(3.0, min(best.outlineW, best.outlineH) * 0.005),
            1.5
        };

        auto [minW0, minH0] = minOutlineWH(design);

        for (double step : steps) {
            bool improved = true;
            int pass = 0;
            while (improved && pass++ < 3) {
                improved = false;

                for (int dim = 0; dim < 3; ++dim) {
                    FPState cand = best;
                    bool cx = (dim == 0 || dim == 2);
                    bool cy = (dim == 1 || dim == 2);
                    double oldW = cand.outlineW;
                    double oldH = cand.outlineH;
                    double newW = cx ? max(minW0, oldW - step) : oldW;
                    double newH = cy ? max(minH0, oldH - step) : oldH;
                    if (newW * newH >= minAllowedArea) {
                        applyDSUAffineCompaction(design, cand, oldW, oldH, newW, newH, cx, cy);
                        if (acceptProxyImprovement(design, anchor, minAllowedArea, best, bestCost, std::move(cand))) improved = true;
                    }
                }

                for (int id = 0; id < static_cast<int>(design.blockSpecs.size()); ++id) {
                    if (!isMovableForSA(design.blockSpecs[id])) continue;
                    const double dirs[4][2] = { {1,0},{-1,0},{0,1},{0,-1} };
                    for (auto& d : dirs) {
                        FPState cand = best;
                        cand.rects[id].x += d[0] * step;
                        cand.rects[id].y += d[1] * step;
                        if (acceptProxyImprovement(design, anchor, minAllowedArea, best, bestCost, std::move(cand))) improved = true;
                    }
                }

                for (int id = 0; id < static_cast<int>(design.blockSpecs.size()); ++id) {
                    if (design.blockSpecs[id].type != BlockType::SOFT) continue;
                    for (double f : {0.96, 1.04}) {
                        FPState cand = best;
                        const BlockSpec& spec = design.blockSpecs[id];
                        double oldCx = rectCx(cand.rects[id]);
                        double oldCy = rectCy(cand.rects[id]);
                        double nr = clampDouble(cand.softRatio[id] * f,
                            max(0.05, spec.aspectMin), max(max(0.05, spec.aspectMin), spec.aspectMax));
                        if (fabs(nr - cand.softRatio[id]) < 1e-6) continue;
                        cand.softRatio[id] = nr;
                        Rect shape = makeShapeFromSpec(spec, nr);
                        cand.rects[id].w = shape.w;
                        cand.rects[id].h = shape.h;
                        cand.rects[id].x = oldCx - 0.5 * shape.w;
                        cand.rects[id].y = oldCy - 0.5 * shape.h;
                        if (acceptProxyImprovement(design, anchor, minAllowedArea, best, bestCost, std::move(cand))) improved = true;
                    }
                }
            }
        }

        cerr << "[FloorplannerPolish] proxy " << floorplanProxyScore(design, anchor)
            << " -> " << floorplanProxyScore(design, bestCost)
            << " area " << anchor.area << " -> " << bestCost.area
            << " hpwl " << anchor.hpwl << " -> " << bestCost.hpwl
            << " cong " << anchor.congestion << " -> " << bestCost.congestion
            << " ft " << anchor.ft << " -> " << bestCost.ft << "\n";
        return best;
    }

    static void writeStateToDesign(Design& design, FPState st) {
        normalizeOutlineInState(design, st);
        applyEdgeAndClamp(design, st, st.outlineW, st.outlineH);
        spreadRepair(design, st, 300);
        // spreadRepair can move non-edge blocks; run edge legalization one more
        // time before committing the rectangles to design.blocks.
        applyEdgeAndClamp(design, st, st.outlineW, st.outlineH);
        closeNumericalSliverGaps(design, st, 10);
        reserveDirectConnectionChannels(design, st, 8);
        spreadRepair(design, st, 80);
        closeNumericalSliverGaps(design, st, 6);
        reserveDirectConnectionChannels(design, st, 5);
        applyEdgeAndClamp(design, st, st.outlineW, st.outlineH);
        normalizeOutlineInState(design, st);

        design.outlineW = st.outlineW;
        design.outlineH = st.outlineH;
        design.blocks.clear();
        design.blocks.reserve(design.blockSpecs.size());

        for (int i = 0; i < static_cast<int>(design.blockSpecs.size()); ++i) {
            BlockInst b;
            b.spec = design.blockSpecs[i];
            b.rect = st.rects[i];
            b.rect.x = clampDouble(b.rect.x, 0.0, design.outlineW - b.rect.w);
            b.rect.y = clampDouble(b.rect.y, 0.0, design.outlineH - b.rect.h);
            // EDGE blocks were already placed collectively by applyEdgeAndClamp().
            // Do not call placeByLocationRule() here, otherwise multiple edge blocks
            // in the same boundary region can be projected back onto the same corner.
            b.ftUsed = 0.0;
            b.ftOverflowArea = 0.0;
            design.blocks.push_back(b);
        }

        // Keep name-to-index consistent with design.blocks order.
        design.blockNameToIndex.clear();
        for (int i = 0; i < static_cast<int>(design.blocks.size()); ++i) {
            design.blockNameToIndex[design.blocks[i].spec.name] = i;
        }
    }

} // namespace

void Floorplanner::run(Design& design) {
    design.outlineW = design.maxOutlineW;
    design.outlineH = design.maxOutlineH;

    if (design.blockSpecs.empty()) {
        design.blocks.clear();
        return;
    }

    const unsigned seedBase = 1u;
    const int n = static_cast<int>(design.blockSpecs.size());
    const int iters = SA_BASE_ITERS + SA_ITERS_PER_BLOCK * n;

    // Prewarm invariant caches before parallel restart workers.  This keeps the
    // cache maps read-only during worker execution and avoids redundant first-use
    // work in every restart.
    (void)totalOriginalBlockArea(design);
    (void)totalNetDemand(design);
    (void)congestionPressureFactor(design);
    (void)maxRequiredChannelWidth(design);
    (void)minOutlineWH(design);
    (void)cachedMovableIds(design);
    (void)cachedSoftIds(design);
    (void)cachedEdgeOrder(design);
    for (const auto& spec : design.blockSpecs) {
        if (spec.type == BlockType::EDGE) (void)collectEdgeLocRules(spec);
    }

    struct RestartResult {
        FPState best;
        CostBreakdown cost;
        double proxy = INF_COST;
        string log;
    };

    auto runOneRestart = [&](int restart) -> RestartResult {
        mt19937 rng(seedBase + 1009u * static_cast<unsigned>(restart) + 17u * static_cast<unsigned>(n));

        FPState cur = makeInitialState(design, rng, restart);
        CostBreakdown curCost = evaluateState(design, cur);
        FPState best = cur;
        CostBreakdown bestCost = curCost;
        double bestProxy = floorplanProxyScore(design, curCost);

        double T0 = max(1.0, curCost.total * 0.08);
        double Tend = max(1.0e-3, T0 * 1.0e-5);

        for (int iter = 0; iter < iters; ++iter) {
            double progress = static_cast<double>(iter) / max(1, iters - 1);
            double T = T0 * pow(Tend / T0, progress);
            double temp01 = max(0.0, 1.0 - progress);

            FPState nxt = cur;
            mutateState(design, nxt, rng, temp01);
            CostBreakdown nxtCost = evaluateState(design, nxt);

            // evaluateState() already applies edge legalization and computes all
            // placement violations.  Reuse that summary instead of calling
            // isLegalState(), which would repeat edge packing and overlap scans.
            double nxtProxy = floorplanProxyScore(design, nxtCost);
            if (nxtProxy < bestProxy && costPlacementLegal(nxtCost)) {
                best = nxt;
                bestCost = nxtCost;
                bestProxy = nxtProxy;
            }

            double diff = nxtCost.total - curCost.total;
            bool accept = diff <= 0.0;
            if (!accept) {
                double p = exp(-diff / max(T, 1.0e-12));
                accept = rand01(rng) < p;
            }

            if (accept) {
                cur = std::move(nxt);
                curCost = nxtCost;
                double curProxy = floorplanProxyScore(design, curCost);
                if (curProxy < bestProxy || (fabs(curProxy - bestProxy) < 1e-6 && curCost.total < bestCost.total)) {
                    best = cur;
                    bestCost = curCost;
                    bestProxy = curProxy;
                }
            }
        }

        // Redundancy only at restart boundary, not in every inner-loop comparison.
        spreadRepair(design, best, 300);
        bestCost = evaluateState(design, best);
        bestProxy = floorplanProxyScore(design, bestCost);

        ostringstream oss;
        oss << "[FloorplannerSA] restart=" << restart
            << " best=" << bestCost.total
            << " area=" << bestCost.area
            << " W/H=" << best.outlineW << "x" << best.outlineH
            << " proxy=" << bestProxy
            << " pressure=" << congestionPressureFactor(design)
            << " hpwl=" << bestCost.hpwl
            << " cong=" << bestCost.congestion
            << " peak=" << bestCost.peak
            << " overlapTerm=" << bestCost.overlap
            << " ftArea=" << bestCost.ft
            << " channelCap=" << bestCost.channelCapacity
            << " bridge=" << bestCost.softBridge
            << " commonEdge=" << bestCost.commonEdgeReward
            << " legal=" << (costPlacementLegal(bestCost) ? "Y" : "N")
            << "\n";

        RestartResult rr;
        rr.best = std::move(best);
        rr.cost = bestCost;
        rr.proxy = bestProxy;
        rr.log = oss.str();
        return rr;
        };

    FPState globalBest;
    CostBreakdown globalBestCost;
    globalBestCost.total = INF_COST;
    double globalBestProxy = INF_COST;

    auto consumeResult = [&](RestartResult&& rr) {
        cerr << rr.log;
        if (rr.proxy < globalBestProxy || (fabs(rr.proxy - globalBestProxy) < 1e-6 && rr.cost.total < globalBestCost.total)) {
            globalBest = std::move(rr.best);
            globalBestCost = rr.cost;
            globalBestProxy = rr.proxy;
        }
        };

    if (ENABLE_PARALLEL_RESTARTS && SA_RESTARTS > 1) {
        int batch = max(1, min(MAX_PARALLEL_RESTARTS, SA_RESTARTS));
        for (int base = 0; base < SA_RESTARTS; base += batch) {
            vector<future<RestartResult>> futs;
            int endRestart = min(SA_RESTARTS, base + batch);
            futs.reserve(endRestart - base);
            for (int restart = base; restart < endRestart; ++restart) {
                futs.push_back(async(std::launch::async, runOneRestart, restart));
            }
            for (auto& f : futs) consumeResult(f.get());
        }
    }
    else {
        for (int restart = 0; restart < SA_RESTARTS; ++restart) consumeResult(runOneRestart(restart));
    }

    {
        mt19937 shrinkRng(seedBase ^ 0x5a5a1234u);
        globalBest = guardedOutlineShrink(design, globalBest, shrinkRng);
        globalBest = localProxyPolish(design, globalBest, shrinkRng);
        globalBestCost = evaluateState(design, globalBest);
        globalBestProxy = floorplanProxyScore(design, globalBestCost);
    }

    writeStateToDesign(design, globalBest);

    // Final redundant safety check only once.  The hot loop uses CostBreakdown's
    // legality summary to avoid repeated edge legalization.
    if (!isLegalState(design, globalBest)) {
        cerr << "[FloorplannerSA] Warning: final state may still contain overlap/outline violation. "
            << "Try increasing SA_RESTARTS or reducing W_CONGESTION if this happens.\n";
    }
}

// --------------------------------------------------------------------------
// Compatibility implementations for the existing Floorplanner.hpp interface.
// The new run() above does not depend on the old greedy pipeline, but these
// methods remain available so the project builds without header changes.
// --------------------------------------------------------------------------

Rect Floorplanner::makeInitialShape(const BlockSpec& spec) const {
    return makeShapeFromSpec(spec, aspectMid(spec));
}

void Floorplanner::initBlockShapes(Design& design) {
    design.blocks.clear();
    design.blocks.reserve(design.blockSpecs.size());
    for (const auto& spec : design.blockSpecs) {
        BlockInst b;
        b.spec = spec;
        b.rect = makeInitialShape(spec);
        design.blocks.push_back(b);
    }
}

Rect Floorplanner::placeByLocation(const Rect& shape, const string& loc, double W, double H) const {
    return placeByLocationRule(shape, loc, W, H);
}

bool Floorplanner::insideOutline(const Rect& r, double W, double H) const {
    return inside(r, W, H);
}

bool Floorplanner::overlapsPlaced(const Rect& r, const vector<BlockInst>& blocks, const vector<bool>& placed) const {
    for (int i = 0; i < static_cast<int>(blocks.size()); ++i) {
        if (!placed[i]) continue;
        if (rectOverlapArea(r, blocks[i].rect) > EPS) return true;
    }
    return false;
}

double Floorplanner::connectionWeightToPlaced(int blockId, const Design& design, const vector<bool>& placed, const Rect& cand) const {
    double score = 0.0;
    for (int j = 0; j < static_cast<int>(design.blocks.size()); ++j) {
        if (!placed[j]) continue;
        int nets = totalConnBetween(design, blockId, j);
        if (nets > 0) {
            score += static_cast<double>(nets) *
                manhattan(rectCx(cand), rectCy(cand), rectCx(design.blocks[j].rect), rectCy(design.blocks[j].rect));
        }
    }
    return score;
}

vector<double> Floorplanner::collectCandidateXs(const Design& design, double w) const {
    vector<double> xs = { 0.0 };
    for (const auto& b : design.blocks) {
        xs.push_back(b.rect.x);
        xs.push_back(rectRight(b.rect));
        xs.push_back(max(0.0, b.rect.x - w));
    }
    xs.push_back(max(0.0, design.outlineW - w));
    sort(xs.begin(), xs.end());
    xs.erase(unique(xs.begin(), xs.end(), [](double a, double b) { return fabs(a - b) < 1e-3; }), xs.end());
    return xs;
}

vector<double> Floorplanner::collectCandidateYs(const Design& design, double h) const {
    vector<double> ys = { 0.0 };
    for (const auto& b : design.blocks) {
        ys.push_back(b.rect.y);
        ys.push_back(rectTop(b.rect));
        ys.push_back(max(0.0, b.rect.y - h));
    }
    ys.push_back(max(0.0, design.outlineH - h));
    sort(ys.begin(), ys.end());
    ys.erase(unique(ys.begin(), ys.end(), [](double a, double b) { return fabs(a - b) < 1e-3; }), ys.end());
    return ys;
}

void Floorplanner::placeEdgeBlocks(Design& design) {
    FPState st;
    st.outlineW = design.outlineW;
    st.outlineH = design.outlineH;
    st.rects.reserve(design.blocks.size());
    st.softRatio.assign(design.blocks.size(), 1.0);
    for (const auto& b : design.blocks) st.rects.push_back(b.rect);

    applyEdgeAndClamp(design, st, design.outlineW, design.outlineH);

    for (int i = 0; i < static_cast<int>(design.blocks.size()); ++i) {
        design.blocks[i].rect = st.rects[i];
    }
}

void Floorplanner::placeRemainingBlocksGreedy(Design& design) {
    // Deprecated by the SA flow.  Kept as a harmless no-op-style fallback.
    // If some old test harness calls this directly, do a small spread repair
    // instead of the former one-block greedy placement.
    FPState st;
    st.rects.reserve(design.blocks.size());
    st.softRatio.assign(design.blocks.size(), 1.0);
    for (const auto& b : design.blocks) st.rects.push_back(b.rect);
    spreadRepair(design, st, 200);
    for (int i = 0; i < static_cast<int>(design.blocks.size()); ++i) design.blocks[i].rect = st.rects[i];
}
