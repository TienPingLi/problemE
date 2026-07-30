#include "Floorplanner.hpp"
#include "ChannelBuilder.hpp"
#include "Utility.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <random>
#include <queue>
#include <set>
#include <stdexcept>
#include <string>
#include <functional>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

using namespace std;

namespace {

namespace phase1_warm_start {

    constexpr double FP_EPS = 1.0e-9;
    constexpr unsigned FINAL_FRAME_SEED = 5u;

    // These are the defaults of fast_sa_floorplan_channel_spread_v1(2).py.
    constexpr unsigned DEFAULT_SEED = 20260716u;
    constexpr int DEFAULT_RESTARTS = 12;
    constexpr int DEFAULT_SMALL_RESTARTS = 4;
    constexpr int DEFAULT_MEDIUM_RESTARTS = 10;
    constexpr int DEFAULT_TEMPERATURE_LEVELS = 125;
    constexpr int DEFAULT_FIXED_MOVES_PER_LEVEL = 16;
    constexpr int DEFAULT_WARMUP_MOVES_PER_BLOCK = 8;
    constexpr int DEFAULT_NORMALIZATION_SAMPLES = 128;
    constexpr int DEFAULT_ARCHIVE_LIMIT = 16;
    constexpr int DEFAULT_FINAL_CANDIDATES = 4;
    constexpr int DEFAULT_PORTFOLIO_FRAME_SEEDS = 3;
    constexpr int DEFAULT_PORTFOLIO_PHASE2_CANDIDATES = 6;
    constexpr int DEFAULT_MEDIUM_PORTFOLIO_KEEP = 24;
    constexpr int DEFAULT_PORTFOLIO_KEEP = 12;
    constexpr int DEFAULT_HPWL_FORCE_ROUNDS = 2;
    constexpr int DEFAULT_SOFT_POLISH_ROUNDS = 6;
    constexpr int DEFAULT_GREEDY_LEVELS = 7;
    constexpr double DEFAULT_GREEDY_C = 100.0;
    constexpr double DEFAULT_INITIAL_ACCEPT = 0.90;
    constexpr double DEFAULT_SUBTREE_RATE = 0.20;
    constexpr int DEFAULT_FRAME_ATTEMPTS = 8192;
    constexpr double DEFAULT_TARGET_UTILIZATION = 0.95;
    constexpr bool DEFAULT_ALLOW_MACRO_ROTATE = false;

    // The Python command-line default is false.  Set to true here only when the
    // project wants the same effect as passing --channel-spread.
    constexpr bool ENABLE_CHANNEL_SPREAD_BY_DEFAULT = false;
    constexpr double DEFAULT_CHANNEL_DENSITY = 25.0;
    constexpr int DEFAULT_CHANNEL_MAX_ROUTES = 16;
    constexpr double DEFAULT_CHANNEL_MIN_DEMAND = 1.0;
    constexpr double DEFAULT_CHANNEL_MIN_WIDTH = 1.0;
    constexpr double DEFAULT_CHANNEL_OUTLINE_GROWTH = 0.08;
    constexpr double DEFAULT_CHANNEL_MARGIN = 1.0e-4;
    constexpr int DEFAULT_CHANNEL_MAX_PUSHES = 256;
    constexpr int DEFAULT_CHANNEL_MAX_CHAIN = 128;

    constexpr bool ENABLE_DUMMY_ALIGNMENT = true;
    constexpr int DEFAULT_DUMMY_LEVELS = 120;
    constexpr int DEFAULT_DUMMY_MOVES = 16;
    constexpr int DEFAULT_DUMMY_FEEDBACK_LEVELS = 240;
    constexpr int DEFAULT_DUMMY_FEEDBACK_MOVES = 24;
    constexpr double DEFAULT_DUMMY_THICKNESS = 2.0;
    constexpr int DEFAULT_DUMMY_MAX_BLOCKS = 128;
    constexpr double DEFAULT_DUMMY_TEMPERATURE = 0.25;
    constexpr double DEFAULT_DUMMY_COOLING = 0.94;
    constexpr double DEFAULT_DUMMY_ALIGNMENT_WEIGHT = 0.35;
    constexpr double DEFAULT_DUMMY_MAIN_CHANNEL_WEIGHT = 1.25;
    constexpr double DEFAULT_DUMMY_BUFFER_WEIGHT = 0.35;
    constexpr double DEFAULT_DUMMY_DIRECT_GAP_WEIGHT = 0.40;
    constexpr double DEFAULT_DUMMY_FRAGMENT_WEIGHT = 0.12;
    constexpr double DEFAULT_DUMMY_HPWL_WEIGHT = 0.08;
    constexpr double DEFAULT_DUMMY_DISPLACEMENT_WEIGHT = 0.12;
    constexpr double DEFAULT_DUMMY_ROUTE_DENSITY = 25.0;
    constexpr int DEFAULT_DUMMY_MAIN_CHANNEL_LIMIT = 3;
    constexpr int DEFAULT_DUMMY_MAX_CUTS = 64;
    constexpr int DEFAULT_DUMMY_MAX_ACTIVE_CUTS = 12;
    constexpr int DEFAULT_DUMMY_TEMPERATURE_SAMPLES = 24;
    constexpr double DEFAULT_DUMMY_INITIAL_ACCEPT = 0.82;
    constexpr double DEFAULT_DUMMY_FEEDBACK_HEADROOM_BASE = 1.15;
    constexpr double DEFAULT_DUMMY_FEEDBACK_HEADROOM_PER_HOT_COMPONENT = 0.02;
    constexpr double DEFAULT_DUMMY_HOTSPOT_CAPACITY_MARGIN = 1.08;
    constexpr double DEFAULT_DUMMY_HOTSPOT_TRACK_MARGIN = 1.0;
    constexpr double DEFAULT_DUMMY_AREA_WEIGHT = 0.22;
    constexpr double DEFAULT_DUMMY_WASTED_AREA_WEIGHT = 0.10;

    static double clampD(double value, double low, double high) {
        if (low > high) return 0.5 * (low + high);
        return max(low, min(high, value));
    }

    static double rightOf(const Rect& r) { return r.x + r.w; }
    static double topOf(const Rect& r) { return r.y + r.h; }
    static pair<double, double> centerOf(const Rect& r) {
        return { r.x + 0.5 * r.w, r.y + 0.5 * r.h };
    }

    static double overlapArea(const Rect& a, const Rect& b) {
        const double ow = max(0.0, min(rightOf(a), rightOf(b)) - max(a.x, b.x));
        const double oh = max(0.0, min(topOf(a), topOf(b)) - max(a.y, b.y));
        return ow * oh;
    }

    static string upperAscii(string s) {
        transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
            return static_cast<char>(toupper(c));
            });
        return s;
    }

    static vector<string> splitLocationTokens(const vector<string>& raw) {
        vector<string> out;
        for (const string& cell : raw) {
            string cur;
            for (char c : cell) {
                if (c == ',' || c == ';' || c == '/' || isspace(static_cast<unsigned char>(c))) {
                    if (!cur.empty()) {
                        string token = upperAscii(cur);
                        if (find(out.begin(), out.end(), token) == out.end()) out.push_back(token);
                        cur.clear();
                    }
                }
                else {
                    cur.push_back(c);
                }
            }
            if (!cur.empty()) {
                string token = upperAscii(cur);
                if (find(out.begin(), out.end(), token) == out.end()) out.push_back(token);
            }
        }
        return out;
    }

    enum class Side { NONE, LEFT, RIGHT, BOTTOM, TOP };

    struct RegionInfo {
        Side side = Side::NONE;
        int segment = 0;
    };

    static optional<RegionInfo> locationRegion(const string& token) {
        static const map<string, RegionInfo> table = {
            {"BL", {Side::BOTTOM, 0}}, {"BM", {Side::BOTTOM, 1}}, {"BR", {Side::BOTTOM, 2}},
            {"LB", {Side::LEFT, 0}},   {"LM", {Side::LEFT, 1}},   {"LT", {Side::LEFT, 2}},
            {"RB", {Side::RIGHT, 0}}, {"RM", {Side::RIGHT, 1}},  {"RT", {Side::RIGHT, 2}},
            {"TL", {Side::TOP, 0}},   {"TM", {Side::TOP, 1}},    {"TR", {Side::TOP, 2}},
        };
        auto it = table.find(upperAscii(token));
        if (it == table.end()) return nullopt;
        return it->second;
    }

    static const char* sideName(Side side) {
        switch (side) {
        case Side::LEFT: return "LEFT";
        case Side::RIGHT: return "RIGHT";
        case Side::BOTTOM: return "BOTTOM";
        case Side::TOP: return "TOP";
        default: return "NONE";
        }
    }

    struct PBlock {
        int index = -1;
        string name;
        double area = 0.0;
        double baseW = 0.0;
        double baseH = 0.0;
        double arMin = 0.5;
        double arMax = 2.0;
        BlockType type = BlockType::HARD;
        vector<string> tokens;
        int portEdge = 0;

        bool isSoft() const { return type == BlockType::SOFT; }
        bool isEdge() const { return type == BlockType::EDGE; }
        bool isMacro() const { return type == BlockType::HARD; }

        Side boundarySide() const {
            if (!isEdge()) return Side::NONE;
            for (const string& token : tokens) {
                auto info = locationRegion(token);
                if (info) return info->side;
            }
            return Side::NONE;
        }

        vector<Side> boundarySides() const {
            vector<Side> sides;
            for (const string& token : tokens) {
                auto info = locationRegion(token);
                if (!info) continue;
                if (find(sides.begin(), sides.end(), info->side) == sides.end()) sides.push_back(info->side);
            }
            return sides;
        }

        vector<int> annealSegments() const {
            vector<int> segments;
            const Side primary = boundarySide();
            for (const string& token : tokens) {
                auto info = locationRegion(token);
                if (!info || info->side != primary) continue;
                if (find(segments.begin(), segments.end(), info->segment) == segments.end()) {
                    segments.push_back(info->segment);
                }
            }
            sort(segments.begin(), segments.end());
            return segments;
        }

        bool isCornerEdge() const {
            const vector<Side> sides = boundarySides();
            const bool horizontal = find(sides.begin(), sides.end(), Side::LEFT) != sides.end() ||
                find(sides.begin(), sides.end(), Side::RIGHT) != sides.end();
            const bool vertical = find(sides.begin(), sides.end(), Side::BOTTOM) != sides.end() ||
                find(sides.begin(), sides.end(), Side::TOP) != sides.end();
            return horizontal && vertical;
        }
    };

    struct PConnection {
        int source = -1;
        int target = -1;
        double weight = 0.0;
    };

    struct Problem {
        vector<PBlock> blocks;
        double maxW = 0.0;
        double maxH = 0.0;
        double alpha = 0.2;
        vector<PConnection> connections;
        vector<set<Side>> sideSets;
        map<Side, vector<int>> boundaryIndices;
        double totalArea = 0.0;
        double boundaryMinW = 0.0;
        double boundaryMinH = 0.0;
    };

    static Problem makeProblem(const Design& design) {
        Problem p;
        p.maxW = design.maxOutlineW;
        p.maxH = design.maxOutlineH;
        // The project Design does not expose a separate alpha field in the supplied
        // interface.  Keep the Python default when no command-line/CSV alpha exists.
        p.alpha = 0.2;
        p.blocks.reserve(design.blockSpecs.size());
        for (int i = 0; i < static_cast<int>(design.blockSpecs.size()); ++i) {
            const BlockSpec& s = design.blockSpecs[i];
            PBlock b;
            b.index = i;
            b.name = s.name;
            b.type = s.type;
            const double floorplanArea = s.floorplanAreaOverride > FP_EPS ? max(s.area, s.floorplanAreaOverride) : s.area;
            b.area = s.hasFixedSize ? s.fixedW * s.fixedH : floorplanArea;
            b.arMin = max(1.0e-6, s.aspectMin);
            b.arMax = max(b.arMin, s.aspectMax);
            if (s.hasFixedSize) {
                b.baseW = s.fixedW;
                b.baseH = s.fixedH;
            }
            else {
                const double ratio = clampD(1.0, b.arMin, b.arMax);
                b.baseW = sqrt(max(b.area, FP_EPS) * ratio);
                b.baseH = b.area / max(b.baseW, FP_EPS);
            }
            b.tokens = splitLocationTokens(s.locations);
            b.portEdge = s.portEdges.empty() ? 0 : s.portEdges.front();
            if (b.isEdge() && b.boundarySide() == Side::NONE) {
                throw runtime_error(b.name + ": EDGE block requires a valid LOCATION token");
            }
            p.totalArea += b.area;
            p.blocks.push_back(std::move(b));
        }

        if (!design.connections.empty()) {
            for (const auto& c : design.connections) {
                if (c.src < 0 || c.dst < 0 || c.src >= static_cast<int>(p.blocks.size()) ||
                    c.dst >= static_cast<int>(p.blocks.size()) || c.netCount <= 0) continue;
                p.connections.push_back({ c.src, c.dst, static_cast<double>(c.netCount) });
            }
        }
        else {
            for (int i = 0; i < static_cast<int>(design.connMatrix.size()); ++i) {
                for (int j = 0; j < static_cast<int>(design.connMatrix[i].size()); ++j) {
                    if (i == j || design.connMatrix[i][j] <= 0) continue;
                    p.connections.push_back({ i, j, static_cast<double>(design.connMatrix[i][j]) });
                }
            }
        }

        p.sideSets.resize(p.blocks.size());
        for (const PBlock& b : p.blocks) {
            for (Side s : b.boundarySides()) {
                p.sideSets[b.index].insert(s);
                p.boundaryIndices[s].push_back(b.index);
            }
        }
        double bottomW = 0.0, topW = 0.0, leftH = 0.0, rightH = 0.0;
        for (int id : p.boundaryIndices[Side::BOTTOM]) bottomW += p.blocks[id].baseW;
        for (int id : p.boundaryIndices[Side::TOP]) topW += p.blocks[id].baseW;
        for (int id : p.boundaryIndices[Side::LEFT]) leftH += p.blocks[id].baseH;
        for (int id : p.boundaryIndices[Side::RIGHT]) rightH += p.blocks[id].baseH;
        p.boundaryMinW = max(bottomW, topW);
        p.boundaryMinH = max(leftH, rightH);
        return p;
    }

    struct State {
        int root = -1;
        vector<int> parent;
        vector<int> left;
        vector<int> right;
        vector<int> moduleAt;
        vector<char> rotated;
        vector<double> softAr;
    };

    struct Placement {
        vector<Rect> rects;
        map<int, Rect> nodeRects;
        double W = 0.0;
        double H = 0.0;
        double decodedW = 0.0;
        double decodedH = 0.0;
        int compactRounds = 0;
        int horizontalMoves = 0;
        int verticalMoves = 0;
    };

    struct ChannelReserve {
        string name;
        int source = -1;
        int target = -1;
        double demand = 0.0;
        char orientation = 'H';
        Rect rect;
        int routeIndex = 0;
        int segmentIndex = 0;
    };

    static pair<double, double> blockShape(const Problem& p, const State& st, int blockId) {
        const PBlock& b = p.blocks[blockId];
        if (b.isSoft()) {
            const double ratio = clampD(st.softAr[blockId], b.arMin, b.arMax);
            const double w = sqrt(max(b.area, FP_EPS) * ratio);
            return { w, b.area / max(w, FP_EPS) };
        }
        double w = b.baseW, h = b.baseH;
        if (st.rotated[blockId]) swap(w, h);
        return { w, h };
    }

    static pair<double, double> portPoint(const PBlock& b, const Rect& r) {
        switch (b.portEdge) {
        case 1: return { r.x, r.y + 0.5 * r.h };
        case 2: return { r.x + 0.5 * r.w, topOf(r) };
        case 3: return { rightOf(r), r.y + 0.5 * r.h };
        case 4: return { r.x + 0.5 * r.w, r.y };
        default: return centerOf(r);
        }
    }

    static double weightedPortHpwl(const Problem& p, const Placement& placement) {
        double total = 0.0;
        for (const PConnection& c : p.connections) {
            const auto s = portPoint(p.blocks[c.source], placement.rects[c.source]);
            const auto t = portPoint(p.blocks[c.target], placement.rects[c.target]);
            total += c.weight * (fabs(s.first - t.first) + fabs(s.second - t.second));
        }
        return total;
    }

    static double weightedCenterHpwl(const Problem& p, const Placement& placement) {
        double total = 0.0;
        for (const PConnection& c : p.connections) {
            const auto s = centerOf(placement.rects[c.source]);
            const auto t = centerOf(placement.rects[c.target]);
            total += c.weight * (fabs(s.first - t.first) + fabs(s.second - t.second));
        }
        return total;
    }

    class HorizontalContour {
    public:
        double query(double x1, double x2) const {
            double result = 0.0;
            for (const auto& s : segments_) {
                if (get<0>(s) < x2 - FP_EPS && get<1>(s) > x1 + FP_EPS) {
                    result = max(result, get<2>(s));
                }
            }
            return result;
        }

        void update(double x1, double x2, double h) {
            vector<tuple<double, double, double>> next;
            for (const auto& s : segments_) {
                const double l = get<0>(s), r = get<1>(s), oldH = get<2>(s);
                if (r <= x1 + FP_EPS || l >= x2 - FP_EPS) {
                    next.push_back(s);
                    continue;
                }
                if (l < x1 - FP_EPS) next.emplace_back(l, x1, oldH);
                if (r > x2 + FP_EPS) next.emplace_back(x2, r, oldH);
            }
            next.emplace_back(x1, x2, h);
            sort(next.begin(), next.end(), [](const auto& a, const auto& b) {
                return get<0>(a) < get<0>(b);
                });
            vector<tuple<double, double, double>> merged;
            for (const auto& s : next) {
                if (!merged.empty() && fabs(get<1>(merged.back()) - get<0>(s)) <= FP_EPS &&
                    fabs(get<2>(merged.back()) - get<2>(s)) <= FP_EPS) {
                    get<1>(merged.back()) = get<1>(s);
                }
                else {
                    merged.push_back(s);
                }
            }
            segments_.swap(merged);
        }

    private:
        vector<tuple<double, double, double>> segments_;
    };

    static Placement contourPack(const Problem& p, const State& st) {
        if (st.root < 0) throw runtime_error("Cannot pack an empty B*-tree");
        Placement out;
        out.rects.resize(p.blocks.size());
        HorizontalContour contour;
        vector<int> stack{ st.root };
        vector<char> visited(st.moduleAt.size(), 0);
        int visitedCount = 0;
        while (!stack.empty()) {
            const int node = stack.back();
            stack.pop_back();
            if (node < 0 || node >= static_cast<int>(st.moduleAt.size()) || visited[node]) {
                throw runtime_error("Invalid B*-tree: cycle detected");
            }
            visited[node] = 1;
            ++visitedCount;
            const int blockId = st.moduleAt[node];
            const auto [w, h] = blockShape(p, st, blockId);
            double x = 0.0;
            const int parent = st.parent[node];
            if (parent != -1) {
                const Rect& pr = out.nodeRects.at(parent);
                if (st.left[parent] == node) x = rightOf(pr);
                else if (st.right[parent] == node) x = pr.x;
                else throw runtime_error("Invalid B*-tree parent/child relation");
            }
            const double y = contour.query(x, x + w);
            Rect r;
            r.x = x; r.y = y; r.w = w; r.h = h;
            out.rects[blockId] = r;
            out.nodeRects[node] = r;
            contour.update(x, x + w, y + h);
            if (st.right[node] != -1) stack.push_back(st.right[node]);
            if (st.left[node] != -1) stack.push_back(st.left[node]);
        }
        if (visitedCount != static_cast<int>(st.moduleAt.size())) {
            throw runtime_error("Invalid B*-tree: disconnected node detected");
        }
        for (const Rect& r : out.rects) {
            out.W = max(out.W, rightOf(r));
            out.H = max(out.H, topOf(r));
        }
        out.W = max(out.W, FP_EPS);
        out.H = max(out.H, FP_EPS);
        out.decodedW = out.W;
        out.decodedH = out.H;
        return out;
    }

    static bool hasSide(const Problem& p, int id, Side s) {
        return p.sideSets[id].find(s) != p.sideSets[id].end();
    }

    static bool edgeLocationRegionsOk(const PBlock& block, const Rect& r,
        double W, double H, bool includeCornerSide = true) {
        if (!block.isEdge() || block.tokens.empty()) return true;
        vector<Side> sides = includeCornerSide ? block.boundarySides() : vector<Side>{ block.boundarySide() };
        for (Side side : sides) {
            if (side == Side::NONE) continue;
            vector<int> segments;
            for (const string& token : block.tokens) {
                auto info = locationRegion(token);
                if (info && info->side == side) segments.push_back(info->segment);
            }
            const double scale = (side == Side::BOTTOM || side == Side::TOP) ? W : H;
            const double low = (side == Side::BOTTOM || side == Side::TOP) ? r.x : r.y;
            const double high = (side == Side::BOTTOM || side == Side::TOP) ? rightOf(r) : topOf(r);
            sort(segments.begin(), segments.end());
            segments.erase(unique(segments.begin(), segments.end()), segments.end());
            bool ok = false;
            for (int segment : segments) {
                const double regionLow = segment * scale / 3.0;
                const double regionHigh = (segment + 1) * scale / 3.0;
                if (min(high, regionHigh) > max(low, regionLow) + 1.0e-7) {
                    ok = true;
                    break;
                }
            }
            if (!ok) return false;
        }
        return true;
    }

    static optional<pair<double, double>> locationFractionRun(
        const PBlock& block, char axis, const Rect& r, double extent) {
        const bool horizontalAxis = axis == 'H';
        vector<int> segments;
        for (const string& token : block.tokens) {
            auto info = locationRegion(token);
            if (!info) continue;
            const bool relevant = horizontalAxis
                ? (info->side == Side::BOTTOM || info->side == Side::TOP)
                : (info->side == Side::LEFT || info->side == Side::RIGHT);
            if (relevant && find(segments.begin(), segments.end(), info->segment) == segments.end()) {
                segments.push_back(info->segment);
            }
        }
        if (segments.empty()) return nullopt;
        sort(segments.begin(), segments.end());
        vector<pair<int, int>> runs;
        int start = segments.front(), prev = segments.front();
        for (size_t i = 1; i < segments.size(); ++i) {
            if (segments[i] == prev + 1) {
                prev = segments[i];
            }
            else {
                runs.emplace_back(start, prev);
                start = prev = segments[i];
            }
        }
        runs.emplace_back(start, prev);
        const double projectionLow = horizontalAxis ? r.x : r.y;
        const double projectionHigh = horizontalAxis ? rightOf(r) : topOf(r);
        auto score = [&](const pair<int, int>& run) {
            const double lo = run.first * extent / 3.0;
            const double hi = (run.second + 1) * extent / 3.0;
            const double overlap = max(0.0, min(projectionHigh, hi) - max(projectionLow, lo));
            const double distance = max({ lo - projectionHigh, projectionLow - hi, 0.0 });
            return make_pair(overlap, -distance);
            };
        auto selected = *max_element(runs.begin(), runs.end(), [&](const auto& a, const auto& b) {
            return score(a) < score(b);
            });
        return pair<double, double>{selected.first / 3.0, (selected.second + 1) / 3.0};
    }

    static pair<vector<double>, double> solveSparseBoundaryAxis(
        const Problem& p, const vector<Rect>& rects, char axis,
        double currentExtent, bool towardLow) {
        const int n = static_cast<int>(rects.size());
        vector<double> physical(n), sizes(n), projLow(n), projHigh(n);
        Side physicalLow, physicalHigh;
        if (axis == 'H') {
            physicalLow = Side::LEFT; physicalHigh = Side::RIGHT;
            for (int i = 0; i < n; ++i) {
                physical[i] = rects[i].x; sizes[i] = rects[i].w;
                projLow[i] = rects[i].y; projHigh[i] = topOf(rects[i]);
            }
        }
        else {
            physicalLow = Side::BOTTOM; physicalHigh = Side::TOP;
            for (int i = 0; i < n; ++i) {
                physical[i] = rects[i].y; sizes[i] = rects[i].h;
                projLow[i] = rects[i].x; projHigh[i] = rightOf(rects[i]);
            }
        }
        vector<double> coord = physical;
        if (!towardLow) {
            for (int i = 0; i < n; ++i) coord[i] = currentExtent - physical[i] - sizes[i];
        }
        const Side lowSide = towardLow ? physicalLow : physicalHigh;
        const Side highSide = towardLow ? physicalHigh : physicalLow;

        vector<pair<int, int>> edges;
        for (int a = 0; a < n; ++a) {
            for (int b = a + 1; b < n; ++b) {
                if (min(projHigh[a], projHigh[b]) <= max(projLow[a], projLow[b]) + FP_EPS) continue;
                if (coord[a] + sizes[a] <= coord[b] + FP_EPS) edges.emplace_back(a, b);
                else if (coord[b] + sizes[b] <= coord[a] + FP_EPS) edges.emplace_back(b, a);
                else throw runtime_error("Exact boundary placement overlaps before sparse compaction");
            }
        }

        vector<vector<int>> adj(n);
        vector<int> indegree(n, 0);
        for (auto [u, v] : edges) { adj[u].push_back(v); ++indegree[v]; }
        priority_queue<int, vector<int>, greater<int>> q;
        for (int i = 0; i < n; ++i) if (indegree[i] == 0) q.push(i);
        vector<int> order;
        while (!q.empty()) {
            int u = q.top(); q.pop(); order.push_back(u);
            for (int v : adj[u]) if (--indegree[v] == 0) q.push(v);
        }
        if (static_cast<int>(order.size()) != n) throw runtime_error("Sparse boundary graph contains a cycle");

        const vector<int>& lowAnchors = p.boundaryIndices.count(lowSide) ? p.boundaryIndices.at(lowSide) : vector<int>{};
        const vector<int>& highAnchors = p.boundaryIndices.count(highSide) ? p.boundaryIndices.at(highSide) : vector<int>{};
        set<int> incoming;
        for (auto [u, v] : edges) incoming.insert(v);
        for (int id : lowAnchors) if (incoming.count(id)) throw runtime_error("Relation before low-side EDGE");
        for (int id : highAnchors) if (!adj[id].empty()) throw runtime_error("Relation after high-side EDGE");

        vector<optional<pair<double, double>>> fractions(n);
        for (int i = 0; i < n; ++i) {
            fractions[i] = locationFractionRun(p.blocks[i], axis, rects[i], max(currentExtent, FP_EPS));
            if (fractions[i] && !towardLow) {
                fractions[i] = pair<double, double>{ 1.0 - fractions[i]->second, 1.0 - fractions[i]->first };
            }
        }
        set<int> lowSet(lowAnchors.begin(), lowAnchors.end());
        vector<double> pos(n, 0.0);
        for (int u : order) for (int v : adj[u]) pos[v] = max(pos[v], pos[u] + sizes[u]);
        double extent = FP_EPS;
        for (int i = 0; i < n; ++i) extent = max(extent, pos[i] + sizes[i]);

        bool converged = false;
        for (int iter = 0; iter < max(64, 8 * n); ++iter) {
            fill(pos.begin(), pos.end(), 0.0);
            const double margin = max(1.0e-5, extent * 1.0e-8);
            for (int i = 0; i < n; ++i) {
                if (!fractions[i] || lowSet.count(i)) continue;
                pos[i] = max(0.0, fractions[i]->first * extent - sizes[i] + margin);
            }
            for (int u : order) for (int v : adj[u]) pos[v] = max(pos[v], pos[u] + sizes[u]);
            double newExtent = FP_EPS;
            for (int i = 0; i < n; ++i) newExtent = max(newExtent, pos[i] + sizes[i]);
            for (int i = 0; i < n; ++i) {
                if (!fractions[i]) continue;
                const double highF = fractions[i]->second;
                if (highF < 1.0 - FP_EPS) newExtent = max(newExtent, (pos[i] + margin) / highF);
            }
            if (fabs(newExtent - extent) <= 1.0e-12 * max(extent, 1.0)) {
                extent = newExtent;
                converged = true;
                break;
            }
            extent = newExtent;
        }
        if (!converged) throw runtime_error("Sparse LOCATION fixed point did not converge");
        for (int id : highAnchors) pos[id] = extent - sizes[id];
        for (auto [u, v] : edges) {
            if (pos[v] + 1.0e-7 < pos[u] + sizes[u]) throw runtime_error("Anchored separation failed");
        }

        const double tolerance = 1.0e-7 * max(currentExtent, 1.0);
        if (extent > currentExtent + tolerance) return { physical, currentExtent };
        for (int i = 0; i < n; ++i) if (pos[i] > coord[i] + tolerance) return { physical, currentExtent };
        if (towardLow) return { pos, max(extent, FP_EPS) };
        vector<double> reflected(n);
        for (int i = 0; i < n; ++i) reflected[i] = extent - pos[i] - sizes[i];
        return { reflected, max(extent, FP_EPS) };
    }

    static Placement iterativeDirectionalCompact(const Problem& p, const State& st,
        const Placement& base,
        bool horizontalTowardLow,
        bool verticalTowardLow) {
        Placement result = base;
        vector<Rect> rects = base.rects;
        double W = max(base.W, FP_EPS), H = max(base.H, FP_EPS);
        const int n = static_cast<int>(rects.size());
        const int maxRounds = max(4, n * n + 1);
        int hMoves = 0, vMoves = 0, rounds = 0;
        for (int round = 1; round <= maxRounds; ++round) {
            vector<double> oldX(n), oldY(n);
            for (int i = 0; i < n; ++i) { oldX[i] = rects[i].x; oldY[i] = rects[i].y; }
            const double oldW = W, oldH = H;
            auto xSolved = solveSparseBoundaryAxis(p, rects, 'H', W, horizontalTowardLow);
            for (int i = 0; i < n; ++i) {
                if (fabs(xSolved.first[i] - rects[i].x) > FP_EPS) ++hMoves;
                rects[i].x = xSolved.first[i];
            }
            W = xSolved.second;
            auto ySolved = solveSparseBoundaryAxis(p, rects, 'V', H, verticalTowardLow);
            for (int i = 0; i < n; ++i) {
                if (fabs(ySolved.first[i] - rects[i].y) > FP_EPS) ++vMoves;
                rects[i].y = ySolved.first[i];
            }
            H = ySolved.second;
            rounds = round;
            double maxChange = max(fabs(W - oldW), fabs(H - oldH));
            for (int i = 0; i < n; ++i) {
                maxChange = max(maxChange, fabs(rects[i].x - oldX[i]));
                maxChange = max(maxChange, fabs(rects[i].y - oldY[i]));
            }
            const double scale = max({ W, H, oldW, oldH, 1.0 });
            const double areaChange = fabs(W * H - oldW * oldH);
            if (maxChange <= 1.0e-9 * scale &&
                areaChange <= 1.0e-9 * max({ W * H, oldW * oldH, 1.0 })) break;
            if (round == maxRounds) throw runtime_error("Final constrained compaction did not converge");
        }

        for (int a = 0; a < n; ++a) for (int b = a + 1; b < n; ++b) {
            if (overlapArea(rects[a], rects[b]) > 1.0e-5) throw runtime_error("Compaction created overlap");
        }
        for (int i = 0; i < n; ++i) {
            const Rect& r = rects[i];
            if (hasSide(p, i, Side::LEFT) && fabs(r.x) > 1.0e-7) throw runtime_error("LEFT EDGE lost boundary");
            if (hasSide(p, i, Side::RIGHT) && fabs(rightOf(r) - W) > 1.0e-7) throw runtime_error("RIGHT EDGE lost boundary");
            if (hasSide(p, i, Side::BOTTOM) && fabs(r.y) > 1.0e-7) throw runtime_error("BOTTOM EDGE lost boundary");
            if (hasSide(p, i, Side::TOP) && fabs(topOf(r) - H) > 1.0e-7) throw runtime_error("TOP EDGE lost boundary");
            if (!edgeLocationRegionsOk(p.blocks[i], r, W, H, true)) throw runtime_error("EDGE left LOCATION interval");
        }
        result.rects = std::move(rects);
        result.W = result.decodedW = W;
        result.H = result.decodedH = H;
        result.compactRounds = rounds;
        result.horizontalMoves = hMoves;
        result.verticalMoves = vMoves;
        result.nodeRects.clear();
        for (int node = 0; node < static_cast<int>(st.moduleAt.size()); ++node) {
            result.nodeRects[node] = result.rects[st.moduleAt[node]];
        }
        return result;
    }

    static Placement boundaryConstraintCompact(const Problem& p, const State& st, const Placement& base) {
        const int n = static_cast<int>(p.blocks.size());
        vector<pair<int, int>> horizontal, vertical;

        auto candidateRelation = [&](int a, int b, char axis) -> optional<pair<int, int>> {
            Side lowSide, highSide;
            double ca, cb;
            if (axis == 'H') {
                lowSide = Side::LEFT; highSide = Side::RIGHT;
                ca = centerOf(base.rects[a]).first; cb = centerOf(base.rects[b]).first;
            }
            else {
                lowSide = Side::BOTTOM; highSide = Side::TOP;
                ca = centerOf(base.rects[a]).second; cb = centerOf(base.rects[b]).second;
            }
            auto anchorSide = [&](int id) {
                if (hasSide(p, id, lowSide)) return lowSide;
                if (hasSide(p, id, highSide)) return highSide;
                return Side::NONE;
                };
            const Side sa = anchorSide(a), sb = anchorSide(b);
            if (sa == sb && (sa == lowSide || sa == highSide)) return nullopt;
            auto key = [&](int id, Side s, double c) {
                int rank = s == lowSide ? -1 : (s == highSide ? 1 : 0);
                return tuple<int, double, int>{rank, c, id};
                };
            int source = a, target = b;
            if (key(b, sb, cb) < key(a, sa, ca)) { source = b; target = a; }
            const Side sourceSide = source == a ? sa : sb;
            const Side targetSide = target == a ? sa : sb;
            if (sourceSide == highSide || targetSide == lowSide) return nullopt;
            return pair<int, int>{source, target};
            };

        for (int a = 0; a < n; ++a) {
            for (int b = a + 1; b < n; ++b) {
                optional<tuple<int, int, double>> hBase, vBase;
                if (rightOf(base.rects[a]) <= base.rects[b].x + FP_EPS)
                    hBase = tuple<int, int, double>{ a, b, max(0.0, base.rects[b].x - rightOf(base.rects[a])) };
                else if (rightOf(base.rects[b]) <= base.rects[a].x + FP_EPS)
                    hBase = tuple<int, int, double>{ b, a, max(0.0, base.rects[a].x - rightOf(base.rects[b])) };
                if (topOf(base.rects[a]) <= base.rects[b].y + FP_EPS)
                    vBase = tuple<int, int, double>{ a, b, max(0.0, base.rects[b].y - topOf(base.rects[a])) };
                else if (topOf(base.rects[b]) <= base.rects[a].y + FP_EPS)
                    vBase = tuple<int, int, double>{ b, a, max(0.0, base.rects[a].y - topOf(base.rects[b])) };
                if (!hBase && !vBase) throw runtime_error("Contour placement overlaps before boundary compaction");

                vector<tuple<double, char, pair<int, int>>> choices;
                auto hc = candidateRelation(a, b, 'H');
                auto vc = candidateRelation(a, b, 'V');
                if (hc) {
                    double score = 1.0;
                    if (hBase && hc->first == get<0>(*hBase) && hc->second == get<1>(*hBase))
                        score = get<2>(*hBase) / max(base.W, 1.0);
                    choices.emplace_back(score, 'H', *hc);
                }
                if (vc) {
                    double score = 1.0;
                    if (vBase && vc->first == get<0>(*vBase) && vc->second == get<1>(*vBase))
                        score = get<2>(*vBase) / max(base.H, 1.0);
                    choices.emplace_back(score, 'V', *vc);
                }
                if (choices.empty()) throw runtime_error("No boundary-compatible separation direction");
                const auto chosen = *min_element(choices.begin(), choices.end());
                if (get<1>(chosen) == 'H') horizontal.push_back(get<2>(chosen));
                else vertical.push_back(get<2>(chosen));
            }
        }

        auto solveAxis = [&](char axis, const vector<pair<int, int>>& edges,
            const vector<double>& sizes,
            const vector<int>& lowAnchors,
            const vector<int>& highAnchors) {
                vector<vector<int>> adj(n);
                vector<int> indegree(n, 0);
                for (auto [u, v] : edges) { adj[u].push_back(v); ++indegree[v]; }
                priority_queue<int, vector<int>, greater<int>> q;
                for (int i = 0; i < n; ++i) if (indegree[i] == 0) q.push(i);
                vector<int> order;
                while (!q.empty()) {
                    int u = q.top(); q.pop(); order.push_back(u);
                    for (int v : adj[u]) if (--indegree[v] == 0) q.push(v);
                }
                if (static_cast<int>(order.size()) != n) throw runtime_error("Boundary separation graph cycle");
                set<int> incoming;
                for (auto [u, v] : edges) incoming.insert(v);
                for (int id : lowAnchors) if (incoming.count(id)) throw runtime_error("Block before low-side EDGE");
                for (int id : highAnchors) if (!adj[id].empty()) throw runtime_error("Block after high-side EDGE");

                vector<optional<pair<double, double>>> fractions(n);
                for (int i = 0; i < n; ++i) {
                    vector<int> segments;
                    for (const string& token : p.blocks[i].tokens) {
                        auto info = locationRegion(token);
                        if (!info) continue;
                        bool relevant = axis == 'H'
                            ? (info->side == Side::BOTTOM || info->side == Side::TOP)
                            : (info->side == Side::LEFT || info->side == Side::RIGHT);
                        if (relevant) segments.push_back(info->segment);
                    }
                    if (!segments.empty()) {
                        fractions[i] = pair<double, double>{ *min_element(segments.begin(), segments.end()) / 3.0,
                            (*max_element(segments.begin(), segments.end()) + 1) / 3.0 };
                    }
                }
                vector<double> pos(n, 0.0);
                for (int u : order) for (int v : adj[u]) pos[v] = max(pos[v], pos[u] + sizes[u]);
                double extent = FP_EPS;
                for (int i = 0; i < n; ++i) extent = max(extent, pos[i] + sizes[i]);
                set<int> lowSet(lowAnchors.begin(), lowAnchors.end());
                bool converged = false;
                for (int iter = 0; iter < max(64, 8 * n); ++iter) {
                    fill(pos.begin(), pos.end(), 0.0);
                    const double margin = max(1.0e-5, extent * 1.0e-8);
                    for (int i = 0; i < n; ++i) {
                        if (!fractions[i] || lowSet.count(i)) continue;
                        pos[i] = max(0.0, fractions[i]->first * extent - sizes[i] + margin);
                    }
                    for (int u : order) for (int v : adj[u]) pos[v] = max(pos[v], pos[u] + sizes[u]);
                    double newExtent = FP_EPS;
                    for (int i = 0; i < n; ++i) newExtent = max(newExtent, pos[i] + sizes[i]);
                    for (int i = 0; i < n; ++i) if (fractions[i] && fractions[i]->second < 1.0 - FP_EPS)
                        newExtent = max(newExtent, (pos[i] + margin) / fractions[i]->second);
                    if (fabs(newExtent - extent) <= 1.0e-12 * max(extent, 1.0)) {
                        extent = newExtent; converged = true; break;
                    }
                    extent = newExtent;
                }
                if (!converged) throw runtime_error("Fractional LOCATION bounds did not converge");
                for (int id : highAnchors) pos[id] = extent - sizes[id];
                for (auto [u, v] : edges) if (pos[v] + 1.0e-7 < pos[u] + sizes[u])
                    throw runtime_error("Anchored separation failed");
                return pair<vector<double>, double>{pos, max(extent, FP_EPS)};
            };

        vector<double> widths(n), heights(n);
        for (int i = 0; i < n; ++i) { widths[i] = base.rects[i].w; heights[i] = base.rects[i].h; }
        const vector<int> leftAnchors = p.boundaryIndices.count(Side::LEFT) ? p.boundaryIndices.at(Side::LEFT) : vector<int>{};
        const vector<int> rightAnchors = p.boundaryIndices.count(Side::RIGHT) ? p.boundaryIndices.at(Side::RIGHT) : vector<int>{};
        const vector<int> bottomAnchors = p.boundaryIndices.count(Side::BOTTOM) ? p.boundaryIndices.at(Side::BOTTOM) : vector<int>{};
        const vector<int> topAnchors = p.boundaryIndices.count(Side::TOP) ? p.boundaryIndices.at(Side::TOP) : vector<int>{};
        auto xs = solveAxis('H', horizontal, widths, leftAnchors, rightAnchors);
        auto ys = solveAxis('V', vertical, heights, bottomAnchors, topAnchors);

        Placement anchored = base;
        anchored.W = anchored.decodedW = xs.second;
        anchored.H = anchored.decodedH = ys.second;
        for (int i = 0; i < n; ++i) {
            anchored.rects[i].x = xs.first[i];
            anchored.rects[i].y = ys.first[i];
        }
        for (int i = 0; i < n; ++i) {
            const Rect& r = anchored.rects[i];
            if (hasSide(p, i, Side::LEFT) && fabs(r.x) > 1.0e-7) throw runtime_error("LEFT legalization failed");
            if (hasSide(p, i, Side::RIGHT) && fabs(rightOf(r) - anchored.W) > 1.0e-7) throw runtime_error("RIGHT legalization failed");
            if (hasSide(p, i, Side::BOTTOM) && fabs(r.y) > 1.0e-7) throw runtime_error("BOTTOM legalization failed");
            if (hasSide(p, i, Side::TOP) && fabs(topOf(r) - anchored.H) > 1.0e-7) throw runtime_error("TOP legalization failed");
            if (!edgeLocationRegionsOk(p.blocks[i], r, anchored.W, anchored.H, true))
                throw runtime_error("Final LOCATION legalization failed");
        }
        anchored.nodeRects.clear();
        for (int node = 0; node < static_cast<int>(st.moduleAt.size()); ++node)
            anchored.nodeRects[node] = anchored.rects[st.moduleAt[node]];
        return iterativeDirectionalCompact(p, st, anchored, true, true);
    }

    static Placement exactPack(const Problem& p, const State& st) {
        return boundaryConstraintCompact(p, st, contourPack(p, st));
    }

    static void replaceParentLink(State& st, int node, int replacement) {
        const int parent = st.parent[node];
        if (parent == -1) st.root = replacement;
        else if (st.left[parent] == node) st.left[parent] = replacement;
        else if (st.right[parent] == node) st.right[parent] = replacement;
        else throw runtime_error("Invalid parent link while deleting B*-tree node");
        if (replacement != -1) st.parent[replacement] = parent;
    }

    static void deleteBtreeNode(State& st, int node, mt19937* rng = nullptr) {
        const int lc = st.left[node], rc = st.right[node];
        vector<int> children;
        if (lc != -1) children.push_back(lc);
        if (rc != -1) children.push_back(rc);
        if (children.empty()) {
            replaceParentLink(st, node, -1);
            st.parent[node] = st.left[node] = st.right[node] = -1;
            return;
        }
        bool pullLeft;
        if (children.size() == 1) pullLeft = lc != -1;
        else pullLeft = rng ? uniform_int_distribution<int>(0, 1)(*rng) == 0 : true;
        const int pulled = pullLeft ? lc : rc;
        const int other = pullLeft ? rc : lc;
        const int displaced = other != -1 ? (pullLeft ? st.right[pulled] : st.left[pulled]) : -1;
        replaceParentLink(st, node, pulled);
        if (other != -1) {
            if (pullLeft) st.right[pulled] = other;
            else st.left[pulled] = other;
            st.parent[other] = pulled;
        }
        if (displaced != -1) {
            int cursor = other;
            while (st.left[cursor] != -1 && st.right[cursor] != -1) {
                if (rng) cursor = uniform_int_distribution<int>(0, 1)(*rng) == 0 ? st.left[cursor] : st.right[cursor];
                else cursor = st.left[cursor];
            }
            vector<char> empty;
            if (st.left[cursor] == -1) empty.push_back('L');
            if (st.right[cursor] == -1) empty.push_back('R');
            const char side = rng ? empty[uniform_int_distribution<int>(0, static_cast<int>(empty.size()) - 1)(*rng)] : empty.front();
            if (side == 'L') st.left[cursor] = displaced;
            else st.right[cursor] = displaced;
            st.parent[displaced] = cursor;
        }
        st.parent[node] = st.left[node] = st.right[node] = -1;
    }

    static void insertIsolatedAtLink(State& st, int node, int parent, char side) {
        if (st.parent[node] != -1 || st.left[node] != -1 || st.right[node] != -1)
            throw runtime_error("Only an isolated node can be inserted");
        int oldChild = -1;
        if (side == 'L') {
            oldChild = st.left[parent];
            st.left[parent] = node;
            st.left[node] = oldChild;
        }
        else {
            oldChild = st.right[parent];
            st.right[parent] = node;
            st.right[node] = oldChild;
        }
        st.parent[node] = parent;
        if (oldChild != -1) st.parent[oldChild] = node;
    }

    static void insertIsolatedAsRoot(State& st, int node, char oldRootSide) {
        const int oldRoot = st.root;
        st.root = node;
        st.parent[node] = -1;
        if (oldRootSide == 'L') st.left[node] = oldRoot;
        else st.right[node] = oldRoot;
        if (oldRoot != -1) st.parent[oldRoot] = node;
    }

    static vector<int> leftmostBranch(const State& st) {
        vector<int> branch;
        for (int node = st.root; node != -1; node = st.left[node]) branch.push_back(node);
        return branch;
    }

    static vector<int> rightmostBranch(const State& st) {
        vector<int> branch;
        for (int node = st.root; node != -1; node = st.right[node]) branch.push_back(node);
        return branch;
    }

    static vector<int> bottomLeftBranch(const State& st) {
        vector<int> base = leftmostBranch(st), branch;
        if (base.empty()) return branch;
        int node = base.back();
        branch.push_back(node);
        for (node = st.right[node]; node != -1; node = st.right[node]) branch.push_back(node);
        return branch;
    }

    static vector<int> bottomRightBranch(const State& st) {
        vector<int> base = rightmostBranch(st), branch;
        if (base.empty()) return branch;
        int node = base.back();
        branch.push_back(node);
        for (node = st.left[node]; node != -1; node = st.left[node]) branch.push_back(node);
        return branch;
    }

    static vector<int> boundaryBranch(const State& st, Side side) {
        if (side == Side::BOTTOM) return leftmostBranch(st);
        if (side == Side::LEFT) return rightmostBranch(st);
        if (side == Side::RIGHT) return bottomLeftBranch(st);
        if (side == Side::TOP) return bottomRightBranch(st);
        throw runtime_error("Unknown boundary side");
    }

    static vector<int> primaryBoundaryModules(const Problem& p, Side side) {
        vector<int> out;
        for (const PBlock& b : p.blocks) if (b.isEdge() && b.boundarySide() == side) out.push_back(b.index);
        return out;
    }

    static int boundaryBranchSegment(int index, int length) {
        if (length <= 1) return 0;
        const double position = static_cast<double>(index) / (length - 1);
        if (position < 1.0 / 3.0 - FP_EPS) return 0;
        if (position <= 2.0 / 3.0 + FP_EPS) return 1;
        return 2;
    }

    static vector<int> moduleToNodeMap(const State& st) {
        vector<int> map(st.moduleAt.size(), -1);
        for (int node = 0; node < static_cast<int>(st.moduleAt.size()); ++node) {
            const int module = st.moduleAt[node];
            if (module < 0 || module >= static_cast<int>(map.size()) || map[module] != -1)
                throw runtime_error("Invalid or duplicate module in B*-tree");
            map[module] = node;
        }
        if (find(map.begin(), map.end(), -1) != map.end()) throw runtime_error("Missing module in B*-tree");
        return map;
    }

    static vector<string> boundaryTreeViolations(const Problem& p, const State& st) {
        const vector<int> nodeFor = moduleToNodeMap(st);
        vector<string> violations;
        for (Side side : {Side::BOTTOM, Side::LEFT, Side::RIGHT, Side::TOP}) {
            const vector<int> modules = primaryBoundaryModules(p, side);
            if (modules.empty()) continue;
            const vector<int> branch = boundaryBranch(st, side);
            map<int, int> position;
            for (int i = 0; i < static_cast<int>(branch.size()); ++i) position[branch[i]] = i;
            for (int module : modules) {
                const int node = nodeFor[module];
                if (!position.count(node)) {
                    violations.push_back(p.blocks[module].name + ":not-" + sideName(side) + "-branch");
                    continue;
                }
                const vector<int> allowed = p.blocks[module].annealSegments();
                const int segment = boundaryBranchSegment(position[node], static_cast<int>(branch.size()));
                if (!allowed.empty() && find(allowed.begin(), allowed.end(), segment) == allowed.end())
                    violations.push_back(p.blocks[module].name + ":wrong-segment");
            }
            if (side == Side::RIGHT) {
                for (int node : branch) if (st.left[node] != -1) violations.push_back("node:" + to_string(node) + ":left-child");
            }
            else if (side == Side::TOP) {
                for (int node : branch) if (st.right[node] != -1) violations.push_back("node:" + to_string(node) + ":right-child");
            }
        }
        return violations;
    }

    static vector<pair<int, char>> offPathExternalSlots(const Problem& p, const State& st, int excluded = -1) {
        const vector<int> left = leftmostBranch(st), right = rightmostBranch(st);
        const vector<int> bottomLeft = primaryBoundaryModules(p, Side::RIGHT).empty() ? vector<int>{} : bottomLeftBranch(st);
        const vector<int> bottomRight = primaryBoundaryModules(p, Side::TOP).empty() ? vector<int>{} : bottomRightBranch(st);
        set<int> critical(left.begin(), left.end());
        critical.insert(right.begin(), right.end());
        critical.insert(bottomLeft.begin(), bottomLeft.end());
        critical.insert(bottomRight.begin(), bottomRight.end());
        vector<pair<int, char>> slots;
        for (size_t i = 0; i + 1 < left.size(); ++i) {
            int node = left[i];
            if (node != excluded && st.right[node] == -1 && find(bottomRight.begin(), bottomRight.end(), node) == bottomRight.end())
                slots.emplace_back(node, 'R');
        }
        for (size_t i = 0; i + 1 < right.size(); ++i) {
            int node = right[i];
            if (node != excluded && st.left[node] == -1 && find(bottomLeft.begin(), bottomLeft.end(), node) == bottomLeft.end())
                slots.emplace_back(node, 'L');
        }
        for (int node = 0; node < static_cast<int>(st.moduleAt.size()); ++node) {
            if (node == excluded || critical.count(node)) continue;
            if (st.parent[node] == -1 && node != st.root) continue;
            if (st.left[node] == -1) slots.emplace_back(node, 'L');
            if (st.right[node] == -1) slots.emplace_back(node, 'R');
        }
        return slots;
    }

    static int randomChoice(const vector<int>& v, mt19937& rng) {
        return v[uniform_int_distribution<int>(0, static_cast<int>(v.size()) - 1)(rng)];
    }

    template <class T>
    static T randomChoiceT(const vector<T>& v, mt19937& rng) {
        return v[uniform_int_distribution<int>(0, static_cast<int>(v.size()) - 1)(rng)];
    }

    static int rebuildBoundaryFeasibleTree(const Problem& p, State& st, mt19937& rng) {
        const vector<int> nodeFor = moduleToNodeMap(st);
        map<Side, vector<int>> bySide;
        for (Side side : {Side::BOTTOM, Side::LEFT, Side::RIGHT, Side::TOP})
            for (int module : primaryBoundaryModules(p, side)) bySide[side].push_back(nodeFor[module]);
        set<int> boundaryNodes;
        for (auto& kv : bySide) boundaryNodes.insert(kv.second.begin(), kv.second.end());
        vector<int> freeNodes;
        for (int node = 0; node < static_cast<int>(st.moduleAt.size()); ++node)
            if (!boundaryNodes.count(node)) freeNodes.push_back(node);
        shuffle(freeNodes.begin(), freeNodes.end(), rng);

        auto allowedSegments = [&](int node) {
            vector<int> allowed = p.blocks[st.moduleAt[node]].annealSegments();
            if (allowed.empty()) allowed = { 0, 1, 2 };
            return allowed;
            };
        auto isExactlyFront = [&](int node) {
            const vector<int> a = allowedSegments(node);
            return a.size() == 1 && a.front() == 0;
            };

        vector<int> rootCandidates;
        for (Side side : {Side::BOTTOM, Side::LEFT}) for (int node : bySide[side])
            if (isExactlyFront(node)) rootCandidates.push_back(node);
        int root = -1;
        if (!rootCandidates.empty()) {
            root = randomChoice(rootCandidates, rng);
            Side rs = p.blocks[st.moduleAt[root]].boundarySide();
            auto& v = bySide[rs]; v.erase(find(v.begin(), v.end(), root));
        }
        else if (!freeNodes.empty()) {
            root = freeNodes.back(); freeNodes.pop_back();
        }
        else {
            vector<int> candidates;
            for (Side side : {Side::BOTTOM, Side::LEFT}) for (int node : bySide[side]) {
                vector<int> a = allowedSegments(node);
                if (find(a.begin(), a.end(), 0) != a.end()) candidates.push_back(node);
            }
            if (candidates.empty()) throw runtime_error("12-region B*-tree needs a front BOTTOM/LEFT root");
            root = randomChoice(candidates, rng);
            Side rs = p.blocks[st.moduleAt[root]].boundarySide();
            auto& v = bySide[rs]; v.erase(find(v.begin(), v.end(), root));
        }

        auto takeFrontAnchor = [&](Side side) -> optional<int> {
            vector<int> candidates;
            for (int node : bySide[side]) if (isExactlyFront(node)) candidates.push_back(node);
            if (candidates.empty()) return nullopt;
            int node = randomChoice(candidates, rng);
            auto& v = bySide[side]; v.erase(find(v.begin(), v.end(), node));
            return node;
            };
        optional<int> rightAnchor = takeFrontAnchor(Side::RIGHT);
        optional<int> topAnchor = takeFrontAnchor(Side::TOP);

        auto layoutTail = [&](Side side, int anchor, vector<int> constrained, optional<int> reservedTail) {
            const PBlock& anchorBlock = p.blocks[st.moduleAt[anchor]];
            if (anchorBlock.boundarySide() == side) {
                vector<int> a = anchorBlock.annealSegments();
                if (!a.empty() && find(a.begin(), a.end(), 0) == a.end())
                    throw runtime_error(anchorBlock.name + " cannot anchor boundary front");
            }
            shuffle(constrained.begin(), constrained.end(), rng);
            const int reserveCount = reservedTail ? 1 : 0;
            for (int fillerCount = 0; fillerCount <= static_cast<int>(freeNodes.size()); ++fillerCount) {
                const int length = 1 + static_cast<int>(constrained.size()) + fillerCount + reserveCount;
                const int lastAssignable = length - reserveCount;
                vector<int> slots;
                for (int slot = 1; slot < lastAssignable; ++slot) slots.push_back(slot);
                map<int, vector<int>> options;
                bool impossible = false;
                for (int node : constrained) {
                    const vector<int> allowed = allowedSegments(node);
                    for (int slot : slots) {
                        int seg = boundaryBranchSegment(slot, length);
                        if (find(allowed.begin(), allowed.end(), seg) != allowed.end()) options[node].push_back(slot);
                    }
                    if (options[node].empty()) impossible = true;
                }
                if (impossible) continue;
                map<int, int> slotOwner;
                function<bool(int, set<int>&)> assign = [&](int node, set<int>& seen) {
                    vector<int> nodeSlots = options[node];
                    shuffle(nodeSlots.begin(), nodeSlots.end(), rng);
                    for (int slot : nodeSlots) {
                        if (seen.count(slot)) continue;
                        seen.insert(slot);
                        auto it = slotOwner.find(slot);
                        if (it == slotOwner.end()) { slotOwner[slot] = node; return true; }
                        int old = it->second;
                        if (assign(old, seen)) { slotOwner[slot] = node; return true; }
                    }
                    return false;
                    };
                vector<int> ordered = constrained;
                sort(ordered.begin(), ordered.end(), [&](int a, int b) { return options[a].size() < options[b].size(); });
                bool matched = true;
                for (int node : ordered) { set<int> seen; if (!assign(node, seen)) { matched = false; break; } }
                if (!matched) continue;
                vector<int> tail;
                int fillerIndex = 0;
                for (int slot : slots) {
                    if (slotOwner.count(slot)) tail.push_back(slotOwner[slot]);
                    else {
                        if (fillerIndex >= fillerCount) { matched = false; break; }
                        tail.push_back(freeNodes[fillerIndex++]);
                    }
                }
                if (!matched) continue;
                freeNodes.erase(freeNodes.begin(), freeNodes.begin() + fillerCount);
                if (reservedTail) tail.push_back(*reservedTail);
                return tail;
            }
            throw runtime_error("Not enough filler nodes for 12-region path");
            };

        vector<int> bottomTail = layoutTail(Side::BOTTOM, root, bySide[Side::BOTTOM], rightAnchor);
        vector<int> leftTail = layoutTail(Side::LEFT, root, bySide[Side::LEFT], topAnchor);
        int bottomAnchorNode = bottomTail.empty() ? root : bottomTail.back();
        int leftAnchorNode = leftTail.empty() ? root : leftTail.back();
        vector<int> rightTailNodes = layoutTail(Side::RIGHT, bottomAnchorNode, bySide[Side::RIGHT], nullopt);
        vector<int> topTailNodes = layoutTail(Side::TOP, leftAnchorNode, bySide[Side::TOP], nullopt);

        int deleted = 0;
        while (st.root != -1) { deleteBtreeNode(st, st.root, &rng); ++deleted; }
        st.root = root; st.parent[root] = -1;
        auto link = [&](int parent, int child, char side) { insertIsolatedAtLink(st, child, parent, side); };
        int cursor = root;
        for (int node : bottomTail) { link(cursor, node, 'L'); cursor = node; }
        int bottomTailEnd = cursor;
        cursor = root;
        for (int node : leftTail) { link(cursor, node, 'R'); cursor = node; }
        int leftTailEnd = cursor;
        cursor = bottomTailEnd;
        for (int node : rightTailNodes) { link(cursor, node, 'R'); cursor = node; }
        cursor = leftTailEnd;
        for (int node : topTailNodes) { link(cursor, node, 'L'); cursor = node; }

        while (!freeNodes.empty()) {
            int node = freeNodes.back();
            freeNodes.pop_back();
            vector<pair<int, char>> candidateSlots = offPathExternalSlots(p, st, node);
            // If no purely off-path external link exists, try every internal or
            // external link and retain only insertions that preserve all four
            // boundary paths and their discrete thirds.
            for (int parent = 0; parent < static_cast<int>(st.moduleAt.size()); ++parent) {
                if (parent == node || (st.parent[parent] == -1 && parent != st.root)) continue;
                candidateSlots.emplace_back(parent, 'L');
                candidateSlots.emplace_back(parent, 'R');
            }
            shuffle(candidateSlots.begin(), candidateSlots.end(), rng);
            bool inserted = false;
            for (auto slot : candidateSlots) {
                State trial = st;
                try {
                    insertIsolatedAtLink(trial, node, slot.first, slot.second);
                    if (boundaryTreeViolations(p, trial).empty()) {
                        st = std::move(trial);
                        inserted = true;
                        break;
                    }
                }
                catch (const exception&) {}
            }
            if (!inserted) throw runtime_error("No boundary-safe slot for free B*-tree node");
        }
        vector<string> violations = boundaryTreeViolations(p, st);
        if (!violations.empty()) throw runtime_error("Strict boundary-frame rebuild failed");
        return deleted;
    }

    static int repairBoundaryConstraints(const Problem& p, State& st, mt19937& rng) {
        if (none_of(p.blocks.begin(), p.blocks.end(), [](const PBlock& b) { return b.isEdge(); })) return 0;
        if (boundaryTreeViolations(p, st).empty()) return 0;
        return rebuildBoundaryFeasibleTree(p, st, rng);
    }

    static State buildInitialState(const Problem& p, mt19937& rng) {
        const int n = static_cast<int>(p.blocks.size());
        State st;
        st.parent.assign(n, -1);
        st.left.assign(n, -1);
        st.right.assign(n, -1);
        st.moduleAt.resize(n);
        iota(st.moduleAt.begin(), st.moduleAt.end(), 0);
        sort(st.moduleAt.begin(), st.moduleAt.end(), [&](int a, int b) { return p.blocks[a].area > p.blocks[b].area; });
        st.rotated.assign(n, 0);
        st.softAr.assign(n, 1.0);
        for (const PBlock& b : p.blocks) if (b.isSoft()) st.softAr[b.index] = clampD(1.0, b.arMin, b.arMax);
        st.root = n ? 0 : -1;
        for (int node = 0; node < n; ++node) {
            int lc = 2 * node + 1, rc = 2 * node + 2;
            if (lc < n) { st.left[node] = lc; st.parent[lc] = node; }
            if (rc < n) { st.right[node] = rc; st.parent[rc] = node; }
        }
        repairBoundaryConstraints(p, st, rng);
        return st;
    }

    static bool moveSingleNode(State& st, mt19937& rng, const vector<int>* candidates = nullptr) {
        vector<int> movable;
        if (candidates) movable = *candidates;
        else { movable.resize(st.moduleAt.size()); iota(movable.begin(), movable.end(), 0); }
        if (movable.empty()) return false;
        const int node = randomChoice(movable, rng);
        const int oldParent = st.parent[node];
        const char oldSide = oldParent == -1 ? 'O' : (st.left[oldParent] == node ? 'L' : 'R');
        deleteBtreeNode(st, node, &rng);
        vector<pair<int, char>> slots;
        for (int parent = 0; parent < static_cast<int>(st.moduleAt.size()); ++parent) {
            if (parent == node) continue;
            slots.emplace_back(parent, 'L');
            slots.emplace_back(parent, 'R');
        }
        if (oldParent != -1 && slots.size() > 1) {
            slots.erase(remove(slots.begin(), slots.end(), pair<int, char>{oldParent, oldSide}), slots.end());
        }
        if (!slots.empty()) {
            auto slot = randomChoiceT(slots, rng);
            insertIsolatedAtLink(st, node, slot.first, slot.second);
        }
        else {
            insertIsolatedAsRoot(st, node, uniform_int_distribution<int>(0, 1)(rng) ? 'L' : 'R');
        }
        return true;
    }

    static string paperRandomMutation(const Problem& p, State& st, mt19937& rng, bool allowMacroRotate) {
        vector<pair<string, double>> ops;
        if (st.moduleAt.size() >= 2) ops.push_back({ "swap", 0.35 });
        if (!st.moduleAt.empty()) ops.push_back({ "move", 0.45 });
        vector<int> macros;
        for (const PBlock& b : p.blocks) if (b.isMacro()) macros.push_back(b.index);
        if (allowMacroRotate && !macros.empty()) ops.push_back({ "rotate", 0.08 });
        if (ops.empty()) return "none";
        double total = 0.0; for (auto& op : ops) total += op.second;
        double pick = uniform_real_distribution<double>(0.0, total)(rng);
        string selected = ops.back().first;
        for (auto& op : ops) { pick -= op.second; if (pick <= 0.0) { selected = op.first; break; } }
        if (selected == "swap") {
            uniform_int_distribution<int> dist(0, static_cast<int>(st.moduleAt.size()) - 1);
            int a = dist(rng), b = dist(rng); while (b == a) b = dist(rng);
            swap(st.moduleAt[a], st.moduleAt[b]);
        }
        else if (selected == "move") {
            moveSingleNode(st, rng);
        }
        else if (selected == "rotate") {
            int id = randomChoice(macros, rng);
            st.rotated[id] = !st.rotated[id];
        }
        return selected;
    }

    static vector<int> subtreeNodes(const State& st, int root) {
        if (root < 0 || root >= static_cast<int>(st.moduleAt.size())) return {};
        vector<int> nodes, stack{ root };
        set<int> seen;
        while (!stack.empty()) {
            int u = stack.back(); stack.pop_back();
            if (seen.count(u)) throw runtime_error("B*-tree cycle in subtree");
            seen.insert(u); nodes.push_back(u);
            if (st.right[u] != -1) stack.push_back(st.right[u]);
            if (st.left[u] != -1) stack.push_back(st.left[u]);
        }
        return nodes;
    }

    static set<int> boundaryProtectedNodes(const Problem& p, const State& st) {
        set<int> protectedNodes;
        for (int node = 0; node < static_cast<int>(st.moduleAt.size()); ++node)
            if (p.blocks[st.moduleAt[node]].isEdge()) protectedNodes.insert(node);
        for (Side side : {Side::BOTTOM, Side::LEFT, Side::RIGHT, Side::TOP})
            if (!primaryBoundaryModules(p, side).empty()) {
                vector<int> branch = boundaryBranch(st, side);
                protectedNodes.insert(branch.begin(), branch.end());
            }
        return protectedNodes;
    }

    static bool moveWholeSubtree(const Problem& p, State& st, int root, mt19937& rng) {
        if (root == st.root || st.parent[root] == -1) return false;
        set<int> members;
        for (int n : subtreeNodes(st, root)) members.insert(n);
        int oldParent = st.parent[root];
        char oldSide = st.left[oldParent] == root ? 'L' : 'R';
        if (oldSide == 'L') st.left[oldParent] = -1; else st.right[oldParent] = -1;
        st.parent[root] = -1;
        vector<pair<int, char>> slots;
        for (auto slot : offPathExternalSlots(p, st, root)) {
            if (!members.count(slot.first) && slot != pair<int, char>{oldParent, oldSide}) slots.push_back(slot);
        }
        if (slots.empty()) {
            if (oldSide == 'L') st.left[oldParent] = root; else st.right[oldParent] = root;
            st.parent[root] = oldParent;
            return false;
        }
        auto slot = randomChoiceT(slots, rng);
        if (slot.second == 'L') st.left[slot.first] = root; else st.right[slot.first] = root;
        st.parent[root] = slot.first;
        return true;
    }

    static bool swapWholeSubtrees(const Problem& p, State& st, int a, int b) {
        if (a == b || a == st.root || b == st.root) return false;
        vector<int> aNodes = subtreeNodes(st, a), bNodes = subtreeNodes(st, b);
        if (find(aNodes.begin(), aNodes.end(), b) != aNodes.end() ||
            find(bNodes.begin(), bNodes.end(), a) != bNodes.end()) return false;
        int pa = st.parent[a], pb = st.parent[b];
        char sa = st.left[pa] == a ? 'L' : 'R';
        char sb = st.left[pb] == b ? 'L' : 'R';
        if (sa == 'L') st.left[pa] = b; else st.right[pa] = b;
        if (sb == 'L') st.left[pb] = a; else st.right[pb] = a;
        st.parent[a] = pb; st.parent[b] = pa;
        (void)p;
        return true;
    }

    static pair<vector<double>, vector<double>> placementAxisSlacks(const Placement& placement) {
        const int n = static_cast<int>(placement.rects.size());
        auto oneAxis = [&](char axis) {
            vector<double> starts(n), sizes(n), orthLow(n), orthHigh(n);
            const double extent = axis == 'x' ? placement.W : placement.H;
            for (int i = 0; i < n; ++i) {
                if (axis == 'x') {
                    starts[i] = placement.rects[i].x; sizes[i] = placement.rects[i].w;
                    orthLow[i] = placement.rects[i].y; orthHigh[i] = topOf(placement.rects[i]);
                }
                else {
                    starts[i] = placement.rects[i].y; sizes[i] = placement.rects[i].h;
                    orthLow[i] = placement.rects[i].x; orthHigh[i] = rightOf(placement.rects[i]);
                }
            }
            vector<vector<int>> adj(n);
            for (int a = 0; a < n; ++a) for (int b = a + 1; b < n; ++b) {
                if (min(orthHigh[a], orthHigh[b]) <= max(orthLow[a], orthLow[b]) + FP_EPS) continue;
                if (starts[a] + sizes[a] <= starts[b] + 1.0e-7) adj[a].push_back(b);
                else if (starts[b] + sizes[b] <= starts[a] + 1.0e-7) adj[b].push_back(a);
            }
            vector<int> order(n); iota(order.begin(), order.end(), 0);
            sort(order.begin(), order.end(), [&](int a, int b) { return tie(starts[a], a) < tie(starts[b], b); });
            vector<double> forward(n, 0.0), backward(n, 0.0);
            for (int u : order) for (int v : adj[u]) forward[v] = max(forward[v], forward[u] + sizes[u]);
            for (auto it = order.rbegin(); it != order.rend(); ++it) for (int v : adj[*it])
                backward[*it] = max(backward[*it], sizes[v] + backward[v]);
            double critical = extent;
            for (int i = 0; i < n; ++i) critical = max(critical, forward[i] + sizes[i] + backward[i]);
            vector<double> slack(n);
            for (int i = 0; i < n; ++i) slack[i] = max(0.0, critical - forward[i] - sizes[i] - backward[i]);
            return slack;
            };
        return { oneAxis('x'), oneAxis('y') };
    }

    static bool relocateBtreeNode(State& st, int node, int target, bool leftChild, mt19937& rng) {
        if (node == target || node < 0 || target < 0 || node >= static_cast<int>(st.moduleAt.size()) ||
            target >= static_cast<int>(st.moduleAt.size())) return false;
        deleteBtreeNode(st, node, &rng);
        insertIsolatedAtLink(st, node, target, leftChild ? 'L' : 'R');
        return true;
    }

    struct Metrics {
        double cost = numeric_limits<double>::infinity();
        double quality = numeric_limits<double>::infinity();
        bool feasible = false;
        double overflow = 0.0;
        double overflowSq = 0.0;
        double boundaryViolation = 0.0;
        double boundarySpanExcess = 0.0;
        double actualArea = numeric_limits<double>::infinity();
        double areaNorm = 0.0;
        double hpwl = numeric_limits<double>::infinity();
        double hpwlNorm = 0.0;
        double centerHpwl = 0.0;
        double utilization = 0.0;
    };

    class Evaluator {
    public:
        Evaluator(const Problem& problem, double wireWeight)
            : p_(problem), wireWeight_(clampD(wireWeight, 0.0, 1.0)) {
            double totalWeight = 0.0;
            for (const auto& c : p_.connections) totalWeight += c.weight;
            areaNorm_ = max(p_.maxW * p_.maxH, 1.0);
            wireNorm_ = max(totalWeight * (p_.maxW + p_.maxH) * 0.5, 1.0);
        }

        void setNormalizers(double area, double wire, int samples) {
            areaNorm_ = max(area, 1.0); wireNorm_ = max(wire, 1.0); samples_ = max(0, samples);
        }
        double wireWeight() const { return wireWeight_; }
        double areaNormalizer() const { return areaNorm_; }
        double wireNormalizer() const { return wireNorm_; }

        Metrics evaluate(const Placement& placement, double areaWeight) const {
            Metrics m;
            double minX = 0.0, minY = 0.0;
            if (!placement.rects.empty()) {
                minX = placement.rects.front().x; minY = placement.rects.front().y;
                for (const Rect& r : placement.rects) { minX = min(minX, r.x); minY = min(minY, r.y); }
            }
            const double dxR = max(0.0, placement.W - p_.maxW) / max(p_.maxW, 1.0);
            const double dyT = max(0.0, placement.H - p_.maxH) / max(p_.maxH, 1.0);
            const double dxL = max(0.0, -minX) / max(p_.maxW, 1.0);
            const double dyB = max(0.0, -minY) / max(p_.maxH, 1.0);
            m.overflow = dxR + dyT + dxL + dyB;
            m.overflowSq = dxR * dxR + dyT * dyT + dxL * dxL + dyB * dyB;
            m.actualArea = placement.W * placement.H;
            m.areaNorm = m.actualArea / areaNorm_;
            const double widthExcess = p_.boundaryMinW > FP_EPS ? max(0.0, placement.W - p_.boundaryMinW) / p_.boundaryMinW : 0.0;
            const double heightExcess = p_.boundaryMinH > FP_EPS ? max(0.0, placement.H - p_.boundaryMinH) / p_.boundaryMinH : 0.0;
            m.boundarySpanExcess = widthExcess + heightExcess;
            m.hpwl = weightedPortHpwl(p_, placement);
            m.centerHpwl = weightedCenterHpwl(p_, placement);
            m.hpwlNorm = m.hpwl / wireNorm_;
            for (const PBlock& b : p_.blocks) {
                if (!b.isEdge()) continue;
                const Rect& r = placement.rects[b.index];
                for (Side s : b.boundarySides()) {
                    if (s == Side::LEFT) m.boundaryViolation += fabs(r.x) / max(placement.W, 1.0);
                    else if (s == Side::RIGHT) m.boundaryViolation += fabs(placement.W - rightOf(r)) / max(placement.W, 1.0);
                    else if (s == Side::BOTTOM) m.boundaryViolation += fabs(r.y) / max(placement.H, 1.0);
                    else if (s == Side::TOP) m.boundaryViolation += fabs(placement.H - topOf(r)) / max(placement.H, 1.0);
                }
                if (!edgeLocationRegionsOk(b, r, placement.W, placement.H, true)) m.boundaryViolation += 1.0;
            }
            const double compactWeight = max(0.02, areaWeight);
            m.quality = m.areaNorm;
            m.cost = 2000.0 * m.overflowSq + 120.0 * m.overflow +
                5000.0 * m.boundaryViolation + 2.0 * m.boundarySpanExcess +
                compactWeight * m.areaNorm + wireWeight_ * m.hpwlNorm;
            m.feasible = m.overflow <= 1.0e-8 && m.boundaryViolation <= 1.0e-8;
            m.utilization = p_.totalArea / max(m.actualArea, 1.0);
            return m;
        }

        double normalizedObjective(const Metrics& m, double areaWeight) const {
            return max(0.02, areaWeight) * m.areaNorm + wireWeight_ * m.hpwlNorm + 2.0 * m.boundarySpanExcess;
        }

    private:
        const Problem& p_;
        double wireWeight_ = 0.2;
        double areaNorm_ = 1.0;
        double wireNorm_ = 1.0;
        int samples_ = 0;
    };

    using Rank = tuple<int, double, double, double, double>;
    static Rank metricRank(const Metrics& m) {
        const double violation = m.overflow + m.boundaryViolation;
        return { m.feasible ? 0 : 1, violation, m.actualArea, m.hpwl, m.cost };
    }

    static string stateSignature(const State& st) {
        string s;
        auto addInts = [&](const vector<int>& v) { for (int x : v) { s += to_string(x); s.push_back(','); } s.push_back('|'); };
        addInts(st.parent); addInts(st.left); addInts(st.right); addInts(st.moduleAt);
        for (char c : st.rotated) { s.push_back(c ? '1' : '0'); }
        s.push_back('|');
        for (double x : st.softAr) { long long q = llround(x * 1.0e9); s += to_string(q); s.push_back(','); }
        return s;
    }

    static vector<pair<string, Placement>> fourCornerPackings(const Problem& p, const State& st, const Placement& base) {
        vector<pair<string, Placement>> out;
        for (auto item : vector<tuple<string, bool, bool>>{ {"BL", true, true}, {"BR", false, true}, {"TL", true, false}, {"TR", false, false} }) {
            try {
                out.emplace_back(get<0>(item), iterativeDirectionalCompact(p, st, base, get<1>(item), get<2>(item)));
            }
            catch (const exception&) {}
        }
        return out;
    }

    class FastSolver {
    public:
        FastSolver(const Problem& p, unsigned seed, optional<tuple<double, double, int>> shared = nullopt)
            : p_(p), rng_(seed), seed_(seed), evaluator_(p, p.alpha), shared_(shared) {
            for (const PBlock& b : p.blocks) if (b.isSoft()) softModules_.push_back(b.index);
            if (shared_) evaluator_.setNormalizers(get<0>(*shared_), get<1>(*shared_), get<2>(*shared_));
        }

        struct Result {
            State state;
            Placement placement;
            Metrics metrics;
            unsigned seed = 0;
            string direction = "BL";
            int evaluations = 0;
        };

        tuple<double, double, int> normalizers() const {
            return { evaluator_.areaNormalizer(), evaluator_.wireNormalizer(), DEFAULT_NORMALIZATION_SAMPLES };
        }

        Result solve() {
            State initial = buildInitialState(p_, rng_);
            Placement initialPlacement = exactPack(p_, initial);
            int evaluations = 1;
            if (!shared_) evaluations += normalize(initial, initialPlacement);
            auto warm = warmup(initial, initialPlacement);
            State current = get<0>(warm);
            Placement currentPlacement = get<1>(warm);
            Metrics currentMetrics = get<2>(warm);
            double temperature = get<3>(warm);
            evaluations += get<4>(warm);
            State bestState = current;
            Placement bestPlacement = currentPlacement;
            Metrics bestMetrics = currentMetrics;
            Rank bestRank = metricRank(bestMetrics);

            struct ArchiveItem { Rank rank; string sig; State state; Placement placement; };
            vector<ArchiveItem> archive;
            set<string> archiveSigs;
            auto remember = [&](const State& st, const Placement& pl, const Metrics& m) {
                if (!m.feasible) return;
                string sig = stateSignature(st);
                if (archiveSigs.count(sig)) return;
                archive.push_back({ metricRank(m), sig, st, pl }); archiveSigs.insert(sig);
                sort(archive.begin(), archive.end(), [](const auto& a, const auto& b) { return a.rank < b.rank; });
                while (static_cast<int>(archive.size()) > DEFAULT_ARCHIVE_LIMIT) {
                    archiveSigs.erase(archive.back().sig); archive.pop_back();
                }
                };
            remember(bestState, bestPlacement, bestMetrics);

            deque<bool> recent;
            double avgDelta = 0.1;
            for (int level = 1; level <= DEFAULT_TEMPERATURE_LEVELS; ++level) {
                string stage;
                if (level == 1) stage = "random";
                else if (level <= DEFAULT_GREEDY_LEVELS) stage = "pseudo-greedy";
                else stage = "hill-climbing";
                const double feasibleRate = recent.empty() ? 0.0 : accumulate(recent.begin(), recent.end(), 0.0) / recent.size();
                const double areaWeight = max(0.05, 1.0 - evaluator_.wireWeight()) * (1.0 + 2.0 * (1.0 - feasibleRate));
                if (level == 1) {
                    // keep warm-up temperature
                }
                else if (level <= DEFAULT_GREEDY_LEVELS) {
                    temperature = max(1.0e-8, temperature * avgDelta / (level * DEFAULT_GREEDY_C));
                }
                else {
                    temperature *= level < (DEFAULT_TEMPERATURE_LEVELS * 3 / 4) ? 0.96 : 0.98;
                }
                vector<double> deltas;
                for (int move = 0; move < DEFAULT_FIXED_MOVES_PER_LEVEL; ++move) {
                    auto proposal = propose(current, currentPlacement, areaWeight, stage);
                    State cand = std::move(get<0>(proposal));
                    Placement candPlacement = std::move(get<1>(proposal));
                    Metrics candMetrics = get<2>(proposal);
                    evaluations += get<3>(proposal);
                    const double delta = candMetrics.cost - currentMetrics.cost;
                    const double nd = evaluator_.normalizedObjective(candMetrics, areaWeight) - evaluator_.normalizedObjective(currentMetrics, areaWeight);
                    if (isfinite(nd)) deltas.push_back(fabs(nd));
                    recent.push_back(candMetrics.feasible); if (recent.size() > 64) recent.pop_front();
                    bool accept = delta <= 0.0;
                    if (!accept && temperature > FP_EPS) {
                        const double probability = exp(-min(delta / temperature, 700.0));
                        accept = uniform_real_distribution<double>(0.0, 1.0)(rng_) < probability;
                    }
                    remember(cand, candPlacement, candMetrics);
                    if (metricRank(candMetrics) < bestRank) {
                        bestRank = metricRank(candMetrics); bestState = cand; bestPlacement = candPlacement; bestMetrics = candMetrics;
                    }
                    if (accept) {
                        current = std::move(cand); currentPlacement = std::move(candPlacement); currentMetrics = candMetrics;
                    }
                }
                if (!deltas.empty()) avgDelta = min(1.0, accumulate(deltas.begin(), deltas.end(), 0.0) / deltas.size());
            }

            if (!archiveSigs.count(stateSignature(bestState))) archive.push_back({ bestRank, stateSignature(bestState), bestState, bestPlacement });
            sort(archive.begin(), archive.end(), [](const auto& a, const auto& b) { return a.rank < b.rank; });
            Result selected{ bestState, bestPlacement, bestMetrics, seed_, "BL", evaluations };
            const int finalists = min(DEFAULT_FINAL_CANDIDATES, static_cast<int>(archive.size()));
            for (int i = 0; i < finalists; ++i) {
                auto polished = deterministicSoftPolish(archive[i].state, archive[i].placement);
                evaluations += get<3>(polished);
                for (auto& variant : fourCornerPackings(p_, get<0>(polished), get<1>(polished))) {
                    Metrics m = evaluator_.evaluate(variant.second, max(0.05, 1.0 - evaluator_.wireWeight()));
                    ++evaluations;
                    if (metricRank(m) < metricRank(selected.metrics)) {
                        selected.state = get<0>(polished); selected.placement = variant.second;
                        selected.metrics = m; selected.direction = variant.first;
                    }
                }
            }
            selected.evaluations = evaluations;
            return selected;
        }

        const Evaluator& evaluator() const { return evaluator_; }

    private:
        const Problem& p_;
        mt19937 rng_;
        unsigned seed_;
        Evaluator evaluator_;
        optional<tuple<double, double, int>> shared_;
        vector<int> softModules_;

        int normalize(const State& initial, const Placement& placement) {
            State probe = initial;
            Placement pp = placement;
            vector<double> areas, wires;
            const double areaWeight = max(0.05, 1.0 - evaluator_.wireWeight());
            int evals = 0;
            for (int i = 0; i < DEFAULT_NORMALIZATION_SAMPLES; ++i) {
                auto q = propose(probe, pp, areaWeight, "random");
                probe = get<0>(q); pp = get<1>(q); Metrics m = get<2>(q); evals += get<3>(q);
                areas.push_back(pp.W * pp.H); wires.push_back(m.hpwl);
            }
            if (areas.empty()) { areas.push_back(placement.W * placement.H); wires.push_back(weightedPortHpwl(p_, placement)); }
            evaluator_.setNormalizers(accumulate(areas.begin(), areas.end(), 0.0) / areas.size(),
                accumulate(wires.begin(), wires.end(), 0.0) / wires.size(),
                static_cast<int>(areas.size()));
            return evals;
        }

        tuple<State, Placement, Metrics, double, int> warmup(const State& initial, const Placement& placement) {
            State probe = initial;
            Placement pp = placement;
            const double areaWeight = max(0.05, 1.0 - evaluator_.wireWeight());
            Metrics pm = evaluator_.evaluate(pp, areaWeight);
            State best = probe; Placement bp = pp; Metrics bm = pm; Rank br = metricRank(bm);
            vector<double> positive;
            int evals = 1;
            const int samples = DEFAULT_WARMUP_MOVES_PER_BLOCK * max(1, static_cast<int>(p_.blocks.size()));
            for (int i = 0; i < samples; ++i) {
                auto q = propose(probe, pp, areaWeight, "random");
                State cand = get<0>(q); Placement cp = get<1>(q); Metrics cm = get<2>(q); evals += get<3>(q);
                double d = evaluator_.normalizedObjective(cm, areaWeight) - evaluator_.normalizedObjective(pm, areaWeight);
                if (d > FP_EPS && isfinite(d)) positive.push_back(d);
                probe = std::move(cand); pp = std::move(cp); pm = cm;
                if (metricRank(cm) < br) { br = metricRank(cm); best = probe; bp = pp; bm = cm; }
            }
            double avg = positive.empty() ? 0.1 : accumulate(positive.begin(), positive.end(), 0.0) / positive.size();
            double temp = max(1.0e-5, -avg / log(DEFAULT_INITIAL_ACCEPT));
            return { best, bp, bm, temp, evals };
        }

        bool parquetSlackMove(State& st, const Placement& placement) {
            auto slacks = placementAxisSlacks(placement);
            const double characteristic = sqrt(max(p_.totalArea, 1.0));
            const double wp = placement.W / max(max(p_.boundaryMinW, characteristic), 1.0);
            const double hp = placement.H / max(max(p_.boundaryMinH, characteristic), 1.0);
            bool horizontal = fabs(wp - hp) > 1.0e-6 ? wp > hp : uniform_int_distribution<int>(0, 1)(rng_);
            const vector<double>& values = horizontal ? slacks.first : slacks.second;
            vector<int> movable;
            for (int node = 0; node < static_cast<int>(st.moduleAt.size()); ++node)
                if (!p_.blocks[st.moduleAt[node]].isEdge()) movable.push_back(node);
            if (movable.empty()) return false;
            sort(movable.begin(), movable.end(), [&](int a, int b) { return tie(values[st.moduleAt[a]], a) < tie(values[st.moduleAt[b]], b); });
            int operandRange = max(1, static_cast<int>(ceil(movable.size() / 5.0)));
            int operand = movable[uniform_int_distribution<int>(0, operandRange - 1)(rng_)];
            vector<int> targets;
            for (int n = 0; n < static_cast<int>(st.moduleAt.size()); ++n) if (n != operand) targets.push_back(n);
            sort(targets.begin(), targets.end(), [&](int a, int b) { return tie(values[st.moduleAt[a]], a) < tie(values[st.moduleAt[b]], b); });
            int targetRange = max(1, static_cast<int>(ceil(targets.size() / 5.0)));
            int target = targets[targets.size() - 1 - uniform_int_distribution<int>(0, targetRange - 1)(rng_)];
            return relocateBtreeNode(st, operand, target, horizontal, rng_);
        }

        bool parquetHpwlMove(State& st, const Placement& placement) {
            vector<int> nodeFor = moduleToNodeMap(st);
            vector<vector<pair<int, double>>> incident(p_.blocks.size());
            for (const auto& c : p_.connections) {
                if (c.source == c.target || c.weight <= 0.0) continue;
                incident[c.source].push_back({ c.target, c.weight });
                incident[c.target].push_back({ c.source, c.weight });
            }
            vector<int> candidates; vector<double> weights;
            for (int module = 0; module < static_cast<int>(p_.blocks.size()); ++module) {
                if (p_.blocks[module].isEdge() || incident[module].empty()) continue;
                auto point = portPoint(p_.blocks[module], placement.rects[module]);
                double contribution = 0.0;
                for (auto [other, w] : incident[module]) {
                    auto op = portPoint(p_.blocks[other], placement.rects[other]);
                    contribution += w * (fabs(point.first - op.first) + fabs(point.second - op.second));
                }
                if (contribution > FP_EPS) { candidates.push_back(module); weights.push_back(contribution); }
            }
            if (candidates.empty()) return false;
            discrete_distribution<int> choose(weights.begin(), weights.end());
            int module = candidates[choose(rng_)];
            double cx = 0.0, cy = 0.0, total = 0.0;
            for (auto [other, w] : incident[module]) {
                auto op = portPoint(p_.blocks[other], placement.rects[other]);
                cx += w * op.first; cy += w * op.second; total += w;
            }
            cx /= max(total, FP_EPS); cy /= max(total, FP_EPS);
            int target = -1; double best = numeric_limits<double>::infinity();
            for (int node = 0; node < static_cast<int>(st.moduleAt.size()); ++node) {
                if (node == nodeFor[module]) continue;
                auto c = centerOf(placement.rects[st.moduleAt[node]]);
                double d = fabs(c.first - cx) + fabs(c.second - cy);
                if (d < best) { best = d; target = node; }
            }
            if (target < 0) return false;
            const Rect& tr = placement.rects[st.moduleAt[target]];
            bool leftChild = cx >= centerOf(tr).first;
            return relocateBtreeNode(st, nodeFor[module], target, leftChild, rng_);
        }

        vector<double> fastShapeRatios(const State& st, const Placement& placement, int blockId, bool includeRandom) {
            const PBlock& b = p_.blocks[blockId];
            const Rect& r = placement.rects[blockId];
            set<double> ratios{ b.arMin, b.arMax, st.softAr[blockId] };
            vector<double> rightEdges, topEdges;
            for (const Rect& o : placement.rects) { rightEdges.push_back(rightOf(o)); topEdges.push_back(topOf(o)); }
            vector<double> greaterX, smallerX, greaterY, smallerY;
            for (double x : rightEdges) { if (x > rightOf(r) + FP_EPS) greaterX.push_back(x); if (x < rightOf(r) - FP_EPS) smallerX.push_back(x); }
            for (double y : topEdges) { if (y > topOf(r) + FP_EPS) greaterY.push_back(y); if (y < topOf(r) - FP_EPS) smallerY.push_back(y); }
            vector<double> widths, heights;
            if (!greaterX.empty()) widths.push_back(*min_element(greaterX.begin(), greaterX.end()) - r.x);
            if (!smallerX.empty()) widths.push_back(*max_element(smallerX.begin(), smallerX.end()) - r.x);
            if (!greaterY.empty()) heights.push_back(*min_element(greaterY.begin(), greaterY.end()) - r.y);
            if (!smallerY.empty()) heights.push_back(*max_element(smallerY.begin(), smallerY.end()) - r.y);
            for (double w : widths) if (w > FP_EPS) ratios.insert(clampD(w * w / b.area, b.arMin, b.arMax));
            for (double h : heights) if (h > FP_EPS) ratios.insert(clampD(b.area / (h * h), b.arMin, b.arMax));
            if (includeRandom && b.arMax > b.arMin + FP_EPS) {
                double lr = uniform_real_distribution<double>(log(b.arMin), log(b.arMax))(rng_);
                ratios.insert(exp(lr));
            }
            return vector<double>(ratios.begin(), ratios.end());
        }

        bool parquetSoftSlackResize(State& st, const Placement& placement) {
            if (softModules_.empty()) return false;
            auto slacks = placementAxisSlacks(placement);
            vector<double> weights;
            for (int id : softModules_) weights.push_back(1.0 / max(1.0e-6, min(slacks.first[id], slacks.second[id]) + 1.0e-6));
            discrete_distribution<int> choose(weights.begin(), weights.end());
            int id = softModules_[choose(rng_)];
            const PBlock& b = p_.blocks[id];
            const Rect& r = placement.rects[id];
            double ratio;
            if (slacks.first[id] > slacks.second[id]) {
                double w = min(sqrt(b.area * b.arMax), r.w + slacks.first[id]);
                ratio = w * w / b.area;
            }
            else {
                double h = min(sqrt(b.area / b.arMin), r.h + slacks.second[id]);
                ratio = b.area / max(h * h, FP_EPS);
            }
            ratio = clampD(ratio, b.arMin, b.arMax);
            if (fabs(ratio - st.softAr[id]) <= 1.0e-10) return false;
            st.softAr[id] = ratio;
            return true;
        }

        tuple<State, Placement, Metrics, int, string> propose(const State& state, const Placement& placement,
            double areaWeight, const string& stage) {
            const bool softExists = !softModules_.empty();
            double choice = uniform_real_distribution<double>(0.0, 1.0)(rng_);
            if (choice < DEFAULT_SUBTREE_RATE) {
                State cand = state;
                set<int> protectedNodes = boundaryProtectedNodes(p_, cand);
                vector<int> roots;
                int limit = stage == "random" ? max(2, min(64, static_cast<int>(p_.blocks.size() / 2))) :
                    (stage == "pseudo-greedy" ? max(2, static_cast<int>(ceil(sqrt(p_.blocks.size())))) : 3);
                for (int node = 0; node < static_cast<int>(cand.moduleAt.size()); ++node) {
                    vector<int> members = subtreeNodes(cand, node);
                    bool protectedHit = any_of(members.begin(), members.end(), [&](int x) { return protectedNodes.count(x); });
                    if (node != cand.root && members.size() >= 2 && static_cast<int>(members.size()) <= limit && !protectedHit) roots.push_back(node);
                }
                bool changed = false;
                if (!roots.empty()) {
                    int a = randomChoice(roots, rng_);
                    if (uniform_int_distribution<int>(0, 1)(rng_) == 0) changed = moveWholeSubtree(p_, cand, a, rng_);
                    else {
                        vector<int> others;
                        for (int b : roots) if (b != a) others.push_back(b);
                        if (!others.empty()) changed = swapWholeSubtrees(p_, cand, a, randomChoice(others, rng_));
                    }
                }
                if (changed) {
                    try {
                        repairBoundaryConstraints(p_, cand, rng_);
                        Placement cp = exactPack(p_, cand);
                        return { cand, cp, evaluator_.evaluate(cp, areaWeight), 1, "subtree" };
                    }
                    catch (const exception&) {}
                }
                choice = DEFAULT_SUBTREE_RATE + (1.0 - DEFAULT_SUBTREE_RATE) * uniform_real_distribution<double>(0.0, 1.0)(rng_);
            }
            const double relative = (choice - DEFAULT_SUBTREE_RATE) / max(1.0 - DEFAULT_SUBTREE_RATE, FP_EPS);
            double slackRate = 0.0, hpwlRate = 0.0, softSlackRate = 0.0, softOpRate = 0.0;
            if (stage == "random") { hpwlRate = evaluator_.wireWeight() > FP_EPS ? 0.08 : 0.0; softOpRate = softExists ? 0.06 : 0.0; }
            else if (stage == "pseudo-greedy") { slackRate = 0.18; hpwlRate = evaluator_.wireWeight() > FP_EPS ? 0.12 : 0.0; softSlackRate = softExists ? 0.08 : 0.0; softOpRate = softExists ? 0.14 : 0.0; }
            else { slackRate = 0.26; hpwlRate = evaluator_.wireWeight() > FP_EPS ? 0.14 : 0.0; softSlackRate = softExists ? 0.12 : 0.0; softOpRate = softExists ? 0.18 : 0.0; }
            double threshold = slackRate;
            auto finish = [&](State cand, bool changed, const string& name) -> optional<tuple<State, Placement, Metrics, int, string>> {
                if (!changed) return nullopt;
                try { repairBoundaryConstraints(p_, cand, rng_); Placement cp = exactPack(p_, cand); return tuple<State, Placement, Metrics, int, string>{cand, cp, evaluator_.evaluate(cp, areaWeight), 1, name}; }
                catch (const exception&) { return nullopt; }
                };
            if (relative < threshold) { State c = state; auto r = finish(c, parquetSlackMove(c, placement), "parquet-slack"); if (r) return *r; }
            threshold += hpwlRate;
            if (relative < threshold) { State c = state; auto r = finish(c, parquetHpwlMove(c, placement), "parquet-hpwl"); if (r) return *r; }
            threshold += softSlackRate;
            if (relative < threshold) { State c = state; auto r = finish(c, parquetSoftSlackResize(c, placement), "parquet-soft"); if (r) return *r; }
            threshold += softOpRate;
            if (softExists && relative < threshold) {
                State c = state;
                int id = randomChoice(softModules_, rng_);
                vector<double> ratios = fastShapeRatios(c, placement, id, true);
                ratios.erase(remove_if(ratios.begin(), ratios.end(), [&](double x) { return fabs(x - c.softAr[id]) <= FP_EPS; }), ratios.end());
                if (!ratios.empty()) c.softAr[id] = randomChoiceT(ratios, rng_);
                try { Placement cp = exactPack(p_, c); return { c, cp, evaluator_.evaluate(cp, areaWeight), 1, "soft-op4" }; }
                catch (const exception&) { return { state, placement, evaluator_.evaluate(placement, areaWeight), 1, "soft-reject" }; }
            }
            State c = state;
            try {
                string op = paperRandomMutation(p_, c, rng_, DEFAULT_ALLOW_MACRO_ROTATE);
                repairBoundaryConstraints(p_, c, rng_);
                Placement cp = exactPack(p_, c);
                return { c, cp, evaluator_.evaluate(cp, areaWeight), 1, op };
            }
            catch (const exception&) {
                return { state, placement, evaluator_.evaluate(placement, areaWeight), 1, "mutation-reject" };
            }
        }

        tuple<State, Placement, Metrics, int> deterministicSoftPolish(State state, Placement placement) {
            const double areaWeight = max(0.05, 1.0 - evaluator_.wireWeight());
            Metrics metrics = evaluator_.evaluate(placement, areaWeight);
            int evals = 1;
            for (int round = 1; round <= DEFAULT_SOFT_POLISH_ROUNDS; ++round) {
                bool improved = false;
                auto slacks = placementAxisSlacks(placement);
                vector<int> order = softModules_;
                sort(order.begin(), order.end(), [&](int a, int b) {
                    const vector<double>& primary = round % 2 ? slacks.first : slacks.second;
                    const vector<double>& secondary = round % 2 ? slacks.second : slacks.first;
                    return tie(primary[a], secondary[a], a) < tie(primary[b], secondary[b], b);
                    });
                for (int id : order) {
                    const PBlock& b = p_.blocks[id];
                    set<double> ratios;
                    for (double x : fastShapeRatios(state, placement, id, false)) ratios.insert(x);
                    double w = min(sqrt(b.area * b.arMax), placement.rects[id].w + max(0.0, slacks.first[id]));
                    double h = min(sqrt(b.area / b.arMin), placement.rects[id].h + max(0.0, slacks.second[id]));
                    ratios.insert(clampD(w * w / b.area, b.arMin, b.arMax));
                    ratios.insert(clampD(b.area / max(h * h, FP_EPS), b.arMin, b.arMax));
                    State localState = state; Placement localPlacement = placement; Metrics localMetrics = metrics;
                    for (double ratio : ratios) {
                        if (fabs(ratio - state.softAr[id]) <= 1.0e-10) continue;
                        State cand = state; cand.softAr[id] = ratio;
                        try {
                            Placement cp = exactPack(p_, cand); Metrics cm = evaluator_.evaluate(cp, areaWeight); ++evals;
                            if (metricRank(cm) < metricRank(localMetrics)) { localState = cand; localPlacement = cp; localMetrics = cm; }
                        }
                        catch (const exception&) {}
                    }
                    if (stateSignature(localState) != stateSignature(state)) {
                        state = std::move(localState); placement = std::move(localPlacement); metrics = localMetrics; improved = true;
                        slacks = placementAxisSlacks(placement);
                    }
                }
                if (!improved) break;
            }
            return { state, placement, metrics, evals };
        }
    };
    static bool placementGeometryLegal(const Problem& p, const Placement& pl) {
        for (const Rect& r : pl.rects) if (r.x < -1.0e-7 || r.y < -1.0e-7 || rightOf(r) > pl.W + 1.0e-7 || topOf(r) > pl.H + 1.0e-7) return false;
        for (int a = 0; a < static_cast<int>(pl.rects.size()); ++a) for (int b = a + 1; b < static_cast<int>(pl.rects.size()); ++b)
            if (overlapArea(pl.rects[a], pl.rects[b]) > 1.0e-5) return false;
        for (const PBlock& b : p.blocks) if (b.isEdge()) {
            const Rect& r = pl.rects[b.index];
            for (Side s : b.boundarySides()) {
                if (s == Side::LEFT && fabs(r.x) > 1.0e-7) return false;
                if (s == Side::RIGHT && fabs(rightOf(r) - pl.W) > 1.0e-7) return false;
                if (s == Side::BOTTOM && fabs(r.y) > 1.0e-7) return false;
                if (s == Side::TOP && fabs(topOf(r) - pl.H) > 1.0e-7) return false;
            }
            if (!edgeLocationRegionsOk(b, r, pl.W, pl.H, true)) return false;
        }
        return true;
    }

    static optional<Placement> integerizePlacementCoordinates(
        const Problem& p, const Placement& input) {
        const int n = static_cast<int>(input.rects.size());
        if (n != static_cast<int>(p.blocks.size()) ||
            !placementGeometryLegal(p, input)) return nullopt;

        vector<pair<int, int>> horizontalConstraints;
        vector<pair<int, int>> verticalConstraints;
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                const Rect& a = input.rects[i];
                const Rect& b = input.rects[j];
                const bool iLeft = rightOf(a) <= b.x + 1.0e-6;
                const bool jLeft = rightOf(b) <= a.x + 1.0e-6;
                const bool iBelow = topOf(a) <= b.y + 1.0e-6;
                const bool jBelow = topOf(b) <= a.y + 1.0e-6;

                if ((iLeft || jLeft) && (iBelow || jBelow)) {
                    const double horizontalGap = iLeft ?
                        b.x - rightOf(a) : a.x - rightOf(b);
                    const double verticalGap = iBelow ?
                        b.y - topOf(a) : a.y - topOf(b);
                    if (horizontalGap <= verticalGap) {
                        horizontalConstraints.push_back(
                            iLeft ? pair<int, int>{i, j} :
                            pair<int, int>{j, i});
                    }
                    else {
                        verticalConstraints.push_back(
                            iBelow ? pair<int, int>{i, j} :
                            pair<int, int>{j, i});
                    }
                }
                else if (iLeft) horizontalConstraints.push_back({ i, j });
                else if (jLeft) horizontalConstraints.push_back({ j, i });
                else if (iBelow) verticalConstraints.push_back({ i, j });
                else if (jBelow) verticalConstraints.push_back({ j, i });
                else return nullopt;
            }
        }

        auto ceilGrid = [](double value) {
            const double lower = floor(value + 1.0e-7);
            return fabs(value - lower) <= 1.0e-6 ?
                lower : ceil(value - 1.0e-7);
            };
        auto floorGrid = [](double value) {
            return fabs(value) <= 1.0e-7 ? 0.0 :
                floor(value + 1.0e-7);
            };
        auto onIntegerGrid = [](double value) {
            return fabs(value - round(value)) <= 1.0e-7;
            };
        const vector<double> outlineSlack = {
            0.0, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0
        };

        for (double extraH : outlineSlack) {
            for (double extraW : outlineSlack) {
                Placement candidate = input;
                candidate.W = candidate.decodedW =
                    ceilGrid(input.W) + extraW;
                candidate.H = candidate.decodedH =
                    ceilGrid(input.H) + extraH;
                if (candidate.W > p.maxW + FP_EPS ||
                    candidate.H > p.maxH + FP_EPS) continue;

                auto solveAxis = [&](bool horizontal,
                    const vector<pair<int, int>>& constraints,
                    double extent) {
                    vector<double> positions(n, 0.0);
                    vector<double> sizes(n, 0.0);
                    vector<char> lowFixed(n, 0);
                    vector<char> highFixed(n, 0);
                    const Side lowSide =
                        horizontal ? Side::LEFT : Side::BOTTOM;
                    const Side highSide =
                        horizontal ? Side::RIGHT : Side::TOP;

                    for (int id = 0; id < n; ++id) {
                        const Rect& rect = candidate.rects[id];
                        positions[id] = round(horizontal ? rect.x : rect.y);
                        sizes[id] = horizontal ? rect.w : rect.h;
                        lowFixed[id] = hasSide(p, id, lowSide);
                        highFixed[id] = hasSide(p, id, highSide);
                        if (lowFixed[id]) positions[id] = 0.0;
                        if (highFixed[id]) positions[id] = extent - sizes[id];
                        if (!onIntegerGrid(positions[id])) return false;
                    }

                    const int iterationLimit = max(8, 4 * n * n);
                    for (int iteration = 0;
                        iteration < iterationLimit; ++iteration) {
                        bool changed = false;
                        for (const auto& constraint : constraints) {
                            const int before = constraint.first;
                            const int after = constraint.second;
                            const double requiredAfter =
                                ceilGrid(positions[before] + sizes[before]);
                            if (positions[after] + FP_EPS >= requiredAfter)
                                continue;

                            if (!highFixed[after]) {
                                positions[after] = requiredAfter;
                                changed = true;
                            }
                            else if (!lowFixed[before]) {
                                const double requiredBefore =
                                    floorGrid(positions[after] - sizes[before]);
                                if (requiredBefore + FP_EPS >=
                                    positions[before]) return false;
                                positions[before] = requiredBefore;
                                changed = true;
                            }
                            else return false;
                        }
                        if (changed) continue;

                        for (int id = 0; id < n; ++id) {
                            if (!onIntegerGrid(positions[id]) ||
                                positions[id] < -FP_EPS ||
                                positions[id] + sizes[id] >
                                    extent + FP_EPS) return false;
                            if (horizontal)
                                candidate.rects[id].x = positions[id];
                            else candidate.rects[id].y = positions[id];
                        }
                        return true;
                    }
                    return false;
                    };

                if (!solveAxis(true, horizontalConstraints, candidate.W) ||
                    !solveAxis(false, verticalConstraints, candidate.H))
                    continue;
                if (!placementGeometryLegal(p, candidate)) continue;
                candidate.nodeRects.clear();
                return candidate;
            }
        }
        return nullopt;
    }


} // namespace phase1_warm_start

    // ============================================================================
    //  Fast routing-gap-aware source-code floorplanner adapter
    // ----------------------------------------------------------------------------
    //  Goal of this version: it must finish quickly and return a legal-ish packed
    //  coordinate floorplan.  No SA inner-loop repacking, no unbounded repair loops.
    //
    //  Flow:
    //    1) Generate a small set of SOFT shapes and block orders.
    //    2) Sweep a bounded set of outline widths.
    //    3) For each width, binary-search height with deterministic bottom-left pack.
    //    4) EDGE blocks are treated as fixed obstacles before movable packing.
    //
    //  Selection is real outline area W*H first; HPWL is only a small tie breaker.
    // ============================================================================

    static constexpr double INF = 1.0e100;
    static constexpr double TINY = 1.0e-9;
    static constexpr unsigned FAST_SEED = 7u;
    static constexpr int FAST_WIDTH_TRIALS = 6;
    static constexpr int FAST_HEIGHT_BISECT = 3; // used only by optional local refinement
    static constexpr int FAST_RANDOM_ORDERS = 2; // base; makeOrders() scales this with block count
    static constexpr int FAST_RANDOM_SHAPES = 0;
    // SOFT shape perturbation + predicted feedthrough reserve.
    // The current engine is not a full SA loop, so these are implemented as
    // deterministic / stochastic shape-state perturbations before each pack.
    static constexpr bool ENABLE_SOFT_SHAPE_PERTURB = true;
    static constexpr bool ENABLE_FT_SOFT_RESERVE = true;
    static constexpr int SOFT_SHAPE_TARGET_STATES = 18;
    static constexpr int SOFT_SHAPE_RANDOM_STATES = 14;
    static constexpr double SOFT_SHAPE_LOG_SIGMA = 0.42;
    static constexpr double SOFT_SHAPE_HOT_EXTREME_BIAS = 0.33;
    static constexpr double FT_SOFT_EXPAND_MIN_RATE = 0.000;
    static constexpr double FT_SOFT_EXPAND_MAX_RATE = 0.220;
    static constexpr double FT_SOFT_EXPAND_BASE_RATE = 0.010;
    static constexpr double FT_SOFT_MULTI_NET_BONUS = 0.030;
    static constexpr double FT_SOFT_EDGE_BONUS = 0.012;
    static constexpr double FT_SOFT_KEEP_CHANNEL_GAP_SCALE = 0.10;
    static constexpr double FT_SOFT_KEEP_CHANNEL_GAP_CAP = 6.0;
    static constexpr double HPWL_TIE_WEIGHT = 0.060;
    static constexpr double CENTER_TIE_WEIGHT = 1.0e-5;
    static constexpr double ROUTE_GAP_TIE_WEIGHT = 0.080;
    static constexpr double ROUTE_DENSITY = 25.0;
    static constexpr double ROUTE_GAP_SAFETY = 1.03;
    static constexpr double ROUTE_GAP_BIAS = 0.50;
    static constexpr double ROUTE_GAP_MAX_ABS = 180.0;
    static constexpr double PORT_WINDOW_SAFETY_FAST = 1.08;
    // Required-gap policy is intentionally used only to generate packing
    // coordinates and to compare candidates already generated by this fast packer.
    // It does NOT add any new SA/annealing cost term or extra inner-loop estimator.
    static constexpr double DIRECT_CHANNEL_SAFETY_FAST = 1.0;
    static constexpr double MIN_DIRECT_GUARD_NETS_FAST = 25.0;
    static constexpr double CHANNEL_THICKNESS_MIN_FAST = 1.0;
    static constexpr double CHANNEL_THICKNESS_MAX_FAST = 6.0;
    static constexpr double CHANNEL_THICKNESS_NET_SCALE_FAST = 0.018;
    static constexpr double SLIVER_SNAP_ABS_FAST = 2.0;
    static constexpr double SLIVER_SNAP_RATIO_FAST = 0.08;
    static constexpr double SPLIT_AWARE_MIN_SCALE_FAST = 0.24;
    static constexpr double SPLIT_AWARE_MAX_SCALE_FAST = 0.78;
    static constexpr double ENDPOINT_GUARD_SCALE_FAST = 0.022;
    static constexpr double ENDPOINT_GUARD_CAP_FAST = 18.0;
    static constexpr double PACK_GAP = 1.0e-3;
    static constexpr int FAST_MAX_TOTAL_LAYOUT_CANDIDATES = 6000;
    static constexpr bool ENABLE_EXPLICIT_TOPOLOGY_SUPPLEMENT = true;
    static constexpr int EXPLICIT_TOPOLOGY_SUPPLEMENT_TRIALS = 1200;
    static constexpr int EXPLICIT_TOPOLOGY_SUPPLEMENT_KEEP = 10;
    static constexpr int FAST_COORD_LIMIT_SMALL = 54;
    static constexpr int FAST_COORD_LIMIT_MID = 36;
    static constexpr int FAST_COORD_LIMIT_LARGE = 24;
    static int g_edgePlacementMode = 0;

    // Phase-1 packing deliberately uses a looser required-gap rule so area search
    // does not get trapped by over-reserved channels.  A bounded DSU-style post pass
    // then compacts the chosen legal floorplan and accepts only if routing-gap proxy
    // and HPWL do not degrade too much.
    static constexpr bool ENABLE_DSU_POST_COMPACTION = true;
    static constexpr double DSU_MAX_AREA_SHRINK = 0.135;
    static constexpr double DSU_DEADSPACE_START = 0.045;
    static constexpr double DSU_ROUTE_GUARD_RATIO = 1.10;
    static constexpr double DSU_HPWL_GUARD_RATIO = 1.10;
    // DSU must be a safe post-compaction.  It may reclaim deadspace, but it should
    // not destroy the route-gap/port-window structure selected by phase-1 packing.
    static constexpr double DSU_GAP_GUARD_RATIO = 1.12;
    static constexpr double DSU_PORT_GUARD_RATIO = 2.00;
    static constexpr double DSU_MIN_AREA_GAIN_IF_ROUTE_WORSE = 0.012;
    static constexpr double DSU_TINY_AREA_GAIN = 0.004;
    static constexpr double DSU_MAX_PORT_MISS_INCREASE = 2.00;
    static constexpr double DSU_MAX_GAP_MISS_INCREASE = 3.00;
    static constexpr int DSU_MAX_VIOLPAIR_INCREASE = 1;
    static constexpr int DSU_MAX_PASSES = 18;
    static constexpr bool FAST_VERBOSE_LOG = true;


    // ============================================================================
    //  B*-tree area-only simulated annealing
    // ----------------------------------------------------------------------------
    //  This is the main SA generator.  The Metropolis cost is intentionally only
    //  outline area (W * H).  Routing pressure / port-window / soft feedthrough
    //  reserve affect only:
    //    1) perturb bias: which topology / soft-shape / outline move to try;
    //    2) packing: B*-tree child spacing and obstacle avoidance use required gap.
    // ============================================================================
    static constexpr bool ENABLE_BSTAR_AREA_SA = true;
    static constexpr bool ENABLE_PHASE1_WARM_START = true;
    static constexpr int PHASE1_WARM_RESTARTS_SMALL = 2;
    static constexpr int PHASE1_WARM_RESTARTS_MEDIUM = 4;
    static constexpr int PHASE1_WARM_RESTARTS_LARGE = 6;
    static constexpr int BSTAR_INIT_TRIAL_CAP = 900;
    static constexpr int BSTAR_SA_INNER_FACTOR = 14;
    static constexpr int BSTAR_SA_MIN_INNER = 48;
    static constexpr int BSTAR_SA_MAX_OUTER = 140;
    static constexpr int BSTAR_SA_MAX_MOVES_CAP = 12000;
    static constexpr double BSTAR_SA_INIT_ACCEPT_P = 0.90;
    static constexpr double BSTAR_SA_COOL = 0.875;
    static constexpr double BSTAR_SA_MIN_TEMP = 1.0e-4;
    static constexpr double BSTAR_OUTLINE_LOG_SIGMA = 0.055;
    static constexpr double BSTAR_OUTLINE_SHRINK_BIAS = -0.018;
    static constexpr double BSTAR_OUTLINE_GROW_SIGMA = 0.035;
    static constexpr double BSTAR_SOFT_LOCAL_LOG_SIGMA = 0.36;
    static constexpr double BSTAR_PACK_DEVIATION_WEIGHT = 1.0e-4;
    static constexpr int BSTAR_ROOT_CAND_LIMIT = 42;
    static constexpr int BSTAR_CHILD_X_CAND_LIMIT = 24;
    static constexpr int BSTAR_ARCHIVE_LIMIT = 24;
    static constexpr int BSTAR_ARCHIVE_SAMPLE_PERIOD = 31;
    static constexpr double BSTAR_ARCHIVE_AREA_WINDOW = 0.14;
    static constexpr bool ENABLE_BSTAR_SECOND_PROXY_SA = true;
    static constexpr int BSTAR_SECOND_ARCHIVE_LIMIT = 8;
    static constexpr int BSTAR_SECOND_SA_MAX_MOVES_CAP = 3200;
    static constexpr double BSTAR_SECOND_ROUTE_PROXY_CAP = 0.060;
    static constexpr double BSTAR_SECOND_STRIP_PROXY_CAP = 0.045;
    static constexpr double BSTAR_SECOND_GROW_PROB = 0.105;
    static constexpr bool ENABLE_BSTAR_WIRE_SA = true;
    static constexpr int BSTAR_WIRE_ARCHIVE_LIMIT = 6;
    static constexpr int BSTAR_SPREAD_ARCHIVE_LIMIT = 5;
    static constexpr int BSTAR_COMPACT_SPREAD_KEEP = 4;
    static constexpr int BSTAR_WIRE_SEED_LIMIT = 7;
    static constexpr int BSTAR_WIRE_SA_MAX_MOVES_CAP = 2600;
    static constexpr int BSTAR_SPREAD_SA_MAX_MOVES_CAP = 2200;
    static constexpr double BSTAR_WIRE_ROUTE_PROXY_CAP = 0.035;
    static constexpr double BSTAR_WIRE_STRIP_PROXY_CAP = 0.025;
    static constexpr double BSTAR_SPREAD_AREA_WEIGHT = 0.64;
    static constexpr double STRIP_PROXY_FREE_UTIL = 0.72;
    static constexpr double STRIP_PROXY_TARGET_UTIL = 0.94;
    static constexpr double STRIP_PROXY_OVERFLOW_WEIGHT = 18000.0;
    static constexpr double STRIP_PROXY_RISK_WEIGHT = 1400.0;
    static constexpr double STRIP_PROXY_PEAK_WEIGHT = 70000.0;
    static constexpr double BSTAR_HPWL_COST_WEIGHT = 0.045;
    static constexpr double BSTAR_ROUTE_COST_WEIGHT = 0.020;
    static constexpr bool ENABLE_HPWL_FORCE_ARCHIVE_REFINEMENT = true;
    static constexpr int FORCE_REFINE_SEED_LIMIT = 8;
    static constexpr int FORCE_REFINE_KEEP_LIMIT = 4;
    static constexpr int FORCE_REFINE_ITERS = 18;
    static constexpr double FORCE_REFINE_START_STEP = 0.045;
    static constexpr double FORCE_REFINE_END_STEP = 0.018;
    static constexpr double FORCE_ATTR_WEIGHT = 1.00;
    static constexpr double FORCE_REPULSE_WEIGHT = 0.85;
    static constexpr double FORCE_CHANNEL_WEIGHT = 0.90;
    static constexpr double FORCE_HOT_PAIR_FRAC = 0.20;
    static constexpr bool ENABLE_ALIGNMENT_ARCHIVE_REFINEMENT = true;
    static constexpr int ALIGNMENT_SEED_LIMIT = 8;
    static constexpr int ALIGNMENT_KEEP_LIMIT = 6;
    static constexpr int ALIGNMENT_MAX_GUIDES_PER_AXIS = 7;
    static constexpr int ALIGNMENT_MAX_ACCEPTED_MOVES = 18;
    static constexpr double ALIGNMENT_EXACT_EPS = 0.55;
    static constexpr double ALIGNMENT_HPWL_GUARD_RATIO = 1.035;
    static constexpr double ALIGNMENT_ROUTE_GUARD_RATIO = 1.080;
    static constexpr double ALIGNMENT_PROXY_GUARD_RATIO = 1.120;
    static constexpr bool ENABLE_DENSITY_AWARE_PACK = false;
    static constexpr double PACK_TOP_PRESSURE_RATIO = 0.0;
    static constexpr double PACK_RIGHT_PRESSURE_RATIO = 0.0;
    static int g_bstarPackMode = 0; // 0=compact route-aware, 1=wire-first, 2=spread-wire

    struct ScopedBStarPackMode {
        int old = 0;
        explicit ScopedBStarPackMode(int mode) : old(g_bstarPackMode) {
            g_bstarPackMode = mode;
        }
        ~ScopedBStarPackMode() {
            g_bstarPackMode = old;
        }
    };


    static double sqr(double x) { return x * x; }

    static double clampD(double v, double lo, double hi) {
        if (hi < lo) return lo;
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    static string upperAscii(string s) {
        transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
            return static_cast<char>(toupper(c));
            });
        return s;
    }

    static double ovLen(double a1, double a2, double b1, double b2) {
        return max(0.0, min(a2, b2) - max(a1, b1));
    }

    static int inwardPortForEdgeSide(char side) {
        if (side == 'L') return 3;
        if (side == 'R') return 1;
        if (side == 'B') return 2;
        return 4; // top edge faces inward through the block bottom edge.
    }

    static bool allowsPortEdge(const BlockSpec& spec, int port) {
        if (spec.portEdges.empty()) return true;
        return find(spec.portEdges.begin(), spec.portEdges.end(), port) != spec.portEdges.end();
    }

    static double ovArea(const Rect& a, const Rect& b) {
        return ovLen(a.x, rectRight(a), b.x, rectRight(b)) *
            ovLen(a.y, rectTop(a), b.y, rectTop(b));
    }

    static bool inside(const Rect& r, double W, double H) {
        return r.x >= -EPS && r.y >= -EPS && rectRight(r) <= W + EPS && rectTop(r) <= H + EPS;
    }

    static bool isMovableBlock(const BlockSpec& s) {
        return s.type != BlockType::EDGE;
    }

    static double aspectMid(const BlockSpec& s) {
        double amin = max(0.05, s.aspectMin);
        double amax = max(amin, s.aspectMax);
        return sqrt(amin * amax);
    }

    static double blockNominalArea(const BlockSpec& s) {
        return s.hasFixedSize ? max(1.0, s.fixedW * s.fixedH) : max(1.0, s.area);
    }

    static double totalNominalArea(const Design& design) {
        double a = 0.0;
        for (const auto& s : design.blockSpecs) a += blockNominalArea(s);
        return max(1.0, a);
    }

    static Rect makeShape(const BlockSpec& s, double ratio) {
        Rect r;
        r.x = r.y = 0.0;
        if (s.hasFixedSize) {
            r.w = s.fixedW;
            r.h = s.fixedH;
            return r;
        }
        double amin = max(0.05, s.aspectMin);
        double amax = max(amin, s.aspectMax);
        ratio = clampD(ratio > 0.0 ? ratio : aspectMid(s), amin, amax);
        double area = max(1.0, s.area);
        r.w = sqrt(area * ratio);
        r.h = area / max(TINY, r.w);
        return r;
    }

    static int totalConnBetween(const Design& design, int a, int b) {
        int n = 0;
        if (a >= 0 && b >= 0 && a < static_cast<int>(design.connMatrix.size()) &&
            b < static_cast<int>(design.connMatrix[a].size())) {
            n += max(0, design.connMatrix[a][b]);
        }
        if (a >= 0 && b >= 0 && b < static_cast<int>(design.connMatrix.size()) &&
            a < static_cast<int>(design.connMatrix[b].size())) {
            n += max(0, design.connMatrix[b][a]);
        }
        return n;
    }

    static double endpointDemand(const Design& design, int id) {
        if (id < 0 || id >= static_cast<int>(design.blockSpecs.size())) return 0.0;
        double s = 0.0;
        if (!design.connections.empty()) {
            for (const auto& c : design.connections) {
                if (c.src == id || c.dst == id) s += max(0, c.netCount);
            }
            return s;
        }
        for (int j = 0; j < static_cast<int>(design.blockSpecs.size()); ++j) {
            if (j == id) continue;
            s += totalConnBetween(design, id, j);
        }
        return s;
    }

    static bool smallOfficialLikeCase(const Design& design) {
        return design.blockSpecs.size() <= 10;
    }

    static double softFtExpansionMaxRateForDesign(const Design& design) {
        if (smallOfficialLikeCase(design)) return 0.020;
        if (design.blockSpecs.size() >= 45) return 0.180;
        return FT_SOFT_EXPAND_MAX_RATE;
    }

    static double maxEndpointDemand(const Design& design) {
        double best = 1.0;
        for (int i = 0; i < static_cast<int>(design.blockSpecs.size()); ++i) {
            best = max(best, endpointDemand(design, i));
        }
        return best;
    }

    static double maxPairConnFrom(const Design& design, int id) {
        double best = 0.0;
        for (int j = 0; j < static_cast<int>(design.blockSpecs.size()); ++j) {
            if (j == id) continue;
            best = max(best, static_cast<double>(totalConnBetween(design, id, j)));
        }
        return best;
    }

    static double softFtExpansionRate(const Design& design, int id) {
        if (!ENABLE_FT_SOFT_RESERVE) return 0.0;
        if (id < 0 || id >= static_cast<int>(design.blockSpecs.size())) return 0.0;
        const BlockSpec& s = design.blockSpecs[id];
        if (s.type != BlockType::SOFT || s.hasFixedSize) return 0.0;

        const double ep = endpointDemand(design, id);
        if (ep <= 0.0) return FT_SOFT_EXPAND_MIN_RATE;
        const double maxEp = maxEndpointDemand(design);
        const double hot = clampD(ep / max(1.0, maxEp), 0.0, 1.0);
        const double maxPair = maxPairConnFrom(design, id);
        const double spread = clampD(1.0 - maxPair / max(1.0, ep), 0.0, 1.0);

        // Two-hop hub proxy: if a soft block connects strongly to multiple blocks,
        // it is more likely to become a useful feedthrough bridge after routing.
        double twoHop = 0.0;
        for (int a = 0; a < static_cast<int>(design.blockSpecs.size()); ++a) {
            if (a == id) continue;
            const double ca = totalConnBetween(design, id, a);
            if (ca <= 0.0) continue;
            for (int b = a + 1; b < static_cast<int>(design.blockSpecs.size()); ++b) {
                if (b == id) continue;
                const double cb = totalConnBetween(design, id, b);
                if (cb <= 0.0) continue;
                twoHop += min(ca, cb);
            }
        }
        const double hub = clampD(twoHop / max(1.0, ep * max(1.0, static_cast<double>(design.blockSpecs.size()) - 2.0)), 0.0, 1.0);

        double edgeTouch = 0.0;
        for (int j = 0; j < static_cast<int>(design.blockSpecs.size()); ++j) {
            if (j == id) continue;
            if (design.blockSpecs[j].type == BlockType::EDGE) edgeTouch += totalConnBetween(design, id, j);
        }
        edgeTouch = clampD(edgeTouch / max(1.0, ep), 0.0, 1.0);

        const double maxRate = softFtExpansionMaxRateForDesign(design);
        double rate = FT_SOFT_EXPAND_BASE_RATE;
        rate += maxRate * (0.56 * sqrt(hot) + 0.24 * spread + 0.20 * sqrt(hub));
        rate += FT_SOFT_MULTI_NET_BONUS * spread * sqrt(hot);
        rate += FT_SOFT_EDGE_BONUS * edgeTouch;
        return clampD(rate, FT_SOFT_EXPAND_MIN_RATE, maxRate);
    }

    static double softFtSideReserve(const Design& design, int id) {
        if (!ENABLE_FT_SOFT_RESERVE) return 0.0;
        if (id < 0 || id >= static_cast<int>(design.blockSpecs.size())) return 0.0;
        const BlockSpec& s = design.blockSpecs[id];
        if (s.type != BlockType::SOFT || s.hasFixedSize) return 0.0;

        const double base = blockNominalArea(s);
        const double eta = softFtExpansionRate(design, id);
        if (eta <= 0.0) return 0.0;

        // The official FT growth is side-length based, not a raw area multiplier.
        // Use endpoint demand as a conservative FT proxy, then reserve a bounded
        // per-side halo.  Actual FT resizing still happens after real routing.
        const double delta = (endpointDemand(design, id) / ROUTE_DENSITY) * eta * 0.5;
        const double cap = max(3.0, 0.045 * sqrt(base));
        return clampD(delta, 0.0, cap);
    }

    static double blockPackingArea(const Design& design, int id) {
        if (id < 0 || id >= static_cast<int>(design.blockSpecs.size())) return 1.0;
        const BlockSpec& s = design.blockSpecs[id];
        const double base = blockNominalArea(s);
        if (s.hasFixedSize) return base;
        const double ratio = aspectMid(s);
        const double w = sqrt(base * ratio);
        const double h = base / max(TINY, w);
        const double d = softFtSideReserve(design, id);
        return (w + d) * (h + d);
    }

    static double totalPackingArea(const Design& design) {
        double a = 0.0;
        for (int i = 0; i < static_cast<int>(design.blockSpecs.size()); ++i) a += blockPackingArea(design, i);
        return max(1.0, a);
    }

    static Rect makePackingShape(const Design& design, int id, double ratio) {
        const BlockSpec& s = design.blockSpecs[id];
        Rect r;
        r.x = r.y = 0.0;
        if (s.hasFixedSize) {
            r.w = s.fixedW;
            r.h = s.fixedH;
            return r;
        }
        double amin = max(0.05, s.aspectMin);
        double amax = max(amin, s.aspectMax);
        ratio = clampD(ratio > 0.0 ? ratio : aspectMid(s), amin, amax);
        double area = blockNominalArea(s);
        r.w = sqrt(area * ratio);
        r.h = area / max(TINY, r.w);
        const double d = softFtSideReserve(design, id);
        r.w += d;
        r.h += d;
        return r;
    }

    static double ftSoftKeepChannelGapBonus(const Design& design, int a, int b, double fullDirect) {
        if (!ENABLE_FT_SOFT_RESERVE) return 0.0;
        const double ra = softFtExpansionRate(design, a);
        const double rb = softFtExpansionRate(design, b);
        const double rate = max(ra, rb) + 0.35 * min(ra, rb);
        if (rate <= 0.0) return 0.0;
        return clampD(fullDirect * FT_SOFT_KEEP_CHANNEL_GAP_SCALE * rate, 0.0, FT_SOFT_KEEP_CHANNEL_GAP_CAP);
    }

    static double requiredForNets(const Design& design, int nets) {
        if (nets <= 0) return PACK_GAP;
        // Raw physical lower bound from the problem rule: 25 nets / um.
        // This helper is the full direct-bundle value.  Pair-aware requiredX/YGap()
        // below may use a split-aware fraction of it because Q&A allows net splitting.
        double req = static_cast<double>(nets) / ROUTE_DENSITY + DIRECT_CHANNEL_SAFETY_FAST;
        req = req * ROUTE_GAP_SAFETY + ROUTE_GAP_BIAS;
        double cap = max(25.0, min(ROUTE_GAP_MAX_ABS, 0.10 * min(design.maxOutlineW, design.maxOutlineH)));
        return clampD(req, PACK_GAP, cap);
    }

    static double maxSingleNetRequired(const Design& design) {
        double req = 1.0;
        if (!design.connections.empty()) {
            for (const auto& c : design.connections) req = max(req, requiredForNets(design, max(0, c.netCount)));
            return req;
        }
        const int n = static_cast<int>(design.blockSpecs.size());
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) req = max(req, requiredForNets(design, totalConnBetween(design, i, j)));
        }
        return req;
    }

    static double sliverSnapThresholdFast(const Design& design) {
        // Imported from the reference sourcecode idea: tiny whitespace should not be
        // treated as a routable channel.  Here it is only a minimum coordinate gap for
        // connected pairs, not a new cost.
        return max(SLIVER_SNAP_ABS_FAST, maxSingleNetRequired(design) * SLIVER_SNAP_RATIO_FAST);
    }

    static double blockRoutingRigidity(const Design& design, int id) {
        if (id < 0 || id >= static_cast<int>(design.blockSpecs.size())) return 1.0;
        const BlockSpec& s = design.blockSpecs[id];
        if (s.type == BlockType::EDGE) return 1.18; // fixed boundary + no feedthrough
        if (s.type == BlockType::HARD) return 1.08; // no feedthrough
        if (s.type == BlockType::SOFT) return 0.88; // may absorb some traffic by FT
        return 1.0;
    }

    static double splitAwarePairScale(const Design& design, int a, int b, int nets) {
        if (nets <= 0) return 0.0;
        const double ea = endpointDemand(design, a);
        const double eb = endpointDemand(design, b);
        const double denom = max(1.0, min(ea, eb));
        const double pairShare = clampD(static_cast<double>(nets) / denom, 0.0, 1.0);

        // If this pair is most of both endpoints' demand, keep almost full direct
        // capacity.  If it is only one of many bundles, reserve a fraction and let
        // later routing/path splitting distribute it.
        double scale = SPLIT_AWARE_MIN_SCALE_FAST +
            (SPLIT_AWARE_MAX_SCALE_FAST - SPLIT_AWARE_MIN_SCALE_FAST) * sqrt(pairShare);

        // Hard/edge endpoints have fewer routing choices, so do not shrink their gap
        // as aggressively as soft-soft pairs.
        scale *= sqrt(blockRoutingRigidity(design, a) * blockRoutingRigidity(design, b));
        return clampD(scale, 0.20, 0.86);
    }

    static double estimatedDirectFlowNets(const Design& design, int a, int b, int nets) {
        if (nets <= 0) return 0.0;
        const double ea = endpointDemand(design, a);
        const double eb = endpointDemand(design, b);
        const double pairShare = clampD(static_cast<double>(nets) / max(1.0, min(ea, eb)), 0.0, 1.0);

        // Only a fraction of a pair's nets should be forced through the immediate
        // interface.  The rest may split through other channels or soft feedthrough.
        double direct = 0.45 + 0.55 * sqrt(pairShare);
        direct *= sqrt(blockRoutingRigidity(design, a) * blockRoutingRigidity(design, b));

        const bool aSoft = design.blockSpecs[a].type == BlockType::SOFT;
        const bool bSoft = design.blockSpecs[b].type == BlockType::SOFT;
        if (aSoft && bSoft) direct *= 0.92;
        else if (aSoft || bSoft) direct *= 0.96;

        return static_cast<double>(nets) * clampD(direct, 0.42, 1.00);
    }

    static double requiredPairGap(const Design& design, int a, int b, bool xGap) {
        (void)xGap; // LR/TB capacity is carried by perpendicular overlap below.
        const int nets = totalConnBetween(design, a, b);
        if (nets <= 0) return PACK_GAP;

        const double flow = estimatedDirectFlowNets(design, a, b, nets);
        double req = CHANNEL_THICKNESS_MIN_FAST + CHANNEL_THICKNESS_NET_SCALE_FAST * sqrt(max(0.0, flow));
        req *= clampD(sqrt(blockRoutingRigidity(design, a) * blockRoutingRigidity(design, b)), 0.85, 1.20);

        // Keep a small numerical channel thickness, but do not let direct-net count
        // become a second capacity reservation on top of requiredPortOverlap().
        req += ftSoftKeepChannelGapBonus(design, a, b, requiredForNets(design, nets));
        req = max(req, min(CHANNEL_THICKNESS_MAX_FAST, sliverSnapThresholdFast(design) * 0.30));
        return clampD(req, PACK_GAP, CHANNEL_THICKNESS_MAX_FAST);
    }

    static double requiredXGap(const Design& design, int a, int b) {
        return requiredPairGap(design, a, b, true);
    }

    static double requiredYGap(const Design& design, int a, int b) {
        return requiredPairGap(design, a, b, false);
    }

    static double requiredPortOverlap(const Design& design, int a, int b) {
        int nets = totalConnBetween(design, a, b);
        if (nets <= 0) return 0.0;
        return estimatedDirectFlowNets(design, a, b, nets) / ROUTE_DENSITY * PORT_WINDOW_SAFETY_FAST;
    }


    static double hpwl(const Design& design, const vector<Rect>& r) {
        double s = 0.0;
        if (!design.connections.empty()) {
            for (const auto& c : design.connections) {
                if (c.src < 0 || c.dst < 0 || c.src >= static_cast<int>(r.size()) || c.dst >= static_cast<int>(r.size())) continue;
                s += static_cast<double>(max(0, c.netCount)) *
                    (fabs(rectCx(r[c.src]) - rectCx(r[c.dst])) + fabs(rectCy(r[c.src]) - rectCy(r[c.dst])));
            }
            return s;
        }
        int n = min(static_cast<int>(r.size()), static_cast<int>(design.connMatrix.size()));
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                int nets = totalConnBetween(design, i, j);
                if (nets <= 0) continue;
                s += static_cast<double>(nets) *
                    (fabs(rectCx(r[i]) - rectCx(r[j])) + fabs(rectCy(r[i]) - rectCy(r[j])));
            }
        }
        return s;
    }

    struct EdgeRule {
        char side = 'B'; // T/B/L/R
        int zone = 0;   // T/B: L,M,R -> 0,1,2. L/R: B,M,T -> 0,1,2.
        bool valid = false;
    };

    static bool containsChar(const string& s, char c) { return s.find(c) != string::npos; }

    static EdgeRule parseRule(const string& raw) {
        EdgeRule r;
        string L = upperAscii(raw);
        if (L.size() >= 2) {
            char a = L[0], b = L[1];
            if (a == 'T' && (b == 'L' || b == 'M' || b == 'R')) { r.side = 'T'; r.zone = (b == 'L') ? 0 : (b == 'M' ? 1 : 2); r.valid = true; return r; }
            if (a == 'B' && (b == 'L' || b == 'M' || b == 'R')) { r.side = 'B'; r.zone = (b == 'L') ? 0 : (b == 'M' ? 1 : 2); r.valid = true; return r; }
            if (a == 'L' && (b == 'B' || b == 'M' || b == 'T')) { r.side = 'L'; r.zone = (b == 'B') ? 0 : (b == 'M' ? 1 : 2); r.valid = true; return r; }
            if (a == 'R' && (b == 'B' || b == 'M' || b == 'T')) { r.side = 'R'; r.zone = (b == 'B') ? 0 : (b == 'M' ? 1 : 2); r.valid = true; return r; }
        }
        if (containsChar(L, 'T')) { r.side = 'T'; r.zone = containsChar(L, 'R') ? 2 : (containsChar(L, 'M') ? 1 : 0); r.valid = true; }
        else if (containsChar(L, 'B')) { r.side = 'B'; r.zone = containsChar(L, 'R') ? 2 : (containsChar(L, 'M') ? 1 : 0); r.valid = true; }
        else if (containsChar(L, 'R')) { r.side = 'R'; r.zone = containsChar(L, 'T') ? 2 : (containsChar(L, 'M') ? 1 : 0); r.valid = true; }
        else if (containsChar(L, 'L')) { r.side = 'L'; r.zone = containsChar(L, 'T') ? 2 : (containsChar(L, 'M') ? 1 : 0); r.valid = true; }
        return r;
    }

    static vector<string> splitLocTokens(const string& s) {
        vector<string> out;
        string cur;
        for (char ch : s) {
            if (ch == ',' || ch == ';' || ch == '/' || ch == ' ' || ch == '\t' || ch == '\n' || ch == '\r') {
                if (!cur.empty()) { out.push_back(cur); cur.clear(); }
            }
            else cur.push_back(ch);
        }
        if (!cur.empty()) out.push_back(cur);
        return out;
    }

    static vector<EdgeRule> edgeRules(const BlockSpec& s) {
        vector<EdgeRule> out;
        if (s.locations.empty()) {
            EdgeRule r = parseRule("BL");
            if (r.valid) out.push_back(r);
            return out;
        }
        for (const string& loc : s.locations) {
            vector<string> toks = splitLocTokens(loc);
            if (toks.empty()) toks.push_back(loc);
            for (const string& t : toks) {
                EdgeRule r = parseRule(t);
                if (r.valid) out.push_back(r);
            }
        }
        if (out.empty()) out.push_back(parseRule("BL"));
        return out;
    }

    static double sideSpan(char side, double W, double H) {
        return (side == 'T' || side == 'B') ? W : H;
    }

    static double sideLen(const Rect& r, char side) {
        return (side == 'T' || side == 'B') ? r.w : r.h;
    }

    static pair<double, double> zoneBounds(const EdgeRule& r, double W, double H) {
        double s = sideSpan(r.side, W, H);
        return { static_cast<double>(r.zone) * s / 3.0, static_cast<double>(r.zone + 1) * s / 3.0 };
    }

    static double zoneCenter(const EdgeRule& r, double W, double H) {
        auto b = zoneBounds(r, W, H);
        return 0.5 * (b.first + b.second);
    }

    static double edgeViolation(const Rect& rc, const BlockSpec& spec, double W, double H) {
        if (spec.type != BlockType::EDGE) return 0.0;
        double best = INF;
        for (const EdgeRule& r : edgeRules(spec)) {
            auto b = zoneBounds(r, W, H);
            double v = 0.0;
            if (r.side == 'T') { v += fabs(rectTop(rc) - H); v += max(0.0, b.first - rc.x); v += max(0.0, rectRight(rc) - b.second); }
            else if (r.side == 'B') { v += fabs(rc.y); v += max(0.0, b.first - rc.x); v += max(0.0, rectRight(rc) - b.second); }
            else if (r.side == 'L') { v += fabs(rc.x); v += max(0.0, b.first - rc.y); v += max(0.0, rectTop(rc) - b.second); }
            else { v += fabs(rectRight(rc) - W); v += max(0.0, b.first - rc.y); v += max(0.0, rectTop(rc) - b.second); }
            best = min(best, v);
        }
        for (char side : { 'T', 'B', 'L', 'R' }) {
            bool used = false;
            double lo = 0.0;
            double hi = 0.0;
            for (const EdgeRule& r : edgeRules(spec)) {
                if (!r.valid || r.side != side) continue;
                auto b = zoneBounds(r, W, H);
                if (!used) {
                    used = true;
                    lo = b.first;
                    hi = b.second;
                }
                else {
                    lo = min(lo, b.first);
                    hi = max(hi, b.second);
                }
            }
            if (!used) continue;
            double v = 0.0;
            if (side == 'T') { v += fabs(rectTop(rc) - H); v += max(0.0, lo - rc.x); v += max(0.0, rectRight(rc) - hi); }
            else if (side == 'B') { v += fabs(rc.y); v += max(0.0, lo - rc.x); v += max(0.0, rectRight(rc) - hi); }
            else if (side == 'L') { v += fabs(rc.x); v += max(0.0, lo - rc.y); v += max(0.0, rectTop(rc) - hi); }
            else { v += fabs(rectRight(rc) - W); v += max(0.0, lo - rc.y); v += max(0.0, rectTop(rc) - hi); }
            best = min(best, v);
        }
        return best >= INF * 0.5 ? 0.0 : best;
    }

    static void setEdgeAxis(Rect& r, const EdgeRule& er, double axis, double W, double H) {
        if (er.side == 'T') { r.x = axis; r.y = H - r.h; }
        else if (er.side == 'B') { r.x = axis; r.y = 0.0; }
        else if (er.side == 'L') { r.x = 0.0; r.y = axis; }
        else { r.x = W - r.w; r.y = axis; }
        r.x = clampD(r.x, 0.0, W - r.w);
        r.y = clampD(r.y, 0.0, H - r.h);
    }

    static bool anyOverlapWith(const Rect& r, const vector<Rect>& placed) {
        for (const Rect& o : placed) {
            if (ovArea(r, o) > EPS) return true;
        }
        return false;
    }

    struct ShapeState {
        vector<double> ratio;
    };

    static vector<Rect> makeShapes(const Design& design, const ShapeState& st) {
        int n = static_cast<int>(design.blockSpecs.size());
        vector<Rect> shapes(n);
        for (int i = 0; i < n; ++i) {
            double ratio = (i < static_cast<int>(st.ratio.size())) ? st.ratio[i] : aspectMid(design.blockSpecs[i]);
            shapes[i] = makePackingShape(design, i, ratio);
        }
        return shapes;
    }

    static double partialWire(const Design& design, const vector<Rect>& rects, const vector<char>& placed, int id, const Rect& cand) {
        double s = 0.0;
        if (!design.connections.empty()) {
            for (const auto& c : design.connections) {
                int other = -1;
                if (c.src == id) other = c.dst;
                else if (c.dst == id) other = c.src;
                if (other < 0 || other >= static_cast<int>(rects.size()) || !placed[other]) continue;
                s += static_cast<double>(max(0, c.netCount)) *
                    (fabs(rectCx(cand) - rectCx(rects[other])) + fabs(rectCy(cand) - rectCy(rects[other])));
            }
            return s;
        }
        for (int other = 0; other < static_cast<int>(rects.size()); ++other) {
            if (other == id || !placed[other]) continue;
            int nets = totalConnBetween(design, id, other);
            if (nets <= 0) continue;
            s += static_cast<double>(nets) *
                (fabs(rectCx(cand) - rectCx(rects[other])) + fabs(rectCy(cand) - rectCy(rects[other])));
        }
        return s;
    }

    static void addCoord(vector<double>& v, double x, double lo, double hi) {
        if (x >= lo - EPS && x <= hi + EPS) v.push_back(clampD(x, lo, hi));
    }

    static void pruneCoords(vector<double>& v, int limit) {
        if (static_cast<int>(v.size()) <= limit) return;
        vector<double> out;
        out.reserve(limit + 2);
        out.push_back(v.front());
        const int n = static_cast<int>(v.size());
        for (int k = 1; k + 1 < limit; ++k) {
            int idx = static_cast<int>(llround(static_cast<double>(k) * (n - 1) / max(1, limit - 1)));
            idx = max(0, min(n - 1, idx));
            out.push_back(v[idx]);
        }
        out.push_back(v.back());
        sort(out.begin(), out.end());
        out.erase(unique(out.begin(), out.end(), [](double a, double b) { return fabs(a - b) < 1e-5; }), out.end());
        v.swap(out);
    }

    static void addQuadrantArea(const Rect& r, double W, double H, double q[4]) {
        const double mx = 0.5 * W;
        const double my = 0.5 * H;
        for (int ix = 0; ix < 2; ++ix) {
            double x1 = (ix == 0) ? 0.0 : mx;
            double x2 = (ix == 0) ? mx : W;
            double ox = max(0.0, min(rectRight(r), x2) - max(r.x, x1));
            if (ox <= EPS) continue;
            for (int iy = 0; iy < 2; ++iy) {
                double y1 = (iy == 0) ? 0.0 : my;
                double y2 = (iy == 0) ? my : H;
                double oy = max(0.0, min(rectTop(r), y2) - max(r.y, y1));
                if (oy <= EPS) continue;
                q[iy * 2 + ix] += ox * oy;
            }
        }
    }

    static double quadrantBalancePenalty(
        const vector<Rect>& placed,
        const Rect& cand,
        double W,
        double H
    ) {
        if (!ENABLE_DENSITY_AWARE_PACK) return 0.0;
        double q[4] = { 0.0, 0.0, 0.0, 0.0 };
        for (const Rect& r : placed) addQuadrantArea(r, W, H, q);
        addQuadrantArea(cand, W, H, q);
        double total = q[0] + q[1] + q[2] + q[3];
        double mean = 0.25 * total;
        double norm = max(1.0, 0.25 * W * H);
        double p = 0.0;
        for (double a : q) p += sqr((a - mean) / norm);
        return p;
    }


    struct EdgePlacedInfo {
        int id = -1;
        EdgeRule rule;
        double pref = 0.0;
        double len = 0.0;
    };

    static bool placeEdgesFast(
        const Design& design,
        const vector<Rect>& shapes,
        double W,
        double H,
        vector<Rect>& rects,
        vector<Rect>& obstacles,
        bool strictZone
    ) {
        rects = shapes;
        obstacles.clear();
        const int n = static_cast<int>(design.blockSpecs.size());

        vector<EdgePlacedInfo> sideItems[4];
        auto sideIndex = [](char s) { return s == 'T' ? 0 : (s == 'B' ? 1 : (s == 'L' ? 2 : 3)); };
        const bool smartEdgeChoice = false;
        const bool smartAxisSpread = (g_edgePlacementMode == 0 && n >= 20);
        const bool loadBalancedEdgeChoice = (g_edgePlacementMode == 1);
        const bool centeredEdgeZones = (g_edgePlacementMode == 2);
        double sideLoad[4] = { 0.0, 0.0, 0.0, 0.0 };
        double zoneLoad[12] = { 0.0 };
        vector<EdgeRule> selectedRule(n);

        vector<int> edgeIds;
        for (int i = 0; i < n; ++i) if (design.blockSpecs[i].type == BlockType::EDGE) edgeIds.push_back(i);
        sort(edgeIds.begin(), edgeIds.end(), [&](int a, int b) {
            double aa = shapes[a].w * shapes[a].h;
            double bb = shapes[b].w * shapes[b].h;
            if (fabs(aa - bb) > 1e-6) return aa > bb;
            return a < b;
            });

        for (int id : edgeIds) {
            const Rect& sh = shapes[id];
            vector<EdgeRule> rules = edgeRules(design.blockSpecs[id]);
            EdgeRule best = rules.empty() ? parseRule("BL") : rules.front();
            double bestScore = INF;
            bool hasPortFacingRule = false;
            for (const EdgeRule& r : rules) {
                if (!r.valid) continue;
                if (allowsPortEdge(design.blockSpecs[id], inwardPortForEdgeSide(r.side))) {
                    hasPortFacingRule = true;
                    break;
                }
            }
            for (int ri = 0; ri < static_cast<int>(rules.size()); ++ri) {
                const EdgeRule& r = rules[ri];
                if (!r.valid) continue;
                double span = sideSpan(r.side, W, H);
                double len = sideLen(sh, r.side);
                auto zb = zoneBounds(r, W, H);
                double zoneCap = max(0.0, zb.second - zb.first);
                double zoneOverflow = max(0.0, len - zoneCap);
                double sideOverflow = max(0.0, len - span);
                int si = sideIndex(r.side);
                int bucket = si * 3 + r.zone;
                double projectedSide = sideLoad[si] + len + PACK_GAP;
                double projectedZone = zoneLoad[bucket] + len + PACK_GAP;
                double packedSideOverflow = max(0.0, projectedSide - span);
                double packedZoneOverflow = max(0.0, projectedZone - zoneCap);
                double sideUtil = projectedSide / max(1.0, span);
                double zoneUtil = projectedZone / max(1.0, zoneCap);
                double score = 1.0e9 * sqr(sideOverflow) + 1.0e6 * sqr(zoneOverflow) + 1.0e-3 * r.zone;
                if (smartEdgeChoice) {
                    if (hasPortFacingRule && !allowsPortEdge(design.blockSpecs[id], inwardPortForEdgeSide(r.side))) {
                        score += 1.0e5;
                    }
                    score +=
                        5.0e-3 * sqr(packedSideOverflow) +
                        5.0e-4 * sqr(packedZoneOverflow) +
                        1.0e-4 * sqr(zoneUtil) +
                        1.0e-5 * sqr(sideUtil) +
                        1.0e-2 * static_cast<double>(ri);
                }
                if (loadBalancedEdgeChoice) {
                    score +=
                        1.0e8 * sqr(packedSideOverflow) +
                        1.0e5 * sqr(packedZoneOverflow) +
                        5.0e3 * sqr(zoneUtil) +
                        5.0e2 * sqr(sideUtil);
                }
                if (score < bestScore) { bestScore = score; best = r; }
            }
            EdgePlacedInfo it;
            it.id = id;
            it.rule = best;
            selectedRule[id] = best;
            it.len = sideLen(sh, best.side);
            it.pref = zoneCenter(best, W, H) - 0.5 * it.len;
            int si = sideIndex(best.side);
            int bucket = si * 3 + best.zone;
            sideLoad[si] += it.len + PACK_GAP;
            zoneLoad[bucket] += it.len + PACK_GAP;
            sideItems[si].push_back(it);
        }

        for (int si = 0; si < 4; ++si) {
            auto& items = sideItems[si];
            if (items.empty()) continue;
            char side = items.front().rule.side;
            double span = sideSpan(side, W, H);
            sort(items.begin(), items.end(), [](const EdgePlacedInfo& a, const EdgePlacedInfo& b) {
                if (a.rule.zone != b.rule.zone) return a.rule.zone < b.rule.zone;
                if (fabs(a.pref - b.pref) > 1e-6) return a.pref < b.pref;
                return a.id < b.id;
                });

            // First try strict zone-wise packing.  If a zone overflows, fall back to
            // full-side packing so the program still finishes and returns coordinates.
            bool zoneOK = true;
            for (int z = 0; z < 3; ++z) {
                double total = 0.0;
                int cnt = 0;
                EdgeRule zr;
                zr.side = side; zr.zone = z; zr.valid = true;
                for (const auto& it : items) if (it.rule.zone == z) { total += it.len; ++cnt; }
                total += PACK_GAP * max(0, cnt - 1);
                auto zb = zoneBounds(zr, W, H);
                if (total > zb.second - zb.first + EPS) zoneOK = false;
            }

            if (strictZone && zoneOK) {
                for (int z = 0; z < 3; ++z) {
                    EdgeRule zr; zr.side = side; zr.zone = z; zr.valid = true;
                    auto zb = zoneBounds(zr, W, H);
                    double total = 0.0;
                    int cnt = 0;
                    for (const auto& it : items) if (it.rule.zone == z) { total += it.len; ++cnt; }
                    total += PACK_GAP * max(0, cnt - 1);
                    double pos = 0.5 * (zb.first + zb.second - total);
                    if (!centeredEdgeZones && !smallOfficialLikeCase(design)) {
                        const double slack = max(0.0, (zb.second - zb.first) - total);
                        const double inset = smartAxisSpread ? min(96.0, 0.14 * slack) : 0.0;
                        if (z == 0) pos = zb.first + inset;
                        else if (z == 2) pos = zb.second - total - inset;
                    }
                    pos = clampD(pos, zb.first, zb.second - total);
                    for (const auto& it : items) {
                        if (it.rule.zone != z) continue;
                        setEdgeAxis(rects[it.id], it.rule, pos, W, H);
                        pos += it.len + PACK_GAP;
                    }
                }
            }
            else {
                double total = 0.0;
                for (const auto& it : items) total += it.len;
                total += PACK_GAP * max(0, static_cast<int>(items.size()) - 1);
                sort(items.begin(), items.end(), [](const EdgePlacedInfo& a, const EdgePlacedInfo& b) {
                    if (a.rule.zone != b.rule.zone) return a.rule.zone < b.rule.zone;
                    if (fabs(a.len - b.len) > 1e-6) return a.len < b.len;
                    if (fabs(a.pref - b.pref) > 1e-6) return a.pref < b.pref;
                    return a.id < b.id;
                    });
                double freeSpan = max(0.0, span - total);
                double pos = centeredEdgeZones
                    ? clampD(0.5 * freeSpan, 0.0, freeSpan)
                    : clampD(smartAxisSpread ? min(96.0, 0.08 * freeSpan) : 0.0, 0.0, freeSpan);
                for (const auto& it : items) {
                    setEdgeAxis(rects[it.id], it.rule, pos, W, H);
                    pos += it.len + PACK_GAP;
                }
            }
        }

        // Bounded corner repair for perpendicular edge overlaps.
        for (int pass = 0; pass < 12; ++pass) {
            bool changed = false;
            for (int ai = 0; ai < static_cast<int>(edgeIds.size()); ++ai) {
                for (int bi = ai + 1; bi < static_cast<int>(edgeIds.size()); ++bi) {
                    int a = edgeIds[ai], b = edgeIds[bi];
                    if (ovArea(rects[a], rects[b]) <= EPS) continue;
                    // Move the smaller edge block along its side away from the conflict.
                    int id = (rects[a].w * rects[a].h <= rects[b].w * rects[b].h) ? a : b;
                    EdgeRule r = selectedRule[id].valid ? selectedRule[id] :
                        (edgeRules(design.blockSpecs[id]).empty() ? parseRule("BL") : edgeRules(design.blockSpecs[id]).front());
                    double span = sideSpan(r.side, W, H);
                    double len = sideLen(rects[id], r.side);
                    double cur = (r.side == 'T' || r.side == 'B') ? rects[id].x : rects[id].y;
                    vector<double> cands = { 0.0, max(0.0, span - len), cur };
                    for (int o : edgeIds) if (o != id) {
                        if (r.side == 'T' || r.side == 'B') {
                            cands.push_back(rects[o].x - rects[id].w - PACK_GAP);
                            cands.push_back(rectRight(rects[o]) + PACK_GAP);
                        }
                        else {
                            cands.push_back(rects[o].y - rects[id].h - PACK_GAP);
                            cands.push_back(rectTop(rects[o]) + PACK_GAP);
                        }
                    }
                    Rect old = rects[id], best = rects[id];
                    double bestOv = INF, bestMove = INF;
                    for (double p : cands) {
                        p = clampD(p, 0.0, max(0.0, span - len));
                        setEdgeAxis(rects[id], r, p, W, H);
                        double ov = 0.0;
                        for (int o : edgeIds) if (o != id) ov += ovArea(rects[id], rects[o]);
                        double mv = fabs(p - cur);
                        if (ov < bestOv - EPS || (fabs(ov - bestOv) <= EPS && mv < bestMove)) {
                            bestOv = ov; bestMove = mv; best = rects[id];
                        }
                    }
                    rects[id] = best;
                    if (bestOv + EPS < ovArea(old, rects[id == a ? b : a])) changed = true;
                }
            }
            if (!changed) break;
        }

        bool ok = true;
        for (int id : edgeIds) {
            if (!inside(rects[id], W, H)) ok = false;
            if (strictZone && edgeViolation(rects[id], design.blockSpecs[id], W, H) > 1.0e-4) ok = false;
            obstacles.push_back(rects[id]);
        }
        return ok || !strictZone;
    }

    struct PlaceKey {
        double top = INF;
        double topPressure = INF;
        double route = INF;
        double balance = INF;
        double rightPressure = INF;
        double right = INF;
        double y = INF;
        double wire = INF;
        double center = INF;
        double x = INF;
    };

    static bool betterKey(const PlaceKey& a, const PlaceKey& b) {
        if (g_bstarPackMode != 0) {
            if (fabs(a.topPressure - b.topPressure) > 1e-6) return a.topPressure < b.topPressure;
            if (g_bstarPackMode == 1 && fabs(a.route - b.route) > 1e-6) return a.route < b.route;
            if (fabs(a.balance - b.balance) > 1e-8) return a.balance < b.balance;
            if (fabs(a.wire - b.wire) > 1e-6) return a.wire < b.wire;
            if (g_bstarPackMode == 2 && fabs(a.route - b.route) > 1e-6) return a.route < b.route;
            if (fabs(a.rightPressure - b.rightPressure) > 1e-6) return a.rightPressure < b.rightPressure;
            if (fabs(a.center - b.center) > 1e-6) return a.center < b.center;
            if (fabs(a.right - b.right) > 1e-6) return a.right < b.right;
            if (fabs(a.top - b.top) > 1e-6) return a.top < b.top;
            if (fabs(a.y - b.y) > 1e-6) return a.y < b.y;
            return a.x < b.x;
        }
        if (fabs(a.topPressure - b.topPressure) > 1e-6) return a.topPressure < b.topPressure;
        // Same-height candidates prefer satisfying direct channel / port-window needs.
        if (fabs(a.route - b.route) > 1e-6) return a.route < b.route;
        if (fabs(a.balance - b.balance) > 1e-8) return a.balance < b.balance;
        if (fabs(a.rightPressure - b.rightPressure) > 1e-6) return a.rightPressure < b.rightPressure;
        if (fabs(a.right - b.right) > 1e-6) return a.right < b.right;
        if (fabs(a.top - b.top) > 1e-6) return a.top < b.top;
        if (fabs(a.y - b.y) > 1e-6) return a.y < b.y;
        if (fabs(a.wire - b.wire) > 1e-6) return a.wire < b.wire;
        if (fabs(a.center - b.center) > 1e-6) return a.center < b.center;
        return a.x < b.x;
    }

    static void fillPlacementPressure(PlaceKey& key, const vector<Rect>& placed, const Rect& cand, double W, double H) {
        key.topPressure = max(0.0, key.top - PACK_TOP_PRESSURE_RATIO * H);
        key.rightPressure = max(0.0, key.right - PACK_RIGHT_PRESSURE_RATIO * W);
        key.balance = quadrantBalancePenalty(placed, cand, W, H);
    }

    static double routeGapPenaltyForCandidate(
        const Design& design,
        const vector<Rect>& placed,
        const vector<int>& placedIds,
        int id,
        const Rect& cand
    ) {
        double p = 0.0;
        for (int k = 0; k < static_cast<int>(placed.size()); ++k) {
            int oid = (k < static_cast<int>(placedIds.size())) ? placedIds[k] : -1;
            if (oid < 0 || oid == id) continue;
            int nets = totalConnBetween(design, id, oid);
            if (nets <= 0) continue;
            const Rect& o = placed[k];
            const double netsW = max(1.0, static_cast<double>(nets));

            double yOv = ovLen(cand.y, rectTop(cand), o.y, rectTop(o));
            if (yOv > EPS) {
                double xGap = INF;
                if (rectRight(o) <= cand.x + EPS) xGap = max(0.0, cand.x - rectRight(o));
                else if (rectRight(cand) <= o.x + EPS) xGap = max(0.0, o.x - rectRight(cand));
                if (xGap < INF * 0.5) {
                    double reqGap = requiredXGap(design, id, oid);
                    double missGap = max(0.0, reqGap - xGap);
                    double reqPort = requiredPortOverlap(design, id, oid);
                    double missPort = max(0.0, reqPort - yOv);
                    p += netsW * (sqr(missGap) + 0.35 * sqr(missPort));
                }
            }

            double xOv = ovLen(cand.x, rectRight(cand), o.x, rectRight(o));
            if (xOv > EPS) {
                double yGap = INF;
                if (rectTop(o) <= cand.y + EPS) yGap = max(0.0, cand.y - rectTop(o));
                else if (rectTop(cand) <= o.y + EPS) yGap = max(0.0, o.y - rectTop(cand));
                if (yGap < INF * 0.5) {
                    double reqGap = requiredYGap(design, id, oid);
                    double missGap = max(0.0, reqGap - yGap);
                    double reqPort = requiredPortOverlap(design, id, oid);
                    double missPort = max(0.0, reqPort - xOv);
                    p += netsW * (sqr(missGap) + 0.35 * sqr(missPort));
                }
            }
        }
        return p;
    }

    struct RouteGapBreakdown {
        double penalty = 0.0;
        double gapPenalty = 0.0;
        double portPenalty = 0.0;
        double maxGapMiss = 0.0;
        double maxPortMiss = 0.0;
        int connectedPairs = 0;
        int violPairs = 0;
        int xInterfaces = 0;
        int yInterfaces = 0;
    };

    static RouteGapBreakdown routeGapBreakdownGlobal(const Design& design, const vector<Rect>& rects) {
        RouteGapBreakdown rb;
        const int n = static_cast<int>(rects.size());
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                int nets = totalConnBetween(design, i, j);
                if (nets <= 0) continue;
                ++rb.connectedPairs;
                bool pairViol = false;
                const double netsW = max(1.0, static_cast<double>(nets));
                const Rect& a = rects[i];
                const Rect& b = rects[j];

                double yOv = ovLen(a.y, rectTop(a), b.y, rectTop(b));
                if (yOv > EPS) {
                    double xGap = INF;
                    if (rectRight(a) <= b.x + EPS) xGap = max(0.0, b.x - rectRight(a));
                    else if (rectRight(b) <= a.x + EPS) xGap = max(0.0, a.x - rectRight(b));
                    if (xGap < INF * 0.5) {
                        ++rb.xInterfaces;
                        double missGap = max(0.0, requiredXGap(design, i, j) - xGap);
                        double missPort = max(0.0, requiredPortOverlap(design, i, j) - yOv);
                        rb.maxGapMiss = max(rb.maxGapMiss, missGap);
                        rb.maxPortMiss = max(rb.maxPortMiss, missPort);
                        rb.gapPenalty += netsW * sqr(missGap);
                        rb.portPenalty += netsW * 0.35 * sqr(missPort);
                        if (missGap > EPS || missPort > EPS) pairViol = true;
                    }
                }
                double xOv = ovLen(a.x, rectRight(a), b.x, rectRight(b));
                if (xOv > EPS) {
                    double yGap = INF;
                    if (rectTop(a) <= b.y + EPS) yGap = max(0.0, b.y - rectTop(a));
                    else if (rectTop(b) <= a.y + EPS) yGap = max(0.0, a.y - rectTop(b));
                    if (yGap < INF * 0.5) {
                        ++rb.yInterfaces;
                        double missGap = max(0.0, requiredYGap(design, i, j) - yGap);
                        double missPort = max(0.0, requiredPortOverlap(design, i, j) - xOv);
                        rb.maxGapMiss = max(rb.maxGapMiss, missGap);
                        rb.maxPortMiss = max(rb.maxPortMiss, missPort);
                        rb.gapPenalty += netsW * sqr(missGap);
                        rb.portPenalty += netsW * 0.35 * sqr(missPort);
                        if (missGap > EPS || missPort > EPS) pairViol = true;
                    }
                }
                if (pairViol) ++rb.violPairs;
            }
        }
        rb.penalty = rb.gapPenalty + rb.portPenalty;
        return rb;
    }

    static double routeGapPenaltyGlobal(const Design& design, const vector<Rect>& rects) {
        return routeGapBreakdownGlobal(design, rects).penalty;
    }


    static bool packOne(
        const Design& design,
        const vector<int>& order,
        const ShapeState& shapeState,
        double W,
        double H,
        bool strictEdge,
        vector<Rect>& outRects
    ) {
        vector<Rect> shapes = makeShapes(design, shapeState);
        vector<Rect> rects, edgeObstacles;
        if (!placeEdgesFast(design, shapes, W, H, rects, edgeObstacles, strictEdge)) return false;

        vector<Rect> placed;
        vector<int> placedIds;
        vector<char> placedMask(design.blockSpecs.size(), 0);
        for (int i = 0; i < static_cast<int>(design.blockSpecs.size()); ++i) {
            if (design.blockSpecs[i].type == BlockType::EDGE) {
                placed.push_back(rects[i]);
                placedIds.push_back(i);
                placedMask[i] = 1;
            }
        }

        double curTop = 0.0;
        for (const Rect& r : placed) curTop = max(curTop, rectTop(r));

        for (int id : order) {
            if (id < 0 || id >= static_cast<int>(design.blockSpecs.size())) continue;
            if (!isMovableBlock(design.blockSpecs[id])) continue;
            const Rect shape = shapes[id];
            if (shape.w > W + EPS || shape.h > H + EPS) return false;
            double maxX = W - shape.w;
            double maxY = H - shape.h;

            vector<double> xs, ys;
            xs.reserve(4 * placed.size() + 6);
            ys.reserve(4 * placed.size() + 6);
            addCoord(xs, 0.0, 0.0, maxX);
            addCoord(xs, maxX, 0.0, maxX);
            addCoord(ys, 0.0, 0.0, maxY);
            addCoord(ys, maxY, 0.0, maxY);
            for (int pi = 0; pi < static_cast<int>(placed.size()); ++pi) {
                const Rect& o = placed[pi];
                int oid = (pi < static_cast<int>(placedIds.size())) ? placedIds[pi] : -1;
                const double gx = (oid >= 0) ? requiredXGap(design, oid, id) : PACK_GAP;
                const double gy = (oid >= 0) ? requiredYGap(design, oid, id) : PACK_GAP;

                // Normal bottom-left / contour candidates.
                addCoord(xs, o.x, 0.0, maxX);
                addCoord(xs, rectRight(o), 0.0, maxX);
                addCoord(xs, o.x - shape.w, 0.0, maxX);
                addCoord(xs, rectRight(o) - shape.w, 0.0, maxX);
                addCoord(ys, o.y, 0.0, maxY);
                addCoord(ys, rectTop(o), 0.0, maxY);
                addCoord(ys, o.y - shape.h, 0.0, maxY);
                addCoord(ys, rectTop(o) - shape.h, 0.0, maxY);

                // Routing-aware candidates.  These are the explicit requested form:
                //   B.x = A.x + A.w + requiredXGap(A, B)
                // plus the symmetric left/top/bottom versions.
                addCoord(xs, rectRight(o) + gx, 0.0, maxX);
                addCoord(xs, o.x - shape.w - gx, 0.0, maxX);
                addCoord(ys, rectTop(o) + gy, 0.0, maxY);
                addCoord(ys, o.y - shape.h - gy, 0.0, maxY);

                // Port-window / common-edge overlap candidates: if we decide to be
                // horizontally adjacent, align y-projections; if vertically adjacent,
                // align x-projections.
                addCoord(ys, o.y, 0.0, maxY);
                addCoord(ys, rectTop(o) - shape.h, 0.0, maxY);
                addCoord(ys, rectCy(o) - 0.5 * shape.h, 0.0, maxY);
                addCoord(xs, o.x, 0.0, maxX);
                addCoord(xs, rectRight(o) - shape.w, 0.0, maxX);
                addCoord(xs, rectCx(o) - 0.5 * shape.w, 0.0, maxX);
            }
            sort(xs.begin(), xs.end());
            xs.erase(unique(xs.begin(), xs.end(), [](double a, double b) { return fabs(a - b) < 1e-5; }), xs.end());
            sort(ys.begin(), ys.end());
            ys.erase(unique(ys.begin(), ys.end(), [](double a, double b) { return fabs(a - b) < 1e-5; }), ys.end());
            const int coordLimit = (static_cast<int>(design.blockSpecs.size()) <= 15) ? FAST_COORD_LIMIT_SMALL :
                (static_cast<int>(design.blockSpecs.size()) <= 35) ? FAST_COORD_LIMIT_MID :
                FAST_COORD_LIMIT_LARGE;
            pruneCoords(xs, coordLimit);
            pruneCoords(ys, coordLimit);

            bool found = false;
            Rect best = shape;
            PlaceKey bestKey;
            for (double y : ys) {
                for (double x : xs) {
                    Rect cand = shape;
                    cand.x = x;
                    cand.y = y;
                    if (!inside(cand, W, H)) continue;
                    if (anyOverlapWith(cand, placed)) continue;
                    PlaceKey key;
                    key.top = max(curTop, rectTop(cand));
                    key.route = routeGapPenaltyForCandidate(design, placed, placedIds, id, cand);
                    key.right = rectRight(cand);
                    key.y = y;
                    key.wire = partialWire(design, rects, placedMask, id, cand);
                    key.center = fabs(rectCx(cand) - 0.5 * W) + fabs(rectCy(cand) - 0.5 * H);
                    key.x = x;
                    fillPlacementPressure(key, placed, cand, W, H);
                    if (!found || betterKey(key, bestKey)) {
                        found = true;
                        bestKey = key;
                        best = cand;
                    }
                }
            }
            if (!found) return false;
            rects[id] = best;
            placed.push_back(best);
            placedIds.push_back(id);
            placedMask[id] = 1;
            curTop = max(curTop, rectTop(best));
        }

        outRects = std::move(rects);
        return true;
    }

    static double overlapAll(const vector<Rect>& r, int& cnt) {
        cnt = 0;
        double a = 0.0;
        for (int i = 0; i < static_cast<int>(r.size()); ++i) {
            for (int j = i + 1; j < static_cast<int>(r.size()); ++j) {
                double ov = ovArea(r[i], r[j]);
                if (ov > EPS) { a += ov; ++cnt; }
            }
        }
        return a;
    }

    struct LayoutResult {
        bool legal = false;
        bool strictEdgeLegal = false;
        double W = INF, H = INF, area = INF, hpwl = INF, score = INF;
        double overlap = 0.0, edgeViol = 0.0, outlineViol = 0.0, routePenalty = 0.0;
        double routeGapPenaltyPart = 0.0;
        double routePortPenaltyPart = 0.0;
        double routeMaxGapMiss = 0.0;
        double routeMaxPortMiss = 0.0;
        int routeConnectedPairs = 0;
        int routeViolPairs = 0;
        int routeXInterfaces = 0;
        int routeYInterfaces = 0;
        double stripOverflowProxy = 0.0;
        double stripRiskProxy = 0.0;
        double stripPeakUtilProxy = 0.0;
        int stripHotComponents = 0;
        vector<Rect> rects;
    };

    static LayoutResult scoreLayout(const Design& design, double W, double H, vector<Rect> rects) {
        LayoutResult r;
        r.W = W;
        r.H = H;
        r.area = W * H;
        r.rects = std::move(rects);
        int cnt = 0;
        r.overlap = overlapAll(r.rects, cnt);
        for (int i = 0; i < static_cast<int>(r.rects.size()); ++i) {
            const Rect& rc = r.rects[i];
            r.outlineViol += max(0.0, -rc.x) + max(0.0, -rc.y) + max(0.0, rectRight(rc) - W) + max(0.0, rectTop(rc) - H);
            r.edgeViol += edgeViolation(rc, design.blockSpecs[i], W, H);
        }
        r.hpwl = hpwl(design, r.rects);
        RouteGapBreakdown rb = routeGapBreakdownGlobal(design, r.rects);
        r.routePenalty = rb.penalty;
        r.routeGapPenaltyPart = rb.gapPenalty;
        r.routePortPenaltyPart = rb.portPenalty;
        r.routeMaxGapMiss = rb.maxGapMiss;
        r.routeMaxPortMiss = rb.maxPortMiss;
        r.routeConnectedPairs = rb.connectedPairs;
        r.routeViolPairs = rb.violPairs;
        r.routeXInterfaces = rb.xInterfaces;
        r.routeYInterfaces = rb.yInterfaces;
        double center = 0.0;
        for (const Rect& rc : r.rects) center += fabs(rectCx(rc) - 0.5 * W) + fabs(rectCy(rc) - 0.5 * H);
        r.strictEdgeLegal = r.edgeViol <= 1e-4;
        r.legal = cnt == 0 && r.overlap <= EPS && r.outlineViol <= EPS && W <= design.maxOutlineW + EPS && H <= design.maxOutlineH + EPS;
        r.score = r.area + HPWL_TIE_WEIGHT * r.hpwl + ROUTE_GAP_TIE_WEIGHT * r.routePenalty + CENTER_TIE_WEIGHT * center;
        if (!r.legal) r.score += 1.0e12 + 1.0e9 * r.overlap + 1.0e10 * r.outlineViol;
        if (!r.strictEdgeLegal) r.score += 1.0e8 * r.edgeViol;
        return r;
    }

    static bool betterLayout(const LayoutResult& a, const LayoutResult& b) {
        if (a.legal != b.legal) return a.legal;
        if (a.strictEdgeLegal != b.strictEdgeLegal) return a.strictEdgeLegal;
        if (a.legal && b.legal) {
            double tol = max(1.0, 0.001 * min(a.area, b.area));
            if (fabs(a.area - b.area) > tol) return a.area < b.area;
            if (fabs(a.routePenalty - b.routePenalty) > max(1.0, 0.02 * min(a.routePenalty, b.routePenalty))) {
                return a.routePenalty < b.routePenalty;
            }
        }
        return a.score < b.score;
    }

    static double intervalDistanceToPoint(double lo, double hi, double p) {
        if (p < lo) return lo - p;
        if (p > hi) return p - hi;
        return 0.0;
    }

    static Design makeTempDesignWithLayout(const Design& design, const LayoutResult& layout) {
        Design tmp = design;
        tmp.outlineW = clampD(layout.W, 1.0, design.maxOutlineW);
        tmp.outlineH = clampD(layout.H, 1.0, design.maxOutlineH);
        tmp.blocks.clear();
        tmp.blocks.reserve(design.blockSpecs.size());
        tmp.blockNameToIndex.clear();
        for (int i = 0; i < static_cast<int>(design.blockSpecs.size()); ++i) {
            BlockInst b;
            b.spec = design.blockSpecs[i];
            if (i < static_cast<int>(layout.rects.size())) b.rect = layout.rects[i];
            tmp.blockNameToIndex[b.spec.name] = i;
            tmp.blocks.push_back(b);
        }
        tmp.channels.clear();
        tmp.routes.clear();
        ChannelBuilder builder;
        builder.build(tmp);
        return tmp;
    }

    static void annotateStripProxy(const Design& design, LayoutResult& layout) {
        layout.stripOverflowProxy = 0.0;
        layout.stripRiskProxy = 0.0;
        layout.stripPeakUtilProxy = 0.0;
        layout.stripHotComponents = 0;
        if (!layout.legal || layout.rects.empty()) return;

        Design tmp = makeTempDesignWithLayout(design, layout);
        const int chN = static_cast<int>(tmp.channels.size());
        if (chN <= 0) return;

        vector<double> lrUse(chN, 0.0), tbUse(chN, 0.0);
        auto addDemand = [&](int src, int dst, int nets) {
            if (nets <= 0 || src < 0 || dst < 0 ||
                src >= static_cast<int>(layout.rects.size()) ||
                dst >= static_cast<int>(layout.rects.size())) return;

            const Rect& a = layout.rects[src];
            const Rect& b = layout.rects[dst];
            const double ax = rectCx(a), ay = rectCy(a);
            const double bx = rectCx(b), by = rectCy(b);
            const double minX = min(ax, bx), maxX = max(ax, bx);
            const double minY = min(ay, by), maxY = max(ay, by);
            const double spanX = max(1.0, maxX - minX);
            const double spanY = max(1.0, maxY - minY);
            const double midX = 0.5 * (ax + bx);
            const double midY = 0.5 * (ay + by);
            const double bandX = max(12.0, 0.08 * layout.W + 0.18 * spanX);
            const double bandY = max(12.0, 0.08 * layout.H + 0.18 * spanY);

            const double demand = 12.0 * max(estimatedDirectFlowNets(design, src, dst, nets), 0.72 * static_cast<double>(nets));
            if (demand <= EPS) return;
            int bestH = -1, bestV = -1;
            double bestHW = 0.0, bestVW = 0.0;
            for (int ci = 0; ci < chN; ++ci) {
                const Channel& ch = tmp.channels[ci];
                const double chR = rectRight(ch.rect);
                const double chT = rectTop(ch.rect);
                const double ox = overlapLen(ch.rect.x, chR, minX, maxX) / spanX;
                const double oy = overlapLen(ch.rect.y, chT, minY, maxY) / spanY;
                const double dy = intervalDistanceToPoint(ch.rect.y, chT, midY);
                const double dx = intervalDistanceToPoint(ch.rect.x, chR, midX);
                const double hWeight = (0.15 + 0.85 * clampD(ox, 0.0, 1.0)) * exp(-dy / bandY);
                const double vWeight = (0.15 + 0.85 * clampD(oy, 0.0, 1.0)) * exp(-dx / bandX);
                if (hWeight > 1.0e-4) lrUse[ci] += demand * hWeight;
                if (vWeight > 1.0e-4) tbUse[ci] += demand * vWeight;
                if (hWeight > bestHW) { bestHW = hWeight; bestH = ci; }
                if (vWeight > bestVW) { bestVW = vWeight; bestV = ci; }
            }
            if (bestH >= 0 && bestHW > 1.0e-4) lrUse[bestH] += 0.65 * demand;
            if (bestV >= 0 && bestVW > 1.0e-4) tbUse[bestV] += 0.65 * demand;
            };

        if (!design.connections.empty()) {
            for (const auto& c : design.connections) addDemand(c.src, c.dst, max(0, c.netCount));
        }
        else {
            const int n = static_cast<int>(design.blockSpecs.size());
            for (int i = 0; i < n; ++i) {
                for (int j = i + 1; j < n; ++j) addDemand(i, j, totalConnBetween(design, i, j));
            }
        }

        for (int ci = 0; ci < chN; ++ci) {
            const Channel& ch = tmp.channels[ci];
            const double capLR = max(1.0, ch.rect.h * ROUTE_DENSITY);
            const double capTB = max(1.0, ch.rect.w * ROUTE_DENSITY);
            const double utilLR = lrUse[ci] / capLR;
            const double utilTB = tbUse[ci] / capTB;
            layout.stripPeakUtilProxy = max(layout.stripPeakUtilProxy, max(utilLR, utilTB));

            auto addComponent = [&](double util, double cap) {
                const double over = max(0.0, util - STRIP_PROXY_TARGET_UTIL);
                const double risk = max(0.0, util - STRIP_PROXY_FREE_UTIL);
                layout.stripOverflowProxy += over * over * cap;
                layout.stripRiskProxy += risk * risk * cap;
                if (util > STRIP_PROXY_TARGET_UTIL) ++layout.stripHotComponents;
                };
            addComponent(utilLR, capLR);
            addComponent(utilTB, capTB);
        }
    }

    static double archiveProxyScore(const LayoutResult& r) {
        const double peak = max(0.0, r.stripPeakUtilProxy - STRIP_PROXY_TARGET_UTIL);
        return r.area
            + HPWL_TIE_WEIGHT * r.hpwl
            + ROUTE_GAP_TIE_WEIGHT * r.routePenalty
            + STRIP_PROXY_OVERFLOW_WEIGHT * r.stripOverflowProxy
            + STRIP_PROXY_RISK_WEIGHT * r.stripRiskProxy
            + STRIP_PROXY_PEAK_WEIGHT * peak * peak * max(1.0, r.area);
    }

    static double bstarCheckerProxyCost(const Design& design, const LayoutResult& r);
    static bool bstarCheckerProxyBetter(const Design& design, const LayoutResult& a, const LayoutResult& b);
    static double bstarWireProxyCost(const Design& design, const LayoutResult& r, bool spreadMode);
    static bool bstarWireProxyBetter(const Design& design, const LayoutResult& a, const LayoutResult& b, bool spreadMode);

    static bool betterArchiveLayout(const LayoutResult& a, const LayoutResult& b) {
        if (a.legal != b.legal) return a.legal;
        if (a.strictEdgeLegal != b.strictEdgeLegal) return a.strictEdgeLegal;
        if (!std::isfinite(b.area) || b.area >= INF * 0.5) return true;
        const double minArea = min(a.area, b.area);
        const double areaTol = max(1.0, BSTAR_ARCHIVE_AREA_WINDOW * minArea);
        if (fabs(a.area - b.area) > areaTol) return a.area < b.area;
        const double as = archiveProxyScore(a);
        const double bs = archiveProxyScore(b);
        if (fabs(as - bs) > max(1.0, 1.0e-6 * min(as, bs))) return as < bs;
        return a.area < b.area;
    }

    static void addToBStarArchive(const Design& design, vector<LayoutResult>& archive, const LayoutResult& cand) {
        if (!cand.legal || !cand.strictEdgeLegal) return;
        LayoutResult item = cand;
        annotateStripProxy(design, item);
        for (const auto& old : archive) {
            const double areaTol = max(1.0, 0.0005 * min(old.area, item.area));
            if (fabs(old.area - item.area) <= areaTol &&
                fabs(old.W - item.W) <= 1.0 &&
                fabs(old.H - item.H) <= 1.0 &&
                fabs(old.stripPeakUtilProxy - item.stripPeakUtilProxy) <= 0.01) {
                return;
            }
        }
        archive.push_back(std::move(item));
        sort(archive.begin(), archive.end(), betterArchiveLayout);
        if (static_cast<int>(archive.size()) > BSTAR_ARCHIVE_LIMIT) archive.resize(BSTAR_ARCHIVE_LIMIT);
    }

    static void addToBStarSecondArchive(const Design& design, vector<LayoutResult>& archive, const LayoutResult& cand) {
        if (!cand.legal || !cand.strictEdgeLegal) return;
        LayoutResult item = cand;
        annotateStripProxy(design, item);
        for (const auto& old : archive) {
            const double areaTol = max(1.0, 0.0005 * min(old.area, item.area));
            if (fabs(old.area - item.area) <= areaTol &&
                fabs(old.W - item.W) <= 1.0 &&
                fabs(old.H - item.H) <= 1.0 &&
                fabs(old.stripPeakUtilProxy - item.stripPeakUtilProxy) <= 0.01) {
                return;
            }
        }
        archive.push_back(std::move(item));
        sort(archive.begin(), archive.end(), [&](const LayoutResult& a, const LayoutResult& b) {
            return bstarCheckerProxyBetter(design, a, b);
            });
        if (static_cast<int>(archive.size()) > BSTAR_SECOND_ARCHIVE_LIMIT) {
            archive.resize(BSTAR_SECOND_ARCHIVE_LIMIT);
        }
    }

    static void addToBStarWireArchive(const Design& design, vector<LayoutResult>& archive, const LayoutResult& cand, bool spreadMode) {
        if (!cand.legal || !cand.strictEdgeLegal) return;
        if (cand.routeViolPairs > (spreadMode ? 1 : 0)) return;
        LayoutResult item = cand;
        annotateStripProxy(design, item);
        for (const auto& old : archive) {
            const double areaTol = max(1.0, 0.0005 * min(old.area, item.area));
            if (fabs(old.area - item.area) <= areaTol &&
                fabs(old.W - item.W) <= 1.0 &&
                fabs(old.H - item.H) <= 1.0 &&
                fabs(old.hpwl - item.hpwl) <= max(1.0, 0.0007 * min(old.hpwl, item.hpwl))) {
                return;
            }
        }
        archive.push_back(std::move(item));
        sort(archive.begin(), archive.end(), [&](const LayoutResult& a, const LayoutResult& b) {
            return bstarWireProxyBetter(design, a, b, spreadMode);
            });
        const int limit = spreadMode ? BSTAR_SPREAD_ARCHIVE_LIMIT : BSTAR_WIRE_ARCHIVE_LIMIT;
        if (static_cast<int>(archive.size()) > limit) archive.resize(limit);
    }

    static vector<int> legalPortEdgesForForce(const BlockSpec& spec) {
        if (!spec.portEdges.empty()) return spec.portEdges;
        return { 1, 2, 3, 4 };
    }

    static pair<double, double> forceEdgeAnchor(const Rect& r, int edge, double t) {
        t = clampD(t, 0.0, 1.0);
        if (edge == 1) return { r.x, r.y + t * r.h };
        if (edge == 3) return { rectRight(r), r.y + t * r.h };
        if (edge == 2) return { r.x + t * r.w, rectTop(r) };
        return { r.x + t * r.w, r.y };
    }

    static double bestPortAwareDelta(
        const Design& design,
        const vector<Rect>& rects,
        int a,
        int b,
        double& dx,
        double& dy
    ) {
        dx = rectCx(rects[b]) - rectCx(rects[a]);
        dy = rectCy(rects[b]) - rectCy(rects[a]);
        if (a < 0 || b < 0 || a >= static_cast<int>(rects.size()) || b >= static_cast<int>(rects.size())) {
            return fabs(dx) + fabs(dy);
        }
        const vector<int> ea = legalPortEdgesForForce(design.blockSpecs[a]);
        const vector<int> eb = legalPortEdgesForForce(design.blockSpecs[b]);
        const double ts[3] = { 0.25, 0.50, 0.75 };
        double best = INF;
        for (int pa : ea) {
            if (pa < 1 || pa > 4) continue;
            for (int pb : eb) {
                if (pb < 1 || pb > 4) continue;
                for (double ta : ts) {
                    const auto aa = forceEdgeAnchor(rects[a], pa, ta);
                    for (double tb : ts) {
                        const auto bb = forceEdgeAnchor(rects[b], pb, tb);
                        const double ddx = bb.first - aa.first;
                        const double ddy = bb.second - aa.second;
                        const double d = fabs(ddx) + fabs(ddy);
                        if (d < best) {
                            best = d;
                            dx = ddx;
                            dy = ddy;
                        }
                    }
                }
            }
        }
        return best;
    }

    static double portAwareHpwl(const Design& design, const vector<Rect>& rects) {
        double s = 0.0;
        auto addPair = [&](int a, int b, int nets) {
            if (nets <= 0 || a < 0 || b < 0 ||
                a >= static_cast<int>(rects.size()) ||
                b >= static_cast<int>(rects.size())) return;
            double dx = 0.0, dy = 0.0;
            s += static_cast<double>(nets) * bestPortAwareDelta(design, rects, a, b, dx, dy);
            };

        if (!design.connections.empty()) {
            for (const auto& c : design.connections) addPair(c.src, c.dst, max(0, c.netCount));
        }
        else {
            const int n = min(static_cast<int>(rects.size()), static_cast<int>(design.connMatrix.size()));
            for (int i = 0; i < n; ++i) {
                for (int j = i + 1; j < n; ++j) addPair(i, j, totalConnBetween(design, i, j));
            }
        }
        return s;
    }

    struct ProxyDirUse {
        double lr = 0.0;
        double tb = 0.0;
    };

    struct RouteProxy {
        double guidePointWL = 0.0;
        double maxDirectionalUtil = 0.0;
        double channelRisk = 0.0;
        double ftRisk = 0.0;
        int disconnectedPairs = 0;
        int routedPairs = 0;
    };

    struct ProxyNode {
        string name;
        Rect rect;
        bool isBlock = false;
        int index = -1;
    };

    struct ProxyAdj {
        int to = -1;
        int edgeFrom = 0;
        int edgeTo = 0;
        double baseCost = 0.0;
    };

    struct ProxyPath {
        bool open = true;
        double guideWL = 0.0;
        vector<int> states;
        vector<int> transOut;
        vector<pair<double, double>> contacts;
    };

    static bool validProxyEdge(int e) {
        return e >= 1 && e <= 4;
    }

    static bool proxyAllowsPortEdge(const BlockSpec& spec, int edge) {
        if (!validProxyEdge(edge)) return false;
        if (spec.portEdges.empty()) return true;
        return find(spec.portEdges.begin(), spec.portEdges.end(), edge) != spec.portEdges.end();
    }

    static bool proxyOppositeLR(int a, int b) {
        return (a == 1 && b == 3) || (a == 3 && b == 1);
    }

    static bool proxyOppositeTB(int a, int b) {
        return (a == 2 && b == 4) || (a == 4 && b == 2);
    }

    static bool proxyTurn(int a, int b) {
        return validProxyEdge(a) && validProxyEdge(b) && a != b &&
            !proxyOppositeLR(a, b) && !proxyOppositeTB(a, b);
    }

    static ProxyDirUse proxyDeltaForTraversal(int inEdge, int outEdge, double nets) {
        ProxyDirUse d;
        if (!validProxyEdge(inEdge) || !validProxyEdge(outEdge) || inEdge == outEdge) return d;
        if (proxyOppositeLR(inEdge, outEdge)) d.lr += nets;
        else if (proxyOppositeTB(inEdge, outEdge)) d.tb += nets;
        else if (proxyTurn(inEdge, outEdge)) {
            d.lr += nets;
            d.tb += nets;
        }
        return d;
    }

    static bool proxyTouchWithEdges(const Rect& a, const Rect& b, int& edgeA, int& edgeB) {
        static constexpr double TOUCH_EPS_LOCAL = 1.0e-3;
        static constexpr double TOUCH_OVERLAP_EPS_LOCAL = 1.0e-7;
        if (fabs(rectRight(a) - b.x) <= TOUCH_EPS_LOCAL &&
            overlapLen(a.y, rectTop(a), b.y, rectTop(b)) > TOUCH_OVERLAP_EPS_LOCAL) {
            edgeA = 3; edgeB = 1; return true;
        }
        if (fabs(a.x - rectRight(b)) <= TOUCH_EPS_LOCAL &&
            overlapLen(a.y, rectTop(a), b.y, rectTop(b)) > TOUCH_OVERLAP_EPS_LOCAL) {
            edgeA = 1; edgeB = 3; return true;
        }
        if (fabs(rectTop(a) - b.y) <= TOUCH_EPS_LOCAL &&
            overlapLen(a.x, rectRight(a), b.x, rectRight(b)) > TOUCH_OVERLAP_EPS_LOCAL) {
            edgeA = 2; edgeB = 4; return true;
        }
        if (fabs(a.y - rectTop(b)) <= TOUCH_EPS_LOCAL &&
            overlapLen(a.x, rectRight(a), b.x, rectRight(b)) > TOUCH_OVERLAP_EPS_LOCAL) {
            edgeA = 4; edgeB = 2; return true;
        }
        return false;
    }

    static bool proxyContactPoint(const Rect& a, int ea, const Rect& b, int eb, pair<double, double>& p) {
        if (!validProxyEdge(ea) || !validProxyEdge(eb)) return false;
        if (ea == 3 && eb == 1 && fabs(rectRight(a) - b.x) <= 1.0e-3) {
            const double lo = max(a.y, b.y);
            const double hi = min(rectTop(a), rectTop(b));
            if (hi <= lo + 1.0e-7) return false;
            p = { 0.5 * (rectRight(a) + b.x), 0.5 * (lo + hi) };
            return true;
        }
        if (ea == 1 && eb == 3 && fabs(a.x - rectRight(b)) <= 1.0e-3) {
            const double lo = max(a.y, b.y);
            const double hi = min(rectTop(a), rectTop(b));
            if (hi <= lo + 1.0e-7) return false;
            p = { 0.5 * (a.x + rectRight(b)), 0.5 * (lo + hi) };
            return true;
        }
        if (ea == 2 && eb == 4 && fabs(rectTop(a) - b.y) <= 1.0e-3) {
            const double lo = max(a.x, b.x);
            const double hi = min(rectRight(a), rectRight(b));
            if (hi <= lo + 1.0e-7) return false;
            p = { 0.5 * (lo + hi), 0.5 * (rectTop(a) + b.y) };
            return true;
        }
        if (ea == 4 && eb == 2 && fabs(a.y - rectTop(b)) <= 1.0e-3) {
            const double lo = max(a.x, b.x);
            const double hi = min(rectRight(a), rectRight(b));
            if (hi <= lo + 1.0e-7) return false;
            p = { 0.5 * (lo + hi), 0.5 * (a.y + rectTop(b)) };
            return true;
        }
        return false;
    }

    static pair<double, double> proxyContactOrEdgePoint(const ProxyNode& a, int ea, const ProxyNode& b, int eb) {
        pair<double, double> gp;
        if (proxyContactPoint(a.rect, ea, b.rect, eb, gp)) return gp;
        auto pa = edgeCenterPoint(a.rect, ea);
        auto pb = edgeCenterPoint(b.rect, eb);
        return { 0.5 * (pa.first + pb.first), 0.5 * (pa.second + pb.second) };
    }

    static bool proxyNodeAllowedIntermediate(const Design& tmp, const ProxyNode& node) {
        if (!node.isBlock) return true;
        if (node.index < 0 || node.index >= static_cast<int>(tmp.blocks.size())) return false;
        return tmp.blocks[node.index].spec.type == BlockType::SOFT;
    }

    static double proxyCapLR(const Channel& ch) {
        return max(1.0, ch.rect.h * ROUTE_DENSITY);
    }

    static double proxyCapTB(const Channel& ch) {
        return max(1.0, ch.rect.w * ROUTE_DENSITY);
    }

    static double proxyFTRequiredArea(const BlockInst& b, double ftNets) {
        const double baseArea = max(1.0, b.spec.area);
        double aspect = b.rect.h > EPS ? b.rect.w / b.rect.h : 1.0;
        if (b.spec.aspectMin > EPS) aspect = max(aspect, b.spec.aspectMin);
        if (b.spec.aspectMax > EPS) aspect = min(aspect, b.spec.aspectMax);
        if (!std::isfinite(aspect) || aspect <= EPS) aspect = 1.0;

        const double coreW = sqrt(baseArea * aspect);
        const double coreH = baseArea / max(1.0, coreW);
        double d = 0.0;
        if (b.spec.type == BlockType::SOFT && ftNets > EPS) {
            double rate = b.spec.ftRate[3];
            if (ftNets <= 3000.0) rate = b.spec.ftRate[0];
            else if (ftNets <= 6000.0) rate = b.spec.ftRate[1];
            else if (ftNets <= 9000.0) rate = b.spec.ftRate[2];
            d = ceil((ftNets / ROUTE_DENSITY) * rate) / 2.0;
        }
        return max(baseArea, (coreW + d) * (coreH + d));
    }

    static double proxyIncrementalFTCost(const BlockInst& b, double oldFt, double addFt) {
        if (b.spec.type != BlockType::SOFT || addFt <= EPS) return INF;
        const double curArea = max(1.0, b.rect.w * b.rect.h);
        const double oldReq = proxyFTRequiredArea(b, oldFt);
        const double newReq = proxyFTRequiredArea(b, oldFt + addFt);
        const double incArea = max(0.0, newReq - oldReq);
        const double oldOv = max(0.0, oldReq - curArea);
        const double newOv = max(0.0, newReq - curArea);
        return 8.0 * incArea + 220.0 * max(0.0, newOv - oldOv) + 0.35 * addFt;
    }

    static vector<ProxyNode> buildProxyNodes(const Design& tmp) {
        vector<ProxyNode> nodes;
        nodes.reserve(tmp.blocks.size() + tmp.channels.size());
        for (int i = 0; i < static_cast<int>(tmp.blocks.size()); ++i) {
            nodes.push_back({ tmp.blocks[i].spec.name, tmp.blocks[i].rect, true, i });
        }
        for (int i = 0; i < static_cast<int>(tmp.channels.size()); ++i) {
            nodes.push_back({ tmp.channels[i].name, tmp.channels[i].rect, false, i });
        }
        return nodes;
    }

    static vector<vector<ProxyAdj>> buildProxyGraph(const vector<ProxyNode>& nodes) {
        const int n = static_cast<int>(nodes.size());
        vector<vector<ProxyAdj>> g(n);
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                int ei = 0, ej = 0;
                if (!proxyTouchWithEdges(nodes[i].rect, nodes[j].rect, ei, ej)) continue;
                const double base = manhattan(rectCx(nodes[i].rect), rectCy(nodes[i].rect),
                    rectCx(nodes[j].rect), rectCy(nodes[j].rect));
                g[i].push_back({ j, ei, ej, base });
                g[j].push_back({ i, ej, ei, base });
            }
        }
        return g;
    }

    static double proxyChannelComponentCost(double used, double cap, double delta) {
        if (delta <= EPS) return 0.0;
        cap = max(1.0, cap);
        const double before = used / cap;
        const double after = (used + delta) / cap;
        const double softRisk = max(0.0, after - 0.72);
        const double hardRisk = max(0.0, after - 0.96);
        const double overflow = max(0.0, used + delta - cap);
        return 0.18 * delta + 900.0 * delta * softRisk * softRisk +
            9000.0 * delta * hardRisk * hardRisk + 3500.0 * overflow * overflow / cap;
    }

    static ProxyPath routeProxyConnection(
        const Design& tmp,
        const vector<ProxyNode>& nodes,
        const vector<vector<ProxyAdj>>& g,
        const Connection& conn,
        const vector<ProxyDirUse>& channelUse,
        const vector<double>& softFT
    ) {
        ProxyPath result;
        if (conn.src < 0 || conn.dst < 0 ||
            conn.src >= static_cast<int>(tmp.blocks.size()) ||
            conn.dst >= static_cast<int>(tmp.blocks.size()) ||
            conn.src >= static_cast<int>(nodes.size()) ||
            conn.dst >= static_cast<int>(nodes.size()) ||
            conn.netCount <= 0) {
            return result;
        }

        const int nodeCount = static_cast<int>(nodes.size());
        const int edgeStates = 5;
        const int stateCount = nodeCount * edgeStates;
        auto sid = [&](int node, int inEdge) { return node * edgeStates + inEdge; };
        auto sNode = [&](int st) { return st / edgeStates; };
        auto sEdge = [&](int st) { return st % edgeStates; };

        vector<double> dist(stateCount, INF);
        vector<int> parent(stateCount, -1);
        vector<int> transOut(stateCount, 0);
        vector<int> transIn(stateCount, 0);

        using QItem = pair<double, int>;
        priority_queue<QItem, vector<QItem>, greater<QItem>> pq;
        const int start = sid(conn.src, 0);
        dist[start] = 0.0;
        pq.push({ 0.0, start });

        int bestDst = -1;
        while (!pq.empty()) {
            const auto item = pq.top();
            pq.pop();
            const double queued = item.first;
            const int st = item.second;
            if (queued > dist[st] + 1.0e-9) continue;

            const int u = sNode(st);
            const int uIn = sEdge(st);
            if (u == conn.dst) {
                bestDst = st;
                break;
            }

            for (const ProxyAdj& e : g[u]) {
                const int v = e.to;
                const bool vEndpoint = (v == conn.src || v == conn.dst);
                if (!vEndpoint && !proxyNodeAllowedIntermediate(tmp, nodes[v])) continue;
                if (u == conn.src && !proxyAllowsPortEdge(tmp.blocks[conn.src].spec, e.edgeFrom)) continue;
                if (v == conn.dst && !proxyAllowsPortEdge(tmp.blocks[conn.dst].spec, e.edgeTo)) continue;

                double resourceCost = 0.0;
                double internalWire = 0.0;
                if (u != conn.src && u != conn.dst) {
                    if (!validProxyEdge(uIn) || uIn == e.edgeFrom) continue;
                    const auto p = edgeCenterPoint(nodes[u].rect, uIn);
                    const auto q = edgeCenterPoint(nodes[u].rect, e.edgeFrom);
                    internalWire = manhattan(p.first, p.second, q.first, q.second);

                    if (!nodes[u].isBlock) {
                        const int ci = nodes[u].index;
                        if (ci < 0 || ci >= static_cast<int>(tmp.channels.size()) ||
                            ci >= static_cast<int>(channelUse.size())) continue;
                        const Channel& ch = tmp.channels[ci];
                        const ProxyDirUse d = proxyDeltaForTraversal(uIn, e.edgeFrom, static_cast<double>(conn.netCount));
                        resourceCost += proxyChannelComponentCost(channelUse[ci].lr, proxyCapLR(ch), d.lr);
                        resourceCost += proxyChannelComponentCost(channelUse[ci].tb, proxyCapTB(ch), d.tb);
                    }
                    else {
                        const int bi = nodes[u].index;
                        if (bi < 0 || bi >= static_cast<int>(tmp.blocks.size()) ||
                            bi >= static_cast<int>(softFT.size()) ||
                            tmp.blocks[bi].spec.type != BlockType::SOFT) continue;
                        resourceCost += proxyIncrementalFTCost(tmp.blocks[bi], softFT[bi], static_cast<double>(conn.netCount));
                        if (!std::isfinite(resourceCost)) continue;
                    }
                }

                const double contactCost = 0.012 * e.baseCost * static_cast<double>(conn.netCount);
                const double wireCost = 0.20 * internalWire * static_cast<double>(conn.netCount) + contactCost;
                const int next = sid(v, e.edgeTo);
                const double nd = dist[st] + wireCost + resourceCost;
                if (nd + 1.0e-9 < dist[next]) {
                    dist[next] = nd;
                    parent[next] = st;
                    transOut[next] = e.edgeFrom;
                    transIn[next] = e.edgeTo;
                    pq.push({ nd, next });
                }
            }
        }

        if (bestDst < 0) return result;
        vector<int> states;
        for (int st = bestDst; st >= 0; st = parent[st]) states.push_back(st);
        reverse(states.begin(), states.end());

        vector<pair<double, double>> contacts;
        for (int k = 1; k < static_cast<int>(states.size()); ++k) {
            const int prev = states[k - 1];
            const int cur = states[k];
            const int u = sNode(prev);
            const int v = sNode(cur);
            contacts.push_back(proxyContactOrEdgePoint(nodes[u], transOut[cur], nodes[v], transIn[cur]));
        }

        double wlOne = 0.0;
        for (int k = 0; k + 1 < static_cast<int>(contacts.size()); ++k) {
            wlOne += manhattan(contacts[k].first, contacts[k].second, contacts[k + 1].first, contacts[k + 1].second);
        }

        result.open = false;
        result.guideWL = wlOne * static_cast<double>(conn.netCount);
        result.states = std::move(states);
        result.transOut = std::move(transOut);
        result.contacts = std::move(contacts);
        return result;
    }

    static void commitProxyPathUse(
        const Design& tmp,
        const vector<ProxyNode>& nodes,
        const Connection& conn,
        const ProxyPath& path,
        vector<ProxyDirUse>& channelUse,
        vector<double>& softFT
    ) {
        if (path.open || path.states.size() < 3) return;
        const int edgeStates = 5;
        auto sNode = [&](int st) { return st / edgeStates; };
        auto sEdge = [&](int st) { return st % edgeStates; };
        const double nets = static_cast<double>(conn.netCount);
        for (int k = 1; k + 1 < static_cast<int>(path.states.size()); ++k) {
            const int cur = path.states[k];
            const int next = path.states[k + 1];
            const int u = sNode(cur);
            if (u < 0 || u >= static_cast<int>(nodes.size())) continue;
            if (u == conn.src || u == conn.dst) continue;
            if (!nodes[u].isBlock) {
                const int ci = nodes[u].index;
                if (ci < 0 || ci >= static_cast<int>(channelUse.size())) continue;
                const ProxyDirUse d = proxyDeltaForTraversal(sEdge(cur), path.transOut[next], nets);
                channelUse[ci].lr += d.lr;
                channelUse[ci].tb += d.tb;
            }
            else {
                const int bi = nodes[u].index;
                if (bi >= 0 && bi < static_cast<int>(softFT.size()) &&
                    bi < static_cast<int>(tmp.blocks.size()) &&
                    tmp.blocks[bi].spec.type == BlockType::SOFT) {
                    softFT[bi] += nets;
                }
            }
        }
    }

    static vector<Connection> collectProxyConnections(const Design& design) {
        vector<Connection> conns;
        if (!design.connections.empty()) {
            for (const Connection& c : design.connections) {
                if (c.netCount > 0) conns.push_back(c);
            }
        }
        else {
            const int n = static_cast<int>(design.connMatrix.size());
            for (int i = 0; i < n; ++i) {
                for (int j = i + 1; j < static_cast<int>(design.connMatrix[i].size()); ++j) {
                    const int nets = totalConnBetween(design, i, j);
                    if (nets > 0) conns.push_back({ i, j, nets });
                }
            }
        }
        sort(conns.begin(), conns.end(), [](const Connection& a, const Connection& b) {
            if (a.netCount != b.netCount) return a.netCount > b.netCount;
            if (a.src != b.src) return a.src < b.src;
            return a.dst < b.dst;
            });
        return conns;
    }

    static RouteProxy estimateRouteProxy(const Design& design, const LayoutResult& layout) {
        RouteProxy proxy;
        if (!layout.legal || !layout.strictEdgeLegal || layout.rects.empty()) {
            proxy.disconnectedPairs = 1000000;
            proxy.channelRisk = INF * 0.25;
            proxy.ftRisk = INF * 0.25;
            proxy.guidePointWL = INF * 0.25;
            return proxy;
        }

        Design tmp = makeTempDesignWithLayout(design, layout);
        vector<ProxyNode> nodes = buildProxyNodes(tmp);
        vector<vector<ProxyAdj>> graph = buildProxyGraph(nodes);
        vector<ProxyDirUse> channelUse(tmp.channels.size());
        vector<double> softFT(tmp.blocks.size(), 0.0);
        vector<Connection> conns = collectProxyConnections(design);

        for (const Connection& conn : conns) {
            ProxyPath p = routeProxyConnection(tmp, nodes, graph, conn, channelUse, softFT);
            if (p.open) {
                ++proxy.disconnectedPairs;
                double dx = 0.0, dy = 0.0;
                proxy.guidePointWL += 4.0 * static_cast<double>(max(0, conn.netCount)) *
                    max(1.0, bestPortAwareDelta(design, layout.rects, conn.src, conn.dst, dx, dy));
                proxy.channelRisk += 5.0e8 + 5000.0 * static_cast<double>(max(0, conn.netCount));
                continue;
            }
            ++proxy.routedPairs;
            proxy.guidePointWL += p.guideWL;
            commitProxyPathUse(tmp, nodes, conn, p, channelUse, softFT);
        }

        for (int ci = 0; ci < static_cast<int>(tmp.channels.size()) && ci < static_cast<int>(channelUse.size()); ++ci) {
            const Channel& ch = tmp.channels[ci];
            const double capLR = proxyCapLR(ch);
            const double capTB = proxyCapTB(ch);
            const double utilLR = channelUse[ci].lr / capLR;
            const double utilTB = channelUse[ci].tb / capTB;
            proxy.maxDirectionalUtil = max(proxy.maxDirectionalUtil, max(utilLR, utilTB));
            auto addRisk = [&](double util, double cap) {
                const double soft = max(0.0, util - 0.72);
                const double target = max(0.0, util - 0.94);
                const double hard = max(0.0, util - 1.00);
                proxy.channelRisk += soft * soft * cap +
                    12.0 * target * target * cap +
                    90.0 * hard * hard * cap;
                };
            addRisk(utilLR, capLR);
            addRisk(utilTB, capTB);
        }

        for (int bi = 0; bi < static_cast<int>(tmp.blocks.size()) && bi < static_cast<int>(softFT.size()); ++bi) {
            const BlockInst& b = tmp.blocks[bi];
            if (b.spec.type != BlockType::SOFT || softFT[bi] <= EPS) continue;
            const double curArea = max(1.0, b.rect.w * b.rect.h);
            const double reqArea = proxyFTRequiredArea(b, softFT[bi]);
            const double overflow = max(0.0, reqArea - curArea);
            proxy.ftRisk += overflow + 0.015 * softFT[bi] * max(0.0, reqArea / curArea - 1.0);
        }
        return proxy;
    }

    struct ForcePair {
        int a = -1;
        int b = -1;
        int nets = 0;
        bool hot = false;
    };

    static vector<ForcePair> collectForcePairs(const Design& design) {
        const int n = static_cast<int>(design.blockSpecs.size());
        vector<vector<int>> w(n, vector<int>(n, 0));
        if (!design.connections.empty()) {
            for (const auto& c : design.connections) {
                if (c.src < 0 || c.dst < 0 || c.src >= n || c.dst >= n || c.src == c.dst) continue;
                int a = min(c.src, c.dst);
                int b = max(c.src, c.dst);
                w[a][b] += max(0, c.netCount);
            }
        }
        else {
            for (int i = 0; i < n; ++i) {
                for (int j = i + 1; j < n; ++j) w[i][j] = totalConnBetween(design, i, j);
            }
        }

        vector<ForcePair> pairs;
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                if (w[i][j] > 0) pairs.push_back({ i, j, w[i][j], false });
            }
        }
        sort(pairs.begin(), pairs.end(), [](const ForcePair& a, const ForcePair& b) {
            if (a.nets != b.nets) return a.nets > b.nets;
            if (a.a != b.a) return a.a < b.a;
            return a.b < b.b;
            });
        const int hotCount = max(1, static_cast<int>(ceil(FORCE_HOT_PAIR_FRAC * static_cast<double>(pairs.size()))));
        for (int i = 0; i < static_cast<int>(pairs.size()) && i < hotCount; ++i) pairs[i].hot = true;
        return pairs;
    }

    static bool forceMovableSoft(const Design& design, int id) {
        return id >= 0 && id < static_cast<int>(design.blockSpecs.size()) &&
            design.blockSpecs[id].type == BlockType::SOFT;
    }

    struct SepConstraint {
        int lo = -1;
        int hi = -1;
        bool xAxis = true;
        double gap = PACK_GAP;
    };

    static vector<SepConstraint> buildSoftProjectionConstraints(const Design& design, const vector<Rect>& ref) {
        vector<SepConstraint> out;
        const int n = static_cast<int>(ref.size());
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                if (!forceMovableSoft(design, i) && !forceMovableSoft(design, j)) continue;

                const Rect& a = ref[i];
                const Rect& b = ref[j];
                const bool iLeft = rectRight(a) <= b.x + EPS;
                const bool jLeft = rectRight(b) <= a.x + EPS;
                const bool iBelow = rectTop(a) <= b.y + EPS;
                const bool jBelow = rectTop(b) <= a.y + EPS;
                if (!iLeft && !jLeft && !iBelow && !jBelow) continue;

                const int nets = totalConnBetween(design, i, j);
                const double xGap = iLeft ? max(0.0, b.x - rectRight(a)) :
                    (jLeft ? max(0.0, a.x - rectRight(b)) : INF);
                const double yGap = iBelow ? max(0.0, b.y - rectTop(a)) :
                    (jBelow ? max(0.0, a.y - rectTop(b)) : INF);
                const bool useX = (iLeft || jLeft) && (!(iBelow || jBelow) || xGap <= yGap);

                SepConstraint c;
                c.xAxis = useX;
                if (useX) {
                    c.lo = iLeft ? i : j;
                    c.hi = iLeft ? j : i;
                    c.gap = nets > 0 ? requiredXGap(design, i, j) : PACK_GAP;
                }
                else {
                    c.lo = iBelow ? i : j;
                    c.hi = iBelow ? j : i;
                    c.gap = nets > 0 ? requiredYGap(design, i, j) : PACK_GAP;
                }
                out.push_back(c);
            }
        }
        return out;
    }

    static void clampSoftInsideOutline(const Design& design, vector<Rect>& r, double W, double H) {
        for (int i = 0; i < static_cast<int>(r.size()); ++i) {
            if (!forceMovableSoft(design, i)) continue;
            r[i].x = clampD(r[i].x, 0.0, max(0.0, W - r[i].w));
            r[i].y = clampD(r[i].y, 0.0, max(0.0, H - r[i].h));
        }
    }

    static bool projectSoftByHcgVcg(
        const Design& design,
        const vector<Rect>& ref,
        vector<Rect>& r,
        double W,
        double H
    ) {
        const vector<SepConstraint> constraints = buildSoftProjectionConstraints(design, ref);
        if (constraints.empty()) return true;

        clampSoftInsideOutline(design, r, W, H);
        for (int iter = 0; iter < 96; ++iter) {
            bool changed = false;
            const bool xPassFirst = (iter % 2 == 0);
            for (int pass = 0; pass < 2; ++pass) {
                const bool xPass = (pass == 0) ? xPassFirst : !xPassFirst;
                for (const SepConstraint& c : constraints) {
                    if (c.xAxis != xPass) continue;
                    if (c.lo < 0 || c.hi < 0 || c.lo >= static_cast<int>(r.size()) || c.hi >= static_cast<int>(r.size())) continue;
                    const bool loMov = forceMovableSoft(design, c.lo);
                    const bool hiMov = forceMovableSoft(design, c.hi);
                    if (!loMov && !hiMov) continue;

                    if (xPass) {
                        const double need = rectRight(r[c.lo]) + c.gap;
                        const double viol = need - r[c.hi].x;
                        if (viol <= 1.0e-5) continue;
                        if (loMov && hiMov) {
                            r[c.lo].x -= 0.5 * viol;
                            r[c.hi].x += 0.5 * viol;
                        }
                        else if (hiMov) r[c.hi].x += viol;
                        else r[c.lo].x -= viol;
                    }
                    else {
                        const double need = rectTop(r[c.lo]) + c.gap;
                        const double viol = need - r[c.hi].y;
                        if (viol <= 1.0e-5) continue;
                        if (loMov && hiMov) {
                            r[c.lo].y -= 0.5 * viol;
                            r[c.hi].y += 0.5 * viol;
                        }
                        else if (hiMov) r[c.hi].y += viol;
                        else r[c.lo].y -= viol;
                    }
                    changed = true;
                    clampSoftInsideOutline(design, r, W, H);
                }
            }
            if (!changed) break;
        }

        LayoutResult projected = scoreLayout(design, W, H, r);
        return projected.legal && projected.strictEdgeLegal && projected.overlap <= EPS && projected.outlineViol <= EPS;
    }

    static bool refineLayoutWithHpwlForce(const Design& design, const LayoutResult& base, LayoutResult& out) {
        if (!ENABLE_HPWL_FORCE_ARCHIVE_REFINEMENT ||
            !base.legal || !base.strictEdgeLegal ||
            base.overlap > EPS || base.outlineViol > EPS) return false;

        vector<Rect> cur = base.rects;
        const vector<Rect> ref = base.rects;
        const vector<ForcePair> pairs = collectForcePairs(design);
        if (pairs.empty()) return false;

        vector<char> movable(cur.size(), 0);
        int movableCount = 0;
        for (int i = 0; i < static_cast<int>(cur.size()); ++i) {
            movable[i] = forceMovableSoft(design, i) ? 1 : 0;
            if (movable[i]) ++movableCount;
        }
        if (movableCount == 0) return false;

        const double W = base.W;
        const double H = base.H;
        for (int iter = 0; iter < FORCE_REFINE_ITERS; ++iter) {
            vector<double> fx(cur.size(), 0.0), fy(cur.size(), 0.0), fw(cur.size(), 0.0);

            for (const ForcePair& p : pairs) {
                double dx = 0.0, dy = 0.0;
                const double dist = max(1.0, bestPortAwareDelta(design, cur, p.a, p.b, dx, dy));
                const double ux = dx / dist;
                const double uy = dy / dist;
                const double w = FORCE_ATTR_WEIGHT * static_cast<double>(p.nets);
                if (movable[p.a]) {
                    fx[p.a] += w * ux;
                    fy[p.a] += w * uy;
                    fw[p.a] += w;
                }
                if (movable[p.b]) {
                    fx[p.b] -= w * ux;
                    fy[p.b] -= w * uy;
                    fw[p.b] += w;
                }

                if (!p.hot) continue;
                const Rect& a = cur[p.a];
                const Rect& b = cur[p.b];
                const double cxD = rectCx(b) - rectCx(a);
                const double cyD = rectCy(b) - rectCy(a);
                const bool horizontal = fabs(cxD) >= fabs(cyD);
                if (horizontal) {
                    const double reqGap = requiredXGap(design, p.a, p.b);
                    const bool aLeft = rectCx(a) <= rectCx(b);
                    const double gap = aLeft ? (b.x - rectRight(a)) : (a.x - rectRight(b));
                    const double miss = max(0.0, reqGap - gap);
                    if (miss > EPS) {
                        const double dir = aLeft ? 1.0 : -1.0;
                        const double push = FORCE_CHANNEL_WEIGHT * static_cast<double>(p.nets) * miss / max(1.0, reqGap);
                        if (movable[p.a]) { fx[p.a] -= dir * push; fw[p.a] += push; }
                        if (movable[p.b]) { fx[p.b] += dir * push; fw[p.b] += push; }
                    }
                    const double reqOv = requiredPortOverlap(design, p.a, p.b);
                    const double yOv = ovLen(a.y, rectTop(a), b.y, rectTop(b));
                    if (reqOv > EPS && yOv < reqOv) {
                        const double dirY = (rectCy(b) >= rectCy(a)) ? 1.0 : -1.0;
                        const double pull = FORCE_CHANNEL_WEIGHT * static_cast<double>(p.nets) * (reqOv - yOv) / reqOv;
                        if (movable[p.a]) { fy[p.a] += dirY * pull; fw[p.a] += pull; }
                        if (movable[p.b]) { fy[p.b] -= dirY * pull; fw[p.b] += pull; }
                    }
                }
                else {
                    const double reqGap = requiredYGap(design, p.a, p.b);
                    const bool aBelow = rectCy(a) <= rectCy(b);
                    const double gap = aBelow ? (b.y - rectTop(a)) : (a.y - rectTop(b));
                    const double miss = max(0.0, reqGap - gap);
                    if (miss > EPS) {
                        const double dir = aBelow ? 1.0 : -1.0;
                        const double push = FORCE_CHANNEL_WEIGHT * static_cast<double>(p.nets) * miss / max(1.0, reqGap);
                        if (movable[p.a]) { fy[p.a] -= dir * push; fw[p.a] += push; }
                        if (movable[p.b]) { fy[p.b] += dir * push; fw[p.b] += push; }
                    }
                    const double reqOv = requiredPortOverlap(design, p.a, p.b);
                    const double xOv = ovLen(a.x, rectRight(a), b.x, rectRight(b));
                    if (reqOv > EPS && xOv < reqOv) {
                        const double dirX = (rectCx(b) >= rectCx(a)) ? 1.0 : -1.0;
                        const double pull = FORCE_CHANNEL_WEIGHT * static_cast<double>(p.nets) * (reqOv - xOv) / reqOv;
                        if (movable[p.a]) { fx[p.a] += dirX * pull; fw[p.a] += pull; }
                        if (movable[p.b]) { fx[p.b] -= dirX * pull; fw[p.b] += pull; }
                    }
                }
            }

            const double nearGuard = max(4.0, 0.004 * min(W, H));
            for (int i = 0; i < static_cast<int>(cur.size()); ++i) {
                if (!movable[i]) continue;
                for (int j = 0; j < static_cast<int>(cur.size()); ++j) {
                    if (i == j) continue;
                    const double xOv = ovLen(cur[i].x, rectRight(cur[i]), cur[j].x, rectRight(cur[j]));
                    const double yOv = ovLen(cur[i].y, rectTop(cur[i]), cur[j].y, rectTop(cur[j]));
                    const bool overlaps = xOv > EPS && yOv > EPS;
                    const double xGap = max(0.0, max(cur[j].x - rectRight(cur[i]), cur[i].x - rectRight(cur[j])));
                    const double yGap = max(0.0, max(cur[j].y - rectTop(cur[i]), cur[i].y - rectTop(cur[j])));
                    const bool nearX = yOv > EPS && xGap < nearGuard;
                    const bool nearY = xOv > EPS && yGap < nearGuard;
                    if (!overlaps && !nearX && !nearY) continue;
                    double dx = rectCx(cur[i]) - rectCx(cur[j]);
                    double dy = rectCy(cur[i]) - rectCy(cur[j]);
                    double len = hypot(dx, dy);
                    if (len < 1.0e-6) {
                        dx = (i < j) ? -1.0 : 1.0;
                        dy = ((i + j) & 1) ? -0.5 : 0.5;
                        len = hypot(dx, dy);
                    }
                    const double strength = FORCE_REPULSE_WEIGHT * (overlaps ? 2.0 : 0.65) *
                        max(1.0, endpointDemand(design, i)) / max(1.0, maxEndpointDemand(design));
                    fx[i] += strength * dx / len;
                    fy[i] += strength * dy / len;
                    fw[i] += strength;
                }
            }

            const double t = static_cast<double>(iter) / max(1, FORCE_REFINE_ITERS - 1);
            const double stepFrac = FORCE_REFINE_START_STEP * (1.0 - t) + FORCE_REFINE_END_STEP * t;
            for (int i = 0; i < static_cast<int>(cur.size()); ++i) {
                if (!movable[i] || fw[i] <= EPS) continue;
                const double maxStep = max(1.0, stepFrac * min(cur[i].w, cur[i].h));
                const double mx = clampD(fx[i] / fw[i] * maxStep, -maxStep, maxStep);
                const double my = clampD(fy[i] / fw[i] * maxStep, -maxStep, maxStep);
                cur[i].x += mx;
                cur[i].y += my;
            }
            clampSoftInsideOutline(design, cur, W, H);
            if (!projectSoftByHcgVcg(design, ref, cur, W, H)) return false;
        }

        LayoutResult refined = scoreLayout(design, W, H, std::move(cur));
        if (!refined.legal || !refined.strictEdgeLegal) return false;
        annotateStripProxy(design, refined);
        out = std::move(refined);
        return true;
    }

    static vector<LayoutResult> makeHpwlForceRefinedArchive(const Design& design, const vector<LayoutResult>& archive) {
        vector<pair<double, LayoutResult>> ranked;
        const int limit = min(FORCE_REFINE_SEED_LIMIT, static_cast<int>(archive.size()));
        for (int i = 0; i < limit; ++i) {
            const LayoutResult& base = archive[i];
            LayoutResult refined;
            if (!refineLayoutWithHpwlForce(design, base, refined)) continue;

            const double basePA = max(1.0, portAwareHpwl(design, base.rects));
            const double newPA = max(1.0, portAwareHpwl(design, refined.rects));
            const RouteProxy baseProxy = estimateRouteProxy(design, base);
            const RouteProxy refinedProxy = estimateRouteProxy(design, refined);

            const bool areaOk = refined.area <= base.area * 1.060 + 1.0;
            const bool noOpenRegression = refinedProxy.disconnectedPairs <= baseProxy.disconnectedPairs;
            const double baseRisk = max(1.0, baseProxy.channelRisk + 0.35 * baseProxy.ftRisk);
            const double refinedRisk = max(1.0, refinedProxy.channelRisk + 0.35 * refinedProxy.ftRisk);
            const bool channelRiskOk =
                refinedProxy.maxDirectionalUtil <= max(baseProxy.maxDirectionalUtil * 1.16 + 0.05, baseProxy.maxDirectionalUtil + 0.22) &&
                refinedProxy.channelRisk <= max(baseProxy.channelRisk * 1.22 + 2500.0, baseProxy.channelRisk + 250000.0);
            const bool ftRiskOk =
                refinedProxy.ftRisk <= max(baseProxy.ftRisk * 1.30 + 500.0, baseProxy.ftRisk + 100000.0);
            const bool stripSafe =
                refined.stripPeakUtilProxy <= max(base.stripPeakUtilProxy * 1.08 + 0.05, base.stripPeakUtilProxy + 6000.0) &&
                refined.stripOverflowProxy <= max(base.stripOverflowProxy * 1.08 + 5000.0, base.stripOverflowProxy + 12000000.0);
            const bool wlBetter =
                refinedProxy.guidePointWL <= baseProxy.guidePointWL * 0.995;
            const bool riskBetter =
                refinedRisk <= baseRisk * 0.90;
            const bool areaBetter =
                refined.area <= base.area * 0.995 &&
                refinedProxy.guidePointWL <= baseProxy.guidePointWL * 1.020 &&
                refinedRisk <= baseRisk * 1.08;
            const bool accept = areaOk && noOpenRegression && channelRiskOk && ftRiskOk && stripSafe &&
                (wlBetter || riskBetter || areaBetter);
            string rejectReason = "accepted";
            if (!accept) {
                if (!areaOk) rejectReason = "areaWorse";
                else if (!noOpenRegression) rejectReason = "openRegression";
                else if (!stripSafe) rejectReason = "stripRiskWorse";
                else if (!channelRiskOk) rejectReason = "channelRiskWorse";
                else if (!ftRiskOk) rejectReason = "ftRiskWorse";
                else rejectReason = "noParetoGain";
            }

            cerr << fixed << setprecision(3)
                << "[HPWLForce] seed=" << i
                << " pa=" << basePA << "->" << newPA
                << " proxyWL=" << baseProxy.guidePointWL << "->" << refinedProxy.guidePointWL
                << " maxUtil=" << baseProxy.maxDirectionalUtil << "->" << refinedProxy.maxDirectionalUtil
                << " chRisk=" << baseProxy.channelRisk << "->" << refinedProxy.channelRisk
                << " ftRisk=" << baseProxy.ftRisk << "->" << refinedProxy.ftRisk
                << " openProxy=" << baseProxy.disconnectedPairs << "->" << refinedProxy.disconnectedPairs
                << " area=" << base.area << "->" << refined.area
                << " stripPeak=" << base.stripPeakUtilProxy << "->" << refined.stripPeakUtilProxy
                << " stripOv=" << base.stripOverflowProxy << "->" << refined.stripOverflowProxy
                << " accept=" << (accept ? "Y" : "N")
                << " rejectReason=" << rejectReason
                << "\n";

            if (!accept) continue;
            const double proxyWLRatio = refinedProxy.guidePointWL / max(1.0, baseProxy.guidePointWL);
            const double areaRatio = refined.area / max(1.0, base.area);
            const double riskRatio = refinedRisk / baseRisk;
            const double paRatio = newPA / basePA;
            const double openPenalty = 1000.0 * static_cast<double>(refinedProxy.disconnectedPairs);
            const double score = openPenalty +
                0.64 * proxyWLRatio +
                0.18 * riskRatio +
                0.12 * areaRatio +
                0.06 * paRatio;
            ranked.push_back({ score, std::move(refined) });
        }

        sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) {
            if (fabs(a.first - b.first) > 1.0e-9) return a.first < b.first;
            return a.second.area < b.second.area;
            });
        vector<LayoutResult> out;
        for (auto& item : ranked) {
            if (static_cast<int>(out.size()) >= FORCE_REFINE_KEEP_LIMIT) break;
            bool dup = false;
            for (const auto& old : out) {
                if (fabs(old.area - item.second.area) <= max(1.0, 0.0005 * min(old.area, item.second.area)) &&
                    fabs(old.hpwl - item.second.hpwl) <= max(1.0, 0.0005 * min(old.hpwl, item.second.hpwl))) {
                    dup = true;
                    break;
                }
            }
            if (!dup) out.push_back(std::move(item.second));
        }
        return out;
    }

    static double minWidthBound(const Design& design, const ShapeState& st) {
        vector<Rect> shapes = makeShapes(design, st);
        double w = 1.0;
        for (const Rect& r : shapes) w = max(w, r.w);
        // EDGE zone feasibility lower bound.
        double needBucket[12] = { 0 };
        for (int i = 0; i < static_cast<int>(design.blockSpecs.size()); ++i) {
            if (design.blockSpecs[i].type != BlockType::EDGE) continue;
            vector<EdgeRule> rs = edgeRules(design.blockSpecs[i]);
            EdgeRule er = rs.empty() ? parseRule("BL") : rs.front();
            int side = (er.side == 'T') ? 0 : (er.side == 'B' ? 1 : (er.side == 'L' ? 2 : 3));
            int bucket = side * 3 + er.zone;
            needBucket[bucket] += sideLen(shapes[i], er.side) + PACK_GAP;
        }
        for (int b = 0; b < 6; ++b) w = max(w, 3.0 * needBucket[b]); // top/bottom buckets
        return min(w, design.maxOutlineW);
    }

    static double minHeightBound(const Design& design, const ShapeState& st) {
        vector<Rect> shapes = makeShapes(design, st);
        double h = 1.0;
        for (const Rect& r : shapes) h = max(h, r.h);
        double needBucket[12] = { 0 };
        for (int i = 0; i < static_cast<int>(design.blockSpecs.size()); ++i) {
            if (design.blockSpecs[i].type != BlockType::EDGE) continue;
            vector<EdgeRule> rs = edgeRules(design.blockSpecs[i]);
            EdgeRule er = rs.empty() ? parseRule("BL") : rs.front();
            int side = (er.side == 'T') ? 0 : (er.side == 'B' ? 1 : (er.side == 'L' ? 2 : 3));
            int bucket = side * 3 + er.zone;
            needBucket[bucket] += sideLen(shapes[i], er.side) + PACK_GAP;
        }
        for (int b = 6; b < 12; ++b) h = max(h, 3.0 * needBucket[b]); // left/right buckets
        return min(h, design.maxOutlineH);
    }

    static vector<int> degreeOrder(const Design& design) {
        int n = static_cast<int>(design.blockSpecs.size());
        vector<int> ids(n);
        iota(ids.begin(), ids.end(), 0);
        vector<int> deg(n, 0);
        if (!design.connections.empty()) {
            for (const auto& c : design.connections) {
                if (c.src >= 0 && c.src < n) deg[c.src] += max(0, c.netCount);
                if (c.dst >= 0 && c.dst < n) deg[c.dst] += max(0, c.netCount);
            }
        }
        else {
            for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) deg[i] += totalConnBetween(design, i, j);
        }
        sort(ids.begin(), ids.end(), [&](int a, int b) {
            bool ma = isMovableBlock(design.blockSpecs[a]);
            bool mb = isMovableBlock(design.blockSpecs[b]);
            if (ma != mb) return ma > mb;
            if (deg[a] != deg[b]) return deg[a] > deg[b];
            double aa = blockNominalArea(design.blockSpecs[a]);
            double bb = blockNominalArea(design.blockSpecs[b]);
            if (fabs(aa - bb) > 1e-6) return aa > bb;
            return a < b;
            });
        return ids;
    }

    static vector<vector<int>> makeOrders(const Design& design) {
        int n = static_cast<int>(design.blockSpecs.size());
        vector<int> ids(n);
        iota(ids.begin(), ids.end(), 0);
        vector<vector<int>> orders;

        auto movableFirst = [&](vector<int> v) {
            stable_sort(v.begin(), v.end(), [&](int a, int b) {
                return isMovableBlock(design.blockSpecs[a]) > isMovableBlock(design.blockSpecs[b]);
                });
            return v;
            };

        vector<int> byArea = ids;
        sort(byArea.begin(), byArea.end(), [&](int a, int b) {
            bool ma = isMovableBlock(design.blockSpecs[a]);
            bool mb = isMovableBlock(design.blockSpecs[b]);
            if (ma != mb) return ma > mb;
            double aa = blockNominalArea(design.blockSpecs[a]);
            double bb = blockNominalArea(design.blockSpecs[b]);
            return aa > bb;
            });
        orders.push_back(byArea);

        vector<int> byHeight = ids;
        sort(byHeight.begin(), byHeight.end(), [&](int a, int b) {
            Rect ra = makeShape(design.blockSpecs[a], aspectMid(design.blockSpecs[a]));
            Rect rb = makeShape(design.blockSpecs[b], aspectMid(design.blockSpecs[b]));
            if (isMovableBlock(design.blockSpecs[a]) != isMovableBlock(design.blockSpecs[b])) return isMovableBlock(design.blockSpecs[a]) > isMovableBlock(design.blockSpecs[b]);
            return ra.h > rb.h;
            });
        orders.push_back(byHeight);

        vector<int> byWidth = ids;
        sort(byWidth.begin(), byWidth.end(), [&](int a, int b) {
            Rect ra = makeShape(design.blockSpecs[a], aspectMid(design.blockSpecs[a]));
            Rect rb = makeShape(design.blockSpecs[b], aspectMid(design.blockSpecs[b]));
            if (isMovableBlock(design.blockSpecs[a]) != isMovableBlock(design.blockSpecs[b])) return isMovableBlock(design.blockSpecs[a]) > isMovableBlock(design.blockSpecs[b]);
            return ra.w > rb.w;
            });
        orders.push_back(byWidth);

        vector<int> byDegree = degreeOrder(design);
        orders.push_back(byDegree);
        orders.push_back(movableFirst(ids));

        // Hot-pair perturbation: visit endpoints of high-demand connections together,
        // encouraging common-edge / near-neighbor placement before the remaining blocks.
        vector<pair<int, pair<int, int>>> hotPairs;
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                int nets = totalConnBetween(design, i, j);
                if (nets > 0 && isMovableBlock(design.blockSpecs[i]) && isMovableBlock(design.blockSpecs[j])) {
                    hotPairs.push_back({ nets, {i, j} });
                }
            }
        }
        sort(hotPairs.begin(), hotPairs.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
        vector<int> hotOrder;
        vector<char> used(n, 0);
        for (const auto& hp : hotPairs) {
            int a = hp.second.first, b = hp.second.second;
            if (!used[a]) { hotOrder.push_back(a); used[a] = 1; }
            if (!used[b]) { hotOrder.push_back(b); used[b] = 1; }
        }
        for (int id : byDegree) if (isMovableBlock(design.blockSpecs[id]) && !used[id]) hotOrder.push_back(id);
        for (int id : ids) if (!isMovableBlock(design.blockSpecs[id])) hotOrder.push_back(id);
        if (!hotOrder.empty()) orders.push_back(hotOrder);

        // Greedy net-growth perturbation: after a high-degree seed, repeatedly pick
        // the unplaced block with strongest connection to the placed set.
        vector<int> growOrder;
        used.assign(n, 0);
        int seed = -1;
        for (int id : byDegree) if (isMovableBlock(design.blockSpecs[id])) { seed = id; break; }
        if (seed >= 0) {
            growOrder.push_back(seed); used[seed] = 1;
            while (true) {
                int best = -1, bestConn = -1;
                double bestArea = -1.0;
                for (int id : ids) {
                    if (used[id] || !isMovableBlock(design.blockSpecs[id])) continue;
                    int conn = 0;
                    for (int p : growOrder) conn += totalConnBetween(design, id, p);
                    double area = blockNominalArea(design.blockSpecs[id]);
                    if (conn > bestConn || (conn == bestConn && area > bestArea)) {
                        bestConn = conn; bestArea = area; best = id;
                    }
                }
                if (best < 0) break;
                growOrder.push_back(best); used[best] = 1;
            }
            for (int id : ids) if (!isMovableBlock(design.blockSpecs[id])) growOrder.push_back(id);
            orders.push_back(growOrder);
        }

        // Edge-interface perturbation: blocks connected to EDGE macros are packed
        // early so they can secure inner-side port windows.
        vector<int> edgeFirst = byDegree;
        stable_sort(edgeFirst.begin(), edgeFirst.end(), [&](int a, int b) {
            if (isMovableBlock(design.blockSpecs[a]) != isMovableBlock(design.blockSpecs[b])) return isMovableBlock(design.blockSpecs[a]) > isMovableBlock(design.blockSpecs[b]);
            auto edgeConn = [&](int id) {
                int s = 0;
                for (int e = 0; e < n; ++e) if (design.blockSpecs[e].type == BlockType::EDGE) s += totalConnBetween(design, id, e);
                return s;
                };
            int ea = edgeConn(a), eb = edgeConn(b);
            if (ea != eb) return ea > eb;
            return endpointDemand(design, a) > endpointDemand(design, b);
            });
        orders.push_back(edgeFirst);

        mt19937 rng(FAST_SEED + 31u * static_cast<unsigned>(n));
        vector<int> mov;
        for (int id : ids) if (isMovableBlock(design.blockSpecs[id])) mov.push_back(id);

        // Runtime-bounded diversity: for n blocks we want at least n^2 high-level
        // layout candidates, but we do not want an unbounded number of random orders.
        // Public problem scale is below 50 blocks, so this cap keeps runtime stable.
        const int randomOrderCount = min(36, max(FAST_RANDOM_ORDERS, max(2, n)));
        for (int k = 0; k < randomOrderCount; ++k) {
            vector<int> v = mov;
            if (k % 3 == 0) {
                shuffle(v.begin(), v.end(), rng);
            }
            else {
                // Degree-random perturbation: mostly high-degree first, with stable
                // precomputed jitter so the comparator remains a strict weak ordering.
                vector<double> score(n, 0.0);
                for (int id : v) {
                    double j = uniform_real_distribution<double>(0.0, 1.0)(rng);
                    score[id] = endpointDemand(design, id) * (0.65 + j) + 1e-6 * id;
                }
                sort(v.begin(), v.end(), [&](int a, int b) {
                    if (fabs(score[a] - score[b]) > 1e-9) return score[a] > score[b];
                    return a < b;
                    });
            }
            if (k % 4 == 2) reverse(v.begin(), v.end());
            for (int id : ids) if (!isMovableBlock(design.blockSpecs[id])) v.push_back(id);
            orders.push_back(v);
        }

        // Remove exact duplicates.
        vector<vector<int>> uniqueOrders;
        for (const auto& o : orders) {
            bool dup = false;
            for (const auto& u : uniqueOrders) if (u == o) { dup = true; break; }
            if (!dup) uniqueOrders.push_back(o);
        }
        return uniqueOrders;
    }

    static double edgeSideDemandBias(const Design& design, int id) {
        // Positive => prefer wider SOFT block (more top/bottom span).
        // Negative => prefer taller SOFT block (more left/right span).
        double topBottom = 0.0;
        double leftRight = 0.0;
        for (int e = 0; e < static_cast<int>(design.blockSpecs.size()); ++e) {
            if (e == id || design.blockSpecs[e].type != BlockType::EDGE) continue;
            const int nets = totalConnBetween(design, id, e);
            if (nets <= 0) continue;
            vector<EdgeRule> rules = edgeRules(design.blockSpecs[e]);
            if (rules.empty()) rules.push_back(parseRule("BL"));
            for (const EdgeRule& r : rules) {
                if (r.side == 'T' || r.side == 'B') topBottom += static_cast<double>(nets) / max(1, static_cast<int>(rules.size()));
                else if (r.side == 'L' || r.side == 'R') leftRight += static_cast<double>(nets) / max(1, static_cast<int>(rules.size()));
            }
        }
        return (topBottom - leftRight) / max(1.0, topBottom + leftRight);
    }

    static double smartSoftTargetRatio(const Design& design, int id, double globalBias = 1.0) {
        const BlockSpec& s = design.blockSpecs[id];
        double amin = max(0.05, s.aspectMin);
        double amax = max(amin, s.aspectMax);
        double mid = aspectMid(s);
        if (s.hasFixedSize || s.type != BlockType::SOFT) return mid;

        const double outlineRatio = clampD(design.maxOutlineW / max(1.0, design.maxOutlineH), amin, amax);
        const double edgeBias = edgeSideDemandBias(design, id);
        const double hot = clampD(endpointDemand(design, id) / max(1.0, maxEndpointDemand(design)), 0.0, 1.0);
        const double ftRate = softFtExpansionRate(design, id);

        // Work in log-ratio space (shape-curve xy=A).  Hot / FT-risk soft blocks
        // are biased more strongly because their boundary length is valuable for
        // routing and later feedthrough growth.
        double logR = log(mid);
        logR = 0.55 * logR + 0.45 * log(outlineRatio);
        logR += (0.48 + 0.36 * sqrt(hot)) * edgeBias;
        if (ftRate > 0.12) {
            // Reserve-heavy blocks should avoid extreme skinny shapes; central ratios
            // leave usable slack on both dimensions after expansion.
            logR = 0.72 * logR + 0.28 * log(mid);
        }
        logR += log(max(0.05, globalBias));
        return clampD(exp(logR), amin, amax);
    }

    static vector<ShapeState> makeShapeStates(const Design& design) {
        int n = static_cast<int>(design.blockSpecs.size());
        vector<ShapeState> states;
        auto pushUnique = [&](const ShapeState& st) {
            for (const auto& u : states) {
                bool same = u.ratio.size() == st.ratio.size();
                if (same) {
                    for (int i = 0; i < static_cast<int>(st.ratio.size()); ++i) {
                        if (fabs(log(max(1e-9, u.ratio[i])) - log(max(1e-9, st.ratio[i]))) > 1e-3) { same = false; break; }
                    }
                }
                if (same) return;
            }
            states.push_back(st);
            };
        auto make = [&](int mode, double globalBias = 1.0) {
            ShapeState st;
            st.ratio.assign(n, 1.0);
            for (int i = 0; i < n; ++i) {
                const BlockSpec& s = design.blockSpecs[i];
                double amin = max(0.05, s.aspectMin);
                double amax = max(amin, s.aspectMax);
                double r = aspectMid(s);
                if (!s.hasFixedSize) {
                    if (s.type == BlockType::SOFT && ENABLE_SOFT_SHAPE_PERTURB) {
                        double smart = smartSoftTargetRatio(design, i, globalBias);
                        if (mode == 0) r = smart;
                        else if (mode == 1) r = clampD(1.0 * globalBias, amin, amax);
                        else if (mode == 2) r = amin;
                        else if (mode == 3) r = amax;
                        else if (mode == 4) r = clampD(sqrt(amin * smart), amin, amax);
                        else if (mode == 5) r = clampD(sqrt(amax * smart), amin, amax);
                        else if (mode == 6) r = (i % 2 == 0) ? amin : amax;
                        else if (mode == 7) r = (i % 2 == 0) ? amax : amin;
                        else r = smart;
                    }
                    else {
                        if (mode == 1) r = clampD(1.0, amin, amax);
                        else if (mode == 2) r = amin;
                        else if (mode == 3) r = amax;
                        else if (mode == 4) r = (i % 2 == 0) ? amin : amax;
                        else if (mode == 5) r = (i % 2 == 0) ? amax : amin;
                    }
                }
                st.ratio[i] = r;
            }
            return st;
            };

        // Deterministic perturbations first: these are the high-value states that
        // will be reached even when the global candidate cap stops exploration early.
        pushUnique(make(0, 1.0));
        pushUnique(make(0, 0.82));
        pushUnique(make(0, 1.22));
        for (int m = 1; m <= 7; ++m) pushUnique(make(m, 1.0));

        // Hot-block targeted states: only the most FT-risk / routing-hot soft blocks
        // are pushed toward aspect extremes.  This mimics the SA "change soft module
        // shape" perturbation without exploding the search space.
        vector<int> softIds;
        for (int i = 0; i < n; ++i) if (design.blockSpecs[i].type == BlockType::SOFT && !design.blockSpecs[i].hasFixedSize) softIds.push_back(i);
        sort(softIds.begin(), softIds.end(), [&](int a, int b) {
            double sa = endpointDemand(design, a) * (1.0 + softFtExpansionRate(design, a));
            double sb = endpointDemand(design, b) * (1.0 + softFtExpansionRate(design, b));
            if (fabs(sa - sb) > 1e-6) return sa > sb;
            return a < b;
            });
        const int hotLimit = min(static_cast<int>(softIds.size()), max(2, static_cast<int>(sqrt(max(1, n))) + 2));
        for (int t = 0; t < hotLimit; ++t) {
            int id = softIds[t];
            const BlockSpec& sp = design.blockSpecs[id];
            double amin = max(0.05, sp.aspectMin);
            double amax = max(amin, sp.aspectMax);
            ShapeState low = make(0, 1.0), high = make(0, 1.0);
            low.ratio[id] = clampD(exp(log(low.ratio[id]) * (1.0 - SOFT_SHAPE_HOT_EXTREME_BIAS) + log(amin) * SOFT_SHAPE_HOT_EXTREME_BIAS), amin, amax);
            high.ratio[id] = clampD(exp(log(high.ratio[id]) * (1.0 - SOFT_SHAPE_HOT_EXTREME_BIAS) + log(amax) * SOFT_SHAPE_HOT_EXTREME_BIAS), amin, amax);
            pushUnique(low);
            pushUnique(high);
        }

        mt19937 rng(FAST_SEED ^ (0x9e3779b9u + static_cast<unsigned>(n)));
        const int randomStates = ENABLE_SOFT_SHAPE_PERTURB ? min(SOFT_SHAPE_RANDOM_STATES, max(0, SOFT_SHAPE_TARGET_STATES - static_cast<int>(states.size()))) : FAST_RANDOM_SHAPES;
        for (int k = 0; k < randomStates; ++k) {
            ShapeState st = make(0, 1.0);
            for (int i = 0; i < n; ++i) {
                const BlockSpec& sp = design.blockSpecs[i];
                double amin = max(0.05, sp.aspectMin);
                double amax = max(amin, sp.aspectMax);
                if (!sp.hasFixedSize && sp.type == BlockType::SOFT) {
                    double target = smartSoftTargetRatio(design, i, 1.0);
                    double hot = clampD(endpointDemand(design, i) / max(1.0, maxEndpointDemand(design)), 0.0, 1.0);
                    double sigma = SOFT_SHAPE_LOG_SIGMA * (0.55 + 0.45 * sqrt(hot));
                    normal_distribution<double> nd(0.0, sigma);
                    double r = exp(log(target) + nd(rng));
                    if (k % 5 == 1 && hot > 0.45) r = (edgeSideDemandBias(design, i) >= 0.0) ? amax : amin;
                    if (k % 7 == 3) r = exp(log(amin) * 0.35 + log(amax) * 0.65);
                    st.ratio[i] = clampD(r, amin, amax);
                }
            }
            pushUnique(st);
        }

        return states;
    }

    static vector<double> makeWidthTrials(const Design& design, const ShapeState& st) {
        vector<double> wv;
        double area = totalPackingArea(design);
        double minW = minWidthBound(design, st);
        double minH = minHeightBound(design, st);
        double aspect = design.maxOutlineW / max(1.0, design.maxOutlineH);
        double base = sqrt(area * aspect);
        double factors[] = { 0.62, 0.70, 0.78, 0.86, 0.94, 1.00, 1.08, 1.16, 1.26, 1.38, 1.52 };
        for (double f : factors) wv.push_back(clampD(base * f, minW, design.maxOutlineW));
        for (int i = 0; i < FAST_WIDTH_TRIALS; ++i) {
            double t = (FAST_WIDTH_TRIALS == 1) ? 0.0 : static_cast<double>(i) / (FAST_WIDTH_TRIALS - 1);
            double curved = t * t;
            wv.push_back(clampD(minW + (design.maxOutlineW - minW) * curved, minW, design.maxOutlineW));
        }
        wv.push_back(design.maxOutlineW);
        wv.push_back(max(minW, area / max(1.0, design.maxOutlineH)));
        sort(wv.begin(), wv.end());
        wv.erase(unique(wv.begin(), wv.end(), [](double a, double b) { return fabs(a - b) < 1.0; }), wv.end());
        (void)minH;
        return wv;
    }

    static vector<double> makeHeightTrials(const Design& design, const ShapeState& st, double W) {
        vector<double> hv;
        const double area = totalPackingArea(design);
        const double minH = clampD(minHeightBound(design, st), 1.0, design.maxOutlineH);
        const double base = clampD(area / max(1.0, W), minH, design.maxOutlineH);

        // Symmetric to makeWidthTrials(), but centered around area/W.  These explicit
        // H candidates replace repeated binary-search packing in the main loop, so
        // each high-level candidate costs one deterministic pack.
        const double factors[] = { 0.70, 0.78, 0.86, 0.94, 1.00, 1.08, 1.18, 1.30, 1.45, 1.62 };
        for (double f : factors) hv.push_back(clampD(base * f, minH, design.maxOutlineH));
        hv.push_back(minH);
        hv.push_back(design.maxOutlineH);
        sort(hv.begin(), hv.end());
        hv.erase(unique(hv.begin(), hv.end(), [](double a, double b) { return fabs(a - b) < 1.0; }), hv.end());
        return hv;
    }

    static bool tryExplicitWH(
        const Design& design,
        const vector<int>& order,
        const ShapeState& st,
        double W,
        double H,
        bool strictEdge,
        LayoutResult& out
    ) {
        if (W > design.maxOutlineW + EPS || H > design.maxOutlineH + EPS) return false;
        if (W < minWidthBound(design, st) - EPS || H < minHeightBound(design, st) - EPS) return false;
        vector<Rect> rects;
        if (!packOne(design, order, st, W, H, strictEdge, rects)) return false;
        out = scoreLayout(design, W, H, std::move(rects));
        return true;
    }

    static bool tryWidthHeight(
        const Design& design,
        const vector<int>& order,
        const ShapeState& st,
        double W,
        bool strictEdge,
        LayoutResult& out
    ) {
        double minH = minHeightBound(design, st);
        minH = clampD(minH, 1.0, design.maxOutlineH);
        vector<Rect> rects;
        if (!packOne(design, order, st, W, design.maxOutlineH, strictEdge, rects)) return false;

        double lo = minH;
        double hi = design.maxOutlineH;
        vector<Rect> bestRects = rects;
        for (int it = 0; it < FAST_HEIGHT_BISECT; ++it) {
            double mid = 0.5 * (lo + hi);
            vector<Rect> trial;
            if (packOne(design, order, st, W, mid, strictEdge, trial)) {
                hi = mid;
                bestRects = std::move(trial);
            }
            else {
                lo = mid;
            }
        }
        // Repack at a small slack above hi for numeric stability.
        double H = min(design.maxOutlineH, hi + 1.0e-3);
        vector<Rect> finalRects;
        if (!packOne(design, order, st, W, H, strictEdge, finalRects)) {
            H = min(design.maxOutlineH, hi + 0.25);
            if (!packOne(design, order, st, W, H, strictEdge, finalRects)) {
                finalRects = bestRects;
            }
        }
        out = scoreLayout(design, W, H, std::move(finalRects));
        return true;
    }


    // --------------------------------------------------------------------------
    // B*-tree SA state and route-aware B*-packing.
    // --------------------------------------------------------------------------
    struct BStarNode {
        int block = -1;   // original design.blockSpecs index
        int parent = -1;
        int left = -1;    // B*-tree left child: place at parent's right side
        int right = -1;   // B*-tree right child: place above parent
    };

    struct BStarState {
        vector<BStarNode> node;
        int root = -1;
        ShapeState shape;
        double W = 1.0;
        double H = 1.0;
        bool strictEdge = true;
    };

    static vector<int> bstarPreorder(const BStarState& bs);
    static bool bstarTreeLegal(const BStarState& bs);

    static vector<int> movableIdsOf(const Design& design) {
        vector<int> ids;
        ids.reserve(design.blockSpecs.size());
        for (int id : degreeOrder(design)) {
            if (id >= 0 && id < static_cast<int>(design.blockSpecs.size()) && isMovableBlock(design.blockSpecs[id])) {
                ids.push_back(id);
            }
        }
        return ids;
    }

    static vector<int> filterMovableOrder(const Design& design, const vector<int>& raw) {
        const int n = static_cast<int>(design.blockSpecs.size());
        vector<int> out;
        vector<char> used(n, 0);
        for (int id : raw) {
            if (id < 0 || id >= n || used[id] || !isMovableBlock(design.blockSpecs[id])) continue;
            out.push_back(id);
            used[id] = 1;
        }
        for (int id : degreeOrder(design)) {
            if (id >= 0 && id < n && !used[id] && isMovableBlock(design.blockSpecs[id])) {
                out.push_back(id);
                used[id] = 1;
            }
        }
        return out;
    }

    static BStarState makeBStarStateFromOrder(
        const Design& design,
        const vector<int>& rawOrder,
        const ShapeState& st,
        double W,
        double H,
        bool strictEdge
    ) {
        vector<int> order = filterMovableOrder(design, rawOrder);
        BStarState bs;
        bs.shape = st;
        bs.W = clampD(W, 1.0, design.maxOutlineW);
        bs.H = clampD(H, 1.0, design.maxOutlineH);
        bs.strictEdge = strictEdge;
        const int m = static_cast<int>(order.size());
        bs.node.assign(m, BStarNode{});
        bs.root = (m > 0 ? 0 : -1);
        for (int i = 0; i < m; ++i) {
            bs.node[i].block = order[i];
            bs.node[i].parent = (i == 0) ? -1 : (i - 1) / 2;
            bs.node[i].left = (2 * i + 1 < m) ? 2 * i + 1 : -1;
            bs.node[i].right = (2 * i + 2 < m) ? 2 * i + 2 : -1;
        }
        return bs;
    }

    static ShapeState shapeStateFromLayout(const Design& design, const LayoutResult& layout) {
        ShapeState st;
        st.ratio.assign(design.blockSpecs.size(), 1.0);
        for (int i = 0; i < static_cast<int>(design.blockSpecs.size()); ++i) {
            const BlockSpec& sp = design.blockSpecs[i];
            double r = aspectMid(sp);
            if (i < static_cast<int>(layout.rects.size()) && layout.rects[i].h > EPS) {
                r = layout.rects[i].w / layout.rects[i].h;
            }
            const double amin = max(0.05, sp.aspectMin);
            const double amax = max(amin, sp.aspectMax);
            st.ratio[i] = clampD(r, amin, amax);
        }
        return st;
    }

    static vector<int> geometryOrderFromLayout(const Design& design, const LayoutResult& layout, int mode) {
        vector<int> ids = movableIdsOf(design);
        auto rectOf = [&](int id) -> Rect {
            if (id >= 0 && id < static_cast<int>(layout.rects.size())) return layout.rects[id];
            return makePackingShape(design, id, aspectMid(design.blockSpecs[id]));
            };
        sort(ids.begin(), ids.end(), [&](int a, int b) {
            const Rect ra = rectOf(a);
            const Rect rb = rectOf(b);
            const double ax = rectCx(ra), ay = rectCy(ra);
            const double bx = rectCx(rb), by = rectCy(rb);
            if (mode == 0) {
                if (fabs(ax - bx) > 1.0e-6) return ax < bx;
                if (fabs(ay - by) > 1.0e-6) return ay < by;
            }
            else if (mode == 1) {
                if (fabs(ay - by) > 1.0e-6) return ay < by;
                if (fabs(ax - bx) > 1.0e-6) return ax < bx;
            }
            else if (mode == 2) {
                if (fabs(ax + ay - bx - by) > 1.0e-6) return ax + ay < bx + by;
                if (fabs(ax - bx) > 1.0e-6) return ax < bx;
            }
            else {
                const double ac = fabs(ax - 0.5 * layout.W) + fabs(ay - 0.5 * layout.H);
                const double bc = fabs(bx - 0.5 * layout.W) + fabs(by - 0.5 * layout.H);
                if (fabs(ac - bc) > 1.0e-6) return ac < bc;
            }
            return a < b;
            });
        return ids;
    }

    static void pushUniqueBStarSeed(vector<BStarState>& seeds, const BStarState& seed) {
        if (!bstarTreeLegal(seed)) return;
        vector<int> order;
        for (int u : bstarPreorder(seed)) {
            if (u >= 0 && u < static_cast<int>(seed.node.size())) order.push_back(seed.node[u].block);
        }
        for (const BStarState& old : seeds) {
            if (fabs(old.W - seed.W) > 4.0 || fabs(old.H - seed.H) > 4.0) continue;
            vector<int> oldOrder;
            for (int u : bstarPreorder(old)) {
                if (u >= 0 && u < static_cast<int>(old.node.size())) oldOrder.push_back(old.node[u].block);
            }
            if (oldOrder == order) return;
        }
        seeds.push_back(seed);
    }

    static vector<BStarState> makeWireBStarSeeds(
        const Design& design,
        const BStarState& anchorState,
        const vector<LayoutResult>& layouts,
        bool spreadMode
    ) {
        vector<BStarState> seeds;
        pushUniqueBStarSeed(seeds, anchorState);
        const int layoutLimit = min(5, static_cast<int>(layouts.size()));
        for (int li = 0; li < layoutLimit && static_cast<int>(seeds.size()) < BSTAR_WIRE_SEED_LIMIT; ++li) {
            const LayoutResult& layout = layouts[li];
            if (!layout.legal || layout.rects.empty()) continue;
            ShapeState st = shapeStateFromLayout(design, layout);
            const double scale = spreadMode ? 1.035 : 1.000;
            const double W = clampD(layout.W * scale, minWidthBound(design, st), design.maxOutlineW);
            const double H = clampD(layout.H * scale, minHeightBound(design, st), design.maxOutlineH);
            for (int mode = 0; mode < 4 && static_cast<int>(seeds.size()) < BSTAR_WIRE_SEED_LIMIT; ++mode) {
                pushUniqueBStarSeed(seeds, makeBStarStateFromOrder(
                    design, geometryOrderFromLayout(design, layout, mode), st, W, H, true));
            }
        }

        vector<vector<int>> orders = makeOrders(design);
        vector<ShapeState> states = makeShapeStates(design);
        const int orderLimit = min(4, static_cast<int>(orders.size()));
        const int shapeLimit = min(3, static_cast<int>(states.size()));
        const double packArea = max(1.0, totalPackingArea(design));
        for (int si = 0; si < shapeLimit && static_cast<int>(seeds.size()) < BSTAR_WIRE_SEED_LIMIT; ++si) {
            const ShapeState& st = states[si];
            double aspect = design.maxOutlineH > EPS ? design.maxOutlineW / design.maxOutlineH : 1.0;
            aspect = clampD(aspect, 0.45, 2.20);
            double targetArea = packArea * (spreadMode ? 1.32 : 1.14);
            double W = sqrt(targetArea * aspect);
            double H = targetArea / max(1.0, W);
            W = clampD(W, minWidthBound(design, st), design.maxOutlineW);
            H = clampD(H, minHeightBound(design, st), design.maxOutlineH);
            for (int oi = 0; oi < orderLimit && static_cast<int>(seeds.size()) < BSTAR_WIRE_SEED_LIMIT; ++oi) {
                pushUniqueBStarSeed(seeds, makeBStarStateFromOrder(design, orders[oi], st, W, H, true));
            }
        }
        return seeds;
    }

    static void bstarPreorderDfs(const vector<BStarNode>& node, int u, vector<int>& out) {
        if (u < 0 || u >= static_cast<int>(node.size())) return;
        out.push_back(u);
        bstarPreorderDfs(node, node[u].left, out);
        bstarPreorderDfs(node, node[u].right, out);
    }

    static vector<int> bstarPreorder(const BStarState& bs) {
        vector<int> out;
        out.reserve(bs.node.size());
        bstarPreorderDfs(bs.node, bs.root, out);
        return out;
    }

    static bool bstarIsLeaf(const BStarState& bs, int u) {
        return u >= 0 && u < static_cast<int>(bs.node.size()) && bs.node[u].left < 0 && bs.node[u].right < 0;
    }

    static void bstarCollectSubtree(const BStarState& bs, int u, vector<int>& out) {
        if (u < 0 || u >= static_cast<int>(bs.node.size())) return;
        out.push_back(u);
        bstarCollectSubtree(bs, bs.node[u].left, out);
        bstarCollectSubtree(bs, bs.node[u].right, out);
    }

    static bool bstarIsDescendant(const BStarState& bs, int ancestor, int maybeDesc) {
        if (ancestor < 0 || maybeDesc < 0) return false;
        vector<int> sub;
        bstarCollectSubtree(bs, ancestor, sub);
        return find(sub.begin(), sub.end(), maybeDesc) != sub.end();
    }

    static bool bstarTreeLegal(const BStarState& bs) {
        const int m = static_cast<int>(bs.node.size());
        if (m == 0) return bs.root < 0;
        if (bs.root < 0 || bs.root >= m) return false;
        vector<int> seen;
        bstarPreorderDfs(bs.node, bs.root, seen);
        if (static_cast<int>(seen.size()) != m) return false;
        vector<char> mark(m, 0);
        for (int u : seen) {
            if (u < 0 || u >= m || mark[u]) return false;
            mark[u] = 1;
            int l = bs.node[u].left, r = bs.node[u].right;
            if (l >= 0 && (l >= m || bs.node[l].parent != u)) return false;
            if (r >= 0 && (r >= m || bs.node[r].parent != u)) return false;
            if (l == r && l >= 0) return false;
        }
        if (bs.node[bs.root].parent != -1) return false;
        return true;
    }

    static double bstarDeadspace(const Design& design, const LayoutResult& r) {
        if (r.area <= 1.0) return 0.0;
        return clampD((r.area - totalPackingArea(design)) / r.area, 0.0, 0.95);
    }

    static void addLimitedCoord(vector<double>& xs, double x, double lo, double hi) {
        addCoord(xs, x, lo, hi);
    }

    static void uniquePrune(vector<double>& v, int limit) {
        sort(v.begin(), v.end());
        v.erase(unique(v.begin(), v.end(), [](double a, double b) { return fabs(a - b) < 1e-5; }), v.end());
        pruneCoords(v, limit);
    }

    static double routeAwareMinYAtX(
        const Design& design,
        int id,
        double x,
        double w,
        double h,
        double baseY,
        const vector<Rect>& placed,
        const vector<int>& placedIds
    ) {
        double y = max(0.0, baseY);
        for (int pass = 0; pass < static_cast<int>(placed.size()) + 8; ++pass) {
            bool changed = false;
            for (int k = 0; k < static_cast<int>(placed.size()); ++k) {
                const Rect& o = placed[k];
                int oid = (k < static_cast<int>(placedIds.size())) ? placedIds[k] : -1;
                if (oid < 0) continue;
                if (ovLen(x, x + w, o.x, rectRight(o)) <= EPS) continue;
                const double gy = requiredYGap(design, id, oid);
                // Bottom-left contour semantics: candidate is moved upward until it
                // clears the obstacle plus route-aware vertical spacing.
                if (y < rectTop(o) + gy && y + h + gy > o.y) {
                    y = rectTop(o) + gy;
                    changed = true;
                }
            }
            if (!changed) break;
        }
        return y;
    }

    static bool chooseRootPlacement(
        const Design& design,
        const vector<Rect>& placed,
        const vector<int>& placedIds,
        const vector<Rect>& rectsSoFar,
        const vector<char>& placedMask,
        int id,
        const Rect& shape,
        double W,
        double H,
        Rect& out
    ) {
        if (shape.w > W + EPS || shape.h > H + EPS) return false;
        const double maxX = W - shape.w;
        const double maxY = H - shape.h;
        vector<double> xs, ys;
        addLimitedCoord(xs, 0.0, 0.0, maxX);
        addLimitedCoord(ys, 0.0, 0.0, maxY);
        addLimitedCoord(xs, maxX, 0.0, maxX);
        addLimitedCoord(ys, maxY, 0.0, maxY);
        for (int k = 0; k < static_cast<int>(placed.size()); ++k) {
            const Rect& o = placed[k];
            int oid = (k < static_cast<int>(placedIds.size())) ? placedIds[k] : -1;
            const double gx = (oid >= 0) ? requiredXGap(design, id, oid) : PACK_GAP;
            const double gy = (oid >= 0) ? requiredYGap(design, id, oid) : PACK_GAP;
            addLimitedCoord(xs, rectRight(o) + gx, 0.0, maxX);
            addLimitedCoord(xs, o.x - shape.w - gx, 0.0, maxX);
            addLimitedCoord(xs, o.x, 0.0, maxX);
            addLimitedCoord(xs, rectRight(o) - shape.w, 0.0, maxX);
            addLimitedCoord(ys, rectTop(o) + gy, 0.0, maxY);
            addLimitedCoord(ys, o.y - shape.h - gy, 0.0, maxY);
            addLimitedCoord(ys, o.y, 0.0, maxY);
            addLimitedCoord(ys, rectTop(o) - shape.h, 0.0, maxY);
        }
        uniquePrune(xs, BSTAR_ROOT_CAND_LIMIT);
        uniquePrune(ys, BSTAR_ROOT_CAND_LIMIT);

        bool found = false;
        PlaceKey bestKey;
        Rect best = shape;
        for (double y0 : ys) {
            for (double x : xs) {
                double y = routeAwareMinYAtX(design, id, x, shape.w, shape.h, y0, placed, placedIds);
                if (y > maxY + EPS) continue;
                Rect cand = shape;
                cand.x = x;
                cand.y = y;
                if (!inside(cand, W, H)) continue;
                if (anyOverlapWith(cand, placed)) continue;
                PlaceKey key;
                key.top = rectTop(cand);
                key.route = routeGapPenaltyForCandidate(design, placed, placedIds, id, cand);
                key.right = rectRight(cand);
                key.y = cand.y;
                key.wire = partialWire(design, rectsSoFar, placedMask, id, cand);
                key.center = fabs(rectCx(cand) - 0.5 * W) + fabs(rectCy(cand) - 0.5 * H);
                key.x = cand.x;
                fillPlacementPressure(key, placed, cand, W, H);
                if (!found || betterKey(key, bestKey)) {
                    found = true;
                    bestKey = key;
                    best = cand;
                }
            }
        }
        if (!found) return false;
        out = best;
        return true;
    }

    static bool chooseBStarChildPlacement(
        const Design& design,
        const vector<Rect>& placed,
        const vector<int>& placedIds,
        const vector<Rect>& rectsSoFar,
        const vector<char>& placedMask,
        int id,
        const Rect& shape,
        const Rect& parentRect,
        int parentId,
        bool isLeftChild,
        double W,
        double H,
        Rect& out
    ) {
        if (shape.w > W + EPS || shape.h > H + EPS) return false;
        const double maxX = W - shape.w;
        const double maxY = H - shape.h;
        const double gx = requiredXGap(design, parentId, id);
        const double gy = requiredYGap(design, parentId, id);
        const double intendedX = isLeftChild ? rectRight(parentRect) + gx : parentRect.x;
        const double baseY = isLeftChild ? 0.0 : rectTop(parentRect) + gy;

        vector<double> xs;
        addLimitedCoord(xs, intendedX, 0.0, maxX);
        if (!isLeftChild) {
            addLimitedCoord(xs, rectRight(parentRect) - shape.w, 0.0, maxX);
            addLimitedCoord(xs, rectCx(parentRect) - 0.5 * shape.w, 0.0, maxX);
        }
        else {
            addLimitedCoord(xs, rectRight(parentRect) + gx, 0.0, maxX);
            addLimitedCoord(xs, parentRect.x, 0.0, maxX);
        }
        for (int k = 0; k < static_cast<int>(placed.size()); ++k) {
            const Rect& o = placed[k];
            int oid = (k < static_cast<int>(placedIds.size())) ? placedIds[k] : -1;
            if (oid < 0) continue;
            const double gox = requiredXGap(design, id, oid);
            // Route-aware escape candidates are still packed by the B*-tree relation,
            // but a few local x alternatives avoid making every failed obstacle a hard reject.
            addLimitedCoord(xs, rectRight(o) + gox, 0.0, maxX);
            addLimitedCoord(xs, o.x - shape.w - gox, 0.0, maxX);
            addLimitedCoord(xs, o.x, 0.0, maxX);
            addLimitedCoord(xs, rectRight(o) - shape.w, 0.0, maxX);
        }
        uniquePrune(xs, BSTAR_CHILD_X_CAND_LIMIT);

        vector<double> ySeeds;
        addLimitedCoord(ySeeds, baseY, 0.0, maxY);
        uniquePrune(ySeeds, 3);

        bool found = false;
        PlaceKey bestKey;
        Rect best = shape;
        for (double x : xs) {
            for (double ySeed : ySeeds) {
                double y = routeAwareMinYAtX(design, id, x, shape.w, shape.h, ySeed, placed, placedIds);
                if (y > maxY + EPS) continue;
                Rect cand = shape;
                cand.x = x;
                cand.y = y;
                if (!inside(cand, W, H)) continue;
                if (anyOverlapWith(cand, placed)) continue;
                PlaceKey key;
                key.top = rectTop(cand);
                key.route = routeGapPenaltyForCandidate(design, placed, placedIds, id, cand);
                key.right = rectRight(cand);
                key.y = cand.y;
                key.wire = partialWire(design, rectsSoFar, placedMask, id, cand);
                key.center = fabs(x - intendedX) * BSTAR_PACK_DEVIATION_WEIGHT +
                    fabs(rectCx(cand) - 0.5 * W) * CENTER_TIE_WEIGHT;
                key.x = x;
                fillPlacementPressure(key, placed, cand, W, H);
                if (!found || betterKey(key, bestKey)) {
                    found = true;
                    bestKey = key;
                    best = cand;
                }
            }
        }
        if (!found) return false;
        out = best;
        return true;
    }

    static bool packBStarState(const Design& design, const BStarState& bs, LayoutResult& out) {
        if (!bstarTreeLegal(bs)) return false;
        if (bs.W > design.maxOutlineW + EPS || bs.H > design.maxOutlineH + EPS) return false;
        if (bs.W < minWidthBound(design, bs.shape) - EPS || bs.H < minHeightBound(design, bs.shape) - EPS) return false;

        vector<Rect> shapes = makeShapes(design, bs.shape);
        vector<Rect> rects;
        vector<Rect> edgeObstacles;
        if (!placeEdgesFast(design, shapes, bs.W, bs.H, rects, edgeObstacles, bs.strictEdge)) return false;

        vector<Rect> placed;
        vector<int> placedIds;
        vector<char> placedMask(design.blockSpecs.size(), 0);
        for (int i = 0; i < static_cast<int>(design.blockSpecs.size()); ++i) {
            if (design.blockSpecs[i].type == BlockType::EDGE) {
                if (!inside(rects[i], bs.W, bs.H)) return false;
                placed.push_back(rects[i]);
                placedIds.push_back(i);
                placedMask[i] = 1;
            }
        }

        vector<int> order = bstarPreorder(bs);
        for (int u : order) {
            const int id = bs.node[u].block;
            if (id < 0 || id >= static_cast<int>(design.blockSpecs.size())) return false;
            Rect shape = shapes[id];
            Rect cand;
            if (u == bs.root) {
                if (!chooseRootPlacement(design, placed, placedIds, rects, placedMask, id, shape, bs.W, bs.H, cand)) return false;
            }
            else {
                int p = bs.node[u].parent;
                if (p < 0 || p >= static_cast<int>(bs.node.size())) return false;
                const int pid = bs.node[p].block;
                if (pid < 0 || pid >= static_cast<int>(rects.size()) || !placedMask[pid]) return false;
                bool isLeftChild = (bs.node[p].left == u);
                if (!chooseBStarChildPlacement(design, placed, placedIds, rects, placedMask,
                    id, shape, rects[pid], pid, isLeftChild, bs.W, bs.H, cand)) return false;
            }
            rects[id] = cand;
            placed.push_back(cand);
            placedIds.push_back(id);
            placedMask[id] = 1;
        }

        for (int i = 0; i < static_cast<int>(design.blockSpecs.size()); ++i) {
            if (!placedMask[i]) return false;
        }

        out = scoreLayout(design, bs.W, bs.H, std::move(rects));
        return out.legal && out.strictEdgeLegal && out.overlap <= EPS && out.outlineViol <= EPS;
    }

    static double bstarAreaCost(const LayoutResult& r) {
        return r.area
            + BSTAR_HPWL_COST_WEIGHT * r.hpwl
            + BSTAR_ROUTE_COST_WEIGHT * r.routePenalty;
    }

    static double bstarCheckerProxyCost(const Design& design, const LayoutResult& r) {
        if (!r.legal || !r.strictEdgeLegal ||
            !std::isfinite(r.area) || r.area >= INF * 0.5) {
            return INF * 0.5;
        }

        const double area = max(1.0, r.area);
        const double alpha = design.alpha > 0.0 ? design.alpha : 0.1;
        const double routeTerm = min(
            BSTAR_SECOND_ROUTE_PROXY_CAP * area,
            BSTAR_ROUTE_COST_WEIGHT * max(0.0, r.routePenalty));
        const double peak = max(0.0, r.stripPeakUtilProxy - STRIP_PROXY_TARGET_UTIL);
        const double rawStripTerm =
            STRIP_PROXY_OVERFLOW_WEIGHT * max(0.0, r.stripOverflowProxy) +
            STRIP_PROXY_RISK_WEIGHT * max(0.0, r.stripRiskProxy) +
            STRIP_PROXY_PEAK_WEIGHT * peak * peak * area;
        const double stripTerm = min(BSTAR_SECOND_STRIP_PROXY_CAP * area, rawStripTerm);
        return area + alpha * max(0.0, r.hpwl) + routeTerm + stripTerm;
    }

    static bool bstarAreaBetter(const LayoutResult& a, const LayoutResult& b) {
        if (a.legal != b.legal) return a.legal;
        if (a.strictEdgeLegal != b.strictEdgeLegal) return a.strictEdgeLegal;
        const double ac = bstarAreaCost(a);
        const double bc = bstarAreaCost(b);
        if (fabs(ac - bc) > max(1.0, 1.0e-6 * min(ac, bc))) return ac < bc;
        if (fabs(a.routePenalty - b.routePenalty) > 1.0) return a.routePenalty < b.routePenalty;
        if (fabs(a.area - b.area) > max(1.0, 1.0e-6 * min(a.area, b.area))) return a.area < b.area;
        return a.hpwl < b.hpwl;
    }

    static bool bstarCheckerProxyBetter(const Design& design, const LayoutResult& a, const LayoutResult& b) {
        if (a.legal != b.legal) return a.legal;
        if (a.strictEdgeLegal != b.strictEdgeLegal) return a.strictEdgeLegal;
        if (!std::isfinite(b.area) || b.area >= INF * 0.5) return true;
        const double ac = bstarCheckerProxyCost(design, a);
        const double bc = bstarCheckerProxyCost(design, b);
        if (fabs(ac - bc) > max(1.0, 1.0e-6 * min(ac, bc))) return ac < bc;
        if (fabs(a.area - b.area) > max(1.0, 1.0e-6 * min(a.area, b.area))) return a.area < b.area;
        if (fabs(a.stripPeakUtilProxy - b.stripPeakUtilProxy) > 0.01) return a.stripPeakUtilProxy < b.stripPeakUtilProxy;
        return a.hpwl < b.hpwl;
    }

    static double bstarWireProxyCost(const Design& design, const LayoutResult& r, bool spreadMode) {
        if (!r.legal || !r.strictEdgeLegal ||
            !std::isfinite(r.area) || r.area >= INF * 0.5) {
            return INF * 0.5;
        }

        const double area = max(1.0, r.area);
        const double alpha = design.alpha > 0.0 ? design.alpha : 0.1;
        const double wl = max(0.0, portAwareHpwl(design, r.rects));
        const double routeTerm = min(
            BSTAR_WIRE_ROUTE_PROXY_CAP * area,
            0.010 * max(0.0, r.routePenalty));
        const double peak = max(0.0, r.stripPeakUtilProxy - STRIP_PROXY_TARGET_UTIL);
        const double rawStripTerm =
            STRIP_PROXY_OVERFLOW_WEIGHT * max(0.0, r.stripOverflowProxy) +
            STRIP_PROXY_RISK_WEIGHT * max(0.0, r.stripRiskProxy) +
            STRIP_PROXY_PEAK_WEIGHT * peak * peak * area;
        const double stripTerm = min(BSTAR_WIRE_STRIP_PROXY_CAP * area, rawStripTerm);
        const double violTerm = static_cast<double>(max(0, r.routeViolPairs)) * 0.18 * area;
        const double areaWeight = spreadMode ? BSTAR_SPREAD_AREA_WEIGHT : 1.0;
        return areaWeight * area + alpha * wl + routeTerm + stripTerm + violTerm;
    }

    static bool bstarWireProxyBetter(const Design& design, const LayoutResult& a, const LayoutResult& b, bool spreadMode) {
        if (a.legal != b.legal) return a.legal;
        if (a.strictEdgeLegal != b.strictEdgeLegal) return a.strictEdgeLegal;
        if (!std::isfinite(b.area) || b.area >= INF * 0.5) return true;
        const double ac = bstarWireProxyCost(design, a, spreadMode);
        const double bc = bstarWireProxyCost(design, b, spreadMode);
        if (fabs(ac - bc) > max(1.0, 1.0e-6 * min(ac, bc))) return ac < bc;
        if (fabs(a.hpwl - b.hpwl) > max(1.0, 1.0e-6 * min(a.hpwl, b.hpwl))) return a.hpwl < b.hpwl;
        return a.area < b.area;
    }

    static bool warmStateToMovableBStar(
        const Design& design,
        const phase1_warm_start::FastSolver::Result& warm,
        BStarState& out
    ) {
        phase1_warm_start::State projected = warm.state;
        const int nodeCount = static_cast<int>(projected.moduleAt.size());
        if (nodeCount != static_cast<int>(design.blockSpecs.size())) return false;

        vector<pair<int, int>> edgeNodes;
        vector<pair<int, int>> stack;
        if (projected.root >= 0) stack.push_back({ projected.root, 0 });
        while (!stack.empty()) {
            const auto [node, depth] = stack.back();
            stack.pop_back();
            if (node < 0 || node >= nodeCount) return false;
            const int block = projected.moduleAt[node];
            if (block < 0 || block >= nodeCount) return false;
            if (design.blockSpecs[block].type == BlockType::EDGE)
                edgeNodes.push_back({ depth, node });
            if (projected.left[node] >= 0)
                stack.push_back({ projected.left[node], depth + 1 });
            if (projected.right[node] >= 0)
                stack.push_back({ projected.right[node], depth + 1 });
        }
        sort(edgeNodes.rbegin(), edgeNodes.rend());
        try {
            for (const auto& item : edgeNodes)
                phase1_warm_start::deleteBtreeNode(
                    projected, item.second, nullptr);
        }
        catch (const exception&) {
            return false;
        }

        vector<int> active;
        vector<int> visit;
        if (projected.root >= 0) visit.push_back(projected.root);
        vector<char> seen(nodeCount, 0);
        while (!visit.empty()) {
            const int node = visit.back();
            visit.pop_back();
            if (node < 0 || node >= nodeCount || seen[node]) return false;
            seen[node] = 1;
            const int block = projected.moduleAt[node];
            if (block < 0 || block >= nodeCount ||
                !isMovableBlock(design.blockSpecs[block])) return false;
            active.push_back(node);
            if (projected.right[node] >= 0)
                visit.push_back(projected.right[node]);
            if (projected.left[node] >= 0)
                visit.push_back(projected.left[node]);
        }
        if (active.size() != movableIdsOf(design).size()) return false;

        vector<int> remap(nodeCount, -1);
        for (int i = 0; i < static_cast<int>(active.size()); ++i)
            remap[active[i]] = i;

        out = BStarState{};
        out.root = active.empty() ? -1 : remap[projected.root];
        out.node.assign(active.size(), BStarNode{});
        for (int i = 0; i < static_cast<int>(active.size()); ++i) {
            const int old = active[i];
            BStarNode& node = out.node[i];
            node.block = projected.moduleAt[old];
            node.parent = projected.parent[old] < 0 ?
                -1 : remap[projected.parent[old]];
            node.left = projected.left[old] < 0 ?
                -1 : remap[projected.left[old]];
            node.right = projected.right[old] < 0 ?
                -1 : remap[projected.right[old]];
            if ((projected.parent[old] >= 0 && node.parent < 0) ||
                (projected.left[old] >= 0 && node.left < 0) ||
                (projected.right[old] >= 0 && node.right < 0))
                return false;
        }

        out.shape.ratio.assign(design.blockSpecs.size(), 1.0);
        for (int id = 0; id < static_cast<int>(design.blockSpecs.size()); ++id) {
            if (design.blockSpecs[id].type == BlockType::SOFT &&
                id < static_cast<int>(projected.softAr.size()))
                out.shape.ratio[id] = clampD(projected.softAr[id],
                    max(0.05, design.blockSpecs[id].aspectMin),
                    max(max(0.05, design.blockSpecs[id].aspectMin),
                        design.blockSpecs[id].aspectMax));
            else
                out.shape.ratio[id] = aspectMid(design.blockSpecs[id]);
        }
        out.W = clampD(warm.placement.W, 1.0, design.maxOutlineW);
        out.H = clampD(warm.placement.H, 1.0, design.maxOutlineH);
        out.strictEdge = true;
        return bstarTreeLegal(out);
    }

    static vector<BStarState> makePhase1WarmBStarSeeds(const Design& design) {
        vector<BStarState> seeds;
        if (!ENABLE_PHASE1_WARM_START) return seeds;

        const int n = static_cast<int>(design.blockSpecs.size());
        const int restartCount = n <= 8 ? PHASE1_WARM_RESTARTS_SMALL :
            (n <= 18 ? PHASE1_WARM_RESTARTS_MEDIUM :
                PHASE1_WARM_RESTARTS_LARGE);
        try {
            phase1_warm_start::Problem problem =
                phase1_warm_start::makeProblem(design);
            optional<tuple<double, double, int>> sharedNormalizers;
            for (int restart = 0; restart < restartCount; ++restart) {
                const unsigned seed =
                    phase1_warm_start::DEFAULT_SEED +
                    1000003u * static_cast<unsigned>(restart);
                phase1_warm_start::FastSolver solver(
                    problem, seed, sharedNormalizers);
                auto result = solver.solve();
                if (!sharedNormalizers)
                    sharedNormalizers = solver.normalizers();
                BStarState converted;
                if (warmStateToMovableBStar(design, result, converted)) {
                    vector<int> preorderBlocks;
                    vector<int> stack;
                    if (result.state.root >= 0)
                        stack.push_back(result.state.root);
                    while (!stack.empty()) {
                        const int node = stack.back();
                        stack.pop_back();
                        const int block = result.state.moduleAt[node];
                        if (isMovableBlock(design.blockSpecs[block]))
                            preorderBlocks.push_back(block);
                        if (result.state.right[node] >= 0)
                            stack.push_back(result.state.right[node]);
                        if (result.state.left[node] >= 0)
                            stack.push_back(result.state.left[node]);
                    }
                    if (!preorderBlocks.empty()) {
                        seeds.push_back(makeBStarStateFromOrder(
                            design, preorderBlocks, converted.shape,
                            converted.W, converted.H, true));
                    }

                    vector<int> geometricOrder = preorderBlocks;
                    sort(geometricOrder.begin(), geometricOrder.end(),
                        [&](int lhs, int rhs) {
                            const Rect& a = result.placement.rects[lhs];
                            const Rect& b = result.placement.rects[rhs];
                            const double ay = rectCy(a);
                            const double by = rectCy(b);
                            if (fabs(ay - by) > 1.0e-6) return ay < by;
                            const double ax = rectCx(a);
                            const double bx = rectCx(b);
                            if (fabs(ax - bx) > 1.0e-6) return ax < bx;
                            return lhs < rhs;
                        });
                    if (!geometricOrder.empty()) {
                        seeds.push_back(makeBStarStateFromOrder(
                            design, geometricOrder, converted.shape,
                            converted.W, converted.H, true));
                    }
                }
            }
        }
        catch (const exception& ex) {
            if (FAST_VERBOSE_LOG)
                cerr << "[Phase1WarmStart] disabled after exception: "
                    << ex.what() << "\n";
        }
        return seeds;
    }

    static bool makeInitialBStarLegalState(
        const Design& design,
        BStarState& bestState,
        LayoutResult& bestLayout,
        vector<LayoutResult>* warmArchiveOut = nullptr
    ) {
        vector<vector<int>> orders = makeOrders(design);
        vector<ShapeState> states = makeShapeStates(design);
        if (states.empty()) {
            ShapeState st;
            st.ratio.assign(design.blockSpecs.size(), 1.0);
            for (int i = 0; i < static_cast<int>(design.blockSpecs.size()); ++i) st.ratio[i] = aspectMid(design.blockSpecs[i]);
            states.push_back(st);
        }
        bool warmHave = false;
        BStarState warmBestState;
        LayoutResult warmBestLayout;
        int tried = 0;

        const auto warmSolverStart = chrono::steady_clock::now();
        vector<BStarState> warmSeeds = makePhase1WarmBStarSeeds(design);
        const auto warmPackingStart = chrono::steady_clock::now();
        int warmPacked = 0;
        int warmTried = 0;
        for (const BStarState& seed : warmSeeds) {
            vector<double> warmWidths = makeWidthTrials(design, seed.shape);
            warmWidths.push_back(seed.W);
            warmWidths.push_back(design.maxOutlineW);
            sort(warmWidths.begin(), warmWidths.end());
            warmWidths.erase(unique(warmWidths.begin(), warmWidths.end(),
                [](double a, double b) { return fabs(a - b) < 1.0e-5; }),
                warmWidths.end());
            vector<pair<double, double>> outlineTrials;
            for (double width : warmWidths) {
                vector<double> warmHeights =
                    makeHeightTrials(design, seed.shape, width);
                warmHeights.push_back(seed.H);
                warmHeights.push_back(design.maxOutlineH);
                sort(warmHeights.begin(), warmHeights.end());
                warmHeights.erase(unique(warmHeights.begin(), warmHeights.end(),
                    [](double a, double b) { return fabs(a - b) < 1.0e-5; }),
                    warmHeights.end());
                for (double height : warmHeights) {
                    outlineTrials.push_back({
                        clampD(width, minWidthBound(design, seed.shape),
                            design.maxOutlineW),
                        clampD(height, minHeightBound(design, seed.shape),
                            design.maxOutlineH)
                        });
                }
            }
            sort(outlineTrials.begin(), outlineTrials.end(),
                [](const auto& lhs, const auto& rhs) {
                    const double lhsArea = lhs.first * lhs.second;
                    const double rhsArea = rhs.first * rhs.second;
                    if (fabs(lhsArea - rhsArea) > 1.0e-6)
                        return lhsArea < rhsArea;
                    return lhs < rhs;
                });
            outlineTrials.erase(unique(outlineTrials.begin(),
                outlineTrials.end(), [](const auto& lhs, const auto& rhs) {
                    return fabs(lhs.first - rhs.first) < 1.0e-5 &&
                        fabs(lhs.second - rhs.second) < 1.0e-5;
                    }), outlineTrials.end());
            for (const auto& outline : outlineTrials) {
                ++warmTried;
                BStarState bs = seed;
                bs.W = outline.first;
                bs.H = outline.second;
                LayoutResult cur;
                if (!packBStarState(design, bs, cur)) continue;
                ++warmPacked;
                if (warmArchiveOut)
                    addToBStarArchive(design, *warmArchiveOut, cur);
                if (!warmHave || bstarAreaBetter(cur, warmBestLayout)) {
                    warmHave = true;
                    warmBestState = std::move(bs);
                    warmBestLayout = std::move(cur);
                }
                break;
            }
        }
        const auto warmEnd = chrono::steady_clock::now();
        if (FAST_VERBOSE_LOG) {
            cerr << fixed << setprecision(3)
                << "[Phase1WarmStart] generated=" << warmSeeds.size()
                << " tried=" << warmTried
                << " repacked=" << warmPacked
                << " solverMs=" <<
                    chrono::duration_cast<chrono::milliseconds>(
                        warmPackingStart - warmSolverStart).count()
                << " repackMs=" <<
                    chrono::duration_cast<chrono::milliseconds>(
                        warmEnd - warmPackingStart).count();
            if (warmHave)
                cerr << " bestArea=" << warmBestLayout.area
                    << " bestHpwl=" << warmBestLayout.hpwl
                    << " bestRouteGap=" << warmBestLayout.routePenalty;
            cerr << "\n";
        }

        bool have = false;
        for (const ShapeState& st : states) {
            vector<double> widths = makeWidthTrials(design, st);
            // Prefer smaller outlines first, but include the full outline as recovery.
            sort(widths.begin(), widths.end());
            for (const vector<int>& order : orders) {
                for (double W : widths) {
                    vector<double> heights = makeHeightTrials(design, st, W);
                    sort(heights.begin(), heights.end());
                    for (double H : heights) {
                        if (++tried > BSTAR_INIT_TRIAL_CAP && have) return true;
                        BStarState bs = makeBStarStateFromOrder(design, order, st, W, H, true);
                        LayoutResult cur;
                        if (!packBStarState(design, bs, cur)) continue;
                        if (!have || bstarAreaBetter(cur, bestLayout)) {
                            have = true;
                            bestState = std::move(bs);
                            bestLayout = std::move(cur);
                        }
                    }
                }
            }
        }
        if (!have) {
            ShapeState st = states.front();
            vector<int> order = degreeOrder(design);
            BStarState bs = makeBStarStateFromOrder(design, order, st, design.maxOutlineW, design.maxOutlineH, false);
            LayoutResult cur;
            if (packBStarState(design, bs, cur)) {
                bestState = std::move(bs);
                bestLayout = std::move(cur);
                have = true;
            }
        }
        if (!have && warmHave) {
            bestState = std::move(warmBestState);
            bestLayout = std::move(warmBestLayout);
            have = true;
        }
        return have;
    }

    static int pickHotMovableNode(const Design& design, const BStarState& bs, mt19937& rng) {
        vector<double> weights(bs.node.size(), 0.0);
        double sum = 0.0;
        for (int u = 0; u < static_cast<int>(bs.node.size()); ++u) {
            const int id = bs.node[u].block;
            double w = 1.0 + sqrt(max(0.0, endpointDemand(design, id)));
            w *= 1.0 + 2.0 * softFtExpansionRate(design, id);
            weights[u] = w;
            sum += w;
        }
        if (sum <= 0.0) return uniform_int_distribution<int>(0, max(0, static_cast<int>(bs.node.size()) - 1))(rng);
        double r = uniform_real_distribution<double>(0.0, sum)(rng);
        for (int u = 0; u < static_cast<int>(weights.size()); ++u) {
            r -= weights[u];
            if (r <= 0.0) return u;
        }
        return static_cast<int>(weights.size()) - 1;
    }

    static void bstarSwapBlocks(BStarState& bs, int a, int b) {
        if (a == b || a < 0 || b < 0 || a >= static_cast<int>(bs.node.size()) || b >= static_cast<int>(bs.node.size())) return;
        swap(bs.node[a].block, bs.node[b].block);
    }

    static bool bstarMoveLeaf(BStarState& bs, mt19937& rng) {
        const int m = static_cast<int>(bs.node.size());
        if (m <= 1) return false;
        vector<int> leaves;
        for (int u = 0; u < m; ++u) {
            if (u != bs.root && bstarIsLeaf(bs, u)) leaves.push_back(u);
        }
        if (leaves.empty()) return false;
        int u = leaves[uniform_int_distribution<int>(0, static_cast<int>(leaves.size()) - 1)(rng)];
        int pOld = bs.node[u].parent;
        if (pOld < 0) return false;
        if (bs.node[pOld].left == u) bs.node[pOld].left = -1;
        else if (bs.node[pOld].right == u) bs.node[pOld].right = -1;
        bs.node[u].parent = -1;

        int target = -1;
        for (int t = 0; t < 32; ++t) {
            int cand = uniform_int_distribution<int>(0, m - 1)(rng);
            if (cand == u) continue;
            target = cand;
            break;
        }
        if (target < 0) {
            // restore
            bs.node[u].parent = pOld;
            if (bs.node[pOld].left < 0) bs.node[pOld].left = u;
            else bs.node[pOld].right = u;
            return false;
        }
        bool leftSide = uniform_int_distribution<int>(0, 1)(rng) == 0;
        int oldChild = leftSide ? bs.node[target].left : bs.node[target].right;
        if (leftSide) bs.node[target].left = u;
        else bs.node[target].right = u;
        bs.node[u].parent = target;
        if (oldChild >= 0) {
            if (uniform_int_distribution<int>(0, 1)(rng) == 0) bs.node[u].left = oldChild;
            else bs.node[u].right = oldChild;
            bs.node[oldChild].parent = u;
        }
        return bstarTreeLegal(bs);
    }

    static void bstarFlipChildren(BStarState& bs, int u) {
        if (u < 0 || u >= static_cast<int>(bs.node.size())) return;
        swap(bs.node[u].left, bs.node[u].right);
    }

    static bool bstarResizeSoft(const Design& design, BStarState& bs, mt19937& rng) {
        vector<int> cand;
        for (int u = 0; u < static_cast<int>(bs.node.size()); ++u) {
            int id = bs.node[u].block;
            if (id >= 0 && id < static_cast<int>(design.blockSpecs.size())) {
                const BlockSpec& sp = design.blockSpecs[id];
                if (sp.type == BlockType::SOFT && !sp.hasFixedSize) cand.push_back(id);
            }
        }
        if (cand.empty()) return false;
        int id;
        if (uniform_real_distribution<double>(0.0, 1.0)(rng) < 0.70) {
            int best = cand.front();
            double bestScore = -1.0;
            for (int c : cand) {
                double sc = endpointDemand(design, c) * (1.0 + softFtExpansionRate(design, c));
                sc *= 0.85 + 0.30 * uniform_real_distribution<double>(0.0, 1.0)(rng);
                if (sc > bestScore) { bestScore = sc; best = c; }
            }
            id = best;
        }
        else {
            id = cand[uniform_int_distribution<int>(0, static_cast<int>(cand.size()) - 1)(rng)];
        }
        const BlockSpec& sp = design.blockSpecs[id];
        const double amin = max(0.05, sp.aspectMin);
        const double amax = max(amin, sp.aspectMax);
        double cur = (id < static_cast<int>(bs.shape.ratio.size())) ? bs.shape.ratio[id] : smartSoftTargetRatio(design, id, 1.0);
        const double hot = clampD(endpointDemand(design, id) / max(1.0, maxEndpointDemand(design)), 0.0, 1.0);
        normal_distribution<double> nd(0.0, BSTAR_SOFT_LOCAL_LOG_SIGMA * (0.65 + 0.35 * sqrt(hot)));
        double r = exp(log(max(1e-9, cur)) + nd(rng));
        if (uniform_real_distribution<double>(0.0, 1.0)(rng) < 0.18 + 0.20 * hot) {
            r = (edgeSideDemandBias(design, id) >= 0.0) ? amax : amin;
        }
        bs.shape.ratio[id] = clampD(r, amin, amax);
        return true;
    }

    static void bstarResizeOutline(const Design& design, BStarState& bs, const LayoutResult& curLayout, mt19937& rng, bool secondPass = false) {
        const double dead = bstarDeadspace(design, curLayout);
        double shrinkBias = BSTAR_OUTLINE_SHRINK_BIAS - 0.020 * clampD(dead / 0.25, 0.0, 1.0);
        if (secondPass) shrinkBias -= 0.014;
        normal_distribution<double> shrinkND(shrinkBias, BSTAR_OUTLINE_LOG_SIGMA);
        normal_distribution<double> growND(0.020, BSTAR_OUTLINE_GROW_SIGMA);
        const double growProb = secondPass ? BSTAR_SECOND_GROW_PROB : 0.22;
        const bool grow = uniform_real_distribution<double>(0.0, 1.0)(rng) < growProb;
        double fx = exp(grow ? growND(rng) : shrinkND(rng));
        double fy = exp(grow ? growND(rng) : shrinkND(rng));
        // Sometimes change aspect instead of pure area to escape bad B*-tree shapes.
        if (uniform_real_distribution<double>(0.0, 1.0)(rng) < 0.45) {
            double a = exp(normal_distribution<double>(0.0, 0.055)(rng));
            fx *= a;
            fy /= a;
        }
        bs.W = clampD(bs.W * fx, minWidthBound(design, bs.shape), design.maxOutlineW);
        bs.H = clampD(bs.H * fy, minHeightBound(design, bs.shape), design.maxOutlineH);
    }

    static void bstarPerturb(const Design& design, BStarState& bs, const LayoutResult& curLayout, mt19937& rng, bool secondPass = false) {
        const int m = static_cast<int>(bs.node.size());
        if (m <= 0) return;
        const double routeBad = clampD(curLayout.routePenalty / max(1.0, totalPackingArea(design)), 0.0, 3.0);
        const double dead = bstarDeadspace(design, curLayout);
        double r = uniform_real_distribution<double>(0.0, 1.0)(rng);

        // Routing/congestion does not enter cost; it only biases what kind of
        // neighbor we sample.  Bad route proxy => more topology/soft moves.  High
        // deadspace => more outline shrinking.
        double pOutline = secondPass ?
            clampD(0.36 + 0.42 * dead, 0.32, 0.62) :
            clampD(0.22 + 0.45 * dead, 0.18, 0.58);
        double pShape = secondPass ?
            clampD(0.16 + 0.08 * routeBad, 0.14, 0.26) :
            clampD(0.14 + 0.10 * routeBad, 0.12, 0.28);
        double pMove = secondPass ?
            clampD(0.18 + 0.08 * routeBad, 0.16, 0.30) :
            clampD(0.22 + 0.10 * routeBad, 0.18, 0.35);
        double pSwap = secondPass ? 0.18 : 0.24;
        double pFlip = 1.0 - (pOutline + pShape + pMove + pSwap);
        if (pFlip < 0.08) { pFlip = 0.08; pOutline = max(0.10, pOutline - 0.05); }

        if (r < pOutline) {
            bstarResizeOutline(design, bs, curLayout, rng, secondPass);
        }
        else if (r < pOutline + pShape) {
            (void)bstarResizeSoft(design, bs, rng);
        }
        else if (r < pOutline + pShape + pMove) {
            BStarState old = bs;
            if (!bstarMoveLeaf(bs, rng)) bs = std::move(old);
        }
        else if (r < pOutline + pShape + pMove + pSwap) {
            int a = pickHotMovableNode(design, bs, rng);
            int b = uniform_int_distribution<int>(0, m - 1)(rng);
            if (a == b) b = (b + 1) % m;
            bstarSwapBlocks(bs, a, b);
        }
        else {
            int u = pickHotMovableNode(design, bs, rng);
            bstarFlipChildren(bs, u);
        }
    }

    static void bstarPerturbWire(const Design& design, BStarState& bs, const LayoutResult& curLayout, mt19937& rng, bool spreadMode) {
        const int m = static_cast<int>(bs.node.size());
        if (m <= 0) return;
        const double routeBad = clampD(curLayout.routePenalty / max(1.0, totalPackingArea(design)), 0.0, 3.0);
        const double dead = bstarDeadspace(design, curLayout);
        const double r = uniform_real_distribution<double>(0.0, 1.0)(rng);

        double pOutline = spreadMode ? clampD(0.12 + 0.18 * dead, 0.10, 0.26)
            : clampD(0.18 + 0.22 * dead, 0.14, 0.34);
        double pShape = clampD(0.20 + 0.08 * routeBad, 0.18, 0.30);
        double pMove = clampD(0.30 + 0.08 * routeBad, 0.26, 0.40);
        double pSwap = 0.24;
        double pFlip = 1.0 - (pOutline + pShape + pMove + pSwap);
        if (pFlip < 0.06) { pFlip = 0.06; pOutline = max(0.08, pOutline - 0.04); }

        if (r < pOutline) {
            bstarResizeOutline(design, bs, curLayout, rng, !spreadMode);
        }
        else if (r < pOutline + pShape) {
            (void)bstarResizeSoft(design, bs, rng);
        }
        else if (r < pOutline + pShape + pMove) {
            BStarState old = bs;
            if (!bstarMoveLeaf(bs, rng)) bs = std::move(old);
        }
        else if (r < pOutline + pShape + pMove + pSwap) {
            int a = pickHotMovableNode(design, bs, rng);
            int b = pickHotMovableNode(design, bs, rng);
            if (a == b) b = (b + 1) % m;
            bstarSwapBlocks(bs, a, b);
        }
        else {
            int u = pickHotMovableNode(design, bs, rng);
            bstarFlipChildren(bs, u);
        }
    }

    static double bstarCostForMode(const Design& design, const LayoutResult& r, bool checkerProxy) {
        return checkerProxy ? bstarCheckerProxyCost(design, r) : bstarAreaCost(r);
    }

    static double estimateBStarInitialTemp(const Design& design, const BStarState& init, const LayoutResult& initLayout, mt19937& rng, bool checkerProxy = false) {
        LayoutResult baseLayout = initLayout;
        if (checkerProxy) annotateStripProxy(design, baseLayout);
        double base = bstarCostForMode(design, baseLayout, checkerProxy);
        double sumUp = 0.0;
        int cntUp = 0;
        const int samples = max(30, min(240, BSTAR_SA_INNER_FACTOR * max(1, static_cast<int>(init.node.size()))));
        for (int i = 0; i < samples; ++i) {
            BStarState trial = init;
            bstarPerturb(design, trial, initLayout, rng, checkerProxy);
            LayoutResult cur;
            if (!packBStarState(design, trial, cur)) continue;
            if (checkerProxy) annotateStripProxy(design, cur);
            double d = bstarCostForMode(design, cur, checkerProxy) - base;
            if (d > 1.0) { sumUp += d; ++cntUp; }
        }
        double avg = (cntUp > 0) ? (sumUp / cntUp) : max(1.0, 0.015 * base);
        return max(1.0, avg / max(1e-9, log(1.0 / BSTAR_SA_INIT_ACCEPT_P)));
    }

    static double estimateBStarWireInitialTemp(const Design& design, const BStarState& init, const LayoutResult& initLayout, mt19937& rng, bool spreadMode) {
        LayoutResult baseLayout = initLayout;
        annotateStripProxy(design, baseLayout);
        const double base = bstarWireProxyCost(design, baseLayout, spreadMode);
        double sumUp = 0.0;
        int cntUp = 0;
        const int samples = max(24, min(160, BSTAR_SA_INNER_FACTOR * max(1, static_cast<int>(init.node.size()))));
        for (int i = 0; i < samples; ++i) {
            BStarState trial = init;
            bstarPerturbWire(design, trial, initLayout, rng, spreadMode);
            LayoutResult cur;
            if (!packBStarState(design, trial, cur)) continue;
            annotateStripProxy(design, cur);
            const double d = bstarWireProxyCost(design, cur, spreadMode) - base;
            if (d > 1.0) { sumUp += d; ++cntUp; }
        }
        const double avg = (cntUp > 0) ? (sumUp / cntUp) : max(1.0, 0.018 * base);
        return max(1.0, avg / max(1e-9, log(1.0 / BSTAR_SA_INIT_ACCEPT_P)));
    }


    static LayoutResult runBStarAreaSA(const Design& design, int& triedOut, int& packedOut, int& legalOut, vector<LayoutResult>* archiveOut = nullptr, BStarState* bestStateOut = nullptr) {
        triedOut = packedOut = legalOut = 0;
        LayoutResult empty;
        BStarState curState;
        LayoutResult curLayout;
        vector<LayoutResult> warmArchive;
        vector<LayoutResult> archive;
        if (!makeInitialBStarLegalState(
            design, curState, curLayout, &warmArchive)) {
            if (FAST_VERBOSE_LOG) cerr << "[BStarSA] init failed, fallback required\n";
            return empty;
        }
        ++packedOut;
        ++legalOut;

        BStarState bestState = curState;
        LayoutResult bestLayout = curLayout;
        addToBStarArchive(design, archive, curLayout);
        mt19937 rng(FAST_SEED ^ 0xB57A5A11u ^ static_cast<unsigned>(design.blockSpecs.size() * 131u));
        double T = estimateBStarInitialTemp(design, curState, curLayout, rng);
        const int m = static_cast<int>(curState.node.size());
        const int inner = max(BSTAR_SA_MIN_INNER, BSTAR_SA_INNER_FACTOR * max(1, m));
        const bool smallCase = smallOfficialLikeCase(design);
        int defaultMoveCap = min(BSTAR_SA_MAX_MOVES_CAP, max(1200, inner * max(18, min(80, m + 12))));
        if (!smallCase && design.blockSpecs.size() >= 45) {
            defaultMoveCap = min(defaultMoveCap, 4800);
        }
        const int moveCap = smallCase ? max(defaultMoveCap, 36000) : defaultMoveCap;
        int accepted = 0, uphill = 0, rejected = 0, bestUpdates = 0, outer = 0;
        double curCost = bstarAreaCost(curLayout);
        const double startArea = curLayout.area;

        const int outerLimit = smallCase ? max(BSTAR_SA_MAX_OUTER, 1200) : BSTAR_SA_MAX_OUTER;
        while (outer++ < outerLimit && triedOut < moveCap && T > BSTAR_SA_MIN_TEMP) {
            int roundAccepted = 0;
            int roundRejected = 0;
            int roundPacked = 0;
            for (int mt = 0; mt < inner && triedOut < moveCap; ++mt) {
                ++triedOut;
                BStarState trial = curState;
                bstarPerturb(design, trial, curLayout, rng);
                LayoutResult cand;
                if (!packBStarState(design, trial, cand)) {
                    ++roundRejected;
                    ++rejected;
                    continue;
                }
                ++packedOut;
                ++roundPacked;
                if (cand.legal && cand.strictEdgeLegal) ++legalOut;
                if (cand.legal && cand.strictEdgeLegal &&
                    (static_cast<int>(archive.size()) < BSTAR_ARCHIVE_LIMIT || (triedOut % BSTAR_ARCHIVE_SAMPLE_PERIOD) == 0)) {
                    addToBStarArchive(design, archive, cand);
                }
                double candCost = bstarAreaCost(cand);
                double d = candCost - curCost;
                bool accept = false;
                if (d <= 0.0) accept = true;
                else {
                    double prob = exp(-d / max(1e-12, T));
                    accept = uniform_real_distribution<double>(0.0, 1.0)(rng) < prob;
                }
                if (accept) {
                    curState = std::move(trial);
                    curLayout = std::move(cand);
                    curCost = candCost;
                    ++accepted;
                    ++roundAccepted;
                    if (d > 0.0) ++uphill;
                    if (curLayout.legal && curLayout.strictEdgeLegal && bstarAreaBetter(curLayout, bestLayout)) {
                        bestLayout = curLayout;
                        bestState = curState;
                        addToBStarArchive(design, archive, curLayout);
                        ++bestUpdates;
                    }
                }
                else {
                    ++rejected;
                    ++roundRejected;
                }
            }
            const double rejectRate = static_cast<double>(roundRejected) / max(1, roundRejected + roundAccepted);
            if (FAST_VERBOSE_LOG && (outer <= 6 || outer % 8 == 0 || bestUpdates > 0)) {
                cerr << fixed << setprecision(3)
                    << "[BStarSA/Round] outer=" << outer
                    << " T=" << T
                    << " packedDelta=" << roundPacked
                    << " acceptDelta=" << roundAccepted
                    << " rejectRate=" << rejectRate
                    << " curArea=" << curLayout.area
                    << " bestArea=" << bestLayout.area
                    << " bestW/H=" << bestLayout.W << "x" << bestLayout.H
                    << " bestRouteGap=" << bestLayout.routePenalty
                    << " bestViolPairs=" << bestLayout.routeViolPairs << "/" << bestLayout.routeConnectedPairs
                    << "\n";
                bestUpdates = 0;
            }
            if (roundPacked == 0 && T < 0.02 * startArea) break;
            if (!smallCase && rejectRate > 0.985 && outer > 12) break;
            T *= BSTAR_SA_COOL;
            if (smallCase && T <= BSTAR_SA_MIN_TEMP && triedOut < moveCap) {
                T = max(1.0, 0.0005 * startArea);
            }
        }

        if (!archive.empty()) {
            bestLayout = archive.front();
        }
        if (archiveOut) {
            *archiveOut = archive;
            archiveOut->insert(archiveOut->end(),
                warmArchive.begin(), warmArchive.end());
        }

        if (FAST_VERBOSE_LOG) {
            cerr << fixed << setprecision(3)
                << "[BStarSA/Selected] startArea=" << startArea
                << " bestArea=" << bestLayout.area
                << " W/H=" << bestLayout.W << "x" << bestLayout.H
                << " hpwl=" << bestLayout.hpwl
                << " routeGap=" << bestLayout.routePenalty
                << " gapPart=" << bestLayout.routeGapPenaltyPart
                << " portPart=" << bestLayout.routePortPenaltyPart
                << " maxGapMiss=" << bestLayout.routeMaxGapMiss
                << " maxPortMiss=" << bestLayout.routeMaxPortMiss
                << " violPairs=" << bestLayout.routeViolPairs << "/" << bestLayout.routeConnectedPairs
                << " tried=" << triedOut
                << " packed=" << packedOut
                << " legal=" << legalOut
                << " accepted=" << accepted
                << " uphill=" << uphill
                << " rejected=" << rejected
                << " archive=" << archive.size()
                << " warmArchive=" << warmArchive.size()
                << " stripPeak=" << bestLayout.stripPeakUtilProxy
                << " stripOvProxy=" << bestLayout.stripOverflowProxy
                << " stripHot=" << bestLayout.stripHotComponents
                << " cost=outlineAreaOnly"
                << "\n";
        }
        if (bestStateOut) *bestStateOut = bestState;
        (void)bestState;
        return bestLayout;
    }

    static vector<LayoutResult> runBStarSecondProxySA(
        const Design& design,
        const BStarState& seedState,
        int& triedOut,
        int& packedOut,
        int& legalOut
    ) {
        triedOut = packedOut = legalOut = 0;
        vector<LayoutResult> archive;
        if (!ENABLE_BSTAR_SECOND_PROXY_SA || seedState.node.empty()) return archive;

        BStarState curState = seedState;
        LayoutResult curLayout;
        if (!packBStarState(design, curState, curLayout)) return archive;
        annotateStripProxy(design, curLayout);
        ++packedOut;
        if (curLayout.legal && curLayout.strictEdgeLegal) ++legalOut;

        BStarState bestState = curState;
        LayoutResult bestLayout = curLayout;
        addToBStarSecondArchive(design, archive, curLayout);

        mt19937 rng(FAST_SEED ^ 0x5EC0A11u ^ static_cast<unsigned>(design.blockSpecs.size() * 977u));
        double T = estimateBStarInitialTemp(design, curState, curLayout, rng, true);
        const int m = static_cast<int>(curState.node.size());
        const int inner = max(BSTAR_SA_MIN_INNER, BSTAR_SA_INNER_FACTOR * max(1, m));
        int moveCap = min(BSTAR_SECOND_SA_MAX_MOVES_CAP, max(900, inner * max(12, min(36, m + 8))));
        if (smallOfficialLikeCase(design)) moveCap = max(moveCap, min(5200, BSTAR_SECOND_SA_MAX_MOVES_CAP + 1800));
        const int outerLimit = min(BSTAR_SA_MAX_OUTER, 90);

        int accepted = 0, uphill = 0, rejected = 0, bestUpdates = 0, outer = 0;
        double curCost = bstarCheckerProxyCost(design, curLayout);
        const double startCost = curCost;
        const double startArea = curLayout.area;

        while (outer++ < outerLimit && triedOut < moveCap && T > BSTAR_SA_MIN_TEMP) {
            int roundAccepted = 0;
            int roundRejected = 0;
            int roundPacked = 0;
            for (int mt = 0; mt < inner && triedOut < moveCap; ++mt) {
                ++triedOut;
                BStarState trial = curState;
                bstarPerturb(design, trial, curLayout, rng, true);
                LayoutResult cand;
                if (!packBStarState(design, trial, cand)) {
                    ++roundRejected;
                    ++rejected;
                    continue;
                }
                annotateStripProxy(design, cand);
                ++packedOut;
                ++roundPacked;
                if (cand.legal && cand.strictEdgeLegal) {
                    ++legalOut;
                    addToBStarSecondArchive(design, archive, cand);
                }

                const double candCost = bstarCheckerProxyCost(design, cand);
                const double d = candCost - curCost;
                bool accept = false;
                if (d <= 0.0) accept = true;
                else {
                    const double prob = exp(-d / max(1.0e-12, T));
                    accept = uniform_real_distribution<double>(0.0, 1.0)(rng) < prob;
                }

                if (accept) {
                    curState = std::move(trial);
                    curLayout = std::move(cand);
                    curCost = candCost;
                    ++accepted;
                    ++roundAccepted;
                    if (d > 0.0) ++uphill;
                    if (curLayout.legal && curLayout.strictEdgeLegal &&
                        bstarCheckerProxyBetter(design, curLayout, bestLayout)) {
                        bestLayout = curLayout;
                        bestState = curState;
                        addToBStarSecondArchive(design, archive, curLayout);
                        ++bestUpdates;
                    }
                }
                else {
                    ++rejected;
                    ++roundRejected;
                }
            }

            const double rejectRate = static_cast<double>(roundRejected) / max(1, roundRejected + roundAccepted);
            if (FAST_VERBOSE_LOG && (outer <= 4 || outer % 10 == 0 || bestUpdates > 0)) {
                cerr << fixed << setprecision(3)
                    << "[BStarSecondSA/Round] outer=" << outer
                    << " T=" << T
                    << " packedDelta=" << roundPacked
                    << " acceptDelta=" << roundAccepted
                    << " rejectRate=" << rejectRate
                    << " curCost=" << curCost
                    << " bestCost=" << bstarCheckerProxyCost(design, bestLayout)
                    << " bestArea=" << bestLayout.area
                    << " bestW/H=" << bestLayout.W << "x" << bestLayout.H
                    << " bestRouteGap=" << bestLayout.routePenalty
                    << " stripPeak=" << bestLayout.stripPeakUtilProxy
                    << "\n";
                bestUpdates = 0;
            }
            if (roundPacked == 0 && T < 0.02 * max(1.0, startCost)) break;
            if (rejectRate > 0.992 && outer > 10) break;
            T *= BSTAR_SA_COOL;
        }

        if (!archive.empty()) {
            sort(archive.begin(), archive.end(), [&](const LayoutResult& a, const LayoutResult& b) {
                return bstarCheckerProxyBetter(design, a, b);
                });
            if (static_cast<int>(archive.size()) > BSTAR_SECOND_ARCHIVE_LIMIT) {
                archive.resize(BSTAR_SECOND_ARCHIVE_LIMIT);
            }
        }

        if (FAST_VERBOSE_LOG) {
            cerr << fixed << setprecision(3)
                << "[BStarSecondSA/Selected]"
                << " startArea=" << startArea
                << " startCost=" << startCost
                << " bestArea=" << bestLayout.area
                << " bestCost=" << bstarCheckerProxyCost(design, bestLayout)
                << " W/H=" << bestLayout.W << "x" << bestLayout.H
                << " hpwl=" << bestLayout.hpwl
                << " routeGap=" << bestLayout.routePenalty
                << " stripPeak=" << bestLayout.stripPeakUtilProxy
                << " stripOvProxy=" << bestLayout.stripOverflowProxy
                << " tried=" << triedOut
                << " packed=" << packedOut
                << " legal=" << legalOut
                << " accepted=" << accepted
                << " uphill=" << uphill
                << " rejected=" << rejected
                << " archive=" << archive.size()
                << " cost=checkerProxy"
                << "\n";
        }

        (void)bestState;
        return archive;
    }

    static vector<LayoutResult> runBStarWireProxySA(
        const Design& design,
        const vector<BStarState>& seeds,
        bool spreadMode,
        int& triedOut,
        int& packedOut,
        int& legalOut
    ) {
        triedOut = packedOut = legalOut = 0;
        vector<LayoutResult> archive;
        if (!ENABLE_BSTAR_WIRE_SA || seeds.empty()) return archive;

        ScopedBStarPackMode packMode(spreadMode ? 2 : 1);
        const int seedCount = min(BSTAR_WIRE_SEED_LIMIT, static_cast<int>(seeds.size()));
        const int totalMoveCap = spreadMode ? BSTAR_SPREAD_SA_MAX_MOVES_CAP : BSTAR_WIRE_SA_MAX_MOVES_CAP;
        const int perSeedCap = max(360, totalMoveCap / max(1, seedCount));
        const int outerLimit = spreadMode ? 56 : 64;

        for (int si = 0; si < seedCount; ++si) {
            BStarState curState = seeds[si];
            LayoutResult curLayout;
            if (!packBStarState(design, curState, curLayout)) continue;
            annotateStripProxy(design, curLayout);
            ++packedOut;
            if (curLayout.legal && curLayout.strictEdgeLegal) ++legalOut;
            addToBStarWireArchive(design, archive, curLayout, spreadMode);

            LayoutResult bestLayout = curLayout;
            BStarState bestState = curState;
            mt19937 rng(FAST_SEED ^
                (spreadMode ? 0x5A9EADu : 0x51EEDu) ^
                static_cast<unsigned>(design.blockSpecs.size() * 4099u + si * 131u));
            double T = estimateBStarWireInitialTemp(design, curState, curLayout, rng, spreadMode);
            const int m = static_cast<int>(curState.node.size());
            const int inner = max(24, min(BSTAR_SA_MIN_INNER, 4 * max(1, m)));
            int accepted = 0, uphill = 0, rejected = 0, bestUpdates = 0, localTried = 0, outer = 0;
            double curCost = bstarWireProxyCost(design, curLayout, spreadMode);
            const double startCost = curCost;
            const double startArea = curLayout.area;

            while (outer++ < outerLimit && localTried < perSeedCap && T > BSTAR_SA_MIN_TEMP) {
                int roundAccepted = 0;
                int roundRejected = 0;
                int roundPacked = 0;
                for (int mt = 0; mt < inner && localTried < perSeedCap; ++mt) {
                    ++triedOut;
                    ++localTried;
                    BStarState trial = curState;
                    bstarPerturbWire(design, trial, curLayout, rng, spreadMode);
                    LayoutResult cand;
                    if (!packBStarState(design, trial, cand)) {
                        ++roundRejected;
                        ++rejected;
                        continue;
                    }
                    annotateStripProxy(design, cand);
                    ++packedOut;
                    ++roundPacked;
                    if (cand.legal && cand.strictEdgeLegal) {
                        ++legalOut;
                        addToBStarWireArchive(design, archive, cand, spreadMode);
                    }

                    const double candCost = bstarWireProxyCost(design, cand, spreadMode);
                    const double d = candCost - curCost;
                    bool accept = false;
                    if (d <= 0.0) accept = true;
                    else {
                        const double prob = exp(-d / max(1.0e-12, T));
                        accept = uniform_real_distribution<double>(0.0, 1.0)(rng) < prob;
                    }

                    if (accept) {
                        curState = std::move(trial);
                        curLayout = std::move(cand);
                        curCost = candCost;
                        ++accepted;
                        ++roundAccepted;
                        if (d > 0.0) ++uphill;
                        if (curLayout.legal && curLayout.strictEdgeLegal &&
                            bstarWireProxyBetter(design, curLayout, bestLayout, spreadMode)) {
                            bestLayout = curLayout;
                            bestState = curState;
                            addToBStarWireArchive(design, archive, curLayout, spreadMode);
                            ++bestUpdates;
                        }
                    }
                    else {
                        ++rejected;
                        ++roundRejected;
                    }
                }

                const double rejectRate = static_cast<double>(roundRejected) / max(1, roundRejected + roundAccepted);
                if (FAST_VERBOSE_LOG && si < 2 && (outer <= 3 || outer % 12 == 0 || bestUpdates > 0)) {
                    cerr << fixed << setprecision(3)
                        << (spreadMode ? "[BStarSpreadWireSA/Round]" : "[BStarWireSA/Round]")
                        << " seed=" << si
                        << " outer=" << outer
                        << " T=" << T
                        << " packedDelta=" << roundPacked
                        << " acceptDelta=" << roundAccepted
                        << " rejectRate=" << rejectRate
                        << " curCost=" << curCost
                        << " bestCost=" << bstarWireProxyCost(design, bestLayout, spreadMode)
                        << " bestArea=" << bestLayout.area
                        << " bestHpwl=" << bestLayout.hpwl
                        << " bestRouteGap=" << bestLayout.routePenalty
                        << "\n";
                    bestUpdates = 0;
                }
                if (roundPacked == 0 && T < 0.02 * max(1.0, startCost)) break;
                if (rejectRate > 0.993 && outer > 10) break;
                T *= BSTAR_SA_COOL;
            }

            if (FAST_VERBOSE_LOG) {
                cerr << fixed << setprecision(3)
                    << (spreadMode ? "[BStarSpreadWireSA/Seed]" : "[BStarWireSA/Seed]")
                    << " seed=" << si
                    << " startArea=" << startArea
                    << " startCost=" << startCost
                    << " bestArea=" << bestLayout.area
                    << " bestCost=" << bstarWireProxyCost(design, bestLayout, spreadMode)
                    << " hpwl=" << bestLayout.hpwl
                    << " routeGap=" << bestLayout.routePenalty
                    << " tried=" << localTried
                    << " accepted=" << accepted
                    << " uphill=" << uphill
                    << " rejected=" << rejected
                    << "\n";
            }
            (void)bestState;
        }

        sort(archive.begin(), archive.end(), [&](const LayoutResult& a, const LayoutResult& b) {
            return bstarWireProxyBetter(design, a, b, spreadMode);
            });
        const int limit = spreadMode ? BSTAR_SPREAD_ARCHIVE_LIMIT : BSTAR_WIRE_ARCHIVE_LIMIT;
        if (static_cast<int>(archive.size()) > limit) archive.resize(limit);
        if (FAST_VERBOSE_LOG) {
            cerr << fixed << setprecision(3)
                << (spreadMode ? "[BStarSpreadWireSA/Selected]" : "[BStarWireSA/Selected]")
                << " seeds=" << seedCount
                << " tried=" << triedOut
                << " packed=" << packedOut
                << " legal=" << legalOut
                << " archive=" << archive.size();
            if (!archive.empty()) {
                cerr << " bestArea=" << archive.front().area
                    << " bestCost=" << bstarWireProxyCost(design, archive.front(), spreadMode)
                    << " bestHpwl=" << archive.front().hpwl
                    << " bestRouteGap=" << archive.front().routePenalty;
            }
            cerr << "\n";
        }
        return archive;
    }

    static double compactColumnOrderScore(const Design& design, int id, const Rect& sh, int mode) {
        const double area = max(1.0, sh.w * sh.h);
        const double demand = endpointDemand(design, id);
        if (mode == 0) return sh.h;
        if (mode == 1) return -sh.h;
        if (mode == 2) return area;
        if (mode == 3) return -area;
        if (mode == 4) return demand;
        if (mode == 5) return -demand;
        return static_cast<double>(id);
    }

    static bool compactColumnPlaceOne(
        const Design& design,
        const vector<Rect>& shapes,
        int id,
        const vector<double>& xCands,
        double W,
        double H,
        vector<Rect>& rects,
        vector<Rect>& placed,
        vector<int>& placedIds
    ) {
        if (id < 0 || id >= static_cast<int>(shapes.size())) return false;
        const Rect& sh = shapes[id];
        if (sh.w > W + EPS || sh.h > H + EPS) return false;

        bool found = false;
        Rect best = sh;
        PlaceKey bestKey;
        vector<double> expandedX = xCands;
        for (int pi = 0; pi < static_cast<int>(placed.size()); ++pi) {
            const Rect& o = placed[pi];
            int oid = (pi < static_cast<int>(placedIds.size())) ? placedIds[pi] : -1;
            const double gx = (oid >= 0) ? requiredXGap(design, oid, id) : PACK_GAP;
            addCoord(expandedX, rectRight(o) + gx, 0.0, max(0.0, W - sh.w));
            addCoord(expandedX, o.x - sh.w - gx, 0.0, max(0.0, W - sh.w));
            addCoord(expandedX, o.x, 0.0, max(0.0, W - sh.w));
            addCoord(expandedX, rectRight(o) - sh.w, 0.0, max(0.0, W - sh.w));
        }
        sort(expandedX.begin(), expandedX.end());
        expandedX.erase(unique(expandedX.begin(), expandedX.end(), [](double a, double b) { return fabs(a - b) < 1.0e-5; }), expandedX.end());
        pruneCoords(expandedX, 18);

        for (double rawX : expandedX) {
            const double x = clampD(rawX, 0.0, max(0.0, W - sh.w));
            vector<double> ys;
            ys.reserve(2 * placed.size() + 4);
            addCoord(ys, 0.0, 0.0, max(0.0, H - sh.h));
            addCoord(ys, H - sh.h, 0.0, max(0.0, H - sh.h));
            for (int pi = 0; pi < static_cast<int>(placed.size()); ++pi) {
                const Rect& o = placed[pi];
                int oid = (pi < static_cast<int>(placedIds.size())) ? placedIds[pi] : -1;
                if (ovLen(x, x + sh.w, o.x, rectRight(o)) <= EPS) continue;
                const double gy = (oid >= 0) ? requiredYGap(design, oid, id) : PACK_GAP;
                addCoord(ys, rectTop(o) + gy, 0.0, max(0.0, H - sh.h));
                addCoord(ys, o.y - sh.h - gy, 0.0, max(0.0, H - sh.h));
            }
            sort(ys.begin(), ys.end());
            ys.erase(unique(ys.begin(), ys.end(), [](double a, double b) { return fabs(a - b) < 1.0e-5; }), ys.end());

            for (double y : ys) {
                Rect cand = sh;
                cand.x = x;
                cand.y = y;
                if (!inside(cand, W, H)) continue;
                if (anyOverlapWith(cand, placed)) continue;
                PlaceKey key;
                key.top = rectTop(cand);
                key.route = routeGapPenaltyForCandidate(design, placed, placedIds, id, cand);
                key.right = rectRight(cand);
                key.y = cand.y;
                vector<char> placedMask(rects.size(), 0);
                for (int pid : placedIds) if (pid >= 0 && pid < static_cast<int>(placedMask.size())) placedMask[pid] = 1;
                key.wire = partialWire(design, rects, placedMask, id, cand);
                key.center = fabs(rectCx(cand) - 0.5 * W) + fabs(rectCy(cand) - 0.5 * H);
                key.x = cand.x;
                fillPlacementPressure(key, placed, cand, W, H);
                if (!found || betterKey(key, bestKey)) {
                    found = true;
                    bestKey = key;
                    best = cand;
                }
            }
        }
        if (!found) return false;
        rects[id] = best;
        placed.push_back(best);
        placedIds.push_back(id);
        return true;
    }

    static bool tryCompactColumnLayout(
        const Design& design,
        const ShapeState& st,
        const vector<int>& movables,
        const vector<int>& assignment,
        int colCount,
        int sortMode,
        int alignMode,
        double gap,
        double wFactor,
        double hFactor,
        LayoutResult& out
    ) {
        if (colCount <= 0 || assignment.size() != movables.size()) return false;
        vector<Rect> shapes = makeShapes(design, st);
        vector<vector<int>> cols(colCount);
        for (int k = 0; k < static_cast<int>(movables.size()); ++k) {
            int col = assignment[k];
            if (col < 0 || col >= colCount) return false;
            cols[col].push_back(movables[k]);
        }
        for (const auto& col : cols) if (col.empty()) return false;

        for (auto& col : cols) {
            stable_sort(col.begin(), col.end(), [&](int a, int b) {
                const double sa = compactColumnOrderScore(design, a, shapes[a], sortMode);
                const double sb = compactColumnOrderScore(design, b, shapes[b], sortMode);
                if (fabs(sa - sb) > 1.0e-6) return sa < sb;
                return a < b;
                });
        }

        vector<double> colW(colCount, 0.0), colH(colCount, 0.0);
        for (int c = 0; c < colCount; ++c) {
            for (int id : cols[c]) {
                colW[c] = max(colW[c], shapes[id].w);
                if (colH[c] > EPS) colH[c] += gap;
                colH[c] += shapes[id].h;
            }
        }

        double W = gap * max(0, colCount - 1);
        for (double w : colW) W += w;
        W = max(W * wFactor, minWidthBound(design, st));
        if (W > design.maxOutlineW + EPS) return false;

        double H = minHeightBound(design, st);
        for (double h : colH) H = max(H, h);
        H = max(H, totalPackingArea(design) / max(1.0, W));
        H *= hFactor;
        H = clampD(H, minHeightBound(design, st), design.maxOutlineH);
        if (H > design.maxOutlineH + EPS) return false;

        vector<Rect> rects, placed;
        if (!placeEdgesFast(design, shapes, W, H, rects, placed, true)) return false;
        vector<int> placedIds;
        placedIds.reserve(placed.size() + movables.size());
        vector<int> edgeIds;
        for (int i = 0; i < static_cast<int>(design.blockSpecs.size()); ++i) {
            if (design.blockSpecs[i].type == BlockType::EDGE) edgeIds.push_back(i);
        }
        sort(edgeIds.begin(), edgeIds.end(), [&](int a, int b) {
            double aa = shapes[a].w * shapes[a].h;
            double bb = shapes[b].w * shapes[b].h;
            if (fabs(aa - bb) > 1.0e-6) return aa > bb;
            return a < b;
            });
        for (int id : edgeIds) placedIds.push_back(id);

        vector<double> colX(colCount, 0.0);
        double x = 0.0;
        for (int c = 0; c < colCount; ++c) {
            colX[c] = x;
            x += colW[c] + gap;
        }

        for (int c = 0; c < colCount; ++c) {
            for (int id : cols[c]) {
                vector<double> xCands;
                const double slack = max(0.0, colW[c] - shapes[id].w);
                if (alignMode == 0 || alignMode == 3) xCands.push_back(colX[c]);
                if (alignMode == 1 || alignMode == 3) xCands.push_back(colX[c] + 0.5 * slack);
                if (alignMode == 2 || alignMode == 3) xCands.push_back(colX[c] + slack);
                if (xCands.empty()) xCands.push_back(colX[c]);
                if (!compactColumnPlaceOne(design, shapes, id, xCands, W, H, rects, placed, placedIds)) return false;
            }
        }

        out = scoreLayout(design, W, H, std::move(rects));
        return out.legal && out.strictEdgeLegal && out.overlap <= EPS && out.outlineViol <= EPS;
    }

    static int strongestUnplacedNeighbor(
        const Design& design,
        int id,
        const vector<char>& used,
        int avoid = -1
    ) {
        int best = -1;
        int bestConn = 0;
        double bestDemand = -1.0;
        for (int j = 0; j < static_cast<int>(design.blockSpecs.size()); ++j) {
            if (j == id || j == avoid || used[j]) continue;
            if (!isMovableBlock(design.blockSpecs[j])) continue;
            const int nets = totalConnBetween(design, id, j);
            if (nets <= 0) continue;
            const double dem = endpointDemand(design, j);
            if (nets > bestConn || (nets == bestConn && dem > bestDemand)) {
                best = j;
                bestConn = nets;
                bestDemand = dem;
            }
        }
        return best;
    }

    static void pushFanoutColumn(vector<vector<int>>& cols, vector<char>& used, int col, int id) {
        if (id < 0 || id >= static_cast<int>(used.size()) || used[id]) return;
        col = max(0, min(col, static_cast<int>(cols.size()) - 1));
        cols[col].push_back(id);
        used[id] = 1;
    }

    static vector<int> hubNeighborsByStrength(const Design& design, int hub) {
        vector<pair<int, int>> items;
        for (int j = 0; j < static_cast<int>(design.blockSpecs.size()); ++j) {
            if (j == hub || !isMovableBlock(design.blockSpecs[j])) continue;
            const int nets = totalConnBetween(design, hub, j);
            if (nets > 0) items.push_back({ -nets, j });
        }
        sort(items.begin(), items.end());
        vector<int> out;
        for (const auto& it : items) out.push_back(it.second);
        return out;
    }

    static int mainEdgeHub(const Design& design) {
        int best = -1;
        int bestEdgeConn = 0;
        double bestDemand = -1.0;
        for (int i = 0; i < static_cast<int>(design.blockSpecs.size()); ++i) {
            if (!isMovableBlock(design.blockSpecs[i])) continue;
            int edgeConn = 0;
            for (int e = 0; e < static_cast<int>(design.blockSpecs.size()); ++e) {
                if (design.blockSpecs[e].type == BlockType::EDGE) edgeConn += totalConnBetween(design, i, e);
            }
            const double dem = endpointDemand(design, i);
            if (edgeConn > bestEdgeConn || (edgeConn == bestEdgeConn && dem > bestDemand)) {
                best = i;
                bestEdgeConn = edgeConn;
                bestDemand = dem;
            }
        }
        if (best >= 0 && bestEdgeConn > 0) return best;

        for (int i = 0; i < static_cast<int>(design.blockSpecs.size()); ++i) {
            if (!isMovableBlock(design.blockSpecs[i])) continue;
            const double dem = endpointDemand(design, i);
            if (dem > bestDemand) {
                best = i;
                bestDemand = dem;
            }
        }
        return best;
    }

    static vector<vector<int>> makeGraphFanoutColumns(const Design& design) {
        const int n = static_cast<int>(design.blockSpecs.size());
        vector<vector<int>> cols(4);
        vector<char> used(n, 0);
        const int hub = mainEdgeHub(design);
        if (hub < 0) return {};

        vector<int> hNbr = hubNeighborsByStrength(design, hub);
        int mainBranch = hNbr.empty() ? -1 : hNbr[0];
        int upperBranch = (hNbr.size() > 1) ? hNbr[1] : -1;
        int midBranch = (hNbr.size() > 2) ? hNbr[2] : -1;
        int lowerBranch = (hNbr.size() > 3) ? hNbr[3] : -1;

        if (mainBranch >= 0) used[mainBranch] = 1; // reserved until its children are chosen.
        int mainTop = (mainBranch >= 0) ? strongestUnplacedNeighbor(design, mainBranch, used, hub) : -1;
        if (mainTop >= 0) used[mainTop] = 1;
        int mainTop2 = (mainTop >= 0) ? strongestUnplacedNeighbor(design, mainTop, used, mainBranch) : -1;
        if (mainTop2 >= 0) used[mainTop2] = 1;
        int mainBottom = (mainBranch >= 0) ? strongestUnplacedNeighbor(design, mainBranch, used, hub) : -1;
        if (mainBottom >= 0) used[mainBottom] = 1;
        int mainTopSide = (mainTop >= 0) ? strongestUnplacedNeighbor(design, mainTop, used, mainBranch) : -1;
        if (mainTopSide >= 0) used[mainTopSide] = 1;
        int mainTop2Child = (mainTop2 >= 0) ? strongestUnplacedNeighbor(design, mainTop2, used, mainTop) : -1;
        if (mainTop2Child >= 0) used[mainTop2Child] = 1;

        used.assign(n, 0);
        pushFanoutColumn(cols, used, 0, mainTopSide);
        pushFanoutColumn(cols, used, 0, mainTop);

        pushFanoutColumn(cols, used, 1, mainBottom);
        pushFanoutColumn(cols, used, 1, mainBranch);
        pushFanoutColumn(cols, used, 1, hub);
        pushFanoutColumn(cols, used, 1, mainTop2);

        pushFanoutColumn(cols, used, 2, lowerBranch);
        pushFanoutColumn(cols, used, 2, mainTop2Child);
        pushFanoutColumn(cols, used, 2, midBranch);
        pushFanoutColumn(cols, used, 2, upperBranch);

        int upperChild = (upperBranch >= 0) ? strongestUnplacedNeighbor(design, upperBranch, used, hub) : -1;
        pushFanoutColumn(cols, used, 2, upperChild);
        int midChild = (midBranch >= 0) ? strongestUnplacedNeighbor(design, midBranch, used, hub) : -1;
        pushFanoutColumn(cols, used, 3, midChild);

        vector<int> leftovers;
        for (int i = 0; i < n; ++i) {
            if (!isMovableBlock(design.blockSpecs[i]) || used[i]) continue;
            leftovers.push_back(i);
        }
        sort(leftovers.begin(), leftovers.end(), [&](int a, int b) {
            const double da = endpointDemand(design, a);
            const double db = endpointDemand(design, b);
            if (fabs(da - db) > 1.0e-6) return da > db;
            return a < b;
            });
        vector<double> colDemand(cols.size(), 0.0);
        for (int c = 0; c < static_cast<int>(cols.size()); ++c) {
            for (int id : cols[c]) colDemand[c] += endpointDemand(design, id);
        }
        for (int id : leftovers) {
            int bestCol = 0;
            double bestScore = INF;
            for (int c = 0; c < static_cast<int>(cols.size()); ++c) {
                double score = colDemand[c];
                if (c == 3) score *= 0.82; // keep weak leaf nodes off the central spine.
                if (score < bestScore) {
                    bestScore = score;
                    bestCol = c;
                }
            }
            pushFanoutColumn(cols, used, bestCol, id);
            colDemand[bestCol] += endpointDemand(design, id);
        }

        for (const auto& col : cols) if (col.empty()) return {};
        return cols;
    }

    static bool tryGraphFanoutColumnLayout(
        const Design& design,
        const ShapeState& st,
        const vector<vector<int>>& cols,
        double W,
        double H,
        double anchorScale,
        LayoutResult& out
    ) {
        if (cols.empty() || W > design.maxOutlineW + EPS || H > design.maxOutlineH + EPS) return false;
        vector<Rect> shapes = makeShapes(design, st);
        if (W < minWidthBound(design, st) - EPS || H < minHeightBound(design, st) - EPS) return false;

        vector<Rect> rects, placed;
        if (!placeEdgesFast(design, shapes, W, H, rects, placed, true)) return false;

        vector<int> placedIds;
        placedIds.reserve(placed.size() + design.blockSpecs.size());
        vector<int> edgeIds;
        for (int i = 0; i < static_cast<int>(design.blockSpecs.size()); ++i) {
            if (design.blockSpecs[i].type == BlockType::EDGE) edgeIds.push_back(i);
        }
        sort(edgeIds.begin(), edgeIds.end(), [&](int a, int b) {
            double aa = shapes[a].w * shapes[a].h;
            double bb = shapes[b].w * shapes[b].h;
            if (fabs(aa - bb) > 1.0e-6) return aa > bb;
            return a < b;
            });
        for (int id : edgeIds) placedIds.push_back(id);

        double leftEdgeRight = 0.0;
        for (int id : edgeIds) {
            if (rects[id].x <= 0.18 * W) leftEdgeRight = max(leftEdgeRight, rectRight(rects[id]));
        }
        const double a1 = clampD(leftEdgeRight + PACK_GAP, 0.0, W);
        vector<double> anchors = {
            0.0,
            a1,
            clampD(anchorScale * 0.46 * W, 0.0, W),
            clampD(anchorScale * 0.72 * W, 0.0, W)
        };
        while (anchors.size() < cols.size()) anchors.push_back(clampD(0.86 * W, 0.0, W));

        for (int c = 0; c < static_cast<int>(cols.size()); ++c) {
            for (int id : cols[c]) {
                vector<double> xCands;
                const double a = anchors[min(c, static_cast<int>(anchors.size()) - 1)];
                xCands.push_back(a);
                xCands.push_back(max(0.0, a - 36.0));
                xCands.push_back(min(W, a + 36.0));
                if (c == 0) xCands.push_back(0.0);
                if (c == 1) xCands.push_back(a1);
                if (c >= 2) xCands.push_back(clampD(a - 0.08 * W, 0.0, W));
                if (!compactColumnPlaceOne(design, shapes, id, xCands, W, H, rects, placed, placedIds)) return false;
            }
        }

        out = scoreLayout(design, W, H, std::move(rects));
        return out.legal && out.strictEdgeLegal && out.overlap <= EPS && out.outlineViol <= EPS;
    }

    static vector<LayoutResult> makeGraphFanoutColumnArchive(const Design& design) {
        vector<LayoutResult> archive;
        const int n = static_cast<int>(design.blockSpecs.size());
        int edgeCount = 0;
        for (const auto& sp : design.blockSpecs) if (sp.type == BlockType::EDGE) ++edgeCount;
        if (edgeCount <= 0 || n < 10 || n > 18) return archive;

        vector<vector<int>> cols = makeGraphFanoutColumns(design);
        if (cols.empty()) return archive;

        vector<ShapeState> states;
        ShapeState wide;
        wide.ratio.assign(n, 1.0);
        ShapeState smart;
        smart.ratio.assign(n, 1.0);
        ShapeState mixed;
        mixed.ratio.assign(n, 1.0);
        for (int i = 0; i < n; ++i) {
            const BlockSpec& sp = design.blockSpecs[i];
            const double amin = max(0.05, sp.aspectMin);
            const double amax = max(amin, sp.aspectMax);
            const double sm = smartSoftTargetRatio(design, i, 1.0);
            wide.ratio[i] = (sp.type == BlockType::SOFT && !sp.hasFixedSize) ? amax : aspectMid(sp);
            smart.ratio[i] = sm;
            mixed.ratio[i] = (sp.type == BlockType::SOFT && !sp.hasFixedSize)
                ? clampD(exp(0.42 * log(max(1.0e-9, sm)) + 0.58 * log(amax)), amin, amax)
                : aspectMid(sp);
        }
        states.push_back(wide);
        states.push_back(mixed);
        states.push_back(smart);

        const double area = totalPackingArea(design);
        vector<double> widths;
        const double baseW = area / max(1.0, design.maxOutlineH);
        const double factors[] = { 1.18, 1.25, 1.31, 1.38, 1.46 };
        for (double f : factors) widths.push_back(clampD(baseW * f, min(1.0, design.maxOutlineW), design.maxOutlineW));
        widths.push_back(0.84 * design.maxOutlineW);
        widths.push_back(0.88 * design.maxOutlineW);
        widths.push_back(0.92 * design.maxOutlineW);
        sort(widths.begin(), widths.end());
        widths.erase(unique(widths.begin(), widths.end(), [](double a, double b) { return fabs(a - b) < 4.0; }), widths.end());

        const double anchorScales[] = { 0.94, 1.00, 1.06 };
        for (const ShapeState& st : states) {
            for (double W : widths) {
                W = clampD(W, minWidthBound(design, st), design.maxOutlineW);
                for (double as : anchorScales) {
                    LayoutResult cur;
                    if (!tryGraphFanoutColumnLayout(design, st, cols, W, design.maxOutlineH, as, cur)) continue;
                    addToBStarArchive(design, archive, cur);
                }
            }
        }

        sort(archive.begin(), archive.end(), betterArchiveLayout);
        const int keep = min(10, static_cast<int>(archive.size()));
        if (static_cast<int>(archive.size()) > keep) archive.resize(keep);
        if (FAST_VERBOSE_LOG && !archive.empty()) {
            cerr << fixed << setprecision(3)
                << "[GraphFanoutColumnArchive] candidates=" << archive.size()
                << " bestArea=" << archive.front().area
                << " bestW/H=" << archive.front().W << "x" << archive.front().H
                << " bestHpwl=" << archive.front().hpwl
                << " bestStripPeak=" << archive.front().stripPeakUtilProxy
                << "\n";
        }
        return archive;
    }

    static vector<vector<int>> compactColumnAssignments(
        const Design& design,
        const vector<int>& movables,
        const vector<Rect>& shapes,
        int colCount,
        int limit
    ) {
        vector<vector<int>> out;
        const int m = static_cast<int>(movables.size());
        if (m <= 0 || colCount <= 0) return out;
        auto pushUnique = [&](const vector<int>& a) {
            for (const auto& old : out) if (old == a) return;
            bool used[5] = { false, false, false, false, false };
            for (int v : a) if (v >= 0 && v < 5) used[v] = true;
            for (int c = 0; c < colCount; ++c) if (!used[c]) return;
            out.push_back(a);
            };

        auto greedyAssign = [&](vector<int> order, int biasMode) {
            vector<int> a(m, 0);
            vector<double> h(colCount, 0.0);
            for (int id : order) {
                int pos = -1;
                for (int k = 0; k < m; ++k) if (movables[k] == id) { pos = k; break; }
                if (pos < 0) continue;
                int best = 0;
                double bestScore = INF;
                for (int c = 0; c < colCount; ++c) {
                    double score = h[c];
                    if (biasMode == 1) score += 0.02 * c * max(1.0, shapes[id].h);
                    if (biasMode == 2) score += 0.02 * (colCount - 1 - c) * max(1.0, shapes[id].h);
                    if (score < bestScore) { bestScore = score; best = c; }
                }
                a[pos] = best;
                h[best] += shapes[id].h;
            }
            pushUnique(a);
            };

        vector<int> byHeight = movables;
        sort(byHeight.begin(), byHeight.end(), [&](int a, int b) { return shapes[a].h > shapes[b].h; });
        vector<int> byArea = movables;
        sort(byArea.begin(), byArea.end(), [&](int a, int b) { return shapes[a].w * shapes[a].h > shapes[b].w * shapes[b].h; });
        vector<int> byDemand = movables;
        sort(byDemand.begin(), byDemand.end(), [&](int a, int b) { return endpointDemand(design, a) > endpointDemand(design, b); });
        for (int bias = 0; bias < 3; ++bias) {
            greedyAssign(byHeight, bias);
            greedyAssign(byArea, bias);
            greedyAssign(byDemand, bias);
        }
        for (int shift = 0; shift < colCount; ++shift) {
            vector<int> a(m, 0);
            for (int k = 0; k < m; ++k) a[k] = (k + shift) % colCount;
            pushUnique(a);
        }

        if (m <= 8) {
            long long total = 1;
            for (int k = 0; k < m; ++k) total *= colCount;
            for (long long mask = 0; mask < total && static_cast<int>(out.size()) < limit; ++mask) {
                long long t = mask;
                vector<int> a(m, 0);
                for (int k = 0; k < m; ++k) {
                    a[k] = static_cast<int>(t % colCount);
                    t /= colCount;
                }
                pushUnique(a);
            }
        }
        else {
            mt19937 rng(FAST_SEED + 7919u * static_cast<unsigned>(m) + 104729u * static_cast<unsigned>(colCount));
            while (static_cast<int>(out.size()) < limit) {
                vector<int> a(m, 0);
                for (int k = 0; k < m; ++k) a[k] = uniform_int_distribution<int>(0, colCount - 1)(rng);
                pushUnique(a);
                if (static_cast<int>(out.size()) >= limit) break;
                if (out.size() > 12 && uniform_int_distribution<int>(0, 16)(rng) == 0) break;
            }
        }

        if (static_cast<int>(out.size()) > limit) out.resize(limit);
        return out;
    }

    static vector<LayoutResult> makeCompactColumnArchive(const Design& design) {
        vector<LayoutResult> archive;
        const int n = static_cast<int>(design.blockSpecs.size());
        if (n > 10) return archive;

        vector<ShapeState> states = makeShapeStates(design);
        if (n <= 10) {
            vector<int> softIds;
            for (int i = 0; i < n; ++i) {
                const BlockSpec& sp = design.blockSpecs[i];
                if (sp.type == BlockType::SOFT && !sp.hasFixedSize) softIds.push_back(i);
            }
            if (!softIds.empty() && softIds.size() <= 8) {
                vector<ShapeState> extraStates;
                const int masks = 1 << static_cast<int>(softIds.size());
                for (int mask = 0; mask < masks; ++mask) {
                    ShapeState st;
                    st.ratio.assign(n, 1.0);
                    for (int i = 0; i < n; ++i) st.ratio[i] = smartSoftTargetRatio(design, i, 1.0);
                    for (int k = 0; k < static_cast<int>(softIds.size()); ++k) {
                        int id = softIds[k];
                        const BlockSpec& sp = design.blockSpecs[id];
                        st.ratio[id] = (mask & (1 << k)) ? max(0.05, sp.aspectMax) : max(0.05, sp.aspectMin);
                    }
                    bool dup = false;
                    for (const auto& oldSt : states) {
                        if (oldSt.ratio.size() != st.ratio.size()) continue;
                        bool same = true;
                        for (int i = 0; i < n; ++i) {
                            if (fabs(log(max(1.0e-9, oldSt.ratio[i])) - log(max(1.0e-9, st.ratio[i]))) > 1.0e-3) {
                                same = false;
                                break;
                            }
                        }
                        if (same) { dup = true; break; }
                    }
                    if (!dup) extraStates.push_back(std::move(st));
                }
                vector<ShapeState> mergedStates;
                mergedStates.reserve(extraStates.size() + states.size());
                for (auto& st : extraStates) mergedStates.push_back(std::move(st));
                for (auto& st : states) mergedStates.push_back(std::move(st));
                states.swap(mergedStates);
            }
        }
        const int stateLimit = min(static_cast<int>(states.size()), n <= 10 ? 10 : 5);
        for (int si = 0; si < stateLimit; ++si) {
            const ShapeState& st = states[si];
            vector<Rect> shapes = makeShapes(design, st);
            vector<int> movables;
            for (int i = 0; i < n; ++i) if (isMovableBlock(design.blockSpecs[i])) movables.push_back(i);
            if (movables.empty()) continue;

            const int maxCols = min(4, max(2, static_cast<int>(movables.size())));
            for (int colCount = 2; colCount <= maxCols; ++colCount) {
                const int assignLimit = (n <= 10) ? 2400 : 120;
                vector<vector<int>> assigns = compactColumnAssignments(design, movables, shapes, colCount, assignLimit);
                const double gaps[] = { PACK_GAP, 24.0 };
                const double wFactors[] = { 1.070, 1.120 };
                const double hFactors[] = { 1.000, 1.012 };
                for (const vector<int>& asg : assigns) {
                    for (int sortMode = 0; sortMode < 2; ++sortMode) {
                        for (int alignMode = 3; alignMode < 4; ++alignMode) {
                            for (double gap : gaps) {
                                for (double wf : wFactors) {
                                    for (double hf : hFactors) {
                                        LayoutResult cur;
                                        if (!tryCompactColumnLayout(design, st, movables, asg, colCount, sortMode, alignMode, gap, wf, hf, cur)) continue;
                                        addToBStarArchive(design, archive, cur);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        sort(archive.begin(), archive.end(), betterArchiveLayout);
        const int keep = min(12, static_cast<int>(archive.size()));
        if (static_cast<int>(archive.size()) > keep) archive.resize(keep);
        if (FAST_VERBOSE_LOG && !archive.empty()) {
            cerr << fixed << setprecision(3)
                << "[CompactColumnArchive] candidates=" << archive.size()
                << " bestArea=" << archive.front().area
                << " bestW/H=" << archive.front().W << "x" << archive.front().H
                << " bestHpwl=" << archive.front().hpwl
                << " bestStripPeak=" << archive.front().stripPeakUtilProxy
                << "\n";
        }
        return archive;
    }

    static LayoutResult fallbackShelf(const Design& design) {
        int n = static_cast<int>(design.blockSpecs.size());
        ShapeState st;
        st.ratio.assign(n, 1.0);
        for (int i = 0; i < n; ++i) st.ratio[i] = aspectMid(design.blockSpecs[i]);
        vector<Rect> shapes = makeShapes(design, st);
        vector<Rect> rects, placed;
        placeEdgesFast(design, shapes, design.maxOutlineW, design.maxOutlineH, rects, placed, false);

        vector<int> order = degreeOrder(design);
        double x = 0.0, y = 0.0, rowH = 0.0;
        for (int id : order) {
            if (!isMovableBlock(design.blockSpecs[id])) continue;
            Rect r = shapes[id];
            if (x + r.w > design.maxOutlineW && x > EPS) {
                x = 0.0;
                y += rowH + PACK_GAP;
                rowH = 0.0;
            }
            r.x = clampD(x, 0.0, max(0.0, design.maxOutlineW - r.w));
            r.y = clampD(y, 0.0, max(0.0, design.maxOutlineH - r.h));
            // If this row conflicts with an EDGE obstacle, slide right/down in a
            // bounded way.  It is fallback only, so keep it simple.
            for (int trial = 0; trial < 80 && anyOverlapWith(r, placed); ++trial) {
                x += 10.0;
                if (x + r.w > design.maxOutlineW) { x = 0.0; y += rowH + 10.0; rowH = 0.0; }
                r.x = clampD(x, 0.0, max(0.0, design.maxOutlineW - r.w));
                r.y = clampD(y, 0.0, max(0.0, design.maxOutlineH - r.h));
            }
            rects[id] = r;
            placed.push_back(r);
            x += r.w + PACK_GAP;
            rowH = max(rowH, r.h);
        }
        return scoreLayout(design, design.maxOutlineW, design.maxOutlineH, std::move(rects));
    }


    static bool layoutPlacementLegal(const LayoutResult& r) {
        return r.legal && r.overlap <= EPS && r.outlineViol <= EPS;
    }

    static bool repackEdgesForLayout(
        const Design& design,
        double W,
        double H,
        bool strictEdge,
        vector<Rect>& rects
    ) {
        vector<Rect> repacked;
        vector<Rect> obstacles;
        if (!placeEdgesFast(design, rects, W, H, repacked, obstacles, strictEdge)) return false;
        rects.swap(repacked);
        return true;
    }

    static void clampMovablesAndRepackEdges(
        const Design& design,
        double W,
        double H,
        bool strictEdge,
        vector<Rect>& rects
    ) {
        for (int i = 0; i < static_cast<int>(rects.size()); ++i) {
            if (!isMovableBlock(design.blockSpecs[i])) continue;
            rects[i].x = clampD(rects[i].x, 0.0, max(0.0, W - rects[i].w));
            rects[i].y = clampD(rects[i].y, 0.0, max(0.0, H - rects[i].h));
        }
        (void)repackEdgesForLayout(design, W, H, strictEdge, rects);
    }

    static void spreadRepairLayout(
        const Design& design,
        double W,
        double H,
        bool strictEdge,
        vector<Rect>& rects,
        int maxPasses
    ) {
        const int n = static_cast<int>(rects.size());
        clampMovablesAndRepackEdges(design, W, H, strictEdge, rects);
        for (int pass = 0; pass < maxPasses; ++pass) {
            bool changed = false;
            for (int i = 0; i < n; ++i) {
                for (int j = i + 1; j < n; ++j) {
                    double ox = ovLen(rects[i].x, rectRight(rects[i]), rects[j].x, rectRight(rects[j]));
                    double oy = ovLen(rects[i].y, rectTop(rects[i]), rects[j].y, rectTop(rects[j]));
                    if (ox <= EPS || oy <= EPS) continue;

                    bool mi = isMovableBlock(design.blockSpecs[i]);
                    bool mj = isMovableBlock(design.blockSpecs[j]);
                    if (!mi && !mj) continue;

                    changed = true;
                    bool pushX = ox <= oy;
                    double px = ox + 1.0;
                    double py = oy + 1.0;
                    if (pushX) {
                        double dir = (rectCx(rects[i]) <= rectCx(rects[j])) ? -1.0 : 1.0;
                        if (mi && mj) {
                            rects[i].x += 0.5 * dir * px;
                            rects[j].x -= 0.5 * dir * px;
                        }
                        else if (mi) {
                            rects[i].x += dir * px;
                        }
                        else if (mj) {
                            rects[j].x -= dir * px;
                        }
                    }
                    else {
                        double dir = (rectCy(rects[i]) <= rectCy(rects[j])) ? -1.0 : 1.0;
                        if (mi && mj) {
                            rects[i].y += 0.5 * dir * py;
                            rects[j].y -= 0.5 * dir * py;
                        }
                        else if (mi) {
                            rects[i].y += dir * py;
                        }
                        else if (mj) {
                            rects[j].y -= dir * py;
                        }
                    }
                    clampMovablesAndRepackEdges(design, W, H, strictEdge, rects);
                }
            }
            if (!changed) break;
        }
    }

    struct AlignmentGuide {
        bool xAxis = true;
        double coordinate = 0.0;
        double rank = 0.0;
        int support = 0;
    };

    struct AlignmentQuality {
        double score = 0.0;
        int sharedLines = 0;
        int alignedEdges = 0;
        int alignedBlocks = 0;
    };

    struct AlignmentProposal {
        int id = -1;
        bool xAxis = true;
        bool highEdge = false;
        bool reshapeSoft = false;
        double coordinate = 0.0;
        double priority = 0.0;
    };

    struct AlignmentRefineConfig {
        double snapFraction = 0.0;
        int maxGuidesPerAxis = 0;
        int passes = 1;
        bool reshapeSoft = false;
    };

    static double alignmentBlockWeight(
        const Design& design,
        int id,
        double maxDemand
    ) {
        if (id < 0 || id >= static_cast<int>(design.blockSpecs.size())) return 1.0;
        const double demand = endpointDemand(design, id);
        double weight = 1.0 + 0.75 * sqrt(clampD(demand / max(1.0, maxDemand), 0.0, 1.0));
        if (design.blockSpecs[id].type == BlockType::EDGE) weight += 0.40;
        return weight;
    }

    static double alignmentEdgeCoordinate(
        const Rect& rect,
        bool xAxis,
        bool highEdge
    ) {
        if (xAxis) return highEdge ? rectRight(rect) : rect.x;
        return highEdge ? rectTop(rect) : rect.y;
    }

    static vector<AlignmentGuide> collectAlignmentGuides(
        const Design& design,
        const LayoutResult& base,
        bool xAxis,
        double radius,
        int guideLimit
    ) {
        struct EdgeSample {
            double coordinate = 0.0;
            int id = -1;
        };

        vector<EdgeSample> samples;
        samples.reserve(base.rects.size() * 2);
        for (int id = 0; id < static_cast<int>(base.rects.size()); ++id) {
            samples.push_back({ alignmentEdgeCoordinate(base.rects[id], xAxis, false), id });
            samples.push_back({ alignmentEdgeCoordinate(base.rects[id], xAxis, true), id });
        }

        const double extent = xAxis ? base.W : base.H;
        const double maxDemand = maxEndpointDemand(design);
        vector<AlignmentGuide> candidates;
        candidates.reserve(samples.size());
        for (const EdgeSample& sample : samples) {
            const double coordinate = round(sample.coordinate);
            if (coordinate <= 0.5 || coordinate >= extent - 0.5) continue;

            vector<char> nearSeen(base.rects.size(), 0);
            vector<char> exactSeen(base.rects.size(), 0);
            int support = 0;
            int movableSupport = 0;
            int fixedSupport = 0;
            int exactSupport = 0;
            double weightedSupport = 0.0;
            double spanLo = INF;
            double spanHi = -INF;
            for (int id = 0; id < static_cast<int>(base.rects.size()); ++id) {
                const Rect& rect = base.rects[id];
                const double low = alignmentEdgeCoordinate(rect, xAxis, false);
                const double high = alignmentEdgeCoordinate(rect, xAxis, true);
                const double distance = min(fabs(low - coordinate), fabs(high - coordinate));
                if (distance <= radius + EPS && !nearSeen[id]) {
                    nearSeen[id] = 1;
                    ++support;
                    weightedSupport += alignmentBlockWeight(design, id, maxDemand);
                    if (isMovableBlock(design.blockSpecs[id])) ++movableSupport;
                    else ++fixedSupport;
                    spanLo = min(spanLo, xAxis ? rect.y : rect.x);
                    spanHi = max(spanHi, xAxis ? rectTop(rect) : rectRight(rect));
                }
                if (distance <= ALIGNMENT_EXACT_EPS && !exactSeen[id]) {
                    exactSeen[id] = 1;
                    ++exactSupport;
                }
            }

            if (movableSupport < 1 || (movableSupport < 2 && fixedSupport < 1)) continue;
            if (support < 2) continue;

            const double span = max(0.0, spanHi - spanLo);
            const double exactBonus = 4.0 * max(0, exactSupport - 1);
            const double fixedBonus = 0.80 * fixedSupport;
            const double continuityBonus =
                0.90 * static_cast<double>(support) * span / max(1.0, xAxis ? base.H : base.W);
            const double rank =
                exactBonus + weightedSupport + 0.60 * static_cast<double>(support - 1) +
                fixedBonus + continuityBonus;
            candidates.push_back({ xAxis, coordinate, rank, support });
        }

        sort(candidates.begin(), candidates.end(), [](const AlignmentGuide& a, const AlignmentGuide& b) {
            if (fabs(a.rank - b.rank) > 1.0e-9) return a.rank > b.rank;
            if (a.support != b.support) return a.support > b.support;
            return a.coordinate < b.coordinate;
            });

        vector<AlignmentGuide> guides;
        const double minSeparation = max(2.0, min(8.0, 0.08 * radius));
        for (const AlignmentGuide& candidate : candidates) {
            bool duplicate = false;
            for (const AlignmentGuide& old : guides) {
                if (fabs(old.coordinate - candidate.coordinate) < minSeparation) {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) continue;
            guides.push_back(candidate);
            if (static_cast<int>(guides.size()) >= guideLimit) break;
        }
        return guides;
    }

    static AlignmentQuality measureAlignment(
        const Design& design,
        const vector<Rect>& rects,
        const vector<AlignmentGuide>& guides
    ) {
        AlignmentQuality quality;
        vector<char> alignedBlock(rects.size(), 0);
        const double maxDemand = maxEndpointDemand(design);
        for (const AlignmentGuide& guide : guides) {
            int support = 0;
            int movableSupport = 0;
            double weightedSupport = 0.0;
            for (int id = 0; id < static_cast<int>(rects.size()); ++id) {
                const double low = alignmentEdgeCoordinate(rects[id], guide.xAxis, false);
                const double high = alignmentEdgeCoordinate(rects[id], guide.xAxis, true);
                if (min(fabs(low - guide.coordinate), fabs(high - guide.coordinate)) >
                    ALIGNMENT_EXACT_EPS) continue;
                ++support;
                weightedSupport += alignmentBlockWeight(design, id, maxDemand);
                if (isMovableBlock(design.blockSpecs[id])) {
                    ++movableSupport;
                    alignedBlock[id] = 1;
                }
            }
            if (support < 2 || movableSupport < 1) continue;

            const double concentration =
                static_cast<double>(support - 1) * max(1.0, weightedSupport - 1.0);
            quality.score += concentration;
            ++quality.sharedLines;
            quality.alignedEdges += support;
        }
        quality.score -= 0.12 * static_cast<double>(quality.sharedLines);
        for (char aligned : alignedBlock) {
            if (aligned) ++quality.alignedBlocks;
        }
        return quality;
    }

    static bool alignmentMoveSafe(
        const LayoutResult& candidate,
        const LayoutResult& base
    ) {
        if (!candidate.legal || candidate.overlap > EPS || candidate.outlineViol > EPS) return false;
        if (base.strictEdgeLegal && !candidate.strictEdgeLegal) return false;
        if (candidate.routeViolPairs > base.routeViolPairs) return false;
        if (candidate.hpwl >
            base.hpwl * ALIGNMENT_HPWL_GUARD_RATIO + 1500.0) return false;
        if (candidate.routePenalty >
            base.routePenalty * ALIGNMENT_ROUTE_GUARD_RATIO + 2500.0) return false;
        if (candidate.routeGapPenaltyPart >
            base.routeGapPenaltyPart * ALIGNMENT_ROUTE_GUARD_RATIO + 2500.0) return false;
        if (candidate.routePortPenaltyPart >
            base.routePortPenaltyPart * ALIGNMENT_ROUTE_GUARD_RATIO + 1500.0) return false;
        if (candidate.routeMaxGapMiss > base.routeMaxGapMiss + 1.0) return false;
        if (candidate.routeMaxPortMiss > base.routeMaxPortMiss + 1.0) return false;
        return true;
    }

    static vector<vector<Rect>> makeAlignmentRectVariants(
        const Design& design,
        const vector<Rect>& current,
        const AlignmentProposal& proposal
    ) {
        vector<vector<Rect>> variants;
        if (proposal.id < 0 || proposal.id >= static_cast<int>(current.size())) return variants;

        const int id = proposal.id;
        const Rect& old = current[id];
        if (!proposal.reshapeSoft) {
            Rect moved = old;
            if (proposal.xAxis) {
                moved.x = proposal.highEdge ?
                    proposal.coordinate - moved.w : proposal.coordinate;
            }
            else {
                moved.y = proposal.highEdge ?
                    proposal.coordinate - moved.h : proposal.coordinate;
            }
            vector<Rect> candidate = current;
            candidate[id] = moved;
            variants.push_back(std::move(candidate));
            return variants;
        }

        const BlockSpec& spec = design.blockSpecs[id];
        if (spec.type != BlockType::SOFT || spec.hasFixedSize) return variants;

        const double reserve = softFtSideReserve(design, id);
        const double area = blockNominalArea(spec);
        double targetDimension = 0.0;
        double ratio = 0.0;
        if (proposal.xAxis) {
            targetDimension = proposal.highEdge ?
                proposal.coordinate - old.x : rectRight(old) - proposal.coordinate;
            const double coreWidth = targetDimension - reserve;
            if (coreWidth <= EPS) return variants;
            ratio = coreWidth * coreWidth / max(1.0, area);
        }
        else {
            targetDimension = proposal.highEdge ?
                proposal.coordinate - old.y : rectTop(old) - proposal.coordinate;
            const double coreHeight = targetDimension - reserve;
            if (coreHeight <= EPS) return variants;
            ratio = area / max(1.0, coreHeight * coreHeight);
        }

        const double amin = max(0.05, spec.aspectMin);
        const double amax = max(amin, spec.aspectMax);
        if (ratio < amin - 1.0e-8 || ratio > amax + 1.0e-8) return variants;

        Rect shaped = makePackingShape(design, id, ratio);
        const double oldDimension = proposal.xAxis ? old.w : old.h;
        const double newDimension = proposal.xAxis ? shaped.w : shaped.h;
        if (newDimension < 0.82 * oldDimension || newDimension > 1.18 * oldDimension) return variants;

        if (proposal.xAxis) {
            shaped.x = proposal.highEdge ?
                proposal.coordinate - shaped.w : proposal.coordinate;
            const array<double, 3> orthogonal = {
                old.y,
                rectCy(old) - 0.5 * shaped.h,
                rectTop(old) - shaped.h
            };
            for (double y : orthogonal) {
                Rect placed = shaped;
                placed.y = y;
                vector<Rect> candidate = current;
                candidate[id] = placed;
                variants.push_back(std::move(candidate));
            }
        }
        else {
            shaped.y = proposal.highEdge ?
                proposal.coordinate - shaped.h : proposal.coordinate;
            const array<double, 3> orthogonal = {
                old.x,
                rectCx(old) - 0.5 * shaped.w,
                rectRight(old) - shaped.w
            };
            for (double x : orthogonal) {
                Rect placed = shaped;
                placed.x = x;
                vector<Rect> candidate = current;
                candidate[id] = placed;
                variants.push_back(std::move(candidate));
            }
        }
        return variants;
    }

    static bool refineLayoutAlignment(
        const Design& design,
        const LayoutResult& base,
        const AlignmentRefineConfig& config,
        LayoutResult& out,
        AlignmentQuality& beforeQuality,
        AlignmentQuality& afterQuality,
        int& acceptedMoves,
        int& guideCount
    ) {
        acceptedMoves = 0;
        guideCount = 0;
        if (!ENABLE_ALIGNMENT_ARCHIVE_REFINEMENT ||
            !base.legal || !base.strictEdgeLegal ||
            base.rects.size() != design.blockSpecs.size()) return false;

        const double xRadius = clampD(config.snapFraction * base.W, 3.0, 180.0);
        const double yRadius = clampD(config.snapFraction * base.H, 3.0, 180.0);
        vector<AlignmentGuide> guides =
            collectAlignmentGuides(design, base, true, xRadius, config.maxGuidesPerAxis);
        vector<AlignmentGuide> yGuides =
            collectAlignmentGuides(design, base, false, yRadius, config.maxGuidesPerAxis);
        guides.insert(guides.end(), yGuides.begin(), yGuides.end());
        guideCount = static_cast<int>(guides.size());
        if (guides.empty()) return false;

        LayoutResult current = base;
        beforeQuality = measureAlignment(design, current.rects, guides);
        AlignmentQuality currentQuality = beforeQuality;
        const double maxDemand = maxEndpointDemand(design);

        for (int pass = 0; pass < config.passes; ++pass) {
            vector<AlignmentProposal> proposals;
            proposals.reserve(current.rects.size() * guides.size() * 4);
            for (int id = 0; id < static_cast<int>(current.rects.size()); ++id) {
                if (!isMovableBlock(design.blockSpecs[id])) continue;
                const double blockWeight = alignmentBlockWeight(design, id, maxDemand);
                for (const AlignmentGuide& guide : guides) {
                    const double maxSnap = guide.xAxis ? xRadius : yRadius;
                    for (int high = 0; high <= 1; ++high) {
                        const double edge = alignmentEdgeCoordinate(
                            current.rects[id], guide.xAxis, high != 0);
                        const double distance = fabs(edge - guide.coordinate);
                        if (distance <= ALIGNMENT_EXACT_EPS || distance > maxSnap + EPS) continue;
                        const double closeness = 1.0 - distance / max(1.0, maxSnap);
                        const double priority =
                            guide.rank * blockWeight * (0.55 + 0.45 * closeness);
                        proposals.push_back({
                            id, guide.xAxis, high != 0, false,
                            guide.coordinate, priority
                            });
                        if (config.reshapeSoft &&
                            design.blockSpecs[id].type == BlockType::SOFT &&
                            !design.blockSpecs[id].hasFixedSize) {
                            proposals.push_back({
                                id, guide.xAxis, high != 0, true,
                                guide.coordinate, 0.96 * priority
                                });
                        }
                    }
                }
            }

            sort(proposals.begin(), proposals.end(), [](const AlignmentProposal& a, const AlignmentProposal& b) {
                if (fabs(a.priority - b.priority) > 1.0e-9) return a.priority > b.priority;
                if (a.reshapeSoft != b.reshapeSoft) return !a.reshapeSoft;
                if (a.id != b.id) return a.id < b.id;
                if (a.xAxis != b.xAxis) return a.xAxis;
                if (a.highEdge != b.highEdge) return !a.highEdge;
                return a.coordinate < b.coordinate;
                });

            vector<char> movedAxis(current.rects.size() * 2, 0);
            bool improvedThisPass = false;
            for (const AlignmentProposal& proposal : proposals) {
                if (acceptedMoves >= ALIGNMENT_MAX_ACCEPTED_MOVES) break;
                const int axisIndex = 2 * proposal.id + (proposal.xAxis ? 0 : 1);
                if (movedAxis[axisIndex]) continue;

                const double edge = alignmentEdgeCoordinate(
                    current.rects[proposal.id], proposal.xAxis, proposal.highEdge);
                const double maxSnap = proposal.xAxis ? xRadius : yRadius;
                const double distance = fabs(edge - proposal.coordinate);
                if (distance <= ALIGNMENT_EXACT_EPS || distance > maxSnap + EPS) continue;

                LayoutResult bestMove;
                AlignmentQuality bestMoveQuality = currentQuality;
                bool haveMove = false;
                vector<vector<Rect>> variants =
                    makeAlignmentRectVariants(design, current.rects, proposal);
                for (vector<Rect>& rects : variants) {
                    LayoutResult candidate =
                        scoreLayout(design, base.W, base.H, std::move(rects));
                    if (!alignmentMoveSafe(candidate, base)) continue;
                    AlignmentQuality quality =
                        measureAlignment(design, candidate.rects, guides);
                    const bool alignmentBetter =
                        quality.score > bestMoveQuality.score + 0.20 ||
                        (fabs(quality.score - bestMoveQuality.score) <= 0.20 &&
                            quality.alignedEdges > bestMoveQuality.alignedEdges);
                    if (!alignmentBetter) continue;
                    if (!haveMove ||
                        quality.score > bestMoveQuality.score + 1.0e-9 ||
                        (fabs(quality.score - bestMoveQuality.score) <= 1.0e-9 &&
                            candidate.score < bestMove.score)) {
                        bestMove = std::move(candidate);
                        bestMoveQuality = quality;
                        haveMove = true;
                    }
                }
                if (!haveMove) continue;

                current = std::move(bestMove);
                currentQuality = bestMoveQuality;
                movedAxis[axisIndex] = 1;
                improvedThisPass = true;
                ++acceptedMoves;
            }
            if (!improvedThisPass) break;
        }

        afterQuality = currentQuality;
        if (afterQuality.score <= beforeQuality.score + 0.50 ||
            afterQuality.alignedEdges <= beforeQuality.alignedEdges) return false;

        annotateStripProxy(design, current);
        const RouteProxy baseProxy = estimateRouteProxy(design, base);
        const RouteProxy refinedProxy = estimateRouteProxy(design, current);
        const bool noOpenRegression =
            refinedProxy.disconnectedPairs <= baseProxy.disconnectedPairs;
        const bool directionalSafe =
            refinedProxy.maxDirectionalUtil <=
            max(baseProxy.maxDirectionalUtil * ALIGNMENT_PROXY_GUARD_RATIO + 0.03,
                baseProxy.maxDirectionalUtil + 0.15);
        const bool channelSafe =
            refinedProxy.channelRisk <=
            max(baseProxy.channelRisk * ALIGNMENT_PROXY_GUARD_RATIO + 2500.0,
                baseProxy.channelRisk + 75000.0);
        const bool ftSafe =
            refinedProxy.ftRisk <=
            max(baseProxy.ftRisk * ALIGNMENT_PROXY_GUARD_RATIO + 1000.0,
                baseProxy.ftRisk + 30000.0);
        const bool stripSafe =
            current.stripPeakUtilProxy <=
            max(base.stripPeakUtilProxy * 1.08 + 0.04,
                base.stripPeakUtilProxy + 0.12) &&
            current.stripOverflowProxy <=
            max(base.stripOverflowProxy * 1.10 + 5000.0,
                base.stripOverflowProxy + 1000000.0);
        if (!noOpenRegression || !directionalSafe || !channelSafe || !ftSafe || !stripSafe) {
            return false;
        }

        out = std::move(current);
        return true;
    }

    static vector<LayoutResult> makeAlignmentArchive(
        const Design& design,
        const vector<LayoutResult>& seeds
    ) {
        struct RankedAlignment {
            LayoutResult layout;
            AlignmentQuality before;
            AlignmentQuality after;
            double rank = INF;
            int seed = -1;
            int variant = -1;
            int acceptedMoves = 0;
            int guideCount = 0;
        };

        const array<AlignmentRefineConfig, 3> configs = {
            AlignmentRefineConfig{ 0.0045, 4, 1, false },
            AlignmentRefineConfig{ 0.0100, 6, 2, true },
            AlignmentRefineConfig{ 0.0220, ALIGNMENT_MAX_GUIDES_PER_AXIS, 2, true }
        };

        vector<RankedAlignment> ranked;
        const int seedLimit = min(ALIGNMENT_SEED_LIMIT, static_cast<int>(seeds.size()));
        for (int seedIndex = 0; seedIndex < seedLimit; ++seedIndex) {
            const LayoutResult& base = seeds[seedIndex];
            for (int variant = 0; variant < static_cast<int>(configs.size()); ++variant) {
                LayoutResult refined;
                AlignmentQuality before;
                AlignmentQuality after;
                int acceptedMoves = 0;
                int guideCount = 0;
                const bool accepted = refineLayoutAlignment(
                    design, base, configs[variant], refined,
                    before, after, acceptedMoves, guideCount);

                cerr << fixed << setprecision(3)
                    << "[Alignment] seed=" << seedIndex
                    << " variant=" << variant
                    << " snapFrac=" << configs[variant].snapFraction
                    << " guides=" << guideCount
                    << " moves=" << acceptedMoves
                    << " lines=" << before.sharedLines << "->" << after.sharedLines
                    << " edges=" << before.alignedEdges << "->" << after.alignedEdges
                    << " blocks=" << before.alignedBlocks << "->" << after.alignedBlocks
                    << " alignScore=" << before.score << "->" << after.score
                    << " accept=" << (accepted ? "Y" : "N");
                if (accepted) {
                    cerr << " hpwl=" << base.hpwl << "->" << refined.hpwl
                        << " routeGap=" << base.routePenalty << "->" << refined.routePenalty
                        << " stripPeak=" << base.stripPeakUtilProxy << "->" << refined.stripPeakUtilProxy
                        << " stripOv=" << base.stripOverflowProxy << "->" << refined.stripOverflowProxy;
                }
                cerr << "\n";

                if (!accepted) continue;
                const RouteProxy baseProxy = estimateRouteProxy(design, base);
                const RouteProxy refinedProxy = estimateRouteProxy(design, refined);
                const double alignmentGain = after.score - before.score;
                const double hpwlRatio = refined.hpwl / max(1.0, base.hpwl);
                const double routeRatio =
                    (refined.routePenalty + 1000.0) / (base.routePenalty + 1000.0);
                const double proxyRatio =
                    (refinedProxy.channelRisk + 0.35 * refinedProxy.ftRisk + 1000.0) /
                    (baseProxy.channelRisk + 0.35 * baseProxy.ftRisk + 1000.0);
                const double rank =
                    -alignmentGain + 0.20 * hpwlRatio +
                    0.35 * routeRatio + 0.45 * proxyRatio;
                ranked.push_back({
                    std::move(refined), before, after, rank,
                    seedIndex, variant, acceptedMoves, guideCount
                    });
            }
        }

        sort(ranked.begin(), ranked.end(), [](const RankedAlignment& a, const RankedAlignment& b) {
            if (fabs(a.rank - b.rank) > 1.0e-9) return a.rank < b.rank;
            if (fabs(a.after.score - b.after.score) > 1.0e-9)
                return a.after.score > b.after.score;
            return a.layout.score < b.layout.score;
            });

        vector<LayoutResult> out;
        vector<AlignmentQuality> keptQuality;
        for (RankedAlignment& candidate : ranked) {
            bool duplicate = false;
            for (int i = 0; i < static_cast<int>(out.size()); ++i) {
                if (fabs(candidate.after.score - keptQuality[i].score) <= 0.25 &&
                    fabs(candidate.layout.hpwl - out[i].hpwl) <=
                    max(1.0, 0.0005 * min(candidate.layout.hpwl, out[i].hpwl)) &&
                    fabs(candidate.layout.routePenalty - out[i].routePenalty) <=
                    max(1.0, 0.005 * min(candidate.layout.routePenalty, out[i].routePenalty))) {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) continue;
            keptQuality.push_back(candidate.after);
            out.push_back(std::move(candidate.layout));
            if (static_cast<int>(out.size()) >= ALIGNMENT_KEEP_LIMIT) break;
        }
        return out;
    }

    static LayoutResult applyDSUScaleCandidate(
        const Design& design,
        const LayoutResult& base,
        double newW,
        double newH,
        bool compactX,
        bool compactY
    ) {
        vector<Rect> rects = base.rects;
        const double fx = compactX ? (newW / max(1.0, base.W)) : 1.0;
        const double fy = compactY ? (newH / max(1.0, base.H)) : 1.0;

        for (int i = 0; i < static_cast<int>(rects.size()); ++i) {
            if (!isMovableBlock(design.blockSpecs[i])) continue;
            double cx = rectCx(rects[i]);
            double cy = rectCy(rects[i]);
            if (compactX) rects[i].x = cx * fx - 0.5 * rects[i].w;
            if (compactY) rects[i].y = cy * fy - 0.5 * rects[i].h;
        }
        clampMovablesAndRepackEdges(design, newW, newH, base.strictEdgeLegal, rects);
        spreadRepairLayout(design, newW, newH, base.strictEdgeLegal, rects, 22);
        return scoreLayout(design, newW, newH, std::move(rects));
    }

    static bool dsuAcceptable(
        const Design& design,
        const LayoutResult& cand,
        const LayoutResult& anchor,
        double minAllowedArea
    ) {
        if (!layoutPlacementLegal(cand)) return false;
        if (cand.strictEdgeLegal != anchor.strictEdgeLegal && anchor.strictEdgeLegal) return false;
        if (cand.area + 1.0 < minAllowedArea) return false;
        if (cand.area >= anchor.area - 1.0) return false;

        // DSU is a post-process, not a new cost.  These guards are intentionally
        // stricter than the phase-1 selector: DSU should compact whitespace only when
        // it does not damage route-aware gaps / port windows.
        const double areaGain = (anchor.area - cand.area) / max(1.0, anchor.area);
        const double routeWorse = (cand.routePenalty - anchor.routePenalty) / max(1.0, anchor.routePenalty);

        if (cand.hpwl > anchor.hpwl * DSU_HPWL_GUARD_RATIO + 8000.0) return false;

        if (anchor.routePenalty <= 1.0) {
            if (cand.routePenalty > 5000.0) return false;
        }
        else if (cand.routePenalty > anchor.routePenalty * DSU_ROUTE_GUARD_RATIO + 5000.0) {
            return false;
        }

        // A tiny area win is not worth opening new route-gap violations.  This catches
        // the log case where area improved only ~0.2%, but routeGap/portPart worsened.
        if (routeWorse > 0.03 && areaGain < DSU_MIN_AREA_GAIN_IF_ROUTE_WORSE) return false;
        if (areaGain < DSU_TINY_AREA_GAIN && cand.routePenalty > anchor.routePenalty + 1000.0) return false;

        // Guard the two sub-parts separately.  routePenalty is often dominated by
        // gapPart, so port-window damage can be hidden if only the total is checked.
        if (cand.routeGapPenaltyPart > anchor.routeGapPenaltyPart * DSU_GAP_GUARD_RATIO + 8000.0) return false;
        if (anchor.routePortPenaltyPart <= 1000.0) {
            if (cand.routePortPenaltyPart > max(3500.0, anchor.routePortPenaltyPart + 2500.0)) return false;
        }
        else if (cand.routePortPenaltyPart > anchor.routePortPenaltyPart * DSU_PORT_GUARD_RATIO + 5000.0) {
            return false;
        }

        if (cand.routeMaxPortMiss > anchor.routeMaxPortMiss + DSU_MAX_PORT_MISS_INCREASE) return false;
        if (areaGain < 0.020 && cand.routeMaxGapMiss > anchor.routeMaxGapMiss + DSU_MAX_GAP_MISS_INCREASE) return false;
        if (cand.routeViolPairs > anchor.routeViolPairs + DSU_MAX_VIOLPAIR_INCREASE) return false;

        return true;
    }

    static LayoutResult dsuPostCompaction(const Design& design, const LayoutResult& start) {
        if (!ENABLE_DSU_POST_COMPACTION || !layoutPlacementLegal(start)) return start;

        const double blockArea = totalPackingArea(design);
        const double deadspace = clampD((start.area - blockArea) / max(1.0, start.area), 0.0, 0.95);
        double cap = 0.0;
        if (deadspace > DSU_DEADSPACE_START) cap = 0.50 * (deadspace - DSU_DEADSPACE_START);
        cap = clampD(cap, 0.0, DSU_MAX_AREA_SHRINK);
        if (cap <= 0.002) return start;

        const double minAllowedArea = start.area * (1.0 - cap);
        LayoutResult best = start;

        vector<double> steps = { 0.030, 0.022, 0.016, 0.011, 0.007, 0.004, 0.002 };
        int acceptCount = 0;
        for (double step : steps) {
            bool improved = true;
            int pass = 0;
            while (improved && pass++ < DSU_MAX_PASSES) {
                improved = false;
                for (int mode = 0; mode < 3; ++mode) {
                    bool cx = (mode == 0 || mode == 2);
                    bool cy = (mode == 1 || mode == 2);
                    double newW = cx ? best.W * (1.0 - step) : best.W;
                    double newH = cy ? best.H * (1.0 - step) : best.H;
                    newW = clampD(newW, 1.0, design.maxOutlineW);
                    newH = clampD(newH, 1.0, design.maxOutlineH);
                    if (newW * newH < minAllowedArea - EPS) continue;

                    LayoutResult cand = applyDSUScaleCandidate(design, best, newW, newH, cx, cy);
                    if (!dsuAcceptable(design, cand, start, minAllowedArea)) continue;
                    if (cand.area < best.area - 1.0 || betterLayout(cand, best)) {
                        best = std::move(cand);
                        improved = true;
                        ++acceptCount;
                    }
                }
            }
        }

        cerr << fixed << setprecision(3)
            << "[DSUPost] deadspace=" << deadspace
            << " cap=" << cap
            << " accept=" << acceptCount
            << " area=" << start.area << "->" << best.area
            << " W/H=" << start.W << "x" << start.H << "->" << best.W << "x" << best.H
            << " hpwl=" << start.hpwl << "->" << best.hpwl
            << " routeGap=" << start.routePenalty << "->" << best.routePenalty
            << " gapPart=" << start.routeGapPenaltyPart << "->" << best.routeGapPenaltyPart
            << " portPart=" << start.routePortPenaltyPart << "->" << best.routePortPenaltyPart
            << " maxGapMiss=" << start.routeMaxGapMiss << "->" << best.routeMaxGapMiss
            << " maxPortMiss=" << start.routeMaxPortMiss << "->" << best.routeMaxPortMiss
            << " violPairs=" << start.routeViolPairs << "->" << best.routeViolPairs
            << "\n";
        return best;
    }

    static vector<LayoutResult> makeCompactAfterSpreadArchive(const Design& design, const vector<LayoutResult>& spreadArchive) {
        vector<LayoutResult> out;
        if (spreadArchive.empty()) return out;
        const int seedLimit = min(4, static_cast<int>(spreadArchive.size()));
        const double scales[] = { 0.970, 0.945, 0.920, 0.895 };
        int tried = 0;
        for (int si = 0; si < seedLimit; ++si) {
            const LayoutResult& base = spreadArchive[si];
            if (!layoutPlacementLegal(base)) continue;
            for (double scale : scales) {
                for (int mode = 0; mode < 3; ++mode) {
                    const bool cx = (mode == 0 || mode == 2);
                    const bool cy = (mode == 1 || mode == 2);
                    double newW = cx ? base.W * scale : base.W;
                    double newH = cy ? base.H * scale : base.H;
                    newW = clampD(newW, 1.0, design.maxOutlineW);
                    newH = clampD(newH, 1.0, design.maxOutlineH);
                    if (newW * newH >= base.area - 1.0) continue;
                    ++tried;
                    LayoutResult cand = applyDSUScaleCandidate(design, base, newW, newH, cx, cy);
                    if (!layoutPlacementLegal(cand)) continue;
                    if (cand.area >= base.area - 1.0) continue;
                    if (cand.hpwl > base.hpwl * 1.10 + 12000.0) continue;
                    if (cand.routeViolPairs > base.routeViolPairs + 1) continue;
                    if (cand.routeMaxGapMiss > base.routeMaxGapMiss + 8.0) continue;
                    addToBStarWireArchive(design, out, cand, false);
                    if (static_cast<int>(out.size()) >= BSTAR_COMPACT_SPREAD_KEEP) break;
                }
                if (static_cast<int>(out.size()) >= BSTAR_COMPACT_SPREAD_KEEP) break;
            }
            if (static_cast<int>(out.size()) >= BSTAR_COMPACT_SPREAD_KEEP) break;
        }
        if (FAST_VERBOSE_LOG) {
            cerr << fixed << setprecision(3)
                << "[CompactAfterSpread] seeds=" << seedLimit
                << " tried=" << tried
                << " keep=" << out.size();
            if (!out.empty()) {
                cerr << " bestArea=" << out.front().area
                    << " bestHpwl=" << out.front().hpwl
                    << " bestRouteGap=" << out.front().routePenalty;
            }
            cerr << "\n";
        }
        return out;
    }

    static void anchorRequiredCornerEdges(Design& design) {
        for (BlockInst& block : design.blocks) {
            if (block.spec.type != BlockType::EDGE) continue;
            bool hasT = false, hasB = false, hasL = false, hasR = false;
            for (const EdgeRule& rule : edgeRules(block.spec)) {
                if (!rule.valid) continue;
                hasT = hasT || rule.side == 'T';
                hasB = hasB || rule.side == 'B';
                hasL = hasL || rule.side == 'L';
                hasR = hasR || rule.side == 'R';
            }
            if (!(hasT || hasB) || !(hasL || hasR)) continue;
            if (hasL && !hasR) block.rect.x = 0.0;
            else if (hasR && !hasL)
                block.rect.x = max(0.0, design.outlineW - block.rect.w);
            if (hasB && !hasT) block.rect.y = 0.0;
            else if (hasT && !hasB)
                block.rect.y = max(0.0, design.outlineH - block.rect.h);
        }
    }

    static bool integerizeCommittedLayout(Design& design) {
        try {
            phase1_warm_start::Problem problem =
                phase1_warm_start::makeProblem(design);
            phase1_warm_start::Placement placement;
            placement.W = placement.decodedW = design.outlineW;
            placement.H = placement.decodedH = design.outlineH;
            placement.rects.reserve(design.blocks.size());
            for (const BlockInst& block : design.blocks)
                placement.rects.push_back(block.rect);
            auto integerPlacement =
                phase1_warm_start::integerizePlacementCoordinates(
                    problem, placement);
            if (!integerPlacement) return false;
            design.outlineW = integerPlacement->W;
            design.outlineH = integerPlacement->H;
            for (int i = 0;
                i < static_cast<int>(design.blocks.size()); ++i)
                design.blocks[i].rect = integerPlacement->rects[i];
            return true;
        }
        catch (const exception&) {
            return false;
        }
    }

    static void commitLayout(Design& design, const LayoutResult& best) {
        design.outlineW = clampD(best.W, 1.0, design.maxOutlineW);
        design.outlineH = clampD(best.H, 1.0, design.maxOutlineH);
        design.blocks.clear();
        design.blocks.reserve(design.blockSpecs.size());
        for (int i = 0; i < static_cast<int>(design.blockSpecs.size()); ++i) {
            BlockInst b;
            b.spec = design.blockSpecs[i];
            if (i < static_cast<int>(best.rects.size())) b.rect = best.rects[i];
            else b.rect = makePackingShape(design, i, aspectMid(design.blockSpecs[i]));
            b.rect.x = clampD(b.rect.x, 0.0, max(0.0, design.outlineW - b.rect.w));
            b.rect.y = clampD(b.rect.y, 0.0, max(0.0, design.outlineH - b.rect.h));
            b.ftUsed = 0.0;
            b.ftOverflowArea = 0.0;
            design.blocks.push_back(b);
        }
        anchorRequiredCornerEdges(design);
        design.blockNameToIndex.clear();
        for (int i = 0; i < static_cast<int>(design.blocks.size()); ++i) {
            design.blockNameToIndex[design.blocks[i].spec.name] = i;
        }
    }

} // namespace

void Floorplanner::run(Design& design) {
    archiveDesigns.clear();
    archiveOrigins.clear();
    if (design.blockSpecs.empty()) {
        design.blocks.clear();
        return;
    }

    design.outlineW = design.maxOutlineW;
    design.outlineH = design.maxOutlineH;

    vector<vector<int>> orders = makeOrders(design);
    vector<ShapeState> shapes = makeShapeStates(design);

    LayoutResult best;
    bool have = false;
    int tried = 0;
    int packed = 0;

    const int nBlocks = static_cast<int>(design.blockSpecs.size());
    const int targetCandidates = max(1, nBlocks * nBlocks);
    const int maxCandidates = min(
        FAST_MAX_TOTAL_LAYOUT_CANDIDATES,
        max(targetCandidates, 2 * targetCandidates + 200)
    );
    int nEdge = 0, nHard = 0, nSoft = 0, nMovable = 0;
    for (const auto& spec : design.blockSpecs) {
        if (spec.type == BlockType::EDGE) ++nEdge;
        else {
            ++nMovable;
            if (spec.type == BlockType::HARD) ++nHard;
            else if (spec.type == BlockType::SOFT) ++nSoft;
        }
    }
    long long totalNets = 0;
    int maxNet = 0;
    int connCount = 0;
    if (!design.connections.empty()) {
        connCount = static_cast<int>(design.connections.size());
        for (const auto& c : design.connections) {
            totalNets += max(0, c.netCount);
            maxNet = max(maxNet, max(0, c.netCount));
        }
    }
    else {
        int nn = static_cast<int>(design.connMatrix.size());
        for (int i = 0; i < nn; ++i) {
            for (int j = i + 1; j < static_cast<int>(design.connMatrix[i].size()); ++j) {
                int nets = totalConnBetween(design, i, j);
                if (nets <= 0) continue;
                ++connCount;
                totalNets += nets;
                maxNet = max(maxNet, nets);
            }
        }
    }

    int strictTried = 0, strictPacked = 0, strictLegal = 0, strictEdgeOK = 0;
    int looseTried = 0, loosePacked = 0, looseLegal = 0, looseEdgeOK = 0;
    int bestUpdates = 0;
    if (FAST_VERBOSE_LOG) {
        cerr << fixed << setprecision(3)
            << "[FastSourcePack/Config] blocks=" << nBlocks
            << " movable=" << nMovable
            << " edge=" << nEdge
            << " hard=" << nHard
            << " soft=" << nSoft
            << " conn=" << connCount
            << " totalNets=" << totalNets
            << " maxNet=" << maxNet
            << " orders=" << orders.size()
            << " shapes=" << shapes.size()
            << " n2Target=" << targetCandidates
            << " maxCap=" << maxCandidates
            << " maxOutline=" << design.maxOutlineW << "x" << design.maxOutlineH
            << " nominalArea=" << totalNominalArea(design)
            << " packingArea=" << totalPackingArea(design)
            << "\n";
        cerr << fixed << setprecision(3)
            << "[FastSourcePack/GapPolicy] density=" << ROUTE_DENSITY
            << " fullMaxGap=" << maxSingleNetRequired(design)
            << " snap=" << sliverSnapThresholdFast(design)
            << " splitScale=" << SPLIT_AWARE_MIN_SCALE_FAST << ".." << SPLIT_AWARE_MAX_SCALE_FAST
            << " endpointGuardScale=" << ENDPOINT_GUARD_SCALE_FAST
            << " endpointGuardCap=" << ENDPOINT_GUARD_CAP_FAST
            << " dsu=" << (ENABLE_DSU_POST_COMPACTION ? "ON" : "OFF")
            << " dsuMaxShrink=" << DSU_MAX_AREA_SHRINK
            << " ftSoftReserve=" << (ENABLE_FT_SOFT_RESERVE ? "ON" : "OFF")
            << " softShapePerturb=" << (ENABLE_SOFT_SHAPE_PERTURB ? "ON" : "OFF")
            << "\n";
        if (ENABLE_FT_SOFT_RESERVE) {
            int ftCnt = 0;
            double ftSum = 0.0, ftMax = 0.0;
            for (int i = 0; i < static_cast<int>(design.blockSpecs.size()); ++i) {
                if (design.blockSpecs[i].type != BlockType::SOFT) continue;
                double rr = softFtExpansionRate(design, i);
                if (rr > 1e-9) { ++ftCnt; ftSum += rr; ftMax = max(ftMax, rr); }
            }
            cerr << fixed << setprecision(3)
                << "[FastSourcePack/FTReserve] softCount=" << ftCnt
                << " avgRate=" << (ftCnt ? ftSum / ftCnt : 0.0)
                << " maxRate=" << ftMax
                << " nominalArea=" << totalNominalArea(design)
                << " packingArea=" << totalPackingArea(design)
                << " addedArea=" << (totalPackingArea(design) - totalNominalArea(design))
                << "\n";
        }
    }

    vector<LayoutResult> explicitArchive;

    auto tryCandidate = [&](const ShapeState& st, const vector<int>& order, double W, double H, bool strictEdge) {
        ++tried;
        if (strictEdge) ++strictTried; else ++looseTried;
        LayoutResult cur;
        if (!tryExplicitWH(design, order, st, W, H, strictEdge, cur)) return;
        ++packed;
        if (strictEdge) {
            ++strictPacked;
            if (cur.legal) ++strictLegal;
            if (cur.strictEdgeLegal) ++strictEdgeOK;
        }
        else {
            ++loosePacked;
            if (cur.legal) ++looseLegal;
            if (cur.strictEdgeLegal) ++looseEdgeOK;
        }
        if (!cur.strictEdgeLegal) return;
        addToBStarArchive(design, explicitArchive, cur);
        if (!have || betterLayout(cur, best)) {
            best = std::move(cur);
            have = true;
            ++bestUpdates;
        }
        };

    auto runPass = [&](bool strictEdge, int stopAfterTried, bool ignoreGlobalCap = false) {
        for (const ShapeState& st : shapes) {
            vector<double> widths = makeWidthTrials(design, st);
            for (const vector<int>& order : orders) {
                for (double W : widths) {
                    vector<double> heights = makeHeightTrials(design, st, W);
                    for (double H : heights) {
                        if (tried >= stopAfterTried && have && best.legal) return;
                        if (!ignoreGlobalCap && tried >= maxCandidates && have) return;
                        tryCandidate(st, order, W, H, strictEdge);
                    }
                }
            }
        }
        };

    auto printPassSummary = [&](const string& name, int triedBefore, int packedBefore, int legalBefore) {
        if (!FAST_VERBOSE_LOG) return;
        int legalNow = strictLegal + looseLegal;
        cerr << fixed << setprecision(3)
            << "[FastSourcePack/Pass] name=" << name
            << " triedDelta=" << (tried - triedBefore)
            << " packedDelta=" << (packed - packedBefore)
            << " legalDelta=" << (legalNow - legalBefore)
            << " totalTried=" << tried
            << " totalPacked=" << packed
            << " strictPacked=" << strictPacked
            << " loosePacked=" << loosePacked
            << " strictLegal=" << strictLegal
            << " looseLegal=" << looseLegal
            << " strictEdgeOK=" << strictEdgeOK
            << " looseEdgeOK=" << looseEdgeOK
            << " bestUpdates=" << bestUpdates
            << " have=" << (have ? "Y" : "N");
        if (have) {
            cerr << " bestLegal=" << (best.legal ? "Y" : "N")
                << " bestEdgeLegal=" << (best.strictEdgeLegal ? "Y" : "N")
                << " bestArea=" << best.area
                << " bestW/H=" << best.W << "x" << best.H
                << " bestHpwl=" << best.hpwl
                << " bestRouteGap=" << best.routePenalty
                << " gapPart=" << best.routeGapPenaltyPart
                << " portPart=" << best.routePortPenaltyPart
                << " violPairs=" << best.routeViolPairs << "/" << best.routeConnectedPairs;
        }
        cerr << "\n";
        };

    // Main generator: B*-tree SA.  The SA acceptance cost is outline area only.
    // The old explicit W/H candidate sweep is now only a recovery fallback if the
    // B*-tree annealer cannot produce a legal strict-edge placement.
    bool usedBStarSA = false;
    if (ENABLE_BSTAR_AREA_SA) {
        int saTried = 0, saPacked = 0, saLegal = 0;
        vector<LayoutResult> saArchive;
        BStarState saBestState;
        LayoutResult saBest = runBStarAreaSA(design, saTried, saPacked, saLegal, &saArchive, &saBestState);
        int secondTried = 0, secondPacked = 0, secondLegal = 0;
        vector<LayoutResult> secondArchive = runBStarSecondProxySA(design, saBestState, secondTried, secondPacked, secondLegal);
        int wireTried = 0, wirePacked = 0, wireLegal = 0;
        int spreadTried = 0, spreadPacked = 0, spreadLegal = 0;
        vector<LayoutResult> wireArchive;
        vector<LayoutResult> spreadArchive;
        vector<LayoutResult> compactSpreadArchive;
        if (static_cast<int>(design.blockSpecs.size()) > 8) {
            vector<LayoutResult> wireSeedLayouts = saArchive;
            for (const LayoutResult& second : secondArchive) wireSeedLayouts.push_back(second);
            vector<BStarState> wireSeeds = makeWireBStarSeeds(design, saBestState, wireSeedLayouts, false);
            wireArchive = runBStarWireProxySA(design, wireSeeds, false, wireTried, wirePacked, wireLegal);
            vector<BStarState> spreadSeeds = makeWireBStarSeeds(design, saBestState, wireSeedLayouts, true);
            spreadArchive = runBStarWireProxySA(design, spreadSeeds, true, spreadTried, spreadPacked, spreadLegal);
            compactSpreadArchive = makeCompactAfterSpreadArchive(design, spreadArchive);
        }
        vector<LayoutResult> forceArchive = makeHpwlForceRefinedArchive(design, saArchive);
        vector<LayoutResult> alignmentSeeds = saArchive;
        for (const LayoutResult& second : secondArchive) alignmentSeeds.push_back(second);
        for (const LayoutResult& wire : wireArchive) alignmentSeeds.push_back(wire);
        for (const LayoutResult& compact : compactSpreadArchive) alignmentSeeds.push_back(compact);
        for (const LayoutResult& refined : forceArchive) alignmentSeeds.push_back(refined);
        vector<LayoutResult> alignmentArchive = makeAlignmentArchive(design, alignmentSeeds);
        const int saArchiveCount = static_cast<int>(saArchive.size());
        const int secondArchiveCount = static_cast<int>(secondArchive.size());
        const int wireArchiveCount = static_cast<int>(wireArchive.size());
        const int spreadArchiveCount = static_cast<int>(spreadArchive.size());
        const int compactSpreadArchiveCount = static_cast<int>(compactSpreadArchive.size());
        const int forceArchiveCount = static_cast<int>(forceArchive.size());
        vector<LayoutResult> commitArchive = saArchive;
        for (auto& second : secondArchive) commitArchive.push_back(std::move(second));
        for (auto& wire : wireArchive) commitArchive.push_back(std::move(wire));
        for (auto& spread : spreadArchive) commitArchive.push_back(std::move(spread));
        for (auto& compact : compactSpreadArchive) commitArchive.push_back(std::move(compact));
        for (auto& refined : forceArchive) commitArchive.push_back(std::move(refined));
        for (auto& aligned : alignmentArchive) commitArchive.push_back(std::move(aligned));
        const int archiveLimit = min(
            BSTAR_ARCHIVE_LIMIT +
                BSTAR_SECOND_ARCHIVE_LIMIT +
                BSTAR_WIRE_ARCHIVE_LIMIT +
                BSTAR_SPREAD_ARCHIVE_LIMIT +
                BSTAR_COMPACT_SPREAD_KEEP +
                2 * PHASE1_WARM_RESTARTS_LARGE +
                FORCE_REFINE_KEEP_LIMIT +
                ALIGNMENT_KEEP_LIMIT,
            static_cast<int>(commitArchive.size()));
        archiveDesigns.reserve(static_cast<size_t>(archiveLimit) + 2);
        archiveOrigins.reserve(static_cast<size_t>(archiveLimit) + 2);
        for (int ai = 0; ai < archiveLimit; ++ai) {
            Design candidate = design;
            commitLayout(candidate, commitArchive[ai]);
            archiveDesigns.push_back(std::move(candidate));
            if (ai < saArchiveCount) archiveOrigins.push_back("archive");
            else if (ai < saArchiveCount + secondArchiveCount) archiveOrigins.push_back("second-sa");
            else if (ai < saArchiveCount + secondArchiveCount + wireArchiveCount) archiveOrigins.push_back("wire-sa");
            else if (ai < saArchiveCount + secondArchiveCount + wireArchiveCount + spreadArchiveCount) archiveOrigins.push_back("spread-wire-sa");
            else if (ai < saArchiveCount + secondArchiveCount + wireArchiveCount + spreadArchiveCount + compactSpreadArchiveCount) archiveOrigins.push_back("compact-spread");
            else if (ai < saArchiveCount + secondArchiveCount + wireArchiveCount + spreadArchiveCount + compactSpreadArchiveCount + forceArchiveCount) archiveOrigins.push_back("force");
            else archiveOrigins.push_back("alignment");
        }
        tried += saTried + secondTried + wireTried + spreadTried;
        packed += saPacked + secondPacked + wirePacked + spreadPacked;
        strictTried += saTried + secondTried + wireTried + spreadTried;
        strictPacked += saPacked + secondPacked + wirePacked + spreadPacked;
        strictLegal += saLegal + secondLegal + wireLegal + spreadLegal;
        strictEdgeOK += saLegal + secondLegal + wireLegal + spreadLegal;
        if (saBest.legal) {
            best = std::move(saBest);
            have = true;
            usedBStarSA = true;
            ++bestUpdates;
        }
        if (FAST_VERBOSE_LOG) {
            cerr << fixed << setprecision(3)
                << "[MainFlow] BStarSA=" << (usedBStarSA ? "USED" : "FAILED")
                << " saTried=" << saTried
                << " saPacked=" << saPacked
                << " saLegal=" << saLegal;
            if (have) {
                cerr << " bestArea=" << best.area
                    << " bestW/H=" << best.W << "x" << best.H
                    << " routeGap=" << best.routePenalty
                    << " archive=" << saArchive.size()
                    << " secondArchive=" << secondArchive.size()
                    << " secondTried=" << secondTried
                    << " wireArchive=" << wireArchiveCount
                    << " wireTried=" << wireTried
                    << " spreadArchive=" << spreadArchiveCount
                    << " spreadTried=" << spreadTried
                    << " compactSpread=" << compactSpreadArchiveCount
                    << " forceArchive=" << forceArchive.size()
                    << " alignmentArchive=" << alignmentArchive.size()
                    << " stripPeak=" << best.stripPeakUtilProxy
                    << " stripOvProxy=" << best.stripOverflowProxy;
            }
            cerr << "\n";
        }
    }

    if (usedBStarSA && ENABLE_EXPLICIT_TOPOLOGY_SUPPLEMENT && nBlocks <= 18) {
        int passTried0 = tried, passPacked0 = packed, passLegal0 = strictLegal + looseLegal;
        const int extraTrials = min(EXPLICIT_TOPOLOGY_SUPPLEMENT_TRIALS, max(240, targetCandidates * 2));
        runPass(true, tried + extraTrials, true);
        printPassSummary("explicit-topology-supplement", passTried0, passPacked0, passLegal0);
    }
    if (!have || !best.legal) {
        int passTried0 = tried, passPacked0 = packed, passLegal0 = strictLegal + looseLegal;
        runPass(true, targetCandidates);
        printPassSummary("strict-edge-recovery", passTried0, passPacked0, passLegal0);
    }

    // If strict EDGE zones prevented legal packing, use permissive EDGE packing
    // only for additional recovery candidates, still under the global cap.
    if (!have || !best.legal) {
        int passTried0 = tried, passPacked0 = packed, passLegal0 = strictLegal + looseLegal;
        runPass(false, maxCandidates);
        printPassSummary("permissive-edge-recovery", passTried0, passPacked0, passLegal0);
    }

    // HeightRefine belongs to the old explicit-W/H recovery path.  When B*-tree SA
    // succeeds, we do not overwrite it with a greedy order-based refinement because
    // that would no longer be a B*-tree annealed solution.
    if (!usedBStarSA && have && best.legal) {
        LayoutResult beforeRefine = best;
        LayoutResult refined = best;
        double lo = minHeightBound(design, shapes.front());
        double hi = best.H;
        int refineTried = 0, refineSuccess = 0, refineImprove = 0;
        for (int it = 0; it < FAST_HEIGHT_BISECT; ++it) {
            double mid = 0.5 * (lo + hi);
            LayoutResult cur;
            bool accepted = false;
            for (const ShapeState& st : shapes) {
                if (accepted) break;
                for (const vector<int>& order : orders) {
                    ++refineTried;
                    if (tryExplicitWH(design, order, st, best.W, mid, best.strictEdgeLegal, cur) && cur.legal) {
                        ++refineSuccess;
                        if (betterLayout(cur, refined)) {
                            refined = std::move(cur);
                            hi = mid;
                            ++refineImprove;
                        }
                        accepted = true;
                        break;
                    }
                }
            }
            if (!accepted) lo = mid;
        }
        if (betterLayout(refined, best)) best = std::move(refined);
        if (FAST_VERBOSE_LOG) {
            cerr << fixed << setprecision(3)
                << "[HeightRefine] tried=" << refineTried
                << " success=" << refineSuccess
                << " improve=" << refineImprove
                << " area=" << beforeRefine.area << "->" << best.area
                << " H=" << beforeRefine.H << "->" << best.H
                << " hpwl=" << beforeRefine.hpwl << "->" << best.hpwl
                << " routeGap=" << beforeRefine.routePenalty << "->" << best.routePenalty
                << "\n";
        }
    }

    if (!have) {
        best = fallbackShelf(design);
        if (FAST_VERBOSE_LOG) {
            cerr << fixed << setprecision(3)
                << "[FastSourcePack/Fallback] used=Y"
                << " legal=" << (best.legal ? "Y" : "N")
                << " area=" << best.area
                << " W/H=" << best.W << "x" << best.H
                << " hpwl=" << best.hpwl
                << " routeGap=" << best.routePenalty
                << "\n";
        }
    }

    LayoutResult preDSU = best;
    // Phase 2: bounded DSU-style compaction.  Phase 1 used relaxed requiredGap;
    // this post pass reclaims deadspace without adding any candidate/SA cost.
    const bool dsuRisky = preDSU.stripPeakUtilProxy > 0.72 || preDSU.stripHotComponents > 0;
    if (best.legal && !dsuRisky) {
        best = dsuPostCompaction(design, best);
    }
    else if (best.legal && FAST_VERBOSE_LOG) {
        cerr << fixed << setprecision(3)
            << "[DSUPost] skipped=Y"
            << " reason=strip_proxy_risk"
            << " stripPeak=" << preDSU.stripPeakUtilProxy
            << " stripHot=" << preDSU.stripHotComponents
            << " stripOvProxy=" << preDSU.stripOverflowProxy
            << "\n";
    }

    if (FAST_VERBOSE_LOG) {
        cerr << fixed << setprecision(3)
            << "[FastSourcePack/Selected] preDSUArea=" << preDSU.area
            << " postDSUArea=" << best.area
            << " preW/H=" << preDSU.W << "x" << preDSU.H
            << " postW/H=" << best.W << "x" << best.H
            << " preRouteGap=" << preDSU.routePenalty
            << " postRouteGap=" << best.routePenalty
            << " preHpwl=" << preDSU.hpwl
            << " postHpwl=" << best.hpwl
            << "\n";
    }

    if (!explicitArchive.empty()) {
        sort(explicitArchive.begin(), explicitArchive.end(), betterArchiveLayout);
        const int explicitLimit = min(EXPLICIT_TOPOLOGY_SUPPLEMENT_KEEP, static_cast<int>(explicitArchive.size()));
        for (int ai = 0; ai < explicitLimit; ++ai) {
            Design candidate = design;
            commitLayout(candidate, explicitArchive[ai]);
            archiveDesigns.push_back(std::move(candidate));
            archiveOrigins.push_back("explicit");
        }
        if (FAST_VERBOSE_LOG) {
            cerr << fixed << setprecision(3)
                << "[ExplicitTopologySupplement] archive=" << explicitArchive.size()
                << " keep=" << explicitLimit
                << " bestArea=" << explicitArchive.front().area
                << " bestW/H=" << explicitArchive.front().W << "x" << explicitArchive.front().H
                << " bestStripPeak=" << explicitArchive.front().stripPeakUtilProxy
                << "\n";
        }
    }

    vector<LayoutResult> compactArchive = makeCompactColumnArchive(design);
    for (int ai = 0; ai < static_cast<int>(compactArchive.size()); ++ai) {
        Design candidate = design;
        commitLayout(candidate, compactArchive[ai]);
        archiveDesigns.push_back(std::move(candidate));
        archiveOrigins.push_back("compact-column");
    }

    commitLayout(design, best);
    archiveDesigns.insert(archiveDesigns.begin(), design);
    archiveOrigins.insert(archiveOrigins.begin(), "selected");

    cerr << fixed << setprecision(3)
        << "[FastSourcePack/N2] n2Target=" << targetCandidates
        << " tried=" << tried
        << " packed=" << packed
        << " legal=" << (best.legal ? "Y" : "N")
        << " edgeLegal=" << (best.strictEdgeLegal ? "Y" : "N")
        << " area=" << best.area
        << " W/H=" << best.W << "x" << best.H
        << " hpwl=" << best.hpwl
        << " routeGap=" << best.routePenalty
        << " gapPart=" << best.routeGapPenaltyPart
        << " portPart=" << best.routePortPenaltyPart
        << " maxGapMiss=" << best.routeMaxGapMiss
        << " maxPortMiss=" << best.routeMaxPortMiss
        << " routeViolPairs=" << best.routeViolPairs << "/" << best.routeConnectedPairs
        << " routeXY=" << best.routeXInterfaces << "/" << best.routeYInterfaces
        << " overlap=" << best.overlap
        << " edgeViol=" << best.edgeViol
        << " outlineViol=" << best.outlineViol
        << " strictTried/Packed/Legal=" << strictTried << "/" << strictPacked << "/" << strictLegal
        << " looseTried/Packed/Legal=" << looseTried << "/" << loosePacked << "/" << looseLegal
        << " bestUpdates=" << bestUpdates
        << " maxTriedCap=" << maxCandidates
        << "\n";
}

// --------------------------------------------------------------------------
// Compatibility methods for the existing Floorplanner.hpp interface.
// --------------------------------------------------------------------------

void Floorplanner::setEdgePlacementMode(int mode) {
    g_edgePlacementMode = max(0, min(2, mode));
}

void Floorplanner::setRoutingFeedback(const Design& routedDesign) {
    routingFeedbackDesign = routedDesign;
    routingFeedbackEnabled = true;
}

const vector<Design>& Floorplanner::archivedCandidates() const {
    return archiveDesigns;
}

const vector<string>& Floorplanner::archivedCandidateOrigins() const {
    return archiveOrigins;
}
