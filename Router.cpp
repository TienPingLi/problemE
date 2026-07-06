#include "Router.hpp"
#include "Utility.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace std;

namespace {

    // ============================================================================
    // 中文註解版 Router.cpp：Directional-capacity + Bottleneck-aware Step1-4
    // ----------------------------------------------------------------------------
    // 本檔案的定位：
    //   這是一版「可當 baseline 使用」的 Router。主體仍然是 Step1~4 的
    //   Dijkstra-based global route，Step5 只作為 deferred fallback：
    //   channel-only whole route + allow overflow。
    //
    // 為什麼要改成 directional capacity？
    //   官方 Q&A 已確認 channel 的水平與垂直方向容量要分開看：
    //     1) 水平走線：edge 1 <-> edge 3，在 channel 裡左右走，
    //        容量 = channel 高度 h * CHANNEL_DENSITY。
    //     2) 垂直走線：edge 2 <-> edge 4，在 channel 裡上下走，
    //        容量 = channel 寬度 w * CHANNEL_DENSITY。
    //     3) L-shape / turn：若在同一個 channel 中由相鄰 edge 進出，
    //        會同時產生水平段與垂直段，因此水平、垂直各吃一次容量。
    //
    // 目前仍採用 baseline 的「方向加總模型」：
    //   對每個 channel 統計 horizontalUsed / verticalUsed，然後分別與
    //   h*25 / w*25 比較。這比舊版 scalar capacity 正確很多，但尚未做到
    //   interval-based 的「重疊區」精算模型。
    //
    // Router 策略總覽：
    //   Step1A：Channel-only whole route，capacityScale=0.85，先保留容量。
    //   Step1B：Channel-only whole route，capacityScale=1.00，放寬容量。
    //   Step2A：Channel-only limited split，capacityScale=0.90。
    //   Step2B：Channel-only limited split，capacityScale=1.00。
    //   Step3 ：FT-enabled whole route，只有 soft block 能作為中繼 feedthrough。
    //   Step4 ：FT-enabled limited split。
    //   Step5 ：Deferred channel-only whole route + allow overflow。
    //
    // Dijkstra 的核心想法：
    //   舊版 Dijkstra 只把「rectangle」當成 node，因此走進 channel 時不知道
    //   是從哪個 edge 進、從哪個 edge 出，也就無法判斷水平/垂直容量。
    //   本版使用 edge-state Dijkstra：state=(nodeId, inEdge)。
    //   當 Dijkstra 從一個 channel state 離開時，已經知道：
    //       inEdge  = 從哪個 edge 進入該 channel
    //       outEdge = 從哪個 edge 離開該 channel
    //   因此可以判斷這次 traversal 是：
    //       1<->3：水平 LR 使用量 + nets
    //       2<->4：垂直 TB 使用量 + nets
    //       turn ：水平與垂直使用量各 + nets
    //
    // Dijkstra cost 組成：
    //   path edge cost = wireCost + resourcePenalty
    //   wireCost = ROUTER_WIRE_WEIGHT * 幾何距離 * netCount
    //   resourcePenalty 對 channel 包含：
    //       * projected utilization penalty：越接近滿載越貴
    //       * near-full convex penalty：超過 60%、85% 後加速變貴
    //       * narrow-component penalty：小容量方向分量會被強烈懲罰
    //       * criticality penalty：預估會被很多 connection 經過的 bottleneck 較貴
    //       * Step5 overflow penalty：allow overflow 時，新增 overflow 會極端昂貴
    //   resourcePenalty 對 soft block 包含：
    //       * 固定 FT penalty
    //       * per-net FT penalty
    //       * feedthrough 所需面積增加的估計 penalty
    //
    // Split 的定位：
    //   Split 不是主 router，只是容量修補工具。若 whole route 過不了，才允許
    //   limited split。Split 不允許拆太多條、不能走極窄方向分量、不能只是在
    //   同一個 bottleneck 裡塞小 chunk。
    //
    // Report 的方向資訊：
    //   最後會印出每個 top channel 的 LR/TB used/cap/util/overflow，並指出
    //   maxUtil 是哪一個 channel 的哪一個方向分量。
    // ============================================================================

    static constexpr bool ROUTER_DIAG_ENABLE = false;
    static constexpr bool ROUTER_DIAG_PRINT_DECISIONS = false;
    static constexpr bool ROUTER_DIAG_PRINT_PATHS = false;
    static constexpr int  ROUTER_DIAG_TOP_CHANNELS = 12;
    static constexpr bool ROUTER_DETOUR_REPORT_ENABLE = true;
    static constexpr int  ROUTER_DETOUR_TOP_ROUTES = 10;
    static constexpr int  ROUTER_CORRIDOR_TOP_RUNS = 10;

    static constexpr double ROUTER_WIRE_WEIGHT = 0.20;
    static constexpr bool ENABLE_EDGE_PAIR_WIRE_METRIC = true;

    // Capacity reservation.  Early channel-only attempts should not fill bottlenecks.
    static constexpr double CAP_SCALE_CH_WHOLE_RESERVED = 0.85;
    static constexpr double CAP_SCALE_CH_SPLIT_RESERVED = 0.90;
    static constexpr double CAP_SCALE_FULL = 1.00;

    // Directional bottleneck penalties.
    static constexpr double CHANNEL_CONGESTION_WEIGHT = 80.0;
    static constexpr double CHANNEL_NEAR_FULL_WEIGHT = 2500.0;
    static constexpr double CHANNEL_CRITICALITY_WEIGHT = 450.0;
    static constexpr double CHANNEL_NARROW_WEIGHT = 7500.0;

    // Component-level channel thresholds.  These apply to the exact component used
    // by a transition.  A very thin channel may still be usable in the long direction,
    // but it is forbidden for the component whose capacity is tiny.
    static constexpr double EXTREME_COMPONENT_CAP = 50.0;    // forbidden even in Step5
    static constexpr double NARROW_COMPONENT_CAP = 300.0;   // strong penalty

    // Split health rules.  Split should distribute a large bundle across healthy
    // corridors; it must not create many tiny chunks or fill a single bottleneck.
    static constexpr int    MAX_SPLIT_ROUTES = 50;
    static constexpr double SPLIT_MIN_COMPONENT_CAP = 100.0;
    static constexpr double SPLIT_MAX_PROJECTED_UTIL = 0.95;
    static constexpr double SPLIT_BOTTLENECK_SHARE = 0.85;
    static constexpr double SPLIT_BOTTLENECK_FINAL_UTIL = 0.90;

    static constexpr int TARGETED_EXCHANGE_MAX_CALLS_PER_RUN = 16;
    static constexpr int TARGETED_EXCHANGE_MAX_ATTEMPTS_PER_RUN = 64;
    static constexpr int TARGETED_EXCHANGE_MAX_DONORS = 6;

    static constexpr int SPLIT_CHUNK_SIZE_1 = 300;
    static constexpr int SPLIT_CHUNK_SIZE_2 = 100;
    static constexpr int SPLIT_CHUNK_SIZE_3 = 50;
    static constexpr int SPLIT_CHUNK_SIZE_4 = 20;

    // Soft feedthrough cost.  FT is a fallback, not default routing.
    static constexpr double SOFT_FT_FIXED_PENALTY_BASE = 50000.0;
    static constexpr double SOFT_FT_PER_NET_PENALTY_BASE = 10.0;
    static constexpr double SOFT_FT_FIXED_PENALTY_RELAXED = 2000.0;
    static constexpr double SOFT_FT_PER_NET_PENALTY_RELAXED = 0.5;
    static double g_softFtFixedPenalty = SOFT_FT_FIXED_PENALTY_BASE;
    static double g_softFtPerNetPenalty = SOFT_FT_PER_NET_PENALTY_BASE;
    static constexpr double FT_INCREMENTAL_AREA_WEIGHT = 10.0;
    static constexpr double FT_OVERFLOW_EXTRA_WEIGHT_BASE = 0.0;
    static constexpr double FT_OVERFLOW_EXTRA_WEIGHT_REPAIR = 1.0e3;
    static double g_ftOverflowExtraWeight = FT_OVERFLOW_EXTRA_WEIGHT_BASE;
    static bool g_ftHardCapacityGuard = true;

    // Concurrent allocation.  The legal-only allocator is kept available, but the
    // current archive portfolio gets the same cost faster from legacy routing plus
    // legal WL exchange, so production keeps this disabled.
    static constexpr bool ENABLE_CONCURRENT_ALLOC_ROUTER = true;
    static constexpr bool ENABLE_CONCURRENT_BIASED_KPATH = true;
    static constexpr int CONCURRENT_ALLOC_ITERS = 4;
    static constexpr int CONCURRENT_ROUND_TOPK = 3;
    static constexpr double CONCURRENT_SOFTMAX_TEMP = 2400.0;
    static constexpr double CONCURRENT_PRICE_FREE_UTIL = 0.72;
    static constexpr double CONCURRENT_PRICE_WEIGHT = 9000.0;
    static constexpr double CONCURRENT_OVERFLOW_PRICE_WEIGHT = 220000.0;
    static constexpr double CONCURRENT_FT_PRICE_WEIGHT = 45000.0;

    // Proactive FT selection.  Router::run does not receive alpha, so this uses
    // the default contest alpha as a local routing-cost proxy.
    static constexpr double ROUTE_CHOICE_ALPHA_PROXY = 0.20;
    static constexpr double FT_CHOICE_AREA_WEIGHT = 1.00;
    static constexpr double FT_CHOICE_OVERFLOW_AREA_WEIGHT = 1000000.0;
    static constexpr double CHANNEL_UTIL_RISK_START = 0.70;
    static constexpr double CHANNEL_UTIL_RISK_WEIGHT = 180000.0;
    static constexpr double FT_PROACTIVE_UTIL_TRIGGER = 0.78;
    static constexpr double FT_PROACTIVE_SCORE_RELAX = 1.08;
    static constexpr double FT_PROACTIVE_SCORE_ABS_RELAX = 75000.0;

    // Step5 overflow weights.  Step5 is no-open rescue only, not a normal router.
    static constexpr bool ENABLE_STEP5_OVERFLOW_COMMIT = true;
    static constexpr double CHANNEL_OVERFLOW_LINEAR_WEIGHT = 1.0e8;
    static constexpr double CHANNEL_OVERFLOW_QUADRATIC_WEIGHT = 1.0e5;

    // Post-route congestion repair.  This pass rips up route groups that actually
    // contribute to final overflow and reroutes them with hot channel/soft-block
    // bias.  A candidate is accepted only if it improves router-side score.
    static constexpr bool ENABLE_CONGESTION_RIPUP = true;
    static constexpr int RIPUP_MAX_GROUPS_MID = 30;
    static constexpr int RIPUP_MAX_GROUPS_LARGE = 40;
    static constexpr double RIPUP_MIN_GROUP_SCORE = 1.0;

    // Negotiated congestion.  Overflow is not the only bad state: a legal channel
    // at 90%+ utilization is fragile and tends to create later detours.  These
    // terms make reroute passes avoid both actual overflow and near-full corridors.
    static constexpr double NEGOTIATED_UTIL_START = 0.82;
    static constexpr double NEGOTIATED_LEGAL_RIPUP_MIN_UTIL = 0.90;
    static constexpr double NEGOTIATED_HISTORY_DECAY = 0.65;
    static constexpr double NEGOTIATED_HISTORY_GAIN = 1.75;
    static constexpr double NEGOTIATED_HISTORY_WEIGHT = 0.65;
    static constexpr double NEGOTIATED_RISK_IMPROVE_RATIO = 0.92;
    static constexpr double NEGOTIATED_MAX_UTIL_IMPROVE = 0.02;
    static constexpr double NEGOTIATED_WL_RELAX = 1.006;
    static constexpr double NEGOTIATED_WL_ABS_RELAX = 200000.0;

    static constexpr double CHANNEL_CAPACITY_EPS = 1e-7;
    static constexpr double TOUCH_EPS = 1e-3;
    static constexpr double TOUCH_OVERLAP_EPS = 1e-7;

    // Number of edge states per node in directional Dijkstra.  Edge id 0 means
    // start/no-entry; 1..4 follow problem convention.
    static constexpr int EDGE_STATE_COUNT = 5;

    // ----------------------------------------------------------------------------
    // Routing mode globals.  Kept as globals to avoid editing Router.hpp.
    // ----------------------------------------------------------------------------
    enum class RouteMode { CHANNEL_ONLY, FT_ENABLED };
    static RouteMode g_routeMode = RouteMode::CHANNEL_ONLY;
    static bool   g_allowChannelOverflow = false;
    static bool   g_splitRouting = false;
    static bool   g_requireSoftFT = false;
    static double g_capacityScale = CAP_SCALE_FULL;
    static vector<double> g_channelCriticality;
    static unordered_map<string, int> g_channelIndexByName;
    static bool g_congestionRerouteMode = false;
    static double g_congestionBiasWeight = 0.0;
    static vector<array<double, 2>> g_hotChannelBias;
    static vector<array<double, 2>> g_negotiatedChannelHistory;
    static vector<double> g_hotSoftBias;
    static bool g_contactAwareCostEnabled = false;
    static bool g_wirePolishMode = false;

    string modeName(RouteMode mode) {
        return mode == RouteMode::CHANNEL_ONLY ? "CHANNEL_ONLY" : "FT_ENABLED";
    }

    bool validEdgeLocal(int e) { return e >= 1 && e <= 4; }
    bool allowsPortEdgeLocal(const BlockSpec& spec, int edge) {
        if (!validEdgeLocal(edge)) return false;
        if (spec.portEdges.empty()) return true;
        return find(spec.portEdges.begin(), spec.portEdges.end(), edge) != spec.portEdges.end();
    }

    int preferredEndpointEdgeLocal(const BlockSpec& spec, const Rect& self, const Rect& other) {
        int geomEdge = 0;
        const double dx = rectCx(other) - rectCx(self);
        const double dy = rectCy(other) - rectCy(self);
        if (fabs(dx) >= fabs(dy)) geomEdge = dx >= 0.0 ? 3 : 1;
        else geomEdge = dy >= 0.0 ? 2 : 4;

        if (allowsPortEdgeLocal(spec, geomEdge)) return geomEdge;
        if (!spec.portEdges.empty()) return spec.portEdges.front();
        return geomEdge;
    }
    bool isOppositeLR(int a, int b) { return (a == 1 && b == 3) || (a == 3 && b == 1); }
    bool isOppositeTB(int a, int b) { return (a == 2 && b == 4) || (a == 4 && b == 2); }
    bool isTurn(int a, int b) {
        if (!validEdgeLocal(a) || !validEdgeLocal(b)) return false;
        return !isOppositeLR(a, b) && !isOppositeTB(a, b) && a != b;
    }

    bool isChannelName(const string& name) { return name.rfind("CH", 0) == 0; }

    bool contactGuidingPointLocal(const Rect& a, int ea, const Rect& b, int eb, pair<double, double>& p) {
        if (!validEdgeLocal(ea) || !validEdgeLocal(eb)) return false;

        if (ea == 3 && eb == 1 && fabs(rectRight(a) - b.x) <= TOUCH_EPS) {
            const double lo = max(a.y, b.y);
            const double hi = min(rectTop(a), rectTop(b));
            if (hi <= lo + TOUCH_OVERLAP_EPS) return false;
            p = { 0.5 * (rectRight(a) + b.x), 0.5 * (lo + hi) };
            return true;
        }
        if (ea == 1 && eb == 3 && fabs(a.x - rectRight(b)) <= TOUCH_EPS) {
            const double lo = max(a.y, b.y);
            const double hi = min(rectTop(a), rectTop(b));
            if (hi <= lo + TOUCH_OVERLAP_EPS) return false;
            p = { 0.5 * (a.x + rectRight(b)), 0.5 * (lo + hi) };
            return true;
        }
        if (ea == 2 && eb == 4 && fabs(rectTop(a) - b.y) <= TOUCH_EPS) {
            const double lo = max(a.x, b.x);
            const double hi = min(rectRight(a), rectRight(b));
            if (hi <= lo + TOUCH_OVERLAP_EPS) return false;
            p = { 0.5 * (lo + hi), 0.5 * (rectTop(a) + b.y) };
            return true;
        }
        if (ea == 4 && eb == 2 && fabs(a.y - rectTop(b)) <= TOUCH_EPS) {
            const double lo = max(a.x, b.x);
            const double hi = min(rectRight(a), rectRight(b));
            if (hi <= lo + TOUCH_OVERLAP_EPS) return false;
            p = { 0.5 * (lo + hi), 0.5 * (a.y + rectTop(b)) };
            return true;
        }
        return false;
    }

    bool isSoftBlockName(const Design& design, const string& name) {
        auto it = design.blockNameToIndex.find(name);
        if (it == design.blockNameToIndex.end()) return false;
        return design.blocks[it->second].spec.type == BlockType::SOFT;
    }

    const Channel* findChannelByName(const Design& design, const string& name) {
        for (const auto& ch : design.channels) if (ch.name == name) return &ch;
        return nullptr;
    }

    Channel* findChannelByNameMutable(Design& design, const string& name) {
        for (auto& ch : design.channels) if (ch.name == name) return &ch;
        return nullptr;
    }

    int channelIndexByName(const Design& design, const string& name) {
        auto it = g_channelIndexByName.find(name);
        if (it != g_channelIndexByName.end()) return it->second;
        for (int i = 0; i < static_cast<int>(design.channels.size()); ++i) {
            if (design.channels[i].name == name) return i;
        }
        return -1;
    }

    // 水平/垂直容量定義（依照 Q&A 修正）：
    //   * LR / horizontal：edge 1 <-> edge 3，代表線在 channel 內左右走。
    //     左右走的水平線需要沿 y 方向排開，因此容量取 channel 高度 h * 25。
    //   * TB / vertical：edge 2 <-> edge 4，代表線在 channel 內上下走。
    //     上下走的垂直線需要沿 x 方向排開，因此容量取 channel 寬度 w * 25。
    // 注意：這裡仍是 baseline 的「方向加總模型」，尚未做 interval / cut-based 重疊區模型。
    double capLR(const Channel& ch) { return max(0.0, ch.rect.h) * CHANNEL_DENSITY; }
    double capTB(const Channel& ch) { return max(0.0, ch.rect.w) * CHANNEL_DENSITY; }

    enum class DirComponent { LR, TB };

    struct ResourceKey {
        int channelIndex = -1;
        DirComponent dir = DirComponent::LR;
        bool operator<(const ResourceKey& o) const {
            if (channelIndex != o.channelIndex) return channelIndex < o.channelIndex;
            return static_cast<int>(dir) < static_cast<int>(o.dir);
        }
        bool operator==(const ResourceKey& o) const {
            return channelIndex == o.channelIndex && dir == o.dir;
        }
    };

    static set<ResourceKey> g_forbiddenResources;

    const char* shortCompName(DirComponent c) {
        return c == DirComponent::LR ? "LR" : "TB";
    }
    string compName(DirComponent c) {
        return c == DirComponent::LR
            ? "LR(horizontal, edge1<->edge3, cap=channel.h*25)"
            : "TB(vertical, edge2<->edge4, cap=channel.w*25)";
    }

    struct DirUse {
        double lr = 0.0;
        double tb = 0.0;
    };

    struct DirDelta {
        double lr = 0.0;
        double tb = 0.0;
    };

    DirDelta deltaForInternalTraversal(int inEdge, int outEdge, double nets) {
        DirDelta d;
        if (!validEdgeLocal(inEdge) || !validEdgeLocal(outEdge) || inEdge == outEdge) return d;
        if (isOppositeLR(inEdge, outEdge)) d.lr += nets;
        else if (isOppositeTB(inEdge, outEdge)) d.tb += nets;
        else if (isTurn(inEdge, outEdge)) { d.lr += nets; d.tb += nets; }
        return d;
    }

    double negotiatedUtilRisk(double util) {
        if (!std::isfinite(util)) return 1000.0;
        const double x = max(0.0, util - NEGOTIATED_UTIL_START);
        return x * x;
    }

    // ----------------------------------------------------------------------------
    // FT helpers
    // ----------------------------------------------------------------------------
    double ftRateForNetsLocal(const BlockSpec& spec, double ftNets) {
        if (ftNets <= 3000.0) return spec.ftRate[0];
        if (ftNets <= 6000.0) return spec.ftRate[1];
        if (ftNets <= 9000.0) return spec.ftRate[2];
        return spec.ftRate[3];
    }

    struct FTShapeLocal {
        double coreW = 0.0;
        double coreH = 0.0;
        double d = 0.0;
        double finalW = 0.0;
        double finalH = 0.0;
        double finalArea = 0.0;
    };

    FTShapeLocal ftShapeForNetsLocal(const BlockInst& b, double ftNets) {
        FTShapeLocal s;
        const double baseArea = max(1.0, b.spec.area);
        double aspect = b.rect.h > EPS ? b.rect.w / b.rect.h : 1.0;
        if (b.spec.aspectMin > EPS) aspect = max(aspect, b.spec.aspectMin);
        if (b.spec.aspectMax > EPS) aspect = min(aspect, b.spec.aspectMax);
        if (!std::isfinite(aspect) || aspect <= EPS) aspect = 1.0;

        s.coreW = sqrt(baseArea * aspect);
        s.coreH = baseArea / max(1.0, s.coreW);
        if (b.spec.type == BlockType::SOFT && ftNets > EPS) {
            const double rate = ftRateForNetsLocal(b.spec, ftNets);
            s.d = ceil((ftNets / CHANNEL_DENSITY) * rate) / 2.0;
        }
        s.finalW = s.coreW + s.d;
        s.finalH = s.coreH + s.d;
        s.finalArea = max(baseArea, s.finalW * s.finalH);
        return s;
    }

    double requiredAreaWithFTLocal(const BlockInst& b, double ftNets) {
        return ftShapeForNetsLocal(b, ftNets).finalArea;
    }
    double incrementalFTAreaLocal(const BlockInst& b, int netCount) {
        const double oldReq = requiredAreaWithFTLocal(b, b.ftUsed);
        const double newReq = requiredAreaWithFTLocal(b, b.ftUsed + static_cast<double>(netCount));
        return max(0.0, newReq - oldReq);
    }

    double estimatedFTOverflowAfterLocal(const BlockInst& b, int netCount) {
        const double currentArea = b.rect.w * b.rect.h;
        const double newReq = requiredAreaWithFTLocal(b, b.ftUsed + static_cast<double>(netCount));
        return max(0.0, newReq - currentArea);
    }

    double incrementalFTOverflowLocal(const BlockInst& b, int netCount) {
        const double currentArea = b.rect.w * b.rect.h;
        const double oldReq = requiredAreaWithFTLocal(b, b.ftUsed);
        const double newReq = requiredAreaWithFTLocal(b, b.ftUsed + static_cast<double>(netCount));
        const double oldOverflow = max(0.0, oldReq - currentArea);
        const double newOverflow = max(0.0, newReq - currentArea);
        return max(0.0, newOverflow - oldOverflow);
    }

    int softFTCapacityNetsLocal(const BlockInst& b) {
        if (b.spec.type != BlockType::SOFT) return 0;
        const double currentArea = max(0.0, b.rect.w * b.rect.h);
        if (currentArea <= EPS) return 0;
        int lo = 0;
        int hi = 1;
        while (hi < 2000000000 && requiredAreaWithFTLocal(b, static_cast<double>(hi)) <= currentArea + 1e-7) {
            lo = hi;
            hi = (hi > 1000000000) ? 2000000000 : hi * 2;
            if (hi == lo) break;
        }
        while (lo + 1 < hi) {
            const int mid = lo + (hi - lo) / 2;
            if (requiredAreaWithFTLocal(b, static_cast<double>(mid)) <= currentArea + 1e-7) lo = mid;
            else hi = mid;
        }
        return lo;
    }

    double softFTPenaltyLocal(const BlockInst& b, int netCount) {
        if (g_ftHardCapacityGuard && b.spec.type == BlockType::SOFT) {
            const double cap = static_cast<double>(softFTCapacityNetsLocal(b));
            if (b.ftUsed + static_cast<double>(netCount) > cap + 1e-7) {
                return numeric_limits<double>::infinity();
            }
        }
        const double incArea = incrementalFTAreaLocal(b, netCount);
        const double overflowAfter = estimatedFTOverflowAfterLocal(b, netCount);
        return g_softFtFixedPenalty
            + g_softFtPerNetPenalty * static_cast<double>(netCount)
            + FT_INCREMENTAL_AREA_WEIGHT * incArea
            + g_ftOverflowExtraWeight * overflowAfter;
    }

    // ----------------------------------------------------------------------------
    // Directional usage recomputation from final/current PATHs.
    // ----------------------------------------------------------------------------
    vector<DirUse> computeDirectionalChannelUseFromRoutes(const Design& design) {
        vector<DirUse> usage(design.channels.size());

        for (const auto& path : design.routes) {
            if (path.open) continue;
            const double nets = static_cast<double>(path.netCount);
            for (int i = 0; i + 1 < static_cast<int>(path.steps.size()); ++i) {
                const auto& a = path.steps[i];
                const auto& b = path.steps[i + 1];
                if (a.rectName != b.rectName) continue;
                if (!isChannelName(a.rectName)) continue;
                const int ci = channelIndexByName(design, a.rectName);
                if (ci < 0) continue;
                DirDelta d = deltaForInternalTraversal(a.edge, b.edge, nets);
                usage[ci].lr += d.lr;
                usage[ci].tb += d.tb;
            }
        }

        return usage;
    }

    struct DirectionalUseCache {
        const Design* design = nullptr;
        const RoutePath* routeData = nullptr;
        size_t routeCount = 0;
        size_t channelCount = 0;
        vector<DirUse> usage;
    };

    static DirectionalUseCache g_directionalUseCache;

    void clearDirectionalUseCache() {
        g_directionalUseCache = DirectionalUseCache{};
    }

    const vector<DirUse>& cachedDirectionalChannelUseFromRoutes(const Design& design) {
        const RoutePath* data = design.routes.empty() ? nullptr : design.routes.data();
        if (g_directionalUseCache.design == &design &&
            g_directionalUseCache.routeData == data &&
            g_directionalUseCache.routeCount == design.routes.size() &&
            g_directionalUseCache.channelCount == design.channels.size()) {
            return g_directionalUseCache.usage;
        }
        g_directionalUseCache.design = &design;
        g_directionalUseCache.routeData = data;
        g_directionalUseCache.routeCount = design.routes.size();
        g_directionalUseCache.channelCount = design.channels.size();
        g_directionalUseCache.usage = computeDirectionalChannelUseFromRoutes(design);
        return g_directionalUseCache.usage;
    }

    struct PathDirUse {
        map<int, DirUse> byChannel;
    };

    PathDirUse computeDirectionalUseForPath(const Design& design, const RoutePath& path) {
        PathDirUse out;
        if (path.open) return out;
        const double nets = static_cast<double>(path.netCount);
        for (int i = 0; i + 1 < static_cast<int>(path.steps.size()); ++i) {
            const auto& a = path.steps[i];
            const auto& b = path.steps[i + 1];
            if (a.rectName != b.rectName) continue;
            if (!isChannelName(a.rectName)) continue;
            const int ci = channelIndexByName(design, a.rectName);
            if (ci < 0) continue;
            DirDelta d = deltaForInternalTraversal(a.edge, b.edge, nets);
            out.byChannel[ci].lr += d.lr;
            out.byChannel[ci].tb += d.tb;
        }
        return out;
    }


    map<int, int> softTransitCountsForPath(const Design& design, const RoutePath& path) {
        map<int, int> out;
        if (path.open) return out;
        for (int i = 0; i + 1 < static_cast<int>(path.steps.size()); ++i) {
            const auto& a = path.steps[i];
            const auto& b = path.steps[i + 1];
            if (a.rectName != b.rectName) continue;
            if (isChannelName(a.rectName)) continue;
            if (a.rectName == path.srcBlock || a.rectName == path.dstBlock) continue;
            auto it = design.blockNameToIndex.find(a.rectName);
            if (it == design.blockNameToIndex.end()) continue;
            const int bi = it->second;
            if (bi < 0 || bi >= static_cast<int>(design.blocks.size())) continue;
            if (design.blocks[bi].spec.type == BlockType::SOFT) ++out[bi];
        }
        return out;
    }    // 將目前 design.routes 重新換算回 DataModel 的 legacy 欄位。
    //
    // 為什麼要重算？
    //   Router.hpp / DataModel 目前沒有正式的 channel.horizontalUsed / verticalUsed 欄位，
    //   只有舊版 scalar：usedNets、capacity、overflow。
    //   為了不改 header，本 baseline 每次接受一條 route 後，就從所有 routes 掃一遍，
    //   重新計算每個 channel 的 LR/TB 使用量，再把「主導瓶頸方向」寫回 legacy 欄位：
    //       usedNets  = max-util 那個方向的 used
    //       capacity  = max-util 那個方向的 cap
    //       overflow  = LR overflow + TB overflow
    //   這樣 Logger 舊欄位仍可用，但更詳細的方向 report 會另外印出 LR/TB。
    void recomputeRouterUsageFields(Design& design) {
        vector<DirUse> usage = computeDirectionalChannelUseFromRoutes(design);

        for (auto& ch : design.channels) {
            ch.usedNets = 0.0;
            ch.overflow = 0.0;
            // Keep capacity as dominant component later.  Do not overwrite geometry.
        }
        for (auto& b : design.blocks) {
            b.ftUsed = 0.0;
            b.ftOverflowArea = 0.0;
        }

        for (int i = 0; i < static_cast<int>(design.channels.size()); ++i) {
            Channel& ch = design.channels[i];
            const double cLR = capLR(ch);
            const double cTB = capTB(ch);
            const double uLR = usage[i].lr;
            const double uTB = usage[i].tb;
            const double ovLR = max(0.0, uLR - cLR);
            const double ovTB = max(0.0, uTB - cTB);
            const double utilLR = cLR > EPS ? uLR / cLR : (uLR > EPS ? numeric_limits<double>::infinity() : 0.0);
            const double utilTB = cTB > EPS ? uTB / cTB : (uTB > EPS ? numeric_limits<double>::infinity() : 0.0);

            // Legacy scalar fields point to the dominant bottleneck component.  This
            // keeps existing Logger useful while ch.overflow stores the correct sum.
            if (utilLR >= utilTB) {
                ch.usedNets = uLR;
                ch.capacity = cLR;
            }
            else {
                ch.usedNets = uTB;
                ch.capacity = cTB;
            }
            ch.overflow = ovLR + ovTB;
        }

        for (const auto& path : design.routes) {
            if (path.open) continue;
            const double nets = static_cast<double>(path.netCount);
            const map<int, int> softCounts = softTransitCountsForPath(design, path);
            for (const auto& kv : softCounts) {
                if (kv.first < 0 || kv.first >= static_cast<int>(design.blocks.size())) continue;
                design.blocks[kv.first].ftUsed += nets * static_cast<double>(kv.second);
            }
        }

        for (auto& b : design.blocks) {
            if (b.spec.type != BlockType::SOFT) continue;
            const double currentArea = b.rect.w * b.rect.h;
            b.ftOverflowArea = max(0.0, requiredAreaWithFTLocal(b, b.ftUsed) - currentArea);
        }
    }

    double totalChannelOverflowNow(const Design& design) {
        double total = 0.0;
        for (const auto& ch : design.channels) total += ch.overflow;
        return total;
    }

    double totalRouteWireLengthNow(const Design& design) {
        double total = 0.0;
        for (const auto& p : design.routes) total += p.wireLength;
        return total;
    }

    // ----------------------------------------------------------------------------
    // Directional channel transition penalty.
    // ----------------------------------------------------------------------------
    struct DirectionalPenaltyInfo {
        bool feasible = true;
        double penalty = 0.0;
        double overflowIncrement = 0.0;
        double maxProjectedUtil = 0.0;
        string maxUtilComponent;
        double minComponentCapacity = numeric_limits<double>::infinity();
        int narrowComponentCount = 0;
    };

    DirectionalPenaltyInfo directionalChannelPenalty(
        const Channel& ch,
        int chIndex,
        int inEdge,
        int outEdge,
        int netCount,
        const vector<DirUse>& currentUse
    ) {
        DirectionalPenaltyInfo info;

        if (!validEdgeLocal(inEdge) || !validEdgeLocal(outEdge) || inEdge == outEdge) {
            info.feasible = false;
            info.penalty = numeric_limits<double>::infinity();
            return info;
        }

        const double nets = static_cast<double>(netCount);
        const DirDelta d = deltaForInternalTraversal(inEdge, outEdge, nets);
        const double uLR = currentUse[chIndex].lr;
        const double uTB = currentUse[chIndex].tb;
        const double cLR = capLR(ch);
        const double cTB = capTB(ch);


        auto evalComponent = [&](DirComponent comp, double used, double cap, double delta) {
            if (delta <= EPS) return;

            if (g_forbiddenResources.count(ResourceKey{ chIndex, comp }) > 0) {
                info.feasible = false;
                info.penalty = numeric_limits<double>::infinity();
                return;
            }

            info.minComponentCapacity = min(info.minComponentCapacity, cap);
            if (cap < NARROW_COMPONENT_CAP) ++info.narrowComponentCount;

            if (cap <= EPS) {
                info.feasible = false;
                info.penalty = numeric_limits<double>::infinity();
                return;
            }

            const double capSafe = max(1.0, cap);
            const double projected = used + delta;
            const double util = projected / capSafe;
            if (util > info.maxProjectedUtil) {
                info.maxProjectedUtil = util;
                info.maxUtilComponent = ch.name + " " + compName(comp);
            }

            if (!g_allowChannelOverflow) {
                const double effectiveCap = max(0.0, cap * g_capacityScale);
                if (projected > effectiveCap + CHANNEL_CAPACITY_EPS) {
                    info.feasible = false;
                    info.penalty = numeric_limits<double>::infinity();
                    return;
                }
            }

            if (g_splitRouting) {
                const double splitHardLimit = min(1.0, max(0.0, g_capacityScale));
                if (util > splitHardLimit + CHANNEL_CAPACITY_EPS) {
                    info.feasible = false;
                    info.penalty = numeric_limits<double>::infinity();
                    return;
                }
            }

            if (g_wirePolishMode) return;

            double p = 0.0;
            p += CHANNEL_CONGESTION_WEIGHT * util * util * nets;
            if (util > 0.60) {
                const double x = util - 0.60;
                p += CHANNEL_NEAR_FULL_WEIGHT * x * x * nets;
            }
            if (util > 0.85) {
                const double x = util - 0.85;
                p += CHANNEL_NEAR_FULL_WEIGHT * 8.0 * x * x * x * x * nets;
            }
            if (cap < NARROW_COMPONENT_CAP) {
                const double shortage = (NARROW_COMPONENT_CAP - cap) / NARROW_COMPONENT_CAP;
                const double demandRatio = delta / capSafe;
                p += CHANNEL_NARROW_WEIGHT * shortage * shortage * (1.0 + demandRatio * demandRatio) * nets;
            }
            if (chIndex >= 0 && chIndex < static_cast<int>(g_channelCriticality.size())) {
                const double crit = g_channelCriticality[chIndex];
                p += CHANNEL_CRITICALITY_WEIGHT * crit * util * util * nets;
            }
            if (g_congestionRerouteMode && chIndex >= 0 && chIndex < static_cast<int>(g_hotChannelBias.size())) {
                const double hot = comp == DirComponent::LR ? g_hotChannelBias[chIndex][0] : g_hotChannelBias[chIndex][1];
                if (hot > EPS) p += g_congestionBiasWeight * hot * delta;
            }
            if (g_congestionRerouteMode && chIndex >= 0 && chIndex < static_cast<int>(g_negotiatedChannelHistory.size())) {
                const double hist = comp == DirComponent::LR ? g_negotiatedChannelHistory[chIndex][0] : g_negotiatedChannelHistory[chIndex][1];
                if (hist > EPS) p += g_congestionBiasWeight * NEGOTIATED_HISTORY_WEIGHT * hist * delta;
            }
            if (g_allowChannelOverflow) {
                const double beforeOv = max(0.0, used - cap);
                const double afterOv = max(0.0, projected - cap);
                const double incOv = max(0.0, afterOv - beforeOv);
                info.overflowIncrement += incOv;
                if (incOv > EPS) {
                    p += CHANNEL_OVERFLOW_LINEAR_WEIGHT * incOv + CHANNEL_OVERFLOW_QUADRATIC_WEIGHT * incOv * incOv;
                }
            }
            info.penalty += p;
            };

        evalComponent(DirComponent::LR, uLR, cLR, d.lr);
        if (!info.feasible) return info;
        evalComponent(DirComponent::TB, uTB, cTB, d.tb);
        return info;
    }

    // ----------------------------------------------------------------------------
    // Path analysis helpers.
    // ----------------------------------------------------------------------------
    set<string> touchedIntermediateSoftBlocks(const Design& design, const RoutePath& path) {
        set<string> out;
        const map<int, int> softCounts = softTransitCountsForPath(design, path);
        for (const auto& kv : softCounts) {
            if (kv.first >= 0 && kv.first < static_cast<int>(design.blocks.size())) {
                out.insert(design.blocks[kv.first].spec.name);
            }
        }
        return out;
    }

    bool pathUsesSoftFT(const Design& design, const RoutePath& path) {
        return !softTransitCountsForPath(design, path).empty();
    }

    int pathSoftFTCount(const Design& design, const RoutePath& path) {
        int total = 0;
        const map<int, int> softCounts = softTransitCountsForPath(design, path);
        for (const auto& kv : softCounts) total += kv.second;
        return total;
    }

    
    bool routeHasIllegalIntermediateBlock(const Design& design, const RoutePath& path) {
        if (path.open) return true;
        if (path.steps.size() < 2) return true;
        if (path.steps.front().rectName != path.srcBlock) return true;
        if (path.steps.back().rectName != path.dstBlock) return true;

        for (int i = 0; i + 1 < static_cast<int>(path.steps.size()); ++i) {
            const auto& in = path.steps[i];
            const auto& out = path.steps[i + 1];
            if (in.rectName != out.rectName) continue;
            if (isChannelName(in.rectName)) continue;
            if (in.rectName == path.srcBlock || in.rectName == path.dstBlock) return true;
            if (!isSoftBlockName(design, in.rectName)) return true;
        }
        return false;
    }
    struct RouteMetrics {
        bool open = false;
        bool usesSoftFT = false;
        int softFTBlockCount = 0;
        double wireLength = 0.0;
        double maxProjectedChannelUtil = 0.0;
        string maxUtilComponent;
        double channelOverflowIncrement = 0.0;
        double ftIncrementalArea = 0.0;
        double ftOverflowIncrement = 0.0;
        double ftOverflowAfterTouched = 0.0;
        double minComponentCapacity = numeric_limits<double>::infinity();
        int narrowComponentCount = 0;
        map<pair<int, DirComponent>, double> componentDelta;
    };

    RouteMetrics analyzeRoute(const Design& design, const RoutePath& path) {
        RouteMetrics m;
        m.open = path.open;
        m.wireLength = path.wireLength;
        if (!m.open && routeHasIllegalIntermediateBlock(design, path)) m.open = true;
        if (m.open) return m;

        const vector<DirUse>& current = cachedDirectionalChannelUseFromRoutes(design);
        const PathDirUse pd = computeDirectionalUseForPath(design, path);

        for (const auto& kv : pd.byChannel) {
            const int ci = kv.first;
            if (ci < 0 || ci >= static_cast<int>(design.channels.size())) continue;
            const Channel& ch = design.channels[ci];
            const double cLR = capLR(ch);
            const double cTB = capTB(ch);
            const double beforeLR = current[ci].lr;
            const double beforeTB = current[ci].tb;
            const double afterLR = beforeLR + kv.second.lr;
            const double afterTB = beforeTB + kv.second.tb;
            const double ovBefore = max(0.0, beforeLR - cLR) + max(0.0, beforeTB - cTB);
            const double ovAfter = max(0.0, afterLR - cLR) + max(0.0, afterTB - cTB);
            m.channelOverflowIncrement += max(0.0, ovAfter - ovBefore);

            if (kv.second.lr > EPS) {
                m.minComponentCapacity = min(m.minComponentCapacity, cLR);
                if (cLR < NARROW_COMPONENT_CAP) ++m.narrowComponentCount;
                const double util = cLR > EPS ? afterLR / cLR : numeric_limits<double>::infinity();
                if (util > m.maxProjectedChannelUtil) {
                    m.maxProjectedChannelUtil = util;
                    m.maxUtilComponent = ch.name + " " + compName(DirComponent::LR);
                }
                m.componentDelta[{ci, DirComponent::LR}] += kv.second.lr;
            }
            if (kv.second.tb > EPS) {
                m.minComponentCapacity = min(m.minComponentCapacity, cTB);
                if (cTB < NARROW_COMPONENT_CAP) ++m.narrowComponentCount;
                const double util = cTB > EPS ? afterTB / cTB : numeric_limits<double>::infinity();
                if (util > m.maxProjectedChannelUtil) {
                    m.maxProjectedChannelUtil = util;
                    m.maxUtilComponent = ch.name + " " + compName(DirComponent::TB);
                }
                m.componentDelta[{ci, DirComponent::TB}] += kv.second.tb;
            }
        }

        const map<int, int> softCounts = softTransitCountsForPath(design, path);
        m.usesSoftFT = !softCounts.empty();
        for (const auto& kv : softCounts) {
            if (kv.first < 0 || kv.first >= static_cast<int>(design.blocks.size())) continue;
            const BlockInst& b = design.blocks[kv.first];
            const int addNets = path.netCount * kv.second;
            m.softFTBlockCount += kv.second;
            m.ftIncrementalArea += incrementalFTAreaLocal(b, addNets);
            m.ftOverflowIncrement += incrementalFTOverflowLocal(b, addNets);
            m.ftOverflowAfterTouched += estimatedFTOverflowAfterLocal(b, addNets);
        }

        return m;
    }

    double totalFTOverflowNow(const Design& design) {
        double total = 0.0;
        for (const auto& b : design.blocks) total += max(0.0, b.ftOverflowArea);
        return total;
    }

    int openRouteCountNow(const Design& design) {
        int cnt = 0;
        for (const auto& p : design.routes) if (p.open) ++cnt;
        return cnt;
    }

    struct RouterScore {
        int open = 0;
        double channelOverflow = 0.0;
        double ftOverflow = 0.0;
        double wireLength = 0.0;
    };

    struct CongestionRiskScore {
        double maxUtil = 0.0;
        double riskSum = 0.0;
    };

    RouterScore scoreRouterSolution(const Design& design) {
        return RouterScore{ openRouteCountNow(design), totalChannelOverflowNow(design), totalFTOverflowNow(design), totalRouteWireLengthNow(design) };
    }

    CongestionRiskScore scoreCongestionRisk(const Design& design) {
        CongestionRiskScore s;
        vector<DirUse> usage = computeDirectionalChannelUseFromRoutes(design);
        for (int i = 0; i < static_cast<int>(design.channels.size()); ++i) {
            const Channel& ch = design.channels[i];
            const double cLR = max(1.0, capLR(ch));
            const double cTB = max(1.0, capTB(ch));
            const double utilLR = usage[i].lr / cLR;
            const double utilTB = usage[i].tb / cTB;
            s.maxUtil = max(s.maxUtil, max(utilLR, utilTB));
            s.riskSum += negotiatedUtilRisk(utilLR) * cLR;
            s.riskSum += negotiatedUtilRisk(utilTB) * cTB;
        }
        return s;
    }

    bool legalCongestionRiskImproved(const RouterScore& baseScore, const CongestionRiskScore& baseRisk,
        const RouterScore& trialScore, const CongestionRiskScore& trialRisk) {
        (void)baseRisk;
        (void)trialRisk;
        return trialScore.open == 0 &&
            trialScore.channelOverflow <= EPS &&
            trialScore.ftOverflow <= EPS &&
            trialScore.wireLength + 1.0e-6 < baseScore.wireLength;
    }

    void updateNegotiatedChannelHistory(const Design& design, bool includeLegalRisk) {
        vector<DirUse> usage = computeDirectionalChannelUseFromRoutes(design);
        if (g_negotiatedChannelHistory.size() != design.channels.size()) {
            g_negotiatedChannelHistory.assign(design.channels.size(), { 0.0, 0.0 });
        }
        for (int i = 0; i < static_cast<int>(design.channels.size()); ++i) {
            const Channel& ch = design.channels[i];
            const double cLR = max(1.0, capLR(ch));
            const double cTB = max(1.0, capTB(ch));
            const double utilLR = usage[i].lr / cLR;
            const double utilTB = usage[i].tb / cTB;
            double pressureLR = max(0.0, usage[i].lr - cLR) / cLR;
            double pressureTB = max(0.0, usage[i].tb - cTB) / cTB;
            if (includeLegalRisk) {
                pressureLR += negotiatedUtilRisk(utilLR);
                pressureTB += negotiatedUtilRisk(utilTB);
            }
            g_negotiatedChannelHistory[i][0] = NEGOTIATED_HISTORY_DECAY * g_negotiatedChannelHistory[i][0]
                + NEGOTIATED_HISTORY_GAIN * pressureLR;
            g_negotiatedChannelHistory[i][1] = NEGOTIATED_HISTORY_DECAY * g_negotiatedChannelHistory[i][1]
                + NEGOTIATED_HISTORY_GAIN * pressureTB;
        }
    }

    bool betterRouterScore(const RouterScore& a, const RouterScore& b) {
        if (a.open != b.open) return a.open < b.open;
        const double ap = a.channelOverflow + a.ftOverflow;
        const double bp = b.channelOverflow + b.ftOverflow;
        if (fabs(ap - bp) > max(1.0, 0.002 * max(1.0, min(ap, bp)))) return ap < bp;
        if (fabs(a.channelOverflow - b.channelOverflow) > 1.0) return a.channelOverflow < b.channelOverflow;
        return a.wireLength < b.wireLength - 1.0;
    }

    struct RipupKey {
        int src = -1;
        int dst = -1;
        bool operator<(const RipupKey& o) const {
            if (src != o.src) return src < o.src;
            return dst < o.dst;
        }
    };

    struct RipupGroup {
        Connection conn;
        double score = 0.0;
        int routeCount = 0;
        double channelScore = 0.0;
        double ftScore = 0.0;
    };

    vector<array<double, 2>> buildHotChannelBias(const Design& design, bool includeLegalRisk) {
        vector<array<double, 2>> bias(design.channels.size());
        vector<DirUse> usage = computeDirectionalChannelUseFromRoutes(design);
        for (int i = 0; i < static_cast<int>(design.channels.size()); ++i) {
            const Channel& ch = design.channels[i];
            const double cLR = max(1.0, capLR(ch));
            const double cTB = max(1.0, capTB(ch));
            const double utilLR = usage[i].lr / cLR;
            const double utilTB = usage[i].tb / cTB;
            const double overflowLR = max(0.0, usage[i].lr - cLR) / cLR;
            const double overflowTB = max(0.0, usage[i].tb - cTB) / cTB;
            bias[i][0] = overflowLR;
            bias[i][1] = overflowTB;
            if (includeLegalRisk) {
                bias[i][0] = max(bias[i][0], negotiatedUtilRisk(utilLR));
                bias[i][1] = max(bias[i][1], negotiatedUtilRisk(utilTB));
            }
            if (i < static_cast<int>(g_negotiatedChannelHistory.size())) {
                bias[i][0] = max(bias[i][0], g_negotiatedChannelHistory[i][0]);
                bias[i][1] = max(bias[i][1], g_negotiatedChannelHistory[i][1]);
            }
        }
        return bias;
    }

    vector<double> buildHotSoftBias(const Design& design) {
        vector<double> bias(design.blocks.size(), 0.0);
        for (int i = 0; i < static_cast<int>(design.blocks.size()); ++i) {
            const BlockInst& b = design.blocks[i];
            if (b.spec.type != BlockType::SOFT) continue;
            const double area = max(1.0, b.rect.w * b.rect.h);
            bias[i] = max(0.0, b.ftOverflowArea) / area;
        }
        return bias;
    }

    vector<RipupGroup> selectRipupGroups(const Design& design, int maxGroups, bool includeLegalRisk) {
        vector<array<double, 2>> hotCh = buildHotChannelBias(design, includeLegalRisk);
        vector<double> hotSoft = buildHotSoftBias(design);
        map<RipupKey, RipupGroup> groups;

        auto addConn = [&](const RoutePath& p, double channelScore, double ftScore) {
            auto sit = design.blockNameToIndex.find(p.srcBlock);
            auto dit = design.blockNameToIndex.find(p.dstBlock);
            if (sit == design.blockNameToIndex.end() || dit == design.blockNameToIndex.end()) return;
            RipupKey key{ sit->second, dit->second };
            RipupGroup& g = groups[key];
            g.conn.src = key.src;
            g.conn.dst = key.dst;
            g.conn.netCount += max(0, p.netCount);
            g.score += channelScore + ftScore;
            g.channelScore += channelScore;
            g.ftScore += ftScore;
            ++g.routeCount;
        };

        for (const auto& p : design.routes) {
            if (p.open) { addConn(p, 1.0e12, 0.0); continue; }
            const PathDirUse pd = computeDirectionalUseForPath(design, p);
            double channelScore = 0.0;
            for (const auto& kv : pd.byChannel) {
                const int ci = kv.first;
                if (ci < 0 || ci >= static_cast<int>(hotCh.size())) continue;
                channelScore += kv.second.lr * hotCh[ci][0];
                channelScore += kv.second.tb * hotCh[ci][1];
            }
            double ftScore = 0.0;
            const map<int, int> softCounts = softTransitCountsForPath(design, p);
            for (const auto& kv : softCounts) {
                const int bi = kv.first;
                if (bi >= 0 && bi < static_cast<int>(hotSoft.size())) {
                    ftScore += static_cast<double>(max(0, p.netCount) * kv.second) * hotSoft[bi];
                }
            }
            if (channelScore + ftScore > RIPUP_MIN_GROUP_SCORE) addConn(p, channelScore, ftScore);
        }

        vector<RipupGroup> out;
        for (auto& kv : groups) if (kv.second.score > RIPUP_MIN_GROUP_SCORE && kv.second.conn.netCount > 0) out.push_back(kv.second);
        sort(out.begin(), out.end(), [](const RipupGroup& a, const RipupGroup& b) {
            if (fabs(a.score - b.score) > 1e-9) return a.score > b.score;
            if (a.conn.netCount != b.conn.netCount) return a.conn.netCount > b.conn.netCount;
            if (a.conn.src != b.conn.src) return a.conn.src < b.conn.src;
            return a.conn.dst < b.conn.dst;
            });
        if (static_cast<int>(out.size()) > maxGroups) out.resize(maxGroups);
        return out;
    }

    bool routeMatchesKey(const Design& design, const RoutePath& p, const set<RipupKey>& keys) {
        auto sit = design.blockNameToIndex.find(p.srcBlock);
        auto dit = design.blockNameToIndex.find(p.dstBlock);
        if (sit == design.blockNameToIndex.end() || dit == design.blockNameToIndex.end()) return false;
        return keys.count(RipupKey{ sit->second, dit->second }) > 0;
    }
    void printPathLine(const RoutePath& p) {
        cerr << "PATH " << p.netCount << " ";
        for (const auto& st : p.steps) cerr << st.rectName << " " << st.edge << " ";
        cerr << "\n";
    }

    struct CorridorRunDiag {
        int id = -1;
        DirComponent dir = DirComponent::LR;
        vector<int> channelIds;
        double guideLength = 0.0;
        double bottleneckResidual = numeric_limits<double>::infinity();
        double maxUtil = 0.0;
        double totalUsed = 0.0;
    };

    struct DetourRouteDiag {
        const RoutePath* path = nullptr;
        string src;
        string dst;
        double freeGuideWL = 0.0;
        double finalGuideWL = 0.0;
        double weightedExcess = 0.0;
        double ratio = 0.0;
        int hops = 0;
        int turns = 0;
        int softFTNets = 0;
        double maxRunUtil = 0.0;
        string hotComponent;
        vector<int> runIds;
    };

    vector<int> legalEdgesForReport(const BlockSpec& spec) {
        if (!spec.portEdges.empty()) return spec.portEdges;
        return { 1, 2, 3, 4 };
    }

    pair<double, double> edgeAnchorForReport(const Rect& r, int edge, double t) {
        t = max(0.0, min(1.0, t));
        if (edge == 1) return { r.x, r.y + r.h * t };
        if (edge == 3) return { rectRight(r), r.y + r.h * t };
        if (edge == 2) return { r.x + r.w * t, rectTop(r) };
        if (edge == 4) return { r.x + r.w * t, r.y };
        return { rectCx(r), rectCy(r) };
    }

    double freeGuideWLForReport(const Design& design, const RoutePath& path) {
        string srcName = path.srcBlock;
        string dstName = path.dstBlock;
        if (srcName.empty() && !path.steps.empty()) srcName = path.steps.front().rectName;
        if (dstName.empty() && !path.steps.empty()) dstName = path.steps.back().rectName;
        auto sit = design.blockNameToIndex.find(srcName);
        auto dit = design.blockNameToIndex.find(dstName);
        if (sit == design.blockNameToIndex.end() || dit == design.blockNameToIndex.end()) return max(0.0, path.wireLength);
        const BlockInst& src = design.blocks[sit->second];
        const BlockInst& dst = design.blocks[dit->second];
        const vector<int> srcEdges = legalEdgesForReport(src.spec);
        const vector<int> dstEdges = legalEdgesForReport(dst.spec);
        const array<double, 3> taps = { 0.25, 0.50, 0.75 };

        double best = numeric_limits<double>::infinity();
        for (int se : srcEdges) {
            if (!validEdgeLocal(se)) continue;
            for (int de : dstEdges) {
                if (!validEdgeLocal(de)) continue;
                for (double st : taps) {
                    const auto sp = edgeAnchorForReport(src.rect, se, st);
                    for (double dt : taps) {
                        const auto dp = edgeAnchorForReport(dst.rect, de, dt);
                        best = min(best, manhattan(sp.first, sp.second, dp.first, dp.second));
                    }
                }
            }
        }
        if (!std::isfinite(best)) best = manhattan(rectCx(src.rect), rectCy(src.rect), rectCx(dst.rect), rectCy(dst.rect));
        return best * static_cast<double>(max(0, path.netCount));
    }

    vector<ResourceKey> routeResourceSequenceForReport(const Design& design, const RoutePath& path) {
        vector<ResourceKey> seq;
        if (path.open) return seq;
        for (int i = 0; i + 1 < static_cast<int>(path.steps.size()); ++i) {
            const auto& a = path.steps[i];
            const auto& b = path.steps[i + 1];
            if (a.rectName != b.rectName || !isChannelName(a.rectName)) continue;
            const int ci = channelIndexByName(design, a.rectName);
            if (ci < 0) continue;
            const DirDelta d = deltaForInternalTraversal(a.edge, b.edge, 1.0);
            if (d.lr > EPS) seq.push_back({ ci, DirComponent::LR });
            if (d.tb > EPS) seq.push_back({ ci, DirComponent::TB });
        }
        return seq;
    }

    string compactChannelListForReport(const Design& design, const vector<int>& channelIds, int limit = 12) {
        ostringstream oss;
        for (int i = 0; i < static_cast<int>(channelIds.size()) && i < limit; ++i) {
            if (i) oss << ",";
            const int ci = channelIds[i];
            if (ci >= 0 && ci < static_cast<int>(design.channels.size())) oss << design.channels[ci].name;
        }
        if (static_cast<int>(channelIds.size()) > limit) oss << ",...";
        return oss.str();
    }

    string runIdListForReport(const vector<int>& runIds, int limit = 10) {
        ostringstream oss;
        for (int i = 0; i < static_cast<int>(runIds.size()) && i < limit; ++i) {
            if (i) oss << ",";
            oss << runIds[i];
        }
        if (static_cast<int>(runIds.size()) > limit) oss << ",...";
        return oss.str();
    }

    struct ReportDsu {
        vector<int> parent;
        vector<int> rank;
        explicit ReportDsu(int n = 0) : parent(n), rank(n, 0) {
            for (int i = 0; i < n; ++i) parent[i] = i;
        }
        int find(int x) {
            while (parent[x] != x) {
                parent[x] = parent[parent[x]];
                x = parent[x];
            }
            return x;
        }
        void unite(int a, int b) {
            a = find(a); b = find(b);
            if (a == b) return;
            if (rank[a] < rank[b]) swap(a, b);
            parent[b] = a;
            if (rank[a] == rank[b]) ++rank[a];
        }
    };

    vector<CorridorRunDiag> buildCorridorRunsForReport(const Design& design, const vector<DirUse>& usage,
        map<ResourceKey, int>& keyToRunId) {
        vector<ResourceKey> keys;
        map<ResourceKey, int> keyToNode;
        for (int ci = 0; ci < static_cast<int>(design.channels.size()); ++ci) {
            if (usage[ci].lr > EPS) {
                ResourceKey k{ ci, DirComponent::LR };
                keyToNode[k] = static_cast<int>(keys.size());
                keys.push_back(k);
            }
            if (usage[ci].tb > EPS) {
                ResourceKey k{ ci, DirComponent::TB };
                keyToNode[k] = static_cast<int>(keys.size());
                keys.push_back(k);
            }
        }

        ReportDsu dsu(static_cast<int>(keys.size()));
        for (const auto& path : design.routes) {
            const vector<ResourceKey> seq = routeResourceSequenceForReport(design, path);
            int prevNode = -1;
            ResourceKey prevKey;
            bool havePrev = false;
            for (const ResourceKey& k : seq) {
                auto it = keyToNode.find(k);
                if (it == keyToNode.end()) continue;
                const int node = it->second;
                if (havePrev && prevKey.dir == k.dir && prevNode >= 0) dsu.unite(prevNode, node);
                prevNode = node;
                prevKey = k;
                havePrev = true;
            }
        }

        map<int, vector<ResourceKey>> grouped;
        for (int i = 0; i < static_cast<int>(keys.size()); ++i) grouped[dsu.find(i)].push_back(keys[i]);

        vector<CorridorRunDiag> runs;
        for (auto& kv : grouped) {
            if (kv.second.empty()) continue;
            CorridorRunDiag run;
            run.dir = kv.second.front().dir;
            set<int> uniqueChannels;
            for (const ResourceKey& k : kv.second) {
                const int ci = k.channelIndex;
                if (ci < 0 || ci >= static_cast<int>(design.channels.size())) continue;
                uniqueChannels.insert(ci);
                const Channel& ch = design.channels[ci];
                const double used = k.dir == DirComponent::LR ? usage[ci].lr : usage[ci].tb;
                const double cap = k.dir == DirComponent::LR ? capLR(ch) : capTB(ch);
                const double util = cap > EPS ? used / cap : (used > EPS ? numeric_limits<double>::infinity() : 0.0);
                run.totalUsed += used;
                run.maxUtil = max(run.maxUtil, util);
                run.bottleneckResidual = min(run.bottleneckResidual, cap - used);
                run.guideLength += k.dir == DirComponent::LR ? max(0.0, ch.rect.w) : max(0.0, ch.rect.h);
            }
            run.channelIds.assign(uniqueChannels.begin(), uniqueChannels.end());
            if (!std::isfinite(run.bottleneckResidual)) run.bottleneckResidual = 0.0;
            runs.push_back(std::move(run));
        }

        sort(runs.begin(), runs.end(), [](const CorridorRunDiag& a, const CorridorRunDiag& b) {
            if (fabs(a.maxUtil - b.maxUtil) > 1.0e-9) return a.maxUtil > b.maxUtil;
            if (fabs(a.totalUsed - b.totalUsed) > 1.0e-9) return a.totalUsed > b.totalUsed;
            if (a.dir != b.dir) return static_cast<int>(a.dir) < static_cast<int>(b.dir);
            return a.channelIds < b.channelIds;
            });
        for (int i = 0; i < static_cast<int>(runs.size()); ++i) {
            runs[i].id = i;
            for (int ci : runs[i].channelIds) keyToRunId[{ ci, runs[i].dir }] = i;
        }
        return runs;
    }

    DetourRouteDiag buildDetourRouteDiagForReport(const Design& design, const RoutePath& path,
        const vector<DirUse>& usage, const map<ResourceKey, int>& keyToRunId) {
        DetourRouteDiag d;
        d.path = &path;
        d.src = !path.srcBlock.empty() ? path.srcBlock : (!path.steps.empty() ? path.steps.front().rectName : "NA");
        d.dst = !path.dstBlock.empty() ? path.dstBlock : (!path.steps.empty() ? path.steps.back().rectName : "NA");
        d.freeGuideWL = freeGuideWLForReport(design, path);
        d.finalGuideWL = max(0.0, path.wireLength);
        d.weightedExcess = max(0.0, d.finalGuideWL - d.freeGuideWL);
        d.ratio = d.freeGuideWL > EPS ? d.finalGuideWL / d.freeGuideWL : 0.0;

        set<int> uniqueRunIds;
        for (int i = 0; i + 1 < static_cast<int>(path.steps.size()); ++i) {
            const auto& a = path.steps[i];
            const auto& b = path.steps[i + 1];
            if (a.rectName != b.rectName || !isChannelName(a.rectName)) continue;
            ++d.hops;
            const int ci = channelIndexByName(design, a.rectName);
            if (ci < 0 || ci >= static_cast<int>(design.channels.size())) continue;
            const DirDelta delta = deltaForInternalTraversal(a.edge, b.edge, 1.0);
            if (delta.lr > EPS && delta.tb > EPS) ++d.turns;
            auto considerComp = [&](DirComponent comp, double active) {
                if (active <= EPS) return;
                const Channel& ch = design.channels[ci];
                const double used = comp == DirComponent::LR ? usage[ci].lr : usage[ci].tb;
                const double cap = comp == DirComponent::LR ? capLR(ch) : capTB(ch);
                const double util = cap > EPS ? used / cap : (used > EPS ? numeric_limits<double>::infinity() : 0.0);
                if (util > d.maxRunUtil) {
                    d.maxRunUtil = util;
                    d.hotComponent = ch.name + "." + shortCompName(comp);
                }
                auto rit = keyToRunId.find({ ci, comp });
                if (rit != keyToRunId.end()) uniqueRunIds.insert(rit->second);
            };
            considerComp(DirComponent::LR, delta.lr);
            considerComp(DirComponent::TB, delta.tb);
        }
        const map<int, int> softCounts = softTransitCountsForPath(design, path);
        for (const auto& kv : softCounts) d.softFTNets += max(0, path.netCount) * max(0, kv.second);
        d.runIds.assign(uniqueRunIds.begin(), uniqueRunIds.end());
        if (d.hotComponent.empty()) d.hotComponent = "NA";
        return d;
    }

    void printDetourReportImpl(const Design& design) {
        if (!ROUTER_DETOUR_REPORT_ENABLE) return;
        cerr << fixed << setprecision(3);
        const vector<DirUse> usage = computeDirectionalChannelUseFromRoutes(design);
        map<ResourceKey, int> keyToRunId;
        vector<CorridorRunDiag> runs = buildCorridorRunsForReport(design, usage, keyToRunId);

        vector<DetourRouteDiag> detours;
        detours.reserve(design.routes.size());
        double freeTotal = 0.0;
        double finalTotal = 0.0;
        int openCount = 0;
        for (const auto& path : design.routes) {
            if (path.open) { ++openCount; continue; }
            DetourRouteDiag d = buildDetourRouteDiagForReport(design, path, usage, keyToRunId);
            freeTotal += d.freeGuideWL;
            finalTotal += d.finalGuideWL;
            detours.push_back(std::move(d));
        }
        sort(detours.begin(), detours.end(), [](const DetourRouteDiag& a, const DetourRouteDiag& b) {
            if (fabs(a.weightedExcess - b.weightedExcess) > 1.0) return a.weightedExcess > b.weightedExcess;
            if (fabs(a.finalGuideWL - b.finalGuideWL) > 1.0) return a.finalGuideWL > b.finalGuideWL;
            return a.src + a.dst < b.src + b.dst;
            });

        cerr << "[RouterPhase1][Bound] routes=" << design.routes.size()
            << " open=" << openCount
            << " freeGuideWL=" << freeTotal
            << " finalGuideWL=" << finalTotal
            << " detourWL=" << max(0.0, finalTotal - freeTotal)
            << " ratio=" << (freeTotal > EPS ? finalTotal / freeTotal : 0.0)
            << " corridorRuns=" << runs.size() << "\n";

        const int runLimit = min(ROUTER_CORRIDOR_TOP_RUNS, static_cast<int>(runs.size()));
        for (int i = 0; i < runLimit; ++i) {
            const CorridorRunDiag& r = runs[i];
            cerr << "[RouterPhase1][CorridorRun] rank=" << (i + 1)
                << " id=" << r.id
                << " dir=" << shortCompName(r.dir)
                << " channels=" << compactChannelListForReport(design, r.channelIds)
                << " guideLength=" << r.guideLength
                << " totalUsed=" << r.totalUsed
                << " bottleneckResidual=" << r.bottleneckResidual
                << " maxUtil=" << r.maxUtil << "\n";
        }

        const int detourLimit = min(ROUTER_DETOUR_TOP_ROUTES, static_cast<int>(detours.size()));
        for (int i = 0; i < detourLimit; ++i) {
            const DetourRouteDiag& d = detours[i];
            cerr << "[RouterPhase1][DetourCertificate] rank=" << (i + 1)
                << " conn=" << d.src << "->" << d.dst
                << " nets=" << (d.path ? d.path->netCount : 0)
                << " freeGuideWL=" << d.freeGuideWL
                << " finalGuideWL=" << d.finalGuideWL
                << " weightedExcess=" << d.weightedExcess
                << " ratio=" << d.ratio
                << " hops=" << d.hops
                << " turns=" << d.turns
                << " maxRunUtil=" << d.maxRunUtil
                << " hot=" << d.hotComponent
                << " runIds=" << runIdListForReport(d.runIds)
                << " softFTNets=" << d.softFTNets << "\n";
        }
    }
    // ----------------------------------------------------------------------------
    // Criticality estimate for shared bottleneck awareness.
    // ----------------------------------------------------------------------------
    bool rectIntersectsLoose(const Rect& a, double x1, double y1, double x2, double y2) {
        return !(rectRight(a) < x1 || a.x > x2 || rectTop(a) < y1 || a.y > y2);
    }

    void buildChannelCriticalityLocal(const Design& design) {
        g_channelCriticality.assign(design.channels.size(), 0.0);
        double maxDemand = 0.0;
        for (int ci = 0; ci < static_cast<int>(design.channels.size()); ++ci) {
            const Rect& cr = design.channels[ci].rect;
            double demand = 0.0;
            for (const auto& conn : design.connections) {
                if (conn.src < 0 || conn.src >= static_cast<int>(design.blocks.size()) ||
                    conn.dst < 0 || conn.dst >= static_cast<int>(design.blocks.size())) continue;
                const Rect& sr = design.blocks[conn.src].rect;
                const Rect& dr = design.blocks[conn.dst].rect;
                double x1 = min(rectCx(sr), rectCx(dr));
                double x2 = max(rectCx(sr), rectCx(dr));
                double y1 = min(rectCy(sr), rectCy(dr));
                double y2 = max(rectCy(sr), rectCy(dr));
                const double pad = 150.0;
                x1 -= pad; x2 += pad; y1 -= pad; y2 += pad;
                if (rectIntersectsLoose(cr, x1, y1, x2, y2)) demand += static_cast<double>(conn.netCount);
            }
            g_channelCriticality[ci] = demand;
            maxDemand = max(maxDemand, demand);
        }
        if (maxDemand > EPS) for (double& v : g_channelCriticality) v /= maxDemand;
    }

    // 最終 router-side diagnosis。
    //
    // 這裡印出的不是官方 evaluator 的最終分數，而是 Router 自己根據目前 PATH
    // 重新計算出的 directional usage summary。重點是幫你 debug：
    //   1. 哪些 soft block 被 FT 穿越，估計需要多少面積擴增。
    //   2. 哪些 channel 的 LR/TB 方向 usage / capacity / overflow 最高。
    //   3. maxUtilComp 會明確指出瓶頸是水平 LR 還是垂直 TB。
    //
    // 若 evaluator 也改成同樣 directional 模型，這裡的方向 overflow 應該會與
    // evaluator report 越來越一致。
    void printFinalDiagnosis(const Design& design) {
        if (!ROUTER_DIAG_ENABLE) return;
        cerr << fixed << setprecision(3);

        cerr << "[RouterRefined][FT] Soft feedthrough usage after routing\n";
        for (const auto& b : design.blocks) {
            if (b.spec.type != BlockType::SOFT) continue;
            const double currentArea = b.rect.w * b.rect.h;
            const double reqArea = requiredAreaWithFTLocal(b, b.ftUsed);
            const double estOverflow = max(0.0, reqArea - currentArea);
            cerr << "  " << b.spec.name
                << " ftUsed=" << b.ftUsed
                << " currentArea=" << currentArea
                << " reqArea=" << reqArea
                << " estFTOverflowArea=" << estOverflow << "\n";
        }

        vector<DirUse> usage = computeDirectionalChannelUseFromRoutes(design);
        struct ChDiag {
            const Channel* ch = nullptr;
            int index = -1;
            double lrUsed = 0.0, tbUsed = 0.0;
            double lrCap = 0.0, tbCap = 0.0;
            double lrUtil = 0.0, tbUtil = 0.0;
            double lrOv = 0.0, tbOv = 0.0;
            double maxUtil = 0.0;
            double totalOv = 0.0;
            string maxComp;
        };

        vector<ChDiag> ds;
        for (int i = 0; i < static_cast<int>(design.channels.size()); ++i) {
            const Channel& ch = design.channels[i];
            ChDiag d;
            d.ch = &ch;
            d.index = i;
            d.lrUsed = usage[i].lr;
            d.tbUsed = usage[i].tb;
            d.lrCap = capLR(ch);
            d.tbCap = capTB(ch);
            d.lrUtil = d.lrCap > EPS ? d.lrUsed / d.lrCap : (d.lrUsed > EPS ? numeric_limits<double>::infinity() : 0.0);
            d.tbUtil = d.tbCap > EPS ? d.tbUsed / d.tbCap : (d.tbUsed > EPS ? numeric_limits<double>::infinity() : 0.0);
            d.lrOv = max(0.0, d.lrUsed - d.lrCap);
            d.tbOv = max(0.0, d.tbUsed - d.tbCap);
            d.maxUtil = max(d.lrUtil, d.tbUtil);
            d.totalOv = d.lrOv + d.tbOv;
            d.maxComp = d.lrUtil >= d.tbUtil ? "LR(horizontal)" : "TB(vertical)";
            ds.push_back(d);
        }
        sort(ds.begin(), ds.end(), [](const ChDiag& a, const ChDiag& b) {
            if (fabs(a.totalOv - b.totalOv) > 1e-12) return a.totalOv > b.totalOv;
            if (fabs(a.maxUtil - b.maxUtil) > 1e-12) return a.maxUtil > b.maxUtil;
            return a.ch->name < b.ch->name;
            });

        cerr << "[RouterRefined][ChannelDirectional] Top overflow/utilization\n";
        const int limit = min(static_cast<int>(ds.size()), ROUTER_DIAG_TOP_CHANNELS);
        for (int i = 0; i < limit; ++i) {
            const auto& d = ds[i];
            cerr << "  " << d.ch->name
                << " maxComp=" << d.maxComp
                << " maxUtil=" << d.maxUtil
                << " overflowTotal=" << d.totalOv
                << " | LR used=" << d.lrUsed << " cap=" << d.lrCap << " util=" << d.lrUtil << " ov=" << d.lrOv
                << " | TB used=" << d.tbUsed << " cap=" << d.tbCap << " util=" << d.tbUtil << " ov=" << d.tbOv
                << " rect=(" << d.ch->rect.x << "," << d.ch->rect.y << "," << d.ch->rect.w << "," << d.ch->rect.h << ")\n";
        }

        int ftPathCount = 0, openCount = 0;
        for (const auto& p : design.routes) {
            if (p.open) ++openCount;
            if (pathUsesSoftFT(design, p)) ++ftPathCount;
        }
        cerr << "[RouterRefined][Summary] routes=" << design.routes.size()
            << " routesUsingSoftFT=" << ftPathCount
            << " open=" << openCount
            << " totalChannelOverflow=" << totalChannelOverflowNow(design)
            << " totalWL=" << totalRouteWireLengthNow(design) << "\n";
    }

} // namespace

// -----------------------------------------------------------------------------
// Router::run
// -----------------------------------------------------------------------------
// 整體 routing 主流程：
//
// 0. 清空 routes、channel usage、soft block FT usage。
// 1. 建立 channel criticality：用 connection bbox 粗估哪些 channel 可能是全局 bottleneck。
// 2. 目前 baseline 仍採用 netCount 大到小的 greedy routing order。
// 3. 對每條 connection 先跑 Step1~4 strict routing：
//      Step1A: channel-only whole, 85% capacity reservation
//      Step1B: channel-only whole, full capacity
//      Step2A: channel-only limited split, 90% reservation
//      Step2B: channel-only limited split, full capacity
//      Step3 : FT-enabled whole
//      Step4 : FT-enabled limited split
// 4. Step1~4 全失敗者先放入 deferred，不立刻 overflow。
// 5. 所有 strict-routable connection 都 route 完後，才對 deferred connection 跑 Step5。
//      Step5: channel-only whole + allow overflow。
// 6. 最後印出 directional channel report 與 FT report。
//
// 注意：這版已含有限度的 congestion rip-up/reroute repair，但主體仍是
// greedy routing order；它仍可能因前面的 greedy commit 導致後面 connection
// 被迫 deferred。
// -----------------------------------------------------------------------------
void Router::setFTOverflowCostEnabled(bool enabled) {
    g_ftOverflowExtraWeight = enabled ? FT_OVERFLOW_EXTRA_WEIGHT_REPAIR : FT_OVERFLOW_EXTRA_WEIGHT_BASE;
}
void Router::setSoftFTCostRelaxed(bool enabled) {
    g_softFtFixedPenalty = enabled ? SOFT_FT_FIXED_PENALTY_RELAXED : SOFT_FT_FIXED_PENALTY_BASE;
    g_softFtPerNetPenalty = enabled ? SOFT_FT_PER_NET_PENALTY_RELAXED : SOFT_FT_PER_NET_PENALTY_BASE;
    g_ftHardCapacityGuard = !enabled;
}
void Router::setContactAwareCostEnabled(bool enabled) {
    g_contactAwareCostEnabled = enabled;
}
const vector<Router::FailureCertificate>& Router::failureCertificates() const {
    return lastFailureCertificates;
}

void Router::printDetourReport(const Design& design) const {
    printDetourReportImpl(design);
}

void Router::run(Design& design) {
    clearDirectionalUseCache();
    lastFailureCertificates.clear();
    g_forbiddenResources.clear();
    design.routes.clear();
    g_channelIndexByName.clear();
    g_channelIndexByName.reserve(design.channels.size() * 2 + 1);
    for (int i = 0; i < static_cast<int>(design.channels.size()); ++i) {
        g_channelIndexByName[design.channels[i].name] = i;
    }

    for (auto& ch : design.channels) {
        ch.usedNets = 0.0;
        ch.overflow = 0.0;
        // Keep capacity initialized to a useful legacy number before first route.
        // Final capacity is updated to the dominant directional component.
        ch.capacity = min(capLR(ch), capTB(ch));
    }
    for (auto& b : design.blocks) {
        b.ftUsed = 0.0;
        b.ftOverflowArea = 0.0;
    }
    
    routeGraphNodes = buildNodes(design);
    routeGraphAdj = buildGraph(design, routeGraphNodes);
    routeGraphCacheValid = true;

    buildChannelCriticalityLocal(design);

    vector<Connection> conns = design.connections;

    auto routeWithPolicy = [&](const Design& d, const Connection& c, RouteMode mode, bool allowOverflow,
        double capacityScale, bool splitRouting, bool requireSoftFT = false) -> RoutePath {
            g_routeMode = mode;
            g_allowChannelOverflow = allowOverflow;
            g_capacityScale = capacityScale;
            g_splitRouting = splitRouting;
            g_requireSoftFT = requireSoftFT;
            g_wirePolishMode = false;
            return routeOneConnection(d, c);
        };

    auto routeWithWirePolish = [&](const Design& d, const Connection& c, RouteMode mode, bool requireSoftFT = false) -> RoutePath {
        g_routeMode = mode;
        g_allowChannelOverflow = false;
        g_capacityScale = CAP_SCALE_FULL;
        g_splitRouting = false;
        g_requireSoftFT = requireSoftFT;
        g_wirePolishMode = true;
        RoutePath p = routeOneConnection(d, c);
        g_wirePolishMode = false;
        return p;
        };

    auto endpointRigidityScore = [&](const Connection& c) {
        double s = 0.0;
        if (c.src >= 0 && c.src < static_cast<int>(design.blocks.size())) {
            const auto& ports = design.blocks[c.src].spec.portEdges;
            if (!ports.empty()) s += 1.0 / static_cast<double>(ports.size());
        }
        if (c.dst >= 0 && c.dst < static_cast<int>(design.blocks.size())) {
            const auto& ports = design.blocks[c.dst].spec.portEdges;
            if (!ports.empty()) s += 1.0 / static_cast<double>(ports.size());
        }
        return s;
        };

    auto connectionCriticalityScore = [&](const Connection& c) {
        Connection unit = c;
        unit.netCount = 1;
        RoutePath p0 = routeWithPolicy(design, unit, RouteMode::CHANNEL_ONLY, false, CAP_SCALE_FULL, false);
        RouteMetrics m0 = analyzeRoute(design, p0);
        double bottleneck = 1.0;
        if (!m0.open && std::isfinite(m0.minComponentCapacity)) bottleneck = max(1.0, m0.minComponentCapacity);

        int candidateCount = (!m0.open ? 1 : 0);
        RoutePath pft = routeWithPolicy(design, unit, RouteMode::FT_ENABLED, false, CAP_SCALE_FULL, false, true);
        RouteMetrics mft = analyzeRoute(design, pft);
        if (!mft.open && mft.usesSoftFT && mft.channelOverflowIncrement <= EPS && mft.ftOverflowAfterTouched <= EPS) ++candidateCount;
        candidateCount = max(1, candidateCount);

        return static_cast<double>(max(0, c.netCount)) / bottleneck
            + 15.0 / static_cast<double>(candidateCount)
            + 0.05 * endpointRigidityScore(c) * static_cast<double>(max(0, c.netCount));
        };

    struct RankedConnection {
        Connection conn;
        double criticality = 0.0;
    };
    vector<RankedConnection> ranked;
    ranked.reserve(conns.size());
    for (const auto& c : conns) {
        RankedConnection rc;
        rc.conn = c;
        rc.criticality = connectionCriticalityScore(c);
        ranked.push_back(std::move(rc));
    }
    stable_sort(ranked.begin(), ranked.end(), [](const RankedConnection& a, const RankedConnection& b) {
        if (fabs(a.criticality - b.criticality) > 1.0e-6) return a.criticality > b.criticality;
        return a.conn.netCount > b.conn.netCount;
        });
    conns.clear();
    conns.reserve(ranked.size());
    for (const auto& rc : ranked) conns.push_back(rc.conn);

    auto applyPath = [&](Design& d, const RoutePath& path) {
        d.routes.push_back(path);
        updateUsage(d, path); // recomputes directional usage from all routes
        };

    auto printDecision = [&](const string& stage, const Design& d, const Connection& c, const RoutePath& p) {
        if (!ROUTER_DIAG_ENABLE || !ROUTER_DIAG_PRINT_DECISIONS) return;
        RouteMetrics m = analyzeRoute(d, p);
        cerr << fixed << setprecision(3)
            << "[RouterRefined][" << stage << "] "
            << d.blocks[c.src].spec.name << "->" << d.blocks[c.dst].spec.name
            << " nets=" << c.netCount
            << " open=" << (m.open ? "Y" : "N")
            << " usesFT=" << (m.usesSoftFT ? "Y" : "N")
            << " softCnt=" << m.softFTBlockCount
            << " ovInc=" << m.channelOverflowIncrement
            << " maxUtil=" << m.maxProjectedChannelUtil
            << " maxUtilComp=" << (m.maxUtilComponent.empty() ? "NA" : m.maxUtilComponent)
            << " minCompCap=" << (std::isfinite(m.minComponentCapacity) ? m.minComponentCapacity : 0.0)
            << " narrowCompCnt=" << m.narrowComponentCount
            << " ftIncArea=" << m.ftIncrementalArea
            << " ftOvInc=" << m.ftOverflowIncrement
            << " ftOvAfter=" << m.ftOverflowAfterTouched
            << " wl=" << m.wireLength << "\n";
        if (ROUTER_DIAG_PRINT_PATHS) { cerr << "  "; printPathLine(p); }
        };

    auto routeChoiceScore = [](const RouteMetrics& m) -> double {
        if (m.open) return numeric_limits<double>::infinity();
        const double utilRisk = max(0.0, m.maxProjectedChannelUtil - CHANNEL_UTIL_RISK_START);
        return ROUTE_CHOICE_ALPHA_PROXY * m.wireLength
            + CHANNEL_OVERFLOW_LINEAR_WEIGHT * m.channelOverflowIncrement
            + FT_CHOICE_AREA_WEIGHT * m.ftIncrementalArea
            + FT_CHOICE_OVERFLOW_AREA_WEIGHT * m.ftOverflowIncrement
            + CHANNEL_UTIL_RISK_WEIGHT * utilRisk * utilRisk;
        };

    auto shouldUseStrictFTCandidate = [&](const RouteMetrics& baseM, const RouteMetrics& ftM) -> bool {
        if (ftM.open || !ftM.usesSoftFT || ftM.channelOverflowIncrement > EPS) return false;
        if (ftM.ftOverflowAfterTouched > EPS) return false;

        const double baseScore = routeChoiceScore(baseM);
        const double ftScore = routeChoiceScore(ftM);
        if (!std::isfinite(baseScore)) return true;
        if (ftScore <= baseScore) return true;
        if (baseM.maxProjectedChannelUtil >= FT_PROACTIVE_UTIL_TRIGGER &&
            ftScore <= baseScore * FT_PROACTIVE_SCORE_RELAX + FT_PROACTIVE_SCORE_ABS_RELAX) return true;
        return false;
        };

    auto shouldUseOverflowFTCandidate = [&](const RouteMetrics& baseM, const RouteMetrics& ftM) -> bool {
        if (ftM.open || !ftM.usesSoftFT) return false;
        if (baseM.open) return true;
        if (ftM.ftOverflowAfterTouched > EPS) return false;

        const double basePenalty = baseM.channelOverflowIncrement + baseM.ftOverflowIncrement;
        const double ftPenalty = ftM.channelOverflowIncrement + ftM.ftOverflowIncrement;
        if (ftPenalty + EPS < basePenalty) return true;
        if (ftPenalty > basePenalty + 500.0) return false;

        const double baseScore = routeChoiceScore(baseM);
        const double ftScore = routeChoiceScore(ftM);
        if (!std::isfinite(baseScore)) return true;
        return ftScore <= baseScore * FT_PROACTIVE_SCORE_RELAX + FT_PROACTIVE_SCORE_ABS_RELAX;
        };
    struct TrialResult {
        bool success = false;
        Design design;
        int chunkSize = 0;
        int routeCountAdded = 0;
        double totalOverflow = numeric_limits<double>::infinity();
        double totalWL = numeric_limits<double>::infinity();
        int ftPathCountAdded = 0;
    };

    // split health check。
//
// 這個檢查用來避免 unhealthy split：
//   1. split route 雖然每條都沒有 overflow，但最後把同一個 channel component 塞到很滿。
//   2. 多數 split chunks 都經過同一個 bottleneck，代表它沒有真正分流。
//   3. 分流後使用極窄 component，未來很容易導致其他 connection open/overflow。
    auto splitTrialHealthy = [&](const Design& base, const Design& trial, const map<pair<int, DirComponent>, int>& hitCount,
        const map<pair<int, DirComponent>, double>& splitLoad, int addedRoutes, double splitCapScale) -> bool {
            if (addedRoutes <= 0) return false;
            vector<DirUse> after = computeDirectionalChannelUseFromRoutes(trial);
            const double legalLimit = (splitCapScale >= 1.0 - 1.0e-9) ? 1.0 : max(0.0, splitCapScale);

            for (const auto& kv : hitCount) {
                int ci = kv.first.first;
                DirComponent comp = kv.first.second;
                int hits = kv.second;
                if (ci < 0 || ci >= static_cast<int>(trial.channels.size())) continue;
                const Channel& ch = trial.channels[ci];
                const double cap = comp == DirComponent::LR ? capLR(ch) : capTB(ch);
                const double used = comp == DirComponent::LR ? after[ci].lr : after[ci].tb;
                const double finalUtil = cap > EPS ? used / cap : numeric_limits<double>::infinity();
                const double share = static_cast<double>(hits) / static_cast<double>(addedRoutes);
                const double load = splitLoad.count(kv.first) ? splitLoad.at(kv.first) : 0.0;

                if (finalUtil > legalLimit + CHANNEL_CAPACITY_EPS) return false;
                if (addedRoutes >= 4 && share >= SPLIT_BOTTLENECK_SHARE && finalUtil >= SPLIT_BOTTLENECK_FINAL_UTIL) {
                    // Diagnostic only.  A split that is still within strict capacity is not
                    // treated as infeasible here; real blocking is handled by capacity and FT checks.
                    (void)load;
                }
            }
            return true;
        };

    // limited split 嘗試。
//
// Split 的目的不是「把 connection 切到很碎就一定塞進去」，而是：
//   大 connection whole route 過不了時，嘗試把它切成少數幾條較大的 chunk，
//   讓不同 chunk 可以走不同健康通道。
//
// 本版限制：
//   - chunk 只允許 300 / 100 / 50 / 20。
//   - split 後 route 數不能超過 MAX_SPLIT_ROUTES。
//   - 每條 chunk path 不准造成 directional overflow。
//   - 每條 chunk path 不准經過 narrow component。
//   - splitTrialHealthy() 會拒絕「多數 chunk 都塞同一個 bottleneck」的假分流。
    auto trySplit = [&](const Design& base, const Connection& conn, RouteMode mode, double splitCapScale) -> TrialResult {
        TrialResult best;
        Design trial = base;
        int remaining = conn.netCount;
        int added = 0;
        int ftAdded = 0;
        map<pair<int, DirComponent>, int> hitCount;
        map<pair<int, DirComponent>, double> splitLoad;

        auto residualForUnitPath = [&](const Design& d, const RoutePath& unitPath) {
            double limit = numeric_limits<double>::infinity();
            vector<DirUse> current = computeDirectionalChannelUseFromRoutes(d);
            PathDirUse pd = computeDirectionalUseForPath(d, unitPath);
            for (const auto& kv : pd.byChannel) {
                const int ci = kv.first;
                if (ci < 0 || ci >= static_cast<int>(d.channels.size())) continue;
                const Channel& ch = d.channels[ci];
                if (kv.second.lr > EPS) {
                    const double cap = max(0.0, capLR(ch) * splitCapScale);
                    limit = min(limit, (cap - current[ci].lr) / kv.second.lr);
                }
                if (kv.second.tb > EPS) {
                    const double cap = max(0.0, capTB(ch) * splitCapScale);
                    limit = min(limit, (cap - current[ci].tb) / kv.second.tb);
                }
            }
            const map<int, int> softCounts = softTransitCountsForPath(d, unitPath);
            for (const auto& kv : softCounts) {
                const int bi = kv.first;
                if (bi < 0 || bi >= static_cast<int>(d.blocks.size()) || kv.second <= 0) return 0;
                const BlockInst& b = d.blocks[bi];
                const double cap = static_cast<double>(softFTCapacityNetsLocal(b));
                limit = min(limit, (cap - b.ftUsed) / static_cast<double>(kv.second));
            }
            if (!std::isfinite(limit)) return conn.netCount;
            return max(0, static_cast<int>(floor(limit + 1.0e-7)));
            };

        while (remaining > 0 && added < MAX_SPLIT_ROUTES) {
            Connection unit = conn;
            unit.netCount = 1;
            RoutePath unitPath = routeWithPolicy(trial, unit, mode, false, splitCapScale, true, mode == RouteMode::FT_ENABLED);
            RouteMetrics unitM = analyzeRoute(trial, unitPath);
            if (unitM.open || unitM.channelOverflowIncrement > EPS ||
                (mode == RouteMode::FT_ENABLED && unitM.ftOverflowAfterTouched > EPS)) break;

            const int residual = residualForUnitPath(trial, unitPath);
            if (residual <= 0) break;
            const int take = min(remaining, residual);
            if (take <= 0) break;

            RoutePath p = unitPath;
            p.netCount = take;
            p.wireLength = calcRouteWireLength(trial, p);
            RouteMetrics m = analyzeRoute(trial, p);
            if (m.open || m.channelOverflowIncrement > EPS ||
                (mode == RouteMode::FT_ENABLED && m.ftOverflowAfterTouched > EPS)) break;

            for (const auto& kv : m.componentDelta) {
                hitCount[kv.first] += 1;
                splitLoad[kv.first] += kv.second;
            }
            if (m.usesSoftFT) ++ftAdded;
            applyPath(trial, p);
            ++added;
            remaining -= take;
        }

        if (remaining > 0) return best;
        if (!splitTrialHealthy(base, trial, hitCount, splitLoad, added, splitCapScale)) return best;

        const double ov = totalChannelOverflowNow(trial);
        const double wl = totalRouteWireLengthNow(trial);
        if (ov <= EPS) {
            best.success = true;
            best.design = std::move(trial);
            best.chunkSize = conn.netCount;
            best.routeCountAdded = added;
            best.totalOverflow = ov;
            best.totalWL = wl;
            best.ftPathCountAdded = ftAdded;
        }
        return best;
        };

    struct DeferredConn { Connection conn; };
    vector<DeferredConn> deferred;

    auto printOpenAnalysis = [&](const Design& d, const Connection& conn, const RoutePath& step5Path) {
        if (!ROUTER_DIAG_ENABLE) return;
        vector<Node> nodes = buildNodes(d);
        vector<vector<AdjEdge>> g = buildGraph(d, nodes);
        const int srcNode = conn.src;
        const int dstNode = conn.dst;
        const int N = static_cast<int>(nodes.size());

        auto allowedIntermediateByMode = [&](RouteMode mode, const Node& node) -> bool {
            if (!node.isBlock) return true;
            if (mode == RouteMode::CHANNEL_ONLY) return false;
            const BlockInst& b = d.blocks[node.index];
            return b.spec.type == BlockType::SOFT;
            };

        auto reachable = [&](RouteMode mode, bool ignoreCapacity) -> bool {
            vector<char> vis(N, 0);
            queue<int> q;
            vis[srcNode] = 1;
            q.push(srcNode);
            while (!q.empty()) {
                int u = q.front(); q.pop();
                if (u == dstNode) return true;
                for (const auto& e : g[u]) {
                    int v = e.to;
                    if (vis[v]) continue;
                    const bool isEndpoint = (v == srcNode || v == dstNode);
                    if (!isEndpoint && !allowedIntermediateByMode(mode, nodes[v])) continue;
                    if (!ignoreCapacity && !isEndpoint && !nodes[v].isBlock) {
                        // Approximate reachability capacity check: if the channel has no
                        // component with enough remaining capacity, consider it blocked.
                        vector<DirUse> cur = computeDirectionalChannelUseFromRoutes(d);
                        const Channel& ch = d.channels[nodes[v].index];
                        const double remLR = capLR(ch) - cur[nodes[v].index].lr;
                        const double remTB = capTB(ch) - cur[nodes[v].index].tb;
                        if (remLR + CHANNEL_CAPACITY_EPS < conn.netCount && remTB + CHANNEL_CAPACITY_EPS < conn.netCount) continue;
                    }
                    vis[v] = 1;
                    q.push(v);
                }
            }
            return false;
            };

        const bool chGeom = reachable(RouteMode::CHANNEL_ONLY, true);
        const bool chStrict = reachable(RouteMode::CHANNEL_ONLY, false);
        const bool ftGeom = reachable(RouteMode::FT_ENABLED, true);
        const bool ftStrict = reachable(RouteMode::FT_ENABLED, false);

        string diagnosis;
        if (!chGeom && !ftGeom) diagnosis = "GEOMETRY_DISCONNECTED_EVEN_WITH_SOFT_FT";
        else if (!chGeom && ftGeom) diagnosis = "CHANNEL_ONLY_GEOMETRY_DISCONNECTED_BUT_FT_GEOMETRY_EXISTS";
        else if (chGeom && !chStrict) diagnosis = "CAPACITY_OR_DIRECTIONAL_GUARD_BLOCKED";
        else if (chStrict) diagnosis = "STRICT_REACHABLE_BUT_DIJKSTRA_OR_POLICY_FAILED";
        else diagnosis = "UNCLASSIFIED_CHECK_PATH_OR_POLICY";

        cerr << fixed << setprecision(3)
            << "[RouterRefined][OPEN_ANALYSIS] "
            << d.blocks[conn.src].spec.name << "->" << d.blocks[conn.dst].spec.name
            << " nets=" << conn.netCount
            << " step5Open=" << (step5Path.open ? "Y" : "N")
            << " chGeom=" << (chGeom ? "Y" : "N")
            << " chStrictApprox=" << (chStrict ? "Y" : "N")
            << " ftGeom=" << (ftGeom ? "Y" : "N")
            << " ftStrictApprox=" << (ftStrict ? "Y" : "N")
            << " diagnosis=" << diagnosis << "\n";
        };

    // 嘗試 Step1~4 strict routing。
// 回傳 true 代表此 connection 已經被成功 route 並 commit 到 design。
// 回傳 false 代表 strict routing 全失敗，需要交給 deferred Step5。
//
// 這裡的所有 Step1~4 都不允許 channel overflow：
//   - Dijkstra 過程中 directionalChannelPenalty() 會以 g_capacityScale 檢查
//     projected LR/TB usage 是否超過有效容量。
//   - 找到 path 後 analyzeRoute() 仍會再做 projected overflow 檢查。
//
// Step1/2 是主要 router；Step3/4 只有在 soft block 幾何上可作為中繼時才會發揮。
    auto addFailureCertificate = [&](const Connection& conn, const RoutePath& chPath, const RouteMetrics& chM,
        const RoutePath& ftPath, const RouteMetrics& ftM) {
            FailureCertificate cert;
            cert.conn = conn;
            const bool useFT = shouldUseOverflowFTCandidate(chM, ftM);
            cert.diagnosticPath = useFT ? ftPath : chPath;
            cert.diagnosticOpen = cert.diagnosticPath.open;
            cert.geometryDisconnected = cert.diagnosticOpen;
            cert.policyBlocked = cert.diagnosticOpen;
            cert.hasLegalFTAlternative = !ftM.open && ftM.usesSoftFT &&
                ftM.channelOverflowIncrement <= EPS && ftM.ftOverflowAfterTouched <= EPS;

            if (!cert.diagnosticOpen) {
                vector<DirUse> current = computeDirectionalChannelUseFromRoutes(design);
                PathDirUse pd = computeDirectionalUseForPath(design, cert.diagnosticPath);
                auto makeNeed = [&](int ci, ChannelDir dir, double used, double cap, double extra, double shortage) {
                    ResourceNeed need;
                    need.channelIndex = ci;
                    need.dir = dir;
                    need.currentUsed = used;
                    need.capacity = cap;
                    need.extraDemand = extra;
                    need.shortageNets = shortage;
                    need.requiredDeltaUm = shortage / CHANNEL_DENSITY + 1.0;
                    return need;
                    };
                auto addNeed = [&](int ci, ChannelDir dir, double used, double cap, double extra) {
                    if (ci < 0 || ci >= static_cast<int>(design.channels.size()) || extra <= EPS) return;
                    const double projected = used + extra;
                    const double hardShortage = max(0.0, projected - cap);
                    if (hardShortage > CHANNEL_CAPACITY_EPS) {
                        cert.hardNeeds.push_back(makeNeed(ci, dir, used, cap, extra, hardShortage));
                        return;
                    }
                    const double util = cap > EPS ? projected / cap : numeric_limits<double>::infinity();
                    double riskShortage = 0.0;
                    if (cap < NARROW_COMPONENT_CAP) riskShortage = max(1.0, min(extra, NARROW_COMPONENT_CAP - cap));
                    else if (util > 0.92) riskShortage = max(1.0, projected - 0.90 * cap);
                    if (riskShortage > CHANNEL_CAPACITY_EPS) {
                        cert.riskNeeds.push_back(makeNeed(ci, dir, used, cap, extra, riskShortage));
                    }
                    };
                for (const auto& kv : pd.byChannel) {
                    const int ci = kv.first;
                    if (ci < 0 || ci >= static_cast<int>(design.channels.size())) continue;
                    const Channel& ch = design.channels[ci];
                    addNeed(ci, ChannelDir::LR, current[ci].lr, capLR(ch), kv.second.lr);
                    addNeed(ci, ChannelDir::TB, current[ci].tb, capTB(ch), kv.second.tb);
                }
                auto needLess = [](const ResourceNeed& a, const ResourceNeed& b) {
                    if (fabs(a.shortageNets - b.shortageNets) > 1.0) return a.shortageNets > b.shortageNets;
                    if (a.channelIndex != b.channelIndex) return a.channelIndex < b.channelIndex;
                    return static_cast<int>(a.dir) < static_cast<int>(b.dir);
                    };
                sort(cert.hardNeeds.begin(), cert.hardNeeds.end(), needLess);
                sort(cert.riskNeeds.begin(), cert.riskNeeds.end(), needLess);
                if (cert.hardNeeds.size() > 8) cert.hardNeeds.resize(8);
                if (cert.riskNeeds.size() > 8) cert.riskNeeds.resize(8);
                cert.policyBlocked = cert.hardNeeds.empty() && !cert.riskNeeds.empty();
            }
            lastFailureCertificates.push_back(std::move(cert));
        };

    auto tryStrictStep1To4 = [&](Design& d, const Connection& conn) -> bool {
        RoutePath chWhole = routeWithPolicy(d, conn, RouteMode::CHANNEL_ONLY, false, CAP_SCALE_CH_WHOLE_RESERVED, false);
        RouteMetrics chWholeM = analyzeRoute(d, chWhole);
        printDecision("STEP1A_CH_WHOLE_RESERVE", d, conn, chWhole);

        RoutePath ftReserve = routeWithPolicy(d, conn, RouteMode::FT_ENABLED, false, CAP_SCALE_CH_WHOLE_RESERVED, false, true);
        RouteMetrics ftReserveM = analyzeRoute(d, ftReserve);
        printDecision("STEP1A_FT_FORCED_RESERVE", d, conn, ftReserve);

        RoutePath chWholeFull = routeWithPolicy(d, conn, RouteMode::CHANNEL_ONLY, false, CAP_SCALE_FULL, false);
        RouteMetrics chWholeFullM = analyzeRoute(d, chWholeFull);
        printDecision("STEP1B_CH_WHOLE_FULL", d, conn, chWholeFull);

        RoutePath ftFull = routeWithPolicy(d, conn, RouteMode::FT_ENABLED, false, CAP_SCALE_FULL, false, true);
        RouteMetrics ftFullM = analyzeRoute(d, ftFull);
        printDecision("STEP1B_FT_FORCED_FULL", d, conn, ftFull);

        if (static_cast<int>(d.blocks.size()) < 24) {
            if (!chWholeM.open && chWholeM.channelOverflowIncrement <= EPS) {
                if (shouldUseStrictFTCandidate(chWholeM, ftReserveM)) { applyPath(d, ftReserve); return true; }
                applyPath(d, chWhole); return true;
            }
            if (shouldUseStrictFTCandidate(chWholeM, ftReserveM)) { applyPath(d, ftReserve); return true; }

            if (!chWholeFullM.open && chWholeFullM.channelOverflowIncrement <= EPS) {
                if (shouldUseStrictFTCandidate(chWholeFullM, ftFullM)) { applyPath(d, ftFull); return true; }
                applyPath(d, chWholeFull); return true;
            }
            if (shouldUseStrictFTCandidate(chWholeFullM, ftFullM)) { applyPath(d, ftFull); return true; }
        }
        bool haveWhole = false;
        RoutePath bestWhole;
        RouteMetrics bestWholeM;
        double bestWholeScore = numeric_limits<double>::infinity();
        auto considerWhole = [&](const RoutePath& p, const RouteMetrics& m, bool requireFTCapacity) {
            if (m.open || m.channelOverflowIncrement > EPS) return;
            if (requireFTCapacity && m.ftOverflowAfterTouched > EPS) return;
            const double s = routeChoiceScore(m);
            if (!haveWhole || s < bestWholeScore) {
                haveWhole = true;
                bestWhole = p;
                bestWholeM = m;
                bestWholeScore = s;
            }
            };
        considerWhole(chWhole, chWholeM, false);
        considerWhole(chWholeFull, chWholeFullM, false);
        considerWhole(ftReserve, ftReserveM, true);
        considerWhole(ftFull, ftFullM, true);
        if (haveWhole) {
            applyPath(d, bestWhole);
            return true;
        }
        TrialResult chSplit = trySplit(d, conn, RouteMode::CHANNEL_ONLY, CAP_SCALE_CH_SPLIT_RESERVED);
        if (chSplit.success) {
            if (ROUTER_DIAG_ENABLE && ROUTER_DIAG_PRINT_DECISIONS) {
                cerr << "[RouterRefined][CH_SPLIT_ACCEPT] " << d.blocks[conn.src].spec.name << "->" << d.blocks[conn.dst].spec.name
                    << " nets=" << conn.netCount << " chunk=" << chSplit.chunkSize
                    << " addedRoutes=" << chSplit.routeCountAdded << " totalWL=" << chSplit.totalWL
                    << " totalOverflow=" << chSplit.totalOverflow << "\n";
            }
            d = std::move(chSplit.design); return true;
        }

        TrialResult chSplitFull = trySplit(d, conn, RouteMode::CHANNEL_ONLY, CAP_SCALE_FULL);
        if (chSplitFull.success) {
            if (ROUTER_DIAG_ENABLE && ROUTER_DIAG_PRINT_DECISIONS) {
                cerr << "[RouterRefined][CH_SPLIT_FULL_ACCEPT] " << d.blocks[conn.src].spec.name << "->" << d.blocks[conn.dst].spec.name
                    << " nets=" << conn.netCount << " chunk=" << chSplitFull.chunkSize
                    << " addedRoutes=" << chSplitFull.routeCountAdded << " totalWL=" << chSplitFull.totalWL
                    << " totalOverflow=" << chSplitFull.totalOverflow << "\n";
            }
            d = std::move(chSplitFull.design); return true;
        }

        RoutePath ftWhole = routeWithPolicy(d, conn, RouteMode::FT_ENABLED, false, CAP_SCALE_FULL, false, true);
        RouteMetrics ftWholeM = analyzeRoute(d, ftWhole);
        printDecision("STEP3_FT_FORCED_WHOLE", d, conn, ftWhole);
        if (!ftWholeM.open && ftWholeM.usesSoftFT && ftWholeM.channelOverflowIncrement <= EPS && ftWholeM.ftOverflowAfterTouched <= EPS) { applyPath(d, ftWhole); return true; }

        TrialResult ftSplit = trySplit(d, conn, RouteMode::FT_ENABLED, CAP_SCALE_FULL);
        if (ftSplit.success) {
            if (ROUTER_DIAG_ENABLE && ROUTER_DIAG_PRINT_DECISIONS) {
                cerr << "[RouterRefined][FT_SPLIT_ACCEPT] " << d.blocks[conn.src].spec.name << "->" << d.blocks[conn.dst].spec.name
                    << " nets=" << conn.netCount << " chunk=" << ftSplit.chunkSize
                    << " addedRoutes=" << ftSplit.routeCountAdded
                    << " ftAddedRoutes=" << ftSplit.ftPathCountAdded << " totalWL=" << ftSplit.totalWL
                    << " totalOverflow=" << ftSplit.totalOverflow << "\n";
            }
            d = std::move(ftSplit.design); return true;
        }
        return false;
        };
    int targetedExchangeCalls = 0;
    int targetedExchangeAttempts = 0;
    auto tryTargetedBlockerExchange = [&](Design& d, const Connection& blocked) -> bool {
        struct TargetNeed {
            ResourceKey key;
            double used = 0.0;
            double cap = 0.0;
            double extra = 0.0;
            double shortage = 0.0;
        };

        auto buildHardNeeds = [&](const RouteMetrics& m) {
            vector<TargetNeed> needs;
            if (m.open) return needs;
            vector<DirUse> current = computeDirectionalChannelUseFromRoutes(d);
            for (const auto& kv : m.componentDelta) {
                const int ci = kv.first.first;
                const DirComponent dir = kv.first.second;
                if (ci < 0 || ci >= static_cast<int>(d.channels.size()) || ci >= static_cast<int>(current.size())) continue;
                const Channel& ch = d.channels[ci];
                const double used = dir == DirComponent::LR ? current[ci].lr : current[ci].tb;
                const double cap = dir == DirComponent::LR ? capLR(ch) : capTB(ch);
                const double extra = kv.second;
                const double shortage = used + extra - cap;
                if (shortage <= CHANNEL_CAPACITY_EPS) continue;
                needs.push_back(TargetNeed{ ResourceKey{ ci, dir }, used, cap, extra, shortage });
            }
            sort(needs.begin(), needs.end(), [](const TargetNeed& a, const TargetNeed& b) {
                if (fabs(a.shortage - b.shortage) > 1.0) return a.shortage > b.shortage;
                if (fabs(a.extra - b.extra) > 1.0) return a.extra > b.extra;
                if (a.key.channelIndex != b.key.channelIndex) return a.key.channelIndex < b.key.channelIndex;
                return static_cast<int>(a.key.dir) < static_cast<int>(b.key.dir);
                });
            return needs;
            };

        RoutePath chDiag = routeWithPolicy(d, blocked, RouteMode::CHANNEL_ONLY, true, CAP_SCALE_FULL, false);
        RouteMetrics chM = analyzeRoute(d, chDiag);
        RoutePath ftDiag = routeWithPolicy(d, blocked, RouteMode::FT_ENABLED, true, CAP_SCALE_FULL, false, true);
        RouteMetrics ftM = analyzeRoute(d, ftDiag);
        const bool useFT = shouldUseOverflowFTCandidate(chM, ftM);
        const RouteMetrics& chosenM = useFT ? ftM : chM;
        vector<TargetNeed> targets = buildHardNeeds(chosenM);
        if (targets.empty()) {
            if (ROUTER_DIAG_ENABLE) {
                cerr << "[FailClass] " << d.blocks[blocked.src].spec.name << "->" << d.blocks[blocked.dst].spec.name
                    << " nets=" << blocked.netCount
                    << " diagnosticOpen=" << (chosenM.open ? "Y" : "N")
                    << " hardNeeds=0 policyOrGeometry=Y\n";
            }
            return false;
        }
        if (targetedExchangeCalls >= TARGETED_EXCHANGE_MAX_CALLS_PER_RUN ||
            targetedExchangeAttempts >= TARGETED_EXCHANGE_MAX_ATTEMPTS_PER_RUN) return false;
        ++targetedExchangeCalls;

        struct DonorGroup {
            RipupKey key;
            Connection conn;
            double release = 0.0;
            double wire = 0.0;
            int routeCount = 0;
        };

        const RouterScore baseScore = scoreRouterSolution(d);
        int attempts = 0;
        const int targetLimit = min(2, static_cast<int>(targets.size()));
        for (int ti = 0; ti < targetLimit; ++ti) {
            const TargetNeed& target = targets[ti];
            map<RipupKey, DonorGroup> donorMap;
            for (const auto& p : d.routes) {
                if (p.open || p.netCount <= 0) continue;
                auto sit = d.blockNameToIndex.find(p.srcBlock);
                auto dit = d.blockNameToIndex.find(p.dstBlock);
                if (sit == d.blockNameToIndex.end() || dit == d.blockNameToIndex.end()) continue;
                RipupKey key{ sit->second, dit->second };
                if (key.src == blocked.src && key.dst == blocked.dst) continue;
                PathDirUse pd = computeDirectionalUseForPath(d, p);
                auto uit = pd.byChannel.find(target.key.channelIndex);
                if (uit == pd.byChannel.end()) continue;
                const double release = target.key.dir == DirComponent::LR ? uit->second.lr : uit->second.tb;
                if (release <= EPS) continue;
                DonorGroup& g = donorMap[key];
                g.key = key;
                g.conn.src = key.src;
                g.conn.dst = key.dst;
                g.conn.netCount += max(0, p.netCount);
                g.release += release;
                g.wire += max(0.0, p.wireLength);
                ++g.routeCount;
            }

            vector<DonorGroup> donors;
            for (auto& kv : donorMap) {
                if (kv.second.conn.netCount > 0) donors.push_back(kv.second);
            }
            sort(donors.begin(), donors.end(), [](const DonorGroup& a, const DonorGroup& b) {
                if (fabs(a.release - b.release) > 1.0) return a.release > b.release;
                if (a.conn.netCount != b.conn.netCount) return a.conn.netCount > b.conn.netCount;
                return a.wire > b.wire;
                });
            if (static_cast<int>(donors.size()) > TARGETED_EXCHANGE_MAX_DONORS) donors.resize(TARGETED_EXCHANGE_MAX_DONORS);

            for (const DonorGroup& donor : donors) {
                if (targetedExchangeAttempts >= TARGETED_EXCHANGE_MAX_ATTEMPTS_PER_RUN) break;
                ++targetedExchangeAttempts;
                ++attempts;
                Design trial = d;
                set<RipupKey> selected;
                selected.insert(donor.key);
                vector<RoutePath> kept;
                kept.reserve(trial.routes.size());
                for (const auto& p : trial.routes) {
                    if (!routeMatchesKey(trial, p, selected)) kept.push_back(p);
                }
                trial.routes = std::move(kept);
                recomputeRouterUsageFields(trial);
                clearDirectionalUseCache();

                set<ResourceKey> savedForbidden = g_forbiddenResources;
                g_forbiddenResources.clear();
                g_forbiddenResources.insert(target.key);
                const bool donorOk = tryStrictStep1To4(trial, donor.conn);
                g_forbiddenResources = std::move(savedForbidden);
                if (!donorOk) continue;

                if (!tryStrictStep1To4(trial, blocked)) continue;
                recomputeRouterUsageFields(trial);
                clearDirectionalUseCache();
                const RouterScore trialScore = scoreRouterSolution(trial);
                const bool legal = trialScore.open <= baseScore.open &&
                    trialScore.channelOverflow <= baseScore.channelOverflow + CHANNEL_CAPACITY_EPS &&
                    trialScore.ftOverflow <= baseScore.ftOverflow + EPS;
                if (!legal) continue;

                if (ROUTER_DIAG_ENABLE) {
                    cerr << fixed << setprecision(3)
                        << "[Exchange] accept blocked=" << d.blocks[blocked.src].spec.name << "->" << d.blocks[blocked.dst].spec.name
                        << " target=CH" << (target.key.channelIndex + 1) << "." << shortCompName(target.key.dir)
                        << " shortage=" << target.shortage
                        << " donor=" << d.blocks[donor.conn.src].spec.name << "->" << d.blocks[donor.conn.dst].spec.name
                        << " release=" << donor.release
                        << " attempts=" << attempts << "\n";
                }
                d = std::move(trial);
                clearDirectionalUseCache();
                return true;
            }
        }

        if (ROUTER_DIAG_ENABLE && attempts > 0) {
            cerr << "[Exchange] reject blocked=" << d.blocks[blocked.src].spec.name << "->" << d.blocks[blocked.dst].spec.name
                << " attempts=" << attempts << "\n";
        }
        return false;
        };

    struct ConcurrentCandidate {
        RoutePath unitPath;
        PathDirUse unitUse;
        map<int, int> softCounts;
        double unitWire = 0.0;
        double baseCost = 0.0;
        double flow = 0.0;
        string signature;
        bool usesSoftFT = false;
        bool allowOverflow = false;
    };

    struct ConcurrentPool {
        Connection conn;
        vector<ConcurrentCandidate> candidates;
    };

    auto concurrentSignature = [](const RoutePath& p) {
        string sig;
        for (const auto& st : p.steps) {
            sig += st.rectName;
            sig += '#';
            sig += to_string(st.edge);
            sig += ';';
        }
        return sig;
        };

    auto addConcurrentCandidate = [&](ConcurrentPool& pool, const Design& seed, const Connection& conn,
        RouteMode mode, bool allowOverflow, double capacityScale, bool requireSoftFT) {
            Connection unit = conn;
            unit.netCount = 1;
            RoutePath p = routeWithPolicy(seed, unit, mode, allowOverflow, capacityScale, false, requireSoftFT);
            RouteMetrics m = analyzeRoute(seed, p);
            if (m.open || routeHasIllegalIntermediateBlock(seed, p)) return;
            if (requireSoftFT && !m.usesSoftFT) return;
            if (m.channelOverflowIncrement > EPS) return;
            if (m.usesSoftFT && m.ftOverflowAfterTouched > EPS) return;

            const string sig = concurrentSignature(p);
            for (const ConcurrentCandidate& old : pool.candidates) {
                if (old.signature == sig) return;
            }

            ConcurrentCandidate cand;
            cand.unitPath = std::move(p);
            cand.unitPath.netCount = 1;
            cand.unitPath.wireLength = calcRouteWireLength(seed, cand.unitPath);
            cand.unitUse = computeDirectionalUseForPath(seed, cand.unitPath);
            cand.softCounts = softTransitCountsForPath(seed, cand.unitPath);
            cand.unitWire = max(1.0, cand.unitPath.wireLength);
            cand.baseCost = routeChoiceScore(m);
            if (!std::isfinite(cand.baseCost)) cand.baseCost = ROUTE_CHOICE_ALPHA_PROXY * cand.unitWire;
            if (allowOverflow) cand.baseCost += 25000.0;
            if (!cand.softCounts.empty()) cand.baseCost += 2000.0;
            cand.signature = sig;
            cand.usesSoftFT = !cand.softCounts.empty();
            cand.allowOverflow = allowOverflow;
            pool.candidates.push_back(std::move(cand));
        };

    auto addConcurrentBiasedCandidate = [&](ConcurrentPool& pool, const Design& seed, const Connection& conn,
        RouteMode mode, bool requireSoftFT, const PathDirUse& avoidUse) {
            if (!ENABLE_CONCURRENT_BIASED_KPATH || avoidUse.byChannel.empty()) return;

            const bool oldCongestionMode = g_congestionRerouteMode;
            const double oldCongestionWeight = g_congestionBiasWeight;
            vector<array<double, 2>> oldHot = g_hotChannelBias;

            g_congestionRerouteMode = true;
            g_congestionBiasWeight = 3.0e7;
            g_hotChannelBias.assign(seed.channels.size(), { 0.0, 0.0 });
            for (const auto& kv : avoidUse.byChannel) {
                const int ci = kv.first;
                if (ci < 0 || ci >= static_cast<int>(g_hotChannelBias.size())) continue;
                if (kv.second.lr > EPS) g_hotChannelBias[ci][0] += 1.0;
                if (kv.second.tb > EPS) g_hotChannelBias[ci][1] += 1.0;
            }

            addConcurrentCandidate(pool, seed, conn, mode, false, CAP_SCALE_FULL, requireSoftFT);

            g_congestionRerouteMode = oldCongestionMode;
            g_congestionBiasWeight = oldCongestionWeight;
            g_hotChannelBias = std::move(oldHot);
        };

    auto concurrentCandidateScore = [&](const ConcurrentCandidate& cand,
        const vector<array<double, 2>>& channelPrice,
        const vector<double>& softPrice) {
            double score = cand.baseCost;
            for (const auto& kv : cand.unitUse.byChannel) {
                const int ci = kv.first;
                if (ci < 0 || ci >= static_cast<int>(channelPrice.size())) continue;
                score += kv.second.lr * channelPrice[ci][0];
                score += kv.second.tb * channelPrice[ci][1];
            }
            for (const auto& kv : cand.softCounts) {
                const int bi = kv.first;
                if (bi >= 0 && bi < static_cast<int>(softPrice.size())) {
                    score += static_cast<double>(kv.second) * softPrice[bi];
                }
            }
            return score;
        };

    struct ConcurrentUsageState {
        vector<DirUse> channel;
        vector<double> softFT;
    };

    auto residualNetsForCandidate = [&](const Design& routed, const ConcurrentCandidate& cand,
        const ConcurrentUsageState& state, bool allowChannelOverflow) {
            double limit = 1.0e9;
            for (const auto& kv : cand.unitUse.byChannel) {
                const int ci = kv.first;
                if (ci < 0 || ci >= static_cast<int>(routed.channels.size()) || ci >= static_cast<int>(state.channel.size())) continue;
                const Channel& ch = routed.channels[ci];
                if (!allowChannelOverflow) {
                    if (kv.second.lr > EPS) limit = min(limit, (capLR(ch) - state.channel[ci].lr) / kv.second.lr);
                    if (kv.second.tb > EPS) limit = min(limit, (capTB(ch) - state.channel[ci].tb) / kv.second.tb);
                }
            }
            for (const auto& kv : cand.softCounts) {
                const int bi = kv.first;
                if (bi < 0 || bi >= static_cast<int>(routed.blocks.size()) || bi >= static_cast<int>(state.softFT.size()) || kv.second <= 0) return 0;
                const BlockInst& b = routed.blocks[bi];
                const double cap = static_cast<double>(softFTCapacityNetsLocal(b));
                limit = min(limit, (cap - state.softFT[bi]) / static_cast<double>(kv.second));
            }
            if (!std::isfinite(limit)) return 0;
            if (limit >= 100000000.0) return 100000000;
            return max(0, static_cast<int>(floor(limit + 1e-7)));
        };

    auto commitConcurrentPath = [&](Design& routed, ConcurrentUsageState& state, const ConcurrentCandidate& cand, int nets) {
        if (nets <= 0) return;
        RoutePath p = cand.unitPath;
        p.netCount = nets;
        p.wireLength = calcRouteWireLength(routed, p);
        routed.routes.push_back(p);
        const double dn = static_cast<double>(nets);
        for (const auto& kv : cand.unitUse.byChannel) {
            const int ci = kv.first;
            if (ci < 0 || ci >= static_cast<int>(state.channel.size())) continue;
            state.channel[ci].lr += kv.second.lr * dn;
            state.channel[ci].tb += kv.second.tb * dn;
        }
        for (const auto& kv : cand.softCounts) {
            const int bi = kv.first;
            if (bi < 0 || bi >= static_cast<int>(state.softFT.size())) continue;
            state.softFT[bi] += dn * static_cast<double>(kv.second);
        }
        };

    auto runConcurrentAllocationRouter = [&]() {
        Design seed = design;
        seed.routes.clear();
        recomputeRouterUsageFields(seed);
        clearDirectionalUseCache();

        vector<ConcurrentPool> pools;
        pools.reserve(conns.size());
        for (const Connection& conn : conns) {
            ConcurrentPool pool;
            pool.conn = conn;
            addConcurrentCandidate(pool, seed, conn, RouteMode::CHANNEL_ONLY, false, CAP_SCALE_FULL, false);
            if (!pool.candidates.empty()) {
                addConcurrentBiasedCandidate(pool, seed, conn, RouteMode::CHANNEL_ONLY, false, pool.candidates.front().unitUse);
            }
            const int ftAnchor = static_cast<int>(pool.candidates.size());
            addConcurrentCandidate(pool, seed, conn, RouteMode::FT_ENABLED, false, CAP_SCALE_FULL, true);
            if (static_cast<int>(pool.candidates.size()) > ftAnchor) {
                addConcurrentBiasedCandidate(pool, seed, conn, RouteMode::FT_ENABLED, true, pool.candidates[ftAnchor].unitUse);
            }
            if (pool.candidates.empty()) return false;
            pools.push_back(std::move(pool));
        }

        vector<array<double, 2>> channelPrice(seed.channels.size(), { 0.0, 0.0 });
        vector<double> softPrice(seed.blocks.size(), 0.0);

        for (int iter = 0; iter < CONCURRENT_ALLOC_ITERS; ++iter) {
            vector<DirUse> projectedChannel(seed.channels.size());
            vector<double> projectedSoft(seed.blocks.size(), 0.0);

            for (ConcurrentPool& pool : pools) {
                double bestScore = numeric_limits<double>::infinity();
                for (ConcurrentCandidate& cand : pool.candidates) {
                    cand.baseCost = max(1.0, cand.baseCost);
                    const double score = concurrentCandidateScore(cand, channelPrice, softPrice);
                    cand.flow = score;
                    bestScore = min(bestScore, score);
                }

                double weightSum = 0.0;
                for (ConcurrentCandidate& cand : pool.candidates) {
                    const double x = max(-60.0, min(0.0, (bestScore - cand.flow) / CONCURRENT_SOFTMAX_TEMP));
                    cand.flow = exp(x);
                    weightSum += cand.flow;
                }
                if (weightSum <= EPS) {
                    int bestIdx = 0;
                    for (int i = 1; i < static_cast<int>(pool.candidates.size()); ++i) {
                        if (pool.candidates[i].baseCost < pool.candidates[bestIdx].baseCost) bestIdx = i;
                    }
                    for (int i = 0; i < static_cast<int>(pool.candidates.size()); ++i) {
                        pool.candidates[i].flow = (i == bestIdx) ? static_cast<double>(pool.conn.netCount) : 0.0;
                    }
                }
                else {
                    for (ConcurrentCandidate& cand : pool.candidates) {
                        cand.flow = static_cast<double>(max(0, pool.conn.netCount)) * cand.flow / weightSum;
                    }
                }

                for (const ConcurrentCandidate& cand : pool.candidates) {
                    if (cand.flow <= EPS) continue;
                    for (const auto& kv : cand.unitUse.byChannel) {
                        const int ci = kv.first;
                        if (ci < 0 || ci >= static_cast<int>(projectedChannel.size())) continue;
                        projectedChannel[ci].lr += kv.second.lr * cand.flow;
                        projectedChannel[ci].tb += kv.second.tb * cand.flow;
                    }
                    for (const auto& kv : cand.softCounts) {
                        const int bi = kv.first;
                        if (bi >= 0 && bi < static_cast<int>(projectedSoft.size())) {
                            projectedSoft[bi] += static_cast<double>(kv.second) * cand.flow;
                        }
                    }
                }
            }

            for (int ci = 0; ci < static_cast<int>(seed.channels.size()); ++ci) {
                const Channel& ch = seed.channels[ci];
                auto priceFor = [](double util) {
                    const double risk = max(0.0, util - CONCURRENT_PRICE_FREE_UTIL);
                    const double overflow = max(0.0, util - 1.0);
                    return CONCURRENT_PRICE_WEIGHT * risk * risk
                        + CONCURRENT_OVERFLOW_PRICE_WEIGHT * overflow * overflow;
                    };
                const double lrUtil = projectedChannel[ci].lr / max(1.0, capLR(ch));
                const double tbUtil = projectedChannel[ci].tb / max(1.0, capTB(ch));
                channelPrice[ci][0] = priceFor(lrUtil);
                channelPrice[ci][1] = priceFor(tbUtil);
            }

            for (int bi = 0; bi < static_cast<int>(seed.blocks.size()); ++bi) {
                const BlockInst& b = seed.blocks[bi];
                if (b.spec.type != BlockType::SOFT) {
                    softPrice[bi] = 0.0;
                    continue;
                }
                const double cap = static_cast<double>(max(0, softFTCapacityNetsLocal(b)));
                if (cap <= EPS) {
                    softPrice[bi] = projectedSoft[bi] > EPS ? 1.0e12 : 0.0;
                    continue;
                }
                const double util = projectedSoft[bi] / cap;
                const double risk = max(0.0, util - CONCURRENT_PRICE_FREE_UTIL);
                const double overflow = max(0.0, util - 1.0);
                softPrice[bi] = CONCURRENT_FT_PRICE_WEIGHT * risk * risk
                    + 10.0 * CONCURRENT_OVERFLOW_PRICE_WEIGHT * overflow * overflow;
            }
        }

        Design routed = seed;
        routed.routes.clear();
        ConcurrentUsageState routedUsage;
        routedUsage.channel.assign(seed.channels.size(), DirUse{});
        routedUsage.softFT.assign(seed.blocks.size(), 0.0);
        recomputeRouterUsageFields(routed);
        clearDirectionalUseCache();

        for (ConcurrentPool& pool : pools) {
            int remaining = max(0, pool.conn.netCount);
            vector<int> order;
            for (int i = 0; i < static_cast<int>(pool.candidates.size()); ++i) order.push_back(i);
            sort(order.begin(), order.end(), [&](int a, int b) {
                if (fabs(pool.candidates[a].flow - pool.candidates[b].flow) > 1e-9) return pool.candidates[a].flow > pool.candidates[b].flow;
                return pool.candidates[a].baseCost < pool.candidates[b].baseCost;
                });

            for (int oi = 0; oi < static_cast<int>(order.size()) && remaining > 0; ++oi) {
                ConcurrentCandidate& cand = pool.candidates[order[oi]];
                const int residual = residualNetsForCandidate(routed, cand, routedUsage, false);
                if (residual <= 0) continue;

                int desired = static_cast<int>(floor(cand.flow + 0.5));
                if (oi >= CONCURRENT_ROUND_TOPK || desired <= 0) desired = remaining;
                const int take = min(remaining, min(desired, residual));
                if (take <= 0) continue;
                commitConcurrentPath(routed, routedUsage, cand, take);
                remaining -= take;
            }

            if (remaining > 0) {
                if (ROUTER_DIAG_ENABLE) {
                    cerr << "[RouterConcurrent] reject=unallocated_demand "
                        << seed.blocks[pool.conn.src].spec.name << "->" << seed.blocks[pool.conn.dst].spec.name
                        << " remaining=" << remaining << "\n";
                }
                return false;
            }
        }

        recomputeRouterUsageFields(routed);
        const RouterScore score = scoreRouterSolution(routed);
        if (score.open != 0 || score.channelOverflow > EPS || score.ftOverflow > EPS) return false;
        design = std::move(routed);
        clearDirectionalUseCache();
        if (ROUTER_DIAG_ENABLE) {
            cerr << fixed << setprecision(3)
                << "[RouterConcurrent] accepted=Y routes=" << design.routes.size()
                << " chOv=" << score.channelOverflow
                << " ftOv=" << score.ftOverflow
                << " wl=" << score.wireLength
                << "\n";
        }
        return true;
        };

    bool usedConcurrentRouter = false;
    const bool useConcurrentForThisDesign = ENABLE_CONCURRENT_ALLOC_ROUTER && design.blocks.size() <= 32;
    if (useConcurrentForThisDesign) {
        usedConcurrentRouter = runConcurrentAllocationRouter();
    }

    if (!usedConcurrentRouter) {
    int strictAccepted = 0;
    for (const auto& conn : conns) {
        if (tryStrictStep1To4(design, conn)) ++strictAccepted;
        else if (tryTargetedBlockerExchange(design, conn)) ++strictAccepted;
        else {
            deferred.push_back({ conn });
            if (ROUTER_DIAG_ENABLE && ROUTER_DIAG_PRINT_DECISIONS) {
                cerr << "[RouterRefined][DEFER_STEP5] " << design.blocks[conn.src].spec.name << "->" << design.blocks[conn.dst].spec.name
                    << " nets=" << conn.netCount << " reason=strict directional Step1-4 failed; defer Step5\n";
            }
        }
    }

    if (ROUTER_DIAG_ENABLE) {
        cerr << "[RouterRefined][DEFER_SUMMARY] strictAccepted=" << strictAccepted
            << " deferred=" << deferred.size()
            << " routesBeforeStep5=" << design.routes.size()
            << " overflowBeforeStep5=" << totalChannelOverflowNow(design) << "\n";
    }

    int step5Accepted = 0, step5Open = 0;
    const double overflowBeforeStep5 = totalChannelOverflowNow(design);
    const double wlBeforeStep5 = totalRouteWireLengthNow(design);

    for (const auto& item : deferred) {
        const Connection& conn = item.conn;
        RoutePath step5 = routeWithPolicy(design, conn, RouteMode::CHANNEL_ONLY, true, CAP_SCALE_FULL, false);
        RouteMetrics m = analyzeRoute(design, step5);
        printDecision("STEP5_DEFERRED_CH_WHOLE_ALLOW_OVERFLOW", design, conn, step5);

        RoutePath step5Ft = routeWithPolicy(design, conn, RouteMode::FT_ENABLED, true, CAP_SCALE_FULL, false, true);
        RouteMetrics ftM = analyzeRoute(design, step5Ft);
        printDecision("STEP5_DEFERRED_FT_FORCED_ALLOW_OVERFLOW", design, conn, step5Ft);

        bool haveLegalStep5 = false;
        RoutePath legalStep5;
        RouteMetrics legalStep5M;
        auto considerLegalStep5 = [&](const RoutePath& p, const RouteMetrics& pm) {
            if (pm.open || pm.channelOverflowIncrement > EPS || pm.ftOverflowAfterTouched > EPS) return;
            if (!haveLegalStep5 || routeChoiceScore(pm) < routeChoiceScore(legalStep5M)) {
                legalStep5 = p;
                legalStep5M = pm;
                haveLegalStep5 = true;
            }
            };
        considerLegalStep5(step5, m);
        if (ftM.usesSoftFT) considerLegalStep5(step5Ft, ftM);
        if (haveLegalStep5) {
            applyPath(design, legalStep5);
            ++step5Accepted;
            continue;
        }

        if (ENABLE_STEP5_OVERFLOW_COMMIT && (!m.open || !ftM.open)) {
            const bool useFT = shouldUseOverflowFTCandidate(m, ftM);
            const RoutePath& chosen = useFT ? step5Ft : step5;
            const RouteMetrics& chosenM = useFT ? ftM : m;
            applyPath(design, chosen);
            ++step5Accepted;
            if (ROUTER_DIAG_ENABLE && ROUTER_DIAG_PRINT_DECISIONS) {
                cerr << "[RouterRefined][STEP5_DEFERRED_ACCEPT] " << design.blocks[conn.src].spec.name << "->" << design.blocks[conn.dst].spec.name
                    << " nets=" << conn.netCount << " ovInc=" << chosenM.channelOverflowIncrement
                    << " maxUtilComp=" << (chosenM.maxUtilComponent.empty() ? "NA" : chosenM.maxUtilComponent)
                    << " usesFT=" << (chosenM.usesSoftFT ? "Y" : "N")
                    << " wl=" << chosenM.wireLength << " note=deferred overflow fallback\n";
            }
            continue;
        }

        addFailureCertificate(conn, step5, m, step5Ft, ftM);

        printOpenAnalysis(design, conn, step5);
        RoutePath open = makeOpenRoute(design.blocks[conn.src], design.blocks[conn.dst], conn.netCount);
        if (ROUTER_DIAG_ENABLE && ROUTER_DIAG_PRINT_DECISIONS) {
            cerr << "[RouterRefined][OPEN_FALLBACK_AFTER_STEP5] " << design.blocks[conn.src].spec.name << "->" << design.blocks[conn.dst].spec.name
                << " nets=" << conn.netCount << " reason=strict Step1-4 and directional Step5 all failed\n";
        }
        applyPath(design, open);
        ++step5Open;
    }
    if (ROUTER_DIAG_ENABLE) {
        const double overflowAfterStep5 = totalChannelOverflowNow(design);
        const double wlAfterStep5 = totalRouteWireLengthNow(design);
        cerr << "[RouterRefined][DEFERRED_STEP5_SUMMARY] deferred=" << deferred.size()
            << " accepted=" << step5Accepted
            << " open=" << step5Open
            << " step5ChannelOverflowIncrease=" << max(0.0, overflowAfterStep5 - overflowBeforeStep5)
            << " step5WLIncrease=" << (wlAfterStep5 - wlBeforeStep5)
            << " finalOverflow=" << overflowAfterStep5
            << " finalWL=" << wlAfterStep5 << "\n";
    }
    }

    auto routeOverflowFallbackForRipup = [&](Design& d, const Connection& conn) {
        if (!ENABLE_STEP5_OVERFLOW_COMMIT) return false;
        RoutePath step5 = routeWithPolicy(d, conn, RouteMode::CHANNEL_ONLY, true, CAP_SCALE_FULL, false);
        RouteMetrics m = analyzeRoute(d, step5);
        RoutePath step5Ft = routeWithPolicy(d, conn, RouteMode::FT_ENABLED, true, CAP_SCALE_FULL, false, true);
        RouteMetrics ftM = analyzeRoute(d, step5Ft);

        if (!m.open || !ftM.open) {
            const bool useFT = shouldUseOverflowFTCandidate(m, ftM);
            applyPath(d, useFT ? step5Ft : step5);
            return true;
        }

        return false;
        };

    auto tryCongestionRipup = [&]() {
        if (!ENABLE_CONGESTION_RIPUP) return false;
        recomputeRouterUsageFields(design);
        const RouterScore baseScore = scoreRouterSolution(design);
        const bool legalRiskMode = baseScore.open == 0 && baseScore.channelOverflow <= EPS && baseScore.ftOverflow <= EPS;
        if (legalRiskMode) return false;
        const CongestionRiskScore baseRisk = scoreCongestionRisk(design);
        g_negotiatedChannelHistory.assign(design.channels.size(), { 0.0, 0.0 });

        const int n = static_cast<int>(design.blocks.size());
        const int maxGroups = legalRiskMode ? max(8, (n >= 45 ? RIPUP_MAX_GROUPS_LARGE : RIPUP_MAX_GROUPS_MID) / 2)
            : (n >= 45 ? RIPUP_MAX_GROUPS_LARGE : RIPUP_MAX_GROUPS_MID);
        vector<RipupGroup> groups = selectRipupGroups(design, maxGroups, legalRiskMode);
        if (groups.empty()) return false;

        set<RipupKey> selected;
        for (const auto& g : groups) selected.insert(RipupKey{ g.conn.src, g.conn.dst });

        map<RipupKey, int> selectedDemand;
        for (const auto& p : design.routes) {
            auto sit = design.blockNameToIndex.find(p.srcBlock);
            auto dit = design.blockNameToIndex.find(p.dstBlock);
            if (sit == design.blockNameToIndex.end() || dit == design.blockNameToIndex.end()) continue;
            RipupKey key{ sit->second, dit->second };
            if (selected.count(key)) selectedDemand[key] += max(0, p.netCount);
        }
        for (auto& g : groups) {
            RipupKey key{ g.conn.src, g.conn.dst };
            auto it = selectedDemand.find(key);
            if (it != selectedDemand.end()) g.conn.netCount = it->second;
        }

        Design trial = design;
        vector<RoutePath> kept;
        kept.reserve(trial.routes.size());
        for (const auto& p : trial.routes) {
            if (!routeMatchesKey(trial, p, selected)) kept.push_back(p);
        }
        trial.routes = std::move(kept);
        recomputeRouterUsageFields(trial);

        g_hotChannelBias = buildHotChannelBias(design, legalRiskMode);
        g_hotSoftBias = buildHotSoftBias(design);
        g_congestionRerouteMode = true;
        g_congestionBiasWeight = n >= 45 ? 8.0e6 : 4.0e6;

        int strictOk = 0;
        int fallbackOk = 0;
        bool ripupRouteFailed = false;
        for (const auto& g : groups) {
            if (tryStrictStep1To4(trial, g.conn)) {
                ++strictOk;
            }
            else if (routeOverflowFallbackForRipup(trial, g.conn)) {
                ++fallbackOk;
            }
            else {
                ripupRouteFailed = true;
                break;
            }
        }

        g_congestionRerouteMode = false;
        g_congestionBiasWeight = 0.0;
        g_hotChannelBias.clear();
        g_hotSoftBias.clear();

        recomputeRouterUsageFields(trial);
        const RouterScore trialScore = scoreRouterSolution(trial);
        const CongestionRiskScore trialRisk = scoreCongestionRisk(trial);
        const bool acceptRipup = !ripupRouteFailed && (betterRouterScore(trialScore, baseScore)
            || (legalRiskMode && legalCongestionRiskImproved(baseScore, baseRisk, trialScore, trialRisk)));
        if (acceptRipup) {
            if (ROUTER_DIAG_ENABLE) {
                cerr << fixed << setprecision(3)
                    << "[RouterRefined][RIPUP_ACCEPT] groups=" << groups.size()
                    << " strict=" << strictOk
                    << " fallback=" << fallbackOk
                    << " channelOv=" << baseScore.channelOverflow << "->" << trialScore.channelOverflow
                    << " ftOv=" << baseScore.ftOverflow << "->" << trialScore.ftOverflow
                    << " wl=" << baseScore.wireLength << "->" << trialScore.wireLength << "\n";
            }
            design = std::move(trial);
            return true;
        }

        if (ROUTER_DIAG_ENABLE) {
            cerr << fixed << setprecision(3)
                << "[RouterRefined][RIPUP_REJECT] groups=" << groups.size()
                << " strict=" << strictOk
                << " fallback=" << fallbackOk
                << " channelOv=" << baseScore.channelOverflow << "->" << trialScore.channelOverflow
                << " ftOv=" << baseScore.ftOverflow << "->" << trialScore.ftOverflow
                << " wl=" << baseScore.wireLength << "->" << trialScore.wireLength << "\n";
        }
        return false;
        };

    auto tryWirePolishRoute = [&](Design& d, const Connection& conn) {
        RoutePath ch = routeWithWirePolish(d, conn, RouteMode::CHANNEL_ONLY, false);
        RouteMetrics chM = analyzeRoute(d, ch);

        RoutePath ft = routeWithWirePolish(d, conn, RouteMode::FT_ENABLED, true);
        RouteMetrics ftM = analyzeRoute(d, ft);

        bool have = false;
        RoutePath best;
        RouteMetrics bestM;
        auto consider = [&](RoutePath& p, const RouteMetrics& m) {
            if (m.open || m.channelOverflowIncrement > EPS || m.ftOverflowAfterTouched > EPS) return;
            if (routeHasIllegalIntermediateBlock(d, p)) return;
            if (!have || m.wireLength + 1.0 < bestM.wireLength) {
                best = p;
                bestM = m;
                have = true;
            }
            };
        consider(ch, chM);
        if (ftM.usesSoftFT) consider(ft, ftM);
        if (!have) return false;
        applyPath(d, best);
        return true;
        };

    auto tryLegalWirelengthExchange = [&]() {
        recomputeRouterUsageFields(design);
        RouterScore baseScore = scoreRouterSolution(design);
        if (baseScore.open != 0 || baseScore.channelOverflow > EPS || baseScore.ftOverflow > EPS) return false;

        struct ExchangePath {
            int routeIndex = -1;
            RipupKey key;
            int demand = 0;
            double wl = 0.0;
        };
        vector<ExchangePath> paths;
        paths.reserve(design.routes.size());
        for (int pi = 0; pi < static_cast<int>(design.routes.size()); ++pi) {
            const auto& p = design.routes[pi];
            if (p.open || p.netCount <= 0 || p.wireLength <= 0.0) continue;
            auto sit = design.blockNameToIndex.find(p.srcBlock);
            auto dit = design.blockNameToIndex.find(p.dstBlock);
            if (sit == design.blockNameToIndex.end() || dit == design.blockNameToIndex.end()) continue;
            paths.push_back({ pi, RipupKey{ sit->second, dit->second }, max(0, p.netCount), max(0.0, p.wireLength) });
        }
        sort(paths.begin(), paths.end(), [](const ExchangePath& a, const ExchangePath& b) {
            if (fabs(a.wl - b.wl) > 1.0) return a.wl > b.wl;
            return a.demand > b.demand;
            });

        const int pathLimit = min(48, static_cast<int>(paths.size()));
        for (int pi = 0; pi < pathLimit; ++pi) {
            const ExchangePath& cand = paths[pi];
            if (cand.routeIndex < 0 || cand.routeIndex >= static_cast<int>(design.routes.size())) continue;

            Design trial = design;
            trial.routes.erase(trial.routes.begin() + cand.routeIndex);
            recomputeRouterUsageFields(trial);

            Connection cc;
            cc.src = cand.key.src;
            cc.dst = cand.key.dst;
            cc.netCount = cand.demand;
            if (!tryWirePolishRoute(trial, cc)) continue;

            recomputeRouterUsageFields(trial);
            RouterScore trialScore = scoreRouterSolution(trial);
            if (trialScore.open != 0 || trialScore.channelOverflow > EPS || trialScore.ftOverflow > EPS) continue;
            if (trialScore.wireLength + 1000.0 >= baseScore.wireLength) continue;

            design = std::move(trial);
            return true;
        }

        struct ExchangeGroup {
            RipupKey key;
            int demand = 0;
            double wl = 0.0;
        };
        map<RipupKey, ExchangeGroup> groupMap;
        for (const auto& p : design.routes) {
            if (p.open || p.netCount <= 0) continue;
            auto sit = design.blockNameToIndex.find(p.srcBlock);
            auto dit = design.blockNameToIndex.find(p.dstBlock);
            if (sit == design.blockNameToIndex.end() || dit == design.blockNameToIndex.end()) continue;
            RipupKey key{ sit->second, dit->second };
            auto& g = groupMap[key];
            g.key = key;
            g.demand += max(0, p.netCount);
            g.wl += max(0.0, p.wireLength);
        }

        vector<ExchangeGroup> groups;
        for (const auto& kv : groupMap) {
            if (kv.second.demand > 0) groups.push_back(kv.second);
        }
        sort(groups.begin(), groups.end(), [](const ExchangeGroup& a, const ExchangeGroup& b) {
            if (fabs(a.wl - b.wl) > 1.0) return a.wl > b.wl;
            return a.demand > b.demand;
            });

        const int limit = min(24, static_cast<int>(groups.size()));
        bool improved = false;
        for (int gi = 0; gi < limit; ++gi) {
            const ExchangeGroup& g = groups[gi];
            Design trial = design;
            vector<RoutePath> kept;
            kept.reserve(trial.routes.size());
            set<RipupKey> selected;
            selected.insert(g.key);
            for (const auto& p : trial.routes) {
                if (!routeMatchesKey(trial, p, selected)) kept.push_back(p);
            }
            trial.routes = std::move(kept);
            recomputeRouterUsageFields(trial);

            Connection cc;
            cc.src = g.key.src;
            cc.dst = g.key.dst;
            cc.netCount = g.demand;
            if (!tryWirePolishRoute(trial, cc)) continue;

            recomputeRouterUsageFields(trial);
            RouterScore trialScore = scoreRouterSolution(trial);
            if (trialScore.open != 0 || trialScore.channelOverflow > EPS || trialScore.ftOverflow > EPS) continue;
            if (trialScore.wireLength + 1000.0 >= baseScore.wireLength) continue;

            design = std::move(trial);
            baseScore = trialScore;
            improved = true;
        }
        return improved;
        };

    auto repairConnectionCoverage = [&]() {
        const int n = static_cast<int>(design.blocks.size());
        if (n <= 0) return;
        vector<vector<int>> expected(n, vector<int>(n, 0));
        auto addDemand = [&](vector<vector<int>>& m, int a, int b, int nets) {
            if (nets <= 0 || a < 0 || b < 0 || a >= n || b >= n) return;
            if (b < a) swap(a, b);
            m[a][b] += nets;
            };

        if (static_cast<int>(design.connMatrix.size()) == n) {
            for (int i = 0; i < n; ++i) {
                if (static_cast<int>(design.connMatrix[i].size()) != n) continue;
                for (int j = 0; j < n; ++j) addDemand(expected, i, j, max(0, design.connMatrix[i][j]));
            }
        }
        else {
            for (const auto& c : design.connections) addDemand(expected, c.src, c.dst, c.netCount);
        }

        auto actualDemand = [&]() {
            vector<vector<int>> actual(n, vector<int>(n, 0));
            for (const auto& p : design.routes) {
                if (p.open || p.netCount <= 0 || p.steps.size() < 2) continue;
                auto sit = design.blockNameToIndex.find(p.steps.front().rectName);
                auto dit = design.blockNameToIndex.find(p.steps.back().rectName);
                if (sit == design.blockNameToIndex.end() || dit == design.blockNameToIndex.end()) continue;
                addDemand(actual, sit->second, dit->second, p.netCount);
            }
            return actual;
            };

        int repaired = 0;
        for (int pass = 0; pass < 2; ++pass) {
            vector<vector<int>> actual = actualDemand();
            bool changed = false;
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    int missing = expected[i][j] - actual[i][j];
                    if (missing <= 0) continue;
                    Connection cc;
                    cc.src = i;
                    cc.dst = j;
                    cc.netCount = missing;
                    if (tryStrictStep1To4(design, cc)) {
                        ++repaired;
                        changed = true;
                    }
                    else {
                        RoutePath step5 = routeWithPolicy(design, cc, RouteMode::CHANNEL_ONLY, true, CAP_SCALE_FULL, false);
                        RouteMetrics m = analyzeRoute(design, step5);
                        RoutePath step5Ft = routeWithPolicy(design, cc, RouteMode::FT_ENABLED, true, CAP_SCALE_FULL, false, true);
                        RouteMetrics ftM = analyzeRoute(design, step5Ft);

                        bool haveLegalStep5 = false;
                        RoutePath legalStep5;
                        RouteMetrics legalStep5M;
                        auto considerLegalStep5 = [&](const RoutePath& p, const RouteMetrics& pm) {
                            if (pm.open || pm.channelOverflowIncrement > EPS || pm.ftOverflowAfterTouched > EPS) return;
                            if (!haveLegalStep5 || routeChoiceScore(pm) < routeChoiceScore(legalStep5M)) {
                                legalStep5 = p;
                                legalStep5M = pm;
                                haveLegalStep5 = true;
                            }
                            };
                        considerLegalStep5(step5, m);
                        if (ftM.usesSoftFT) considerLegalStep5(step5Ft, ftM);
                        if (haveLegalStep5) {
                            applyPath(design, legalStep5);
                            ++repaired;
                            changed = true;
                        }
                        else {
                            addFailureCertificate(cc, step5, m, step5Ft, ftM);
                        }
                    }
                }
            }
            if (!changed) break;
        }
        if (ROUTER_DIAG_ENABLE && repaired > 0) {
            recomputeRouterUsageFields(design);
            cerr << "[RouterRefined][COVERAGE_REPAIR] repaired=" << repaired
                << " finalOverflow=" << totalChannelOverflowNow(design)
                << " finalWL=" << totalRouteWireLengthNow(design) << "\n";
        }
        };
    if (!usedConcurrentRouter) {
        g_negotiatedChannelHistory.assign(design.channels.size(), { 0.0, 0.0 });
        for (int ripupPass = 0; ripupPass < 3; ++ripupPass) {
            if (!tryCongestionRipup()) break;
        }
        g_negotiatedChannelHistory.clear();
        repairConnectionCoverage();
        for (int wlPolishPass = 0; wlPolishPass < 6; ++wlPolishPass) {
            if (!tryLegalWirelengthExchange()) break;
        }
    }
    printFinalDiagnosis(design);
}

// -----------------------------------------------------------------------------
// Node and graph construction
// -----------------------------------------------------------------------------
// buildNodes():
//   將所有 block 與 channel 都轉成 Dijkstra graph 的 node。
//   node.index 的意義依 isBlock 決定：
//     isBlock=true  -> index 對應 design.blocks[index]
//     isBlock=false -> index 對應 design.channels[index]
//
// buildGraph():
//   對每一對 node 檢查它們的 rectangle 是否有合法 edge contact。
//   若兩個 rectangle 貼邊且投影 overlap > TOUCH_OVERLAP_EPS，就建立雙向邊。
//   AdjEdge 會記錄：
//     to       : 鄰居 node id
//     edgeFrom : 從目前 node 的哪個 edge 離開
//     edgeTo   : 進入鄰居 node 的哪個 edge
//     baseCost : 幾何距離近似，供 Dijkstra wireCost 使用
//
// touchWithEdges():
//   負責判斷四種接觸：right-left、left-right、top-bottom、bottom-top。
//   edge 編號遵循題目：1=left, 2=top, 3=right, 4=bottom。
// -----------------------------------------------------------------------------
vector<Router::Node> Router::buildNodes(const Design& design) const {
    vector<Node> nodes;
    for (int i = 0; i < static_cast<int>(design.blocks.size()); ++i) {
        nodes.push_back({ design.blocks[i].spec.name, design.blocks[i].rect, true, i });
    }
    for (int i = 0; i < static_cast<int>(design.channels.size()); ++i) {
        nodes.push_back({ design.channels[i].name, design.channels[i].rect, false, i });
    }
    return nodes;
}

bool Router::touchWithEdges(const Rect& a, const Rect& b, int& edgeA, int& edgeB) const {
    if (fabs(rectRight(a) - b.x) < TOUCH_EPS &&
        overlapLen(a.y, rectTop(a), b.y, rectTop(b)) > TOUCH_OVERLAP_EPS) {
        edgeA = 3; edgeB = 1; return true;
    }
    if (fabs(a.x - rectRight(b)) < TOUCH_EPS &&
        overlapLen(a.y, rectTop(a), b.y, rectTop(b)) > TOUCH_OVERLAP_EPS) {
        edgeA = 1; edgeB = 3; return true;
    }
    if (fabs(rectTop(a) - b.y) < TOUCH_EPS &&
        overlapLen(a.x, rectRight(a), b.x, rectRight(b)) > TOUCH_OVERLAP_EPS) {
        edgeA = 2; edgeB = 4; return true;
    }
    if (fabs(a.y - rectTop(b)) < TOUCH_EPS &&
        overlapLen(a.x, rectRight(a), b.x, rectRight(b)) > TOUCH_OVERLAP_EPS) {
        edgeA = 4; edgeB = 2; return true;
    }
    return false;
}

vector<vector<Router::AdjEdge>> Router::buildGraph(const Design& design, const vector<Node>& nodes) const {
    (void)design;
    const int N = static_cast<int>(nodes.size());
    vector<vector<AdjEdge>> g(N);
    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            int ei = 0, ej = 0;
            if (!touchWithEdges(nodes[i].rect, nodes[j].rect, ei, ej)) continue;
            const double base = manhattan(rectCx(nodes[i].rect), rectCy(nodes[i].rect), rectCx(nodes[j].rect), rectCy(nodes[j].rect));
            g[i].push_back({ j, ei, ej, base });
            g[j].push_back({ i, ej, ei, base });
        }
    }
    return g;
}

bool Router::nodeAllowedAsIntermediate(const Design& design, const Node& node) const {
    if (!node.isBlock) return true;
    if (g_routeMode == RouteMode::CHANNEL_ONLY) return false;
    const BlockInst& b = design.blocks[node.index];
    return b.spec.type == BlockType::SOFT;
}

// Legacy interface.  Directional Dijkstra no longer uses nodePenalty for channels,
// because channel resource cost depends on entry edge and exit edge.  Keep a safe
// implementation for compatibility with Router.hpp.
double Router::nodePenalty(const Design& design, const Node& node, int netCount) const {
    if (!node.isBlock) {
        (void)design; (void)node; (void)netCount;
        return 0.0;
    }
    const BlockInst& b = design.blocks[node.index];
    if (b.spec.type == BlockType::SOFT && g_routeMode == RouteMode::FT_ENABLED) {
        return softFTPenaltyLocal(b, netCount);
    }
    return numeric_limits<double>::infinity();
}

// -----------------------------------------------------------------------------
// Directional edge-state Dijkstra
// -----------------------------------------------------------------------------
// 這是本 Router 最重要的函式：對「一組 connection」用 Dijkstra 找路。
//
// 為什麼不是普通 Dijkstra？
//   普通 Dijkstra state 只有 nodeId，例如 CH10。這樣只知道路徑經過 CH10，
//   但不知道在 CH10 內是 left->right、top->bottom，還是 L-shape turn。
//   方向容量必須知道 inEdge/outEdge，所以本版 state 改為：
//       state = (nodeId, inEdge)
//   其中 inEdge=0 只代表 source 起點尚未進入任何 rectangle。
//
// Dijkstra transition 的意義：
//   目前 state 在 node u，且 uIn 表示「進入 u 的 edge」。
//   嘗試走到鄰居 v 時，AdjEdge 會提供：
//       e.edgeFrom = 從 u 的哪個 edge 離開
//       e.edgeTo   = 從 v 的哪個 edge 進入
//   若 u 是中繼 channel，uIn + e.edgeFrom 就形成一次 channel 內部 traversal，
//   此時立刻計算該 traversal 對 LR/TB 容量的使用與 cost。
//
// Cost 細節：
//   wireCost = ROUTER_WIRE_WEIGHT * e.baseCost * netCount
//     e.baseCost 目前使用兩個 rectangle center 的 Manhattan 距離，屬於 global
//     routing 近似；最後輸出的 wireLength 會由 calcRouteWireLength() 重新計算。
//   resourcePenalty：
//     - 若中繼是 channel：呼叫 directionalChannelPenalty()，根據 in/out edge
//       決定吃 LR、TB 或兩者，並檢查 capacity reservation / split guard / Step5 overflow。
//     - 若中繼是 soft block：在 FT_ENABLED mode 下允許，並加入 softFTPenaltyLocal()。
//     - hard / edge block 不能當中繼。
//
// 輸出 path 格式：
//   題目要求除了起點與終點之外，中間每個 rectangle 都必須一進一出。
//   本函式在回溯 parent state 時，對每一段 u->v 輸出：
//       {u, edgeOutOfU}, {v, edgeIntoV}
//   因此中間 rectangle 會自然形成：CHx in, CHx out 的 pair 格式。
// -----------------------------------------------------------------------------
RoutePath Router::routeOneConnection(const Design& design, const Connection& conn) const {
    RoutePath result;
    result.netCount = conn.netCount;
    result.srcBlock = design.blocks[conn.src].spec.name;
    result.dstBlock = design.blocks[conn.dst].spec.name;

    vector<Node> localNodes;
    vector<vector<AdjEdge>> localGraph;
    const vector<Node>* nodesPtr = &routeGraphNodes;
    const vector<vector<AdjEdge>>* graphPtr = &routeGraphAdj;
    if (!routeGraphCacheValid) {
        localNodes = buildNodes(design);
        localGraph = buildGraph(design, localNodes);
        nodesPtr = &localNodes;
        graphPtr = &localGraph;
    }
    const vector<Node>& nodes = *nodesPtr;
    const vector<vector<AdjEdge>>& g = *graphPtr;
    const vector<DirUse>& currentUse = cachedDirectionalChannelUseFromRoutes(design);

    const int srcNode = conn.src;
    const int dstNode = conn.dst;
    const int N = static_cast<int>(nodes.size());

    if (!g_contactAwareCostEnabled) {
        const int SOFT_STATE_COUNT = 2;
        const int S = N * EDGE_STATE_COUNT * SOFT_STATE_COUNT;

        auto sid = [&](int node, int inEdge, int softSeen) {
            return (node * EDGE_STATE_COUNT + inEdge) * SOFT_STATE_COUNT + softSeen;
            };
        auto sSoft = [&](int state) { return state % SOFT_STATE_COUNT; };
        auto sEdge = [&](int state) { return (state / SOFT_STATE_COUNT) % EDGE_STATE_COUNT; };
        auto sNode = [&](int state) { return (state / SOFT_STATE_COUNT) / EDGE_STATE_COUNT; };

        auto edgePairWireCostLocal = [&](const Rect& r, int inEdge, int outEdge) {
            if (!validEdgeLocal(inEdge) || !validEdgeLocal(outEdge)) return 0.0;
            auto p = edgeCenterPoint(r, inEdge);
            auto q = edgeCenterPoint(r, outEdge);
            return manhattan(p.first, p.second, q.first, q.second);
            };

        vector<double> dist(S, numeric_limits<double>::infinity());        vector<int> parent(S, -1);
        vector<int> transEdgeOut(S, 0);
        vector<int> transEdgeIn(S, 0);

        using P = pair<double, int>;
        priority_queue<P, vector<P>, greater<P>> pq;

        const int start = sid(srcNode, 0, 0);
        dist[start] = 0.0;
        pq.push({ 0.0, start });

        int bestDst = -1;
        while (!pq.empty()) {
            auto [queuedCost, st] = pq.top(); pq.pop();
            if (queuedCost > dist[st] + 1e-9) continue;

            const int u = sNode(st);
            const int uIn = sEdge(st);
            const int uSoftSeen = sSoft(st);
            const double d = dist[st];

            if (u == dstNode && (!g_requireSoftFT || uSoftSeen != 0)) { bestDst = st; break; }

            for (const AdjEdge& e : g[u]) {
                const int v = e.to;
                const bool vEndpoint = (v == srcNode || v == dstNode);
                if (!vEndpoint && !nodeAllowedAsIntermediate(design, nodes[v])) continue;
                if (u == srcNode && !allowsPortEdgeLocal(design.blocks[conn.src].spec, e.edgeFrom)) continue;
                if (v == dstNode && !allowsPortEdgeLocal(design.blocks[conn.dst].spec, e.edgeTo)) continue;

                double resourcePenalty = 0.0;
                if (u != srcNode && u != dstNode) {
                    if (!validEdgeLocal(uIn) || uIn == e.edgeFrom) continue;

                    if (!nodes[u].isBlock) {
                        const Channel& ch = design.channels[nodes[u].index];
                        DirectionalPenaltyInfo pi = directionalChannelPenalty(ch, nodes[u].index, uIn, e.edgeFrom, conn.netCount, currentUse);
                        if (!pi.feasible || !std::isfinite(pi.penalty)) continue;
                        resourcePenalty += pi.penalty;
                    }
                    else {
                        const BlockInst& b = design.blocks[nodes[u].index];
                        if (b.spec.type != BlockType::SOFT || g_routeMode != RouteMode::FT_ENABLED) continue;
                        const double softPenalty = softFTPenaltyLocal(b, conn.netCount);
                        if (!std::isfinite(softPenalty)) continue;
                        if (!g_wirePolishMode) resourcePenalty += softPenalty;
                        if (!std::isfinite(resourcePenalty)) continue;
                        if (g_congestionRerouteMode && nodes[u].index >= 0 && nodes[u].index < static_cast<int>(g_hotSoftBias.size())) {
                            const double hotSoft = g_hotSoftBias[nodes[u].index];
                            if (hotSoft > EPS) resourcePenalty += g_congestionBiasWeight * hotSoft * static_cast<double>(conn.netCount);
                        }
                    }
                }

                int nextSoftSeen = uSoftSeen;
                if (u != srcNode && u != dstNode && nodes[u].isBlock) {
                    const BlockInst& b = design.blocks[nodes[u].index];
                    if (b.spec.type == BlockType::SOFT && g_routeMode == RouteMode::FT_ENABLED) nextSoftSeen = 1;
                }

                double wireMetric = e.baseCost;
                const bool useEdgePairMetric = ENABLE_EDGE_PAIR_WIRE_METRIC && g_wirePolishMode;
                if (useEdgePairMetric && u != srcNode && u != dstNode) {
                    wireMetric = edgePairWireCostLocal(nodes[u].rect, uIn, e.edgeFrom);
                }
                const double wireCost = ROUTER_WIRE_WEIGHT * wireMetric * static_cast<double>(conn.netCount);
                const int next = sid(v, e.edgeTo, nextSoftSeen);
                const double nd = d + wireCost + resourcePenalty;
                if (nd < dist[next]) {
                    dist[next] = nd;
                    parent[next] = st;
                    transEdgeOut[next] = e.edgeFrom;
                    transEdgeIn[next] = e.edgeTo;
                    pq.push({ nd, next });
                }
            }
        }

        if (bestDst < 0) {
            return makeOpenRoute(design.blocks[conn.src], design.blocks[conn.dst], conn.netCount);
        }

        vector<int> states;
        for (int cur = bestDst; cur != -1; cur = parent[cur]) states.push_back(cur);
        reverse(states.begin(), states.end());

        for (int k = 1; k < static_cast<int>(states.size()); ++k) {
            const int prev = states[k - 1];
            const int cur = states[k];
            const int u = sNode(prev);
            const int v = sNode(cur);
            result.steps.push_back({ nodes[u].name, transEdgeOut[cur] });
            result.steps.push_back({ nodes[v].name, transEdgeIn[cur] });
        }

        result.open = false;
        result.wireLength = calcRouteWireLength(design, result);
        return result;
    }

    struct DirectedContact {
        int from = -1;
        int to = -1;
        int edgeFrom = 0;
        int edgeTo = 0;
        double x = 0.0;
        double y = 0.0;
    };

    vector<DirectedContact> contacts;
    vector<vector<int>> outgoing(N);
    for (int u = 0; u < N; ++u) {
        for (const AdjEdge& e : g[u]) {
            pair<double, double> gp;
            if (!contactGuidingPointLocal(nodes[u].rect, e.edgeFrom, nodes[e.to].rect, e.edgeTo, gp)) {
                auto pu = edgeCenterPoint(nodes[u].rect, e.edgeFrom);
                auto pv = edgeCenterPoint(nodes[e.to].rect, e.edgeTo);
                gp = { 0.5 * (pu.first + pv.first), 0.5 * (pu.second + pv.second) };
            }
            const int id = static_cast<int>(contacts.size());
            contacts.push_back(DirectedContact{ u, e.to, e.edgeFrom, e.edgeTo, gp.first, gp.second });
            outgoing[u].push_back(id);
        }
    }

    const int SOFT_STATE_COUNT = 2;
    const int CONTACT_STATE_COUNT = static_cast<int>(contacts.size()) + 1;
    const int S = N * CONTACT_STATE_COUNT * SOFT_STATE_COUNT;

    auto sid = [&](int node, int incomingContactState, int softSeen) {
        return (node * CONTACT_STATE_COUNT + incomingContactState) * SOFT_STATE_COUNT + softSeen;
        };
    auto sSoft = [&](int state) { return state % SOFT_STATE_COUNT; };
    auto sNode = [&](int state) { return (state / SOFT_STATE_COUNT) / CONTACT_STATE_COUNT; };
    auto sIncomingContactState = [&](int state) { return (state / SOFT_STATE_COUNT) % CONTACT_STATE_COUNT; };

    vector<double> dist(S, numeric_limits<double>::infinity());
    vector<int> parent(S, -1);

    using P = pair<double, int>;
    priority_queue<P, vector<P>, greater<P>> pq;

    const int start = sid(srcNode, 0, 0);
    dist[start] = 0.0;
    pq.push({ 0.0, start });

    int bestDst = -1;
    while (!pq.empty()) {
        auto [queuedCost, st] = pq.top(); pq.pop();
        if (queuedCost > dist[st] + 1e-9) continue;

        const int u = sNode(st);
        const int incomingState = sIncomingContactState(st);
        const int prevContactId = incomingState - 1;
        const int uIn = prevContactId >= 0 ? contacts[prevContactId].edgeTo : 0;
        const int uSoftSeen = sSoft(st);
        const double d = dist[st];

        if (u == dstNode && (!g_requireSoftFT || uSoftSeen != 0)) { bestDst = st; break; }

        for (int contactId : outgoing[u]) {
            const DirectedContact& dc = contacts[contactId];
            const int v = dc.to;
            const bool vEndpoint = (v == srcNode || v == dstNode);
            if (!vEndpoint && !nodeAllowedAsIntermediate(design, nodes[v])) continue;
            if (u == srcNode && !allowsPortEdgeLocal(design.blocks[conn.src].spec, dc.edgeFrom)) continue;
            if (v == dstNode && !allowsPortEdgeLocal(design.blocks[conn.dst].spec, dc.edgeTo)) continue;

            double resourcePenalty = 0.0;
            if (u != srcNode && u != dstNode) {
                if (!validEdgeLocal(uIn) || uIn == dc.edgeFrom) continue;

                if (!nodes[u].isBlock) {
                    const Channel& ch = design.channels[nodes[u].index];
                    DirectionalPenaltyInfo pi = directionalChannelPenalty(ch, nodes[u].index, uIn, dc.edgeFrom, conn.netCount, currentUse);
                    if (!pi.feasible || !std::isfinite(pi.penalty)) continue;
                    resourcePenalty += pi.penalty;
                }
                else {
                    const BlockInst& b = design.blocks[nodes[u].index];
                    if (b.spec.type != BlockType::SOFT || g_routeMode != RouteMode::FT_ENABLED) continue;
                    resourcePenalty += softFTPenaltyLocal(b, conn.netCount);
                    if (!std::isfinite(resourcePenalty)) continue;
                    if (g_congestionRerouteMode && nodes[u].index >= 0 && nodes[u].index < static_cast<int>(g_hotSoftBias.size())) {
                        const double hotSoft = g_hotSoftBias[nodes[u].index];
                        if (hotSoft > EPS) resourcePenalty += g_congestionBiasWeight * hotSoft * static_cast<double>(conn.netCount);
                    }
                }
            }

            int nextSoftSeen = uSoftSeen;
            if (u != srcNode && u != dstNode && nodes[u].isBlock) {
                const BlockInst& b = design.blocks[nodes[u].index];
                if (b.spec.type == BlockType::SOFT && g_routeMode == RouteMode::FT_ENABLED) nextSoftSeen = 1;
            }

            double contactWire = 0.0;
            if (prevContactId >= 0) {
                const DirectedContact& prev = contacts[prevContactId];
                contactWire = manhattan(prev.x, prev.y, dc.x, dc.y);
            }
            const double wireCost = ROUTER_WIRE_WEIGHT * contactWire * static_cast<double>(conn.netCount);
            const int next = sid(v, contactId + 1, nextSoftSeen);
            const double nd = d + wireCost + resourcePenalty;
            if (nd < dist[next]) {
                dist[next] = nd;
                parent[next] = st;
                pq.push({ nd, next });
            }
        }
    }

    if (bestDst < 0) {
        return makeOpenRoute(design.blocks[conn.src], design.blocks[conn.dst], conn.netCount);
    }

    vector<int> states;
    for (int cur = bestDst; cur != -1; cur = parent[cur]) states.push_back(cur);
    reverse(states.begin(), states.end());

    for (int k = 1; k < static_cast<int>(states.size()); ++k) {
        const int cur = states[k];
        const int contactState = sIncomingContactState(cur);
        if (contactState <= 0) continue;
        const DirectedContact& dc = contacts[contactState - 1];
        result.steps.push_back({ nodes[dc.from].name, dc.edgeFrom });
        result.steps.push_back({ nodes[dc.to].name, dc.edgeTo });
    }

    result.open = false;
    result.wireLength = calcRouteWireLength(design, result);
    return result;
}
RoutePath Router::makeOpenRoute(const BlockInst& src, const BlockInst& dst, int netCount) const {
    RoutePath p;
    p.netCount = netCount;
    p.srcBlock = src.spec.name;
    p.dstBlock = dst.spec.name;
    p.open = true;
    const int srcEdge = preferredEndpointEdgeLocal(src.spec, src.rect, dst.rect);
    int dstEdge = edgeOpposite(srcEdge);
    if (!allowsPortEdgeLocal(dst.spec, dstEdge)) {
        dstEdge = preferredEndpointEdgeLocal(dst.spec, dst.rect, src.rect);
    }
    p.steps.push_back({ src.spec.name, srcEdge });
    p.steps.push_back({ dst.spec.name, dstEdge });
    p.wireLength = calcPathWLByRects(src.rect, srcEdge, dst.rect, dstEdge) * netCount;
    return p;
}

double Router::calcPathWLByRects(const Rect& a, int ea, const Rect& b, int eb) const {
    auto pa = edgeCenterPoint(a, ea);
    auto pb = edgeCenterPoint(b, eb);
    return manhattan(pa.first, pa.second, pb.first, pb.second);
}

const Rect* Router::findRectByName(const Design& design, const string& name) const {
    for (const auto& b : design.blocks) if (b.spec.name == name) return &b.rect;
    for (const auto& ch : design.channels) if (ch.name == name) return &ch.rect;
    return nullptr;
}

double Router::calcRouteWireLength(const Design& design, const RoutePath& path) const {
    if (path.steps.size() < 2) return 0.0;

    vector<pair<double, double>> guiding;
    for (int i = 0; i + 1 < static_cast<int>(path.steps.size()); ++i) {
        const auto& a = path.steps[i];
        const auto& b = path.steps[i + 1];
        if (a.rectName == b.rectName) continue;

        const Rect* ra = findRectByName(design, a.rectName);
        const Rect* rb = findRectByName(design, b.rectName);
        if (!ra || !rb) continue;

        pair<double, double> gp;
        if (contactGuidingPointLocal(*ra, a.edge, *rb, b.edge, gp)) {
            guiding.push_back(gp);
        }
        else {
            guiding.push_back(edgeCenterPoint(*ra, a.edge));
            guiding.push_back(edgeCenterPoint(*rb, b.edge));
        }
    }

    double wlOne = 0.0;
    for (int i = 0; i + 1 < static_cast<int>(guiding.size()); ++i) {
        wlOne += manhattan(guiding[i].first, guiding[i].second, guiding[i + 1].first, guiding[i + 1].second);
    }
    return wlOne * path.netCount;
}

// updateUsage():
//   Recompute authoritative router-side usage from accepted PATHs.
//   The graph itself is cached per Router::run; usage remains full-rescan to keep
//   routing decisions identical to the previous candidate.
void Router::updateUsage(Design& design, const RoutePath& path) const {
    (void)path;
    recomputeRouterUsageFields(design);
}

