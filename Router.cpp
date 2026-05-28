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
#include <string>
#include <tuple>
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

    static constexpr bool ROUTER_DIAG_ENABLE = true;
    static constexpr bool ROUTER_DIAG_PRINT_DECISIONS = true;
    static constexpr bool ROUTER_DIAG_PRINT_PATHS = false;
    static constexpr int  ROUTER_DIAG_TOP_CHANNELS = 12;

    static constexpr double ROUTER_WIRE_WEIGHT = 0.20;

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

    static constexpr int SPLIT_CHUNK_SIZE_1 = 300;
    static constexpr int SPLIT_CHUNK_SIZE_2 = 100;
    static constexpr int SPLIT_CHUNK_SIZE_3 = 50;
    static constexpr int SPLIT_CHUNK_SIZE_4 = 20;

    // Soft feedthrough cost.  FT is a fallback, not default routing.
    static constexpr double SOFT_FT_FIXED_PENALTY = 50000.0;
    static constexpr double SOFT_FT_PER_NET_PENALTY = 10.0;
    static constexpr double FT_INCREMENTAL_AREA_WEIGHT = 10.0;
    static constexpr double FT_OVERFLOW_EXTRA_WEIGHT = 0.0;

    // Step5 overflow weights.  Step5 is no-open rescue only, not a normal router.
    static constexpr double CHANNEL_OVERFLOW_LINEAR_WEIGHT = 1.0e8;
    static constexpr double CHANNEL_OVERFLOW_QUADRATIC_WEIGHT = 1.0e5;

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
    static double g_capacityScale = CAP_SCALE_FULL;
    static vector<double> g_channelCriticality;

    string modeName(RouteMode mode) {
        return mode == RouteMode::CHANNEL_ONLY ? "CHANNEL_ONLY" : "FT_ENABLED";
    }

    bool validEdgeLocal(int e) { return e >= 1 && e <= 4; }
    bool isOppositeLR(int a, int b) { return (a == 1 && b == 3) || (a == 3 && b == 1); }
    bool isOppositeTB(int a, int b) { return (a == 2 && b == 4) || (a == 4 && b == 2); }
    bool isTurn(int a, int b) {
        if (!validEdgeLocal(a) || !validEdgeLocal(b)) return false;
        return !isOppositeLR(a, b) && !isOppositeTB(a, b) && a != b;
    }

    bool isChannelName(const string& name) { return name.rfind("CH", 0) == 0; }

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

    // ----------------------------------------------------------------------------
    // FT helpers
    // ----------------------------------------------------------------------------
    double ftRateForNetsLocal(const BlockSpec& spec, double ftNets) {
        if (ftNets <= 3000.0) return spec.ftRate[0];
        if (ftNets <= 6000.0) return spec.ftRate[1];
        if (ftNets <= 9000.0) return spec.ftRate[2];
        return spec.ftRate[3];
    }

    double requiredAreaWithFTLocal(const BlockInst& b, double ftNets) {
        const double currentArea = b.rect.w * b.rect.h;
        if (b.spec.type != BlockType::SOFT || ftNets <= EPS) return currentArea;
        const double rate = ftRateForNetsLocal(b.spec, ftNets);
        const double delta = (ftNets / CHANNEL_DENSITY) * rate / 2.0;
        const double reqW = b.rect.w + delta;
        const double reqH = b.rect.h + delta;
        return reqW * reqH;
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

    double softFTPenaltyLocal(const BlockInst& b, int netCount) {
        const double incArea = incrementalFTAreaLocal(b, netCount);
        const double overflowAfter = estimatedFTOverflowAfterLocal(b, netCount);
        return SOFT_FT_FIXED_PENALTY
            + SOFT_FT_PER_NET_PENALTY * static_cast<double>(netCount)
            + FT_INCREMENTAL_AREA_WEIGHT * incArea
            + FT_OVERFLOW_EXTRA_WEIGHT * overflowAfter;
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

    // 將目前 design.routes 重新換算回 DataModel 的 legacy 欄位。
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
            for (int i = 0; i + 1 < static_cast<int>(path.steps.size()); ++i) {
                const auto& a = path.steps[i];
                const auto& b = path.steps[i + 1];
                if (a.rectName != b.rectName) continue;
                if (isChannelName(a.rectName)) continue;
                if (a.rectName == path.srcBlock || a.rectName == path.dstBlock) continue;
                auto it = design.blockNameToIndex.find(a.rectName);
                if (it == design.blockNameToIndex.end()) continue;
                BlockInst& blk = design.blocks[it->second];
                if (blk.spec.type == BlockType::SOFT) {
                    blk.ftUsed += nets;
                }
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

            info.minComponentCapacity = min(info.minComponentCapacity, cap);
            if (cap < NARROW_COMPONENT_CAP) ++info.narrowComponentCount;

            if (cap < EXTREME_COMPONENT_CAP) {
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
                if (cap < SPLIT_MIN_COMPONENT_CAP) {
                    info.feasible = false;
                    info.penalty = numeric_limits<double>::infinity();
                    return;
                }
                if (util > SPLIT_MAX_PROJECTED_UTIL + 1e-12) {
                    info.feasible = false;
                    info.penalty = numeric_limits<double>::infinity();
                    return;
                }
            }

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
        if (path.open) return out;
        for (int i = 0; i + 1 < static_cast<int>(path.steps.size()); ++i) {
            const auto& a = path.steps[i];
            const auto& b = path.steps[i + 1];
            if (a.rectName != b.rectName) continue;
            if (isChannelName(a.rectName)) continue;
            if (a.rectName == path.srcBlock || a.rectName == path.dstBlock) continue;
            if (isSoftBlockName(design, a.rectName)) out.insert(a.rectName);
        }
        return out;
    }

    bool pathUsesSoftFT(const Design& design, const RoutePath& path) {
        return !touchedIntermediateSoftBlocks(design, path).empty();
    }

    int pathSoftFTCount(const Design& design, const RoutePath& path) {
        return static_cast<int>(touchedIntermediateSoftBlocks(design, path).size());
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
        double ftOverflowAfterTouched = 0.0;
        double minComponentCapacity = numeric_limits<double>::infinity();
        int narrowComponentCount = 0;
        map<pair<int, DirComponent>, double> componentDelta;
    };

    RouteMetrics analyzeRoute(const Design& design, const RoutePath& path) {
        RouteMetrics m;
        m.open = path.open;
        m.wireLength = path.wireLength;
        if (path.open) return m;

        const vector<DirUse> current = computeDirectionalChannelUseFromRoutes(design);
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

        const set<string> softs = touchedIntermediateSoftBlocks(design, path);
        m.usesSoftFT = !softs.empty();
        m.softFTBlockCount = static_cast<int>(softs.size());
        for (const string& bName : softs) {
            auto it = design.blockNameToIndex.find(bName);
            if (it == design.blockNameToIndex.end()) continue;
            const BlockInst& b = design.blocks[it->second];
            m.ftIncrementalArea += incrementalFTAreaLocal(b, path.netCount);
            m.ftOverflowAfterTouched += estimatedFTOverflowAfterLocal(b, path.netCount);
        }

        return m;
    }

    void printPathLine(const RoutePath& p) {
        cerr << "PATH " << p.netCount << " ";
        for (const auto& st : p.steps) cerr << st.rectName << " " << st.edge << " ";
        cerr << "\n";
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
// 注意：這版仍然是 baseline，不含 rip-up/reroute，也沒有多 routing order rerun。
// 因此它仍可能因前面的 greedy commit 導致後面 connection 被迫 deferred。
// -----------------------------------------------------------------------------
void Router::run(Design& design) {
    design.routes.clear();
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

    buildChannelCriticalityLocal(design);

    vector<Connection> conns = design.connections;
    sort(conns.begin(), conns.end(), [](const Connection& a, const Connection& b) {
        return a.netCount > b.netCount;
        });

    auto routeWithPolicy = [&](const Design& d, const Connection& c, RouteMode mode, bool allowOverflow,
        double capacityScale, bool splitRouting) -> RoutePath {
            g_routeMode = mode;
            g_allowChannelOverflow = allowOverflow;
            g_capacityScale = capacityScale;
            g_splitRouting = splitRouting;
            return routeOneConnection(d, c);
        };

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
            << " wl=" << m.wireLength << "\n";
        if (ROUTER_DIAG_PRINT_PATHS) { cerr << "  "; printPathLine(p); }
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
        const map<pair<int, DirComponent>, double>& splitLoad, int addedRoutes) -> bool {
            if (addedRoutes <= 0) return false;
            vector<DirUse> after = computeDirectionalChannelUseFromRoutes(trial);

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

                if (cap < SPLIT_MIN_COMPONENT_CAP) return false;
                if (finalUtil > SPLIT_MAX_PROJECTED_UTIL + 1e-12) return false;
                if (addedRoutes >= 4 && share >= SPLIT_BOTTLENECK_SHARE && finalUtil >= SPLIT_BOTTLENECK_FINAL_UTIL) {
                    if (cap < 1000.0 || load > 0.50 * cap) return false;
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
        const int chunkCandidates[] = { SPLIT_CHUNK_SIZE_1, SPLIT_CHUNK_SIZE_2, SPLIT_CHUNK_SIZE_3, SPLIT_CHUNK_SIZE_4 };
        TrialResult best;

        for (int rawChunk : chunkCandidates) {
            const int chunkSize = max(1, min(rawChunk, conn.netCount));
            const int estimatedRoutes = (conn.netCount + chunkSize - 1) / chunkSize;
            if (estimatedRoutes > MAX_SPLIT_ROUTES) continue;

            Design trial = base;
            int remaining = conn.netCount;
            bool ok = true;
            int added = 0;
            int ftAdded = 0;
            map<pair<int, DirComponent>, int> hitCount;
            map<pair<int, DirComponent>, double> splitLoad;

            while (remaining > 0) {
                const int k = min(chunkSize, remaining);
                Connection cc = conn;
                cc.netCount = k;
                RoutePath p = routeWithPolicy(trial, cc, mode, false, splitCapScale, true);
                RouteMetrics m = analyzeRoute(trial, p);
                if (m.open || m.channelOverflowIncrement > EPS || m.narrowComponentCount > 0) { ok = false; break; }
                for (const auto& kv : m.componentDelta) {
                    hitCount[kv.first] += 1;
                    splitLoad[kv.first] += kv.second;
                }
                if (m.usesSoftFT) ++ftAdded;
                applyPath(trial, p);
                ++added;
                remaining -= k;
            }

            if (!ok) continue;
            if (!splitTrialHealthy(base, trial, hitCount, splitLoad, added)) continue;

            const double ov = totalChannelOverflowNow(trial);
            const double wl = totalRouteWireLengthNow(trial);
            if (ov <= EPS) {
                best.success = true;
                best.design = std::move(trial);
                best.chunkSize = chunkSize;
                best.routeCountAdded = added;
                best.totalOverflow = ov;
                best.totalWL = wl;
                best.ftPathCountAdded = ftAdded;
                return best;
            }
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
    auto tryStrictStep1To4 = [&](Design& d, const Connection& conn) -> bool {
        RoutePath chWhole = routeWithPolicy(d, conn, RouteMode::CHANNEL_ONLY, false, CAP_SCALE_CH_WHOLE_RESERVED, false);
        RouteMetrics chWholeM = analyzeRoute(d, chWhole);
        printDecision("STEP1A_CH_WHOLE_RESERVE", d, conn, chWhole);
        if (!chWholeM.open && chWholeM.channelOverflowIncrement <= EPS) { applyPath(d, chWhole); return true; }

        RoutePath chWholeFull = routeWithPolicy(d, conn, RouteMode::CHANNEL_ONLY, false, CAP_SCALE_FULL, false);
        RouteMetrics chWholeFullM = analyzeRoute(d, chWholeFull);
        printDecision("STEP1B_CH_WHOLE_FULL", d, conn, chWholeFull);
        if (!chWholeFullM.open && chWholeFullM.channelOverflowIncrement <= EPS) { applyPath(d, chWholeFull); return true; }

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

        RoutePath ftWhole = routeWithPolicy(d, conn, RouteMode::FT_ENABLED, false, CAP_SCALE_FULL, false);
        RouteMetrics ftWholeM = analyzeRoute(d, ftWhole);
        printDecision("STEP3_FT_WHOLE", d, conn, ftWhole);
        if (!ftWholeM.open && ftWholeM.channelOverflowIncrement <= EPS) { applyPath(d, ftWhole); return true; }

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

    int strictAccepted = 0;
    for (const auto& conn : conns) {
        if (tryStrictStep1To4(design, conn)) ++strictAccepted;
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
        if (!m.open) {
            applyPath(design, step5);
            ++step5Accepted;
            if (ROUTER_DIAG_ENABLE && ROUTER_DIAG_PRINT_DECISIONS) {
                cerr << "[RouterRefined][STEP5_DEFERRED_ACCEPT] " << design.blocks[conn.src].spec.name << "->" << design.blocks[conn.dst].spec.name
                    << " nets=" << conn.netCount << " ovInc=" << m.channelOverflowIncrement
                    << " maxUtilComp=" << (m.maxUtilComponent.empty() ? "NA" : m.maxUtilComponent)
                    << " wl=" << m.wireLength << " note=directional channel-only whole overflow fallback\n";
            }
            continue;
        }

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

    vector<Node> nodes = buildNodes(design);
    vector<vector<AdjEdge>> g = buildGraph(design, nodes);
    vector<DirUse> currentUse = computeDirectionalChannelUseFromRoutes(design);

    const int srcNode = conn.src;
    const int dstNode = conn.dst;
    const int N = static_cast<int>(nodes.size());
    const int S = N * EDGE_STATE_COUNT;

    auto sid = [](int node, int inEdge) { return node * EDGE_STATE_COUNT + inEdge; };
    auto sNode = [](int state) { return state / EDGE_STATE_COUNT; };
    auto sEdge = [](int state) { return state % EDGE_STATE_COUNT; };

    vector<double> dist(S, numeric_limits<double>::infinity());
    vector<int> parent(S, -1);
    vector<int> transEdgeOut(S, 0); // edge of parent node used to leave parent
    vector<int> transEdgeIn(S, 0);  // edge of current node used to enter current

    using P = pair<double, int>;
    priority_queue<P, vector<P>, greater<P>> pq;

    const int start = sid(srcNode, 0);
    dist[start] = 0.0;
    pq.push({ 0.0, start });

    int bestDst = -1;
    while (!pq.empty()) {
        auto [d, st] = pq.top(); pq.pop();
        if (d != dist[st]) continue;
        const int u = sNode(st);
        const int uIn = sEdge(st);
        if (u == dstNode) { bestDst = st; break; }

        for (const auto& e : g[u]) {
            const int v = e.to;
            const bool vEndpoint = (v == srcNode || v == dstNode);
            if (!vEndpoint && !nodeAllowedAsIntermediate(design, nodes[v])) continue;

            double resourcePenalty = 0.0;

            // When leaving an intermediate node u, we now know both its entry and
            // exit edges, so we can charge directional channel capacity or FT cost.
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
                    resourcePenalty += softFTPenaltyLocal(b, conn.netCount);
                }
            }

            const double wireCost = ROUTER_WIRE_WEIGHT * e.baseCost * static_cast<double>(conn.netCount);
            const int next = sid(v, e.edgeTo);
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

RoutePath Router::makeOpenRoute(const BlockInst& src, const BlockInst& dst, int netCount) const {
    RoutePath p;
    p.netCount = netCount;
    p.srcBlock = src.spec.name;
    p.dstBlock = dst.spec.name;
    p.open = true;
    const int srcEdge = rectCx(dst.rect) >= rectCx(src.rect) ? 3 : 1;
    const int dstEdge = edgeOpposite(srcEdge);
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
    vector<pair<double, double>> pts;
    for (const auto& st : path.steps) {
        const Rect* r = findRectByName(design, st.rectName);
        if (!r) continue;
        pts.push_back(edgeCenterPoint(*r, st.edge));
    }
    double wlOne = 0.0;
    for (int i = 0; i + 1 < static_cast<int>(pts.size()); ++i) {
        wlOne += manhattan(pts[i].first, pts[i].second, pts[i + 1].first, pts[i + 1].second);
    }
    return wlOne * path.netCount;
}

// updateUsage():
//   舊版是只把 path 經過的每個 channel usedNets += netCount。
//   這在 directional capacity 下不夠，因為同一個 channel 的 LR/TB 使用量不同。
//
//   本版做法：接受 route 後，直接從 design.routes 重算所有 directional usage，
//   再回填 legacy scalar 欄位。雖然效率不是最佳，但作為 baseline 最安全，
//   也避免 incremental update 漏算 L-shape / FT pair。
void Router::updateUsage(Design& design, const RoutePath& path) const {
    (void)path;
    recomputeRouterUsageFields(design);
}

