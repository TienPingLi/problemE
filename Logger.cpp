#include "Logger.hpp"
#include "Utility.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

using namespace std;

namespace {

    // 不需要改 main.cpp / Logger.hpp。
    // 這個時間點會在程式啟動時建立，接近 main() 開始時間。
    const auto PROGRAM_START_TIME = chrono::steady_clock::now();

    double getRuntimeSeconds() {
        const auto now = chrono::steady_clock::now();
        chrono::duration<double> elapsed = now - PROGRAM_START_TIME;
        return elapsed.count();
    }

    // =========================================================================
// Logger-side directional channel usage recomputation
// -------------------------------------------------------------------------
// Logger 的任務是「報告」，不應該依賴 Router 留下的 ch.usedNets/ch.capacity。
// 在 direction-aware channel model 下，一個 channel 有兩個容量分量：
//
//   Horizontal / LR / edge 1 <-> edge 3:
//       左右走的水平線，需要沿 y 方向並排，
//       所以容量 = channel height * CHANNEL_DENSITY。
//
//   Vertical / TB / edge 2 <-> edge 4:
//       上下走的垂直線，需要沿 x 方向並排，
//       所以容量 = channel width * CHANNEL_DENSITY。
//
//   Turn / L-shape:
//       同時吃 horizontal 與 vertical 各一次。
//
// 注意：這裡是 aggregate directional model，還不是 interval/cut-based
// overlap model。也就是同一個 channel 內所有水平段先加總、所有垂直段
// 先加總。這和目前 baseline Router / Evaluator 比較容易對齊。
// =========================================================================

    struct LoggerChannelUse {
        double horizontalNets = 0.0; // LR: edge 1 <-> edge 3
        double verticalNets = 0.0;   // TB: edge 2 <-> edge 4
    };

    struct LoggerChannelComponentView {
        string channelName;
        string directionName;
        double used = 0.0;
        double capacity = 0.0;
        double utilization = 0.0;
        double overflow = 0.0;
        Rect rect;
    };

    bool validEdgeForLogger(int e) {
        return e >= 1 && e <= 4;
    }

    bool isOppositeLRForLogger(int a, int b) {
        return (a == 1 && b == 3) || (a == 3 && b == 1);
    }

    bool isOppositeTBForLogger(int a, int b) {
        return (a == 2 && b == 4) || (a == 4 && b == 2);
    }

    bool isTurnForLogger(int a, int b) {
        if (!validEdgeForLogger(a) || !validEdgeForLogger(b)) return false;
        if (a == b) return false;
        return !isOppositeLRForLogger(a, b) && !isOppositeTBForLogger(a, b);
    }

    double horizontalCapacityForLogger(const Channel& ch) {
        // edge 1 <-> edge 3，左右走，吃 channel 高度。
        return max(0.0, ch.rect.h) * CHANNEL_DENSITY;
    }

    double verticalCapacityForLogger(const Channel& ch) {
        // edge 2 <-> edge 4，上下走，吃 channel 寬度。
        return max(0.0, ch.rect.w) * CHANNEL_DENSITY;
    }

    vector<LoggerChannelUse> recomputeDirectionalChannelUseForLogger(const Design& design) {
        unordered_map<string, int> channelNameToIndex;
        channelNameToIndex.reserve(design.channels.size());

        for (int i = 0; i < static_cast<int>(design.channels.size()); ++i) {
            channelNameToIndex[design.channels[i].name] = i;
        }

        vector<LoggerChannelUse> use(design.channels.size());

        for (const auto& route : design.routes) {
            if (route.open) continue;
            if (route.netCount <= 0) continue;
            if (route.steps.size() < 4) continue;

            // PATH 格式：
            //   start block
            //   intermediate rectangle in/out pair
            //   intermediate rectangle in/out pair
            //   end block
            //
            // 所以中間 rectangle 應該出現在 steps[1], steps[2]、
            // steps[3], steps[4] ... 這種 pair。
            for (int i = 1; i + 1 < static_cast<int>(route.steps.size()); i += 2) {
                const auto& in = route.steps[i];
                const auto& out = route.steps[i + 1];

                // 中繼 rectangle 必須是一進一出同一個物件。
                if (in.rectName != out.rectName) continue;

                auto it = channelNameToIndex.find(in.rectName);
                if (it == channelNameToIndex.end()) continue;

                LoggerChannelUse& u = use[it->second];

                if (isOppositeLRForLogger(in.edge, out.edge)) {
                    u.horizontalNets += static_cast<double>(route.netCount);
                }
                else if (isOppositeTBForLogger(in.edge, out.edge)) {
                    u.verticalNets += static_cast<double>(route.netCount);
                }
                else if (isTurnForLogger(in.edge, out.edge)) {
                    u.horizontalNets += static_cast<double>(route.netCount);
                    u.verticalNets += static_cast<double>(route.netCount);
                }
            }
        }

        return use;
    }

} // namespace

void Logger::printFinalReport(const Design& design, const EvalReport& rpt, double alpha,
    const string& inputPath, const string& outputPath) {
    const double runtimeSec = getRuntimeSeconds();

    cout << fixed << setprecision(3);

    cout << "========== Problem E ==========" << '\n';
    cout << "Input file              : " << inputPath << '\n';
    cout << "Output cfg              : " << outputPath << '\n';
    cout << "Blocks                  : " << design.blocks.size() << '\n';
    cout << "Connections             : " << design.connections.size() << '\n';
    cout << "Channels                : " << design.channels.size() << '\n';
    cout << "Alpha                   : " << alpha << '\n';
    cout << "Runtime                 : " << runtimeSec << " sec\n\n";

    cout << "========== Floorplan ==========" << '\n';
    cout << "Max outline             : " << design.maxOutlineW << " x " << design.maxOutlineH << '\n';
    cout << "Output outline          : " << design.outlineW << " x " << design.outlineH << '\n';
    cout << "OutlineArea             : " << rpt.outlineArea << '\n';
    cout << "BlockOverlap            : " << yesNo(rpt.blockOverlap) << "  count=" << rpt.overlapCount << '\n';
    cout << "OutlineViolation        : " << yesNo(rpt.outlineViolation) << "  count=" << rpt.outlineViolationCount << "\n\n";

    cout << "========== Channel ==========" << '\n';
    cout << "Channel count           : " << design.channels.size() << '\n';
    cout << "Total channel overflow  : " << rpt.totalChannelOverflow << '\n';
    cout << "Max channel overflow    : " << rpt.maxChannelOverflow << '\n';

    // -------------------------------------------------------------------------
    // Direction-aware channel report
    // -------------------------------------------------------------------------
    // 舊版 Logger 直接加總 ch.capacity / ch.usedNets。
    // 但現在 channel capacity 已經拆成水平、垂直兩個方向，因此這裡重新
    // 從 PATH 掃描一次，避免誤把 legacy scalar 欄位當作正式容量。
    // -------------------------------------------------------------------------
    vector<LoggerChannelUse> dirUse = recomputeDirectionalChannelUseForLogger(design);

    double totalChannelArea = 0.0;

    double totalHorizontalCapacity = 0.0;
    double totalVerticalCapacity = 0.0;
    double totalHorizontalUsed = 0.0;
    double totalVerticalUsed = 0.0;
    double totalHorizontalOverflow = 0.0;
    double totalVerticalOverflow = 0.0;

    vector<LoggerChannelComponentView> componentViews;
    componentViews.reserve(design.channels.size() * 2);

    for (int i = 0; i < static_cast<int>(design.channels.size()); ++i) {
        const Channel& ch = design.channels[i];
        const LoggerChannelUse& u = dirUse[i];

        const double hCap = horizontalCapacityForLogger(ch);
        const double vCap = verticalCapacityForLogger(ch);

        const double hUsed = u.horizontalNets;
        const double vUsed = u.verticalNets;

        const double hOverflow = max(0.0, hUsed - hCap);
        const double vOverflow = max(0.0, vUsed - vCap);

        const double hUtil = hCap > EPS ? hUsed / hCap : numeric_limits<double>::infinity();
        const double vUtil = vCap > EPS ? vUsed / vCap : numeric_limits<double>::infinity();

        totalChannelArea += ch.rect.w * ch.rect.h;

        totalHorizontalCapacity += hCap;
        totalVerticalCapacity += vCap;
        totalHorizontalUsed += hUsed;
        totalVerticalUsed += vUsed;
        totalHorizontalOverflow += hOverflow;
        totalVerticalOverflow += vOverflow;

        componentViews.push_back(LoggerChannelComponentView{
            ch.name,
            "Horizontal/LR(edge1<->edge3)",
            hUsed,
            hCap,
            hUtil,
            hOverflow,
            ch.rect
            });

        componentViews.push_back(LoggerChannelComponentView{
            ch.name,
            "Vertical/TB(edge2<->edge4)",
            vUsed,
            vCap,
            vUtil,
            vOverflow,
            ch.rect
            });
    }

    const double totalDirectionalOverflow = totalHorizontalOverflow + totalVerticalOverflow;

    cout << "Total channel area      : " << totalChannelArea << '\n';

    cout << "Horizontal capacity     : " << totalHorizontalCapacity
        << "  used=" << totalHorizontalUsed
        << "  overflow=" << totalHorizontalOverflow << '\n';

    cout << "Vertical capacity       : " << totalVerticalCapacity
        << "  used=" << totalVerticalUsed
        << "  overflow=" << totalVerticalOverflow << '\n';

    cout << "Directional overflow    : " << totalDirectionalOverflow << '\n';

    // 如果這裡和 rpt.totalChannelOverflow 不一致，代表 Evaluator 的 channel
    // overflow 計算模型尚未和 Logger/Router 對齊。
    if (fabs(totalDirectionalOverflow - rpt.totalChannelOverflow) > 1e-3) {
        cout << "Directional overflow note: logger recompute differs from evaluator report by "
            << fabs(totalDirectionalOverflow - rpt.totalChannelOverflow)
            << '\n';
    }

    sort(componentViews.begin(), componentViews.end(),
        [](const LoggerChannelComponentView& a, const LoggerChannelComponentView& b) {
            if (fabs(a.overflow - b.overflow) > 1e-12) return a.overflow > b.overflow;
            if (fabs(a.utilization - b.utilization) > 1e-12) return a.utilization > b.utilization;
            return a.used > b.used;
        });

    cout << "Top channel components  :\n";
    const int topLimit = min(10, static_cast<int>(componentViews.size()));
    for (int i = 0; i < topLimit; ++i) {
        const auto& c = componentViews[i];

        cout << "  " << c.channelName
            << " " << c.directionName
            << " used=" << c.used
            << " cap=" << c.capacity
            << " util=" << c.utilization
            << " overflow=" << c.overflow
            << " rect=(" << c.rect.x << "," << c.rect.y
            << "," << c.rect.w << "," << c.rect.h << ")"
            << '\n';
    }

    cout << "\n";

    cout << "========== Routing ==========" << '\n';
    cout << "Route paths             : " << design.routes.size() << '\n';
    cout << "Open paths              : " << rpt.openPathCount << '\n';
    cout << "TotalWireLength         : " << rpt.totalWireLength << '\n';
    cout << "Total FT overflow       : " << rpt.totalFeedthroughOverflow << '\n';
    cout << "Max FT overflow         : " << rpt.maxFeedthroughOverflow << '\n';
    cout << "Illegal FT blocks       : " << rpt.illegalFeedthroughCount << '\n';
    cout << "Illegal FT delta area   : " << rpt.illegalFeedthroughDeltaArea << "\n\n";

    cout << "========== Cost ==========" << '\n';
    cout << "Base cost               : " << rpt.baseCost << '\n';
    cout << "Overflow rate           : " << rpt.channelOverflowRate << '\n';
    cout << "Overflow penalty        : " << rpt.overflowPenalty << '\n';
    cout << "FT penalty              : " << rpt.feedthroughPenalty << '\n';
    cout << "Illegal FT penalty      : " << rpt.illegalFeedthroughPenalty << '\n';
    cout << "Edge location offset    : " << rpt.edgeLocationOffset
        << "  count=" << rpt.edgeLocationViolationCount << '\n';
    cout << "Edge location penalty   : " << rpt.edgeLocationPenalty << '\n';
    cout << "Runtime penalty         : " << rpt.runtimePenalty << '\n';
    cout << "Cost                    : " << rpt.cost << '\n';
    cout << "Formula                 : OutlineArea + alpha * TotalWireLength + V5 penalties\n\n";

    cout << "========== Penalty condition ==========" << '\n';
    cout << "Channel overflow        : " << passFail(rpt.totalChannelOverflow > EPS) << '\n';
    cout << "Feedthrough overflow    : " << passFail(rpt.totalFeedthroughOverflow > EPS) << '\n';
    cout << "Illegal feedthrough     : " << passFail(rpt.illegalFeedthroughDeltaArea > EPS) << '\n';
    cout << "Edge location warning   : " << passFail(rpt.edgeLocationOffset > EPS) << "\n\n";

    cout << "========== Fail condition ==========" << '\n';
    cout << "Format failed           : " << passFail(rpt.formatFailed) << '\n';
    cout << "Block overlap           : " << passFail(rpt.blockOverlap) << '\n';
    cout << "Routing open            : " << passFail(rpt.routingOpen) << '\n';
    cout << "Outline violation       : " << passFail(rpt.outlineViolation) << "\n\n";

    cout << "========== Final ==========" << '\n';
    if (rpt.hasFail()) {
        cout << "Status                  : FAIL\n";
    }
    else if (rpt.hasPenalty()) {
        cout << "Status                  : LEGAL_WITH_PENALTY\n";
    }
    else {
        cout << "Status                  : LEGAL\n";
    }

    cout << "Runtime                 : " << runtimeSec << " sec\n";
}
