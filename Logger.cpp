#include "Logger.hpp"
#include "Utility.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>

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

    double totalChannelArea = 0.0;
    double totalChannelCapacity = 0.0;
    double totalChannelUsed = 0.0;
    for (const auto& ch : design.channels) {
        totalChannelArea += ch.rect.w * ch.rect.h;
        totalChannelCapacity += ch.capacity;
        totalChannelUsed += ch.usedNets;
    }
    cout << "Total channel area      : " << totalChannelArea << '\n';
    cout << "Total channel capacity  : " << totalChannelCapacity << '\n';
    cout << "Total channel used      : " << totalChannelUsed << "\n\n";

    cout << "========== Routing ==========" << '\n';
    cout << "Route paths             : " << design.routes.size() << '\n';
    cout << "Open paths              : " << rpt.openPathCount << '\n';
    cout << "TotalWireLength         : " << rpt.totalWireLength << '\n';
    cout << "Total FT overflow       : " << rpt.totalFeedthroughOverflow << '\n';
    cout << "Max FT overflow         : " << rpt.maxFeedthroughOverflow << "\n\n";

    cout << "========== Cost ==========" << '\n';
    cout << "Cost                    : " << rpt.cost << '\n';
    cout << "Formula                 : OutlineArea + alpha * TotalWireLength\n\n";

    cout << "========== Penalty condition ==========" << '\n';
    cout << "Channel overflow        : " << passFail(rpt.totalChannelOverflow > EPS) << '\n';
    cout << "Feedthrough overflow    : " << passFail(rpt.totalFeedthroughOverflow > EPS) << "\n\n";

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
