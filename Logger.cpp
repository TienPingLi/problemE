#include "Logger.hpp"
#include "Utility.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

using namespace std;
namespace fs = std::filesystem;

namespace {

    const auto PROGRAM_START_TIME = chrono::steady_clock::now();

    double getRuntimeSeconds() {
        const auto now = chrono::steady_clock::now();
        chrono::duration<double> elapsed = now - PROGRAM_START_TIME;
        return elapsed.count();
    }

    string finiteOrInf(double v) {
        if (std::isinf(v)) return "INF";
        if (std::isnan(v)) return "NAN";
        ostringstream oss;
        oss << fixed << setprecision(6) << v;
        return oss.str();
    }

    string csvEscape(const string& s) {
        bool needQuote = false;
        for (char c : s) {
            if (c == ',' || c == '"' || c == '\n' || c == '\r') {
                needQuote = true;
                break;
            }
        }
        if (!needQuote) return s;
        string out = "\"";
        for (char c : s) {
            if (c == '"') out += "\"\"";
            else out += c;
        }
        out += "\"";
        return out;
    }

    string boolText(bool v) {
        return v ? "YES" : "NO";
    }

    fs::path statisticsPathFor(const string& outputPath, const string& suffix) {
        fs::path out(outputPath);
        fs::path dir("Router_Statistics");
        fs::create_directories(dir);
        string stem = out.stem().string();
        if (stem.empty()) stem = "phase0";
        return dir / (stem + suffix);
    }

    double totalChannelArea(const Design& design) {
        double sum = 0.0;
        for (const auto& ch : design.channels) sum += ch.rect.w * ch.rect.h;
        return sum;
    }

    double totalLRUsed(const EvalReport& rpt) {
        double sum = 0.0;
        for (const auto& ch : rpt.channelTruth) sum += ch.lrUsed;
        return sum;
    }

    double totalTBUsed(const EvalReport& rpt) {
        double sum = 0.0;
        for (const auto& ch : rpt.channelTruth) sum += ch.tbUsed;
        return sum;
    }

    double totalLRCapacity(const EvalReport& rpt) {
        double sum = 0.0;
        for (const auto& ch : rpt.channelTruth) sum += ch.lrCapacity;
        return sum;
    }

    double totalTBCapacity(const EvalReport& rpt) {
        double sum = 0.0;
        for (const auto& ch : rpt.channelTruth) sum += ch.tbCapacity;
        return sum;
    }

    double totalLROverflow(const EvalReport& rpt) {
        double sum = 0.0;
        for (const auto& ch : rpt.channelTruth) sum += ch.lrOverflow;
        return sum;
    }

    double totalTBOverflow(const EvalReport& rpt) {
        double sum = 0.0;
        for (const auto& ch : rpt.channelTruth) sum += ch.tbOverflow;
        return sum;
    }

    vector<ChannelTruth> sortedChannelComponents(const EvalReport& rpt) {
        vector<ChannelTruth> out = rpt.channelTruth;
        sort(out.begin(), out.end(), [](const ChannelTruth& a, const ChannelTruth& b) {
            double aOv = a.lrOverflow + a.tbOverflow;
            double bOv = b.lrOverflow + b.tbOverflow;
            if (fabs(aOv - bOv) > 1e-12) return aOv > bOv;
            double aUtil = max(a.lrUtilization, a.tbUtilization);
            double bUtil = max(b.lrUtilization, b.tbUtilization);
            if (fabs(aUtil - bUtil) > 1e-12) return aUtil > bUtil;
            return a.name < b.name;
        });
        return out;
    }

    bool writeRoutesCsv(const fs::path& path, const EvalReport& rpt) {
        ofstream os(path);
        if (!os) return false;
        os << "route_id,net_count,src,dst,item_count,guiding_points,wirelength,explicit_open,invalid,invalid_reason,invalid_detail,channel_traversals,feedthrough_traversals\n";
        for (const auto& r : rpt.routeTruth) {
            os << r.routeId << ','
                << r.netCount << ','
                << csvEscape(r.srcBlock) << ','
                << csvEscape(r.dstBlock) << ','
                << r.itemCount << ','
                << r.guidingPointCount << ','
                << finiteOrInf(r.wireLength) << ','
                << boolText(r.explicitOpen) << ','
                << boolText(r.invalid) << ','
                << csvEscape(r.invalidReason) << ','
                << csvEscape(r.invalidDetail) << ','
                << r.channelTraversalCount << ','
                << r.feedthroughTraversalCount << '\n';
        }
        return true;
    }

    bool writeChannelsCsv(const fs::path& path, const EvalReport& rpt) {
        ofstream os(path);
        if (!os) return false;
        os << "channel,x,y,w,h,lr_used,lr_capacity,lr_utilization,lr_overflow,tb_used,tb_capacity,tb_utilization,tb_overflow,lr_traversals,tb_traversals,turn_traversals,total_overflow\n";
        for (const auto& ch : rpt.channelTruth) {
            os << csvEscape(ch.name) << ','
                << finiteOrInf(ch.rect.x) << ','
                << finiteOrInf(ch.rect.y) << ','
                << finiteOrInf(ch.rect.w) << ','
                << finiteOrInf(ch.rect.h) << ','
                << finiteOrInf(ch.lrUsed) << ','
                << finiteOrInf(ch.lrCapacity) << ','
                << finiteOrInf(ch.lrUtilization) << ','
                << finiteOrInf(ch.lrOverflow) << ','
                << finiteOrInf(ch.tbUsed) << ','
                << finiteOrInf(ch.tbCapacity) << ','
                << finiteOrInf(ch.tbUtilization) << ','
                << finiteOrInf(ch.tbOverflow) << ','
                << ch.lrTraversalCount << ','
                << ch.tbTraversalCount << ','
                << ch.turnTraversalCount << ','
                << finiteOrInf(ch.lrOverflow + ch.tbOverflow) << '\n';
        }
        return true;
    }

    bool writeSegmentsCsv(const fs::path& path, const EvalReport& rpt) {
        ofstream os(path);
        if (!os) return false;
        os << "route_id,channel,net_count,in_edge,out_edge,from_x,from_y,to_x,to_y,lr_demand,tb_demand,lr_span_lo,lr_span_hi,tb_span_lo,tb_span_hi\n";
        for (const auto& seg : rpt.channelSegments) {
            os << seg.routeId << ','
                << csvEscape(seg.channelName) << ','
                << seg.netCount << ','
                << seg.inEdge << ','
                << seg.outEdge << ','
                << finiteOrInf(seg.fromX) << ','
                << finiteOrInf(seg.fromY) << ','
                << finiteOrInf(seg.toX) << ','
                << finiteOrInf(seg.toY) << ','
                << finiteOrInf(seg.lrDemand) << ','
                << finiteOrInf(seg.tbDemand) << ','
                << finiteOrInf(seg.lrSpanLo) << ','
                << finiteOrInf(seg.lrSpanHi) << ','
                << finiteOrInf(seg.tbSpanLo) << ','
                << finiteOrInf(seg.tbSpanHi) << '\n';
        }
        return true;
    }

    bool writeFeedthroughCsv(const fs::path& path, const EvalReport& rpt) {
        ofstream os(path);
        if (!os) return false;
        os << "block,used_nets,conversion_rate,side_delta,base_area,current_area,required_area,overflow_area\n";
        for (const auto& ft : rpt.feedthroughTruth) {
            os << csvEscape(ft.blockName) << ','
                << finiteOrInf(ft.usedNets) << ','
                << finiteOrInf(ft.conversionRate) << ','
                << finiteOrInf(ft.sideDelta) << ','
                << finiteOrInf(ft.baseArea) << ','
                << finiteOrInf(ft.currentArea) << ','
                << finiteOrInf(ft.requiredArea) << ','
                << finiteOrInf(ft.overflowArea) << '\n';
        }
        return true;
    }

    bool writeSummaryTxt(const fs::path& path, const Design& design, const EvalReport& rpt, double alpha,
        const string& inputPath, const string& outputPath) {
        ofstream os(path);
        if (!os) return false;
        os << fixed << setprecision(6);
        os << "Phase 0 Routing Truth Summary\n";
        os << "input=" << inputPath << "\n";
        os << "output=" << outputPath << "\n";
        os << "alpha=" << alpha << "\n";
        os << "blocks=" << design.blocks.size() << "\n";
        os << "channels=" << design.channels.size() << "\n";
        os << "routes=" << design.routes.size() << "\n";
        os << "outline_area=" << rpt.outlineArea << "\n";
        os << "total_wirelength=" << rpt.totalWireLength << "\n";
        os << "base_cost=" << rpt.cost << "\n";
        os << "format_failed=" << boolText(rpt.formatFailed) << "\n";
        os << "path_invalid=" << boolText(rpt.pathInvalid) << " count=" << rpt.invalidPathCount << "\n";
        os << "block_overlap=" << boolText(rpt.blockOverlap) << " count=" << rpt.overlapCount << "\n";
        os << "routing_open=" << boolText(rpt.routingOpen) << " count=" << rpt.openPathCount << "\n";
        os << "outline_violation=" << boolText(rpt.outlineViolation) << " count=" << rpt.outlineViolationCount << "\n";
        os << "channel_overflow_total=" << rpt.totalChannelOverflow << "\n";
        os << "channel_overflow_max=" << rpt.maxChannelOverflow << "\n";
        os << "ft_overflow_total=" << rpt.totalFeedthroughOverflow << "\n";
        os << "ft_overflow_max=" << rpt.maxFeedthroughOverflow << "\n";
        os << "bad_topology=" << rpt.badTopologyCount << "\n";
        os << "unknown_object=" << rpt.unknownObjectCount << "\n";
        os << "bad_item=" << rpt.badItemCount << "\n";
        os << "illegal_feedthrough=" << rpt.illegalFeedthroughCount << "\n";
        os << "contact_fail=" << rpt.contactFailCount << "\n";
        os << "same_object_same_edge=" << rpt.sameObjectSameEdgeCount << "\n";
        os << "edge_port_violation=" << rpt.edgePortViolationCount << "\n";
        return true;
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
    cout << "Routes                  : " << design.routes.size() << '\n';
    cout << "Alpha                   : " << alpha << '\n';
    cout << "Runtime                 : " << runtimeSec << " sec\n\n";

    cout << "========== Floorplan ==========" << '\n';
    cout << "Max outline             : " << design.maxOutlineW << " x " << design.maxOutlineH << '\n';
    cout << "Output outline          : " << design.outlineW << " x " << design.outlineH << '\n';
    cout << "OutlineArea             : " << rpt.outlineArea << '\n';
    cout << "BlockOverlap            : " << yesNo(rpt.blockOverlap) << "  count=" << rpt.overlapCount << '\n';
    cout << "OutlineViolation        : " << yesNo(rpt.outlineViolation) << "  count=" << rpt.outlineViolationCount << "\n\n";

    cout << "========== Routing Truth ==========" << '\n';
    cout << "Route paths             : " << design.routes.size() << '\n';
    cout << "Open paths              : " << rpt.openPathCount << '\n';
    cout << "Invalid paths           : " << rpt.invalidPathCount << '\n';
    cout << "  bad topology          : " << rpt.badTopologyCount << '\n';
    cout << "  unknown object        : " << rpt.unknownObjectCount << '\n';
    cout << "  bad item              : " << rpt.badItemCount << '\n';
    cout << "  illegal feedthrough   : " << rpt.illegalFeedthroughCount << '\n';
    cout << "  contact fail          : " << rpt.contactFailCount << '\n';
    cout << "  same object same edge : " << rpt.sameObjectSameEdgeCount << '\n';
    cout << "  edge port violation   : " << rpt.edgePortViolationCount << '\n';
    cout << "TotalWireLength         : " << rpt.totalWireLength << "\n\n";

    cout << "========== Channel ==========" << '\n';
    cout << "Channel count           : " << design.channels.size() << '\n';
    cout << "Total channel area      : " << totalChannelArea(design) << '\n';
    cout << "LR capacity             : " << totalLRCapacity(rpt)
        << "  used=" << totalLRUsed(rpt)
        << "  overflow=" << totalLROverflow(rpt) << '\n';
    cout << "TB capacity             : " << totalTBCapacity(rpt)
        << "  used=" << totalTBUsed(rpt)
        << "  overflow=" << totalTBOverflow(rpt) << '\n';
    cout << "Total channel overflow  : " << rpt.totalChannelOverflow << '\n';
    cout << "Max channel overflow    : " << rpt.maxChannelOverflow << '\n';

    vector<ChannelTruth> topChannels = sortedChannelComponents(rpt);
    cout << "Top channel components  :\n";
    const int topLimit = min(10, static_cast<int>(topChannels.size()));
    for (int i = 0; i < topLimit; ++i) {
        const auto& ch = topChannels[i];
        cout << "  " << ch.name
            << " LR used=" << ch.lrUsed << "/" << ch.lrCapacity
            << " ov=" << ch.lrOverflow
            << " TB used=" << ch.tbUsed << "/" << ch.tbCapacity
            << " ov=" << ch.tbOverflow
            << " rect=(" << ch.rect.x << "," << ch.rect.y << "," << ch.rect.w << "," << ch.rect.h << ")"
            << '\n';
    }
    cout << "\n";

    cout << "========== Feedthrough ==========" << '\n';
    cout << "Total FT overflow       : " << rpt.totalFeedthroughOverflow << '\n';
    cout << "Max FT overflow         : " << rpt.maxFeedthroughOverflow << '\n';
    for (const auto& ft : rpt.feedthroughTruth) {
        if (ft.usedNets <= EPS && ft.overflowArea <= EPS) continue;
        cout << "  " << ft.blockName
            << " used=" << ft.usedNets
            << " rate=" << ft.conversionRate
            << " delta=" << ft.sideDelta
            << " currentArea=" << ft.currentArea
            << " requiredArea=" << ft.requiredArea
            << " overflow=" << ft.overflowArea
            << '\n';
    }
    cout << "\n";

    cout << "========== Cost ==========" << '\n';
    cout << "Cost                    : " << rpt.cost << '\n';
    cout << "Formula                 : OutlineArea + alpha * TotalWireLength\n";
    cout << "Overflow penalty        : not included in public formula; reported separately\n\n";

    cout << "========== Penalty condition ==========" << '\n';
    cout << "Channel overflow        : " << passFail(rpt.totalChannelOverflow > EPS) << '\n';
    cout << "Feedthrough overflow    : " << passFail(rpt.totalFeedthroughOverflow > EPS) << "\n\n";

    cout << "========== Fail condition ==========" << '\n';
    cout << "Format failed           : " << passFail(rpt.formatFailed) << '\n';
    cout << "Path invalid            : " << passFail(rpt.pathInvalid) << '\n';
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

bool Logger::writePhase0Reports(const Design& design, const EvalReport& rpt, double alpha,
    const string& inputPath, const string& outputPath) {
    fs::path summary = statisticsPathFor(outputPath, "_phase0_summary.txt");
    fs::path routes = statisticsPathFor(outputPath, "_phase0_routes.csv");
    fs::path channels = statisticsPathFor(outputPath, "_phase0_channels.csv");
    fs::path segments = statisticsPathFor(outputPath, "_phase0_channel_segments.csv");
    fs::path feedthrough = statisticsPathFor(outputPath, "_phase0_feedthrough.csv");

    fs::create_directories(summary.parent_path());

    bool ok = true;
    ok = writeSummaryTxt(summary, design, rpt, alpha, inputPath, outputPath) && ok;
    ok = writeRoutesCsv(routes, rpt) && ok;
    ok = writeChannelsCsv(channels, rpt) && ok;
    ok = writeSegmentsCsv(segments, rpt) && ok;
    ok = writeFeedthroughCsv(feedthrough, rpt) && ok;

    if (ok) {
        cerr << "[Phase0] Wrote routing truth reports:\n";
        cerr << "  " << summary.string() << "\n";
        cerr << "  " << routes.string() << "\n";
        cerr << "  " << channels.string() << "\n";
        cerr << "  " << segments.string() << "\n";
        cerr << "  " << feedthrough.string() << "\n";
    }
    return ok;
}
