#include "RouterPhase2.hpp"
#include "Evaluator.hpp"
#include "OutputWriter.hpp"
#include "Utility.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <tuple>

using namespace std;
namespace fs = std::filesystem;

namespace {

string joinPathSteps(const vector<RouteStep>& steps) {
    ostringstream oss;
    for (int i = 0; i < static_cast<int>(steps.size()); ++i) {
        if (i) oss << '|';
        oss << steps[i].rectName << ':' << steps[i].edge;
    }
    return oss.str();
}

template <typename T>
T clampLocal(T v, T lo, T hi) {
    return min(hi, max(lo, v));
}

double ftRateForNetsLocal(const BlockSpec& spec, double ftNets) {
    if (ftNets <= 3000.0) return spec.ftRate[0];
    if (ftNets <= 6000.0) return spec.ftRate[1];
    if (ftNets <= 9000.0) return spec.ftRate[2];
    return spec.ftRate[3];
}

double requiredAreaWithCurrentLocal(const BlockInst& b, double ftNets) {
    const double currentArea = max(EPS, b.rect.w * b.rect.h);
    if (b.spec.type != BlockType::SOFT) return currentArea;
    const double baseArea = max(0.0, b.spec.area);
    if (ftNets <= EPS) return baseArea;
    const double rate = ftRateForNetsLocal(b.spec, ftNets);
    const double delta = (ftNets / CHANNEL_DENSITY) * rate / 2.0;
    const double baseSide = sqrt(baseArea);
    return (baseSide + delta) * (baseSide + delta);
}

double softFtUsedNetsLocal(const RoutingResourceModel& model, int blockIndex) {
    for (const auto& r : model.softFtResources()) {
        if (r.blockIndex == blockIndex) return max(0.0, r.usedNets);
    }
    return 0.0;
}

double softFtOverflowAreaLocal(const Design& design, int blockIndex, double ftNets) {
    if (blockIndex < 0 || blockIndex >= static_cast<int>(design.blocks.size())) return 0.0;
    const BlockInst& b = design.blocks[blockIndex];
    if (b.spec.type != BlockType::SOFT) return 0.0;
    const double required = requiredAreaWithCurrentLocal(b, ftNets);
    const double current = max(EPS, b.rect.w * b.rect.h);
    return max(0.0, required - current);
}

} // namespace

int RouterPhase2::chooseChunkSize(int remaining, int allocatedSoFar) const {
    (void)allocatedSoFar;
    for (int c : opt_.chunkSizes) {
        if (c <= 0) continue;
        if (remaining >= c) return c;
    }
    return max(1, remaining);
}

Phase1ResourceDelta RouterPhase2::scaleDelta(const Phase1ResourceDelta& base, double scale) const {
    Phase1ResourceDelta d = base;
    const double s = max(0.0, scale);
    for (auto& kv : d.channelLR) kv.second *= s;
    for (auto& kv : d.channelTB) kv.second *= s;
    for (auto& kv : d.softFtNets) kv.second *= s;
    for (auto& t : d.blockEdgeUse) get<2>(t) *= s;
    d.wireLength *= s;
    return d;
}

RoutePath RouterPhase2::scalePath(const RoutePath& base, int nets) const {
    RoutePath p = base;
    const double denom = max(1, base.netCount);
    const double scale = static_cast<double>(nets) / static_cast<double>(denom);
    p.netCount = nets;
    p.wireLength = base.wireLength * scale;
    return p;
}

double RouterPhase2::evalHardOverflow(const RoutingResourceModel& model, const Phase1ResourceDelta& d) const {
    double ov = 0.0;
    map<int, pair<double, double>> use;
    for (const auto& kv : d.channelLR) use[kv.first].first += kv.second;
    for (const auto& kv : d.channelTB) use[kv.first].second += kv.second;

    const auto& cr = model.channelResources();
    for (const auto& kv : use) {
        const int ci = kv.first;
        if (ci < 0 || ci >= static_cast<int>(cr.size())) continue;
        ov += max(0.0, cr[ci].usedLR + kv.second.first - cr[ci].hardCapLR);
        ov += max(0.0, cr[ci].usedTB + kv.second.second - cr[ci].hardCapTB);
    }
    return ov;
}

double RouterPhase2::evalSoftOverflow(const RoutingResourceModel& model, const Phase1ResourceDelta& d) const {
    double ov = 0.0;
    map<int, pair<double, double>> use;
    for (const auto& kv : d.channelLR) use[kv.first].first += kv.second;
    for (const auto& kv : d.channelTB) use[kv.first].second += kv.second;

    const auto& cr = model.channelResources();
    for (const auto& kv : use) {
        const int ci = kv.first;
        if (ci < 0 || ci >= static_cast<int>(cr.size())) continue;
        ov += max(0.0, cr[ci].usedLR + cr[ci].ambientLR + kv.second.first - cr[ci].softCapLR);
        ov += max(0.0, cr[ci].usedTB + cr[ci].ambientTB + kv.second.second - cr[ci].softCapTB);
    }

    for (const auto& kv : d.softFtNets) {
        if (!model.softFeasibleSoftFt(kv.first, kv.second)) {
            ov += kv.second;
        }
    }
    return ov;
}

pair<int, int> RouterPhase2::dominantComponent(const Phase1ResourceDelta& d) const {
    int bestCi = -1;
    int bestComp = 0;
    double best = -1.0;
    for (const auto& kv : d.channelLR) {
        if (kv.second > best) {
            best = kv.second;
            bestCi = kv.first;
            bestComp = 1;
        }
    }
    for (const auto& kv : d.channelTB) {
        if (kv.second > best) {
            best = kv.second;
            bestCi = kv.first;
            bestComp = 2;
        }
    }
    return {bestCi, bestComp};
}

double RouterPhase2::dynamicFtPenalty(
    const Design& design,
    const RoutingResourceModel& model,
    const Phase1ResourceDelta& d,
    const vector<double>& histFt
) const {
    double penalty = 0.0;
    for (const auto& kv : d.softFtNets) {
        const int bi = kv.first;
        const double add = kv.second;
        if (bi < 0 || bi >= static_cast<int>(design.blocks.size()) || add <= EPS) continue;
        const BlockInst& b = design.blocks[bi];
        if (b.spec.type != BlockType::SOFT) continue;

        const double used0 = softFtUsedNetsLocal(model, bi);
        const double used1 = used0 + add;
        const double curOv = softFtOverflowAreaLocal(design, bi, used0);
        const double projOv = softFtOverflowAreaLocal(design, bi, used1);
        const double incOv = max(0.0, projOv - curOv);
        const double currentArea = max(EPS, b.rect.w * b.rect.h);
        const double reqArea = requiredAreaWithCurrentLocal(b, used1);
        const double areaUtil = reqArea / currentArea;
        const double hist = (bi < static_cast<int>(histFt.size())) ? histFt[bi] : 0.0;

        penalty += add * (45.0 + 130.0 * hist);
        penalty += 0.16 * incOv;
        penalty += 0.045 * projOv;
        penalty += add * 700.0 * max(0.0, areaUtil - 1.0) * max(0.0, areaUtil - 1.0);
    }
    return penalty;
}

void RouterPhase2::updateFtHistoryFromModel(
    const Design& design,
    const RoutingResourceModel& model,
    vector<double>& histFt
) const {
    if (histFt.size() < design.blocks.size()) histFt.resize(design.blocks.size(), 0.0);
    for (const auto& r : model.softFtResources()) {
        const int bi = r.blockIndex;
        if (bi < 0 || bi >= static_cast<int>(histFt.size())) continue;
        const double ov = softFtOverflowAreaLocal(design, bi, r.usedNets);
        const double usageTerm = 0.20 * log1p(max(0.0, r.usedNets) / 1000.0);
        const double overflowTerm = ov > 1.0e-9 ? 0.75 * min(12.0, 1.0 + log1p(ov) / 2.0) : 0.0;
        histFt[bi] = min(80.0, histFt[bi] + usageTerm + overflowTerm);
    }
}

double RouterPhase2::marginalScore(
    const Design& design,
    const RoutingResourceModel& model,
    const Phase1ResourceDelta& d,
    double diversityPenalty,
    const vector<double>& histFt,
    double& ftPenaltyOut
) const {
    double s = 0.0;
    s += 0.20 * d.wireLength;
    s += 40.0 * static_cast<double>(d.bendCount);

    map<int, pair<double, double>> use;
    for (const auto& kv : d.channelLR) use[kv.first].first += kv.second;
    for (const auto& kv : d.channelTB) use[kv.first].second += kv.second;
    for (const auto& kv : use) {
        s += model.scoreChannelUse(kv.first, kv.second.first, kv.second.second);
    }

    for (const auto& kv : d.softFtNets) {
        s += model.scoreSoftFtUse(kv.first, kv.second);
    }
    for (const auto& t : d.blockEdgeUse) {
        s += model.scoreBlockAccessUse(get<0>(t), get<1>(t), get<2>(t));
    }

    ftPenaltyOut = dynamicFtPenalty(design, model, d, histFt);
    s += ftPenaltyOut;
    s += diversityPenalty;
    return s;
}

double RouterPhase2::connectionPriority(const Connection& conn, const Step0ConnectionGuide* guide, int maxNetCount) const {
    const double netNorm = static_cast<double>(conn.netCount) / static_cast<double>(max(1, maxNetCount));
    if (!guide) return netNorm;

    const double routePriority = clampLocal(guide->routePriority, 0.0, 1.5);
    const double openRisk = clampLocal(guide->openRiskScore, 0.0, 1.5);
    const double endpointAccess = guide->endpointAccessRisk ? 1.0 : 0.0;
    const double endpointBottleneck = guide->endpointBottleneckRisk ? 1.0 : 0.0;
    const double sharedBottleneck = guide->sharedBottleneckRisk ? 1.0 : 0.0;
    const double lackAlt = clampLocal(guide->lackOfAlternative, 0.0, 1.5);
    const double coupling = clampLocal(guide->couplingRiskScore, 0.0, 1.5);

    // Prioritize hard-to-fix / high-open-risk connections first (NTHU-Route style ordering).
    return 4.0 * openRisk
        + 3.0 * endpointBottleneck
        + 2.5 * sharedBottleneck
        + 2.0 * lackAlt
        + 1.5 * routePriority
        + 1.0 * endpointAccess
        + 0.8 * coupling
        + 1.0 * netNorm;
}

double RouterPhase2::guideAdjustment(const Step0ConnectionGuide* guide, const Phase1ResourceDelta& d, int chunkNets) const {
    if (!guide || chunkNets <= 0) return 0.0;
    if (guide->patternGuideMode == "ROUTER_FREE") return 0.0;

    set<int> usedChannels;
    for (const auto& kv : d.channelLR) if (kv.second > 1.0e-9) usedChannels.insert(kv.first);
    for (const auto& kv : d.channelTB) if (kv.second > 1.0e-9) usedChannels.insert(kv.first);
    if (usedChannels.empty()) return 0.0;

    set<int> preferred(guide->preferredChannelIndices.begin(), guide->preferredChannelIndices.end());
    set<int> avoid(guide->avoidChannelIndices.begin(), guide->avoidChannelIndices.end());

    int preferredHit = 0;
    int avoidHit = 0;
    for (int ci : usedChannels) {
        if (preferred.count(ci)) preferredHit += 1;
        if (avoid.count(ci)) avoidHit += 1;
    }

    const double strength = clampLocal(guide->patternGuideStrength, 0.0, 1.0);
    const double predictability = clampLocal(guide->patternPredictabilityScore, 0.0, 1.0);
    const double coupling = clampLocal(guide->couplingRiskScore, 0.0, 2.0);

    const double wBonus = static_cast<double>(chunkNets) * (10.0 + 25.0 * strength * predictability);
    const double wPenalty = static_cast<double>(chunkNets) * (18.0 + 35.0 * strength * (1.0 + 0.5 * coupling));

    return static_cast<double>(avoidHit) * wPenalty - static_cast<double>(preferredHit) * wBonus;
}

int RouterPhase2::centerFacingEdge(const Design& design, int blockIndex) const {
    const Rect& r = design.blocks[blockIndex].rect;
    const double ocx = design.outlineW * 0.5;
    const double ocy = design.outlineH * 0.5;
    const double dx = ocx - rectCx(r);
    const double dy = ocy - rectCy(r);
    if (fabs(dx) >= fabs(dy)) return dx >= 0.0 ? 3 : 1;
    return dy >= 0.0 ? 2 : 4;
}

RoutePath RouterPhase2::makeOpenPath(const Design& design, const Connection& conn, int nets) const {
    RoutePath p;
    p.netCount = nets;
    p.srcBlock = design.blocks[conn.src].spec.name;
    p.dstBlock = design.blocks[conn.dst].spec.name;
    p.open = true;

    int srcEdge = rectCx(design.blocks[conn.dst].rect) >= rectCx(design.blocks[conn.src].rect) ? 3 : 1;
    int dstEdge = edgeOpposite(srcEdge);
    if (design.blocks[conn.src].spec.type == BlockType::EDGE) srcEdge = centerFacingEdge(design, conn.src);
    if (design.blocks[conn.dst].spec.type == BlockType::EDGE) dstEdge = centerFacingEdge(design, conn.dst);
    p.steps.push_back({p.srcBlock, srcEdge});
    p.steps.push_back({p.dstBlock, dstEdge});
    return p;
}

Design RouterPhase2::buildPhase2RoutedDesign(
    const Design& design,
    const vector<Phase2AllocationPiece>& pieces,
    const vector<Phase2ConnectionStatus>& connStatus
) const {
    Design out = design;
    out.routes.clear();

    for (const auto& p : pieces) {
        if (p.connectionIndex < 0 || p.connectionIndex >= static_cast<int>(design.connections.size())) continue;
        if (p.allocatedNets <= 0) continue;
        out.routes.push_back(p.path);
    }

    for (int ci = 0; ci < static_cast<int>(connStatus.size()); ++ci) {
        const int unalloc = connStatus[ci].unallocatedNets;
        if (unalloc <= 0) continue;
        out.routes.push_back(makeOpenPath(design, design.connections[ci], unalloc));
    }

    return out;
}

RouterPhase2::RunResult RouterPhase2::run(const Design& design, const string& inputPath, const string& outputCfgPath, double alpha) const {
    RouterPhase1::Options p1opt;
    p1opt.maxCandidatesPerConnection = opt_.maxCandidatesPerConnection;
    p1opt.exportFiles = false;
    RouterPhase1 phase1(p1opt);
    RouterPhase1::CandidateBuildResult built = phase1.buildCandidates(design);
    return run(design, inputPath, outputCfgPath, alpha, built);
}

RouterPhase2::RunResult RouterPhase2::run(
    const Design& design,
    const string& inputPath,
    const string& outputCfgPath,
    double alpha,
    const RouterPhase1::CandidateBuildResult& built
) const {
    RunResult rr;
    rr.totalConnections = static_cast<int>(design.connections.size());
    rr.ok = built.ok && built.candidates.size() == design.connections.size();
    if (!rr.ok) return rr;

    RoutingResourceModel model;
    model.initialize(design, built.step0);

    vector<int> connOrder(design.connections.size());
    for (int i = 0; i < static_cast<int>(connOrder.size()); ++i) connOrder[i] = i;
    int maxNetCount = 1;
    for (const auto& c : design.connections) maxNetCount = max(maxNetCount, c.netCount);
    sort(connOrder.begin(), connOrder.end(), [&](int a, int b) {
        const Step0ConnectionGuide* ga = a < static_cast<int>(built.step0.connectionGuide.size()) ? &built.step0.connectionGuide[a] : nullptr;
        const Step0ConnectionGuide* gb = b < static_cast<int>(built.step0.connectionGuide.size()) ? &built.step0.connectionGuide[b] : nullptr;
        const double pa = connectionPriority(design.connections[a], ga, maxNetCount);
        const double pb = connectionPriority(design.connections[b], gb, maxNetCount);
        if (fabs(pa - pb) > 1.0e-9) return pa > pb;
        if (design.connections[a].netCount != design.connections[b].netCount) return design.connections[a].netCount > design.connections[b].netCount;
        return a < b;
    });

    vector<Phase2AllocationPiece> pieces;
    vector<Phase2ConnectionStatus> connStatus(design.connections.size());
    vector<double> histFt(design.blocks.size(), 0.0);

    for (int ord = 0; ord < static_cast<int>(connOrder.size()); ++ord) {
        const int ci = connOrder[ord];
        const Connection& conn = design.connections[ci];
        const auto& candVec = built.candidates[ci];

        Phase2ConnectionStatus st;
        st.connectionIndex = ci;
        st.srcBlock = design.blocks[conn.src].spec.name;
        st.dstBlock = design.blocks[conn.dst].spec.name;
        st.netCount = conn.netCount;
        const Step0ConnectionGuide* guide = ci < static_cast<int>(built.step0.connectionGuide.size()) ? &built.step0.connectionGuide[ci] : nullptr;

        if (candVec.empty()) {
            st.unallocatedNets = conn.netCount;
            connStatus[ci] = st;
            rr.failedConnections += 1;
            rr.totalUnallocatedNets += conn.netCount;
            continue;
        }

        int remaining = conn.netCount;
        int allocated = 0;
        bool usedHardFallback = false;
        set<int> usedCandidates;
        double scoreSum = 0.0;
        map<pair<int, int>, int> dominantLoad;

        while (remaining > 0) {
            const int chunk = chooseChunkSize(remaining, allocated);
            vector<ScoredCandidate> scored;
            scored.reserve(candVec.size());

            for (const auto& base : candVec) {
                const int denom = max(1, base.path.netCount);
                const double scale = static_cast<double>(chunk) / static_cast<double>(denom);
                Phase1ResourceDelta d = scaleDelta(base.delta, scale);
                RoutePath p = scalePath(base.path, chunk);

                const double hardOv = evalHardOverflow(model, d);
                const double softOv = evalSoftOverflow(model, d);
                const bool hardOk = hardOv <= 1.0e-9;
                const bool softOk = softOv <= 1.0e-9;

                const auto dom = dominantComponent(d);
                double diversityPenalty = 0.0;
                const int domLoad = dominantLoad[dom];
                if (allocated >= 300 && dom.first >= 0) {
                    const double share = static_cast<double>(domLoad) / static_cast<double>(max(1, allocated));
                    if (share > 0.80) {
                        diversityPenalty = (share - 0.80) * static_cast<double>(chunk) * 200.0;
                    }
                }

                ScoredCandidate sc;
                sc.candidateIndex = base.candidateIndex;
                sc.score = marginalScore(design, model, d, diversityPenalty, histFt, sc.ftDynamicPenalty)
                    + guideAdjustment(guide, d, chunk);
                sc.hardFeasible = hardOk;
                sc.softFeasible = softOk;
                sc.hardOverflowAmount = hardOv;
                sc.softOverflowAmount = softOv;
                sc.dominantComp = dom;
                sc.delta = move(d);
                sc.path = move(p);
                scored.push_back(move(sc));
            }

            if (scored.empty()) break;

            sort(scored.begin(), scored.end(), [](const ScoredCandidate& a, const ScoredCandidate& b) {
                if (a.hardFeasible != b.hardFeasible) return a.hardFeasible > b.hardFeasible;
                if (a.softFeasible != b.softFeasible) return a.softFeasible > b.softFeasible;
                if (fabs(a.hardOverflowAmount - b.hardOverflowAmount) > 1.0e-9) return a.hardOverflowAmount < b.hardOverflowAmount;
                if (fabs(a.score - b.score) > 1.0e-9) return a.score < b.score;
                return a.candidateIndex < b.candidateIndex;
            });

            const ScoredCandidate* chosen = nullptr;
            for (const auto& sc : scored) {
                if (sc.hardFeasible) {
                    chosen = &sc;
                    break;
                }
            }

            if (!chosen && opt_.allowHardFallback) {
                chosen = &scored.front();
                usedHardFallback = true;
                rr.hardFallbackPieces += 1;
            }
            if (!chosen) break;

            model.commit(chosen->delta);
            updateFtHistoryFromModel(design, model, histFt);

            Phase2AllocationPiece piece;
            piece.connectionIndex = ci;
            piece.candidateIndex = chosen->candidateIndex;
            piece.family = candVec[chosen->candidateIndex].family;
            piece.allocatedNets = chunk;
            piece.score = chosen->score;
            piece.hardFeasible = chosen->hardFeasible;
            piece.softFeasible = chosen->softFeasible;
            piece.hardOverflowAmount = chosen->hardOverflowAmount;
            piece.softOverflowAmount = chosen->softOverflowAmount;
            piece.ftDynamicPenalty = chosen->ftDynamicPenalty;
            piece.path = chosen->path;
            piece.delta = chosen->delta;
            pieces.push_back(piece);
            if (piece.ftDynamicPenalty > 1.0e-9) {
                rr.dynamicFtPricedPieces += 1;
                rr.totalDynamicFtPenalty += piece.ftDynamicPenalty;
            }

            allocated += chunk;
            remaining -= chunk;
            usedCandidates.insert(chosen->candidateIndex);
            scoreSum += chosen->score;
            dominantLoad[chosen->dominantComp] += chunk;
        }

        st.allocatedNets = allocated;
        st.unallocatedNets = remaining;
        st.pieceCount = 0;
        for (const auto& p : pieces) if (p.connectionIndex == ci) st.pieceCount += 1;
        st.uniqueCandidateCount = static_cast<int>(usedCandidates.size());
        st.avgPieceScore = st.pieceCount > 0 ? (scoreSum / static_cast<double>(st.pieceCount)) : 0.0;
        st.usedHardFallback = usedHardFallback;

        int maxLoad = 0;
        for (const auto& kv : dominantLoad) maxLoad = max(maxLoad, kv.second);
        st.maxDominantShare = allocated > 0 ? static_cast<double>(maxLoad) / static_cast<double>(allocated) : 0.0;

        connStatus[ci] = st;

        rr.totalAllocatedNets += allocated;
        rr.totalUnallocatedNets += remaining;
        if (remaining == 0) rr.fullyAllocatedConnections += 1;
        else if (allocated > 0) rr.partiallyAllocatedConnections += 1;
        else rr.failedConnections += 1;
    }

    rr.totalPieces = static_cast<int>(pieces.size());
    for (double h : histFt) rr.maxFtHistory = max(rr.maxFtHistory, h);

    Design phase2Design = buildPhase2RoutedDesign(design, pieces, connStatus);
    Evaluator evaluator;
    EvalReport phase2Eval = evaluator.evaluate(phase2Design, alpha);
    rr.phase2EvalComputed = true;
    rr.phase2EvalHasFail = phase2Eval.hasFail();
    rr.phase2TotalChannelOverflow = phase2Eval.totalChannelOverflow;
    rr.phase2MaxChannelOverflow = phase2Eval.maxChannelOverflow;
    rr.phase2TotalFeedthroughOverflow = phase2Eval.totalFeedthroughOverflow;
    rr.phase2OpenPathCount = static_cast<double>(phase2Eval.openPathCount);

    if (opt_.exportFiles) {
        fs::path outPath(outputCfgPath);
        const fs::path stem = outPath.parent_path() / outPath.stem();
        const string phase2CfgPath = stem.string() + "_phase2.cfg";
        OutputWriter writer;
        if (!writer.write(phase2CfgPath, phase2Design)) {
            rr.ok = false;
        } else {
            rr.ok = writeReports(design, built, model, phase2Eval, phase2CfgPath, pieces, connStatus, rr, inputPath, outputCfgPath, alpha);
        }
    }

    return rr;
}

bool RouterPhase2::writeReports(
    const Design& design,
    const RouterPhase1::CandidateBuildResult& phase1,
    const RoutingResourceModel& modelAfter,
    const EvalReport& phase2Eval,
    const string& phase2CfgPath,
    const vector<Phase2AllocationPiece>& pieces,
    const vector<Phase2ConnectionStatus>& connStatus,
    const RunResult& rr,
    const string& inputPath,
    const string& outputCfgPath,
    double alpha
) const {
    fs::path outPath(outputCfgPath);
    fs::path reportDir("Router_Statistics");
    fs::create_directories(reportDir);
    const fs::path stem = reportDir / outPath.stem();

    const string summaryPath = (stem.string() + "_phase2_summary.txt");
    const string piecesPath = (stem.string() + "_phase2_allocations.csv");
    const string connPath = (stem.string() + "_phase2_connection_status.csv");
    const string resPath = (stem.string() + "_phase2_resource_after.csv");
    const string routePath = (stem.string() + "_phase2_selected_routes.csv");

    ofstream fsum(summaryPath);
    if (!fsum) return false;
    fsum << fixed << setprecision(6);
    fsum << "Phase 2 Initial Router + Greedy BundleAllocator Summary\n";
    fsum << "input=" << inputPath << "\n";
    fsum << "requested_output=" << outputCfgPath << "\n";
    fsum << "phase2_output_cfg=" << phase2CfgPath << "\n";
    fsum << "alpha=" << alpha << "\n";
    fsum << "connections=" << rr.totalConnections << "\n";
    fsum << "fully_allocated_connections=" << rr.fullyAllocatedConnections << "\n";
    fsum << "partially_allocated_connections=" << rr.partiallyAllocatedConnections << "\n";
    fsum << "failed_connections=" << rr.failedConnections << "\n";
    fsum << "total_allocated_nets=" << rr.totalAllocatedNets << "\n";
    fsum << "total_unallocated_nets=" << rr.totalUnallocatedNets << "\n";
    fsum << "total_pieces=" << rr.totalPieces << "\n";
    fsum << "hard_fallback_pieces=" << rr.hardFallbackPieces << "\n";
    fsum << "dynamic_ft_pricing=ON\n";
    fsum << "dynamic_ft_priced_pieces=" << rr.dynamicFtPricedPieces << "\n";
    fsum << "dynamic_ft_total_penalty=" << rr.totalDynamicFtPenalty << "\n";
    fsum << "dynamic_ft_max_history=" << rr.maxFtHistory << "\n";
    fsum << "phase1_generated_candidates=" << phase1.stats.generatedCandidates << "\n";
    fsum << "phase1_connections_without_candidate=" << phase1.stats.connectionsWithoutCandidate << "\n";
    fsum << "phase2_eval_has_fail=" << yesNo(phase2Eval.hasFail()) << "\n";
    const bool phase2HasPenalty = (phase2Eval.totalChannelOverflow > 1.0e-9) || (phase2Eval.totalFeedthroughOverflow > 1.0e-9);
    fsum << "phase2_eval_has_penalty=" << yesNo(phase2HasPenalty) << "\n";
    fsum << "phase2_eval_path_invalid=" << yesNo(phase2Eval.pathInvalid) << "\n";
    fsum << "phase2_eval_open_paths=" << phase2Eval.openPathCount << "\n";
    fsum << "phase2_eval_total_channel_overflow=" << phase2Eval.totalChannelOverflow << "\n";
    fsum << "phase2_eval_max_channel_overflow=" << phase2Eval.maxChannelOverflow << "\n";
    fsum << "phase2_eval_total_feedthrough_overflow=" << phase2Eval.totalFeedthroughOverflow << "\n";
    fsum << "phase2_eval_total_wire_length=" << phase2Eval.totalWireLength << "\n";
    fsum << "phase2_eval_cost=" << phase2Eval.cost << "\n";
    fsum.close();

    ofstream fp(piecesPath);
    if (!fp) return false;
    fp << fixed << setprecision(6);
    fp << "connection_index,src,dst,candidate_index,family,allocated_nets,score,hard_feasible,soft_feasible,hard_overflow_amount,soft_overflow_amount,dynamic_ft_penalty,path_steps\n";
    for (const auto& p : pieces) {
        const Connection& c = design.connections[p.connectionIndex];
        fp << p.connectionIndex << ','
           << design.blocks[c.src].spec.name << ','
           << design.blocks[c.dst].spec.name << ','
           << p.candidateIndex << ','
           << p.family << ','
           << p.allocatedNets << ','
           << p.score << ','
           << yesNo(p.hardFeasible) << ','
           << yesNo(p.softFeasible) << ','
           << p.hardOverflowAmount << ','
           << p.softOverflowAmount << ','
           << p.ftDynamicPenalty << ','
           << '"' << joinPathSteps(p.path.steps) << '"' << "\n";
    }
    fp.close();

    ofstream fc(connPath);
    if (!fc) return false;
    fc << fixed << setprecision(6);
    fc << "connection_index,src,dst,net_count,allocated_nets,unallocated_nets,piece_count,unique_candidate_count,avg_piece_score,max_dominant_share,used_hard_fallback\n";
    for (const auto& st : connStatus) {
        fc << st.connectionIndex << ','
           << st.srcBlock << ','
           << st.dstBlock << ','
           << st.netCount << ','
           << st.allocatedNets << ','
           << st.unallocatedNets << ','
           << st.pieceCount << ','
           << st.uniqueCandidateCount << ','
           << st.avgPieceScore << ','
           << st.maxDominantShare << ','
           << yesNo(st.usedHardFallback) << "\n";
    }
    fc.close();

    ofstream fr(resPath);
    if (!fr) return false;
    fr << fixed << setprecision(6);
    fr << "resource_type,name,index,edge,hard_cap,soft_cap,ambient,criticality,history,used,overflow_hard,overflow_soft\n";
    for (const auto& r : modelAfter.channelResources()) {
        const double hardOvLR = max(0.0, r.usedLR - r.hardCapLR);
        const double hardOvTB = max(0.0, r.usedTB - r.hardCapTB);
        const double softOvLR = max(0.0, r.usedLR + r.ambientLR - r.softCapLR);
        const double softOvTB = max(0.0, r.usedTB + r.ambientTB - r.softCapTB);
        fr << "CHANNEL_LR," << r.channelName << ',' << r.channelIndex << ",0,"
           << r.hardCapLR << ',' << r.softCapLR << ',' << r.ambientLR << ','
           << r.criticalityLR << ',' << r.historyLR << ',' << r.usedLR << ','
           << hardOvLR << ',' << softOvLR << "\n";
        fr << "CHANNEL_TB," << r.channelName << ',' << r.channelIndex << ",0,"
           << r.hardCapTB << ',' << r.softCapTB << ',' << r.ambientTB << ','
           << r.criticalityTB << ',' << r.historyTB << ',' << r.usedTB << ','
           << hardOvTB << ',' << softOvTB << "\n";
    }
    for (const auto& r : modelAfter.softFtResources()) {
        double overflowSoft = 0.0;
        if (r.blockIndex >= 0 && r.blockIndex < static_cast<int>(design.blocks.size())) {
            const BlockInst& b = design.blocks[r.blockIndex];
            const double reqArea = requiredAreaWithCurrentLocal(b, r.usedNets);
            overflowSoft = max(0.0, reqArea - max(EPS, b.rect.w * b.rect.h));
        }
        fr << "SOFT_FT," << r.blockName << ',' << r.blockIndex << ",0,"
           << r.currentArea << ',' << r.baseArea << ",0,0,1,"
           << r.usedNets << ",0," << overflowSoft << "\n";
    }
    for (const auto& r : modelAfter.blockAccessResources()) {
        fr << "BLOCK_ACCESS," << r.blockName << ',' << r.blockIndex << ',' << r.edge
           << ",0,0,0," << r.severity << ",1,0,0,0\n";
    }
    fr.close();

    ofstream froute(routePath);
    if (!froute) return false;
    froute << "connection_index,candidate_index,family,allocated_nets,steps\n";
    for (const auto& p : pieces) {
        froute << p.connectionIndex << ','
               << p.candidateIndex << ','
               << p.family << ','
               << p.allocatedNets << ','
               << '"' << joinPathSteps(p.path.steps) << '"' << "\n";
    }
    froute.close();

    return true;
}
