#include "RouterPhase3.hpp"

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
#include <unordered_set>

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

double canonicalChannelOverflow(const EvalReport& eval, double totalHardOverflow) {
    return max(eval.totalChannelOverflow, totalHardOverflow);
}

bool phase3StrictConverged(const EvalReport& eval, double totalHardOverflow) {
    return eval.openPathCount == 0
        && !eval.pathInvalid
        && eval.invalidPathCount == 0
        && canonicalChannelOverflow(eval, totalHardOverflow) <= 1.0e-9
        && eval.totalFeedthroughOverflow <= 1.0e-9;
}

double phase3Objective(const EvalReport& eval, double totalHardOverflow) {
    // Prioritize legality and overflow before wirelength. Channel overflow is
    // counted once through the canonical checker/model maximum. FT overflow is
    // weighted high enough to influence negotiated best-snapshot selection;
    // otherwise every channel improvement would dominate large FT regressions.
    const double failPenalty = eval.hasFail() ? 1.0e14 : 0.0;
    const double openPenalty = static_cast<double>(eval.openPathCount) * 1.0e12;
    const double invalidPenalty = static_cast<double>(eval.invalidPathCount) * 1.0e11;
    const double chPenalty = canonicalChannelOverflow(eval, totalHardOverflow) * 1.0e6;
    const double ftPenalty = eval.totalFeedthroughOverflow * 5.0e4;
    const double wlPenalty = eval.totalWireLength * 1.0e-3;
    return failPenalty + openPenalty + invalidPenalty + chPenalty + ftPenalty + wlPenalty;
}

} // namespace

int RouterPhase3::chooseChunkSize(int remaining, int allocatedSoFar) const {
    (void)allocatedSoFar;
    for (int c : opt_.chunkSizes) {
        if (c <= 0) continue;
        if (remaining >= c) return c;
    }
    return max(1, remaining);
}

Phase1ResourceDelta RouterPhase3::scaleDelta(const Phase1ResourceDelta& base, double scale) const {
    Phase1ResourceDelta d = base;
    const double s = max(0.0, scale);
    for (auto& kv : d.channelLR) kv.second *= s;
    for (auto& kv : d.channelTB) kv.second *= s;
    for (auto& kv : d.softFtNets) kv.second *= s;
    for (auto& t : d.blockEdgeUse) get<2>(t) *= s;
    d.wireLength *= s;
    return d;
}

RoutePath RouterPhase3::scalePath(const RoutePath& base, int nets) const {
    RoutePath p = base;
    const double denom = max(1, base.netCount);
    const double scale = static_cast<double>(nets) / static_cast<double>(denom);
    p.netCount = nets;
    p.wireLength = base.wireLength * scale;
    return p;
}

double RouterPhase3::evalHardOverflow(const RoutingResourceModel& model, const Phase1ResourceDelta& d) const {
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

double RouterPhase3::evalSoftOverflow(const RoutingResourceModel& model, const Phase1ResourceDelta& d) const {
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
        if (!model.softFeasibleSoftFt(kv.first, kv.second)) ov += kv.second;
    }
    return ov;
}

pair<int, int> RouterPhase3::dominantComponent(const Phase1ResourceDelta& d) const {
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

double RouterPhase3::connectionPriority(const Connection& conn, const Step0ConnectionGuide* guide, int maxNetCount) const {
    const double netNorm = static_cast<double>(conn.netCount) / static_cast<double>(max(1, maxNetCount));
    if (!guide) return netNorm;

    const double routePriority = clampLocal(guide->routePriority, 0.0, 1.5);
    const double openRisk = clampLocal(guide->openRiskScore, 0.0, 1.5);
    const double endpointAccess = guide->endpointAccessRisk ? 1.0 : 0.0;
    const double endpointBottleneck = guide->endpointBottleneckRisk ? 1.0 : 0.0;
    const double sharedBottleneck = guide->sharedBottleneckRisk ? 1.0 : 0.0;
    const double lackAlt = clampLocal(guide->lackOfAlternative, 0.0, 1.5);
    const double coupling = clampLocal(guide->couplingRiskScore, 0.0, 1.5);

    return 4.0 * openRisk
        + 3.0 * endpointBottleneck
        + 2.5 * sharedBottleneck
        + 2.0 * lackAlt
        + 1.5 * routePriority
        + 1.0 * endpointAccess
        + 0.8 * coupling
        + 1.0 * netNorm;
}

double RouterPhase3::guideAdjustment(const Step0ConnectionGuide* guide, const Phase1ResourceDelta& d, int chunkNets) const {
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

double RouterPhase3::negotiatedPenalty(
    const RoutingResourceModel& model,
    const Phase1ResourceDelta& d,
    const vector<double>& histLR,
    const vector<double>& histTB,
    const vector<double>& histFt,
    double presentFactor
) const {
    double s = 0.0;

    map<int, pair<double, double>> use;
    for (const auto& kv : d.channelLR) use[kv.first].first += kv.second;
    for (const auto& kv : d.channelTB) use[kv.first].second += kv.second;

    const auto& cr = model.channelResources();
    for (const auto& kv : use) {
        const int ci = kv.first;
        if (ci < 0 || ci >= static_cast<int>(cr.size())) continue;
        const auto& r = cr[ci];

        const double addLR = kv.second.first;
        if (addLR > EPS) {
            const double utilSoft = (r.usedLR + r.ambientLR + addLR) / max(EPS, r.softCapLR);
            const double ovHard = max(0.0, r.usedLR + addLR - r.hardCapLR);
            const double hist = (ci < static_cast<int>(histLR.size())) ? histLR[ci] : 0.0;
            s += addLR * (
                hist +
                presentFactor * 12.0 * max(0.0, utilSoft - 0.80) * max(0.0, utilSoft - 0.80) +
                presentFactor * 120.0 * ovHard / max(1.0, r.hardCapLR)
                );
        }

        const double addTB = kv.second.second;
        if (addTB > EPS) {
            const double utilSoft = (r.usedTB + r.ambientTB + addTB) / max(EPS, r.softCapTB);
            const double ovHard = max(0.0, r.usedTB + addTB - r.hardCapTB);
            const double hist = (ci < static_cast<int>(histTB.size())) ? histTB[ci] : 0.0;
            s += addTB * (
                hist +
                presentFactor * 12.0 * max(0.0, utilSoft - 0.80) * max(0.0, utilSoft - 0.80) +
                presentFactor * 120.0 * ovHard / max(1.0, r.hardCapTB)
                );
        }
    }

    for (const auto& kv : d.softFtNets) {
        const int bi = kv.first;
        const double add = kv.second;
        if (add <= EPS) continue;
        const double hist = (bi >= 0 && bi < static_cast<int>(histFt.size())) ? histFt[bi] : 0.0;
        s += add * hist;
    }

    return s;
}

double RouterPhase3::dynamicFtPenalty(
    const Design& design,
    const RoutingResourceModel& model,
    const Phase1ResourceDelta& d,
    const vector<double>& histFt,
    double presentFactor
) const {
    double penalty = 0.0;
    const double pf = max(1.0, presentFactor);
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

        penalty += add * (35.0 + 145.0 * hist);
        penalty += pf * 0.18 * incOv;
        penalty += pf * 0.055 * projOv;
        penalty += add * pf * 800.0 * max(0.0, areaUtil - 1.0) * max(0.0, areaUtil - 1.0);
    }
    return penalty;
}

void RouterPhase3::updateFtHistoryFromModel(
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
        const double overflowTerm = ov > 1.0e-9 ? opt_.ftHistoryStep * min(14.0, 1.0 + log1p(ov) / 2.0) : 0.0;
        histFt[bi] = min(opt_.ftHistoryCap * 2.0, histFt[bi] + usageTerm + overflowTerm);
    }
}

double RouterPhase3::marginalScore(
    const Design& design,
    const RoutingResourceModel& model,
    const Phase1ResourceDelta& d,
    double diversityPenalty,
    const vector<double>& histLR,
    const vector<double>& histTB,
    const vector<double>& histFt,
    double presentFactor
) const {
    double s = 0.0;
    s += 0.20 * d.wireLength;
    s += 40.0 * static_cast<double>(d.bendCount);

    map<int, pair<double, double>> use;
    for (const auto& kv : d.channelLR) use[kv.first].first += kv.second;
    for (const auto& kv : d.channelTB) use[kv.first].second += kv.second;
    for (const auto& kv : use) s += model.scoreChannelUse(kv.first, kv.second.first, kv.second.second);

    for (const auto& kv : d.softFtNets) s += model.scoreSoftFtUse(kv.first, kv.second);
    for (const auto& t : d.blockEdgeUse) s += model.scoreBlockAccessUse(get<0>(t), get<1>(t), get<2>(t));

    s += negotiatedPenalty(model, d, histLR, histTB, histFt, presentFactor);
    s += dynamicFtPenalty(design, model, d, histFt, presentFactor);
    s += diversityPenalty;
    return s;
}

int RouterPhase3::centerFacingEdge(const Design& design, int blockIndex) const {
    const Rect& r = design.blocks[blockIndex].rect;
    const double ocx = design.outlineW * 0.5;
    const double ocy = design.outlineH * 0.5;
    const double dx = ocx - rectCx(r);
    const double dy = ocy - rectCy(r);
    if (fabs(dx) >= fabs(dy)) return dx >= 0.0 ? 3 : 1;
    return dy >= 0.0 ? 2 : 4;
}

RoutePath RouterPhase3::makeOpenPath(const Design& design, const Connection& conn, int nets) const {
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

bool RouterPhase3::allocateConnection(
    const Design& design,
    int ci,
    const vector<Phase1RouteCandidate>& candVec,
    const Step0ConnectionGuide* guide,
    RoutingResourceModel& model,
    MutableConnectionState& state,
    int assignedIteration,
    bool allowHardFallback,
    const vector<double>& histLR,
    const vector<double>& histTB,
    vector<double>& histFt,
    double presentFactor,
    int& hardFallbackPieces
) const {
    state.pieces.clear();
    state.usedHardFallback = false;
    if (candVec.empty()) return false;

    int remaining = state.netCount;
    int allocated = 0;
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
                if (share > 0.80) diversityPenalty = (share - 0.80) * static_cast<double>(chunk) * 200.0;
            }

            ScoredCandidate sc;
            sc.candidateIndex = base.candidateIndex;
            sc.ftDynamicPenalty = dynamicFtPenalty(design, model, d, histFt, presentFactor);
            sc.score = marginalScore(design, model, d, diversityPenalty, histLR, histTB, histFt, presentFactor)
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
        if (!chosen && allowHardFallback) {
            chosen = &scored.front();
            state.usedHardFallback = true;
            hardFallbackPieces += 1;
        }
        if (!chosen) break;

        model.commit(chosen->delta);
        updateFtHistoryFromModel(design, model, histFt);

        Phase3AllocationPiece piece;
        piece.connectionIndex = ci;
        piece.candidateIndex = chosen->candidateIndex;
        piece.family = candVec[chosen->candidateIndex].family;
        piece.allocatedNets = chunk;
        piece.assignedIteration = assignedIteration;
        piece.score = chosen->score;
        piece.hardFeasible = chosen->hardFeasible;
        piece.softFeasible = chosen->softFeasible;
        piece.hardOverflowAmount = chosen->hardOverflowAmount;
        piece.softOverflowAmount = chosen->softOverflowAmount;
        piece.ftDynamicPenalty = chosen->ftDynamicPenalty;
        piece.path = chosen->path;
        piece.delta = chosen->delta;
        state.pieces.push_back(move(piece));

        allocated += chunk;
        remaining -= chunk;
        usedCandidates.insert(chosen->candidateIndex);
        scoreSum += chosen->score;
        dominantLoad[chosen->dominantComp] += chunk;
    }

    (void)design;
    return remaining == 0;
}

void RouterPhase3::ripupConnection(RoutingResourceModel& model, MutableConnectionState& state) const {
    for (const auto& p : state.pieces) model.ripup(p.delta);
    state.pieces.clear();
}

RouterPhase3::OverflowSnapshot RouterPhase3::computeOverflowSnapshot(const Design& design, const RoutingResourceModel& model) const {
    OverflowSnapshot ov;
    ov.channelOverflowLR.assign(design.channels.size(), 0.0);
    ov.channelOverflowTB.assign(design.channels.size(), 0.0);
    ov.softFtOverflow.assign(design.blocks.size(), 0.0);

    const auto& cr = model.channelResources();
    for (const auto& r : cr) {
        const double ovLR = max(0.0, r.usedLR - r.hardCapLR);
        const double ovTB = max(0.0, r.usedTB - r.hardCapTB);
        if (r.channelIndex >= 0 && r.channelIndex < static_cast<int>(design.channels.size())) {
            ov.channelOverflowLR[r.channelIndex] = ovLR;
            ov.channelOverflowTB[r.channelIndex] = ovTB;
        }
        const double sum = ovLR + ovTB;
        if (sum > 1.0e-9) ov.overflowChannelCount += 1;
        ov.totalHardOverflow += sum;
        ov.maxHardOverflow = max(ov.maxHardOverflow, sum);

        ov.totalSoftOverflow += max(0.0, r.usedLR + r.ambientLR - r.softCapLR);
        ov.totalSoftOverflow += max(0.0, r.usedTB + r.ambientTB - r.softCapTB);
    }

    const auto& ft = model.softFtResources();
    for (const auto& r : ft) {
        if (r.blockIndex < 0 || r.blockIndex >= static_cast<int>(design.blocks.size())) continue;
        const BlockInst& b = design.blocks[r.blockIndex];
        const double reqArea = requiredAreaWithCurrentLocal(b, r.usedNets);
        const double overflow = max(0.0, reqArea - max(EPS, b.rect.w * b.rect.h));
        ov.softFtOverflow[r.blockIndex] = overflow;
        ov.totalSoftOverflow += overflow;
        if (overflow > 1.0e-9) ov.overflowSoftFtBlockCount += 1;
    }

    return ov;
}

void RouterPhase3::updateHistoryFromOverflow(
    const OverflowSnapshot& ov,
    vector<double>& histLR,
    vector<double>& histTB,
    vector<double>& histFt
) const {
    for (int i = 0; i < static_cast<int>(ov.channelOverflowLR.size()) && i < static_cast<int>(histLR.size()); ++i) {
        const double o = ov.channelOverflowLR[i];
        if (o <= 1.0e-9) continue;
        const double inc = opt_.historyStep * min(4.0, 1.0 + log1p(o));
        histLR[i] = min(opt_.historyCap, histLR[i] + inc);
    }
    for (int i = 0; i < static_cast<int>(ov.channelOverflowTB.size()) && i < static_cast<int>(histTB.size()); ++i) {
        const double o = ov.channelOverflowTB[i];
        if (o <= 1.0e-9) continue;
        const double inc = opt_.historyStep * min(4.0, 1.0 + log1p(o));
        histTB[i] = min(opt_.historyCap, histTB[i] + inc);
    }
    for (int i = 0; i < static_cast<int>(ov.softFtOverflow.size()) && i < static_cast<int>(histFt.size()); ++i) {
        const double o = ov.softFtOverflow[i];
        if (o <= 1.0e-9) continue;
        const double inc = opt_.ftHistoryStep * min(4.0, 1.0 + log1p(o));
        histFt[i] = min(opt_.ftHistoryCap, histFt[i] + inc);
    }
}

void RouterPhase3::computeContributorScores(
    const OverflowSnapshot& ov,
    vector<MutableConnectionState>& states
) const {
    for (auto& st : states) {
        double score = 0.0;
        for (const auto& p : st.pieces) {
            for (const auto& kv : p.delta.channelLR) {
                const int ci = kv.first;
                if (ci >= 0 && ci < static_cast<int>(ov.channelOverflowLR.size())) score += kv.second * ov.channelOverflowLR[ci];
            }
            for (const auto& kv : p.delta.channelTB) {
                const int ci = kv.first;
                if (ci >= 0 && ci < static_cast<int>(ov.channelOverflowTB.size())) score += kv.second * ov.channelOverflowTB[ci];
            }
            for (const auto& kv : p.delta.softFtNets) {
                const int bi = kv.first;
                if (bi >= 0 && bi < static_cast<int>(ov.softFtOverflow.size())) score += kv.second * ov.softFtOverflow[bi];
            }
        }

        int allocated = 0;
        for (const auto& p : st.pieces) allocated += p.allocatedNets;
        if (allocated < st.netCount) {
            // Open connection has highest priority for reroute.
            score += 1.0e9 + static_cast<double>(st.netCount - allocated) * 1.0e6;
        }
        st.contributorScore = score;
    }
}

vector<int> RouterPhase3::selectRipupSet(const OverflowSnapshot& ov, const vector<MutableConnectionState>& states) const {
    vector<int> cand;
    cand.reserve(states.size());
    for (const auto& st : states) {
        int allocated = 0;
        for (const auto& p : st.pieces) allocated += p.allocatedNets;
        const bool isOpen = allocated < st.netCount;
        if (isOpen || st.contributorScore > 1.0e-9) cand.push_back(st.connectionIndex);
    }
    if (cand.empty()) return {};

    const int desiredByRatio = static_cast<int>(ceil(opt_.ripupRatio * static_cast<double>(cand.size())));
    int target = max(opt_.minRipupConnections, desiredByRatio);
    target = min(target, opt_.maxRipupConnections);
    target = min(target, static_cast<int>(cand.size()));

    sort(cand.begin(), cand.end(), [&](int a, int b) {
        if (fabs(states[a].contributorScore - states[b].contributorScore) > 1.0e-9) {
            return states[a].contributorScore > states[b].contributorScore;
        }
        if (states[a].netCount != states[b].netCount) return states[a].netCount > states[b].netCount;
        return a < b;
    });

    cand.resize(target);

    // If there is no overflow and no open connection, do not keep rerouting.
    if (ov.totalHardOverflow <= 1.0e-9 && ov.overflowSoftFtBlockCount == 0) {
        bool hasOpen = false;
        for (const auto& st : states) {
            int allocated = 0;
            for (const auto& p : st.pieces) allocated += p.allocatedNets;
            if (allocated < st.netCount) {
                hasOpen = true;
                break;
            }
        }
        if (!hasOpen) return {};
    }

    return cand;
}

vector<Phase3ConnectionStatus> RouterPhase3::buildConnectionStatus(
    const Design& design,
    const vector<MutableConnectionState>& states
) const {
    vector<Phase3ConnectionStatus> out(states.size());
    for (int ci = 0; ci < static_cast<int>(states.size()); ++ci) {
        const auto& s = states[ci];
        const Connection& conn = design.connections[ci];
        Phase3ConnectionStatus st;
        st.connectionIndex = ci;
        st.srcBlock = design.blocks[conn.src].spec.name;
        st.dstBlock = design.blocks[conn.dst].spec.name;
        st.netCount = conn.netCount;
        st.rerouteCount = s.rerouteCount;
        st.usedHardFallback = s.usedHardFallback;
        st.contributorScore = s.contributorScore;

        int allocated = 0;
        double scoreSum = 0.0;
        int maxLoad = 0;
        map<pair<int, int>, int> domLoad;
        set<int> uniqueCandidates;
        for (const auto& p : s.pieces) {
            allocated += p.allocatedNets;
            scoreSum += p.score;
            uniqueCandidates.insert(p.candidateIndex);
            const auto dom = dominantComponent(p.delta);
            domLoad[dom] += p.allocatedNets;
            maxLoad = max(maxLoad, domLoad[dom]);
        }
        st.allocatedNets = allocated;
        st.unallocatedNets = max(0, st.netCount - allocated);
        st.pieceCount = static_cast<int>(s.pieces.size());
        st.uniqueCandidateCount = static_cast<int>(uniqueCandidates.size());
        st.avgPieceScore = st.pieceCount > 0 ? scoreSum / static_cast<double>(st.pieceCount) : 0.0;
        st.maxDominantShare = allocated > 0 ? static_cast<double>(maxLoad) / static_cast<double>(allocated) : 0.0;
        st.isOpen = st.unallocatedNets > 0;

        out[ci] = st;
    }
    return out;
}

Design RouterPhase3::buildPhase3RoutedDesign(const Design& design, const vector<MutableConnectionState>& states) const {
    Design out = design;
    out.routes.clear();

    for (int ci = 0; ci < static_cast<int>(states.size()); ++ci) {
        const auto& st = states[ci];
        for (const auto& p : st.pieces) {
            if (p.allocatedNets <= 0) continue;
            out.routes.push_back(p.path);
        }
        int allocated = 0;
        for (const auto& p : st.pieces) allocated += p.allocatedNets;
        const int unallocated = max(0, st.netCount - allocated);
        if (unallocated > 0) out.routes.push_back(makeOpenPath(design, design.connections[ci], unallocated));
    }

    return out;
}

RouterPhase3::RunResult RouterPhase3::run(const Design& design, const string& inputPath, const string& outputCfgPath, double alpha) const {
    RouterPhase1::Options p1opt;
    p1opt.maxCandidatesPerConnection = opt_.maxCandidatesPerConnection;
    p1opt.exportFiles = false;
    RouterPhase1 phase1(p1opt);
    RouterPhase1::CandidateBuildResult built = phase1.buildCandidates(design);
    return run(design, inputPath, outputCfgPath, alpha, built);
}

RouterPhase3::RunResult RouterPhase3::run(
    const Design& design,
    const string& inputPath,
    const string& outputCfgPath,
    double alpha,
    const RouterPhase1::CandidateBuildResult& built
) const {
    RunResult rr;
    rr.ok = built.ok && built.candidates.size() == design.connections.size();
    rr.totalConnections = static_cast<int>(design.connections.size());
    if (!rr.ok) return rr;

    RoutingResourceModel model;
    model.initialize(design, built.step0);

    vector<MutableConnectionState> states(design.connections.size());
    for (int ci = 0; ci < static_cast<int>(states.size()); ++ci) {
        states[ci].connectionIndex = ci;
        states[ci].netCount = design.connections[ci].netCount;
    }

    int maxNetCount = 1;
    for (const auto& c : design.connections) maxNetCount = max(maxNetCount, c.netCount);

    vector<int> connOrder(design.connections.size());
    for (int i = 0; i < static_cast<int>(connOrder.size()); ++i) connOrder[i] = i;
    sort(connOrder.begin(), connOrder.end(), [&](int a, int b) {
        const Step0ConnectionGuide* ga = a < static_cast<int>(built.step0.connectionGuide.size()) ? &built.step0.connectionGuide[a] : nullptr;
        const Step0ConnectionGuide* gb = b < static_cast<int>(built.step0.connectionGuide.size()) ? &built.step0.connectionGuide[b] : nullptr;
        const double pa = connectionPriority(design.connections[a], ga, maxNetCount);
        const double pb = connectionPriority(design.connections[b], gb, maxNetCount);
        if (fabs(pa - pb) > 1.0e-9) return pa > pb;
        if (design.connections[a].netCount != design.connections[b].netCount) return design.connections[a].netCount > design.connections[b].netCount;
        return a < b;
    });

    vector<double> histLR(design.channels.size(), 0.0);
    vector<double> histTB(design.channels.size(), 0.0);
    vector<double> histFt(design.blocks.size(), 0.0);
    double presentFactor = opt_.initialPresentFactor;

    int hardFallbackPieces = 0;
    for (int ci : connOrder) {
        const auto& candVec = built.candidates[ci];
        const Step0ConnectionGuide* guide = ci < static_cast<int>(built.step0.connectionGuide.size()) ? &built.step0.connectionGuide[ci] : nullptr;
        allocateConnection(
            design, ci, candVec, guide, model, states[ci], 0,
            opt_.allowInitialHardFallback, histLR, histTB, histFt, presentFactor, hardFallbackPieces
        );
    }

    Evaluator evaluator;
    vector<Phase3IterationRecord> iterRecs;
    Design curDesign = buildPhase3RoutedDesign(design, states);
    EvalReport curEval = evaluator.evaluate(curDesign, alpha);
    OverflowSnapshot curOv = computeOverflowSnapshot(design, model);

    Phase3IterationRecord iter0;
    iter0.iteration = 0;
    iter0.rippedConnections = 0;
    iter0.reroutedConnections = 0;
    iter0.openConnections = curEval.openPathCount;
    iter0.overflowChannelCount = curOv.overflowChannelCount;
    iter0.overflowSoftFtBlockCount = curOv.overflowSoftFtBlockCount;
    iter0.totalHardOverflow = curOv.totalHardOverflow;
    iter0.maxHardOverflow = curOv.maxHardOverflow;
    iter0.totalSoftOverflow = curOv.totalSoftOverflow;
    iter0.evaluatorChannelOverflow = curEval.totalChannelOverflow;
    iter0.evaluatorFeedthroughOverflow = curEval.totalFeedthroughOverflow;
    iter0.evaluatorWireLength = curEval.totalWireLength;
    iter0.evaluatorHasFail = curEval.hasFail();
    iterRecs.push_back(iter0);

    rr.initialOpenConnections = curEval.openPathCount;
    rr.initialTotalHardOverflow = curOv.totalHardOverflow;
    rr.initialTotalChannelOverflowEval = curEval.totalChannelOverflow;
    rr.initialTotalFeedthroughOverflowEval = curEval.totalFeedthroughOverflow;

    double bestObj = phase3Objective(curEval, curOv.totalHardOverflow);
    vector<MutableConnectionState> bestStates = states;
    RoutingResourceModel bestModel = model;
    EvalReport bestEval = curEval;
    OverflowSnapshot bestOv = curOv;
    Design bestDesign = curDesign;

    int stagnantRounds = 0;
    int totalRipped = 0;
    int totalRerouted = 0;

    for (int iter = 1; iter <= opt_.maxIterations; ++iter) {
        if (phase3StrictConverged(curEval, curOv.totalHardOverflow)) {
            rr.converged = true;
            rr.iterationsRun = iter - 1;
            break;
        }

        updateHistoryFromOverflow(curOv, histLR, histTB, histFt);
        presentFactor *= opt_.presentFactorGrowth;

        computeContributorScores(curOv, states);
        vector<int> ripSet = selectRipupSet(curOv, states);
        if (ripSet.empty()) {
            rr.iterationsRun = iter - 1;
            break;
        }

        totalRipped += static_cast<int>(ripSet.size());

        // Rip-up selected contributors first.
        for (int ci : ripSet) ripupConnection(model, states[ci]);

        const bool allowFallback = opt_.allowRerouteHardFallback || (iter >= opt_.hardFallbackEnableIteration);
        int rerouted = 0;

        sort(ripSet.begin(), ripSet.end(), [&](int a, int b) {
            if (fabs(states[a].contributorScore - states[b].contributorScore) > 1.0e-9) {
                return states[a].contributorScore > states[b].contributorScore;
            }
            const Step0ConnectionGuide* ga = a < static_cast<int>(built.step0.connectionGuide.size()) ? &built.step0.connectionGuide[a] : nullptr;
            const Step0ConnectionGuide* gb = b < static_cast<int>(built.step0.connectionGuide.size()) ? &built.step0.connectionGuide[b] : nullptr;
            const double pa = connectionPriority(design.connections[a], ga, maxNetCount);
            const double pb = connectionPriority(design.connections[b], gb, maxNetCount);
            if (fabs(pa - pb) > 1.0e-9) return pa > pb;
            return a < b;
        });

        for (int ci : ripSet) {
            states[ci].rerouteCount += 1;
            const auto& candVec = built.candidates[ci];
            const Step0ConnectionGuide* guide = ci < static_cast<int>(built.step0.connectionGuide.size()) ? &built.step0.connectionGuide[ci] : nullptr;
            if (allocateConnection(
                design, ci, candVec, guide, model, states[ci], iter,
                allowFallback, histLR, histTB, histFt, presentFactor, hardFallbackPieces
            )) {
                rerouted += 1;
            }
        }
        totalRerouted += rerouted;

        curDesign = buildPhase3RoutedDesign(design, states);
        curEval = evaluator.evaluate(curDesign, alpha);
        curOv = computeOverflowSnapshot(design, model);

        Phase3IterationRecord rec;
        rec.iteration = iter;
        rec.rippedConnections = static_cast<int>(ripSet.size());
        rec.reroutedConnections = rerouted;
        rec.openConnections = curEval.openPathCount;
        rec.overflowChannelCount = curOv.overflowChannelCount;
        rec.overflowSoftFtBlockCount = curOv.overflowSoftFtBlockCount;
        rec.totalHardOverflow = curOv.totalHardOverflow;
        rec.maxHardOverflow = curOv.maxHardOverflow;
        rec.totalSoftOverflow = curOv.totalSoftOverflow;
        rec.evaluatorChannelOverflow = curEval.totalChannelOverflow;
        rec.evaluatorFeedthroughOverflow = curEval.totalFeedthroughOverflow;
        rec.evaluatorWireLength = curEval.totalWireLength;
        rec.evaluatorHasFail = curEval.hasFail();
        iterRecs.push_back(rec);

        const double obj = phase3Objective(curEval, curOv.totalHardOverflow);
        if (obj + 1.0e-6 < bestObj) {
            bestObj = obj;
            bestStates = states;
            bestModel = model;
            bestEval = curEval;
            bestOv = curOv;
            bestDesign = curDesign;
            stagnantRounds = 0;
        } else {
            stagnantRounds += 1;
        }

        rr.iterationsRun = iter;
        if (stagnantRounds >= 3) break;
    }

    // Use best snapshot in case the last round regresses.
    states = move(bestStates);
    model = move(bestModel);
    curEval = bestEval;
    curOv = bestOv;
    curDesign = move(bestDesign);

    rr.totalPieces = 0;
    for (const auto& st : states) {
        rr.totalPieces += static_cast<int>(st.pieces.size());
        for (const auto& p : st.pieces) {
            if (p.ftDynamicPenalty > 1.0e-9) {
                rr.dynamicFtPricedPieces += 1;
                rr.totalDynamicFtPenalty += p.ftDynamicPenalty;
            }
        }
    }
    for (double h : histFt) rr.maxFtHistory = max(rr.maxFtHistory, h);
    rr.hardFallbackPieces = hardFallbackPieces;
    rr.totalRippedConnections = totalRipped;
    rr.totalReroutedConnections = totalRerouted;
    rr.finalOpenConnections = curEval.openPathCount;
    rr.finalTotalHardOverflow = curOv.totalHardOverflow;
    rr.finalTotalChannelOverflowEval = curEval.totalChannelOverflow;
    rr.finalTotalFeedthroughOverflowEval = curEval.totalFeedthroughOverflow;
    rr.finalEvalHasFail = curEval.hasFail();
    if (phase3StrictConverged(curEval, curOv.totalHardOverflow)) rr.converged = true;

    vector<Phase3ConnectionStatus> connStatus = buildConnectionStatus(design, states);

    if (opt_.exportFiles) {
        fs::path outPath(outputCfgPath);
        const fs::path stem = outPath.parent_path() / outPath.stem();
        const string phase3CfgPath = stem.string() + "_phase3.cfg";
        OutputWriter writer;
        if (!writer.write(phase3CfgPath, curDesign)) {
            rr.ok = false;
        } else {
            rr.ok = writeReports(
                design, built, model, states, connStatus, iterRecs, curEval,
                phase3CfgPath, rr, inputPath, outputCfgPath, alpha
            );
        }
    }

    return rr;
}

bool RouterPhase3::writeReports(
    const Design& design,
    const RouterPhase1::CandidateBuildResult& phase1,
    const RoutingResourceModel& modelAfter,
    const vector<MutableConnectionState>& states,
    const vector<Phase3ConnectionStatus>& connStatus,
    const vector<Phase3IterationRecord>& iters,
    const EvalReport& finalEval,
    const string& phase3CfgPath,
    const RunResult& rr,
    const string& inputPath,
    const string& outputCfgPath,
    double alpha
) const {
    fs::path outPath(outputCfgPath);
    fs::path reportDir("Router_Statistics");
    fs::create_directories(reportDir);
    const fs::path stem = reportDir / outPath.stem();

    const string summaryPath = (stem.string() + "_phase3_summary.txt");
    const string iterPath = (stem.string() + "_phase3_iterations.csv");
    const string connPath = (stem.string() + "_phase3_connection_status.csv");
    const string resPath = (stem.string() + "_phase3_resource_after.csv");
    const string routePath = (stem.string() + "_phase3_selected_routes.csv");

    ofstream fsum(summaryPath);
    if (!fsum) return false;
    fsum << fixed << setprecision(6);
    fsum << "Phase 3 Negotiated Rip-up and Reroute Summary\n";
    fsum << "input=" << inputPath << "\n";
    fsum << "requested_output=" << outputCfgPath << "\n";
    fsum << "phase3_output_cfg=" << phase3CfgPath << "\n";
    fsum << "alpha=" << alpha << "\n";
    fsum << "connections=" << rr.totalConnections << "\n";
    fsum << "iterations_run=" << rr.iterationsRun << "\n";
    fsum << "converged=" << yesNo(rr.converged) << "\n";
    fsum << "total_pieces=" << rr.totalPieces << "\n";
    fsum << "hard_fallback_pieces=" << rr.hardFallbackPieces << "\n";
    fsum << "dynamic_ft_pricing=ON\n";
    fsum << "dynamic_ft_priced_pieces=" << rr.dynamicFtPricedPieces << "\n";
    fsum << "dynamic_ft_total_penalty=" << rr.totalDynamicFtPenalty << "\n";
    fsum << "dynamic_ft_max_history=" << rr.maxFtHistory << "\n";
    fsum << "total_ripped_connections=" << rr.totalRippedConnections << "\n";
    fsum << "total_rerouted_connections=" << rr.totalReroutedConnections << "\n";
    fsum << "initial_open_connections=" << rr.initialOpenConnections << "\n";
    fsum << "final_open_connections=" << rr.finalOpenConnections << "\n";
    fsum << "initial_total_hard_overflow=" << rr.initialTotalHardOverflow << "\n";
    fsum << "final_total_hard_overflow=" << rr.finalTotalHardOverflow << "\n";
    fsum << "initial_eval_channel_overflow=" << rr.initialTotalChannelOverflowEval << "\n";
    fsum << "final_eval_channel_overflow=" << rr.finalTotalChannelOverflowEval << "\n";
    fsum << "initial_eval_feedthrough_overflow=" << rr.initialTotalFeedthroughOverflowEval << "\n";
    fsum << "final_eval_feedthrough_overflow=" << rr.finalTotalFeedthroughOverflowEval << "\n";
    fsum << "final_eval_has_fail=" << yesNo(rr.finalEvalHasFail) << "\n";
    fsum << "final_eval_path_invalid=" << yesNo(finalEval.pathInvalid) << "\n";
    fsum << "final_eval_invalid_path_count=" << finalEval.invalidPathCount << "\n";
    fsum << "final_eval_wire_length=" << finalEval.totalWireLength << "\n";
    fsum << "final_eval_cost=" << finalEval.cost << "\n";
    fsum << "phase1_generated_candidates=" << phase1.stats.generatedCandidates << "\n";
    fsum << "phase1_connections_without_candidate=" << phase1.stats.connectionsWithoutCandidate << "\n";
    fsum.close();

    ofstream fi(iterPath);
    if (!fi) return false;
    fi << fixed << setprecision(6);
    fi << "iteration,ripped_connections,rerouted_connections,open_connections,overflow_channel_count,overflow_soft_ft_block_count,total_hard_overflow,max_hard_overflow,total_soft_overflow,eval_channel_overflow,eval_feedthrough_overflow,eval_wirelength,eval_has_fail\n";
    for (const auto& r : iters) {
        fi << r.iteration << ','
           << r.rippedConnections << ','
           << r.reroutedConnections << ','
           << r.openConnections << ','
           << r.overflowChannelCount << ','
           << r.overflowSoftFtBlockCount << ','
           << r.totalHardOverflow << ','
           << r.maxHardOverflow << ','
           << r.totalSoftOverflow << ','
           << r.evaluatorChannelOverflow << ','
           << r.evaluatorFeedthroughOverflow << ','
           << r.evaluatorWireLength << ','
           << yesNo(r.evaluatorHasFail) << "\n";
    }
    fi.close();

    ofstream fc(connPath);
    if (!fc) return false;
    fc << fixed << setprecision(6);
    fc << "connection_index,src,dst,net_count,allocated_nets,unallocated_nets,piece_count,unique_candidate_count,reroute_count,avg_piece_score,max_dominant_share,contributor_score,used_hard_fallback,is_open\n";
    for (const auto& st : connStatus) {
        fc << st.connectionIndex << ','
           << st.srcBlock << ','
           << st.dstBlock << ','
           << st.netCount << ','
           << st.allocatedNets << ','
           << st.unallocatedNets << ','
           << st.pieceCount << ','
           << st.uniqueCandidateCount << ','
           << st.rerouteCount << ','
           << st.avgPieceScore << ','
           << st.maxDominantShare << ','
           << st.contributorScore << ','
           << yesNo(st.usedHardFallback) << ','
           << yesNo(st.isOpen) << "\n";
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
    froute << "connection_index,candidate_index,family,allocated_nets,assigned_iteration,score,dynamic_ft_penalty,path_steps\n";
    for (const auto& st : states) {
        for (const auto& p : st.pieces) {
            froute << p.connectionIndex << ','
                   << p.candidateIndex << ','
                   << p.family << ','
                   << p.allocatedNets << ','
                   << p.assignedIteration << ','
                   << p.score << ','
                   << p.ftDynamicPenalty << ','
                   << '"' << joinPathSteps(p.path.steps) << '"' << "\n";
        }
    }
    froute.close();

    return true;
}
