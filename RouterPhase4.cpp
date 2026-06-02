#include "RouterPhase4.hpp"

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
#include <queue>
#include <set>
#include <sstream>
#include <tuple>
#include <unordered_set>

using namespace std;
namespace fs = std::filesystem;

namespace {

inline bool validEdge(int e) { return e >= 1 && e <= 4; }
inline bool isOppositeLR(int a, int b) { return (a == 1 && b == 3) || (a == 3 && b == 1); }
inline bool isOppositeTB(int a, int b) { return (a == 2 && b == 4) || (a == 4 && b == 2); }
inline bool isTurn(int a, int b) {
    if (!validEdge(a) || !validEdge(b)) return false;
    if (a == b) return false;
    return !isOppositeLR(a, b) && !isOppositeTB(a, b);
}

inline bool isChannelName(const string& s) {
    return s.rfind("CH", 0) == 0;
}

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

double canonicalChannelOverflow(const EvalReport& eval, double totalHardOverflow) {
    return max(eval.totalChannelOverflow, totalHardOverflow);
}

bool phase4StrictConverged(const EvalReport& eval, double totalHardOverflow) {
    return eval.openPathCount == 0
        && !eval.pathInvalid
        && eval.invalidPathCount == 0
        && canonicalChannelOverflow(eval, totalHardOverflow) <= 1.0e-9
        && eval.totalFeedthroughOverflow <= 1.0e-9;
}

double Phase4Objective(const EvalReport& eval, double totalHardOverflow) {
    // Prioritize legality and overflow before wirelength. Channel overflow is
    // counted once through the canonical checker/model maximum.
    const double failPenalty = eval.hasFail() ? 1.0e14 : 0.0;
    const double openPenalty = static_cast<double>(eval.openPathCount) * 1.0e12;
    const double invalidPenalty = static_cast<double>(eval.invalidPathCount) * 1.0e11;
    const double chPenalty = canonicalChannelOverflow(eval, totalHardOverflow) * 1.0e6;
    const double ftPenalty = eval.totalFeedthroughOverflow * 1.0;
    const double wlPenalty = eval.totalWireLength * 1.0e-3;
    return failPenalty + openPenalty + invalidPenalty + chPenalty + ftPenalty + wlPenalty;
}

} // namespace

int RouterPhase4::chooseChunkSize(int remaining, int allocatedSoFar) const {
    (void)allocatedSoFar;
    for (int c : opt_.chunkSizes) {
        if (c <= 0) continue;
        if (remaining >= c) return c;
    }
    return max(1, remaining);
}

Phase1ResourceDelta RouterPhase4::scaleDelta(const Phase1ResourceDelta& base, double scale) const {
    Phase1ResourceDelta d = base;
    const double s = max(0.0, scale);
    for (auto& kv : d.channelLR) kv.second *= s;
    for (auto& kv : d.channelTB) kv.second *= s;
    for (auto& kv : d.softFtNets) kv.second *= s;
    for (auto& t : d.blockEdgeUse) get<2>(t) *= s;
    d.wireLength *= s;
    return d;
}

RoutePath RouterPhase4::scalePath(const RoutePath& base, int nets) const {
    RoutePath p = base;
    const double denom = max(1, base.netCount);
    const double scale = static_cast<double>(nets) / static_cast<double>(denom);
    p.netCount = nets;
    p.wireLength = base.wireLength * scale;
    return p;
}

double RouterPhase4::evalHardOverflow(const RoutingResourceModel& model, const Phase1ResourceDelta& d) const {
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

double RouterPhase4::evalSoftOverflow(const RoutingResourceModel& model, const Phase1ResourceDelta& d) const {
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

pair<int, int> RouterPhase4::dominantComponent(const Phase1ResourceDelta& d) const {
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

double RouterPhase4::connectionPriority(const Connection& conn, const Step0ConnectionGuide* guide, int maxNetCount) const {
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

double RouterPhase4::guideAdjustment(const Step0ConnectionGuide* guide, const Phase1ResourceDelta& d, int chunkNets) const {
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

double RouterPhase4::negotiatedPenalty(
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

double RouterPhase4::marginalScore(
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
    s += diversityPenalty;
    return s;
}

int RouterPhase4::centerFacingEdge(const Design& design, int blockIndex) const {
    const Rect& r = design.blocks[blockIndex].rect;
    const double ocx = design.outlineW * 0.5;
    const double ocy = design.outlineH * 0.5;
    const double dx = ocx - rectCx(r);
    const double dy = ocy - rectCy(r);
    if (fabs(dx) >= fabs(dy)) return dx >= 0.0 ? 3 : 1;
    return dy >= 0.0 ? 2 : 4;
}

RoutePath RouterPhase4::makeOpenPath(const Design& design, const Connection& conn, int nets) const {
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

vector<RouterPhase4::Node> RouterPhase4::buildNodes(const Design& design) const {
    vector<Node> nodes;
    nodes.reserve(design.blocks.size() + design.channels.size());
    for (int i = 0; i < static_cast<int>(design.blocks.size()); ++i) {
        Node n;
        n.name = design.blocks[i].spec.name;
        n.rect = design.blocks[i].rect;
        n.isBlock = true;
        n.index = i;
        nodes.push_back(n);
    }
    for (int i = 0; i < static_cast<int>(design.channels.size()); ++i) {
        Node n;
        n.name = design.channels[i].name;
        n.rect = design.channels[i].rect;
        n.isBlock = false;
        n.index = i;
        nodes.push_back(n);
    }
    return nodes;
}

bool RouterPhase4::touchWithEdges(const Rect& a, const Rect& b, int& edgeA, int& edgeB) const {
    edgeA = edgeB = 0;
    const double eps = 1.0e-3;
    const double ovEps = 1.0e-7;

    const bool bAtRight = fabs((a.x + a.w) - b.x) <= eps;
    const bool bAtLeft = fabs((b.x + b.w) - a.x) <= eps;
    const bool bAtTop = fabs((a.y + a.h) - b.y) <= eps;
    const bool bAtBottom = fabs((b.y + b.h) - a.y) <= eps;

    if (bAtRight) {
        const double ov = overlapLen(a.y, a.y + a.h, b.y, b.y + b.h);
        if (ov > ovEps) { edgeA = 3; edgeB = 1; return true; }
    }
    if (bAtLeft) {
        const double ov = overlapLen(a.y, a.y + a.h, b.y, b.y + b.h);
        if (ov > ovEps) { edgeA = 1; edgeB = 3; return true; }
    }
    if (bAtTop) {
        const double ov = overlapLen(a.x, a.x + a.w, b.x, b.x + b.w);
        if (ov > ovEps) { edgeA = 2; edgeB = 4; return true; }
    }
    if (bAtBottom) {
        const double ov = overlapLen(a.x, a.x + a.w, b.x, b.x + b.w);
        if (ov > ovEps) { edgeA = 4; edgeB = 2; return true; }
    }
    return false;
}

vector<vector<RouterPhase4::AdjEdge>> RouterPhase4::buildGraph(const Design& design, const vector<Node>& nodes) const {
    vector<vector<AdjEdge>> g(nodes.size());
    for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
        for (int j = i + 1; j < static_cast<int>(nodes.size()); ++j) {
            int ei = 0;
            int ej = 0;
            if (!touchWithEdges(nodes[i].rect, nodes[j].rect, ei, ej)) continue;
            const double w = manhattan(rectCx(nodes[i].rect), rectCy(nodes[i].rect), rectCx(nodes[j].rect), rectCy(nodes[j].rect));
            g[i].push_back({j, ei, ej, w});
            g[j].push_back({i, ej, ei, w});
        }
    }
    (void)design;
    return g;
}

bool RouterPhase4::nodeAllowedAsIntermediate(const Design& design, const Node& node, bool allowSoftFt) const {
    if (!node.isBlock) return true;
    const BlockInst& b = design.blocks[node.index];
    if (!allowSoftFt) return false;
    return b.spec.type == BlockType::SOFT;
}

bool RouterPhase4::edgeAllowedAtEndpoint(const Design& design, int blockIndex, int edge) const {
    if (!validEdge(edge)) return false;
    const BlockInst& b = design.blocks[blockIndex];
    if (b.spec.type != BlockType::EDGE) return true;
    return centerFacingEdge(design, blockIndex) == edge;
}

const Rect* RouterPhase4::findRectByName(const Design& design, const string& name) const {
    for (const auto& b : design.blocks) if (b.spec.name == name) return &b.rect;
    for (const auto& ch : design.channels) if (ch.name == name) return &ch.rect;
    return nullptr;
}

RoutePath RouterPhase4::findCandidatePath(
    const Design& design,
    const Connection& conn,
    const vector<Node>& nodes,
    const vector<vector<AdjEdge>>& graph,
    const RoutingResourceModel& model,
    const Step0ConnectionGuide* guide,
    const SearchPolicy& policy,
    const set<int>& extraAvoidChannels,
    const set<int>& extraAvoidFtBlocks
) const {
    RoutePath out;
    out.netCount = conn.netCount;
    out.srcBlock = design.blocks[conn.src].spec.name;
    out.dstBlock = design.blocks[conn.dst].spec.name;

    const int srcNode = conn.src;
    const int dstNode = conn.dst;
    const int N = static_cast<int>(nodes.size());
    static constexpr int EDGE_STATES = 5;

    auto sid = [](int node, int inEdge) { return node * EDGE_STATES + inEdge; };
    auto sNode = [](int state) { return state / EDGE_STATES; };
    auto sEdge = [](int state) { return state % EDGE_STATES; };

    const int S = N * EDGE_STATES;
    vector<double> dist(S, numeric_limits<double>::infinity());
    vector<int> hops(S, numeric_limits<int>::max());
    vector<int> parent(S, -1);
    vector<int> transOut(S, 0);
    vector<int> transIn(S, 0);

    set<int> preferredChannels;
    set<int> avoidChannels;
    if (guide) {
        preferredChannels.insert(guide->preferredChannelIndices.begin(), guide->preferredChannelIndices.end());
        avoidChannels.insert(guide->avoidChannelIndices.begin(), guide->avoidChannelIndices.end());
    }

    using QN = pair<double, int>;
    priority_queue<QN, vector<QN>, greater<QN>> pq;
    const int st0 = sid(srcNode, 0);
    dist[st0] = 0.0;
    hops[st0] = 0;
    pq.push({0.0, st0});

    int bestDst = -1;
    while (!pq.empty()) {
        const auto [cd, st] = pq.top();
        pq.pop();
        if (cd > dist[st] + 1.0e-12) continue;
        const int u = sNode(st);
        const int uIn = sEdge(st);
        if (u == dstNode) {
            bestDst = st;
            break;
        }
        if (hops[st] >= policy.maxHops) continue;

        for (const auto& e : graph[u]) {
            const int v = e.to;
            const bool vIsEndpoint = (v == srcNode || v == dstNode);
            if (!vIsEndpoint && !nodeAllowedAsIntermediate(design, nodes[v], policy.allowSoftFT)) continue;

            if (u == srcNode && !edgeAllowedAtEndpoint(design, conn.src, e.edgeFrom)) continue;
            if (v == dstNode && !edgeAllowedAtEndpoint(design, conn.dst, e.edgeTo)) continue;

            double extra = 0.0;
            extra += policy.wireWeight * e.baseCost * static_cast<double>(conn.netCount);

            if (u != srcNode && u != dstNode) {
                if (!validEdge(uIn) || uIn == e.edgeFrom) continue;
                if (!nodes[u].isBlock) {
                    const int ci = nodes[u].index;
                    double addLR = 0.0;
                    double addTB = 0.0;
                    if (isOppositeLR(uIn, e.edgeFrom)) addLR = static_cast<double>(conn.netCount);
                    else if (isOppositeTB(uIn, e.edgeFrom)) addTB = static_cast<double>(conn.netCount);
                    else if (isTurn(uIn, e.edgeFrom)) {
                        addLR = static_cast<double>(conn.netCount);
                        addTB = static_cast<double>(conn.netCount);
                    }

                    if (!model.hardFeasibleChannel(ci, addLR, addTB)) continue;

                    double chScore = model.scoreChannelUse(ci, addLR, addTB);
                    if (!isfinite(chScore)) continue;
                    extra += policy.channelCostWeight * chScore;

                    if (preferredChannels.count(ci)) extra -= policy.preferredBonusWeight * static_cast<double>(conn.netCount);
                    if (avoidChannels.count(ci)) extra += policy.avoidPenaltyWeight * static_cast<double>(conn.netCount);
                    if (extraAvoidChannels.count(ci)) extra += 1.5 * policy.avoidPenaltyWeight * static_cast<double>(conn.netCount);
                } else {
                    const int bi = nodes[u].index;
                    if (!policy.allowSoftFT) continue;
                    double ftScore = model.scoreSoftFtUse(bi, static_cast<double>(conn.netCount));
                    if (!isfinite(ftScore)) continue;
                    extra += policy.softFtCostWeight * ftScore;
                    if (extraAvoidFtBlocks.count(bi)) {
                        extra += policy.avoidFtPenaltyWeight * static_cast<double>(conn.netCount);
                    }
                }
            }

            if (u == srcNode) {
                extra += policy.accessCostWeight * model.scoreBlockAccessUse(conn.src, e.edgeFrom, static_cast<double>(conn.netCount));
            }
            if (v == dstNode) {
                extra += policy.accessCostWeight * model.scoreBlockAccessUse(conn.dst, e.edgeTo, static_cast<double>(conn.netCount));
            }

            const int nxt = sid(v, e.edgeTo);
            const int nh = hops[st] + 1;
            const double nd = dist[st] + extra;
            if (nd + 1.0e-12 < dist[nxt] || (fabs(nd - dist[nxt]) <= 1.0e-12 && nh < hops[nxt])) {
                dist[nxt] = nd;
                hops[nxt] = nh;
                parent[nxt] = st;
                transOut[nxt] = e.edgeFrom;
                transIn[nxt] = e.edgeTo;
                pq.push({nd, nxt});
            }
        }
    }

    if (bestDst < 0) {
        out.open = true;
        int srcEdge = 3;
        int dstEdge = 1;
        if (design.blocks[conn.src].spec.type == BlockType::EDGE) {
            srcEdge = centerFacingEdge(design, conn.src);
        } else {
            srcEdge = rectCx(design.blocks[conn.dst].rect) >= rectCx(design.blocks[conn.src].rect) ? 3 : 1;
        }
        if (design.blocks[conn.dst].spec.type == BlockType::EDGE) {
            dstEdge = centerFacingEdge(design, conn.dst);
        } else {
            dstEdge = edgeOpposite(srcEdge);
        }
        out.steps.push_back({out.srcBlock, srcEdge});
        out.steps.push_back({out.dstBlock, dstEdge});
        out.wireLength = calcRouteWireLength(design, out);
        return out;
    }

    vector<int> states;
    for (int cur = bestDst; cur != -1; cur = parent[cur]) states.push_back(cur);
    reverse(states.begin(), states.end());

    for (int i = 1; i < static_cast<int>(states.size()); ++i) {
        const int prev = states[i - 1];
        const int cur = states[i];
        const int u = sNode(prev);
        const int v = sNode(cur);
        out.steps.push_back({nodes[u].name, transOut[cur]});
        out.steps.push_back({nodes[v].name, transIn[cur]});
    }

    out.open = false;
    out.wireLength = calcRouteWireLength(design, out);
    return out;
}

Phase1ResourceDelta RouterPhase4::analyzeDelta(const Design& design, const RoutePath& path) const {
    Phase1ResourceDelta d;
    d.wireLength = path.wireLength;
    if (path.open) return d;

    map<int, double> lr;
    map<int, double> tb;
    map<int, double> ft;
    set<int> ftBlocks;

    const double nets = static_cast<double>(path.netCount);
    for (int i = 0; i + 1 < static_cast<int>(path.steps.size()); ++i) {
        const auto& a = path.steps[i];
        const auto& b = path.steps[i + 1];
        if (a.rectName != b.rectName) continue;

        if (isChannelName(a.rectName)) {
            int ci = -1;
            for (int k = 0; k < static_cast<int>(design.channels.size()); ++k) {
                if (design.channels[k].name == a.rectName) {
                    ci = k;
                    break;
                }
            }
            if (ci < 0) continue;
            if (isOppositeLR(a.edge, b.edge)) lr[ci] += nets;
            else if (isOppositeTB(a.edge, b.edge)) tb[ci] += nets;
            else if (isTurn(a.edge, b.edge)) {
                lr[ci] += nets;
                tb[ci] += nets;
                d.bendCount += 1;
            }
            d.channelTraversalCount += 1;
            continue;
        }

        if (a.rectName == path.srcBlock || a.rectName == path.dstBlock) continue;
        auto it = design.blockNameToIndex.find(a.rectName);
        if (it == design.blockNameToIndex.end()) continue;
        const BlockInst& blk = design.blocks[it->second];
        if (blk.spec.type != BlockType::SOFT) continue;
        ft[it->second] += nets;
        ftBlocks.insert(it->second);
    }

    d.softFtBlockCount = static_cast<int>(ftBlocks.size());

    if (!path.steps.empty()) {
        auto srcIt = design.blockNameToIndex.find(path.srcBlock);
        if (srcIt != design.blockNameToIndex.end()) {
            d.blockEdgeUse.push_back({srcIt->second, path.steps.front().edge, nets});
        }
    }
    if (!path.steps.empty()) {
        auto dstIt = design.blockNameToIndex.find(path.dstBlock);
        if (dstIt != design.blockNameToIndex.end()) {
            d.blockEdgeUse.push_back({dstIt->second, path.steps.back().edge, nets});
        }
    }

    for (const auto& kv : lr) d.channelLR.push_back(kv);
    for (const auto& kv : tb) d.channelTB.push_back(kv);
    for (const auto& kv : ft) d.softFtNets.push_back(kv);
    return d;
}

double RouterPhase4::scoreCandidateBase(const RoutingResourceModel& model, const Phase1ResourceDelta& delta) const {
    double s = 0.0;
    s += 0.20 * delta.wireLength;
    s += 40.0 * static_cast<double>(delta.bendCount);
    map<int, pair<double, double>> channelUse;
    for (const auto& kv : delta.channelLR) channelUse[kv.first].first += kv.second;
    for (const auto& kv : delta.channelTB) channelUse[kv.first].second += kv.second;
    for (const auto& kv : channelUse) s += model.scoreChannelUse(kv.first, kv.second.first, kv.second.second);
    for (const auto& kv : delta.softFtNets) s += model.scoreSoftFtUse(kv.first, kv.second);
    for (const auto& t : delta.blockEdgeUse) s += model.scoreBlockAccessUse(get<0>(t), get<1>(t), get<2>(t));
    return s;
}

bool RouterPhase4::hardFeasibleCandidate(const RoutingResourceModel& model, const Phase1ResourceDelta& delta) const {
    map<int, pair<double, double>> channelUse;
    for (const auto& kv : delta.channelLR) channelUse[kv.first].first += kv.second;
    for (const auto& kv : delta.channelTB) channelUse[kv.first].second += kv.second;
    for (const auto& kv : channelUse) {
        if (!model.hardFeasibleChannel(kv.first, kv.second.first, kv.second.second)) return false;
    }
    return true;
}

bool RouterPhase4::softFeasibleCandidate(const RoutingResourceModel& model, const Phase1ResourceDelta& delta) const {
    map<int, pair<double, double>> channelUse;
    for (const auto& kv : delta.channelLR) channelUse[kv.first].first += kv.second;
    for (const auto& kv : delta.channelTB) channelUse[kv.first].second += kv.second;
    for (const auto& kv : channelUse) {
        if (!model.softFeasibleChannel(kv.first, kv.second.first, kv.second.second)) return false;
    }
    for (const auto& kv : delta.softFtNets) {
        if (!model.softFeasibleSoftFt(kv.first, kv.second)) return false;
    }
    return true;
}

double RouterPhase4::calcRouteWireLength(const Design& design, const RoutePath& path) const {
    if (path.steps.size() < 2) return 0.0;
    vector<pair<double, double>> pts;
    pts.reserve(path.steps.size());
    for (const auto& st : path.steps) {
        const Rect* r = findRectByName(design, st.rectName);
        if (!r) continue;
        pts.push_back(edgeCenterPoint(*r, st.edge));
    }
    double wl = 0.0;
    for (int i = 0; i + 1 < static_cast<int>(pts.size()); ++i) {
        wl += manhattan(pts[i].first, pts[i].second, pts[i + 1].first, pts[i + 1].second);
    }
    return wl * static_cast<double>(path.netCount);
}

string RouterPhase4::routeSignature(const RoutePath& path) const {
    ostringstream oss;
    oss << path.netCount << '|';
    for (const auto& st : path.steps) oss << st.rectName << ':' << st.edge << ';';
    return oss.str();
}

bool RouterPhase4::addCandidateIfUnique(
    const Design& design,
    int ci,
    const RoutePath& path,
    const SearchPolicy& policy,
    const RoutingResourceModel& model,
    set<string>& signatures,
    vector<Phase1RouteCandidate>& pool
) const {
    if (path.open || path.steps.size() < 2) return false;
    const string sig = routeSignature(path);
    if (signatures.count(sig)) return false;
    signatures.insert(sig);

    Phase1ResourceDelta d = analyzeDelta(design, path);
    if (policy.maxBends >= 0 && d.bendCount > policy.maxBends) return false;

    Phase1RouteCandidate c;
    c.connectionIndex = ci;
    c.candidateIndex = static_cast<int>(pool.size());
    c.family = policy.family;
    c.allowSoftFT = policy.allowSoftFT;
    c.path = path;
    c.delta = d;
    c.wireLength = path.wireLength;
    c.bendCount = d.bendCount;
    c.channelTraversalCount = d.channelTraversalCount;
    c.softFtBlockCount = d.softFtBlockCount;
    c.score = scoreCandidateBase(model, d);
    c.hardFeasible = hardFeasibleCandidate(model, d);
    c.softFeasible = softFeasibleCandidate(model, d);
    pool.push_back(move(c));
    return true;
}

int RouterPhase4::augmentCandidatesPreRRR(
    const Design& design,
    const RouterPhase1::CandidateBuildResult& built,
    const RoutingResourceModel& model,
    const vector<Node>& nodes,
    const vector<vector<AdjEdge>>& graph,
    vector<vector<Phase1RouteCandidate>>& candidates
) const {
    int added = 0;
    const vector<SearchPolicy> policies = {
        {"P4_DETOUR_CH", false, 0.22, 1.80, 12.0, 1.20, 1.00, 2.20, 0.0, 110, -1},
        {"P4_BEND3_STRONG", false, 0.20, 1.35, 12.0, 1.00, 1.20, 1.80, 0.0, 90, 3},
        {"P4_BOUNDED_MAZE", true, 0.25, 1.90, 1.30, 1.20, 0.60, 2.80, 3.00, 125, -1},
        {"P4_FT_AVOID_PREF", true, 0.23, 1.70, 2.20, 1.20, 1.00, 2.50, 3.60, 120, -1},
    };

    const int targetMin = max(opt_.preAugmentMinCandidates, 1);
    const int targetMax = max(targetMin, opt_.preAugmentMaxPerConnection);

    for (int ci = 0; ci < static_cast<int>(design.connections.size()); ++ci) {
        auto& pool = candidates[ci];
        if (static_cast<int>(pool.size()) >= targetMin) continue;

        set<string> signatures;
        for (const auto& c : pool) signatures.insert(routeSignature(c.path));

        const Connection& conn = design.connections[ci];
        const Step0ConnectionGuide* guide = ci < static_cast<int>(built.step0.connectionGuide.size()) ? &built.step0.connectionGuide[ci] : nullptr;
        const set<int> noAvoidCh;
        const set<int> noAvoidFt;

        for (const auto& pol : policies) {
            if (static_cast<int>(pool.size()) >= targetMax) break;
            RoutePath p = findCandidatePath(design, conn, nodes, graph, model, guide, pol, noAvoidCh, noAvoidFt);
            if (addCandidateIfUnique(design, ci, p, pol, model, signatures, pool)) added += 1;
        }

        sort(pool.begin(), pool.end(), [](const Phase1RouteCandidate& a, const Phase1RouteCandidate& b) {
            if (fabs(a.score - b.score) > 1.0e-9) return a.score < b.score;
            if (fabs(a.wireLength - b.wireLength) > 1.0e-9) return a.wireLength < b.wireLength;
            return a.family < b.family;
        });
        if (static_cast<int>(pool.size()) > targetMax) pool.resize(targetMax);
        for (int i = 0; i < static_cast<int>(pool.size()); ++i) pool[i].candidateIndex = i;
    }

    return added;
}

int RouterPhase4::generateRepairCandidatesForConnection(
    const Design& design,
    int ci,
    const RouterPhase1::CandidateBuildResult& built,
    const RoutingResourceModel& model,
    const vector<Node>& nodes,
    const vector<vector<AdjEdge>>& graph,
    const set<int>& hotChannels,
    const set<int>& hotFtBlocks,
    vector<Phase1RouteCandidate>& pool
) const {
    set<string> signatures;
    for (const auto& c : pool) signatures.insert(routeSignature(c.path));
    const int oldSize = static_cast<int>(pool.size());

    const vector<SearchPolicy> policies = {
        {"P4_REPAIR_AVOID_HOT_CH", false, 0.24, 2.40, 16.0, 1.30, 1.10, 3.20, 0.0, 140, -1},
        {"P4_REPAIR_AVOID_HOT_FT", true, 0.24, 2.10, 3.20, 1.30, 1.10, 3.20, 4.20, 145, -1},
        {"P4_REPAIR_DETOUR_MAZE", true, 0.27, 2.50, 2.00, 1.30, 0.80, 3.60, 4.00, 160, -1},
    };

    const Connection& conn = design.connections[ci];
    const Step0ConnectionGuide* guide = ci < static_cast<int>(built.step0.connectionGuide.size()) ? &built.step0.connectionGuide[ci] : nullptr;
    for (const auto& pol : policies) {
        if (static_cast<int>(pool.size()) >= opt_.maxRepairCandidatesPerConn) break;
        RoutePath p = findCandidatePath(design, conn, nodes, graph, model, guide, pol, hotChannels, hotFtBlocks);
        addCandidateIfUnique(design, ci, p, pol, model, signatures, pool);
    }

    sort(pool.begin(), pool.end(), [](const Phase1RouteCandidate& a, const Phase1RouteCandidate& b) {
        if (fabs(a.score - b.score) > 1.0e-9) return a.score < b.score;
        if (fabs(a.wireLength - b.wireLength) > 1.0e-9) return a.wireLength < b.wireLength;
        return a.family < b.family;
    });
    if (static_cast<int>(pool.size()) > opt_.maxRepairCandidatesPerConn) pool.resize(opt_.maxRepairCandidatesPerConn);
    for (int i = 0; i < static_cast<int>(pool.size()); ++i) pool[i].candidateIndex = i;

    return static_cast<int>(pool.size()) - oldSize;
}

set<int> RouterPhase4::collectHotChannels(const OverflowSnapshot& ov) const {
    vector<pair<double, int>> ranked;
    ranked.reserve(ov.channelOverflowLR.size());
    for (int ci = 0; ci < static_cast<int>(ov.channelOverflowLR.size()); ++ci) {
        const double v = ov.channelOverflowLR[ci] + ov.channelOverflowTB[ci];
        if (v > 1.0e-9) ranked.push_back({v, ci});
    }
    sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) {
        if (fabs(a.first - b.first) > 1.0e-12) return a.first > b.first;
        return a.second < b.second;
    });
    set<int> out;
    for (int i = 0; i < static_cast<int>(ranked.size()) && i < opt_.hotChannelTopK; ++i) {
        out.insert(ranked[i].second);
    }
    return out;
}

set<int> RouterPhase4::collectHotFtBlocks(const OverflowSnapshot& ov) const {
    vector<pair<double, int>> ranked;
    ranked.reserve(ov.softFtOverflow.size());
    for (int bi = 0; bi < static_cast<int>(ov.softFtOverflow.size()); ++bi) {
        const double v = ov.softFtOverflow[bi];
        if (v > 1.0e-9) ranked.push_back({v, bi});
    }
    sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) {
        if (fabs(a.first - b.first) > 1.0e-12) return a.first > b.first;
        return a.second < b.second;
    });
    set<int> out;
    for (int i = 0; i < static_cast<int>(ranked.size()) && i < opt_.hotFtBlockTopK; ++i) {
        out.insert(ranked[i].second);
    }
    return out;
}

double RouterPhase4::localRepairPenalty(
    const Phase1ResourceDelta& d,
    const set<int>& hotChannels,
    const set<int>& hotFtBlocks
) const {
    double p = 0.0;
    for (const auto& kv : d.channelLR) {
        if (hotChannels.count(kv.first)) p += kv.second * opt_.hotChannelPenaltyWeight;
    }
    for (const auto& kv : d.channelTB) {
        if (hotChannels.count(kv.first)) p += kv.second * opt_.hotChannelPenaltyWeight;
    }
    for (const auto& kv : d.softFtNets) {
        if (hotFtBlocks.count(kv.first)) p += kv.second * opt_.hotFtPenaltyWeight;
    }
    return p;
}

bool RouterPhase4::betterSnapshot(
    const EvalReport& candEval,
    const OverflowSnapshot& candOv,
    const EvalReport& bestEval,
    const OverflowSnapshot& bestOv,
    double ftAnchor
) const {
    if (candEval.hasFail() != bestEval.hasFail()) return !candEval.hasFail();
    if (candEval.openPathCount != bestEval.openPathCount) return candEval.openPathCount < bestEval.openPathCount;
    if (candEval.invalidPathCount != bestEval.invalidPathCount) return candEval.invalidPathCount < bestEval.invalidPathCount;

    const double candHard = canonicalChannelOverflow(candEval, candOv.totalHardOverflow);
    const double bestHard = canonicalChannelOverflow(bestEval, bestOv.totalHardOverflow);
    const double candFt = candEval.totalFeedthroughOverflow;
    const double bestFt = bestEval.totalFeedthroughOverflow;

    const double hardImprove = bestHard - candHard;
    const double ftRegress = candFt - bestFt;
    const double ftImprove = bestFt - candFt;

    const double ftTotalGuard = max(
        opt_.ftTotalRegressionGuardAbs,
        opt_.ftTotalRegressionGuardRel * max(1.0, ftAnchor)
    );
    if (candFt > ftAnchor + ftTotalGuard + 1.0e-6) return false;

    const double ftGuard = max(
        opt_.ftRegressionGuardAbs,
        opt_.ftRegressionGuardRel * max(1.0, bestFt)
    );

    // Strict FT guard: hard-overflow improvements are accepted only if FT regression
    // stays within configured tolerance.
    if (hardImprove > 1.0e-6) return ftRegress <= ftGuard;

    // If hard overflow gets worse, allow it only for meaningful FT recovery.
    if (candHard > bestHard + 1.0e-6) {
        if (ftImprove > 1.0e-6) {
            return candHard <= bestHard + opt_.hardSlackWhenFtImproves;
        }
        return false;
    }

    if (ftImprove > 1.0e-6) return true;
    if (ftRegress > 1.0e-6) return false;
    if (fabs(candOv.totalSoftOverflow - bestOv.totalSoftOverflow) > 1.0e-6) return candOv.totalSoftOverflow < bestOv.totalSoftOverflow;
    if (fabs(candEval.totalWireLength - bestEval.totalWireLength) > 1.0e-6) return candEval.totalWireLength < bestEval.totalWireLength;
    return false;
}

bool RouterPhase4::allocateConnection(
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
    const vector<double>& histFt,
    double presentFactor,
    const set<int>* hotChannels,
    const set<int>* hotFtBlocks,
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
            sc.score = marginalScore(model, d, diversityPenalty, histLR, histTB, histFt, presentFactor)
                + guideAdjustment(guide, d, chunk);
            sc.score += opt_.candidateSoftOverflowWeight * softOv;
            if (hotChannels && hotFtBlocks) {
                sc.score += localRepairPenalty(d, *hotChannels, *hotFtBlocks);
            }
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
            if (fabs(a.softOverflowAmount - b.softOverflowAmount) > 1.0e-9) return a.softOverflowAmount < b.softOverflowAmount;
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

        Phase4AllocationPiece piece;
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

void RouterPhase4::ripupConnection(RoutingResourceModel& model, MutableConnectionState& state) const {
    for (const auto& p : state.pieces) model.ripup(p.delta);
    state.pieces.clear();
}

RouterPhase4::OverflowSnapshot RouterPhase4::computeOverflowSnapshot(const Design& design, const RoutingResourceModel& model) const {
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

void RouterPhase4::updateHistoryFromOverflow(
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

void RouterPhase4::computeContributorScores(
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
                if (bi >= 0 && bi < static_cast<int>(ov.softFtOverflow.size())) {
                    score += kv.second * ov.softFtOverflow[bi] * opt_.ftContributorWeight;
                }
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

vector<int> RouterPhase4::selectRipupSet(const OverflowSnapshot& ov, const vector<MutableConnectionState>& states) const {
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

vector<Phase4ConnectionStatus> RouterPhase4::buildConnectionStatus(
    const Design& design,
    const vector<MutableConnectionState>& states
) const {
    vector<Phase4ConnectionStatus> out(states.size());
    for (int ci = 0; ci < static_cast<int>(states.size()); ++ci) {
        const auto& s = states[ci];
        const Connection& conn = design.connections[ci];
        Phase4ConnectionStatus st;
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

Design RouterPhase4::buildPhase4RoutedDesign(const Design& design, const vector<MutableConnectionState>& states) const {
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

RouterPhase4::RunResult RouterPhase4::run(const Design& design, const string& inputPath, const string& outputCfgPath, double alpha) const {
    RunResult rr;
    rr.ok = true;
    rr.totalConnections = static_cast<int>(design.connections.size());

    RouterPhase1::Options p1opt;
    p1opt.maxCandidatesPerConnection = opt_.maxCandidatesPerConnection;
    p1opt.exportFiles = false;
    RouterPhase1 phase1(p1opt);
    RouterPhase1::CandidateBuildResult built = phase1.buildCandidates(design);

    RoutingResourceModel model;
    model.initialize(design, built.step0);
    vector<vector<Phase1RouteCandidate>> candidates = built.candidates;
    const vector<Node> nodes = buildNodes(design);
    const vector<vector<AdjEdge>> graph = buildGraph(design, nodes);
    rr.preAugmentedCandidates = augmentCandidatesPreRRR(design, built, model, nodes, graph, candidates);

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
        const auto& candVec = candidates[ci];
        const Step0ConnectionGuide* guide = ci < static_cast<int>(built.step0.connectionGuide.size()) ? &built.step0.connectionGuide[ci] : nullptr;
        allocateConnection(
            design, ci, candVec, guide, model, states[ci], 0,
            opt_.allowInitialHardFallback, histLR, histTB, histFt, presentFactor, nullptr, nullptr, hardFallbackPieces
        );
    }

    Evaluator evaluator;
    vector<Phase4IterationRecord> iterRecs;
    Design curDesign = buildPhase4RoutedDesign(design, states);
    EvalReport curEval = evaluator.evaluate(curDesign, alpha);
    OverflowSnapshot curOv = computeOverflowSnapshot(design, model);

    Phase4IterationRecord iter0;
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
    const double ftAnchor = curEval.totalFeedthroughOverflow;

    vector<MutableConnectionState> bestStates = states;
    RoutingResourceModel bestModel = model;
    EvalReport bestEval = curEval;
    OverflowSnapshot bestOv = curOv;
    Design bestDesign = curDesign;

    int stagnantRounds = 0;
    int totalRipped = 0;
    int totalRerouted = 0;

    for (int iter = 1; iter <= opt_.maxIterations; ++iter) {
        if (phase4StrictConverged(curEval, curOv.totalHardOverflow)) {
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
            auto& candVec = candidates[ci];
            const Step0ConnectionGuide* guide = ci < static_cast<int>(built.step0.connectionGuide.size()) ? &built.step0.connectionGuide[ci] : nullptr;
            if (allocateConnection(
                design, ci, candVec, guide, model, states[ci], iter,
                allowFallback, histLR, histTB, histFt, presentFactor, nullptr, nullptr, hardFallbackPieces
            )) {
                rerouted += 1;
            }
        }
        totalRerouted += rerouted;

        curDesign = buildPhase4RoutedDesign(design, states);
        curEval = evaluator.evaluate(curDesign, alpha);
        curOv = computeOverflowSnapshot(design, model);

        Phase4IterationRecord rec;
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

        if (betterSnapshot(curEval, curOv, bestEval, bestOv, ftAnchor)) {
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

    int localRipped = 0;
    int localRerouted = 0;
    int localAddedCandidates = 0;
    int localStagnant = 0;
    for (int round = 1; round <= opt_.localRepairRounds; ++round) {
        const bool onlyFtMode = (curEval.openPathCount == 0
            && !curEval.pathInvalid
            && curEval.invalidPathCount == 0
            && curOv.totalHardOverflow <= 1.0e-9
            && curEval.totalChannelOverflow <= 1.0e-9);
        if (onlyFtMode && round > 1) break;

        const set<int> hotChannels = collectHotChannels(curOv);
        const set<int> hotFtBlocks = collectHotFtBlocks(curOv);
        if (hotChannels.empty() && hotFtBlocks.empty()) break;

        computeContributorScores(curOv, states);
        vector<int> repairSet;
        for (const auto& st : states) {
            if (st.contributorScore > 1.0e-9) repairSet.push_back(st.connectionIndex);
        }
        if (repairSet.empty()) break;

        sort(repairSet.begin(), repairSet.end(), [&](int a, int b) {
            if (fabs(states[a].contributorScore - states[b].contributorScore) > 1.0e-9) {
                return states[a].contributorScore > states[b].contributorScore;
            }
            return a < b;
        });

        int target = static_cast<int>(ceil(opt_.localRepairRipupRatio * static_cast<double>(repairSet.size())));
        target = max(target, opt_.localRepairMinRipupConnections);
        target = min(target, opt_.localRepairMaxRipupConnections);
        target = min(target, static_cast<int>(repairSet.size()));
        repairSet.resize(target);

        for (int ci : repairSet) ripupConnection(model, states[ci]);
        localRipped += static_cast<int>(repairSet.size());

        int rerouted = 0;
        const double localPresent = presentFactor * opt_.localRepairPresentFactorBoost;
        for (int ci : repairSet) {
            states[ci].rerouteCount += 1;
            localAddedCandidates += generateRepairCandidatesForConnection(
                design, ci, built, model, nodes, graph, hotChannels, hotFtBlocks, candidates[ci]
            );
            const Step0ConnectionGuide* guide = ci < static_cast<int>(built.step0.connectionGuide.size()) ? &built.step0.connectionGuide[ci] : nullptr;
            const int assignedIter = iterRecs.empty() ? 1 : (iterRecs.back().iteration + 1);
            if (allocateConnection(
                design, ci, candidates[ci], guide, model, states[ci], assignedIter,
                opt_.allowLocalRepairHardFallback, histLR, histTB, histFt, localPresent,
                &hotChannels, &hotFtBlocks, hardFallbackPieces
            )) {
                rerouted += 1;
            }
        }
        localRerouted += rerouted;

        curDesign = buildPhase4RoutedDesign(design, states);
        curEval = evaluator.evaluate(curDesign, alpha);
        curOv = computeOverflowSnapshot(design, model);
        const int iterId = iterRecs.empty() ? 1 : (iterRecs.back().iteration + 1);

        Phase4IterationRecord rec;
        rec.iteration = iterId;
        rec.rippedConnections = static_cast<int>(repairSet.size());
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

        if (betterSnapshot(curEval, curOv, bestEval, bestOv, ftAnchor)) {
            bestStates = states;
            bestModel = model;
            bestEval = curEval;
            bestOv = curOv;
            bestDesign = curDesign;
            localStagnant = 0;
        } else {
            localStagnant += 1;
        }

        rr.localRepairRoundsRun = round;
        rr.iterationsRun = rec.iteration;
        if (localStagnant >= 2) break;
    }

    // Use best snapshot in case the last round regresses.
    states = move(bestStates);
    model = move(bestModel);
    curEval = bestEval;
    curOv = bestOv;
    curDesign = move(bestDesign);

    rr.totalPieces = 0;
    for (const auto& st : states) rr.totalPieces += static_cast<int>(st.pieces.size());
    rr.hardFallbackPieces = hardFallbackPieces;
    rr.totalRippedConnections = totalRipped;
    rr.totalReroutedConnections = totalRerouted;
    rr.localRepairRippedConnections = localRipped;
    rr.localRepairReroutedConnections = localRerouted;
    rr.localRepairAddedCandidates = localAddedCandidates;
    rr.finalOpenConnections = curEval.openPathCount;
    rr.finalTotalHardOverflow = curOv.totalHardOverflow;
    rr.finalTotalChannelOverflowEval = curEval.totalChannelOverflow;
    rr.finalTotalFeedthroughOverflowEval = curEval.totalFeedthroughOverflow;
    rr.finalTotalSoftOverflowModel = curOv.totalSoftOverflow;
    rr.ftHandoffRequired = curEval.totalFeedthroughOverflow > 1.0e-9;
    rr.ftHandoffBlockCount = 0;
    for (const auto& ft : curEval.feedthroughTruth) {
        if (ft.overflowArea > 1.0e-9) rr.ftHandoffBlockCount += 1;
    }
    rr.ftHandoffTotalOverflow = curEval.totalFeedthroughOverflow;
    rr.finalEvalHasFail = curEval.hasFail();
    if (phase4StrictConverged(curEval, curOv.totalHardOverflow)) rr.converged = true;

    vector<Phase4ConnectionStatus> connStatus = buildConnectionStatus(design, states);

    if (opt_.exportFiles) {
        fs::path outPath(outputCfgPath);
        const fs::path stem = outPath.parent_path() / outPath.stem();
        fs::path reportDir("Router_Statistics");
        fs::create_directories(reportDir);
        const fs::path reportStem = reportDir / outPath.stem();
        const string Phase4CfgPath = stem.string() + "_phase4.cfg";
        rr.ftHandoffCsvPath = reportStem.string() + "_phase4_ft_handoff.csv";
        OutputWriter writer;
        if (!writer.write(Phase4CfgPath, curDesign)) {
            rr.ok = false;
        } else {
            rr.ok = writeReports(
                design, built, model, states, connStatus, iterRecs, curEval,
                Phase4CfgPath, rr, inputPath, outputCfgPath, alpha
            );
        }
    }

    return rr;
}

bool RouterPhase4::writeReports(
    const Design& design,
    const RouterPhase1::CandidateBuildResult& phase1,
    const RoutingResourceModel& modelAfter,
    const vector<MutableConnectionState>& states,
    const vector<Phase4ConnectionStatus>& connStatus,
    const vector<Phase4IterationRecord>& iters,
    const EvalReport& finalEval,
    const string& Phase4CfgPath,
    const RunResult& rr,
    const string& inputPath,
    const string& outputCfgPath,
    double alpha
) const {
    fs::path outPath(outputCfgPath);
    fs::path reportDir("Router_Statistics");
    fs::create_directories(reportDir);
    const fs::path stem = reportDir / outPath.stem();

    const string summaryPath = (stem.string() + "_phase4_summary.txt");
    const string iterPath = (stem.string() + "_phase4_iterations.csv");
    const string connPath = (stem.string() + "_phase4_connection_status.csv");
    const string resPath = (stem.string() + "_phase4_resource_after.csv");
    const string routePath = (stem.string() + "_phase4_selected_routes.csv");
    const string ftHandoffPath = rr.ftHandoffCsvPath.empty()
        ? (stem.string() + "_phase4_ft_handoff.csv")
        : rr.ftHandoffCsvPath;

    ofstream fsum(summaryPath);
    if (!fsum) return false;
    fsum << fixed << setprecision(6);
    fsum << "Phase 4 FT-Aware Local Repair Summary\n";
    fsum << "input=" << inputPath << "\n";
    fsum << "requested_output=" << outputCfgPath << "\n";
    fsum << "phase4_output_cfg=" << Phase4CfgPath << "\n";
    fsum << "alpha=" << alpha << "\n";
    fsum << "connections=" << rr.totalConnections << "\n";
    fsum << "iterations_run=" << rr.iterationsRun << "\n";
    fsum << "converged=" << yesNo(rr.converged) << "\n";
    fsum << "pre_augmented_candidates=" << rr.preAugmentedCandidates << "\n";
    fsum << "total_pieces=" << rr.totalPieces << "\n";
    fsum << "hard_fallback_pieces=" << rr.hardFallbackPieces << "\n";
    fsum << "total_ripped_connections=" << rr.totalRippedConnections << "\n";
    fsum << "total_rerouted_connections=" << rr.totalReroutedConnections << "\n";
    fsum << "local_repair_rounds_run=" << rr.localRepairRoundsRun << "\n";
    fsum << "local_repair_ripped_connections=" << rr.localRepairRippedConnections << "\n";
    fsum << "local_repair_rerouted_connections=" << rr.localRepairReroutedConnections << "\n";
    fsum << "local_repair_added_candidates=" << rr.localRepairAddedCandidates << "\n";
    fsum << "initial_open_connections=" << rr.initialOpenConnections << "\n";
    fsum << "final_open_connections=" << rr.finalOpenConnections << "\n";
    fsum << "initial_total_hard_overflow=" << rr.initialTotalHardOverflow << "\n";
    fsum << "final_total_hard_overflow=" << rr.finalTotalHardOverflow << "\n";
    fsum << "initial_eval_channel_overflow=" << rr.initialTotalChannelOverflowEval << "\n";
    fsum << "final_eval_channel_overflow=" << rr.finalTotalChannelOverflowEval << "\n";
    fsum << "initial_eval_feedthrough_overflow=" << rr.initialTotalFeedthroughOverflowEval << "\n";
    fsum << "final_eval_feedthrough_overflow=" << rr.finalTotalFeedthroughOverflowEval << "\n";
    fsum << "final_model_total_soft_overflow=" << rr.finalTotalSoftOverflowModel << "\n";
    fsum << "ft_handoff_required=" << yesNo(rr.ftHandoffRequired) << "\n";
    fsum << "ft_handoff_block_count=" << rr.ftHandoffBlockCount << "\n";
    fsum << "ft_handoff_total_overflow=" << rr.ftHandoffTotalOverflow << "\n";
    fsum << "ft_handoff_csv=" << ftHandoffPath << "\n";
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
    froute << "connection_index,candidate_index,family,allocated_nets,assigned_iteration,score,path_steps\n";
    for (const auto& st : states) {
        for (const auto& p : st.pieces) {
            froute << p.connectionIndex << ','
                   << p.candidateIndex << ','
                   << p.family << ','
                   << p.allocatedNets << ','
                   << p.assignedIteration << ','
                   << p.score << ','
                   << '"' << joinPathSteps(p.path.steps) << '"' << "\n";
        }
    }
    froute.close();

    ofstream fft(ftHandoffPath);
    if (!fft) return false;
    fft << fixed << setprecision(6);
    fft << "schema,input,phase4_output_cfg,total_feedthrough_overflow,block_index,block_name,used_ft_nets,conversion_rate,side_delta,current_area,required_area,overflow_area,handoff_required\n";
    for (const auto& ft : finalEval.feedthroughTruth) {
        int blockIndex = -1;
        auto it = design.blockNameToIndex.find(ft.blockName);
        if (it != design.blockNameToIndex.end()) blockIndex = it->second;
        fft << "phase4_ft_handoff_v1,"
            << inputPath << ','
            << Phase4CfgPath << ','
            << finalEval.totalFeedthroughOverflow << ','
            << blockIndex << ','
            << ft.blockName << ','
            << ft.usedNets << ','
            << ft.conversionRate << ','
            << ft.sideDelta << ','
            << ft.currentArea << ','
            << ft.requiredArea << ','
            << ft.overflowArea << ','
            << yesNo(ft.overflowArea > 1.0e-9) << "\n";
    }
    fft.close();

    return true;
}

