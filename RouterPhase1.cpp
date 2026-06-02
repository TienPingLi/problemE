#include "RouterPhase1.hpp"
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

pair<double, double> pointOf(const Rect& r, int edge) {
    return edgeCenterPoint(r, edge);
}

double capLR(const Rect& r) { return max(0.0, r.h) * CHANNEL_DENSITY; }
double capTB(const Rect& r) { return max(0.0, r.w) * CHANNEL_DENSITY; }

double ftRateForNets(const BlockSpec& spec, double nets) {
    if (nets <= 3000.0) return spec.ftRate[0];
    if (nets <= 6000.0) return spec.ftRate[1];
    if (nets <= 9000.0) return spec.ftRate[2];
    return spec.ftRate[3];
}

double requiredAreaFromBase(const BlockInst& b, double ftNets) {
    const double baseArea = max(EPS, b.spec.area);
    if (b.spec.type != BlockType::SOFT || ftNets <= EPS) return baseArea;
    const double rate = ftRateForNets(b.spec, ftNets);
    const double delta = (ftNets / CHANNEL_DENSITY) * rate / 2.0;
    const double side = sqrt(baseArea) + delta;
    return side * side;
}

double softFtOverflowArea(const BlockInst& b, double ftNets) {
    const double currentArea = max(EPS, b.rect.w * b.rect.h);
    const double reqArea = requiredAreaFromBase(b, ftNets);
    return max(0.0, reqArea - currentArea);
}

string csvJoinInts(const vector<int>& v) {
    ostringstream oss;
    for (int i = 0; i < static_cast<int>(v.size()); ++i) {
        if (i) oss << '/';
        oss << v[i];
    }
    return oss.str();
}

string csvJoinSteps(const vector<RouteStep>& steps) {
    ostringstream oss;
    for (int i = 0; i < static_cast<int>(steps.size()); ++i) {
        if (i) oss << '|';
        oss << steps[i].rectName << ':' << steps[i].edge;
    }
    return oss.str();
}

set<int> phase1ChannelSet(const Phase1ResourceDelta& d) {
    set<int> out;
    for (const auto& kv : d.channelLR) out.insert(kv.first);
    for (const auto& kv : d.channelTB) out.insert(kv.first);
    return out;
}

string phase1DiversitySignature(const Phase1RouteCandidate& c) {
    ostringstream oss;
    if (!c.path.steps.empty()) {
        oss << "S" << c.path.steps.front().edge << "|D" << c.path.steps.back().edge << '|';
    }
    oss << "CH";
    for (int ci : phase1ChannelSet(c.delta)) oss << ci << '/';
    oss << "|FT";
    for (const auto& kv : c.delta.softFtNets) oss << kv.first << '/';
    oss << "|B" << c.bendCount;
    return oss.str();
}

template <typename T>
T clampLocal(T v, T lo, T hi) {
    return min(hi, max(lo, v));
}

} // namespace

void RoutingResourceModel::initialize(const Design& design, const Step0CongestionMapResult& step0) {
    design_ = &design;
    channelRes_.clear();
    softFtRes_.clear();
    blockAccessRes_.clear();
    accessLookup_.clear();

    channelRes_.resize(design.channels.size());
    for (int i = 0; i < static_cast<int>(design.channels.size()); ++i) {
        Phase1ChannelResource r;
        r.channelIndex = i;
        r.channelName = design.channels[i].name;
        r.hardCapLR = capLR(design.channels[i].rect);
        r.hardCapTB = capTB(design.channels[i].rect);
        r.softCapLR = r.hardCapLR;
        r.softCapTB = r.hardCapTB;
        r.ambientLR = 0.0;
        r.ambientTB = 0.0;
        r.criticalityLR = 0.0;
        r.criticalityTB = 0.0;
        r.historyLR = 1.0;
        r.historyTB = 1.0;

        if (i < static_cast<int>(step0.channelPressure.size())) {
            const auto& p = step0.channelPressure[i];
            if (p.effectiveCapLR > EPS) r.softCapLR = min(r.hardCapLR, p.effectiveCapLR);
            if (p.effectiveCapTB > EPS) r.softCapTB = min(r.hardCapTB, p.effectiveCapTB);
            r.softCapLR = max(r.softCapLR, 0.20 * r.hardCapLR);
            r.softCapTB = max(r.softCapTB, 0.20 * r.hardCapTB);

            r.ambientLR = max(0.0, p.ambientDemandLR);
            r.ambientTB = max(0.0, p.ambientDemandTB);
            r.criticalityLR = max(0.0, p.routerCriticalityLR);
            r.criticalityTB = max(0.0, p.routerCriticalityTB);
            r.historyLR = max(1.0, p.histLR);
            r.historyTB = max(1.0, p.histTB);
        }

        channelRes_[i] = r;
    }

    for (int bi = 0; bi < static_cast<int>(design.blocks.size()); ++bi) {
        const BlockInst& b = design.blocks[bi];
        if (b.spec.type != BlockType::SOFT) continue;
        Phase1SoftFtResource r;
        r.blockIndex = bi;
        r.blockName = b.spec.name;
        r.baseArea = max(EPS, b.spec.area);
        r.currentArea = max(EPS, b.rect.w * b.rect.h);
        softFtRes_.push_back(r);
    }

    for (const auto& ep : step0.blockEdgePressure) {
        auto it = design.blockNameToIndex.find(ep.blockName);
        if (it == design.blockNameToIndex.end()) continue;
        Phase1BlockAccessResource r;
        r.blockIndex = it->second;
        r.blockName = ep.blockName;
        r.edge = ep.edge;
        r.edgePressure = max(0.0, ep.edgePressure);
        r.severity = clampLocal(ep.severity, 0.0, 1.0);
        r.healthyAccessChannelCount = max(0, ep.healthyAccessChannelCount);
        accessLookup_[{r.blockIndex, r.edge}] = static_cast<int>(blockAccessRes_.size());
        blockAccessRes_.push_back(r);
    }
}

double RoutingResourceModel::scoreChannelUse(int channelIndex, double addLR, double addTB) const {
    if (channelIndex < 0 || channelIndex >= static_cast<int>(channelRes_.size())) return numeric_limits<double>::infinity();
    const auto& r = channelRes_[channelIndex];

    auto scoreComp = [](double used, double ambient, double add, double softCap, double hardCap, double crit, double hist) {
        if (add <= EPS) return 0.0;
        const double safeSoft = max(EPS, softCap);
        const double safeHard = max(EPS, hardCap);

        const double util0 = (used + ambient) / safeSoft;
        const double util1 = (used + ambient + add) / safeSoft;
        const double hardUtil1 = (used + add) / safeHard;

        const double base = add;
        const double cong = 12.0 * max(0.0, util1 - 0.70) * max(0.0, util1 - 0.70);
        const double nearFull = 45.0 * max(0.0, util1 - 1.00) * max(0.0, util1 - 1.00);
        const double hardStress = 80.0 * max(0.0, hardUtil1 - 1.00) * max(0.0, hardUtil1 - 1.00);
        const double critScale = 1.0 + 4.0 * crit;
        const double histScale = 1.0 + 0.40 * (hist - 1.0) + 0.25 * max(0.0, util0 - 0.85);
        return base * (1.0 + cong + nearFull + hardStress) * critScale * histScale;
    };

    double s = 0.0;
    s += scoreComp(r.usedLR, r.ambientLR, addLR, r.softCapLR, r.hardCapLR, r.criticalityLR, r.historyLR);
    s += scoreComp(r.usedTB, r.ambientTB, addTB, r.softCapTB, r.hardCapTB, r.criticalityTB, r.historyTB);
    return s;
}

bool RoutingResourceModel::hardFeasibleChannel(int channelIndex, double addLR, double addTB) const {
    if (channelIndex < 0 || channelIndex >= static_cast<int>(channelRes_.size())) return false;
    const auto& r = channelRes_[channelIndex];
    if (addLR > EPS && (r.usedLR + addLR > r.hardCapLR + 1.0e-9)) return false;
    if (addTB > EPS && (r.usedTB + addTB > r.hardCapTB + 1.0e-9)) return false;
    return true;
}

bool RoutingResourceModel::softFeasibleChannel(int channelIndex, double addLR, double addTB) const {
    if (channelIndex < 0 || channelIndex >= static_cast<int>(channelRes_.size())) return false;
    const auto& r = channelRes_[channelIndex];
    if (addLR > EPS && (r.usedLR + r.ambientLR + addLR > r.softCapLR + 1.0e-9)) return false;
    if (addTB > EPS && (r.usedTB + r.ambientTB + addTB > r.softCapTB + 1.0e-9)) return false;
    return true;
}

double RoutingResourceModel::scoreSoftFtUse(int blockIndex, double addNets) const {
    if (!design_ || addNets <= EPS) return 0.0;
    if (blockIndex < 0 || blockIndex >= static_cast<int>(design_->blocks.size())) return numeric_limits<double>::infinity();
    const BlockInst& b = design_->blocks[blockIndex];
    if (b.spec.type != BlockType::SOFT) return numeric_limits<double>::infinity();

    double usedNow = 0.0;
    for (const auto& r : softFtRes_) {
        if (r.blockIndex == blockIndex) {
            usedNow = r.usedNets;
            break;
        }
    }

    const double oldReq = requiredAreaFromBase(b, usedNow);
    const double newReq = requiredAreaFromBase(b, usedNow + addNets);
    const double incArea = max(0.0, newReq - oldReq);
    const double overflow = softFtOverflowArea(b, usedNow + addNets);

    return 1500.0 + 8.0 * addNets + 12.0 * incArea + 0.02 * overflow;
}

bool RoutingResourceModel::softFeasibleSoftFt(int blockIndex, double addNets) const {
    if (!design_ || addNets <= EPS) return true;
    if (blockIndex < 0 || blockIndex >= static_cast<int>(design_->blocks.size())) return false;
    const BlockInst& b = design_->blocks[blockIndex];
    if (b.spec.type != BlockType::SOFT) return false;

    double usedNow = 0.0;
    for (const auto& r : softFtRes_) {
        if (r.blockIndex == blockIndex) {
            usedNow = r.usedNets;
            break;
        }
    }

    const double currentArea = max(EPS, b.rect.w * b.rect.h);
    const double reqArea = requiredAreaFromBase(b, usedNow + addNets);
    return reqArea <= currentArea * 1.30 + 1.0e-9;
}

double RoutingResourceModel::scoreBlockAccessUse(int blockIndex, int edge, double addNets) const {
    if (addNets <= EPS) return 0.0;
    auto it = accessLookup_.find({blockIndex, edge});
    if (it == accessLookup_.end()) return 0.0;
    const auto& r = blockAccessRes_[it->second];
    const double scarcity = r.healthyAccessChannelCount <= 0 ? 1.0 : 1.0 / static_cast<double>(r.healthyAccessChannelCount);
    return addNets * (20.0 * r.severity + 8.0 * clampLocal(r.edgePressure, 0.0, 10.0) + 15.0 * scarcity);
}

void RoutingResourceModel::commit(const Phase1ResourceDelta& delta) {
    for (const auto& kv : delta.channelLR) {
        if (kv.first < 0 || kv.first >= static_cast<int>(channelRes_.size())) continue;
        channelRes_[kv.first].usedLR += kv.second;
    }
    for (const auto& kv : delta.channelTB) {
        if (kv.first < 0 || kv.first >= static_cast<int>(channelRes_.size())) continue;
        channelRes_[kv.first].usedTB += kv.second;
    }
    for (const auto& kv : delta.softFtNets) {
        for (auto& r : softFtRes_) {
            if (r.blockIndex == kv.first) r.usedNets += kv.second;
        }
    }
}

void RoutingResourceModel::ripup(const Phase1ResourceDelta& delta) {
    for (const auto& kv : delta.channelLR) {
        if (kv.first < 0 || kv.first >= static_cast<int>(channelRes_.size())) continue;
        channelRes_[kv.first].usedLR = max(0.0, channelRes_[kv.first].usedLR - kv.second);
    }
    for (const auto& kv : delta.channelTB) {
        if (kv.first < 0 || kv.first >= static_cast<int>(channelRes_.size())) continue;
        channelRes_[kv.first].usedTB = max(0.0, channelRes_[kv.first].usedTB - kv.second);
    }
    for (const auto& kv : delta.softFtNets) {
        for (auto& r : softFtRes_) {
            if (r.blockIndex == kv.first) r.usedNets = max(0.0, r.usedNets - kv.second);
        }
    }
}

vector<RouterPhase1::Node> RouterPhase1::buildNodes(const Design& design) const {
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

bool RouterPhase1::touchWithEdges(const Rect& a, const Rect& b, int& edgeA, int& edgeB) const {
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

vector<vector<RouterPhase1::AdjEdge>> RouterPhase1::buildGraph(const Design& design, const vector<Node>& nodes) const {
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

bool RouterPhase1::nodeAllowedAsIntermediate(const Design& design, const Node& node, bool allowSoftFt) const {
    if (!node.isBlock) return true;
    const BlockInst& b = design.blocks[node.index];
    if (!allowSoftFt) return false;
    return b.spec.type == BlockType::SOFT;
}

int RouterPhase1::centerFacingEdge(const Design& design, int blockIndex) const {
    const Rect& r = design.blocks[blockIndex].rect;
    const double ocx = design.outlineW * 0.5;
    const double ocy = design.outlineH * 0.5;
    const double dx = ocx - rectCx(r);
    const double dy = ocy - rectCy(r);
    if (fabs(dx) >= fabs(dy)) return dx >= 0.0 ? 3 : 1;
    return dy >= 0.0 ? 2 : 4;
}

bool RouterPhase1::edgeAllowedAtEndpoint(const Design& design, int blockIndex, int edge) const {
    if (!validEdge(edge)) return false;
    const BlockInst& b = design.blocks[blockIndex];
    if (b.spec.type != BlockType::EDGE) return true;
    return centerFacingEdge(design, blockIndex) == edge;
}

const Rect* RouterPhase1::findRectByName(const Design& design, const string& name) const {
    for (const auto& b : design.blocks) if (b.spec.name == name) return &b.rect;
    for (const auto& ch : design.channels) if (ch.name == name) return &ch.rect;
    return nullptr;
}

RoutePath RouterPhase1::findCandidatePath(
    const Design& design,
    const Connection& conn,
    int connectionIndex,
    const vector<Node>& nodes,
    const vector<vector<AdjEdge>>& graph,
    const RoutingResourceModel& model,
    const Step0ConnectionGuide* guide,
    const SearchPolicy& policy
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
    preferredChannels.insert(policy.extraPreferredChannelIndices.begin(), policy.extraPreferredChannelIndices.end());
    avoidChannels.insert(policy.extraAvoidChannelIndices.begin(), policy.extraAvoidChannelIndices.end());

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
            if (!nodes[u].isBlock && policy.bannedChannelIndices.count(nodes[u].index)) continue;
            if (!nodes[v].isBlock && policy.bannedChannelIndices.count(nodes[v].index)) continue;

            if (u == srcNode && !edgeAllowedAtEndpoint(design, conn.src, e.edgeFrom)) continue;
            if (v == dstNode && !edgeAllowedAtEndpoint(design, conn.dst, e.edgeTo)) continue;
            if (u == srcNode && validEdge(policy.forcedSrcEdge) && e.edgeFrom != policy.forcedSrcEdge) continue;
            if (v == dstNode && validEdge(policy.forcedDstEdge) && e.edgeTo != policy.forcedDstEdge) continue;

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
                    if (policy.extraAvoidChannelIndices.count(ci)) {
                        extra += policy.diversityPenaltyWeight * static_cast<double>(conn.netCount);
                    }
                } else {
                    const int bi = nodes[u].index;
                    if (!policy.allowSoftFT) continue;
                    double ftScore = model.scoreSoftFtUse(bi, static_cast<double>(conn.netCount));
                    if (!isfinite(ftScore)) continue;
                    extra += policy.softFtCostWeight * ftScore;
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
        (void)connectionIndex;
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

Phase1ResourceDelta RouterPhase1::analyzeDelta(const Design& design, const RoutePath& path) const {
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

double RouterPhase1::scoreCandidate(const RoutingResourceModel& model, const Phase1ResourceDelta& delta) const {
    if (delta.wireLength < 0.0) return numeric_limits<double>::infinity();
    double s = 0.0;
    s += 0.20 * delta.wireLength;
    s += 40.0 * static_cast<double>(delta.bendCount);

    map<int, pair<double, double>> channelUse;
    for (const auto& kv : delta.channelLR) channelUse[kv.first].first += kv.second;
    for (const auto& kv : delta.channelTB) channelUse[kv.first].second += kv.second;
    for (const auto& kv : channelUse) {
        s += model.scoreChannelUse(kv.first, kv.second.first, kv.second.second);
    }

    for (const auto& kv : delta.softFtNets) {
        s += model.scoreSoftFtUse(kv.first, kv.second);
    }
    for (const auto& t : delta.blockEdgeUse) {
        s += model.scoreBlockAccessUse(get<0>(t), get<1>(t), get<2>(t));
    }
    return s;
}

bool RouterPhase1::hardFeasibleCandidate(const RoutingResourceModel& model, const Phase1ResourceDelta& delta) const {
    map<int, pair<double, double>> channelUse;
    for (const auto& kv : delta.channelLR) channelUse[kv.first].first += kv.second;
    for (const auto& kv : delta.channelTB) channelUse[kv.first].second += kv.second;
    for (const auto& kv : channelUse) {
        if (!model.hardFeasibleChannel(kv.first, kv.second.first, kv.second.second)) return false;
    }
    return true;
}

bool RouterPhase1::softFeasibleCandidate(const RoutingResourceModel& model, const Phase1ResourceDelta& delta) const {
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

double RouterPhase1::calcRouteWireLength(const Design& design, const RoutePath& path) const {
    if (path.steps.size() < 2) return 0.0;
    vector<pair<double, double>> pts;
    pts.reserve(path.steps.size());
    for (const auto& st : path.steps) {
        const Rect* r = findRectByName(design, st.rectName);
        if (!r) continue;
        pts.push_back(pointOf(*r, st.edge));
    }
    double wl = 0.0;
    for (int i = 0; i + 1 < static_cast<int>(pts.size()); ++i) {
        wl += manhattan(pts[i].first, pts[i].second, pts[i + 1].first, pts[i + 1].second);
    }
    return wl * static_cast<double>(path.netCount);
}

string RouterPhase1::routeSignature(const RoutePath& path) const {
    ostringstream oss;
    oss << path.netCount << '|';
    for (const auto& st : path.steps) oss << st.rectName << ':' << st.edge << ';';
    return oss.str();
}

RouterPhase1::RunResult RouterPhase1::run(const Design& design, const string& inputPath, const string& outputCfgPath, double alpha) const {
    CandidateBuildResult built = buildCandidates(design);
    RunResult rr = built.stats;

    if (opt_.exportFiles) {
        RoutingResourceModel model;
        model.initialize(design, built.step0);
        rr.ok = writeReports(design, built.step0, model, built.candidates, rr, inputPath, outputCfgPath, alpha);
    }

    return rr;
}

RouterPhase1::CandidateBuildResult RouterPhase1::buildCandidates(const Design& design) const {
    CandidateBuildResult out;
    out.ok = true;
    out.stats.totalConnections = static_cast<int>(design.connections.size());

    CongestionMapBuilder::Options cmbOpt;
    cmbOpt.exportFiles = false;
    cmbOpt.verbose = false;
    cmbOpt.enableZPattern = true;
    cmbOpt.reroutePasses = 1;

    CongestionMapBuilder cmb(cmbOpt);
    out.step0 = cmb.build(design);

    RoutingResourceModel model;
    model.initialize(design, out.step0);

    vector<Node> nodes = buildNodes(design);
    vector<vector<AdjEdge>> graph = buildGraph(design, nodes);

    const vector<SearchPolicy> policies = {
        {"CH_MINWL", false, 0.20, 0.25, 10.0, 0.50, 0.00, 0.00, 70, -1},
        {"CH_BALANCED", false, 0.20, 1.00, 10.0, 1.00, 0.80, 1.20, 80, -1},
        {"CH_CMB_PREF", false, 0.20, 1.25, 10.0, 1.20, 1.80, 1.60, 85, -1},
        {"CH_DETOUR", false, 0.25, 1.80, 10.0, 1.10, 0.40, 2.20, 95, -1},
        {"BEND3", false, 0.22, 1.00, 10.0, 1.00, 1.00, 1.00, 75, 3},
        {"FT_ASSIST", true, 0.20, 1.10, 0.70, 1.00, 1.20, 1.60, 90, -1},
        {"FT_SAFE", true, 0.22, 1.40, 1.40, 1.20, 0.80, 1.80, 100, -1},
        {"BOUNDED_MAZE", true, 0.25, 1.70, 1.10, 1.00, 0.60, 2.00, 65, -1},
    };

    out.candidates.assign(design.connections.size(), {});

    for (int ci = 0; ci < static_cast<int>(design.connections.size()); ++ci) {
        const Connection& conn = design.connections[ci];
        const Step0ConnectionGuide* guide = ci < static_cast<int>(out.step0.connectionGuide.size()) ? &out.step0.connectionGuide[ci] : nullptr;

        vector<Phase1RouteCandidate> allCandidates;
        set<string> seenSig;

        auto addCandidate = [&](const SearchPolicy& pol) {
            RoutePath p = findCandidatePath(design, conn, ci, nodes, graph, model, guide, pol);
            if (p.open || p.steps.size() < 2) return;

            Phase1ResourceDelta d = analyzeDelta(design, p);
            if (pol.maxBends >= 0 && d.bendCount > pol.maxBends) return;

            const string sig = routeSignature(p);
            if (seenSig.count(sig)) return;
            seenSig.insert(sig);

            Phase1RouteCandidate c;
            c.connectionIndex = ci;
            c.candidateIndex = static_cast<int>(allCandidates.size());
            c.family = pol.family;
            c.allowSoftFT = pol.allowSoftFT;
            c.path = p;
            c.delta = d;
            c.wireLength = p.wireLength;
            c.bendCount = d.bendCount;
            c.channelTraversalCount = d.channelTraversalCount;
            c.softFtBlockCount = d.softFtBlockCount;
            c.score = scoreCandidate(model, d);
            c.hardFeasible = hardFeasibleCandidate(model, d);
            c.softFeasible = softFeasibleCandidate(model, d);

            allCandidates.push_back(c);
        };

        for (const auto& pol : policies) {
            addCandidate(pol);
        }

        set<int> srcEdges;
        set<int> dstEdges;
        for (const auto& e : graph[conn.src]) {
            if (edgeAllowedAtEndpoint(design, conn.src, e.edgeFrom)) srcEdges.insert(e.edgeFrom);
        }
        for (const auto& e : graph[conn.dst]) {
            if (edgeAllowedAtEndpoint(design, conn.dst, e.edgeFrom)) dstEdges.insert(e.edgeFrom);
        }

        for (int se : srcEdges) {
            SearchPolicy pol = policies[1];
            pol.family = "ACCESS_SRC_E" + to_string(se);
            pol.forcedSrcEdge = se;
            pol.maxHops = 100;
            addCandidate(pol);
        }
        for (int de : dstEdges) {
            SearchPolicy pol = policies[1];
            pol.family = "ACCESS_DST_E" + to_string(de);
            pol.forcedDstEdge = de;
            pol.maxHops = 100;
            addCandidate(pol);
        }

        int pairAttempts = 0;
        for (int se : srcEdges) {
            for (int de : dstEdges) {
                if (pairAttempts >= 12) break;
                SearchPolicy pol = policies[2];
                pol.family = "ACCESS_PAIR_E" + to_string(se) + "_E" + to_string(de);
                pol.forcedSrcEdge = se;
                pol.forcedDstEdge = de;
                pol.maxHops = 110;
                addCandidate(pol);
                ++pairAttempts;
            }
            if (pairAttempts >= 12) break;
        }

        map<int, int> usedChannelFreq;
        for (const auto& c : allCandidates) {
            for (int ch : phase1ChannelSet(c.delta)) usedChannelFreq[ch] += 1;
        }
        vector<pair<int, int>> channelAvoidList;
        for (const auto& kv : usedChannelFreq) channelAvoidList.push_back({kv.second, kv.first});
        sort(channelAvoidList.begin(), channelAvoidList.end(), greater<pair<int, int>>());

        int avoidAttempts = 0;
        for (const auto& kv : channelAvoidList) {
            if (avoidAttempts >= 8) break;
            const int ch = kv.second;
            SearchPolicy pol = policies[3];
            pol.family = "AVOID_CH" + to_string(ch);
            pol.bannedChannelIndices.insert(ch);
            pol.extraAvoidChannelIndices.insert(ch);
            pol.diversityPenaltyWeight = 250.0;
            pol.maxHops = 120;
            addCandidate(pol);

            SearchPolicy ftPol = policies[5];
            ftPol.family = "FT_DETOUR_CH" + to_string(ch);
            ftPol.channelCostWeight = 2.00;
            ftPol.softFtCostWeight = 2.40;
            ftPol.extraAvoidChannelIndices.insert(ch);
            ftPol.diversityPenaltyWeight = 400.0;
            ftPol.maxHops = 125;
            addCandidate(ftPol);
            ++avoidAttempts;
        }

        sort(allCandidates.begin(), allCandidates.end(), [](const Phase1RouteCandidate& a, const Phase1RouteCandidate& b) {
            if (a.hardFeasible != b.hardFeasible) return a.hardFeasible > b.hardFeasible;
            if (a.softFeasible != b.softFeasible) return a.softFeasible > b.softFeasible;
            if (fabs(a.score - b.score) > 1.0e-9) return a.score < b.score;
            if (fabs(a.wireLength - b.wireLength) > 1.0e-9) return a.wireLength < b.wireLength;
            return a.family < b.family;
        });

        vector<Phase1RouteCandidate> candVec;
        set<string> selectedResourceSig;
        set<string> selectedEndpointSig;
        set<string> selectedFamily;
        auto trySelect = [&](const Phase1RouteCandidate& c, bool requireNewResource, bool requireNewEndpointOrFamily) {
            if (static_cast<int>(candVec.size()) >= opt_.maxCandidatesPerConnection) return false;
            const string resSig = phase1DiversitySignature(c);
            string epSig = "S0D0";
            if (!c.path.steps.empty()) {
                epSig = "S" + to_string(c.path.steps.front().edge) + "D" + to_string(c.path.steps.back().edge);
            }
            if (requireNewResource && selectedResourceSig.count(resSig)) return false;
            if (requireNewEndpointOrFamily && selectedEndpointSig.count(epSig) && selectedFamily.count(c.family)) return false;
            candVec.push_back(c);
            selectedResourceSig.insert(resSig);
            selectedEndpointSig.insert(epSig);
            selectedFamily.insert(c.family);
            return true;
        };

        for (const auto& c : allCandidates) {
            trySelect(c, true, true);
        }
        for (const auto& c : allCandidates) {
            trySelect(c, true, false);
        }
        for (const auto& c : allCandidates) {
            trySelect(c, false, false);
        }

        if (static_cast<int>(candVec.size()) > opt_.maxCandidatesPerConnection) candVec.resize(opt_.maxCandidatesPerConnection);
        for (int i = 0; i < static_cast<int>(candVec.size()); ++i) candVec[i].candidateIndex = i;

        bool hasHard = false;
        set<string> finalResourceSig;
        for (const auto& c : candVec) {
            finalResourceSig.insert(phase1DiversitySignature(c));
            out.stats.generatedCandidates += 1;
            if (c.hardFeasible) {
                out.stats.hardFeasibleCandidates += 1;
                hasHard = true;
            }
            if (c.softFeasible) out.stats.softFeasibleCandidates += 1;
        }
        out.stats.resourceDiversitySignatures += static_cast<int>(finalResourceSig.size());
        const int diversityTarget = min(4, max(1, opt_.maxCandidatesPerConnection));
        if (static_cast<int>(finalResourceSig.size()) < min(diversityTarget, static_cast<int>(allCandidates.size()))) {
            out.stats.connectionsBelowDiversityTarget += 1;
        }
        if (candVec.empty()) out.stats.connectionsWithoutCandidate += 1;
        if (!hasHard) out.stats.connectionsWithoutHardCandidate += 1;

        out.candidates[ci] = move(candVec);
    }

    out.stats.ok = true;
    return out;
}

bool RouterPhase1::writeReports(
    const Design& design,
    const Step0CongestionMapResult& step0,
    const RoutingResourceModel& model,
    const vector<vector<Phase1RouteCandidate>>& candidates,
    const RunResult& rr,
    const string& inputPath,
    const string& outputCfgPath,
    double alpha
) const {
    fs::path outPath(outputCfgPath);
    fs::path reportDir("Router_Statistics");
    fs::create_directories(reportDir);
    const fs::path stem = reportDir / outPath.stem();

    const string summaryPath = (stem.string() + "_phase1_summary.txt");
    const string resourcePath = (stem.string() + "_phase1_resources.csv");
    const string candidatePath = (stem.string() + "_phase1_candidates.csv");
    const string routePath = (stem.string() + "_phase1_candidate_routes.csv");

    ofstream fsum(summaryPath);
    if (!fsum) return false;
    fsum << fixed << setprecision(6);
    fsum << "Phase 1 ResourceModel + CandidateFactory Summary\n";
    fsum << "input=" << inputPath << "\n";
    fsum << "output=" << outputCfgPath << "\n";
    fsum << "alpha=" << alpha << "\n";
    fsum << "connections=" << rr.totalConnections << "\n";
    fsum << "generated_candidates=" << rr.generatedCandidates << "\n";
    fsum << "hard_feasible_candidates=" << rr.hardFeasibleCandidates << "\n";
    fsum << "soft_feasible_candidates=" << rr.softFeasibleCandidates << "\n";
    fsum << "connections_without_candidate=" << rr.connectionsWithoutCandidate << "\n";
    fsum << "connections_without_hard_candidate=" << rr.connectionsWithoutHardCandidate << "\n";
    fsum << "resource_diversity_signatures=" << rr.resourceDiversitySignatures << "\n";
    fsum << "avg_resource_diversity_per_connection="
         << (rr.totalConnections > 0 ? static_cast<double>(rr.resourceDiversitySignatures) / static_cast<double>(rr.totalConnections) : 0.0) << "\n";
    fsum << "connections_below_diversity_target=" << rr.connectionsBelowDiversityTarget << "\n";
    fsum << "step0_channels=" << step0.channelPressure.size() << "\n";
    fsum << "step0_connection_guides=" << step0.connectionGuide.size() << "\n";
    fsum << "step0_block_edge_pressure=" << step0.blockEdgePressure.size() << "\n";
    fsum.close();

    ofstream fres(resourcePath);
    if (!fres) return false;
    fres << fixed << setprecision(6);
    fres << "resource_type,name,index,edge,hard_cap,soft_cap,ambient,criticality,history,used,aux0,aux1\n";
    for (const auto& r : model.channelResources()) {
        fres << "CHANNEL_LR," << r.channelName << ',' << r.channelIndex << ",0,"
             << r.hardCapLR << ',' << r.softCapLR << ',' << r.ambientLR << ','
             << r.criticalityLR << ',' << r.historyLR << ',' << r.usedLR << ",0,0\n";
        fres << "CHANNEL_TB," << r.channelName << ',' << r.channelIndex << ",0,"
             << r.hardCapTB << ',' << r.softCapTB << ',' << r.ambientTB << ','
             << r.criticalityTB << ',' << r.historyTB << ',' << r.usedTB << ",0,0\n";
    }
    for (const auto& r : model.softFtResources()) {
        const double overflow = softFtOverflowArea(design.blocks[r.blockIndex], r.usedNets);
        fres << "SOFT_FT," << r.blockName << ',' << r.blockIndex << ",0,"
             << r.currentArea << ',' << r.baseArea << ",0,0,1,"
             << r.usedNets << ',' << overflow << ",0\n";
    }
    for (const auto& r : model.blockAccessResources()) {
        fres << "BLOCK_ACCESS," << r.blockName << ',' << r.blockIndex << ',' << r.edge << ",0,0,0,"
             << r.severity << ",1,0," << r.edgePressure << ',' << r.healthyAccessChannelCount << "\n";
    }
    fres.close();

    ofstream fc(candidatePath);
    if (!fc) return false;
    fc << fixed << setprecision(6);
    fc << "connection_index,src,dst,net_count,candidate_index,family,allow_soft_ft,hard_feasible,soft_feasible,score,wirelength,bend_count,channel_traversals,soft_ft_blocks,path_step_count,diversity_signature\n";
    for (int ci = 0; ci < static_cast<int>(candidates.size()); ++ci) {
        const Connection& conn = design.connections[ci];
        for (const auto& c : candidates[ci]) {
            fc << ci << ','
               << design.blocks[conn.src].spec.name << ','
               << design.blocks[conn.dst].spec.name << ','
               << conn.netCount << ','
               << c.candidateIndex << ','
               << c.family << ','
               << yesNo(c.allowSoftFT) << ','
               << yesNo(c.hardFeasible) << ','
               << yesNo(c.softFeasible) << ','
               << c.score << ','
               << c.wireLength << ','
               << c.bendCount << ','
               << c.channelTraversalCount << ','
               << c.softFtBlockCount << ','
               << c.path.steps.size() << ','
               << '"' << phase1DiversitySignature(c) << '"' << "\n";
        }
    }
    fc.close();

    ofstream fr(routePath);
    if (!fr) return false;
    fr << "connection_index,candidate_index,family,steps\n";
    for (int ci = 0; ci < static_cast<int>(candidates.size()); ++ci) {
        for (const auto& c : candidates[ci]) {
            fr << ci << ',' << c.candidateIndex << ',' << c.family << ','
               << '"' << csvJoinSteps(c.path.steps) << '"' << "\n";
        }
    }
    fr.close();

    return true;
}
