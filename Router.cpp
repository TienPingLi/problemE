#include "Router.hpp"
#include "Utility.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <set>
#include <utility>

using namespace std;

void Router::run(Design& design) {
    design.routes.clear();

    for (auto& ch : design.channels) {
        ch.usedNets = 0.0;
        ch.overflow = 0.0;
    }
    for (auto& b : design.blocks) {
        b.ftUsed = 0.0;
        b.ftOverflowArea = 0.0;
    }

    vector<Connection> conns = design.connections;
    sort(conns.begin(), conns.end(), [](const Connection& a, const Connection& b) {
        return a.netCount > b.netCount;
    });

    for (const auto& conn : conns) {
        RoutePath path = routeOneConnection(design, conn);
        updateUsage(design, path);
        design.routes.push_back(path);
    }
}

vector<Router::Node> Router::buildNodes(const Design& design) const {
    vector<Node> nodes;
    for (int i = 0; i < static_cast<int>(design.blocks.size()); ++i) {
        nodes.push_back({design.blocks[i].spec.name, design.blocks[i].rect, true, i});
    }
    for (int i = 0; i < static_cast<int>(design.channels.size()); ++i) {
        nodes.push_back({design.channels[i].name, design.channels[i].rect, false, i});
    }
    return nodes;
}

bool Router::touchWithEdges(const Rect& a, const Rect& b, int& edgeA, int& edgeB) const {
    if (fabs(rectRight(a) - b.x) < 1e-5 && overlapLen(a.y, rectTop(a), b.y, rectTop(b)) > EPS) {
        edgeA = 3;
        edgeB = 1;
        return true;
    }
    if (fabs(a.x - rectRight(b)) < 1e-5 && overlapLen(a.y, rectTop(a), b.y, rectTop(b)) > EPS) {
        edgeA = 1;
        edgeB = 3;
        return true;
    }
    if (fabs(rectTop(a) - b.y) < 1e-5 && overlapLen(a.x, rectRight(a), b.x, rectRight(b)) > EPS) {
        edgeA = 2;
        edgeB = 4;
        return true;
    }
    if (fabs(a.y - rectTop(b)) < 1e-5 && overlapLen(a.x, rectRight(a), b.x, rectRight(b)) > EPS) {
        edgeA = 4;
        edgeB = 2;
        return true;
    }
    return false;
}

vector<vector<Router::AdjEdge>> Router::buildGraph(const Design& design, const vector<Node>& nodes) const {
    int N = static_cast<int>(nodes.size());
    vector<vector<AdjEdge>> g(N);

    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            int ei = 0, ej = 0;
            if (!touchWithEdges(nodes[i].rect, nodes[j].rect, ei, ej)) continue;

            double base = manhattan(rectCx(nodes[i].rect), rectCy(nodes[i].rect), rectCx(nodes[j].rect), rectCy(nodes[j].rect));
            g[i].push_back({j, ei, ej, base});
            g[j].push_back({i, ej, ei, base});
        }
    }
    return g;
}

bool Router::nodeAllowedAsIntermediate(const Design& design, const Node& node) const {
    if (!node.isBlock) return true;
    const auto& b = design.blocks[node.index];
    return b.spec.type == BlockType::SOFT;
}

double Router::nodePenalty(const Design& design, const Node& node, int netCount) const {
    if (!node.isBlock) {
        const Channel& ch = design.channels[node.index];
        double projected = ch.usedNets + netCount;
        double overflow = max(0.0, projected - ch.capacity);
        return 0.05 * projected + 1000.0 * overflow;
    }

    const BlockInst& b = design.blocks[node.index];
    if (b.spec.type == BlockType::SOFT) return 200.0 + 0.02 * (b.ftUsed + netCount);
    return 1e12;
}

RoutePath Router::routeOneConnection(const Design& design, const Connection& conn) const {
    RoutePath result;
    result.netCount = conn.netCount;
    result.srcBlock = design.blocks[conn.src].spec.name;
    result.dstBlock = design.blocks[conn.dst].spec.name;

    vector<Node> nodes = buildNodes(design);
    vector<vector<AdjEdge>> g = buildGraph(design, nodes);

    int srcNode = conn.src;
    int dstNode = conn.dst;
    int N = static_cast<int>(nodes.size());

    vector<double> dist(N, numeric_limits<double>::infinity());
    vector<int> parent(N, -1);
    vector<int> parentEdgeIn(N, 0);
    vector<int> parentEdgeOut(N, 0);

    using P = pair<double, int>;
    priority_queue<P, vector<P>, greater<P>> pq;

    dist[srcNode] = 0.0;
    pq.push({0.0, srcNode});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();
        if (d != dist[u]) continue;
        if (u == dstNode) break;

        for (const auto& e : g[u]) {
            int v = e.to;
            if (v != dstNode && v != srcNode && !nodeAllowedAsIntermediate(design, nodes[v])) continue;

            double nd = d + e.baseCost + nodePenalty(design, nodes[v], conn.netCount);
            if (nd < dist[v]) {
                dist[v] = nd;
                parent[v] = u;
                parentEdgeOut[v] = e.edgeFrom;
                parentEdgeIn[v] = e.edgeTo;
                pq.push({nd, v});
            }
        }
    }

    if (parent[dstNode] < 0) return makeOpenRoute(design.blocks[conn.src], design.blocks[conn.dst], conn.netCount);

    vector<int> pathNodes;
    for (int cur = dstNode; cur != -1; cur = parent[cur]) pathNodes.push_back(cur);
    reverse(pathNodes.begin(), pathNodes.end());

    for (int k = 1; k < static_cast<int>(pathNodes.size()); ++k) {
        int u = pathNodes[k - 1];
        int v = pathNodes[k];
        int edgeU = parentEdgeOut[v];
        int edgeV = parentEdgeIn[v];
        result.steps.push_back({nodes[u].name, edgeU});
        result.steps.push_back({nodes[v].name, edgeV});
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

    int srcEdge = rectCx(dst.rect) >= rectCx(src.rect) ? 3 : 1;
    int dstEdge = edgeOpposite(srcEdge);
    p.steps.push_back({src.spec.name, srcEdge});
    p.steps.push_back({dst.spec.name, dstEdge});
    p.wireLength = calcPathWLByRects(src.rect, srcEdge, dst.rect, dstEdge) * netCount;
    return p;
}

double Router::calcPathWLByRects(const Rect& a, int ea, const Rect& b, int eb) const {
    auto pa = edgeCenterPoint(a, ea);
    auto pb = edgeCenterPoint(b, eb);
    return manhattan(pa.first, pa.second, pb.first, pb.second);
}

const Rect* Router::findRectByName(const Design& design, const string& name) const {
    for (const auto& b : design.blocks) {
        if (b.spec.name == name) return &b.rect;
    }
    for (const auto& ch : design.channels) {
        if (ch.name == name) return &ch.rect;
    }
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

void Router::updateUsage(Design& design, const RoutePath& path) const {
    if (path.open) return;

    set<string> channelTouched;
    set<string> blockTouched;

    for (const auto& st : path.steps) {
        if (st.rectName.rfind("CH", 0) == 0) channelTouched.insert(st.rectName);
        else if (startsWithBlockName(st.rectName)) blockTouched.insert(st.rectName);
    }

    for (const string& chName : channelTouched) {
        for (auto& ch : design.channels) {
            if (ch.name == chName) {
                ch.usedNets += path.netCount;
                break;
            }
        }
    }

    for (const string& bName : blockTouched) {
        if (bName == path.srcBlock || bName == path.dstBlock) continue;
        auto it = design.blockNameToIndex.find(bName);
        if (it == design.blockNameToIndex.end()) continue;

        BlockInst& b = design.blocks[it->second];
        if (b.spec.type == BlockType::SOFT) b.ftUsed += path.netCount;
    }
}
