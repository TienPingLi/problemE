#pragma once
#include "DataModel.hpp"
#include <string>
#include <vector>

class Router {
public:
    void setFTOverflowCostEnabled(bool enabled);
    void run(Design& design);

private:
    struct Node {
        std::string name;
        Rect rect;
        bool isBlock = false;
        int index = -1;
    };

    struct AdjEdge {
        int to = -1;
        int edgeFrom = 0;
        int edgeTo = 0;
        double baseCost = 0.0;
    };

    std::vector<Node> buildNodes(const Design& design) const;
    bool touchWithEdges(const Rect& a, const Rect& b, int& edgeA, int& edgeB) const;
    std::vector<std::vector<AdjEdge>> buildGraph(const Design& design, const std::vector<Node>& nodes) const;
    bool nodeAllowedAsIntermediate(const Design& design, const Node& node) const;
    double nodePenalty(const Design& design, const Node& node, int netCount) const;
    RoutePath routeOneConnection(const Design& design, const Connection& conn) const;
    RoutePath makeOpenRoute(const BlockInst& src, const BlockInst& dst, int netCount) const;
    double calcPathWLByRects(const Rect& a, int ea, const Rect& b, int eb) const;
    const Rect* findRectByName(const Design& design, const std::string& name) const;
    double calcRouteWireLength(const Design& design, const RoutePath& path) const;
    void updateUsage(Design& design, const RoutePath& path) const;

    mutable bool routeGraphCacheValid = false;
    mutable std::vector<Node> routeGraphNodes;
    mutable std::vector<std::vector<AdjEdge>> routeGraphAdj;
};
