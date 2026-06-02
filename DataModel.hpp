#pragma once
#include <string>
#include <unordered_map>
#include <vector>

// ================================================================
// Shared data model for all modules.
// Everyone in the team should include this file and avoid redefining
// their own Block / Channel / Route structures.
// ================================================================

enum class BlockType {
    EDGE,
    HARD,
    SOFT,
    UNKNOWN
};

std::string blockTypeToString(BlockType t);

struct Rect {
    double x = 0.0;
    double y = 0.0;
    double w = 0.0;
    double h = 0.0;
};

double rectRight(const Rect& r);
double rectTop(const Rect& r);
double rectCx(const Rect& r);
double rectCy(const Rect& r);
double overlapLen(double a1, double a2, double b1, double b2);
bool rectOverlapAreaPositive(const Rect& a, const Rect& b);
double manhattan(double x1, double y1, double x2, double y2);
std::pair<double, double> edgeCenterPoint(const Rect& r, int edge);

struct BlockSpec {
    std::string name;
    BlockType type = BlockType::UNKNOWN;

    double area = 0.0;

    bool hasFixedSize = false;
    double fixedW = 0.0;
    double fixedH = 0.0;

    double aspectMin = 1.0;
    double aspectMax = 1.0;

    std::vector<std::string> locations;

    // index 0: <=3000
    // index 1: >3000 && <=6000
    // index 2: >6000 && <=9000
    // index 3: >9000
    double ftRate[4] = {0.20, 0.40, 0.80, 1.00};
};

struct BlockInst {
    BlockSpec spec;
    Rect rect;

    double ftUsed = 0.0;
    double ftOverflowArea = 0.0;
};

struct Channel {
    std::string name;
    Rect rect;

    double usedNets = 0.0;
    double capacity = 0.0;
    double overflow = 0.0;
};

struct Connection {
    int src = -1;
    int dst = -1;
    int netCount = 0;
};

struct RouteStep {
    std::string rectName; // BLKxx or CHxx
    int edge = 0;         // 1:left, 2:top, 3:right, 4:bottom
};

struct RoutePath {
    int netCount = 0;
    std::string srcBlock;
    std::string dstBlock;

    std::vector<RouteStep> steps;

    bool open = false;
    double wireLength = 0.0;
};

struct RouteTruth {
    int routeId = -1;
    int netCount = 0;
    std::string srcBlock;
    std::string dstBlock;
    int itemCount = 0;
    int guidingPointCount = 0;
    int channelTraversalCount = 0;
    int feedthroughTraversalCount = 0;
    bool explicitOpen = false;
    bool invalid = false;
    std::string invalidReason;
    std::string invalidDetail;
    double wireLength = 0.0;
};

struct ChannelTruth {
    std::string name;
    Rect rect;
    double lrUsed = 0.0;
    double tbUsed = 0.0;
    double lrCapacity = 0.0;
    double tbCapacity = 0.0;
    double lrUtilization = 0.0;
    double tbUtilization = 0.0;
    double lrOverflow = 0.0;
    double tbOverflow = 0.0;
    int lrTraversalCount = 0;
    int tbTraversalCount = 0;
    int turnTraversalCount = 0;
};

struct ChannelSegmentTruth {
    int routeId = -1;
    std::string channelName;
    int netCount = 0;
    int inEdge = 0;
    int outEdge = 0;
    double fromX = 0.0;
    double fromY = 0.0;
    double toX = 0.0;
    double toY = 0.0;
    double lrDemand = 0.0;
    double tbDemand = 0.0;
    double lrSpanLo = 0.0;
    double lrSpanHi = 0.0;
    double tbSpanLo = 0.0;
    double tbSpanHi = 0.0;
};

struct FeedthroughTruth {
    std::string blockName;
    double usedNets = 0.0;
    double conversionRate = 0.0;
    double sideDelta = 0.0;
    double baseArea = 0.0;
    double currentArea = 0.0;
    double requiredArea = 0.0;
    double overflowArea = 0.0;
};

struct Design {
    double maxOutlineW = 0.0;
    double maxOutlineH = 0.0;

    double outlineW = 0.0;
    double outlineH = 0.0;

    std::vector<BlockSpec> blockSpecs;
    std::vector<BlockInst> blocks;

    std::vector<std::vector<int>> connMatrix;
    std::vector<Connection> connections;

    std::vector<Channel> channels;
    std::vector<RoutePath> routes;

    std::unordered_map<std::string, int> blockNameToIndex;
};

struct EvalReport {
    double outlineArea = 0.0;
    double totalWireLength = 0.0;
    double cost = 0.0;

    double totalChannelOverflow = 0.0;
    double maxChannelOverflow = 0.0;

    double totalFeedthroughOverflow = 0.0;
    double maxFeedthroughOverflow = 0.0;

    bool formatFailed = false;
    bool pathInvalid = false;
    bool blockOverlap = false;
    bool routingOpen = false;
    bool outlineViolation = false;

    int overlapCount = 0;
    int openPathCount = 0;
    int outlineViolationCount = 0;
    int invalidPathCount = 0;
    int badTopologyCount = 0;
    int unknownObjectCount = 0;
    int badItemCount = 0;
    int illegalFeedthroughCount = 0;
    int contactFailCount = 0;
    int sameObjectSameEdgeCount = 0;
    int edgePortViolationCount = 0;

    std::vector<RouteTruth> routeTruth;
    std::vector<ChannelTruth> channelTruth;
    std::vector<ChannelSegmentTruth> channelSegments;
    std::vector<FeedthroughTruth> feedthroughTruth;

    bool hasPenalty() const;
    bool hasFail() const;
};
