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
    double floorplanAreaOverride = 0.0;

    bool hasFixedSize = false;
    double fixedW = 0.0;
    double fixedH = 0.0;

    double aspectMin = 1.0;
    double aspectMax = 1.0;

    std::vector<std::string> locations;
    std::vector<int> portEdges; // Empty means any edge is allowed. Uses output edge ids 1..4.

    // index 0: <=3000
    // index 1: >3000 && <=6000
    // index 2: >6000 && <=9000
    // index 3: >9000
    double ftRate[4] = { 0.20, 0.40, 0.80, 1.00 };
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

    // Directional whole-channel capacity/usage.  LR capacity is based on the
    // channel height; TB capacity is based on its width (25 nets/um).
    double lrUsed = 0.0;
    double tbUsed = 0.0;
    double lrCapacity = 0.0;
    double tbCapacity = 0.0;
    double lrOverflow = 0.0;
    double tbOverflow = 0.0;

    // Legacy aggregate fields are retained for Evaluator/Logger compatibility.
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

struct Design {
    double maxOutlineW = 0.0;
    double maxOutlineH = 0.0;
    double alpha = 1.0;

    double outlineW = 0.0;
    double outlineH = 0.0;

    // Optional routing core retained when the declared output outline is
    // expanded to create a legal four-side perimeter halo.  These coordinates
    // are runtime-only metadata (the CFG still declares just the final outline):
    // a perimeter candidate must visit a channel extending beyond this core.
    bool hasRoutingCore = false;
    double routingCoreX = 0.0;
    double routingCoreY = 0.0;
    double routingCoreW = 0.0;
    double routingCoreH = 0.0;

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
    double baseCost = 0.0;
    double cost = 0.0;

    double totalChannelCapacity = 0.0;
    double channelOverflowRate = 0.0;
    double totalChannelOverflow = 0.0;
    double maxChannelOverflow = 0.0;

    double totalFeedthroughOverflow = 0.0;
    double maxFeedthroughOverflow = 0.0;

    double illegalFeedthroughDeltaArea = 0.0;
    int illegalFeedthroughCount = 0;

    double edgeLocationOffset = 0.0;
    int edgeLocationViolationCount = 0;

    double overflowPenalty = 0.0;
    double feedthroughPenalty = 0.0;
    double illegalFeedthroughPenalty = 0.0;
    double edgeLocationPenalty = 0.0;
    double runtimePenalty = 0.0;
    double warningPenaltyCost = 0.0;

    bool formatFailed = false;
    bool blockOverlap = false;
    bool routingOpen = false;
    bool outlineViolation = false;

    int overlapCount = 0;
    int openPathCount = 0;
    int outlineViolationCount = 0;

    bool hasPenalty() const;
    bool hasFail() const;
};
