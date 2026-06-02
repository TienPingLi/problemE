#pragma once

#include "CongestionMapBuilder.hpp"
#include "DataModel.hpp"

#include <map>
#include <set>
#include <string>
#include <vector>

struct Phase1ChannelResource {
    int channelIndex = -1;
    std::string channelName;
    double hardCapLR = 0.0;
    double hardCapTB = 0.0;
    double softCapLR = 0.0;
    double softCapTB = 0.0;
    double ambientLR = 0.0;
    double ambientTB = 0.0;
    double criticalityLR = 0.0;
    double criticalityTB = 0.0;
    double historyLR = 1.0;
    double historyTB = 1.0;
    double usedLR = 0.0;
    double usedTB = 0.0;
};

struct Phase1SoftFtResource {
    int blockIndex = -1;
    std::string blockName;
    double baseArea = 0.0;
    double currentArea = 0.0;
    double usedNets = 0.0;
};

struct Phase1BlockAccessResource {
    int blockIndex = -1;
    std::string blockName;
    int edge = 0;
    double edgePressure = 0.0;
    double severity = 0.0;
    int healthyAccessChannelCount = 0;
};

struct Phase1ResourceDelta {
    std::vector<std::pair<int, double>> channelLR;
    std::vector<std::pair<int, double>> channelTB;
    std::vector<std::pair<int, double>> softFtNets;
    std::vector<std::tuple<int, int, double>> blockEdgeUse;
    double wireLength = 0.0;
    int bendCount = 0;
    int channelTraversalCount = 0;
    int softFtBlockCount = 0;
};

struct Phase1RouteCandidate {
    int connectionIndex = -1;
    int candidateIndex = -1;
    std::string family;
    bool allowSoftFT = false;
    bool hardFeasible = false;
    bool softFeasible = false;
    double score = 0.0;
    double wireLength = 0.0;
    int bendCount = 0;
    int channelTraversalCount = 0;
    int softFtBlockCount = 0;
    RoutePath path;
    Phase1ResourceDelta delta;
};

class RoutingResourceModel {
public:
    void initialize(const Design& design, const Step0CongestionMapResult& step0);

    double scoreChannelUse(int channelIndex, double addLR, double addTB) const;
    bool hardFeasibleChannel(int channelIndex, double addLR, double addTB) const;
    bool softFeasibleChannel(int channelIndex, double addLR, double addTB) const;

    double scoreSoftFtUse(int blockIndex, double addNets) const;
    bool softFeasibleSoftFt(int blockIndex, double addNets) const;

    double scoreBlockAccessUse(int blockIndex, int edge, double addNets) const;

    void commit(const Phase1ResourceDelta& delta);
    void ripup(const Phase1ResourceDelta& delta);

    const std::vector<Phase1ChannelResource>& channelResources() const { return channelRes_; }
    const std::vector<Phase1SoftFtResource>& softFtResources() const { return softFtRes_; }
    const std::vector<Phase1BlockAccessResource>& blockAccessResources() const { return blockAccessRes_; }

private:
    const Design* design_ = nullptr;
    std::vector<Phase1ChannelResource> channelRes_;
    std::vector<Phase1SoftFtResource> softFtRes_;
    std::vector<Phase1BlockAccessResource> blockAccessRes_;
    std::map<std::pair<int, int>, int> accessLookup_;
};

class RouterPhase1 {
public:
    struct Options {
        int maxCandidatesPerConnection = 16;
        bool exportFiles = true;
    };

    struct RunResult {
        bool ok = false;
        int totalConnections = 0;
        int generatedCandidates = 0;
        int hardFeasibleCandidates = 0;
        int softFeasibleCandidates = 0;
        int connectionsWithoutCandidate = 0;
        int connectionsWithoutHardCandidate = 0;
        int resourceDiversitySignatures = 0;
        int connectionsBelowDiversityTarget = 0;
    };

    struct CandidateBuildResult {
        bool ok = false;
        Step0CongestionMapResult step0;
        std::vector<std::vector<Phase1RouteCandidate>> candidates;
        RunResult stats;
    };

    explicit RouterPhase1(Options opt = Options{}) : opt_(opt) {}

    CandidateBuildResult buildCandidates(const Design& design) const;
    RunResult run(const Design& design, const std::string& inputPath, const std::string& outputCfgPath, double alpha) const;

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

    struct SearchPolicy {
        std::string family;
        bool allowSoftFT = false;
        double wireWeight = 0.20;
        double channelCostWeight = 1.0;
        double softFtCostWeight = 1.0;
        double accessCostWeight = 1.0;
        double preferredBonusWeight = 0.0;
        double avoidPenaltyWeight = 0.0;
        int maxHops = 80;
        int maxBends = -1;
        int forcedSrcEdge = 0;
        int forcedDstEdge = 0;
        std::set<int> bannedChannelIndices;
        std::set<int> extraPreferredChannelIndices;
        std::set<int> extraAvoidChannelIndices;
        double diversityPenaltyWeight = 0.0;
    };

    std::vector<Node> buildNodes(const Design& design) const;
    bool touchWithEdges(const Rect& a, const Rect& b, int& edgeA, int& edgeB) const;
    std::vector<std::vector<AdjEdge>> buildGraph(const Design& design, const std::vector<Node>& nodes) const;
    bool nodeAllowedAsIntermediate(const Design& design, const Node& node, bool allowSoftFt) const;
    int centerFacingEdge(const Design& design, int blockIndex) const;
    bool edgeAllowedAtEndpoint(const Design& design, int blockIndex, int edge) const;
    const Rect* findRectByName(const Design& design, const std::string& name) const;

    RoutePath findCandidatePath(
        const Design& design,
        const Connection& conn,
        int connectionIndex,
        const std::vector<Node>& nodes,
        const std::vector<std::vector<AdjEdge>>& graph,
        const RoutingResourceModel& model,
        const Step0ConnectionGuide* guide,
        const SearchPolicy& policy
    ) const;

    Phase1ResourceDelta analyzeDelta(const Design& design, const RoutePath& path) const;
    double scoreCandidate(const RoutingResourceModel& model, const Phase1ResourceDelta& delta) const;
    bool hardFeasibleCandidate(const RoutingResourceModel& model, const Phase1ResourceDelta& delta) const;
    bool softFeasibleCandidate(const RoutingResourceModel& model, const Phase1ResourceDelta& delta) const;
    double calcRouteWireLength(const Design& design, const RoutePath& path) const;
    std::string routeSignature(const RoutePath& path) const;

    bool writeReports(
        const Design& design,
        const Step0CongestionMapResult& step0,
        const RoutingResourceModel& model,
        const std::vector<std::vector<Phase1RouteCandidate>>& candidates,
        const RunResult& rr,
        const std::string& inputPath,
        const std::string& outputCfgPath,
        double alpha
    ) const;

    Options opt_;
};
