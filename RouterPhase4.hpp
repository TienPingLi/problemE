#pragma once

#include "DataModel.hpp"
#include "RouterPhase1.hpp"

#include <string>
#include <set>
#include <utility>
#include <vector>

struct Phase4AllocationPiece {
    int connectionIndex = -1;
    int candidateIndex = -1;
    std::string family;
    int allocatedNets = 0;
    int assignedIteration = 0;
    double score = 0.0;
    bool hardFeasible = false;
    bool softFeasible = false;
    double hardOverflowAmount = 0.0;
    double softOverflowAmount = 0.0;
    RoutePath path;
    Phase1ResourceDelta delta;
};

struct Phase4ConnectionStatus {
    int connectionIndex = -1;
    std::string srcBlock;
    std::string dstBlock;
    int netCount = 0;
    int allocatedNets = 0;
    int unallocatedNets = 0;
    int pieceCount = 0;
    int uniqueCandidateCount = 0;
    int rerouteCount = 0;
    double avgPieceScore = 0.0;
    double maxDominantShare = 0.0;
    double contributorScore = 0.0;
    bool usedHardFallback = false;
    bool isOpen = false;
};

struct Phase4IterationRecord {
    int iteration = 0;
    int rippedConnections = 0;
    int reroutedConnections = 0;
    int openConnections = 0;
    int overflowChannelCount = 0;
    int overflowSoftFtBlockCount = 0;
    double totalHardOverflow = 0.0;
    double maxHardOverflow = 0.0;
    double totalSoftOverflow = 0.0;
    double evaluatorChannelOverflow = 0.0;
    double evaluatorFeedthroughOverflow = 0.0;
    double evaluatorWireLength = 0.0;
    bool evaluatorHasFail = false;
};

class RouterPhase4 {
public:
    struct Options {
        bool exportFiles = true;
        int maxCandidatesPerConnection = 8;
        int preAugmentMinCandidates = 3;
        int preAugmentMaxPerConnection = 6;
        int maxRepairCandidatesPerConn = 10;
        std::vector<int> chunkSizes = {300, 100, 50, 20};
        int maxIterations = 8;
        double ripupRatio = 0.35;
        int minRipupConnections = 8;
        int maxRipupConnections = 120;
        bool allowInitialHardFallback = true;
        bool allowRerouteHardFallback = true;
        int hardFallbackEnableIteration = 4;
        double initialPresentFactor = 1.0;
        double presentFactorGrowth = 1.35;
        double historyStep = 1.0;
        double historyCap = 64.0;
        double ftHistoryStep = 0.50;
        double ftHistoryCap = 40.0;

        int localRepairRounds = 3;
        double localRepairRipupRatio = 0.25;
        int localRepairMinRipupConnections = 6;
        int localRepairMaxRipupConnections = 80;
        int hotChannelTopK = 24;
        int hotFtBlockTopK = 12;
        bool allowLocalRepairHardFallback = true;
        double localRepairPresentFactorBoost = 1.4;
        double hotChannelPenaltyWeight = 120.0;
        double hotFtPenaltyWeight = 2000.0;
        double ftContributorWeight = 2.0;
        double ftRegressionGuardRel = 0.01;
        double ftRegressionGuardAbs = 20000.0;
        double ftTotalRegressionGuardRel = 0.01;
        double ftTotalRegressionGuardAbs = 25000.0;
        double hardSlackWhenFtImproves = 2000.0;
        double candidateSoftOverflowWeight = 2.0;
    };

    struct RunResult {
        bool ok = false;
        bool converged = false;
        int totalConnections = 0;
        int iterationsRun = 0;
        int totalPieces = 0;
        int hardFallbackPieces = 0;
        int totalRippedConnections = 0;
        int totalReroutedConnections = 0;
        int preAugmentedCandidates = 0;
        int localRepairRoundsRun = 0;
        int localRepairRippedConnections = 0;
        int localRepairReroutedConnections = 0;
        int localRepairAddedCandidates = 0;
        int initialOpenConnections = 0;
        int finalOpenConnections = 0;
        double initialTotalHardOverflow = 0.0;
        double finalTotalHardOverflow = 0.0;
        double initialTotalChannelOverflowEval = 0.0;
        double finalTotalChannelOverflowEval = 0.0;
        double initialTotalFeedthroughOverflowEval = 0.0;
        double finalTotalFeedthroughOverflowEval = 0.0;
        double finalTotalSoftOverflowModel = 0.0;
        bool ftHandoffRequired = false;
        int ftHandoffBlockCount = 0;
        double ftHandoffTotalOverflow = 0.0;
        std::string ftHandoffCsvPath;
        bool finalEvalHasFail = false;
    };

    explicit RouterPhase4(Options opt = Options{}) : opt_(opt) {}

    RunResult run(const Design& design, const std::string& inputPath, const std::string& outputCfgPath, double alpha) const;

private:
    struct ScoredCandidate {
        int candidateIndex = -1;
        double score = 0.0;
        bool hardFeasible = false;
        bool softFeasible = false;
        bool usesHardFallback = false;
        double hardOverflowAmount = 0.0;
        double softOverflowAmount = 0.0;
        std::pair<int, int> dominantComp = {-1, 0}; // channelIndex, 1=LR 2=TB
        Phase1ResourceDelta delta;
        RoutePath path;
    };

    struct MutableConnectionState {
        int connectionIndex = -1;
        int netCount = 0;
        int rerouteCount = 0;
        bool usedHardFallback = false;
        double contributorScore = 0.0;
        std::vector<Phase4AllocationPiece> pieces;
    };

    struct OverflowSnapshot {
        int overflowChannelCount = 0;
        int overflowSoftFtBlockCount = 0;
        double totalHardOverflow = 0.0;
        double maxHardOverflow = 0.0;
        double totalSoftOverflow = 0.0;
        std::vector<double> channelOverflowLR;
        std::vector<double> channelOverflowTB;
        std::vector<double> softFtOverflow;
    };

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
        double avoidFtPenaltyWeight = 0.0;
        int maxHops = 80;
        int maxBends = -1;
    };

    int chooseChunkSize(int remaining, int allocatedSoFar) const;
    Phase1ResourceDelta scaleDelta(const Phase1ResourceDelta& base, double scale) const;
    RoutePath scalePath(const RoutePath& base, int nets) const;
    double evalHardOverflow(const RoutingResourceModel& model, const Phase1ResourceDelta& d) const;
    double evalSoftOverflow(const RoutingResourceModel& model, const Phase1ResourceDelta& d) const;
    std::pair<int, int> dominantComponent(const Phase1ResourceDelta& d) const;

    double connectionPriority(const Connection& conn, const Step0ConnectionGuide* guide, int maxNetCount) const;
    double guideAdjustment(const Step0ConnectionGuide* guide, const Phase1ResourceDelta& d, int chunkNets) const;
    double negotiatedPenalty(
        const RoutingResourceModel& model,
        const Phase1ResourceDelta& d,
        const std::vector<double>& histLR,
        const std::vector<double>& histTB,
        const std::vector<double>& histFt,
        double presentFactor
    ) const;
    double marginalScore(
        const RoutingResourceModel& model,
        const Phase1ResourceDelta& d,
        double diversityPenalty,
        const std::vector<double>& histLR,
        const std::vector<double>& histTB,
        const std::vector<double>& histFt,
        double presentFactor
    ) const;

    int centerFacingEdge(const Design& design, int blockIndex) const;
    RoutePath makeOpenPath(const Design& design, const Connection& conn, int nets) const;

    std::vector<Node> buildNodes(const Design& design) const;
    bool touchWithEdges(const Rect& a, const Rect& b, int& edgeA, int& edgeB) const;
    std::vector<std::vector<AdjEdge>> buildGraph(const Design& design, const std::vector<Node>& nodes) const;
    bool nodeAllowedAsIntermediate(const Design& design, const Node& node, bool allowSoftFt) const;
    bool edgeAllowedAtEndpoint(const Design& design, int blockIndex, int edge) const;
    const Rect* findRectByName(const Design& design, const std::string& name) const;
    RoutePath findCandidatePath(
        const Design& design,
        const Connection& conn,
        const std::vector<Node>& nodes,
        const std::vector<std::vector<AdjEdge>>& graph,
        const RoutingResourceModel& model,
        const Step0ConnectionGuide* guide,
        const SearchPolicy& policy,
        const std::set<int>& extraAvoidChannels,
        const std::set<int>& extraAvoidFtBlocks
    ) const;
    Phase1ResourceDelta analyzeDelta(const Design& design, const RoutePath& path) const;
    double scoreCandidateBase(const RoutingResourceModel& model, const Phase1ResourceDelta& delta) const;
    bool hardFeasibleCandidate(const RoutingResourceModel& model, const Phase1ResourceDelta& delta) const;
    bool softFeasibleCandidate(const RoutingResourceModel& model, const Phase1ResourceDelta& delta) const;
    double calcRouteWireLength(const Design& design, const RoutePath& path) const;
    std::string routeSignature(const RoutePath& path) const;
    bool addCandidateIfUnique(
        const Design& design,
        int ci,
        const RoutePath& path,
        const SearchPolicy& policy,
        const RoutingResourceModel& model,
        std::set<std::string>& signatures,
        std::vector<Phase1RouteCandidate>& pool
    ) const;
    int augmentCandidatesPreRRR(
        const Design& design,
        const RouterPhase1::CandidateBuildResult& built,
        const RoutingResourceModel& model,
        const std::vector<Node>& nodes,
        const std::vector<std::vector<AdjEdge>>& graph,
        std::vector<std::vector<Phase1RouteCandidate>>& candidates
    ) const;
    int generateRepairCandidatesForConnection(
        const Design& design,
        int ci,
        const RouterPhase1::CandidateBuildResult& built,
        const RoutingResourceModel& model,
        const std::vector<Node>& nodes,
        const std::vector<std::vector<AdjEdge>>& graph,
        const std::set<int>& hotChannels,
        const std::set<int>& hotFtBlocks,
        std::vector<Phase1RouteCandidate>& pool
    ) const;
    std::set<int> collectHotChannels(const OverflowSnapshot& ov) const;
    std::set<int> collectHotFtBlocks(const OverflowSnapshot& ov) const;
    double localRepairPenalty(
        const Phase1ResourceDelta& d,
        const std::set<int>& hotChannels,
        const std::set<int>& hotFtBlocks
    ) const;
    bool betterSnapshot(
        const EvalReport& candEval,
        const OverflowSnapshot& candOv,
        const EvalReport& bestEval,
        const OverflowSnapshot& bestOv,
        double ftAnchor
    ) const;

    bool allocateConnection(
        const Design& design,
        int ci,
        const std::vector<Phase1RouteCandidate>& candVec,
        const Step0ConnectionGuide* guide,
        RoutingResourceModel& model,
        MutableConnectionState& state,
        int assignedIteration,
        bool allowHardFallback,
        const std::vector<double>& histLR,
        const std::vector<double>& histTB,
        const std::vector<double>& histFt,
        double presentFactor,
        const std::set<int>* hotChannels,
        const std::set<int>* hotFtBlocks,
        int& hardFallbackPieces
    ) const;

    void ripupConnection(RoutingResourceModel& model, MutableConnectionState& state) const;
    OverflowSnapshot computeOverflowSnapshot(const Design& design, const RoutingResourceModel& model) const;
    void updateHistoryFromOverflow(
        const OverflowSnapshot& ov,
        std::vector<double>& histLR,
        std::vector<double>& histTB,
        std::vector<double>& histFt
    ) const;
    void computeContributorScores(
        const OverflowSnapshot& ov,
        std::vector<MutableConnectionState>& states
    ) const;
    std::vector<int> selectRipupSet(
        const OverflowSnapshot& ov,
        const std::vector<MutableConnectionState>& states
    ) const;

    std::vector<Phase4ConnectionStatus> buildConnectionStatus(
        const Design& design,
        const std::vector<MutableConnectionState>& states
    ) const;

    Design buildPhase4RoutedDesign(
        const Design& design,
        const std::vector<MutableConnectionState>& states
    ) const;

    bool writeReports(
        const Design& design,
        const RouterPhase1::CandidateBuildResult& phase1,
        const RoutingResourceModel& modelAfter,
        const std::vector<MutableConnectionState>& states,
        const std::vector<Phase4ConnectionStatus>& connStatus,
        const std::vector<Phase4IterationRecord>& iters,
        const EvalReport& finalEval,
        const std::string& Phase4CfgPath,
        const RunResult& rr,
        const std::string& inputPath,
        const std::string& outputCfgPath,
        double alpha
    ) const;

    Options opt_;
};

