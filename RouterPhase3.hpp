#pragma once

#include "DataModel.hpp"
#include "RouterPhase1.hpp"

#include <string>
#include <utility>
#include <vector>

struct Phase3AllocationPiece {
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
    double ftDynamicPenalty = 0.0;
    RoutePath path;
    Phase1ResourceDelta delta;
};

struct Phase3ConnectionStatus {
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

struct Phase3IterationRecord {
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

class RouterPhase3 {
public:
    struct Options {
        bool exportFiles = true;
        int maxCandidatesPerConnection = 8;
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
    };

    struct RunResult {
        bool ok = false;
        bool converged = false;
        int totalConnections = 0;
        int iterationsRun = 0;
        int totalPieces = 0;
        int hardFallbackPieces = 0;
        int dynamicFtPricedPieces = 0;
        double maxFtHistory = 0.0;
        double totalDynamicFtPenalty = 0.0;
        int totalRippedConnections = 0;
        int totalReroutedConnections = 0;
        int initialOpenConnections = 0;
        int finalOpenConnections = 0;
        double initialTotalHardOverflow = 0.0;
        double finalTotalHardOverflow = 0.0;
        double initialTotalChannelOverflowEval = 0.0;
        double finalTotalChannelOverflowEval = 0.0;
        double initialTotalFeedthroughOverflowEval = 0.0;
        double finalTotalFeedthroughOverflowEval = 0.0;
        bool finalEvalHasFail = false;
    };

    explicit RouterPhase3(Options opt = Options{}) : opt_(opt) {}

    RunResult run(const Design& design, const std::string& inputPath, const std::string& outputCfgPath, double alpha) const;
    RunResult run(
        const Design& design,
        const std::string& inputPath,
        const std::string& outputCfgPath,
        double alpha,
        const RouterPhase1::CandidateBuildResult& candidates
    ) const;

private:
    struct ScoredCandidate {
        int candidateIndex = -1;
        double score = 0.0;
        bool hardFeasible = false;
        bool softFeasible = false;
        bool usesHardFallback = false;
        double hardOverflowAmount = 0.0;
        double softOverflowAmount = 0.0;
        double ftDynamicPenalty = 0.0;
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
        std::vector<Phase3AllocationPiece> pieces;
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
    double dynamicFtPenalty(
        const Design& design,
        const RoutingResourceModel& model,
        const Phase1ResourceDelta& d,
        const std::vector<double>& histFt,
        double presentFactor
    ) const;
    void updateFtHistoryFromModel(
        const Design& design,
        const RoutingResourceModel& model,
        std::vector<double>& histFt
    ) const;
    double marginalScore(
        const Design& design,
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
        std::vector<double>& histFt,
        double presentFactor,
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

    std::vector<Phase3ConnectionStatus> buildConnectionStatus(
        const Design& design,
        const std::vector<MutableConnectionState>& states
    ) const;

    Design buildPhase3RoutedDesign(
        const Design& design,
        const std::vector<MutableConnectionState>& states
    ) const;

    bool writeReports(
        const Design& design,
        const RouterPhase1::CandidateBuildResult& phase1,
        const RoutingResourceModel& modelAfter,
        const std::vector<MutableConnectionState>& states,
        const std::vector<Phase3ConnectionStatus>& connStatus,
        const std::vector<Phase3IterationRecord>& iters,
        const EvalReport& finalEval,
        const std::string& phase3CfgPath,
        const RunResult& rr,
        const std::string& inputPath,
        const std::string& outputCfgPath,
        double alpha
    ) const;

    Options opt_;
};
