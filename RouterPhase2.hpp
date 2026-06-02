#pragma once

#include "DataModel.hpp"
#include "RouterPhase1.hpp"

#include <string>
#include <vector>

struct Phase2AllocationPiece {
    int connectionIndex = -1;
    int candidateIndex = -1;
    std::string family;
    int allocatedNets = 0;
    double score = 0.0;
    bool hardFeasible = false;
    bool softFeasible = false;
    double hardOverflowAmount = 0.0;
    double softOverflowAmount = 0.0;
    double ftDynamicPenalty = 0.0;
    RoutePath path;
    Phase1ResourceDelta delta;
};

struct Phase2ConnectionStatus {
    int connectionIndex = -1;
    std::string srcBlock;
    std::string dstBlock;
    int netCount = 0;
    int allocatedNets = 0;
    int unallocatedNets = 0;
    int pieceCount = 0;
    int uniqueCandidateCount = 0;
    double avgPieceScore = 0.0;
    double maxDominantShare = 0.0;
    bool usedHardFallback = false;
};

class RouterPhase2 {
public:
    struct Options {
        bool exportFiles = true;
        int maxCandidatesPerConnection = 8;
        std::vector<int> chunkSizes = {300, 100, 50, 20};
        bool allowHardFallback = true;
    };

    struct RunResult {
        bool ok = false;
        int totalConnections = 0;
        int fullyAllocatedConnections = 0;
        int partiallyAllocatedConnections = 0;
        int failedConnections = 0;
        int totalAllocatedNets = 0;
        int totalUnallocatedNets = 0;
        int totalPieces = 0;
        int hardFallbackPieces = 0;
        int dynamicFtPricedPieces = 0;
        double maxFtHistory = 0.0;
        double totalDynamicFtPenalty = 0.0;
        bool phase2EvalComputed = false;
        bool phase2EvalHasFail = false;
        double phase2TotalChannelOverflow = 0.0;
        double phase2MaxChannelOverflow = 0.0;
        double phase2TotalFeedthroughOverflow = 0.0;
        double phase2OpenPathCount = 0.0;
    };

    explicit RouterPhase2(Options opt = Options{}) : opt_(opt) {}

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
        double hardOverflowAmount = 0.0;
        double softOverflowAmount = 0.0;
        double ftDynamicPenalty = 0.0;
        std::pair<int, int> dominantComp = {-1, 0}; // channelIndex, 1=LR 2=TB
        Phase1ResourceDelta delta;
        RoutePath path;
    };

    int chooseChunkSize(int remaining, int allocatedSoFar) const;
    Phase1ResourceDelta scaleDelta(const Phase1ResourceDelta& base, double scale) const;
    RoutePath scalePath(const RoutePath& base, int nets) const;
    double evalHardOverflow(const RoutingResourceModel& model, const Phase1ResourceDelta& d) const;
    double evalSoftOverflow(const RoutingResourceModel& model, const Phase1ResourceDelta& d) const;
    std::pair<int, int> dominantComponent(const Phase1ResourceDelta& d) const;
    double dynamicFtPenalty(
        const Design& design,
        const RoutingResourceModel& model,
        const Phase1ResourceDelta& d,
        const std::vector<double>& histFt
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
        const std::vector<double>& histFt,
        double& ftPenaltyOut
    ) const;
    double connectionPriority(const Connection& conn, const Step0ConnectionGuide* guide, int maxNetCount) const;
    double guideAdjustment(const Step0ConnectionGuide* guide, const Phase1ResourceDelta& d, int chunkNets) const;
    int centerFacingEdge(const Design& design, int blockIndex) const;
    RoutePath makeOpenPath(const Design& design, const Connection& conn, int nets) const;
    Design buildPhase2RoutedDesign(
        const Design& design,
        const std::vector<Phase2AllocationPiece>& pieces,
        const std::vector<Phase2ConnectionStatus>& connStatus
    ) const;

    bool writeReports(
        const Design& design,
        const RouterPhase1::CandidateBuildResult& phase1,
        const RoutingResourceModel& modelAfter,
        const EvalReport& phase2Eval,
        const std::string& phase2CfgPath,
        const std::vector<Phase2AllocationPiece>& pieces,
        const std::vector<Phase2ConnectionStatus>& connStatus,
        const RunResult& rr,
        const std::string& inputPath,
        const std::string& outputCfgPath,
        double alpha
    ) const;

    Options opt_;
};
