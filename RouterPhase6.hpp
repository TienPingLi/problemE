#pragma once

#include "DataModel.hpp"

#include <string>

class RouterPhase6 {
public:
    struct Options {
        bool exportFiles = true;
        int highRiskConnectionLimit = 50;
        int longPathStepThreshold = 6;
        int maxRecoveryConnections = 80;
        int maxCandidateTrialsPerConnection = 8;
        double wireLengthWeight = 1.0;
        double routeCountWeight = 10000.0;
        double duplicateSplitWeight = 20000.0;
        double routeChangePenalty = 2500.0;
    };

    struct RunResult {
        bool ok = false;
        int connectionCount = 0;
        int splitConnectionCount = 0;
        int duplicateSplitCount = 0;
        int ftHeavyConnectionCount = 0;
        int hotChannelConnectionCount = 0;
        int recoveryActionCount = 0;
        int attemptedRecoveries = 0;
        int acceptedRecoveries = 0;
        int candidateTrials = 0;
        int mergedDuplicatePieces = 0;
        int initialRouteCount = 0;
        int finalRouteCount = 0;
        int initialOpenPaths = 0;
        int finalOpenPaths = 0;
        int initialInvalidPaths = 0;
        int finalInvalidPaths = 0;
        double initialChannelOverflow = 0.0;
        double finalChannelOverflow = 0.0;
        double initialFeedthroughOverflow = 0.0;
        double finalFeedthroughOverflow = 0.0;
        double initialWireLength = 0.0;
        double finalWireLength = 0.0;
        double initialObjective = 0.0;
        double finalObjective = 0.0;
        bool phase6EvalComputed = false;
        bool phase6EvalHasFail = false;
        int rejectedByGuardCount = 0;
        int rejectedByObjectiveCount = 0;
        bool missingPhase4Artifacts = false;
        bool usedPhase5Artifacts = false;
        std::string summaryPath;
        std::string qualityPath;
        std::string actionsPath;
        std::string attemptsPath;
        std::string selectedRoutesPath;
        std::string resourceAfterPath;
        std::string phase6CfgPath;
    };

    explicit RouterPhase6(Options opt = Options{}) : opt_(opt) {}

    RunResult run(
        const Design& design,
        const std::string& inputPath,
        const std::string& outputCfgPath,
        double alpha,
        Design* outputDesign = nullptr
    ) const;

private:
    Options opt_;
};
