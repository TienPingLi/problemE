#pragma once

#include "DataModel.hpp"
#include "RouterPhase4.hpp"

#include <string>

class RouterPhase5 {
public:
    struct Options {
        bool exportFiles = true;
        int maxBoxes = 16;
        int maxFtBoxes = 10;
        int maxChannelBoxes = 10;
        int maxRepairConnections = 80;
        int maxTrialsPerConnection = 8;
        double minBoxExpansion = 20.0;
        double maxBoxExpansionRatio = 0.18;
        double channelRegressionGuardAbs = 5000.0;
        double ftRegressionGuardAbs = 50000.0;
        double channelPenaltyWeight = 1000000.0;
        double ftPenaltyWeight = 2000.0;
        double openPenaltyWeight = 1000000000000.0;
        double invalidPenaltyWeight = 1000000000000.0;
        double wireLengthWeight = 0.2;
        double routeChangePenalty = 2500.0;
    };

    struct RunResult {
        bool ok = false;
        int ftHotspotCount = 0;
        int channelHotspotCount = 0;
        int repairBoxCount = 0;
        int ftTargetCount = 0;
        int affectedRoutePieces = 0;
        int affectedNetCount = 0;
        int repairConnectionCount = 0;
        int attemptedRepairs = 0;
        int acceptedRepairs = 0;
        int candidateTrials = 0;
        bool phase5EvalComputed = false;
        bool phase5EvalHasFail = false;
        int initialOpenPaths = 0;
        int finalOpenPaths = 0;
        double initialChannelOverflow = 0.0;
        double finalChannelOverflow = 0.0;
        double initialFeedthroughOverflow = 0.0;
        double finalFeedthroughOverflow = 0.0;
        double initialWireLength = 0.0;
        double finalWireLength = 0.0;
        double initialObjective = 0.0;
        double finalObjective = 0.0;
        bool missingPhase4Artifacts = false;
        std::string summaryPath;
        std::string boxesPath;
        std::string actionsPath;
        std::string ftTargetsPath;
        std::string repairAttemptsPath;
        std::string selectedRoutesPath;
        std::string resourceAfterPath;
        std::string phase5CfgPath;
    };

    explicit RouterPhase5(Options opt = Options{}) : opt_(opt) {}

    RunResult run(
        const Design& design,
        const std::string& inputPath,
        const std::string& outputCfgPath,
        double alpha,
        const RouterPhase4::RunResult* phase4Result = nullptr
    ) const;

private:
    Options opt_;
};
