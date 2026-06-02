#include "LocalRepairRouter.hpp"

#include "RouterPhase4.hpp"
#include "RouterPhase5.hpp"

bool LocalRepairRouter::run(
    const Design& design,
    const std::string& inputPath,
    const std::string& outputPath,
    double alpha,
    RouterV2State& state
) const {
    RouterPhase4::Options p4opt;
    p4opt.exportFiles = opt_.exportFiles;
    RouterPhase4 phase4(p4opt);
    state.phase4 = phase4.run(design, inputPath, outputPath, alpha);

    RouterV2PhaseStatus p4;
    p4.name = "LocalRepairRouter.Phase4";
    p4.ok = state.phase4.ok;
    p4.hasFail = state.phase4.finalEvalHasFail;
    p4.converged = state.phase4.converged;
    p4.routeCount = state.phase4.totalPieces;
    p4.openPaths = state.phase4.finalOpenConnections;
    p4.channelOverflow = state.phase4.finalTotalChannelOverflowEval;
    p4.feedthroughOverflow = state.phase4.finalTotalFeedthroughOverflowEval;
    p4.summaryPath = state.phase4.ftHandoffCsvPath;
    state.phaseStatus.push_back(p4);

    if (!state.phase4.ok) return false;

    RouterPhase5::Options p5opt;
    p5opt.exportFiles = opt_.exportFiles;
    RouterPhase5 phase5(p5opt);
    state.phase5 = phase5.run(design, inputPath, outputPath, alpha, &state.phase4);

    RouterV2PhaseStatus p5;
    p5.name = "LocalRepairRouter.Phase5";
    p5.ok = state.phase5.ok;
    p5.hasFail = state.phase5.phase5EvalHasFail;
    p5.routeCount = state.phase5.affectedRoutePieces;
    p5.openPaths = state.phase5.finalOpenPaths;
    p5.channelOverflow = state.phase5.finalChannelOverflow;
    p5.feedthroughOverflow = state.phase5.finalFeedthroughOverflow;
    p5.summaryPath = state.phase5.summaryPath;
    state.phaseStatus.push_back(p5);
    return state.phase5.ok;
}
