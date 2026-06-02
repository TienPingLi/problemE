#include "NegotiatedRouter.hpp"

#include "RouterPhase3.hpp"

bool NegotiatedRouter::run(
    const Design& design,
    const std::string& inputPath,
    const std::string& outputPath,
    double alpha,
    RouterV2State& state
) const {
    RouterPhase3::Options opt;
    opt.exportFiles = opt_.exportFiles;
    RouterPhase3 phase3(opt);
    state.phase3 = phase3.run(design, inputPath, outputPath, alpha, state.candidateBuild);

    RouterV2PhaseStatus st;
    st.name = "NegotiatedRouter";
    st.ok = state.phase3.ok;
    st.hasFail = state.phase3.finalEvalHasFail;
    st.converged = state.phase3.converged;
    st.routeCount = state.phase3.totalPieces;
    st.openPaths = state.phase3.finalOpenConnections;
    st.channelOverflow = state.phase3.finalTotalChannelOverflowEval;
    st.feedthroughOverflow = state.phase3.finalTotalFeedthroughOverflowEval;
    state.phaseStatus.push_back(st);
    return st.ok;
}
