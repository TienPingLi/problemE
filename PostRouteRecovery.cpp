#include "PostRouteRecovery.hpp"

#include "RouterPhase6.hpp"

bool PostRouteRecovery::run(
    Design& design,
    const std::string& inputPath,
    const std::string& outputPath,
    double alpha,
    RouterV2State& state
) const {
    RouterPhase6::Options opt;
    opt.exportFiles = opt_.exportFiles;
    RouterPhase6 phase6(opt);
    state.phase6 = phase6.run(design, inputPath, outputPath, alpha, &design);

    RouterV2PhaseStatus st;
    st.name = "PostRouteRecovery";
    st.ok = state.phase6.ok;
    st.hasFail = state.phase6.phase6EvalHasFail;
    st.routeCount = state.phase6.finalRouteCount;
    st.openPaths = state.phase6.finalOpenPaths;
    st.invalidPaths = state.phase6.finalInvalidPaths;
    st.channelOverflow = state.phase6.finalChannelOverflow;
    st.feedthroughOverflow = state.phase6.finalFeedthroughOverflow;
    st.summaryPath = state.phase6.summaryPath;
    state.phaseStatus.push_back(st);

    state.finalDesign = design;
    return state.phase6.ok;
}
