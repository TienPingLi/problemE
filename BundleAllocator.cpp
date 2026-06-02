#include "BundleAllocator.hpp"

#include "RouterPhase2.hpp"

bool BundleAllocator::run(
    const Design& design,
    const std::string& inputPath,
    const std::string& outputPath,
    double alpha,
    RouterV2State& state
) const {
    RouterPhase2::Options opt;
    opt.exportFiles = opt_.exportFiles;
    RouterPhase2 phase2(opt);
    state.phase2 = phase2.run(design, inputPath, outputPath, alpha, state.candidateBuild);

    RouterV2PhaseStatus st;
    st.name = "BundleAllocator";
    st.ok = state.phase2.ok;
    st.hasFail = state.phase2.phase2EvalHasFail;
    st.routeCount = state.phase2.totalPieces;
    st.openPaths = static_cast<int>(state.phase2.phase2OpenPathCount);
    st.channelOverflow = state.phase2.phase2TotalChannelOverflow;
    st.feedthroughOverflow = state.phase2.phase2TotalFeedthroughOverflow;
    state.phaseStatus.push_back(st);
    return st.ok;
}
