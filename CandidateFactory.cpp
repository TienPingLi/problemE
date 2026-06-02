#include "CandidateFactory.hpp"

#include "RouterPhase1.hpp"

bool CandidateFactory::run(
    const Design& design,
    const std::string& inputPath,
    const std::string& outputPath,
    double alpha,
    RouterV2State& state
) const {
    RouterPhase1 reporter(RouterPhase1::Options{16, opt_.exportFiles});
    state.phase1 = reporter.run(design, inputPath, outputPath, alpha);

    RouterPhase1::Options buildOpt;
    buildOpt.exportFiles = false;
    buildOpt.maxCandidatesPerConnection = 16;
    RouterPhase1 builder(buildOpt);
    state.candidateBuild = builder.buildCandidates(design);

    RouterV2PhaseStatus st;
    st.name = "CandidateFactory";
    st.ok = state.phase1.ok && state.candidateBuild.ok;
    st.hasFail = !state.candidateBuild.ok;
    state.phaseStatus.push_back(st);
    return st.ok;
}
