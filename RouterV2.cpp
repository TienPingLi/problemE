#include "RouterV2.hpp"

#include "BundleAllocator.hpp"
#include "CandidateFactory.hpp"
#include "Evaluator.hpp"
#include "LocalRepairRouter.hpp"
#include "NegotiatedRouter.hpp"
#include "PostRouteRecovery.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>

using namespace std;
namespace fs = std::filesystem;

namespace {

string yesNo(bool v) {
    return v ? "YES" : "NO";
}

string outputStemName(const string& outputPath) {
    fs::path out(outputPath);
    string stem = out.stem().string();
    return stem.empty() ? "routerv2" : stem;
}

fs::path reportStemPath(const string& outputPath) {
    fs::path dir("Router_Statistics");
    fs::create_directories(dir);
    return dir / outputStemName(outputPath);
}

} // namespace

RouterV2::RunResult RouterV2::route(
    Design& design,
    const string& inputPath,
    const string& outputPath,
    double alpha
) const {
    RunResult rr;
    rr.state.reset(inputPath, outputPath, alpha);
    rr.state.finalDesign = design;
    rr.stateSummaryPath = (reportStemPath(outputPath).string() + "_routerv2_state_summary.txt");

    CandidateFactory candidateFactory(CandidateFactory::Options{opt_.exportFiles});
    BundleAllocator bundleAllocator(BundleAllocator::Options{opt_.exportFiles});
    NegotiatedRouter negotiatedRouter(NegotiatedRouter::Options{opt_.exportFiles});
    LocalRepairRouter localRepairRouter(LocalRepairRouter::Options{opt_.exportFiles});
    PostRouteRecovery postRouteRecovery(PostRouteRecovery::Options{opt_.exportFiles});

    bool ok = true;
    ok = candidateFactory.run(design, inputPath, outputPath, alpha, rr.state) && ok;
    ok = bundleAllocator.run(design, inputPath, outputPath, alpha, rr.state) && ok;
    ok = negotiatedRouter.run(design, inputPath, outputPath, alpha, rr.state) && ok;
    ok = localRepairRouter.run(design, inputPath, outputPath, alpha, rr.state) && ok;
    ok = postRouteRecovery.run(design, inputPath, outputPath, alpha, rr.state) && ok;

    Evaluator evaluator;
    rr.state.finalEval = evaluator.evaluate(design, alpha);
    rr.state.finalEvalComputed = true;
    rr.state.finalDesign = design;
    rr.state.ok = ok && !rr.state.finalEval.hasFail();

    if (opt_.exportFiles) {
        ok = writeStateSummary(rr.state, outputPath) && ok;
    }

    rr.ok = ok;
    return rr;
}

bool RouterV2::writeStateSummary(const RouterV2State& state, const string& outputPath) const {
    const fs::path path = reportStemPath(outputPath).string() + "_routerv2_state_summary.txt";
    ofstream f(path);
    if (!f) return false;
    f << fixed << setprecision(6);
    f << "RouterV2 Shared State Summary\n";
    f << "input=" << state.inputPath << "\n";
    f << "output=" << state.outputPath << "\n";
    f << "alpha=" << state.alpha << "\n";
    f << "ok=" << yesNo(state.ok) << "\n";
    f << "final_eval_computed=" << yesNo(state.finalEvalComputed) << "\n";
    f << "final_has_fail=" << yesNo(state.finalEval.hasFail()) << "\n";
    f << "final_route_count=" << state.finalDesign.routes.size() << "\n";
    f << "final_open_paths=" << state.finalEval.openPathCount << "\n";
    f << "final_invalid_paths=" << state.finalEval.invalidPathCount << "\n";
    f << "final_channel_overflow=" << state.finalEval.totalChannelOverflow << "\n";
    f << "final_feedthrough_overflow=" << state.finalEval.totalFeedthroughOverflow << "\n";
    f << "candidate_connections=" << state.candidateBuild.candidates.size() << "\n";
    f << "generated_candidates=" << state.phase1.generatedCandidates << "\n";
    f << "phase_status_count=" << state.phaseStatus.size() << "\n";
    f << "phase,name,ok,has_fail,converged,route_count,open_paths,invalid_paths,channel_overflow,feedthrough_overflow,summary_path\n";
    for (size_t i = 0; i < state.phaseStatus.size(); ++i) {
        const auto& s = state.phaseStatus[i];
        f << i << ','
          << s.name << ','
          << yesNo(s.ok) << ','
          << yesNo(s.hasFail) << ','
          << yesNo(s.converged) << ','
          << s.routeCount << ','
          << s.openPaths << ','
          << s.invalidPaths << ','
          << s.channelOverflow << ','
          << s.feedthroughOverflow << ','
          << s.summaryPath << "\n";
    }
    return true;
}
