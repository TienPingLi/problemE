#pragma once

#include "DataModel.hpp"
#include "RouterPhase1.hpp"
#include "RouterPhase2.hpp"
#include "RouterPhase3.hpp"
#include "RouterPhase4.hpp"
#include "RouterPhase5.hpp"
#include "RouterPhase6.hpp"

#include <string>
#include <vector>

struct RouterV2PhaseStatus {
    std::string name;
    bool ok = false;
    bool hasFail = false;
    bool converged = false;
    int routeCount = 0;
    int openPaths = 0;
    int invalidPaths = 0;
    double channelOverflow = 0.0;
    double feedthroughOverflow = 0.0;
    std::string summaryPath;
};

struct RouterV2State {
    std::string inputPath;
    std::string outputPath;
    double alpha = 0.2;

    RouterPhase1::CandidateBuildResult candidateBuild;
    RouterPhase1::RunResult phase1;
    RouterPhase2::RunResult phase2;
    RouterPhase3::RunResult phase3;
    RouterPhase4::RunResult phase4;
    RouterPhase5::RunResult phase5;
    RouterPhase6::RunResult phase6;

    Design finalDesign;
    EvalReport finalEval;
    bool finalEvalComputed = false;
    bool ok = false;

    std::vector<RouterV2PhaseStatus> phaseStatus;

    void reset(const std::string& in, const std::string& out, double a) {
        *this = RouterV2State{};
        inputPath = in;
        outputPath = out;
        alpha = a;
    }
};
