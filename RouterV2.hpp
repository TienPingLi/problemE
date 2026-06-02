#pragma once

#include "DataModel.hpp"
#include "RouterV2State.hpp"

#include <string>

class RouterV2 {
public:
    struct Options {
        bool exportFiles = true;
    };

    struct RunResult {
        bool ok = false;
        RouterV2State state;
        std::string stateSummaryPath;
    };

    explicit RouterV2(Options opt = Options{}) : opt_(opt) {}

    RunResult route(
        Design& design,
        const std::string& inputPath,
        const std::string& outputPath,
        double alpha
    ) const;

private:
    bool writeStateSummary(const RouterV2State& state, const std::string& outputPath) const;
    Options opt_;
};
