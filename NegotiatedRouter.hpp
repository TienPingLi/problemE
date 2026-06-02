#pragma once

#include "DataModel.hpp"
#include "RouterV2State.hpp"

#include <string>

class NegotiatedRouter {
public:
    struct Options {
        bool exportFiles = true;
    };

    explicit NegotiatedRouter(Options opt = Options{}) : opt_(opt) {}

    bool run(
        const Design& design,
        const std::string& inputPath,
        const std::string& outputPath,
        double alpha,
        RouterV2State& state
    ) const;

private:
    Options opt_;
};
