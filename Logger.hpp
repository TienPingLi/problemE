#pragma once
#include "DataModel.hpp"
#include <string>

class Logger {
public:
    static void printFinalReport(const Design& design, const EvalReport& rpt, double alpha,
        const std::string& inputPath, const std::string& outputPath);
};
