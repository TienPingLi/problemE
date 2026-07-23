#pragma once
#include <string>
#include <vector>
#include <filesystem>

// 
namespace IOUtils {

    struct Options {
        std::string inputPath;
        std::string outputPath;
        bool outputPathProvided = false;
        std::string evalCfgPath;
        bool evalCfgProvided = false;
        std::string routeCfgBlocksPath;
        bool routeCfgBlocksProvided = false;
        double alpha = 1.0;
        bool alphaOverride = false;
    };

    Options parseArgs(int argc, char** argv);
    void printUsage();
    std::string makeDefaultOutputPathFromInput(const std::string& inputPath);
    std::string resolveOutputPath(const std::string& rawOutputPath, size_t blockCount);

} // namespace IOUtils