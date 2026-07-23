
/*
* MOVE
* parseArgs、printUsage
* makeDefaultOutputPathFromInput、resolveOutputPath、endsWithCfg
* getLocalTimeNow、makeAutoCfgFileName
*/
#include "IOUtils.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <filesystem>
#include <algorithm>

namespace fs = std::filesystem;

namespace IOUtils {

    void printUsage() {
        std::cerr << "Usage:\n";
        std::cerr << "  ./EarlyFloorplanning_with_GlobalRoute input.csv\n\n";
        std::cerr << "Output defaults to the input filename with .cfg extension.\n";
        std::cerr << "Local debug options are still accepted: -o output.cfg --alpha 0.2 --eval-cfg candidate.cfg --route-cfg-blocks candidate.cfg\n";
    }

    Options parseArgs(int argc, char** argv) {
        Options opt;
        if (argc < 2) {
            printUsage();
            exit(1);
        }

        opt.inputPath = argv[1];

        for (int i = 2; i < argc; ++i) {
            std::string arg = argv[i];
            if ((arg == "-o" || arg == "--o" || arg == "--output") && i + 1 < argc) {
                opt.outputPath = argv[++i];
                opt.outputPathProvided = true;
            }
            else if (arg == "--alpha" && i + 1 < argc) {
                opt.alpha = std::stod(argv[++i]);
                opt.alphaOverride = true;
            }
            else if (arg == "--eval-cfg" && i + 1 < argc) {
                opt.evalCfgPath = argv[++i];
                opt.evalCfgProvided = true;
            }
            else if (arg == "--route-cfg-blocks" && i + 1 < argc) {
                opt.routeCfgBlocksPath = argv[++i];
                opt.routeCfgBlocksProvided = true;
            }
            else if (arg == "-h" || arg == "--help") {
                printUsage();
                exit(0);
            }
            else {
                std::cerr << "[Warning] Unknown argument ignored: " << arg << "\n";
            }
        }
        return opt;
    }

    static bool endsWithCfg(const std::string& s) {
        if (s.size() < 4) return false;
        std::string tail = s.substr(s.size() - 4);
        for (char& c : tail) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        return tail == ".cfg";
    }

    std::string makeDefaultOutputPathFromInput(const std::string& inputPath) {
        fs::path filename = fs::path(inputPath).filename();
        filename.replace_extension(".cfg");
        return filename.string();
    }

    static std::tm getLocalTimeNow() {
        auto now = std::chrono::system_clock::now();
        std::time_t tt = std::chrono::system_clock::to_time_t(now);
        std::tm localTm{};
#ifdef _WIN32
        localtime_s(&localTm, &tt);
#else
        localtime_r(&tt, &localTm);
#endif
        return localTm;
    }

    static std::string makeAutoCfgFileName(size_t blockCount) {
        std::tm localTm = getLocalTimeNow();
        std::ostringstream oss;
        oss << std::setfill('0')
            << std::setw(2) << blockCount
            << "blk"
            << std::setw(2) << (localTm.tm_mon + 1)
            << std::setw(2) << localTm.tm_mday
            << std::setw(2) << localTm.tm_hour
            << std::setw(2) << localTm.tm_min
            << ".cfg";
        return oss.str();
    }

    std::string resolveOutputPath(const std::string& rawOutputPath, size_t blockCount) {
        fs::path p(rawOutputPath);
        bool outputIsDirectory = false;

        if (fs::exists(p) && fs::is_directory(p)) {
            outputIsDirectory = true;
        }
        else if (!endsWithCfg(p.string())) {
            outputIsDirectory = true;
        }

        if (outputIsDirectory) {
            fs::create_directories(p);
            return (p / makeAutoCfgFileName(blockCount)).string();
        }

        fs::path parent = p.parent_path();
        if (!parent.empty()) {
            fs::create_directories(parent);
        }
        return p.string();
    }

} // namespace IOUtils