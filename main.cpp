#include "DataModel.hpp"
#include "Parser.hpp"
#include "Floorplanner.hpp"
#include "ChannelBuilder.hpp"
#include "Router.hpp"
#include "Evaluator.hpp"
#include "OutputWriter.hpp"
#include "Logger.hpp"
#include "RouterV2.hpp"

#include <chrono>
#include <cctype>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

using namespace std;
namespace fs = std::filesystem;

enum class RouterMode {
    Baseline,
    RouterV2
};

struct Options {
    string inputPath;
    string outputPath = "output.cfg";
    double alpha = 0.2;
    RouterMode routerMode = RouterMode::RouterV2;
};

static void printUsage() {
    cerr << "Usage:\n";
    cerr << "  ./solver input.csv -o output.cfg --alpha 0.2 --router-mode routerv2\n";
    cerr << "  ./solver input.csv -o output_folder --alpha 0.2 --router-mode baseline\n";
    cerr << "\n";
    cerr << "Router modes:\n";
    cerr << "  baseline : original floorplanner + baseline Router final output\n";
    cerr << "  routerv2 : Phase1-6 router final output (default)\n";
    cerr << "\n";
    cerr << "If -o is a folder, output filename will be generated like:\n";
    cerr << "  07blk05240238.cfg\n";
}

static string toLowerAscii(string s) {
    for (char& c : s) {
        c = static_cast<char>(tolower(static_cast<unsigned char>(c)));
    }
    return s;
}

static RouterMode parseRouterModeValue(const string& raw) {
    const string v = toLowerAscii(raw);
    if (v == "baseline" || v == "base" || v == "v1") return RouterMode::Baseline;
    if (v == "routerv2" || v == "v2" || v == "phase" || v == "phase1-6") return RouterMode::RouterV2;
    cerr << "[Warning] Unknown router mode '" << raw << "', using routerv2.\n";
    return RouterMode::RouterV2;
}

static string routerModeName(RouterMode mode) {
    return mode == RouterMode::Baseline ? "baseline" : "routerv2";
}

static Options parseArgs(int argc, char** argv) {
    Options opt;

    if (argc < 2) {
        printUsage();
        exit(1);
    }

    opt.inputPath = argv[1];

    for (int i = 2; i < argc; ++i) {
        string arg = argv[i];

        if ((arg == "-o" || arg == "--o" || arg == "--output") && i + 1 < argc) {
            opt.outputPath = argv[++i];
        }
        else if (arg == "--alpha" && i + 1 < argc) {
            opt.alpha = stod(argv[++i]);
        }
        else if ((arg == "--router-mode" || arg == "--mode" || arg == "--router") && i + 1 < argc) {
            opt.routerMode = parseRouterModeValue(argv[++i]);
        }
        else if (arg == "-h" || arg == "--help") {
            printUsage();
            exit(0);
        }
        else {
            cerr << "[Warning] Unknown argument ignored: " << arg << "\n";
        }
    }

    return opt;
}

static bool endsWithCfg(const string& s) {
    if (s.size() < 4) return false;

    string tail = s.substr(s.size() - 4);
    for (char& c : tail) {
        c = static_cast<char>(tolower(static_cast<unsigned char>(c)));
    }
    return tail == ".cfg";
}

static tm getLocalTimeNow() {
    auto now = chrono::system_clock::now();
    time_t tt = chrono::system_clock::to_time_t(now);

    tm localTm{};
#ifdef _WIN32
    localtime_s(&localTm, &tt);
#else
    localtime_r(&tt, &localTm);
#endif
    return localTm;
}

static string makeAutoCfgFileName(size_t blockCount) {
    tm localTm = getLocalTimeNow();

    ostringstream oss;
    oss << setfill('0')
        << setw(2) << blockCount
        << "blk"
        << setw(2) << (localTm.tm_mon + 1)
        << setw(2) << localTm.tm_mday
        << setw(2) << localTm.tm_hour
        << setw(2) << localTm.tm_min
        << ".cfg";
    return oss.str();
}

static string resolveOutputPath(const string& rawOutputPath, size_t blockCount) {
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
    if (!parent.empty()) fs::create_directories(parent);
    return p.string();
}

int main(int argc, char** argv) {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    Options opt = parseArgs(argc, argv);

    Design design;
    Parser parser;
    Floorplanner floorplanner;
    ChannelBuilder channelBuilder;
    Router router;
    RouterV2 routerV2;
    Evaluator evaluator;
    OutputWriter writer;

    bool parseOk = parser.read(opt.inputPath, design);
    if (!parseOk) {
        EvalReport rpt;
        rpt.formatFailed = true;
        Logger::printFinalReport(design, rpt, opt.alpha, opt.inputPath, opt.outputPath);
        return 1;
    }

    opt.outputPath = resolveOutputPath(opt.outputPath, design.blockSpecs.size());

    floorplanner.run(design);
    channelBuilder.build(design);

    cerr << "[RouterMode] " << routerModeName(opt.routerMode) << "\n";
    if (opt.routerMode == RouterMode::Baseline) {
        router.run(design);
    } else {
        RouterV2::RunResult routerV2Result = routerV2.route(design, opt.inputPath, opt.outputPath, opt.alpha);
        if (!routerV2Result.ok) {
            cerr << "[RouterV2] Failed to produce complete routerv2 final design.\n";
            return 1;
        }
    }

    EvalReport rpt = evaluator.evaluate(design, opt.alpha);

    bool writeOk = writer.write(opt.outputPath, design);
    if (!writeOk) {
        cerr << "[OutputWriter] Failed to write cfg: " << opt.outputPath << "\n";
        return 1;
    }

    Logger::printFinalReport(design, rpt, opt.alpha, opt.inputPath, opt.outputPath);
    if (!Logger::writePhase0Reports(design, rpt, opt.alpha, opt.inputPath, opt.outputPath)) {
        cerr << "[Phase0] Failed to write one or more routing truth reports.\n";
    }

    if (rpt.hasFail()) return 2;
    return 0;
}
