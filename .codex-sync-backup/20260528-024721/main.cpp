#include "DataModel.hpp"
#include "Parser.hpp"
#include "Floorplanner.hpp"
#include "ChannelBuilder.hpp"
#include "Router.hpp"
#include "Evaluator.hpp"
#include "OutputWriter.hpp"
#include "Logger.hpp"
#include "Utility.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using namespace std;
namespace fs = std::filesystem;

struct Options {
    string inputPath;
    string outputPath = "output.cfg";
    double alpha = 0.2;
};

static void printUsage() {
    cerr << "Usage:\n";
    cerr << "  ./solver input.csv -o output.cfg --alpha 0.2\n";
    cerr << "  ./solver input.csv -o output_folder --alpha 0.2\n";
    cerr << "\n";
    cerr << "If -o is a folder, output filename will be generated like:\n";
    cerr << "  07blk05240238.cfg\n";
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

        // 支援 -o / --o / --output
        if ((arg == "-o" || arg == "--o" || arg == "--output") && i + 1 < argc) {
            opt.outputPath = argv[++i];
        }
        else if (arg == "--alpha" && i + 1 < argc) {
            opt.alpha = stod(argv[++i]);
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

// 產生檔名：07blk05240238.cfg
// 格式：<兩位數block數>blk<月日時分>.cfg
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

// 如果 -o 給的是資料夾，例如 C:\problemE\problemE\result
// 就自動變成 C:\problemE\problemE\result\07blk05240238.cfg
//
// 如果 -o 給的是完整檔名，例如 C:\problemE\problemE\result\my.cfg
// 就照原本檔名輸出。
static string resolveOutputPath(const string& rawOutputPath, size_t blockCount) {
    fs::path p(rawOutputPath);

    bool outputIsDirectory = false;

    if (fs::exists(p) && fs::is_directory(p)) {
        outputIsDirectory = true;
    }
    else if (!endsWithCfg(p.string())) {
        // 沒有 .cfg 副檔名，就把它當資料夾。
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


struct ChannelDemand {
    double lr = 0.0;
    double tb = 0.0;
};

static bool isOppositeLR(int a, int b) {
    return (a == 1 && b == 3) || (a == 3 && b == 1);
}

static bool isOppositeTB(int a, int b) {
    return (a == 2 && b == 4) || (a == 4 && b == 2);
}

static bool isTurn(int a, int b) {
    return a >= 1 && a <= 4 && b >= 1 && b <= 4 && a != b &&
        !isOppositeLR(a, b) && !isOppositeTB(a, b);
}

static vector<ChannelDemand> collectChannelDemand(const Design& design) {
    unordered_map<string, int> channelId;
    for (int i = 0; i < static_cast<int>(design.channels.size()); ++i) {
        channelId[design.channels[i].name] = i;
    }

    vector<ChannelDemand> demand(design.channels.size());
    for (const auto& route : design.routes) {
        double nets = static_cast<double>(max(0, route.netCount));
        for (int i = 0; i + 1 < static_cast<int>(route.steps.size()); ++i) {
            const auto& a = route.steps[i];
            const auto& b = route.steps[i + 1];
            if (a.rectName != b.rectName) continue;

            auto it = channelId.find(a.rectName);
            if (it == channelId.end()) continue;

            ChannelDemand& d = demand[it->second];
            if (isOppositeLR(a.edge, b.edge)) d.lr += nets;
            else if (isOppositeTB(a.edge, b.edge)) d.tb += nets;
            else if (isTurn(a.edge, b.edge)) {
                d.lr += nets;
                d.tb += nets;
            }
        }
    }
    return demand;
}

static double ftRateForNetsLocal(const BlockSpec& spec, double ftNets) {
    if (ftNets <= 3000.0) return spec.ftRate[0];
    if (ftNets <= 6000.0) return spec.ftRate[1];
    if (ftNets <= 9000.0) return spec.ftRate[2];
    return spec.ftRate[3];
}

static pair<double, double> requiredSoftSizeWithFT(const BlockInst& b) {
    double ratio = b.rect.w / max(EPS, b.rect.h);
    double amin = max(0.05, b.spec.aspectMin);
    double amax = max(amin, b.spec.aspectMax);
    ratio = max(amin, min(ratio, amax));

    double baseArea = max(1.0, b.spec.area);
    double baseW = sqrt(baseArea * ratio);
    double baseH = baseArea / max(EPS, baseW);
    if (b.ftUsed <= EPS) return { baseW, baseH };

    double delta = (b.ftUsed / CHANNEL_DENSITY) * ftRateForNetsLocal(b.spec, b.ftUsed) / 2.0;
    return { baseW + delta, baseH + delta };
}

static double growOutlineW(Design& design, double need) {
    if (need <= EPS) return 0.0;
    double grow = min(need, max(0.0, design.maxOutlineW - design.outlineW));
    design.outlineW += grow;
    return grow;
}

static double growOutlineH(Design& design, double need) {
    if (need <= EPS) return 0.0;
    double grow = min(need, max(0.0, design.maxOutlineH - design.outlineH));
    design.outlineH += grow;
    return grow;
}

static bool edgeSpecHasSide(const BlockSpec& spec, char side) {
    if (spec.type != BlockType::EDGE) return false;
    for (const string& locRaw : spec.locations) {
        string loc = upperStr(locRaw);
        if (!loc.empty() && loc[0] == side) return true;
    }
    return false;
}

static bool canShiftXRightForRepair(const BlockInst& b) {
    return b.spec.type != BlockType::EDGE || edgeSpecHasSide(b.spec, 'R');
}

static bool canShiftXLeftForRepair(const BlockInst& b) {
    return b.spec.type != BlockType::EDGE || edgeSpecHasSide(b.spec, 'L');
}

static bool canShiftYUpForRepair(const BlockInst& b) {
    return b.spec.type != BlockType::EDGE || edgeSpecHasSide(b.spec, 'T');
}

static bool widenVerticalStrip(Design& design, const Channel& ch, double need) {
    double grow = growOutlineW(design, need);
    if (grow <= EPS) return false;

    double split = rectRight(ch.rect);
    for (auto& b : design.blocks) {
        if (canShiftXRightForRepair(b) && b.rect.x >= split - 1.0e-4) b.rect.x += grow;
    }
    return true;
}

static bool closeVerticalSliver(Design& design, const Channel& ch) {
    if (ch.rect.w <= EPS || ch.rect.w > 2.0) return false;

    bool changed = false;
    for (auto& b : design.blocks) {
        if (canShiftXRightForRepair(b) && fabs(rectRight(b.rect) - ch.rect.x) <= 1.0e-4) {
            b.rect.x += ch.rect.w;
            changed = true;
        }
    }
    if (changed) return true;

    double right = rectRight(ch.rect);
    for (auto& b : design.blocks) {
        if (canShiftXLeftForRepair(b) && fabs(b.rect.x - right) <= 1.0e-4) {
            b.rect.x -= ch.rect.w;
            changed = true;
        }
    }
    return changed;
}

static bool widenHorizontalGap(Design& design, const Channel& ch, double need) {
    double grow = growOutlineH(design, need);
    if (grow <= EPS) return false;

    double split = rectTop(ch.rect);
    for (auto& b : design.blocks) {
        if (canShiftYUpForRepair(b) && b.rect.y >= split - 1.0e-4) b.rect.y += grow;
    }
    return true;
}

static bool growSoftBlocksForFT(Design& design) {
    bool changed = false;

    for (int i = 0; i < static_cast<int>(design.blocks.size()); ++i) {
        BlockInst& b = design.blocks[i];
        if (b.spec.type != BlockType::SOFT || b.ftUsed <= EPS) continue;

        auto [reqW, reqH] = requiredSoftSizeWithFT(b);
        double oldRight = rectRight(b.rect);
        double oldTop = rectTop(b.rect);
        double dw = max(0.0, reqW - b.rect.w);
        double dh = max(0.0, reqH - b.rect.h);

        if (dw > EPS) {
            double grow = growOutlineW(design, dw);
            if (grow > EPS) {
                for (int j = 0; j < static_cast<int>(design.blocks.size()); ++j) {
                    if (j != i && canShiftXRightForRepair(design.blocks[j]) &&
                        design.blocks[j].rect.x >= oldRight - 1.0e-4) {
                        design.blocks[j].rect.x += grow;
                    }
                }
                b.rect.w += grow;
                changed = true;
            }
        }

        if (dh > EPS) {
            double grow = growOutlineH(design, dh);
            if (grow > EPS) {
                for (int j = 0; j < static_cast<int>(design.blocks.size()); ++j) {
                    if (j != i && canShiftYUpForRepair(design.blocks[j]) &&
                        design.blocks[j].rect.y >= oldTop - 1.0e-4) {
                        design.blocks[j].rect.y += grow;
                    }
                }
                b.rect.h += grow;
                changed = true;
            }
        }
    }

    return changed;
}

static bool repairRoutingResources(Design& design) {
    bool changed = false;
    vector<ChannelDemand> demand = collectChannelDemand(design);

    for (int i = 0; i < static_cast<int>(design.channels.size()); ++i) {
        const Channel& ch = design.channels[i];
        double requiredW = demand[i].lr / CHANNEL_DENSITY + 1.0;
        double requiredH = demand[i].tb / CHANNEL_DENSITY + 1.0;

        if (requiredW > ch.rect.w + EPS) {
            if (ch.rect.w <= 2.0) {
                changed = closeVerticalSliver(design, ch) || changed;
            }
            else {
                changed = widenVerticalStrip(design, ch, requiredW - ch.rect.w) || changed;
            }
        }
        if (requiredH > ch.rect.h + EPS) {
            changed = widenHorizontalGap(design, ch, requiredH - ch.rect.h) || changed;
        }
    }

    changed = growSoftBlocksForFT(design) || changed;
    return changed;
}

static double totalPenalty(const EvalReport& rpt) {
    return rpt.totalChannelOverflow + rpt.totalFeedthroughOverflow;
}

static bool betterNoFailReport(const EvalReport& cand, const EvalReport& best) {
    double candPenalty = totalPenalty(cand);
    double bestPenalty = totalPenalty(best);
    if (fabs(candPenalty - bestPenalty) > EPS) return candPenalty < bestPenalty;
    return cand.cost + EPS < best.cost;
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
    Evaluator evaluator;
    OutputWriter writer;

    bool parseOk = parser.read(opt.inputPath, design);
    if (!parseOk) {
        EvalReport rpt;
        rpt.formatFailed = true;
        Logger::printFinalReport(design, rpt, opt.alpha, opt.inputPath, opt.outputPath);
        return 1;
    }

    // 讀完 parser 後才知道 block 數量，所以在這裡決定真正 output cfg 路徑。
    opt.outputPath = resolveOutputPath(opt.outputPath, design.blockSpecs.size());

    // Parser -> Floorplanner -> (ChannelBuilder -> Router -> Evaluator -> repair)* -> OutputWriter
    floorplanner.run(design);

    EvalReport rpt;
    Design bestNoFailDesign;
    EvalReport bestNoFailRpt;
    bool haveNoFail = false;

    for (int pass = 0; pass < 8; ++pass) {
        channelBuilder.build(design);
        router.run(design);
        rpt = evaluator.evaluate(design, opt.alpha);

        if (!rpt.hasFail() && (!haveNoFail || betterNoFailReport(rpt, bestNoFailRpt))) {
            bestNoFailDesign = design;
            bestNoFailRpt = rpt;
            haveNoFail = true;
        }

        if (rpt.hasFail() || !rpt.hasPenalty()) break;
        if (!repairRoutingResources(design)) break;
    }

    if (haveNoFail) {
        design = bestNoFailDesign;
        rpt = bestNoFailRpt;
    }

    bool writeOk = writer.write(opt.outputPath, design);
    if (!writeOk) {
        cerr << "[OutputWriter] Failed to write cfg: " << opt.outputPath << "\n";
        return 1;
    }

    Logger::printFinalReport(design, rpt, opt.alpha, opt.inputPath, opt.outputPath);

    if (rpt.hasFail()) return 2;
    return 0;
}
