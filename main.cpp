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
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

using namespace std;
namespace fs = std::filesystem;

struct Options {
    string inputPath;
    string outputPath;
    bool outputPathProvided = false;
    double alpha = 1.0;
    bool alphaOverride = false;
};

static void printUsage() {
    cerr << "Usage:\n";
    cerr << "  ./EarlyFloorplanning_with_GlobalRoute input.csv\n";
    cerr << "\n";
    cerr << "Output defaults to the input filename with .cfg extension.\n";
    cerr << "Local debug options are still accepted: -o output.cfg --alpha 0.2\n";
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
            opt.outputPathProvided = true;
        }
        else if (arg == "--alpha" && i + 1 < argc) {
            opt.alpha = stod(argv[++i]);
            opt.alphaOverride = true;
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

static string makeDefaultOutputPathFromInput(const string& inputPath) {
    fs::path filename = fs::path(inputPath).filename();
    filename.replace_extension(".cfg");
    return filename.string();
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

static double ftRateForNetsMain(const BlockSpec& spec, double ftNets) {
    if (ftNets <= 3000.0) return spec.ftRate[0];
    if (ftNets <= 6000.0) return spec.ftRate[1];
    if (ftNets <= 9000.0) return spec.ftRate[2];
    return spec.ftRate[3];
}

static double requiredSoftAreaWithFTMain(const BlockInst& b) {
    const double baseArea = max(1.0, b.spec.area);
    if (b.spec.type != BlockType::SOFT || b.ftUsed <= EPS) return baseArea;
    const double rate = ftRateForNetsMain(b.spec, b.ftUsed);
    const double delta = (b.ftUsed / CHANNEL_DENSITY) * rate / 2.0;
    const double side = sqrt(baseArea) + delta;
    return side * side;
}

static bool rectInsideOutlineMain(const Rect& r, double W, double H) {
    return r.x >= -EPS && r.y >= -EPS && rectRight(r) <= W + EPS && rectTop(r) <= H + EPS;
}

static bool overlapsAnyOtherBlockMain(const vector<BlockInst>& blocks, int id, const Rect& cand) {
    for (int i = 0; i < static_cast<int>(blocks.size()); ++i) {
        if (i == id) continue;
        if (rectOverlapAreaPositive(cand, blocks[i].rect)) return true;
    }
    return false;
}

static int ftResizeIterationLimit(const Design& design) {
    const int n = static_cast<int>(design.blockSpecs.size());
    if (n >= 45) return 0;
    if (n >= 20) return 1;
    return 3;
}

static bool resizeSoftBlocksForActualFeedthrough(Design& design) {
    bool changed = false;

    struct ResizeNeed {
        int id = -1;
        double overflow = 0.0;
    };

    vector<ResizeNeed> needs;
    for (int i = 0; i < static_cast<int>(design.blocks.size()); ++i) {
        const BlockInst& b = design.blocks[i];
        if (b.spec.type != BlockType::SOFT || b.ftUsed <= EPS) continue;

        const double currentArea = max(0.0, b.rect.w * b.rect.h);
        const double requiredArea = requiredSoftAreaWithFTMain(b);
        if (requiredArea <= currentArea + 1.0e-3) continue;
        needs.push_back({ i, requiredArea - currentArea });
    }

    sort(needs.begin(), needs.end(), [](const ResizeNeed& a, const ResizeNeed& b) {
        if (fabs(a.overflow - b.overflow) > 1.0) return a.overflow > b.overflow;
        return a.id < b.id;
        });

    for (const ResizeNeed& need : needs) {
        if (need.id < 0 || need.id >= static_cast<int>(design.blocks.size())) continue;
        BlockInst& b = design.blocks[need.id];
        if (b.spec.type != BlockType::SOFT || b.ftUsed <= EPS) continue;

        const double currentArea = max(0.0, b.rect.w * b.rect.h);
        const double requiredArea = requiredSoftAreaWithFTMain(b);
        if (requiredArea <= currentArea + 1.0e-3) continue;

        const double oldCx = rectCx(b.rect);
        const double oldCy = rectCy(b.rect);
        const double oldR = rectRight(b.rect);
        const double oldT = rectTop(b.rect);
        const double amin = max(0.05, b.spec.aspectMin);
        const double amax = max(amin, b.spec.aspectMax);
        double currentRatio = b.rect.h > EPS ? b.rect.w / b.rect.h : 1.0;
        currentRatio = max(amin, min(amax, currentRatio));

        vector<double> ratios = { currentRatio, 1.0, amin, amax, sqrt(amin * amax) };
        Rect best = b.rect;
        double bestScore = 1.0e100;

        for (double ratio : ratios) {
            ratio = max(amin, min(amax, ratio));
            Rect shape = b.rect;
            shape.w = sqrt(requiredArea * ratio);
            shape.h = requiredArea / max(EPS, shape.w);
            if (shape.w > design.outlineW + EPS || shape.h > design.outlineH + EPS) continue;

            vector<double> xs = {
                oldCx - shape.w * 0.5,
                b.rect.x,
                oldR - shape.w,
                0.0,
                design.outlineW - shape.w
            };
            vector<double> ys = {
                oldCy - shape.h * 0.5,
                b.rect.y,
                oldT - shape.h,
                0.0,
                design.outlineH - shape.h
            };

            for (double x : xs) {
                for (double y : ys) {
                    Rect cand = shape;
                    cand.x = max(0.0, min(x, design.outlineW - cand.w));
                    cand.y = max(0.0, min(y, design.outlineH - cand.h));
                    if (!rectInsideOutlineMain(cand, design.outlineW, design.outlineH)) continue;
                    if (overlapsAnyOtherBlockMain(design.blocks, need.id, cand)) continue;

                    const double move = fabs(rectCx(cand) - oldCx) + fabs(rectCy(cand) - oldCy);
                    const double grow = max(0.0, cand.w - b.rect.w) + max(0.0, cand.h - b.rect.h);
                    const double score = move + 0.01 * grow;
                    if (score < bestScore) {
                        bestScore = score;
                        best = cand;
                    }
                }
            }
        }

        if (bestScore < 1.0e90) {
            b.rect = best;
            changed = true;
        }
    }

    return changed;
}
static bool blockMovableForHotRepair(const BlockSpec& spec) {
    return spec.type != BlockType::EDGE;
}

static bool placementLegalAfterMove(const Design& design, const vector<BlockInst>& blocks) {
    for (const auto& b : blocks) {
        if (!rectInsideOutlineMain(b.rect, design.outlineW, design.outlineH)) return false;
    }
    for (int i = 0; i < static_cast<int>(blocks.size()); ++i) {
        for (int j = i + 1; j < static_cast<int>(blocks.size()); ++j) {
            if (rectOverlapAreaPositive(blocks[i].rect, blocks[j].rect)) return false;
        }
    }
    return true;
}

static bool tryMoveBlocksY(Design& design, const vector<int>& ids, double dy) {
    if (ids.empty() || fabs(dy) <= EPS) return false;
    vector<BlockInst> moved = design.blocks;
    for (int id : ids) {
        if (id < 0 || id >= static_cast<int>(moved.size())) return false;
        if (!blockMovableForHotRepair(moved[id].spec)) return false;
        moved[id].rect.y += dy;
    }
    if (!placementLegalAfterMove(design, moved)) return false;
    design.blocks.swap(moved);
    return true;
}

static bool relieveHorizontalHotChannels(Design& design) {
    struct HotChannel {
        int index = -1;
        double overflow = 0.0;
    };

    vector<HotChannel> hot;
    for (int i = 0; i < static_cast<int>(design.channels.size()); ++i) {
        const Channel& ch = design.channels[i];
        if (ch.overflow <= EPS) continue;
        const double lrCap = max(0.0, ch.rect.h * CHANNEL_DENSITY);
        const double tbCap = max(0.0, ch.rect.w * CHANNEL_DENSITY);
        const bool dominantLR = fabs(ch.capacity - lrCap) <= fabs(ch.capacity - tbCap) + 1.0e-3;
        if (!dominantLR) continue;
        hot.push_back({ i, ch.overflow });
    }

    sort(hot.begin(), hot.end(), [](const HotChannel& a, const HotChannel& b) {
        if (fabs(a.overflow - b.overflow) > 1.0) return a.overflow > b.overflow;
        return a.index < b.index;
        });

    bool changed = false;
    const int limit = min(6, static_cast<int>(hot.size()));
    for (int hi = 0; hi < limit; ++hi) {
        const Channel& ch = design.channels[hot[hi].index];
        const double x1 = ch.rect.x;
        const double x2 = rectRight(ch.rect);
        const double y1 = ch.rect.y;
        const double y2 = rectTop(ch.rect);

        vector<int> above;
        vector<int> below;
        for (int bi = 0; bi < static_cast<int>(design.blocks.size()); ++bi) {
            const BlockInst& b = design.blocks[bi];
            if (!blockMovableForHotRepair(b.spec)) continue;
            if (overlapLen(b.rect.x, rectRight(b.rect), x1, x2) <= 1.0e-4) continue;
            if (fabs(b.rect.y - y2) <= 1.0e-3) above.push_back(bi);
            if (fabs(rectTop(b.rect) - y1) <= 1.0e-3) below.push_back(bi);
        }

        const double wanted = min(260.0, max(20.0, ch.overflow / CHANNEL_DENSITY));
        vector<double> deltas = { wanted, wanted * 0.67, wanted * 0.40, 16.0, 8.0 };
        for (double d : deltas) {
            if (tryMoveBlocksY(design, above, d)) { changed = true; break; }
            if (tryMoveBlocksY(design, below, -d)) { changed = true; break; }
            if (!above.empty() && !below.empty()) {
                Design trial = design;
                if (tryMoveBlocksY(trial, above, d * 0.5) && tryMoveBlocksY(trial, below, -d * 0.5)) {
                    design = std::move(trial);
                    changed = true;
                    break;
                }
            }
        }
    }

    return changed;
}
int main(int argc, char** argv) {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    Options opt = parseArgs(argc, argv);
    if (!opt.outputPathProvided) {
        opt.outputPath = makeDefaultOutputPathFromInput(opt.inputPath);
    }

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

    if (!opt.alphaOverride) {
        opt.alpha = design.alpha;
    }

    // 讀完 parser 後才知道 block 數量，所以在這裡決定真正 output cfg 路徑。
    if (opt.outputPathProvided) {
        opt.outputPath = resolveOutputPath(opt.outputPath, design.blockSpecs.size());
    }

    auto totalPenalty = [](const EvalReport& r) {
        return r.totalChannelOverflow + r.totalFeedthroughOverflow;
        };

    auto betterEval = [&](const EvalReport& a, const EvalReport& b) {
        if (a.hasFail() != b.hasFail()) return !a.hasFail();
        const double ap = totalPenalty(a);
        const double bp = totalPenalty(b);
        if (fabs(ap - bp) > 1.0) return ap < bp;
        return a.cost < b.cost;
        };

    auto routeCandidate = [&](const Design& seed, bool ftOverflowAware, EvalReport& outRpt) {
        Design trial = seed;
        channelBuilder.build(trial);
        router.setFTOverflowCostEnabled(ftOverflowAware);
        router.run(trial);
        outRpt = evaluator.evaluate(trial, opt.alpha);
        return trial;
        };

    auto bestRoutedCandidate = [&](const Design& seed, EvalReport& outRpt) {
        EvalReport baseRpt;
        Design base = routeCandidate(seed, false, baseRpt);
        Design best = base;
        outRpt = baseRpt;
        if (baseRpt.hasFail() || totalPenalty(baseRpt) > 1.0) {
            EvalReport repairRpt;
            Design repair = routeCandidate(seed, true, repairRpt);
            if (betterEval(repairRpt, outRpt)) {
                best = std::move(repair);
                outRpt = repairRpt;
            }
        }
        return best;
        };

    // One-way architecture:
    // Parser -> Floorplanner -> ChannelBuilder -> Router -> Evaluator -> OutputWriter
    floorplanner.run(design);

    EvalReport rpt;
    design = bestRoutedCandidate(design, rpt);
    Design bestDesign = design;
    EvalReport bestRpt = rpt;

    for (int hotRepairIter = 0; hotRepairIter < 6; ++hotRepairIter) {
        Design moved = bestDesign;
        if (!relieveHorizontalHotChannels(moved)) break;
        EvalReport trialRpt;
        Design trial = bestRoutedCandidate(moved, trialRpt);
        if (betterEval(trialRpt, bestRpt)) {
            bestDesign = trial;
            bestRpt = trialRpt;
            design = bestDesign;
            rpt = bestRpt;
        }
        else {
            break;
        }
    }

    const int ftResizeLimit = ftResizeIterationLimit(bestDesign);
    for (int ftResizeIter = 0; ftResizeIter < ftResizeLimit; ++ftResizeIter) {
        Design resized = bestDesign;
        if (!resizeSoftBlocksForActualFeedthrough(resized)) break;
        EvalReport resizedRpt;
        Design trial = bestRoutedCandidate(resized, resizedRpt);
        if (betterEval(resizedRpt, bestRpt)) {
            bestDesign = trial;
            bestRpt = resizedRpt;
            design = bestDesign;
            rpt = bestRpt;
        }
        else {
            break;
        }
    }
    design = bestDesign;
    rpt = bestRpt;

    bool writeOk = writer.write(opt.outputPath, design);
    if (!writeOk) {
        cerr << "[OutputWriter] Failed to write cfg: " << opt.outputPath << "\n";
        return 1;
    }

    Logger::printFinalReport(design, rpt, opt.alpha, opt.inputPath, opt.outputPath);

    if (rpt.hasFail()) return 2;
    return 0;
}
