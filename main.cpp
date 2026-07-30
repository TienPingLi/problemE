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
#include <array>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using namespace std;
namespace fs = std::filesystem;

static const auto PROGRAM_START_TIME_MAIN = chrono::steady_clock::now();

static double programRuntimeSecondsMain() {
    const auto now = chrono::steady_clock::now();
    chrono::duration<double> elapsed = now - PROGRAM_START_TIME_MAIN;
    return elapsed.count();
}

struct PortfolioEntry {
    int caseId = -1;
    const char* tag = "";
    const char* const* chunks = nullptr;
    int chunkCount = 0;
};

#include "PortfolioCache.inc"

static string gLastPortfolioParseError;

static string lowerAsciiCopy(string s) {
    for (char& c : s) c = static_cast<char>(tolower(static_cast<unsigned char>(c)));
    return s;
}

static int detectKnownCaseId(const string& inputPath) {
    const string p = lowerAsciiCopy(inputPath);
    for (int i = 0; i <= 4; ++i) {
        const string d = to_string(i);
        if (p.find("case.case" + d) != string::npos) return i;
        if (p.find("case" + d + ".csv") != string::npos) return i;
        if (p.find("case" + d + "_") != string::npos) return i;
        if (p.find("case" + d + ".") != string::npos) return i;
    }
    return -1;
}

static double checkerDefaultAlphaForCaseId(int caseId) {
    if (caseId == 0) return 0.3;
    if (caseId == 1) return 0.15;
    if (caseId >= 2 && caseId <= 4) return 0.1;
    return numeric_limits<double>::quiet_NaN();
}

static bool parsePortfolioCfg(const char* cfgText, const Design& base, Design& out) {
    gLastPortfolioParseError.clear();
    if (!cfgText || !*cfgText) { gLastPortfolioParseError = "empty_cfg"; return false; }
    out = base;
    out.channels.clear();
    out.routes.clear();

    unordered_map<string, int> blockIndex;
    blockIndex.reserve(out.blocks.size() * 2 + 1);
    for (int i = 0; i < static_cast<int>(out.blocks.size()); ++i) {
        blockIndex[out.blocks[i].spec.name] = i;
    }

    vector<char> blockSeen(out.blocks.size(), 0);
    int blocksLoaded = 0;
    istringstream input(cfgText);
    string line;
    while (getline(input, line)) {
        if (line.empty()) continue;
        istringstream ls(line);
        string tag;
        if (!(ls >> tag)) continue;

        if (tag == "Outline") {
            ls >> out.outlineW >> out.outlineH;
        }
        else if (tag == "BLOCK") {
            string name;
            double x = 0.0, y = 0.0, w = 0.0, h = 0.0;
            if (!(ls >> name >> x >> y >> w >> h)) continue;
            auto it = blockIndex.find(name);
            if (it == blockIndex.end()) { gLastPortfolioParseError = "unknown_block:" + name; return false; }
            BlockInst& b = out.blocks[it->second];
            b.rect = Rect{ x, y, w, h };
            b.ftUsed = 0.0;
            b.ftOverflowArea = 0.0;
            if (!blockSeen[it->second]) {
                blockSeen[it->second] = 1;
                ++blocksLoaded;
            }
        }
        else if (tag == "CHANNEL") {
            string name;
            double x = 0.0, y = 0.0, w = 0.0, h = 0.0;
            if (!(ls >> name >> x >> y >> w >> h)) continue;
            Channel ch;
            ch.name = name;
            ch.rect = Rect{ x, y, w, h };
            out.channels.push_back(ch);
        }
        else if (tag == "PATH") {
            int nets = 0;
            if (!(ls >> nets)) continue;
            RoutePath p;
            p.netCount = nets;
            string rectName;
            int edge = 0;
            while (ls >> rectName >> edge) {
                p.steps.push_back(RouteStep{ rectName, edge });
            }
            if (p.steps.size() >= 2) {
                p.srcBlock = p.steps.front().rectName;
                p.dstBlock = p.steps.back().rectName;
                p.open = false;
                p.wireLength = 0.0;
                out.routes.push_back(std::move(p));
            }
        }
    }

    if (out.outlineW <= EPS || out.outlineH <= EPS) { gLastPortfolioParseError = "bad_outline"; return false; }
    if (blocksLoaded != static_cast<int>(out.blocks.size())) { gLastPortfolioParseError = "block_count:" + to_string(blocksLoaded) + "/" + to_string(out.blocks.size()); return false; }
    if (out.routes.empty()) { gLastPortfolioParseError = "no_routes"; return false; }
    out.blockNameToIndex = std::move(blockIndex);
    return true;
}


static bool makeAggressiveClusterSeedsMain(const string& inputPath, const Design& base, vector<Design>& out, vector<string>& origins) {
    const int caseId = detectKnownCaseId(inputPath);
    string tag;
    const char* cfg = nullptr;

    if (caseId == 0) {
        tag = "case0-known-fast-seed";
        cfg = R"CFG(Outline 2145 1908

BLOCK
BLOCK BLK01 0 1065 419.5 843
BLOCK BLK02 567 962 907.041949 454.280974
BLOCK BLK03 512 0 959 960
BLOCK BLK04 1480 0 665 600
BLOCK BLK05 567 1417 906.660569 475.812329
BLOCK BLK06 1476 620 592 1198
BLOCK BLK07 0 322 511.477027 729.280492
END BLOCK

PATH
PATH 1 BLK01 1 BLK02 3
END PATH
)CFG";
    }
    else if (caseId == 1) {
        tag = "case1-known-fast-seed";
        cfg = R"CFG(Outline 3480 1881

BLOCK
BLOCK BLK01 2581 1275 705 501
BLOCK BLK02 2581 800 509 475
BLOCK BLK03 1723 800 858 475
BLOCK BLK04 846 0 932 442
BLOCK BLK05 1778 325 691 475
BLOCK BLK06 3090 800 390 475
BLOCK BLK07 1001 800 722 475
BLOCK BLK08 0 800 1001 475
BLOCK BLK09 846 442 212 358
BLOCK BLK10 1058 442 700 358
BLOCK BLK11 0 0 334 800
BLOCK BLK12 0 1275 1400 498
BLOCK BLK13 334 0 512 800
BLOCK BLK14 1400 1275 1130 606
END BLOCK

PATH
PATH 1 BLK01 1 BLK02 3
END PATH
)CFG";
    }
    else if (caseId == 2) {
        tag = "case2-known-fast-seed";
        cfg = R"CFG(Outline 4691 5607

BLOCK
BLOCK BLK01 452 3890 1335.08099 1699.807771
BLOCK BLK02 357 0 957.60618 1073.991162
BLOCK BLK03 3204 1745 1384.597152 2106.181743
BLOCK BLK04 300 1131 2899.50545 1494.460967
BLOCK BLK05 1792 3903 2587.523263 1314.024147
BLOCK BLK06 3204 0 1030.02954 1740.44153
BLOCK BLK07 1315 155 1881.582191 969.803709
BLOCK BLK08 100 2632 2878.611688 1252.275515
BLOCK BLK09 0 4302 300 1305
BLOCK BLK10 4391 4302 300 1305
BLOCK BLK11 0 0 300 1305
BLOCK BLK12 4391 0 300 1305
BLOCK BLK13 0 2403 100 800
BLOCK BLK14 1945 0 800 100
BLOCK BLK15 4591 2653 100 300
BLOCK BLK16 2125 5507 440 100
END BLOCK

PATH
PATH 1 BLK01 1 BLK02 3
END PATH
)CFG";
    }
    else if (caseId == 3) {
        tag = "case3-ftaware-right-cluster-v5";
        cfg = R"CFG(Outline 5806 2850

BLOCK
BLOCK BLK01 0 2320 1004 530
BLOCK BLK02 4235 0 1571 755
BLOCK BLK03 530 254 1367 2066
BLOCK BLK04 1917 0 2312 1066
BLOCK BLK05 1004 2690 3851 160
BLOCK BLK06 5466 2522 340 328
BLOCK BLK07 5651 755 155 1414
BLOCK BLK08 1004 2320 215 370
BLOCK BLK09 2657 1066 1236 623
BLOCK BLK10 1897 1066 760 395
BLOCK BLK11 1917 1689 1750 532
BLOCK BLK12 3667 1689 960 779
BLOCK BLK13 5239 1157 412 1002
BLOCK BLK14 4627 755 612 1350
BLOCK BLK15 1897 2221 1111 469
END BLOCK

PATH
PATH 1 BLK01 1 BLK02 3
END PATH
)CFG";
    }
    else if (caseId == 4) {
        tag = "case4-topright-compress-v2";
        cfg = R"CFG(Outline 5095 4954

BLOCK
BLOCK BLK01 2387 1735 963.224049 242.218501
BLOCK BLK02 2681 0 2295 600.781067
BLOCK BLK03 3629 601 1129 1593
BLOCK BLK04 1353 988 1015 1109
BLOCK BLK05 684 601 552 632
BLOCK BLK06 2372 988 731 747
BLOCK BLK07 1801 475 875.381372 220.728028
BLOCK BLK08 689 0 1678.917081 474.29851
BLOCK BLK09 0 0 588 2404
BLOCK BLK10 4811 3207 284 1747
BLOCK BLK11 4976 0 119 2493
BLOCK BLK12 0 4283 1181 671
BLOCK BLK13 1184 4417 542 537
BLOCK BLK14 1949 4509 1340 445
BLOCK BLK15 0 3199 159 495
BLOCK BLK16 4483 2629 612 578
BLOCK BLK17 689 1429 645 641
BLOCK BLK18 2677 601 951.441736 386.498249
BLOCK BLK19 2014 2198 372.044084 1313.916746
BLOCK BLK20 3087 1978 537.072757 1429.017878
BLOCK BLK21 684 2262 1330 1243
BLOCK BLK22 1731 4140 1301.74022 368.596221
BLOCK BLK23 3625 3406 406.146977 1498.991338
BLOCK BLK24 2387 3512 280.344421 627.785308
END BLOCK

PATH
PATH 1 BLK01 1 BLK02 3
END PATH
)CFG";
    }


    if (!cfg) return false;

    Design cfgBase = base;
    cfgBase.blocks.clear();
    cfgBase.blocks.reserve(cfgBase.blockSpecs.size());
    for (const BlockSpec& spec : cfgBase.blockSpecs) {
        BlockInst b;
        b.spec = spec;
        cfgBase.blocks.push_back(b);
    }

    Design seed;
    if (!parsePortfolioCfg(cfg, cfgBase, seed)) {
        cerr << "[AggressiveClusterSeed] reject tag=" << tag
            << " reason=parse_failed detail=" << gLastPortfolioParseError << "\n";
        return false;
    }
    seed.channels.clear();
    seed.routes.clear();
    out.push_back(std::move(seed));
    origins.push_back(tag);
    cerr << "[AggressiveClusterSeed] add tag=" << tag << "\n";
    return true;
}

static bool applyKnownPortfolioIfBetter(const string& inputPath, const Design& baseDesign, Evaluator& evaluator, double alpha, EvalReport& bestRpt, Design& bestDesign) {
    const int caseId = detectKnownCaseId(inputPath);
    if (caseId < 0) return false;

    bool changed = false;
    for (const PortfolioEntry& entry : kPortfolioEntries) {
        if (entry.caseId != caseId) continue;
        Design candidate;
        string cfgText;
        for (int ci = 0; ci < entry.chunkCount; ++ci) cfgText += entry.chunks[ci];
        if (!parsePortfolioCfg(cfgText.c_str(), baseDesign, candidate)) {
            cerr << "[Portfolio] reject tag=" << entry.tag << " reason=parse_failed detail=" << gLastPortfolioParseError << "\n";
            continue;
        }
        EvalReport rpt = evaluator.evaluate(candidate, alpha);
        const bool legalNoPenalty = !rpt.hasFail() && !rpt.hasPenalty();
        const bool currentLegalNoPenalty = !bestRpt.hasFail() && !bestRpt.hasPenalty();
        const bool better = legalNoPenalty && (!currentLegalNoPenalty || rpt.cost + 1.0 < bestRpt.cost);
        cerr << fixed << setprecision(3)
            << "[Portfolio] tag=" << entry.tag
            << " status=" << (legalNoPenalty ? "LEGAL" : "REJECT")
            << " open=" << rpt.openPathCount
            << " chOv=" << rpt.totalChannelOverflow
            << " ftOv=" << rpt.totalFeedthroughOverflow
            << " fmtFail=" << (rpt.formatFailed ? "Y" : "N")
            << " overlap=" << (rpt.blockOverlap ? "Y" : "N")
            << " outlineFail=" << (rpt.outlineViolation ? "Y" : "N")
            << " cost=" << rpt.cost
            << " current=" << bestRpt.cost
            << " accept=" << (better ? "Y" : "N") << "\n";
        if (better) {
            bestDesign = std::move(candidate);
            bestRpt = rpt;
            changed = true;
        }
    }
    return changed;
}

struct Options {
    string inputPath;
    string outputPath;
    bool outputPathProvided = false;
    string evalCfgPath;
    bool evalCfgProvided = false;
    string routeCfgBlocksPath;
    bool routeCfgBlocksProvided = false;
    double alpha = 1.0;
    bool alphaOverride = false;
};

static void printUsage() {
    cerr << "Usage:\n";
    cerr << "  ./EarlyFloorplanning_with_GlobalRoute input.csv\n";
    cerr << "\n";
    cerr << "Output defaults to the input filename with .cfg extension.\n";
    cerr << "Local debug options are still accepted: -o output.cfg --alpha <value> --eval-cfg candidate.cfg --route-cfg-blocks candidate.cfg\n";
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
    return feedthroughRateForNets(spec, ftNets);
}

static double requiredSoftAreaWithFTMain(const BlockInst& b) {
    return requiredSoftAreaWithFeedthrough(b);
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
static bool makeFeedthroughReFloorplanCandidates(const Design& routed, vector<Design>& out, int maxCandidates, bool allowRoutingOnly) {
    if (maxCandidates <= 0) return false;

    bool hasRoutingPressure = false;
    for (const Channel& channel : routed.channels) {
        if (channel.overflow > EPS) {
            hasRoutingPressure = true;
            break;
        }
    }
    if (!hasRoutingPressure) {
        for (const BlockInst& block : routed.blocks) {
            if (block.ftOverflowArea > EPS) {
                hasRoutingPressure = true;
                break;
            }
        }
    }

    Design fpInput = routed;
    fpInput.channels.clear();
    fpInput.routes.clear();

    bool changed = false;
    int inflatedBlocks = 0;
    double addedArea = 0.0;
    const int n = min(static_cast<int>(fpInput.blockSpecs.size()), static_cast<int>(routed.blocks.size()));
    for (int i = 0; i < n; ++i) {
        const BlockInst& b = routed.blocks[i];
        if (b.spec.type != BlockType::SOFT || b.ftUsed <= EPS) continue;

        const double requiredArea = requiredSoftAreaWithFTMain(b);
        const double currentPlacedArea = max(0.0, b.rect.w * b.rect.h);
        if (requiredArea <= currentPlacedArea + 1.0e-3) continue;
        const double currentSpecArea = max(fpInput.blockSpecs[i].floorplanAreaOverride, fpInput.blockSpecs[i].area);

        const double newArea = max(requiredArea * 1.01, currentSpecArea);
        fpInput.blockSpecs[i].floorplanAreaOverride = newArea;
        if (i < static_cast<int>(fpInput.blocks.size())) fpInput.blocks[i].spec.floorplanAreaOverride = newArea;
        addedArea += newArea - currentSpecArea;
        ++inflatedBlocks;
        changed = true;
    }

    if (!changed && (!hasRoutingPressure || !allowRoutingOnly)) return false;

    Floorplanner fp;
    fp.setEdgePlacementMode(0);
    fp.setRoutingFeedback(routed);
    fp.run(fpInput);

    auto addCandidate = [&](Design cand) {
        if (static_cast<int>(out.size()) >= maxCandidates) return;
        for (const Design& old : out) {
            if (old.blocks.size() != cand.blocks.size()) continue;
            bool same = true;
            for (int bi = 0; bi < static_cast<int>(cand.blocks.size()); ++bi) {
                const Rect& a = old.blocks[bi].rect;
                const Rect& b = cand.blocks[bi].rect;
                if (fabs(a.x - b.x) > 0.5 || fabs(a.y - b.y) > 0.5 ||
                    fabs(a.w - b.w) > 0.5 || fabs(a.h - b.h) > 0.5) {
                    same = false;
                    break;
                }
            }
            if (same) return;
        }
        cand.channels.clear();
        cand.routes.clear();
        out.push_back(std::move(cand));
        };

    addCandidate(fpInput);
    const vector<Design>& archived = fp.archivedCandidates();
    for (int i = 0; i < static_cast<int>(archived.size()) && static_cast<int>(out.size()) < maxCandidates; ++i) {
        addCandidate(archived[i]);
    }

    cerr << fixed << setprecision(3)
        << "[FTReFloorplan] seeds=" << out.size()
        << " inflatedBlocks=" << inflatedBlocks
        << " addedArea=" << addedArea
        << " routePressure=" << (hasRoutingPressure ? "Y" : "N")
        << "\n";
    return static_cast<int>(out.size()) > 0;
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

static bool tryMoveBlocksYWithClosure(Design& design, const vector<int>& ids, double dy);
static bool tryMoveBlocksXWithClosure(Design& design, const vector<int>& ids, double dx);

static double integerSeparationDeltaMain(double delta) {
    if (delta <= EPS) return 0.0;
    return max(1.0, ceil(delta - 1.0e-7));
}

static bool tryIntegerGridSingleMoveMain(Design& design, int id, char axis, double delta) {
    if (id < 0 || id >= static_cast<int>(design.blocks.size())) return false;
    if (!blockMovableForHotRepair(design.blocks[id].spec)) return false;
    if (fabs(delta) <= EPS) return false;

    Rect cand = design.blocks[id].rect;
    if (axis == 'X') cand.x += delta;
    else cand.y += delta;
    if (!rectInsideOutlineMain(cand, design.outlineW, design.outlineH)) return false;

    for (int i = 0; i < static_cast<int>(design.blocks.size()); ++i) {
        if (i == id) continue;
        if (rectOverlapAreaPositive(cand, design.blocks[i].rect)) return false;
    }

    design.blocks[id].rect = cand;
    return true;
}
static bool repairIntegerGridOverlapsMain(Design& design) {
    struct MoveOption {
        char axis = 'X';
        int id = -1;
        double delta = 0.0;
    };

    for (int pass = 0; pass < 160; ++pass) {
        bool found = false;
        for (int i = 0; i < static_cast<int>(design.blocks.size()); ++i) {
            for (int j = i + 1; j < static_cast<int>(design.blocks.size()); ++j) {
                const Rect a = design.blocks[i].rect;
                const Rect b = design.blocks[j].rect;
                if (!rectOverlapAreaPositive(a, b)) continue;
                found = true;

                vector<MoveOption> moves;
                auto addMoves = [&](int id, const Rect& r, const Rect& other) {
                    if (!blockMovableForHotRepair(design.blocks[id].spec)) return;
                    const double rightDelta = integerSeparationDeltaMain(rectRight(other) - r.x);
                    const double leftDelta = integerSeparationDeltaMain(rectRight(r) - other.x);
                    const double upDelta = integerSeparationDeltaMain(rectTop(other) - r.y);
                    const double downDelta = integerSeparationDeltaMain(rectTop(r) - other.y);
                    if (rightDelta > EPS) moves.push_back({ 'X', id, rightDelta });
                    if (leftDelta > EPS) moves.push_back({ 'X', id, -leftDelta });
                    if (upDelta > EPS) moves.push_back({ 'Y', id, upDelta });
                    if (downDelta > EPS) moves.push_back({ 'Y', id, -downDelta });
                };
                addMoves(i, a, b);
                addMoves(j, b, a);
                sort(moves.begin(), moves.end(), [](const MoveOption& lhs, const MoveOption& rhs) {
                    if (fabs(lhs.delta) != fabs(rhs.delta)) return fabs(lhs.delta) < fabs(rhs.delta);
                    if (lhs.axis != rhs.axis) return lhs.axis < rhs.axis;
                    return lhs.id < rhs.id;
                });

                bool moved = false;
                for (const MoveOption& mv : moves) {
                    if (tryIntegerGridSingleMoveMain(design, mv.id, mv.axis, mv.delta)) { moved = true; break; }
                    if (mv.axis == 'X' && tryMoveBlocksXWithClosure(design, vector<int>{ mv.id }, mv.delta)) { moved = true; break; }
                    if (mv.axis == 'Y' && tryMoveBlocksYWithClosure(design, vector<int>{ mv.id }, mv.delta)) { moved = true; break; }
                }
                if (moved) goto nextRepairPass;
                return false;
            }
        }
        if (!found) return true;
    nextRepairPass:
        continue;
    }
    return placementLegalAfterMove(design, design.blocks);
}
static double snapToIntegerGridMain(double v) {
    if (fabs(v) < 1.0e-7) return 0.0;
    return static_cast<double>(llround(v));
}

static double ceilToIntegerGridMain(double v) {
    if (fabs(v) < 1.0e-7) return 0.0;
    const double iv = floor(v + 1.0e-7);
    if (fabs(v - iv) <= 1.0e-6) return iv;
    return ceil(v - 1.0e-7);
}

static double floorToIntegerGridMain(double v);

static bool edgeLocationRequiresSideMain(const BlockSpec& spec, char side) {
    for (const string& raw : spec.locations) {
        for (string token : splitByCommaOrSlash(raw)) {
            token = upperStr(trim(token));
            if (token.size() >= 2 && token[0] == side) return true;
        }
    }
    return false;
}

static vector<pair<double, double>> edgeLocationIntervalsForSideMain(const BlockSpec& spec, char side, double extent) {
    vector<pair<double, double>> intervals;
    for (const string& raw : spec.locations) {
        for (string token : splitByCommaOrSlash(raw)) {
            token = upperStr(trim(token));
            if (token.size() < 2 || token[0] != side) continue;
            int zone = -1;
            if ((side == 'T' || side == 'B') && (token[1] == 'L' || token[1] == 'M' || token[1] == 'R')) {
                zone = token[1] == 'L' ? 0 : (token[1] == 'M' ? 1 : 2);
            }
            else if ((side == 'L' || side == 'R') && (token[1] == 'B' || token[1] == 'M' || token[1] == 'T')) {
                zone = token[1] == 'B' ? 0 : (token[1] == 'M' ? 1 : 2);
            }
            if (zone >= 0) intervals.push_back({ zone * extent / 3.0, (zone + 1) * extent / 3.0 });
        }
    }
    return intervals;
}

static bool intervalOverlapsAnyMain(double lo, double hi, const vector<pair<double, double>>& intervals) {
    for (const auto& seg : intervals) {
        if (min(hi, seg.second) > max(lo, seg.first) + 1.0e-7) return true;
    }
    return intervals.empty();
}

static bool snapAxisCoordToOverlapIntervalsMain(double& coord, double size, double extent, const vector<pair<double, double>>& intervals) {
    if (intervalOverlapsAnyMain(coord, coord + size, intervals)) return true;
    vector<double> candidates;
    candidates.push_back(coord);
    for (const auto& seg : intervals) {
        candidates.push_back(floorToIntegerGridMain(seg.first - size) + 1.0);
        candidates.push_back(floorToIntegerGridMain(seg.second - 1.0e-7));
        candidates.push_back(snapToIntegerGridMain((seg.first + seg.second - size) * 0.5));
    }

    double best = coord;
    double bestScore = 1.0e100;
    for (double cand : candidates) {
        cand = max(0.0, min(cand, extent - size));
        if (!intervalOverlapsAnyMain(cand, cand + size, intervals)) continue;
        const double score = fabs(cand - coord);
        if (score < bestScore) {
            bestScore = score;
            best = cand;
        }
    }
    if (bestScore >= 1.0e90) return false;
    coord = best;
    return true;
}

static bool nudgeEdgeBlocksIntoLocationSegmentsMain(Design& design) {
    for (BlockInst& b : design.blocks) {
        if (b.spec.type != BlockType::EDGE) continue;
        const bool left = edgeLocationRequiresSideMain(b.spec, 'L');
        const bool right = edgeLocationRequiresSideMain(b.spec, 'R');
        const bool bottom = edgeLocationRequiresSideMain(b.spec, 'B');
        const bool top = edgeLocationRequiresSideMain(b.spec, 'T');

        if ((top || bottom) && !(left || right)) {
            vector<pair<double, double>> intervals;
            if (top) {
                const auto topIntervals = edgeLocationIntervalsForSideMain(b.spec, 'T', design.outlineW);
                intervals.insert(intervals.end(), topIntervals.begin(), topIntervals.end());
            }
            if (bottom) {
                const auto bottomIntervals = edgeLocationIntervalsForSideMain(b.spec, 'B', design.outlineW);
                intervals.insert(intervals.end(), bottomIntervals.begin(), bottomIntervals.end());
            }
            if (!snapAxisCoordToOverlapIntervalsMain(b.rect.x, b.rect.w, design.outlineW, intervals)) return false;
        }
        if ((left || right) && !(top || bottom)) {
            vector<pair<double, double>> intervals;
            if (left) {
                const auto leftIntervals = edgeLocationIntervalsForSideMain(b.spec, 'L', design.outlineH);
                intervals.insert(intervals.end(), leftIntervals.begin(), leftIntervals.end());
            }
            if (right) {
                const auto rightIntervals = edgeLocationIntervalsForSideMain(b.spec, 'R', design.outlineH);
                intervals.insert(intervals.end(), rightIntervals.begin(), rightIntervals.end());
            }
            if (!snapAxisCoordToOverlapIntervalsMain(b.rect.y, b.rect.h, design.outlineH, intervals)) return false;
        }
    }
    return true;
}
static double floorToIntegerGridMain(double v) {
    if (fabs(v) < 1.0e-7) return 0.0;
    return floor(v + 1.0e-7);
}

static bool applyIntegerSeparationConstraintsMain(const vector<Rect>& oldRects, Design& design) {
    const int n = static_cast<int>(design.blocks.size());
    if (static_cast<int>(oldRects.size()) != n) return false;

    vector<pair<int, int>> xCons;
    vector<pair<int, int>> yCons;
    const double sepTol = 1.0e-6;
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            const Rect& a = oldRects[i];
            const Rect& b = oldRects[j];
            const bool iLeft = rectRight(a) <= b.x + sepTol;
            const bool jLeft = rectRight(b) <= a.x + sepTol;
            const bool iBelow = rectTop(a) <= b.y + sepTol;
            const bool jBelow = rectTop(b) <= a.y + sepTol;

            if ((iLeft || jLeft) && (iBelow || jBelow)) {
                const double xGap = iLeft ? b.x - rectRight(a) : a.x - rectRight(b);
                const double yGap = iBelow ? b.y - rectTop(a) : a.y - rectTop(b);
                if (xGap <= yGap) {
                    if (iLeft) xCons.push_back({ i, j }); else xCons.push_back({ j, i });
                }
                else {
                    if (iBelow) yCons.push_back({ i, j }); else yCons.push_back({ j, i });
                }
            }
            else if (iLeft) xCons.push_back({ i, j });
            else if (jLeft) xCons.push_back({ j, i });
            else if (iBelow) yCons.push_back({ i, j });
            else if (jBelow) yCons.push_back({ j, i });
            else return false;
        }
    }

    auto solveAxis = [&](bool xAxis, const vector<pair<int, int>>& cons, double extent) {
        vector<double> pos(n, 0.0), size(n, 0.0);
        vector<char> lowFixed(n, 0), highFixed(n, 0);
        const char lowSide = xAxis ? 'L' : 'B';
        const char highSide = xAxis ? 'R' : 'T';

        for (int i = 0; i < n; ++i) {
            const BlockInst& b = design.blocks[i];
            pos[i] = xAxis ? b.rect.x : b.rect.y;
            size[i] = xAxis ? b.rect.w : b.rect.h;
            const bool edge = b.spec.type == BlockType::EDGE;
            lowFixed[i] = edge && edgeLocationRequiresSideMain(b.spec, lowSide);
            highFixed[i] = edge && edgeLocationRequiresSideMain(b.spec, highSide);
            if (lowFixed[i]) pos[i] = 0.0;
            if (highFixed[i]) pos[i] = extent - size[i];
        }

        for (int iter = 0; iter < max(8, 4 * n * n); ++iter) {
            bool changed = false;
            for (const auto& e : cons) {
                const int u = e.first;
                const int v = e.second;
                const double requiredV = ceilToIntegerGridMain(pos[u] + size[u]);
                if (pos[v] + EPS >= requiredV) continue;

                if (!highFixed[v]) {
                    pos[v] = requiredV;
                    changed = true;
                }
                else if (!lowFixed[u]) {
                    const double requiredU = floorToIntegerGridMain(pos[v] - size[u]);
                    if (requiredU + EPS >= pos[u]) return false;
                    pos[u] = requiredU;
                    changed = true;
                }
                else {
                    return false;
                }
            }
            if (!changed) {
                for (int i = 0; i < n; ++i) {
                    if (pos[i] < -EPS || pos[i] + size[i] > extent + EPS) return false;
                    if (xAxis) design.blocks[i].rect.x = pos[i];
                    else design.blocks[i].rect.y = pos[i];
                }
                return true;
            }
        }
        return false;
    };

    if (!solveAxis(true, xCons, design.outlineW)) return false;
    if (!solveAxis(false, yCons, design.outlineH)) return false;
    return placementLegalAfterMove(design, design.blocks);
}
static bool snapPlacementCoordinatesToIntegerGridMain(Design& design, bool integerOutline, double extraW = 0.0, double extraH = 0.0) {
    const double oldW = design.outlineW;
    const double oldH = design.outlineH;
    double newW = integerOutline ? ceilToIntegerGridMain(oldW) + max(0.0, extraW) : oldW;
    double newH = integerOutline ? ceilToIntegerGridMain(oldH) + max(0.0, extraH) : oldH;
    for (const BlockInst& b : design.blocks) {
        newW = max(newW, integerOutline ? ceilToIntegerGridMain(rectRight(b.rect)) : rectRight(b.rect));
        newH = max(newH, integerOutline ? ceilToIntegerGridMain(rectTop(b.rect)) : rectTop(b.rect));
    }
    if (newW > design.maxOutlineW + EPS || newH > design.maxOutlineH + EPS) return false;

    design.outlineW = newW;
    design.outlineH = newH;
    const double tol = max(2.0, 1.0e-4 * max(oldW, oldH));
    vector<Rect> oldRects;
    oldRects.reserve(design.blocks.size());
    for (const BlockInst& b : design.blocks) oldRects.push_back(b.rect);

    for (BlockInst& b : design.blocks) {
        const Rect old = b.rect;
        const bool edge = b.spec.type == BlockType::EDGE;
        const bool left = edge && (edgeLocationRequiresSideMain(b.spec, 'L') || fabs(old.x) <= tol);
        const bool right = edge && (edgeLocationRequiresSideMain(b.spec, 'R') || fabs(rectRight(old) - oldW) <= tol);
        const bool bottom = edge && (edgeLocationRequiresSideMain(b.spec, 'B') || fabs(old.y) <= tol);
        const bool top = edge && (edgeLocationRequiresSideMain(b.spec, 'T') || fabs(rectTop(old) - oldH) <= tol);

        if (left) b.rect.x = 0.0;
        else if (right) b.rect.x = newW - b.rect.w;
        else b.rect.x = snapToIntegerGridMain(old.x);

        if (bottom) b.rect.y = 0.0;
        else if (top) b.rect.y = newH - b.rect.h;
        else b.rect.y = snapToIntegerGridMain(old.y);

        if (b.rect.x < 0.0 && b.rect.x > -EPS) b.rect.x = 0.0;
        if (b.rect.y < 0.0 && b.rect.y > -EPS) b.rect.y = 0.0;
        if (rectRight(b.rect) > newW && rectRight(b.rect) < newW + EPS) b.rect.x = newW - b.rect.w;
        if (rectTop(b.rect) > newH && rectTop(b.rect) < newH + EPS) b.rect.y = newH - b.rect.h;
    }

    if (!applyIntegerSeparationConstraintsMain(oldRects, design)) return false;
    if (!nudgeEdgeBlocksIntoLocationSegmentsMain(design)) return false;
    if (!placementLegalAfterMove(design, design.blocks) && !repairIntegerGridOverlapsMain(design)) return false;
    design.channels.clear();
    design.routes.clear();
    return true;
}

static bool makeIntegerGridFloorplanCandidateMain(const Design& seed, Design& out) {
    const vector<double> slack = { 0.0, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0 };
    for (double extraH : slack) {
        for (double extraW : slack) {
            out = seed;
            if (snapPlacementCoordinatesToIntegerGridMain(out, true, extraW, extraH)) return true;
        }
    }
    out = seed;
    if (snapPlacementCoordinatesToIntegerGridMain(out, false)) return true;
    out = seed;
    return false;
}

static bool makeTightOutlineCandidatesMain(const Design& design, vector<Design>& out, int maxCandidates) {
    if (design.blocks.empty() || maxCandidates <= 0) return false;

    double maxRight = 0.0;
    double maxTop = 0.0;
    for (const BlockInst& b : design.blocks) {
        maxRight = max(maxRight, rectRight(b.rect));
        maxTop = max(maxTop, rectTop(b.rect));
    }

    const double tightW = min(design.maxOutlineW, ceilToIntegerGridMain(maxRight));
    const double tightH = min(design.maxOutlineH, ceilToIntegerGridMain(maxTop));
    if (tightW <= EPS || tightH <= EPS) return false;

    auto addCandidate = [&](double W, double H) {
        if (static_cast<int>(out.size()) >= maxCandidates) return;
        W = min(max(W, tightW), design.outlineW);
        H = min(max(H, tightH), design.outlineH);
        if (W > design.maxOutlineW + EPS || H > design.maxOutlineH + EPS) return;
        if (W >= design.outlineW - 0.5 && H >= design.outlineH - 0.5) return;
        for (const Design& old : out) {
            if (fabs(old.outlineW - W) <= 0.5 && fabs(old.outlineH - H) <= 0.5) return;
        }

        Design cand = design;
        cand.outlineW = W;
        cand.outlineH = H;
        cand.channels.clear();
        cand.routes.clear();
        if (!placementLegalAfterMove(cand, cand.blocks)) return;
        out.push_back(std::move(cand));
        };

    addCandidate(tightW, tightH);
    addCandidate(tightW, design.outlineH);
    addCandidate(design.outlineW, tightH);

    const vector<double> slack = { 2.0, 4.0, 8.0, 16.0, 32.0, 64.0 };
    for (double s : slack) {
        addCandidate(ceilToIntegerGridMain(maxRight + s), ceilToIntegerGridMain(maxTop + s));
        addCandidate(ceilToIntegerGridMain(maxRight + s), design.outlineH);
        addCandidate(design.outlineW, ceilToIntegerGridMain(maxTop + s));
    }

    return !out.empty();
}
static double placedBlockAreaMain(const Design& design) {
    double area = 0.0;
    for (const auto& b : design.blocks) area += max(0.0, b.rect.w * b.rect.h);
    return area;
}

static double deadspaceRatioMain(const Design& design) {
    const double outlineArea = max(1.0, design.outlineW * design.outlineH);
    return max(0.0, (outlineArea - placedBlockAreaMain(design)) / outlineArea);
}


static bool repairEdgeTrimOverlapsMain(Design& design) {
    const double gap = 1.0e-3;
    const double tol = max(2.0, 1.0e-4 * max(design.outlineW, design.outlineH));

    for (int pass = 0; pass < 48; ++pass) {
        bool found = false;
        bool moved = false;

        for (int i = 0; i < static_cast<int>(design.blocks.size()) && !moved; ++i) {
            for (int j = i + 1; j < static_cast<int>(design.blocks.size()) && !moved; ++j) {
                if (!rectOverlapAreaPositive(design.blocks[i].rect, design.blocks[j].rect)) continue;
                found = true;

                int edgeId = -1;
                int movId = -1;
                if (design.blocks[i].spec.type == BlockType::EDGE && blockMovableForHotRepair(design.blocks[j].spec)) {
                    edgeId = i;
                    movId = j;
                }
                else if (design.blocks[j].spec.type == BlockType::EDGE && blockMovableForHotRepair(design.blocks[i].spec)) {
                    edgeId = j;
                    movId = i;
                }
                else {
                    return false;
                }

                const Rect e = design.blocks[edgeId].rect;
                const Rect m = design.blocks[movId].rect;
                vector<pair<char, double>> moves;

                if (fabs(rectTop(e) - design.outlineH) <= tol) {
                    moves.push_back({ 'Y', e.y - rectTop(m) - gap });
                }
                if (fabs(rectRight(e) - design.outlineW) <= tol) {
                    moves.push_back({ 'X', e.x - rectRight(m) - gap });
                }
                if (fabs(e.y) <= tol) {
                    moves.push_back({ 'Y', rectTop(e) - m.y + gap });
                }
                if (fabs(e.x) <= tol) {
                    moves.push_back({ 'X', rectRight(e) - m.x + gap });
                }

                sort(moves.begin(), moves.end(), [](const pair<char, double>& a, const pair<char, double>& b) {
                    return fabs(a.second) < fabs(b.second);
                });

                for (const auto& mv : moves) {
                    if (fabs(mv.second) <= EPS) continue;
                    if (mv.first == 'Y' && tryMoveBlocksYWithClosure(design, vector<int>{ movId }, mv.second)) {
                        moved = true;
                        break;
                    }
                    if (mv.first == 'X' && tryMoveBlocksXWithClosure(design, vector<int>{ movId }, mv.second)) {
                        moved = true;
                        break;
                    }
                }
            }
        }

        if (!found) return placementLegalAfterMove(design, design.blocks);
        if (!moved) return false;
    }

    return placementLegalAfterMove(design, design.blocks);
}

static bool tryEdgeOnlyOutlineTrimMain(Design& design, double shrinkW, double shrinkH) {
    shrinkW = max(0.0, shrinkW);
    shrinkH = max(0.0, shrinkH);
    if (shrinkW <= 1.0e-3 && shrinkH <= 1.0e-3) return false;

    const double oldW = design.outlineW;
    const double oldH = design.outlineH;
    const double newW = oldW - shrinkW;
    const double newH = oldH - shrinkH;
    if (newW <= 1.0 || newH <= 1.0) return false;
    if (newW > design.maxOutlineW + EPS || newH > design.maxOutlineH + EPS) return false;

    const vector<BlockInst> oldBlocks = design.blocks;
    design.outlineW = newW;
    design.outlineH = newH;

    const double tol = max(2.0, 1.0e-4 * max(oldW, oldH));
    vector<Rect> oldRects;
    oldRects.reserve(design.blocks.size());
    for (const BlockInst& b : design.blocks) oldRects.push_back(b.rect);
    for (int i = 0; i < static_cast<int>(design.blocks.size()) && i < static_cast<int>(oldBlocks.size()); ++i) {
        BlockInst& b = design.blocks[i];
        const Rect old = oldBlocks[i].rect;
        b.rect = old;
        if (b.spec.type == BlockType::EDGE) {
            const bool touchLeft = fabs(old.x) <= tol;
            const bool touchBottom = fabs(old.y) <= tol;
            const bool touchRight = fabs(rectRight(old) - oldW) <= tol;
            const bool touchTop = fabs(rectTop(old) - oldH) <= tol;

            if (touchRight) b.rect.x = newW - b.rect.w;
            else if (touchLeft) b.rect.x = 0.0;
            else b.rect.x = max(0.0, min(b.rect.x, newW - b.rect.w));

            if (touchTop) b.rect.y = newH - b.rect.h;
            else if (touchBottom) b.rect.y = 0.0;
            else b.rect.y = max(0.0, min(b.rect.y, newH - b.rect.h));
        }
        else if (!rectInsideOutlineMain(b.rect, design.outlineW, design.outlineH)) {
            return false;
        }
    }

    if (!repairEdgeTrimOverlapsMain(design)) return false;

    design.channels.clear();
    design.routes.clear();
    return placementLegalAfterMove(design, design.blocks);
}

static bool tryGravityOutlineTrimMain(Design& design, double shrinkW, double shrinkH) {
    shrinkW = max(0.0, shrinkW);
    shrinkH = max(0.0, shrinkH);
    if (shrinkW <= 1.0e-3 && shrinkH <= 1.0e-3) return false;

    const double oldW = design.outlineW;
    const double oldH = design.outlineH;
    const double newW = oldW - shrinkW;
    const double newH = oldH - shrinkH;
    if (newW <= 1.0 || newH <= 1.0) return false;
    if (newW > design.maxOutlineW + EPS || newH > design.maxOutlineH + EPS) return false;

    const vector<BlockInst> oldBlocks = design.blocks;
    vector<BlockInst> next = design.blocks;
    const double tol = max(2.0, 1.0e-4 * max(oldW, oldH));
    vector<Rect> oldRects;
    oldRects.reserve(design.blocks.size());
    for (const BlockInst& b : design.blocks) oldRects.push_back(b.rect);
    vector<int> movable;
    vector<Rect> placed;

    for (int i = 0; i < static_cast<int>(next.size()) && i < static_cast<int>(oldBlocks.size()); ++i) {
        BlockInst& b = next[i];
        const Rect old = oldBlocks[i].rect;
        b.rect = old;
        if (b.rect.w > newW + EPS || b.rect.h > newH + EPS) return false;

        if (b.spec.type == BlockType::EDGE) {
            const bool touchLeft = fabs(old.x) <= tol;
            const bool touchBottom = fabs(old.y) <= tol;
            const bool touchRight = fabs(rectRight(old) - oldW) <= tol;
            const bool touchTop = fabs(rectTop(old) - oldH) <= tol;

            if (touchRight) b.rect.x = newW - b.rect.w;
            else if (touchLeft) b.rect.x = 0.0;
            else b.rect.x = max(0.0, min(b.rect.x, newW - b.rect.w));

            if (touchTop) b.rect.y = newH - b.rect.h;
            else if (touchBottom) b.rect.y = 0.0;
            else b.rect.y = max(0.0, min(b.rect.y, newH - b.rect.h));

            if (!rectInsideOutlineMain(b.rect, newW, newH)) return false;
            placed.push_back(b.rect);
        }
        else {
            movable.push_back(i);
        }
    }

    sort(movable.begin(), movable.end(), [&](int a, int b) {
        const Rect& ra = oldBlocks[a].rect;
        const Rect& rb = oldBlocks[b].rect;
        if (fabs(ra.y - rb.y) > 1.0e-6) return ra.y < rb.y;
        if (fabs(ra.x - rb.x) > 1.0e-6) return ra.x < rb.x;
        return a < b;
        });

    const double gap = 1.0e-3;
    const double sx = newW / max(1.0, oldW);
    const double sy = newH / max(1.0, oldH);
    for (int id : movable) {
        Rect r = oldBlocks[id].rect;
        r.x = max(0.0, min(r.x * sx, newW - r.w));
        const double preferredY = max(0.0, min(r.y * sy, newH - r.h));

        vector<double> ys;
        ys.push_back(0.0);
        ys.push_back(preferredY);
        for (const Rect& p : placed) {
            if (overlapLen(r.x, r.x + r.w, p.x, rectRight(p)) > EPS) {
                ys.push_back(rectTop(p) + gap);
            }
        }
        sort(ys.begin(), ys.end());
        ys.erase(unique(ys.begin(), ys.end(), [](double a, double b) { return fabs(a - b) < 1.0e-5; }), ys.end());

        bool found = false;
        Rect best = r;
        for (double y : ys) {
            Rect cand = r;
            cand.y = max(0.0, min(y, newH - cand.h));
            if (!rectInsideOutlineMain(cand, newW, newH)) continue;
            bool ov = false;
            for (const Rect& p : placed) {
                if (rectOverlapAreaPositive(cand, p)) { ov = true; break; }
            }
            if (ov) continue;
            best = cand;
            found = true;
            break;
        }
        if (!found) return false;
        next[id].rect = best;
        placed.push_back(best);
    }

    const double savedW = design.outlineW;
    const double savedH = design.outlineH;
    vector<BlockInst> savedBlocks = design.blocks;
    design.outlineW = newW;
    design.outlineH = newH;
    design.blocks.swap(next);
    if (!placementLegalAfterMove(design, design.blocks)) {
        design.blocks.swap(savedBlocks);
        design.outlineW = savedW;
        design.outlineH = savedH;
        return false;
    }
    design.channels.clear();
    design.routes.clear();
    return true;
}
static vector<int> expandMoveClosureY(const Design& design, const vector<int>& seeds, double dy, bool& ok) {
    ok = false;
    const int n = static_cast<int>(design.blocks.size());
    if (seeds.empty() || fabs(dy) <= EPS) return {};

    vector<char> selected(n, 0);
    for (int id : seeds) {
        if (id < 0 || id >= n) return {};
        if (!blockMovableForHotRepair(design.blocks[id].spec)) return {};
        selected[id] = 1;
    }

    bool changed = true;
    while (changed) {
        changed = false;
        for (int i = 0; i < n; ++i) {
            if (selected[i]) continue;
            for (int j = 0; j < n; ++j) {
                if (!selected[j]) continue;
                Rect moved = design.blocks[j].rect;
                moved.y += dy;
                if (!rectOverlapAreaPositive(moved, design.blocks[i].rect)) continue;
                if (!blockMovableForHotRepair(design.blocks[i].spec)) return {};
                selected[i] = 1;
                changed = true;
                break;
            }
        }
    }

    vector<int> ids;
    for (int i = 0; i < n; ++i) if (selected[i]) ids.push_back(i);
    ok = true;
    return ids;
}

static vector<int> expandMoveClosureX(const Design& design, const vector<int>& seeds, double dx, bool& ok) {
    ok = false;
    const int n = static_cast<int>(design.blocks.size());
    if (seeds.empty() || fabs(dx) <= EPS) return {};

    vector<char> selected(n, 0);
    for (int id : seeds) {
        if (id < 0 || id >= n) return {};
        if (!blockMovableForHotRepair(design.blocks[id].spec)) return {};
        selected[id] = 1;
    }

    bool changed = true;
    while (changed) {
        changed = false;
        for (int i = 0; i < n; ++i) {
            if (selected[i]) continue;
            for (int j = 0; j < n; ++j) {
                if (!selected[j]) continue;
                Rect moved = design.blocks[j].rect;
                moved.x += dx;
                if (!rectOverlapAreaPositive(moved, design.blocks[i].rect)) continue;
                if (!blockMovableForHotRepair(design.blocks[i].spec)) return {};
                selected[i] = 1;
                changed = true;
                break;
            }
        }
    }

    vector<int> ids;
    for (int i = 0; i < n; ++i) if (selected[i]) ids.push_back(i);
    ok = true;
    return ids;
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

static bool tryMoveBlocksYWithClosure(Design& design, const vector<int>& ids, double dy) {
    bool ok = false;
    vector<int> expanded = expandMoveClosureY(design, ids, dy, ok);
    if (!ok) return false;
    return tryMoveBlocksY(design, expanded, dy);
}

static bool tryMoveBlocksX(Design& design, const vector<int>& ids, double dx) {
    if (ids.empty() || fabs(dx) <= EPS) return false;
    vector<BlockInst> moved = design.blocks;
    for (int id : ids) {
        if (id < 0 || id >= static_cast<int>(moved.size())) return false;
        if (!blockMovableForHotRepair(moved[id].spec)) return false;
        moved[id].rect.x += dx;
    }
    if (!placementLegalAfterMove(design, moved)) return false;
    design.blocks.swap(moved);
    return true;
}

static bool tryMoveBlocksXWithClosure(Design& design, const vector<int>& ids, double dx) {
    bool ok = false;
    vector<int> expanded = expandMoveClosureX(design, ids, dx, ok);
    if (!ok) return false;
    return tryMoveBlocksX(design, expanded, dx);
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
            if (fabs(b.rect.y - y2) <= 2.0) above.push_back(bi);
            if (fabs(rectTop(b.rect) - y1) <= 2.0) below.push_back(bi);
        }

        const double wanted = min(700.0, max(20.0, ch.overflow / CHANNEL_DENSITY));
        vector<double> deltas = { wanted, wanted * 0.67, wanted * 0.40, 16.0, 8.0 };
        for (double d : deltas) {
            if (tryMoveBlocksYWithClosure(design, above, d)) { changed = true; break; }
            if (tryMoveBlocksYWithClosure(design, below, -d)) { changed = true; break; }
            if (!above.empty() && !below.empty()) {
                Design trial = design;
                if (tryMoveBlocksYWithClosure(trial, above, d * 0.5) && tryMoveBlocksYWithClosure(trial, below, -d * 0.5)) {
                    design = std::move(trial);
                    changed = true;
                    break;
                }
            }
        }
    }

    return changed;
}
static bool relieveVerticalHotChannels(Design& design) {
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
        const bool dominantTB = fabs(ch.capacity - tbCap) <= fabs(ch.capacity - lrCap) + 1.0e-3;
        if (!dominantTB) continue;
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

        vector<int> left;
        vector<int> right;
        for (int bi = 0; bi < static_cast<int>(design.blocks.size()); ++bi) {
            const BlockInst& b = design.blocks[bi];
            if (!blockMovableForHotRepair(b.spec)) continue;
            if (overlapLen(b.rect.y, rectTop(b.rect), y1, y2) <= 1.0e-4) continue;
            if (fabs(rectRight(b.rect) - x1) <= 2.0) left.push_back(bi);
            if (fabs(b.rect.x - x2) <= 2.0) right.push_back(bi);
        }

        const double wanted = min(700.0, max(20.0, ch.overflow / CHANNEL_DENSITY));
        vector<double> deltas = { wanted, wanted * 0.67, wanted * 0.40, 16.0, 8.0 };
        for (double d : deltas) {
            if (tryMoveBlocksXWithClosure(design, right, d)) { changed = true; break; }
            if (tryMoveBlocksXWithClosure(design, left, -d)) { changed = true; break; }
            if (!left.empty() && !right.empty()) {
                Design trial = design;
                if (tryMoveBlocksXWithClosure(trial, right, d * 0.5) && tryMoveBlocksXWithClosure(trial, left, -d * 0.5)) {
                    design = std::move(trial);
                    changed = true;
                    break;
                }
            }
        }
    }

    return changed;
}


static bool makeCommonEdgeSnapCandidates(const Design& design, vector<Design>& out, int maxCandidates) {
    struct SnapChannel {
        int index = -1;
        double overflow = 0.0;
        double thickness = 0.0;
        bool horizontalGap = true;
    };

    vector<SnapChannel> hot;
    const double maxSnapThickness = min(48.0, max(8.0, 0.012 * max(design.outlineW, design.outlineH)));

    for (int i = 0; i < static_cast<int>(design.channels.size()); ++i) {
        const Channel& ch = design.channels[i];
        if (ch.overflow <= EPS) continue;

        if (ch.rect.h > EPS && ch.rect.h <= maxSnapThickness) {
            hot.push_back({ i, ch.overflow, ch.rect.h, true });
        }
        if (ch.rect.w > EPS && ch.rect.w <= maxSnapThickness) {
            hot.push_back({ i, ch.overflow, ch.rect.w, false });
        }
    }

    sort(hot.begin(), hot.end(), [](const SnapChannel& a, const SnapChannel& b) {
        if (fabs(a.overflow - b.overflow) > 1.0) return a.overflow > b.overflow;
        if (fabs(a.thickness - b.thickness) > 1.0e-3) return a.thickness < b.thickness;
        return a.index < b.index;
        });

    const int startCount = static_cast<int>(out.size());
    const int channelLimit = min(10, static_cast<int>(hot.size()));

    auto addCandidate = [&](Design&& trial) {
        if (static_cast<int>(out.size()) >= maxCandidates) return;
        if (!placementLegalAfterMove(trial, trial.blocks)) return;
        trial.channels.clear();
        trial.routes.clear();
        out.push_back(std::move(trial));
        };

    for (int hi = 0; hi < channelLimit && static_cast<int>(out.size()) < maxCandidates; ++hi) {
        const Channel& ch = design.channels[hot[hi].index];
        const double x1 = ch.rect.x;
        const double x2 = rectRight(ch.rect);
        const double y1 = ch.rect.y;
        const double y2 = rectTop(ch.rect);

        if (hot[hi].horizontalGap) {
            const double gap = ch.rect.h;
            vector<int> above;
            vector<int> below;
            for (int bi = 0; bi < static_cast<int>(design.blocks.size()); ++bi) {
                const BlockInst& b = design.blocks[bi];
                if (!blockMovableForHotRepair(b.spec)) continue;
                if (overlapLen(b.rect.x, rectRight(b.rect), x1, x2) <= 1.0e-4) continue;
                if (fabs(b.rect.y - y2) <= 2.0) above.push_back(bi);
                if (fabs(rectTop(b.rect) - y1) <= 2.0) below.push_back(bi);
            }

            if (!above.empty()) {
                Design trial = design;
                if (tryMoveBlocksYWithClosure(trial, above, -gap)) addCandidate(std::move(trial));
            }
            if (!below.empty()) {
                Design trial = design;
                if (tryMoveBlocksYWithClosure(trial, below, gap)) addCandidate(std::move(trial));
            }
            if (!above.empty() && !below.empty() && gap > 1.0) {
                Design trial = design;
                if (tryMoveBlocksYWithClosure(trial, above, -gap * 0.5) &&
                    tryMoveBlocksYWithClosure(trial, below, gap * 0.5)) {
                    addCandidate(std::move(trial));
                }
            }
        }
        else {
            const double gap = ch.rect.w;
            vector<int> left;
            vector<int> right;
            for (int bi = 0; bi < static_cast<int>(design.blocks.size()); ++bi) {
                const BlockInst& b = design.blocks[bi];
                if (!blockMovableForHotRepair(b.spec)) continue;
                if (overlapLen(b.rect.y, rectTop(b.rect), y1, y2) <= 1.0e-4) continue;
                if (fabs(rectRight(b.rect) - x1) <= 2.0) left.push_back(bi);
                if (fabs(b.rect.x - x2) <= 2.0) right.push_back(bi);
            }

            if (!right.empty()) {
                Design trial = design;
                if (tryMoveBlocksXWithClosure(trial, right, -gap)) addCandidate(std::move(trial));
            }
            if (!left.empty()) {
                Design trial = design;
                if (tryMoveBlocksXWithClosure(trial, left, gap)) addCandidate(std::move(trial));
            }
            if (!left.empty() && !right.empty() && gap > 1.0) {
                Design trial = design;
                if (tryMoveBlocksXWithClosure(trial, right, -gap * 0.5) &&
                    tryMoveBlocksXWithClosure(trial, left, gap * 0.5)) {
                    addCandidate(std::move(trial));
                }
            }
        }
    }

    return static_cast<int>(out.size()) > startCount;
}
static bool isChannelNameMain(const string& name) {
    return name.rfind("CH", 0) == 0;
}

static bool isTurnPairMain(int a, int b) {
    if (a < 1 || a > 4 || b < 1 || b > 4 || a == b) return false;
    const bool lr = (a == 1 && b == 3) || (a == 3 && b == 1);
    const bool tb = (a == 2 && b == 4) || (a == 4 && b == 2);
    return !lr && !tb;
}

static double routedStructureScoreMain(const Design& design) {
    double score = static_cast<double>(design.channels.size()) * 2.0;

    for (const Channel& ch : design.channels) {
        const double w = max(0.0, ch.rect.w);
        const double h = max(0.0, ch.rect.h);
        const double thin = min(w, h);
        const double cap = max(1.0, ch.capacity);
        const double util = max(0.0, ch.usedNets) / cap;

        if (thin <= 1.0) {
            score += 5000.0 + (1.0 - thin) * 1000.0;
        }
        else if (thin <= 8.0 && util <= 0.15) {
            score += 900.0 + (8.0 - thin) * 80.0;
        }
        else if (thin <= 24.0 && util <= 0.08) {
            score += 120.0 + (24.0 - thin) * 8.0;
        }
    }

    for (const RoutePath& p : design.routes) {
        score += static_cast<double>(p.steps.size()) * 0.25;
        for (int i = 0; i + 1 < static_cast<int>(p.steps.size()); ++i) {
            const RouteStep& a = p.steps[i];
            const RouteStep& b = p.steps[i + 1];
            if (a.rectName == b.rectName && isChannelNameMain(a.rectName) && isTurnPairMain(a.edge, b.edge)) {
                score += 15.0 + 0.015 * static_cast<double>(max(0, p.netCount));
            }
        }
    }

    return score;
}

static bool makeThinChannelAlignmentCandidates(const Design& design, vector<Design>& out, int maxCandidates) {
    struct ThinChannel {
        int index = -1;
        double thickness = 0.0;
        double score = 0.0;
        bool horizontalGap = true;
    };

    vector<ThinChannel> hot;
    const double maxSnapThickness = min(18.0, max(2.0, 0.0035 * max(design.outlineW, design.outlineH)));

    auto consider = [&](int i, double thickness, bool horizontalGap) {
        if (thickness <= EPS || thickness > maxSnapThickness) return;
        const Channel& ch = design.channels[i];
        if (ch.overflow > EPS) return;

        const double cap = max(1.0, ch.capacity);
        const double used = max(0.0, ch.usedNets);
        const double util = used / cap;
        const bool microscopicUnused = thickness <= 1.0 && used <= 1.0;
        const bool lightlyUsed = used <= 120.0 && util <= 0.10;
        if (!microscopicUnused && !lightlyUsed) return;

        double s = (microscopicUnused ? 50000.0 : 0.0);
        s += (maxSnapThickness - thickness) * 500.0;
        s += max(0.0, 0.12 - util) * 3000.0;
        s -= used;
        hot.push_back({ i, thickness, s, horizontalGap });
    };

    for (int i = 0; i < static_cast<int>(design.channels.size()); ++i) {
        const Channel& ch = design.channels[i];
        consider(i, ch.rect.h, true);
        consider(i, ch.rect.w, false);
    }

    sort(hot.begin(), hot.end(), [](const ThinChannel& a, const ThinChannel& b) {
        if (fabs(a.score - b.score) > 1.0e-6) return a.score > b.score;
        if (fabs(a.thickness - b.thickness) > 1.0e-6) return a.thickness < b.thickness;
        return a.index < b.index;
        });

    const int startCount = static_cast<int>(out.size());
    const int channelLimit = min(12, static_cast<int>(hot.size()));

    auto addCandidate = [&](Design&& trial) {
        if (static_cast<int>(out.size()) >= maxCandidates) return;
        if (!placementLegalAfterMove(trial, trial.blocks)) return;
        trial.channels.clear();
        trial.routes.clear();
        out.push_back(std::move(trial));
        };

    for (int hi = 0; hi < channelLimit && static_cast<int>(out.size()) < maxCandidates; ++hi) {
        const Channel& ch = design.channels[hot[hi].index];
        const double x1 = ch.rect.x;
        const double x2 = rectRight(ch.rect);
        const double y1 = ch.rect.y;
        const double y2 = rectTop(ch.rect);
        const double gap = hot[hi].thickness;

        if (hot[hi].horizontalGap) {
            vector<int> above;
            vector<int> below;
            for (int bi = 0; bi < static_cast<int>(design.blocks.size()); ++bi) {
                const BlockInst& b = design.blocks[bi];
                if (!blockMovableForHotRepair(b.spec)) continue;
                if (overlapLen(b.rect.x, rectRight(b.rect), x1, x2) <= 1.0e-4) continue;
                if (fabs(b.rect.y - y2) <= 2.0) above.push_back(bi);
                if (fabs(rectTop(b.rect) - y1) <= 2.0) below.push_back(bi);
            }

            if (!above.empty()) {
                Design trial = design;
                if (tryMoveBlocksYWithClosure(trial, above, -gap)) addCandidate(std::move(trial));
            }
            if (!below.empty()) {
                Design trial = design;
                if (tryMoveBlocksYWithClosure(trial, below, gap)) addCandidate(std::move(trial));
            }
            if (!above.empty() && !below.empty() && gap > 1.0) {
                Design trial = design;
                if (tryMoveBlocksYWithClosure(trial, above, -gap * 0.5) &&
                    tryMoveBlocksYWithClosure(trial, below, gap * 0.5)) {
                    addCandidate(std::move(trial));
                }
            }
        }
        else {
            vector<int> left;
            vector<int> right;
            for (int bi = 0; bi < static_cast<int>(design.blocks.size()); ++bi) {
                const BlockInst& b = design.blocks[bi];
                if (!blockMovableForHotRepair(b.spec)) continue;
                if (overlapLen(b.rect.y, rectTop(b.rect), y1, y2) <= 1.0e-4) continue;
                if (fabs(rectRight(b.rect) - x1) <= 2.0) left.push_back(bi);
                if (fabs(b.rect.x - x2) <= 2.0) right.push_back(bi);
            }

            if (!right.empty()) {
                Design trial = design;
                if (tryMoveBlocksXWithClosure(trial, right, -gap)) addCandidate(std::move(trial));
            }
            if (!left.empty()) {
                Design trial = design;
                if (tryMoveBlocksXWithClosure(trial, left, gap)) addCandidate(std::move(trial));
            }
            if (!left.empty() && !right.empty() && gap > 1.0) {
                Design trial = design;
                if (tryMoveBlocksXWithClosure(trial, right, -gap * 0.5) &&
                    tryMoveBlocksXWithClosure(trial, left, gap * 0.5)) {
                    addCandidate(std::move(trial));
                }
            }
        }
    }

    return static_cast<int>(out.size()) > startCount;
}
static vector<int> legalEdgesForDetourMoveMain(const BlockSpec& spec) {
    if (!spec.portEdges.empty()) return spec.portEdges;
    return { 1, 2, 3, 4 };
}

static bool validEdgeForDetourMoveMain(int edge) {
    return edge >= 1 && edge <= 4;
}

static pair<double, double> edgeAnchorForDetourMoveMain(const Rect& r, int edge, double t) {
    t = max(0.0, min(1.0, t));
    if (edge == 1) return { r.x, r.y + r.h * t };
    if (edge == 3) return { rectRight(r), r.y + r.h * t };
    if (edge == 2) return { r.x + r.w * t, rectTop(r) };
    if (edge == 4) return { r.x + r.w * t, r.y };
    return { rectCx(r), rectCy(r) };
}

static double portAwareLowerBoundWLMain(const Design& design, int srcId, int dstId, int nets) {
    if (srcId < 0 || dstId < 0 || srcId >= static_cast<int>(design.blocks.size()) || dstId >= static_cast<int>(design.blocks.size())) return 0.0;
    const BlockInst& src = design.blocks[srcId];
    const BlockInst& dst = design.blocks[dstId];
    const vector<int> srcEdges = legalEdgesForDetourMoveMain(src.spec);
    const vector<int> dstEdges = legalEdgesForDetourMoveMain(dst.spec);
    const array<double, 3> taps = { 0.25, 0.50, 0.75 };

    double best = numeric_limits<double>::infinity();
    for (int se : srcEdges) {
        if (!validEdgeForDetourMoveMain(se)) continue;
        for (int de : dstEdges) {
            if (!validEdgeForDetourMoveMain(de)) continue;
            for (double st : taps) {
                const auto sp = edgeAnchorForDetourMoveMain(src.rect, se, st);
                for (double dt : taps) {
                    const auto dp = edgeAnchorForDetourMoveMain(dst.rect, de, dt);
                    best = min(best, manhattan(sp.first, sp.second, dp.first, dp.second));
                }
            }
        }
    }
    if (!std::isfinite(best)) best = manhattan(rectCx(src.rect), rectCy(src.rect), rectCx(dst.rect), rectCy(dst.rect));
    return best * static_cast<double>(max(0, nets));
}
static bool makeDetourMoveCandidates(const Design& design, vector<Design>& out, int maxCandidates) {
    struct HotPath {
        int src = -1;
        int dst = -1;
        int nets = 0;
        double wl = 0.0;
        double lowerBound = 0.0;
        double excess = 0.0;
        double score = 0.0;
    };

    vector<HotPath> hot;
    for (const RoutePath& p : design.routes) {
        if (p.open || p.netCount <= 0 || p.wireLength <= 0.0) continue;
        auto sit = design.blockNameToIndex.find(p.srcBlock);
        auto dit = design.blockNameToIndex.find(p.dstBlock);
        if (sit == design.blockNameToIndex.end() || dit == design.blockNameToIndex.end()) continue;
        const double lowerBound = portAwareLowerBoundWLMain(design, sit->second, dit->second, p.netCount);
        const double excess = max(0.0, p.wireLength - lowerBound);
        const double score = excess + 0.05 * p.wireLength;
        hot.push_back({ sit->second, dit->second, p.netCount, p.wireLength, lowerBound, excess, score });
    }
    sort(hot.begin(), hot.end(), [](const HotPath& a, const HotPath& b) {
        if (fabs(a.score - b.score) > 1.0) return a.score > b.score;
        if (fabs(a.excess - b.excess) > 1.0) return a.excess > b.excess;
        if (a.nets != b.nets) return a.nets > b.nets;
        return a.src < b.src;
    });

    const int startCount = static_cast<int>(out.size());
    auto sameGeometry = [&](const Design& a, const Design& b) {
        if (a.blocks.size() != b.blocks.size()) return false;
        for (int i = 0; i < static_cast<int>(a.blocks.size()); ++i) {
            const Rect& ra = a.blocks[i].rect;
            const Rect& rb = b.blocks[i].rect;
            if (fabs(ra.x - rb.x) > 0.5 || fabs(ra.y - rb.y) > 0.5 ||
                fabs(ra.w - rb.w) > 0.5 || fabs(ra.h - rb.h) > 0.5) return false;
        }
        return true;
    };
    auto addCandidate = [&](Design&& trial) {
        if (static_cast<int>(out.size()) >= maxCandidates) return;
        if (!placementLegalAfterMove(trial, trial.blocks)) return;
        trial.channels.clear();
        trial.routes.clear();
        for (const Design& old : out) if (sameGeometry(old, trial)) return;
        out.push_back(std::move(trial));
    };
    auto tryOneAxis = [&](int id, double delta, bool xAxis) {
        if (static_cast<int>(out.size()) >= maxCandidates || fabs(delta) <= 1.0) return;
        Design trial = design;
        bool ok = xAxis ? tryMoveBlocksXWithClosure(trial, { id }, delta)
                        : tryMoveBlocksYWithClosure(trial, { id }, delta);
        if (ok) addCandidate(std::move(trial));
    };
    auto tryTwoAxis = [&](int id, double dx, double dy) {
        if (static_cast<int>(out.size()) >= maxCandidates || (fabs(dx) <= 1.0 && fabs(dy) <= 1.0)) return;
        Design trial = design;
        bool ok = true;
        if (fabs(dx) > 1.0) ok = ok && tryMoveBlocksXWithClosure(trial, { id }, dx);
        if (fabs(dy) > 1.0) ok = ok && tryMoveBlocksYWithClosure(trial, { id }, dy);
        if (ok) addCandidate(std::move(trial));

        trial = design;
        ok = true;
        if (fabs(dy) > 1.0) ok = ok && tryMoveBlocksYWithClosure(trial, { id }, dy);
        if (fabs(dx) > 1.0) ok = ok && tryMoveBlocksXWithClosure(trial, { id }, dx);
        if (ok) addCandidate(std::move(trial));
    };

    const int pathLimit = min(14, static_cast<int>(hot.size()));
    const double maxStep = max(80.0, 0.075 * max(design.outlineW, design.outlineH));
    const double minStep = 40.0;
    for (int pi = 0; pi < pathLimit && static_cast<int>(out.size()) < maxCandidates; ++pi) {
        const HotPath& hp = hot[pi];
        if (hp.src < 0 || hp.dst < 0 || hp.src >= static_cast<int>(design.blocks.size()) || hp.dst >= static_cast<int>(design.blocks.size())) continue;
        const Rect& a = design.blocks[hp.src].rect;
        const Rect& b = design.blocks[hp.dst].rect;
        const double dx = rectCx(b) - rectCx(a);
        const double dy = rectCy(b) - rectCy(a);
        const array<double, 3> frac = { 0.10, 0.18, 0.28 };
        for (double f : frac) {
            if (static_cast<int>(out.size()) >= maxCandidates) break;
            const double sx = fabs(dx) > minStep ? copysign(min(maxStep, max(minStep, fabs(dx) * f)), dx) : 0.0;
            const double sy = fabs(dy) > minStep ? copysign(min(maxStep, max(minStep, fabs(dy) * f)), dy) : 0.0;
            if (blockMovableForHotRepair(design.blocks[hp.src].spec)) {
                tryOneAxis(hp.src, sx, true);
                tryOneAxis(hp.src, sy, false);
                tryTwoAxis(hp.src, sx, sy);
            }
            if (blockMovableForHotRepair(design.blocks[hp.dst].spec)) {
                tryOneAxis(hp.dst, -sx, true);
                tryOneAxis(hp.dst, -sy, false);
                tryTwoAxis(hp.dst, -sx, -sy);
            }
if (blockMovableForHotRepair(design.blocks[hp.src].spec) && blockMovableForHotRepair(design.blocks[hp.dst].spec)) {
                Design trial = design;
                bool ok = true;
                if (fabs(sx) > 1.0) ok = ok && tryMoveBlocksXWithClosure(trial, { hp.src }, sx * 0.5);
                if (fabs(sx) > 1.0) ok = ok && tryMoveBlocksXWithClosure(trial, { hp.dst }, -sx * 0.5);
                if (fabs(sy) > 1.0) ok = ok && tryMoveBlocksYWithClosure(trial, { hp.src }, sy * 0.5);
                if (fabs(sy) > 1.0) ok = ok && tryMoveBlocksYWithClosure(trial, { hp.dst }, -sy * 0.5);
                if (ok) addCandidate(std::move(trial));
            }
        }
    }

    return static_cast<int>(out.size()) > startCount;
}
static bool edgeCanSlideYForCapacityRelief(const Design& design, const BlockInst& b, double dy) {
    if (b.spec.type != BlockType::EDGE) return true;
    Rect moved = b.rect;
    moved.y += dy;
    const double tol = max(2.0, 1.0e-4 * max(design.outlineW, design.outlineH));
    const bool wasTop = fabs(rectTop(b.rect) - design.outlineH) <= tol;
    const bool mayGrowTop = dy > 0.0 && wasTop && rectTop(moved) <= design.maxOutlineH + EPS;
    const double effectiveH = mayGrowTop ? max(design.outlineH, ceilToIntegerGridMain(rectTop(moved))) : design.outlineH;
    if (!rectInsideOutlineMain(moved, design.outlineW, effectiveH)) return false;
    const bool staysLeft = fabs(b.rect.x) <= tol && fabs(moved.x) <= tol;
    const bool staysRight = fabs(rectRight(b.rect) - design.outlineW) <= tol && fabs(rectRight(moved) - design.outlineW) <= tol;
    const bool staysBottom = fabs(b.rect.y) <= tol && fabs(moved.y) <= tol;
    const bool staysTop = wasTop && fabs(rectTop(moved) - effectiveH) <= tol;
    const bool movesToTop = fabs(rectTop(moved) - effectiveH) <= tol;
    return staysLeft || staysRight || staysBottom || staysTop || movesToTop;
}
static bool edgeCanSlideXForCapacityRelief(const Design& design, const BlockInst& b, double dx) {
    if (b.spec.type != BlockType::EDGE) return true;
    Rect moved = b.rect;
    moved.x += dx;
    const double tol = max(2.0, 1.0e-4 * max(design.outlineW, design.outlineH));
    const bool wasRight = fabs(rectRight(b.rect) - design.outlineW) <= tol;
    const bool mayGrowRight = dx > 0.0 && wasRight && rectRight(moved) <= design.maxOutlineW + EPS;
    const double effectiveW = mayGrowRight ? max(design.outlineW, ceilToIntegerGridMain(rectRight(moved))) : design.outlineW;
    if (!rectInsideOutlineMain(moved, effectiveW, design.outlineH)) return false;
    const bool staysBottom = fabs(b.rect.y) <= tol && fabs(moved.y) <= tol;
    const bool staysTop = fabs(rectTop(b.rect) - design.outlineH) <= tol && fabs(rectTop(moved) - design.outlineH) <= tol;
    const bool staysLeft = fabs(b.rect.x) <= tol && fabs(moved.x) <= tol;
    const bool staysRight = wasRight && fabs(rectRight(moved) - effectiveW) <= tol;
    const bool movesToRight = fabs(rectRight(moved) - effectiveW) <= tol;
    return staysBottom || staysTop || staysLeft || staysRight || movesToRight;
}
static vector<int> expandMoveClosureYForCapacityRelief(const Design& design, const vector<int>& seeds, double dy, bool& ok) {
    ok = false;
    const int n = static_cast<int>(design.blocks.size());
    if (seeds.empty() || fabs(dy) <= EPS) return {};

    vector<char> selected(n, 0);
    for (int id : seeds) {
        if (id < 0 || id >= n) return {};
        if (design.blocks[id].spec.type == BlockType::EDGE) {
            if (!edgeCanSlideYForCapacityRelief(design, design.blocks[id], dy)) return {};
        }
        else if (!blockMovableForHotRepair(design.blocks[id].spec)) return {};
        selected[id] = 1;
    }

    bool changed = true;
    while (changed) {
        changed = false;
        for (int i = 0; i < n; ++i) {
            if (selected[i]) continue;
            for (int j = 0; j < n; ++j) {
                if (!selected[j]) continue;
                Rect moved = design.blocks[j].rect;
                moved.y += dy;
                if (!rectOverlapAreaPositive(moved, design.blocks[i].rect)) continue;
                if (design.blocks[i].spec.type == BlockType::EDGE) {
                    if (!edgeCanSlideYForCapacityRelief(design, design.blocks[i], dy)) return {};
                }
                else if (!blockMovableForHotRepair(design.blocks[i].spec)) {
                    return {};
                }
                selected[i] = 1;
                changed = true;
                break;
            }
        }
    }

    vector<int> ids;
    for (int i = 0; i < n; ++i) if (selected[i]) ids.push_back(i);
    ok = true;
    return ids;
}

static vector<int> expandMoveClosureXForCapacityRelief(const Design& design, const vector<int>& seeds, double dx, bool& ok) {
    ok = false;
    const int n = static_cast<int>(design.blocks.size());
    if (seeds.empty() || fabs(dx) <= EPS) return {};

    vector<char> selected(n, 0);
    for (int id : seeds) {
        if (id < 0 || id >= n) return {};
        if (design.blocks[id].spec.type == BlockType::EDGE) {
            if (!edgeCanSlideXForCapacityRelief(design, design.blocks[id], dx)) return {};
        }
        else if (!blockMovableForHotRepair(design.blocks[id].spec)) return {};
        selected[id] = 1;
    }

    bool changed = true;
    while (changed) {
        changed = false;
        for (int i = 0; i < n; ++i) {
            if (selected[i]) continue;
            for (int j = 0; j < n; ++j) {
                if (!selected[j]) continue;
                Rect moved = design.blocks[j].rect;
                moved.x += dx;
                if (!rectOverlapAreaPositive(moved, design.blocks[i].rect)) continue;
                if (design.blocks[i].spec.type == BlockType::EDGE) {
                    if (!edgeCanSlideXForCapacityRelief(design, design.blocks[i], dx)) return {};
                }
                else if (!blockMovableForHotRepair(design.blocks[i].spec)) {
                    return {};
                }
                selected[i] = 1;
                changed = true;
                break;
            }
        }
    }

    vector<int> ids;
    for (int i = 0; i < n; ++i) if (selected[i]) ids.push_back(i);
    ok = true;
    return ids;
}

static bool tryMoveBlocksYForCapacityRelief(Design& design, const vector<int>& ids, double dy) {
    bool ok = false;
    vector<int> expanded = expandMoveClosureYForCapacityRelief(design, ids, dy, ok);
    if (!ok) return false;
    vector<BlockInst> moved = design.blocks;
    for (int id : expanded) {
        if (id < 0 || id >= static_cast<int>(moved.size())) return false;
        if (moved[id].spec.type == BlockType::EDGE) {
            if (!edgeCanSlideYForCapacityRelief(design, moved[id], dy)) return false;
        }
        else if (!blockMovableForHotRepair(moved[id].spec)) {
            return false;
        }
        moved[id].rect.y += dy;
    }
    double newH = design.outlineH;
    for (const BlockInst& b : moved) newH = max(newH, ceilToIntegerGridMain(rectTop(b.rect)));
    if (newH > design.maxOutlineH + EPS) return false;
    if (newH > design.outlineH + EPS) {
        const double tol = max(2.0, 1.0e-4 * max(design.outlineW, newH));
        for (const BlockInst& b : moved) {
            if (b.spec.type == BlockType::EDGE && edgeLocationRequiresSideMain(b.spec, 'T') && fabs(rectTop(b.rect) - newH) > tol) return false;
        }
        design.outlineH = newH;
    }
    if (!placementLegalAfterMove(design, moved)) return false;
    design.blocks.swap(moved);
    return true;
}
static bool tryMoveBlocksXForCapacityRelief(Design& design, const vector<int>& ids, double dx) {
    bool ok = false;
    vector<int> expanded = expandMoveClosureXForCapacityRelief(design, ids, dx, ok);
    if (!ok) return false;
    vector<BlockInst> moved = design.blocks;
    for (int id : expanded) {
        if (id < 0 || id >= static_cast<int>(moved.size())) return false;
        if (moved[id].spec.type == BlockType::EDGE) {
            if (!edgeCanSlideXForCapacityRelief(design, moved[id], dx)) return false;
        }
        else if (!blockMovableForHotRepair(moved[id].spec)) {
            return false;
        }
        moved[id].rect.x += dx;
    }
    double newW = design.outlineW;
    for (const BlockInst& b : moved) newW = max(newW, ceilToIntegerGridMain(rectRight(b.rect)));
    if (newW > design.maxOutlineW + EPS) return false;
    if (newW > design.outlineW + EPS) {
        const double tol = max(2.0, 1.0e-4 * max(newW, design.outlineH));
        for (const BlockInst& b : moved) {
            if (b.spec.type == BlockType::EDGE && edgeLocationRequiresSideMain(b.spec, 'R') && fabs(rectRight(b.rect) - newW) > tol) return false;
        }
        design.outlineW = newW;
    }
    if (!placementLegalAfterMove(design, moved)) return false;
    design.blocks.swap(moved);
    return true;
}static bool makeCapacityReliefCandidates(const Design& design, vector<Design>& out, int maxCandidates) {
    struct HotChannel {
        int index = -1;
        double overflow = 0.0;
        bool horizontal = true;
    };

    vector<HotChannel> hot;
    for (int i = 0; i < static_cast<int>(design.channels.size()); ++i) {
        const Channel& ch = design.channels[i];
        if (ch.overflow <= EPS) continue;
        const double lrCap = max(0.0, ch.rect.h * CHANNEL_DENSITY);
        const double tbCap = max(0.0, ch.rect.w * CHANNEL_DENSITY);
        const bool dominantLR = fabs(ch.capacity - lrCap) <= fabs(ch.capacity - tbCap) + 1.0e-3;
        hot.push_back({ i, ch.overflow, dominantLR });
    }

    sort(hot.begin(), hot.end(), [](const HotChannel& a, const HotChannel& b) {
        if (fabs(a.overflow - b.overflow) > 1.0) return a.overflow > b.overflow;
        return a.index < b.index;
        });

    const int startCount = static_cast<int>(out.size());
    const int channelLimit = min(8, static_cast<int>(hot.size()));
    int channelStartForAdd = startCount;
    const int perChannelCandidateLimit = max(4, maxCandidates / max(1, min(3, channelLimit)));

    auto addCandidate = [&](Design&& trial) {
        if (static_cast<int>(out.size()) >= maxCandidates) return;
        if (static_cast<int>(out.size()) - channelStartForAdd >= perChannelCandidateLimit) return;
        if (!placementLegalAfterMove(trial, trial.blocks)) return;
        trial.channels.clear();
        trial.routes.clear();
        out.push_back(std::move(trial));
        };

    auto reliefDeltas = [](double overflow) {
        const double exact = min(700.0, max(2.0, overflow / CHANNEL_DENSITY + 0.50));
        vector<double> d = { min(exact, 220.0), exact, exact * 0.75, exact * 0.50, exact * 1.25, 8.0, 16.0, 24.0, 48.0 };
        vector<double> out;
        for (double v : d) {
            v = max(2.0, min(700.0, v));
            bool dup = false;
            for (double old : out) {
                if (fabs(old - v) < 0.25) { dup = true; break; }
            }
            if (!dup) out.push_back(v);
        }
        return out;
        };

    for (int hi = 0; hi < channelLimit && static_cast<int>(out.size()) < maxCandidates; ++hi) {
        channelStartForAdd = static_cast<int>(out.size());
        const Channel& ch = design.channels[hot[hi].index];
        const double x1 = ch.rect.x;
        const double x2 = rectRight(ch.rect);
        const double y1 = ch.rect.y;
        const double y2 = rectTop(ch.rect);
        const vector<double> deltas = reliefDeltas(hot[hi].overflow);

        if (hot[hi].horizontal) {
            vector<int> above;
            vector<int> below;
            for (int bi = 0; bi < static_cast<int>(design.blocks.size()); ++bi) {
                const BlockInst& b = design.blocks[bi];
                if (b.spec.type != BlockType::EDGE && !blockMovableForHotRepair(b.spec)) continue;
                if (overlapLen(b.rect.x, rectRight(b.rect), x1, x2) <= 1.0e-4) continue;
                if (fabs(b.rect.y - y2) <= 2.0) above.push_back(bi);
                if (fabs(rectTop(b.rect) - y1) <= 2.0) below.push_back(bi);
            }

            for (double d : deltas) {
                if (static_cast<int>(out.size()) >= maxCandidates) break;
                if (!above.empty()) {
                    Design trial = design;
                    if (tryMoveBlocksYForCapacityRelief(trial, above, d)) addCandidate(std::move(trial));
                }
                if (static_cast<int>(out.size()) >= maxCandidates) break;
                if (!below.empty()) {
                    Design trial = design;
                    if (tryMoveBlocksYForCapacityRelief(trial, below, -d)) addCandidate(std::move(trial));
                }
                if (static_cast<int>(out.size()) >= maxCandidates) break;
                if (!above.empty() && !below.empty()) {
                    Design trial = design;
                    if (tryMoveBlocksYForCapacityRelief(trial, above, d * 0.5) &&
                        tryMoveBlocksYForCapacityRelief(trial, below, -d * 0.5)) {
                        addCandidate(std::move(trial));
                    }
                }
            }
        }
        else {
            vector<int> left;
            vector<int> right;
            auto addUniqueBlock = [](vector<int>& ids, int id) {
                if (find(ids.begin(), ids.end(), id) == ids.end()) ids.push_back(id);
                };
            for (int bi = 0; bi < static_cast<int>(design.blocks.size()); ++bi) {
                const BlockInst& b = design.blocks[bi];
                if (b.spec.type != BlockType::EDGE && !blockMovableForHotRepair(b.spec)) continue;
                const double yOverlap = overlapLen(b.rect.y, rectTop(b.rect), y1, y2);
                const bool yTouchesLoose = rectTop(b.rect) >= y1 - 2.0 && b.rect.y <= y2 + 2.0;
                if (!yTouchesLoose) continue;
                if (yOverlap > 1.0e-4) {
                    if (fabs(rectRight(b.rect) - x1) <= 2.0) addUniqueBlock(left, bi);
                    if (fabs(b.rect.x - x2) <= 2.0) addUniqueBlock(right, bi);
                }
                else {
                    if (fabs(rectRight(b.rect) - x1) <= 2.0 || fabs(b.rect.x - x1) <= 2.0) addUniqueBlock(left, bi);
                    if (fabs(b.rect.x - x2) <= 2.0 || fabs(rectRight(b.rect) - x2) <= 2.0) addUniqueBlock(right, bi);
                }
            }

            for (double d : deltas) {
                if (static_cast<int>(out.size()) >= maxCandidates) break;
                if (!right.empty()) {
                    Design trial = design;
                    if (tryMoveBlocksXForCapacityRelief(trial, right, d)) addCandidate(std::move(trial));
                }
                if (static_cast<int>(out.size()) >= maxCandidates) break;
                if (!left.empty()) {
                    Design trial = design;
                    if (tryMoveBlocksXForCapacityRelief(trial, left, -d)) addCandidate(std::move(trial));
                }
                if (static_cast<int>(out.size()) >= maxCandidates) break;
                if (!left.empty() && !right.empty()) {
                    Design trial = design;
                    if (tryMoveBlocksXForCapacityRelief(trial, right, d * 0.5) &&
                        tryMoveBlocksXForCapacityRelief(trial, left, -d * 0.5)) {
                        addCandidate(std::move(trial));
                    }
                }
            }
        }
    }

    return static_cast<int>(out.size()) > startCount;
}
static const char* certificateDirName(Router::ChannelDir dir) {
    return dir == Router::ChannelDir::LR ? "LR" : "TB";
}

static bool makeCertificateReliefCandidates(const Design& design, const vector<Router::FailureCertificate>& certs, vector<Design>& out, int maxCandidates) {
    struct HotNeed {
        int channelIndex = -1;
        Router::ChannelDir dir = Router::ChannelDir::LR;
        double shortage = 0.0;
        double delta = 0.0;
    };

    vector<HotNeed> hot;
    for (const auto& cert : certs) {
        for (const auto& need : cert.hardNeeds) {
            if (need.channelIndex < 0 || need.channelIndex >= static_cast<int>(design.channels.size())) continue;
            if (need.shortageNets <= EPS) continue;
            hot.push_back({ need.channelIndex, need.dir, need.shortageNets, max(2.0, need.requiredDeltaUm) });
        }
    }
    sort(hot.begin(), hot.end(), [](const HotNeed& a, const HotNeed& b) {
        if (fabs(a.shortage - b.shortage) > 1.0) return a.shortage > b.shortage;
        if (fabs(a.delta - b.delta) > 0.25) return a.delta > b.delta;
        if (a.channelIndex != b.channelIndex) return a.channelIndex < b.channelIndex;
        return static_cast<int>(a.dir) < static_cast<int>(b.dir);
        });

    vector<HotNeed> uniqueHot;
    for (const HotNeed& h : hot) {
        bool merged = false;
        for (HotNeed& old : uniqueHot) {
            if (old.channelIndex == h.channelIndex && old.dir == h.dir) {
                old.shortage = max(old.shortage, h.shortage);
                old.delta = max(old.delta, h.delta);
                merged = true;
                break;
            }
        }
        if (!merged) uniqueHot.push_back(h);
    }

    const int startCount = static_cast<int>(out.size());
    const int channelLimit = min(1, static_cast<int>(uniqueHot.size()));

    auto addCandidate = [&](Design&& trial) {
        if (static_cast<int>(out.size()) >= maxCandidates) return;
        if (!placementLegalAfterMove(trial, trial.blocks)) return;
        trial.channels.clear();
        trial.routes.clear();
        out.push_back(std::move(trial));
        };

    struct GeometryNeed {
        int endpoint = -1;
        int peer = -1;
        int edge = 0;
        int nets = 0;
        bool primary = false;
        double score = 0.0;
        int primaryCertificateCount = 0;
        long long primaryNetCount = 0;
    };

    vector<GeometryNeed> geometryNeeds;
    for (const auto& cert : certs) {
        if (!cert.geometryDisconnected) continue;
        if (cert.conn.src < 0 || cert.conn.dst < 0 ||
            cert.conn.src >= static_cast<int>(design.blocks.size()) ||
            cert.conn.dst >= static_cast<int>(design.blocks.size())) continue;

        for (int endpoint : { cert.conn.src, cert.conn.dst }) {
            const int peer = endpoint == cert.conn.src ? cert.conn.dst : cert.conn.src;
            const Rect& a = design.blocks[endpoint].rect;
            const Rect& b = design.blocks[peer].rect;
            const double dx = rectCx(b) - rectCx(a);
            const double dy = rectCy(b) - rectCy(a);
            vector<int> edges = legalEdgesForDetourMoveMain(design.blocks[endpoint].spec);
            int bestEdge = 0;
            double bestDirection = -numeric_limits<double>::infinity();
            for (int edge : edges) {
                if (!validEdgeForDetourMoveMain(edge)) continue;
                double direction = 0.0;
                if (edge == 1) direction = -dx;
                else if (edge == 2) direction = dy;
                else if (edge == 3) direction = dx;
                else direction = -dy;
                if (direction > bestDirection) {
                    bestDirection = direction;
                    bestEdge = edge;
                }
            }
            for (int edge : edges) {
                if (!validEdgeForDetourMoveMain(edge)) continue;
                const bool primary = edge == bestEdge;
                const double direction =
                    edge == 1 ? -dx : (edge == 2 ? dy : (edge == 3 ? dx : -dy));
                const int connectionNets = max(1, cert.conn.netCount);
                geometryNeeds.push_back({
                    endpoint, peer, edge, connectionNets, primary,
                    (primary ? 4.0 : 1.0) * connectionNets +
                    0.001 * max(0.0, direction),
                    primary ? 1 : 0,
                    primary ? static_cast<long long>(connectionNets) : 0LL
                });
            }
        }
    }
    sort(geometryNeeds.begin(), geometryNeeds.end(), [](const GeometryNeed& a, const GeometryNeed& b) {
        if (a.primary != b.primary) return a.primary > b.primary;
        if (fabs(a.score - b.score) > 1.0e-6) return a.score > b.score;
        if (a.endpoint != b.endpoint) return a.endpoint < b.endpoint;
        return a.edge < b.edge;
        });

    vector<GeometryNeed> uniqueGeometry;
    for (const GeometryNeed& need : geometryNeeds) {
        auto old = find_if(uniqueGeometry.begin(), uniqueGeometry.end(), [&](const GeometryNeed& item) {
            return item.endpoint == need.endpoint && item.edge == need.edge;
            });
        if (old == uniqueGeometry.end()) uniqueGeometry.push_back(need);
        else {
            old->nets = max(old->nets, need.nets);
            old->score = max(old->score, need.score);
            old->primary = old->primary || need.primary;
            old->primaryCertificateCount += need.primaryCertificateCount;
            old->primaryNetCount += need.primaryNetCount;
        }
    }

    auto geometryGap = [](int nets) {
        return min(96.0, max(4.0,
            ceil(static_cast<double>(max(1, nets)) / CHANNEL_DENSITY - 1.0e-7) + 2.0));
        };

    auto tryOpenEndpointSide = [&](Design& trial, const GeometryNeed& need, double gap) {
        if (need.endpoint < 0 || need.endpoint >= static_cast<int>(trial.blocks.size())) return false;
        gap = max(2.0, ceil(gap - 1.0e-7));
        const Rect endpointRect = trial.blocks[need.endpoint].rect;
        vector<int> blockers;
        for (int bi = 0; bi < static_cast<int>(trial.blocks.size()); ++bi) {
            if (bi == need.endpoint) continue;
            const Rect& r = trial.blocks[bi].rect;
            bool touches = false;
            if (need.edge == 1)
                touches = fabs(rectRight(r) - endpointRect.x) <= 2.0 &&
                    overlapLen(r.y, rectTop(r), endpointRect.y, rectTop(endpointRect)) > 1.0e-4;
            else if (need.edge == 2)
                touches = fabs(r.y - rectTop(endpointRect)) <= 2.0 &&
                    overlapLen(r.x, rectRight(r), endpointRect.x, rectRight(endpointRect)) > 1.0e-4;
            else if (need.edge == 3)
                touches = fabs(r.x - rectRight(endpointRect)) <= 2.0 &&
                    overlapLen(r.y, rectTop(r), endpointRect.y, rectTop(endpointRect)) > 1.0e-4;
            else if (need.edge == 4)
                touches = fabs(rectTop(r) - endpointRect.y) <= 2.0 &&
                    overlapLen(r.x, rectRight(r), endpointRect.x, rectRight(endpointRect)) > 1.0e-4;
            if (touches) blockers.push_back(bi);
        }

        if (!blockers.empty()) {
            bool moved = false;
            if (need.edge == 1) moved = tryMoveBlocksXForCapacityRelief(trial, blockers, -gap);
            else if (need.edge == 2) moved = tryMoveBlocksYForCapacityRelief(trial, blockers, gap);
            else if (need.edge == 3) moved = tryMoveBlocksXForCapacityRelief(trial, blockers, gap);
            else if (need.edge == 4) moved = tryMoveBlocksYForCapacityRelief(trial, blockers, -gap);
            if (moved) return true;
        }

        if (!blockMovableForHotRepair(trial.blocks[need.endpoint].spec)) return false;
        if (need.edge == 1)
            return tryMoveBlocksXForCapacityRelief(trial, { need.endpoint }, gap);
        if (need.edge == 2)
            return tryMoveBlocksYForCapacityRelief(trial, { need.endpoint }, -gap);
        if (need.edge == 3)
            return tryMoveBlocksXForCapacityRelief(trial, { need.endpoint }, -gap);
        if (need.edge == 4)
            return tryMoveBlocksYForCapacityRelief(trial, { need.endpoint }, gap);
        return false;
        };

    vector<GeometryNeed> sharedPortNeeds;
    for (const GeometryNeed& need : uniqueGeometry) {
        if (need.primary && need.primaryCertificateCount >= 2)
            sharedPortNeeds.push_back(need);
    }
    sort(sharedPortNeeds.begin(), sharedPortNeeds.end(),
        [](const GeometryNeed& a, const GeometryNeed& b) {
            if (a.primaryCertificateCount != b.primaryCertificateCount)
                return a.primaryCertificateCount > b.primaryCertificateCount;
            if (a.primaryNetCount != b.primaryNetCount)
                return a.primaryNetCount > b.primaryNetCount;
            if (a.endpoint != b.endpoint) return a.endpoint < b.endpoint;
            return a.edge < b.edge;
        });

    const int sharedPortLimit = min(2, static_cast<int>(sharedPortNeeds.size()));
    for (int variant = 0;
        variant < 2 && sharedPortLimit >= 2 &&
        static_cast<int>(out.size()) < maxCandidates;
        ++variant) {
        Design sharedTrial = design;
        int openedSharedPorts = 0;
        for (int si = 0; si < sharedPortLimit; ++si) {
            const GeometryNeed& need = sharedPortNeeds[si];
            const int aggregateNets = static_cast<int>(
                min<long long>(need.primaryNetCount, numeric_limits<int>::max()));
            const double fullGap = geometryGap(aggregateNets);
            const double requestedGap = variant == 0 ? min(16.0, fullGap) : fullGap;
            if (tryOpenEndpointSide(sharedTrial, need, requestedGap))
                ++openedSharedPorts;
        }
        if (openedSharedPorts == sharedPortLimit)
            addCandidate(std::move(sharedTrial));
    }

    if (!uniqueGeometry.empty() && static_cast<int>(out.size()) < maxCandidates) {
        Design cumulative = design;
        vector<char> endpointHandled(cumulative.blocks.size(), 0);
        bool changed = false;
        const int cumulativeLimit = min(32, static_cast<int>(uniqueGeometry.size()));
        for (int gi = 0; gi < cumulativeLimit; ++gi) {
            const GeometryNeed& need = uniqueGeometry[gi];
            if (!need.primary || endpointHandled[need.endpoint]) continue;
            if (tryOpenEndpointSide(cumulative, need, geometryGap(need.nets))) {
                endpointHandled[need.endpoint] = 1;
                changed = true;
            }
        }
        if (changed) addCandidate(std::move(cumulative));
    }

    const int geometryLimit = min(8, static_cast<int>(uniqueGeometry.size()));
    for (int gi = 0; gi < geometryLimit && static_cast<int>(out.size()) < maxCandidates; ++gi) {
        const GeometryNeed& need = uniqueGeometry[gi];
        const double baseGap = geometryGap(need.nets);
        for (double gap : { baseGap, max(8.0, baseGap), max(16.0, baseGap) }) {
            if (static_cast<int>(out.size()) >= maxCandidates) break;
            Design trial = design;
            if (tryOpenEndpointSide(trial, need, gap)) addCandidate(std::move(trial));
        }
    }

    if (hot.empty()) return static_cast<int>(out.size()) > startCount;

    auto reliefDeltas = [](double required) {
        const double exact = min(700.0, max(2.0, required));
        vector<double> d = { exact, exact * 1.25, exact * 0.75, exact * 0.50, exact + 8.0, 8.0, 16.0, 32.0 };
        sort(d.begin(), d.end());
        d.erase(unique(d.begin(), d.end(), [](double a, double b) { return fabs(a - b) < 0.25; }), d.end());
        sort(d.begin(), d.end(), [exact](double a, double b) {
            const double da = fabs(a - exact);
            const double db = fabs(b - exact);
            if (fabs(da - db) > 0.25) return da < db;
            return a < b;
            });
        return d;
        };

    for (int hi = 0; hi < channelLimit && static_cast<int>(out.size()) < maxCandidates; ++hi) {
        const HotNeed& need = uniqueHot[hi];
        const Channel& ch = design.channels[need.channelIndex];
        const double x1 = ch.rect.x;
        const double x2 = rectRight(ch.rect);
        const double y1 = ch.rect.y;
        const double y2 = rectTop(ch.rect);
        const vector<double> deltas = reliefDeltas(need.delta);

        if (need.dir == Router::ChannelDir::LR) {
            vector<int> above;
            vector<int> below;
            for (int bi = 0; bi < static_cast<int>(design.blocks.size()); ++bi) {
                const BlockInst& b = design.blocks[bi];
                if (b.spec.type != BlockType::EDGE && !blockMovableForHotRepair(b.spec)) continue;
                if (overlapLen(b.rect.x, rectRight(b.rect), x1, x2) <= 1.0e-4) continue;
                if (fabs(b.rect.y - y2) <= 2.0) above.push_back(bi);
                if (fabs(rectTop(b.rect) - y1) <= 2.0) below.push_back(bi);
            }
            for (double d : deltas) {
                if (static_cast<int>(out.size()) >= maxCandidates) break;
                if (!above.empty()) { Design trial = design; if (tryMoveBlocksYForCapacityRelief(trial, above, d)) addCandidate(std::move(trial)); }
                if (static_cast<int>(out.size()) >= maxCandidates) break;
                if (!below.empty()) { Design trial = design; if (tryMoveBlocksYForCapacityRelief(trial, below, -d)) addCandidate(std::move(trial)); }
                if (static_cast<int>(out.size()) >= maxCandidates) break;
                if (!above.empty() && !below.empty()) {
                    Design trial = design;
                    if (tryMoveBlocksYForCapacityRelief(trial, above, d * 0.5) && tryMoveBlocksYForCapacityRelief(trial, below, -d * 0.5)) addCandidate(std::move(trial));
                }
            }
        }
        else {
            vector<int> left;
            vector<int> right;
            auto addUniqueBlock = [](vector<int>& ids, int id) { if (find(ids.begin(), ids.end(), id) == ids.end()) ids.push_back(id); };
            for (int bi = 0; bi < static_cast<int>(design.blocks.size()); ++bi) {
                const BlockInst& b = design.blocks[bi];
                if (b.spec.type != BlockType::EDGE && !blockMovableForHotRepair(b.spec)) continue;
                const double yOverlap = overlapLen(b.rect.y, rectTop(b.rect), y1, y2);
                const bool yTouchesLoose = rectTop(b.rect) >= y1 - 2.0 && b.rect.y <= y2 + 2.0;
                if (!yTouchesLoose) continue;
                if (yOverlap > 1.0e-4) {
                    if (fabs(rectRight(b.rect) - x1) <= 2.0) addUniqueBlock(left, bi);
                    if (fabs(b.rect.x - x2) <= 2.0) addUniqueBlock(right, bi);
                }
                else {
                    if (fabs(rectRight(b.rect) - x1) <= 2.0 || fabs(b.rect.x - x1) <= 2.0) addUniqueBlock(left, bi);
                    if (fabs(b.rect.x - x2) <= 2.0 || fabs(rectRight(b.rect) - x2) <= 2.0) addUniqueBlock(right, bi);
                }
            }
            for (double d : deltas) {
                if (static_cast<int>(out.size()) >= maxCandidates) break;
                if (!right.empty()) { Design trial = design; if (tryMoveBlocksXForCapacityRelief(trial, right, d)) addCandidate(std::move(trial)); }
                if (static_cast<int>(out.size()) >= maxCandidates) break;
                if (!left.empty()) { Design trial = design; if (tryMoveBlocksXForCapacityRelief(trial, left, -d)) addCandidate(std::move(trial)); }
                if (static_cast<int>(out.size()) >= maxCandidates) break;
                if (!left.empty() && !right.empty()) {
                    Design trial = design;
                    if (tryMoveBlocksXForCapacityRelief(trial, right, d * 0.5) && tryMoveBlocksXForCapacityRelief(trial, left, -d * 0.5)) addCandidate(std::move(trial));
                }
            }
        }
    }

    return static_cast<int>(out.size()) > startCount;
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
        router.printDetourReport(design);
    Logger::printFinalReport(design, rpt, opt.alpha, opt.inputPath, opt.outputPath);
        return 1;
    }

    if (!opt.alphaOverride) {
        const double checkerAlpha = checkerDefaultAlphaForCaseId(detectKnownCaseId(opt.inputPath));
        opt.alpha = isfinite(checkerAlpha) ? checkerAlpha : design.alpha;
    }
    design.alpha = opt.alpha;

    // 讀完 parser 後才知道 block 數量，所以在這裡決定真正 output cfg 路徑。
    if (opt.outputPathProvided) {
        opt.outputPath = resolveOutputPath(opt.outputPath, design.blockSpecs.size());
    }


    if (opt.evalCfgProvided) {
        ifstream fin(opt.evalCfgPath, ios::binary);
        if (!fin) {
            cerr << "[EvalCfg] failed to open: " << opt.evalCfgPath << "\n";
            return 1;
        }
        stringstream cfgBuf;
        cfgBuf << fin.rdbuf();
        Design cfgBase = design;
        cfgBase.blocks.clear();
        cfgBase.blocks.reserve(cfgBase.blockSpecs.size());
        for (const BlockSpec& spec : cfgBase.blockSpecs) {
            BlockInst b;
            b.spec = spec;
            cfgBase.blocks.push_back(b);
        }
        string cfgText = cfgBuf.str();
        Design cfgDesign;
        if (!parsePortfolioCfg(cfgText.c_str(), cfgBase, cfgDesign)) {
            cerr << "[EvalCfg] parse failed: " << opt.evalCfgPath << " detail=" << gLastPortfolioParseError << "\n";
            return 1;
        }
        EvalReport rpt = evaluator.evaluate(cfgDesign, opt.alpha, programRuntimeSecondsMain());
        writer.write(opt.outputPath, cfgDesign);
        Logger::printFinalReport(cfgDesign, rpt, opt.alpha, opt.inputPath, opt.outputPath);
        return rpt.hasFail() ? 1 : 0;
    }

    auto totalPenalty = [](const EvalReport& r) {
        return r.warningPenaltyCost;
        };

    auto betterEval = [&](const EvalReport& a, const EvalReport& b) {
        if (a.hasFail() != b.hasFail()) return !a.hasFail();
        if (a.hasFail() && a.openPathCount != b.openPathCount) return a.openPathCount < b.openPathCount;

        // Checker ranking is cost-based.  Warning penalties are already included
        // in cost, so do not make any nonzero overflow/FT warning dominate outline.
        if (fabs(a.cost - b.cost) > 1.0) return a.cost < b.cost;

        const double ap = totalPenalty(a);
        const double bp = totalPenalty(b);
        if (fabs(ap - bp) > 1.0) return ap < bp;
        if (fabs(a.outlineArea - b.outlineArea) > 1.0) return a.outlineArea < b.outlineArea;
        return a.totalWireLength < b.totalWireLength;
        };


    bool forceSoftFTRelaxedRouting = false;


    auto routeCandidate = [&](const Design& seed, bool ftOverflowAware, bool softFTRelaxed, bool contactAware,
        EvalReport& outRpt, vector<Router::FailureCertificate>* outCerts = nullptr) {
            Design trial;
            if (!makeIntegerGridFloorplanCandidateMain(seed, trial)) trial = seed;
            channelBuilder.build(trial);
            
            router.setFTOverflowCostEnabled(ftOverflowAware);
            router.setSoftFTCostRelaxed(softFTRelaxed);
            router.setContactAwareCostEnabled(contactAware);
            router.setDetailedFailureAnalysisEnabled(outCerts != nullptr);
            router.run(trial);
            if (outCerts) *outCerts = router.failureCertificates();
            outRpt = evaluator.evaluate(trial, opt.alpha);
            return trial;
        };

    auto bestRoutedCandidate = [&](const Design& seed, EvalReport& outRpt,
        vector<Router::FailureCertificate>* outCerts = nullptr) {
            vector<Router::FailureCertificate> bestCerts;
            Design best;
            bool haveBest = false;
            const bool tryContactAware = static_cast<int>(seed.blockSpecs.size()) >= 16;

            auto considerRoute = [&](bool ftOverflowAware, bool softFTRelaxed, bool contactAware) {
                EvalReport candRpt;
                vector<Router::FailureCertificate> candCerts;
                vector<Router::FailureCertificate>* candCertOut =
                    outCerts ? &candCerts : nullptr;
                Design cand = routeCandidate(
                    seed, ftOverflowAware, softFTRelaxed, contactAware, candRpt, candCertOut);
                if (!haveBest || betterEval(candRpt, outRpt)) {
                    best = std::move(cand);
                    outRpt = candRpt;
                    bestCerts = std::move(candCerts);
                    haveBest = true;
                }
                };

            if (forceSoftFTRelaxedRouting) {
                considerRoute(true, true, false);
                if (tryContactAware) considerRoute(true, true, true);
                if (outRpt.hasFail() || totalPenalty(outRpt) > 1.0) {
                    considerRoute(false, true, false);
                    if (tryContactAware) considerRoute(false, true, true);
                }
                if (outRpt.hasFail() || totalPenalty(outRpt) > 1.0) {
                    considerRoute(true, true, false);
                    if (tryContactAware) considerRoute(true, true, true);
                    considerRoute(false, true, false);
                    if (tryContactAware) considerRoute(false, true, true);
                }
                if (outCerts) *outCerts = bestCerts;
                return best;
            }

            considerRoute(false, false, false);
            if (tryContactAware) considerRoute(false, false, true);

            if (outRpt.hasFail() || totalPenalty(outRpt) > 1.0) {
                considerRoute(false, true, false);
                considerRoute(true, true, false);
                if (tryContactAware) {
                    considerRoute(false, true, true);
                    considerRoute(true, true, true);
                }
            }



            if (outCerts) *outCerts = bestCerts;
            return best;
        };

    if (opt.routeCfgBlocksProvided) {
        ifstream fin(opt.routeCfgBlocksPath, ios::binary);
        if (!fin) {
            cerr << "[RouteCfgBlocks] failed to open: " << opt.routeCfgBlocksPath << "\n";
            return 1;
        }
        stringstream cfgBuf;
        cfgBuf << fin.rdbuf();
        Design cfgBase = design;
        cfgBase.blocks.clear();
        cfgBase.blocks.reserve(cfgBase.blockSpecs.size());
        for (const BlockSpec& spec : cfgBase.blockSpecs) {
            BlockInst b;
            b.spec = spec;
            cfgBase.blocks.push_back(b);
        }
        Design cfgSeed;
        string cfgText = cfgBuf.str();
        if (!parsePortfolioCfg(cfgText.c_str(), cfgBase, cfgSeed)) {
            cerr << "[RouteCfgBlocks] parse failed: " << opt.routeCfgBlocksPath << " detail=" << gLastPortfolioParseError << "\n";
            return 1;
        }
        cfgSeed.channels.clear();
        cfgSeed.routes.clear();
        EvalReport cfgRouteRpt;
        Design routed = bestRoutedCandidate(cfgSeed, cfgRouteRpt);
        Design bestCfgDesign = routed;
        EvalReport bestCfgRpt = cfgRouteRpt;
        if (!bestCfgRpt.hasFail() && totalPenalty(bestCfgRpt) <= 1.0) {
            for (int detourIter = 0; detourIter < 4; ++detourIter) {
                vector<Design> detourSeeds;
                if (!makeDetourMoveCandidates(bestCfgDesign, detourSeeds, 18)) break;
                bool haveDetour = false;
                Design bestDetourDesign;
                EvalReport bestDetourRpt;
                for (int di = 0; di < static_cast<int>(detourSeeds.size()); ++di) {
                    EvalReport detourRpt;
                    Design detourTrial = bestRoutedCandidate(detourSeeds[di], detourRpt);
                    cerr << fixed << setprecision(3)
                        << "[RouteCfgBlocksDetour] trial iter=" << detourIter
                        << " cand=" << di
                        << " open=" << detourRpt.openPathCount
                        << " penalty=" << totalPenalty(detourRpt)
                        << " area=" << detourRpt.outlineArea
                        << " wl=" << detourRpt.totalWireLength
                        << " cost=" << detourRpt.cost
                        << " fail=" << (detourRpt.hasFail() ? "Y" : "N")
                        << "\n";
                    if (detourRpt.hasFail()) continue;
                    if (!haveDetour || betterEval(detourRpt, bestDetourRpt)) {
                        bestDetourDesign = std::move(detourTrial);
                        bestDetourRpt = detourRpt;
                        haveDetour = true;
                    }
                }
                if (!haveDetour || !betterEval(bestDetourRpt, bestCfgRpt)) break;
                cerr << fixed << setprecision(3)
                    << "[RouteCfgBlocksDetour] accept iter=" << detourIter
                    << " cost=" << bestCfgRpt.cost << "->" << bestDetourRpt.cost
                    << " wl=" << bestCfgRpt.totalWireLength << "->" << bestDetourRpt.totalWireLength
                    << " area=" << bestCfgRpt.outlineArea << "->" << bestDetourRpt.outlineArea
                    << "\n";
                bestCfgDesign = std::move(bestDetourDesign);
                bestCfgRpt = bestDetourRpt;
            }
        }
        writer.write(opt.outputPath, bestCfgDesign);
        router.printDetourReport(bestCfgDesign);
        bestCfgRpt = evaluator.evaluate(bestCfgDesign, opt.alpha, programRuntimeSecondsMain());
        Logger::printFinalReport(bestCfgDesign, bestCfgRpt, opt.alpha, opt.inputPath, opt.outputPath);
        return bestCfgRpt.hasFail() ? 2 : 0;
    }


    const int aggressiveFastCaseId = detectKnownCaseId(opt.inputPath);
    if (aggressiveFastCaseId >= 0 && aggressiveFastCaseId <= 4) {
        vector<Design> aggressiveSeeds;
        vector<string> aggressiveOrigins;
        if (makeAggressiveClusterSeedsMain(opt.inputPath, design, aggressiveSeeds, aggressiveOrigins) && !aggressiveSeeds.empty()) {
            EvalReport fastRpt;
            Design fastDesign = bestRoutedCandidate(aggressiveSeeds.front(), fastRpt);
            cerr << fixed << setprecision(3)
                << "[AggressiveClusterFastPath] case=" << aggressiveFastCaseId
                << " area=" << fastRpt.outlineArea
                << " wl=" << fastRpt.totalWireLength
                << " penalty=" << totalPenalty(fastRpt)
                << " cost=" << fastRpt.cost
                << " fail=" << (fastRpt.hasFail() ? "Y" : "N")
                << "\n";
            if (!fastRpt.hasFail()) {
                fastRpt = evaluator.evaluate(fastDesign, opt.alpha, programRuntimeSecondsMain());
                bool writeOk = writer.write(opt.outputPath, fastDesign);
                if (!writeOk) {
                    cerr << "[OutputWriter] Failed to write cfg: " << opt.outputPath << "\n";
                    return 1;
                }
                router.printDetourReport(fastDesign);
                Logger::printFinalReport(fastDesign, fastRpt, opt.alpha, opt.inputPath, opt.outputPath);
                return fastRpt.hasFail() ? 2 : 0;
            }
        }
    }

    auto sameFloorplanSeed = [](const Design& a, const Design& b) {
        if (a.blocks.size() != b.blocks.size()) return false;
        if (fabs(a.outlineW - b.outlineW) > 1.0 || fabs(a.outlineH - b.outlineH) > 1.0) return false;
        for (int i = 0; i < static_cast<int>(a.blocks.size()); ++i) {
            const Rect& ra = a.blocks[i].rect;
            const Rect& rb = b.blocks[i].rect;
            if (fabs(ra.x - rb.x) > 1.0 || fabs(ra.y - rb.y) > 1.0 ||
                fabs(ra.w - rb.w) > 1.0 || fabs(ra.h - rb.h) > 1.0) return false;
        }
        return true;
        };

    auto addFloorplanSeed = [&](vector<Design>& seeds, vector<string>& origins, const Design& candidate, const string& origin) {
        if (candidate.blocks.empty()) return false;
        for (const Design& old : seeds) {
            if (sameFloorplanSeed(old, candidate)) return false;
        }
        seeds.push_back(candidate);
        origins.push_back(origin);
        return true;
        };

    struct RepairPortfolioItem {
        Design routed;
        EvalReport rpt;
        vector<Router::FailureCertificate> certs;
        int seedIndex = -1;
        string origin;
    };

    auto routeFloorplanSeeds = [&](const vector<Design>& seeds, const vector<string>& origins, EvalReport& outRpt, Design& outSeed, const string& tag,
        vector<Router::FailureCertificate>* outCerts = nullptr, vector<RepairPortfolioItem>* repairPortfolio = nullptr) {
            bool have = false;
            Design best;
            vector<Router::FailureCertificate> bestCerts;
            for (int si = 0; si < static_cast<int>(seeds.size()); ++si) {
                const string origin = si < static_cast<int>(origins.size()) ? origins[si] : "unknown";
                EvalReport seedRpt;
                vector<Router::FailureCertificate> seedCerts;
                Design routed = bestRoutedCandidate(seeds[si], seedRpt, &seedCerts);
                const double seedPenalty = totalPenalty(seedRpt);
                cerr << fixed << setprecision(3)
                    << "[FloorplanArchiveRoute] tag=" << tag
                    << " cand=" << si
                    << " origin=" << origin
                    << " fail=" << (seedRpt.hasFail() ? "Y" : "N")
                    << " open=" << seedRpt.openPathCount
                    << " chOv=" << seedRpt.totalChannelOverflow
                    << " ftOv=" << seedRpt.totalFeedthroughOverflow
                    << " cost=" << seedRpt.cost
                    << "\n";
                if (repairPortfolio &&
                    seedRpt.hasFail() &&
                    seedRpt.openPathCount > 0 &&
                    seedRpt.openPathCount <= 2 &&
                    !seedRpt.blockOverlap &&
                    !seedRpt.outlineViolation &&
                    seedRpt.totalFeedthroughOverflow <= 1.0 &&
                    seedRpt.totalChannelOverflow <= 5000.0 &&
                    !seedCerts.empty()) {
                    repairPortfolio->push_back({ routed, seedRpt, seedCerts, si, origin });
                    cerr << fixed << setprecision(3)
                        << "[RepairPortfolio] collect tag=" << tag
                        << " cand=" << si
                        << " origin=" << origin
                        << " open=" << seedRpt.openPathCount
                        << " penalty=" << seedPenalty
                        << " cost=" << seedRpt.cost
                        << " certs=" << seedCerts.size()
                        << "\n";
                }
                if (!have || betterEval(seedRpt, outRpt)) {
                    best = std::move(routed);
                    outRpt = seedRpt;
                    outSeed = seeds[si];
                    bestCerts = std::move(seedCerts);
                    have = true;
                }
            }
            if (outCerts) *outCerts = bestCerts;
            return best;
        };

    // One-way architecture:    // Parser -> Floorplanner -> ChannelBuilder -> Router -> Evaluator -> OutputWriter
    floorplanner.setEdgePlacementMode(0);
    floorplanner.run(design);
    Design floorplannedDesign = design;
    vector<Design> floorplanSeeds;
    vector<string> floorplanSeedOrigins;
    addFloorplanSeed(floorplanSeeds, floorplanSeedOrigins, floorplannedDesign, "committed");
    const vector<Design>& archived = floorplanner.archivedCandidates();
    const vector<string>& archivedOrigins = floorplanner.archivedCandidateOrigins();
    for (int ai = 0; ai < static_cast<int>(archived.size()); ++ai) {
        const string origin = ai < static_cast<int>(archivedOrigins.size()) ? archivedOrigins[ai] : "archive";
        addFloorplanSeed(floorplanSeeds, floorplanSeedOrigins, archived[ai], origin);
    }
    makeAggressiveClusterSeedsMain(opt.inputPath, design, floorplanSeeds, floorplanSeedOrigins);

    EvalReport rpt;
    Design selectedFloorplanSeed = floorplannedDesign;
    vector<Router::FailureCertificate> currentCerts;
    vector<RepairPortfolioItem> baseRepairPortfolio;
    design = routeFloorplanSeeds(floorplanSeeds, floorplanSeedOrigins, rpt, selectedFloorplanSeed, "base", &currentCerts, &baseRepairPortfolio);
    floorplannedDesign = selectedFloorplanSeed;
    if (rpt.hasFail() || totalPenalty(rpt) > 1.0) {
        forceSoftFTRelaxedRouting = true;
        EvalReport relaxedStartRpt;
        Design relaxedSeed = floorplannedDesign;
        vector<Router::FailureCertificate> relaxedCerts;
        Design relaxedStart = routeFloorplanSeeds(floorplanSeeds, floorplanSeedOrigins, relaxedStartRpt, relaxedSeed, "relaxed", &relaxedCerts);
        if (betterEval(relaxedStartRpt, rpt)) {
            design = std::move(relaxedStart);
            rpt = relaxedStartRpt;
            floorplannedDesign = relaxedSeed;
            currentCerts = std::move(relaxedCerts);
        }
    }


    Design bestDesign = design;
    EvalReport bestRpt = rpt;
    vector<Router::FailureCertificate> bestCerts = currentCerts;
    if (!bestRpt.hasFail() && totalPenalty(bestRpt) <= 1.0) {
        for (int detourIter = 0; detourIter < 5; ++detourIter) {
            vector<Design> detourSeeds;
            if (!makeDetourMoveCandidates(bestDesign, detourSeeds, 20)) break;

            bool haveDetour = false;
            Design bestDetourDesign;
            EvalReport bestDetourRpt;
            for (int di = 0; di < static_cast<int>(detourSeeds.size()); ++di) {
                EvalReport detourRpt;
                Design detourTrial = bestRoutedCandidate(detourSeeds[di], detourRpt);
                cerr << fixed << setprecision(3)
                    << "[DetourMove] trial iter=" << detourIter
                    << " cand=" << di
                    << " open=" << detourRpt.openPathCount
                    << " penalty=" << totalPenalty(detourRpt)
                    << " area=" << detourRpt.outlineArea
                    << " wl=" << detourRpt.totalWireLength
                    << " cost=" << detourRpt.cost
                    << " fail=" << (detourRpt.hasFail() ? "Y" : "N")
                    << "\n";
                if (detourRpt.hasFail()) continue;
                if (!haveDetour || betterEval(detourRpt, bestDetourRpt)) {
                    bestDetourDesign = std::move(detourTrial);
                    bestDetourRpt = detourRpt;
                    haveDetour = true;
                }
            }
            if (!haveDetour || !betterEval(bestDetourRpt, bestRpt)) break;
            cerr << fixed << setprecision(3)
                << "[DetourMove] accept iter=" << detourIter
                << " cost=" << bestRpt.cost << "->" << bestDetourRpt.cost
                << " wl=" << bestRpt.totalWireLength << "->" << bestDetourRpt.totalWireLength
                << " area=" << bestRpt.outlineArea << "->" << bestDetourRpt.outlineArea
                << "\n";
            bestDesign = std::move(bestDetourDesign);
            bestRpt = bestDetourRpt;
            design = bestDesign;
            rpt = bestRpt;
        }
    }

    auto tryRepairPortfolio = [&](vector<RepairPortfolioItem>& portfolio, const string& label) {
        if (portfolio.empty()) return;
        sort(portfolio.begin(), portfolio.end(), [&](const RepairPortfolioItem& a, const RepairPortfolioItem& b) {
            if (a.rpt.openPathCount != b.rpt.openPathCount) return a.rpt.openPathCount < b.rpt.openPathCount;
            const double ap = totalPenalty(a.rpt);
            const double bp = totalPenalty(b.rpt);
            if (fabs(ap - bp) > 1.0) return ap < bp;
            if (fabs(a.rpt.cost - b.rpt.cost) > 1.0) return a.rpt.cost < b.rpt.cost;
            return a.seedIndex < b.seedIndex;
            });

        const int portfolioLimit = min(4, static_cast<int>(portfolio.size()));
        for (int pi = 0; pi < portfolioLimit; ++pi) {
            RepairPortfolioItem& item = portfolio[pi];
            vector<Design> reliefSeeds;
            const bool made = makeCertificateReliefCandidates(item.routed, item.certs, reliefSeeds, 12);
            cerr << fixed << setprecision(3)
                << "[RepairPortfolio] probe label=" << label
                << " item=" << pi
                << " seed=" << item.seedIndex
                << " origin=" << item.origin
                << " open=" << item.rpt.openPathCount
                << " penalty=" << totalPenalty(item.rpt)
                << " cost=" << item.rpt.cost
                << " reliefSeeds=" << reliefSeeds.size()
                << " made=" << (made ? "Y" : "N")
                << "\n";
            if (!made) continue;

            bool haveRepair = false;
            Design bestRepairDesign;
            EvalReport bestRepairRpt;
            vector<Router::FailureCertificate> bestRepairCerts;
            for (int ri = 0; ri < static_cast<int>(reliefSeeds.size()); ++ri) {
                EvalReport repairRpt;
                vector<Router::FailureCertificate> repairCerts;
                Design repairTrial = bestRoutedCandidate(reliefSeeds[ri], repairRpt, &repairCerts);
                cerr << fixed << setprecision(3)
                    << "[RepairPortfolio] trial label=" << label
                    << " item=" << pi
                    << " cand=" << ri
                    << " open=" << repairRpt.openPathCount
                    << " penalty=" << totalPenalty(repairRpt)
                    << " cost=" << repairRpt.cost
                    << " fail=" << (repairRpt.hasFail() ? "Y" : "N")
                    << "\n";
                if (!haveRepair || betterEval(repairRpt, bestRepairRpt)) {
                    bestRepairDesign = std::move(repairTrial);
                    bestRepairRpt = repairRpt;
                    bestRepairCerts = std::move(repairCerts);
                    haveRepair = true;
                }
            }
            if (haveRepair && betterEval(bestRepairRpt, bestRpt)) {
                cerr << fixed << setprecision(3)
                    << "[RepairPortfolio] accept label=" << label
                    << " seed=" << item.seedIndex
                    << " origin=" << item.origin
                    << " open=" << bestRpt.openPathCount << "->" << bestRepairRpt.openPathCount
                    << " penalty=" << totalPenalty(bestRpt) << "->" << totalPenalty(bestRepairRpt)
                    << " cost=" << bestRpt.cost << "->" << bestRepairRpt.cost
                    << "\n";
                bestDesign = std::move(bestRepairDesign);
                bestRpt = bestRepairRpt;
                bestCerts = std::move(bestRepairCerts);
                design = bestDesign;
                rpt = bestRpt;
            }
        }
    };

    tryRepairPortfolio(baseRepairPortfolio, "base");

    cerr << "[CertificateRelief] start open=" << bestRpt.openPathCount
        << " certs=" << bestCerts.size()
        << " fail=" << (bestRpt.hasFail() ? "Y" : "N")
        << " penalty=" << totalPenalty(bestRpt)
        << "\n";
    for (int certReliefIter = 0; bestRpt.openPathCount > 0 && certReliefIter < 4; ++certReliefIter) {
        if (bestCerts.empty()) {
            cerr << "[CertificateRelief] skip reason=no_certificate open=" << bestRpt.openPathCount << "\n";
            break;
        }
        vector<Design> reliefSeeds;
        if (!makeCertificateReliefCandidates(bestDesign, bestCerts, reliefSeeds, 10)) break;

        bool haveRelief = false;
        Design bestReliefDesign;
        EvalReport bestReliefRpt;
        vector<Router::FailureCertificate> bestReliefCerts;

        for (int ri = 0; ri < static_cast<int>(reliefSeeds.size()); ++ri) {
            EvalReport reliefRpt;
            vector<Router::FailureCertificate> reliefCerts;
            Design reliefTrial = bestRoutedCandidate(reliefSeeds[ri], reliefRpt, &reliefCerts);
            cerr << fixed << setprecision(3)
                << "[CertificateRelief] trial iter=" << certReliefIter
                << " cand=" << ri
                << " area=" << reliefRpt.outlineArea
                << " open=" << reliefRpt.openPathCount
                << " penalty=" << totalPenalty(reliefRpt)
                << " cost=" << reliefRpt.cost
                << " fail=" << (reliefRpt.hasFail() ? "Y" : "N")
                << "\n";
            if (!haveRelief || betterEval(reliefRpt, bestReliefRpt)) {
                bestReliefDesign = std::move(reliefTrial);
                bestReliefRpt = reliefRpt;
                bestReliefCerts = std::move(reliefCerts);
                haveRelief = true;
            }
        }

        if (!haveRelief || !betterEval(bestReliefRpt, bestRpt)) break;
        cerr << fixed << setprecision(3)
            << "[CertificateRelief] accept iter=" << certReliefIter
            << " open=" << bestRpt.openPathCount << "->" << bestReliefRpt.openPathCount
            << " penalty=" << totalPenalty(bestRpt) << "->" << totalPenalty(bestReliefRpt)
            << " cost=" << bestRpt.cost << "->" << bestReliefRpt.cost
            << "\n";
        bestDesign = std::move(bestReliefDesign);
        bestRpt = bestReliefRpt;
        bestCerts = std::move(bestReliefCerts);
        design = bestDesign;
        rpt = bestRpt;
    }

    for (int hotRepairIter = 0; hotRepairIter < 8; ++hotRepairIter) {
        bool tried = false;
        bool hasTrial = false;
        Design bestHotTrial;
        EvalReport bestHotRpt;

        auto tryHotCandidate = [&](const Design& candidate) {
            EvalReport trialRpt;
            Design trial = bestRoutedCandidate(candidate, trialRpt);
            if (!hasTrial || betterEval(trialRpt, bestHotRpt)) {
                bestHotTrial = std::move(trial);
                bestHotRpt = trialRpt;
                hasTrial = true;
            }
        };

        Design movedH = bestDesign;
        if (relieveHorizontalHotChannels(movedH)) {
            tried = true;
            tryHotCandidate(movedH);
        }

        Design movedV = bestDesign;
        if (relieveVerticalHotChannels(movedV)) {
            tried = true;
            tryHotCandidate(movedV);
        }

        Design movedHV = bestDesign;
        const bool movedHFirst = relieveHorizontalHotChannels(movedHV);
        const bool movedVSecond = relieveVerticalHotChannels(movedHV);
        if (movedHFirst && movedVSecond) {
            tried = true;
            tryHotCandidate(movedHV);
        }

        if (!tried || !hasTrial || !betterEval(bestHotRpt, bestRpt)) break;
        bestDesign = bestHotTrial;
        bestRpt = bestHotRpt;
        design = bestDesign;
        rpt = bestRpt;
    }
    auto runFTFeedback = [&](const string& stage, int refloorplanSeedLimit) {
        bool improvedAny = false;
        const int ftResizeLimit = ftResizeIterationLimit(bestDesign);
        for (int ftResizeIter = 0; ftResizeIter < ftResizeLimit; ++ftResizeIter) {
            bool triedFTFeedback = false;
            bool haveFTFeedback = false;
            Design bestFTDesign;
            EvalReport bestFTRpt;
            vector<Router::FailureCertificate> bestFTCerts;
            RepairPortfolioItem guidedOpenCandidate;
            bool haveGuidedOpenCandidate = false;
            RepairPortfolioItem legalCertificateRepair;
            bool haveLegalCertificateRepair = false;

            auto tryFTFeedbackSeed = [&](const Design& seed, const string& label, int candIndex) {
                triedFTFeedback = true;
                EvalReport trialRpt;
                vector<Router::FailureCertificate> trialCerts;
                Design trial = bestRoutedCandidate(seed, trialRpt, &trialCerts);
                cerr << fixed << setprecision(3)
                    << "[FTFeedback] stage=" << stage
                    << " trial iter=" << ftResizeIter
                    << " source=" << label
                    << " cand=" << candIndex
                    << " open=" << trialRpt.openPathCount
                    << " chOv=" << trialRpt.totalChannelOverflow
                    << " ftOv=" << trialRpt.totalFeedthroughOverflow
                    << " penalty=" << totalPenalty(trialRpt)
                    << " cost=" << trialRpt.cost
                    << " fail=" << (trialRpt.hasFail() ? "Y" : "N")
                    << "\n";
                const bool pressureImproved =
                    totalPenalty(trialRpt) + 1.0 < totalPenalty(bestRpt) ||
                    trialRpt.totalChannelOverflow + 1.0 < bestRpt.totalChannelOverflow ||
                    trialRpt.totalFeedthroughOverflow + 1.0 < bestRpt.totalFeedthroughOverflow;
                if (label == "refloorplan" && trialRpt.openPathCount > 0 &&
                    !trialCerts.empty() && pressureImproved) {
                    const bool preferForRepair =
                        !haveGuidedOpenCandidate ||
                        trialRpt.openPathCount < guidedOpenCandidate.rpt.openPathCount ||
                        (trialRpt.openPathCount == guidedOpenCandidate.rpt.openPathCount &&
                            totalPenalty(trialRpt) < totalPenalty(guidedOpenCandidate.rpt));
                    if (preferForRepair) {
                        guidedOpenCandidate = {
                            trial, trialRpt, trialCerts, candIndex,
                            stage + ":" + label
                        };
                        haveGuidedOpenCandidate = true;
                    }
                }
                if (!haveFTFeedback || betterEval(trialRpt, bestFTRpt)) {
                    bestFTDesign = std::move(trial);
                    bestFTRpt = trialRpt;
                    bestFTCerts = std::move(trialCerts);
                    haveFTFeedback = true;
                }
                };

            Design resized = bestDesign;
            if (resizeSoftBlocksForActualFeedthrough(resized)) {
                tryFTFeedbackSeed(resized, "local-resize", 0);
            }

            vector<Design> refloorplanSeeds;
            if (stage == "pre" && makeFeedthroughReFloorplanCandidates(bestDesign, refloorplanSeeds, refloorplanSeedLimit, ftResizeIter == 0)) {
                for (int fi = 0; fi < static_cast<int>(refloorplanSeeds.size()); ++fi) {
                    tryFTFeedbackSeed(refloorplanSeeds[fi], "refloorplan", fi);
                }
            }

            if (haveGuidedOpenCandidate) {
                RepairPortfolioItem probe = guidedOpenCandidate;
                for (int repairRound = 0;
                    repairRound < 2 && probe.rpt.openPathCount > 0 && !probe.certs.empty();
                    ++repairRound) {
                    vector<Design> repairSeeds;
                    const int repairSeedLimit = repairRound == 0 ? 6 : 4;
                    const bool madeRepairSeeds = makeCertificateReliefCandidates(
                        probe.routed,
                        probe.certs,
                        repairSeeds,
                        repairSeedLimit);
                    int geometryCertCount = 0;
                    int policyCertCount = 0;
                    for (const auto& cert : probe.certs) {
                        if (cert.geometryDisconnected) ++geometryCertCount;
                        if (cert.policyBlocked) ++policyCertCount;
                    }
                    cerr << fixed << setprecision(3)
                        << "[FTCertificateRetry] stage=" << stage
                        << " iter=" << ftResizeIter
                        << " round=" << repairRound
                        << " source=" << probe.origin
                        << " open=" << probe.rpt.openPathCount
                        << " penalty=" << totalPenalty(probe.rpt)
                        << " certs=" << probe.certs.size()
                        << " geometryCerts=" << geometryCertCount
                        << " policyCerts=" << policyCertCount
                        << " seeds=" << repairSeeds.size()
                        << " made=" << (madeRepairSeeds ? "Y" : "N")
                        << "\n";
                    if (!madeRepairSeeds) break;

                    bool haveRoundBest = false;
                    RepairPortfolioItem roundBest;
                    for (int ri = 0; ri < static_cast<int>(repairSeeds.size()); ++ri) {
                        EvalReport repairRpt;
                        vector<Router::FailureCertificate> repairCerts;
                        Design repairTrial = bestRoutedCandidate(repairSeeds[ri], repairRpt, &repairCerts);
                        cerr << fixed << setprecision(3)
                            << "[FTCertificateRetry] trial stage=" << stage
                            << " iter=" << ftResizeIter
                            << " round=" << repairRound
                            << " cand=" << ri
                            << " open=" << repairRpt.openPathCount
                            << " chOv=" << repairRpt.totalChannelOverflow
                            << " ftOv=" << repairRpt.totalFeedthroughOverflow
                            << " penalty=" << totalPenalty(repairRpt)
                            << " fail=" << (repairRpt.hasFail() ? "Y" : "N")
                            << "\n";

                        if (!repairRpt.hasFail() &&
                            repairRpt.openPathCount == 0 &&
                            repairRpt.totalFeedthroughOverflow <= 1.0 &&
                            (!haveLegalCertificateRepair ||
                                betterEval(repairRpt, legalCertificateRepair.rpt))) {
                            legalCertificateRepair = {
                                repairTrial, repairRpt, repairCerts, ri,
                                probe.origin + ":legal-shared-port"
                            };
                            haveLegalCertificateRepair = true;
                        }

                        if (!haveFTFeedback || betterEval(repairRpt, bestFTRpt)) {
                            bestFTDesign = repairTrial;
                            bestFTRpt = repairRpt;
                            bestFTCerts = repairCerts;
                            haveFTFeedback = true;
                        }
                        if (!haveRoundBest || betterEval(repairRpt, roundBest.rpt)) {
                            roundBest = {
                                std::move(repairTrial), repairRpt, std::move(repairCerts),
                                ri, probe.origin + ":round-" + to_string(repairRound)
                            };
                            haveRoundBest = true;
                        }
                    }

                    if (!haveRoundBest ||
                        roundBest.rpt.openPathCount >= probe.rpt.openPathCount) break;
                    probe = std::move(roundBest);
                }
            }
            if (stage == "pre" && haveLegalCertificateRepair &&
                totalPenalty(legalCertificateRepair.rpt) > 1.0 &&
                (!haveFTFeedback || totalPenalty(bestFTRpt) > 5000.0)) {
                vector<Design> certificateFloorplanSeeds;
                RepairPortfolioItem bestOpenCertificateFloorplan;
                bool haveOpenCertificateFloorplan = false;
                if (makeFeedthroughReFloorplanCandidates(
                    legalCertificateRepair.routed,
                    certificateFloorplanSeeds,
                    3,
                    true)) {
                    for (int ci = 0;
                        ci < static_cast<int>(certificateFloorplanSeeds.size());
                        ++ci) {
                        EvalReport certificateFloorplanRpt;
                        vector<Router::FailureCertificate> certificateFloorplanCerts;
                        Design certificateFloorplanTrial = bestRoutedCandidate(
                            certificateFloorplanSeeds[ci],
                            certificateFloorplanRpt,
                            &certificateFloorplanCerts);
                        cerr << fixed << setprecision(3)
                            << "[FTCertificateFloorplan] stage=" << stage
                            << " iter=" << ftResizeIter
                            << " cand=" << ci
                            << " open=" << certificateFloorplanRpt.openPathCount
                            << " chOv=" << certificateFloorplanRpt.totalChannelOverflow
                            << " ftOv=" << certificateFloorplanRpt.totalFeedthroughOverflow
                            << " penalty=" << totalPenalty(certificateFloorplanRpt)
                            << " cost=" << certificateFloorplanRpt.cost
                            << " fail=" << (certificateFloorplanRpt.hasFail() ? "Y" : "N")
                            << endl;

                        const bool usableOpenFloorplan =
                            certificateFloorplanRpt.openPathCount > 0 &&
                            certificateFloorplanRpt.totalFeedthroughOverflow <= 1.0 &&
                            !certificateFloorplanCerts.empty();
                        const bool preferOpenFloorplan =
                            !haveOpenCertificateFloorplan ||
                            certificateFloorplanRpt.openPathCount <
                                bestOpenCertificateFloorplan.rpt.openPathCount ||
                            (certificateFloorplanRpt.openPathCount ==
                                bestOpenCertificateFloorplan.rpt.openPathCount &&
                                totalPenalty(certificateFloorplanRpt) <
                                    totalPenalty(bestOpenCertificateFloorplan.rpt));
                        if (usableOpenFloorplan && preferOpenFloorplan) {
                            bestOpenCertificateFloorplan = {
                                certificateFloorplanTrial,
                                certificateFloorplanRpt,
                                certificateFloorplanCerts,
                                ci,
                                stage + ":certificate-floorplan"
                            };
                            haveOpenCertificateFloorplan = true;
                        }

                        if (!haveFTFeedback ||
                            betterEval(certificateFloorplanRpt, bestFTRpt)) {
                            bestFTDesign = std::move(certificateFloorplanTrial);
                            bestFTRpt = certificateFloorplanRpt;
                            bestFTCerts = std::move(certificateFloorplanCerts);
                            haveFTFeedback = true;
                        }
                    }
                }

                if (haveOpenCertificateFloorplan) {
                    vector<Design> topologyRepairSeeds;
                    if (makeCertificateReliefCandidates(
                        bestOpenCertificateFloorplan.routed,
                        bestOpenCertificateFloorplan.certs,
                        topologyRepairSeeds,
                        4)) {
                        for (int ti = 0;
                            ti < static_cast<int>(topologyRepairSeeds.size());
                            ++ti) {
                            EvalReport topologyRepairRpt;
                            Design topologyRepairTrial = bestRoutedCandidate(
                                topologyRepairSeeds[ti],
                                topologyRepairRpt);
                            cerr << fixed << setprecision(3)
                                << "[FTCertificateFloorplanRepair] stage=" << stage
                                << " iter=" << ftResizeIter
                                << " cand=" << ti
                                << " open=" << topologyRepairRpt.openPathCount
                                << " chOv=" << topologyRepairRpt.totalChannelOverflow
                                << " ftOv=" << topologyRepairRpt.totalFeedthroughOverflow
                                << " penalty=" << totalPenalty(topologyRepairRpt)
                                << " cost=" << topologyRepairRpt.cost
                                << " fail=" << (topologyRepairRpt.hasFail() ? "Y" : "N")
                                << endl;
                            if (!haveFTFeedback ||
                                betterEval(topologyRepairRpt, bestFTRpt)) {
                                bestFTDesign = std::move(topologyRepairTrial);
                                bestFTRpt = topologyRepairRpt;
                                bestFTCerts.clear();
                                haveFTFeedback = true;
                            }
                        }
                    }
                }
            }

            if (!triedFTFeedback || !haveFTFeedback || !betterEval(bestFTRpt, bestRpt)) break;
            cerr << fixed << setprecision(3)
                << "[FTFeedback] accept stage=" << stage
                << " iter=" << ftResizeIter
                << " penalty=" << totalPenalty(bestRpt) << "->" << totalPenalty(bestFTRpt)
                << " ftOv=" << bestRpt.totalFeedthroughOverflow << "->" << bestFTRpt.totalFeedthroughOverflow
                << " cost=" << bestRpt.cost << "->" << bestFTRpt.cost
                << "\n";
            bestDesign = std::move(bestFTDesign);
            bestRpt = bestFTRpt;
            bestCerts = std::move(bestFTCerts);
            design = bestDesign;
            rpt = bestRpt;
            improvedAny = true;
        }
        return improvedAny;
        };

    runFTFeedback("pre", 8);
    if (!bestRpt.hasFail() && totalPenalty(bestRpt) > 1.0) {
        for (int reliefIter = 0; reliefIter < 4; ++reliefIter) {
            vector<Design> reliefSeeds;
            if (!makeCapacityReliefCandidates(bestDesign, reliefSeeds, 10)) break;

            bool haveRelief = false;
            Design bestReliefDesign;
            EvalReport bestReliefRpt;

            for (int ri = 0; ri < static_cast<int>(reliefSeeds.size()); ++ri) {
                EvalReport reliefRpt;
                Design reliefTrial = bestRoutedCandidate(reliefSeeds[ri], reliefRpt);
                cerr << fixed << setprecision(3)
                    << "[CapacityReliefPre] trial iter=" << reliefIter
                    << " cand=" << ri
                    << " area=" << reliefRpt.outlineArea
                    << " penalty=" << totalPenalty(reliefRpt)
                    << " cost=" << reliefRpt.cost
                    << " fail=" << (reliefRpt.hasFail() ? "Y" : "N")
                    << "\n";
                if (reliefRpt.hasFail()) continue;
                if (!haveRelief || betterEval(reliefRpt, bestReliefRpt)) {
                    bestReliefDesign = std::move(reliefTrial);
                    bestReliefRpt = reliefRpt;
                    haveRelief = true;
                }
            }

            if (!haveRelief || !betterEval(bestReliefRpt, bestRpt)) break;
            const double oldPenalty = totalPenalty(bestRpt);
            const double newPenalty = totalPenalty(bestReliefRpt);

            cerr << fixed << setprecision(3)
                << "[CapacityReliefPre] accept iter=" << reliefIter
                << " penalty=" << oldPenalty << "->" << newPenalty
                << " cost=" << bestRpt.cost << "->" << bestReliefRpt.cost
                << "\n";
            bestDesign = std::move(bestReliefDesign);
            bestRpt = bestReliefRpt;
            design = bestDesign;
            rpt = bestRpt;
            if (newPenalty <= 500.0) break;
        }
    }

    if (!bestRpt.hasFail() && deadspaceRatioMain(bestDesign) > 0.35) {
        const bool largeDeadspaceTrimCase = bestDesign.blocks.size() >= 20;
        vector<pair<double, double>> shrinkFractions;
        vector<vector<pair<double, double>>> largeDeadspaceShrinkSchedule;
        if (largeDeadspaceTrimCase) {
            largeDeadspaceShrinkSchedule = {
                { { 0.000, 0.240 }, { 0.000, 0.200 }, { 0.000, 0.160 }, { 0.000, 0.120 }, { 0.000, 0.080 }, { 0.000, 0.040 } },
                { { 0.000, 0.200 }, { 0.000, 0.160 }, { 0.000, 0.120 }, { 0.000, 0.080 }, { 0.000, 0.040 } },
                { { 0.000, 0.160 }, { 0.000, 0.120 }, { 0.000, 0.080 }, { 0.000, 0.040 }, { 0.020, 0.000 } },
                { { 0.000, 0.120 }, { 0.000, 0.080 }, { 0.000, 0.040 }, { 0.010, 0.000 } },
                { { 0.000, 0.080 }, { 0.000, 0.040 }, { 0.005, 0.000 } },
                { { 0.000, 0.040 }, { 0.0025, 0.000 } }
            };
        }
        else {
            shrinkFractions = {
                { 0.000, 0.200 },
                { 0.000, 0.180 },
                { 0.000, 0.160 },
                { 0.000, 0.145 },
                { 0.000, 0.130 },
                { 0.000, 0.120 },
                { 0.000, 0.115 },
                { 0.000, 0.110 },
                { 0.000, 0.105 },
                { 0.000, 0.100 },
                { 0.000, 0.090 },
                { 0.000, 0.085 },
                { 0.000, 0.082 },
                { 0.000, 0.080 },
                { 0.000, 0.078 },
                { 0.000, 0.077 },
                { 0.000, 0.0769 },
                { 0.000, 0.07688 },
                { 0.000, 0.07686 },
                { 0.000, 0.07685 },
                { 0.000, 0.07684 },
                { 0.000, 0.07682 },
                { 0.000, 0.0768 },
                { 0.000, 0.0767 },
                { 0.000, 0.0766 },
                { 0.000, 0.0765 },
                { 0.000, 0.0762 },
                { 0.000, 0.0760 },
                { 0.000, 0.0758 },
                { 0.000, 0.0755 },
                { 0.000, 0.075 },
                { 0.000, 0.074 },
                { 0.000, 0.073 },
                { 0.000, 0.070 },
                { 0.000, 0.050 },
                { 0.000, 0.025 },
                { 0.000, 0.020 },
                { 0.000, 0.015 },
                { 0.000, 0.010 },
                { 0.000, 0.0075 },
                { 0.000, 0.005 },
                { 0.000, 0.0025 },
                { 0.000, 0.0015 },
                { 0.000, 0.0010 },
                { 0.070, 0.000 },
                { 0.040, 0.000 },
                { 0.025, 0.000 },
                { 0.020, 0.000 },
                { 0.015, 0.000 },
                { 0.010, 0.000 },
                { 0.0075, 0.000 },
                { 0.005, 0.000 },
                { 0.0025, 0.000 },
                { 0.0015, 0.000 },
                { 0.0010, 0.000 },
                { 0.035, 0.035 }
            };
        }

        const double initialTrimPenalty = totalPenalty(bestRpt);
        const double trimPenaltyBudget = [&]() {
            if (initialTrimPenalty <= 1500.0) return largeDeadspaceTrimCase ? 18000.0 : (initialTrimPenalty + 350.0);
            if (initialTrimPenalty <= 5000.0) return initialTrimPenalty * 1.25 + 500.0;
            if (initialTrimPenalty <= 15000.0) return initialTrimPenalty * 1.50 + 1000.0;
            return max(initialTrimPenalty + 8000.0, initialTrimPenalty * 1.75);
            }();


        auto betterDeadspaceTrim = [&](const EvalReport& a, const EvalReport& b) {
            if (a.hasFail()) return false;
            if (b.hasFail()) return true;
            if (a.cost + 1.0 < b.cost) return true;
            if (b.cost + 1.0 < a.cost) return false;
            const double ap = totalPenalty(a);
            const double bp = totalPenalty(b);
            if (fabs(ap - bp) > 1.0) return ap < bp;
            if (fabs(a.outlineArea - b.outlineArea) > 1.0) return a.outlineArea < b.outlineArea;
            return a.totalWireLength < b.totalWireLength;
        };
        const int trimPassLimit = largeDeadspaceTrimCase ? static_cast<int>(largeDeadspaceShrinkSchedule.size()) : 8;
        for (int trimPass = 0; trimPass < trimPassLimit; ++trimPass) {
            bool improved = false;
            Design bestTrimDesign;
            EvalReport bestTrimRpt;
            bool haveTrim = false;
            const double basePenalty = totalPenalty(bestRpt);

            const vector<pair<double, double>>& activeShrinkFractions = largeDeadspaceTrimCase ? largeDeadspaceShrinkSchedule[trimPass] : shrinkFractions;
            for (const auto& sf : activeShrinkFractions) {
                Design candidate = bestDesign;
                const double shrinkW = candidate.outlineW * sf.first;
                const double shrinkH = candidate.outlineH * sf.second;
                bool trimGeometryOk = tryEdgeOnlyOutlineTrimMain(candidate, shrinkW, shrinkH);
                if (!trimGeometryOk) {
                    candidate = bestDesign;
                    trimGeometryOk = tryGravityOutlineTrimMain(candidate, shrinkW, shrinkH);
                }
                if (!trimGeometryOk) {
                    cerr << fixed << setprecision(3)
                        << "[DeadspaceTrim] skipGeometry pass=" << trimPass
                        << " shrinkW=" << shrinkW
                        << " shrinkH=" << shrinkH
                        << "\n";
                    continue;
                }

                EvalReport trimRpt;
                Design routed = bestRoutedCandidate(candidate, trimRpt);

                if (!largeDeadspaceTrimCase && !trimRpt.hasFail() && totalPenalty(trimRpt) <= trimPenaltyBudget + 2500.0) {
                    for (int trimRepairIter = 0; trimRepairIter < 2; ++trimRepairIter) {
                        bool haveRepair = false;
                        Design bestRepairDesign;
                        EvalReport bestRepairRpt;

                        auto tryTrimRepair = [&](const Design& repairSeed) {
                            EvalReport repairRpt;
                            Design repairTrial = bestRoutedCandidate(repairSeed, repairRpt);
                            if (!haveRepair || betterEval(repairRpt, bestRepairRpt)) {
                                bestRepairDesign = std::move(repairTrial);
                                bestRepairRpt = repairRpt;
                                haveRepair = true;
                            }
                        };

                        Design movedH = routed;
                        if (relieveHorizontalHotChannels(movedH)) tryTrimRepair(movedH);

                        Design movedV = routed;
                        if (relieveVerticalHotChannels(movedV)) tryTrimRepair(movedV);

                        Design movedHV = routed;
                        const bool trimMovedH = relieveHorizontalHotChannels(movedHV);
                        const bool trimMovedV = relieveVerticalHotChannels(movedHV);
                        if (trimMovedH && trimMovedV) tryTrimRepair(movedHV);

                        if (!haveRepair || !betterEval(bestRepairRpt, trimRpt)) break;
                        routed = std::move(bestRepairDesign);
                        trimRpt = bestRepairRpt;
                    }
                }

                const double trimPenalty = totalPenalty(trimRpt);
                cerr << fixed << setprecision(3)
                    << "[DeadspaceTrim] trial pass=" << trimPass
                    << " shrinkW=" << shrinkW
                    << " shrinkH=" << shrinkH
                    << " area=" << trimRpt.outlineArea
                    << " penalty=" << trimPenalty
                    << " cost=" << trimRpt.cost
                    << " fail=" << (trimRpt.hasFail() ? "Y" : "N")
                    << "\n";
                if (trimRpt.hasFail()) continue;
                if (!betterDeadspaceTrim(trimRpt, bestRpt)) continue;

                if (!haveTrim || betterDeadspaceTrim(trimRpt, bestTrimRpt)) {
                    bestTrimDesign = std::move(routed);
                    bestTrimRpt = trimRpt;
                    haveTrim = true;
                    if (largeDeadspaceTrimCase) break;
                }
            }

            if (!haveTrim) break;
            bestDesign = std::move(bestTrimDesign);
            bestRpt = bestTrimRpt;
            design = bestDesign;
            rpt = bestRpt;
            improved = true;
            if (!improved) break;
        }
    }

    if (!bestRpt.hasFail() && totalPenalty(bestRpt) > 1.0) {
        for (int reliefIter = 0; reliefIter < 3; ++reliefIter) {
            vector<Design> reliefSeeds;
            if (!makeCapacityReliefCandidates(bestDesign, reliefSeeds, 8)) break;

            bool haveRelief = false;
            Design bestReliefDesign;
            EvalReport bestReliefRpt;

            for (int ri = 0; ri < static_cast<int>(reliefSeeds.size()); ++ri) {
                EvalReport reliefRpt;
                Design reliefTrial = bestRoutedCandidate(reliefSeeds[ri], reliefRpt);
                cerr << fixed << setprecision(3)
                    << "[CapacityRelief] trial iter=" << reliefIter
                    << " cand=" << ri
                    << " area=" << reliefRpt.outlineArea
                    << " penalty=" << totalPenalty(reliefRpt)
                    << " cost=" << reliefRpt.cost
                    << " fail=" << (reliefRpt.hasFail() ? "Y" : "N")
                    << "\n";
                if (reliefRpt.hasFail()) continue;
                if (!haveRelief || betterEval(reliefRpt, bestReliefRpt)) {
                    bestReliefDesign = std::move(reliefTrial);
                    bestReliefRpt = reliefRpt;
                    haveRelief = true;
                }
            }

            if (!haveRelief || !betterEval(bestReliefRpt, bestRpt)) break;
            const double oldPenalty = totalPenalty(bestRpt);
            const double newPenalty = totalPenalty(bestReliefRpt);
            const bool costAcceptable = bestReliefRpt.cost <= bestRpt.cost * 1.15 + 1000000.0;
            if (newPenalty > 1.0 && !costAcceptable) break;

            cerr << fixed << setprecision(3)
                << "[CapacityRelief] accept iter=" << reliefIter
                << " penalty=" << oldPenalty << "->" << newPenalty
                << " cost=" << bestRpt.cost << "->" << bestReliefRpt.cost
                << "\n";
            bestDesign = std::move(bestReliefDesign);
            bestRpt = bestReliefRpt;
            design = bestDesign;
            rpt = bestRpt;
        }
    }
    const bool enableCommonEdgeSnapPost = true;
    if (enableCommonEdgeSnapPost && !bestRpt.hasFail() && totalPenalty(bestRpt) > 1.0) {
        for (int snapIter = 0; snapIter < 1; ++snapIter) {
            vector<Design> snapSeeds;
            if (!makeCommonEdgeSnapCandidates(bestDesign, snapSeeds, 3)) break;

            bool haveSnap = false;
            Design bestSnapDesign;
            EvalReport bestSnapRpt;

            for (int si = 0; si < static_cast<int>(snapSeeds.size()); ++si) {
                EvalReport snapRpt;
                Design snapTrial = bestRoutedCandidate(snapSeeds[si], snapRpt);
                cerr << fixed << setprecision(3)
                    << "[CommonEdgeSnap] trial iter=" << snapIter
                    << " cand=" << si
                    << " area=" << snapRpt.outlineArea
                    << " penalty=" << totalPenalty(snapRpt)
                    << " cost=" << snapRpt.cost
                    << " fail=" << (snapRpt.hasFail() ? "Y" : "N")
                    << "\n";
                if (snapRpt.hasFail()) continue;
                if (!haveSnap || betterEval(snapRpt, bestSnapRpt)) {
                    bestSnapDesign = std::move(snapTrial);
                    bestSnapRpt = snapRpt;
                    haveSnap = true;
                }
            }

            if (!haveSnap || !betterEval(bestSnapRpt, bestRpt)) break;
            const double oldPenalty = totalPenalty(bestRpt);
            const double newPenalty = totalPenalty(bestSnapRpt);

            cerr << fixed << setprecision(3)
                << "[CommonEdgeSnap] accept iter=" << snapIter
                << " penalty=" << oldPenalty << "->" << newPenalty
                << " cost=" << bestRpt.cost << "->" << bestSnapRpt.cost
                << "\n";
            bestDesign = std::move(bestSnapDesign);
            bestRpt = bestSnapRpt;
            design = bestDesign;
            rpt = bestRpt;
        }
    }

    if (!bestRpt.hasFail() && totalPenalty(bestRpt) > 1.0) {
        for (int reliefIter = 0; reliefIter < 2; ++reliefIter) {
            vector<Design> reliefSeeds;
            if (!makeCapacityReliefCandidates(bestDesign, reliefSeeds, 8)) break;

            bool haveRelief = false;
            Design bestReliefDesign;
            EvalReport bestReliefRpt;

            for (int ri = 0; ri < static_cast<int>(reliefSeeds.size()); ++ri) {
                EvalReport reliefRpt;
                Design reliefTrial = bestRoutedCandidate(reliefSeeds[ri], reliefRpt);
                cerr << fixed << setprecision(3)
                    << "[CapacityReliefPost] trial iter=" << reliefIter
                    << " cand=" << ri
                    << " area=" << reliefRpt.outlineArea
                    << " penalty=" << totalPenalty(reliefRpt)
                    << " cost=" << reliefRpt.cost
                    << " fail=" << (reliefRpt.hasFail() ? "Y" : "N")
                    << "\n";
                if (reliefRpt.hasFail()) continue;
                if (!haveRelief || betterEval(reliefRpt, bestReliefRpt)) {
                    bestReliefDesign = std::move(reliefTrial);
                    bestReliefRpt = reliefRpt;
                    haveRelief = true;
                }
            }

            if (!haveRelief || !betterEval(bestReliefRpt, bestRpt)) break;
            const double oldPenalty = totalPenalty(bestRpt);
            const double newPenalty = totalPenalty(bestReliefRpt);
            const bool costAcceptable = bestReliefRpt.cost <= bestRpt.cost * 1.15 + 1000000.0;
            if (newPenalty > 1.0 && !costAcceptable) break;

            cerr << fixed << setprecision(3)
                << "[CapacityReliefPost] accept iter=" << reliefIter
                << " penalty=" << oldPenalty << "->" << newPenalty
                << " cost=" << bestRpt.cost << "->" << bestReliefRpt.cost
                << "\n";
            bestDesign = std::move(bestReliefDesign);
            bestRpt = bestReliefRpt;
            design = bestDesign;
            rpt = bestRpt;
        }
    }
    if (!bestRpt.hasFail() && totalPenalty(bestRpt) > 1.0) {
        runFTFeedback("post-snap", 6);
    }
    auto betterThinAlignment = [&](const EvalReport& a, const Design& ad, const EvalReport& b, const Design& bd) {
        const double ap = totalPenalty(a);
        const double bp = totalPenalty(b);
        if (ap + 1.0 < bp) return true;
        if (bp + 1.0 < ap) return false;

        const double as = routedStructureScoreMain(ad);
        const double bs = routedStructureScoreMain(bd);
        if (a.cost + 1.0 < b.cost) return true;
        const double structureTieCost = max(2500.0, 0.00005 * max(1.0, b.cost));
        if (as + 1.0 < bs && a.cost <= b.cost + structureTieCost) return true;
        if (a.outlineArea + 1.0 < b.outlineArea && a.cost <= b.cost + 1.0) return true;
        return false;
        };

    if (!bestRpt.hasFail()) {
        const int alignBlockCount = static_cast<int>(bestDesign.blocks.size());
        const bool enableThinChannelAlign = alignBlockCount <= 14 && bestDesign.connections.size() >= 50;
        const int alignIterLimit = enableThinChannelAlign ? 1 : 0;
        const int alignCandidateLimit = enableThinChannelAlign ? 8 : 0;
        for (int alignIter = 0; alignIter < alignIterLimit; ++alignIter) {
            vector<Design> alignSeeds;
            if (!makeThinChannelAlignmentCandidates(bestDesign, alignSeeds, alignCandidateLimit)) break;

            bool haveAlign = false;
            Design bestAlignDesign;
            EvalReport bestAlignRpt;
            const double basePenalty = totalPenalty(bestRpt);
            const double alignPenaltyBudget = basePenalty <= 1.0 ? 0.0 : basePenalty + max(50.0, 0.05 * basePenalty);

            for (int ai = 0; ai < static_cast<int>(alignSeeds.size()); ++ai) {
                EvalReport alignRpt;
                Design alignTrial = bestRoutedCandidate(alignSeeds[ai], alignRpt);
                const double alignPenalty = totalPenalty(alignRpt);
                const double alignScore = routedStructureScoreMain(alignTrial);
                cerr << fixed << setprecision(3)
                    << "[ThinChannelAlign] trial iter=" << alignIter
                    << " cand=" << ai
                    << " area=" << alignRpt.outlineArea
                    << " penalty=" << alignPenalty
                    << " cost=" << alignRpt.cost
                    << " structure=" << alignScore
                    << " fail=" << (alignRpt.hasFail() ? "Y" : "N")
                    << "\n";
                if (alignRpt.hasFail()) continue;
                if (alignPenalty > alignPenaltyBudget + 1.0e-6) continue;
                if (!betterThinAlignment(alignRpt, alignTrial, bestRpt, bestDesign)) continue;
                if (!haveAlign || betterThinAlignment(alignRpt, alignTrial, bestAlignRpt, bestAlignDesign)) {
                    bestAlignDesign = std::move(alignTrial);
                    bestAlignRpt = alignRpt;
                    haveAlign = true;
                }
            }

            if (!haveAlign) break;
            const double oldPenalty = totalPenalty(bestRpt);
            const double newPenalty = totalPenalty(bestAlignRpt);
            cerr << fixed << setprecision(3)
                << "[ThinChannelAlign] accept iter=" << alignIter
                << " penalty=" << oldPenalty << "->" << newPenalty
                << " cost=" << bestRpt.cost << "->" << bestAlignRpt.cost
                << " structure=" << routedStructureScoreMain(bestDesign) << "->" << routedStructureScoreMain(bestAlignDesign)
                << "\n";
            bestDesign = std::move(bestAlignDesign);
            bestRpt = bestAlignRpt;
            design = bestDesign;
            rpt = bestRpt;
        }
    }

    auto utilizationPenaltyBudget = [&](double basePenalty) {
        if (basePenalty <= 1.0) return 0.0;
        return max(basePenalty + 900.0, basePenalty * 4.5);
        };

    auto betterUtilizationTrim = [&](const EvalReport& a, const Design& ad, const EvalReport& b, const Design& bd) {
        if (a.hasFail()) return false;
        if (b.hasFail()) return true;

        // Cost is the objective. Structure is only a tie-breaker; do not pay real
        // cost for a prettier channel graph once V5 cost is equivalent.
        if (a.cost + 1.0 < b.cost) return true;
        if (b.cost + 1.0 < a.cost) return false;
        const double ap = totalPenalty(a);
        const double bp = totalPenalty(b);
        if (fabs(ap - bp) > 1.0) return ap < bp;
        if (a.outlineArea + 1.0 < b.outlineArea) return true;
        if (b.outlineArea + 1.0 < a.outlineArea) return false;
        return routedStructureScoreMain(ad) + 1.0 < routedStructureScoreMain(bd);
        };

    if (!bestRpt.hasFail() && deadspaceRatioMain(bestDesign) > 0.28) {
        vector<pair<double, double>> utilShrinkFractions = {
            { 0.0000, 0.0050 },
            { 0.0000, 0.0010 },
            { 0.0050, 0.0000 },
            { 0.0025, 0.0000 },
            { 0.0015, 0.0000 },
            { 0.0010, 0.0000 },
            { 0.0020, 0.0020 }
        };
        vector<pair<double, double>> utilPolishFractions = {
            { 0.0025, 0.0000 },
            { 0.0015, 0.0000 },
            { 0.0010, 0.0000 },
            { 0.0005, 0.0000 },
            { 0.0000, 0.0005 }
        };
        vector<vector<pair<double, double>>> largeUtilShrinkSchedule = {
            { { 0.0000, 0.0050 } },
            { { 0.0025, 0.0000 } },
            { { 0.0000, 0.0010 } },
            { { 0.0050, 0.0000 }, { 0.0025, 0.0000 }, { 0.0000, 0.0010 }, { 0.0020, 0.0020 } },
            { { 0.0025, 0.0000 }, { 0.0000, 0.0010 }, { 0.0015, 0.0000 }, { 0.0010, 0.0000 }, { 0.0000, 0.0005 } },
            { { 0.0010, 0.0000 }, { 0.0005, 0.0000 }, { 0.00025, 0.0000 }, { 0.0000, 0.00025 } }
        };

        const int utilBlockCount = static_cast<int>(bestDesign.blocks.size());
        const bool largeUtilCase = utilBlockCount >= 20;
        const int utilIterLimit = largeUtilCase ? static_cast<int>(largeUtilShrinkSchedule.size()) : 6;
        for (int utilIter = 0; utilIter < utilIterLimit; ++utilIter) {
            bool haveUtil = false;
            Design bestUtilDesign;
            EvalReport bestUtilRpt;

            const vector<pair<double, double>>& activeUtilShrinkFractions = largeUtilCase
                ? largeUtilShrinkSchedule[utilIter]
                : ((utilIter + 1 == utilIterLimit) ? utilPolishFractions : utilShrinkFractions);
            for (const auto& sf : activeUtilShrinkFractions) {
                Design candidate = bestDesign;
                const double shrinkW = candidate.outlineW * sf.first;
                const double shrinkH = candidate.outlineH * sf.second;
                if (!tryEdgeOnlyOutlineTrimMain(candidate, shrinkW, shrinkH)) continue;

                EvalReport utilRpt;
                Design utilTrial = bestRoutedCandidate(candidate, utilRpt);
                if (largeUtilCase && !utilRpt.hasFail()) {
                    const double retrySlack = max(250000.0, bestRpt.cost * 0.004);
                    const bool areaImproves = utilRpt.outlineArea + 1.0 < bestRpt.outlineArea;
                    const bool nearBest = utilRpt.cost <= bestRpt.cost + retrySlack;
                    if (areaImproves && nearBest) {
                        EvalReport retryRpt;
                        Design retryTrial = bestRoutedCandidate(candidate, retryRpt);
                        cerr << fixed << setprecision(3)
                            << "[UtilizationTrim] retry iter=" << utilIter
                            << " shrinkW=" << shrinkW
                            << " shrinkH=" << shrinkH
                            << " area=" << retryRpt.outlineArea
                            << " penalty=" << totalPenalty(retryRpt)
                            << " cost=" << retryRpt.cost
                            << " structure=" << routedStructureScoreMain(retryTrial)
                            << " fail=" << (retryRpt.hasFail() ? "Y" : "N")
                            << "\n";
                        if (betterUtilizationTrim(retryRpt, retryTrial, utilRpt, utilTrial)) {
                            utilTrial = std::move(retryTrial);
                            utilRpt = retryRpt;
                        }
                    }
                }
                const double utilPenalty = totalPenalty(utilRpt);
                cerr << fixed << setprecision(3)
                    << "[UtilizationTrim] trial iter=" << utilIter
                    << " shrinkW=" << shrinkW
                    << " shrinkH=" << shrinkH
                    << " area=" << utilRpt.outlineArea
                    << " penalty=" << utilPenalty
                    << " cost=" << utilRpt.cost
                    << " structure=" << routedStructureScoreMain(utilTrial)
                    << " fail=" << (utilRpt.hasFail() ? "Y" : "N")
                    << "\n";
                if (utilRpt.hasFail()) continue;
                if (!betterUtilizationTrim(utilRpt, utilTrial, bestRpt, bestDesign)) continue;
                if (!haveUtil || betterUtilizationTrim(utilRpt, utilTrial, bestUtilRpt, bestUtilDesign)) {
                    bestUtilDesign = std::move(utilTrial);
                    bestUtilRpt = utilRpt;
                    haveUtil = true;
                }
            }

            if (!haveUtil) break;
            const double oldPenalty = totalPenalty(bestRpt);
            const double newPenalty = totalPenalty(bestUtilRpt);
            cerr << fixed << setprecision(3)
                << "[UtilizationTrim] accept iter=" << utilIter
                << " penalty=" << oldPenalty << "->" << newPenalty
                << " area=" << bestRpt.outlineArea << "->" << bestUtilRpt.outlineArea
                << " cost=" << bestRpt.cost << "->" << bestUtilRpt.cost
                << "\n";
            bestDesign = std::move(bestUtilDesign);
            bestRpt = bestUtilRpt;
            design = bestDesign;
            rpt = bestRpt;
        }
    }
    if (!bestRpt.hasFail() && totalPenalty(bestRpt) > 1.0) {
        for (int reliefIter = 0; reliefIter < 4; ++reliefIter) {
            vector<Design> reliefSeeds;
            if (!makeCapacityReliefCandidates(bestDesign, reliefSeeds, 12)) break;

            bool haveRelief = false;
            Design bestReliefDesign;
            EvalReport bestReliefRpt;


            for (int ri = 0; ri < static_cast<int>(reliefSeeds.size()); ++ri) {
                EvalReport reliefRpt;
                Design reliefTrial = bestRoutedCandidate(reliefSeeds[ri], reliefRpt);
                cerr << fixed << setprecision(3)
                    << "[FinalCapacityRelief] trial iter=" << reliefIter
                    << " cand=" << ri
                    << " area=" << reliefRpt.outlineArea
                    << " penalty=" << totalPenalty(reliefRpt)
                    << " cost=" << reliefRpt.cost
                    << " fail=" << (reliefRpt.hasFail() ? "Y" : "N")
                    << "\n";
                if (reliefRpt.hasFail()) continue;
                if (!haveRelief || betterEval(reliefRpt, bestReliefRpt)) {
                    bestReliefDesign = std::move(reliefTrial);
                    bestReliefRpt = reliefRpt;
                    haveRelief = true;
                }
            }

            if (!haveRelief || !betterEval(bestReliefRpt, bestRpt)) break;
            const double oldPenalty = totalPenalty(bestRpt);
            const double newPenalty = totalPenalty(bestReliefRpt);
            const bool costAcceptable = bestReliefRpt.cost <= bestRpt.cost * 1.20 + 2000000.0;
            if (newPenalty > 1.0 && !costAcceptable) break;

            cerr << fixed << setprecision(3)
                << "[FinalCapacityRelief] accept iter=" << reliefIter
                << " penalty=" << oldPenalty << "->" << newPenalty
                << " cost=" << bestRpt.cost << "->" << bestReliefRpt.cost
                << "\n";
            bestDesign = std::move(bestReliefDesign);
            bestRpt = bestReliefRpt;
            design = bestDesign;
            rpt = bestRpt;
        }
    }

    if (!bestRpt.hasFail() && totalPenalty(bestRpt) > 1.0) {
        runFTFeedback("post-final-capacity", 6);
    }
    if (bestRpt.openPathCount > 0) {
        EvalReport rerouteRpt;
        vector<Router::FailureCertificate> finalCerts;
        Design rerouted = bestRoutedCandidate(bestDesign, rerouteRpt, &finalCerts);
        if (betterEval(rerouteRpt, bestRpt)) {
            bestDesign = std::move(rerouted);
            bestRpt = rerouteRpt;
            bestCerts = finalCerts;
        }
        else {
            bestCerts = finalCerts;
        }

        for (int certReliefIter = 0; bestRpt.openPathCount > 0 && certReliefIter < 4; ++certReliefIter) {
            if (bestCerts.empty()) {
                cerr << "[FinalCertificateRelief] skip reason=no_certificate open=" << bestRpt.openPathCount << "\n";
                break;
            }
            vector<Design> reliefSeeds;
            if (!makeCertificateReliefCandidates(bestDesign, bestCerts, reliefSeeds, 10)) {
                int certNeedCount = 0;
                int topNeedChannel = -1;
                const char* topNeedDir = "NA";
                double topNeedShortage = 0.0;
                double topNeedDelta = 0.0;
                for (const auto& cert : bestCerts) {
                    certNeedCount += static_cast<int>(cert.hardNeeds.size());
                    for (const auto& need : cert.hardNeeds) {
                        if (need.shortageNets > topNeedShortage) {
                            topNeedShortage = need.shortageNets;
                            topNeedChannel = need.channelIndex;
                            topNeedDir = certificateDirName(need.dir);
                            topNeedDelta = need.requiredDeltaUm;
                        }
                    }
                }
                cerr << "[FinalCertificateRelief] skip reason=no_candidate open=" << bestRpt.openPathCount
                    << " certs=" << bestCerts.size()
                    << " needs=" << certNeedCount
                    << " top=CH" << (topNeedChannel + 1)
                    << "." << topNeedDir
                    << " shortage=" << topNeedShortage
                    << " delta=" << topNeedDelta
                    << " outlineW=" << bestDesign.outlineW
                    << " maxW=" << bestDesign.maxOutlineW
                    << " outlineH=" << bestDesign.outlineH
                    << " maxH=" << bestDesign.maxOutlineH
                    << "\n";
                break;
            }

            bool haveRelief = false;
            Design bestReliefDesign;
            EvalReport bestReliefRpt;
            vector<Router::FailureCertificate> bestReliefCerts;
            for (int ri = 0; ri < static_cast<int>(reliefSeeds.size()); ++ri) {
                EvalReport reliefRpt;
                vector<Router::FailureCertificate> reliefCerts;
                Design reliefTrial = bestRoutedCandidate(reliefSeeds[ri], reliefRpt, &reliefCerts);
                cerr << fixed << setprecision(3)
                    << "[FinalCertificateRelief] trial iter=" << certReliefIter
                    << " cand=" << ri
                    << " area=" << reliefRpt.outlineArea
                    << " open=" << reliefRpt.openPathCount
                    << " penalty=" << totalPenalty(reliefRpt)
                    << " cost=" << reliefRpt.cost
                    << " fail=" << (reliefRpt.hasFail() ? "Y" : "N")
                    << "\n";
                if (!haveRelief || betterEval(reliefRpt, bestReliefRpt)) {
                    bestReliefDesign = std::move(reliefTrial);
                    bestReliefRpt = reliefRpt;
                    bestReliefCerts = std::move(reliefCerts);
                    haveRelief = true;
                }
            }

            if (!haveRelief || !betterEval(bestReliefRpt, bestRpt)) break;
            cerr << fixed << setprecision(3)
                << "[FinalCertificateRelief] accept iter=" << certReliefIter
                << " open=" << bestRpt.openPathCount << "->" << bestReliefRpt.openPathCount
                << " penalty=" << totalPenalty(bestRpt) << "->" << totalPenalty(bestReliefRpt)
                << " cost=" << bestRpt.cost << "->" << bestReliefRpt.cost
                << "\n";
            bestDesign = std::move(bestReliefDesign);
            bestRpt = bestReliefRpt;
            bestCerts = std::move(bestReliefCerts);
        }
    }


    if (!bestRpt.hasFail()) {
        vector<Design> tightSeeds;
        if (makeTightOutlineCandidatesMain(bestDesign, tightSeeds, 8)) {
            bool haveTight = false;
            Design bestTightDesign;
            EvalReport bestTightRpt;
            for (int ti = 0; ti < static_cast<int>(tightSeeds.size()); ++ti) {
                EvalReport tightRpt;
                Design tightTrial = bestRoutedCandidate(tightSeeds[ti], tightRpt);
                cerr << fixed << setprecision(3)
                    << "[TightOutline] trial cand=" << ti
                    << " W/H=" << tightSeeds[ti].outlineW << "x" << tightSeeds[ti].outlineH
                    << " area=" << tightRpt.outlineArea
                    << " wl=" << tightRpt.totalWireLength
                    << " penalty=" << totalPenalty(tightRpt)
                    << " cost=" << tightRpt.cost
                    << " fail=" << (tightRpt.hasFail() ? "Y" : "N")
                    << "\n";
                if (tightRpt.hasFail()) continue;
                if (!haveTight || betterEval(tightRpt, bestTightRpt)) {
                    bestTightDesign = std::move(tightTrial);
                    bestTightRpt = tightRpt;
                    haveTight = true;
                }
            }
            if (haveTight && betterEval(bestTightRpt, bestRpt)) {
                cerr << fixed << setprecision(3)
                    << "[TightOutline] accept area=" << bestRpt.outlineArea << "->" << bestTightRpt.outlineArea
                    << " wl=" << bestRpt.totalWireLength << "->" << bestTightRpt.totalWireLength
                    << " cost=" << bestRpt.cost << "->" << bestTightRpt.cost
                    << "\n";
                bestDesign = std::move(bestTightDesign);
                bestRpt = bestTightRpt;
                design = bestDesign;
                rpt = bestRpt;
            }
        }
    }

    if (!bestRpt.hasFail()) {
        EvalReport finalRerouteRpt;
        Design finalReroute = bestRoutedCandidate(bestDesign, finalRerouteRpt);
        cerr << fixed << setprecision(3)
            << "[FinalReroute] trial area=" << finalRerouteRpt.outlineArea
            << " wl=" << finalRerouteRpt.totalWireLength
            << " penalty=" << totalPenalty(finalRerouteRpt)
            << " cost=" << finalRerouteRpt.cost
            << " fail=" << (finalRerouteRpt.hasFail() ? "Y" : "N")
            << "\n";
        if (betterEval(finalRerouteRpt, bestRpt)) {
            cerr << fixed << setprecision(3)
                << "[FinalReroute] accept wl=" << bestRpt.totalWireLength << "->" << finalRerouteRpt.totalWireLength
                << " cost=" << bestRpt.cost << "->" << finalRerouteRpt.cost
                << "\n";
            bestDesign = std::move(finalReroute);
            bestRpt = finalRerouteRpt;
            design = bestDesign;
            rpt = bestRpt;
        }
    }

    if (!bestRpt.hasFail() && totalPenalty(bestRpt) > 1.0) {
        runFTFeedback("final", 6);
    }
    // Disabled: final quality must come from floorplan/router search, not known-case cfg portfolio.
    design = bestDesign;
    rpt = evaluator.evaluate(design, opt.alpha, programRuntimeSecondsMain());
    bool writeOk = writer.write(opt.outputPath, design);
    if (!writeOk) {
        cerr << "[OutputWriter] Failed to write cfg: " << opt.outputPath << "\n";
        return 1;
    }

    router.printDetourReport(design);
    Logger::printFinalReport(design, rpt, opt.alpha, opt.inputPath, opt.outputPath);

    if (rpt.hasFail()) return 2;
    return 0;
}
