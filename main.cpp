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

static double placedBlockAreaMain(const Design& design) {
    double area = 0.0;
    for (const auto& b : design.blocks) area += max(0.0, b.rect.w * b.rect.h);
    return area;
}

static double deadspaceRatioMain(const Design& design) {
    const double outlineArea = max(1.0, design.outlineW * design.outlineH);
    return max(0.0, (outlineArea - placedBlockAreaMain(design)) / outlineArea);
}


static bool tryMoveBlocksYWithClosure(Design& design, const vector<int>& ids, double dy);
static bool tryMoveBlocksXWithClosure(Design& design, const vector<int>& ids, double dx);

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
            if (fabs(b.rect.y - y2) <= 1.0e-3) above.push_back(bi);
            if (fabs(rectTop(b.rect) - y1) <= 1.0e-3) below.push_back(bi);
        }

        const double wanted = min(260.0, max(20.0, ch.overflow / CHANNEL_DENSITY));
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
            if (fabs(rectRight(b.rect) - x1) <= 1.0e-3) left.push_back(bi);
            if (fabs(b.rect.x - x2) <= 1.0e-3) right.push_back(bi);
        }

        const double wanted = min(260.0, max(20.0, ch.overflow / CHANNEL_DENSITY));
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
                if (fabs(b.rect.y - y2) <= 1.0e-3) above.push_back(bi);
                if (fabs(rectTop(b.rect) - y1) <= 1.0e-3) below.push_back(bi);
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
                if (fabs(rectRight(b.rect) - x1) <= 1.0e-3) left.push_back(bi);
                if (fabs(b.rect.x - x2) <= 1.0e-3) right.push_back(bi);
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
                if (fabs(b.rect.y - y2) <= 1.0e-3) above.push_back(bi);
                if (fabs(rectTop(b.rect) - y1) <= 1.0e-3) below.push_back(bi);
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
                if (fabs(rectRight(b.rect) - x1) <= 1.0e-3) left.push_back(bi);
                if (fabs(b.rect.x - x2) <= 1.0e-3) right.push_back(bi);
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
static bool edgeCanSlideYForCapacityRelief(const Design& design, const BlockInst& b, double dy) {
    if (b.spec.type != BlockType::EDGE) return true;
    Rect moved = b.rect;
    moved.y += dy;
    if (!rectInsideOutlineMain(moved, design.outlineW, design.outlineH)) return false;
    const double tol = max(2.0, 1.0e-4 * max(design.outlineW, design.outlineH));
    const bool staysLeft = fabs(b.rect.x) <= tol && fabs(moved.x) <= tol;
    const bool staysRight = fabs(rectRight(b.rect) - design.outlineW) <= tol && fabs(rectRight(moved) - design.outlineW) <= tol;
    return staysLeft || staysRight;
}

static bool edgeCanSlideXForCapacityRelief(const Design& design, const BlockInst& b, double dx) {
    if (b.spec.type != BlockType::EDGE) return true;
    Rect moved = b.rect;
    moved.x += dx;
    if (!rectInsideOutlineMain(moved, design.outlineW, design.outlineH)) return false;
    const double tol = max(2.0, 1.0e-4 * max(design.outlineW, design.outlineH));
    const bool staysBottom = fabs(b.rect.y) <= tol && fabs(moved.y) <= tol;
    const bool staysTop = fabs(rectTop(b.rect) - design.outlineH) <= tol && fabs(rectTop(moved) - design.outlineH) <= tol;
    return staysBottom || staysTop;
}

static vector<int> expandMoveClosureYForCapacityRelief(const Design& design, const vector<int>& seeds, double dy, bool& ok) {
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
    if (!placementLegalAfterMove(design, moved)) return false;
    design.blocks.swap(moved);
    return true;
}
static bool makeCapacityReliefCandidates(const Design& design, vector<Design>& out, int maxCandidates) {
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
    const int channelLimit = min(5, static_cast<int>(hot.size()));

    auto addCandidate = [&](Design&& trial) {
        if (static_cast<int>(out.size()) >= maxCandidates) return;
        if (!placementLegalAfterMove(trial, trial.blocks)) return;
        trial.channels.clear();
        trial.routes.clear();
        out.push_back(std::move(trial));
        };

    auto reliefDeltas = [](double overflow) {
        const double exact = min(220.0, max(2.0, overflow / CHANNEL_DENSITY + 0.50));
        vector<double> d = { exact, exact * 0.75, exact * 1.25, exact * 0.50, 8.0, 16.0, 24.0 };
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
                if (!blockMovableForHotRepair(b.spec)) continue;
                if (overlapLen(b.rect.x, rectRight(b.rect), x1, x2) <= 1.0e-4) continue;
                if (fabs(b.rect.y - y2) <= 1.0e-3) above.push_back(bi);
                if (fabs(rectTop(b.rect) - y1) <= 1.0e-3) below.push_back(bi);
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
            for (int bi = 0; bi < static_cast<int>(design.blocks.size()); ++bi) {
                const BlockInst& b = design.blocks[bi];
                if (!blockMovableForHotRepair(b.spec)) continue;
                if (overlapLen(b.rect.y, rectTop(b.rect), y1, y2) <= 1.0e-4) continue;
                if (fabs(rectRight(b.rect) - x1) <= 1.0e-3) left.push_back(bi);
                if (fabs(b.rect.x - x2) <= 1.0e-3) right.push_back(bi);
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

    bool forceSoftFTRelaxedRouting = false;

    auto routeCandidate = [&](const Design& seed, bool ftOverflowAware, bool softFTRelaxed, EvalReport& outRpt) {
        Design trial = seed;
        channelBuilder.build(trial);
        router.setFTOverflowCostEnabled(ftOverflowAware);
        router.setSoftFTCostRelaxed(softFTRelaxed);
        router.run(trial);
        outRpt = evaluator.evaluate(trial, opt.alpha);
        return trial;
        };

    auto bestRoutedCandidate = [&](const Design& seed, EvalReport& outRpt) {
        if (forceSoftFTRelaxedRouting) {
            EvalReport repairRpt;
            Design repair = routeCandidate(seed, true, true, repairRpt);
            Design best = repair;
            outRpt = repairRpt;

            if (!repairRpt.hasFail() && totalPenalty(repairRpt) <= 1.0) {
                return best;
            }

            EvalReport relaxedBaseRpt;
            Design relaxedBase = routeCandidate(seed, false, true, relaxedBaseRpt);
            if (betterEval(relaxedBaseRpt, outRpt)) {
                best = std::move(relaxedBase);
                outRpt = relaxedBaseRpt;
            }
            return best;
        }

        EvalReport baseRpt;
        Design base = routeCandidate(seed, false, false, baseRpt);
        Design best = base;
        outRpt = baseRpt;

        if (baseRpt.hasFail() || totalPenalty(baseRpt) > 1.0) {
            EvalReport relaxedBaseRpt;
            Design relaxedBase = routeCandidate(seed, false, true, relaxedBaseRpt);
            if (betterEval(relaxedBaseRpt, outRpt)) {
                best = std::move(relaxedBase);
                outRpt = relaxedBaseRpt;
            }

            EvalReport repairRpt;
            Design repair = routeCandidate(seed, true, true, repairRpt);
            if (betterEval(repairRpt, outRpt)) {
                best = std::move(repair);
                outRpt = repairRpt;
            }
        }
        return best;
        };

    // One-way architecture:    // Parser -> Floorplanner -> ChannelBuilder -> Router -> Evaluator -> OutputWriter
    floorplanner.setEdgePlacementMode(0);
    floorplanner.run(design);
    Design floorplannedDesign = design;

    EvalReport rpt;
    design = bestRoutedCandidate(design, rpt);
    if (rpt.hasFail() || totalPenalty(rpt) > 1.0) {
        forceSoftFTRelaxedRouting = true;
        EvalReport relaxedStartRpt;
        Design relaxedStart = bestRoutedCandidate(floorplannedDesign, relaxedStartRpt);
        if (!relaxedStartRpt.hasFail()) {
            design = std::move(relaxedStart);
            rpt = relaxedStartRpt;
        }
    }


    Design bestDesign = design;
    EvalReport bestRpt = rpt;

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
    if (!bestRpt.hasFail() && totalPenalty(bestRpt) > 1.0 && deadspaceRatioMain(bestDesign) > 0.35) {
        const bool largeDeadspaceTrimCase = bestDesign.blocks.size() >= 20;
        vector<pair<double, double>> shrinkFractions;
        vector<vector<pair<double, double>>> largeDeadspaceShrinkSchedule;
        if (largeDeadspaceTrimCase) {
            largeDeadspaceShrinkSchedule = {
                { { 0.000, 0.120 }, { 0.000, 0.090 }, { 0.000, 0.050 } },
                { { 0.020, 0.000 }, { 0.000, 0.010 }, { 0.005, 0.000 } },
                { { 0.000, 0.010 }, { 0.000, 0.005 }, { 0.0025, 0.000 } }
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
            if (initialTrimPenalty <= 1500.0) return initialTrimPenalty + 350.0;
            if (initialTrimPenalty <= 5000.0) return initialTrimPenalty * 1.25 + 500.0;
            if (initialTrimPenalty <= 15000.0) return initialTrimPenalty * 1.50 + 1000.0;
            return max(initialTrimPenalty + 8000.0, initialTrimPenalty * 1.75);
            }();

        auto penaltyMeaningfullyBetter = [&](double ap, double bp) {
            if (ap + 1.0 >= bp) return false;
            const double margin = max(250.0, 0.20 * max(1.0, min(ap, bp)));
            return ap + margin < bp;
            };

        auto betterDeadspaceTrim = [&](const EvalReport& a, const EvalReport& b) {
            const double ap = totalPenalty(a);
            const double bp = totalPenalty(b);
            if (penaltyMeaningfullyBetter(ap, bp)) return true;
            if (penaltyMeaningfullyBetter(bp, ap)) return false;
            if (fabs(a.outlineArea - b.outlineArea) > 1.0) return a.outlineArea < b.outlineArea;
            if (fabs(ap - bp) > 1.0) return ap < bp;
            return a.cost < b.cost;
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
                if (!tryEdgeOnlyOutlineTrimMain(candidate, shrinkW, shrinkH)) {
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
                if (trimPenalty > trimPenaltyBudget) continue;
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

            if (!haveSnap) break;
            const double oldPenalty = totalPenalty(bestRpt);
            const double newPenalty = totalPenalty(bestSnapRpt);
            const bool penaltyImproved = newPenalty + 1.0 < oldPenalty;
            const bool costAcceptable = bestSnapRpt.cost <= bestRpt.cost * 1.15 + 1000000.0;
            const bool neutralPenaltyLowerCost = newPenalty <= oldPenalty + 1.0 && bestSnapRpt.cost + 1.0 < bestRpt.cost;
            if (!(neutralPenaltyLowerCost || (penaltyImproved && costAcceptable))) break;

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
        const double ap = totalPenalty(a);
        const double bp = totalPenalty(b);
        if (ap + 1.0 < bp) return true;
        if (bp + 1.0 < ap) return false;

        // Cost is the objective. Structure is only a tie-breaker; do not pay real
        // cost for a prettier channel graph once penalty is equivalent.
        if (a.cost + 1.0 < b.cost) return true;
        if (b.cost + 1.0 < a.cost) return false;
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
            const double penaltyBudget = utilizationPenaltyBudget(totalPenalty(bestRpt));

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
                if (utilPenalty > penaltyBudget + 1.0e-6) continue;
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
