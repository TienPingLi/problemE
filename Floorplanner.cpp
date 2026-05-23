#include "Floorplanner.hpp"
#include "Utility.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

using namespace std;

namespace {

    // -----------------------------------------------------------------------------
    // Placement keepout control
    // -----------------------------------------------------------------------------
    // 這份 Floorplanner 的核心想法：
    //   1. placement 時暫時把 block 加 padding，讓 block 之間自然保留 channel 空間。
    //   2. 找不到合法位置時，padding 會自動縮小重試。
    //   3. 最後輸出 / routing 前，把 block 還原成原本長寬。
    //
    // 你可以先調這兩個數字：
    //   SOFT_BASE_PAD  : soft block 四周預留距離
    //   OTHER_BASE_PAD : EDGE / HARD(MACRO) block 四周預留距離
    // -----------------------------------------------------------------------------
    static constexpr double SOFT_BASE_PAD = 80.0;
    static constexpr double OTHER_BASE_PAD = 60.0;

    // run() 會改這個倍率：1.0 -> 0.75 -> 0.5 -> 0.25 -> 0.0
    // makeInitialShape() 會依照目前倍率產生 inflated block。
    static double gPaddingScale = 1.0;

    static double basePaddingByType(const BlockSpec& spec) {
        if (spec.type == BlockType::SOFT) return SOFT_BASE_PAD;
        return OTHER_BASE_PAD;
    }

    static double placementPadding(const BlockSpec& spec) {
        return basePaddingByType(spec) * gPaddingScale;
    }

    static Rect makeOriginalShapeFromSpec(const BlockSpec& spec) {
        Rect r;

        if (spec.hasFixedSize) {
            r.w = spec.fixedW;
            r.h = spec.fixedH;
            r.x = 0.0;
            r.y = 0.0;
            return r;
        }

        double area = max(1.0, spec.area);
        double ratio = min(max(1.0, spec.aspectMin), spec.aspectMax);
        r.w = sqrt(area * ratio);
        r.h = area / r.w;

        if (r.w <= EPS || r.h <= EPS) {
            r.w = sqrt(area);
            r.h = sqrt(area);
        }

        r.x = 0.0;
        r.y = 0.0;
        return r;
    }

    static Rect placeByLocationLocal(const Rect& shape, const string& loc, double W, double H) {
        Rect r = shape;
        string L = upperStr(loc);

        if (L == "TL") {
            r.x = 0.0;       r.y = H - r.h;
        }
        else if (L == "TM") {
            r.x = (W - r.w) * 0.5; r.y = H - r.h;
        }
        else if (L == "TR") {
            r.x = W - r.w;   r.y = H - r.h;
        }
        else if (L == "BL") {
            r.x = 0.0;       r.y = 0.0;
        }
        else if (L == "BM") {
            r.x = (W - r.w) * 0.5; r.y = 0.0;
        }
        else if (L == "BR") {
            r.x = W - r.w;   r.y = 0.0;
        }
        else if (L == "LT") {
            r.x = 0.0;       r.y = H - r.h;
        }
        else if (L == "LM") {
            r.x = 0.0;       r.y = (H - r.h) * 0.5;
        }
        else if (L == "LB") {
            r.x = 0.0;       r.y = 0.0;
        }
        else if (L == "RT") {
            r.x = W - r.w;   r.y = H - r.h;
        }
        else if (L == "RM") {
            r.x = W - r.w;   r.y = (H - r.h) * 0.5;
        }
        else if (L == "RB") {
            r.x = W - r.w;   r.y = 0.0;
        }
        else {
            r.x = 0.0;       r.y = 0.0;
        }

        r.x = max(0.0, min(r.x, W - r.w));
        r.y = max(0.0, min(r.y, H - r.h));
        return r;
    }

    static bool rectInsideOutlineLocal(const Rect& r, double W, double H) {
        return r.x >= -EPS && r.y >= -EPS && rectRight(r) <= W + EPS && rectTop(r) <= H + EPS;
    }

    static bool validateNoOverlapAndInside(const Design& design) {
        const int n = static_cast<int>(design.blocks.size());

        for (int i = 0; i < n; ++i) {
            if (!rectInsideOutlineLocal(design.blocks[i].rect, design.outlineW, design.outlineH)) {
                return false;
            }
        }

        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                if (rectOverlapAreaPositive(design.blocks[i].rect, design.blocks[j].rect)) {
                    return false;
                }
            }
        }

        return true;
    }

    static void restoreOriginalShapesForOutput(Design& design) {
        for (auto& b : design.blocks) {
            Rect inflated = b.rect;
            Rect orig = makeOriginalShapeFromSpec(b.spec);

            // 一般 block：放在 inflated rect 的中心。
            // 這樣四周會保留 placement padding 形成 channel/spacing。
            orig.x = inflated.x + (inflated.w - orig.w) * 0.5;
            orig.y = inflated.y + (inflated.h - orig.h) * 0.5;

            // EDGE block：輸出前必須重新貼回指定邊界，避免縮回原尺寸後離開邊界。
            if (b.spec.type == BlockType::EDGE) {
                string loc = b.spec.locations.empty() ? "BL" : b.spec.locations[0];
                orig = placeByLocationLocal(orig, loc, design.outlineW, design.outlineH);
            }

            orig.x = max(0.0, min(orig.x, design.outlineW - orig.w));
            orig.y = max(0.0, min(orig.y, design.outlineH - orig.h));
            b.rect = orig;
        }
    }

} // namespace

void Floorplanner::run(Design& design) {
    design.outlineW = design.maxOutlineW;
    design.outlineH = design.maxOutlineH;

    // padding 太大可能放不下，所以自動縮小重試。
    // 只要 inflated block 全部合法，就代表縮回原尺寸後通常會更安全。
    const vector<double> paddingScales = { 1.0, 0.75, 0.5, 0.25, 0.0 };

    bool ok = false;
    double usedScale = 0.0;

    for (double scale : paddingScales) {
        gPaddingScale = scale;

        design.blocks.clear();
        initBlockShapes(design);             // 產生 inflated block
        placeEdgeBlocks(design);             // inflated EDGE block 先貼邊
        placeRemainingBlocksGreedy(design);  // inflated non-edge block greedy placement

        if (validateNoOverlapAndInside(design)) {
            ok = true;
            usedScale = scale;
            break;
        }
    }

    if (!ok) {
        cerr << "[Warning] Floorplanner could not find a fully legal inflated placement. "
            << "Using the last attempt with zero padding.\n";
    }
    else if (usedScale < 1.0) {
        cerr << "[Info] Placement padding was reduced to scale=" << usedScale
            << " to avoid overlap / outline violation.\n";
    }

    // 輸出 / 後續 routing 前，還原原本 block 長寬。
    restoreOriginalShapesForOutput(design);

    if (!validateNoOverlapAndInside(design)) {
        cerr << "[Warning] Restored original block placement still has overlap or outline violation.\n";
    }
}

Rect Floorplanner::makeInitialShape(const BlockSpec& spec) const {
    Rect r = makeOriginalShapeFromSpec(spec);

    // placement-only padding：只影響排版，不影響最後輸出的 block 長寬。
    const double pad = placementPadding(spec);
    r.w += 2.0 * pad;
    r.h += 2.0 * pad;

    return r;
}

void Floorplanner::initBlockShapes(Design& design) {
    design.blocks.reserve(design.blockSpecs.size());
    for (const auto& spec : design.blockSpecs) {
        BlockInst b;
        b.spec = spec;
        b.rect = makeInitialShape(spec);
        design.blocks.push_back(b);
    }
}

Rect Floorplanner::placeByLocation(const Rect& shape, const string& loc, double W, double H) const {
    Rect r = shape;
    string L = upperStr(loc);

    if (L == "TL") {
        r.x = 0.0; r.y = H - r.h;
    }
    else if (L == "TM") {
        r.x = (W - r.w) * 0.5; r.y = H - r.h;
    }
    else if (L == "TR") {
        r.x = W - r.w; r.y = H - r.h;
    }
    else if (L == "BL") {
        r.x = 0.0; r.y = 0.0;
    }
    else if (L == "BM") {
        r.x = (W - r.w) * 0.5; r.y = 0.0;
    }
    else if (L == "BR") {
        r.x = W - r.w; r.y = 0.0;
    }
    else if (L == "LT") {
        r.x = 0.0; r.y = H - r.h;
    }
    else if (L == "LM") {
        r.x = 0.0; r.y = (H - r.h) * 0.5;
    }
    else if (L == "LB") {
        r.x = 0.0; r.y = 0.0;
    }
    else if (L == "RT") {
        r.x = W - r.w; r.y = H - r.h;
    }
    else if (L == "RM") {
        r.x = W - r.w; r.y = (H - r.h) * 0.5;
    }
    else if (L == "RB") {
        r.x = W - r.w; r.y = 0.0;
    }
    else {
        r.x = 0.0; r.y = 0.0;
    }

    r.x = max(0.0, min(r.x, W - r.w));
    r.y = max(0.0, min(r.y, H - r.h));
    return r;
}

bool Floorplanner::insideOutline(const Rect& r, double W, double H) const {
    return r.x >= -EPS && r.y >= -EPS && rectRight(r) <= W + EPS && rectTop(r) <= H + EPS;
}

bool Floorplanner::overlapsPlaced(const Rect& r, const vector<BlockInst>& blocks, const vector<bool>& placed) const {
    for (int i = 0; i < static_cast<int>(blocks.size()); ++i) {
        if (!placed[i]) continue;
        if (rectOverlapAreaPositive(r, blocks[i].rect)) return true;
    }
    return false;
}

void Floorplanner::placeEdgeBlocks(Design& design) {
    for (auto& b : design.blocks) {
        if (b.spec.type != BlockType::EDGE) continue;
        string loc = b.spec.locations.empty() ? "BL" : b.spec.locations[0];
        b.rect = placeByLocation(b.rect, loc, design.outlineW, design.outlineH);
    }
}

double Floorplanner::connectionWeightToPlaced(int blockId, const Design& design, const vector<bool>& placed, const Rect& cand) const {
    double score = 0.0;
    int n = static_cast<int>(design.blocks.size());

    for (int j = 0; j < n; ++j) {
        if (!placed[j]) continue;

        int nets = 0;
        if (blockId < static_cast<int>(design.connMatrix.size()) && j < static_cast<int>(design.connMatrix[blockId].size())) {
            nets += design.connMatrix[blockId][j];
        }
        if (j < static_cast<int>(design.connMatrix.size()) && blockId < static_cast<int>(design.connMatrix[j].size())) {
            nets += design.connMatrix[j][blockId];
        }

        if (nets > 0) {
            score += nets * manhattan(rectCx(cand), rectCy(cand), rectCx(design.blocks[j].rect), rectCy(design.blocks[j].rect));
        }
    }
    return score;
}

vector<double> Floorplanner::collectCandidateXs(const Design& design, double w) const {
    vector<double> xs = { 0.0 };
    for (const auto& b : design.blocks) {
        xs.push_back(b.rect.x);
        xs.push_back(rectRight(b.rect));
        xs.push_back(max(0.0, b.rect.x - w));
    }

    double step = max(50.0, design.outlineW / 20.0);
    for (double x = 0.0; x + w <= design.outlineW + EPS; x += step) xs.push_back(x);

    for (double& x : xs) x = max(0.0, min(x, design.outlineW - w));
    sort(xs.begin(), xs.end());
    xs.erase(unique(xs.begin(), xs.end(), [](double a, double b) { return fabs(a - b) < 1e-3; }), xs.end());
    return xs;
}

vector<double> Floorplanner::collectCandidateYs(const Design& design, double h) const {
    vector<double> ys = { 0.0 };
    for (const auto& b : design.blocks) {
        ys.push_back(b.rect.y);
        ys.push_back(rectTop(b.rect));
        ys.push_back(max(0.0, b.rect.y - h));
    }

    double step = max(50.0, design.outlineH / 20.0);
    for (double y = 0.0; y + h <= design.outlineH + EPS; y += step) ys.push_back(y);

    for (double& y : ys) y = max(0.0, min(y, design.outlineH - h));
    sort(ys.begin(), ys.end());
    ys.erase(unique(ys.begin(), ys.end(), [](double a, double b) { return fabs(a - b) < 1e-3; }), ys.end());
    return ys;
}

void Floorplanner::placeRemainingBlocksGreedy(Design& design) {
    int n = static_cast<int>(design.blocks.size());
    vector<bool> placed(n, false);

    for (int i = 0; i < n; ++i) {
        if (design.blocks[i].spec.type == BlockType::EDGE) placed[i] = true;
    }

    vector<int> order;
    for (int i = 0; i < n; ++i) if (!placed[i]) order.push_back(i);

    auto connDegree = [&](int id) {
        int sum = 0;
        for (int j = 0; j < n; ++j) {
            if (id < static_cast<int>(design.connMatrix.size()) && j < static_cast<int>(design.connMatrix[id].size())) {
                sum += design.connMatrix[id][j];
            }
            if (j < static_cast<int>(design.connMatrix.size()) && id < static_cast<int>(design.connMatrix[j].size())) {
                sum += design.connMatrix[j][id];
            }
        }
        return sum;
        };

    sort(order.begin(), order.end(), [&](int a, int b) {
        if (design.blocks[a].spec.type != design.blocks[b].spec.type) {
            if (design.blocks[a].spec.type == BlockType::HARD) return true;
            if (design.blocks[b].spec.type == BlockType::HARD) return false;
        }
        int ca = connDegree(a);
        int cb = connDegree(b);
        if (ca != cb) return ca > cb;
        return design.blocks[a].spec.area > design.blocks[b].spec.area;
        });

    for (int id : order) {
        Rect shape = design.blocks[id].rect;
        vector<double> xs = collectCandidateXs(design, shape.w);
        vector<double> ys = collectCandidateYs(design, shape.h);

        bool found = false;
        Rect best = shape;
        double bestScore = numeric_limits<double>::infinity();

        for (double y : ys) {
            for (double x : xs) {
                Rect cand = shape;
                cand.x = x;
                cand.y = y;

                if (!insideOutline(cand, design.outlineW, design.outlineH)) continue;
                if (overlapsPlaced(cand, design.blocks, placed)) continue;

                double hpwlScore = connectionWeightToPlaced(id, design, placed, cand);
                double centerBias = 0.001 * manhattan(rectCx(cand), rectCy(cand), design.outlineW * 0.5, design.outlineH * 0.5);
                double score = hpwlScore + centerBias;

                if (score < bestScore) {
                    bestScore = score;
                    best = cand;
                    found = true;
                }
            }
        }

        // fallback：更密集掃描。這裡補上 insideOutline，避免邊界誤差。
        if (!found) {
            double yStep = max(20.0, shape.h * 0.5);
            double xStep = max(20.0, shape.w * 0.5);

            for (double y = 0.0; y + shape.h <= design.outlineH + EPS && !found; y += yStep) {
                for (double x = 0.0; x + shape.w <= design.outlineW + EPS; x += xStep) {
                    Rect cand = shape;
                    cand.x = x;
                    cand.y = y;
                    if (insideOutline(cand, design.outlineW, design.outlineH) &&
                        !overlapsPlaced(cand, design.blocks, placed)) {
                        best = cand;
                        found = true;
                        break;
                    }
                }
            }
        }

        if (!found) {
            cerr << "[Warning] Cannot legally place one block with current padding scale="
                << gPaddingScale << ". The outer retry loop may reduce padding.\n";
        }

        design.blocks[id].rect = best;
        placed[id] = true;
    }
}
