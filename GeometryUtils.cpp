#include "GeometryUtils.hpp"
#include "Utility.hpp"
#include <algorithm>
#include <cmath>
#include <array>

namespace GeometryUtils {

    bool rectInsideOutline(const Rect& r, double W, double H) {
        return r.x >= -EPS && r.y >= -EPS &&
            rectRight(r) <= W + EPS && rectTop(r) <= H + EPS;
    }

    bool overlapsAnyOtherBlock(const std::vector<BlockInst>& blocks, int id, const Rect& cand) {
        for (int i = 0; i < static_cast<int>(blocks.size()); ++i) {
            if (i == id) continue;
            if (rectOverlapAreaPositive(cand, blocks[i].rect)) return true;
        }
        return false;
    }

    bool placementLegalAfterMove(const Design& design, const std::vector<BlockInst>& blocks) {
        for (const auto& b : blocks) {
            // 
            if (!rectInsideOutline(b.rect, design.outlineW, design.outlineH)) return false;
        }
        for (int i = 0; i < static_cast<int>(blocks.size()); ++i) {
            for (int j = i + 1; j < static_cast<int>(blocks.size()); ++j) {
                if (rectOverlapAreaPositive(blocks[i].rect, blocks[j].rect)) return false;
            }
        }
        return true;
    }

    double placedBlockArea(const Design& design) {
        double area = 0.0;
        for (const auto& b : design.blocks) {
            area += std::max(0.0, b.rect.w * b.rect.h);
        }
        return area;
    }

    double deadspaceRatio(const Design& design) {
        const double outlineArea = std::max(1.0, design.outlineW * design.outlineH);
        return std::max(0.0, (outlineArea - placedBlockArea(design)) / outlineArea);
    }

    bool isChannelName(const std::string& name) {
        return name.rfind("CH", 0) == 0;
    }

    std::vector<int> legalEdgesForDetourMove(const BlockSpec& spec) {
        if (!spec.portEdges.empty()) return spec.portEdges;
        return { 1, 2, 3, 4 };
    }

    std::pair<double, double> edgeAnchorForDetourMove(const Rect& r, int edge, double t) {
        t = std::max(0.0, std::min(1.0, t));
        if (edge == 1) return { r.x, r.y + r.h * t };
        if (edge == 3) return { rectRight(r), r.y + r.h * t };
        if (edge == 2) return { r.x + r.w * t, rectTop(r) };
        if (edge == 4) return { r.x + r.w * t, r.y };
        return { rectCx(r), rectCy(r) };
    }

    double portAwareLowerBoundWL(const Design& design, int srcId, int dstId, int nets) {
        if (srcId < 0 || dstId < 0 || srcId >= static_cast<int>(design.blocks.size()) || dstId >= static_cast<int>(design.blocks.size()))
            return 0.0;

        const BlockInst& src = design.blocks[srcId];
        const BlockInst& dst = design.blocks[dstId];

        const auto srcEdges = legalEdgesForDetourMove(src.spec);
        const auto dstEdges = legalEdgesForDetourMove(dst.spec);
        const std::array<double, 3> taps = { 0.25, 0.50, 0.75 };

        double best = 1.0e100;
        for (const int se : srcEdges) {
            for (const int de : dstEdges) {
                for (const double st : taps) {
                    const auto sp = edgeAnchorForDetourMove(src.rect, se, st);
                    for (const double dt : taps) {
                        const auto dp = edgeAnchorForDetourMove(dst.rect, de, dt);
                        best = std::min(best, manhattan(sp.first, sp.second, dp.first, dp.second));
                    }
                }
            }
        }
        if (best > 1.0e90)
            best = manhattan(rectCx(src.rect), rectCy(src.rect), rectCx(dst.rect), rectCy(dst.rect));

        return best * static_cast<double>(std::max(0, nets));
    }
}