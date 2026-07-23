#ifndef GEOMETRY_UTILS_HPP
#define GEOMETRY_UTILS_HPP

#include "DataModel.hpp"
#include <vector>
#include <string>
#include <utility>

namespace GeometryUtils {
    // 
    bool rectInsideOutline(const Rect& r, double W, double H);
    bool overlapsAnyOtherBlock(const std::vector<BlockInst>& blocks, int id, const Rect& cand);
    bool placementLegalAfterMove(const Design& design, const std::vector<BlockInst>& blocks);

    // 
    double placedBlockArea(const Design& design);
    double deadspaceRatio(const Design& design);

    // 
    bool isChannelName(const std::string& name);

    // 
    std::vector<int> legalEdgesForDetourMove(const BlockSpec& spec);
    std::pair<double, double> edgeAnchorForDetourMove(const Rect& r, int edge, double t);
    double portAwareLowerBoundWL(const Design& design, int srcId, int dstId, int nets);
}

#endif