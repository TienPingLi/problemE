#pragma once
#include "DataModel.hpp"
#include <vector>

class Floorplanner {
public:
    void setEdgePlacementMode(int mode);
    void run(Design& design);

private:
    Rect makeInitialShape(const BlockSpec& spec) const;
    void initBlockShapes(Design& design);
    Rect placeByLocation(const Rect& shape, const std::string& loc, double W, double H) const;
    bool insideOutline(const Rect& r, double W, double H) const;
    bool overlapsPlaced(const Rect& r, const std::vector<BlockInst>& blocks, const std::vector<bool>& placed) const;
    double connectionWeightToPlaced(int blockId, const Design& design, const std::vector<bool>& placed, const Rect& cand) const;
    std::vector<double> collectCandidateXs(const Design& design, double w) const;
    std::vector<double> collectCandidateYs(const Design& design, double h) const;
    void placeEdgeBlocks(Design& design);
    void placeRemainingBlocksGreedy(Design& design);
};
