#pragma once
#include "DataModel.hpp"
#include <utility>
#include <vector>

class ChannelBuilder {
public:
    void build(Design& design);

private:
    std::vector<double> collectXEdges(const Design& design) const;
    std::vector<std::pair<double, double>> collectCoveredYIntervals(double x1, double x2, const Design& design) const;
    std::vector<std::pair<double, double>> mergeIntervals(std::vector<std::pair<double, double>> intervals) const;
    void addChannel(Design& design, int id, double x, double y, double w, double h);
};
