#include "ChannelBuilder.hpp"
#include "Utility.hpp"

#include <algorithm>
#include <cmath>

using namespace std;

namespace {

    // Channel geometry is produced exactly as Problem E describes:
    // collect all block left/right x-edges, sweep each vertical strip, and emit
    // every unblocked rectangle as one CH*.
    //
    // Important: direction-aware capacity is NOT fully representable by the old
    // scalar Channel::capacity field.  The real capacities are:
    //   LR component, edge 1 <-> 3 : rect.h * CHANNEL_DENSITY
    //   TB component, edge 2 <-> 4 : rect.w * CHANNEL_DENSITY
    // The evaluator/router should compute these two values from ch.rect.
    //
    // ChannelBuilder only owns static geometry.  It initializes the legacy scalar
    // capacity conservatively so old diagnostics/router code does not become too
    // optimistic before the router is upgraded to directional usage.
    static double legacyScalarCapacity(const Rect& r) {
        return max(0.0, min(r.w, r.h)) * CHANNEL_DENSITY;
    }

} // namespace

void ChannelBuilder::build(Design& design) {
    design.channels.clear();

    vector<double> xEdges = collectXEdges(design);
    int chCount = 0;

    for (int i = 0; i + 1 < static_cast<int>(xEdges.size()); ++i) {
        double x1 = xEdges[i];
        double x2 = xEdges[i + 1];
        if (x2 - x1 <= EPS) continue;

        vector<pair<double, double>> covered = collectCoveredYIntervals(x1, x2, design);
        vector<pair<double, double>> merged = mergeIntervals(covered);

        double yPrev = 0.0;
        for (const auto& seg : merged) {
            double a = max(0.0, seg.first);
            double b = min(design.outlineH, seg.second);

            if (a - yPrev > EPS) {
                addChannel(design, ++chCount, x1, yPrev, x2 - x1, a - yPrev);
            }
            yPrev = max(yPrev, b);
        }

        if (design.outlineH - yPrev > EPS) {
            addChannel(design, ++chCount, x1, yPrev, x2 - x1, design.outlineH - yPrev);
        }
    }
}

vector<double> ChannelBuilder::collectXEdges(const Design& design) const {
    vector<double> xs;
    xs.push_back(0.0);
    xs.push_back(design.outlineW);

    for (const auto& b : design.blocks) {
        double l = max(0.0, min(design.outlineW, b.rect.x));
        double r = max(0.0, min(design.outlineW, rectRight(b.rect)));
        xs.push_back(l);
        xs.push_back(r);
    }

    sort(xs.begin(), xs.end());
    xs.erase(unique(xs.begin(), xs.end(), [](double a, double b) {
        return fabs(a - b) < 1e-6;
        }), xs.end());
    return xs;
}

vector<pair<double, double>> ChannelBuilder::collectCoveredYIntervals(double x1, double x2, const Design& design) const {
    vector<pair<double, double>> intervals;

    for (const auto& b : design.blocks) {
        bool horizontalOverlap = !(b.rect.x >= x2 - EPS || rectRight(b.rect) <= x1 + EPS);
        if (!horizontalOverlap) continue;

        double y1 = max(0.0, b.rect.y);
        double y2 = min(design.outlineH, rectTop(b.rect));
        if (y2 - y1 > EPS) intervals.push_back({ y1, y2 });
    }

    return intervals;
}

vector<pair<double, double>> ChannelBuilder::mergeIntervals(vector<pair<double, double>> intervals) const {
    vector<pair<double, double>> merged;
    if (intervals.empty()) return merged;

    sort(intervals.begin(), intervals.end());
    for (const auto& in : intervals) {
        if (merged.empty() || in.first > merged.back().second + EPS) {
            merged.push_back(in);
        }
        else {
            merged.back().second = max(merged.back().second, in.second);
        }
    }
    return merged;
}

void ChannelBuilder::addChannel(Design& design, int id, double x, double y, double w, double h) {
    if (w <= EPS || h <= EPS) return;

    Channel ch;
    ch.name = "CH" + to_string(id);
    ch.rect = { x, y, w, h };

    // Legacy scalar only.  Directional capacities are derived from ch.rect by
    // Evaluator and, ideally, by Router:
    //   capLR = h * CHANNEL_DENSITY;
    //   capTB = w * CHANNEL_DENSITY.
    ch.capacity = legacyScalarCapacity(ch.rect);
    ch.usedNets = 0.0;
    ch.overflow = 0.0;

    design.channels.push_back(ch);
}
