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
    // Capacity is whole-channel aggregate per direction (spec: 25 nets/um),
    // not per-segment/track: LR capacity comes from channel height, TB
    // capacity comes from channel width. No further slicing is needed here.
    static double lrCapacity(const Rect& r) {
        return max(0.0, r.h) * CHANNEL_DENSITY;
    }

    static double tbCapacity(const Rect& r) {
        return max(0.0, r.w) * CHANNEL_DENSITY;
    }

} // namespace

void ChannelBuilder::build(Design& design) {
    design.channels.clear();

    vector<double> xEdges = collectXEdges(design);
    int chCount = 0;

    // A four-side routing candidate has an inner placement core and a larger
    // declared outline.  Cut otherwise-maximal vacant rectangles at the core
    // boundary so a required perimeter channel is geometrically outside the
    // old outline, rather than merely being one tall/wide channel which also
    // happens to reach the new outline.  Ordinary designs have no routing core
    // and therefore retain the attachment's exact maximal-strip construction.
    const bool splitAtRoutingCore =
        design.hasRoutingCore && design.routingCoreW > EPS &&
        design.routingCoreH > EPS;
    const double coreBottom = design.routingCoreY;
    const double coreTop = design.routingCoreY + design.routingCoreH;

    auto addVacantRange = [&](double x, double w, double y1, double y2) {
        if (y2 - y1 <= EPS) return;
        if (!splitAtRoutingCore) {
            addChannel(design, ++chCount, x, y1, w, y2 - y1);
            return;
        }

        vector<double> cuts{ y1, y2 };
        if (coreBottom > y1 + EPS && coreBottom < y2 - EPS)
            cuts.push_back(coreBottom);
        if (coreTop > y1 + EPS && coreTop < y2 - EPS)
            cuts.push_back(coreTop);
        sort(cuts.begin(), cuts.end());
        cuts.erase(unique(cuts.begin(), cuts.end(), [](double a, double b) {
            return fabs(a - b) < 1e-6;
        }), cuts.end());
        for (size_t k = 0; k + 1 < cuts.size(); ++k) {
            addChannel(design, ++chCount, x, cuts[k], w,
                       cuts[k + 1] - cuts[k]);
        }
    };

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
                addVacantRange(x1, x2 - x1, yPrev, a);
            }
            yPrev = max(yPrev, b);
        }

        if (design.outlineH - yPrev > EPS) {
            addVacantRange(x1, x2 - x1, yPrev, design.outlineH);
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

    if (design.hasRoutingCore && design.routingCoreW > EPS) {
        const double l = max(0.0, min(design.outlineW, design.routingCoreX));
        const double r = max(0.0, min(design.outlineW,
                                      design.routingCoreX + design.routingCoreW));
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

    ch.lrCapacity = lrCapacity(ch.rect);
    ch.tbCapacity = tbCapacity(ch.rect);
    ch.lrUsed = 0.0;
    ch.tbUsed = 0.0;
    ch.lrOverflow = 0.0;
    ch.tbOverflow = 0.0;

    design.channels.push_back(ch);
}
