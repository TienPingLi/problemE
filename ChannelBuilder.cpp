#include "ChannelBuilder.hpp"
#include "Utility.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>

using namespace std;

namespace {

    // Channel geometry follows the Figure 13 style vertical decomposition:
    // each block left/right edge creates vertical cut segments only through
    // visible free space.  A segment stops when it reaches another block or the
    // outline, so the same x coordinate does not keep cutting unrelated free
    // space above/below a blocking rectangle.
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

    struct VerticalCut {
        double x = 0.0;
        double y1 = 0.0;
        double y2 = 0.0;
    };

    static bool coordEqual(double a, double b) {
        return fabs(a - b) <= 1.0e-6;
    }

    static bool xPiercesBlock(double x, const Rect& r) {
        return x >= r.x - EPS && x <= rectRight(r) + EPS;
    }

    static bool rectsOverlapPositive(const Rect& a, const Rect& b) {
        return min(rectRight(a), rectRight(b)) > max(a.x, b.x) + EPS &&
            min(rectTop(a), rectTop(b)) > max(a.y, b.y) + EPS;
    }

    static void addUniqueCoord(vector<double>& values, double v) {
        for (double old : values) {
            if (coordEqual(old, v)) return;
        }
        values.push_back(v);
    }

    static void addCut(vector<VerticalCut>& cuts, double x, double y1, double y2) {
        y1 = max(0.0, y1);
        y2 = max(0.0, y2);
        if (y2 - y1 <= EPS) return;
        for (const auto& old : cuts) {
            if (coordEqual(old.x, x) && coordEqual(old.y1, y1) && coordEqual(old.y2, y2)) return;
        }
        cuts.push_back({ x, y1, y2 });
    }

    static vector<VerticalCut> buildVisibleVerticalCuts(const Design& design) {
        vector<VerticalCut> cuts;
        for (const auto& b : design.blocks) {
            const double ys[2] = { b.rect.y, rectTop(b.rect) };
            const double xs[2] = { b.rect.x, rectRight(b.rect) };
            for (double x : xs) {
                if (x <= EPS || x >= design.outlineW - EPS) continue;

                const double yBottom = ys[0];
                const double yTop = ys[1];
                double stopUp = design.outlineH;
                double stopDown = 0.0;

                for (const auto& other : design.blocks) {
                    if (&other == &b) continue;
                    if (!xPiercesBlock(x, other.rect)) continue;
                    if (other.rect.y >= yTop - EPS) {
                        stopUp = min(stopUp, other.rect.y);
                    }
                    if (rectTop(other.rect) <= yBottom + EPS) {
                        stopDown = max(stopDown, rectTop(other.rect));
                    }
                }

                addCut(cuts, x, yTop, stopUp);
                addCut(cuts, x, stopDown, yBottom);
            }
        }
        return cuts;
    }

    static bool rectCoveredByAnyBlock(double x1, double x2, double y1, double y2, const Design& design) {
        Rect cell{ x1, y1, x2 - x1, y2 - y1 };
        for (const auto& b : design.blocks) {
            if (rectsOverlapPositive(cell, b.rect)) return true;
        }
        return false;
    }

    static bool hasVerticalWall(const vector<VerticalCut>& cuts, double x, double y1, double y2) {
        for (const auto& cut : cuts) {
            if (!coordEqual(cut.x, x)) continue;
            if (cut.y1 <= y1 + EPS && cut.y2 >= y2 - EPS) return true;
        }
        return false;
    }

    struct DSU {
        vector<int> parent;
        vector<int> rank;
        explicit DSU(int n = 0) : parent(n), rank(n, 0) {
            iota(parent.begin(), parent.end(), 0);
        }
        int find(int x) {
            if (parent[x] == x) return x;
            parent[x] = find(parent[x]);
            return parent[x];
        }
        void unite(int a, int b) {
            a = find(a);
            b = find(b);
            if (a == b) return;
            if (rank[a] < rank[b]) swap(a, b);
            parent[b] = a;
            if (rank[a] == rank[b]) ++rank[a];
        }
    };

} // namespace

void ChannelBuilder::build(Design& design) {
    design.channels.clear();

    vector<VerticalCut> cuts = buildVisibleVerticalCuts(design);
    vector<double> xEdges;
    vector<double> yEdges;
    addUniqueCoord(xEdges, 0.0);
    addUniqueCoord(xEdges, design.outlineW);
    addUniqueCoord(yEdges, 0.0);
    addUniqueCoord(yEdges, design.outlineH);

    for (const auto& b : design.blocks) {
        addUniqueCoord(xEdges, max(0.0, min(design.outlineW, b.rect.x)));
        addUniqueCoord(xEdges, max(0.0, min(design.outlineW, rectRight(b.rect))));
        addUniqueCoord(yEdges, max(0.0, min(design.outlineH, b.rect.y)));
        addUniqueCoord(yEdges, max(0.0, min(design.outlineH, rectTop(b.rect))));
    }
    for (const auto& cut : cuts) {
        addUniqueCoord(xEdges, max(0.0, min(design.outlineW, cut.x)));
        addUniqueCoord(yEdges, max(0.0, min(design.outlineH, cut.y1)));
        addUniqueCoord(yEdges, max(0.0, min(design.outlineH, cut.y2)));
    }

    sort(xEdges.begin(), xEdges.end());
    sort(yEdges.begin(), yEdges.end());

    const int nx = static_cast<int>(xEdges.size()) - 1;
    const int ny = static_cast<int>(yEdges.size()) - 1;
    if (nx <= 0 || ny <= 0) return;

    vector<char> empty(nx * ny, 0);
    auto id = [ny](int ix, int iy) { return ix * ny + iy; };

    for (int ix = 0; ix < nx; ++ix) {
        for (int iy = 0; iy < ny; ++iy) {
            const double x1 = xEdges[ix], x2 = xEdges[ix + 1];
            const double y1 = yEdges[iy], y2 = yEdges[iy + 1];
            if (x2 - x1 <= EPS || y2 - y1 <= EPS) continue;
            empty[id(ix, iy)] = !rectCoveredByAnyBlock(x1, x2, y1, y2, design);
        }
    }

    DSU dsu(nx * ny);
    for (int ix = 0; ix < nx; ++ix) {
        for (int iy = 0; iy < ny; ++iy) {
            if (!empty[id(ix, iy)]) continue;
            if (iy + 1 < ny && empty[id(ix, iy + 1)]) {
                dsu.unite(id(ix, iy), id(ix, iy + 1));
            }
            if (ix + 1 < nx && empty[id(ix + 1, iy)] &&
                !hasVerticalWall(cuts, xEdges[ix + 1], yEdges[iy], yEdges[iy + 1])) {
                dsu.unite(id(ix, iy), id(ix + 1, iy));
            }
        }
    }

    map<int, vector<pair<int, int>>> components;
    for (int ix = 0; ix < nx; ++ix) {
        for (int iy = 0; iy < ny; ++iy) {
            if (!empty[id(ix, iy)]) continue;
            components[dsu.find(id(ix, iy))].push_back({ ix, iy });
        }
    }

    int chCount = 0;
    for (const auto& kv : components) {
        int minIx = nx, maxIx = -1, minIy = ny, maxIy = -1;
        double area = 0.0;
        for (const auto& cell : kv.second) {
            minIx = min(minIx, cell.first);
            maxIx = max(maxIx, cell.first + 1);
            minIy = min(minIy, cell.second);
            maxIy = max(maxIy, cell.second + 1);
            area += (xEdges[cell.first + 1] - xEdges[cell.first]) *
                (yEdges[cell.second + 1] - yEdges[cell.second]);
        }
        if (minIx >= maxIx || minIy >= maxIy) continue;

        const double x = xEdges[minIx];
        const double y = yEdges[minIy];
        const double w = xEdges[maxIx] - x;
        const double h = yEdges[maxIy] - y;
        if (fabs(area - w * h) <= 1.0e-5 * max(1.0, w * h) &&
            !rectCoveredByAnyBlock(x, x + w, y, y + h, design)) {
            addChannel(design, ++chCount, x, y, w, h);
            continue;
        }

        map<int, vector<int>> byRow;
        for (const auto& cell : kv.second) byRow[cell.second].push_back(cell.first);
        for (auto& row : byRow) {
            vector<int>& xs = row.second;
            sort(xs.begin(), xs.end());
            int runStart = xs.front();
            int prev = xs.front();
            for (int p = 1; p <= static_cast<int>(xs.size()); ++p) {
                if (p < static_cast<int>(xs.size()) && xs[p] == prev + 1) {
                    prev = xs[p];
                    continue;
                }
                addChannel(design, ++chCount,
                    xEdges[runStart], yEdges[row.first],
                    xEdges[prev + 1] - xEdges[runStart],
                    yEdges[row.first + 1] - yEdges[row.first]);
                if (p < static_cast<int>(xs.size())) {
                    runStart = prev = xs[p];
                }
            }
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
