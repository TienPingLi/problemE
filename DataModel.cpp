#include "DataModel.hpp"
#include "Utility.hpp"
#include <algorithm>
#include <cmath>

using namespace std;

string blockTypeToString(BlockType t) {
    switch (t) {
        case BlockType::EDGE: return "EDGE";
        case BlockType::HARD: return "HARD";
        case BlockType::SOFT: return "SOFT";
        default: return "UNKNOWN";
    }
}

double rectRight(const Rect& r) { return r.x + r.w; }
double rectTop(const Rect& r) { return r.y + r.h; }
double rectCx(const Rect& r) { return r.x + r.w * 0.5; }
double rectCy(const Rect& r) { return r.y + r.h * 0.5; }

double overlapLen(double a1, double a2, double b1, double b2) {
    return max(0.0, min(a2, b2) - max(a1, b1));
}

bool rectOverlapAreaPositive(const Rect& a, const Rect& b) {
    return overlapLen(a.x, rectRight(a), b.x, rectRight(b)) > EPS &&
           overlapLen(a.y, rectTop(a), b.y, rectTop(b)) > EPS;
}

double manhattan(double x1, double y1, double x2, double y2) {
    return fabs(x1 - x2) + fabs(y1 - y2);
}

pair<double, double> edgeCenterPoint(const Rect& r, int edge) {
    switch (edge) {
        case 1: return {r.x, rectCy(r)};          // left
        case 2: return {rectCx(r), rectTop(r)};   // top
        case 3: return {rectRight(r), rectCy(r)}; // right
        case 4: return {rectCx(r), r.y};          // bottom
        default: return {rectCx(r), rectCy(r)};
    }
}

bool EvalReport::hasPenalty() const {
    return totalChannelOverflow > EPS || totalFeedthroughOverflow > EPS;
}

bool EvalReport::hasFail() const {
    return formatFailed || pathInvalid || blockOverlap || routingOpen || outlineViolation;
}
