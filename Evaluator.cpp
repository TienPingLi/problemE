#include "Evaluator.hpp"
#include "Utility.hpp"
#include <algorithm>

using namespace std;

EvalReport Evaluator::evaluate(Design& design, double alpha) {
    EvalReport rpt;

    rpt.outlineArea = design.outlineW * design.outlineH;
    rpt.totalWireLength = calcTotalWireLength(design);
    rpt.cost = rpt.outlineArea + alpha * rpt.totalWireLength;

    rpt.blockOverlap = checkBlockOverlap(design, rpt.overlapCount);
    rpt.outlineViolation = checkOutlineViolation(design, rpt.outlineViolationCount);
    rpt.routingOpen = checkRoutingOpen(design, rpt.openPathCount);

    calcChannelOverflow(design, rpt.totalChannelOverflow, rpt.maxChannelOverflow);
    calcFeedthroughOverflow(design, rpt.totalFeedthroughOverflow, rpt.maxFeedthroughOverflow);

    return rpt;
}

double Evaluator::calcTotalWireLength(const Design& design) const {
    double sum = 0.0;
    for (const auto& p : design.routes) sum += p.wireLength;
    return sum;
}

bool Evaluator::checkBlockOverlap(const Design& design, int& overlapCount) const {
    overlapCount = 0;
    for (int i = 0; i < static_cast<int>(design.blocks.size()); ++i) {
        for (int j = i + 1; j < static_cast<int>(design.blocks.size()); ++j) {
            if (rectOverlapAreaPositive(design.blocks[i].rect, design.blocks[j].rect)) ++overlapCount;
        }
    }
    return overlapCount > 0;
}

bool Evaluator::checkOutlineViolation(const Design& design, int& violationCount) const {
    violationCount = 0;

    if (design.outlineW > design.maxOutlineW + EPS || design.outlineH > design.maxOutlineH + EPS) ++violationCount;

    for (const auto& b : design.blocks) {
        const Rect& r = b.rect;
        if (r.x < -EPS || r.y < -EPS || rectRight(r) > design.outlineW + EPS || rectTop(r) > design.outlineH + EPS) {
            ++violationCount;
        }
    }
    return violationCount > 0;
}

bool Evaluator::checkRoutingOpen(const Design& design, int& openPathCount) const {
    openPathCount = 0;
    for (const auto& p : design.routes) {
        if (p.open) ++openPathCount;
    }
    return openPathCount > 0;
}

void Evaluator::calcChannelOverflow(Design& design, double& totalOverflow, double& maxOverflow) const {
    totalOverflow = 0.0;
    maxOverflow = 0.0;

    for (auto& ch : design.channels) {
        ch.overflow = max(0.0, ch.usedNets - ch.capacity);
        totalOverflow += ch.overflow;
        maxOverflow = max(maxOverflow, ch.overflow);
    }
}

double Evaluator::ftRateForNets(const BlockSpec& spec, double ftNets) const {
    if (ftNets <= 3000.0) return spec.ftRate[0];
    if (ftNets <= 6000.0) return spec.ftRate[1];
    if (ftNets <= 9000.0) return spec.ftRate[2];
    return spec.ftRate[3];
}

double Evaluator::estimateRequiredAreaWithFT(const BlockInst& b) const {
    if (b.spec.type != BlockType::SOFT || b.ftUsed <= EPS) return b.rect.w * b.rect.h;

    double rate = ftRateForNets(b.spec, b.ftUsed);
    double delta = (b.ftUsed / CHANNEL_DENSITY) * rate / 2.0;
    double reqW = b.rect.w + delta;
    double reqH = b.rect.h + delta;
    return reqW * reqH;
}

void Evaluator::calcFeedthroughOverflow(Design& design, double& totalOverflow, double& maxOverflow) const {
    totalOverflow = 0.0;
    maxOverflow = 0.0;

    for (auto& b : design.blocks) {
        b.ftOverflowArea = 0.0;
        if (b.spec.type != BlockType::SOFT || b.ftUsed <= EPS) continue;

        double currentArea = b.rect.w * b.rect.h;
        double requiredArea = estimateRequiredAreaWithFT(b);
        b.ftOverflowArea = max(0.0, requiredArea - currentArea);
        totalOverflow += b.ftOverflowArea;
        maxOverflow = max(maxOverflow, b.ftOverflowArea);
    }
}
