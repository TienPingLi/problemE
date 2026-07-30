#pragma once
#include "DataModel.hpp"

class Evaluator {
public:
    EvalReport evaluate(Design& design, double alpha, double runtimeSec = 0.0);

private:
    double calcTotalWireLength(const Design& design) const;
    bool checkBlockOverlap(const Design& design, int& overlapCount) const;
    bool checkOutlineViolation(const Design& design, int& violationCount) const;
    bool checkRoutingOpen(const Design& design, int& openPathCount) const;
    void calcChannelOverflow(Design& design, double& totalOverflow, double& maxOverflow, double& totalCapacity) const;
    double ftRateForNets(const BlockSpec& spec, double ftNets) const;
    double estimateRequiredAreaWithFT(const BlockInst& b) const;
    void calcFeedthroughOverflow(Design& design, double& totalOverflow, double& maxOverflow) const;
};
