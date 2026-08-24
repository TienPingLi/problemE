#pragma once
#include "DataModel.hpp"
#include <string>
#include <vector>

class Router {
public:
    enum class ChannelDir {
        LR,
        TB
    };

    struct ResourceNeed {
        int channelIndex = -1;
        ChannelDir dir = ChannelDir::LR;
        double currentUsed = 0.0;
        double capacity = 0.0;
        double extraDemand = 0.0;
        double shortageNets = 0.0;
        double requiredDeltaUm = 0.0;
    };

    struct FailureCertificate {
        Connection conn;
        RoutePath diagnosticPath;
        bool diagnosticOpen = true;
        bool geometryDisconnected = false;
        bool policyBlocked = false;
        bool hasLegalFTAlternative = false;
        std::vector<ResourceNeed> hardNeeds;
        std::vector<ResourceNeed> riskNeeds;
    };

    void setFTOverflowCostEnabled(bool enabled);
    void setSoftFTCostRelaxed(bool enabled);
    void setContactAwareCostEnabled(bool enabled);
    void setDetailedFailureAnalysisEnabled(bool enabled);
    void run(Design& design);
    void printDetourReport(const Design& design) const;
    const std::vector<FailureCertificate>& failureCertificates() const;

private:
    std::vector<FailureCertificate> lastFailureCertificates;
    bool detailedFailureAnalysisEnabled = false;
    bool ftOverflowCostEnabled = false;
    bool softFTCostRelaxed = false;
    bool contactAwareCostEnabled = true;
};
