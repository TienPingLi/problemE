#pragma once
#include "DataModel.hpp"
#include <string>
#include <vector>

class Floorplanner {
public:
    void setRandomSeed(unsigned seed);
    void setTimeBudgetSeconds(double seconds);
    void setEdgePlacementMode(int mode);
    void setRoutingFeedback(const Design& routedDesign);
    void setBoundedFastPathEnabled(bool enabled);
    void runEmergencyFallback(Design& design);
    void run(Design& design);
    const std::vector<Design>& archivedCandidates() const;
    const std::vector<std::string>& archivedCandidateOrigins() const;

private:
    std::vector<Design> archiveDesigns;
    std::vector<std::string> archiveOrigins;
    Design routingFeedbackDesign;
    bool routingFeedbackEnabled = false;
    bool boundedFastPathEnabled = true;
};
