#pragma once
#include "DataModel.hpp"
#include <string>
#include <vector>

class Floorplanner {
public:
    void setEdgePlacementMode(int mode);
    void setRoutingFeedback(const Design& routedDesign);
    void run(Design& design);
    const std::vector<Design>& archivedCandidates() const;
    const std::vector<std::string>& archivedCandidateOrigins() const;

private:
    std::vector<Design> archiveDesigns;
    std::vector<std::string> archiveOrigins;
    Design routingFeedbackDesign;
    bool routingFeedbackEnabled = false;
};
