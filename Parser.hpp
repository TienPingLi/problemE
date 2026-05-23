#pragma once
#include "DataModel.hpp"
#include <string>
#include <vector>

class Parser {
public:
    bool read(const std::string& inputPath, Design& design);

private:
    bool readCSV(const std::string& path, std::vector<std::vector<std::string>>& rows);
    BlockType parseBlockTypeFromRow(const std::vector<std::string>& row);
    std::vector<double> numericValuesInRow(const std::vector<std::string>& row);
    bool parseAspectRange(const std::vector<std::string>& row, double& amin, double& amax);
    std::vector<std::string> parseLocationsFromRow(const std::vector<std::string>& row);
    std::vector<double> parsePercentRates(const std::vector<std::string>& row);

    void parseBlockRows(const std::vector<std::vector<std::string>>& rows, Design& design);
    void parseOutline(const std::vector<std::vector<std::string>>& rows, Design& design);
    void parseConnectionMatrix(const std::vector<std::vector<std::string>>& rows, Design& design);
    void buildConnections(Design& design);
};
