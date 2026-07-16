#pragma once
#include "DataModel.hpp"
#include <string>
#include <vector>

class Parser {
public:
    bool read(const std::string& inputPath, Design& design);

    //
    bool parsePortfolioCfg(const char* cfgText, const Design& base, Design& out);

    //
    std::string getLastError() const { return lastError; }

private:
    //
    std::string lastError;

    struct BlockColumns {
        int name = -1;
        int area = -1;
        int width = -1;
        int height = -1;
        int aspect = -1;
        int type = -1;
        int location = -1;
        int portEdge = -1;
        int ftConversion = -1;

        bool valid() const { return name >= 0 && area >= 0 && type >= 0; }
    };

    bool readCSV(const std::string& path, std::vector<std::vector<std::string>>& rows);
    BlockColumns detectBlockColumns(const std::vector<std::vector<std::string>>& rows);
    BlockType parseBlockTypeFromRow(const std::vector<std::string>& row);
    std::vector<double> numericValuesInRow(const std::vector<std::string>& row);
    bool parseAspectRangeCell(const std::string& cell, double& amin, double& amax);
    bool parseAspectRange(const std::vector<std::string>& row, double& amin, double& amax);
    std::vector<std::string> parseLocationsFromRow(const std::vector<std::string>& row);
    std::vector<int> parsePortEdgesFromCell(const std::string& cell);
    std::vector<double> parsePercentRates(const std::vector<std::string>& row);
    std::vector<double> parsePercentRatesFromColumns(const std::vector<std::string>& row, int startCol);

    void parseBlockRows(const std::vector<std::vector<std::string>>& rows, Design& design);
    void parseOutline(const std::vector<std::vector<std::string>>& rows, Design& design);
    void parseAlpha(const std::vector<std::vector<std::string>>& rows, Design& design);
    void parseConnectionMatrix(const std::vector<std::vector<std::string>>& rows, Design& design);
    void buildConnections(Design& design);
};