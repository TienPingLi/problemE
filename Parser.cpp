#include "Parser.hpp"
#include "Utility.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <unordered_map>

using namespace std;

bool Parser::read(const string& inputPath, Design& design) {
    vector<vector<string>> rows;
    if (!readCSV(inputPath, rows)) {
        cerr << "[Parser] Cannot open input: " << inputPath << "\n";
        return false;
    }

    parseBlockRows(rows, design);
    parseOutline(rows, design);
    parseConnectionMatrix(rows, design);
    buildConnections(design);

    if (design.blockSpecs.empty()) {
        cerr << "[Parser] No block specs parsed.\n";
        return false;
    }

    if (design.maxOutlineW <= EPS || design.maxOutlineH <= EPS) {
        cerr << "[Parser] Invalid outline. maxOutlineW=" << design.maxOutlineW
            << " maxOutlineH=" << design.maxOutlineH << "\n";
        return false;
    }

    if (design.connMatrix.empty()) {
        cerr << "[Parser] Warning: no connection matrix parsed. Continue with no routes.\n";
        int n = static_cast<int>(design.blockSpecs.size());
        design.connMatrix.assign(n, vector<int>(n, 0));
    }

    design.blockNameToIndex.clear();
    for (int i = 0; i < static_cast<int>(design.blockSpecs.size()); ++i) {
        design.blockNameToIndex[design.blockSpecs[i].name] = i;
    }

    return true;
}

bool Parser::readCSV(const string& path, vector<vector<string>>& rows) {
    ifstream fin(path);
    if (!fin) return false;

    string line;
    while (getline(fin, line)) {
        if (!line.empty() && static_cast<unsigned char>(line[0]) == 0xEF) {
            if (line.size() >= 3) line = line.substr(3);
        }
        rows.push_back(splitCSVLine(line));
    }
    return true;
}

BlockType Parser::parseBlockTypeFromRow(const vector<string>& row) {
    for (const string& cell : row) {
        string u = upperStr(trim(cell));
        if (u == "EDGE") return BlockType::EDGE;
        if (u == "SOFT") return BlockType::SOFT;
        if (u == "MACRO" || u == "HARD" || u == "HARD MACRO" || u == "HARDMACRO") {
            return BlockType::HARD;
        }
    }
    return BlockType::UNKNOWN;
}

vector<double> Parser::numericValuesInRow(const vector<string>& row) {
    vector<double> nums;
    for (const string& cell : row) {
        string t = trim(cell);
        if (t.empty()) continue;
        if (t.find(',') != string::npos && t.find('%') == string::npos) continue;

        double v;
        if (tryParseDouble(t, v)) nums.push_back(v);
    }
    return nums;
}

bool Parser::parseAspectRange(const vector<string>& row, double& amin, double& amax) {
    for (const string& cell : row) {
        string t = trim(cell);
        if (t.empty()) continue;

        if (t.find(',') != string::npos) {
            vector<string> parts;
            string cur;
            for (char c : t) {
                if (c == ',') {
                    if (!trim(cur).empty()) parts.push_back(trim(cur));
                    cur.clear();
                }
                else {
                    cur.push_back(c);
                }
            }
            if (!trim(cur).empty()) parts.push_back(trim(cur));

            if (parts.size() >= 2) {
                double a, b;
                if (tryParseDouble(parts[0], a) && tryParseDouble(parts[1], b)) {
                    amin = min(a, b);
                    amax = max(a, b);
                    return true;
                }
            }
        }
    }
    return false;
}

vector<string> Parser::parseLocationsFromRow(const vector<string>& row) {
    static const set<string> legal = {
        "TL", "TM", "TR", "BL", "BM", "BR",
        "LT", "LM", "LB", "RT", "RM", "RB"
    };

    vector<string> locs;
    for (const string& cell : row) {
        vector<string> parts = splitByCommaOrSlash(cell);
        for (string p : parts) {
            p = upperStr(trim(p));
            if (legal.count(p)) locs.push_back(p);
        }
    }
    return locs;
}

vector<double> Parser::parsePercentRates(const vector<string>& row) {
    vector<double> rates;
    for (const string& cell : row) {
        string t = trim(cell);
        if (t.empty()) continue;
        if (t.find('%') != string::npos) {
            double v = 0.0;
            if (tryParseDouble(t, v)) rates.push_back(v / 100.0);
        }
    }
    return rates;
}

void Parser::parseBlockRows(const vector<vector<string>>& rows, Design& design) {
    design.blockSpecs.clear();

    for (const auto& row : rows) {
        int first = firstNonEmptyIndex(row);
        if (first < 0) continue;

        string name = trim(row[first]);
        if (!startsWithBlockName(name)) continue;

        BlockType type = parseBlockTypeFromRow(row);
        if (type == BlockType::UNKNOWN) continue;

        vector<double> nums = numericValuesInRow(row);
        if (nums.empty()) continue;

        BlockSpec spec;
        spec.name = name;
        spec.type = type;
        spec.area = nums[0];

        double w = 0.0;
        double h = 0.0;
        if (nums.size() >= 3) {
            w = nums[1];
            h = nums[2];
            if (type == BlockType::SOFT && (w <= 100.0 || h <= 100.0)) {
                w = 0.0;
                h = 0.0;
            }
        }

        if (w > EPS && h > EPS && type != BlockType::SOFT) {
            spec.hasFixedSize = true;
            spec.fixedW = w;
            spec.fixedH = h;
        }
        else if (w > EPS && h > EPS && fabs(w * h - spec.area) / max(1.0, spec.area) < 0.50) {
            spec.hasFixedSize = true;
            spec.fixedW = w;
            spec.fixedH = h;
        }

        double amin = 1.0;
        double amax = 1.0;
        if (parseAspectRange(row, amin, amax)) {
            spec.aspectMin = amin;
            spec.aspectMax = amax;
        }
        else if (type == BlockType::SOFT) {
            spec.aspectMin = 0.5;
            spec.aspectMax = 2.0;
        }
        else {
            spec.aspectMin = 1.0;
            spec.aspectMax = 1.0;
        }

        spec.locations = parseLocationsFromRow(row);

        vector<double> rates = parsePercentRates(row);
        if (rates.size() >= 4) {
            for (int i = 0; i < 4; ++i) spec.ftRate[i] = rates[i];
        }
        else {
            vector<double> maybeRates;
            for (double v : nums) {
                if (v == 20.0 || v == 40.0 || v == 80.0 || v == 100.0) {
                    maybeRates.push_back(v / 100.0);
                }
            }
            if (maybeRates.size() >= 4) {
                for (int i = 0; i < 4; ++i) spec.ftRate[i] = maybeRates[i];
            }
        }

        design.blockSpecs.push_back(spec);
    }

    sort(design.blockSpecs.begin(), design.blockSpecs.end(),
        [](const BlockSpec& a, const BlockSpec& b) { return a.name < b.name; });
}

void Parser::parseOutline(const vector<vector<string>>& rows, Design& design) {
    for (size_t i = 0; i < rows.size(); ++i) {
        const auto& row = rows[i];
        for (size_t j = 0; j < row.size(); ++j) {
            string u = upperStr(trim(row[j]));
            if (u == "MAX") {
                vector<double> nums;
                for (size_t k = j + 1; k < row.size(); ++k) {
                    double v = 0.0;
                    if (tryParseDouble(row[k], v)) nums.push_back(v);
                }
                if (nums.size() >= 2) {
                    design.maxOutlineW = nums[0];
                    design.maxOutlineH = nums[1];
                    return;
                }
            }
        }
    }

    for (size_t i = 0; i < rows.size(); ++i) {
        if (!containsToken(rows[i], "OUTLINE")) continue;
        for (size_t r = i; r < min(rows.size(), i + 8); ++r) {
            vector<double> nums = numericValuesInRow(rows[r]);
            if (nums.size() >= 2) {
                design.maxOutlineW = nums[0];
                design.maxOutlineH = nums[1];
                return;
            }
        }
    }
}

void Parser::parseConnectionMatrix(const vector<vector<string>>& rows, Design& design) {
    int n = static_cast<int>(design.blockSpecs.size());
    if (n == 0) return;

    unordered_map<string, int> nameToIdx;
    for (int i = 0; i < n; ++i) nameToIdx[design.blockSpecs[i].name] = i;

    int matrixStart = -1;
    for (int i = 0; i < static_cast<int>(rows.size()); ++i) {
        if (containsToken(rows[i], "CONN MATRIX") ||
            containsToken(rows[i], "CONNECTION") ||
            containsToken(rows[i], "MATRIX")) {
            matrixStart = i;
            break;
        }
    }
    if (matrixStart < 0) return;

    int headerRow = -1;
    vector<int> headerCols;
    vector<int> headerBlockIdx;

    for (int i = matrixStart; i < static_cast<int>(rows.size()); ++i) {
        headerCols.clear();
        headerBlockIdx.clear();

        for (int c = 0; c < static_cast<int>(rows[i].size()); ++c) {
            string cell = trim(rows[i][c]);
            if (nameToIdx.count(cell)) {
                headerCols.push_back(c);
                headerBlockIdx.push_back(nameToIdx[cell]);
            }
        }

        if (static_cast<int>(headerCols.size()) >= max(2, n / 2)) {
            headerRow = i;
            break;
        }
    }
    if (headerRow < 0) return;

    design.connMatrix.assign(n, vector<int>(n, 0));

    for (int r = headerRow + 1; r < static_cast<int>(rows.size()); ++r) {
        int first = firstNonEmptyIndex(rows[r]);
        if (first < 0) continue;

        string rowName = trim(rows[r][first]);
        if (!nameToIdx.count(rowName)) {
            if (r > headerRow + 1) break;
            continue;
        }

        int srcIdx = nameToIdx[rowName];
        for (int k = 0; k < static_cast<int>(headerCols.size()); ++k) {
            int col = headerCols[k];
            int dstIdx = headerBlockIdx[k];
            if (col >= static_cast<int>(rows[r].size())) continue;

            double v = 0.0;
            if (tryParseDouble(rows[r][col], v)) {
                design.connMatrix[srcIdx][dstIdx] = static_cast<int>(llround(v));
            }
        }
    }
}

void Parser::buildConnections(Design& design) {
    design.connections.clear();
    int n = static_cast<int>(design.connMatrix.size());
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < static_cast<int>(design.connMatrix[i].size()); ++j) {
            int nets = design.connMatrix[i][j];
            if (nets > 0) design.connections.push_back({ i, j, nets });
        }
    }
}
