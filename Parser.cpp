#include "Parser.hpp"
#include "Utility.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <unordered_map>

using namespace std;

namespace {

string stripUtf8Bom(string s) {
    if (s.size() >= 3 &&
        static_cast<unsigned char>(s[0]) == 0xEF &&
        static_cast<unsigned char>(s[1]) == 0xBB &&
        static_cast<unsigned char>(s[2]) == 0xBF) {
        s.erase(0, 3);
    }
    return s;
}

string normalizedHeaderCell(const string& cell) {
    string u = upperStr(trim(cell));
    string out;
    for (unsigned char c : u) {
        if (isalnum(c)) out.push_back(static_cast<char>(c));
    }
    return out;
}

bool cellAt(const vector<string>& row, int col, string& out) {
    if (col < 0 || col >= static_cast<int>(row.size())) return false;
    out = trim(row[col]);
    return !out.empty();
}

bool tryParseCellDouble(const vector<string>& row, int col, double& out) {
    string cell;
    return cellAt(row, col, cell) && tryParseDouble(cell, out);
}

bool isAlphaLabelCell(const string& cell) {
    string t = trim(cell);
    string key = normalizedHeaderCell(t);
    if (key == "ALPHA") return true;

    // UTF-8 Greek alpha, lower/upper.  Keep this as byte escapes so the
    // parser does not depend on the source file encoding.
    return t.find("\xCE\xB1") != string::npos || t.find("\xCE\x91") != string::npos;
}

} // namespace

bool Parser::read(const string& inputPath, Design& design) {
    vector<vector<string>> rows;
    if (!readCSV(inputPath, rows)) {
        cerr << "[Parser] Cannot open input: " << inputPath << "\n";
        return false;
    }

    parseBlockRows(rows, design);
    parseOutline(rows, design);
    parseAlpha(rows, design);
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
    ifstream fin(path, ios::binary);
    if (!fin) return false;

    string record;
    bool inQuote = false;
    char ch = '\0';
    while (fin.get(ch)) {
        if (ch == '"') {
            record.push_back(ch);
            if (inQuote && fin.peek() == '"') {
                char escaped = '\0';
                fin.get(escaped);
                record.push_back(escaped);
            }
            else {
                inQuote = !inQuote;
            }
        }
        else if ((ch == '\n' || ch == '\r') && !inQuote) {
            if (ch == '\r' && fin.peek() == '\n') {
                char lf = '\0';
                fin.get(lf);
            }
            rows.push_back(splitCSVLine(stripUtf8Bom(record)));
            record.clear();
        }
        else {
            record.push_back(ch);
        }
    }

    if (!record.empty()) {
        rows.push_back(splitCSVLine(stripUtf8Bom(record)));
    }

    return true;
}

Parser::BlockColumns Parser::detectBlockColumns(const vector<vector<string>>& rows) {
    for (const auto& row : rows) {
        BlockColumns cols;
        for (int i = 0; i < static_cast<int>(row.size()); ++i) {
            string key = normalizedHeaderCell(row[i]);
            if (key == "BLOCK") cols.name = i;
            else if (key == "AREA") cols.area = i;
            else if (key == "WIDTH") cols.width = i;
            else if (key == "HEIGHT") cols.height = i;
            else if (key == "ASPECTRATIORANGE") cols.aspect = i;
            else if (key == "EDGEHARDMACROSOFT" || key == "EDGEHARDMACROSOFTTYPE") cols.type = i;
            else if (key == "LOCATION") cols.location = i;
            else if (key == "PORTEDGE") cols.portEdge = i;
            else if (key == "FTCONVERSION") cols.ftConversion = i;
        }

        if (cols.valid()) return cols;
    }

    return {};
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

bool Parser::parseAspectRangeCell(const string& cell, double& amin, double& amax) {
    string t = trim(cell);
    if (t.empty()) return false;

    vector<string> parts;
    string cur;
    for (char c : t) {
        if (c == ',' || c == '~' || c == '-') {
            if (!trim(cur).empty()) parts.push_back(trim(cur));
            cur.clear();
        }
        else {
            cur.push_back(c);
        }
    }
    if (!trim(cur).empty()) parts.push_back(trim(cur));

    if (parts.size() >= 2) {
        double a = 0.0;
        double b = 0.0;
        if (tryParseDouble(parts[0], a) && tryParseDouble(parts[1], b)) {
            amin = min(a, b);
            amax = max(a, b);
            return true;
        }
    }

    double v = 0.0;
    if (tryParseDouble(t, v) && v > EPS) {
        amin = v;
        amax = v;
        return true;
    }

    return false;
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

vector<int> Parser::parsePortEdgesFromCell(const string& cell) {
    vector<int> edges;
    for (string part : splitByCommaOrSlash(cell)) {
        double v = 0.0;
        if (!tryParseDouble(part, v)) continue;
        int edge = static_cast<int>(llround(v));
        if (edge >= 1 && edge <= 4 && find(edges.begin(), edges.end(), edge) == edges.end()) {
            edges.push_back(edge);
        }
    }
    sort(edges.begin(), edges.end());
    return edges;
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

vector<double> Parser::parsePercentRatesFromColumns(const vector<string>& row, int startCol) {
    vector<double> rates;
    if (startCol < 0) return rates;

    for (int i = startCol; i < static_cast<int>(row.size()) && rates.size() < 4; ++i) {
        string t = trim(row[i]);
        if (t.empty()) continue;

        double v = 0.0;
        if (!tryParseDouble(t, v)) continue;
        if (t.find('%') != string::npos || v > 1.0) v /= 100.0;
        rates.push_back(v);
    }

    return rates;
}

void Parser::parseBlockRows(const vector<vector<string>>& rows, Design& design) {
    design.blockSpecs.clear();
    const BlockColumns cols = detectBlockColumns(rows);

    for (const auto& row : rows) {
        int first = firstNonEmptyIndex(row);
        if (first < 0) continue;

        string name = trim(row[first]);
        if (cols.valid() && cols.name < static_cast<int>(row.size())) {
            name = trim(row[cols.name]);
        }
        if (!startsWithBlockName(name)) continue;

        BlockType type = BlockType::UNKNOWN;
        if (cols.valid() && cols.type < static_cast<int>(row.size())) {
            type = parseBlockTypeFromRow(vector<string>{ row[cols.type] });
        }
        if (type == BlockType::UNKNOWN) type = parseBlockTypeFromRow(row);
        if (type == BlockType::UNKNOWN) continue;

        vector<double> nums = numericValuesInRow(row);

        BlockSpec spec;
        spec.name = name;
        spec.type = type;
        if (cols.valid()) {
            if (!tryParseCellDouble(row, cols.area, spec.area)) continue;
        }
        else {
            if (nums.empty()) continue;
            spec.area = nums[0];
        }

        double w = 0.0;
        double h = 0.0;
        if (cols.valid()) {
            tryParseCellDouble(row, cols.width, w);
            tryParseCellDouble(row, cols.height, h);
        }
        else if (nums.size() >= 3) {
            w = nums[1];
            h = nums[2];
        }

        if (w > EPS && h > EPS) {
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
        bool aspectOk = false;
        if (cols.valid() && cols.aspect >= 0 && cols.aspect < static_cast<int>(row.size())) {
            aspectOk = parseAspectRangeCell(row[cols.aspect], amin, amax);
        }
        if (!aspectOk && !cols.valid()) {
            aspectOk = parseAspectRange(row, amin, amax);
        }

        if (aspectOk) {
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

        if (cols.valid() && cols.location >= 0 && cols.location < static_cast<int>(row.size())) {
            spec.locations = parseLocationsFromRow(vector<string>{ row[cols.location] });
        }
        else {
            spec.locations = parseLocationsFromRow(row);
        }

        if (cols.valid() && cols.portEdge >= 0 && cols.portEdge < static_cast<int>(row.size())) {
            spec.portEdges = parsePortEdgesFromCell(row[cols.portEdge]);
        }

        vector<double> rates;
        if (cols.valid() && cols.ftConversion >= 0) {
            rates = parsePercentRatesFromColumns(row, cols.ftConversion);
        }
        if (rates.empty()) rates = parsePercentRates(row);
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

void Parser::parseAlpha(const vector<vector<string>>& rows, Design& design) {
    design.alpha = 1.0;

    for (const auto& row : rows) {
        for (int c = 0; c < static_cast<int>(row.size()); ++c) {
            if (!isAlphaLabelCell(row[c])) continue;

            for (int k = c + 1; k < static_cast<int>(row.size()); ++k) {
                double v = 0.0;
                if (tryParseDouble(row[k], v)) {
                    design.alpha = v;
                    return;
                }
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
    int pairCount = 0;
    int symmetricPairCount = 0;
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < static_cast<int>(design.connMatrix[i].size()); ++j) {
            int fwd = max(0, design.connMatrix[i][j]);
            int rev = 0;
            if (j < static_cast<int>(design.connMatrix.size()) &&
                i < static_cast<int>(design.connMatrix[j].size())) {
                rev = max(0, design.connMatrix[j][i]);
            }
            if (fwd > 0 || rev > 0) {
                ++pairCount;
                if (fwd > 0 && rev > 0) ++symmetricPairCount;
            }
        }
    }

    const bool mostlySymmetric =
        pairCount > 0 &&
        symmetricPairCount * 100 >= pairCount * 65;

    if (mostlySymmetric) {
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < static_cast<int>(design.connMatrix[i].size()); ++j) {
                int nets = max(0, design.connMatrix[i][j]);
                if (j < static_cast<int>(design.connMatrix.size()) &&
                    i < static_cast<int>(design.connMatrix[j].size())) {
                    nets += max(0, design.connMatrix[j][i]);
                }
                if (nets > 0) design.connections.push_back({ i, j, nets });
            }
        }
        return;
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < static_cast<int>(design.connMatrix[i].size()); ++j) {
            int nets = design.connMatrix[i][j];
            if (nets > 0) design.connections.push_back({ i, j, nets });
        }
    }
}
