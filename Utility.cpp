#include "Utility.hpp"
#include <algorithm>
#include <cctype>
#include <cstdlib>

using namespace std;

string trim(const string& s) {
    size_t b = 0;
    while (b < s.size() && isspace(static_cast<unsigned char>(s[b]))) ++b;
    size_t e = s.size();
    while (e > b && isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

string upperStr(string s) {
    for (char& c : s) c = static_cast<char>(toupper(static_cast<unsigned char>(c)));
    return s;
}

bool startsWithBlockName(const string& s) {
    string t = upperStr(trim(s));
    return t.size() >= 4 && t[0] == 'B' && t[1] == 'L' && t[2] == 'K';
}

bool isEmptyCell(const string& s) {
    return trim(s).empty();
}

bool tryParseDouble(string s, double& out) {
    s = trim(s);
    if (s.empty()) return false;

    string t;
    for (char c : s) {
        if (c == '%') continue;
        if (c == ',') continue;
        t.push_back(c);
    }
    if (t.empty()) return false;

    char* endptr = nullptr;
    out = strtod(t.c_str(), &endptr);
    return endptr != t.c_str() && *endptr == '\0';
}

double parseDoubleOrZero(const string& s) {
    double v = 0.0;
    return tryParseDouble(s, v) ? v : 0.0;
}

vector<string> splitCSVLine(const string& line) {
    vector<string> cells;
    string cur;
    bool inQuote = false;

    for (size_t i = 0; i < line.size(); ++i) {
        char c = line[i];
        if (c == '"') {
            if (inQuote && i + 1 < line.size() && line[i + 1] == '"') {
                cur.push_back('"');
                ++i;
            }
            else {
                inQuote = !inQuote;
            }
        }
        else if (c == ',' && !inQuote) {
            cells.push_back(trim(cur));
            cur.clear();
        }
        else {
            cur.push_back(c);
        }
    }
    cells.push_back(trim(cur));
    return cells;
}

vector<string> splitByCommaOrSlash(string s) {
    vector<string> out;
    string cur;
    for (char c : s) {
        if (c == ',' || c == '/' || c == ';') {
            if (!trim(cur).empty()) out.push_back(trim(cur));
            cur.clear();
        }
        else {
            cur.push_back(c);
        }
    }
    if (!trim(cur).empty()) out.push_back(trim(cur));
    return out;
}

bool containsToken(const vector<string>& row, const string& tokenUpper) {
    for (const string& c : row) {
        if (upperStr(trim(c)).find(tokenUpper) != string::npos) return true;
    }
    return false;
}

int firstNonEmptyIndex(const vector<string>& row) {
    for (int i = 0; i < static_cast<int>(row.size()); ++i) {
        if (!isEmptyCell(row[i])) return i;
    }
    return -1;
}

string firstNonEmptyCell(const vector<string>& row) {
    int idx = firstNonEmptyIndex(row);
    return idx >= 0 ? trim(row[idx]) : "";
}

int edgeOpposite(int e) {
    if (e == 1) return 3;
    if (e == 3) return 1;
    if (e == 2) return 4;
    if (e == 4) return 2;
    return 0;
}

string passFail(bool bad) {
    return bad ? "FAIL" : "PASS";
}

string yesNo(bool b) {
    return b ? "YES" : "NO";
}
