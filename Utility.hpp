#pragma once
#include <string>
#include <vector>

struct BlockInst;
struct BlockSpec;

inline constexpr double EPS = 1e-6;
inline constexpr double CHANNEL_DENSITY = 25.0; // 25 nets / um

std::string trim(const std::string& s);
std::string upperStr(std::string s);
bool startsWithBlockName(const std::string& s);
bool isEmptyCell(const std::string& s);
bool tryParseDouble(std::string s, double& out);
double parseDoubleOrZero(const std::string& s);
std::vector<std::string> splitCSVLine(const std::string& line);
std::vector<std::string> splitByCommaOrSlash(std::string s);
bool containsToken(const std::vector<std::string>& row, const std::string& tokenUpper);
int firstNonEmptyIndex(const std::vector<std::string>& row);
std::string firstNonEmptyCell(const std::vector<std::string>& row);
int edgeOpposite(int e);
double feedthroughRateForNets(const BlockSpec& spec, double ftNets);
double requiredSoftAreaWithFeedthrough(const BlockInst& b, double ftNets);
double requiredSoftAreaWithFeedthrough(const BlockInst& b);
std::string passFail(bool bad);
std::string yesNo(bool b);
