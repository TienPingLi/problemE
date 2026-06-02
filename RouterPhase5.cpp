#include "RouterPhase5.hpp"

#include "Evaluator.hpp"
#include "OutputWriter.hpp"
#include "RouterPhase1.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <unordered_map>
#include <vector>

using namespace std;
namespace fs = std::filesystem;

namespace {

constexpr double EPS = 1.0e-9;

struct CsvTable {
    vector<string> header;
    unordered_map<string, int> col;
    vector<vector<string>> rows;
};

struct SelectedRoute {
    int connectionIndex = -1;
    int candidateIndex = -1;
    string family;
    int allocatedNets = 0;
    int assignedIteration = 0;
    double score = 0.0;
    string pathSteps;
    RoutePath path;
};

struct FtHotspot {
    int blockIndex = -1;
    string blockName;
    double usedFtNets = 0.0;
    double conversionRate = 0.0;
    double sideDelta = 0.0;
    double currentArea = 0.0;
    double requiredArea = 0.0;
    double overflowArea = 0.0;
};

struct ChannelHotspot {
    int channelIndex = -1;
    string channelName;
    string component;
    double used = 0.0;
    double hardCap = 0.0;
    double softCap = 0.0;
    double overflowHard = 0.0;
    double overflowSoft = 0.0;
    double score = 0.0;
};

struct RepairBox {
    int boxId = -1;
    string seedType;
    string seedName;
    double score = 0.0;
    Rect rect;
    int affectedPieces = 0;
    int affectedNets = 0;
    int hotChannelCount = 0;
    int hotFtCount = 0;
    string primaryAction;
};

struct RepairCandidate {
    int connectionIndex = -1;
    int candidateIndex = -1;
    string family;
    RoutePath path;
    double baseScore = 0.0;
    double wireLength = 0.0;
    int hotFtTouches = 0;
    int hotChannelTouches = 0;
    int softFtBlocks = 0;
    double localScore = 0.0;
};

struct RepairAttempt {
    int connectionIndex = -1;
    int candidateIndex = -1;
    string family;
    bool accepted = false;
    string reason;
    double beforeObjective = 0.0;
    double afterObjective = 0.0;
    double beforeChannelOverflow = 0.0;
    double afterChannelOverflow = 0.0;
    double beforeFtOverflow = 0.0;
    double afterFtOverflow = 0.0;
    double beforeWireLength = 0.0;
    double afterWireLength = 0.0;
    string pathSteps;
};

string yesNo(bool v) {
    return v ? "YES" : "NO";
}

string csvEscape(const string& s) {
    const bool needQuote = s.find_first_of(",\"\n\r") != string::npos;
    if (!needQuote) return s;
    string out = "\"";
    for (char c : s) {
        if (c == '"') out += "\"\"";
        else out += c;
    }
    out += "\"";
    return out;
}

vector<string> splitCsvLine(const string& line) {
    vector<string> fields;
    string cur;
    bool quoted = false;
    for (size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (quoted) {
            if (c == '"') {
                if (i + 1 < line.size() && line[i + 1] == '"') {
                    cur += '"';
                    ++i;
                } else {
                    quoted = false;
                }
            } else {
                cur += c;
            }
        } else {
            if (c == '"') {
                quoted = true;
            } else if (c == ',') {
                fields.push_back(cur);
                cur.clear();
            } else {
                cur += c;
            }
        }
    }
    fields.push_back(cur);
    return fields;
}

CsvTable readCsvTable(const string& path) {
    CsvTable t;
    ifstream fin(path);
    string line;
    if (!getline(fin, line)) return t;
    t.header = splitCsvLine(line);
    for (int i = 0; i < static_cast<int>(t.header.size()); ++i) {
        t.col[t.header[i]] = i;
    }
    while (getline(fin, line)) {
        if (line.empty()) continue;
        t.rows.push_back(splitCsvLine(line));
    }
    return t;
}

string getField(const CsvTable& t, const vector<string>& row, const string& name) {
    auto it = t.col.find(name);
    if (it == t.col.end()) return "";
    const int idx = it->second;
    if (idx < 0 || idx >= static_cast<int>(row.size())) return "";
    return row[idx];
}

double toDouble(const string& s) {
    try {
        if (s.empty()) return 0.0;
        return stod(s);
    } catch (...) {
        return 0.0;
    }
}

int toInt(const string& s) {
    try {
        if (s.empty()) return 0;
        return stoi(s);
    } catch (...) {
        return 0;
    }
}

string joinPathSteps(const vector<RouteStep>& steps) {
    ostringstream oss;
    for (size_t i = 0; i < steps.size(); ++i) {
        if (i) oss << '|';
        oss << steps[i].rectName << ':' << steps[i].edge;
    }
    return oss.str();
}

bool containsToken(const string& pathSteps, const string& name) {
    if (name.empty()) return false;
    size_t pos = pathSteps.find(name);
    while (pos != string::npos) {
        const bool leftOk = (pos == 0) || pathSteps[pos - 1] == '|';
        const size_t end = pos + name.size();
        const bool rightOk = (end >= pathSteps.size()) || pathSteps[end] == ':';
        if (leftOk && rightOk) return true;
        pos = pathSteps.find(name, pos + 1);
    }
    return false;
}

string outputStemName(const string& outputCfgPath) {
    fs::path out(outputCfgPath);
    string stem = out.stem().string();
    if (stem.empty()) stem = "router";
    return stem;
}

fs::path phase4StemPath(const string& outputCfgPath) {
    fs::path out(outputCfgPath);
    return out.parent_path() / out.stem();
}

fs::path reportStemPath(const string& outputCfgPath) {
    fs::path dir("Router_Statistics");
    fs::create_directories(dir);
    return dir / outputStemName(outputCfgPath);
}

Rect clampBox(const Rect& r, const Design& design) {
    Rect out = r;
    const double maxW = design.outlineW > EPS ? design.outlineW : design.maxOutlineW;
    const double maxH = design.outlineH > EPS ? design.outlineH : design.maxOutlineH;
    out.x = max(0.0, out.x);
    out.y = max(0.0, out.y);
    if (maxW > EPS) {
        const double right = min(maxW, out.x + max(0.0, out.w));
        out.w = max(0.0, right - out.x);
    }
    if (maxH > EPS) {
        const double top = min(maxH, out.y + max(0.0, out.h));
        out.h = max(0.0, top - out.y);
    }
    return out;
}

Rect expandedRect(const Rect& base, double expansion, const Design& design) {
    Rect r;
    r.x = base.x - expansion;
    r.y = base.y - expansion;
    r.w = base.w + 2.0 * expansion;
    r.h = base.h + 2.0 * expansion;
    return clampBox(r, design);
}

bool rectIntersectsOrTouches(const Rect& a, const Rect& b) {
    return rectRight(a) + EPS >= b.x && rectRight(b) + EPS >= a.x
        && rectTop(a) + EPS >= b.y && rectTop(b) + EPS >= a.y;
}

vector<string> splitPathTokens(const string& pathSteps) {
    vector<string> out;
    string cur;
    for (char c : pathSteps) {
        if (c == '|') {
            if (!cur.empty()) out.push_back(cur);
            cur.clear();
        } else {
            cur += c;
        }
    }
    if (!cur.empty()) out.push_back(cur);
    return out;
}

RoutePath parseRoutePathFromSteps(const Design& design, int connectionIndex, int nets, const string& pathSteps) {
    RoutePath p;
    p.netCount = nets;
    if (connectionIndex >= 0 && connectionIndex < static_cast<int>(design.connections.size())) {
        const Connection& conn = design.connections[connectionIndex];
        if (conn.src >= 0 && conn.src < static_cast<int>(design.blocks.size())) p.srcBlock = design.blocks[conn.src].spec.name;
        if (conn.dst >= 0 && conn.dst < static_cast<int>(design.blocks.size())) p.dstBlock = design.blocks[conn.dst].spec.name;
    }
    for (const string& token : splitPathTokens(pathSteps)) {
        const size_t colon = token.rfind(':');
        if (colon == string::npos) continue;
        RouteStep st;
        st.rectName = token.substr(0, colon);
        st.edge = toInt(token.substr(colon + 1));
        p.steps.push_back(st);
    }
    if (p.steps.size() < 2) p.open = true;
    return p;
}

RoutePath makeOpenPath(const Design& design, int connectionIndex, int nets) {
    RoutePath p;
    p.netCount = nets;
    p.open = true;
    if (connectionIndex >= 0 && connectionIndex < static_cast<int>(design.connections.size())) {
        const Connection& conn = design.connections[connectionIndex];
        if (conn.src >= 0 && conn.src < static_cast<int>(design.blocks.size())) p.srcBlock = design.blocks[conn.src].spec.name;
        if (conn.dst >= 0 && conn.dst < static_cast<int>(design.blocks.size())) p.dstBlock = design.blocks[conn.dst].spec.name;
    }
    return p;
}

vector<SelectedRoute> parseSelectedRoutes(const Design& design, const string& path) {
    CsvTable t = readCsvTable(path);
    vector<SelectedRoute> out;
    for (const auto& row : t.rows) {
        SelectedRoute r;
        r.connectionIndex = toInt(getField(t, row, "connection_index"));
        r.candidateIndex = toInt(getField(t, row, "candidate_index"));
        r.family = getField(t, row, "family");
        r.allocatedNets = toInt(getField(t, row, "allocated_nets"));
        r.assignedIteration = toInt(getField(t, row, "assigned_iteration"));
        r.score = toDouble(getField(t, row, "score"));
        r.pathSteps = getField(t, row, "path_steps");
        r.path = parseRoutePathFromSteps(design, r.connectionIndex, r.allocatedNets, r.pathSteps);
        out.push_back(r);
    }
    return out;
}

vector<FtHotspot> parseFtHotspots(const string& path) {
    CsvTable t = readCsvTable(path);
    vector<FtHotspot> out;
    for (const auto& row : t.rows) {
        FtHotspot h;
        h.blockIndex = toInt(getField(t, row, "block_index"));
        h.blockName = getField(t, row, "block_name");
        h.usedFtNets = toDouble(getField(t, row, "used_ft_nets"));
        h.conversionRate = toDouble(getField(t, row, "conversion_rate"));
        h.sideDelta = toDouble(getField(t, row, "side_delta"));
        h.currentArea = toDouble(getField(t, row, "current_area"));
        h.requiredArea = toDouble(getField(t, row, "required_area"));
        h.overflowArea = toDouble(getField(t, row, "overflow_area"));
        if (h.overflowArea > EPS) out.push_back(h);
    }
    sort(out.begin(), out.end(), [](const FtHotspot& a, const FtHotspot& b) {
        return a.overflowArea > b.overflowArea;
    });
    return out;
}

vector<ChannelHotspot> parseChannelHotspots(const string& path) {
    CsvTable t = readCsvTable(path);
    vector<ChannelHotspot> out;
    for (const auto& row : t.rows) {
        const string type = getField(t, row, "resource_type");
        if (type != "CHANNEL_LR" && type != "CHANNEL_TB") continue;
        ChannelHotspot h;
        h.channelIndex = toInt(getField(t, row, "index"));
        h.channelName = getField(t, row, "name");
        h.component = (type == "CHANNEL_LR") ? "LR" : "TB";
        h.used = toDouble(getField(t, row, "used"));
        h.hardCap = toDouble(getField(t, row, "hard_cap"));
        h.softCap = toDouble(getField(t, row, "soft_cap"));
        h.overflowHard = toDouble(getField(t, row, "overflow_hard"));
        h.overflowSoft = toDouble(getField(t, row, "overflow_soft"));
        h.score = 1000.0 * h.overflowHard + h.overflowSoft;
        if (h.score > EPS) out.push_back(h);
    }
    sort(out.begin(), out.end(), [](const ChannelHotspot& a, const ChannelHotspot& b) {
        return a.score > b.score;
    });
    return out;
}

pair<int, int> countAffected(const vector<SelectedRoute>& routes, const string& token) {
    int pieces = 0;
    int nets = 0;
    for (const auto& r : routes) {
        if (containsToken(r.pathSteps, token)) {
            ++pieces;
            nets += r.allocatedNets;
        }
    }
    return {pieces, nets};
}

int countHotChannelsInBox(const Design& design, const Rect& box, const vector<ChannelHotspot>& hotChannels) {
    set<int> seen;
    for (const auto& h : hotChannels) {
        if (h.channelIndex < 0 || h.channelIndex >= static_cast<int>(design.channels.size())) continue;
        if (rectIntersectsOrTouches(box, design.channels[h.channelIndex].rect)) seen.insert(h.channelIndex);
    }
    return static_cast<int>(seen.size());
}

int countHotFtInBox(const Design& design, const Rect& box, const vector<FtHotspot>& hotFts) {
    int count = 0;
    for (const auto& h : hotFts) {
        if (h.blockIndex < 0 || h.blockIndex >= static_cast<int>(design.blocks.size())) continue;
        if (rectIntersectsOrTouches(box, design.blocks[h.blockIndex].rect)) ++count;
    }
    return count;
}

double localExpansion(double severityScore, const Design& design, const RouterPhase5::Options& opt) {
    const double outlineScale = max(design.outlineW, design.outlineH);
    const double maxExpansion = max(opt.minBoxExpansion, outlineScale * opt.maxBoxExpansionRatio);
    const double raw = opt.minBoxExpansion + 0.08 * sqrt(max(0.0, severityScore));
    return min(maxExpansion, max(opt.minBoxExpansion, raw));
}

set<string> hotFtNames(const vector<FtHotspot>& hotFts) {
    set<string> out;
    for (const auto& h : hotFts) out.insert(h.blockName);
    return out;
}

set<string> hotChannelNames(const vector<ChannelHotspot>& hotChannels) {
    set<string> out;
    for (const auto& h : hotChannels) out.insert(h.channelName);
    return out;
}

int countTouches(const string& pathSteps, const set<string>& names) {
    int count = 0;
    for (const auto& n : names) {
        if (containsToken(pathSteps, n)) ++count;
    }
    return count;
}

vector<vector<SelectedRoute>> groupRoutes(const Design& design, const vector<SelectedRoute>& routes) {
    vector<vector<SelectedRoute>> grouped(design.connections.size());
    for (const auto& r : routes) {
        if (r.connectionIndex >= 0 && r.connectionIndex < static_cast<int>(grouped.size())) {
            grouped[r.connectionIndex].push_back(r);
        }
    }
    return grouped;
}

Design buildRoutedDesignFromGroups(const Design& design, const vector<vector<SelectedRoute>>& grouped) {
    Design out = design;
    out.routes.clear();
    for (int ci = 0; ci < static_cast<int>(design.connections.size()); ++ci) {
        int allocated = 0;
        for (const auto& r : grouped[ci]) {
            if (r.allocatedNets <= 0) continue;
            RoutePath p = r.path;
            p.netCount = r.allocatedNets;
            out.routes.push_back(p);
            allocated += r.allocatedNets;
        }
        const int missing = max(0, design.connections[ci].netCount - allocated);
        if (missing > 0) out.routes.push_back(makeOpenPath(design, ci, missing));
    }
    return out;
}

double repairObjective(const EvalReport& eval, const RouterPhase5::Options& opt, int routeChanges) {
    const double failPart = eval.hasFail() ? opt.invalidPenaltyWeight : 0.0;
    return failPart
        + opt.openPenaltyWeight * static_cast<double>(eval.openPathCount)
        + opt.invalidPenaltyWeight * static_cast<double>(eval.invalidPathCount)
        + opt.channelPenaltyWeight * eval.totalChannelOverflow
        + opt.ftPenaltyWeight * eval.totalFeedthroughOverflow
        + opt.wireLengthWeight * eval.totalWireLength
        + opt.routeChangePenalty * static_cast<double>(routeChanges);
}

bool basicLegalityNotWorse(const EvalReport& cand, const EvalReport& base) {
    if (cand.formatFailed && !base.formatFailed) return false;
    if (cand.pathInvalid && !base.pathInvalid) return false;
    if (cand.blockOverlap && !base.blockOverlap) return false;
    if (cand.routingOpen && !base.routingOpen) return false;
    if (cand.outlineViolation && !base.outlineViolation) return false;
    if (cand.openPathCount > base.openPathCount) return false;
    if (cand.invalidPathCount > base.invalidPathCount) return false;
    return true;
}

bool acceptableRepair(
    const EvalReport& cand,
    double candObj,
    const EvalReport& best,
    double bestObj,
    const EvalReport& phase4Anchor,
    const RouterPhase5::Options& opt
) {
    if (!basicLegalityNotWorse(cand, best)) return false;
    if (cand.totalChannelOverflow > phase4Anchor.totalChannelOverflow + opt.channelRegressionGuardAbs) return false;
    if (cand.totalFeedthroughOverflow > phase4Anchor.totalFeedthroughOverflow + opt.ftRegressionGuardAbs) return false;
    return candObj + 1.0e-6 < bestObj;
}

vector<int> selectRepairConnections(
    const Design& design,
    const vector<vector<SelectedRoute>>& grouped,
    const set<string>& hotFts,
    const set<string>& hotChannels,
    int limit
) {
    vector<pair<double, int>> scored;
    for (int ci = 0; ci < static_cast<int>(grouped.size()); ++ci) {
        double score = 0.0;
        int pieces = 0;
        for (const auto& r : grouped[ci]) {
            const int ft = countTouches(r.pathSteps, hotFts);
            const int ch = countTouches(r.pathSteps, hotChannels);
            if (ft > 0 || ch > 0) {
                score += static_cast<double>(r.allocatedNets) * (10000.0 * ft + 100.0 * ch);
                ++pieces;
            }
        }
        if (score > EPS) {
            score += static_cast<double>(design.connections[ci].netCount) + 10.0 * pieces;
            scored.push_back({score, ci});
        }
    }
    sort(scored.begin(), scored.end(), [](const auto& a, const auto& b) {
        if (fabs(a.first - b.first) > EPS) return a.first > b.first;
        return a.second < b.second;
    });
    if (static_cast<int>(scored.size()) > limit) scored.resize(limit);
    vector<int> out;
    for (const auto& kv : scored) out.push_back(kv.second);
    return out;
}

vector<RepairCandidate> buildRepairCandidates(
    const Design& design,
    int connectionIndex,
    const vector<Phase1RouteCandidate>& candidates,
    const set<string>& hotFts,
    const set<string>& hotChannels,
    int maxTrials
) {
    vector<RepairCandidate> out;
    set<string> seen;
    for (const auto& c : candidates) {
        if (c.path.open || c.path.steps.size() < 2) continue;
        const string sig = joinPathSteps(c.path.steps);
        if (seen.count(sig)) continue;
        seen.insert(sig);
        RepairCandidate rc;
        rc.connectionIndex = connectionIndex;
        rc.candidateIndex = c.candidateIndex;
        rc.family = c.family;
        rc.path = c.path;
        rc.path.netCount = design.connections[connectionIndex].netCount;
        rc.baseScore = c.score;
        rc.wireLength = c.wireLength;
        rc.hotFtTouches = countTouches(sig, hotFts);
        rc.hotChannelTouches = countTouches(sig, hotChannels);
        rc.softFtBlocks = c.softFtBlockCount;
        rc.localScore =
            100000000.0 * static_cast<double>(rc.hotFtTouches)
            + 1000000.0 * static_cast<double>(rc.hotChannelTouches)
            + 250000.0 * static_cast<double>(rc.softFtBlocks)
            + c.score
            + 0.02 * c.wireLength;
        out.push_back(rc);
    }
    sort(out.begin(), out.end(), [](const RepairCandidate& a, const RepairCandidate& b) {
        if (fabs(a.localScore - b.localScore) > EPS) return a.localScore < b.localScore;
        if (fabs(a.wireLength - b.wireLength) > EPS) return a.wireLength < b.wireLength;
        return a.family < b.family;
    });
    if (static_cast<int>(out.size()) > maxTrials) out.resize(maxTrials);
    return out;
}

SelectedRoute makeSelectedRouteFromCandidate(
    const Design& design,
    int connectionIndex,
    const RepairCandidate& cand,
    int assignedIteration
) {
    SelectedRoute r;
    r.connectionIndex = connectionIndex;
    r.candidateIndex = cand.candidateIndex;
    r.family = "P5_" + cand.family;
    r.allocatedNets = design.connections[connectionIndex].netCount;
    r.assignedIteration = assignedIteration;
    r.score = cand.localScore;
    r.path = cand.path;
    r.path.netCount = r.allocatedNets;
    r.pathSteps = joinPathSteps(r.path.steps);
    return r;
}

void writeSelectedRoutes(const string& path, const vector<vector<SelectedRoute>>& grouped) {
    ofstream f(path);
    f << fixed << setprecision(6);
    f << "connection_index,candidate_index,family,allocated_nets,assigned_iteration,score,path_steps\n";
    for (const auto& vec : grouped) {
        for (const auto& r : vec) {
            f << r.connectionIndex << ','
              << r.candidateIndex << ','
              << csvEscape(r.family) << ','
              << r.allocatedNets << ','
              << r.assignedIteration << ','
              << r.score << ','
              << csvEscape(r.pathSteps) << "\n";
        }
    }
}

void writeResourceAfter(const string& path, const EvalReport& eval) {
    ofstream f(path);
    f << fixed << setprecision(6);
    f << "resource_type,name,index,edge,hard_cap,soft_cap,used,overflow,aux0,aux1\n";
    for (int i = 0; i < static_cast<int>(eval.channelTruth.size()); ++i) {
        const auto& ch = eval.channelTruth[i];
        f << "CHANNEL_LR," << csvEscape(ch.name) << ',' << i << ",0,"
          << ch.lrCapacity << ',' << ch.lrCapacity << ',' << ch.lrUsed << ',' << ch.lrOverflow << ','
          << ch.lrUtilization << ',' << ch.lrTraversalCount << "\n";
        f << "CHANNEL_TB," << csvEscape(ch.name) << ',' << i << ",0,"
          << ch.tbCapacity << ',' << ch.tbCapacity << ',' << ch.tbUsed << ',' << ch.tbOverflow << ','
          << ch.tbUtilization << ',' << ch.tbTraversalCount << "\n";
    }
    for (int i = 0; i < static_cast<int>(eval.feedthroughTruth.size()); ++i) {
        const auto& ft = eval.feedthroughTruth[i];
        f << "SOFT_FT," << csvEscape(ft.blockName) << ',' << i << ",0,"
          << ft.currentArea << ',' << ft.baseArea << ',' << ft.usedNets << ',' << ft.overflowArea << ','
          << ft.requiredArea << ',' << ft.sideDelta << "\n";
    }
}

void writeRepairAttempts(const string& path, const vector<RepairAttempt>& attempts) {
    ofstream f(path);
    f << fixed << setprecision(6);
    f << "connection_index,candidate_index,family,accepted,reason,before_objective,after_objective,before_channel_overflow,after_channel_overflow,before_ft_overflow,after_ft_overflow,before_wirelength,after_wirelength,path_steps\n";
    for (const auto& a : attempts) {
        f << a.connectionIndex << ','
          << a.candidateIndex << ','
          << csvEscape(a.family) << ','
          << yesNo(a.accepted) << ','
          << csvEscape(a.reason) << ','
          << a.beforeObjective << ','
          << a.afterObjective << ','
          << a.beforeChannelOverflow << ','
          << a.afterChannelOverflow << ','
          << a.beforeFtOverflow << ','
          << a.afterFtOverflow << ','
          << a.beforeWireLength << ','
          << a.afterWireLength << ','
          << csvEscape(a.pathSteps) << "\n";
    }
}

} // namespace

RouterPhase5::RunResult RouterPhase5::run(
    const Design& design,
    const string& inputPath,
    const string& outputCfgPath,
    double alpha,
    const RouterPhase4::RunResult* phase4Result
) const {
    RunResult rr;
    if (!opt_.exportFiles) {
        rr.ok = true;
        return rr;
    }

    const fs::path p4ResultStem = phase4StemPath(outputCfgPath);
    const fs::path rptStem = reportStemPath(outputCfgPath);
    const string phase4SelectedRoutesNew = rptStem.string() + "_phase4_selected_routes.csv";
    const string phase4SelectedRoutesOld = p4ResultStem.string() + "_phase4_selected_routes.csv";
    const string phase4ResourceAfterNew = rptStem.string() + "_phase4_resource_after.csv";
    const string phase4ResourceAfterOld = p4ResultStem.string() + "_phase4_resource_after.csv";
    const string phase4FtHandoffNew = rptStem.string() + "_phase4_ft_handoff.csv";
    const string phase4FtHandoffOld = p4ResultStem.string() + "_phase4_ft_handoff.csv";

    const string phase4SelectedRoutesPath = fs::exists(phase4SelectedRoutesNew) ? phase4SelectedRoutesNew : phase4SelectedRoutesOld;
    const string phase4ResourceAfterPath = fs::exists(phase4ResourceAfterNew) ? phase4ResourceAfterNew : phase4ResourceAfterOld;
    string ftHandoffPath = fs::exists(phase4FtHandoffNew) ? phase4FtHandoffNew : phase4FtHandoffOld;
    if (phase4Result && !phase4Result->ftHandoffCsvPath.empty()) ftHandoffPath = phase4Result->ftHandoffCsvPath;

    rr.summaryPath = rptStem.string() + "_phase5_summary.txt";
    rr.boxesPath = rptStem.string() + "_phase5_boxes.csv";
    rr.actionsPath = rptStem.string() + "_phase5_actions.csv";
    rr.ftTargetsPath = rptStem.string() + "_phase5_ft_targets.csv";
    rr.repairAttemptsPath = rptStem.string() + "_phase5_repair_attempts.csv";
    rr.selectedRoutesPath = rptStem.string() + "_phase5_selected_routes.csv";
    rr.resourceAfterPath = rptStem.string() + "_phase5_resource_after.csv";
    rr.phase5CfgPath = p4ResultStem.string() + "_phase5.cfg";

    const bool hasRoutes = fs::exists(phase4SelectedRoutesPath);
    const bool hasResources = fs::exists(phase4ResourceAfterPath);
    const bool hasFt = fs::exists(ftHandoffPath);
    rr.missingPhase4Artifacts = !(hasRoutes && hasResources && hasFt);

    vector<SelectedRoute> routes;
    vector<FtHotspot> ftHotspots;
    vector<ChannelHotspot> channelHotspots;
    if (!rr.missingPhase4Artifacts) {
        routes = parseSelectedRoutes(design, phase4SelectedRoutesPath);
        ftHotspots = parseFtHotspots(ftHandoffPath);
        channelHotspots = parseChannelHotspots(phase4ResourceAfterPath);
    }

    rr.ftHotspotCount = static_cast<int>(ftHotspots.size());
    rr.channelHotspotCount = static_cast<int>(channelHotspots.size());

    vector<RepairBox> boxes;
    int ftBoxCount = 0;
    int boxId = 0;
    for (const auto& ft : ftHotspots) {
        if (static_cast<int>(boxes.size()) >= opt_.maxBoxes) break;
        if (ftBoxCount >= opt_.maxFtBoxes) break;
        if (ft.blockIndex < 0 || ft.blockIndex >= static_cast<int>(design.blocks.size())) continue;
        RepairBox b;
        b.boxId = boxId++;
        b.seedType = "FT_OVERFLOW";
        b.seedName = ft.blockName;
        b.score = ft.overflowArea;
        b.rect = expandedRect(design.blocks[ft.blockIndex].rect, localExpansion(ft.overflowArea, design, opt_), design);
        auto affected = countAffected(routes, ft.blockName);
        b.affectedPieces = affected.first;
        b.affectedNets = affected.second;
        b.hotChannelCount = countHotChannelsInBox(design, b.rect, channelHotspots);
        b.hotFtCount = countHotFtInBox(design, b.rect, ftHotspots);
        b.primaryAction = "FT_TO_CHANNEL_REPLACE";
        boxes.push_back(b);
        ++ftBoxCount;
    }

    int channelBoxCount = 0;
    for (const auto& ch : channelHotspots) {
        if (static_cast<int>(boxes.size()) >= opt_.maxBoxes) break;
        if (channelBoxCount >= opt_.maxChannelBoxes) break;
        if (ch.channelIndex < 0 || ch.channelIndex >= static_cast<int>(design.channels.size())) continue;
        RepairBox b;
        b.boxId = boxId++;
        b.seedType = "CHANNEL_OVERFLOW_" + ch.component;
        b.seedName = ch.channelName;
        b.score = ch.score;
        b.rect = expandedRect(design.channels[ch.channelIndex].rect, localExpansion(ch.score, design, opt_), design);
        auto affected = countAffected(routes, ch.channelName);
        b.affectedPieces = affected.first;
        b.affectedNets = affected.second;
        b.hotChannelCount = countHotChannelsInBox(design, b.rect, channelHotspots);
        b.hotFtCount = countHotFtInBox(design, b.rect, ftHotspots);
        b.primaryAction = b.hotFtCount > 0 ? "MIXED_CHANNEL_FT_SUBSTITUTION" : "REROUTE_AROUND_HOT_CHANNEL";
        boxes.push_back(b);
        ++channelBoxCount;
    }

    sort(boxes.begin(), boxes.end(), [](const RepairBox& a, const RepairBox& b) {
        if (a.affectedNets != b.affectedNets) return a.affectedNets > b.affectedNets;
        return a.score > b.score;
    });
    for (int i = 0; i < static_cast<int>(boxes.size()); ++i) {
        boxes[i].boxId = i;
        rr.affectedRoutePieces += boxes[i].affectedPieces;
        rr.affectedNetCount += boxes[i].affectedNets;
    }
    rr.repairBoxCount = static_cast<int>(boxes.size());
    rr.ftTargetCount = rr.ftHotspotCount;

    vector<vector<SelectedRoute>> grouped = groupRoutes(design, routes);
    vector<vector<SelectedRoute>> bestGrouped = grouped;

    Evaluator evaluator;
    Design phase4Design = buildRoutedDesignFromGroups(design, grouped);
    EvalReport phase4Eval = evaluator.evaluate(phase4Design, alpha);
    double bestObjective = repairObjective(phase4Eval, opt_, 0);
    EvalReport bestEval = phase4Eval;

    rr.phase5EvalComputed = !rr.missingPhase4Artifacts;
    rr.initialOpenPaths = phase4Eval.openPathCount;
    rr.initialChannelOverflow = phase4Eval.totalChannelOverflow;
    rr.initialFeedthroughOverflow = phase4Eval.totalFeedthroughOverflow;
    rr.initialWireLength = phase4Eval.totalWireLength;
    rr.initialObjective = bestObjective;

    vector<RepairAttempt> attempts;
    if (!rr.missingPhase4Artifacts && (!ftHotspots.empty() || !channelHotspots.empty())) {
        RouterPhase1::Options p1opt;
        p1opt.exportFiles = false;
        p1opt.maxCandidatesPerConnection = max(12, opt_.maxTrialsPerConnection * 2);
        RouterPhase1 phase1(p1opt);
        RouterPhase1::CandidateBuildResult built = phase1.buildCandidates(design);

        const set<string> hotFts = hotFtNames(ftHotspots);
        const set<string> hotChannels = hotChannelNames(channelHotspots);
        vector<int> repairConnections = selectRepairConnections(
            design, bestGrouped, hotFts, hotChannels, opt_.maxRepairConnections
        );
        rr.repairConnectionCount = static_cast<int>(repairConnections.size());

        int acceptedCount = 0;
        for (int ci : repairConnections) {
            if (ci < 0 || ci >= static_cast<int>(built.candidates.size())) continue;
            rr.attemptedRepairs += 1;
            vector<RepairCandidate> trials = buildRepairCandidates(
                design, ci, built.candidates[ci], hotFts, hotChannels, opt_.maxTrialsPerConnection
            );
            bool acceptedForConn = false;
            for (const auto& trial : trials) {
                rr.candidateTrials += 1;
                vector<vector<SelectedRoute>> candGrouped = bestGrouped;
                candGrouped[ci].clear();
                candGrouped[ci].push_back(makeSelectedRouteFromCandidate(design, ci, trial, 5));

                Design candDesign = buildRoutedDesignFromGroups(design, candGrouped);
                EvalReport candEval = evaluator.evaluate(candDesign, alpha);
                const double candObj = repairObjective(candEval, opt_, acceptedCount + 1);

                RepairAttempt a;
                a.connectionIndex = ci;
                a.candidateIndex = trial.candidateIndex;
                a.family = trial.family;
                a.beforeObjective = bestObjective;
                a.afterObjective = candObj;
                a.beforeChannelOverflow = bestEval.totalChannelOverflow;
                a.afterChannelOverflow = candEval.totalChannelOverflow;
                a.beforeFtOverflow = bestEval.totalFeedthroughOverflow;
                a.afterFtOverflow = candEval.totalFeedthroughOverflow;
                a.beforeWireLength = bestEval.totalWireLength;
                a.afterWireLength = candEval.totalWireLength;
                a.pathSteps = joinPathSteps(trial.path.steps);

                if (acceptableRepair(candEval, candObj, bestEval, bestObjective, phase4Eval, opt_)) {
                    a.accepted = true;
                    a.reason = "accepted_objective_improved";
                    bestGrouped = move(candGrouped);
                    bestEval = candEval;
                    bestObjective = repairObjective(bestEval, opt_, acceptedCount + 1);
                    acceptedCount += 1;
                    attempts.push_back(a);
                    acceptedForConn = true;
                    break;
                }

                a.accepted = false;
                if (!basicLegalityNotWorse(candEval, bestEval)) {
                    a.reason = "rejected_legality_regression";
                } else if (candEval.totalChannelOverflow > phase4Eval.totalChannelOverflow + opt_.channelRegressionGuardAbs) {
                    a.reason = "rejected_channel_regression_guard";
                } else if (candEval.totalFeedthroughOverflow > phase4Eval.totalFeedthroughOverflow + opt_.ftRegressionGuardAbs) {
                    a.reason = "rejected_ft_regression_guard";
                } else {
                    a.reason = "rejected_no_objective_improvement";
                }
                attempts.push_back(a);
            }
            if (acceptedForConn) continue;
        }
        rr.acceptedRepairs = acceptedCount;
    }

    Design phase5Design = buildRoutedDesignFromGroups(design, bestGrouped);
    EvalReport phase5Eval = evaluator.evaluate(phase5Design, alpha);
    rr.phase5EvalHasFail = phase5Eval.hasFail();
    rr.finalOpenPaths = phase5Eval.openPathCount;
    rr.finalChannelOverflow = phase5Eval.totalChannelOverflow;
    rr.finalFeedthroughOverflow = phase5Eval.totalFeedthroughOverflow;
    rr.finalWireLength = phase5Eval.totalWireLength;
    rr.finalObjective = repairObjective(phase5Eval, opt_, rr.acceptedRepairs);

    OutputWriter writer;
    bool writeOk = true;
    if (!writer.write(rr.phase5CfgPath, phase5Design)) writeOk = false;
    writeSelectedRoutes(rr.selectedRoutesPath, bestGrouped);
    writeResourceAfter(rr.resourceAfterPath, phase5Eval);
    writeRepairAttempts(rr.repairAttemptsPath, attempts);

    ofstream fsum(rr.summaryPath);
    if (!fsum) return rr;
    fsum << fixed << setprecision(6);
    fsum << "Phase 5 Local Box Repair Summary\n";
    fsum << "input=" << inputPath << "\n";
    fsum << "baseline_output=" << outputCfgPath << "\n";
    fsum << "phase5_output_cfg=" << rr.phase5CfgPath << "\n";
    fsum << "alpha=" << alpha << "\n";
    fsum << "phase4_selected_routes=" << phase4SelectedRoutesPath << "\n";
    fsum << "phase4_resource_after=" << phase4ResourceAfterPath << "\n";
    fsum << "phase4_ft_handoff=" << ftHandoffPath << "\n";
    fsum << "missing_phase4_artifacts=" << yesNo(rr.missingPhase4Artifacts) << "\n";
    fsum << "ft_hotspot_count=" << rr.ftHotspotCount << "\n";
    fsum << "channel_hotspot_count=" << rr.channelHotspotCount << "\n";
    fsum << "repair_box_count=" << rr.repairBoxCount << "\n";
    fsum << "ft_target_count=" << rr.ftTargetCount << "\n";
    fsum << "affected_route_pieces=" << rr.affectedRoutePieces << "\n";
    fsum << "affected_net_count=" << rr.affectedNetCount << "\n";
    fsum << "repair_connection_count=" << rr.repairConnectionCount << "\n";
    fsum << "attempted_repairs=" << rr.attemptedRepairs << "\n";
    fsum << "candidate_trials=" << rr.candidateTrials << "\n";
    fsum << "accepted_repairs=" << rr.acceptedRepairs << "\n";
    fsum << "initial_open_paths=" << rr.initialOpenPaths << "\n";
    fsum << "final_open_paths=" << rr.finalOpenPaths << "\n";
    fsum << "initial_channel_overflow=" << rr.initialChannelOverflow << "\n";
    fsum << "final_channel_overflow=" << rr.finalChannelOverflow << "\n";
    fsum << "initial_feedthrough_overflow=" << rr.initialFeedthroughOverflow << "\n";
    fsum << "final_feedthrough_overflow=" << rr.finalFeedthroughOverflow << "\n";
    fsum << "initial_wire_length=" << rr.initialWireLength << "\n";
    fsum << "final_wire_length=" << rr.finalWireLength << "\n";
    fsum << "initial_objective=" << rr.initialObjective << "\n";
    fsum << "final_objective=" << rr.finalObjective << "\n";
    fsum << "phase5_eval_has_fail=" << yesNo(rr.phase5EvalHasFail) << "\n";
    fsum << "boxes_csv=" << rr.boxesPath << "\n";
    fsum << "actions_csv=" << rr.actionsPath << "\n";
    fsum << "ft_targets_csv=" << rr.ftTargetsPath << "\n";
    fsum << "repair_attempts_csv=" << rr.repairAttemptsPath << "\n";
    fsum << "selected_routes_csv=" << rr.selectedRoutesPath << "\n";
    fsum << "resource_after_csv=" << rr.resourceAfterPath << "\n";
    fsum << "implementation_scope=deterministic_local_substitution_repair_no_final_output_takeover\n";
    fsum << "reference_basis=BoxRouter_progressive_boxes|Sidewinder_local_candidate_selection|NTHU_range_repair|CUGR_patching|qrouter_window_maze\n";
    fsum.close();

    ofstream fbox(rr.boxesPath);
    if (!fbox) return rr;
    fbox << fixed << setprecision(6);
    fbox << "box_id,seed_type,seed_name,score,x,y,w,h,affected_route_pieces,affected_nets,hot_channels_in_box,hot_ft_blocks_in_box,primary_action,solver_input_required,reference_sources\n";
    for (const auto& b : boxes) {
        fbox << b.boxId << ','
             << csvEscape(b.seedType) << ','
             << csvEscape(b.seedName) << ','
             << b.score << ','
             << b.rect.x << ',' << b.rect.y << ',' << b.rect.w << ',' << b.rect.h << ','
             << b.affectedPieces << ','
             << b.affectedNets << ','
             << b.hotChannelCount << ','
             << b.hotFtCount << ','
             << csvEscape(b.primaryAction) << ",YES,"
             << "BoxRouter|Sidewinder|NTHU-Route2.0|CUGR|qrouter"
             << "\n";
    }
    fbox.close();

    ofstream fact(rr.actionsPath);
    if (!fact) return rr;
    fact << "box_id,action,priority,target_resource,target_connection_hint_count,target_net_hint_count,expected_effect,ft_module_interface,reference_sources\n";
    for (const auto& b : boxes) {
        const string primaryEffect = (b.primaryAction.find("FT") != string::npos)
            ? "reduce_feedthrough_overflow_without_creating_open_path"
            : "reduce_channel_overflow_with_bounded_detour";
        fact << b.boxId << ','
             << csvEscape(b.primaryAction) << ",P0,"
             << csvEscape(b.seedName) << ','
             << b.affectedPieces << ','
             << b.affectedNets << ','
             << csvEscape(primaryEffect) << ','
             << yesNo(b.primaryAction.find("FT") != string::npos) << ','
             << "BoxRouter|Sidewinder|cspy|LEMON|OR-Tools"
             << "\n";
        fact << b.boxId << ",LOCAL_CANDIDATE_REGEN,P0,"
             << csvEscape(b.seedName) << ','
             << b.affectedPieces << ','
             << b.affectedNets << ','
             << "generate_channel_only_and_ft_avoidance_alternatives,"
             << yesNo(b.hotFtCount > 0) << ','
             << "EDGE|CU-GR-2|NCTU-GR2|qrouter"
             << "\n";
    }
    fact.close();

    map<string, int> ftBoxByName;
    for (const auto& b : boxes) {
        if (b.seedType == "FT_OVERFLOW") ftBoxByName[b.seedName] = b.boxId;
    }

    ofstream fft(rr.ftTargetsPath);
    if (!fft) return rr;
    fft << fixed << setprecision(6);
    fft << "schema,block_index,block_name,overflow_area,used_ft_nets,conversion_rate,side_delta,current_area,required_area,repair_box_id,requires_ft_module,interface_payload\n";
    for (const auto& ft : ftHotspots) {
        const int repairBoxId = ftBoxByName.count(ft.blockName) ? ftBoxByName[ft.blockName] : -1;
        ostringstream payload;
        payload << "block=" << ft.blockName
                << "|overflow_area=" << ft.overflowArea
                << "|used_ft_nets=" << ft.usedFtNets
                << "|side_delta=" << ft.sideDelta
                << "|repair_box_id=" << repairBoxId;
        fft << "phase5_ft_target_v1,"
            << ft.blockIndex << ','
            << csvEscape(ft.blockName) << ','
            << ft.overflowArea << ','
            << ft.usedFtNets << ','
            << ft.conversionRate << ','
            << ft.sideDelta << ','
            << ft.currentArea << ','
            << ft.requiredArea << ','
            << repairBoxId << ",YES,"
            << csvEscape(payload.str()) << "\n";
    }
    fft.close();

    rr.ok = writeOk && !rr.missingPhase4Artifacts;
    return rr;
}
