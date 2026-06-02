#include "RouterPhase6.hpp"

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

struct ConnQuality {
    int connectionIndex = -1;
    string src;
    string dst;
    int netCount = 0;
    int pieceCount = 0;
    int allocatedNets = 0;
    int uniquePathCount = 0;
    int duplicateSplitCount = 0;
    int maxPathStepCount = 0;
    int repeatedObjectPathCount = 0;
    int ftHotspotTouchCount = 0;
    int hotChannelTouchCount = 0;
    double qualityRiskScore = 0.0;
};

struct RecoveryAttempt {
    int connectionIndex = -1;
    string action;
    int candidateIndex = -1;
    string family;
    bool accepted = false;
    string reason;
    int beforeRouteCount = 0;
    int afterRouteCount = 0;
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
    for (int i = 0; i < static_cast<int>(t.header.size()); ++i) t.col[t.header[i]] = i;
    while (getline(fin, line)) {
        if (!line.empty()) t.rows.push_back(splitCsvLine(line));
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

string outputStemName(const string& outputCfgPath) {
    fs::path out(outputCfgPath);
    string stem = out.stem().string();
    return stem.empty() ? "router" : stem;
}

fs::path resultStemPath(const string& outputCfgPath) {
    fs::path out(outputCfgPath);
    return out.parent_path() / out.stem();
}

fs::path reportStemPath(const string& outputCfgPath) {
    fs::path dir("Router_Statistics");
    fs::create_directories(dir);
    return dir / outputStemName(outputCfgPath);
}

vector<string> splitRawPathTokens(const string& pathSteps) {
    vector<string> tokens;
    string cur;
    for (char c : pathSteps) {
        if (c == '|') {
            if (!cur.empty()) tokens.push_back(cur);
            cur.clear();
        } else {
            cur += c;
        }
    }
    if (!cur.empty()) tokens.push_back(cur);
    return tokens;
}

vector<string> splitPathObjects(const string& pathSteps) {
    vector<string> tokens = splitRawPathTokens(pathSteps);
    for (string& t : tokens) {
        const size_t colon = t.find(':');
        if (colon != string::npos) t = t.substr(0, colon);
    }
    return tokens;
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

bool hasRepeatedObject(const string& pathSteps) {
    set<string> seen;
    for (const auto& token : splitPathObjects(pathSteps)) {
        if (seen.count(token)) return true;
        seen.insert(token);
    }
    return false;
}

string joinPathSteps(const vector<RouteStep>& steps) {
    ostringstream oss;
    for (size_t i = 0; i < steps.size(); ++i) {
        if (i) oss << '|';
        oss << steps[i].rectName << ':' << steps[i].edge;
    }
    return oss.str();
}

RoutePath parseRoutePathFromSteps(const Design& design, int connectionIndex, int nets, const string& pathSteps) {
    RoutePath p;
    p.netCount = nets;
    if (connectionIndex >= 0 && connectionIndex < static_cast<int>(design.connections.size())) {
        const Connection& conn = design.connections[connectionIndex];
        if (conn.src >= 0 && conn.src < static_cast<int>(design.blocks.size())) p.srcBlock = design.blocks[conn.src].spec.name;
        if (conn.dst >= 0 && conn.dst < static_cast<int>(design.blocks.size())) p.dstBlock = design.blocks[conn.dst].spec.name;
    }
    for (const string& token : splitRawPathTokens(pathSteps)) {
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

set<string> parseHotFtNamesFromResource(const string& path) {
    CsvTable t = readCsvTable(path);
    set<string> out;
    for (const auto& row : t.rows) {
        if (getField(t, row, "resource_type") != "SOFT_FT") continue;
        if (toDouble(getField(t, row, "overflow")) > EPS || toDouble(getField(t, row, "overflow_soft")) > EPS) {
            out.insert(getField(t, row, "name"));
        }
    }
    return out;
}

set<string> parseHotChannelNamesFromResource(const string& path) {
    CsvTable t = readCsvTable(path);
    set<string> out;
    for (const auto& row : t.rows) {
        const string type = getField(t, row, "resource_type");
        if (type != "CHANNEL_LR" && type != "CHANNEL_TB") continue;
        const double ov = max(toDouble(getField(t, row, "overflow")), max(toDouble(getField(t, row, "overflow_hard")), toDouble(getField(t, row, "overflow_soft"))));
        if (ov > EPS) out.insert(getField(t, row, "name"));
    }
    return out;
}

vector<vector<SelectedRoute>> groupRoutes(const Design& design, const vector<SelectedRoute>& routes) {
    vector<vector<SelectedRoute>> grouped(design.connections.size());
    for (const auto& r : routes) {
        if (r.connectionIndex >= 0 && r.connectionIndex < static_cast<int>(grouped.size())) grouped[r.connectionIndex].push_back(r);
    }
    return grouped;
}

int routeCount(const vector<vector<SelectedRoute>>& grouped) {
    int n = 0;
    for (const auto& vec : grouped) n += static_cast<int>(vec.size());
    return n;
}

int duplicateSplitCount(const vector<vector<SelectedRoute>>& grouped) {
    int dup = 0;
    for (const auto& vec : grouped) {
        map<string, int> counts;
        for (const auto& r : vec) counts[r.pathSteps] += 1;
        for (const auto& kv : counts) dup += max(0, kv.second - 1);
    }
    return dup;
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

double recoveryObjective(const EvalReport& eval, const vector<vector<SelectedRoute>>& grouped, const RouterPhase6::Options& opt, int routeChanges) {
    return opt.wireLengthWeight * eval.totalWireLength
        + opt.routeCountWeight * static_cast<double>(routeCount(grouped))
        + opt.duplicateSplitWeight * static_cast<double>(duplicateSplitCount(grouped))
        + opt.routeChangePenalty * static_cast<double>(routeChanges);
}

bool basicNotWorse(const EvalReport& cand, const EvalReport& base) {
    if (cand.formatFailed && !base.formatFailed) return false;
    if (cand.pathInvalid && !base.pathInvalid) return false;
    if (cand.blockOverlap && !base.blockOverlap) return false;
    if (cand.routingOpen && !base.routingOpen) return false;
    if (cand.outlineViolation && !base.outlineViolation) return false;
    if (cand.openPathCount > base.openPathCount) return false;
    if (cand.invalidPathCount > base.invalidPathCount) return false;
    if (cand.totalChannelOverflow > base.totalChannelOverflow + 1.0e-6) return false;
    if (cand.totalFeedthroughOverflow > base.totalFeedthroughOverflow + 1.0e-6) return false;
    return true;
}

bool acceptRecovery(const EvalReport& cand, double candObj, const EvalReport& best, double bestObj) {
    if (!basicNotWorse(cand, best)) return false;
    return candObj + 1.0e-6 < bestObj;
}

map<int, ConnQuality> buildConnectionQuality(
    const Design& design,
    const vector<vector<SelectedRoute>>& grouped,
    const set<string>& ftHotspots,
    const set<string>& hotChannels,
    int longPathStepThreshold
) {
    map<int, ConnQuality> out;
    for (int ci = 0; ci < static_cast<int>(design.connections.size()); ++ci) {
        const Connection& conn = design.connections[ci];
        ConnQuality q;
        q.connectionIndex = ci;
        q.netCount = conn.netCount;
        if (conn.src >= 0 && conn.src < static_cast<int>(design.blocks.size())) q.src = design.blocks[conn.src].spec.name;
        if (conn.dst >= 0 && conn.dst < static_cast<int>(design.blocks.size())) q.dst = design.blocks[conn.dst].spec.name;

        set<string> uniquePaths;
        for (const auto& r : grouped[ci]) {
            q.pieceCount += 1;
            q.allocatedNets += r.allocatedNets;
            uniquePaths.insert(r.pathSteps);
            q.maxPathStepCount = max(q.maxPathStepCount, static_cast<int>(splitPathObjects(r.pathSteps).size()));
            if (hasRepeatedObject(r.pathSteps)) q.repeatedObjectPathCount += 1;
            for (const auto& name : ftHotspots) if (containsToken(r.pathSteps, name)) q.ftHotspotTouchCount += 1;
            for (const auto& name : hotChannels) if (containsToken(r.pathSteps, name)) q.hotChannelTouchCount += 1;
        }
        q.uniquePathCount = static_cast<int>(uniquePaths.size());
        q.duplicateSplitCount = max(0, q.pieceCount - q.uniquePathCount);
        q.qualityRiskScore =
            40.0 * q.duplicateSplitCount
            + 8.0 * max(0, q.pieceCount - 1)
            + 4.0 * max(0, q.maxPathStepCount - longPathStepThreshold)
            + 3.0 * q.repeatedObjectPathCount
            + 2.0 * q.ftHotspotTouchCount
            + 1.0 * q.hotChannelTouchCount;
        out[ci] = q;
    }
    return out;
}

vector<int> rankedRecoveryConnections(const map<int, ConnQuality>& quality, int limit) {
    vector<ConnQuality> ranked;
    for (const auto& kv : quality) {
        if (kv.second.qualityRiskScore > EPS) ranked.push_back(kv.second);
    }
    sort(ranked.begin(), ranked.end(), [](const ConnQuality& a, const ConnQuality& b) {
        if (fabs(a.qualityRiskScore - b.qualityRiskScore) > EPS) return a.qualityRiskScore > b.qualityRiskScore;
        return a.connectionIndex < b.connectionIndex;
    });
    if (static_cast<int>(ranked.size()) > limit) ranked.resize(limit);
    vector<int> out;
    for (const auto& q : ranked) out.push_back(q.connectionIndex);
    return out;
}

vector<vector<SelectedRoute>> mergeDuplicateSplits(const vector<vector<SelectedRoute>>& grouped, int& mergedPieces) {
    vector<vector<SelectedRoute>> out = grouped;
    mergedPieces = 0;
    for (auto& vec : out) {
        map<string, SelectedRoute> merged;
        vector<string> order;
        for (const auto& r : vec) {
            auto it = merged.find(r.pathSteps);
            if (it == merged.end()) {
                SelectedRoute m = r;
                m.family = "P6_MERGED_" + r.family;
                m.assignedIteration = 6;
                merged[r.pathSteps] = m;
                order.push_back(r.pathSteps);
            } else {
                it->second.allocatedNets += r.allocatedNets;
                it->second.path.netCount = it->second.allocatedNets;
                it->second.score += r.score;
                mergedPieces += 1;
            }
        }
        vec.clear();
        for (const string& key : order) vec.push_back(merged[key]);
    }
    return out;
}

SelectedRoute makeSelectedRouteFromCandidate(const Design& design, int ci, const Phase1RouteCandidate& cand) {
    SelectedRoute r;
    r.connectionIndex = ci;
    r.candidateIndex = cand.candidateIndex;
    r.family = "P6_" + cand.family;
    r.allocatedNets = design.connections[ci].netCount;
    r.assignedIteration = 6;
    r.score = cand.score;
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

void writeAttempts(const string& path, const vector<RecoveryAttempt>& attempts) {
    ofstream f(path);
    f << fixed << setprecision(6);
    f << "connection_index,action,candidate_index,family,accepted,reason,before_route_count,after_route_count,before_objective,after_objective,before_channel_overflow,after_channel_overflow,before_ft_overflow,after_ft_overflow,before_wirelength,after_wirelength,path_steps\n";
    for (const auto& a : attempts) {
        f << a.connectionIndex << ','
          << csvEscape(a.action) << ','
          << a.candidateIndex << ','
          << csvEscape(a.family) << ','
          << yesNo(a.accepted) << ','
          << csvEscape(a.reason) << ','
          << a.beforeRouteCount << ','
          << a.afterRouteCount << ','
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

RouterPhase6::RunResult RouterPhase6::run(
    const Design& design,
    const string& inputPath,
    const string& outputCfgPath,
    double alpha,
    Design* outputDesign
) const {
    RunResult rr;
    if (!opt_.exportFiles) {
        rr.ok = true;
        return rr;
    }

    const fs::path resultStem = resultStemPath(outputCfgPath);
    const fs::path rptStem = reportStemPath(outputCfgPath);

    const string phase5RoutesPath = rptStem.string() + "_phase5_selected_routes.csv";
    const string phase5ResourcePath = rptStem.string() + "_phase5_resource_after.csv";
    const string phase4RoutesNew = rptStem.string() + "_phase4_selected_routes.csv";
    const string phase4ResourceNew = rptStem.string() + "_phase4_resource_after.csv";
    const string phase4RoutesOld = resultStem.string() + "_phase4_selected_routes.csv";
    const string phase4ResourceOld = resultStem.string() + "_phase4_resource_after.csv";
    const string phase4RoutesPath = fs::exists(phase4RoutesNew) ? phase4RoutesNew : phase4RoutesOld;
    const string phase4ResourcePath = fs::exists(phase4ResourceNew) ? phase4ResourceNew : phase4ResourceOld;

    const bool hasPhase5 = fs::exists(phase5RoutesPath) && fs::exists(phase5ResourcePath);
    rr.usedPhase5Artifacts = hasPhase5;
    const string inputRoutesPath = hasPhase5 ? phase5RoutesPath : phase4RoutesPath;
    const string inputResourcePath = hasPhase5 ? phase5ResourcePath : phase4ResourcePath;

    rr.summaryPath = rptStem.string() + "_phase6_summary.txt";
    rr.qualityPath = rptStem.string() + "_phase6_connection_quality.csv";
    rr.actionsPath = rptStem.string() + "_phase6_recovery_actions.csv";
    rr.attemptsPath = rptStem.string() + "_phase6_recovery_attempts.csv";
    rr.selectedRoutesPath = rptStem.string() + "_phase6_selected_routes.csv";
    rr.resourceAfterPath = rptStem.string() + "_phase6_resource_after.csv";
    rr.phase6CfgPath = resultStem.string() + "_phase6.cfg";

    rr.missingPhase4Artifacts = !(fs::exists(inputRoutesPath) && fs::exists(inputResourcePath));

    vector<SelectedRoute> routes;
    set<string> ftHotspots;
    set<string> hotChannels;
    if (!rr.missingPhase4Artifacts) {
        routes = parseSelectedRoutes(design, inputRoutesPath);
        ftHotspots = parseHotFtNamesFromResource(inputResourcePath);
        hotChannels = parseHotChannelNamesFromResource(inputResourcePath);
    }

    vector<vector<SelectedRoute>> grouped = groupRoutes(design, routes);
    Evaluator evaluator;
    Design initialDesign = buildRoutedDesignFromGroups(design, grouped);
    EvalReport initialEval = evaluator.evaluate(initialDesign, alpha);
    double bestObjective = recoveryObjective(initialEval, grouped, opt_, 0);
    EvalReport bestEval = initialEval;

    rr.phase6EvalComputed = !rr.missingPhase4Artifacts;
    rr.initialRouteCount = routeCount(grouped);
    rr.initialOpenPaths = initialEval.openPathCount;
    rr.initialInvalidPaths = initialEval.invalidPathCount;
    rr.initialChannelOverflow = initialEval.totalChannelOverflow;
    rr.initialFeedthroughOverflow = initialEval.totalFeedthroughOverflow;
    rr.initialWireLength = initialEval.totalWireLength;
    rr.initialObjective = bestObjective;

    vector<RecoveryAttempt> attempts;
    int routeChanges = 0;

    if (!rr.missingPhase4Artifacts) {
        int mergedPieces = 0;
        vector<vector<SelectedRoute>> mergedGrouped = mergeDuplicateSplits(grouped, mergedPieces);
        if (mergedPieces > 0) {
            Design candDesign = buildRoutedDesignFromGroups(design, mergedGrouped);
            EvalReport candEval = evaluator.evaluate(candDesign, alpha);
            const double candObj = recoveryObjective(candEval, mergedGrouped, opt_, routeChanges + mergedPieces);
            RecoveryAttempt a;
            a.connectionIndex = -1;
            a.action = "MERGE_DUPLICATE_SPLITS";
            a.candidateIndex = -1;
            a.family = "P6_MERGE";
            a.beforeRouteCount = routeCount(grouped);
            a.afterRouteCount = routeCount(mergedGrouped);
            a.beforeObjective = bestObjective;
            a.afterObjective = candObj;
            a.beforeChannelOverflow = bestEval.totalChannelOverflow;
            a.afterChannelOverflow = candEval.totalChannelOverflow;
            a.beforeFtOverflow = bestEval.totalFeedthroughOverflow;
            a.afterFtOverflow = candEval.totalFeedthroughOverflow;
            a.beforeWireLength = bestEval.totalWireLength;
            a.afterWireLength = candEval.totalWireLength;
            a.pathSteps = "global_duplicate_merge";
            if (acceptRecovery(candEval, candObj, bestEval, bestObjective)) {
                a.accepted = true;
                a.reason = "accepted_route_count_and_duplicate_reduction";
                grouped = move(mergedGrouped);
                bestEval = candEval;
                bestObjective = recoveryObjective(bestEval, grouped, opt_, routeChanges + mergedPieces);
                routeChanges += mergedPieces;
                rr.mergedDuplicatePieces = mergedPieces;
                rr.acceptedRecoveries += 1;
            } else {
                a.accepted = false;
                if (basicNotWorse(candEval, bestEval)) {
                    a.reason = "rejected_no_objective_improvement";
                    rr.rejectedByObjectiveCount += 1;
                } else {
                    a.reason = "rejected_guard_regression";
                    rr.rejectedByGuardCount += 1;
                }
            }
            attempts.push_back(a);
            rr.attemptedRecoveries += 1;
        }

        map<int, ConnQuality> quality = buildConnectionQuality(design, grouped, ftHotspots, hotChannels, opt_.longPathStepThreshold);
        vector<int> recoveryConnections = rankedRecoveryConnections(quality, opt_.maxRecoveryConnections);

        RouterPhase1::Options p1opt;
        p1opt.exportFiles = false;
        p1opt.maxCandidatesPerConnection = max(12, opt_.maxCandidateTrialsPerConnection * 2);
        RouterPhase1 phase1(p1opt);
        RouterPhase1::CandidateBuildResult built = phase1.buildCandidates(design);

        for (int ci : recoveryConnections) {
            if (ci < 0 || ci >= static_cast<int>(built.candidates.size())) continue;
            rr.attemptedRecoveries += 1;
            int trials = 0;
            vector<Phase1RouteCandidate> candVec = built.candidates[ci];
            sort(candVec.begin(), candVec.end(), [](const Phase1RouteCandidate& a, const Phase1RouteCandidate& b) {
                if (fabs(a.wireLength - b.wireLength) > EPS) return a.wireLength < b.wireLength;
                if (fabs(a.score - b.score) > EPS) return a.score < b.score;
                return a.family < b.family;
            });
            set<string> tried;
            bool acceptedForConn = false;
            for (const auto& cand : candVec) {
                if (trials >= opt_.maxCandidateTrialsPerConnection) break;
                if (cand.path.open || cand.path.steps.size() < 2) continue;
                const string sig = joinPathSteps(cand.path.steps);
                if (tried.count(sig)) continue;
                tried.insert(sig);
                ++trials;
                ++rr.candidateTrials;

                vector<vector<SelectedRoute>> candGrouped = grouped;
                candGrouped[ci].clear();
                candGrouped[ci].push_back(makeSelectedRouteFromCandidate(design, ci, cand));
                Design candDesign = buildRoutedDesignFromGroups(design, candGrouped);
                EvalReport candEval = evaluator.evaluate(candDesign, alpha);
                const double candObj = recoveryObjective(candEval, candGrouped, opt_, routeChanges + 1);

                RecoveryAttempt a;
                a.connectionIndex = ci;
                a.action = "SAFE_FULL_CONNECTION_SUBSTITUTION";
                a.candidateIndex = cand.candidateIndex;
                a.family = cand.family;
                a.beforeRouteCount = routeCount(grouped);
                a.afterRouteCount = routeCount(candGrouped);
                a.beforeObjective = bestObjective;
                a.afterObjective = candObj;
                a.beforeChannelOverflow = bestEval.totalChannelOverflow;
                a.afterChannelOverflow = candEval.totalChannelOverflow;
                a.beforeFtOverflow = bestEval.totalFeedthroughOverflow;
                a.afterFtOverflow = candEval.totalFeedthroughOverflow;
                a.beforeWireLength = bestEval.totalWireLength;
                a.afterWireLength = candEval.totalWireLength;
                a.pathSteps = sig;

                if (acceptRecovery(candEval, candObj, bestEval, bestObjective)) {
                    a.accepted = true;
                    a.reason = "accepted_quality_objective_improved";
                    grouped = move(candGrouped);
                    bestEval = candEval;
                    bestObjective = recoveryObjective(bestEval, grouped, opt_, routeChanges + 1);
                    routeChanges += 1;
                    rr.acceptedRecoveries += 1;
                    attempts.push_back(a);
                    acceptedForConn = true;
                    break;
                }

                a.accepted = false;
                if (basicNotWorse(candEval, bestEval)) {
                    a.reason = "rejected_no_objective_improvement";
                    rr.rejectedByObjectiveCount += 1;
                } else {
                    a.reason = "rejected_guard_regression";
                    rr.rejectedByGuardCount += 1;
                }
                attempts.push_back(a);
            }
            if (acceptedForConn) continue;
        }
    }

    Design phase6Design = buildRoutedDesignFromGroups(design, grouped);
    EvalReport finalEval = evaluator.evaluate(phase6Design, alpha);
    if (outputDesign) {
        *outputDesign = phase6Design;
    }
    rr.phase6EvalHasFail = finalEval.hasFail();
    rr.finalRouteCount = routeCount(grouped);
    rr.finalOpenPaths = finalEval.openPathCount;
    rr.finalInvalidPaths = finalEval.invalidPathCount;
    rr.finalChannelOverflow = finalEval.totalChannelOverflow;
    rr.finalFeedthroughOverflow = finalEval.totalFeedthroughOverflow;
    rr.finalWireLength = finalEval.totalWireLength;
    rr.finalObjective = recoveryObjective(finalEval, grouped, opt_, routeChanges);

    map<int, ConnQuality> finalQuality = buildConnectionQuality(design, grouped, ftHotspots, hotChannels, opt_.longPathStepThreshold);
    vector<ConnQuality> rankedQuality;
    for (const auto& kv : finalQuality) {
        const ConnQuality& q = kv.second;
        rr.connectionCount += 1;
        if (q.pieceCount > 1) rr.splitConnectionCount += 1;
        rr.duplicateSplitCount += q.duplicateSplitCount;
        if (q.ftHotspotTouchCount > 0) rr.ftHeavyConnectionCount += 1;
        if (q.hotChannelTouchCount > 0) rr.hotChannelConnectionCount += 1;
        if (q.qualityRiskScore > EPS) rankedQuality.push_back(q);
    }
    sort(rankedQuality.begin(), rankedQuality.end(), [](const ConnQuality& a, const ConnQuality& b) {
        if (fabs(a.qualityRiskScore - b.qualityRiskScore) > EPS) return a.qualityRiskScore > b.qualityRiskScore;
        return a.connectionIndex < b.connectionIndex;
    });
    OutputWriter writer;
    bool writeOk = writer.write(rr.phase6CfgPath, phase6Design);
    writeSelectedRoutes(rr.selectedRoutesPath, grouped);
    writeResourceAfter(rr.resourceAfterPath, finalEval);
    writeAttempts(rr.attemptsPath, attempts);

    ofstream fsum(rr.summaryPath);
    if (!fsum) return rr;
    fsum << fixed << setprecision(6);
    fsum << "Phase 6 Quality Recovery Summary\n";
    fsum << "input=" << inputPath << "\n";
    fsum << "requested_output=" << outputCfgPath << "\n";
    fsum << "phase6_output_cfg=" << rr.phase6CfgPath << "\n";
    fsum << "alpha=" << alpha << "\n";
    fsum << "used_phase5_artifacts=" << yesNo(rr.usedPhase5Artifacts) << "\n";
    fsum << "input_selected_routes=" << inputRoutesPath << "\n";
    fsum << "input_resource_after=" << inputResourcePath << "\n";
    fsum << "missing_input_artifacts=" << yesNo(rr.missingPhase4Artifacts) << "\n";
    fsum << "connection_count=" << rr.connectionCount << "\n";
    fsum << "split_connection_count=" << rr.splitConnectionCount << "\n";
    fsum << "duplicate_split_count=" << rr.duplicateSplitCount << "\n";
    fsum << "merged_duplicate_pieces=" << rr.mergedDuplicatePieces << "\n";
    fsum << "attempted_recoveries=" << rr.attemptedRecoveries << "\n";
    fsum << "candidate_trials=" << rr.candidateTrials << "\n";
    fsum << "accepted_recoveries=" << rr.acceptedRecoveries << "\n";
    fsum << "ft_heavy_connection_count=" << rr.ftHeavyConnectionCount << "\n";
    fsum << "hot_channel_connection_count=" << rr.hotChannelConnectionCount << "\n";
    fsum << "rejected_by_guard_count=" << rr.rejectedByGuardCount << "\n";
    fsum << "rejected_by_objective_count=" << rr.rejectedByObjectiveCount << "\n";
    fsum << "initial_route_count=" << rr.initialRouteCount << "\n";
    fsum << "final_route_count=" << rr.finalRouteCount << "\n";
    fsum << "initial_open_paths=" << rr.initialOpenPaths << "\n";
    fsum << "final_open_paths=" << rr.finalOpenPaths << "\n";
    fsum << "initial_invalid_paths=" << rr.initialInvalidPaths << "\n";
    fsum << "final_invalid_paths=" << rr.finalInvalidPaths << "\n";
    fsum << "initial_channel_overflow=" << rr.initialChannelOverflow << "\n";
    fsum << "final_channel_overflow=" << rr.finalChannelOverflow << "\n";
    fsum << "initial_feedthrough_overflow=" << rr.initialFeedthroughOverflow << "\n";
    fsum << "final_feedthrough_overflow=" << rr.finalFeedthroughOverflow << "\n";
    fsum << "initial_wire_length=" << rr.initialWireLength << "\n";
    fsum << "final_wire_length=" << rr.finalWireLength << "\n";
    fsum << "initial_objective=" << rr.initialObjective << "\n";
    fsum << "final_objective=" << rr.finalObjective << "\n";
    fsum << "phase6_eval_has_fail=" << yesNo(rr.phase6EvalHasFail) << "\n";
    fsum << "quality_csv=" << rr.qualityPath << "\n";
    fsum << "actions_csv=" << rr.actionsPath << "\n";
    fsum << "attempts_csv=" << rr.attemptsPath << "\n";
    fsum << "selected_routes_csv=" << rr.selectedRoutesPath << "\n";
    fsum << "resource_after_csv=" << rr.resourceAfterPath << "\n";
    fsum << "implementation_scope=safe_postroute_recovery_final_output_ready\n";
    fsum << "reference_basis=MaizeRouter_retraction|FastRoute_cleanup|CUGR_patching|VGR_recovery|NTHU_postprocess|RapidRoute_reuse\n";
    fsum.close();

    ofstream fq(rr.qualityPath);
    if (!fq) return rr;
    fq << fixed << setprecision(6);
    fq << "connection_index,src,dst,net_count,allocated_nets,piece_count,unique_path_count,duplicate_split_count,max_path_step_count,repeated_object_path_count,ft_hotspot_touch_count,hot_channel_touch_count,quality_risk_score\n";
    for (const auto& q : rankedQuality) {
        fq << q.connectionIndex << ','
           << csvEscape(q.src) << ','
           << csvEscape(q.dst) << ','
           << q.netCount << ','
           << q.allocatedNets << ','
           << q.pieceCount << ','
           << q.uniquePathCount << ','
           << q.duplicateSplitCount << ','
           << q.maxPathStepCount << ','
           << q.repeatedObjectPathCount << ','
           << q.ftHotspotTouchCount << ','
           << q.hotChannelTouchCount << ','
           << q.qualityRiskScore << "\n";
    }
    fq.close();

    ofstream fa(rr.actionsPath);
    if (!fa) return rr;
    fa << "connection_index,action,priority,allowed_to_mutate_now,reason,expected_effect,reference_sources\n";
    int emitted = 0;
    for (const auto& q : rankedQuality) {
        if (emitted >= opt_.highRiskConnectionLimit) break;
        if (q.duplicateSplitCount > 0) {
            fa << q.connectionIndex << ",MERGE_DUPLICATE_SPLITS,P0,YES,same_path_used_by_multiple_split_pieces,reduce_route_count_without_resource_change,MaizeRouter|NTHU-Route2.0|RapidRoute\n";
            ++rr.recoveryActionCount;
        }
        if (q.repeatedObjectPathCount > 0 || q.maxPathStepCount > opt_.longPathStepThreshold) {
            fa << q.connectionIndex << ",SAFE_FULL_CONNECTION_SUBSTITUTION,P1,YES,path_has_repeated_object_or_long_detour,reduce_wirelength_if_overflow_and_legality_do_not_regress,MaizeRouter|FastRoute4.0|V-GR\n";
            ++rr.recoveryActionCount;
        }
        if (q.ftHotspotTouchCount > 0) {
            fa << q.connectionIndex << ",FT_TO_CHANNEL_RECOVERY,P0,NO,route_still_touches_ft_hotspot,delegate_to_future_ft_module_or_phase5_box_solver,CUGR|V-GR|Phase5_FT_interface\n";
            ++rr.recoveryActionCount;
        }
        if (q.hotChannelTouchCount > 0) {
            fa << q.connectionIndex << ",HOT_CHANNEL_DETOUR_CLEANUP,P1,NO,route_still_touches_overflow_channel,requires_phase5_box_candidate_regeneration,BoxRouter|CUGR|NCTU-GR2\n";
            ++rr.recoveryActionCount;
        }
        ++emitted;
    }
    fa.close();

    rr.ok = writeOk && !rr.missingPhase4Artifacts;
    return rr;
}
