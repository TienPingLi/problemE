#include "CongestionMapBuilder.hpp"
#include "Utility.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <vector>
#include <utility>

using namespace std;

namespace {

    static constexpr double STEP0_EPS = 1.0e-6;

    struct ConnState {
        vector<Step0PatternCandidate> candidates;
        vector<double> assignedWeight; // 每個 candidate 目前分到的比例，總和約為 1。
        int selectedCandidate = -1;
        double selectedCost = 0.0;
    };

    static double safeCap(double cap) {
        return max(1.0, cap);
    }

    static bool isHorizontal(const Step0PatternSegment& s) {
        return fabs(s.y1 - s.y2) <= STEP0_EPS && fabs(s.x1 - s.x2) > STEP0_EPS;
    }

    static bool isVertical(const Step0PatternSegment& s) {
        return fabs(s.x1 - s.x2) <= STEP0_EPS && fabs(s.y1 - s.y2) > STEP0_EPS;
    }

    static double segLength(const Step0PatternSegment& s) {
        return fabs(s.x1 - s.x2) + fabs(s.y1 - s.y2);
    }

    static void addSegment(vector<Step0PatternSegment>& segs, double x1, double y1, double x2, double y2) {
        if (fabs(x1 - x2) <= STEP0_EPS && fabs(y1 - y2) <= STEP0_EPS) return;
        segs.push_back({ x1, y1, x2, y2 });
    }

    static double candidateWireLength(const Step0PatternCandidate& c) {
        double wl = 0.0;
        for (const auto& s : c.segs) wl += segLength(s);
        return wl;
    }


    static int bendCountForCandidateStep0(const Step0PatternCandidate& c) {
        if (c.segs.empty()) return 0;
        int bends = 0;
        for (int i = 1; i < static_cast<int>(c.segs.size()); ++i) {
            const bool prevH = isHorizontal(c.segs[i - 1]);
            const bool curH = isHorizontal(c.segs[i]);
            const bool prevV = isVertical(c.segs[i - 1]);
            const bool curV = isVertical(c.segs[i]);
            if ((prevH && curV) || (prevV && curH)) ++bends;
        }
        return bends;
    }

    static bool patternNameIsOuterDetourStep0(const string& name) {
        return name.find("LEFT") != string::npos || name.find("RIGHT") != string::npos ||
            name.find("TOP") != string::npos || name.find("BOTTOM") != string::npos;
    }

    static int hardEdgeBlockOverlapCountForConnStep0(const Design& design, const Connection& conn) {
        if (conn.src < 0 || conn.src >= static_cast<int>(design.blocks.size()) ||
            conn.dst < 0 || conn.dst >= static_cast<int>(design.blocks.size())) return 0;
        const Rect& a = design.blocks[conn.src].rect;
        const Rect& b = design.blocks[conn.dst].rect;
        Rect box;
        box.x = min(rectCx(a), rectCx(b));
        box.y = min(rectCy(a), rectCy(b));
        box.w = max(rectCx(a), rectCx(b)) - box.x;
        box.h = max(rectCy(a), rectCy(b)) - box.y;
        box.x -= 0.5; box.y -= 0.5; box.w += 1.0; box.h += 1.0;
        int cnt = 0;
        for (int i = 0; i < static_cast<int>(design.blocks.size()); ++i) {
            if (i == conn.src || i == conn.dst) continue;
            const auto& blk = design.blocks[i];
            if (blk.spec.type != BlockType::HARD && blk.spec.type != BlockType::EDGE) continue;
            if (overlapLen(box.x, rectRight(box), blk.rect.x, rectRight(blk.rect)) > STEP0_EPS && overlapLen(box.y, rectTop(box), blk.rect.y, rectTop(blk.rect)) > STEP0_EPS) ++cnt;
        }
        return cnt;
    }

    static double nearFullPenalty(double util) {
        if (!std::isfinite(util)) return 1.0e9;

        // 平滑 penalty：遠低於容量時溫和，接近滿載時快速變貴。
        double p = util * util;

        if (util > 0.70) {
            const double x = util - 0.70;
            p += 5.0 * x * x;
        }
        if (util > 0.90) {
            const double x = util - 0.90;
            p += 30.0 * x * x * x * x;
        }
        if (util > 1.00) {
            const double x = util - 1.00;
            p += 200.0 * x * x;
        }
        return p;
    }

    static bool horizontalLineCrossesRect(const Step0PatternSegment& s, const Rect& r, double tol) {
        if (!isHorizontal(s)) return false;
        const double xa = min(s.x1, s.x2);
        const double xb = max(s.x1, s.x2);
        const bool yInside = s.y1 >= r.y - tol && s.y1 <= rectTop(r) + tol;
        const bool xOverlap = overlapLen(xa, xb, r.x, rectRight(r)) > EPS;
        return yInside && xOverlap;
    }

    static bool verticalLineCrossesRect(const Step0PatternSegment& s, const Rect& r, double tol) {
        if (!isVertical(s)) return false;
        const double ya = min(s.y1, s.y2);
        const double yb = max(s.y1, s.y2);
        const bool xInside = s.x1 >= r.x - tol && s.x1 <= rectRight(r) + tol;
        const bool yOverlap = overlapLen(ya, yb, r.y, rectTop(r)) > EPS;
        return xInside && yOverlap;
    }

    static double blockagePenaltyForCandidate(
        const Design& design,
        const Connection& conn,
        const Step0PatternCandidate& cand,
        const CongestionMapBuilder::Options& opt
    ) {
        double hardHits = 0.0;
        double softHits = 0.0;

        for (int bi = 0; bi < static_cast<int>(design.blocks.size()); ++bi) {
            if (bi == conn.src || bi == conn.dst) continue;
            const BlockInst& b = design.blocks[bi];

            for (const auto& seg : cand.segs) {
                const bool crosses =
                    horizontalLineCrossesRect(seg, b.rect, 0.0) ||
                    verticalLineCrossesRect(seg, b.rect, 0.0);

                if (!crosses) continue;

                if (b.spec.type == BlockType::HARD || b.spec.type == BlockType::EDGE) {
                    hardHits += 1.0;
                }
                else if (b.spec.type == BlockType::SOFT) {
                    softHits += 1.0;
                }
            }
        }

        // 這裡只是 guide penalty，不是合法性判定。
        return static_cast<double>(conn.netCount) *
            (opt.wHardBlockage * hardHits + opt.wSoftBlockage * softHits);
    }

    static vector<Step0PatternCandidate> generateCandidates(
        const Design& design,
        const Connection& conn,
        const CongestionMapBuilder::Options& opt
    ) {
        vector<Step0PatternCandidate> out;
        if (conn.src < 0 || conn.src >= static_cast<int>(design.blocks.size()) ||
            conn.dst < 0 || conn.dst >= static_cast<int>(design.blocks.size())) {
            return out;
        }

        const Rect& sr = design.blocks[conn.src].rect;
        const Rect& tr = design.blocks[conn.dst].rect;

        // 第一版用 block center 當 pattern guide 端點。
        // 注意：正式 Router 仍會用合法接觸 edge 產生 PATH，Step0 不處理 port/edge legality。
        const double sx = rectCx(sr);
        const double sy = rectCy(sr);
        const double tx = rectCx(tr);
        const double ty = rectCy(tr);

        auto finalize = [&](Step0PatternCandidate c) {
            c.wireLength = candidateWireLength(c);
            c.blockagePenalty = blockagePenaltyForCandidate(design, conn, c, opt);
            c.bendCount = bendCountForCandidateStep0(c);
            c.isOuterDetour = patternNameIsOuterDetourStep0(c.name);
            c.isPredictablePattern = (c.bendCount <= 2 && !c.isOuterDetour && c.blockagePenalty <= STEP0_EPS);
            if (!c.segs.empty()) out.push_back(c);
            };

        // STRAIGHT：source / target 大致同水平或同垂直時才加入。
        if (fabs(sy - ty) <= STEP0_EPS || fabs(sx - tx) <= STEP0_EPS) {
            Step0PatternCandidate c;
            c.name = "STRAIGHT";
            addSegment(c.segs, sx, sy, tx, ty);
            finalize(c);
        }

        // L-HV：先 Horizontal，再 Vertical。
        // S(sx,sy) -> (tx,sy) -> T(tx,ty)
        {
            Step0PatternCandidate c;
            c.name = "L-HV";
            addSegment(c.segs, sx, sy, tx, sy);
            addSegment(c.segs, tx, sy, tx, ty);
            finalize(c);
        }

        // L-VH：先 Vertical，再 Horizontal。
        // S(sx,sy) -> (sx,ty) -> T(tx,ty)
        {
            Step0PatternCandidate c;
            c.name = "L-VH";
            addSegment(c.segs, sx, sy, sx, ty);
            addSegment(c.segs, sx, ty, tx, ty);
            finalize(c);
        }

        if (opt.enableZPattern) {
            // V6.3：Z-shaped pattern 不是只考慮一種方向。
            //
            // 1) Detour-free Z：trunk 放在 source/target 的中線。
            //    這會在 bbox 內產生一般 Z-HVH / Z-VHV。
            // 2) Directional Z：若 enableDirectionalZPattern=true，額外加入
            //    bbox 外側 left/right/top/bottom 的 guide trunk。
            //    這四個候選代表「向左繞、向右繞、向下繞、向上繞」四種 detour 方向。
            //
            // 注意：實際線段方向，例如 left-to-right / right-to-left / up / down，
            // 由 source/target 座標自動決定，不需要再另外列舉象限。
            const double xmin = min(sx, tx);
            const double xmax = max(sx, tx);
            const double ymin = min(sy, ty);
            const double ymax = max(sy, ty);
            const double spanX = max(1.0, xmax - xmin);
            const double spanY = max(1.0, ymax - ymin);
            const double marginX = max(opt.zOuterMarginMin, opt.zOuterMarginRatio * spanX);
            const double marginY = max(opt.zOuterMarginMin, opt.zOuterMarginRatio * spanY);

            auto addZHVH = [&](const string& name, double mx) {
                mx = max(0.0, min(design.outlineW, mx));
                Step0PatternCandidate c;
                c.name = name;
                addSegment(c.segs, sx, sy, mx, sy);
                addSegment(c.segs, mx, sy, mx, ty);
                addSegment(c.segs, mx, ty, tx, ty);
                finalize(c);
                };

            auto addZVHV = [&](const string& name, double my) {
                my = max(0.0, min(design.outlineH, my));
                Step0PatternCandidate c;
                c.name = name;
                addSegment(c.segs, sx, sy, sx, my);
                addSegment(c.segs, sx, my, tx, my);
                addSegment(c.segs, tx, my, tx, ty);
                finalize(c);
                };

            addZHVH("Z-HVH-MID", 0.5 * (sx + tx));
            addZVHV("Z-VHV-MID", 0.5 * (sy + ty));

            bool allowDirectionalZ = opt.enableDirectionalZPattern;
            if (allowDirectionalZ && opt.conditionalDirectionalZPattern) {
                const double bboxAreaRatio = (spanX * spanY) / max(1.0, design.outlineW * design.outlineH);
                const bool highDemand = conn.netCount >= opt.conditionalOuterZNetThreshold;
                const bool largeBBox = bboxAreaRatio >= opt.conditionalOuterZBboxAreaRatio;
                const bool blockageOverlap = hardEdgeBlockOverlapCountForConnStep0(design, conn) > 0;
                allowDirectionalZ = highDemand || largeBBox || blockageOverlap;
            }
            if (allowDirectionalZ) {
                addZHVH("Z-HVH-LEFT", xmin - marginX);
                addZHVH("Z-HVH-RIGHT", xmax + marginX);
                addZVHV("Z-VHV-BOTTOM", ymin - marginY);
                addZVHV("Z-VHV-TOP", ymax + marginY);
            }
        }

        // 去掉完全重複或退化候選，避免相同幾何被重複分配造成 bias。
        vector<Step0PatternCandidate> uniq;
        set<string> seen;
        for (auto& c : out) {
            ostringstream key;
            for (const auto& s : c.segs) {
                key << llround(s.x1 * 1000.0) << "," << llround(s.y1 * 1000.0) << ":"
                    << llround(s.x2 * 1000.0) << "," << llround(s.y2 * 1000.0) << ";";
            }
            const string k = key.str();
            if (seen.insert(k).second) uniq.push_back(c);
        }

        return uniq;
    }

    static void addCandidateDemand(
        const Design& design,
        const Step0PatternCandidate& cand,
        double demand,
        vector<Step0ChannelPressure>& pressure,
        const CongestionMapBuilder::Options& opt
    ) {
        if (fabs(demand) <= EPS) return;

        for (const auto& seg : cand.segs) {
            if (!isHorizontal(seg) && !isVertical(seg)) continue;

            for (int ci = 0; ci < static_cast<int>(design.channels.size()); ++ci) {
                const Rect& r = design.channels[ci].rect;

                if (isHorizontal(seg)) {
                    if (horizontalLineCrossesRect(seg, r, opt.projectionTolerance)) {
                        pressure[ci].predLR += demand;
                    }
                }
                else if (isVertical(seg)) {
                    if (verticalLineCrossesRect(seg, r, opt.projectionTolerance)) {
                        pressure[ci].predTB += demand;
                    }
                }
            }
        }
    }

    static double evaluateCandidateCost(
        const Design& design,
        const Connection& conn,
        const Step0PatternCandidate& cand,
        const vector<Step0ChannelPressure>& pressure,
        const CongestionMapBuilder::Options& opt
    ) {
        const double nets = static_cast<double>(max(0, conn.netCount));
        double congCost = 0.0;
        double histCost = 0.0;
        double peakUtil = 0.0;
        double narrowCost = 0.0;
        int touchedComponents = 0;

        auto consumeComponent = [&](int ci, bool lr) {
            if (ci < 0 || ci >= static_cast<int>(pressure.size())) return;
            const Step0ChannelPressure& p = pressure[ci];
            const double cap = lr ? p.capLR : p.capTB;
            const double cur = lr ? p.predLR : p.predTB;
            const double hist = lr ? p.histLR : p.histTB;
            const double projected = cur + nets;
            const double util = projected / safeCap(cap);

            congCost += nets * nearFullPenalty(util);
            histCost += nets * hist * nearFullPenalty(util);
            peakUtil = max(peakUtil, util);
            ++touchedComponents;

            if (cap < opt.narrowComponentCap) {
                const double shortage = (opt.narrowComponentCap - cap) / max(1.0, opt.narrowComponentCap);
                narrowCost += nets * shortage * shortage * (1.0 + util * util);
            }
            };

        for (const auto& seg : cand.segs) {
            for (int ci = 0; ci < static_cast<int>(design.channels.size()); ++ci) {
                const Rect& r = design.channels[ci].rect;
                if (isHorizontal(seg) && horizontalLineCrossesRect(seg, r, opt.projectionTolerance)) {
                    consumeComponent(ci, true);
                }
                else if (isVertical(seg) && verticalLineCrossesRect(seg, r, opt.projectionTolerance)) {
                    consumeComponent(ci, false);
                }
            }
        }

        // 若 pattern 完全沒有投影到 channel，代表這個 guide 對目前 actual channels 幫助很小，
        // 給一個較大懲罰，避免它因 congestion=0 被誤選。
        const double noProjectionPenalty = touchedComponents == 0 ? nets * 1000.0 : 0.0;

        return opt.wWire * cand.wireLength * nets
            + opt.wCongestion * congCost
            + opt.wHistory * histCost
            + opt.wPeak * peakUtil * nets
            + opt.wNarrow * narrowCost
            + cand.blockagePenalty
            + noProjectionPenalty;
    }

    static void finalizeUtilAndHistory(vector<Step0ChannelPressure>& pressure) {
        for (auto& p : pressure) {
            p.utilLR = p.predLR / safeCap(p.capLR);
            p.utilTB = p.predTB / safeCap(p.capTB);

            // 簡化 NTHU-style history：反覆 near-full / overflow 的方向會更貴。
            if (p.utilLR > 1.00) p.histLR += 1.0;
            else if (p.utilLR > 0.85) p.histLR += 0.5;

            if (p.utilTB > 1.00) p.histTB += 1.0;
            else if (p.utilTB > 0.85) p.histTB += 0.5;
        }
    }

    static string colorForUtil(double util) {
        if (util < 0.50) return "#b7e4a1";   // green
        if (util < 0.75) return "#fff3a3";   // yellow
        if (util < 1.00) return "#fdae61";   // orange
        if (util < 1.25) return "#f46d43";   // red orange
        if (util < 1.50) return "#d73027";   // red
        return "#7b3294";                    // purple
    }

    static string fmt(double v) {
        // 視覺化/CSV 中常會出現 -0.000，這只是浮點誤差，容易誤導 debug。
        // 這裡只在輸出字串時清成 0，不改任何內部計算。
        if (fabs(v) < 5.0e-7) v = 0.0;
        ostringstream oss;
        oss << fixed << setprecision(3) << v;
        return oss.str();
    }

    // SVG 是 XML 格式，文字中的 <、>、&、"、' 必須跳脫，
    // 否則瀏覽器會把例如 legend 的 "<0.50" 誤認成 XML tag，
    // 出現 StartTag: invalid element name。
    static string xmlEscape(const string& s) {
        string out;
        out.reserve(s.size());
        for (char c : s) {
            switch (c) {
            case '&': out += "&amp;"; break;
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            case '\"': out += "&quot;"; break;
            case '\'': out += "&apos;"; break;
            default: out.push_back(c); break;
            }
        }
        return out;
    }

    static string csvEscapeStep0(const string& s) {
        bool needQuote = false;
        for (char c : s) {
            if (c == ',' || c == '"' || c == '\n' || c == '\r') {
                needQuote = true;
                break;
            }
        }
        if (!needQuote) return s;
        string out = "\"";
        for (char c : s) {
            if (c == '"') out += "\"\"";
            else out.push_back(c);
        }
        out += "\"";
        return out;
    }

    static double clamp01Step0(double v) {
        if (v < 0.0) return 0.0;
        if (v > 1.0) return 1.0;
        return v;
    }


    // SVG region 類型分類：只影響視覺化，不影響 CMB 數值與 Router guide。
    // 目的：避免所有 region 都用紅色外框，導致使用者無法分辨它是 pattern congestion、
    // RISA supply deficit、RUDY background demand、hybrid demand 或 criticality 主導。
    static string regionVisualTypeStep0(const Step0CongestedRegion& r) {
        const double patternPeak = r.peakUtil;
        const double risaPeak = r.peakRisaUtil;
        const double rudyPeak = r.peakRudyUtil;
        const double hybridPeak = r.peakHybridUtil;
        const double critPeak = r.peakCriticality;

        // V7.5 diagnostic refinement:
        // 有些 region 的 peakUtil 很高，其實是極窄 channel / channel pinch 造成的，
        // 若仍標成 PATTERN_CONGESTION，使用者會以為是一般 pattern demand 熱區。
        // 這裡用 region 幾何與 peak/avg 差距辨識這種「瓶頸型」區域。
        const double minDim = max(1.0, min(r.bbox.w, r.bbox.h));
        const double maxDim = max(r.bbox.w, r.bbox.h);
        const double aspect = maxDim / minDim;
        const bool oneHotChannelDominates = (r.avgUtil > EPS && patternPeak > 3.0 * r.avgUtil && patternPeak >= 1.0);
        if (patternPeak >= 1.0 && (minDim < 10.0 || aspect > 30.0 || oneHotChannelDominates)) {
            return "CHANNEL_PINCH";
        }

        // 以「相對主因」分類，而不是單純誰的絕對值最大。
        // RISA / RUDY / HYB 是用來幫助判斷供給不足、背景需求或融合需求，
        // 不應被 patternPeak 永遠壓過。
        const double m = max(1.0e-9, max(max(patternPeak, risaPeak), max(rudyPeak, hybridPeak)));
        if (risaPeak >= 0.85 && risaPeak >= 0.85 * m && r.totalRisaCost > 0.0) return "RISA_SUPPLY_DEFICIT";
        if (rudyPeak >= 0.75 && rudyPeak >= 0.55 * m && r.totalRudyDemand > 0.0) return "RUDY_BACKGROUND";
        if (hybridPeak >= 0.85 && hybridPeak >= 0.80 * m && r.totalHybridCost > 0.0) return "HYBRID_DEMAND";
        if (patternPeak >= 1.0) return "PATTERN_CONGESTION";
        if (critPeak >= 0.35) return "CRITICALITY_HISTORY";
        return "MIXED_REGION";
    }

    static string regionFillColorStep0(const string& type) {
        // V7.5 interactive visual refinement:
        // 使用較深的熱區色塊，讓 hotspot 在主圖上明顯可辨；hover 時會由 CSS 自動淡化，露出底下 block/channel。
        if (type == "CHANNEL_PINCH") return "#e6550d";          // deep amber / orange-red
        if (type == "PATTERN_CONGESTION") return "#de2d26";    // deep red
        if (type == "RISA_SUPPLY_DEFICIT") return "#f16913";   // deep orange
        if (type == "RUDY_BACKGROUND") return "#2171b5";       // deep blue
        if (type == "HYBRID_DEMAND") return "#6a51a3";         // deep purple
        if (type == "CRITICALITY_HISTORY") return "#ae017e";   // deep magenta
        return "#636363";                                      // mixed gray
    }

    static string regionStrokeColorStep0(const string& type) {
        if (type == "CHANNEL_PINCH") return "#e6550d";
        if (type == "PATTERN_CONGESTION") return "#de2d26";
        if (type == "RISA_SUPPLY_DEFICIT") return "#f16913";
        if (type == "RUDY_BACKGROUND") return "#2171b5";
        if (type == "HYBRID_DEMAND") return "#6a51a3";
        if (type == "CRITICALITY_HISTORY") return "#ae017e";
        return "#636363";
    }

    static string regionShortLabelStep0(const string& type) {
        if (type == "CHANNEL_PINCH") return "PINCH";
        if (type == "PATTERN_CONGESTION") return "PAT";
        if (type == "RISA_SUPPLY_DEFICIT") return "RISA";
        if (type == "RUDY_BACKGROUND") return "RUDY";
        if (type == "HYBRID_DEMAND") return "HYB";
        if (type == "CRITICALITY_HISTORY") return "CRIT";
        return "MIX";
    }



    // -----------------------------------------------------------------------------
    // V6.1 helper：per-connection guide / pattern weighting
    // -----------------------------------------------------------------------------
    static bool patternIsLShapeStep0(const string& name) {
        return name == "L-HV" || name == "L-VH";
    }

    static bool patternIsZShapeStep0(const string& name) {
        return name.rfind("Z-", 0) == 0;
    }

    static double baseWeightForPatternStep0(
        const Step0PatternCandidate& cand,
        const CongestionMapBuilder::Options& opt
    ) {
        if (!opt.useAdaptivePatternWeights) return 1.0;
        if (cand.name == "STRAIGHT") return max(0.0, opt.straightWeight);
        if (patternIsLShapeStep0(cand.name)) return max(0.0, opt.totalLShapeWeight) * 0.5;
        if (patternIsZShapeStep0(cand.name)) return max(0.0, opt.totalZShapeWeight) * 0.5;
        return 1.0;
    }

    static vector<double> initialCandidateWeightsStep0(
        vector<Step0PatternCandidate>& candidates,
        const CongestionMapBuilder::Options& opt
    ) {
        vector<double> w(candidates.size(), 0.0);

        // V6.3：Z candidates 可能從原本 2 條增加到 6 條。
        // 因此不能讓每條 Z 都拿 totalZShapeWeight * 0.5，否則 Z 會被過度放大。
        // 這裡先統計各類候選數量，再把 totalLShapeWeight / totalZShapeWeight
        // 平均分配給各自類別內的候選。
        int nStraight = 0, nL = 0, nZ = 0, nOther = 0;
        for (const auto& c : candidates) {
            if (c.name == "STRAIGHT") ++nStraight;
            else if (patternIsLShapeStep0(c.name)) ++nL;
            else if (patternIsZShapeStep0(c.name)) ++nZ;
            else ++nOther;
        }

        double sum = 0.0;
        for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
            const auto& c = candidates[i];
            double wi = 1.0;
            if (!opt.useAdaptivePatternWeights) {
                wi = 1.0;
            }
            else if (c.name == "STRAIGHT") {
                wi = max(0.0, opt.straightWeight) / static_cast<double>(max(1, nStraight));
            }
            else if (patternIsLShapeStep0(c.name)) {
                wi = max(0.0, opt.totalLShapeWeight) / static_cast<double>(max(1, nL));
            }
            else if (patternIsZShapeStep0(c.name)) {
                wi = max(0.0, opt.totalZShapeWeight) / static_cast<double>(max(1, nZ));
            }
            else {
                wi = 1.0 / static_cast<double>(max(1, nOther));
            }
            candidates[i].baseWeight = wi;
            w[i] = wi;
            sum += wi;
        }

        if (sum <= STEP0_EPS) {
            const double eq = candidates.empty() ? 0.0 : 1.0 / static_cast<double>(candidates.size());
            fill(w.begin(), w.end(), eq);
        }
        else {
            for (double& x : w) x /= sum;
        }
        return w;
    }

    static vector<int> uniqueChannelsTouchedByCandidateStep0(
        const Design& design,
        const Step0PatternCandidate& cand,
        const CongestionMapBuilder::Options& opt
    ) {
        set<int> ids;
        for (const auto& seg : cand.segs) {
            for (int ci = 0; ci < static_cast<int>(design.channels.size()); ++ci) {
                const Rect& r = design.channels[ci].rect;
                if ((isHorizontal(seg) && horizontalLineCrossesRect(seg, r, opt.projectionTolerance)) ||
                    (isVertical(seg) && verticalLineCrossesRect(seg, r, opt.projectionTolerance))) {
                    ids.insert(ci);
                }
            }
        }
        return vector<int>(ids.begin(), ids.end());
    }

    struct CandidateUtilStatsStep0 {
        vector<int> channels;
        double maxUtil = 0.0;
        double avgUtil = 0.0;
        int touchedComponents = 0;
    };

    static CandidateUtilStatsStep0 candidateUtilStatsStep0(
        const Design& design,
        const Step0PatternCandidate& cand,
        const vector<Step0ChannelPressure>& pressure,
        const CongestionMapBuilder::Options& opt
    ) {
        CandidateUtilStatsStep0 st;
        set<int> channels;
        double sumUtil = 0.0;
        for (const auto& seg : cand.segs) {
            for (int ci = 0; ci < static_cast<int>(design.channels.size()) && ci < static_cast<int>(pressure.size()); ++ci) {
                const Rect& r = design.channels[ci].rect;
                if (isHorizontal(seg) && horizontalLineCrossesRect(seg, r, opt.projectionTolerance)) {
                    const double u = pressure[ci].utilLR;
                    st.maxUtil = max(st.maxUtil, u);
                    sumUtil += u;
                    ++st.touchedComponents;
                    channels.insert(ci);
                }
                else if (isVertical(seg) && verticalLineCrossesRect(seg, r, opt.projectionTolerance)) {
                    const double u = pressure[ci].utilTB;
                    st.maxUtil = max(st.maxUtil, u);
                    sumUtil += u;
                    ++st.touchedComponents;
                    channels.insert(ci);
                }
            }
        }
        st.channels.assign(channels.begin(), channels.end());
        st.avgUtil = st.touchedComponents > 0 ? sumUtil / static_cast<double>(st.touchedComponents) : 0.0;
        return st;
    }

    static vector<int> trimChannelListStep0(vector<int> ids, int limit) {
        if (limit <= 0 || static_cast<int>(ids.size()) <= limit) return ids;
        ids.resize(limit);
        return ids;
    }

    static string joinChannelNamesStep0(const Design& design, const vector<int>& ids) {
        ostringstream oss;
        for (int i = 0; i < static_cast<int>(ids.size()); ++i) {
            if (i) oss << '|';
            const int ci = ids[i];
            if (ci >= 0 && ci < static_cast<int>(design.channels.size())) oss << design.channels[ci].name;
            else oss << "CH?";
        }
        return oss.str();
    }

    static void fillConnectionGuideChannelsStep0(
        const Design& design,
        const vector<ConnState>& states,
        Step0CongestionMapResult& result,
        const CongestionMapBuilder::Options& opt
    ) {
        for (int k = 0; k < static_cast<int>(result.connectionGuide.size()) && k < static_cast<int>(states.size()); ++k) {
            Step0ConnectionGuide& g = result.connectionGuide[k];
            const ConnState& st = states[k];

            if (st.selectedCandidate >= 0 && st.selectedCandidate < static_cast<int>(st.candidates.size())) {
                const Step0PatternCandidate& best = st.candidates[st.selectedCandidate];
                CandidateUtilStatsStep0 bs = candidateUtilStatsStep0(design, best, result.channelPressure, opt);
                g.preferredChannelIndices = trimChannelListStep0(bs.channels, opt.maxPreferredChannels);
                g.preferredChannelsCSV = joinChannelNamesStep0(design, g.preferredChannelIndices);
                g.bestPatternAvgUtil = bs.avgUtil;
                g.bestPatternMaxUtil = max(g.bestPatternMaxUtil, bs.maxUtil);
            }

            set<int> avoid;
            int healthy = 0;
            for (const auto& cand : st.candidates) {
                CandidateUtilStatsStep0 cs = candidateUtilStatsStep0(design, cand, result.channelPressure, opt);
                if (cs.touchedComponents > 0 && cs.maxUtil <= opt.preferredHealthyUtilThreshold && cand.blockagePenalty <= STEP0_EPS) {
                    ++healthy;
                }
                for (int ci : cs.channels) {
                    if (ci < 0 || ci >= static_cast<int>(result.channelPressure.size())) continue;
                    const auto& p = result.channelPressure[ci];
                    const double maxUtil = max(p.utilLR, p.utilTB);
                    if (maxUtil >= opt.avoidUtilThreshold || max(p.criticality, p.routerCriticality) >= opt.avoidCriticalityThreshold) {
                        avoid.insert(ci);
                    }
                }
            }

            // preferred channel 不列入 avoid，避免 Router guide 語意衝突。
            for (int ci : g.preferredChannelIndices) avoid.erase(ci);
            g.avoidChannelIndices.assign(avoid.begin(), avoid.end());
            g.avoidChannelIndices = trimChannelListStep0(g.avoidChannelIndices, opt.maxAvoidChannels);
            g.avoidChannelsCSV = joinChannelNamesStep0(design, g.avoidChannelIndices);

            if (!st.candidates.empty()) {
                g.healthyCandidateCount = max(g.healthyCandidateCount, healthy);
                g.lackOfAlternative = 1.0 / static_cast<double>(1 + max(0, g.healthyCandidateCount));
                g.guideConfidence = static_cast<double>(g.healthyCandidateCount) / static_cast<double>(st.candidates.size());
            }
        }
    }

    static void refineConnectionPriorityStep0(Step0CongestionMapResult& result) {
        for (auto& g : result.connectionGuide) {
            const double nets = static_cast<double>(max(0, g.netCount));
            const double utilTerm = g.hasProjectedPattern
                ? min(3.0, max(0.0, g.bestPatternMaxUtil))
                : 0.0;
            const double noProjectionTerm = (!g.hasProjectedPattern && g.totalCandidateCount > 0) ? 0.30 : 0.0;
            const double avgTerm = std::isfinite(g.bestPatternAvgUtil) ? min(2.0, max(0.0, g.bestPatternAvgUtil)) : 0.0;
            const double alternativeTerm = min(2.0, max(0.0, g.lackOfAlternative));
            const double openTerm = min(2.0, max(0.0, g.openRiskScore));
            const double avoidTerm = min(1.5, static_cast<double>(g.avoidChannelIndices.size()) / 8.0);
            const double endpointTerm = (g.endpointAccessRisk ? 1.2 : 0.0) + (g.endpointBottleneckRisk ? 0.8 : 0.0);
            const double sharedTerm = g.sharedBottleneckRisk ? min(1.0, 0.5 + g.sharedBottleneckScore) : 0.0;
            const double unpredictableTerm = min(1.0, max(0.0, 1.0 - g.patternPredictabilityScore));
            const double couplingTerm = min(1.2, max(0.0, g.couplingRiskScore));

            // V7.4 routePriority：
            //   Pattern-routing 論文提醒我們：只有一部分 connection 適合被 pattern 強烈約束。
            //   因此 routePriority 同時考慮「不容易被 pattern 預測」與 coupling-like risk，
            //   讓 Router 優先處理這些需要保留彈性的困難 connection。
            g.routePriority = nets * (1.0 + utilTerm + 0.5 * avgTerm + 0.8 * alternativeTerm
                + openTerm + 0.4 * avoidTerm + endpointTerm + sharedTerm
                + 0.6 * unpredictableTerm + 0.7 * couplingTerm + noProjectionTerm);
        }
    }

    static double clampPatternStep0(double v, double lo, double hi) {
        return max(lo, min(hi, v));
    }

    static double selectedPatternTypeScoreStep0(const Step0PatternCandidate& cand) {
        if (cand.name == "STRAIGHT") return 1.00;
        if (patternIsLShapeStep0(cand.name)) return 0.90;
        if (cand.name == "Z-HVH-MID" || cand.name == "Z-VHV-MID") return 0.72;
        if (patternIsZShapeStep0(cand.name)) return 0.48;
        return 0.55;
    }

    static double bboxAreaRatioForConnStep0(const Design& design, const Connection& conn) {
        if (conn.src < 0 || conn.src >= static_cast<int>(design.blocks.size()) ||
            conn.dst < 0 || conn.dst >= static_cast<int>(design.blocks.size())) return 1.0;
        const Rect& a = design.blocks[conn.src].rect;
        const Rect& b = design.blocks[conn.dst].rect;
        const double spanX = fabs(rectCx(a) - rectCx(b));
        const double spanY = fabs(rectCy(a) - rectCy(b));
        return (max(1.0, spanX) * max(1.0, spanY)) / max(1.0, design.outlineW * design.outlineH);
    }

    static double bboxPredictabilityScoreStep0(double ratio, const CongestionMapBuilder::Options& opt) {
        if (ratio <= opt.patternSmallBboxAreaRatio) return 1.0;
        if (ratio >= opt.patternLargeBboxAreaRatio) return 0.25;
        const double t = (ratio - opt.patternSmallBboxAreaRatio) /
            max(1.0e-9, opt.patternLargeBboxAreaRatio - opt.patternSmallBboxAreaRatio);
        return 1.0 - 0.75 * clampPatternStep0(t, 0.0, 1.0);
    }

    static double couplingRiskForCandidateStep0(
        const Design& design,
        const Step0PatternCandidate& cand,
        const vector<Step0ChannelPressure>& pressure,
        const CongestionMapBuilder::Options& opt
    ) {
        double maxRisk = 0.0;
        double accum = 0.0;
        int n = 0;
        for (const auto& seg : cand.segs) {
            for (int ci = 0; ci < static_cast<int>(design.channels.size()) && ci < static_cast<int>(pressure.size()); ++ci) {
                const Rect& r = design.channels[ci].rect;
                const auto& p = pressure[ci];
                double util = -1.0;
                if (isHorizontal(seg) && horizontalLineCrossesRect(seg, r, opt.projectionTolerance)) {
                    util = (p.predLR + p.ambientDemandLR) / safeCap(p.capLR);
                }
                else if (isVertical(seg) && verticalLineCrossesRect(seg, r, opt.projectionTolerance)) {
                    util = (p.predTB + p.ambientDemandTB) / safeCap(p.capTB);
                }
                if (util < -0.5) continue;
                const double risk = max(0.0, util - opt.patternCouplingRiskThreshold) /
                    max(1.0e-9, 1.0 - opt.patternCouplingRiskThreshold);
                maxRisk = max(maxRisk, risk);
                accum += risk;
                ++n;
            }
        }
        const double avg = n > 0 ? accum / static_cast<double>(n) : 0.0;
        return clampPatternStep0(0.70 * maxRisk + 0.30 * avg, 0.0, 2.0);
    }

    static void computePatternPredictabilityStep0(
        const Design& design,
        const vector<ConnState>& states,
        Step0CongestionMapResult& result,
        const CongestionMapBuilder::Options& opt
    ) {
        if (!opt.enablePatternPredictabilityRefinement) return;

        for (int k = 0; k < static_cast<int>(result.connectionGuide.size()) && k < static_cast<int>(states.size()) && k < static_cast<int>(design.connections.size()); ++k) {
            Step0ConnectionGuide& g = result.connectionGuide[k];
            const ConnState& st = states[k];
            const Connection& conn = design.connections[k];
            if (st.candidates.empty() || st.selectedCandidate < 0 || st.selectedCandidate >= static_cast<int>(st.candidates.size())) {
                g.patternPredictabilityScore = 0.0;
                g.patternPredictabilityClass = "ROUTER_FREE";
                g.patternGuideMode = "ROUTER_FREE";
                g.patternReason = "NO_SELECTED_PATTERN";
                continue;
            }

            const Step0PatternCandidate& best = st.candidates[st.selectedCandidate];
            const double typeScore = selectedPatternTypeScoreStep0(best);
            const double bboxRatio = bboxAreaRatioForConnStep0(design, conn);
            const double bboxScore = bboxPredictabilityScoreStep0(bboxRatio, opt);
            const double healthyScore = st.candidates.empty() ? 0.0 :
                clampPatternStep0(static_cast<double>(max(0, g.healthyCandidateCount)) / static_cast<double>(st.candidates.size()), 0.0, 1.0);

            double secondBest = numeric_limits<double>::infinity();
            double bestCost = numeric_limits<double>::infinity();
            for (int i = 0; i < static_cast<int>(st.candidates.size()); ++i) {
                const double c = evaluateCandidateCost(design, conn, st.candidates[i], result.channelPressure, opt);
                if (c < bestCost) { secondBest = bestCost; bestCost = c; }
                else if (c < secondBest) secondBest = c;
            }
            double separationScore = 0.55;
            if (isfinite(bestCost) && isfinite(secondBest)) {
                const double sep = max(0.0, secondBest - bestCost);
                separationScore = clampPatternStep0(sep / (fabs(bestCost) * opt.patternCostSeparationScale + 1.0), 0.0, 1.0);
            }

            const double blockageScore = best.blockagePenalty <= STEP0_EPS ? 1.0 : 0.35;
            const double netPenalty = clampPatternStep0(static_cast<double>(max(0, conn.netCount)) / max(1.0, opt.patternHighNetCountThreshold), 0.0, 2.0);
            const double netScore = clampPatternStep0(1.0 - 0.30 * netPenalty, 0.35, 1.0);
            const double openScore = clampPatternStep0(1.0 - 0.55 * min(1.0, max(0.0, g.openRiskScore)), 0.0, 1.0);

            g.couplingRiskScore = couplingRiskForCandidateStep0(design, best, result.channelPressure, opt);
            const double couplingScore = clampPatternStep0(1.0 - 0.50 * min(1.0, g.couplingRiskScore), 0.0, 1.0);

            double score = 0.22 * typeScore + 0.18 * bboxScore + 0.16 * healthyScore +
                0.16 * separationScore + 0.12 * blockageScore + 0.08 * netScore +
                0.08 * openScore;
            score *= couplingScore;
            score = clampPatternStep0(score, 0.0, 1.0);

            g.patternPredictabilityScore = score;
            g.patternGuideStrength = clampPatternStep0(score * (1.0 - 0.35 * min(1.0, g.couplingRiskScore)), 0.0, opt.patternGuideMaxStrength);
            // 原本 guideConfidence 只看 healthyCandidateCount，現在改成 pattern-aware confidence。
            g.guideConfidence = clampPatternStep0(0.5 * g.guideConfidence + 0.5 * g.patternPredictabilityScore, 0.0, 1.0);

            if (score >= opt.patternHighPredictabilityThreshold && g.couplingRiskScore < opt.patternCouplingRiskThreshold) {
                g.patternPredictabilityClass = "HIGH";
                g.patternGuideMode = "STRONG_SOFT";
            }
            else if (score <= opt.patternLowPredictabilityThreshold || g.couplingRiskScore >= 1.0) {
                g.patternPredictabilityClass = "LOW";
                g.patternGuideMode = "ROUTER_FREE";
                g.patternGuideStrength = min(g.patternGuideStrength, 0.20);
            }
            else {
                g.patternPredictabilityClass = "MEDIUM";
                g.patternGuideMode = "NORMAL_SOFT";
            }

            ostringstream reason;
            reason << "type=" << best.name
                << ";bboxRatio=" << fmt(bboxRatio)
                << ";typeScore=" << fmt(typeScore)
                << ";bboxScore=" << fmt(bboxScore)
                << ";healthyScore=" << fmt(healthyScore)
                << ";costSep=" << fmt(separationScore)
                << ";couplingRisk=" << fmt(g.couplingRiskScore);
            if (best.isOuterDetour) reason << ";outerDetour";
            if (best.blockagePenalty > STEP0_EPS) reason << ";blockagePenalty";
            g.patternReason = reason.str();
        }
    }

    // -----------------------------------------------------------------------------
    // V5 helper：lightweight contact graph / route-open risk analysis
    // -----------------------------------------------------------------------------
    static bool rectTouchesForStep0(const Rect& a, const Rect& b, double eps = 1.0e-3) {
        const bool aRightBLeft = fabs(rectRight(a) - b.x) <= eps &&
            overlapLen(a.y, rectTop(a), b.y, rectTop(b)) > STEP0_EPS;
        const bool aLeftBRight = fabs(a.x - rectRight(b)) <= eps &&
            overlapLen(a.y, rectTop(a), b.y, rectTop(b)) > STEP0_EPS;
        const bool aTopBBottom = fabs(rectTop(a) - b.y) <= eps &&
            overlapLen(a.x, rectRight(a), b.x, rectRight(b)) > STEP0_EPS;
        const bool aBottomBTop = fabs(a.y - rectTop(b)) <= eps &&
            overlapLen(a.x, rectRight(a), b.x, rectRight(b)) > STEP0_EPS;
        return aRightBLeft || aLeftBRight || aTopBBottom || aBottomBTop;
    }

    static int nodeCountForStep0(const Design& design) {
        return static_cast<int>(design.blocks.size() + design.channels.size());
    }

    static bool nodeIsBlockForStep0(const Design& design, int node) {
        return node >= 0 && node < static_cast<int>(design.blocks.size());
    }

    static bool nodeIsChannelForStep0(const Design& design, int node) {
        const int nb = static_cast<int>(design.blocks.size());
        return node >= nb && node < nb + static_cast<int>(design.channels.size());
    }

    static bool nodeIsSoftBlockForStep0(const Design& design, int node) {
        return nodeIsBlockForStep0(design, node) && design.blocks[node].spec.type == BlockType::SOFT;
    }

    static const Rect& nodeRectForStep0(const Design& design, int node) {
        const int nb = static_cast<int>(design.blocks.size());
        if (node < nb) return design.blocks[node].rect;
        return design.channels[node - nb].rect;
    }

    static vector<vector<int>> buildContactAdjForStep0(const Design& design) {
        const int n = nodeCountForStep0(design);
        vector<vector<int>> adj(n);
        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                if (rectTouchesForStep0(nodeRectForStep0(design, i), nodeRectForStep0(design, j))) {
                    adj[i].push_back(j);
                    adj[j].push_back(i);
                }
            }
        }
        return adj;
    }

    static bool allowedNodeForModeStep0(const Design& design, int node, int src, int dst, bool ftEnabled) {
        if (node == src || node == dst) return true;
        if (nodeIsChannelForStep0(design, node)) return true;
        if (ftEnabled && nodeIsSoftBlockForStep0(design, node)) return true;
        return false;
    }

    static bool connectedInModeStep0(const Design& design, const vector<vector<int>>& adj, int src, int dst, bool ftEnabled) {
        const int n = nodeCountForStep0(design);
        if (src < 0 || src >= n || dst < 0 || dst >= n) return false;

        vector<char> vis(n, 0);
        queue<int> q;
        vis[src] = 1;
        q.push(src);

        while (!q.empty()) {
            const int u = q.front();
            q.pop();
            if (u == dst) return true;
            for (int v : adj[u]) {
                if (vis[v]) continue;
                if (!allowedNodeForModeStep0(design, v, src, dst, ftEnabled)) continue;
                vis[v] = 1;
                q.push(v);
            }
        }
        return false;
    }

    static pair<double, int> candidateMaxUtilAndTouchedStep0(
        const Design& design,
        const Step0PatternCandidate& cand,
        const vector<Step0ChannelPressure>& pressure,
        const CongestionMapBuilder::Options& opt
    ) {
        double mx = 0.0;
        int touched = 0;
        for (const auto& seg : cand.segs) {
            for (int ci = 0; ci < static_cast<int>(design.channels.size()) && ci < static_cast<int>(pressure.size()); ++ci) {
                const Rect& r = design.channels[ci].rect;
                if (isHorizontal(seg) && horizontalLineCrossesRect(seg, r, opt.projectionTolerance)) {
                    mx = max(mx, pressure[ci].utilLR);
                    ++touched;
                }
                else if (isVertical(seg) && verticalLineCrossesRect(seg, r, opt.projectionTolerance)) {
                    mx = max(mx, pressure[ci].utilTB);
                    ++touched;
                }
            }
        }
        return { mx, touched };
    }



    // -----------------------------------------------------------------------------
    // V6.2 helper：endpoint access / shared bottleneck risk
    // -----------------------------------------------------------------------------
    static double channelMaxUtilStep0(const Step0ChannelPressure& p) {
        // V7.3：risk analysis 使用 pattern util、RISA util 與 ambient util 的綜合視角。
        // utilLR/TB 是原始 pattern demand；risaUtilLR/TB 反映 effective supply；
        // ambientUtilLR/TB 則是 amplified congestion guide 對 Router 的 background pressure。
        return max(max(p.utilLR, p.utilTB), max(max(p.risaUtilLR, p.risaUtilTB), max(p.ambientUtilLR, p.ambientUtilTB)));
    }

    static bool channelIsHighRiskForEndpointStep0(
        const Step0ChannelPressure& p,
        const CongestionMapBuilder::Options& opt
    ) {
        return channelMaxUtilStep0(p) >= opt.endpointAccessUtilThreshold ||
            max(p.criticality, p.routerCriticality) >= opt.endpointAccessCriticalityThreshold;
    }

    static bool channelIsSharedBottleneckStep0(
        const Step0ChannelPressure& p,
        const CongestionMapBuilder::Options& opt
    ) {
        return channelMaxUtilStep0(p) >= opt.sharedBottleneckUtilThreshold ||
            max(p.criticality, p.routerCriticality) >= opt.sharedBottleneckCriticalityThreshold;
    }

    struct EndpointAccessStatsStep0 {
        int accessCount = 0;
        int healthyCount = 0;
        double maxUtil = 0.0;
        vector<int> accessChannels;
    };

    static EndpointAccessStatsStep0 endpointAccessStatsStep0(
        const Design& design,
        const vector<vector<int>>& adj,
        int blockNode,
        const vector<Step0ChannelPressure>& pressure,
        const CongestionMapBuilder::Options& opt
    ) {
        EndpointAccessStatsStep0 st;
        if (blockNode < 0 || blockNode >= static_cast<int>(adj.size())) return st;

        const int nb = static_cast<int>(design.blocks.size());
        set<int> seen;
        for (int v : adj[blockNode]) {
            if (!nodeIsChannelForStep0(design, v)) continue;
            const int ci = v - nb;
            if (ci < 0 || ci >= static_cast<int>(design.channels.size()) || ci >= static_cast<int>(pressure.size())) continue;
            if (!seen.insert(ci).second) continue;

            const Step0ChannelPressure& p = pressure[ci];
            const double u = channelMaxUtilStep0(p);
            st.accessChannels.push_back(ci);
            st.maxUtil = max(st.maxUtil, u);
            ++st.accessCount;

            // access channel 若沒有接近滿載，且 criticality 不高，就視為健康出口。
            if (!channelIsHighRiskForEndpointStep0(p, opt)) {
                ++st.healthyCount;
            }
        }
        return st;
    }

    static set<int> highRiskChannelsInCandidateStep0(
        const Design& design,
        const Step0PatternCandidate& cand,
        const vector<Step0ChannelPressure>& pressure,
        const CongestionMapBuilder::Options& opt
    ) {
        set<int> out;
        vector<int> touched = uniqueChannelsTouchedByCandidateStep0(design, cand, opt);
        for (int ci : touched) {
            if (ci < 0 || ci >= static_cast<int>(pressure.size())) continue;
            if (channelIsSharedBottleneckStep0(pressure[ci], opt)) out.insert(ci);
        }
        return out;
    }

    static set<int> intersectSetsStep0(const set<int>& a, const set<int>& b) {
        set<int> out;
        std::set_intersection(a.begin(), a.end(), b.begin(), b.end(), inserter(out, out.begin()));
        return out;
    }

    static void fillSharedBottleneckRiskStep0(
        const Design& design,
        const ConnState& st,
        Step0ConnectionGuide& g,
        const Step0CongestionMapResult& result,
        const CongestionMapBuilder::Options& opt
    ) {
        bool initialized = false;
        set<int> common;
        int usableCandidateCount = 0;
        double commonScore = 0.0;

        for (const auto& cand : st.candidates) {
            vector<int> touched = uniqueChannelsTouchedByCandidateStep0(design, cand, opt);
            if (touched.empty()) continue;
            ++usableCandidateCount;

            set<int> risk = highRiskChannelsInCandidateStep0(design, cand, result.channelPressure, opt);
            if (!initialized) {
                common = risk;
                initialized = true;
            }
            else {
                common = intersectSetsStep0(common, risk);
            }
        }

        // 至少要有兩條以上可比較的 candidate，shared bottleneck 才有意義。
        if (usableCandidateCount >= 2 && !common.empty()) {
            vector<int> commonVec(common.begin(), common.end());
            commonVec = trimChannelListStep0(commonVec, opt.maxCommonBottleneckChannels);
            for (int ci : commonVec) {
                if (ci < 0 || ci >= static_cast<int>(result.channelPressure.size())) continue;
                const Step0ChannelPressure& p = result.channelPressure[ci];
                commonScore = max(commonScore, max(channelMaxUtilStep0(p), max(p.criticality, p.routerCriticality)));
            }
            g.sharedBottleneckRisk = true;
            g.commonBottleneckChannelIndices = commonVec;
            g.commonBottleneckChannelsCSV = joinChannelNamesStep0(design, g.commonBottleneckChannelIndices);
            g.sharedBottleneckScore = commonScore;
        }
    }

    static void fillConnectionOpenRiskStep0(
        const Design& design,
        const vector<ConnState>& states,
        Step0CongestionMapResult& result,
        const CongestionMapBuilder::Options& opt
    ) {
        const vector<vector<int>> adj = buildContactAdjForStep0(design);

        for (int k = 0; k < static_cast<int>(design.connections.size()) && k < static_cast<int>(result.connectionGuide.size()); ++k) {
            const Connection& conn = design.connections[k];
            Step0ConnectionGuide& g = result.connectionGuide[k];

            if (conn.src < 0 || conn.dst < 0 ||
                conn.src >= static_cast<int>(design.blocks.size()) ||
                conn.dst >= static_cast<int>(design.blocks.size())) {
                g.geometryOpenRisk = true;
                g.capacityOpenRisk = true;
                g.openRiskScore = 1.0;
                g.openRiskType = "INVALID_ENDPOINT";
                continue;
            }

            g.channelOnlyConnected = connectedInModeStep0(design, adj, conn.src, conn.dst, false);
            g.ftEnabledConnected = connectedInModeStep0(design, adj, conn.src, conn.dst, true);
            g.geometryOpenRisk = !g.ftEnabledConnected;

            // V6.2：endpoint access risk。
            // route open 很常不是中間 corridor 問題，而是 source/destination 周圍
            // 沒有可進入 channel，或可進入 channel 全都已經接近滿載。
            EndpointAccessStatsStep0 srcAcc = endpointAccessStatsStep0(design, adj, conn.src, result.channelPressure, opt);
            EndpointAccessStatsStep0 dstAcc = endpointAccessStatsStep0(design, adj, conn.dst, result.channelPressure, opt);
            g.srcAccessChannelCount = srcAcc.accessCount;
            g.dstAccessChannelCount = dstAcc.accessCount;
            g.srcHealthyAccessChannelCount = srcAcc.healthyCount;
            g.dstHealthyAccessChannelCount = dstAcc.healthyCount;
            g.srcAccessMaxUtil = srcAcc.maxUtil;
            g.dstAccessMaxUtil = dstAcc.maxUtil;
            g.endpointAccessRisk = (srcAcc.accessCount == 0 || dstAcc.accessCount == 0);
            g.endpointBottleneckRisk = (!g.endpointAccessRisk && (srcAcc.healthyCount == 0 || dstAcc.healthyCount == 0));

            // V6.2：shared bottleneck risk。
            // 若所有可用 pattern 都被迫經過同一個高風險 channel，代表替代路徑不足，
            // 後續 Dijkstra 即使有路，也容易因容量或 cost policy 產生 open / overflow。
            if (k < static_cast<int>(states.size())) {
                fillSharedBottleneckRiskStep0(design, states[k], g, result, opt);
            }

            g.totalCandidateCount = 0;
            g.healthyCandidateCount = 0;
            g.bestPatternMaxUtil = 0.0;
            g.hasProjectedPattern = false;
            g.lackOfAlternative = 1.0;
            double bestProjectedUtil = numeric_limits<double>::infinity();

            if (k < static_cast<int>(states.size())) {
                const ConnState& st = states[k];
                g.totalCandidateCount = static_cast<int>(st.candidates.size());
                for (const auto& cand : st.candidates) {
                    auto mt = candidateMaxUtilAndTouchedStep0(design, cand, result.channelPressure, opt);
                    const double mx = mt.first;
                    const int touched = mt.second;
                    if (touched <= 0) continue;
                    g.hasProjectedPattern = true;
                    bestProjectedUtil = min(bestProjectedUtil, mx);

                    const double hardLikePenalty = static_cast<double>(max(1, conn.netCount)) * opt.wHardBlockage * 0.5;
                    if (mx <= opt.openRiskUtilThreshold && cand.blockagePenalty <= hardLikePenalty) {
                        ++g.healthyCandidateCount;
                    }
                }
            }

            if (g.hasProjectedPattern && std::isfinite(bestProjectedUtil)) {
                g.bestPatternMaxUtil = max(0.0, bestProjectedUtil);
            }
            g.lackOfAlternative = 1.0 / static_cast<double>(1 + max(0, g.healthyCandidateCount));
            const double utilForRisk = g.hasProjectedPattern ? max(0.0, g.bestPatternMaxUtil) : 0.0;
            g.capacityOpenRisk = (g.hasProjectedPattern && g.healthyCandidateCount == 0 && utilForRisk >= opt.openRiskUtilThreshold);

            if (g.geometryOpenRisk) {
                g.openRiskType = "GEOMETRY_DISCONNECTED";
                g.openRiskScore = 1.0;
            }
            else if (g.endpointAccessRisk) {
                g.openRiskType = "ENDPOINT_ACCESS_RISK";
                g.openRiskScore = 0.92;
            }
            else if (!g.channelOnlyConnected && g.ftEnabledConnected) {
                g.openRiskType = "FT_REQUIRED";
                g.openRiskScore = 0.65;
            }
            else if (g.endpointBottleneckRisk) {
                g.openRiskType = "ENDPOINT_BOTTLENECK_RISK";
                const double endpointScore = min(1.0, max(g.srcAccessMaxUtil, g.dstAccessMaxUtil) / max(0.1, opt.endpointAccessUtilThreshold));
                g.openRiskScore = max(0.72, endpointScore);
            }
            else if (g.totalCandidateCount <= 0) {
                g.openRiskType = "NO_PATTERN";
                g.openRiskScore = 0.85;
            }
            else if (g.sharedBottleneckRisk) {
                g.openRiskType = "SHARED_BOTTLENECK_RISK";
                g.openRiskScore = max(0.70, min(1.0, g.sharedBottleneckScore));
            }
            else if (g.capacityOpenRisk) {
                g.openRiskType = "CAPACITY_RISK";
                const double utilScore = min(1.0, utilForRisk / max(0.1, opt.openRiskUtilThreshold));
                g.openRiskScore = max(0.70, utilScore);
            }
            else {
                g.openRiskType = "OK";
                const double utilScore = min(0.45, utilForRisk / max(0.1, opt.openRiskUtilThreshold) * 0.35);
                const double endpointTerm = 0.05 * min(1.0, max(g.srcAccessMaxUtil, g.dstAccessMaxUtil));
                g.openRiskScore = max(0.0, utilScore + 0.10 * g.lackOfAlternative + endpointTerm);
            }

            const double noProjectionPriorityTerm = (!g.hasProjectedPattern && g.totalCandidateCount > 0) ? 0.35 : 0.0;
            g.routePriority = static_cast<double>(conn.netCount) *
                (1.0 + utilForRisk
                    + 0.5 * g.openRiskScore
                    + g.lackOfAlternative
                    + (g.endpointBottleneckRisk ? 0.8 : 0.0)
                    + (g.sharedBottleneckRisk ? 0.7 : 0.0)
                    + noProjectionPriorityTerm);
        }
    }


    // -----------------------------------------------------------------------------
    // V6.3 helper：congested region clustering
    // -----------------------------------------------------------------------------
    static double channelRegionScoreUtilStep0(const Step0ChannelPressure& p) {
        return max(max(max(p.utilLR, p.utilTB), max(p.risaUtilLR, p.risaUtilTB)),
            max(max(p.rudyUtilLR, p.rudyUtilTB), max(p.hybridUtilLR, p.hybridUtilTB)));
    }

    static bool channelIsRegionSeedStep0(
        const Step0ChannelPressure& p,
        const CongestionMapBuilder::Options& opt
    ) {
        return channelRegionScoreUtilStep0(p) >= opt.regionUtilThreshold ||
            max(p.criticality, p.routerCriticality) >= opt.regionCriticalityThreshold;
    }

    static Rect unionRectStep0(const Rect& a, const Rect& b) {
        const double x1 = min(a.x, b.x);
        const double y1 = min(a.y, b.y);
        const double x2 = max(rectRight(a), rectRight(b));
        const double y2 = max(rectTop(a), rectTop(b));
        return Rect{ x1, y1, max(0.0, x2 - x1), max(0.0, y2 - y1) };
    }

    static bool channelsAdjacentForRegionStep0(const Channel& a, const Channel& b, double tol) {
        // 兩個 channel 若幾何接觸，或中間只差很小浮點誤差，就視為同一個可擴張區域。
        return rectTouchesForStep0(a.rect, b.rect, tol);
    }

    static vector<Step0CongestedRegion> buildCongestedRegionsStep0(
        const Design& design,
        Step0CongestionMapResult& result,
        const CongestionMapBuilder::Options& opt
    ) {
        vector<Step0CongestedRegion> regions;
        for (auto& p : result.channelPressure) p.regionId = -1;
        if (!opt.enableRegionClustering) return regions;

        const int n = min(static_cast<int>(design.channels.size()), static_cast<int>(result.channelPressure.size()));
        vector<char> seed(n, 0), vis(n, 0);
        for (int i = 0; i < n; ++i) {
            seed[i] = channelIsRegionSeedStep0(result.channelPressure[i], opt) ? 1 : 0;
        }

        for (int start = 0; start < n; ++start) {
            if (!seed[start] || vis[start]) continue;

            vector<int> comp;
            queue<int> q;
            q.push(start);
            vis[start] = 1;

            while (!q.empty()) {
                const int u = q.front();
                q.pop();
                comp.push_back(u);

                for (int v = 0; v < n; ++v) {
                    if (!seed[v] || vis[v]) continue;
                    if (!channelsAdjacentForRegionStep0(design.channels[u], design.channels[v], opt.regionAdjacencyTolerance)) continue;
                    vis[v] = 1;
                    q.push(v);
                }
            }

            if (comp.empty()) continue;

            Step0CongestedRegion r;
            r.channelIndices = comp;
            r.bbox = design.channels[comp.front()].rect;
            double sumUtil = 0.0;
            double sumCrit = 0.0;
            double overflowRisk = 0.0;
            for (int ci : comp) {
                const auto& p = result.channelPressure[ci];
                const double u = channelRegionScoreUtilStep0(p);
                r.bbox = unionRectStep0(r.bbox, design.channels[ci].rect);
                r.peakUtil = max(r.peakUtil, u);
                r.peakCriticality = max(r.peakCriticality, max(p.criticality, p.routerCriticality));
                r.totalPredLR += p.predLR;
                r.totalPredTB += p.predTB;
                sumUtil += u;
                sumCrit += max(p.criticality, p.routerCriticality);
                overflowRisk += max(0.0, p.utilLR - 1.0) + max(0.0, p.utilTB - 1.0);
                r.peakRisaUtil = max(r.peakRisaUtil, max(p.risaUtilLR, p.risaUtilTB));
                r.totalRisaOverflowRisk += p.risaOverflowLR + p.risaOverflowTB;
                r.totalRisaCost += p.risaCost;
                r.peakRudyUtil = max(r.peakRudyUtil, max(p.rudyUtilLR, p.rudyUtilTB));
                r.peakHybridUtil = max(r.peakHybridUtil, max(p.hybridUtilLR, p.hybridUtilTB));
                r.totalRudyDemand += p.rudyDemandLR + p.rudyDemandTB;
                r.totalHybridDemand += p.hybridDemandLR + p.hybridDemandTB;
                r.totalHybridCost += p.hybridCost;
            }
            r.avgUtil = sumUtil / static_cast<double>(comp.size());
            r.avgCriticality = sumCrit / static_cast<double>(comp.size());
            r.totalOverflowRisk = overflowRisk;
            r.regionScore = r.peakUtil + 0.7 * r.peakCriticality + 0.25 * r.avgUtil
                + 0.20 * log(1.0 + static_cast<double>(comp.size()))
                + 0.50 * r.totalOverflowRisk
                + 0.45 * r.peakRisaUtil
                + 0.35 * r.totalRisaCost
                + opt.rudyRegionScoreWeight * r.peakHybridUtil
                + 0.25 * opt.rudyRegionScoreWeight * r.totalHybridCost;
            r.channelsCSV = joinChannelNamesStep0(design, comp);
            regions.push_back(r);
        }

        sort(regions.begin(), regions.end(), [](const Step0CongestedRegion& a, const Step0CongestedRegion& b) {
            if (fabs(a.regionScore - b.regionScore) > 1e-12) return a.regionScore > b.regionScore;
            if (fabs(a.peakUtil - b.peakUtil) > 1e-12) return a.peakUtil > b.peakUtil;
            return a.channelIndices.size() > b.channelIndices.size();
            });

        for (int rid = 0; rid < static_cast<int>(regions.size()); ++rid) {
            regions[rid].regionId = rid;
            for (int ci : regions[rid].channelIndices) {
                if (ci >= 0 && ci < static_cast<int>(result.channelPressure.size())) {
                    result.channelPressure[ci].regionId = rid;
                }
            }
        }
        return regions;
    }

    static string joinRegionIdsStep0(const set<int>& ids) {
        ostringstream oss;
        bool first = true;
        for (int rid : ids) {
            if (rid < 0) continue;
            if (!first) oss << '|';
            first = false;
            oss << 'R' << rid;
        }
        return oss.str();
    }

    static string regionsFromChannelsStep0(const vector<int>& channels, const Step0CongestionMapResult& result) {
        set<int> ids;
        for (int ci : channels) {
            if (ci < 0 || ci >= static_cast<int>(result.channelPressure.size())) continue;
            if (result.channelPressure[ci].regionId >= 0) ids.insert(result.channelPressure[ci].regionId);
        }
        return joinRegionIdsStep0(ids);
    }

    static void fillConnectionRegionLinksStep0(Step0CongestionMapResult& result) {
        for (auto& g : result.connectionGuide) {
            g.preferredRegionsCSV = regionsFromChannelsStep0(g.preferredChannelIndices, result);
            g.avoidRegionsCSV = regionsFromChannelsStep0(g.avoidChannelIndices, result);
            g.commonBottleneckRegionsCSV = regionsFromChannelsStep0(g.commonBottleneckChannelIndices, result);
        }
    }


    // -----------------------------------------------------------------------------
    // V7.1 helper：CRISP-style floorplan feedback
    // -----------------------------------------------------------------------------
    // CRISP 的核心精神不是讓 congestion estimator 取代 placement，而是：
    //   1) 先量測 routing congestion 與 pin-density-like hotspot。
    //   2) 將問題區域轉成局部 spreading / inflation 建議。
    //   3) 優先動 congested region，並限制每輪改動量，避免破壞整體 placement。
    // 在 Problem E 中沒有 standard-cell pin map，因此這裡用 block endpoint pressure
    // 近似 pin-density：某 block 的 incident netCount 很高、但周圍健康 channel 很少，
    // 就代表它是 CRISP-style pin-density / access hotspot。
    static double clampStep0(double v, double lo, double hi) {
        if (hi < lo) return lo;
        return max(lo, min(v, hi));
    }

    static double rectAreaStep0(const Rect& r) {
        return max(0.0, r.w) * max(0.0, r.h);
    }

    static Rect expandRectClippedStep0(const Rect& r, double pad, double W, double H) {
        const double x1 = clampStep0(r.x - pad, 0.0, W);
        const double y1 = clampStep0(r.y - pad, 0.0, H);
        const double x2 = clampStep0(rectRight(r) + pad, 0.0, W);
        const double y2 = clampStep0(rectTop(r) + pad, 0.0, H);
        return Rect{ x1, y1, max(0.0, x2 - x1), max(0.0, y2 - y1) };
    }

    static bool rectIntersectsStep0(const Rect& a, const Rect& b) {
        return overlapLen(a.x, rectRight(a), b.x, rectRight(b)) > STEP0_EPS &&
            overlapLen(a.y, rectTop(a), b.y, rectTop(b)) > STEP0_EPS;
    }

    static string joinBlockNamesNearRectStep0(const Design& design, const Rect& box, int limit = 16) {
        ostringstream oss;
        int count = 0;
        for (int i = 0; i < static_cast<int>(design.blocks.size()); ++i) {
            if (!rectIntersectsStep0(design.blocks[i].rect, box)) continue;
            if (count > 0) oss << '|';
            oss << design.blocks[i].spec.name;
            ++count;
            if (count >= limit) break;
        }
        return oss.str();
    }

    static bool channelListContainsAnyStep0(const vector<int>& a, const vector<int>& b) {
        if (a.empty() || b.empty()) return false;
        set<int> s(a.begin(), a.end());
        for (int x : b) if (s.count(x)) return true;
        return false;
    }

    static string relatedConnectionsForRegionStep0(
        const Design& design,
        const Step0CongestionMapResult& result,
        const Step0CongestedRegion& region,
        int limit = 20
    ) {
        ostringstream oss;
        int count = 0;
        for (const auto& g : result.connectionGuide) {
            bool hit = channelListContainsAnyStep0(g.preferredChannelIndices, region.channelIndices) ||
                channelListContainsAnyStep0(g.avoidChannelIndices, region.channelIndices) ||
                channelListContainsAnyStep0(g.commonBottleneckChannelIndices, region.channelIndices);
            if (!hit) continue;
            string srcName = (g.src >= 0 && g.src < static_cast<int>(design.blocks.size())) ? design.blocks[g.src].spec.name : "?";
            string dstName = (g.dst >= 0 && g.dst < static_cast<int>(design.blocks.size())) ? design.blocks[g.dst].spec.name : "?";
            if (count > 0) oss << '|';
            oss << srcName << "->" << dstName << "#" << g.netCount;
            ++count;
            if (count >= limit) break;
        }
        return oss.str();
    }

    static string relatedConnectionsForBlockStep0(
        const Design& design,
        const Step0CongestionMapResult& result,
        int blockIndex,
        int limit = 20
    ) {
        ostringstream oss;
        int count = 0;
        for (const auto& g : result.connectionGuide) {
            if (g.src != blockIndex && g.dst != blockIndex) continue;
            string srcName = (g.src >= 0 && g.src < static_cast<int>(design.blocks.size())) ? design.blocks[g.src].spec.name : "?";
            string dstName = (g.dst >= 0 && g.dst < static_cast<int>(design.blocks.size())) ? design.blocks[g.dst].spec.name : "?";
            if (count > 0) oss << '|';
            oss << srcName << "->" << dstName << "#" << g.netCount;
            ++count;
            if (count >= limit) break;
        }
        return oss.str();
    }

    static string dominantRegionDirectionStep0(const Step0CongestedRegion& r) {
        if (r.totalPredLR > r.totalPredTB * 1.15) return "LR";
        if (r.totalPredTB > r.totalPredLR * 1.15) return "TB";
        return "MIXED";
    }

    static double suggestedGapIncreaseForRegionStep0(
        const Step0CongestedRegion& r,
        const CongestionMapBuilder::Options& opt
    ) {
        const double target = max(0.10, opt.crispTargetMaxUtil);
        const double excess = max(0.0, r.peakUtil - target) / target;
        const double baseDim = dominantRegionDirectionStep0(r) == "LR" ? max(1.0, r.bbox.h) : max(1.0, r.bbox.w);
        const double raw = max(opt.actionMinGapIncrease, baseDim * (0.15 + 0.35 * excess));
        return clampStep0(raw, opt.actionMinGapIncrease, opt.actionMaxGapIncrease);
    }

    static vector<double> blockIncidentNetsStep0(const Design& design) {
        vector<double> incident(design.blocks.size(), 0.0);
        for (const auto& c : design.connections) {
            const double nets = static_cast<double>(max(0, c.netCount));
            if (c.src >= 0 && c.src < static_cast<int>(incident.size())) incident[c.src] += nets;
            if (c.dst >= 0 && c.dst < static_cast<int>(incident.size())) incident[c.dst] += nets;
        }
        return incident;
    }

    static vector<double> blockEndpointPressureStep0(const Design& design) {
        vector<double> incident = blockIncidentNetsStep0(design);
        vector<double> pressure(design.blocks.size(), 0.0);
        for (int i = 0; i < static_cast<int>(design.blocks.size()); ++i) {
            const Rect& r = design.blocks[i].rect;
            const double perimeter = max(1.0, 2.0 * (max(0.0, r.w) + max(0.0, r.h)));
            // 用 block 周邊可容納 routing ports 的能力近似 pin-density supply。
            // 0.25 是保守因子，避免把所有邊界都當成完全可用 routing entrance。
            const double boundarySupply = max(1.0, perimeter * CHANNEL_DENSITY * 0.25);
            pressure[i] = incident[i] / boundarySupply;
        }
        return pressure;
    }


    static string blockTypeNameLocalStep0(BlockType t) {
        switch (t) {
        case BlockType::SOFT: return "SOFT";
        case BlockType::HARD: return "HARD";
        case BlockType::EDGE: return "EDGE";
        default: return "UNKNOWN";
        }
    }

    static string edgeNameStep0(int edge) {
        if (edge == 1) return "LEFT";
        if (edge == 2) return "TOP";
        if (edge == 3) return "RIGHT";
        if (edge == 4) return "BOTTOM";
        return "UNKNOWN";
    }

    static string reserveActionForEdgeStep0(int edge) {
        if (edge == 1) return "RESERVE_LEFT_CHANNEL";
        if (edge == 2) return "RESERVE_TOP_CHANNEL";
        if (edge == 3) return "RESERVE_RIGHT_CHANNEL";
        if (edge == 4) return "RESERVE_BOTTOM_CHANNEL";
        return "RESERVE_ACCESS_CHANNEL";
    }

    static string moveDirectionForEdgeStep0(int edge) {
        // 為了在該 edge 外側保留 channel，對 block 自身的保守建議是往反方向移。
        // 實際 Floorplanner 也可以選擇移動鄰近 block，而不是移動此 block。
        if (edge == 1) return "MOVE_RIGHT";
        if (edge == 2) return "MOVE_DOWN";
        if (edge == 3) return "MOVE_LEFT";
        if (edge == 4) return "MOVE_UP";
        return "KEEP_AWAY";
    }

    static double blockEdgeLengthStep0(const Rect& r, int edge) {
        if (edge == 1 || edge == 3) return max(0.0, r.h);
        if (edge == 2 || edge == 4) return max(0.0, r.w);
        return 0.0;
    }

    static bool channelTouchesBlockEdgeStep0(const Rect& block, const Rect& ch, int edge, double eps = 1.0e-3) {
        if (edge == 1) { // block left, channel right
            return fabs(ch.x + ch.w - block.x) <= eps && overlapLen(block.y, rectTop(block), ch.y, rectTop(ch)) > STEP0_EPS;
        }
        if (edge == 3) { // block right, channel left
            return fabs(rectRight(block) - ch.x) <= eps && overlapLen(block.y, rectTop(block), ch.y, rectTop(ch)) > STEP0_EPS;
        }
        if (edge == 2) { // block top, channel bottom
            return fabs(rectTop(block) - ch.y) <= eps && overlapLen(block.x, rectRight(block), ch.x, rectRight(ch)) > STEP0_EPS;
        }
        if (edge == 4) { // block bottom, channel top
            return fabs(ch.y + ch.h - block.y) <= eps && overlapLen(block.x, rectRight(block), ch.x, rectRight(ch)) > STEP0_EPS;
        }
        return false;
    }

    static int likelyBlockExitEdgeForConnectionStep0(const Design& design, int blockIdx, int otherIdx) {
        if (blockIdx < 0 || blockIdx >= static_cast<int>(design.blocks.size()) ||
            otherIdx < 0 || otherIdx >= static_cast<int>(design.blocks.size())) return 0;

        const Rect& a = design.blocks[blockIdx].rect;
        const Rect& b = design.blocks[otherIdx].rect;
        const double dx = rectCx(b) - rectCx(a);
        const double dy = rectCy(b) - rectCy(a);

        // 只作 floorplan feedback，不作 routing legality：用主方向決定端點壓力落在哪側。
        if (fabs(dx) >= fabs(dy)) return dx >= 0.0 ? 3 : 1;
        return dy >= 0.0 ? 2 : 4;
    }

    static vector<double> blockEdgeIncidentDemandStep0(const Design& design) {
        vector<double> demand(design.blocks.size() * 5, 0.0);
        for (const auto& c : design.connections) {
            const double nets = static_cast<double>(max(0, c.netCount));
            if (nets <= 0.0) continue;
            if (c.src >= 0 && c.src < static_cast<int>(design.blocks.size()) &&
                c.dst >= 0 && c.dst < static_cast<int>(design.blocks.size())) {
                const int es = likelyBlockExitEdgeForConnectionStep0(design, c.src, c.dst);
                const int ed = likelyBlockExitEdgeForConnectionStep0(design, c.dst, c.src);
                if (es >= 1 && es <= 4) demand[c.src * 5 + es] += nets;
                if (ed >= 1 && ed <= 4) demand[c.dst * 5 + ed] += nets;
            }
        }
        return demand;
    }



    // -----------------------------------------------------------------------------
    // V7.2-RISA helper：explicit supply-demand / blockage-aware modeling
    // -----------------------------------------------------------------------------
    // RISA 的核心是：routability 不是單純 demand，而是 D(需求) 與 S(供給) 在
    // horizontal / vertical 方向上的 balance。Problem E 已經有 actual channels，
    // 因此這裡不再建立 uniform NxN grid，而是把每個 actual channel 當成供給單元。
    // 另外，RISA 的 mega-cell NBB partitioning 精神被轉譯為：若 connection bbox
    // 覆蓋 hard/edge block，則在該 blockage 周圍的 channels 加上 detour demand，
    // 並對 connection guide 標記 NBB_PARTITION_HINT。
    static double risaSafeRatioStep0(double num, double den) {
        return num / max(1.0, den);
    }

    static Rect centerBBoxForConnectionStep0(const Design& design, const Connection& conn, double padRatio) {
        if (conn.src < 0 || conn.src >= static_cast<int>(design.blocks.size()) ||
            conn.dst < 0 || conn.dst >= static_cast<int>(design.blocks.size())) return Rect{};
        const Rect& a = design.blocks[conn.src].rect;
        const Rect& b = design.blocks[conn.dst].rect;
        double x1 = min(rectCx(a), rectCx(b));
        double x2 = max(rectCx(a), rectCx(b));
        double y1 = min(rectCy(a), rectCy(b));
        double y2 = max(rectCy(a), rectCy(b));
        const double pad = max(1.0, padRatio * max(max(1.0, x2 - x1), max(1.0, y2 - y1)));
        x1 = clampStep0(x1 - pad, 0.0, design.outlineW);
        y1 = clampStep0(y1 - pad, 0.0, design.outlineH);
        x2 = clampStep0(x2 + pad, 0.0, design.outlineW);
        y2 = clampStep0(y2 + pad, 0.0, design.outlineH);
        return Rect{ x1, y1, max(0.0, x2 - x1), max(0.0, y2 - y1) };
    }

    static bool risaBlockageTypeStep0(BlockType t) {
        return t == BlockType::HARD || t == BlockType::EDGE;
    }

    static bool channelIntersectsExpandedBoxStep0(const Channel& ch, const Rect& box) {
        return rectIntersectsStep0(ch.rect, box);
    }

    static double risaAdjacencyDiscountForBlockStep0(BlockType t, const CongestionMapBuilder::Options& opt) {
        if (t == BlockType::HARD) return max(0.0, opt.risaHardAdjacencySupplyDiscount);
        if (t == BlockType::EDGE) return max(0.0, opt.risaEdgeAdjacencySupplyDiscount);
        if (t == BlockType::SOFT) return max(0.0, opt.risaSoftAdjacencySupplyDiscount);
        return 0.0;
    }

    static void appendUniqueTokenStep0(string& s, const string& tok) {
        if (tok.empty()) return;
        if (!s.empty()) {
            // 避免重複 token 讓 CSV 太長。
            size_t start = 0;
            while (start <= s.size()) {
                size_t pos = s.find('|', start);
                string cur = s.substr(start, pos == string::npos ? string::npos : pos - start);
                if (cur == tok) return;
                if (pos == string::npos) break;
                start = pos + 1;
            }
            s += '|';
        }
        s += tok;
    }

    static void computeRisaSupplyDemandStep0(
        const Design& design,
        const vector<ConnState>& states,
        Step0CongestionMapResult& result,
        const CongestionMapBuilder::Options& opt
    ) {
        if (result.channelPressure.empty()) return;

        // 1) Supply modeling：actual channel 的幾何容量是 base supply；
        //    hard/edge block 造成的 pinch/access 風險會降低 effective supply。
        for (int ci = 0; ci < static_cast<int>(design.channels.size()) && ci < static_cast<int>(result.channelPressure.size()); ++ci) {
            const Channel& ch = design.channels[ci];
            Step0ChannelPressure& p = result.channelPressure[ci];
            p.effectiveCapLR = p.capLR;
            p.effectiveCapTB = p.capTB;
            p.supplyReductionLR = 0.0;
            p.supplyReductionTB = 0.0;
            p.supplyScaleLR = 1.0;
            p.supplyScaleTB = 1.0;
            p.risaExtraDemandLR = 0.0;
            p.risaExtraDemandTB = 0.0;

            double lrDiscountScore = 0.0;
            double tbDiscountScore = 0.0;
            for (const auto& b : design.blocks) {
                const double discount = risaAdjacencyDiscountForBlockStep0(b.spec.type, opt);
                if (discount <= STEP0_EPS) continue;

                // top/bottom contact 對 LR component 較敏感，因為 LR 需要 channel 高度。
                if (channelTouchesBlockEdgeStep0(b.rect, ch.rect, 2) || channelTouchesBlockEdgeStep0(b.rect, ch.rect, 4)) {
                    const double ov = overlapLen(ch.rect.x, rectRight(ch.rect), b.rect.x, rectRight(b.rect));
                    const double ratio = risaSafeRatioStep0(ov, max(1.0, ch.rect.w));
                    lrDiscountScore += discount * ratio;
                }
                // left/right contact 對 TB component 較敏感，因為 TB 需要 channel 寬度。
                if (channelTouchesBlockEdgeStep0(b.rect, ch.rect, 1) || channelTouchesBlockEdgeStep0(b.rect, ch.rect, 3)) {
                    const double ov = overlapLen(ch.rect.y, rectTop(ch.rect), b.rect.y, rectTop(b.rect));
                    const double ratio = risaSafeRatioStep0(ov, max(1.0, ch.rect.h));
                    tbDiscountScore += discount * ratio;
                }
            }

            const double minScale = clampStep0(opt.risaMinEffectiveCapScale, 0.05, 1.0);
            const double scaleLR = clampStep0(1.0 - lrDiscountScore, minScale, 1.0);
            const double scaleTB = clampStep0(1.0 - tbDiscountScore, minScale, 1.0);
            p.effectiveCapLR = max(1.0, p.capLR * scaleLR);
            p.effectiveCapTB = max(1.0, p.capTB * scaleTB);
            p.supplyReductionLR = max(0.0, p.capLR - p.effectiveCapLR);
            p.supplyReductionTB = max(0.0, p.capTB - p.effectiveCapTB);
            p.supplyScaleLR = scaleLR;
            p.supplyScaleTB = scaleTB;
        }

        // 2) Demand modeling：RISA 的 NBB partitioning 對 mega cell overlap 特別重要。
        //    Problem E 中 hard/edge block 不能 feedthrough，等價於 zero-porosity blockage。
        //    若 connection bbox 穿過 hard/edge block，對 blockage 周邊 channels 加 detour demand。
        for (int k = 0; k < static_cast<int>(design.connections.size()); ++k) {
            const Connection& conn = design.connections[k];
            if (conn.src < 0 || conn.dst < 0 || conn.src >= static_cast<int>(design.blocks.size()) || conn.dst >= static_cast<int>(design.blocks.size())) continue;
            Rect bbox = centerBBoxForConnectionStep0(design, conn, opt.risaNbbPaddingRatio);
            if (bbox.w <= STEP0_EPS && bbox.h <= STEP0_EPS) continue;

            Step0ConnectionGuide& guide = result.connectionGuide[k];
            int overlapCount = 0;
            double totalAdded = 0.0;
            string blockedCSV;

            for (int bi = 0; bi < static_cast<int>(design.blocks.size()); ++bi) {
                if (bi == conn.src || bi == conn.dst) continue;
                const BlockInst& b = design.blocks[bi];
                if (!risaBlockageTypeStep0(b.spec.type)) continue;
                if (!rectIntersectsStep0(bbox, b.rect)) continue;

                vector<pair<int, int>> around; // (channel index, touched block edge)
                for (int ci = 0; ci < static_cast<int>(design.channels.size()) && ci < static_cast<int>(result.channelPressure.size()); ++ci) {
                    if (!channelIntersectsExpandedBoxStep0(design.channels[ci], bbox)) continue;
                    for (int edge = 1; edge <= 4; ++edge) {
                        if (channelTouchesBlockEdgeStep0(b.rect, design.channels[ci].rect, edge)) {
                            around.push_back({ ci, edge });
                        }
                    }
                }
                if (around.empty()) continue;

                ++overlapCount;
                appendUniqueTokenStep0(blockedCSV, b.spec.name);
                const double detourTotal = static_cast<double>(max(0, conn.netCount)) * max(0.0, opt.risaDetourDemandRatio);
                const double each = detourTotal / static_cast<double>(max<size_t>(1, around.size()));
                totalAdded += detourTotal;

                for (auto [ci, edge] : around) {
                    if (ci < 0 || ci >= static_cast<int>(result.channelPressure.size())) continue;
                    Step0ChannelPressure& p = result.channelPressure[ci];
                    // hard block 的 left/right 側通常形成 vertical detour corridor，吃 TB；
                    // top/bottom 側形成 horizontal detour corridor，吃 LR。
                    if (edge == 1 || edge == 3) p.risaExtraDemandTB += each;
                    else if (edge == 2 || edge == 4) p.risaExtraDemandLR += each;
                }
            }

            if (overlapCount > 0) {
                guide.risaMegaOverlapCount += overlapCount;
                guide.risaDetourDemandAdded += totalAdded;
                guide.risaBlockedBlocksCSV = blockedCSV;
                guide.risaNbbPartitionHint = "NBB_PARTITION_DETOUR_AROUND_HARD_EDGE_BLOCK";
                // RISA 提醒這種 bbox-over-mega-cell net 的 demand 需要重新分配；
                // 在 route priority 上稍微提高，讓 Router 早處理。
                guide.routePriority *= (1.0 + min(0.35, 0.05 * overlapCount + 0.00005 * totalAdded));
            }
        }

        // 3) Cost modeling：RISA metric = sum Max(D - t*S, 0)^2，方向分開。
        double maxRisaUtil = 0.0;
        for (auto& p : result.channelPressure) {
            p.risaDemandLR = p.predLR + p.risaExtraDemandLR;
            p.risaDemandTB = p.predTB + p.risaExtraDemandTB;
            p.risaUtilLR = p.risaDemandLR / safeCap(p.effectiveCapLR);
            p.risaUtilTB = p.risaDemandTB / safeCap(p.effectiveCapTB);

            const double safeSupplyLR = max(1.0, opt.risaSupplySafetyFactor * p.effectiveCapLR);
            const double safeSupplyTB = max(1.0, opt.risaSupplySafetyFactor * p.effectiveCapTB);
            p.risaOverflowLR = max(0.0, p.risaDemandLR - safeSupplyLR) / safeCap(p.effectiveCapLR);
            p.risaOverflowTB = max(0.0, p.risaDemandTB - safeSupplyTB) / safeCap(p.effectiveCapTB);
            p.risaCostLR = p.risaOverflowLR * p.risaOverflowLR;
            p.risaCostTB = p.risaOverflowTB * p.risaOverflowTB;
            p.risaCost = p.risaCostLR + p.risaCostTB;

            const double risaU = max(p.risaUtilLR, p.risaUtilTB);
            maxRisaUtil = max(maxRisaUtil, risaU);
            const double risaCritRaw = opt.risaCriticalityWeight * nearFullPenalty(risaU) * max(p.histLR, p.histTB);
            p.criticalityRaw += risaCritRaw;
            p.criticality = min(1.0, max(p.criticality, risaCritRaw / max(1.0, nearFullPenalty(1.0))));
        }
        result.maxRisaUtil = max(result.maxRisaUtil, maxRisaUtil);
    }



    // -----------------------------------------------------------------------------
    // V7.5-RUDY helper：router-independent background demand + hybrid demand
    // -----------------------------------------------------------------------------
    // RUDY (Rectangular Uniform wire DensitY) 的核心觀念：
    //   不需要預測單條 net 最後會走哪一條 pattern；只要假設 router 會盡量把線
    //   放在 enclosing rectangle 內，並把 wire density 均勻分布於該 rectangle。
    //
    // 在 Problem E 中，我們不建立額外 uniform bin，而是把 RUDY rectangle 投影到
    // ChannelBuilder 產生的 actual channels：
    //   - rudyDemand*: router-independent background pressure。
    //   - hybridDemand*: patternDemand 與 rudyDemand 的混合，給 floorplan health / region
    //     score 使用，避免完全依賴目前 L/Z pattern candidate。
    //
    // 注意：RUDY 是 background，不是正式路徑。它不產生 preferredChannels，也不直接
    // 封鎖 Router，只用來提升 health / region / criticality 的穩定性。
    static double rectIntersectionAreaStep0(const Rect& a, const Rect& b) {
        const double ox = overlapLen(a.x, rectRight(a), b.x, rectRight(b));
        const double oy = overlapLen(a.y, rectTop(a), b.y, rectTop(b));
        if (ox <= STEP0_EPS || oy <= STEP0_EPS) return 0.0;
        return ox * oy;
    }

    static Rect rudyBBoxForConnectionStep0(
        const Design& design,
        const Connection& conn,
        const CongestionMapBuilder::Options& opt
    ) {
        const Rect& sr = design.blocks[conn.src].rect;
        const Rect& tr = design.blocks[conn.dst].rect;
        double x1 = min(rectCx(sr), rectCx(tr));
        double x2 = max(rectCx(sr), rectCx(tr));
        double y1 = min(rectCy(sr), rectCy(tr));
        double y2 = max(rectCy(sr), rectCy(tr));

        // RUDY 的 net area 不可為 0。若兩端同 row/col，給一個很小但合理的寬/高，
        // 避免密度無限大，同時仍保留 flat net 的背景壓力。
        const double minSize = max(1.0, opt.rudyMinBBoxSize);
        if (x2 - x1 < minSize) {
            const double mid = 0.5 * (x1 + x2);
            x1 = mid - 0.5 * minSize;
            x2 = mid + 0.5 * minSize;
        }
        if (y2 - y1 < minSize) {
            const double mid = 0.5 * (y1 + y2);
            y1 = mid - 0.5 * minSize;
            y2 = mid + 0.5 * minSize;
        }

        x1 = max(0.0, min(design.outlineW, x1));
        x2 = max(0.0, min(design.outlineW, x2));
        y1 = max(0.0, min(design.outlineH, y1));
        y2 = max(0.0, min(design.outlineH, y2));
        if (x2 - x1 < minSize) x2 = min(design.outlineW, x1 + minSize);
        if (y2 - y1 < minSize) y2 = min(design.outlineH, y1 + minSize);
        return Rect{ x1, y1, max(minSize, x2 - x1), max(minSize, y2 - y1) };
    }

    static void computeRudyBackgroundDemandStep0(
        const Design& design,
        Step0CongestionMapResult& result,
        const CongestionMapBuilder::Options& opt
    ) {
        if (!opt.enableRudyBackgroundDemand) return;
        if (result.channelPressure.empty()) return;

        for (auto& p : result.channelPressure) {
            p.rudyDemandLR = p.rudyDemandTB = 0.0;
            p.rudyUtilLR = p.rudyUtilTB = 0.0;
            p.rudyOverlapArea = 0.0;
            p.hybridDemandLR = p.hybridDemandTB = 0.0;
            p.hybridUtilLR = p.hybridUtilTB = 0.0;
            p.hybridOverflowLR = p.hybridOverflowTB = 0.0;
            p.hybridCost = 0.0;
        }

        const double outlineArea = max(1.0, design.outlineW * design.outlineH);

        for (int k = 0; k < static_cast<int>(design.connections.size()) && k < static_cast<int>(result.connectionGuide.size()); ++k) {
            const Connection& conn = design.connections[k];
            if (conn.src < 0 || conn.dst < 0 ||
                conn.src >= static_cast<int>(design.blocks.size()) ||
                conn.dst >= static_cast<int>(design.blocks.size())) continue;

            const Rect& sr = design.blocks[conn.src].rect;
            const Rect& tr = design.blocks[conn.dst].rect;
            const double dx = fabs(rectCx(sr) - rectCx(tr));
            const double dy = fabs(rectCy(sr) - rectCy(tr));
            const double hpwl = max(1.0, dx + dy);
            const double lrShare = dx / hpwl;
            const double tbShare = dy / hpwl;

            const Rect bbox = rudyBBoxForConnectionStep0(design, conn, opt);
            const double bboxArea = max(1.0, rectAreaStep0(bbox));
            const double density = static_cast<double>(max(0, conn.netCount)) * hpwl / bboxArea;

            vector<pair<int, double>> overlaps;
            double totalOverlap = 0.0;
            for (int ci = 0; ci < static_cast<int>(design.channels.size()) && ci < static_cast<int>(result.channelPressure.size()); ++ci) {
                const double a = rectIntersectionAreaStep0(bbox, design.channels[ci].rect);
                if (a <= STEP0_EPS) continue;
                overlaps.push_back({ ci, a });
                totalOverlap += a;
            }
            if (totalOverlap <= STEP0_EPS) continue;

            // RUDY 是 background demand。為避免和 patternDemand 重複計算而過度保守，
            // 使用 rudyDemandScale 壓低強度；hpwl/sqrt(area) 只作為形狀修正，避免長細 bbox 被低估。
            const double shapeFactor = clampStep0(hpwl / max(1.0, sqrt(bboxArea)), 1.0, 4.0);
            const double totalBackground = static_cast<double>(max(0, conn.netCount)) * max(0.0, opt.rudyDemandScale) * shapeFactor;
            double contributed = 0.0;
            vector<int> touched;

            for (auto [ci, a] : overlaps) {
                if (ci < 0 || ci >= static_cast<int>(result.channelPressure.size())) continue;
                const double w = a / totalOverlap;
                Step0ChannelPressure& p = result.channelPressure[ci];
                const double add = totalBackground * w;
                p.rudyDemandLR += add * lrShare;
                p.rudyDemandTB += add * tbShare;
                p.rudyOverlapArea += a;
                contributed += add;
                touched.push_back(ci);
            }

            Step0ConnectionGuide& g = result.connectionGuide[k];
            g.rudyBBoxArea = bboxArea;
            g.rudyHpwl = hpwl;
            g.rudyDensity = density;
            g.rudyContribution = contributed;
            g.rudyTouchedChannelCount = static_cast<int>(touched.size());
            sort(touched.begin(), touched.end());
            touched.erase(unique(touched.begin(), touched.end()), touched.end());
            g.rudyTouchedChannelsCSV = joinChannelNamesStep0(design, trimChannelListStep0(touched, opt.maxPreferredChannels));
        }

        const double wPattern = clampStep0(opt.rudyHybridPatternWeight, 0.0, 1.0);
        double maxHybridUtil = 0.0;
        for (auto& p : result.channelPressure) {
            const double effLR = safeCap(p.effectiveCapLR > STEP0_EPS ? p.effectiveCapLR : p.capLR);
            const double effTB = safeCap(p.effectiveCapTB > STEP0_EPS ? p.effectiveCapTB : p.capTB);

            p.rudyUtilLR = p.rudyDemandLR / effLR;
            p.rudyUtilTB = p.rudyDemandTB / effTB;

            p.hybridDemandLR = wPattern * p.predLR + (1.0 - wPattern) * p.rudyDemandLR + p.risaExtraDemandLR;
            p.hybridDemandTB = wPattern * p.predTB + (1.0 - wPattern) * p.rudyDemandTB + p.risaExtraDemandTB;
            p.hybridUtilLR = p.hybridDemandLR / effLR;
            p.hybridUtilTB = p.hybridDemandTB / effTB;

            const double safeLR = max(1.0, opt.risaSupplySafetyFactor * effLR);
            const double safeTB = max(1.0, opt.risaSupplySafetyFactor * effTB);
            p.hybridOverflowLR = max(0.0, p.hybridDemandLR - safeLR) / effLR;
            p.hybridOverflowTB = max(0.0, p.hybridDemandTB - safeTB) / effTB;
            p.hybridCost = p.hybridOverflowLR * p.hybridOverflowLR + p.hybridOverflowTB * p.hybridOverflowTB;

            const double hu = max(p.hybridUtilLR, p.hybridUtilTB);
            maxHybridUtil = max(maxHybridUtil, hu);
            const double rudyCritRaw = max(0.0, opt.rudyCriticalityWeight) * nearFullPenalty(hu) * max(p.histLR, p.histTB);
            p.criticalityRaw += rudyCritRaw;
            p.criticality = min(1.0, max(p.criticality, rudyCritRaw / max(1.0, nearFullPenalty(1.0))));
        }
        result.maxHybridUtil = max(result.maxHybridUtil, maxHybridUtil);
    }


    // -----------------------------------------------------------------------------
    // V7.3-HM helper：amplified congestion estimate + ambient demand
    // -----------------------------------------------------------------------------
    // Hadsell-Madden 的核心觀念：
    //   1) static / dynamic congestion estimates 可以作為 router cost 的 hint。
    //   2) 但 congestion estimate 不能線性使用；低壓區不應影響 route，避免無謂 detour。
    //   3) 真正高壓區要 amplification，讓 router 明顯遠離。
    //   4) amplified estimate 以 ambient demand 形式加入 route cost：actual demand + ambient demand。
    // 本版 CMB 在正式 Router 之前只能穩定提供 static estimate，因此先實作 static amplified
    // demand；dynamic estimate 預留給未來 Router postmortem / RRR loop 回寫。
    static double hmAmplificationFactorStep0(
        double util,
        const CongestionMapBuilder::Options& opt
    ) {
        if (!std::isfinite(util)) return opt.amplifyHighFactor;

        // V7.5 Channel-metrics refinement:
        // 原本 Hadsell-Madden style amplification 是 hard piecewise：
        //   util < low -> 0, low~high -> 1, high -> 1.2。
        // 這會讓 routerCriticality 在圖上看起來接近 0/1 二分。
        // 這裡改成 smoothstep ramp，保留「低壓區不干擾、高壓區強化」精神，
        // 但讓 Router guide/ambient demand 變成連續訊號，避免過度保守。
        const double low = opt.amplifyLowUtilCutoff;
        const double high = max(low + STEP0_EPS, opt.amplifyHighUtilCutoff);
        if (util <= low) return opt.amplifyLowFactor;
        if (util >= high) return opt.amplifyHighFactor;

        const double t = clampStep0((util - low) / (high - low), 0.0, 1.0);
        const double smooth = t * t * (3.0 - 2.0 * t);
        return opt.amplifyLowFactor + (opt.amplifyHighFactor - opt.amplifyLowFactor) * smooth;
    }

    static double hmRouterCriticalityFromUtilStep0(
        double util,
        double amplification,
        const CongestionMapBuilder::Options& opt
    ) {
        if (!std::isfinite(util)) return 1.0;
        if (amplification <= STEP0_EPS) return 0.0;
        const double ref = max(1.0, nearFullPenalty(1.0));
        double c = amplification * nearFullPenalty(util) / ref;
        c *= max(0.0, opt.amplifiedCriticalityWeight);
        return clampStep0(c, 0.0, 1.0);
    }

    static void computeAmplifiedCongestionGuideStep0(
        Step0CongestionMapResult& result,
        const CongestionMapBuilder::Options& opt
    ) {
        if (result.channelPressure.empty()) return;

        for (auto& p : result.channelPressure) {
            const double capLR = safeCap(p.effectiveCapLR > STEP0_EPS ? p.effectiveCapLR : p.capLR);
            const double capTB = safeCap(p.effectiveCapTB > STEP0_EPS ? p.effectiveCapTB : p.capTB);

            // 使用 hybrid demand 作為較穩定的 static estimate：
            //   patternDemand 捕捉目前 Router 可能 corridor；RUDY 提供 router-independent 背景壓力；
            //   RISA extra demand 補 hard/edge blockage detour。
            // 若 RUDY 尚未啟用，hybridDemand 會是 0，因此退回 max(pattern, RISA)。
            const double staticDemandLR = max(max(p.predLR, p.risaDemandLR), p.hybridDemandLR);
            const double staticDemandTB = max(max(p.predTB, p.risaDemandTB), p.hybridDemandTB);
            const double staticUtilLR = staticDemandLR / capLR;
            const double staticUtilTB = staticDemandTB / capTB;

            p.amplificationFactorLR = hmAmplificationFactorStep0(staticUtilLR, opt);
            p.amplificationFactorTB = hmAmplificationFactorStep0(staticUtilTB, opt);
            p.amplifiedStaticDemandLR = p.amplificationFactorLR * staticDemandLR;
            p.amplifiedStaticDemandTB = p.amplificationFactorTB * staticDemandTB;

            const double ambientStrength = max(0.0, opt.ambientDemandStrength) * max(0.0, opt.ambientDecayFactor);
            const double maxAmbientLR = max(0.0, opt.ambientMaxFractionOfCap) * capLR;
            const double maxAmbientTB = max(0.0, opt.ambientMaxFractionOfCap) * capTB;
            p.ambientDemandLR = min(maxAmbientLR, ambientStrength * p.amplifiedStaticDemandLR);
            p.ambientDemandTB = min(maxAmbientTB, ambientStrength * p.amplifiedStaticDemandTB);
            p.ambientUtilLR = p.ambientDemandLR / capLR;
            p.ambientUtilTB = p.ambientDemandTB / capTB;

            // routerCriticality 是 Router 應使用的 final criticality。它刻意和 visual/general criticality 分開：
            //   - util < 0.8 時幾乎為 0，避免低壓區被誤懲罰。
            //   - util 接近/超過 capacity 時快速上升。
            //   - RISA supply-demand risk 仍以較小權重保留，避免 effective supply 低的 channel 被忽略。
            p.routerCriticalityLR = hmRouterCriticalityFromUtilStep0(staticUtilLR, p.amplificationFactorLR, opt);
            p.routerCriticalityTB = hmRouterCriticalityFromUtilStep0(staticUtilTB, p.amplificationFactorTB, opt);

            const double risaComponentCrit = clampStep0(
                max(p.risaCostLR, p.risaCostTB) * max(0.0, opt.amplifiedRisaCriticalityWeight),
                0.0,
                1.0
            );
            p.routerCriticality = clampStep0(max(max(p.routerCriticalityLR, p.routerCriticalityTB), risaComponentCrit), 0.0, 1.0);
        }
    }

    static vector<Step0BlockEdgePressure> computeBlockEdgePressureStep0(
        const Design& design,
        const Step0CongestionMapResult& result,
        const CongestionMapBuilder::Options& opt
    ) {
        vector<Step0BlockEdgePressure> out;
        vector<double> edgeDemand = blockEdgeIncidentDemandStep0(design);

        for (int bi = 0; bi < static_cast<int>(design.blocks.size()); ++bi) {
            const BlockInst& b = design.blocks[bi];
            for (int edge = 1; edge <= 4; ++edge) {
                Step0BlockEdgePressure ep;
                ep.blockIndex = bi;
                ep.blockName = b.spec.name;
                ep.blockType = blockTypeNameLocalStep0(b.spec.type);
                ep.edge = edge;
                ep.edgeName = edgeNameStep0(edge);
                ep.incidentDemandEstimate = edgeDemand[bi * 5 + edge];
                ep.edgeSupply = max(1.0, blockEdgeLengthStep0(b.rect, edge) * CHANNEL_DENSITY * opt.edgeAccessConservativeSupplyFactor);
                ep.edgePressure = ep.incidentDemandEstimate / max(1.0, ep.edgeSupply);

                double sumUtil = 0.0;
                set<int> accessChannels;
                for (int ci = 0; ci < static_cast<int>(design.channels.size()) && ci < static_cast<int>(result.channelPressure.size()); ++ci) {
                    if (!channelTouchesBlockEdgeStep0(b.rect, design.channels[ci].rect, edge)) continue;
                    accessChannels.insert(ci);
                    const auto& p = result.channelPressure[ci];
                    const double u = max(p.utilLR, p.utilTB);
                    ep.maxAccessUtil = max(ep.maxAccessUtil, u);
                    ep.maxAccessCriticality = max(ep.maxAccessCriticality, max(p.criticality, p.routerCriticality));
                    sumUtil += u;
                    if (!channelIsHighRiskForEndpointStep0(p, opt)) ++ep.healthyAccessChannelCount;
                }
                ep.accessChannelCount = static_cast<int>(accessChannels.size());
                ep.avgAccessUtil = ep.accessChannelCount > 0 ? sumUtil / static_cast<double>(ep.accessChannelCount) : 0.0;
                vector<int> ids(accessChannels.begin(), accessChannels.end());
                ep.accessChannelsCSV = joinChannelNamesStep0(design, ids);

                // 只有此 edge 真的有 incident demand 時，才把沒有 access 視為風險；避免孤立但無需求的邊誤報。
                ep.edgeAccessRisk = ep.incidentDemandEstimate > STEP0_EPS && ep.accessChannelCount == 0;
                ep.edgeBottleneckRisk = ep.incidentDemandEstimate > STEP0_EPS && ep.accessChannelCount > 0 && ep.healthyAccessChannelCount == 0;

                const double pressureTerm = min(1.0, ep.edgePressure / max(0.1, opt.edgePressureCriticalThreshold));
                const double accessTerm = ep.edgeAccessRisk ? 1.0 : (ep.edgeBottleneckRisk ? 0.80 : 0.0);
                const double utilTerm = min(1.0, ep.maxAccessUtil / max(0.1, opt.endpointAccessUtilThreshold));
                ep.severity = clampStep0(max(pressureTerm, max(accessTerm, utilTerm * (ep.edgePressure >= opt.edgePressureHotThreshold ? 0.85 : 0.35))), 0.0, 1.0);

                ep.suggestedAction = reserveActionForEdgeStep0(edge);
                ep.suggestedMoveDirection = moveDirectionForEdgeStep0(edge);
                ep.suggestedDelta = clampStep0(opt.actionMinGapIncrease + ep.severity * (opt.actionMaxGapIncrease - opt.actionMinGapIncrease),
                    opt.actionMinGapIncrease, opt.actionMaxGapIncrease);

                if (ep.edgeAccessRisk) {
                    ep.reason = "EDGE_ACCESS_RISK: incident demand points to this side, but no channel touches this block edge";
                }
                else if (ep.edgeBottleneckRisk) {
                    ep.reason = "EDGE_BOTTLENECK_RISK: this side has access channels, but all are high-util or high-criticality";
                }
                else if (ep.edgePressure >= opt.edgePressureCriticalThreshold) {
                    ep.reason = "HIGH_EDGE_PIN_PRESSURE: estimated endpoint demand exceeds conservative edge supply";
                }
                else if (ep.edgePressure >= opt.edgePressureHotThreshold) {
                    ep.reason = "WATCH_EDGE_PIN_PRESSURE: estimated endpoint demand is high on this edge";
                }
                else {
                    ep.reason = "OK";
                }

                out.push_back(ep);
            }
        }

        sort(out.begin(), out.end(), [](const Step0BlockEdgePressure& a, const Step0BlockEdgePressure& b) {
            if (fabs(a.severity - b.severity) > 1e-12) return a.severity > b.severity;
            if (fabs(a.edgePressure - b.edgePressure) > 1e-12) return a.edgePressure > b.edgePressure;
            if (a.blockIndex != b.blockIndex) return a.blockIndex < b.blockIndex;
            return a.edge < b.edge;
            });
        return out;
    }

    static int countListItemsStep0(const string& s) {
        if (s.empty()) return 0;
        int count = 1;
        for (char c : s) if (c == '|') ++count;
        return count;
    }

    static string firstTokenStep0(const string& s) {
        const size_t p = s.find('|');
        return p == string::npos ? s : s.substr(0, p);
    }

    static string moveDirectionAwayFromBoxStep0(const Rect& block, const Rect& box, const string& dominantDir) {
        const double dx = rectCx(block) - rectCx(box);
        const double dy = rectCy(block) - rectCy(box);

        // LR 壓力代表水平線多，通常需要增加 channel 高度，所以優先上下分開。
        if (dominantDir == "LR") return dy >= 0.0 ? "MOVE_UP" : "MOVE_DOWN";
        // TB 壓力代表垂直線多，通常需要增加 channel 寬度，所以優先左右分開。
        if (dominantDir == "TB") return dx >= 0.0 ? "MOVE_RIGHT" : "MOVE_LEFT";

        if (fabs(dx) >= fabs(dy)) return dx >= 0.0 ? "MOVE_RIGHT" : "MOVE_LEFT";
        return dy >= 0.0 ? "MOVE_UP" : "MOVE_DOWN";
    }

    static double totalConnectionDemandStep0(const Design& design) {
        double s = 0.0;
        for (const auto& c : design.connections) s += max(0, c.netCount);
        return max(1.0, s);
    }

    static void finalizeFloorplanActionTradeoffsStep0(
        const Design& design,
        vector<Step0FloorplanAction>& actions,
        const CongestionMapBuilder::Options& opt
    ) {
        const double outlineArea = max(1.0, design.outlineW * design.outlineH);
        const double totalDemand = totalConnectionDemandStep0(design);

        for (auto& a : actions) {
            const double actionArea = rectAreaStep0(a.bbox);
            const double normalizedArea = actionArea / outlineArea;
            const double relatedConnFactor = min(1.0, static_cast<double>(countListItemsStep0(a.relatedConnectionsCSV)) / 12.0);

            a.expectedRiskReduction = clampStep0(a.severity * (0.55 + 0.45 * relatedConnFactor), 0.0, 1.0);
            a.estimatedAreaPenalty = clampStep0(normalizedArea * (0.25 + 2.0 * a.suggestedInflationRatio)
                + a.suggestedGapIncrease / max(1.0, max(design.outlineW, design.outlineH)) * 0.15,
                0.0, 1.0);
            a.estimatedWirePenalty = clampStep0(relatedConnFactor * a.suggestedGapIncrease / max(1.0, max(design.outlineW, design.outlineH)),
                0.0, 1.0);
            const double denom = 1e-6 + opt.actionAreaPenaltyWeight * a.estimatedAreaPenalty
                + opt.actionWirePenaltyWeight * a.estimatedWirePenalty;
            a.actionEfficiency = a.expectedRiskReduction / denom;
            a.primaryBlock = firstTokenStep0(a.nearbyBlocksCSV);
            a.suggestedDelta = max(a.suggestedGapIncrease, a.suggestedInflationRatio * sqrt(outlineArea));

            if (a.actionType == "WIDEN_REGION") {
                a.suggestedMoveDirection = "KEEP_AWAY_FROM_REGION";
            }
            else if (a.actionType == "SPREAD_HIGH_PIN_BLOCK" || a.actionType == "RELAX_ENDPOINT_ACCESS") {
                a.suggestedMoveDirection = "RESERVE_LOCAL_ACCESS";
            }
            else {
                a.suggestedMoveDirection = "KEEP_AWAY";
            }

            (void)totalDemand;
        }
    }

    static vector<Step0FloorplanMoveHint> buildFloorplanMoveHintsStep0(
        const Design& design,
        const Step0CongestionMapResult& result,
        const CongestionMapBuilder::Options& opt
    ) {
        vector<Step0FloorplanMoveHint> hints;
        const double outlineArea = max(1.0, design.outlineW * design.outlineH);
        const double chipSpan = max(1.0, max(design.outlineW, design.outlineH));

        auto pushHint = [&](Step0FloorplanMoveHint h) {
            if (h.confidence < opt.moveHintMinConfidence) return;
            h.hintId = static_cast<int>(hints.size());
            const double denom = 1e-6 + opt.actionAreaPenaltyWeight * h.estimatedAreaPenalty
                + opt.actionWirePenaltyWeight * h.estimatedWirePenalty;
            h.actionEfficiency = h.expectedRiskReduction / denom;
            hints.push_back(h);
            };

        // A. edge-level endpoint / pin-pressure hints。
        for (const auto& ep : result.blockEdgePressure) {
            if (static_cast<int>(hints.size()) >= opt.maxFloorplanMoveHints) break;
            if (!(ep.edgeAccessRisk || ep.edgeBottleneckRisk || ep.edgePressure >= opt.edgePressureHotThreshold)) continue;
            if (ep.blockIndex < 0 || ep.blockIndex >= static_cast<int>(design.blocks.size())) continue;

            Step0FloorplanMoveHint h;
            h.sourceActionId = -1;
            h.blockIndex = ep.blockIndex;
            h.blockName = ep.blockName;
            h.blockType = ep.blockType;
            h.targetEdge = ep.edgeName;
            h.moveDirection = ep.suggestedMoveDirection;
            h.suggestedDelta = ep.suggestedDelta;
            h.confidence = ep.severity;
            h.expectedRiskReduction = clampStep0(ep.severity * (0.65 + 0.35 * min(1.0, ep.edgePressure)), 0.0, 1.0);
            h.estimatedAreaPenalty = clampStep0((blockEdgeLengthStep0(design.blocks[ep.blockIndex].rect, ep.edge) * ep.suggestedDelta) / outlineArea, 0.0, 1.0);
            h.estimatedWirePenalty = clampStep0(ep.suggestedDelta / chipSpan * min(1.0, ep.incidentDemandEstimate / totalConnectionDemandStep0(design) * 8.0), 0.0, 1.0);
            h.relatedActionType = ep.suggestedAction;
            h.relatedConnectionsCSV = relatedConnectionsForBlockStep0(design, result, ep.blockIndex);
            h.reason = ep.reason;
            pushHint(h);
        }

        // B. region/action-level spacing hints：把 action bbox 轉成附近 block 的 keep-away 建議。
        for (const auto& a : result.floorplanActions) {
            if (static_cast<int>(hints.size()) >= opt.maxFloorplanMoveHints) break;
            if (a.actionType != "WIDEN_REGION") continue;

            for (int bi = 0; bi < static_cast<int>(design.blocks.size()); ++bi) {
                if (static_cast<int>(hints.size()) >= opt.maxFloorplanMoveHints) break;
                if (!rectIntersectsStep0(design.blocks[bi].rect, a.bbox)) continue;

                Step0FloorplanMoveHint h;
                h.sourceActionId = a.actionId;
                h.blockIndex = bi;
                h.blockName = design.blocks[bi].spec.name;
                h.blockType = blockTypeNameLocalStep0(design.blocks[bi].spec.type);
                h.targetEdge = "REGION";
                h.moveDirection = moveDirectionAwayFromBoxStep0(design.blocks[bi].rect, a.bbox, a.dominantDirection);
                h.suggestedDelta = clampStep0(a.suggestedGapIncrease * 0.60, opt.actionMinGapIncrease, opt.actionMaxGapIncrease);
                h.confidence = clampStep0(a.severity * 0.85, 0.0, 1.0);
                h.expectedRiskReduction = clampStep0(a.expectedRiskReduction * 0.70, 0.0, 1.0);
                h.estimatedAreaPenalty = clampStep0((max(design.blocks[bi].rect.w, design.blocks[bi].rect.h) * h.suggestedDelta) / outlineArea, 0.0, 1.0);
                h.estimatedWirePenalty = clampStep0(h.suggestedDelta / chipSpan * 0.20, 0.0, 1.0);
                h.relatedActionType = a.actionType;
                h.relatedConnectionsCSV = a.relatedConnectionsCSV;
                h.reason = "REGION_KEEP_AWAY: keep block away from CMB bottleneck action box to reserve local channel whitespace";
                pushHint(h);
            }
        }

        sort(hints.begin(), hints.end(), [](const Step0FloorplanMoveHint& a, const Step0FloorplanMoveHint& b) {
            if (fabs(a.actionEfficiency - b.actionEfficiency) > 1e-12) return a.actionEfficiency > b.actionEfficiency;
            if (fabs(a.confidence - b.confidence) > 1e-12) return a.confidence > b.confidence;
            return a.hintId < b.hintId;
            });
        if (static_cast<int>(hints.size()) > opt.maxFloorplanMoveHints) hints.resize(opt.maxFloorplanMoveHints);
        for (int i = 0; i < static_cast<int>(hints.size()); ++i) hints[i].hintId = i;
        return hints;
    }

    static void computeFloorplanFeedbackStep0(
        const Design& design,
        Step0CongestionMapResult& result,
        const CongestionMapBuilder::Options& opt
    ) {
        result.floorplanHealth = Step0FloorplanHealth{};
        result.floorplanActions.clear();
        result.blockEdgePressure.clear();
        result.floorplanMoveHints.clear();
        if (!opt.enableFloorplanFeedback) return;

        Step0FloorplanHealth h;
        h.outlineArea = max(1.0, design.outlineW * design.outlineH);
        for (const auto& b : design.blocks) h.blockArea += rectAreaStep0(b.rect);
        for (const auto& ch : design.channels) h.channelArea += rectAreaStep0(ch.rect);
        h.deadspaceArea = max(0.0, h.outlineArea - h.blockArea);
        h.deadspaceRatio = h.deadspaceArea / max(1.0, h.outlineArea);
        h.maxUtil = result.maxRawUtil > STEP0_EPS ? result.maxRawUtil : result.maxUtil;
        h.avgMaxUtil = result.avgRawUtil > STEP0_EPS ? result.avgRawUtil : result.avgMaxUtil;
        h.numChannels = static_cast<int>(result.channelPressure.size());
        h.numCongestedRegions = static_cast<int>(result.congestedRegions.size());

        double sumRisaUtil = 0.0;
        double sumRudyUtil = 0.0;
        double sumHybridUtil = 0.0;
        for (const auto& p : result.channelPressure) {
            const double u = max(p.utilLR, p.utilTB);
            const double ru = max(p.risaUtilLR, p.risaUtilTB);
            const double rudyU = max(p.rudyUtilLR, p.rudyUtilTB);
            const double hybridU = max(p.hybridUtilLR, p.hybridUtilTB);
            sumRisaUtil += ru;
            sumRudyUtil += rudyU;
            sumHybridUtil += hybridU;
            h.maxRisaUtil = max(h.maxRisaUtil, ru);
            h.maxRudyUtil = max(h.maxRudyUtil, rudyU);
            h.maxHybridUtil = max(h.maxHybridUtil, hybridU);
            h.totalRudyDemand += p.rudyDemandLR + p.rudyDemandTB;
            h.totalHybridDemand += p.hybridDemandLR + p.hybridDemandTB;
            h.hybridDemandCost += p.hybridCost;
            if (rudyU >= opt.floorplanHotUtilThreshold) ++h.numRudyHotChannels;
            if (hybridU >= opt.floorplanHotUtilThreshold || p.hybridCost > STEP0_EPS) ++h.numHybridHotChannels;
            h.totalRisaOverflow += p.risaOverflowLR + p.risaOverflowTB;
            h.risaSupplyDemandCost += p.risaCost;
            if (ru >= opt.floorplanHotUtilThreshold || p.risaCost > STEP0_EPS) ++h.numRisaSupplyDeficitChannels;
            if (p.risaExtraDemandLR > STEP0_EPS || p.risaExtraDemandTB > STEP0_EPS) ++h.numRisaDetourImpactedChannels;
            h.maxCriticality = max(h.maxCriticality, p.criticality);
            h.maxRouterCriticality = max(h.maxRouterCriticality, p.routerCriticality);
            h.maxAmbientUtil = max(h.maxAmbientUtil, max(p.ambientUtilLR, p.ambientUtilTB));
            h.totalAmbientDemand += p.ambientDemandLR + p.ambientDemandTB;
            if (p.amplificationFactorLR > STEP0_EPS || p.amplificationFactorTB > STEP0_EPS) ++h.numAmplifiedChannels;
            if (u >= opt.floorplanHotUtilThreshold) ++h.numHotChannels;
            if (u >= opt.floorplanNearFullUtilThreshold) ++h.numNearFullChannels;
            if (u >= 1.0) ++h.numOverflowChannels;
        }

        h.avgRisaUtil = h.numChannels > 0 ? sumRisaUtil / static_cast<double>(h.numChannels) : 0.0;
        h.avgRudyUtil = h.numChannels > 0 ? sumRudyUtil / static_cast<double>(h.numChannels) : 0.0;
        h.avgHybridUtil = h.numChannels > 0 ? sumHybridUtil / static_cast<double>(h.numChannels) : 0.0;

        for (const auto& r : result.congestedRegions) {
            h.worstRegionScore = max(h.worstRegionScore, r.regionScore);
        }

        double sumPatternPredictability = 0.0;
        for (const auto& g : result.connectionGuide) {
            sumPatternPredictability += g.patternPredictabilityScore;
            if (g.patternPredictabilityClass == "HIGH") ++h.numHighPredictabilityConnections;
            if (g.patternPredictabilityClass == "LOW" || g.patternPredictabilityClass == "ROUTER_FREE") ++h.numLowPredictabilityConnections;
            if (g.couplingRiskScore >= opt.patternCouplingRiskThreshold) ++h.numHighCouplingRiskConnections;
            h.maxCouplingRiskScore = max(h.maxCouplingRiskScore, g.couplingRiskScore);

            if (g.risaMegaOverlapCount > 0) ++h.numRisaMegaOverlapConnections;
            if (g.openRiskScore >= opt.floorplanHighRiskConnectionThreshold || g.openRiskType != "OK") ++h.numHighRiskConnections;
            if (g.openRiskType == "GEOMETRY_DISCONNECTED") ++h.numGeometryDisconnected;
            else if (g.openRiskType == "FT_REQUIRED") ++h.numFTRequired;
            else if (g.openRiskType == "CAPACITY_RISK") ++h.numCapacityRisk;
            else if (g.openRiskType == "ENDPOINT_ACCESS_RISK") ++h.numEndpointAccessRisk;
            else if (g.openRiskType == "ENDPOINT_BOTTLENECK_RISK") ++h.numEndpointBottleneckRisk;
            else if (g.openRiskType == "SHARED_BOTTLENECK_RISK") ++h.numSharedBottleneckRisk;
        }

        h.avgPatternPredictabilityScore = result.connectionGuide.empty() ? 0.0 :
            sumPatternPredictability / static_cast<double>(result.connectionGuide.size());

        vector<double> blockPressure = blockEndpointPressureStep0(design);
        double sumPressure = 0.0;
        for (double v : blockPressure) {
            sumPressure += v;
            h.maxBlockEndpointPressure = max(h.maxBlockEndpointPressure, v);
        }
        h.avgBlockEndpointPressure = blockPressure.empty() ? 0.0 : sumPressure / static_cast<double>(blockPressure.size());

        // V7.1 Modified：edge-level access pressure。
        // 這補上 CRISP pin-density map 對 Problem E 的近似：不只知道某 block 壓力高，
        // 還知道壓力集中在哪一側，以及該側是否有健康 channel access。
        result.blockEdgePressure = computeBlockEdgePressureStep0(design, result, opt);
        for (const auto& ep : result.blockEdgePressure) {
            h.maxEdgePressure = max(h.maxEdgePressure, ep.edgePressure);
            if (ep.edgePressure >= opt.edgePressureHotThreshold) ++h.numHighEdgePressure;
            if (ep.edgeAccessRisk) ++h.numNoAccessEdges;
            if (ep.edgeBottleneckRisk) ++h.numEdgeBottleneckRisk;
        }

        const double hotRatio = h.numChannels > 0 ? static_cast<double>(h.numHotChannels) / h.numChannels : 0.0;
        const double riskConnRatio = design.connections.empty() ? 0.0 : static_cast<double>(h.numHighRiskConnections) / design.connections.size();
        const double edgeRiskRatio = result.blockEdgePressure.empty() ? 0.0
            : static_cast<double>(h.numHighEdgePressure + h.numNoAccessEdges + h.numEdgeBottleneckRisk)
            / static_cast<double>(result.blockEdgePressure.size());
        h.floorplanRiskScore = clampStep0(
            0.26 * min(1.5, h.maxUtil) +
            0.16 * min(1.5, h.worstRegionScore / 2.0) +
            0.13 * min(1.0, hotRatio * 4.0) +
            0.18 * min(1.0, riskConnRatio * 3.0) +
            0.13 * min(1.0, edgeRiskRatio * 8.0) +
            0.08 * min(1.0, h.maxRisaUtil) +
            0.06 * min(1.0, h.risaSupplyDemandCost) +
            0.07 * min(1.0, h.maxHybridUtil) +
            0.04 * min(1.0, h.hybridDemandCost * opt.rudyFloorplanCostWeight) +
            0.08 * min(1.0, h.maxRouterCriticality),
            0.0, 1.0);

        if (h.numOverflowChannels > 0 || h.numGeometryDisconnected > 0 || h.floorplanRiskScore >= 0.65) {
            h.floorplanStatus = "ACTION_NEEDED";
        }
        else if (h.numHotChannels > 0 || h.numHighRiskConnections > 0 || h.floorplanRiskScore >= 0.35) {
            h.floorplanStatus = "WATCH";
        }
        else {
            h.floorplanStatus = "HEALTHY";
        }

        // CRISP 每輪 inflation/spreading 有上限。這裡只是輸出建議，不直接改 block。
        h.suggestedMaxInflationAreaRatio = (h.floorplanStatus == "HEALTHY")
            ? 0.0
            : min(opt.crispMaxInflationPerIter, max(0.0, h.deadspaceRatio * 0.20));

        // 若 congestion 風險高，floorplanner 不該積極壓縮 outline；若健康，才給較寬鬆 compaction 建議。
        const double baseCompaction = clampStep0(0.35 * max(0.0, h.deadspaceRatio - 0.03), 0.0, 0.12);
        h.suggestedMaxCompactionRatio = baseCompaction * (1.0 - h.floorplanRiskScore);
        // 注意：h 後面還會加入 action / move hint 的成本摘要，最後才寫回 result。

        vector<Step0FloorplanAction> actions;
        int aid = 0;

        // A. region-level action：對 CRISP 的 congested-region spreading 做 Problem E 轉譯。
        const int rLimit = min(static_cast<int>(result.congestedRegions.size()), max(0, opt.maxFloorplanActions));
        for (int i = 0; i < rLimit; ++i) {
            const auto& r = result.congestedRegions[i];
            if (r.peakUtil < opt.floorplanHotUtilThreshold && r.peakCriticality < opt.regionCriticalityThreshold) continue;

            const double pad = max(5.0, opt.actionSearchPaddingRatio * max(design.outlineW, design.outlineH));
            Step0FloorplanAction a;
            a.actionId = aid++;
            a.actionType = "WIDEN_REGION";
            a.regionId = r.regionId;
            a.bbox = expandRectClippedStep0(r.bbox, pad, design.outlineW, design.outlineH);
            a.dominantDirection = dominantRegionDirectionStep0(r);
            a.severity = clampStep0(max(r.peakUtil / max(0.1, opt.crispTargetMaxUtil), r.regionScore / 3.0), 0.0, 1.0);
            a.suggestedGapIncrease = suggestedGapIncreaseForRegionStep0(r, opt);
            a.suggestedInflationRatio = min(opt.crispMaxInflationPerIter, 0.0025 + 0.0075 * a.severity);
            a.nearbyBlocksCSV = joinBlockNamesNearRectStep0(design, a.bbox);
            a.relatedConnectionsCSV = relatedConnectionsForRegionStep0(design, result, r);
            a.reason = "CRISP_REGION_SPREADING: high channel utilization/criticality; preserve or add whitespace around this corridor";
            actions.push_back(a);
        }



        // A2. RISA supply-demand action：若 channel effective supply 明顯不足或 detour demand 高，
        // 產生 SUPPLY_DEMAND_BALANCE action。這對應 RISA 的 Max(D - tS,0) 成本：
        // Floorplanner 應在此區增加可用 supply，或降低通過此區的 demand。
        for (int ci = 0; ci < static_cast<int>(design.channels.size()) && ci < static_cast<int>(result.channelPressure.size()); ++ci) {
            if (static_cast<int>(actions.size()) >= opt.maxFloorplanActions) break;
            const auto& p = result.channelPressure[ci];
            const double ru = max(p.risaUtilLR, p.risaUtilTB);
            if (ru < opt.floorplanHotUtilThreshold && p.risaCost <= STEP0_EPS &&
                p.risaExtraDemandLR <= STEP0_EPS && p.risaExtraDemandTB <= STEP0_EPS) continue;

            Step0FloorplanAction a;
            a.actionId = aid++;
            a.actionType = "RISA_SUPPLY_DEMAND_BALANCE";
            a.regionId = p.regionId;
            const double pad = max(5.0, opt.actionSearchPaddingRatio * max(design.outlineW, design.outlineH) * 0.5);
            a.bbox = expandRectClippedStep0(design.channels[ci].rect, pad, design.outlineW, design.outlineH);
            a.dominantDirection = (p.risaUtilLR > p.risaUtilTB * 1.15) ? "LR" : (p.risaUtilTB > p.risaUtilLR * 1.15 ? "TB" : "MIXED");
            a.severity = clampStep0(max(ru / max(0.1, opt.crispTargetMaxUtil), p.risaCost), 0.0, 1.0);
            a.suggestedGapIncrease = clampStep0(opt.actionMinGapIncrease + a.severity * 0.65 * opt.actionMaxGapIncrease,
                opt.actionMinGapIncrease, opt.actionMaxGapIncrease);
            a.suggestedInflationRatio = min(opt.crispMaxInflationPerIter, 0.002 + 0.006 * a.severity);
            a.nearbyBlocksCSV = joinBlockNamesNearRectStep0(design, a.bbox);
            a.relatedConnectionsCSV = "";
            a.reason = "RISA_SUPPLY_DEMAND: effective supply is insufficient after blockage/detour modeling; increase local channel supply or reduce demand through this corridor";
            actions.push_back(a);
        }

        // B. endpoint / pin-density-like action：CRISP 另外使用 pin density map，
        // Problem E 中用 incident netCount 與 endpoint access 近似。
        vector<double> blockRisk(design.blocks.size(), 0.0);
        for (const auto& g : result.connectionGuide) {
            double risk = 0.0;
            if (g.endpointAccessRisk) risk = max(risk, 1.0);
            if (g.endpointBottleneckRisk) risk = max(risk, 0.85);
            if (g.sharedBottleneckRisk) risk = max(risk, 0.55);
            if (g.openRiskScore >= opt.floorplanHighRiskConnectionThreshold) risk = max(risk, g.openRiskScore);
            if (risk <= 0.0) continue;
            if (g.src >= 0 && g.src < static_cast<int>(blockRisk.size())) blockRisk[g.src] = max(blockRisk[g.src], risk);
            if (g.dst >= 0 && g.dst < static_cast<int>(blockRisk.size())) blockRisk[g.dst] = max(blockRisk[g.dst], risk);
        }

        const double pressureThreshold = max(0.25, max(h.avgBlockEndpointPressure * 1.60, opt.floorplanPinPressureScale / 10000.0));
        for (int bi = 0; bi < static_cast<int>(design.blocks.size()); ++bi) {
            const double pressureRisk = blockPressure[bi] >= pressureThreshold ? min(1.0, blockPressure[bi] / max(0.1, pressureThreshold)) : 0.0;
            const double severity = max(blockRisk[bi], pressureRisk);
            if (severity < 0.55) continue;

            const double pad = max(8.0, opt.actionSearchPaddingRatio * max(design.blocks[bi].rect.w, design.blocks[bi].rect.h));
            Step0FloorplanAction a;
            a.actionId = aid++;
            a.actionType = (design.blocks[bi].spec.type == BlockType::SOFT) ? "SPREAD_HIGH_PIN_BLOCK" : "RELAX_ENDPOINT_ACCESS";
            a.regionId = -1;
            a.bbox = expandRectClippedStep0(design.blocks[bi].rect, pad, design.outlineW, design.outlineH);
            a.dominantDirection = "ACCESS";
            a.severity = clampStep0(severity, 0.0, 1.0);
            a.suggestedGapIncrease = clampStep0(opt.actionMinGapIncrease + 0.40 * opt.actionMaxGapIncrease * a.severity,
                opt.actionMinGapIncrease, opt.actionMaxGapIncrease);
            a.suggestedInflationRatio = (design.blocks[bi].spec.type == BlockType::SOFT)
                ? min(opt.crispMaxInflationPerIter, 0.003 + 0.007 * a.severity)
                : 0.0;
            a.nearbyBlocksCSV = design.blocks[bi].spec.name;
            a.relatedConnectionsCSV = relatedConnectionsForBlockStep0(design, result, bi);
            a.reason = (design.blocks[bi].spec.type == BlockType::SOFT)
                ? "CRISP_PIN_DENSITY: high incident net demand or endpoint bottleneck; consider spreading/reshaping this soft block or reserving more local channel"
                : "CRISP_ENDPOINT_ACCESS: hard/edge/high-demand block needs healthier surrounding channel entrances";
            actions.push_back(a);
            if (static_cast<int>(actions.size()) >= opt.maxFloorplanActions) break;
        }

        sort(actions.begin(), actions.end(), [](const Step0FloorplanAction& a, const Step0FloorplanAction& b) {
            if (fabs(a.severity - b.severity) > 1e-12) return a.severity > b.severity;
            return a.actionId < b.actionId;
            });
        for (int i = 0; i < static_cast<int>(actions.size()); ++i) actions[i].actionId = i;

        // V7.1 Modified：補上 action trade-off，讓 Floorplanner 可以比較
        // risk reduction 與 area / wire 代價，而不是只看 severity。
        finalizeFloorplanActionTradeoffsStep0(design, actions, opt);
        result.floorplanActions = actions;

        // V7.1 Modified：把 action / edge pressure 轉成 block-level move hints。
        result.floorplanMoveHints = buildFloorplanMoveHintsStep0(design, result, opt);

        double sumEff = 0.0;
        for (const auto& a : result.floorplanActions) {
            h.bestActionEfficiency = max(h.bestActionEfficiency, a.actionEfficiency);
            sumEff += a.actionEfficiency;
            if (a.actionType == "WIDEN_REGION") h.regionActionCost += a.severity;
            else h.endpointActionCost += a.severity;
        }
        h.avgActionEfficiency = result.floorplanActions.empty() ? 0.0 : sumEff / static_cast<double>(result.floorplanActions.size());
        h.numMoveHints = static_cast<int>(result.floorplanMoveHints.size());
        for (const auto& mh : result.floorplanMoveHints) {
            h.moveHintCost += mh.confidence * (1.0 + 0.25 * mh.estimatedAreaPenalty + 0.25 * mh.estimatedWirePenalty);
        }
        h.floorplanFeedbackCost = h.regionActionCost + h.endpointActionCost + 0.50 * h.moveHintCost
            + opt.risaActionCostWeight * h.risaSupplyDemandCost;

        result.floorplanHealth = h;
    }

} // namespace

vector<double> Step0CongestionMapResult::channelCriticality() const {
    vector<double> out;
    out.reserve(channelPressure.size());
    for (const auto& p : channelPressure) {
        // V7.3：Router 使用 routerCriticality；若尚未啟用 amplified guide，退回 general criticality。
        out.push_back(p.routerCriticality > STEP0_EPS ? p.routerCriticality : p.criticality);
    }
    return out;
}

vector<double> Step0CongestionMapResult::channelAmbientDemandLR() const {
    vector<double> out;
    out.reserve(channelPressure.size());
    for (const auto& p : channelPressure) out.push_back(p.ambientDemandLR);
    return out;
}

vector<double> Step0CongestionMapResult::channelAmbientDemandTB() const {
    vector<double> out;
    out.reserve(channelPressure.size());
    for (const auto& p : channelPressure) out.push_back(p.ambientDemandTB);
    return out;
}

vector<double> Step0CongestionMapResult::connectionRoutePriority() const {
    vector<double> out;
    out.reserve(connectionGuide.size());
    for (const auto& g : connectionGuide) out.push_back(g.routePriority);
    return out;
}

CongestionMapBuilder::CongestionMapBuilder()
    : opt_(Options{}) {}

CongestionMapBuilder::CongestionMapBuilder(Options opt)
    : opt_(std::move(opt)) {}

Step0CongestionMapResult CongestionMapBuilder::build(const Design& design) const {
    Step0CongestionMapResult result;
    result.channelPressure.assign(design.channels.size(), Step0ChannelPressure{});
    result.connectionGuide.assign(design.connections.size(), Step0ConnectionGuide{});

    // 初始化 channel direction capacity。
    for (int ci = 0; ci < static_cast<int>(design.channels.size()); ++ci) {
        const Channel& ch = design.channels[ci];
        result.channelPressure[ci].capLR = max(0.0, ch.rect.h) * CHANNEL_DENSITY;
        result.channelPressure[ci].capTB = max(0.0, ch.rect.w) * CHANNEL_DENSITY;
    }

    vector<ConnState> states(design.connections.size());

    // A. 產生 candidates，並做 initial probabilistic assignment。
    for (int k = 0; k < static_cast<int>(design.connections.size()); ++k) {
        const Connection& conn = design.connections[k];
        states[k].candidates = generateCandidates(design, conn, opt_);

        result.connectionGuide[k].connectionIndex = k;
        result.connectionGuide[k].src = conn.src;
        result.connectionGuide[k].dst = conn.dst;
        result.connectionGuide[k].netCount = conn.netCount;

        const int m = static_cast<int>(states[k].candidates.size());
        if (m <= 0) continue;

        // V6.1：初始 assignment 不再對所有 pattern 等權。
        // Straight / L / Z 依據 probabilistic congestion prediction 的直覺給不同 base weight，
        // 避免 Z-shape 在沒有壅塞壓力時被過度放大，也避免 L/Z 完全平均造成不自然 demand。
        states[k].assignedWeight = initialCandidateWeightsStep0(states[k].candidates, opt_);
        for (int j = 0; j < m; ++j) {
            const double demand = static_cast<double>(conn.netCount) * states[k].assignedWeight[j];
            addCandidateDemand(design, states[k].candidates[j], demand, result.channelPressure, opt_);
        }
    }

    finalizeUtilAndHistory(result.channelPressure);

    // B. Cheap pattern reroute pass。
    //    每次移除該 connection 原本分配，依目前 map 選最低成本 pattern，再加回 full netCount。
    const int passes = max(0, opt_.reroutePasses);
    for (int pass = 0; pass < passes; ++pass) {
        for (int k = 0; k < static_cast<int>(design.connections.size()); ++k) {
            const Connection& conn = design.connections[k];
            ConnState& st = states[k];
            const int m = static_cast<int>(st.candidates.size());
            if (m <= 0) continue;

            // remove previous assignment
            for (int j = 0; j < m; ++j) {
                const double w = j < static_cast<int>(st.assignedWeight.size()) ? st.assignedWeight[j] : 0.0;
                const double demand = static_cast<double>(conn.netCount) * w;
                if (fabs(demand) > EPS) {
                    addCandidateDemand(design, st.candidates[j], -demand, result.channelPressure, opt_);
                }
            }

            int best = 0;
            double bestCost = numeric_limits<double>::infinity();
            for (int j = 0; j < m; ++j) {
                const double cost = evaluateCandidateCost(design, conn, st.candidates[j], result.channelPressure, opt_);
                if (cost < bestCost) {
                    bestCost = cost;
                    best = j;
                }
            }

            st.assignedWeight.assign(m, 0.0);
            st.assignedWeight[best] = 1.0;
            st.selectedCandidate = best;
            st.selectedCost = bestCost;
            addCandidateDemand(design, st.candidates[best], static_cast<double>(conn.netCount), result.channelPressure, opt_);

            result.connectionGuide[k].selectedPattern = st.candidates[best].name;
            result.connectionGuide[k].selectedCost = bestCost;
        }

        finalizeUtilAndHistory(result.channelPressure);
    }

    // C. Criticality / route priority。
    double sumMaxUtil = 0.0;
    result.maxUtil = 0.0;
    result.avgMaxUtil = 0.0;
    result.maxRawUtil = 0.0;
    result.avgRawUtil = 0.0;
    result.maxRisaUtil = 0.0;
    result.maxHybridUtil = 0.0;
    result.maxModelUtil = 0.0;
    result.maxCriticalityRaw = 0.0;

    for (int ci = 0; ci < static_cast<int>(result.channelPressure.size()); ++ci) {
        Step0ChannelPressure& p = result.channelPressure[ci];
        p.utilLR = p.predLR / safeCap(p.capLR);
        p.utilTB = p.predTB / safeCap(p.capTB);

        const double maxUtil = max(p.utilLR, p.utilTB);
        const double maxHist = max(p.histLR, p.histTB);
        p.criticalityRaw = nearFullPenalty(maxUtil) * maxHist;

        const bool narrowUsed =
            (p.predLR > EPS && p.capLR < opt_.narrowComponentCap) ||
            (p.predTB > EPS && p.capTB < opt_.narrowComponentCap);
        if (narrowUsed) p.criticalityRaw *= 1.25;

        result.maxCriticalityRaw = max(result.maxCriticalityRaw, p.criticalityRaw);
        result.maxRawUtil = max(result.maxRawUtil, maxUtil);
        sumMaxUtil += maxUtil;
    }

    result.avgRawUtil = result.channelPressure.empty() ? 0.0 : sumMaxUtil / static_cast<double>(result.channelPressure.size());
    result.maxUtil = result.maxRawUtil;
    result.avgMaxUtil = result.avgRawUtil;

    // 第一版不要做「相對正規化到最大值=1」。
    // 若整張 map 都很健康，例如最大 util 只有 0.18，
    // 相對正規化會讓最不健康的 channel 仍被設成 criticality=1，
    // 導致 Router 誤以為它是嚴重瓶頸。
    // 這裡改成絕對尺度：以 util 接近 1.0 的 nearFullPenalty 作為參考。
    // 因此健康 channel 的 criticality 會維持很小，真的接近滿載才會接近 1。
    const double criticalityReference = max(1.0, nearFullPenalty(1.0));
    for (auto& p : result.channelPressure) {
        p.criticality = min(1.0, p.criticalityRaw / criticalityReference);
    }

    // V7.2-RISA：在 basic pattern demand 後加入 supply-demand balance、
    // effective supply、hard/edge blockage detour demand，並更新 criticality。
    if (opt_.enableRisaSupplyDemand) {
        computeRisaSupplyDemandStep0(design, states, result, opt_);
    }

    // V7.5-RUDY：加入 router-independent background demand，並形成 hybrid demand。
    // 它補足 L/Z pattern 估計對目前 Router 行為的依賴，特別適合 floorplan health / region score。
    if (opt_.enableRudyBackgroundDemand) {
        computeRudyBackgroundDemandStep0(design, result, opt_);
    }

    // V7.3-HM：將 congestion estimate 做 amplification，並產生 ambient demand。
    // 注意：ambient demand 是 Router cost guide，不是 hard capacity feasibility。
    if (opt_.enableAmplifiedCongestionGuide) {
        computeAmplifiedCongestionGuideStep0(result, opt_);
    }

    // 保留 raw 與 model 的分離語意：
    //   raw = pattern demand utilization；
    //   model = max(raw, RISA, hybrid)。
    result.maxModelUtil = max(result.maxRawUtil, max(result.maxRisaUtil, result.maxHybridUtil));

    // Connection priority 先給一個簡化版，之後 Router 若要調排序可直接使用。
    for (int k = 0; k < static_cast<int>(design.connections.size()); ++k) {
        const Connection& conn = design.connections[k];
        double risk = 0.0;
        if (states[k].selectedCandidate >= 0) {
            const Step0PatternCandidate& c = states[k].candidates[states[k].selectedCandidate];
            for (const auto& seg : c.segs) {
                for (int ci = 0; ci < static_cast<int>(design.channels.size()); ++ci) {
                    const Rect& r = design.channels[ci].rect;
                    const Step0ChannelPressure& p = result.channelPressure[ci];
                    if (isHorizontal(seg) && horizontalLineCrossesRect(seg, r, opt_.projectionTolerance)) {
                        risk = max(risk, p.utilLR);
                    }
                    else if (isVertical(seg) && verticalLineCrossesRect(seg, r, opt_.projectionTolerance)) {
                        risk = max(risk, p.utilTB);
                    }
                }
            }
        }
        result.connectionGuide[k].routePriority = static_cast<double>(conn.netCount) * (1.0 + risk);
    }

    // V5：補上 connection-level route-open risk analysis。
    fillConnectionOpenRiskStep0(design, states, result, opt_);

    // V6.1：補上 per-connection preferred / avoid channel guide。
    // 注意：這仍然只是 Router 的 soft guide，不是硬性路徑限制。
    fillConnectionGuideChannelsStep0(design, states, result, opt_);

    // V7.4-Pattern：判斷每條 connection 是否適合被 pattern guide 強力引導，
    // 並估算同方向 parallel demand 的 coupling-like risk。
    computePatternPredictabilityStep0(design, states, result, opt_);

    // 重新計算 route priority，納入 pattern predictability / coupling risk。
    refineConnectionPriorityStep0(result);

    // V6.3：把相鄰的高壓 channels 合併成 congested regions。
    // 這讓 debug 從「單一 CH 很紅」提升成「某一整塊 corridor 是瓶頸」。
    result.congestedRegions = buildCongestedRegionsStep0(design, result, opt_);
    fillConnectionRegionLinksStep0(result);

    // V7.1：依照 CRISP 的 measure-and-improve 精神，
    // 將 channel/connection/region risk 轉成 floorplan health 與 action hints。
    computeFloorplanFeedbackStep0(design, result, opt_);

    if (opt_.verbose) printSummary(design, result);
    if (opt_.exportFiles) {
        exportCSV(design, result, opt_.csvPath);
        if (opt_.exportConnectionRiskCSV) {
            exportConnectionRiskCSV(design, result, opt_.connectionRiskCsvPath);
        }
        if (opt_.exportConnectionGuideCSV) {
            exportConnectionGuideCSV(design, result, opt_.connectionGuideCsvPath);
        }
        if (opt_.exportCongestedRegionsCSV) {
            exportCongestedRegionsCSV(design, result, opt_.congestedRegionsCsvPath);
        }
        if (opt_.exportFloorplanHealthCSV) {
            exportFloorplanHealthCSV(design, result, opt_.floorplanHealthCsvPath);
        }
        if (opt_.exportFloorplanActionsCSV) {
            exportFloorplanActionsCSV(design, result, opt_.floorplanActionsCsvPath);
        }
        if (opt_.exportBlockAccessPressureCSV) {
            exportBlockAccessPressureCSV(design, result, opt_.blockAccessPressureCsvPath);
        }
        if (opt_.exportFloorplanMoveHintsCSV) {
            exportFloorplanMoveHintsCSV(design, result, opt_.floorplanMoveHintsCsvPath);
        }
        if (opt_.exportFloorplanDeltaCSV) {
            exportFloorplanDeltaCSV(design, result, opt_.floorplanDeltaCsvPath);
        }
        if (opt_.exportRisaSupplyDemandCSV) {
            exportRisaSupplyDemandCSV(design, result, opt_.risaSupplyDemandCsvPath);
        }
        if (opt_.exportAmplifiedGuideCSV) {
            exportAmplifiedGuideCSV(design, result, opt_.amplifiedGuideCsvPath);
        }
        if (opt_.exportPatternPredictabilityCSV) {
            exportPatternPredictabilityCSV(design, result, opt_.patternPredictabilityCsvPath);
        }
        if (opt_.exportRudyBackgroundCSV) {
            exportRudyBackgroundCSV(design, result, opt_.rudyBackgroundCsvPath);
        }
        if (opt_.exportDashboardCSV) {
            exportDashboardCSV(design, result, opt_.dashboardCsvPath);
        }
        if (opt_.exportSignalConflictCSV) {
            exportSignalConflictCSV(design, result, opt_.signalConflictCsvPath);
        }
        if (opt_.exportTextReport) {
            exportTextReport(design, result, opt_.textReportPath);
        }
        exportSVG(design, result, opt_.svgPath);
        if (opt_.verbose) {
            cerr << "[Step0CongestionMap] exported " << opt_.csvPath << " and " << opt_.svgPath;
            if (opt_.exportConnectionRiskCSV) cerr << " and " << opt_.connectionRiskCsvPath;
            if (opt_.exportConnectionGuideCSV) cerr << " and " << opt_.connectionGuideCsvPath;
            if (opt_.exportCongestedRegionsCSV) cerr << " and " << opt_.congestedRegionsCsvPath;
            if (opt_.exportFloorplanHealthCSV) cerr << " and " << opt_.floorplanHealthCsvPath;
            if (opt_.exportFloorplanActionsCSV) cerr << " and " << opt_.floorplanActionsCsvPath;
            if (opt_.exportBlockAccessPressureCSV) cerr << " and " << opt_.blockAccessPressureCsvPath;
            if (opt_.exportFloorplanMoveHintsCSV) cerr << " and " << opt_.floorplanMoveHintsCsvPath;
            if (opt_.exportFloorplanDeltaCSV) cerr << " and " << opt_.floorplanDeltaCsvPath;
            if (opt_.exportRisaSupplyDemandCSV) cerr << " and " << opt_.risaSupplyDemandCsvPath;
            if (opt_.exportAmplifiedGuideCSV) cerr << " and " << opt_.amplifiedGuideCsvPath;
            if (opt_.exportPatternPredictabilityCSV) cerr << " and " << opt_.patternPredictabilityCsvPath;
            if (opt_.exportRudyBackgroundCSV) cerr << " and " << opt_.rudyBackgroundCsvPath;
            if (opt_.exportDashboardCSV) cerr << " and " << opt_.dashboardCsvPath;
            if (opt_.exportSignalConflictCSV) cerr << " and " << opt_.signalConflictCsvPath;
            if (opt_.exportTextReport) cerr << " and " << opt_.textReportPath;
            cerr << "\n";
        }
    }

    return result;
}

void CongestionMapBuilder::exportCSV(const Design& design, const Step0CongestionMapResult& result, const string& path) const {
    ofstream fout(path);
    if (!fout) return;

    fout << "channel,index,x,y,w,h,capLR,capTB,effectiveCapLR,effectiveCapTB,supplyReductionLR,supplyReductionTB,"
        << "predLR,predTB,risaExtraDemandLR,risaExtraDemandTB,risaDemandLR,risaDemandTB,"
        << "rudyDemandLR,rudyDemandTB,rudyUtilLR,rudyUtilTB,rudyOverlapArea,"
        << "hybridDemandLR,hybridDemandTB,hybridUtilLR,hybridUtilTB,hybridOverflowLR,hybridOverflowTB,hybridCost,"
        << "utilLR,utilTB,risaUtilLR,risaUtilTB,risaOverflowLR,risaOverflowTB,risaCost,"
        << "amplificationFactorLR,amplificationFactorTB,amplifiedStaticDemandLR,amplifiedStaticDemandTB,"
        << "ambientDemandLR,ambientDemandTB,ambientUtilLR,ambientUtilTB,"
        << "routerCriticalityLR,routerCriticalityTB,routerCriticality,"
        << "histLR,histTB,criticalityRaw,criticality,regionId\n";
    for (int i = 0; i < static_cast<int>(design.channels.size()) && i < static_cast<int>(result.channelPressure.size()); ++i) {
        const Channel& ch = design.channels[i];
        const Step0ChannelPressure& p = result.channelPressure[i];
        fout << ch.name << ',' << i << ','
            << ch.rect.x << ',' << ch.rect.y << ',' << ch.rect.w << ',' << ch.rect.h << ','
            << p.capLR << ',' << p.capTB << ','
            << p.effectiveCapLR << ',' << p.effectiveCapTB << ','
            << p.supplyReductionLR << ',' << p.supplyReductionTB << ','
            << p.predLR << ',' << p.predTB << ','
            << p.risaExtraDemandLR << ',' << p.risaExtraDemandTB << ','
            << p.risaDemandLR << ',' << p.risaDemandTB << ','
            << p.rudyDemandLR << ',' << p.rudyDemandTB << ',' << p.rudyUtilLR << ',' << p.rudyUtilTB << ',' << p.rudyOverlapArea << ','
            << p.hybridDemandLR << ',' << p.hybridDemandTB << ',' << p.hybridUtilLR << ',' << p.hybridUtilTB << ','
            << p.hybridOverflowLR << ',' << p.hybridOverflowTB << ',' << p.hybridCost << ','
            << p.utilLR << ',' << p.utilTB << ','
            << p.risaUtilLR << ',' << p.risaUtilTB << ','
            << p.risaOverflowLR << ',' << p.risaOverflowTB << ',' << p.risaCost << ','
            << p.amplificationFactorLR << ',' << p.amplificationFactorTB << ','
            << p.amplifiedStaticDemandLR << ',' << p.amplifiedStaticDemandTB << ','
            << p.ambientDemandLR << ',' << p.ambientDemandTB << ','
            << p.ambientUtilLR << ',' << p.ambientUtilTB << ','
            << p.routerCriticalityLR << ',' << p.routerCriticalityTB << ',' << p.routerCriticality << ','
            << p.histLR << ',' << p.histTB << ','
            << p.criticalityRaw << ',' << p.criticality << ',' << p.regionId << '\n';
    }
}

void CongestionMapBuilder::exportConnectionRiskCSV(const Design& design, const Step0CongestionMapResult& result, const string& path) const {
    ofstream fout(path);
    if (!fout) return;

    fout << "connIndex,src,dst,netCount,selectedPattern,selectedCost,routePriority,"
        << "channelOnlyConnected,ftEnabledConnected,geometryOpenRisk,capacityOpenRisk,"
        << "srcAccessChannelCount,dstAccessChannelCount,srcHealthyAccessChannelCount,dstHealthyAccessChannelCount,"
        << "srcAccessMaxUtil,dstAccessMaxUtil,endpointAccessRisk,endpointBottleneckRisk,"
        << "sharedBottleneckRisk,sharedBottleneckScore,commonBottleneckChannels,commonBottleneckRegions,"
        << "totalCandidateCount,healthyCandidateCount,bestPatternMaxUtil,hasProjectedPattern,lackOfAlternative,"
        << "openRiskScore,openRiskType,risaMegaOverlapCount,risaDetourDemandAdded,risaBlockedBlocks,risaNbbPartitionHint\n";

    for (const auto& g : result.connectionGuide) {
        string srcName = "?";
        string dstName = "?";
        if (g.src >= 0 && g.src < static_cast<int>(design.blocks.size())) srcName = design.blocks[g.src].spec.name;
        if (g.dst >= 0 && g.dst < static_cast<int>(design.blocks.size())) dstName = design.blocks[g.dst].spec.name;

        fout << g.connectionIndex << ','
            << srcName << ',' << dstName << ',' << g.netCount << ','
            << g.selectedPattern << ',' << g.selectedCost << ',' << g.routePriority << ','
            << (g.channelOnlyConnected ? 1 : 0) << ','
            << (g.ftEnabledConnected ? 1 : 0) << ','
            << (g.geometryOpenRisk ? 1 : 0) << ','
            << (g.capacityOpenRisk ? 1 : 0) << ','
            << g.srcAccessChannelCount << ',' << g.dstAccessChannelCount << ','
            << g.srcHealthyAccessChannelCount << ',' << g.dstHealthyAccessChannelCount << ','
            << g.srcAccessMaxUtil << ',' << g.dstAccessMaxUtil << ','
            << (g.endpointAccessRisk ? 1 : 0) << ','
            << (g.endpointBottleneckRisk ? 1 : 0) << ','
            << (g.sharedBottleneckRisk ? 1 : 0) << ','
            << g.sharedBottleneckScore << ',' << g.commonBottleneckChannelsCSV << ',' << g.commonBottleneckRegionsCSV << ','
            << g.totalCandidateCount << ',' << g.healthyCandidateCount << ','
            << g.bestPatternMaxUtil << ',' << (g.hasProjectedPattern ? 1 : 0) << ',' << g.lackOfAlternative << ','
            << g.openRiskScore << ',' << g.openRiskType << '\n';
    }
}


void CongestionMapBuilder::exportConnectionGuideCSV(const Design& design, const Step0CongestionMapResult& result, const string& path) const {
    ofstream fout(path);
    if (!fout) return;

    fout << "connIndex,src,dst,netCount,selectedPattern,selectedCost,routePriority,"
        << "preferredChannels,avoidChannels,commonBottleneckChannels,"
        << "preferredRegions,avoidRegions,commonBottleneckRegions,"
        << "rudyBBoxArea,rudyHpwl,rudyDensity,rudyContribution,rudyTouchedChannelCount,rudyTouchedChannels,"
        << "bestPatternMaxUtil,hasProjectedPattern,bestPatternAvgUtil,guideConfidence,patternPredictabilityScore,couplingRiskScore,patternGuideStrength,patternPredictabilityClass,patternGuideMode,patternReason,healthyCandidateCount,lackOfAlternative,"
        << "srcAccessChannelCount,dstAccessChannelCount,srcHealthyAccessChannelCount,dstHealthyAccessChannelCount,"
        << "endpointAccessRisk,endpointBottleneckRisk,sharedBottleneckRisk,sharedBottleneckScore,"
        << "openRiskScore,openRiskType,risaMegaOverlapCount,risaDetourDemandAdded,risaBlockedBlocks,risaNbbPartitionHint\n";

    for (const auto& g : result.connectionGuide) {
        string srcName = "?";
        string dstName = "?";
        if (g.src >= 0 && g.src < static_cast<int>(design.blocks.size())) srcName = design.blocks[g.src].spec.name;
        if (g.dst >= 0 && g.dst < static_cast<int>(design.blocks.size())) dstName = design.blocks[g.dst].spec.name;

        fout << g.connectionIndex << ','
            << srcName << ',' << dstName << ',' << g.netCount << ','
            << g.selectedPattern << ',' << g.selectedCost << ',' << g.routePriority << ','
            << g.preferredChannelsCSV << ',' << g.avoidChannelsCSV << ',' << g.commonBottleneckChannelsCSV << ','
            << g.preferredRegionsCSV << ',' << g.avoidRegionsCSV << ',' << g.commonBottleneckRegionsCSV << ','
            << g.rudyBBoxArea << ',' << g.rudyHpwl << ',' << g.rudyDensity << ',' << g.rudyContribution << ','
            << g.rudyTouchedChannelCount << ',' << g.rudyTouchedChannelsCSV << ','
            << g.bestPatternMaxUtil << ',' << (g.hasProjectedPattern ? 1 : 0) << ',' << g.bestPatternAvgUtil << ','
            << g.guideConfidence << ',' << g.patternPredictabilityScore << ',' << g.couplingRiskScore << ','
            << g.patternGuideStrength << ',' << g.patternPredictabilityClass << ',' << g.patternGuideMode << ','
            << '"' << g.patternReason << '"' << ',' << g.healthyCandidateCount << ',' << g.lackOfAlternative << ','
            << g.srcAccessChannelCount << ',' << g.dstAccessChannelCount << ','
            << g.srcHealthyAccessChannelCount << ',' << g.dstHealthyAccessChannelCount << ','
            << (g.endpointAccessRisk ? 1 : 0) << ','
            << (g.endpointBottleneckRisk ? 1 : 0) << ','
            << (g.sharedBottleneckRisk ? 1 : 0) << ',' << g.sharedBottleneckScore << ','
            << g.openRiskScore << ',' << g.openRiskType << ','
            << g.risaMegaOverlapCount << ',' << g.risaDetourDemandAdded << ','
            << g.risaBlockedBlocksCSV << ',' << g.risaNbbPartitionHint << '\n';
    }
}

void CongestionMapBuilder::exportCongestedRegionsCSV(const Design& design, const Step0CongestionMapResult& result, const string& path) const {
    ofstream fout(path);
    if (!fout) return;

    fout << "regionId,numChannels,bboxX,bboxY,bboxW,bboxH,peakUtil,avgUtil,peakCriticality,avgCriticality,"
        << "totalPredLR,totalPredTB,totalOverflowRisk,peakRisaUtil,totalRisaOverflowRisk,totalRisaCost,"
        << "peakRudyUtil,peakHybridUtil,totalRudyDemand,totalHybridDemand,totalHybridCost,regionScore,channels\n";

    for (const auto& r : result.congestedRegions) {
        fout << r.regionId << ','
            << r.channelIndices.size() << ','
            << r.bbox.x << ',' << r.bbox.y << ',' << r.bbox.w << ',' << r.bbox.h << ','
            << r.peakUtil << ',' << r.avgUtil << ','
            << r.peakCriticality << ',' << r.avgCriticality << ','
            << r.totalPredLR << ',' << r.totalPredTB << ','
            << r.totalOverflowRisk << ',' << r.peakRisaUtil << ','
            << r.totalRisaOverflowRisk << ',' << r.totalRisaCost << ','
            << r.peakRudyUtil << ',' << r.peakHybridUtil << ','
            << r.totalRudyDemand << ',' << r.totalHybridDemand << ',' << r.totalHybridCost << ','
            << r.regionScore << ',' << r.channelsCSV << '\n';
    }
}


void CongestionMapBuilder::exportFloorplanHealthCSV(const Design& design, const Step0CongestionMapResult& result, const string& path) const {
    ofstream fout(path);
    if (!fout) return;

    const auto& h = result.floorplanHealth;
    fout << "outlineArea,blockArea,channelArea,deadspaceArea,deadspaceRatio,"
        << "maxUtil,avgMaxUtil,maxRawUtil,avgRawUtil,maxModelUtil,maxCriticality,worstRegionScore,"
        << "maxRisaUtil,avgRisaUtil,totalRisaOverflow,risaSupplyDemandCost,numRisaSupplyDeficitChannels,numRisaDetourImpactedChannels,numRisaMegaOverlapConnections,"
        << "maxRouterCriticality,maxAmbientUtil,totalAmbientDemand,numAmplifiedChannels,"
        << "numHighPredictabilityConnections,numLowPredictabilityConnections,numHighCouplingRiskConnections,avgPatternPredictabilityScore,maxCouplingRiskScore,"
        << "maxRudyUtil,avgRudyUtil,totalRudyDemand,numRudyHotChannels,maxHybridUtil,avgHybridUtil,totalHybridDemand,hybridDemandCost,numHybridHotChannels,"
        << "numChannels,numHotChannels,numNearFullChannels,numOverflowChannels,numCongestedRegions,"
        << "numHighRiskConnections,numGeometryDisconnected,numFTRequired,numCapacityRisk,"
        << "numEndpointAccessRisk,numEndpointBottleneckRisk,numSharedBottleneckRisk,"
        << "avgBlockEndpointPressure,maxBlockEndpointPressure,"
        << "numHighEdgePressure,numNoAccessEdges,numEdgeBottleneckRisk,maxEdgePressure,"
        << "numMoveHints,avgActionEfficiency,bestActionEfficiency,"
        << "regionActionCost,endpointActionCost,moveHintCost,floorplanFeedbackCost,"
        << "suggestedMaxInflationAreaRatio,suggestedMaxCompactionRatio,floorplanRiskScore,floorplanStatus\n";

    fout << h.outlineArea << ',' << h.blockArea << ',' << h.channelArea << ','
        << h.deadspaceArea << ',' << h.deadspaceRatio << ','
        << h.maxUtil << ',' << h.avgMaxUtil << ','
        << result.maxRawUtil << ',' << result.avgRawUtil << ',' << result.maxModelUtil << ','
        << h.maxCriticality << ',' << h.worstRegionScore << ','
        << h.maxRisaUtil << ',' << h.avgRisaUtil << ',' << h.totalRisaOverflow << ',' << h.risaSupplyDemandCost << ','
        << h.numRisaSupplyDeficitChannels << ',' << h.numRisaDetourImpactedChannels << ',' << h.numRisaMegaOverlapConnections << ','
        << h.maxRouterCriticality << ',' << h.maxAmbientUtil << ',' << h.totalAmbientDemand << ',' << h.numAmplifiedChannels << ','
        << h.numHighPredictabilityConnections << ',' << h.numLowPredictabilityConnections << ',' << h.numHighCouplingRiskConnections << ','
        << h.avgPatternPredictabilityScore << ',' << h.maxCouplingRiskScore << ','
        << h.maxRudyUtil << ',' << h.avgRudyUtil << ',' << h.totalRudyDemand << ',' << h.numRudyHotChannels << ','
        << h.maxHybridUtil << ',' << h.avgHybridUtil << ',' << h.totalHybridDemand << ',' << h.hybridDemandCost << ',' << h.numHybridHotChannels << ','
        << h.numChannels << ',' << h.numHotChannels << ',' << h.numNearFullChannels << ','
        << h.numOverflowChannels << ',' << h.numCongestedRegions << ','
        << h.numHighRiskConnections << ',' << h.numGeometryDisconnected << ',' << h.numFTRequired << ','
        << h.numCapacityRisk << ',' << h.numEndpointAccessRisk << ',' << h.numEndpointBottleneckRisk << ','
        << h.numSharedBottleneckRisk << ',' << h.avgBlockEndpointPressure << ',' << h.maxBlockEndpointPressure << ','
        << h.numHighEdgePressure << ',' << h.numNoAccessEdges << ',' << h.numEdgeBottleneckRisk << ',' << h.maxEdgePressure << ','
        << h.numMoveHints << ',' << h.avgActionEfficiency << ',' << h.bestActionEfficiency << ','
        << h.regionActionCost << ',' << h.endpointActionCost << ',' << h.moveHintCost << ',' << h.floorplanFeedbackCost << ','
        << h.suggestedMaxInflationAreaRatio << ',' << h.suggestedMaxCompactionRatio << ','
        << h.floorplanRiskScore << ',' << h.floorplanStatus << '\n';
}

void CongestionMapBuilder::exportFloorplanActionsCSV(const Design& design, const Step0CongestionMapResult& result, const string& path) const {
    ofstream fout(path);
    if (!fout) return;

    fout << "actionId,actionType,regionId,bboxX,bboxY,bboxW,bboxH,dominantDirection,"
        << "severity,suggestedGapIncrease,suggestedInflationRatio,"
        << "expectedRiskReduction,estimatedAreaPenalty,estimatedWirePenalty,actionEfficiency,"
        << "primaryBlock,suggestedMoveDirection,suggestedDelta,nearbyBlocks,relatedConnections,reason\n";
    for (const auto& a : result.floorplanActions) {
        fout << a.actionId << ',' << a.actionType << ',' << a.regionId << ','
            << a.bbox.x << ',' << a.bbox.y << ',' << a.bbox.w << ',' << a.bbox.h << ','
            << a.dominantDirection << ',' << a.severity << ',' << a.suggestedGapIncrease << ','
            << a.suggestedInflationRatio << ','
            << a.expectedRiskReduction << ',' << a.estimatedAreaPenalty << ',' << a.estimatedWirePenalty << ',' << a.actionEfficiency << ','
            << a.primaryBlock << ',' << a.suggestedMoveDirection << ',' << a.suggestedDelta << ','
            << a.nearbyBlocksCSV << ',' << a.relatedConnectionsCSV << ',' << a.reason << '\n';
    }
}

void CongestionMapBuilder::exportBlockAccessPressureCSV(const Design& design, const Step0CongestionMapResult& result, const string& path) const {
    ofstream fout(path);
    if (!fout) return;

    fout << "blockIndex,blockName,blockType,edge,edgeName,incidentDemandEstimate,edgeSupply,edgePressure,"
        << "accessChannelCount,healthyAccessChannelCount,maxAccessUtil,avgAccessUtil,maxAccessCriticality,"
        << "edgeAccessRisk,edgeBottleneckRisk,severity,accessChannels,suggestedAction,suggestedMoveDirection,suggestedDelta,reason\n";
    for (const auto& ep : result.blockEdgePressure) {
        fout << ep.blockIndex << ',' << ep.blockName << ',' << ep.blockType << ','
            << ep.edge << ',' << ep.edgeName << ','
            << ep.incidentDemandEstimate << ',' << ep.edgeSupply << ',' << ep.edgePressure << ','
            << ep.accessChannelCount << ',' << ep.healthyAccessChannelCount << ','
            << ep.maxAccessUtil << ',' << ep.avgAccessUtil << ',' << ep.maxAccessCriticality << ','
            << (ep.edgeAccessRisk ? 1 : 0) << ',' << (ep.edgeBottleneckRisk ? 1 : 0) << ','
            << ep.severity << ',' << ep.accessChannelsCSV << ','
            << ep.suggestedAction << ',' << ep.suggestedMoveDirection << ',' << ep.suggestedDelta << ','
            << ep.reason << '\n';
    }
}

void CongestionMapBuilder::exportFloorplanMoveHintsCSV(const Design& design, const Step0CongestionMapResult& result, const string& path) const {
    ofstream fout(path);
    if (!fout) return;

    fout << "hintId,sourceActionId,blockIndex,blockName,blockType,targetEdge,moveDirection,suggestedDelta,confidence,"
        << "expectedRiskReduction,estimatedAreaPenalty,estimatedWirePenalty,actionEfficiency,relatedActionType,relatedConnections,reason\n";
    for (const auto& h : result.floorplanMoveHints) {
        fout << h.hintId << ',' << h.sourceActionId << ',' << h.blockIndex << ','
            << h.blockName << ',' << h.blockType << ',' << h.targetEdge << ','
            << h.moveDirection << ',' << h.suggestedDelta << ',' << h.confidence << ','
            << h.expectedRiskReduction << ',' << h.estimatedAreaPenalty << ',' << h.estimatedWirePenalty << ','
            << h.actionEfficiency << ',' << h.relatedActionType << ','
            << h.relatedConnectionsCSV << ',' << h.reason << '\n';
    }
}

static bool readPrevHealthValueStep0(const string& path, const string& key, double& value) {
    ifstream fin(path);
    if (!fin) return false;
    string header, row;
    if (!getline(fin, header) || !getline(fin, row)) return false;

    vector<string> hs;
    vector<string> vs;
    string cur;
    for (char c : header) {
        if (c == ',') { hs.push_back(cur); cur.clear(); }
        else cur.push_back(c);
    }
    hs.push_back(cur);
    cur.clear();
    for (char c : row) {
        if (c == ',') { vs.push_back(cur); cur.clear(); }
        else cur.push_back(c);
    }
    vs.push_back(cur);

    for (int i = 0; i < static_cast<int>(hs.size()) && i < static_cast<int>(vs.size()); ++i) {
        if (hs[i] != key) continue;
        try {
            value = stod(vs[i]);
            return true;
        }
        catch (...) {
            return false;
        }
    }
    return false;
}

void CongestionMapBuilder::exportFloorplanDeltaCSV(const Design& design, const Step0CongestionMapResult& result, const string& path) const {
    ofstream fout(path);
    if (!fout) return;

    double prevRisk = 0.0, prevMaxUtil = 0.0, prevWorstRegion = 0.0, prevFeedbackCost = 0.0;
    const bool okRisk = readPrevHealthValueStep0(opt_.previousFloorplanHealthCsvPath, "floorplanRiskScore", prevRisk);
    const bool okUtil = readPrevHealthValueStep0(opt_.previousFloorplanHealthCsvPath, "maxUtil", prevMaxUtil);
    const bool okRegion = readPrevHealthValueStep0(opt_.previousFloorplanHealthCsvPath, "worstRegionScore", prevWorstRegion);
    const bool okCost = readPrevHealthValueStep0(opt_.previousFloorplanHealthCsvPath, "floorplanFeedbackCost", prevFeedbackCost);
    const bool hasPrev = okRisk || okUtil || okRegion || okCost;

    const auto& h = result.floorplanHealth;
    fout << "hasPrevious,previousPath,prevRiskScore,currRiskScore,riskImprovement,"
        << "prevMaxUtil,currMaxUtil,maxUtilImprovement,"
        << "prevWorstRegionScore,currWorstRegionScore,regionImprovement,"
        << "prevFloorplanFeedbackCost,currFloorplanFeedbackCost,feedbackCostImprovement,status\n";
    fout << (hasPrev ? 1 : 0) << ',' << opt_.previousFloorplanHealthCsvPath << ','
        << (okRisk ? prevRisk : 0.0) << ',' << h.floorplanRiskScore << ',' << (okRisk ? prevRisk - h.floorplanRiskScore : 0.0) << ','
        << (okUtil ? prevMaxUtil : 0.0) << ',' << h.maxUtil << ',' << (okUtil ? prevMaxUtil - h.maxUtil : 0.0) << ','
        << (okRegion ? prevWorstRegion : 0.0) << ',' << h.worstRegionScore << ',' << (okRegion ? prevWorstRegion - h.worstRegionScore : 0.0) << ','
        << (okCost ? prevFeedbackCost : 0.0) << ',' << h.floorplanFeedbackCost << ',' << (okCost ? prevFeedbackCost - h.floorplanFeedbackCost : 0.0) << ','
        << (hasPrev ? "COMPARE_OK" : "NO_PREVIOUS_HEALTH_FILE") << '\n';
}



void CongestionMapBuilder::exportRisaSupplyDemandCSV(const Design& design, const Step0CongestionMapResult& result, const string& path) const {
    ofstream fout(path);
    if (!fout) return;

    fout << "channel,index,x,y,w,h,"
        << "capLR,capTB,effectiveCapLR,effectiveCapTB,supplyScaleLR,supplyScaleTB,supplyReductionLR,supplyReductionTB,"
        << "predLR,predTB,risaExtraDemandLR,risaExtraDemandTB,risaDemandLR,risaDemandTB,"
        << "risaUtilLR,risaUtilTB,risaOverflowLR,risaOverflowTB,risaCostLR,risaCostTB,risaCost,"
        << "dominantDirection,regionId,nearbyBlocks\n";

    for (int i = 0; i < static_cast<int>(design.channels.size()) && i < static_cast<int>(result.channelPressure.size()); ++i) {
        const Channel& ch = design.channels[i];
        const auto& p = result.channelPressure[i];
        string dom = "MIXED";
        if (p.risaUtilLR > p.risaUtilTB * 1.15) dom = "LR";
        else if (p.risaUtilTB > p.risaUtilLR * 1.15) dom = "TB";
        Rect box = expandRectClippedStep0(ch.rect, 1.0, design.outlineW, design.outlineH);
        fout << ch.name << ',' << i << ','
            << ch.rect.x << ',' << ch.rect.y << ',' << ch.rect.w << ',' << ch.rect.h << ','
            << p.capLR << ',' << p.capTB << ','
            << p.effectiveCapLR << ',' << p.effectiveCapTB << ','
            << p.supplyScaleLR << ',' << p.supplyScaleTB << ','
            << p.supplyReductionLR << ',' << p.supplyReductionTB << ','
            << p.predLR << ',' << p.predTB << ','
            << p.risaExtraDemandLR << ',' << p.risaExtraDemandTB << ','
            << p.risaDemandLR << ',' << p.risaDemandTB << ','
            << p.risaUtilLR << ',' << p.risaUtilTB << ','
            << p.risaOverflowLR << ',' << p.risaOverflowTB << ','
            << p.risaCostLR << ',' << p.risaCostTB << ',' << p.risaCost << ','
            << dom << ',' << p.regionId << ',' << joinBlockNamesNearRectStep0(design, box) << '\n';
    }
}



void CongestionMapBuilder::exportAmplifiedGuideCSV(const Design& design, const Step0CongestionMapResult& result, const string& path) const {
    ofstream fout(path);
    if (!fout) return;

    fout << "channel,index,x,y,w,h,"
        << "capLR,capTB,effectiveCapLR,effectiveCapTB,"
        << "staticDemandLR,staticDemandTB,staticUtilLR,staticUtilTB,"
        << "amplificationFactorLR,amplificationFactorTB,"
        << "amplifiedStaticDemandLR,amplifiedStaticDemandTB,"
        << "ambientDemandLR,ambientDemandTB,ambientUtilLR,ambientUtilTB,"
        << "routerCriticalityLR,routerCriticalityTB,routerCriticality,"
        << "generalCriticality,risaCost,regionId,interpretation\n";

    for (int i = 0; i < static_cast<int>(design.channels.size()) && i < static_cast<int>(result.channelPressure.size()); ++i) {
        const Channel& ch = design.channels[i];
        const auto& p = result.channelPressure[i];
        const double capLRv = safeCap(p.effectiveCapLR > STEP0_EPS ? p.effectiveCapLR : p.capLR);
        const double capTBv = safeCap(p.effectiveCapTB > STEP0_EPS ? p.effectiveCapTB : p.capTB);
        const double staticLR = max(p.predLR, p.risaDemandLR);
        const double staticTB = max(p.predTB, p.risaDemandTB);
        const double staticUtilLR = staticLR / capLRv;
        const double staticUtilTB = staticTB / capTBv;
        string interp = "LOW_IMPACT";
        if (p.routerCriticality >= 0.75) interp = "STRONGLY_AVOID_FOR_ROUTER";
        else if (p.routerCriticality >= 0.35) interp = "GUIDE_AVOID_IF_ALTERNATIVE_EXISTS";
        else if (p.ambientDemandLR > STEP0_EPS || p.ambientDemandTB > STEP0_EPS) interp = "AMBIENT_BACKGROUND_PRESSURE";

        fout << ch.name << ',' << i << ','
            << ch.rect.x << ',' << ch.rect.y << ',' << ch.rect.w << ',' << ch.rect.h << ','
            << p.capLR << ',' << p.capTB << ',' << p.effectiveCapLR << ',' << p.effectiveCapTB << ','
            << staticLR << ',' << staticTB << ',' << staticUtilLR << ',' << staticUtilTB << ','
            << p.amplificationFactorLR << ',' << p.amplificationFactorTB << ','
            << p.amplifiedStaticDemandLR << ',' << p.amplifiedStaticDemandTB << ','
            << p.ambientDemandLR << ',' << p.ambientDemandTB << ',' << p.ambientUtilLR << ',' << p.ambientUtilTB << ','
            << p.routerCriticalityLR << ',' << p.routerCriticalityTB << ',' << p.routerCriticality << ','
            << p.criticality << ',' << p.risaCost << ',' << p.regionId << ',' << interp << '\n';
    }
}


void CongestionMapBuilder::exportPatternPredictabilityCSV(const Design& design, const Step0CongestionMapResult& result, const string& path) const {
    ofstream fout(path);
    if (!fout) return;

    fout << "connIndex,src,dst,netCount,selectedPattern,preferredChannels,avoidChannels,"
        << "patternPredictabilityScore,couplingRiskScore,patternGuideStrength,"
        << "patternPredictabilityClass,patternGuideMode,guideConfidence,"
        << "bestPatternMaxUtil,hasProjectedPattern,bestPatternAvgUtil,healthyCandidateCount,lackOfAlternative,"
        << "openRiskType,openRiskScore,routePriority,reason\n";

    for (const auto& g : result.connectionGuide) {
        string srcName = "?";
        string dstName = "?";
        if (g.src >= 0 && g.src < static_cast<int>(design.blocks.size())) srcName = design.blocks[g.src].spec.name;
        if (g.dst >= 0 && g.dst < static_cast<int>(design.blocks.size())) dstName = design.blocks[g.dst].spec.name;

        fout << g.connectionIndex << ',' << srcName << ',' << dstName << ',' << g.netCount << ','
            << g.selectedPattern << ',' << g.preferredChannelsCSV << ',' << g.avoidChannelsCSV << ','
            << g.patternPredictabilityScore << ',' << g.couplingRiskScore << ',' << g.patternGuideStrength << ','
            << g.patternPredictabilityClass << ',' << g.patternGuideMode << ',' << g.guideConfidence << ','
            << g.bestPatternMaxUtil << ',' << (g.hasProjectedPattern ? 1 : 0) << ',' << g.bestPatternAvgUtil << ',' << g.healthyCandidateCount << ','
            << g.lackOfAlternative << ',' << g.openRiskType << ',' << g.openRiskScore << ','
            << g.routePriority << ',' << '"' << g.patternReason << '"' << '\n';
    }
}


void CongestionMapBuilder::exportRudyBackgroundCSV(const Design& design, const Step0CongestionMapResult& result, const string& path) const {
    ofstream fout(path);
    if (!fout) return;

    fout << "channel,index,x,y,w,h,capLR,capTB,effectiveCapLR,effectiveCapTB,"
        << "patternDemandLR,patternDemandTB,rudyDemandLR,rudyDemandTB,rudyUtilLR,rudyUtilTB,"
        << "hybridDemandLR,hybridDemandTB,hybridUtilLR,hybridUtilTB,hybridOverflowLR,hybridOverflowTB,hybridCost,"
        << "risaExtraDemandLR,risaExtraDemandTB,ambientDemandLR,ambientDemandTB,routerCriticality,regionId\n";
    for (int i = 0; i < static_cast<int>(design.channels.size()) && i < static_cast<int>(result.channelPressure.size()); ++i) {
        const auto& ch = design.channels[i];
        const auto& p = result.channelPressure[i];
        fout << ch.name << ',' << i << ','
            << ch.rect.x << ',' << ch.rect.y << ',' << ch.rect.w << ',' << ch.rect.h << ','
            << p.capLR << ',' << p.capTB << ',' << p.effectiveCapLR << ',' << p.effectiveCapTB << ','
            << p.predLR << ',' << p.predTB << ',' << p.rudyDemandLR << ',' << p.rudyDemandTB << ','
            << p.rudyUtilLR << ',' << p.rudyUtilTB << ','
            << p.hybridDemandLR << ',' << p.hybridDemandTB << ',' << p.hybridUtilLR << ',' << p.hybridUtilTB << ','
            << p.hybridOverflowLR << ',' << p.hybridOverflowTB << ',' << p.hybridCost << ','
            << p.risaExtraDemandLR << ',' << p.risaExtraDemandTB << ','
            << p.ambientDemandLR << ',' << p.ambientDemandTB << ',' << p.routerCriticality << ',' << p.regionId << '\n';
    }

    fout << "\nconnection,src,dst,netCount,rudyBBoxArea,rudyHpwl,rudyDensity,rudyContribution,rudyTouchedChannelCount,rudyTouchedChannels,selectedPattern,patternPredictabilityScore,guideMode\n";
    for (const auto& g : result.connectionGuide) {
        string srcName = "?";
        string dstName = "?";
        if (g.src >= 0 && g.src < static_cast<int>(design.blocks.size())) srcName = design.blocks[g.src].spec.name;
        if (g.dst >= 0 && g.dst < static_cast<int>(design.blocks.size())) dstName = design.blocks[g.dst].spec.name;
        fout << g.connectionIndex << ',' << srcName << ',' << dstName << ',' << g.netCount << ','
            << g.rudyBBoxArea << ',' << g.rudyHpwl << ',' << g.rudyDensity << ',' << g.rudyContribution << ','
            << g.rudyTouchedChannelCount << ',' << g.rudyTouchedChannelsCSV << ','
            << g.selectedPattern << ',' << g.patternPredictabilityScore << ',' << g.patternGuideMode << '\n';
    }
}

void CongestionMapBuilder::exportDashboardCSV(const Design& design, const Step0CongestionMapResult& result, const string& path) const {
    ofstream fout(path);
    if (!fout) return;
    const int K = max(1, opt_.dashboardTopK);
    fout << "section,rank,item,score,metricA,metricB,interpretation,details\n";

    auto emit = [&](const string& section, int rank, const string& item, double score,
        const string& metricA, const string& metricB,
        const string& interpretation, const string& details) {
            fout << csvEscapeStep0(section) << ',' << rank << ',' << csvEscapeStep0(item) << ',' << score << ','
                << csvEscapeStep0(metricA) << ',' << csvEscapeStep0(metricB) << ','
                << csvEscapeStep0(interpretation) << ',' << csvEscapeStep0(details) << '\n';
        };

    const auto& h = result.floorplanHealth;
    emit("OVERALL_HEALTH", 1, h.floorplanStatus, h.floorplanRiskScore,
        string("maxHybridUtil=") + fmt(h.maxHybridUtil),
        string("floorplanFeedbackCost=") + fmt(h.floorplanFeedbackCost),
        "Global floorplan routing-health status",
        string("regions=") + to_string(result.congestedRegions.size()) +
        " hotChannels=" + to_string(h.numHotChannels) +
        " highRiskConnections=" + to_string(h.numHighRiskConnections) +
        " overflowChannels=" + to_string(h.numOverflowChannels));

    struct ChannelScore { int ci; double score; string metricA; string metricB; string interp; };
    auto emitChannelTop = [&](const string& section, vector<ChannelScore> rows) {
        sort(rows.begin(), rows.end(), [](const ChannelScore& a, const ChannelScore& b) {
            if (fabs(a.score - b.score) > 1e-12) return a.score > b.score;
            return a.ci < b.ci;
            });
        int n = min(K, static_cast<int>(rows.size()));
        for (int i = 0; i < n; ++i) {
            const auto& r = rows[i];
            if (r.ci < 0 || r.ci >= static_cast<int>(design.channels.size())) continue;
            const Channel& ch = design.channels[r.ci];
            const auto& p = result.channelPressure[r.ci];
            string details = string("rect=(") + fmt(ch.rect.x) + ":" + fmt(ch.rect.y) + ":" + fmt(ch.rect.w) + ":" + fmt(ch.rect.h) + ")"
                + " region=R" + to_string(p.regionId)
                + " rawLR=" + fmt(p.utilLR) + " rawTB=" + fmt(p.utilTB)
                + " risaLR=" + fmt(p.risaUtilLR) + " risaTB=" + fmt(p.risaUtilTB)
                + " rudyLR=" + fmt(p.rudyUtilLR) + " rudyTB=" + fmt(p.rudyUtilTB)
                + " hybridLR=" + fmt(p.hybridUtilLR) + " hybridTB=" + fmt(p.hybridUtilTB)
                + " routerCrit=" + fmt(p.routerCriticality);
            emit(section, i + 1, ch.name, r.score, r.metricA, r.metricB, r.interp, details);
        }
        };

    vector<ChannelScore> rawRows, risaRows, rudyRows, hybridRows, routerRows, pinchRows;
    for (int ci = 0; ci < static_cast<int>(result.channelPressure.size()) && ci < static_cast<int>(design.channels.size()); ++ci) {
        const auto& p = result.channelPressure[ci];
        const double raw = max(p.utilLR, p.utilTB);
        const double risa = max(p.risaUtilLR, p.risaUtilTB);
        const double rudy = max(p.rudyUtilLR, p.rudyUtilTB);
        const double hybrid = max(p.hybridUtilLR, p.hybridUtilTB);
        const double router = p.routerCriticality;
        rawRows.push_back({ ci, raw, string("rawLR=") + fmt(p.utilLR), string("rawTB=") + fmt(p.utilTB), raw >= 1.0 ? "Raw overflow risk" : (raw >= 0.9 ? "Raw near-full" : "Raw pressure") });
        risaRows.push_back({ ci, risa, string("risaLR=") + fmt(p.risaUtilLR), string("risaTB=") + fmt(p.risaUtilTB), risa >= 1.0 ? "Effective-supply deficit" : "Supply-demand pressure" });
        rudyRows.push_back({ ci, rudy, string("rudyLR=") + fmt(p.rudyUtilLR), string("rudyTB=") + fmt(p.rudyUtilTB), "Router-independent background pressure" });
        hybridRows.push_back({ ci, hybrid, string("hybridLR=") + fmt(p.hybridUtilLR), string("hybridTB=") + fmt(p.hybridUtilTB), "Overall channel health signal" });
        routerRows.push_back({ ci, router, string("ambientLR=") + fmt(p.ambientUtilLR), string("ambientTB=") + fmt(p.ambientUtilTB), "Soft avoid signal for Router" });
        if (p.capLR < opt_.narrowComponentCap || p.capTB < opt_.narrowComponentCap || max(p.utilLR, p.utilTB) >= 3.0) {
            const double pinchScore = max(raw, max(p.capLR > 0 ? opt_.narrowComponentCap / safeCap(p.capLR) : 0.0,
                p.capTB > 0 ? opt_.narrowComponentCap / safeCap(p.capTB) : 0.0));
            pinchRows.push_back({ ci, pinchScore, string("capLR=") + fmt(p.capLR), string("capTB=") + fmt(p.capTB), "Possible narrow/sliver channel pinch" });
        }
    }
    emitChannelTop("TOP_RAW_HOT_CHANNELS", rawRows);
    emitChannelTop("TOP_RISA_SUPPLY_DEFICIT_CHANNELS", risaRows);
    emitChannelTop("TOP_RUDY_BACKGROUND_CHANNELS", rudyRows);
    emitChannelTop("TOP_HYBRID_HEALTH_CHANNELS", hybridRows);
    emitChannelTop("TOP_ROUTER_AVOID_CHANNELS", routerRows);
    emitChannelTop("TOP_PINCH_CANDIDATE_CHANNELS", pinchRows);

    vector<int> regionIdx(result.congestedRegions.size());
    for (int i = 0; i < static_cast<int>(regionIdx.size()); ++i) regionIdx[i] = i;
    sort(regionIdx.begin(), regionIdx.end(), [&](int a, int b) {
        const auto& ra = result.congestedRegions[a];
        const auto& rb = result.congestedRegions[b];
        if (fabs(ra.regionScore - rb.regionScore) > 1e-12) return ra.regionScore > rb.regionScore;
        return ra.regionId < rb.regionId;
        });
    for (int r = 0; r < min(K, static_cast<int>(regionIdx.size())); ++r) {
        const auto& rg = result.congestedRegions[regionIdx[r]];
        const string typ = regionVisualTypeStep0(rg);
        string details = string("bbox=(") + fmt(rg.bbox.x) + ":" + fmt(rg.bbox.y) + ":" + fmt(rg.bbox.w) + ":" + fmt(rg.bbox.h) + ")"
            + " peakRaw=" + fmt(rg.peakUtil)
            + " peakRISA=" + fmt(rg.peakRisaUtil)
            + " peakRUDY=" + fmt(rg.peakRudyUtil)
            + " peakHybrid=" + fmt(rg.peakHybridUtil)
            + " channels=" + rg.channelsCSV;
        emit("TOP_CONGESTED_REGIONS", r + 1, string("R") + to_string(rg.regionId) + " " + typ,
            rg.regionScore, string("peakHybrid=") + fmt(rg.peakHybridUtil), string("peakRISA=") + fmt(rg.peakRisaUtil),
            "Bottleneck region ranked by score", details);
    }

    vector<int> connIdx(result.connectionGuide.size());
    for (int i = 0; i < static_cast<int>(connIdx.size()); ++i) connIdx[i] = i;
    sort(connIdx.begin(), connIdx.end(), [&](int a, int b) {
        const auto& ga = result.connectionGuide[a];
        const auto& gb = result.connectionGuide[b];
        if (fabs(ga.openRiskScore - gb.openRiskScore) > 1e-12) return ga.openRiskScore > gb.openRiskScore;
        return ga.connectionIndex < gb.connectionIndex;
        });
    for (int r = 0; r < min(K, static_cast<int>(connIdx.size())); ++r) {
        const auto& g = result.connectionGuide[connIdx[r]];
        string srcName = (g.src >= 0 && g.src < static_cast<int>(design.blocks.size())) ? design.blocks[g.src].spec.name : "?";
        string dstName = (g.dst >= 0 && g.dst < static_cast<int>(design.blocks.size())) ? design.blocks[g.dst].spec.name : "?";
        string details = string("netCount=") + to_string(g.netCount) + " pattern=" + g.selectedPattern
            + " guideMode=" + g.patternGuideMode + " preferred=" + g.preferredChannelsCSV
            + " avoid=" + g.avoidChannelsCSV + " commonBottleneck=" + g.commonBottleneckChannelsCSV;
        emit("TOP_OPEN_RISK_CONNECTIONS", r + 1, srcName + "->" + dstName, g.openRiskScore,
            string("type=") + g.openRiskType, string("priority=") + fmt(g.routePriority),
            "Connection likely to open, fallback, or cause bottleneck", details);
    }

    sort(connIdx.begin(), connIdx.end(), [&](int a, int b) {
        const auto& ga = result.connectionGuide[a];
        const auto& gb = result.connectionGuide[b];
        if (fabs(ga.routePriority - gb.routePriority) > 1e-12) return ga.routePriority > gb.routePriority;
        return ga.connectionIndex < gb.connectionIndex;
        });
    for (int r = 0; r < min(K, static_cast<int>(connIdx.size())); ++r) {
        const auto& g = result.connectionGuide[connIdx[r]];
        string srcName = (g.src >= 0 && g.src < static_cast<int>(design.blocks.size())) ? design.blocks[g.src].spec.name : "?";
        string dstName = (g.dst >= 0 && g.dst < static_cast<int>(design.blocks.size())) ? design.blocks[g.dst].spec.name : "?";
        emit("TOP_ROUTE_PRIORITY_CONNECTIONS", r + 1, srcName + "->" + dstName, g.routePriority,
            string("netCount=") + to_string(g.netCount), string("guide=") + g.patternGuideMode,
            "Suggested early routing order", string("selected=") + g.selectedPattern + " openRisk=" + fmt(g.openRiskScore));
    }

    vector<int> actionIdx(result.floorplanActions.size());
    for (int i = 0; i < static_cast<int>(actionIdx.size()); ++i) actionIdx[i] = i;
    sort(actionIdx.begin(), actionIdx.end(), [&](int a, int b) {
        const auto& aa = result.floorplanActions[a];
        const auto& ab = result.floorplanActions[b];
        const double sa = max(aa.severity, aa.actionEfficiency);
        const double sb = max(ab.severity, ab.actionEfficiency);
        if (fabs(sa - sb) > 1e-12) return sa > sb;
        return aa.actionId < ab.actionId;
        });
    for (int r = 0; r < min(K, static_cast<int>(actionIdx.size())); ++r) {
        const auto& a = result.floorplanActions[actionIdx[r]];
        emit("TOP_FLOORPLAN_ACTIONS", r + 1, string("A") + to_string(a.actionId) + " " + a.actionType,
            max(a.severity, a.actionEfficiency), string("severity=") + fmt(a.severity), string("efficiency=") + fmt(a.actionEfficiency),
            "Floorplanner action hint", string("bbox=(") + fmt(a.bbox.x) + ":" + fmt(a.bbox.y) + ":" + fmt(a.bbox.w) + ":" + fmt(a.bbox.h) + ") blocks=" + a.nearbyBlocksCSV + " reason=" + a.reason);
    }

    vector<int> edgeIdx(result.blockEdgePressure.size());
    for (int i = 0; i < static_cast<int>(edgeIdx.size()); ++i) edgeIdx[i] = i;
    sort(edgeIdx.begin(), edgeIdx.end(), [&](int a, int b) {
        const auto& ea = result.blockEdgePressure[a];
        const auto& eb = result.blockEdgePressure[b];
        if (fabs(ea.severity - eb.severity) > 1e-12) return ea.severity > eb.severity;
        if (fabs(ea.edgePressure - eb.edgePressure) > 1e-12) return ea.edgePressure > eb.edgePressure;
        return ea.blockIndex < eb.blockIndex;
        });
    for (int r = 0; r < min(K, static_cast<int>(edgeIdx.size())); ++r) {
        const auto& e = result.blockEdgePressure[edgeIdx[r]];
        emit("TOP_BLOCK_EDGE_ACCESS_PRESSURE", r + 1, e.blockName + "." + e.edgeName,
            e.severity, string("edgePressure=") + fmt(e.edgePressure),
            string("healthyAccess=") + to_string(e.healthyAccessChannelCount) + "/" + to_string(e.accessChannelCount),
            "Block-side access risk", string("suggest=") + e.suggestedAction + " move=" + e.suggestedMoveDirection + " channels=" + e.accessChannelsCSV);
    }
}

void CongestionMapBuilder::exportSignalConflictCSV(const Design& design, const Step0CongestionMapResult& result, const string& path) const {
    ofstream fout(path);
    if (!fout) return;
    fout << "level,item,conflictType,severity,rawUtil,risaUtil,rudyUtil,hybridUtil,routerCriticality,interpretation,recommendedCheck\n";
    auto emit = [&](const string& level, const string& item, const string& type, double sev,
        double raw, double risa, double rudy, double hybrid, double router,
        const string& interp, const string& check) {
            fout << csvEscapeStep0(level) << ',' << csvEscapeStep0(item) << ',' << csvEscapeStep0(type) << ',' << sev << ','
                << raw << ',' << risa << ',' << rudy << ',' << hybrid << ',' << router << ','
                << csvEscapeStep0(interp) << ',' << csvEscapeStep0(check) << '\n';
        };

    for (int ci = 0; ci < static_cast<int>(result.channelPressure.size()) && ci < static_cast<int>(design.channels.size()); ++ci) {
        const auto& p = result.channelPressure[ci];
        const string item = design.channels[ci].name;
        const double raw = max(p.utilLR, p.utilTB);
        const double risa = max(p.risaUtilLR, p.risaUtilTB);
        const double rudy = max(p.rudyUtilLR, p.rudyUtilTB);
        const double hybrid = max(p.hybridUtilLR, p.hybridUtilTB);
        const double router = p.routerCriticality;
        if (raw < opt_.conflictRawLowThreshold && risa >= opt_.conflictModelHotThreshold) {
            emit("CHANNEL", item, "RISA_HOT_BUT_RAW_LOW", risa - raw, raw, risa, rudy, hybrid, router,
                "Raw capacity looks safe, but effective supply / detour model says this channel is risky.",
                "Check effectiveCap, hard/edge adjacency, risaExtraDemand, and PINCH/RISA region type.");
        }
        if (raw < opt_.conflictRawLowThreshold && rudy >= opt_.conflictModelHotThreshold) {
            emit("CHANNEL", item, "RUDY_HOT_BUT_RAW_LOW", rudy - raw, raw, risa, rudy, hybrid, router,
                "Pattern routes do not heavily use this channel, but bounding-box background demand is high.",
                "Use for floorplan whitespace planning, not as raw overflow evidence.");
        }
        if (raw < opt_.conflictRawLowThreshold && router >= opt_.conflictRouterCriticalityThreshold) {
            emit("CHANNEL", item, "ROUTER_GUIDE_HOT_BUT_RAW_LOW", router - raw, raw, risa, rudy, hybrid, router,
                "Router is advised to avoid this channel even though raw pattern utilization is not high.",
                "Check RISA/Hybrid/ambientDemand. This is a soft guide, not capacity overflow.");
        }
        if (raw >= opt_.conflictRawNearFullThreshold && router < 0.20) {
            emit("CHANNEL", item, "RAW_NEAR_FULL_BUT_ROUTER_GUIDE_LOW", raw - router, raw, risa, rudy, hybrid, router,
                "Raw usage is high but amplified router criticality remains low.",
                "Check amplification thresholds and whether ambient cap/decay suppresses this channel.");
        }
        if (hybrid >= opt_.conflictModelHotThreshold && raw < opt_.conflictRawLowThreshold) {
            emit("CHANNEL", item, "HYBRID_HOT_BUT_RAW_LOW", hybrid - raw, raw, risa, rudy, hybrid, router,
                "Overall floorplan health signal is high but raw utilization is low.",
                "Likely RUDY/RISA-driven; treat as planning warning rather than immediate overflow.");
        }
    }

    for (const auto& g : result.connectionGuide) {
        string srcName = (g.src >= 0 && g.src < static_cast<int>(design.blocks.size())) ? design.blocks[g.src].spec.name : "?";
        string dstName = (g.dst >= 0 && g.dst < static_cast<int>(design.blocks.size())) ? design.blocks[g.dst].spec.name : "?";
        string item = srcName + "->" + dstName;
        const double pseudoRaw = g.hasProjectedPattern ? g.bestPatternMaxUtil : 0.0;
        if (g.patternGuideMode.find("ROUTER_FREE") != string::npos && g.routePriority >= 1000.0) {
            emit("CONNECTION", item, "HIGH_PRIORITY_BUT_ROUTER_FREE", g.routePriority / 1000.0,
                pseudoRaw, 0.0, 0.0, 0.0, 0.0,
                "Connection is important/difficult, but pattern guide is unreliable.",
                "Route early, but do not force preferred pattern. Let Dijkstra explore freely.");
        }
        if (g.patternPredictabilityScore >= 0.70 && g.openRiskScore >= 0.60) {
            emit("CONNECTION", item, "PREDICTABLE_PATTERN_BUT_OPEN_RISK", g.openRiskScore,
                pseudoRaw, 0.0, 0.0, 0.0, 0.0,
                "Pattern seems reliable, but endpoint/capacity/connectivity risk remains high.",
                "Check endpoint access, shared bottleneck, and preferred/avoid channels.");
        }
    }
}

void CongestionMapBuilder::exportTextReport(const Design& design, const Step0CongestionMapResult& result, const string& path) const {
    ofstream fout(path);
    if (!fout) return;
    fout << fixed << setprecision(3);
    const auto& h = result.floorplanHealth;
    fout << "CMB OVERALL REPORT\n";
    fout << "==================\n";
    fout << "Purpose: summarize Raw / RISA / RUDY / Hybrid / Router-guide signals without adding new CMB models.\n\n";
    fout << "1) Global floorplan health\n";
    fout << "   status=" << h.floorplanStatus << " riskScore=" << h.floorplanRiskScore
        << " feedbackCost=" << h.floorplanFeedbackCost << " worstRegionScore=" << h.worstRegionScore << "\n";
    fout << "   raw: maxRawUtil=" << h.maxUtil << " hotChannels=" << h.numHotChannels
        << " nearFull=" << h.numNearFullChannels << " overflow=" << h.numOverflowChannels << "\n";
    fout << "   risa: maxRisaUtil=" << h.maxRisaUtil << " supplyDemandCost=" << h.risaSupplyDemandCost
        << " supplyDeficitChannels=" << h.numRisaSupplyDeficitChannels << "\n";
    fout << "   rudy: maxRudyUtil=" << h.maxRudyUtil << " totalRudyDemand=" << h.totalRudyDemand
        << " rudyHotChannels=" << h.numRudyHotChannels << "\n";
    fout << "   hybrid: maxHybridUtil=" << h.maxHybridUtil << " hybridCost=" << h.hybridDemandCost
        << " hybridHotChannels=" << h.numHybridHotChannels << "\n";
    fout << "   model: maxModelUtil=" << result.maxModelUtil << "\n";
    fout << "   routerGuide: maxRouterCriticality=" << h.maxRouterCriticality
        << " maxAmbientUtil=" << h.maxAmbientUtil << " amplifiedChannels=" << h.numAmplifiedChannels << "\n\n";

    auto topChannels = [&](const string& title, char mode) {
        struct Row { int ci; double score; };
        vector<Row> rows;
        for (int ci = 0; ci < static_cast<int>(result.channelPressure.size()) && ci < static_cast<int>(design.channels.size()); ++ci) {
            const auto& p = result.channelPressure[ci];
            double v = 0.0;
            if (mode == 'W') v = max(p.utilLR, p.utilTB);
            else if (mode == 'S') v = max(p.risaUtilLR, p.risaUtilTB);
            else if (mode == 'B') v = max(p.rudyUtilLR, p.rudyUtilTB);
            else if (mode == 'H') v = max(p.hybridUtilLR, p.hybridUtilTB);
            else if (mode == 'R') v = p.routerCriticality;
            rows.push_back({ ci, v });
        }
        sort(rows.begin(), rows.end(), [](const Row& a, const Row& b) { return a.score > b.score; });
        fout << title << "\n";
        int n = min(max(1, opt_.dashboardTopK), static_cast<int>(rows.size()));
        for (int i = 0; i < n; ++i) {
            int ci = rows[i].ci;
            const auto& p = result.channelPressure[ci];
            fout << "   #" << (i + 1) << " " << design.channels[ci].name
                << " score=" << rows[i].score
                << " raw=" << max(p.utilLR, p.utilTB)
                << " risa=" << max(p.risaUtilLR, p.risaUtilTB)
                << " rudy=" << max(p.rudyUtilLR, p.rudyUtilTB)
                << " hybrid=" << max(p.hybridUtilLR, p.hybridUtilTB)
                << " routerCrit=" << p.routerCriticality
                << " region=" << p.regionId << "\n";
        }
        fout << "\n";
        };
    fout << "2) Top channel lists\n";
    topChannels("   2.1 Top raw-utilization channels", 'W');
    topChannels("   2.2 Top RISA supply-demand channels", 'S');
    topChannels("   2.3 Top RUDY background-demand channels", 'B');
    topChannels("   2.4 Top hybrid-health channels", 'H');
    topChannels("   2.5 Top Router-guide avoid channels", 'R');

    fout << "3) Top congested regions\n";
    vector<int> regionIdx(result.congestedRegions.size());
    for (int i = 0; i < static_cast<int>(regionIdx.size()); ++i) regionIdx[i] = i;
    sort(regionIdx.begin(), regionIdx.end(), [&](int a, int b) { return result.congestedRegions[a].regionScore > result.congestedRegions[b].regionScore; });
    for (int r = 0; r < min(max(1, opt_.dashboardTopK), static_cast<int>(regionIdx.size())); ++r) {
        const auto& rg = result.congestedRegions[regionIdx[r]];
        fout << "   #" << (r + 1) << " R" << rg.regionId << " type=" << regionVisualTypeStep0(rg)
            << " score=" << rg.regionScore << " peakRaw=" << rg.peakUtil
            << " peakRISA=" << rg.peakRisaUtil << " peakRUDY=" << rg.peakRudyUtil
            << " peakHybrid=" << rg.peakHybridUtil << " channels=" << rg.channelsCSV << "\n";
    }
    fout << "\n";

    fout << "4) Interpretation guide\n";
    fout << "   Raw high: actual pattern demand is close to raw channel capacity.\n";
    fout << "   RISA high: effective supply is insufficient or blockage/detour pressure exists.\n";
    fout << "   RUDY high: router-independent background wiring pressure exists; use for floorplan whitespace planning.\n";
    fout << "   Hybrid high: combined floorplan/channel health is poor.\n";
    fout << "   RouterCrit high: CMB wants Router to avoid this channel as a soft cost guide.\n";
    fout << "   If signals disagree, read step0_signal_conflicts.csv before changing cost weights.\n";
}

void CongestionMapBuilder::exportSVG(const Design& design, const Step0CongestionMapResult& result, const string& path) const {
    ofstream svg(path);
    if (!svg) return;

    const double W = max(1.0, design.outlineW);
    const double H = max(1.0, design.outlineH);
    const double marginBottom = 360.0;

    auto sy = [&](const Rect& r) {
        return H - r.y - r.h;
        };

    // -------------------------------------------------------------------------
    // 視覺化重點：
    //   Router 用的 util / criticality 仍然是「絕對值」。
    //   但如果整張 map 都很健康，例如 raw max util 只有 0.18，直接用絕對色階會全部是綠色，
    //   肉眼幾乎看不出哪個 channel 較熱。
    //
    //   因此 SVG 顏色採用「顯示用放大倍率 visualScale」：
    //      displayUtil = actualUtil * visualScale
    //   這只影響圖片顏色，不影響 CSV、Router criticality、正式 routing。
    //   actual util 仍然會印在 label 與 CSV 裡。
    // -------------------------------------------------------------------------
    double visualScale = 1.0;
    const double actualRawMaxUtil = result.maxRawUtil > EPS ? result.maxRawUtil : result.maxUtil;
    if (actualRawMaxUtil > EPS && actualRawMaxUtil < 0.75) {
        visualScale = min(8.0, 0.75 / actualRawMaxUtil);
    }

    // Route-open analysis footprint：
    //   open_conn   = 高 open-risk connection 經過(或共瓶頸)channel 的次數。
    //   open_channel= 高 open-risk connection 的 weighted risk 投影到 channel。
    // 這兩個值只用於 SVG 分析模式，不回寫 Router 成本。
    vector<double> openRiskChannelWeight(result.channelPressure.size(), 0.0);
    vector<int> openRiskChannelTouch(result.channelPressure.size(), 0);
    double maxOpenRiskChannelWeight = 0.0;
    int maxOpenRiskChannelTouch = 0;

    for (const auto& g : result.connectionGuide) {
        const bool highOpenRisk = (g.openRiskType != "OK" || g.openRiskScore >= 0.60);
        if (!highOpenRisk) continue;

        set<int> touched;
        for (int ci : g.preferredChannelIndices) {
            if (ci >= 0 && ci < static_cast<int>(openRiskChannelWeight.size())) touched.insert(ci);
        }
        for (int ci : g.commonBottleneckChannelIndices) {
            if (ci >= 0 && ci < static_cast<int>(openRiskChannelWeight.size())) touched.insert(ci);
        }
        // 若 preferred/common 都沒有，退回 avoid list 讓 open-risk 仍有可視化 footprint。
        if (touched.empty()) {
            for (int ci : g.avoidChannelIndices) {
                if (ci >= 0 && ci < static_cast<int>(openRiskChannelWeight.size())) touched.insert(ci);
            }
        }
        if (touched.empty()) continue;

        double riskWeight = max(0.0, g.openRiskScore) * static_cast<double>(max(1, g.netCount));
        if (g.openRiskType == "GEOMETRY_DISCONNECTED" || g.openRiskType == "ENDPOINT_ACCESS_RISK") riskWeight *= 1.20;
        else if (g.openRiskType == "ENDPOINT_BOTTLENECK_RISK") riskWeight *= 1.10;

        const double each = riskWeight / static_cast<double>(max<size_t>(1, touched.size()));
        for (int ci : touched) {
            openRiskChannelWeight[ci] += each;
            openRiskChannelTouch[ci] += 1;
            maxOpenRiskChannelWeight = max(maxOpenRiskChannelWeight, openRiskChannelWeight[ci]);
            maxOpenRiskChannelTouch = max(maxOpenRiskChannelTouch, openRiskChannelTouch[ci]);
        }
    }
    svg << "<?xml version='1.0' encoding='UTF-8'?>\n";
    svg << "<svg xmlns='http://www.w3.org/2000/svg' width='" << W << "' height='" << (H + marginBottom)
        << "' viewBox='0 0 " << W << " " << (H + marginBottom) << "'>\n";
    svg << "<rect x='0' y='0' width='" << W << "' height='" << H
        << "' fill='white' stroke='black' stroke-width='1'/>\n";
    svg << "<defs><marker id='arrowHead' markerWidth='8' markerHeight='8' refX='7' refY='3' orient='auto' markerUnits='strokeWidth'>"
        << "<path d='M0,0 L0,6 L7,3 z' fill='#252525'/></marker></defs>\n";

    // V7.5 interactive visual refinement:
    // Hot region / action box 預設用深色塊與粗 outline 強調；滑鼠移到其 bbox 上時只有色塊淡化，
    // label 保持不透明。滑鼠停在 label 文字/透明 hitbox 上時顯示 title 詳細資訊。
    svg << "<style><![CDATA[\n"
        // 三層互動視覺規則：\n"
        //   1) WIDEN_REGION / floorplan action 是最高層，深色顯示，但 pointer-events:none，\n"
        //      不擋住底下 channel / flyline / movehint 的滑鼠 title。\n"
        //   2) Hotspot region 是第二層，深色顯示，同樣不擋底下物件。\n"
        //   3) JS 會依滑鼠座標判斷是否在 action / hotspot bbox 內，若在其中就把該層淡化，\n"
        //      讓使用者可以看清楚底下 block / channel。\n"
        << ".step0-hot-region{pointer-events:all;}\n"
        << ".step0-hot-region .hotspot-fill{transition:opacity .14s ease, stroke-opacity .14s ease; pointer-events:none; opacity:0.62;}\n"
        << ".step0-hot-region .hotspot-label{transition:opacity .14s ease; pointer-events:none; opacity:0.98;}\n"
        << ".step0-hot-region .hotspot-label-hit{pointer-events:all; cursor:help;}\n"
        << ".step0-hot-region.hovered .hotspot-fill{opacity:0.07 !important;}\n"
        << ".step0-hot-region.hovered .hotspot-label{opacity:0.98 !important;}\n"
        << ".step0-action-box-group{pointer-events:all;}\n"
        << ".step0-action-box-group .action-fill{transition:opacity .14s ease, stroke-opacity .14s ease; pointer-events:none; opacity:0.56;}\n"
        << ".step0-action-box-group .action-label{transition:opacity .14s ease; pointer-events:none; opacity:0.98;}\n"
        << ".step0-action-box-group .action-label-hit{pointer-events:all; cursor:help;}\n"
        << ".step0-action-box-group.hovered .action-fill{opacity:0.07 !important;}\n"
        << ".step0-action-box-group.hovered .action-label{opacity:0.98 !important;}\n"
        << ".step0-movehint{pointer-events:none;}\n"
        << ".step0-flyline{pointer-events:stroke; transition:stroke-width .12s ease, stroke-opacity .12s ease;}\n"
        << ".step0-block-label{pointer-events:all; cursor:help;}\n"
        << ".step0-block-label text{pointer-events:none;}\n"
        << ".step0-block-info{pointer-events:none;}\n"
        << ".step0-block-outline{pointer-events:none;}\n"
        << ".step0-control-panel{font-family:Arial, sans-serif; font-size:13px; color:#111;}\n"
        << ".step0-control-panel label{margin-right:14px; white-space:nowrap;}\n"
        << ".step0-control-panel select{font-size:13px; padding:2px 6px;}\n"
        << ".step0-ui-note{font-size:12px; color:#555; margin-top:4px;}\n"
        << "]]></style>\n";
    svg << "<script><![CDATA[\n"
        << "(function(){\n"
        << "  var svg=document.currentScript.ownerSVGElement;\n"
        << "  function svgPoint(evt){var p=svg.createSVGPoint();p.x=evt.clientX;p.y=evt.clientY;return p.matrixTransform(svg.getScreenCTM().inverse());}\n"
        << "  function update(sel,p){var nodes=svg.querySelectorAll(sel);for(var i=0;i<nodes.length;i++){var g=nodes[i];var x=parseFloat(g.getAttribute('data-x'));var y=parseFloat(g.getAttribute('data-y'));var w=parseFloat(g.getAttribute('data-w'));var h=parseFloat(g.getAttribute('data-h'));var inside=p.x>=x&&p.x<=x+w&&p.y>=y&&p.y<=y+h; if(inside) g.classList.add('hovered'); else g.classList.remove('hovered');}}\n"
        << "  function setVisible(sel,on){var nodes=svg.querySelectorAll(sel);for(var i=0;i<nodes.length;i++){nodes[i].style.display=on?'':'none';}}\n"
        << "  var modeDesc={\n"
        << "    raw:'<b>Raw utilization / &#21407;&#22987;&#20351;&#29992;&#29575;</b>: channel color uses only max(utilLR, utilTB). Best for checking whether a channel really overflows before diagnostic penalties.',\n"
        << "    hybrid:'<b>Hybrid health / &#32156;&#21512;&#20581;&#24247;&#24230;</b>: combines pattern demand, RUDY background demand, and RISA extra detour demand. Best for floorplan-level health.',\n"
        << "    risa:'<b>RISA supply-demand / &#20379;&#38656;&#24179;&#34913;</b>: highlights effective supply shortage and hard/edge-block detour pressure. Purple/red means supply deficit, not necessarily raw overflow.',\n"
        << "    rudy:'<b>RUDY background / &#32972;&#26223;&#38656;&#27714;</b>: shows router-independent rectangular uniform demand. Useful for finding broad routing-pressure regions not tied to one pattern.',\n"
        << "    router:'<b>Router guide / Router &#24341;&#23566;</b>: shows ambientDemand and routerCriticality used by Dijkstra cost. High color means CMB wants Router to avoid if possible.',\n"
        << "    open_conn:'<b>Route-open connection focus / Open &#36899;&#32218;&#28966;&#40670;</b>: keeps channels in a neutral context and emphasizes high open-risk flylines by severity. Use this to inspect which connections are likely to fail before detailed routing.',\n"
        << "    open_channel:'<b>Route-open channel projection / Open channel &#29105;&#21312;</b>: projects connection open-risk onto channels to highlight likely failure corridors. This is a risk footprint, not actual overflow proof.',\n"
        << "    diagnostic:'<b>Diagnostic mix / &#28151;&#21512;&#35386;&#26039;</b>: legacy mixed visualization using all signals with visual amplification. Good for quick inspection, but not raw overflow semantics.'\n"
        << "  };\n"
        << "  window.step0ToggleLayer=function(layer,on){if(layer==='hotspot')setVisible('.step0-hot-region',on);else if(layer==='action')setVisible('.step0-action-box-group',on);else if(layer==='movehint'){setVisible('.step0-movehint',on);setVisible('.step0-block-move-info',on);}else if(layer==='flyline')setVisible('.step0-flyline',on);else if(layer==='channelLabel')setVisible('.step0-channel-label',on);else if(layer==='blockLabel')setVisible('.step0-block-label',on);else if(layer==='blockAccess'){setVisible('.step0-block-access-pressure',on);setVisible('.step0-block-edge-info',on);}};\n"
        << "  window.step0SetChannelMode=function(mode){var nodes=svg.querySelectorAll('.step0-channel');for(var i=0;i<nodes.length;i++){var v=nodes[i].getAttribute('data-fill-'+mode);if(v)nodes[i].setAttribute('fill',v);}var lines=svg.querySelectorAll('.step0-flyline');for(var j=0;j<lines.length;j++){var ln=lines[j];var w=(mode===\"open_conn\")?ln.getAttribute('data-open-width'):ln.getAttribute('data-default-width');var o=(mode===\"open_conn\")?ln.getAttribute('data-open-opacity'):ln.getAttribute('data-default-opacity');if(w)ln.setAttribute('stroke-width',w);if(o)ln.setAttribute('stroke-opacity',o);}var t=svg.querySelector('#step0-current-mode');if(t)t.textContent=mode;var d=svg.querySelector('#step0-mode-description');if(d)d.innerHTML=modeDesc[mode]||'';};\n"
        << "  svg.addEventListener('mousemove',function(evt){var p=svgPoint(evt);update('.step0-hot-region',p);update('.step0-action-box-group',p);});\n"
        << "  svg.addEventListener('mouseleave',function(){var nodes=svg.querySelectorAll('.hovered');for(var i=0;i<nodes.length;i++)nodes[i].classList.remove('hovered');});\n"
        << "})();\n"
        << "]]></script>\n";

    // 先畫 block 底色；channel heatmap 之後會畫在上層。
    // V5：block stroke 先保留，但最後還會再補一層粗 outline，避免 channel 疊上來後不好辨識。
    for (const auto& b : design.blocks) {
        string fill = "#bdbdbd";
        if (b.spec.type == BlockType::SOFT) fill = "#9ecae1";
        else if (b.spec.type == BlockType::HARD) fill = "#f4a6a6";
        else if (b.spec.type == BlockType::EDGE) fill = "#a1d99b";

        svg << "<rect x='" << b.rect.x << "' y='" << sy(b.rect) << "' width='" << b.rect.w << "' height='" << b.rect.h
            << "' fill='" << fill << "' fill-opacity='0.50' stroke='black' stroke-width='1.0'/>\n";
    }

    // 畫 channel heatmap。實際 congestion 使用 max(utilLR, utilTB)，顏色用 displayUtil 放大後呈現。
    for (int i = 0; i < static_cast<int>(design.channels.size()) && i < static_cast<int>(result.channelPressure.size()); ++i) {
        const Channel& ch = design.channels[i];
        const Step0ChannelPressure& p = result.channelPressure[i];
        const double actualUtil = max(max(max(p.utilLR, p.utilTB), max(p.risaUtilLR, p.risaUtilTB)),
            max(max(p.rudyUtilLR, p.rudyUtilTB),
                max(max(p.hybridUtilLR, p.hybridUtilTB), max(p.ambientUtilLR, p.ambientUtilTB))));
        const double displayUtil = actualUtil * visualScale;
        const double strokeW = actualUtil >= 1.0 ? 2.0 : 0.9;
        const string stroke = actualUtil >= 1.0 ? string("#7f0000") : string("#111111");

        const string clipId = string("step0_clip_ch_") + to_string(i);
        const double rx = ch.rect.x;
        const double ry = sy(ch.rect);

        // 每個 channel 都附上 <title>，即使 label 因為 channel 太窄被裁切，
        // 在瀏覽器中滑鼠移到 channel 上仍可看到完整數值。
        const double rawModeUtil = max(p.utilLR, p.utilTB);
        const double risaModeUtil = max(p.risaUtilLR, p.risaUtilTB);
        const double rudyModeUtil = max(p.rudyUtilLR, p.rudyUtilTB);
        const double hybridModeUtil = max(p.hybridUtilLR, p.hybridUtilTB);
        const double routerModeUtil = max(p.routerCriticality, max(p.ambientUtilLR, p.ambientUtilTB));
        const double diagModeUtil = actualUtil * visualScale;

        const double openConnRel = (maxOpenRiskChannelTouch > 0) ? static_cast<double>(openRiskChannelTouch[i]) / static_cast<double>(maxOpenRiskChannelTouch) : 0.0;
        const double openConnModeUtil = min(1.6, 1.6 * pow(clampStep0(openConnRel, 0.0, 1.0), 0.75));
        const double openChannelRel = (maxOpenRiskChannelWeight > STEP0_EPS) ? (openRiskChannelWeight[i] / maxOpenRiskChannelWeight) : 0.0;
        const double openChannelModeUtil = min(1.6, 1.6 * sqrt(clampStep0(openChannelRel, 0.0, 1.0)));

        const string channelTitle =
            ch.name + string("\n") +
            string("RAW: LR=") + fmt(p.utilLR) + string(" TB=") + fmt(p.utilTB) + string(" RawMax=") + fmt(rawModeUtil) + string("\n") +
            string("  predLR=") + fmt(p.predLR) + string(" predTB=") + fmt(p.predTB) +
            string(" capLR=") + fmt(p.capLR) + string(" capTB=") + fmt(p.capTB) + string("\n") +
            string("RISA: LR=") + fmt(p.risaUtilLR) + string(" TB=") + fmt(p.risaUtilTB) + string(" RisaMax=") + fmt(risaModeUtil) +
            string(" risaCost=") + fmt(p.risaCost) + string("\n") +
            string("  effLR=") + fmt(p.effectiveCapLR) + string(" effTB=") + fmt(p.effectiveCapTB) +
            string(" extraLR=") + fmt(p.risaExtraDemandLR) + string(" extraTB=") + fmt(p.risaExtraDemandTB) + string("\n") +
            string("RUDY: LR=") + fmt(p.rudyUtilLR) + string(" TB=") + fmt(p.rudyUtilTB) + string(" RudyMax=") + fmt(rudyModeUtil) + string("\n") +
            string("  rudyDemandLR=") + fmt(p.rudyDemandLR) + string(" rudyDemandTB=") + fmt(p.rudyDemandTB) + string("\n") +
            string("HYBRID: LR=") + fmt(p.hybridUtilLR) + string(" TB=") + fmt(p.hybridUtilTB) + string(" HybridMax=") + fmt(hybridModeUtil) +
            string(" hybridCost=") + fmt(p.hybridCost) + string("\n") +
            string("ROUTER GUIDE: ambientLR=") + fmt(p.ambientDemandLR) + string(" ambientTB=") + fmt(p.ambientDemandTB) +
            string(" ambientUtilLR=") + fmt(p.ambientUtilLR) + string(" ambientUtilTB=") + fmt(p.ambientUtilTB) +
            string(" routerCritLR=") + fmt(p.routerCriticalityLR) + string(" routerCritTB=") + fmt(p.routerCriticalityTB) +
            string(" routerCrit=") + fmt(p.routerCriticality) + string("\n") +
            string("OPEN_CONN: touchCount=") + to_string(openRiskChannelTouch[i]) +
            string(" rel=") + fmt(openConnRel) + string("\n") +
            string("OPEN_CHANNEL: weightedScore=") + fmt(openRiskChannelWeight[i]) +
            string(" rel=") + fmt(openChannelRel) + string("\n") +
            string("REGION: regionId=") + to_string(p.regionId);

        svg << "<rect class='step0-channel' x='" << rx << "' y='" << ry << "' width='" << ch.rect.w << "' height='" << ch.rect.h
            << "' data-fill-raw='" << colorForUtil(rawModeUtil) << "' data-fill-risa='" << colorForUtil(risaModeUtil)
            << "' data-fill-rudy='" << colorForUtil(rudyModeUtil) << "' data-fill-hybrid='" << colorForUtil(hybridModeUtil)
            << "' data-fill-router='" << colorForUtil(routerModeUtil) << "' data-fill-open_conn='" << colorForUtil(openConnModeUtil)
            << "' data-fill-open_channel='" << colorForUtil(openChannelModeUtil) << "' data-fill-diagnostic='" << colorForUtil(diagModeUtil)
            << "' fill='" << colorForUtil(rawModeUtil) << "' fill-opacity='0.88' stroke='" << stroke
            << "' stroke-width='" << strokeW << "'>"
            << "<title>" << xmlEscape(channelTitle) << "</title></rect>\n";

        // 為每個 channel 建 clipPath，確保文字永遠只會顯示在自己的 channel 內，
        // 不會像舊版一樣橫向覆蓋到隔壁 channel 或 block。
        svg << "<clipPath id='" << clipId << "'><rect x='" << (rx + 1.0) << "' y='" << (ry + 1.0)
            << "' width='" << max(0.0, ch.rect.w - 2.0) << "' height='" << max(0.0, ch.rect.h - 2.0)
            << "'/></clipPath>\n";

        // channel label：改成「總覽型」而不是只印 LR/TB。
        // Raw/RISA/RUDY/Hybrid 都是 max(LR,TB) 的整體值；LR/TB 細節保留在 hover title。
        // 這樣可以直接在圖上比較各模式，不必每次都切下拉選單。
        const bool importantChannel = rawModeUtil >= 0.50 || risaModeUtil >= 0.75 ||
            rudyModeUtil >= 0.75 || hybridModeUtil >= 0.75 ||
            p.routerCriticality >= 0.20 || p.regionId >= 0;
        const bool enoughRoomForLabel = ch.rect.w > 48.0 && ch.rect.h > 32.0;
        const bool bigQuietChannel = ch.rect.w > 120.0 && ch.rect.h > 70.0;
        if ((importantChannel && enoughRoomForLabel) || bigQuietChannel) {
            const double fs = max(5.5, min(11.5, min(ch.rect.w * 0.12, ch.rect.h * 0.13)));
            const double xText = rx + 3.0;
            const double yText = ry + fs + 3.0;
            const double dy = fs * 1.12;

            svg << "<g class='step0-channel-label' clip-path='url(#" << clipId << ")'>\n";
            svg << "<text x='" << xText << "' y='" << yText
                << "' font-size='" << fs << "' font-family='Arial' font-weight='bold' fill='#111' "
                << "fill-opacity='0.82' stroke='white' stroke-opacity='0.48' stroke-width='1.6' paint-order='stroke'>";
            svg << "<tspan x='" << xText << "' dy='0'>" << xmlEscape(ch.name) << "</tspan>";
            svg << "<tspan x='" << xText << "' dy='" << dy << "'>Raw=" << fmt(rawModeUtil) << "</tspan>";
            if (ch.rect.h > fs * 4.1) {
                svg << "<tspan x='" << xText << "' dy='" << dy << "'>RISA=" << fmt(risaModeUtil) << "</tspan>";
                svg << "<tspan x='" << xText << "' dy='" << dy << "'>RUDY=" << fmt(rudyModeUtil) << "</tspan>";
            }
            if (ch.rect.h > fs * 5.3) {
                svg << "<tspan x='" << xText << "' dy='" << dy << "'>HYB=" << fmt(hybridModeUtil) << "</tspan>";
            }
            if (ch.rect.h > fs * 7.2 && ch.rect.w > 95.0) {
                svg << "<tspan x='" << xText << "' dy='" << dy << "'>rudyLR=" << fmt(p.rudyUtilLR) << "</tspan>";
                svg << "<tspan x='" << xText << "' dy='" << dy << "'>rudyTB=" << fmt(p.rudyUtilTB) << "</tspan>";
            }
            svg << "</text>\n";
            svg << "</g>\n";
        }
    }

    // V7.5-visual-refine：疊加 congested region bbox。
    // 改良重點：
    //   1) 不再只用紅色粗框，改成依 region 主因分類的半透明淺色底。
    //   2) region label 改成置中、放大、半透明，避免像舊版一樣擋住 channel/block/flyline。
    //   3) outline 仍保留 dashed stroke，但降低線寬與透明度，讓它是「輔助 overlay」而不是主圖。
    if (opt_.drawRegionOverlay && !result.congestedRegions.empty()) {
        const int limit = min(static_cast<int>(result.congestedRegions.size()), max(0, opt_.maxRegionOverlays));
        for (int ri = 0; ri < limit; ++ri) {
            const Step0CongestedRegion& r = result.congestedRegions[ri];
            const double pad = 2.0;
            const double rx = max(0.0, r.bbox.x - pad);
            const double ry = max(0.0, sy(r.bbox) - pad);
            const double rw = min(W - rx, r.bbox.w + 2.0 * pad);
            const double rh = min(H - ry, r.bbox.h + 2.0 * pad);
            if (rw <= 1.0 || rh <= 1.0) continue;

            const string rtype = regionVisualTypeStep0(r);
            const string fill = regionFillColorStep0(rtype);
            const string stroke = regionStrokeColorStep0(rtype);
            const string shortLabel = regionShortLabelStep0(rtype);

            const string regionTitle = string("Region R") + to_string(r.regionId)
                + " type=" + rtype
                + " peakUtil=" + fmt(r.peakUtil)
                + " avgUtil=" + fmt(r.avgUtil)
                + " peakCrit=" + fmt(r.peakCriticality)
                + " peakRISA=" + fmt(r.peakRisaUtil)
                + " peakRUDY=" + fmt(r.peakRudyUtil)
                + " peakHybrid=" + fmt(r.peakHybridUtil)
                + " score=" + fmt(r.regionScore)
                + " channels=" + r.channelsCSV
                + " blocks=" + joinBlockNamesNearRectStep0(design, r.bbox);

            svg << "<g class='step0-hot-region' data-x='" << rx << "' data-y='" << ry
                << "' data-w='" << rw << "' data-h='" << rh << "'>\n";
            svg << "<rect class='hotspot-fill' x='" << rx << "' y='" << ry << "' width='" << rw << "' height='" << rh
                << "' fill='" << fill << "' stroke='" << stroke
                << "' stroke-width='6.2' stroke-opacity='0.98'/>\n";

            // Region label hit-area：只有停在文字/文字透明 hitbox 上才顯示 title；
            // 平常 hotspot 本身保持半透明，避免遮住內部 block / channel。
            if (rw > 46.0 && rh > 28.0) {
                const double base = min(rw, rh);
                const double fs = max(13.0, min(30.0, base * 0.22));
                const double cx = rx + rw * 0.5;
                const double cy = ry + rh * 0.5 - fs * 0.20;
                const double dy = fs * 1.05;
                const double labelW = min(rw * 0.88, max(54.0, fs * 5.2));
                const double labelH = (rw > 85.0 && rh > 62.0) ? fs * 3.05 : fs * 2.05;
                const double labelX = cx - labelW * 0.5;
                const double labelY = cy - fs * 0.95;

                svg << "<g class='hotspot-label-hit'>\n"
                    << "<title>" << xmlEscape(regionTitle) << "</title>\n";
                svg << "<rect x='" << labelX << "' y='" << labelY << "' width='" << labelW
                    << "' height='" << labelH << "' fill='white' fill-opacity='0.001' stroke='none'/>\n";
                svg << "<text class='hotspot-label' x='" << cx << "' y='" << cy
                    << "' font-size='" << fs << "' font-family='Arial' font-weight='bold' text-anchor='middle' "
                    << "fill='white' fill-opacity='0.96' "
                    << "stroke='" << stroke << "' stroke-opacity='0.98' stroke-width='5.0' paint-order='stroke'>";
                svg << "<tspan x='" << cx << "' dy='0'>R" << r.regionId << "</tspan>";
                svg << "<tspan x='" << cx << "' dy='" << dy << "'>" << xmlEscape(shortLabel) << "</tspan>";
                if (rw > 85.0 && rh > 62.0) {
                    svg << "<tspan x='" << cx << "' dy='" << dy << "' font-size='" << max(10.0, fs * 0.55)
                        << "' fill-opacity='0.70'>peak=" << fmt(r.peakUtil) << "</tspan>";
                }
                svg << "</text>\n</g>\n";
            }
            svg << "</g>\n";
        }
    }

    // V6.2：疊加高 open-risk connection flyline。
    // 這不是正式 route，只是 debug 用：不同顏色代表 Step0 判斷的 open-risk 類型，
    // 包含幾何不連通、需要 FT、容量風險、endpoint 風險與 shared bottleneck。
    if (opt_.drawOpenRiskOverlay) {
        struct RiskLine {
            int idx = -1;
            double score = 0.0;
        };
        vector<RiskLine> riskLines;
        for (int i = 0; i < static_cast<int>(result.connectionGuide.size()); ++i) {
            const auto& g = result.connectionGuide[i];
            if (g.openRiskType != "OK" || g.openRiskScore >= 0.60) {
                riskLines.push_back({ i, g.openRiskScore });
            }
        }
        sort(riskLines.begin(), riskLines.end(), [](const RiskLine& a, const RiskLine& b) {
            if (fabs(a.score - b.score) > 1e-12) return a.score > b.score;
            return a.idx < b.idx;
            });

        const int limit = min(static_cast<int>(riskLines.size()), max(0, opt_.maxRiskFlylines));
        for (int ri = 0; ri < limit; ++ri) {
            const Step0ConnectionGuide& g = result.connectionGuide[riskLines[ri].idx];
            if (g.src < 0 || g.dst < 0 || g.src >= static_cast<int>(design.blocks.size()) || g.dst >= static_cast<int>(design.blocks.size())) continue;

            const Rect& a = design.blocks[g.src].rect;
            const Rect& b = design.blocks[g.dst].rect;
            const double x1 = rectCx(a);
            const double y1 = H - rectCy(a);
            const double x2 = rectCx(b);
            const double y2 = H - rectCy(b);

            string stroke = "#e31a1c"; // geometry / high risk: red
            if (g.openRiskType == "FT_REQUIRED") stroke = "#1f78b4";              // blue
            else if (g.openRiskType == "CAPACITY_RISK") stroke = "#ff7f00";        // orange
            else if (g.openRiskType == "NO_PATTERN") stroke = "#6a3d9a";           // purple
            else if (g.openRiskType == "ENDPOINT_ACCESS_RISK") stroke = "#b15928"; // brown
            else if (g.openRiskType == "ENDPOINT_BOTTLENECK_RISK") stroke = "#fb9a99"; // pink
            else if (g.openRiskType == "SHARED_BOTTLENECK_RISK") stroke = "#cab2d6"; // lavender

            const double openConnScore01 = clampStep0(g.openRiskScore, 0.0, 1.0);
            const double openConnLineW = 2.8 + 2.8 * openConnScore01;
            const double openConnLineOpacity = 0.55 + 0.40 * openConnScore01;

            svg << "<line class='step0-flyline' x1='" << x1 << "' y1='" << y1 << "' x2='" << x2 << "' y2='" << y2
                << "' stroke='" << stroke << "' stroke-width='2.6' stroke-opacity='0.75' stroke-dasharray='9 6' data-default-width='2.6' data-default-opacity='0.75' data-open-width='" << fmt(openConnLineW) << "' data-open-opacity='" << fmt(openConnLineOpacity) << "'>"
                << "<title>" << xmlEscape(string("conn#") + to_string(g.connectionIndex)
                    + " " + design.blocks[g.src].spec.name + "->" + design.blocks[g.dst].spec.name
                    + " nets=" + to_string(g.netCount)
                    + " risk=" + g.openRiskType
                    + " score=" + fmt(g.openRiskScore)
                    + " bestPatternMaxUtil=" + fmt(g.bestPatternMaxUtil)
                    + " hasProjectedPattern=" + to_string(g.hasProjectedPattern ? 1 : 0)
                    + " srcAccess=" + to_string(g.srcHealthyAccessChannelCount) + "/" + to_string(g.srcAccessChannelCount)
                    + " dstAccess=" + to_string(g.dstHealthyAccessChannelCount) + "/" + to_string(g.dstAccessChannelCount)
                    + " commonBottleneck=" + g.commonBottleneckChannelsCSV)
                << "</title></line>\n";
        }
    }


    // V7.1：疊加 CRISP-style floorplan action boxes。
    // 注意：這些 action boxes 不是 hot region，而是 Floorplanner 可操作的建議區域，
    // bbox 會刻意比 hot region 更大，用來保留 / 增加局部 whitespace。
    // V7.5 hotspot refinement: main SVG 預設關閉這層，避免藍色 WIDEN_REGION 框和熱區混淆。
    if (false && opt_.drawFloorplanActionOverlay && !result.floorplanActions.empty()) {
        const int limit = min(static_cast<int>(result.floorplanActions.size()), max(0, opt_.maxActionOverlays));
        for (int ai = 0; ai < limit; ++ai) {
            const auto& a = result.floorplanActions[ai];
            const double rx = clampStep0(a.bbox.x, 0.0, W);
            const double ry = clampStep0(sy(a.bbox), 0.0, H);
            const double rw = max(0.0, min(W - rx, a.bbox.w));
            const double rh = max(0.0, min(H - ry, a.bbox.h));
            if (rw <= 1.0 || rh <= 1.0) continue;

            string stroke = "#08519c";
            if (a.actionType == "SPREAD_HIGH_PIN_BLOCK") stroke = "#54278f";
            else if (a.actionType == "RELAX_ENDPOINT_ACCESS") stroke = "#006d2c";

            svg << "<rect class='step0-action-box' x='" << rx << "' y='" << ry << "' width='" << rw << "' height='" << rh
                << "' fill='none' stroke='" << stroke << "' stroke-width='4.4' stroke-opacity='0.90' stroke-dasharray='7 5'>"
                << "<title>" << xmlEscape(string("Action A") + to_string(a.actionId)
                    + " type=" + a.actionType
                    + " severity=" + fmt(a.severity)
                    + " dir=" + a.dominantDirection
                    + " gap=" + fmt(a.suggestedGapIncrease)
                    + " inflate=" + fmt(a.suggestedInflationRatio)
                    + " blocks=" + a.nearbyBlocksCSV
                    + " conns=" + a.relatedConnectionsCSV
                    + " reason=" + a.reason)
                << "</title></rect>\n";

            if (rw > 40.0 && rh > 16.0) {
                svg << "<text x='" << (rx + 4.0) << "' y='" << (ry + rh - 5.0)
                    << "' font-size='11' font-family='Arial' font-weight='bold' fill='" << stroke << "' "
                    << "stroke='white' stroke-width='2.5' paint-order='stroke'>A" << a.actionId
                    << " " << xmlEscape(a.actionType) << "</text>\n";
            }
        }
    }


    // V7.1 Modified：疊加 block edge-level access pressure。
    // 粗紅/棕色線代表該 block 某一側 endpoint demand 高，或該側 access channel 不健康。
    if (false && opt_.drawBlockAccessPressureOverlay && !result.blockEdgePressure.empty()) {
        for (const auto& ep : result.blockEdgePressure) {
            if (ep.blockIndex < 0 || ep.blockIndex >= static_cast<int>(design.blocks.size())) continue;
            if (!(ep.edgeAccessRisk || ep.edgeBottleneckRisk || ep.edgePressure >= opt_.edgePressureHotThreshold)) continue;
            const Rect& r = design.blocks[ep.blockIndex].rect;
            double x1 = 0.0, y1 = 0.0, x2 = 0.0, y2 = 0.0;
            if (ep.edge == 1) { x1 = r.x; x2 = r.x; y1 = H - r.y; y2 = H - rectTop(r); }
            else if (ep.edge == 3) { x1 = rectRight(r); x2 = rectRight(r); y1 = H - r.y; y2 = H - rectTop(r); }
            else if (ep.edge == 2) { x1 = r.x; x2 = rectRight(r); y1 = H - rectTop(r); y2 = H - rectTop(r); }
            else if (ep.edge == 4) { x1 = r.x; x2 = rectRight(r); y1 = H - r.y; y2 = H - r.y; }
            else continue;

            string stroke = ep.edgeAccessRisk ? "#7f0000" : (ep.edgeBottleneckRisk ? "#b15928" : "#ff7f00");
            const double sw = 2.0 + 4.0 * clampStep0(ep.severity, 0.0, 1.0);
            svg << "<line class='step0-block-access-pressure' x1='" << x1 << "' y1='" << y1 << "' x2='" << x2 << "' y2='" << y2
                << "' stroke='" << stroke << "' stroke-width='" << sw << "' stroke-opacity='0.90'>"
                << "<title>" << xmlEscape(ep.blockName + string(" ") + ep.edgeName
                    + " edgePressure=" + fmt(ep.edgePressure)
                    + " access=" + to_string(ep.healthyAccessChannelCount) + "/" + to_string(ep.accessChannelCount)
                    + " severity=" + fmt(ep.severity)
                    + " action=" + ep.suggestedAction
                    + " move=" + ep.suggestedMoveDirection
                    + " reason=" + ep.reason)
                << "</title></line>\n";
        }
    }

    // V7.1 Modified：疊加 top block-level move hints。
    // 箭頭只代表 floorplanner 的 soft hint，不是強制 move，也不是正式 routing path。
    if (false && opt_.drawMoveHintOverlay && !result.floorplanMoveHints.empty()) {
        const int limit = min(static_cast<int>(result.floorplanMoveHints.size()), max(0, opt_.maxMoveHintOverlays));
        for (int hi = 0; hi < limit; ++hi) {
            const auto& h = result.floorplanMoveHints[hi];
            if (h.blockIndex < 0 || h.blockIndex >= static_cast<int>(design.blocks.size())) continue;
            const Rect& r = design.blocks[h.blockIndex].rect;
            // V7.5 three-layer refinement:
            // move hint 改畫在 block 內部、block 文字下層，且不超過 block outline。
            // 為了避免穿過置中的 block 名稱，水平箭頭放在 block 下半部，垂直箭頭放在右半部。
            const double margin = max(3.0, min(10.0, min(r.w, r.h) * 0.10));
            if (r.w <= 2.0 * margin + 8.0 || r.h <= 2.0 * margin + 8.0) continue;
            const double blockFontSize = max(11.0, min(34.0, min(r.w, r.h) * 0.18));
            // Move hint 放在 block 名稱正下方；線段仍完全限制在 block outline 內。
            double centerY = sy(r) + r.h * 0.5 + blockFontSize * 0.35;
            double arrowY = min(sy(r) + r.h - margin, centerY + blockFontSize * 0.95);
            arrowY = max(sy(r) + margin, arrowY);
            double arrowX = rectCx(r);
            const double maxHLen = max(8.0, r.w - 2.0 * margin);
            const double maxVLen = max(8.0, r.h - 2.0 * margin);
            const double hLen = max(8.0, min(maxHLen, max(14.0, h.suggestedDelta)));
            const double vLen = max(8.0, min(maxVLen, max(14.0, h.suggestedDelta)));
            double x1 = arrowX, y1 = arrowY, x2 = arrowX, y2 = arrowY;
            if (h.moveDirection == "MOVE_LEFT") {
                x1 = min(rectRight(r) - margin, arrowX + hLen * 0.50);
                x2 = max(r.x + margin, x1 - hLen);
                y1 = y2 = arrowY;
            }
            else if (h.moveDirection == "MOVE_RIGHT") {
                x1 = max(r.x + margin, arrowX - hLen * 0.50);
                x2 = min(rectRight(r) - margin, x1 + hLen);
                y1 = y2 = arrowY;
            }
            else if (h.moveDirection == "MOVE_UP") {
                x1 = x2 = min(rectRight(r) - margin, max(r.x + margin, arrowX + blockFontSize * 0.75));
                y1 = min(sy(r) + r.h - margin, arrowY + vLen * 0.35);
                y2 = max(sy(r) + margin, y1 - vLen);
            }
            else if (h.moveDirection == "MOVE_DOWN") {
                x1 = x2 = min(rectRight(r) - margin, max(r.x + margin, arrowX + blockFontSize * 0.75));
                y1 = max(sy(r) + margin, arrowY - vLen * 0.35);
                y2 = min(sy(r) + r.h - margin, y1 + vLen);
            }
            else continue;
            svg << "<line class='step0-movehint' x1='" << x1 << "' y1='" << y1 << "' x2='" << x2 << "' y2='" << y2
                << "' stroke='#252525' stroke-width='3.0' stroke-opacity='0.86' marker-end='url(#arrowHead)'>"
                << "<title>" << xmlEscape(string("MoveHint H") + to_string(h.hintId)
                    + " block=" + h.blockName
                    + " dir=" + h.moveDirection
                    + " delta=" + fmt(h.suggestedDelta)
                    + " confidence=" + fmt(h.confidence)
                    + " efficiency=" + fmt(h.actionEfficiency)
                    + " reason=" + h.reason)
                << "</title></line>\n";
        }
    }

    // V5：在 channel 上方補 block outline，讓 block 邊界在 heatmap 上更清楚。
    for (const auto& b : design.blocks) {
        svg << "<rect x='" << b.rect.x << "' y='" << sy(b.rect) << "' width='" << b.rect.w << "' height='" << b.rect.h
            << "' class='step0-block-outline' fill='none' stroke='#111111' stroke-width='1.8'/>\n";
    }

    // V7.5 Block-centered diagnostics：
    // edge pressure / move hint 不再用獨立大量 overlay 干擾主圖，而是集中到 block label 的 tooltip。
    // 視覺上只在 block 名稱下方顯示 move-hint 方向符號；edge pressure 僅保留在 block tooltip 中。
    auto blockMoveBrief = [&](int bi) {
        string out;
        vector<const Step0FloorplanMoveHint*> rows;
        for (const auto& h : result.floorplanMoveHints) {
            if (h.blockIndex != bi) continue;
            rows.push_back(&h);
        }
        sort(rows.begin(), rows.end(), [](const Step0FloorplanMoveHint* a, const Step0FloorplanMoveHint* b) {
            if (fabs(a->actionEfficiency - b->actionEfficiency) > 1e-12) return a->actionEfficiency > b->actionEfficiency;
            return a->hintId < b->hintId;
            });
        auto dirToken = [](const string& d) {
            // Use XML numeric entities instead of raw UTF-8 arrows to avoid SVG encoding issues on Windows builds.
            if (d == "MOVE_LEFT") return string("&#8592;");
            if (d == "MOVE_RIGHT") return string("&#8594;");
            if (d == "MOVE_UP") return string("&#8593;");
            if (d == "MOVE_DOWN") return string("&#8595;");
            if (d == "KEEP_AWAY") return string("KA");
            return string("?");
            };
        for (int i = 0; i < min(4, static_cast<int>(rows.size())); ++i) {
            if (!out.empty()) out += " ";
            out += dirToken(rows[i]->moveDirection);
        }
        return out;
        };
    auto blockDiagnosticTitle = [&](int bi) {
        const auto& b = design.blocks[bi];
        string t = string("BLOCK ") + b.spec.name + " type=" + blockTypeToString(b.spec.type)
            + " rect=(" + fmt(b.rect.x) + "," + fmt(b.rect.y) + "," + fmt(b.rect.w) + "," + fmt(b.rect.h) + ")";
        t += "\n\nEDGE PRESSURE:";
        int edgeCount = 0;
        for (const auto& ep : result.blockEdgePressure) {
            if (ep.blockIndex != bi) continue;
            if (!(ep.edgeAccessRisk || ep.edgeBottleneckRisk || ep.edgePressure >= opt_.edgePressureHotThreshold)) continue;
            ++edgeCount;
            t += "\n  " + ep.edgeName
                + " pressure=" + fmt(ep.edgePressure)
                + " access=" + to_string(ep.healthyAccessChannelCount) + "/" + to_string(ep.accessChannelCount)
                + " maxUtil=" + fmt(ep.maxAccessUtil)
                + " maxCrit=" + fmt(ep.maxAccessCriticality)
                + " severity=" + fmt(ep.severity)
                + " action=" + ep.suggestedAction
                + " move=" + ep.suggestedMoveDirection
                + " channels=" + ep.accessChannelsCSV;
        }
        if (edgeCount == 0) t += "\n  no hot edge-pressure risk";
        t += "\n\nMOVE HINTS:";
        int hintCount = 0;
        for (const auto& h : result.floorplanMoveHints) {
            if (h.blockIndex != bi) continue;
            ++hintCount;
            t += "\n  H" + to_string(h.hintId)
                + " target=" + h.targetEdge
                + " dir=" + h.moveDirection
                + " delta=" + fmt(h.suggestedDelta)
                + " confidence=" + fmt(h.confidence)
                + " efficiency=" + fmt(h.actionEfficiency)
                + " sourceAction=A" + to_string(h.sourceActionId)
                + " reason=" + h.reason;
            if (hintCount >= 6) { t += "\n  ..."; break; }
        }
        if (hintCount == 0) t += "\n  no move hint";
        return t;
        };

    for (int bi = 0; bi < static_cast<int>(design.blocks.size()); ++bi) {
        const auto& b = design.blocks[bi];
        if (b.rect.w <= 22.0 || b.rect.h <= 14.0) continue;
        const double base = min(b.rect.w, b.rect.h);
        const double fontSize = max(11.0, min(34.0, base * 0.18));
        const double cx = rectCx(b.rect);
        const double cy = sy(b.rect) + b.rect.h * 0.5 + fontSize * 0.20;
        const string mhBrief = blockMoveBrief(bi);
        const bool hasMH = !mhBrief.empty();
        const double infoSize = max(10.0, min(20.0, fontSize * 0.62));
        const double hitW = min(b.rect.w - 2.0, max(44.0, b.rect.w * 0.72));
        const double hitH = min(b.rect.h - 2.0, fontSize * (hasMH ? 2.30 : 1.45));
        const double hitX = cx - hitW * 0.5;
        const double hitY = max(sy(b.rect) + 1.0, cy - fontSize * 1.05);

        svg << "<g class='step0-block-label'>\n"
            << "<title>" << xmlEscape(blockDiagnosticTitle(bi)) << "</title>\n";
        svg << "<rect x='" << hitX << "' y='" << hitY << "' width='" << hitW << "' height='" << hitH
            << "' fill='white' fill-opacity='0.001' stroke='none'/>\n";
        svg << "<text x='" << cx << "' y='" << cy << "' font-size='" << fontSize
            << "' font-family='Arial' font-weight='bold' text-anchor='middle' "
            << "fill='black' stroke='white' stroke-width='3.0' paint-order='stroke'>"
            << xmlEscape(b.spec.name) << "</text>\n";
        // Visible block annotation is intentionally minimal: only move-hint direction symbols below the block name.
        // Edge-pressure details are available in the block tooltip to avoid clutter such as EP:B/L/R on the canvas.
        if (hasMH && b.rect.h > fontSize * 2.05) {
            const double iy = cy + fontSize * 0.88;
            svg << "<text class='step0-block-info step0-block-move-info' x='" << cx << "' y='" << iy
                << "' font-size='" << infoSize << "' font-family='Arial' font-weight='bold' text-anchor='middle' "
                << "fill='#111111' stroke='white' stroke-width='2.4' paint-order='stroke'>" << mhBrief << "</text>\n";
        }
        svg << "</g>\n";
    }

    // V7.5 three-layer refinement：最高層 WIDEN_REGION / floorplanner action overlay。
    // 這一層用深色表示「可操作的 floorplanner 調整區」，不是 congestion hotspot。
    // 它畫在最高層，但 pointer-events:none，不會阻擋使用者 hover channel / flyline / movehint。
    // JS 會在滑鼠座標落入 action bbox 時自動把它透明化，方便檢查底下內容。
    if (opt_.drawFloorplanActionOverlay && !result.floorplanActions.empty()) {
        const int limit = min(static_cast<int>(result.floorplanActions.size()), max(0, opt_.maxActionOverlays));
        for (int ai = 0; ai < limit; ++ai) {
            const auto& a = result.floorplanActions[ai];
            const double rx = clampStep0(a.bbox.x, 0.0, W);
            const double ry = clampStep0(sy(a.bbox), 0.0, H);
            const double rw = max(0.0, min(W - rx, a.bbox.w));
            const double rh = max(0.0, min(H - ry, a.bbox.h));
            if (rw <= 1.0 || rh <= 1.0) continue;

            string fill = "#08306b";    // WIDEN_REGION: deep blue, top layer
            string stroke = "#041b4d";
            string label = "WIDEN";
            if (a.actionType == "SPREAD_HIGH_PIN_BLOCK") { fill = "#4a1486"; stroke = "#2b0054"; label = "SPREAD"; }
            else if (a.actionType == "RELAX_ENDPOINT_ACCESS") { fill = "#00441b"; stroke = "#002b10"; label = "ACCESS"; }
            else if (a.actionType == "RISA_SUPPLY_DEMAND_BALANCE") { fill = "#7f2704"; stroke = "#4d1600"; label = "SUPPLY"; }

            const string actionTitle = string("Action A") + to_string(a.actionId)
                + " type=" + a.actionType
                + " severity=" + fmt(a.severity)
                + " dir=" + a.dominantDirection
                + " gap=" + fmt(a.suggestedGapIncrease)
                + " inflate=" + fmt(a.suggestedInflationRatio)
                + " riskReduction=" + fmt(a.expectedRiskReduction)
                + " areaPenalty=" + fmt(a.estimatedAreaPenalty)
                + " wirePenalty=" + fmt(a.estimatedWirePenalty)
                + " efficiency=" + fmt(a.actionEfficiency)
                + " blocks=" + a.nearbyBlocksCSV
                + " conns=" + a.relatedConnectionsCSV
                + " reason=" + a.reason;

            svg << "<g class='step0-action-box-group' style='display:none;' data-x='" << rx << "' data-y='" << ry
                << "' data-w='" << rw << "' data-h='" << rh << "'>\n";

            svg << "<rect class='action-fill' x='" << rx << "' y='" << ry << "' width='" << rw << "' height='" << rh
                << "' fill='" << fill << "' stroke='" << stroke
                << "' stroke-width='7.8' stroke-opacity='0.98' stroke-dasharray='10 6'/>\n";

            if (rw > 56.0 && rh > 30.0) {
                const double fs = max(12.0, min(24.0, min(rw, rh) * 0.16));
                const double cx = rx + rw * 0.5;
                const double cy = ry + rh * 0.5 - fs * 0.10;
                const double labelW = min(rw * 0.86, max(54.0, fs * 5.8));
                const double labelH = fs * 2.25;
                svg << "<g class='action-label-hit'>\n"
                    << "<title>" << xmlEscape(actionTitle) << "</title>\n";
                svg << "<rect x='" << (cx - labelW * 0.5) << "' y='" << (cy - fs * 0.95)
                    << "' width='" << labelW << "' height='" << labelH
                    << "' fill='white' fill-opacity='0.001' stroke='none'/>\n";
                svg << "<text class='action-label' x='" << cx << "' y='" << cy
                    << "' font-size='" << fs << "' font-family='Arial' font-weight='bold' text-anchor='middle' "
                    << "fill='white' fill-opacity='0.92' stroke='" << stroke
                    << "' stroke-opacity='0.96' stroke-width='4.0' paint-order='stroke'>";
                svg << "<tspan x='" << cx << "' dy='0'>A" << a.actionId << "</tspan>";
                svg << "<tspan x='" << cx << "' dy='" << fs * 1.05 << "'>" << xmlEscape(label) << "</tspan>";
                svg << "</text>\n</g>\n";
            }
            svg << "</g>\n";
        }
    }

    const double ly = H + 18.0;
    ostringstream title;
    title << "Step0 Congestion Map: channel color = max(LR/TB util), visualScale=" << fixed << setprecision(2) << visualScale
        << ", actualRawMaxUtil=" << setprecision(3) << actualRawMaxUtil
        << ", modelMaxUtil=" << setprecision(3) << result.maxModelUtil
        << ". Labels/CSV show actual util. FloorplanStatus=" << result.floorplanHealth.floorplanStatus << ", risk=" << setprecision(3) << result.floorplanHealth.floorplanRiskScore << ". Block colors: soft=blue, hard=pink, edge=green.";
    svg << "<text x='5' y='" << ly << "' font-size='14' font-family='Arial' fill='black'>"
        << xmlEscape(title.str()) << "</text>\n";

    // Interactive control panel: ASCII-only SVG text with HTML numeric entities to avoid XML encoding issues.
    const double panelY = H + 28.0;
    const double panelH = 154.0;
    svg << "<foreignObject x='5' y='" << panelY << "' width='" << max(100.0, W - 10.0) << "' height='" << panelH << "'>\n"
        << "<div xmlns='http://www.w3.org/1999/xhtml' class='step0-control-panel' style='background:#f8f8f8;border:1px solid #bbb;border-radius:6px;padding:8px 10px;box-sizing:border-box;line-height:1.35;'>"
        << "<div style='font-weight:bold;margin-bottom:6px;'>Layer controls / &#22294;&#23652;&#25511;&#21046;</div>"
        << "<div style='display:flex;flex-wrap:wrap;gap:6px 14px;'>"
        << "<label title='Show congestion/bottleneck hot regions. Hover a hotspot to fade it.'><input type='checkbox' checked='checked' onchange='window.step0ToggleLayer(&quot;hotspot&quot;,this.checked)'/> Hotspots &#39023;&#31034;&#29105;&#21312;</label>"
        << "<label title='Show floorplanner action boxes such as WIDEN_REGION. Hover an action box to fade it.'><input type='checkbox' onchange='window.step0ToggleLayer(&quot;action&quot;,this.checked)'/> Actions &#39023;&#31034;&#34253;&#26694;</label>"
        << "<label title='Show block-level move or keep-away hints.'><input type='checkbox' checked='checked' onchange='window.step0ToggleLayer(&quot;movehint&quot;,this.checked)'/> MoveHint</label>"
        << "<label title='Show high-risk connection flylines.'><input type='checkbox' checked='checked' onchange='window.step0ToggleLayer(&quot;flyline&quot;,this.checked)'/> Flyline</label>"
        << "<label title='Show channel LR/TB labels.'><input type='checkbox' checked='checked' onchange='window.step0ToggleLayer(&quot;channelLabel&quot;,this.checked)'/> Channel label</label>"
        << "<label title='Show block names.'><input type='checkbox' checked='checked' onchange='window.step0ToggleLayer(&quot;blockLabel&quot;,this.checked)'/> Block label</label>"
        << "<label title='Show edge-level block access pressure overlays.'><input type='checkbox' checked='checked' onchange='window.step0ToggleLayer(&quot;blockAccess&quot;,this.checked)'/> Edge pressure</label>"
        << "</div>"
        << "<div style='margin-top:9px;font-weight:bold;'>Channel color mode / Channel &#38991;&#33394;&#27169;&#24335;</div>"
        << "<div style='display:flex;flex-wrap:wrap;align-items:center;gap:8px;margin-top:4px;'>"
        << "<select onchange='window.step0SetChannelMode(this.value)' style='min-width:270px;'>"
        << "<option value='raw' selected='selected'>Raw utilization - &#21407;&#22987; LR/TB &#20351;&#29992;&#29575;</option>"
        << "<option value='hybrid'>Hybrid health - pattern + RUDY + RISA</option>"
        << "<option value='risa'>RISA supply-demand - effective supply risk</option>"
        << "<option value='rudy'>RUDY background - router-independent demand</option>"
        << "<option value='router'>Router guide - ambient + criticality</option>"
        << "<option value='open_conn'>Route-open connection focus - risk flyline intensity</option>"
        << "<option value='open_channel'>Route-open channel projection - open-risk footprint</option>"
        << "<option value='diagnostic'>Diagnostic mix - legacy mixed view</option>"
        << "</select>"
        << "<span class='step0-ui-note'>Current / &#30446;&#21069;&#27169;&#24335;: <b id='step0-current-mode'>raw</b></span>"
        << "</div>"
        << "<div id='step0-mode-description' class='step0-ui-note' style='margin-top:6px;border-left:4px solid #999;padding-left:8px;'>"
        << "<b>Raw utilization / &#21407;&#22987;&#20351;&#29992;&#29575;</b>: channel color uses only max(utilLR, utilTB). Best for checking whether a channel really overflows before diagnostic penalties."
        << "</div>"
        << "<div class='step0-ui-note' style='margin-top:5px;'>Hint / &#25552;&#31034;: overlay layers fade on hover; channel, flyline, and movehint titles remain available underneath.</div>"
        << "</div></foreignObject>\n";

    const vector<pair<string, string>> legend = {
        {"display <0.50", "#b7e4a1"}, {"0.50-0.75", "#fff3a3"}, {"0.75-1.00", "#fdae61"},
        {"1.00-1.25", "#f46d43"}, {"1.25-1.50", "#d73027"}, {">=1.50", "#7b3294"}
    };

    double lx = 5.0;
    for (const auto& kv : legend) {
        svg << "<rect x='" << lx << "' y='" << (ly + 206.0) << "' width='25' height='12' fill='" << kv.second << "' stroke='black' stroke-width='0.3'/>\n";
        svg << "<text x='" << (lx + 30.0) << "' y='" << (ly + 217.0) << "' font-size='10' font-family='Arial'>" << xmlEscape(kv.first) << "</text>\n";
        lx += 112.0;
    }

    const vector<pair<string, string>> regionLegend = {
        {"PINCH", "#e6550d"},
        {"PAT pattern", "#de2d26"},
        {"RISA supply", "#f16913"},
        {"RUDY bg", "#2171b5"},
        {"HYB hybrid", "#6a51a3"},
        {"CRIT history", "#ae017e"},
        {"MIX", "#636363"}
    };
    lx = 5.0;
    svg << "<text x='5' y='" << (ly + 238.0) << "' font-size='11' font-family='Arial' font-weight='bold' fill='#333'>"
        << xmlEscape("Hotspot region legend: darker translucent area shows dominant hot-region type. Hover a hotspot to fade it and inspect blocks/channels underneath.") << "</text>\n";
    for (const auto& kv : regionLegend) {
        svg << "<rect x='" << lx << "' y='" << (ly + 184.0) << "' width='25' height='12' fill='" << kv.second
            << "' fill-opacity='0.40' stroke='black' stroke-width='0.3'/>\n";
        svg << "<text x='" << (lx + 30.0) << "' y='" << (ly + 195.0) << "' font-size='10' font-family='Arial'>" << xmlEscape(kv.first) << "</text>\n";
        lx += 118.0;
    }

    svg << "<text x='5' y='" << (ly + 258.0) << "' font-size='11' font-family='Arial' fill='#333'>"
        << xmlEscape("Note: SVG colors may be amplified for visibility; use CSV utilLR/utilTB for exact values. Hotspots are dark by default, but fade on hover. WIDEN_REGION boxes and move-hint arrows are floorplanner action hints, not congestion regions.")
        << "</text>\n";

    svg << "</svg>\n";
}

void CongestionMapBuilder::printSummary(const Design& design, const Step0CongestionMapResult& result) const {
    struct Row {
        int ci = -1;
        double maxUtil = 0.0;
        double utilLR = 0.0;
        double utilTB = 0.0;
        double predLR = 0.0;
        double predTB = 0.0;
        double capLR = 0.0;
        double capTB = 0.0;
        double crit = 0.0;
    };

    vector<Row> rows;
    for (int i = 0; i < static_cast<int>(result.channelPressure.size()); ++i) {
        const auto& p = result.channelPressure[i];
        rows.push_back(Row{ i, max(p.utilLR, p.utilTB), p.utilLR, p.utilTB, p.predLR, p.predTB, p.capLR, p.capTB, p.criticality });
    }

    sort(rows.begin(), rows.end(), [](const Row& a, const Row& b) {
        if (fabs(a.maxUtil - b.maxUtil) > 1e-12) return a.maxUtil > b.maxUtil;
        return a.ci < b.ci;
        });

    cerr << fixed << setprecision(3);
    cerr << "[Step0CongestionMap] channels=" << design.channels.size()
        << " connections=" << design.connections.size()
        << " patterns=STRAIGHT/L-HV/L-VH" << (opt_.enableZPattern ? "/Z-HVH/Z-VHV" : "")
        << (opt_.enableDirectionalZPattern ? "/Z-directional" : "")
        << " reroutePasses=" << max(0, opt_.reroutePasses)
        << " rawMaxUtil=" << result.maxRawUtil
        << " avgRawUtil=" << result.avgRawUtil
        << " modelMaxUtil=" << result.maxModelUtil
        << " regions=" << result.congestedRegions.size() << "\n";

    const int limit = min(static_cast<int>(rows.size()), max(0, opt_.topPrintCount));
    for (int k = 0; k < limit; ++k) {
        const Row& r = rows[k];
        const Channel& ch = design.channels[r.ci];
        cerr << "  " << ch.name
            << " maxUtil=" << r.maxUtil
            << " crit=" << r.crit
            << " | LR pred=" << r.predLR << " cap=" << r.capLR << " util=" << r.utilLR
            << " | TB pred=" << r.predTB << " cap=" << r.capTB << " util=" << r.utilTB
            << " region=" << result.channelPressure[r.ci].regionId
            << " rect=(" << ch.rect.x << "," << ch.rect.y << "," << ch.rect.w << "," << ch.rect.h << ")\n";
    }

    const int rlimit = min(static_cast<int>(result.congestedRegions.size()), 6);
    for (int i = 0; i < rlimit; ++i) {
        const auto& rg = result.congestedRegions[i];
        cerr << "  Region R" << rg.regionId
            << " channels=" << rg.channelIndices.size()
            << " peakUtil=" << rg.peakUtil
            << " avgUtil=" << rg.avgUtil
            << " peakCrit=" << rg.peakCriticality
            << " score=" << rg.regionScore
            << " bbox=(" << rg.bbox.x << "," << rg.bbox.y << "," << rg.bbox.w << "," << rg.bbox.h << ")"
            << " channels=" << rg.channelsCSV << "\n";
    }

    const auto& h = result.floorplanHealth;
    cerr << "  [CRISP-FP] status=" << h.floorplanStatus
        << " risk=" << h.floorplanRiskScore
        << " hotCH=" << h.numHotChannels
        << " nearFullCH=" << h.numNearFullChannels
        << " overflowCH=" << h.numOverflowChannels
        << " highRiskConn=" << h.numHighRiskConnections
        << " deadspace=" << h.deadspaceRatio
        << " suggestedInflation=" << h.suggestedMaxInflationAreaRatio
        << " suggestedCompaction=" << h.suggestedMaxCompactionRatio
        << " actions=" << result.floorplanActions.size() << "\n";

    const int alimit = min(static_cast<int>(result.floorplanActions.size()), 6);
    for (int i = 0; i < alimit; ++i) {
        const auto& a = result.floorplanActions[i];
        cerr << "  Action A" << a.actionId
            << " type=" << a.actionType
            << " severity=" << a.severity
            << " dir=" << a.dominantDirection
            << " gap=" << a.suggestedGapIncrease
            << " inflate=" << a.suggestedInflationRatio
            << " bbox=(" << a.bbox.x << "," << a.bbox.y << "," << a.bbox.w << "," << a.bbox.h << ")"
            << " blocks=" << a.nearbyBlocksCSV << "\n";
    }
}
