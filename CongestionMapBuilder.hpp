#pragma once

#include "DataModel.hpp"

#include <string>
#include <vector>

// =============================================================================
// Step0 Congestion Map / Routing Guide
// -----------------------------------------------------------------------------
// 這個模組是放在 ChannelBuilder 之後、Router 之前的「輕量級 routing 健康檢查」。
//
// 它不是真正 Router：
//   - 不輸出 PATH。
//   - 不保證每個 pattern 最後都是合法路徑。
//   - 不取代 Router 的 Dijkstra。
//
// 它的任務是：
//   1) 使用 ChannelBuilder 已經產生的 actual channel rectangles。
//   2) 將每個 channel 拆成 LR / TB 兩個方向性資源。
//   3) 用 Straight / L-shaped / Z-shaped pattern 將 connection matrix 的 netCount
//      投影到 channel 上，估計 predLR / predTB。
//   4) 產生每個 channel 的 utilization / criticality，供 Router cost 使用。
//   5) 輸出 CSV / SVG，讓你可以圖片化檢查目前 floorplan 的 routing health。
//   6) V6.2 進一步檢查 endpoint access 與 shared bottleneck，
//      用來提前定位 route-open 可能出現的位置。
//   7) V6.3 進一步把相鄰高壓 channels 聚合成 congested regions，
//      並補齊 Z-shaped 的 bbox 外側四方向 guide。
//   8) V7.1 依照 CRISP 的 measure-and-improve 精神，新增 floorplan feedback：
//      將 congestion / pin-density-like endpoint risk 轉成 health summary 與 action hints。
//   9) V7.1 Modified 補上 floorplanner feedback 的可操作資訊：
//      edge-level block access pressure、block-level move hints、action trade-off、
//      action-to-cost 欄位，以及 optional before/after delta report。
//  10) V7.2-RISA 依照 RISA 的 supply-demand 精神，新增 effective supply、
//      demand-supply overflow cost、hard/edge block detour demand，以及 RISA-style
//      channel supply-demand CSV，讓 CMB 不只看 demand，也明確看 supply。
//  11) V7.3-HM 依照 Hadsell-Madden 的 amplified congestion estimation 精神，
//      將低壓區的 guide 影響降到近乎 0，並強化真正接近滿載/overflow 的熱區；
//      另外輸出 ambient demand，讓 Router cost 使用「actual + ambient」做 soft guide。
//  12) V7.5 Channel-metrics refinement 額外強化 SVG channel hover/label：
//      在 hover 中分組顯示 Raw/RISA/RUDY/Hybrid/Router Guide，並把
//      routerCriticality amplification 改為 smooth ramp，避免 0/1 過度保守。
//  12) V7.4-Pattern 依照 Pattern Routing: Use and Theory for Increasing
//      Predictability and Avoiding Coupling 的精神，新增 pattern predictability、
//      conditional pattern expansion、coupling-risk proxy、guide strength。CMB 仍然
//      不硬指定路徑，而是判斷哪些 connection 適合被 pattern guide 強烈引導，
//      哪些 connection 應交給 Router 保持自由。
//  13) V7.5-RUDY 依照 Fast and Accurate Routing Demand Estimation 的精神，
//      新增 router-independent 的 Rectangular Uniform wire DensitY background demand。
//      CMB 同時保留 patternDemand 與 rudyDemand，並形成 hybridDemand：
//      patternDemand 給 Router guide，rudyDemand 給 Floorplanner/routing health 的 robust baseline。
//  14) V7.5 Diagnostic refinement：重新整理 SVG overlay 與 CMB 指標分工。
//      修正 region overlay 過度遮擋、channel label 過密、region 主因分類過度偏向 PATTERN 的問題；
//      同時保留完整 hover title / CSV，讓圖上呈現重點、CSV 保留完整數據。
//  15) V7.5 Dashboard refinement：新增 dashboard / signal-conflict / text report。
//      目的不是增加新模型，而是整理既有 Raw/RISA/RUDY/Hybrid/Router-guide 訊號，
//      幫助使用者快速判斷：哪些 channel/region/connection/action 最重要，以及哪些訊號彼此看似衝突。
//
// 為什麼獨立成 cpp/header：
//   - 上層 Floorplanner 未來可以拿 Step0CongestionMapResult 做 floorplan feedback。
//   - 下層 Router 可以拿 channelCriticality 當 Dijkstra cost 的全域 guide。
//   - 中間層清楚，不會把 congestion-map 邏輯塞在 Router.cpp 裡。
// =============================================================================

struct Step0PatternSegment {
    double x1 = 0.0;
    double y1 = 0.0;
    double x2 = 0.0;
    double y2 = 0.0;
};

struct Step0PatternCandidate {
    std::string name;                       // STRAIGHT / L-HV / L-VH / Z-HVH / Z-VHV
    std::vector<Step0PatternSegment> segs;  // pattern 被拆成的水平/垂直線段
    double wireLength = 0.0;                // pattern 幾何線長，用於 candidate cost
    double blockagePenalty = 0.0;           // 穿越 hard/edge/soft block 的 guide penalty

    // V6.1：初始 probabilistic assignment 的權重。
    // 不是正式 routing 機率，只是讓 L/Z/straight 在 congestion-map 中的分配更合理。
    double baseWeight = 0.0;

    // V7.4-Pattern：pattern 本身的簡化屬性。
    // bendCount 越低，通常越可預測、越接近 minimal route；但若穿過 blockage / hot channel，
    // CMB 仍會降低該 pattern 的可用性。
    int bendCount = 0;
    bool isOuterDetour = false;
    bool isPredictablePattern = true;
};

struct Step0ChannelPressure {
    // LR / horizontal：edge 1 <-> edge 3，線在 channel 裡左右走。
    // 左右走需要沿 y 方向排開，所以容量 = channel height * CHANNEL_DENSITY。
    double capLR = 0.0;

    // TB / vertical：edge 2 <-> edge 4，線在 channel 裡上下走。
    // 上下走需要沿 x 方向排開，所以容量 = channel width * CHANNEL_DENSITY。
    double capTB = 0.0;

    // V7.2-RISA：effective supply。
    // RISA 的核心是 supply-demand balance。Problem E 的 actual channel 已經是 routing
    // resource，但 hard/edge block 造成的不可穿越、pinch/access 限制會降低可用供給。
    // 因此這裡保留原始 capLR/capTB，也另外估計 effectiveCapLR/effectiveCapTB。
    double effectiveCapLR = 0.0;
    double effectiveCapTB = 0.0;
    double supplyReductionLR = 0.0;
    double supplyReductionTB = 0.0;
    double supplyScaleLR = 1.0;
    double supplyScaleTB = 1.0;

    // Step0 pattern routing 預估需求。
    double predLR = 0.0;
    double predTB = 0.0;

    // utilization = predicted demand / capacity。
    double utilLR = 0.0;
    double utilTB = 0.0;

    // V7.2-RISA：demand-side correction。
    // predLR/predTB 是 pattern demand；risaExtraDemand* 來自 hard/edge block detour
    // 造成的額外需求；risaDemand* = pred* + risaExtraDemand*。
    double risaExtraDemandLR = 0.0;
    double risaExtraDemandTB = 0.0;
    double risaDemandLR = 0.0;
    double risaDemandTB = 0.0;

    // V7.5-RUDY：router-independent background demand。
    // RUDY 不嘗試預測單條 net 的實際路徑，而是在 source/destination enclosing rectangle
    // 內均勻分配 wire density。這能補足 patternDemand 過度依賴 L/Z candidate 的偏誤。
    double rudyDemandLR = 0.0;
    double rudyDemandTB = 0.0;
    double rudyUtilLR = 0.0;
    double rudyUtilTB = 0.0;
    double rudyOverlapArea = 0.0;

    // V7.5-RUDY：hybrid demand。
    // Router guide 仍以 pattern/amplified guide 為主；Floorplanner health / region score
    // 則更重視 RUDY 這種不依賴 router model 的背景 demand。
    double hybridDemandLR = 0.0;
    double hybridDemandTB = 0.0;
    double hybridUtilLR = 0.0;
    double hybridUtilTB = 0.0;
    double hybridOverflowLR = 0.0;
    double hybridOverflowTB = 0.0;
    double hybridCost = 0.0;

    // V7.2-RISA：supply-demand result。
    // risaUtil 使用 effective capacity，risaOverflow 以 t*S 作為 safety supply，
    // 對應 RISA cost 中 Max(D - tS, 0) 的精神。
    double risaUtilLR = 0.0;
    double risaUtilTB = 0.0;
    double risaOverflowLR = 0.0;
    double risaOverflowTB = 0.0;
    double risaCostLR = 0.0;
    double risaCostTB = 0.0;
    double risaCost = 0.0;

    // 簡化 history：記錄這個 channel component 是否反覆 near-full / overflow。
    // 第一版預設只做 1 輪 reroute，因此 history 主要是標記初始與 reroute 後的熱區。
    double histLR = 1.0;
    double histTB = 1.0;

    // 未正規化 / 正規化 criticality。
    // criticality 保留為 CMB 內部 / visualization 用的一般風險值。
    double criticalityRaw = 0.0;
    double criticality = 0.0;

    // V7.3-HM：amplified congestion guide。
    // Hadsell-Madden 的重點是：低/中壅塞區不應過度影響路徑，以免製造無謂 detour；
    // 真正高壅塞區要被放大，讓後續 route 明顯避開。
    // amplificationFactor*: 依 component utilization 分段放大。
    // amplifiedStaticDemand*: 被放大後的 static congestion estimate。
    // ambientDemand*: 給 Router cost 的 persistent background demand；不作為 hard feasibility。
    // routerCriticality*: 專門給 Router 使用的 criticality，與 visual criticality 分開。
    double amplificationFactorLR = 0.0;
    double amplificationFactorTB = 0.0;
    double amplifiedStaticDemandLR = 0.0;
    double amplifiedStaticDemandTB = 0.0;
    double ambientDemandLR = 0.0;
    double ambientDemandTB = 0.0;
    double ambientUtilLR = 0.0;
    double ambientUtilTB = 0.0;
    double routerCriticalityLR = 0.0;
    double routerCriticalityTB = 0.0;
    double routerCriticality = 0.0;

    // V6.3：所屬的 congested region id。-1 代表不屬於高風險區域。
    int regionId = -1;
};

struct Step0CongestedRegion {
    int regionId = -1;
    std::vector<int> channelIndices;
    std::string channelsCSV;

    // region 的 bounding box，方便 SVG 畫出區域框。
    Rect bbox;

    double peakUtil = 0.0;
    double avgUtil = 0.0;
    double peakCriticality = 0.0;
    double avgCriticality = 0.0;
    double totalPredLR = 0.0;
    double totalPredTB = 0.0;
    double totalOverflowRisk = 0.0;

    // V7.2-RISA：region-level supply-demand aggregation。
    double peakRisaUtil = 0.0;
    double totalRisaOverflowRisk = 0.0;
    double totalRisaCost = 0.0;

    // V7.5-RUDY：region-level background / hybrid aggregation。
    double peakRudyUtil = 0.0;
    double peakHybridUtil = 0.0;
    double totalRudyDemand = 0.0;
    double totalHybridDemand = 0.0;
    double totalHybridCost = 0.0;

    double regionScore = 0.0;
};

// V7.1 Modified：block edge-level access / pin-density-like pressure。
// CRISP 有 congestion map，也有 pin-density map；Problem E 沒有真實 pin，
// 因此用 connection demand 的方向性與每個 block 四側的 access channel 健康度近似。
struct Step0BlockEdgePressure {
    int blockIndex = -1;
    std::string blockName;
    std::string blockType;
    int edge = 0;                       // 1=left, 2=top, 3=right, 4=bottom
    std::string edgeName;

    double incidentDemandEstimate = 0.0; // 粗估會從此 edge 進出的 netCount
    double edgeSupply = 0.0;             // edge length * density * conservative factor
    double edgePressure = 0.0;           // incidentDemandEstimate / edgeSupply

    int accessChannelCount = 0;
    int healthyAccessChannelCount = 0;
    double maxAccessUtil = 0.0;
    double avgAccessUtil = 0.0;
    double maxAccessCriticality = 0.0;

    bool edgeAccessRisk = false;
    bool edgeBottleneckRisk = false;
    double severity = 0.0;

    std::string accessChannelsCSV;
    std::string suggestedAction;         // RESERVE_LEFT_CHANNEL / RESERVE_TOP_CHANNEL ...
    std::string suggestedMoveDirection;  // MOVE_RIGHT / MOVE_DOWN ...，只是 hint，不是硬限制
    double suggestedDelta = 0.0;
    std::string reason;
};

// V7.1 Modified：block-level move / spacing hint。
// 這是把 region/action report 轉成 floorplanner 比較容易使用的操作提示。
// 它不是直接修改 floorplan，而是描述「哪個 block、哪個方向、建議保留多少距離」。
struct Step0FloorplanMoveHint {
    int hintId = -1;
    int sourceActionId = -1;
    int blockIndex = -1;
    std::string blockName;
    std::string blockType;
    std::string targetEdge;              // LEFT / RIGHT / TOP / BOTTOM / REGION
    std::string moveDirection;           // MOVE_LEFT / MOVE_RIGHT / MOVE_UP / MOVE_DOWN / KEEP_AWAY
    double suggestedDelta = 0.0;
    double confidence = 0.0;

    double expectedRiskReduction = 0.0;
    double estimatedAreaPenalty = 0.0;
    double estimatedWirePenalty = 0.0;
    double actionEfficiency = 0.0;

    std::string relatedActionType;
    std::string relatedConnectionsCSV;
    std::string reason;
};

// V7.1：CRISP-style floorplan health summary。
// 這不是正式 legalization / spreading，而是把 CMB 偵測到的 routing 壓力
// 轉成 Floorplanner 可以讀取的整體健康度與保守壓縮建議。
struct Step0FloorplanHealth {
    double outlineArea = 0.0;
    double blockArea = 0.0;
    double channelArea = 0.0;
    double deadspaceArea = 0.0;
    double deadspaceRatio = 0.0;

    double maxUtil = 0.0;
    double avgMaxUtil = 0.0;
    double maxCriticality = 0.0;
    double worstRegionScore = 0.0;

    // V7.2-RISA：supply-demand summary。
    double maxRisaUtil = 0.0;
    double avgRisaUtil = 0.0;
    double totalRisaOverflow = 0.0;
    double risaSupplyDemandCost = 0.0;
    int numRisaSupplyDeficitChannels = 0;
    int numRisaDetourImpactedChannels = 0;
    int numRisaMegaOverlapConnections = 0;

    // V7.3-HM：amplified guide summary。
    double maxRouterCriticality = 0.0;
    double maxAmbientUtil = 0.0;
    double totalAmbientDemand = 0.0;
    int numAmplifiedChannels = 0;

    // V7.4-Pattern：pattern guide summary。
    int numHighPredictabilityConnections = 0;
    int numLowPredictabilityConnections = 0;
    int numHighCouplingRiskConnections = 0;
    double avgPatternPredictabilityScore = 0.0;
    double maxCouplingRiskScore = 0.0;

    // V7.5-RUDY：router-independent background / hybrid demand summary。
    double maxRudyUtil = 0.0;
    double avgRudyUtil = 0.0;
    double totalRudyDemand = 0.0;
    int numRudyHotChannels = 0;
    double maxHybridUtil = 0.0;
    double avgHybridUtil = 0.0;
    double totalHybridDemand = 0.0;
    double hybridDemandCost = 0.0;
    int numHybridHotChannels = 0;

    int numChannels = 0;
    int numHotChannels = 0;        // max(utilLR, utilTB) >= floorplanHotUtilThreshold
    int numNearFullChannels = 0;   // max(utilLR, utilTB) >= floorplanNearFullUtilThreshold
    int numOverflowChannels = 0;   // max(utilLR, utilTB) >= 1.0
    int numCongestedRegions = 0;

    int numHighRiskConnections = 0;
    int numGeometryDisconnected = 0;
    int numFTRequired = 0;
    int numCapacityRisk = 0;
    int numEndpointAccessRisk = 0;
    int numEndpointBottleneckRisk = 0;
    int numSharedBottleneckRisk = 0;

    double avgBlockEndpointPressure = 0.0;
    double maxBlockEndpointPressure = 0.0;

    // V7.1 Modified：edge-level access / move-hint summary。
    int numHighEdgePressure = 0;
    int numNoAccessEdges = 0;
    int numEdgeBottleneckRisk = 0;
    int numMoveHints = 0;
    double maxEdgePressure = 0.0;
    double avgActionEfficiency = 0.0;
    double bestActionEfficiency = 0.0;

    // 將 CMB feedback 轉成 Floorplanner 可加入 SA cost 的摘要項。
    double regionActionCost = 0.0;
    double endpointActionCost = 0.0;
    double moveHintCost = 0.0;
    double floorplanFeedbackCost = 0.0;

    // CRISP 對應：每輪只做有限度 local spreading / inflation，避免破壞整體 placement。
    // 這裡輸出的是建議比例，不直接改 floorplan。
    double suggestedMaxInflationAreaRatio = 0.0;
    double suggestedMaxCompactionRatio = 0.0;
    double floorplanRiskScore = 0.0;
    std::string floorplanStatus = "UNKNOWN"; // HEALTHY / WATCH / ACTION_NEEDED
};

// V7.1：CRISP-style action hint。
// 它把「哪裡塞」轉成「Floorplanner 該在哪裡保留/增加空白」的建議。
struct Step0FloorplanAction {
    int actionId = -1;
    std::string actionType;        // WIDEN_REGION / RELAX_ENDPOINT_ACCESS / PROTECT_FROM_COMPACTION / SPREAD_HIGH_PIN_BLOCK
    int regionId = -1;
    Rect bbox;
    std::string dominantDirection; // LR / TB / MIXED / ACCESS
    double severity = 0.0;
    double suggestedGapIncrease = 0.0;
    double suggestedInflationRatio = 0.0;

    // V7.1 Modified：把 action 的 trade-off 顯式化，避免 Floorplanner 為小風險付出大 area/wire cost。
    double expectedRiskReduction = 0.0;
    double estimatedAreaPenalty = 0.0;
    double estimatedWirePenalty = 0.0;
    double actionEfficiency = 0.0;

    std::string primaryBlock;
    std::string suggestedMoveDirection;
    double suggestedDelta = 0.0;

    std::string nearbyBlocksCSV;
    std::string relatedConnectionsCSV;
    std::string reason;
};

struct Step0ConnectionGuide {
    int connectionIndex = -1;
    int src = -1;
    int dst = -1;
    int netCount = 0;

    std::string selectedPattern;   // reroute pass 後被選中的 pattern 名稱
    double selectedCost = 0.0;
    double routePriority = 0.0;    // V6.1：Router 可用這個欄位改 routing order

    // V6.1：per-connection guide。
    // preferredChannels 是 Step0 認為這條 connection 的 selectedPattern 會經過的 channel。
    // avoidChannels 則是這條 connection 的候選路徑中碰到的高風險 channel。
    // 注意：這些只是 soft guide，不能當作硬限制；正式 PATH 仍由 Router Dijkstra 決定。
    std::vector<int> preferredChannelIndices;
    std::vector<int> avoidChannelIndices;
    std::string preferredChannelsCSV;
    std::string avoidChannelsCSV;
    double bestPatternAvgUtil = 0.0;
    double guideConfidence = 0.0;

    // V7.5-RUDY：connection-level RUDY diagnostics。
    double rudyBBoxArea = 0.0;
    double rudyHpwl = 0.0;
    double rudyDensity = 0.0;
    double rudyContribution = 0.0;
    int rudyTouchedChannelCount = 0;
    std::string rudyTouchedChannelsCSV;

    // V7.4-Pattern：pattern routing 不應無條件強迫所有 connection 服從。
    // 這些欄位用來判斷這條 connection 的 selectedPattern 是否足夠可預測，
    // 以及是否有大量同方向並行需求造成 coupling-like risk。
    double patternPredictabilityScore = 0.0;   // 0~1，越高越適合由 Step0 pattern guide 強力引導
    double couplingRiskScore = 0.0;            // 0~1+，同方向 parallel demand / proximity 的 proxy
    double patternGuideStrength = 0.0;         // 給 Router soft guide 的建議強度
    std::string patternPredictabilityClass = "UNKNOWN"; // HIGH / MEDIUM / LOW / ROUTER_FREE
    std::string patternGuideMode = "SOFT";              // STRONG_SOFT / NORMAL_SOFT / WEAK_SOFT / ROUTER_FREE
    std::string patternReason;

    // V6.2：endpoint access / shared bottleneck risk。
    // 這些欄位用來判斷 route open 更可能出現在端點出口，
    // 還是所有候選 pattern 都被迫共用同一個瓶頸 corridor。
    int srcAccessChannelCount = 0;
    int dstAccessChannelCount = 0;
    int srcHealthyAccessChannelCount = 0;
    int dstHealthyAccessChannelCount = 0;
    double srcAccessMaxUtil = 0.0;
    double dstAccessMaxUtil = 0.0;
    bool endpointAccessRisk = false;
    bool endpointBottleneckRisk = false;

    bool sharedBottleneckRisk = false;
    double sharedBottleneckScore = 0.0;
    std::vector<int> commonBottleneckChannelIndices;
    std::string commonBottleneckChannelsCSV;

    // V7.2-RISA：NBB / mega-cell-like blockage hints。
    // RISA 對 net bounding box overlap mega cell 做 partitioning；Problem E 中 hard/edge
    // block 等同 zero-porosity routing blockage，因此若 connection bbox 覆蓋 hard/edge block，
    // CMB 會標記 detour demand 與 NBB partition hint。
    int risaMegaOverlapCount = 0;
    double risaDetourDemandAdded = 0.0;
    std::string risaBlockedBlocksCSV;
    std::string risaNbbPartitionHint;

    // V6.3：將 connection guide 與 congested regions 連結。
    // preferredRegions：selectedPattern 經過的高風險區域。
    // avoidRegions：候選路徑碰到、但不建議走的高風險區域。
    // commonBottleneckRegions：所有候選共同被迫經過的高風險區域。
    std::string preferredRegionsCSV;
    std::string avoidRegionsCSV;
    std::string commonBottleneckRegionsCSV;

    // V5：Route-open risk analysis。
    // 這些欄位不是正式 routing 結果，而是 Step0 的早期風險判斷。
    bool channelOnlyConnected = false;
    bool ftEnabledConnected = false;
    bool geometryOpenRisk = false;
    bool capacityOpenRisk = false;

    int totalCandidateCount = 0;
    int healthyCandidateCount = 0;
    double bestPatternMaxUtil = 0.0;
    bool hasProjectedPattern = false; // 至少一個 candidate 能有效投影到 actual channel
    double lackOfAlternative = 1.0;
    double openRiskScore = 0.0;
    std::string openRiskType = "UNKNOWN"; // OK / FT_REQUIRED / CAPACITY_RISK / GEOMETRY_DISCONNECTED / NO_PATTERN
};

struct Step0CongestionMapResult {
    std::vector<Step0ChannelPressure> channelPressure;
    std::vector<Step0ConnectionGuide> connectionGuide;

    // V6.3：相鄰 high-util / high-criticality channels 形成的 bottleneck regions。
    std::vector<Step0CongestedRegion> congestedRegions;

    // V7.1：CRISP-style floorplan feedback。
    Step0FloorplanHealth floorplanHealth;
    std::vector<Step0FloorplanAction> floorplanActions;

    // V7.1 Modified：更細的 floorplanner feedback。
    std::vector<Step0BlockEdgePressure> blockEdgePressure;
    std::vector<Step0FloorplanMoveHint> floorplanMoveHints;

    // 原始 pattern demand 的 raw utilization（不含 RISA/RUDY/hybrid 放大）。
    // maxUtil/avgMaxUtil 保留給相容舊欄位，語意等同 maxRawUtil/avgRawUtil。
    double maxUtil = 0.0;
    double avgMaxUtil = 0.0;
    double maxRawUtil = 0.0;
    double avgRawUtil = 0.0;

    // 各模型的最大 utilization，用來避免 raw/model 混用。
    double maxRisaUtil = 0.0;
    double maxHybridUtil = 0.0;
    double maxModelUtil = 0.0; // max(raw, RISA, hybrid)

    double maxCriticalityRaw = 0.0;

    std::vector<double> channelCriticality() const;

    // V7.3-HM：Router 可用 ambient demand 作為 soft cost guide。
    // 注意：這不應作為 hard feasibility，只是把預估熱區當成 background demand。
    std::vector<double> channelAmbientDemandLR() const;
    std::vector<double> channelAmbientDemandTB() const;

    // V6.1：Router 可用此向量替代單純 netCount sorting。
    // 長度等於 design.connections.size()。
    std::vector<double> connectionRoutePriority() const;
};

class CongestionMapBuilder {
public:
    struct Options {
        // 是否產生 Z-HVH / Z-VHV。第一版可開啟，但若覺得太重可關掉。
        bool enableZPattern = true;

        // 初始 probabilistic assignment 後，做幾輪 cheap pattern reroute。
        // 0 = 只做平均分配；1 = 建議第一版。
        int reroutePasses = 1;

        // 是否輸出圖片與 CSV。
        bool exportFiles = true;
        std::string csvPath = "step0_congestion_map.csv";
        std::string svgPath = "step0_congestion_map.svg";

        // V5：輸出每條 connection 的 open-risk / routing-risk 分析。
        bool exportConnectionRiskCSV = true;
        std::string connectionRiskCsvPath = "step0_connection_risk.csv";

        // V6.1：輸出每條 connection 的 preferred / avoid channel guide。
        bool exportConnectionGuideCSV = true;
        std::string connectionGuideCsvPath = "step0_connection_guide.csv";

        // V6.3：輸出 congested region clustering。
        bool enableRegionClustering = true;
        bool exportCongestedRegionsCSV = true;
        std::string congestedRegionsCsvPath = "step0_congested_regions.csv";
        double regionUtilThreshold = 0.75;
        double regionCriticalityThreshold = 0.25;
        double regionAdjacencyTolerance = 1.0e-3;
        bool drawRegionOverlay = true;
        int maxRegionOverlays = 24;  // V7.5 hotspot refinement: main SVG 預設強調 top regions。

        // V7.1：CRISP-style floorplan feedback 輸出。
        bool enableFloorplanFeedback = true;
        bool exportFloorplanHealthCSV = true;
        bool exportFloorplanActionsCSV = true;
        bool drawFloorplanActionOverlay = true;  // V7.5 interactive visual: 預設顯示 WIDEN_REGION 等 floorplanner action 框，但用較粗、半透明輪廓和 hot region 區分。
        std::string floorplanHealthCsvPath = "step0_floorplan_health.csv";
        std::string floorplanActionsCsvPath = "step0_floorplan_actions.csv";

        // V7.1 Modified：edge-level access pressure、move hints、delta report。
        bool exportBlockAccessPressureCSV = true;
        bool exportFloorplanMoveHintsCSV = true;
        bool exportFloorplanDeltaCSV = true;
        bool drawBlockAccessPressureOverlay = false; // endpoint overlay 預設仍關閉，避免主圖過亂；需要看 block-edge access 時再開。
        bool drawMoveHintOverlay = true;        // V7.5 interactive visual: 預設顯示 move hint，但箭頭改畫在 block 外側，避免擠到 block 名稱。
        std::string blockAccessPressureCsvPath = "step0_block_access_pressure.csv";
        std::string floorplanMoveHintsCsvPath = "step0_floorplan_move_hints.csv";
        std::string floorplanDeltaCsvPath = "step0_floorplan_delta.csv";
        std::string previousFloorplanHealthCsvPath = "step0_floorplan_health_prev.csv";

        int maxFloorplanActions = 80;
        int maxActionOverlays = 18;
        int maxFloorplanMoveHints = 120;
        int maxMoveHintOverlays = 24;
        double floorplanHotUtilThreshold = 0.75;
        double floorplanNearFullUtilThreshold = 0.90;
        double floorplanHighRiskConnectionThreshold = 0.60;
        double floorplanPinPressureScale = 1000.0;
        double crispMaxInflationPerIter = 0.01;  // CRISP 每輪限制 inflation，避免過度擾動。
        double crispTargetMaxUtil = 0.85;
        double actionSearchPaddingRatio = 0.08;
        double actionMinGapIncrease = 5.0;
        double actionMaxGapIncrease = 80.0;

        // V7.2-RISA：supply-demand analysis。
        bool enableRisaSupplyDemand = true;
        bool exportRisaSupplyDemandCSV = true;
        std::string risaSupplyDemandCsvPath = "step0_risa_supply_demand.csv";

        // V7.3-HM：amplified congestion estimation / ambient demand。
        bool enableAmplifiedCongestionGuide = true;
        bool exportAmplifiedGuideCSV = true;
        std::string amplifiedGuideCsvPath = "step0_amplified_guide.csv";
        // 對應 Hadsell-Madden：低於 80% capacity 的估計需求不應影響 router；
        // 高於 120% capacity 的估計需求要強化。
        double amplifyLowUtilCutoff = 0.80;
        double amplifyHighUtilCutoff = 1.20;
        double amplifyLowFactor = 0.0;
        double amplifyMidFactor = 1.0;
        double amplifyHighFactor = 1.2;
        // ambient demand = amplifiedStaticDemand * strength * decay，並限制不超過 cap 的一定比例。
        // decayFactor 保留給未來多輪 RRR：越後期 ambient 影響越小。
        double ambientDemandStrength = 0.35;
        double ambientDecayFactor = 1.0;
        double ambientMaxFractionOfCap = 0.50;
        // 產生 Router criticality 時，RISA 與 HM amplification 都會納入。
        double amplifiedCriticalityWeight = 0.75;
        double amplifiedRisaCriticalityWeight = 0.35;

        // V7.4-Pattern：Pattern routing predictability / coupling-aware guide。
        bool enablePatternPredictabilityRefinement = true;
        bool exportPatternPredictabilityCSV = true;
        std::string patternPredictabilityCsvPath = "step0_pattern_predictability.csv";

        // V7.5-RUDY：router-independent background demand。
        // RUDY 不預測單條 connection 會選哪個 L/Z pattern，而是將 netCount*HPWL
        // 以 uniform density 方式分布到 enclosing rectangle 內的 actual channels。
        bool enableRudyBackgroundDemand = true;
        bool exportRudyBackgroundCSV = true;
        std::string rudyBackgroundCsvPath = "step0_rudy_background.csv";

        // V7.5 Dashboard refinement：把多模型訊號整理成少數 top lists 與 conflict diagnostics。
        // 這不是新增 congestion 模型，而是讓 Raw / RISA / RUDY / Hybrid / Router guide 的界線更清楚。
        bool exportDashboardCSV = true;
        bool exportSignalConflictCSV = true;
        bool exportTextReport = true;
        std::string dashboardCsvPath = "step0_dashboard.csv";
        std::string signalConflictCsvPath = "step0_signal_conflicts.csv";
        std::string textReportPath = "step0_cmb_report.txt";
        int dashboardTopK = 10;
        double conflictRawLowThreshold = 0.75;
        double conflictRawNearFullThreshold = 0.90;
        double conflictModelHotThreshold = 1.00;
        double conflictRouterCriticalityThreshold = 0.75;
        // rudyDemandScale 控制 RUDY 背景需求強度；它是 background，不應直接壓過 pattern guide。
        double rudyDemandScale = 0.35;
        double rudyMinBBoxSize = 1.0;
        // hybridDemand = patternWeight * patternDemand + (1-patternWeight) * rudyDemand + RISA extra demand。
        // Router guide 偏向 pattern；floorplan/region health 偏向 hybrid/RUDY。
        double rudyHybridPatternWeight = 0.65;
        double rudyCriticalityWeight = 0.35;
        double rudyRegionScoreWeight = 0.35;
        double rudyFloorplanCostWeight = 0.40;
        // 只有 high-risk / high-demand / blockage-overlap connection 才展開 bbox 外側 Z。
        // 這避免 CMB 對所有 connection 都枚舉過多 detour，維持 pattern routing 的快速與可預測性。
        bool conditionalDirectionalZPattern = true;
        int conditionalOuterZNetThreshold = 900;
        double conditionalOuterZBboxAreaRatio = 0.08;
        // predictability scoring
        double patternSmallBboxAreaRatio = 0.03;
        double patternLargeBboxAreaRatio = 0.18;
        double patternHighNetCountThreshold = 1500.0;
        double patternCostSeparationScale = 0.25;
        double patternCouplingRiskThreshold = 0.65;
        double patternLowPredictabilityThreshold = 0.35;
        double patternHighPredictabilityThreshold = 0.70;
        double patternGuideMaxStrength = 1.0;
        // RISA cost 使用 Max(D - t*S, 0)，t < 1 代表較積極地保留 supply margin。
        double risaSupplySafetyFactor = 0.85;
        double risaMinEffectiveCapScale = 0.35;
        double risaHardAdjacencySupplyDiscount = 0.18;
        double risaEdgeAdjacencySupplyDiscount = 0.12;
        double risaSoftAdjacencySupplyDiscount = 0.04;
        double risaDetourDemandRatio = 0.20;
        double risaNbbPaddingRatio = 0.04;
        double risaCriticalityWeight = 0.65;
        double risaActionCostWeight = 0.45;

        // V7.1 Modified：edge-level endpoint pressure 門檻。
        double edgePressureHotThreshold = 0.65;
        double edgePressureCriticalThreshold = 1.00;
        double edgeAccessConservativeSupplyFactor = 0.25;
        double moveHintMinConfidence = 0.45;
        double actionAreaPenaltyWeight = 0.40;
        double actionWirePenaltyWeight = 0.35;

        // V6.3：Z-shape 方向候選。
        // 原本 Z-HVH / Z-VHV 使用 bbox 中線；這版額外加入 bbox 外側的
        // left/right/top/bottom guide trunk，用來更完整表示四個繞行方向。
        bool enableDirectionalZPattern = true;
        double zOuterMarginRatio = 0.20;
        double zOuterMarginMin = 10.0;

        // V5：在 SVG 上疊加高 open-risk connection flyline。
        bool drawOpenRiskOverlay = true;
        int maxRiskFlylines = 50;
        double openRiskUtilThreshold = 0.95;

        // V6.1：pattern 初始權重。
        // Probabilistic congestion prediction 的精神是 L/Z 不該完全等權。
        // 直線若存在，通常是最自然候選；L-shape 是主力；Z-shape 是少量 detour。
        bool useAdaptivePatternWeights = true;
        double straightWeight = 1.00;
        double totalLShapeWeight = 0.70;
        double totalZShapeWeight = 0.30;

        // V6.1：connection guide 判斷門檻。
        double preferredHealthyUtilThreshold = 0.95;
        double avoidUtilThreshold = 0.85;
        double avoidCriticalityThreshold = 0.35;
        int maxPreferredChannels = 24;
        int maxAvoidChannels = 24;

        // V6.2：route-open risk 強化。
        // endpoint access：source / destination block 旁邊是否有可進入的 channel，
        // 且這些 access channel 是否已經接近滿載或被 Step0 標為 critical。
        double endpointAccessUtilThreshold = 0.85;
        double endpointAccessCriticalityThreshold = 0.35;

        // shared bottleneck：若所有 pattern candidates 都被迫經過同一組高風險 channel，
        // 即使目前尚未 overflow，也代表替代路徑不足，後續 Router 容易 open / overflow。
        double sharedBottleneckUtilThreshold = 0.85;
        double sharedBottleneckCriticalityThreshold = 0.35;
        int maxCommonBottleneckChannels = 24;

        // 是否在 stderr 印出 top congested channels。
        bool verbose = true;
        int topPrintCount = 12;

        // Step0 是 guide，不是正式 Router；權重不應過大。
        double wWire = 0.0002;
        double wCongestion = 1.0;
        double wPeak = 3.0;
        double wHistory = 0.6;
        double wNarrow = 0.8;
        double wHardBlockage = 8.0;
        double wSoftBlockage = 1.0;

        // 小容量方向分量被 pattern 使用時，criticality 會放大。
        double narrowComponentCap = 300.0;

        // 投影 tolerance。0 代表只看線段是否真的穿過 channel rect。
        // 若之後發現太多 segment 沒有投影到 channel，可設為 1~5um。
        double projectionTolerance = 0.0;
    };

    CongestionMapBuilder();
    explicit CongestionMapBuilder(Options opt);

    // 主要入口：建立 congestion map / routing guide。
    Step0CongestionMapResult build(const Design& design) const;

    // 輸出檔案。build() 若 options.exportFiles=true 會自動呼叫。
    void exportCSV(const Design& design, const Step0CongestionMapResult& result, const std::string& path) const;
    void exportConnectionRiskCSV(const Design& design, const Step0CongestionMapResult& result, const std::string& path) const;
    void exportConnectionGuideCSV(const Design& design, const Step0CongestionMapResult& result, const std::string& path) const;
    void exportCongestedRegionsCSV(const Design& design, const Step0CongestionMapResult& result, const std::string& path) const;
    void exportFloorplanHealthCSV(const Design& design, const Step0CongestionMapResult& result, const std::string& path) const;
    void exportFloorplanActionsCSV(const Design& design, const Step0CongestionMapResult& result, const std::string& path) const;
    void exportBlockAccessPressureCSV(const Design& design, const Step0CongestionMapResult& result, const std::string& path) const;
    void exportFloorplanMoveHintsCSV(const Design& design, const Step0CongestionMapResult& result, const std::string& path) const;
    void exportFloorplanDeltaCSV(const Design& design, const Step0CongestionMapResult& result, const std::string& path) const;
    void exportRisaSupplyDemandCSV(const Design& design, const Step0CongestionMapResult& result, const std::string& path) const;
    void exportAmplifiedGuideCSV(const Design& design, const Step0CongestionMapResult& result, const std::string& path) const;
    void exportPatternPredictabilityCSV(const Design& design, const Step0CongestionMapResult& result, const std::string& path) const;
    void exportRudyBackgroundCSV(const Design& design, const Step0CongestionMapResult& result, const std::string& path) const;
    void exportDashboardCSV(const Design& design, const Step0CongestionMapResult& result, const std::string& path) const;
    void exportSignalConflictCSV(const Design& design, const Step0CongestionMapResult& result, const std::string& path) const;
    void exportTextReport(const Design& design, const Step0CongestionMapResult& result, const std::string& path) const;
    void exportSVG(const Design& design, const Step0CongestionMapResult& result, const std::string& path) const;
    void printSummary(const Design& design, const Step0CongestionMapResult& result) const;

private:
    Options opt_;
};
