#include "Router.hpp"
#include "RouterLiteEngine.hpp"

#include <algorithm>
#include <iostream>

using namespace std;

void Router::setFTOverflowCostEnabled(bool enabled) {
    ftOverflowCostEnabled = enabled;
}

void Router::setSoftFTCostRelaxed(bool enabled) {
    softFTCostRelaxed = enabled;
}

void Router::setContactAwareCostEnabled(bool enabled) {
    contactAwareCostEnabled = enabled;
}

void Router::setDetailedFailureAnalysisEnabled(bool enabled) {
    detailedFailureAnalysisEnabled = enabled;
}

const vector<Router::FailureCertificate>& Router::failureCertificates() const {
    return lastFailureCertificates;
}

void Router::run(Design& design) {
    lastFailureCertificates.clear();

    // Router-lite owns its scoring/search policy.  The legacy toggles above are
    // intentionally retained only to keep the floorplanner-facing API stable.
    (void)ftOverflowCostEnabled;
    (void)softFTCostRelaxed;
    (void)contactAwareCostEnabled;
    const bool gateOk = routerlite::runBaseline(design);

    if (!detailedFailureAnalysisEnabled && gateOk) return;

    for (const Connection& conn : design.connections) {
        if (conn.src < 0 || conn.dst < 0 ||
            conn.src >= static_cast<int>(design.blocks.size()) ||
            conn.dst >= static_cast<int>(design.blocks.size()) ||
            conn.netCount <= 0) {
            continue;
        }

        const string& src = design.blocks[conn.src].spec.name;
        const string& dst = design.blocks[conn.dst].spec.name;
        int delivered = 0;
        const RoutePath* diagnostic = nullptr;
        for (const RoutePath& route : design.routes) {
            if (route.srcBlock != src || route.dstBlock != dst) continue;
            if (!diagnostic) diagnostic = &route;
            if (!route.open) delivered += max(0, route.netCount);
        }
        if (delivered >= conn.netCount) continue;

        FailureCertificate cert;
        cert.conn = conn;
        cert.diagnosticOpen = true;
        if (diagnostic) cert.diagnosticPath = *diagnostic;
        lastFailureCertificates.push_back(std::move(cert));
    }
}

void Router::printDetourReport(const Design& design) const {
    int openRoutes = 0;
    for (const RoutePath& route : design.routes) {
        if (route.open) ++openRoutes;
    }
    if (openRoutes > 0) {
        cerr << "[RouterLite] open_routes=" << openRoutes
             << " routes=" << design.routes.size() << "\n";
    }
}
