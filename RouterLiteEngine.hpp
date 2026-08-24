#pragma once

struct Design;

namespace routerlite {

// Runs the attachment's R2s2 baseline on an already floorplanned Design and
// materializes the selected (including split-allocation) paths in design.routes.
bool runBaseline(Design& design);

} // namespace routerlite
