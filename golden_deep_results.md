# Deep golden-answer audit (2026-08-11)

The package contains exactly one checker-valid CFG for each case. Every case beats its first-place reference by at least 5% in either TotalWireLength or TotalArea.

| Case | Passing metric | First place | Deep golden | Improvement | Change vs previous 5% set |
|---:|---|---:|---:|---:|---:|
| 0 | TotalWireLength | 1,404,302 | 1,289,877.558 | 8.1481% | unchanged |
| 1 | TotalArea | 5,406,271 | 5,116,080.000 | 5.3677% | -720.000 area |
| 2 | TotalArea | 25,558,911 | 22,094,490.000 | 13.5547% | unchanged |
| 3 | TotalArea | 14,327,456 | 13,563,222.000 | 5.3341% | -42,361.000 area |
| 4 | TotalArea | 22,893,825 | 18,234,060.000 | 20.3538% | unchanged |
| 5 | TotalArea | 27,421,020 | 17,949,228.779 | 34.5421% | -4,520,946.221 area |
| 6 | TotalArea | 3,365,461 | 3,155,025.000 | 6.2528% | -6,975.000 area |
| 7 | TotalArea | 4,838,400 | 4,450,519.380 | 8.0167% | unchanged |

Near-lower-bound packings:

- Case 1 block-area sum: 5,115,888; golden area is only 192 above it.
- Case 5 block-area sum: 17,949,129; golden area is only 99.779 above it.
- Case 6 block-area sum: 3,130,000; the remaining 25,025 is primarily forced by the two fixed hard macros and their cavity.

Final checker audit for all eight packaged files:

- Format failed: PASS
- Block overlap: PASS
- Routing open: PASS
- Outline violation: PASS
- Status: LEGAL_WITH_PENALTY

More aggressive case 7 geometries reached 4,255,000 area, but produced open routes and were rejected. Long alpha/seed searches for cases 0, 2, 3, 4, and 7 also produced no checker-valid candidate that dominated the selected CFG in the requested passing metric.
