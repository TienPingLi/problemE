# Golden 5% result audit (2026-08-11)

Each case satisfies the requested condition: at least one of TotalWireLength or TotalArea is at most 95% of that case's first-place value.

| Case | Passing metric | First place | 95% threshold | Golden result | Improvement |
|---:|---|---:|---:|---:|---:|
| 0 | TotalWireLength | 1,404,302 | 1,334,086.900 | 1,289,877.558 | 8.1481% |
| 1 | TotalArea | 5,406,271 | 5,135,957.450 | 5,116,800.000 | 5.3544% |
| 2 | TotalArea | 25,558,911 | 24,280,965.450 | 22,094,490.000 | 13.5547% |
| 3 | TotalArea | 14,327,456 | 13,611,083.200 | 13,605,583.000 | 5.0384% |
| 4 | TotalArea | 22,893,825 | 21,749,133.750 | 18,234,060.000 | 20.3538% |
| 5 | TotalArea | 27,421,020 | 26,049,969.000 | 22,470,175.000 | 18.0549% |
| 6 | TotalArea | 3,365,461 | 3,197,187.950 | 3,162,000.000 | 6.0456% |
| 7 | TotalArea | 4,838,400 | 4,596,480.000 | 4,450,519.380 | 8.0167% |

Final checker audit for all eight files:

- Format failed: PASS
- Block overlap: PASS
- Routing open: PASS
- Outline violation: PASS
- Status: LEGAL_WITH_PENALTY

`LEGAL_WITH_PENALTY` means there are scoring penalties (for example channel/feedthrough/edge warnings) in one or more cases; none of the four hard fail conditions above is triggered.
