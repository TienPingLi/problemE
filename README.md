# Problem E Greedy Architecture Split Version

這版把原本單檔 `problemE_greedy_arch.cpp` 拆成多個 `.hpp/.cpp`，方便分工。

## 架構

```text
main
 ├── Parser
 ├── Floorplanner
 ├── ChannelBuilder
 ├── Router
 ├── Evaluator
 ├── OutputWriter
 └── Logger
```

## 分工建議

- `DataModel.hpp/.cpp`：共用資料結構，大家都會 include。
- `Parser.hpp/.cpp`：讀 csv、block、outline、connection matrix。
- `Floorplanner.hpp/.cpp`：greedy floorplan，只做一次。
- `ChannelBuilder.hpp/.cpp`：依 block x-edge 做 vertical strip channel calculation。
- `Router.hpp/.cpp`：greedy/Dijkstra global routing，只做一次。
- `Evaluator.hpp/.cpp`：計算 cost、outline area、wirelength、penalty/fail condition。
- `OutputWriter.hpp/.cpp`：輸出 cfg。
- `Logger.hpp/.cpp`：terminal report。

## 編譯

```bash
make
```

或：

```bash
g++ -std=c++17 -O2 -Wall -Wextra \
  main.cpp Utility.cpp DataModel.cpp Parser.cpp Floorplanner.cpp ChannelBuilder.cpp Router.cpp Evaluator.cpp OutputWriter.cpp Logger.cpp \
  -o solver
```

## 執行

```bash
./solver "E_testcase_20260414.xlsx - case00.csv" -o case00_greedy.cfg --alpha 0.2
```

## 注意

這版目標是架構清楚，不是最佳化結果。沒有 SA，沒有 repair loop。
