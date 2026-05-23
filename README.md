下面是適合你目前這份 `problemE` 專案的 **VS2022 詳細使用說明**。你可以直接放進 `README.md`。

# Problem E Solver：VS2022 使用說明

## 1. 專案資料夾結構

解壓縮後建議放在：

```text
C:\problemE\problemE
```

你的資料夾大概會長這樣：

```text
C:\problemE\problemE
 ├── problemE.sln
 ├── problemE.vcxproj
 ├── main.cpp
 ├── Parser.cpp / Parser.hpp
 ├── Floorplanner.cpp / Floorplanner.hpp
 ├── ChannelBuilder.cpp / ChannelBuilder.hpp
 ├── Router.cpp / Router.hpp
 ├── Evaluator.cpp / Evaluator.hpp
 ├── OutputWriter.cpp / OutputWriter.hpp
 ├── Logger.cpp / Logger.hpp
 ├── DataModel.cpp / DataModel.hpp
 ├── Utility.cpp / Utility.hpp
 ├── testcase
 │    ├── case00.csv
 │    ├── case30.csv
 │    └── case50.csv
 └── result
```

其中：

```text
testcase/   放 input csv
result/     放 output cfg
```

---

# 2. 用 VS2022 開啟專案

請開啟：

```text
C:\problemE\problemE\problemE.sln
```

不要直接開 `main.cpp`。

正確方式：

```text
Visual Studio 2022
  → 開啟專案或方案
  → 選 problemE.sln
```

如果你只是直接開 `main.cpp`，會沒有完整的專案設定，也會找不到「偵錯 → 命令引數」。

---

# 3. 選擇編譯組態

VS2022 上方會有：

```text
Debug / Release
x86 / x64
```

建議你先用：

```text
Release | x64
```

或如果你要 debug，用：

```text
Debug | x64
```

目前建議：

```text
一般測試：Release x64
追 bug：Debug x64
```

---

# 4. 設定 C++17

因為 `main.cpp` 有用到：

```cpp
#include <filesystem>
```

所以需要 C++17。

設定位置：

```text
右鍵專案 problemE
  → 屬性
    → 組態屬性
      → C/C++
        → 語言
          → C++ 語言標準
```

設定成：

```text
ISO C++17 標準 (/std:c++17)
```

注意，要右鍵 **專案名稱 problemE**，不是右鍵 `main.cpp`。

---

# 5. Debug 模式常見錯誤：/O2 和 /RTC1 不相容

如果你看到：

```text
'/O2' 和 '/RTC1' 的命令列選項不相容
```

代表你在 Debug 模式同時開了最佳化 `/O2` 和 runtime check `/RTC1`。

解法：

```text
右鍵專案 problemE
  → 屬性
    → 組態屬性
      → C/C++
        → 最佳化
          → 最佳化
```

Debug 模式請設成：

```text
已停用 (/Od)
```

Release 模式可以用：

```text
最大化速度 (/O2)
```

建議設定：

```text
Debug x64:
  最佳化 = 已停用 (/Od)

Release x64:
  最佳化 = 最大化速度 (/O2)
```

---

# 6. 設定命令引數

程式需要 input csv 和 output 位置。

到這裡設定：

```text
右鍵專案 problemE
  → 屬性
    → 組態屬性
      → 偵錯
        → 命令引數
```

填入：

```text
"C:\problemE\problemE\testcase\case00.csv" -o "C:\problemE\problemE\result" --alpha 0.2
```

也可以用 `--o`：

```text
"C:\problemE\problemE\testcase\case00.csv" --o "C:\problemE\problemE\result" --alpha 0.2
```

目前 `main.cpp` 支援：

```text
-o
--o
--output
```

---

# 7. Output 檔名規則

如果 `-o` 後面給的是資料夾：

```text
-o "C:\problemE\problemE\result"
```

程式會自動產生檔名：

```text
07blk05240238.cfg
```

格式是：

```text
<兩位數block數>blk<月日時分>.cfg
```

例如：

```text
07blk05240238.cfg
```

意思是：

```text
07      = 7 個 block
blk     = block
0524    = 5 月 24 日
0238    = 02:38
```

所以輸出完整路徑會像：

```text
C:\problemE\problemE\result\07blk05240238.cfg
```

如果你想自己指定完整檔名，也可以：

```text
"C:\problemE\problemE\testcase\case00.csv" -o "C:\problemE\problemE\result\case00_test.cfg" --alpha 0.2
```

這樣就會直接輸出：

```text
C:\problemE\problemE\result\case00_test.cfg
```

---

# 8. 建議工作目錄

在 VS2022 屬性裡：

```text
右鍵專案 problemE
  → 屬性
    → 組態屬性
      → 偵錯
        → 工作目錄
```

可以設成：

```text
C:\problemE\problemE
```

不過你現在命令引數都用完整路徑，所以工作目錄不設也通常可以。

---

# 9. 編譯方式

在 VS2022 上方選單：

```text
建置
  → 建置方案
```

或快捷鍵：

```text
Ctrl + Shift + B
```

如果改過很多設定，建議：

```text
建置
  → 清除方案
建置
  → 重新建置方案
```

---

# 10. 執行方式

設定完命令引數後，按：

```text
Ctrl + F5
```

或：

```text
偵錯
  → 開始執行但不偵錯
```

如果你要 debug，可以按：

```text
F5
```

---

# 11. Terminal 輸出說明

成功執行後，terminal 會看到類似：

```text
========== Problem E Greedy Architecture ==========
Input file              : C:\problemE\problemE\testcase\case00.csv
Output cfg              : C:\problemE\problemE\result\07blk05240238.cfg
Blocks                  : 7
Connections             : 11
Channels                : 18
Alpha                   : 0.200
Runtime                 : 0.123 sec

========== Floorplan ==========
Max outline             : 2916.000 x 2220.000
Output outline          : 2916.000 x 2220.000
OutlineArea             : 6473520.000
BlockOverlap            : NO  count=0
OutlineViolation        : NO  count=0

========== Channel ==========
Channel count           : 18
Total channel overflow  : 0.000
Max channel overflow    : 0.000

========== Routing ==========
Route paths             : 11
Open paths              : 0
TotalWireLength         : 3052700.354
Total FT overflow       : 0.000
Max FT overflow         : 0.000

========== Cost ==========
Cost                    : 7084060.071
Formula                 : OutlineArea + alpha * TotalWireLength

========== Penalty condition ==========
Channel overflow        : PASS
Feedthrough overflow    : PASS

========== Fail condition ==========
Format failed           : PASS
Block overlap           : PASS
Routing open            : PASS
Outline violation       : PASS

========== Final ==========
Status                  : LEGAL
Runtime                 : 0.123 sec
```

如果看到：

```text
Status : FAIL
```

代表目前輸出可能有：

```text
Block overlap
Routing open
Outline violation
Format failed
```

其中一種錯誤。

---

# 12. 常見錯誤處理

## 錯誤 1：找不到輸入檔

可能訊息：

```text
[Parser] Cannot open input
```

檢查命令引數：

```text
"C:\problemE\problemE\testcase\case00.csv"
```

確認檔案真的存在。

---

## 錯誤 2：輸出 cfg 失敗

可能訊息：

```text
[OutputWriter] Failed to write cfg
```

檢查：

```text
C:\problemE\problemE\result
```

是否存在，或是否有權限。

目前新版 `main.cpp` 會自動建立資料夾，但如果路徑打錯，例如：

```text
C:\problemE\problemE\reslut
```

就會輸出到錯的資料夾。

---

## 錯誤 3：沒有「命令引數」設定頁

這通常是因為你右鍵了 `main.cpp`。

請右鍵：

```text
方案總管 → problemE 專案名稱
```

不是右鍵：

```text
main.cpp
```

正確屬性頁左邊應該有：

```text
組態屬性
  ├── 一般
  ├── 偵錯
  ├── VC++ 目錄
  ├── C/C++
  ├── 連結器
```

---

## 錯誤 4：/O2 和 /RTC1 不相容

Debug 模式請把最佳化改成：

```text
已停用 (/Od)
```

Release 模式才使用：

```text
最大化速度 (/O2)
```

---

# 13. 測不同 testcase

case00：

```text
"C:\problemE\problemE\testcase\case00.csv" -o "C:\problemE\problemE\result" --alpha 0.2
```

case30：

```text
"C:\problemE\problemE\testcase\case30.csv" -o "C:\problemE\problemE\result" --alpha 0.2
```

case50：

```text
"C:\problemE\problemE\testcase\case50.csv" -o "C:\problemE\problemE\result" --alpha 0.2
```

---

# 14. 分工建議

目前架構適合這樣分工：

```text
Parser.cpp
  負責讀 testcase csv

Floorplanner.cpp
  負責 block placement
  接下來主要優化這裡

ChannelBuilder.cpp
  負責依 block 位置切 channel

Router.cpp
  負責產生 PATH
  接下來主要優化這裡

Evaluator.cpp
  負責計算 cost / overflow / fail condition

OutputWriter.cpp
  負責輸出 cfg 格式

Logger.cpp
  負責 terminal 報告
```

接下來最重要的是：

```text
1. 修 Floorplanner.cpp
   目標：no overlap、no outline violation、edge block 合法

2. 修 Router.cpp
   目標：routing open = 0，channel overflow 盡量低

3. 修 Evaluator.cpp
   目標：wirelength / overflow 計算更接近官方 checker
```

---

# 15. 最推薦的 VS2022 設定

```text
組態：
  Release

平台：
  x64

C++ 語言標準：
  ISO C++17 (/std:c++17)

命令引數：
  "C:\problemE\problemE\testcase\case00.csv" -o "C:\problemE\problemE\result" --alpha 0.2

工作目錄：
  C:\problemE\problemE
```

這樣按 `Ctrl + F5` 就可以跑，輸出會自動放到：

```text
C:\problemE\problemE\result
```
