# 車隊與多樓層

這一區的 4 篇處理的是**多台車 + 多樓層**這個問題域:OmniGraph 與 ROS 2 橋接、差速車的動力學、電梯與門檻、感測器。內容與 Isaac Sim 版本無關,但它自成一個域——單車的物理調參([共通區](../common/))解決不了「五台車在三層樓之間互相讓路」的問題。

素材來自一段多車 + 多樓層的場域前期研究(2026-07 至 08),在 Isaac Sim 5.1.2 上實測。⚠ **未在 6.0.x 上複驗**;6.0 的 ROS 2 bridge 已重組(見 [14 篇](../6.0.1/14-ros2-bridge-6.0-architecture/README.md)),行為要自己驗一次。

## 這一區的主線

四篇有一條共同的線,而它比任何單一結論都重要:

> **這個問題域的失敗,幾乎全部是「回傳成功、沒有錯誤、就是不對」。**

- 圖:extension 啟用回 `True` 而它 17 ms 後自己收掉
- 車:五個成因完全不同的「車不動」,而 USD 檢查、bbox、overlap 查詢**全部是綠的**
- 電梯:USD 的 joints 清單裡有那個關節,而 articulation 的自由度清單裡沒有
- 感測器:240 束全部回傳量程下限,**穩定、一致、在合理範圍內**

所以這一區花在「怎麼確認它真的在跑」的篇幅,比花在「怎麼設定」的還多。

## 篇章

| # | 主題 | 一句話 |
|---|---|---|
| [31](31-omnigraph-and-ros2-bridge-truth/README.md) | OmniGraph 與 ROS 2 橋接的真相 | `SimulationApp` 之前不能 import `pxr`(**間接 import 也算**);啟用回 `True` 不是它活著的證明;節點型別寫在 `customData` 等於沒寫;圖的連線就是普通 USD attribute connection——**整張圖可以離線寫全**;`world.step(render=False)` **不會 tick action graph**,而物理時間照樣正確前進 |
| [32](32-differential-drive-vehicle-model/README.md) | 把車做成真的會動的車 | 五個成因完全不同的「車不動」,五個都不報錯;**一個算錯的滑移指標(41%)長出一整套合理但錯誤的成因**,真值是 2~3%;腳輪半徑只有驅動輪一半就在平地被彈飛;宣告 1.0 m/s 的車撐不住 1.0,**限速之後反而更快**;車開不完路網的五個成因**沒有一個是路線規劃不好** |
| [33](33-elevator-and-multi-floor/README.md) | 電梯與多樓層 | articulation 的根不能帶自由度,轎廂因此升不起來而每層檢查都綠;承重接觸的三個數字與三個量測坑;跳動 22 mm 是**指令形狀**不是接觸;全零的陰性結果一定要配正對照;門檻上 200 mm 的洞**只在開進去時發作**;擋住轎廂的是它自己的門片;**車站在會動的地板上時,相對量會恆真** |
| [34](34-lidar-and-sensor-plausible-but-wrong/README.md) | 感測器的假數字 | 感測器埋在自己的碰撞盒裡 → 240 束全回量程下限,**與「車貼著牆」在資料上完全一樣**;視角上限是幾何算得出來的硬約束;**正對照自己的幾何錯了**;三個獨立的「看不見」成因;ROS 2 那側四個會讓數字說謊的量法 |

## 怎麼讀

**要接 ROS 2 或做多車** → 先 [31](31-omnigraph-and-ros2-bridge-truth/README.md)。圖不 tick 的話後面每一篇的量測都是假的。

**車在場景裡不動、或動得很奇怪** → [32](32-differential-drive-vehicle-model/README.md)。特別是 §2 那個算錯的指標——它示範了為什麼「改了有反應」不能當成「找對方向」。

**要做多樓層** → [33](33-elevator-and-multi-floor/README.md)。⚠ §7 那個「車在移動中的轎廂裡繼續開」是這一區代價最高的一次事故,而**兩道既有防線都沒寫錯,是它們的聯集不完整**。

**要接感測器** → [34](34-lidar-and-sensor-plausible-but-wrong/README.md)。

## 與其他區的關係

- 單車的物理調參、接觸力學、量測紀律在[共通區](../common/)。[24 篇](../common/24-nonholonomic-vehicle-control/README.md)(轉向式底盤)與本區 [32](32-differential-drive-vehicle-model/README.md)(差速底盤)互補:**兩種底盤的前提鏈不同,但「症狀都不指向成因」這件事一樣**。
- ROS 2 bridge 的基本機制在 [05 篇](../common/05-ros2-bridge/README.md),6.0 的架構重組在 [14 篇](../6.0.1/14-ros2-bridge-6.0-architecture/README.md)。
- 實驗方法與紀錄:[19](../common/19-tuning-experiment-methodology/README.md)、[27](../common/27-failure-mode-taxonomy/README.md)、[30](../common/30-acceptance-probes-and-preregistration/README.md)。
