# 31 · OmniGraph 與 ROS 2 橋接的真相:五個「成功了但沒有在跑」

多車場景幾乎一定會用到 OmniGraph——每台車一組節點,發 odom、收 twist、接 TF。而這一層最麻煩的地方是:**它的失敗形狀幾乎全部是「回傳成功、沒有錯誤、就是沒有在跑」**。

這一篇把五個這樣的位置講清楚:模組載入順序、extension 啟用、節點型別怎麼寫、圖怎麼接線、以及圖到底跟著誰 tick。

> **驗證狀態**:內容來自一段多車 + 多樓層的 Isaac Sim 場域前期研究(2026-07 至 08),在 Isaac Sim 5.1.2 的 `isaacsim.ros2.bridge` 上實測。所有數字與 log 字串都是實測值。⚠ **未在 6.0.x 上複驗**;6.0 的 bridge 已重組成門面 + 四個實作(見 [14 篇](../../6.0.1/14-ros2-bridge-6.0-architecture/README.md)),extension 名稱雖然沒變,行為要自己驗一次。

## 1. `SimulationApp` 之前不可以 import `pxr`

pip 安裝的 `isaacsim[all]` 會把 `usd_exchange` 一起裝進來,而它在 site-packages 頂層放了一份 `pxr`;Isaac 自己那份在 `isaacsim/extscache/omni.usd.libs-*/pxr`。

**這是兩套 C++ USD,誰先被 import 誰贏。** 而 Kit 稍後仍然會載入自己那份,於是 boost::python 的轉換器註冊在不同型別上。

症狀不是 `ImportError`,是啟動九秒後:

```
TypeError: No to_python (by-value) converter found for C++ type:
           std::vector<pxrInternal_v0_25_11__pxrReserved__::SdfPath, ...>
RuntimeError: Caught an unknown exception!
```

**看起來完全像 Isaac 內部壞掉**,跟「我在檔案開頭多 import 了一個模組」沒有任何外觀上的關聯。

判準很簡單但要嚴格執行:

> **啟動腳本的頂層 import 只能有 stdlib**,其餘一律延到 `SimulationApp(...)` 之後。

⚠ **間接 import 一樣算。** 頂層寫 `from my_pkg.ros2_graph import NODE_TYPES` 就中了——因為那個模組自己 `from pxr import Usd`。你的檔案裡看不到 `pxr` 三個字,照樣爆。

## 2. 「啟用成功」不是它活著的證明

pip 版的 base python 體驗(`isaacsim.exp.base.python.kit`)**沒有** `isaacsim.ros2.bridge`,所以 `isaacsim.ros2.bridge.*` 的節點型別一個都不存在,`og.Controller` 會丟 `Could not create node using unrecognized type ...`。

啟用它之後,log 長這樣:

```
啟用 isaacsim.ros2.bridge -> True
[234.513s] [ext: isaacsim.ros2.bridge-5.1.2] startup
[234.530s] [ext: isaacsim.ros2.bridge-5.1.2] shutdown     ← 17 ms 後自己收掉
```

**回傳值是 `True`,而它已經死了。** 理由只在 log 裡一行,而那一行不會冒到你的例外處理裡。

### 2.1 生效證明要看型別在不在

```python
registered = set(og.get_registered_nodes())          # 回的是字串
alive = "isaacsim.ros2.bridge.ROS2PublishOdometry" in registered
```

實測那台註冊了 **614 個節點型別**,用到的 11 個全部在內——包含 `isaacsim.ros2.bridge.*` 那六個。

### 2.2 ⚠ 列舉查詢一定要配正對照

這一條是整篇最容易複製到別處的:

> `og.get_registered_nodes()` 回的是**字串**。若照物件寫成 `{n.get_node_type_name() for n in ...}` 會拋 `AttributeError`,集合停在空的,於是十一個型別全印「沒有」——**包括一定存在的 `omni.graph.action.OnPlaybackTick`**。

**正對照就是拿它當試紙**:查一個你確定存在的東西。它若也回「沒有」,那是查法壞了,不是東西不在。

這與[27 篇](../../common/27-failure-mode-taxonomy/README.md) §6 是同一條規則的不同穿法——**一個回空的查詢,同時相容於「東西不在」與「我的查法有洞」兩個世界**。

### 2.3 `ros_distro` 與「不要 source 別人的 ROS」

設定項 `ros_distro` 預設 `"system_default"`(= 沒有 source 任何 ROS 2 就退回內建的),而它退回的是 **jazzy**。

那台雖然有 `/opt/ros/humble`,**不要去 source 它**——那是那台上別人的環境。要讓內建那份被找到,補 library path 即可:

```bash
ISAAC_ROS2_LIB="$VENV/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy/lib"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$ISAAC_ROS2_LIB"
```

「啟動前不要 source ROS」這條在 [14 篇](../../6.0.1/14-ros2-bridge-6.0-architecture/README.md) 也有,機制相同:內建那份與系統那份會互相蓋。

## 3. 節點型別是真的 USD 屬性,寫在 `customData` 等於沒寫

`OmniGraphNode` 的型別是**真的 USD 屬性** `token node:type`,配一個 `int node:typeVersion`。

寫進 `customData` 的話,OmniGraph 讀到空字串:

```
[omni.graph.core.plugin] Could not find node type interface for ''
```

發現的方式值得記:**數字對得上就不必猜**——五台車 × 每台 9 個節點 = 45 個 `OmniGraphNode`,而每一批警告正好 **45 條**。範圍吻合就確認了是全體而不是個案。

⚠ 後果比「幾行警告」大得多:**全 stage 的過時檢查都因此沒跑完**,而表面上只是少了幾行警告。**沉默不等於沒問題。**

## 4. 圖的連線就是普通的 USD attribute connection

這一節推翻了一個常見的假設。三件事:

1. **吃 prim 的輸入是 `rel`(關係)不是字串**,而且帶 `customData.omni.graph.relType = "target"`。`chassisPrim`、`targetPrims` 都是。
2. **連線是 `<attr>.connect`,純 USD。** 所以**整張圖可以離線寫全,不需要在 Isaac 裡跑 `og.Controller`**——這推翻了「接線只能在 Isaac 裡做」。
3. **執行埠帶 `customData = { bool isExecution = 1 }`**,沒有這個標記的 `uint` 不會被當成執行流。

第 2 點的工程價值很大:圖可以進版控、可以 review、可以離線批次生成五台車的九個節點,而不必每次開一次 Isaac。這跟 [02 篇](../../common/02-python-no-ui/README.md)「不碰 UI」是同一個方向,只是延伸到了圖這一層。

### 4.1 ⚠ 不要照出貨的樣板抄

Isaac 出貨的 `ogn/tests/usd/*Template.usda` 寫的是 `flatCacheBacking` 與 `fileFormatVersion = (1, 3)`,而**執行期實際產出的是 `fabricCacheBacking` 與 `(1, 9)`**。

**樣板比執行期舊。** 要參考長相,就在自己這台建一個真的圖匯出來,不要抄靜態檔案。這與 [21 篇](../../common/21-cad-asset-reading-and-conversion/README.md)「哪一份能用不寫在檔名上」是同一類問題——**檔案存在不代表它反映現況**。

## 5. `world.step(render=False)` 不會 tick action graph

這是最貴的一條,因為它讓「物理在跑」與「圖在跑」看起來是同一件事。

> **物理與圖掛在不同的東西上**:物理跟著 `world.step()` 走,action graph 跟著主迴圈走。
> `render=False` 只推物理,**圖完全不被求值**。

實測對照(同一個場景、跑 300 步):

| | `render=False` | `render=True` |
|---|---|---|
| 模擬時間 | 正確前進 2.000 s / 120 步 | 正確前進 |
| `OnTick` 被算幾次 | **1** | 301 |
| 其餘八個節點 | **0** | 300 |
| odom 的 ROS 2 publisher | **沒有建出來** | 每台車 1 個 |
| 收到的 odom | **0** | 296~299 / 300 |

⚠ **物理步進那一項會綠,而圖那一項是空的——兩個訊號指向不同的東西。** 不要拿「物理時間有前進」當成「模擬在跑」。

這與 [05 篇](../../common/05-ros2-bridge/README.md)記的 headless 下 OmniGraph 不 tick 是同一個家族的問題,只是那次的觸發條件是 headless,這次是 `render=False`。**共同點是:圖的求值不在物理迴圈上。**

### 5.1 算圖本身是非同步的

一次 `world.step(render=True)` 之後**立刻** `get_data()` 會拿到空的。連續錄影感覺不到(每隔幾步就算一次),**單幀預覽要自己多轉幾圈**。

## 6. 三個「檔案正常、程式沒報錯、畫面是黑的」

第一次錄影一次撞出三個缺口,而**三個的症狀完全一樣**:

**① 缺貼圖不是素色,是純黑。** `UsdUVTexture` 讀不到檔案回 `(0,0,0)`,而 `diffuseColor` 是**連到**它的,所以整個表面算出來是黑的。

**② 樓板擋住一切。** 三層疊在 z 上、每層樓板不透明,任何外部視角看到的都是屋頂。錄影時把樓板設成 invisible——⚠ **`visibility` 只影響算圖,不改變碰撞**,所以車照樣在樓板上跑。

**③ 算圖非同步**(見 §5.1)。

三個加起來的結果:錄出來 **2833 幀全黑**,而 mp4 的參數(1280×720、94 秒)完全正常,寫檔也沒有報錯。

> **「錄到了」與「錄了一片黑」在檔案層面分不出來,要抽幀看畫素才知道。**

這條的一般形式:**產出物存在 ≠ 產出物有內容**。跟 [30 篇](../../common/30-acceptance-probes-and-preregistration/README.md) §4.1 的「表頭有 64 欄而 13632 格全是 nan」是同一件事,只是換成了影像。

### 6.1 ⚠ 錄影的寫入量要自己守門

`rep.orchestrator.wait_until_complete()` 配 `BasicWriter` 會**一直寫**。踩過:九分鐘寫了 **57601 張、1.2 GB** 到共用網路磁碟。

改用 `AnnotatorRegistry` 的拉式介面自己取幀——`get_data()` 呼叫幾次就有幾幀,寫不出界。

## 7. 東西都在,只是沒有人下指令

最後一個形狀:**設定齊全與「有人驅動」是兩回事**。

電梯的門在 USD 裡是完整的——門片帶剛體與質量、門的關節是 articulation 的自由度(冒煙測試印的自由度清單裡就有)。缺的是**沒有人下指令**,所以整段都停在關的位置。

⚠ **不要靠看畫面判斷門有沒有動。** 側視角的轎廂內部很暗,兩幀擺在一起也難說。改成把**關節位置量出來**:實測兩台電梯的門都走完 `0 → 0.475 m` 的全行程。

同一次查證還抓到另一個缺口:**轎廂得先來接。** 原本車要上車時,轎廂可能還停在別層——某台電梯該走到 8.0 m 只走到 **7.589 m**,**沒有報錯**,只是差了 41 公分,而那 41 公分正好是「車站在哪裡」。時刻表補上「接車」那一段之後是 7.996 m。

## 8. 檢查清單

- [ ] 啟動腳本頂層 import 只有 stdlib,**含間接 import**
- [ ] extension 的生效證明看**節點型別在不在**,不看 `enable_extension` 的回傳值
- [ ] 每個列舉查詢都配一個「確定存在」的正對照當試紙
- [ ] 節點型別寫在 `node:type` 屬性,不是 `customData`
- [ ] 圖的長相參考自己這台匯出的真圖,不抄出貨樣板
- [ ] 確認圖有沒有在 tick,用節點被算的次數,**不用物理時間有沒有前進**
- [ ] 單幀取像多轉幾圈再 `get_data()`
- [ ] 錄影/寫檔的量自己守門,不靠 writer 停
- [ ] 驗「東西有沒有動」量關節位置,不看畫面
- [ ] 產出物驗的是**內容**(抽幀看畫素),不是檔案存不存在

---

延伸閱讀:[05 ROS2 橋接](../../common/05-ros2-bridge/README.md)(headless 下 OmniGraph 不 tick 的實案與 UDP 解耦)、[14 ROS 2 Bridge 在 6.0 的架構重組](../../6.0.1/14-ros2-bridge-6.0-architecture/README.md)、[02 不碰 UI:用 Python 操作](../../common/02-python-no-ui/README.md)、[30 驗收探針與實驗預先登記](../../common/30-acceptance-probes-and-preregistration/README.md)。
