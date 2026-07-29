# 14 · ROS 2 Bridge 在 6.0 的架構重組:一個 extension 拆成五個之後

從 Isaac Sim 5.1 升到 6.0,ROS 2 橋接這一塊的**外部介面幾乎沒變**——啟動參數照舊寫 `--enable isaacsim.ros2.bridge`,設定鍵照舊叫 `exts."isaacsim.ros2.bridge".ros_distro`,OmniGraph 節點型別名稱也照舊。但底下的組織方式換了一輪:原本一個 extension 承擔的事,現在由五個分擔。

介面沒變而結構變了,是最容易產生錯誤判斷的一種升級。本篇把變了什麼、沒變什麼、以及三個因此而生的判讀陷阱寫清楚。

延伸閱讀:[05 ROS2 橋接](../05-ros2-bridge/README.md)(bridge 的基本機制與 headless 下的 OmniGraph 陷阱)、[08 5.1 → 6.0.1 遷移風險調查](../08-migration-5.1-to-6.0-oom-risk/README.md)(場景與記憶體面的遷移)。

本篇的官方依據全部來自 `isaac-sim/IsaacSim` GitHub repo 的 tag 快照,逐條標了路徑;實測部分來自一台跑 `nvcr.io/nvidia/isaac-sim:6.0.1` 的機器,標為實測。

---

## 1. 拆了什麼

5.1 的 `isaacsim.ros2.bridge` 是一個完整的 extension:自己宣告設定、自己實作 OmniGraph 節點、自己帶 UI 與範例。6.0 把它拆成一個門面加四個實作。

| tag | `source/extensions/` 底下的 `isaacsim.ros2.*` |
|---|---|
| **v5.1.0** | `bridge`、`sim_control`、`tf_viewer`、`urdf` |
| **v6.0.1** | `bridge`、**`core`**、**`examples`**、**`nodes`**、`sim_control`、`tf_viewer`、**`ui`**、`urdf` |

四個粗體是 6.0 新增的。拆分本身記在 bridge 的 CHANGELOG 裡,發生在 extension 版本 5.0.0:

> ## [5.0.0] - 2025-11-02
> ### Changed
> - Split extension into multiple extensions.
> - isaacsim.ros2.core: Core ROS 2 libraries and backend functionality
> - isaacsim.ros2.examples: ROS 2 examples
> - isaacsim.ros2.nodes: ROS 2 OmniGraph nodes and components
> - isaacsim.ros2.ui: ROS 2 UI components

— `source/extensions/isaacsim.ros2.bridge/docs/CHANGELOG.md` @ v6.0.1

拆完之後,`isaacsim.ros2.bridge` 的 `extension.toml` 只剩四行依賴,自己不再實作任何東西:

```toml
# v6.0.1: source/extensions/isaacsim.ros2.bridge/config/extension.toml
[dependencies]
"isaacsim.ros2.core" = {}
"isaacsim.ros2.examples" = {}
"isaacsim.ros2.nodes" = {}
"isaacsim.ros2.ui" = {}

[settings]
# All ROS 2 Bridge settings are centrally defined in the `ros2.core` extension.
```

對照 5.1 的同一個檔案,`[dependencies]` 底下是 20 項(`isaacsim.core.api`、`isaacsim.sensors.*`、`omni.replicator.core`、`omni.syntheticdata` …),`[settings]` 底下是四個設定鍵的完整宣告。

**`bridge` 從實作者變成了門面(umbrella package)。** 官方 Overview 用的詞是 consolidates:

> The extension consolidates multiple ROS 2-related extensions into a unified bridge

— `source/extensions/isaacsim.ros2.bridge/docs/Overview.md` @ v6.0.1

### 1.1 對啟動參數的實際影響:沒有

門面仍然把四個實作列為依賴,而 Kit 的 extension 系統會遞迴解析依賴。所以 `--enable isaacsim.ros2.bridge` 這一行在 6.0 依然會把整套帶起來,不需要改成列舉四個。

實測(6.0.1 容器,啟動參數只給了 `--enable isaacsim.ros2.bridge --enable isaacsim.ros2.sim_control`)的 kit log:

```
[23.274s] [ext: isaacsim.ros2.core-1.9.4] startup
[23.390s] [ext: isaacsim.ros2.nodes-1.18.13] startup
[23.518s] [ext: isaacsim.ros2.examples-1.2.4] startup
[23.527s] [ext: isaacsim.ros2.ui-1.6.5] startup
[23.536s] [ext: isaacsim.ros2.bridge-5.1.2] startup
[23.540s] [ext: isaacsim.ros2.sim_control-1.6.6] startup
```

`core` 先起、`bridge` 後起——依賴先於門面,符合拓撲順序。**沒有顯式 enable 的 `core`/`nodes`/`examples`/`ui` 四個都被自動帶起來了。**

---

## 2. 陷阱一:設定鍵沒有跟著搬家

拆分之後,設定的**宣告位置**從 `bridge` 移到了 `core`。但鍵的**名字**沒有改:

```toml
# v6.0.1: source/extensions/isaacsim.ros2.core/config/extension.toml
[settings]
# ROS 2 Bridge settings are centralized in the core extension.
exts."isaacsim.ros2.bridge".ros_distro = "system_default"
exts."isaacsim.ros2.bridge".publish_without_verification = false
exts."isaacsim.ros2.bridge".publish_multithreading_disabled = false
exts."isaacsim.ros2.bridge".publish_with_queue_thread = true
exts."isaacsim.ros2.bridge".publish_queue_thread_sleep_us = 1000
exts."isaacsim.ros2.bridge".enable_nitros_bridge = false
```

注意這是 **`core` 的 toml 檔,宣告的卻是 `isaacsim.ros2.bridge` 命名空間下的鍵**。檔案裡那句註解「The core extension uses bridge settings, so no duplicate settings are needed here」講的就是這件事。

於是啟動參數 `--/exts/isaacsim.ros2.bridge/ros_distro=humble` 在 6.0 **仍然有效**,不需要改寫成 `.../isaacsim.ros2.core/...`。

> 這一條值得專門寫出來,是因為它的表面證據會把人帶往相反的結論。看到 bridge 的 toml 裡 `[settings]` 底下只剩一行註解、設定都在 core,很自然會推論「鍵應該也搬到 core 命名空間了,舊參數是孤兒」。**推論成立的前提是宣告位置與命名空間一致,而這個前提在這裡剛好不成立。** 判斷任何具名設定鍵是否仍然有效,要去讀當前版本的 toml 實際宣告了什麼字串,不能從檔案位置反推。

### 2.1 6.0 真正新增的兩個設定

把兩版的設定清單對齊,實質新增是兩個,都與影像發布的執行緒模型有關:

| 設定鍵(`exts."isaacsim.ros2.bridge".` 之下) | 5.1 | 6.0.1 | 預設 |
|---|---|---|---|
| `ros_distro` | ✅ | ✅ | `system_default` |
| `publish_without_verification` | ✅ | ✅ | `false` |
| `publish_multithreading_disabled` | ✅ | ✅ | `false` |
| `enable_nitros_bridge` | ✅ | ✅ | `false` |
| **`publish_with_queue_thread`** | ❌ | ✅ | `true` |
| **`publish_queue_thread_sleep_us`** | ❌ | ✅ | `1000` |

兩個新鍵只作用在 `ROS2PublishImage` 節點:改用佇列式的發布執行緒,並可調它兩次發布之間的休眠微秒數。不發影像的場景不受影響。

---

## 3. 陷阱二:extension 版本號與產品版本號是兩套

在 v6.0.1 這個 tag 底下,`isaacsim.ros2.bridge` 的 `[package] version` 是 **5.1.2**。

```
Isaac Sim 產品版本   v6.0.1     (2026-06-22 released)
└─ isaacsim.ros2.bridge   5.1.2  (CHANGELOG 最後一筆 2026-06-09)
   ├─ isaacsim.ros2.core       1.9.4
   ├─ isaacsim.ros2.nodes      1.18.13
   ├─ isaacsim.ros2.examples   1.2.4
   └─ isaacsim.ros2.ui         1.6.5
```

每個 extension 有自己的語意化版本與自己的 CHANGELOG,獨立於產品版本演進。所以:

- **「bridge 5.1.2」不代表這是 Isaac Sim 5.1 的東西。** 它是 6.0.1 隨附的 bridge。
- 反過來,**查某個行為是哪一版引入的,要查 extension 的 CHANGELOG,不是產品 release notes。** 前面那條「拆成多個 extension」記在 extension 5.0.0 (2025-11-02),而產品 5.1.0 是 2025-10-21 發布的——拆分發生在產品 5.1 之後、6.0 之前,這也是為什麼 v5.1.0 tag 裡看不到 `core`/`nodes`/`ui`。

實測用途:比對一台機器上的 bridge 是否被人改過,對照的基準是 `extension.toml` 的 `[package] version` 與官方同 tag 的值。在 6.0.1 容器上實測得到 `5.1.2`,與官方 v6.0.1 一致,即為原廠未改。

---

## 4. rclpy 的載入順序,以及「啟動前不要 source ROS」的機制

6.0 啟動 ROS 2 時的實測 log:

```
[23.313s] Attempting to load system rclpy
[23.313s] Could not import system rclpy: No module named 'rclpy'
[23.313s] Attempting to load internal rclpy for ROS Distro: humble
[23.383s] rclpy loaded
```

順序是**先試系統的、失敗才用內建的**。官方 Overview 對應的敘述:

> Users must either source their system's ROS 2 installation in the terminal prior to starting Isaac Sim, or utilize the lightweight ROS 2 libraries bundled with Isaac Sim.

兩條路徑二選一,而選擇是由「啟動當下環境裡找不找得到 `rclpy`」隱式決定的。

這條 fallback 鏈解釋了一個在實務上常被寫成硬規則、卻很少寫出理由的操作紀律:**啟動 Isaac Sim 之前不要 `source /opt/ros/<distro>/setup.bash`**。

機制是這樣:Isaac Sim 內建一套自帶的 ROS 2 函式庫,它是針對 Kit 內建的那個 Python 版本編譯的。一旦啟動前 source 了系統 ROS,`PYTHONPATH` 上就會出現系統的 `rclpy`,第一步「Attempting to load system rclpy」就會成功,於是載進來的是**為系統 Python 版本編譯的 C extension**。ABI 不匹配的後果不是啟動時報一個乾淨的錯,而是後續 bridge 行為異常——這正是「沒有錯誤訊息的失效」的典型形狀([13 篇](../13-contact-and-grasp-first-principles/README.md) §5 談的是物理層的同一件事)。

要判斷實際走了哪條路,就看 log 裡那三行:出現 `Attempting to load internal rclpy` 才是走內建。**「bridge 起來了」不足以區分兩條路徑,要看載入來源。**

---

## 5. 陷阱三:節點還在,但用法被 deprecate 了

5.1 時代常見的接法是:把要發布的 prim 直接填進 publisher 節點的 `targetPrims`,由 publisher 自己算。6.0 保留了這些節點型別(`isaacsim.ros2.bridge.ROS2PublishTransformTree`、`ROS2PublishJointState` 都還在,場景不會壞),但把「自己算」這條路標為 deprecated,實測 log:

```
[Warning] [isaacsim.ros2.nodes] OgnROS2PublishTransformTree: using targetPrims for
  internal computation is deprecated. Connect OgnIsaacComputeTransformTree to
  inputs:parentFrames/childFrames/translations/orientations instead.

[Warning] [isaacsim.ros2.nodes] [ROS2 Publish Joint State] Reading from targetPrim
  is deprecated. Connect an Isaac Read Joint State node and use its outputs instead.
```

方向是一致的:**把「取得資料」與「發布資料」拆成兩個節點**,publisher 只負責轉成 ROS 訊息。

| 資料 | 5.1 的接法 | 6.0 建議的接法 |
|---|---|---|
| TF | `ROS2PublishTransformTree.targetPrims` | `isaacsim.core.nodes.IsaacComputeTransformTree` → publisher 的 `parentFrames`/`childFrames`/`translations`/`orientations` |
| JointState | `ROS2PublishJointState.targetPrim` | `Isaac Read Joint State` → publisher 的對應輸入 |

這與 §1 的拆分是同一種設計動作,只是發生在節點粒度上。

實務上的處置有兩種。**留著不改**——只是 warning,5.1 場景搬過來照跑;代價是這條路徑未來會消失,而且每次啟動洗版。**啟動時就地改接**——用 `--exec` 的啟動腳本在場景載入後偵測版本、建立 compute 節點、接線、清空 publisher 的 `targetPrims`,場景檔本身維持不動。後者的好處是同一份 USD 能同時餵給 5.1 與 6.0 兩台機器,版本差異收斂在啟動腳本裡;代價是啟動腳本要維護一段版本判斷。

> 順帶一提,改接完成前 publisher 的 `targetPrims` 若已被清空,log 會出現一則 `[Error] ... Please specify at least one valid target prim for the ROS pose tree component`。它是**改接過程中的中間狀態**,不是故障——判讀時要對照後續有沒有接線完成的訊息,不能看到 Error 就回頭。

---

## 6. 驗收:怎麼確認 bridge 真的起來了

按可信度由低到高。前兩項單獨成立時**不足以下結論**。

| # | 檢查 | 憑據 | 能證明什麼 |
|---|---|---|---|
| 1 | 進程活著 | `ps` 看得到 kit | 只證明 kit 沒死 |
| 2 | extension startup | log 有六行 `[ext: isaacsim.ros2.*] startup` | extension 載入了,不代表資料在流 |
| 3 | rclpy 來源正確 | log 有 `Attempting to load internal rclpy` → `rclpy loaded` | 走的是內建函式庫,沒有踩到 §4 的 ABI 問題 |
| 4 | **話題有頻率** | `ros2 topic hz /tf` 量到穩定 Hz | **資料面真的通了** |

第 4 項是唯一直接證據。前三項全過而第 4 項失敗,是這套環境常見的失效形狀:extension 起來了、節點也在,但 OmniGraph 沒有 tick(headless 下的成因見 [05 篇](../05-ros2-bridge/README.md))。

量測時要注意環境:若 DDS 用了 SHM 相容性 profile,量測端必須帶同一份 profile 才看得到發布者,否則量到 0 Hz 會被誤讀成「bridge 沒起來」。

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/profile.xml
timeout 14 ros2 topic hz /tf
```

---

## 7. 一頁摘要

| 項目 | 5.1 | 6.0.1 | 升級要不要動 |
|---|---|---|---|
| extension 數量 | bridge 一個實作全部 | bridge 門面 + core/nodes/examples/ui | ❌ 依賴會自動帶起 |
| `--enable` 寫法 | `isaacsim.ros2.bridge` | 同左 | ❌ |
| 設定鍵命名空間 | `exts."isaacsim.ros2.bridge".*` | 同左(宣告位置搬到 core) | ❌ |
| 設定鍵數量 | 4 | 6(新增 queue thread 兩個) | 只在發影像時需要 |
| bridge 的 `[package] version` | 4.12.4 | 5.1.2 | 查版本用它,不是產品版號 |
| rclpy 來源 | system → internal fallback | 同左 | 啟動前不可 source 系統 ROS |
| TF / JointState 接法 | publisher 直接吃 `targetPrims` | 建議改接 compute 節點 | 只是 warning,可延後 |

**一句話**:6.0 的 ROS 2 bridge 對既有場景與啟動腳本幾乎完全向後相容,真正需要留意的不是「什麼壞了」,而是**「什麼看起來變了但其實沒變」**——設定鍵、版本號、以及被 deprecate 但仍可用的節點接法,三者都會誘導出過度反應的修改。

---

## 參考

官方(`github.com/isaac-sim/IsaacSim`,標明 tag):

- `source/extensions/isaacsim.ros2.bridge/config/extension.toml` @ v5.1.0 / v6.0.1 — 依賴與設定宣告的對照
- `source/extensions/isaacsim.ros2.core/config/extension.toml` @ v6.0.1 — 設定鍵的實際宣告位置
- `source/extensions/isaacsim.ros2.bridge/docs/CHANGELOG.md` @ v6.0.1 — 拆分記於 extension 5.0.0 (2025-11-02)
- `source/extensions/isaacsim.ros2.bridge/docs/Overview.md` @ v6.0.1 — consolidates 的官方敘述、ROS 2 環境前置條件

實測:`nvcr.io/nvidia/isaac-sim:6.0.1` 容器,2026-07-29。extension 版本、startup 順序、rclpy fallback log、兩則 deprecation warning 均為該機實際輸出。
