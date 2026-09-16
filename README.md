# Isaac Sim 實戰筆記:不碰 UI 的模擬工作流

NVIDIA Isaac Sim 的教學多半從 GUI 開始:開視窗、點選單、拖物件。但真正把它用在工程上——跑在遠端 GPU 伺服器、由程式建立與控制物理世界、接進既有系統——需要的是另一套「不碰 UI」的工作方法。本 repo 把一段倉儲物流模擬專案(堆高機 AMR、貨架、派工系統整合)累積的實戰經驗整理成教學:每篇從「要解決什麼根本問題」出發,標明哪些是官方機制、哪些是實測踩坑後的結論、哪些還只是推測。

線上版:**<https://wicanr2.github.io/isaac-sim-study/>**

## 教學文件的分區

39 篇裡真正綁死版本的只有七篇,其餘的機制兩版共用,因此不複製成兩份。跨版本排查先看 **[5.1 ↔ 6.0.1 差異速查](docs/version-matrix.md)**。

| 區 | 入口 | 篇數 | 收什麼 |
|---|---|---|---|
| 共通 | [`docs/common/`](docs/common/) | 23 | 機制與方法論,兩版通用 |
| 5.1 | [`docs/5.1/`](docs/5.1/) | 2 | 綁 `isaacsim.core.*` 命名空間的程式碼 |
| 6.0.1 | [`docs/6.0.1/`](docs/6.0.1/) | 5 | 6.0.x 專屬的架構、後端與調參 |
| 車隊 | [`docs/fleet/`](docs/fleet/) | 4 | 多車、多樓層、電梯、感測器(版本無關,但自成一域) |
| HIL | [`docs/hil/`](docs/hil/) | 5 | 把 STM32 下位控制器(Renode 模擬)放進迴路,Isaac 6.0.1 當受控體;閉環實測 |


### 共通:機制與方法論

在 5.1 與 6.0.1 上都成立——講機制怎麼運作、參數為什麼那樣排序、實驗怎麼設計才算數。各篇內文標明實測所用的版本與環境。

| # | 主題 | 一句話 |
|---|---|---|
| [01](docs/common/01-install-and-run-modes/README.md) | 安裝與執行模式 | GUI / headless / streaming 是同一核心的三種前端;版本×驅動相容性、Python 環境隔離 |
| [02](docs/common/02-python-no-ui/README.md) | 不碰 UI:用 Python 操作 | `--exec` 啟動腳本、ScriptNode、UDP 遠端命令通道三層做法 |
| [03](docs/common/03-model-import/README.md) | 模型格式與匯入 | 一切都先轉 USD:CAD/URDF 轉換流程、依賴解析、資產授權 |
| [04](docs/common/04-physics-world/README.md) | 建立物理世界 | PLAYING 才有物理;虛擬世界關節建模法;teleport vs drive;互斥的控制路徑 |
| [05](docs/common/05-ros2-bridge/README.md) | ROS2 橋接 | 官方 bridge 機制;headless 下 OmniGraph 不 tick 的實案與 UDP 解耦架構 |
| [06](docs/common/06-webrtc-streaming/README.md) | WebRTC 串流 | 單 client 限制與 relay 分流架構;兩個「症狀騙人」的排錯實例 |
| [09](docs/common/09-physics-simulation-fundamentals/README.md) | 物理模擬基礎 | timestep/substep、contact/rest offset、CCD、joint drive PD 公式、PGS/TGS solver、kinematic target vs teleport、reset 語意——接進穿模/暴走/reset 三個實戰案例 |
| [10](docs/common/10-scene-physics-authoring/README.md) | 場景資產的物理結構 | 剛體與碰撞為什麼一定要分層、質量比是隱藏參數、物理材質綁定(`ComputeBoundMaterial` 幾乎不會回 None)、執行期補綁的三個邊界 |
| [12](docs/common/12-long-run-operations/README.md) | 長跑維運 | 重啟造成的兩份狀態分歧(表現形式是「成功」)、看門狗分層、串流靜默卡死偵測、三個殼層陷阱 |
| [13](docs/common/13-contact-and-grasp-first-principles/README.md) | 接觸與抓握的第一性原理 | Signorini 互補條件 + 摩擦錐推出「μ 是乘在一個可能為零的量上」;碰撞近似是有損編碼、凸包填實凹特徵是定義的後果;調參順序為何必然是幾何→質量→offset→摩擦;開環致動的結構性漂移;為什麼模擬器永遠不報錯 |
| [18](docs/common/18-finding-physical-parameters/README.md) | **建場域時,物理參數要去哪裡找** | PhysX 對未授權質量的預設是「網格體積 × **1000 kg/m³**(水)」—— 鋼構件因此輕 7.9 倍,而且**不會有任何警告**;四種來源的優先順序與各自的盲點;⚠ 規格書公布的是**載重能力不是自重**(製造商不公布 tare weight);查不到時用「幾何 × 材料密度」估,含常用密度表與合理性檢核;⚠ NVIDIA Warehouse 資產包(24 GB)實測**完全沒有物理 API**,純幾何+材質;為什麼不能用 grep 判斷 usdc 有沒有某屬性;建場域的七項檢查清單 |
| [19](docs/common/19-tuning-experiment-methodology/README.md) | **調參實驗的方法論** | 極端值正對照(旋鈕接上了嗎)、耦合參數等比例動、二元判準的統計陷阱與連續量出路(30% 對半砍要 121 輪/組)、逐輪交錯 A/B、每輪閘門(臂別/生效證據/輪數對帳)、低佔比模式的取樣經濟學、間歇性問題的宣告門檻;附開跑前檢查清單 |
| [20](docs/common/20-claude-code-driven-tuning/README.md) | **用 Claude Code 跑調參的工作法** | agent 不是常駐進程 → 兩層監看(事件層+後備層,安靜≠順利);批次腳本自己守門;逐輪紀錄/失敗清單當跨 session 記憶;模型成本分工(貴的判斷、便宜的機械活);「固定參數重試 N 次不是實驗」;長時間工具要冪等;驗證用與執行不同的機制 |
| [21](docs/common/21-cad-asset-reading-and-conversion/README.md) | **CAD 資產的判讀與轉換** | 同一個物件在資產庫裡常有三份(CAD 原始檔 / CAD 轉出的 USD / 美術資產),而哪一份能用不寫在檔名上;不開 CAD 軟體判讀 IGES(實體型別決定要不要 tessellation、Hollerith 單位陷阱、**blanked 佔八成是常態不是失敗原因**);🔴 **`stage.Traverse()` 對 instanced 資產回 0 mesh** —— CAD 轉換器預設就開 instancing,數 mesh 前先問 `GetPrototypes()`;Isaac Sim 6.0.1 內建轉換鏈的實際呼叫方式與三條死路;驗證三層與 world AABB／軸向／單位三個「看起來合理但錯誤」的陷阱 |
| [22](docs/common/22-geometry-and-measurement-discipline/README.md) | **幾何的量測紀律** | 薄件插進窄縫,決定成敗的是**姿態掃過的垂直包絡**而不是件的厚度(25 mm 的板在 −2.6° 下佔 75 mm,可插入窗口只剩 5.8 mm,比掃描步距還小);對稱撐開一個開口時**邊界移 12 mm 而中心只移 0.02 mm** —— 配對高度該跟哪一個,取決於哪個接觸在管事;prim 原點不是功能面(兩台板車原點差 6 mm、承載面差 75 mm);七種不會報錯的錯誤查法(數頂點判空腔、單位/軸向寫死、world AABB 被傾角撐大、不同截面相減、authored 姿態≠runtime 姿態、正對照挑錯同類);驗證用行為不用回讀(剛體屬性回讀成功但 PhysX 不採用,連帶讓 40 輪實驗的變因從未被施加);離線讀 USD 的環境、`--user 0:0` 與「必須在原位改」 |
| [23](docs/common/23-no-shortcuts-in-physics-sim/README.md) | **物理模擬不可以偷懶** | 為了「先讓它動起來」而繞過物理的每一個捷徑，症狀不會消失、只會搬到一個沒人會聯想到成因的地方（底盤用 3-DOF 自由關節取代輪系 → 偏航角加速度無上限 → 載貨轉彎棧板必滑，而中間隔了七層、每層都有自己的可調參數，於是排查停在第一個「調了有反應」的參數上）；**算數量級再決定要不要調參**——需要的 α_max = μ·g/r ≈ 3.3 rad/s² 而關節可達 1100，差 330 倍時把 μ 調到四倍真實值也只是杯水車薪；捷徑會生出捷徑（`restOffset` 繞過干涉 → 咬合鬆 → 用摩擦補 → 退出時同一個摩擦把貨拖走）；**捷徑的前提會過期而捷徑不會自己失效**（干涉早已消失，那個 offset 還每輪自動套用）；寫捷徑的人通常在註解裡預測過它會在哪壞——排查時先 grep 那些註解；「用參數模仿限制」vs「讓限制自己長出來」的判準與檢查清單 |
| [24](docs/common/24-nonholonomic-vehicle-control/README.md) | **把「會瞬移的底盤」換成「真的有輪子的車」** | 位置控制底盤與輪系底盤是**兩個不同的系統**,中間有一條**順序不能顛倒的前提鏈**,而每一環沒滿足時的症狀**都不會指向那一環**;① 車重到不了輪子(地面無碰撞 / `disableGravity` **文件寫 3 個實查 8 個** / 底盤沒有垂直自由度 —— 三者是同一件事的三個半,**分開做都是 no-op 也分不出誰有效**;μ 掃 250 倍位移完全平坦;判別要看垂直自由度**會不會回彈**,靜態值同時相容於兩個世界;⚠ 量到「輪胎抵抗 15 kN」是**穿透回復力**不是輪重);② 算清楚馬達扭力 vs 抓地力誰是限制(5 kN vs 27 kN,餘裕 5.4 倍);③ 轉向符號用資料驗(**量級 0.94 對、符號 73% 反** = 只有符號錯;翻號後轉向中位 65.1°→2.3°),而且**符號會隨行進方向再翻一次**(正轉 9% vs 反轉 97% 相反);④ 追蹤參考點要是**不側滑的固定軸**(車體原點離它 0.85 m → 半徑大 16.7%、偏航差 31°,看起來像軸距錯;**後處理路徑的三種做法全錯**,正解是在固定軸座標裡規劃);⑤ pure pursuit 三前提(引導點按**弧長**取、保證在前方、`Ld` ≤ 曲率半徑)+ 規劃要留轉向餘裕 + **行進方向整段鎖定**(逐步重判會讓倒車路徑被自己切成前進);⑥ 到位判定用弧長不用歐氏距離(Brockett:**症狀隨控制器變好而惡化** —— 從繞圈變成追著後退的點跑出場景);工程紀律:幾何做成**離線可測的純函式**(五個 bug 全是純幾何錯)、自測釘住踩過的坑、**指標本身會騙人**(`cross` 對航向盲、log 少乘 `sign(v)`)、事後讀狀態讀不到真相(中止會歸零) |
| [25](docs/common/25-offline-assets-deployment/README.md) | **官方資產的預先下載與離線佈署** | 場域主機不能對外時,資產只是四條對外連線裡的一條(asset root / extension registry / 容器映像 / telemetry),把資產搬到本地卻仍連不上,通常是撞到另外三條;資產包分片數隨版本不同(5.1 三片、6.0.1 五片)且要**逐片**驗 MD5;⚠ **檔名帶修訂號而目錄名不帶**——6.0.1 的包要落在 `Isaac/6.0` 底下;asset root 四層優先序裡**環境變數 `ISAACSIM_ASSET_ROOT` 蓋過命令列**,「參數寫了卻還在往外連」先查它;⚠ 離線拿不到 extension 會表現成 `ModuleNotFoundError` 而不是網路錯誤;驗收唯一有效的方式是**把出口斷掉再跑一次**,而且看行為不看設定回讀 |
| [26](docs/common/26-forklift-physics-and-articulation/README.md) | **從規格表到會動的叉車:物理參數與 articulation 的非 GUI 建法** | 把 VDI 2198 的欄位編號**逐欄對到 USD 屬性**,並標明貼在 prim 樹的哪一層(以 Linde R16 實抽值走完整流程);三個要換算而不是直接填的欄位——軸荷推質心(`(1280/3470)×1380 ≈ 509 mm`,⚠ 填之前先加總比對自重,型錄標籤可能互換)、牽引力**乘輪半徑才是力矩**、車速除輪半徑才是角速度;離線寫檔 vs runtime patch 的取捨(後者**不是所有屬性都吃**,踩過 40 輪實驗變因從未被施加);articulation 的兩條路(在匯入的 USD 上補 / 從零建)與四個不報錯的坑:`body0` 是父 `body1` 是子(**PhysX 不在乎、Newton 硬性要求**)、joint 都要在 `ArticulationRootAPI` 底下、動態剛體不得零質量、⚠ **油壓缸照搬會變成閉合運動鏈**(Newton 明確不支援);`solverVelocityIterationCount` 預設 1 而官方建議 16;驗收的三個叉車專屬行為檢查(飽和行為要**故意去撞上限**才驗得到) |
| [27](docs/common/27-failure-mode-taxonomy/README.md) | **失效模式分類學** | 把「東西壞了」變成可歸因的類別:一個 bit 換不到歸因(32 輪裡 12 輪根本不是物理失敗);判準要標門檻出處;**待判桶是分類器的自我檢驗**;只看失敗當下的窗口不掃全 log;x 與 z 分開判會把兩種病混成一類;⚠ **驗法要有鑑別力**——問「假說為假的世界裡這個觀測會不會也長成這樣」;分好類才算得出「消除滑移只解決 7/13」 |
| [28](docs/common/28-error-accumulation-and-harmful-compensation/README.md) | **誤差累積與「補償反而有害」** | 誤差在載運途中被注入、車一停被靜摩擦凍結(放貨九步只動 0.3°);`容差 ÷ 單趟誤差` 算得出撐幾趟;⚠ **補償被執行了(增益 0.86)卻讓事情更糟**——0° 成功、−4°/−8° 全部留在齒上;歪 4° 換來 71 mm 額外包絡而餘裕只有 15 mm;相對角與世界角是兩件事;正解是**截斷傳遞**而不是修正誤差;推翻斷言前先算新條件往哪個方向推 |
| [29](docs/common/29-long-run-error-budget-and-clock-drift/README.md) | **長跑才會浮現的兩件事** | PhysX 錯誤上限預設 1000,**填滿它的是無害的 API 警告**(951 筆佔 86%);⚠ 物理死了而容器/log/探針/畫面全部正常,唯一有鑑別力的是模擬時鐘有沒有前進;修掉一類不夠要看總量;控制端走牆鐘而物理 RTF 0.21 → 軌跡被 3.5 倍速播放,四個症狀同一個原因;降速沒用因為根因是比例;**RTF 由機器負載主導(探針只佔 9%)**,修好之前所有 A/B 都被汙染 |
| [30](docs/common/30-acceptance-probes-and-preregistration/README.md) | **驗收探針與實驗預先登記** | 預期落點由機構原理推導不是靠樣本統計;兩條互相獨立的證據鏈;⚠ **只跑正對照不算驗過**——永遠回 ✅ 的閘門與正確的閘門輸出相同;表頭有 64 欄而 13632 格全是 nan;樣本數決定能問什麼(6 輪全過的整輪下界只有 61%,要 ≳90% 得跑 29 輪);組態連**不改變的觀測條件**都寫死;**修壞掉的尺 ≠ 換一把尺**;停止規則要跟統計目的對齊;附三層紀錄範本 |


### Isaac Sim 5.1

只在 5.x 命名空間下成立的內容,主要是程式碼:6.0 起 `isaacsim.core.api` / `isaacsim.core.prims` / `isaacsim.core.utils` 整組移到 `isaacsim.core.experimental.*`。

| # | 主題 | 一句話 |
|---|---|---|
| [07](docs/5.1/07-minimal-example/README.md) | 最小可跑範例 | 三個由小到大的 standalone 範例:方塊落地、開官方倉庫、機器人讀位姿 |
| [11](docs/5.1/11-live-pose-and-accuracy/README.md) | 即時位姿與放置精度 | 四種讀位姿的方法只有一種能用、唯讀的觀測 API 反而弄壞控制鏈、把「放得準不準」變成可驗收的量測管線 |


### Isaac Sim 6.0.1

extension 架構重組、PhysX 換代(107→110)與 Newton 後端、從 5.1 搬場景的風險、6.0 的物理調參。

| # | 主題 | 一句話 |
|---|---|---|
| [08](docs/6.0.1/08-migration-5.1-to-6.0-oom-risk/README.md) | 5.1 → 6.0.1 遷移風險調查 | 5.1 USD 場景搬進 6.0.1 的 OOM/異常風險:官方變更點對照、記憶體機轉查證、本機兩版場景 schema 比對、遷移 SOP |
| [14](docs/6.0.1/14-ros2-bridge-6.0-architecture/README.md) | ROS 2 Bridge 在 6.0 的架構重組 | 一個 extension 拆成五個;設定鍵命名空間沒跟著搬家、extension 版本號與產品版號脫鉤、被 deprecate 但仍可用的 TF/JointState 接法——三個「看起來變了其實沒變」的判讀陷阱;rclpy 的 system→internal fallback 與「啟動前不要 source ROS」的機制 |
| [15](docs/6.0.1/15-physics-backend-5.1-to-6.0/README.md) | 5.1 → 6.0 的物理層變動 | PhysX 換代(107→110)與 Newton 後端加入是**兩件獨立的事**;怎麼確定自己跑哪個後端(log 有 newton ≠ Newton 在跑);5.x 場景官方建議留在 PhysX;`MassAPI` 授權規則改變的無聲影響;跨版本排查順序 |
| [16](docs/6.0.1/16-model-tuning-for-6.0/README.md) | **把 5.x 場景調到 6.0 能跑,東西不會亂飛** | 從零講起,不需先熟 Isaac Sim:一個「會被搬動的箱子」由哪些貼紙組成、為什麼 `.usd` 用 VS Code 打不開、怎麼把 crate 轉成文字改、什麼時候該改啟動腳本而不是改檔;四個「設定得進去但不生效」的結構問題(剛體/碰撞分層、質量掛錯層、材質綁定 fallback 回渲染材質、SDF 解析度不足以表達孔洞);東西亂飛的成因排序與診斷決策樹 |
| [17](docs/6.0.1/17-physics-parameter-tuning-6.0/README.md) | **6.0 的物理調參:入口、生效條件、完整參數表** | 三個調參入口(USD 屬性 / 啟動參數 / runtime API);五個會讓設定**無聲失效**的條件(貼錯 prim 缺對應 API、後端不吃、被 runtime patch 覆蓋、combine mode 稀釋、runtime 寫入不一定被採用);`physxScene`/`RigidBody`/`Collision`/`SDF`/`Material`/`Articulation`/`Joint` 七類的完整預設值表(取自 6.0.1 實機 schema);穩定性問題的調參順序;為什麼只有「設極端值看行為差異」能證明參數生效 |


### 車隊與多樓層

多台車 + 多樓層自成一個問題域。內容與版本無關,但單車的物理調參解決不了「五台車在三層樓之間互相讓路」。實測於 Isaac Sim 5.1.2,⚠ 未在 6.0.x 複驗。

| # | 主題 | 一句話 |
|---|---|---|
| [31](docs/fleet/31-omnigraph-and-ros2-bridge-truth/README.md) | **OmniGraph 與 ROS 2 橋接的真相** | 五個「成功了但沒有在跑」:`SimulationApp` 之前不能 import `pxr`(**間接 import 也算**,症狀是啟動九秒後的 boost::python 轉換器錯誤,看起來完全像 Isaac 內部壞掉);extension 啟用**回 `True` 而它 17 ms 後自己收掉**,生效證明要看節點型別在不在;⚠ **列舉查詢一定要配正對照**——查法寫錯會讓 11 個型別全印「沒有」,包括一定存在的那個;節點型別是真的 USD 屬性 `node:type`,寫在 `customData` 等於沒寫(45 個節點對 45 條警告);圖的連線就是普通 USD attribute connection,所以**整張圖可以離線寫全**,不必在 Isaac 裡跑 `og.Controller`;⚠ **`world.step(render=False)` 不會 tick action graph**——物理時間正確前進 2.000 s,而 `OnTick` 只被算 1 次、publisher 根本沒建出來;三個「檔案正常、沒報錯、畫面全黑」的成因 |
| [32](docs/fleet/32-differential-drive-vehicle-model/README.md) | **把車做成真的會動的車** | 五個成因完全不同的「車不動」,而 **USD 檢查、bbox、overlap 查詢全部是綠的**:`enabledSelfCollisions` 預設 True、增益 1e4 配 500 N·m(1 kg 輪子等於 12 萬 rad/s²)、`maxJointVelocity` 的單位是**度/秒**(輪速卡在 0.52 = 30°/s)、`UsdGeom.Cylinder` 當碰撞體、凸包軸向映射寫反;🔴 **一個算錯的滑移指標(41%)長出一整套合理但錯誤的成因**——真值 2~3%,而順著錯指標做的每一步都「有一點效果」;真因是慣量少了馬達與減速機的反射慣量(追隨誤差 22.8% → 0.7%);腳輪半徑只有驅動輪一半,車在**平坦空曠**走廊上 1.3 m 就翻(**靜態輪載三款車完全相同**——靜態正常與動態正常是兩件事);宣告 1.0 m/s 的車撐不住 1.0,**限速到 0.6 之後絕對時間反而更快**(22.5 → 16.1 s);車開不完路網的五個成因**沒有一個是路線規劃不好**;⚠ **陳舊的輸入不會報錯,它只會給你一組合理的數字** |
| [33](docs/fleet/33-elevator-and-multi-floor/README.md) | **電梯與多樓層** | articulation 的**根附著不能帶自由度**,轎廂因此升不起來,而 USD 檢查、joint、`DriveAPI` 全都在——證據要問自由度清單;承重接觸的三個數字(下沉 4.22 對理論 4.01、追隨誤差 27.2 對 28.3、橫向滑動 0.0)與三個量測坑(**靜態下沉不能在限位上量**、追隨誤差不能拿去比下沉需求、載重要對上設計條件);跳動 22 mm 是**指令形狀**不是接觸(只在速度不連續的三個瞬間出現,等速段為 0);**全零的陰性結果一定要配正對照**,否則「很穩」與「量尺沒在量」輸出一模一樣;門檻上 200 mm 的洞**只在開進去時發作**——方向不對稱本身就是幾何成因的指紋;⚠ **負對照證明「是這次改動之後才有的」,不證明「是這次改動造成的」**;擋住轎廂的是它自己的門片(差半個門厚),而**只量相對量的量尺量不到參考系有沒有動**;車在移動中的轎廂裡繼續開——兩道防線都沒寫錯,是聯集不完整 |
| [34](docs/fleet/34-lidar-and-sensor-plausible-but-wrong/README.md) | **感測器的假數字** | 三次錯誤,**三次的輸出都是一組合理的數字**:感測器埋在自己的碰撞盒裡 → 240 束全回量程下限,**與「車貼著牆」在資料上完全一樣**;視角裝不下車體(上限 `180° − atan((車寬/2)/淨空)`,實測約 93.5°,±120° 根本放不下)——把幾何約束寫成程式裡的守衛;🔴 **正對照自己的幾何錯了**(從車體原點量而不是從感測器量,差 0.4 m;而牆與箱子落在同一判定帶),正解是閉環對照 2.095 → 1.048 → 2.095;三個獨立的「看不見」成因(形狀建太晚被 Hydra 收下、線寬是次像素、`displayColor` 只是提示);ROS 2 那側四個會讓數字說謊的量法(topic 在 ≠ 有人發、一次 spin 只處理一個 callback、不要換算成 Hz、沒訂閱者就不發);素材亮度**用量的不看檔名** |

### HIL:把下位控制器放進迴路

讓真的底盤韌體去驅動 Isaac 裡的車。STM32F4 跑在 Renode 1.16.1 裡,Rust 橋接把匯流排訊號(UART、GPIO、Timer PWM、CAN)接到受控體;假受控體閉環本機實測,Isaac 6.0.1 受控體在場域 GPU 主機實測(經 `ssh -L`,ALL PASS,odom 對真值 3.4 mm / 0.028 rad)。程式碼在 [`examples/hil-stm32/`](examples/hil-stm32/)。

| # | 主題 | 一句話 |
|---|---|---|
| [35](docs/hil/35-hil-what-and-why/README.md) | **HIL 是什麼,為什麼硬體要放在下位** | NVIDIA 課程的 HIL 是 Jetson 跑感知,這裡是 MCU 跑底盤——協定、CRC、安全閘門、閉環控制、動態限制、時序五類邏輯純運動學版本一條都沒有;每個行程只認一種語言,橋接是唯一懂兩邊的;**三個時鐘域**(牆鐘、Renode 虛擬時間、Isaac 模擬時間),`lockstep` 實測 1200 步時間分毫不差、兩次 CSV 逐 byte 相同;三條規則:**韌體沒有「模擬模式」、橋接不做安全、每輪要有生效證明**;純軟體 HIL 的邊界——時序只有實板算數 |
| [36](docs/hil/36-stm32-firmware-on-renode/README.md) | **STM32F4 韌體在 Renode 上開機** | 無 HAL、無 libc 的 5 KB 韌體,每個寫進週邊的位元都能回答「模擬器有沒有實作它」;平台描述 vendor 一份、拿掉會上網的 `ApplySVD`、版本鎖死;**「型別存在 ≠ 夠用」逐項盤點**:CCR 讀得回、PWM 通道是真的 GPIO 線、**timer 週期是 ARR 不是 ARR+1**(兩組量測)、CAN 交握有回應但 `FMR` 寫 `1` 會把 bank 0 劃給 CAN2 而訊框靜默丟掉;printer 與 SRAM 兩條觀測管道(機器沒跑時 SRAM 全零);**WFI 讓 1 s 虛擬時間從 13.4 s 降到 1.83 s**,配套是收訊改中斷——Renode 的 UART 有佇列,輪詢版「看起來也對」 |
| [37](docs/hil/37-bus-signal-bridging/README.md) | **匯流排訊號串接** | External Control 協定逐 byte(握手 14 B、六種回應碼、GPIO 事件 16 B 含 7 bytes 填充);`GetState` 讀輸出腳、`SetState` 寫輸入腳;IronPython hook 一條 TCP 收發 CAN 與 UART,**每筆注入回 ack**——沒有 ack 就沒有決定性;`FrameSent` 當下快照 CCR 給橋接做同時刻比對;**CAN 送出模擬器的三條路各碰到哪一層**(只有 IronPython 那條不碰核心;`vcan` 是純軟體介面但是核心模組);lockstep 六步、一步延遲、每步 29.6 ms 的成本拆解;⚠ 用 `echo >/dev/tcp/…` 探埠會把一個換行送進握手 |
| [38](docs/hil/38-acceptance-and-failure-modes/README.md) | **驗收與失敗形態** | 十項判準在開跑前寫死,生效證明六行;負對照 `--negative bad-crc` 紅在 C2 而 **C6 證明韌體擋了全部 300 個壞框包**,`no-ramp` 紅在 C9 並卡住容差;**五項安全 I/O 各一個故障注入(C10)與一個 `*-off` 負對照**——韌體死掉沒看門狗時馬達用最後的 duty 轉了 4 s;**斜坡在韌體、馬達層在受控體**,步階超調 Isaac 60% → 3%;決定性:兩次 1201 行逐 byte 相同,但 Rust 與 Python 兩個「同一個模型」末端一致、途中 811 欄位差 ±1 tick——**決定性是每個實作各自成立**;**步邊界取樣的盲點**:2/305 筆 CAN duty 在任何邊界都沒出現過,一個視窗跑了兩次控制步,解法是事件時刻快照;十八種失敗形態,每一種的第一眼症狀都指向別的地方;換成 Isaac 6.0.1 的七件事各量到什麼——PhysX 介面沒有 `update`、joint state 不寫回、**地面 xformOpOrder 反了讓車在 5 ms 內以 2.9 m/s 飛起來**(所有東西靜止後都停在 +45 mm 就是線索)、yaw 差一個正負號、滑移 3.1%;分六階段,這一區做到第五 |
| [39](docs/hil/39-freertos-firmware-in-the-loop/README.md) | **同一台車換 FreeRTOS** | Cortex-M4F 是官方 port,Renode 三個核心例外都有;vendor V11.3.1 最小子集、自給 4 個 libc 符號、hard-float;三個 task 一條 ISR,協定、暫存器、控制律、`g_dbg` 版面與裸機版逐字相同,**橋接一行不改**;`xTaskDelayUntil` 的回傳值變成「錯過週期」計數(實測 0);末端位姿與裸機版相同、途中 CCR 差在相位;**RTOS 才踩到的 Renode 缺口:port 先寫 CVR 再寫 LOAD,SysTick 第一週期 2^24 cycle = 233 ms 沒有任何錯誤**,修在 NVIC(ENABLE 0→1 載入 RELOAD)不改 port.c;換一支韌體,週邊要重新盤點 |

## 兩條常見的入場路徑

**完全不熟 Isaac Sim、但手上有一個「物理跑不對」的場景要修** → 直接讀 **[16](docs/6.0.1/16-model-tuning-for-6.0/README.md)**,它從「一個會被搬動的箱子由什麼組成」講起,不預設前置知識。

**從零開始** → 01 → 04 → 09 → **13**,然後照自己的版本跳 [5.1](docs/5.1/) 或 [6.0.1](docs/6.0.1/) 區動手;要自己建一個能跑物理搬運的場景,接著讀 10 → 11 → 12。13 篇是「為什麼調摩擦常常是錯的第一步」的完整推導,遇到夾不住/插不進去先讀它。已有經驗、只想解特定問題,直接跳對應篇,每篇可獨立閱讀。

升到 6.0 的人先讀 [15](docs/6.0.1/15-physics-backend-5.1-to-6.0/README.md)(物理後端)與 [14](docs/6.0.1/14-ros2-bridge-6.0-architecture/README.md)(ROS 2),兩篇都以官方 repo tag 快照為依據並標註實測來源;其他 breaking change 見 [01 §3](docs/common/01-install-and-run-modes/README.md) 與 [08](docs/6.0.1/08-migration-5.1-to-6.0-oom-risk/README.md)。08 篇性質是調查報告而非教學,結論分「官方出處」與「推測」兩級,誠實標註尚未實機重現的部分。

## 工具

- [物理模擬健檢](https://wicanr2.github.io/isaac-sim-study/tools/physics-checkup/)(原始檔 [`docs/tools/physics-checkup/`](docs/tools/physics-checkup/))—— 把 [23](docs/common/23-no-shortcuts-in-physics-sim/README.md)、[22](docs/common/22-geometry-and-measurement-discipline/README.md)、[18](docs/common/18-finding-physical-parameters/README.md) 三篇的判準算成可以當場填數字的檢查:所需摩擦、姿態包絡、質量合理性。

## Claude Code skill

兩支,分工互補。複製整個目錄到 `~/.claude/skills/` 即可使用;正文有完整推導與圖,skill 是濃縮版。

- [`skills/isaac-sim-physical-ai/SKILL.md`](skills/isaac-sim-physical-ai/SKILL.md) —— **版本無關的第一性原理**:
  接觸力學決定調參順序、碰撞近似是有損編碼、為什麼模擬器不報錯、三層真值、版本差異矩陣。
- [`skills/isaac-sim-60/SKILL.md`](skills/isaac-sim-60/SKILL.md) —— **6.0.x 特有的行為與陷阱**:
  物理後端判定(log 有 newton ≠ Newton 在跑)、`maxJointVelocity` 從 1e6 變 inf、
  參數的五個無聲失效條件(含 runtime 授權剛體屬性不被採用)、關節名≠世界軸、
  drive 力與增益要等比例、睡不著的接觸對、三個標 Deprecated 指向 Newton 的物理屬性、
  ROS 2 Bridge 的三個判讀陷阱、容器內沒有 usdcat 時怎麼讀寫 USD、關鍵預設值速查。

## 範例程式

- [`examples/scriptnode_udp_pose.py`](examples/scriptnode_udp_pose.py) — ScriptNode:UDP 收 pose 直接控制 prim 位姿(實戰使用過的完整版)
- [`examples/scan_physics.py`](examples/scan_physics.py) — 掃描場景所有 **authored** 物理屬性(區分「刻意設定」與「吃預設」),並列出各 prim 的 `apiSchemas`。跨版本/跨主機比對場景時的主力工具
- [`examples/audit_asset_physics.py`](examples/audit_asset_physics.py) — 稽核一份 USD 有沒有**授權**質量/密度/碰撞近似;會一併走訪 instance prototype(否則 `Traverse()` 對 CAD 轉出的資產回 0 mesh)
- [`examples/templates/`](examples/templates/) — 實驗紀錄範本:輪次表、分期敘事、場景檔 manifest、事前登記、失敗總表(用法見 [30 篇](docs/common/30-acceptance-probes-and-preregistration/README.md) §8)
- [`examples/hil-stm32/`](examples/hil-stm32/) — HIL 閉環全部程式碼:STM32F4 最小韌體(C,無 HAL)、Renode 平台與 IronPython hook、FreeRTOS V11.3.1 版韌體、Rust 橋接(std-only)、假受控體(Rust / Python UDP)、Isaac 6.0.1 受控體(場域 GPU 主機實測)。`./run_loop.sh` 一條指令跑,`--negative bad-crc` 是負對照,`PLANT=remote` 經 `ssh -L` 接遠端受控體
- [`examples/usd_peek.py`](examples/usd_peek.py) — 唯讀檢視 crate 場景裡某個 prim 的物理結構(貼了哪些 API、質量、碰撞近似、bbox),並可把子樹匯出成 `.usda` 文字。搭配 [16 篇](docs/6.0.1/16-model-tuning-for-6.0/README.md)

## 其他

- [`CONTEXT.md`](CONTEXT.md) — 術語表
- [`PLAN.md`](PLAN.md) — 主題規劃與進度
- [`build_site.py`](build_site.py) — 把 `docs/` 的 markdown 轉成 GitHub Pages 的靜態 HTML(docker uv 環境,見檔頭說明)
- 本 repo 不含任何 USD 模型二進位檔:公司自製資產與 NVIDIA 官方資產都有授權限制,教學一律改用「官方管道下載」的方式描述(見 [03 篇](docs/common/03-model-import/README.md) §2)。
