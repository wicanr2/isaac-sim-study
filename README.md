# Isaac Sim 實戰筆記:不碰 UI 的模擬工作流

NVIDIA Isaac Sim 的教學多半從 GUI 開始:開視窗、點選單、拖物件。但真正把它用在工程上——跑在遠端 GPU 伺服器、由程式建立與控制物理世界、接進既有系統——需要的是另一套「不碰 UI」的工作方法。本 repo 把一段倉儲物流模擬專案(堆高機 AMR、貨架、派工系統整合)累積的實戰經驗整理成教學:每篇從「要解決什麼根本問題」出發,標明哪些是官方機制、哪些是實測踩坑後的結論。

## 閱讀動線

| # | 主題 | 一句話 |
|---|---|---|
| [01](docs/01-install-and-run-modes/README.md) | 安裝與執行模式 | GUI / headless / streaming 是同一核心的三種前端;版本×驅動相容性、Python 環境隔離 |
| [02](docs/02-python-no-ui/README.md) | 不碰 UI:用 Python 操作 | `--exec` 啟動腳本、ScriptNode、UDP 遠端命令通道三層做法 |
| [03](docs/03-model-import/README.md) | 模型格式與匯入 | 一切都先轉 USD:CAD/URDF 轉換流程、依賴解析、資產授權 |
| [04](docs/04-physics-world/README.md) | 建立物理世界 | PLAYING 才有物理;虛擬世界關節建模法;teleport vs drive;互斥的控制路徑 |
| [05](docs/05-ros2-bridge/README.md) | ROS2 橋接 | 官方 bridge 機制;headless 下 OmniGraph 不 tick 的實案與 UDP 解耦架構 |
| [06](docs/06-webrtc-streaming/README.md) | WebRTC 串流 | 單 client 限制與 relay 分流架構;兩個「症狀騙人」的排錯實例 |
| [07](docs/07-minimal-example/README.md) | 最小可跑範例 | 三個由小到大的 standalone 範例:方塊落地、開官方倉庫、機器人讀位姿 |
| [08](docs/08-migration-5.1-to-6.0-oom-risk/README.md) | 5.1 → 6.0.1 遷移風險調查 | 5.1 USD 場景搬進 6.0.1 的 OOM/異常風險:官方變更點對照、記憶體機轉查證、本機兩版場景 schema 比對、遷移 SOP |
| [09](docs/09-physics-simulation-fundamentals/README.md) | 物理模擬基礎 | timestep/substep、contact/rest offset、CCD、joint drive PD 公式、PGS/TGS solver、kinematic target vs teleport、reset 語意——接進穿模/暴走/reset 三個實戰案例 |
| [10](docs/10-scene-physics-authoring/README.md) | 場景資產的物理結構 | 剛體與碰撞為什麼一定要分層、質量比是隱藏參數、物理材質綁定(`ComputeBoundMaterial` 幾乎不會回 None)、執行期補綁的三個邊界 |
| [11](docs/11-live-pose-and-accuracy/README.md) | 即時位姿與放置精度 | 四種讀位姿的方法只有一種能用、唯讀的觀測 API 反而弄壞控制鏈、把「放得準不準」變成可驗收的量測管線 |
| [12](docs/12-long-run-operations/README.md) | 長跑維運 | 重啟造成的兩份狀態分歧(表現形式是「成功」)、看門狗分層、串流靜默卡死偵測、三個殼層陷阱 |
| [13](docs/13-contact-and-grasp-first-principles/README.md) | 接觸與抓握的第一性原理 | Signorini 互補條件 + 摩擦錐推出「μ 是乘在一個可能為零的量上」;碰撞近似是有損編碼、凸包填實凹特徵是定義的後果;調參順序為何必然是幾何→質量→offset→摩擦;開環致動的結構性漂移;為什麼模擬器永遠不報錯 |
| [14](docs/14-ros2-bridge-6.0-architecture/README.md) | ROS 2 Bridge 在 6.0 的架構重組 | 一個 extension 拆成五個;設定鍵命名空間沒跟著搬家、extension 版本號與產品版號脫鉤、被 deprecate 但仍可用的 TF/JointState 接法——三個「看起來變了其實沒變」的判讀陷阱;rclpy 的 system→internal fallback 與「啟動前不要 source ROS」的機制 |
| [15](docs/15-physics-backend-5.1-to-6.0/README.md) | 5.1 → 6.0 的物理層變動 | PhysX 換代(107→110)與 Newton 後端加入是**兩件獨立的事**;怎麼確定自己跑哪個後端(log 有 newton ≠ Newton 在跑);5.x 場景官方建議留在 PhysX;`MassAPI` 授權規則改變的無聲影響;跨版本排查順序 |
| [16](docs/16-model-tuning-for-6.0/README.md) | **把 5.x 場景調到 6.0 能跑,東西不會亂飛** | 從零講起,不需先熟 Isaac Sim:一個「會被搬動的箱子」由哪些貼紙組成、為什麼 `.usd` 用 VS Code 打不開、怎麼把 crate 轉成文字改、什麼時候該改啟動腳本而不是改檔;四個「設定得進去但不生效」的結構問題(剛體/碰撞分層、質量掛錯層、材質綁定 fallback 回渲染材質、SDF 解析度不足以表達孔洞);東西亂飛的成因排序與診斷決策樹 |
| [17](docs/17-physics-parameter-tuning-6.0/README.md) | **6.0 的物理調參:入口、生效條件、完整參數表** | 三個調參入口(USD 屬性 / 啟動參數 / runtime API);四個會讓設定**無聲失效**的條件(貼錯 prim 缺對應 API、後端不吃、被 runtime patch 覆蓋、combine mode 稀釋);`physxScene`/`RigidBody`/`Collision`/`SDF`/`Material`/`Articulation`/`Joint` 七類的完整預設值表(取自 6.0.1 實機 schema);穩定性問題的調參順序;為什麼只有「設極端值看行為差異」能證明參數生效 |

**完全不熟 Isaac Sim、但手上有一個「物理跑不對」的場景要修** → 直接讀 **[16](docs/16-model-tuning-for-6.0/README.md)**,它從「一個會被搬動的箱子由什麼組成」講起,不預設前置知識。

從零開始建議按順序讀 01 → 04 → 09 → **13**,然後跳 07 動手;要自己建一個能跑物理搬運的場景,接著讀 10 → 11 → 12。13 篇是「為什麼調摩擦常常是錯的第一步」的完整推導,遇到夾不住/插不進去先讀它。已有 Isaac Sim 經驗、只想解特定問題,直接跳對應篇,每篇可獨立閱讀。API 版本以 Isaac Sim 4.5–5.1.x 為準;**升到 6.0 的人先讀 15 篇**(物理後端)與 14 篇(ROS 2),兩篇都以官方 repo tag 快照為依據並標註實測來源,6.0 的其他 breaking change 見 01 篇 §3、08 篇。08 篇性質是調查報告而非教學,結論分「官方出處」與「推測」兩級,誠實標註尚未實機重現的部分。

## Claude Code skill

兩支,分工互補。複製整個目錄到 `~/.claude/skills/` 即可使用;正文有完整推導與圖,skill 是濃縮版。

- [`skills/isaac-sim-physical-ai/SKILL.md`](skills/isaac-sim-physical-ai/SKILL.md) —— **版本無關的第一性原理**:
  接觸力學決定調參順序、碰撞近似是有損編碼、為什麼模擬器不報錯、三層真值、版本差異矩陣。
- [`skills/isaac-sim-60/SKILL.md`](skills/isaac-sim-60/SKILL.md) —— **6.0.x 特有的行為與陷阱**:
  物理後端判定(log 有 newton ≠ Newton 在跑)、`maxJointVelocity` 從 1e6 變 inf、
  參數的四個無聲失效條件、三個標 Deprecated 指向 Newton 的物理屬性、ROS 2 Bridge 的三個判讀陷阱、
  容器內沒有 usdcat 時怎麼讀寫 USD、關鍵預設值速查。

## 範例程式

- [`examples/scriptnode_udp_pose.py`](examples/scriptnode_udp_pose.py) — ScriptNode:UDP 收 pose 直接控制 prim 位姿(實戰使用過的完整版)
- [`examples/scan_physics.py`](examples/scan_physics.py) — 掃描場景所有 **authored** 物理屬性(區分「刻意設定」與「吃預設」),並列出各 prim 的 `apiSchemas`。跨版本/跨主機比對場景時的主力工具
- [`examples/usd_peek.py`](examples/usd_peek.py) — 唯讀檢視 crate 場景裡某個 prim 的物理結構(貼了哪些 API、質量、碰撞近似、bbox),並可把子樹匯出成 `.usda` 文字。搭配 [16 篇](docs/16-model-tuning-for-6.0/README.md)

## 其他

- [`CONTEXT.md`](CONTEXT.md) — 術語表
- [`PLAN.md`](PLAN.md) — 主題規劃與進度
- 本 repo 不含任何 USD 模型二進位檔:公司自製資產與 NVIDIA 官方資產都有授權限制,教學一律改用「官方管道下載」的方式描述(見 [03 篇](docs/03-model-import/README.md) §2)。
