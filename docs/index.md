# Isaac Sim 實戰筆記:不碰 UI 的模擬工作流

NVIDIA Isaac Sim 的教學多半從 GUI 開始:開視窗、點選單、拖物件。但真正把它用在工程上——跑在遠端 GPU 伺服器、由程式建立與控制物理世界、接進既有系統——需要的是另一套「不碰 UI」的工作方法。

這裡整理的是一段倉儲物流模擬專案(堆高機 AMR、貨架、派工系統整合)累積下來的東西:每篇從「要解決什麼根本問題」出發,標明哪些是官方機制、哪些是實測踩坑後的結論、哪些還只是推測。

## 三個入口

**[共通:機制與方法論](common/)** · 19 篇
引擎怎麼算一步、碰撞近似損掉什麼、質量該掛在哪一層、實驗怎麼設計才算數。這些在 5.1 與 6.0.1 上是同一套,因此不按版本分。

**[Isaac Sim 5.1](5.1/)** · 2 篇
只在 5.x 命名空間下成立的內容,主要是程式碼——6.0 起 `isaacsim.core.*` 整組搬到 `isaacsim.core.experimental.*`。

**[Isaac Sim 6.0.1](6.0.1/)** · 5 篇
extension 架構重組、PhysX 換代與 Newton 後端、從 5.1 搬場景的風險、6.0 的物理調參。

**[5.1 ↔ 6.0.1 差異速查](version-matrix.md)**
跨版本排查時最花時間的不是「哪裡不一樣」,而是「這個症狀該不該歸給版本」。每一列都標出處篇章。

## 怎麼開始

| 你的處境 | 從這裡進 |
|---|---|
| 完全不熟 Isaac Sim,手上有一個物理跑不對的場景 | [16 把 5.x 場景調到 6.0 能跑](6.0.1/16-model-tuning-for-6.0/README.md) —— 從「一個會被搬動的箱子由哪些東西組成」講起 |
| 從零開始學,想建一個能跑物理的場景 | [01 安裝與執行模式](common/01-install-and-run-modes/README.md) → 04 → 09 → 13 |
| 正要從 5.1 升到 6.0 | [15 物理層變動](6.0.1/15-physics-backend-5.1-to-6.0/README.md) → [14 ROS 2 架構重組](6.0.1/14-ros2-bridge-6.0-architecture/README.md) |
| 夾不住、叉不起來、轉彎會滑 | [13 接觸與抓握的第一性原理](common/13-contact-and-grasp-first-principles/README.md) |
| 尺寸都對但件插不進去 | [22 幾何的量測紀律](common/22-geometry-and-measurement-discipline/README.md) |
| 要跑幾十輪調參 | [19 調參實驗的方法論](common/19-tuning-experiment-methodology/README.md) |
| 手上有一台叉車/機器人,要把物理與關節建起來 | [26 從規格表到會動的叉車](common/26-forklift-physics-and-articulation/README.md) |
| 場域主機不能對外,資產抓不到 | [25 官方資產的預先下載與離線佈署](common/25-offline-assets-deployment/README.md) |

## 工具

- [物理模擬健檢](tools/physics-checkup/) —— 把 [23](common/23-no-shortcuts-in-physics-sim/README.md)、[22](common/22-geometry-and-measurement-discipline/README.md)、[18](common/18-finding-physical-parameters/README.md) 三篇的判準算成幾個可以當場填數字的檢查:所需摩擦、姿態包絡、質量合理性。

## Claude Code skill

兩支,分工互補。複製整個目錄到 `~/.claude/skills/` 即可使用;正文有完整推導與圖,skill 是濃縮版。

- [`skills/isaac-sim-physical-ai/`](../skills/isaac-sim-physical-ai/SKILL.md) —— 版本無關的第一性原理:接觸力學決定調參順序、碰撞近似是有損編碼、為什麼模擬器不報錯、三層真值、版本差異矩陣。
- [`skills/isaac-sim-60/`](../skills/isaac-sim-60/SKILL.md) —— 6.0.x 特有的行為與陷阱:物理後端判定、`maxJointVelocity` 從 1e6 變 inf、參數的五個無聲失效條件、關鍵預設值速查。

## 範例程式

- [`examples/scriptnode_udp_pose.py`](../examples/scriptnode_udp_pose.py) —— ScriptNode:UDP 收 pose 直接控制 prim 位姿
- [`examples/scan_physics.py`](../examples/scan_physics.py) —— 掃描場景所有 **authored** 物理屬性(區分「刻意設定」與「吃預設」),跨版本/跨主機比對場景時的主力工具
- [`examples/usd_peek.py`](../examples/usd_peek.py) —— 唯讀檢視 crate 場景裡某個 prim 的物理結構,並可把子樹匯出成 `.usda` 文字

## 其他

- [`CONTEXT.md`](../CONTEXT.md) —— 術語表
- [`PLAN.md`](../PLAN.md) —— 主題規劃與進度
- 本 repo 不含任何 USD 模型二進位檔:公司自製資產與 NVIDIA 官方資產都有授權限制,教學一律改用「官方管道下載」的方式描述(見 [03 篇](common/03-model-import/README.md) §2)。
