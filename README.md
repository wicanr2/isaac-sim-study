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

從零開始建議按順序讀 01 → 04,然後跳 07 動手;已有 Isaac Sim 經驗、只想解特定問題,直接跳對應篇,每篇可獨立閱讀。API 版本以 Isaac Sim 4.5–5.1.x 為準(6.0 的 breaking change 見 01 篇 §3、08 篇)。08 篇性質是調查報告而非教學,結論分「官方出處」與「推測」兩級,誠實標註尚未實機重現的部分。

## 範例程式

- [`examples/scriptnode_udp_pose.py`](examples/scriptnode_udp_pose.py) — ScriptNode:UDP 收 pose 直接控制 prim 位姿(實戰使用過的完整版)

## 其他

- [`CONTEXT.md`](CONTEXT.md) — 術語表
- [`PLAN.md`](PLAN.md) — 主題規劃與進度
- 本 repo 不含任何 USD 模型二進位檔:公司自製資產與 NVIDIA 官方資產都有授權限制,教學一律改用「官方管道下載」的方式描述(見 [03 篇](docs/03-model-import/README.md) §2)。
