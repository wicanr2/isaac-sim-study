# 共通:機制與方法論

這一區的 17 篇在 5.1 與 6.0.1 上都成立——講的是機制怎麼運作、參數為什麼那樣排序、實驗怎麼設計才算數,而不是某一版的 API 長相。各篇內文標明實測所用的版本與環境;凡是只在單一版本成立的操作細節,已經移到 [5.1](../5.1/) 或 [6.0.1](../6.0.1/) 區。

## 起步:把模擬跑起來

| # | 主題 | 一句話 |
|---|---|---|
| [01](01-install-and-run-modes/README.md) | 安裝與執行模式 | GUI / headless / streaming 是同一核心的三種前端;版本×驅動相容性、Python 環境隔離 |
| [02](02-python-no-ui/README.md) | 不碰 UI:用 Python 操作 | `--exec` 啟動腳本、ScriptNode、UDP 遠端命令通道三層做法 |
| [03](03-model-import/README.md) | 模型格式與匯入 | 一切都先轉 USD:CAD/URDF 轉換流程、依賴解析、資產授權 |
| [04](04-physics-world/README.md) | 建立物理世界 | PLAYING 才有物理;虛擬世界關節建模法;teleport vs drive;互斥的控制路徑 |
| [25](25-offline-assets-deployment/README.md) | 官方資產的預先下載與離線佈署 | 場域主機不能對外時怎麼辦:四條對外連線各自的替代方案;資產包分片與 MD5;檔名帶修訂號而目錄名不帶;`ISAACSIM_ASSET_ROOT` 蓋過命令列;離線失敗會表現成 `ModuleNotFoundError` |

## 接進外部系統

| # | 主題 | 一句話 |
|---|---|---|
| [05](05-ros2-bridge/README.md) | ROS2 橋接 | 官方 bridge 機制;headless 下 OmniGraph 不 tick 的實案與 UDP 解耦架構 |
| [06](06-webrtc-streaming/README.md) | WebRTC 串流 | 單 client 限制與 relay 分流架構;兩個「症狀騙人」的排錯實例 |
| [12](12-long-run-operations/README.md) | 長跑維運 | 重啟造成的兩份狀態分歧(表現形式是「成功」)、看門狗分層、串流靜默卡死偵測、三個殼層陷阱 |

## 物理:引擎怎麼算,場景怎麼搭

| # | 主題 | 一句話 |
|---|---|---|
| [09](09-physics-simulation-fundamentals/README.md) | 物理模擬基礎 | timestep/substep、contact/rest offset、CCD、joint drive PD 公式、PGS/TGS solver、kinematic target vs teleport、reset 語意——接進穿模/暴走/reset 三個實戰案例 |
| [10](10-scene-physics-authoring/README.md) | 場景資產的物理結構 | 剛體與碰撞為什麼一定要分層、質量比是隱藏參數、物理材質綁定(`ComputeBoundMaterial` 幾乎不會回 None)、執行期補綁的三個邊界 |
| [13](13-contact-and-grasp-first-principles/README.md) | 接觸與抓握的第一性原理 | Signorini 互補條件 + 摩擦錐推出「μ 是乘在一個可能為零的量上」;碰撞近似是有損編碼、凸包填實凹特徵是定義的後果;調參順序為何必然是幾何→質量→offset→摩擦;為什麼模擬器永遠不報錯 |

夾不住、叉不起來、東西在載具上滑掉——這類問題最常見的處理順序是先調摩擦係數,而 13 篇要說明的是這個順序在相當多情況下從一開始就錯了,錯得可以從接觸力學的定義直接推出來,不必靠試誤。

## 資產與幾何:數字都對,件還是插不進去

| # | 主題 | 一句話 |
|---|---|---|
| [18](18-finding-physical-parameters/README.md) | 建場域時,物理參數要去哪裡找 | PhysX 對未授權質量的預設是「網格體積 × 1000 kg/m³(水)」——鋼構件因此輕 7.9 倍且不會有任何警告;四種來源的優先順序與盲點;規格書公布的是載重能力不是自重;查不到時怎麼估 |
| [21](21-cad-asset-reading-and-conversion/README.md) | CAD 資產的判讀與轉換 | 同一物件在資產庫常有三份而哪份能用不寫在檔名上;不開 CAD 軟體判讀 IGES;`stage.Traverse()` 對 instanced 資產回 0 mesh;驗證三層與三個「看起來合理但錯誤」的陷阱 |
| [22](22-geometry-and-measurement-discipline/README.md) | 幾何的量測紀律 | 決定成敗的是姿態掃過的垂直包絡而不是件的厚度;prim 原點不是功能面;七種不會報錯的錯誤查法;驗證用行為不用回讀 |

## 實驗、工程紀律與控制

| # | 主題 | 一句話 |
|---|---|---|
| [19](19-tuning-experiment-methodology/README.md) | 調參實驗的方法論 | 極端值正對照(旋鈕接上了嗎)、耦合參數等比例動、二元判準的統計陷阱與連續量出路、逐輪交錯 A/B、每輪閘門、間歇性問題的宣告門檻 |
| [20](20-claude-code-driven-tuning/README.md) | 用 Claude Code 跑調參的工作法 | agent 不是常駐進程 → 兩層監看(安靜≠順利);批次腳本自己守門;模型成本分工;「固定參數重試 N 次不是實驗」;長時間工具要冪等 |
| [23](23-no-shortcuts-in-physics-sim/README.md) | 物理模擬不可以偷懶 | 繞過物理的每個捷徑,症狀不會消失、只會搬到一個沒人會聯想到成因的地方;算數量級再決定要不要調參;捷徑的前提會過期而捷徑不會自己失效 |
| [24](24-nonholonomic-vehicle-control/README.md) | 把「會瞬移的底盤」換成「真的有輪子的車」 | 位置控制底盤與輪系底盤是兩個系統,中間有一條順序不能顛倒的前提鏈,而每一環沒滿足時的症狀都不指向那一環;車重到不了輪子的三個成因、轉向符號用資料驗、追蹤參考點要是不側滑的固定軸、pure pursuit 三前提 |

## 兩條常見的入場路徑

**完全不熟 Isaac Sim,但手上有一個物理跑不對的場景要修** → 跳到 [6.0.1 區的 16 篇](../6.0.1/16-model-tuning-for-6.0/README.md),它從「一個會被搬動的箱子由哪些東西組成」講起,不預設前置知識。

**從零開始學** → 01 → 04 → 09 → 13,然後照自己的版本跳 [5.1](../5.1/) 或 [6.0.1](../6.0.1/) 區動手;要建能跑物理搬運的場景,接著讀 10 → 12。每篇都可獨立閱讀,已有經驗的直接跳對應篇。
