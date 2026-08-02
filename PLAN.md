# PLAN — Isaac Sim 教學知識庫

> 目標:把 Isaac Sim 的實戰經驗整理成可公開閱讀的繁中教學 repo。
> 素材來自 2026 物流展專案的內部文件(節錄時剔除敏感資訊:IP、帳密、金鑰、內部主機名)。
> 工作法:每輪推進一個主題;文件從根本問題推導(第一性原理),關鍵概念配 SVG 圖。

## 主題規劃

| # | 主題 | 目錄 | 狀態 |
|---|---|---|---|
| 1 | 安裝與執行模式(GUI / headless / streaming) | `docs/01-install-and-run-modes/` | 第一版完成(2026-07-20) |
| 2 | 不碰 UI:用 Python 操作 Isaac Sim(standalone script、遠端) | `docs/02-python-no-ui/` | 第一版完成(2026-07-20) |
| 3 | 模型檔案格式與匯入(USD 為核心;URDF / OBJ / FBX 轉換) | `docs/03-model-import/` | 第一版完成(2026-07-20) |
| 4 | 建立物理世界(Stage、物理場景、剛體、碰撞、地面、重力) | `docs/04-physics-world/` | 第一版完成(2026-07-20) |
| 5 | ROS2 橋接(topic、TF、模擬狀態控制) | `docs/05-ros2-bridge/` | 第一版完成(2026-07-20) |
| 6 | WebRTC 串流與多 client 觀看 | `docs/06-webrtc-streaming/` | 第一版完成(2026-07-20) |
| 8 | 5.1 → 6.0.1 遷移 OOM/異常風險調查 | `docs/08-migration-5.1-to-6.0-oom-risk/` | 第一版完成(2026-07-20,調查報告性質,原始觀察未直接重現) |
| 9 | 物理模擬基礎(timestep/solver/joint drive/reset 語意) | `docs/09-physics-simulation-fundamentals/` | 第一版完成(2026-07-23,含 3 個實戰案例接引) |
| 10 | 場景資產的物理結構(剛體分層、質量、物理材質綁定) | `docs/10-scene-physics-authoring/` | 第一版完成(2026-07-26) |
| 11 | 即時位姿與放置精度驗收 | `docs/11-live-pose-and-accuracy/` | 第一版完成(2026-07-26) |
| 12 | 長跑維運(狀態分歧、看門狗分層、靜默失敗) | `docs/12-long-run-operations/` | 第一版完成(2026-07-26) |
| 19 | 調參實驗的方法論(正對照、交錯 A/B、連續量、檢定力) | `docs/19-tuning-experiment-methodology/` | 第一版完成(2026-08-02) |
| 20 | 用 Claude Code 跑調參的工作法(監看、守門、成本分工) | `docs/20-claude-code-driven-tuning/` | 第一版完成(2026-08-02) |

## R4(2026-08-02 完成)

素材來自東京 6.0 調校的暴衝根因追查與底盤驅動實驗週(isaac-sim-60-tuning docs 147~158)。

- [x] 新增 19 篇「調參實驗的方法論」:極端值正對照、耦合參數等比例、二元判準的檢定力陷阱
      (30% 對半砍需 121 輪/組)、連續量判別式、逐輪交錯、每輪閘門、取樣經濟學、
      間歇性問題的宣告門檻;附開跑前檢查清單。
- [x] 新增 20 篇「用 Claude Code 跑調參的工作法」:兩層監看(事件層+後備層)、批次自守門、
      逐輪紀錄當跨 session 記憶、模型成本分工、實驗迴圈紀律、工具冪等、獨立驗證機制。
- [x] 09 篇補 §2.5 睡眠機制(wake counter、抖動接觸對永遠不睡——對應暴衝現場)與
      §4 的 maxForce/type、等比例縮放、「關節名≠世界軸」(根 orient 使 world_x→世界 −Y 實例
      + 官方 known issue「parent≠body0 回傳值取反」)。
- [x] 15 篇補 §3.1 Newton 官方九條已知限制(@6.0.0 官方文件)與匯入器 schema 授權變化
      (Newton schema 併授、MassAPI 僅非預設密度才授);§4.5 補「schema 事實 ≠ 你場景的根因」
      實測警語(限速後暴衝率無可量測下降,真因是睡不著的接觸對)。
- [x] 04 篇補「關節名不保證世界軸向」警語;skill isaac-sim-60 擴充:第五個無聲失效條件
      (runtime 剛體授權不被採用)、§2.3 軸向、§2.4 drive 等比例、§2.5 睡眠對、
      預設值表補 sleepThreshold/disableSleeping。

## R3(2026-07-26 完成)

素材來自一段連續調試期的實測:場景物理結構盤點、放置精度量測工具鏈,以及一次「唯讀觀測 API 弄壞控制鏈」的事故。

- [x] 新增 10 篇「場景資產的物理結構」:從「剛體 = 一組碰撞體的剛性集合」推導三層結構的必然性,並說明兩種錯法(RigidBodyAPI 掛葉節點 / CollisionAPI 只在剛體層)各自怎麼壞;質量比與接觸求解收斂的關係;USD material purpose 機制與 `ComputeBoundMaterial("physics")` 的誤判判準(回傳非 None 不代表綁上了);執行期補綁的三個邊界。
- [x] 新增 11 篇「即時位姿與放置精度」:四種讀位姿方法的實測對照(只有 `omni.physx` 的 `get_rigidbody_transformation` 可用);一次 simulation view 永久失效的事故(含官方查證後的因果修訂,見下);讀錯層的雙胞胎陷阱(含跟隨鏡頭);量測管線的自證步驟(靜置全 0);誤差來源拆解與逐輪正回饋累積;兩個失敗修法(收緊容差反而惡化、高摩擦只治側滑)。
- [x] 新增 12 篇「長跑維運」:重啟只重置物理狀態造成的帳面分歧(表現形式是「成功」);三層看門狗與執行器不重疊原則;基於 `framesDecoded` 的串流卡死偵測;三個殼層陷阱(`/proc/<pid>/fd/1` 反查 log、`pkill -f` 自匹配、zsh 不斷詞);A/B 測試的單變因紀律。
- [x] **官方查證後的修訂(重要)**:原稿把「`XFormPrim.get_world_poses()` 建立 tensor view → 弄壞 ActionGraph」寫成機制,但 Isaac Sim 5.1.0 官方文件明寫 `XFormPrim.get_world_poses()` 讀的是 USD/Fabric、`XFormPrim.initialize()` 「will do nothing」,明文與 tensor API 綁定的是 `RigidPrim` / `Articulation` 子類。已改寫成「可重現的相關性、機制未確認」,並補上 `SimulationView.is_valid` / `invalidate()` 的官方失效條件,以及「該錯誤字串只出現在 runtime 與論壇,非文件用語」的標註。同時修正 usdrt `HasWorldXform()` 的語意(官方是「Fabric prim 有無被寫入 world transform 屬性」,不是「有沒有被同步過」),並補上四元數順序在 `omni.physx`(x,y,z,w)與 `isaacsim.core.prims`(w,x,y,z)之間分裂的官方對照。
- [x] 補齊逐字引用與 URL:omni.physx `get_rigidbody_transformation`(107.3,含 110.1 索引查無的版本邊界標註)、OpenUSD 材質解析第 3 條規則與 UsdPhysics 的 `"physics"` purpose 明文、W3C `framesDecoded` 定義。移除先前憑印象寫下、未經查證的連結。
- [x] 新增 5 張 SVG,chrome-headless 逐張渲染檢查(過大箭頭、標籤壓線、標記顏色不一致均已修):`rigidbody-collision-layering`、`live-pose-read-paths`、`placement-error-decomposition`、`state-divergence-on-restart`、`watchdog-layers`。

## R2(2026-07-23 完成)

- [x] 新增 09 篇「物理模擬基礎」:官方 Physics Simulation Fundamentals + Articulation Stability Guide + PhysX Joints/Rigid Body Dynamics 文件查證(WebFetch/WebSearch),涵蓋 timestep/substep、rigid body/collision(contact/rest offset)、CCD、articulation joint drive PD 公式(逐字核對 PhysX 官方 `force = stiffness*(targetPosition-position)+damping*(targetVelocity-velocity)`)、PGS/TGS solver 與 iteration count、kinematic target vs teleport 的官方語意差異(`setKinematicTarget` vs `setGlobalPose`)、timeline stop/play 原生重置。
- [x] 沿用前一輪(未提交)畫好的 5 張 SVG,原樣接進對應章節,chrome-headless 渲染驗證(900×600 @2x)無破圖:`physics-timestep-substep.svg`、`solver-pgs-tgs-iterations.svg`、`contact-offset-ccd.svg`、`joint-drive-pd-control.svg`、`teleport-vs-native-reset.svg`。
- [x] 接入三個內部實戰案例(引用本機 `2026-logistical-expo` repo 對應文件,不複製其內容):95 篇(穿模根因:teleport 掛載 vs PD 物理叉取)、88 篇(目的地無效→NaN→AMR 暴走)、91 篇(teleport-only reset 救不回發散、timeline 原生重置)。
- [x] `2026-logistical-expo/docs/circ-ai-isaac-ros/` 同步新增 96 篇精簡版(連回本篇詳版與 88/91/95),INDEX.md 補列。

## R1.5(2026-07-20 完成)

- [x] 專家/學生雙視角審查 + 官方文件查證(WebSearch 對 GitHub tag 原始碼),套用 24 項修訂:
  - 更正:URDF importer 為 `isaacsim.asset.importer.urdf`(`isaacsim.ros2.urdf` 是其 ROS2 擴充);`SingleArticulation` 無 `set_joint_position_targets`,PD 目標改教 `apply_action(ArticulationAction)`。
  - 版本邊界:全系列標明 API 以 4.5–5.1.x 為準;6.0 起 `isaacsim.core.*` 移至 `experimental.*`、`open_stage` 回傳 tuple。
  - 術語補翻譯(Kit/OptiX/ECC/DDS/MDL/DOF/PD/ICE)+ 05 篇加 ROS2 最小背景;個案觀察改經驗歸納語氣。
  - `virtual-joints.svg` 改雙色 + 圖例(虛擬 vs 機構關節)。
- [x] 新增 07 最小可跑範例(方塊落地 / 開官方倉庫 / Nova Carter 讀位姿)——依官方文件與 standalone_examples 組合,**尚未實機驗證**,文內已標註。

## 下一輪候選

- [ ] 07 篇三個範例實機驗證(需 GPU 機),回填實測輸出並移除「未驗證」標註。
- [ ] 04 篇的軌跡規劃/殘差補償細節(soft-pass、overdrive)可展開成獨立進階篇。
- [ ] 6.x `isaacsim.core.experimental.*` 等價 API 對照表(研究時標記為待查證)。
- [ ] 08 篇:向同事追問 OOM 觀察的實際機器/場景/發生階段,若能重現則補實測數據、移除「尚未重現」標註。

## 每輪收尾

1. 文件寫入 `docs/NN-主題/`,更新 `README.md` 索引與 `CONTEXT.md` 術語表。
2. SVG 放 `img/`,chrome-headless 轉 PNG 自我檢查後才算完成。
3. 本機 commit(繁中訊息);push 前經使用者確認。

## 待決事項

- [x] GitHub remote 名稱已定:`wicanr2/issac-sim-study`(拼字沿用既有 repo,不更名)。
- [ ] USD 素材(倉儲場景、棧板、AMR 模型)體積大,決定放 repo、Git LFS 或外部下載連結。
