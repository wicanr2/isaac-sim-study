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

- [ ] GitHub remote 名稱:CLAUDE.md 寫 `issac-sim-study`(疑為 isaac 拼字誤植),push 前與使用者確認。
- [ ] USD 素材(倉儲場景、棧板、AMR 模型)體積大,決定放 repo、Git LFS 或外部下載連結。
