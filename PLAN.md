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

## 下一輪候選

- [ ] 專家/學生雙視角審查(依 first-principles-tech-notes 工作法第 6 步),整理修訂清單後出 R1.5。
- [ ] 補「從零最小可跑範例」:官方 Simple Warehouse + Nova Carter 的 headless 完整腳本(可實機驗證後收錄)。
- [ ] 04 篇的軌跡規劃/殘差補償細節(soft-pass、overdrive)可展開成獨立進階篇。

## 每輪收尾

1. 文件寫入 `docs/NN-主題/`,更新 `README.md` 索引與 `CONTEXT.md` 術語表。
2. SVG 放 `img/`,chrome-headless 轉 PNG 自我檢查後才算完成。
3. 本機 commit(繁中訊息);push 前經使用者確認。

## 待決事項

- [ ] GitHub remote 名稱:CLAUDE.md 寫 `issac-sim-study`(疑為 isaac 拼字誤植),push 前與使用者確認。
- [ ] USD 素材(倉儲場景、棧板、AMR 模型)體積大,決定放 repo、Git LFS 或外部下載連結。
