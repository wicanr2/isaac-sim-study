# GOAL 紀錄

每一輪工作的目標檔。一份 GOAL 檔寫的是這一輪「做什麼、依什麼順序、守哪些硬規則、怎樣算完成」,由 Claude Code 的 `/goal 讀取 docs/goal/<檔名> 執行` 載入執行。結果不回寫進 GOAL 檔,而是回填到對應的 GitHub issue;GOAL 檔本身產生後就不再改動,保留當時的判斷。

命名:`GOAL-YYYY-MM-DD-HHMM.md`,時間是產生時間。新一輪的檔案放這個目錄,並在下表加一列;這一輪收工時更新該列的狀態。

| 輪 | 檔案 | 主題 | 結果回填 | 狀態 |
|---|---|---|---|---|
| 1 | [GOAL-2026-09-15-1538](GOAL-2026-09-15-1538.md) | 新增 HIL 教學區(35–38):Renode 模擬 STM32F4、Rust 橋接、假受控體閉環 | [#1](https://github.com/wicanr2/isaac-sim-study/issues/1) | 完成 |
| 2 | [GOAL-2026-09-15-1705](GOAL-2026-09-15-1705.md) | 受控體換成 Isaac 6.0.1、Renode 修正送上游、上位接 ROS 2 Jazzy | [#1](https://github.com/wicanr2/isaac-sim-study/issues/1) | 完成 |
| 3 | [GOAL-2026-09-16-1118](GOAL-2026-09-16-1118.md) | 補圖過稿、Renode PWM 事件成本、vcan 對照、上游 PR | [#1](https://github.com/wicanr2/isaac-sim-study/issues/1) | 完成 |
| 4 | [GOAL-2026-09-16-1547](GOAL-2026-09-16-1547.md) | 編碼器 encoder mode、加減速與馬達層、安全 I/O 五項、Nav2 in the loop | [#2](https://github.com/wicanr2/isaac-sim-study/issues/2)–[#5](https://github.com/wicanr2/isaac-sim-study/issues/5) | 完成 |
| 5 | [GOAL-2026-09-16-1846](GOAL-2026-09-16-1846.md) | 閒時量測、兩版韌體去重、Isaac 側補齊、上位對安全旗標的反應、俯視圖錄影 | [#1](https://github.com/wicanr2/isaac-sim-study/issues/1)、[#3](https://github.com/wicanr2/isaac-sim-study/issues/3)–[#6](https://github.com/wicanr2/isaac-sim-study/issues/6) | 完成 |
| 6 | [GOAL-2026-09-17-1240](GOAL-2026-09-17-1240.md) | realtime 連續注入、打滑偵測、IWDGRSTF 與 AMCL 重定位、Isaac 側次要項 | [#7](https://github.com/wicanr2/isaac-sim-study/issues/7)–[#10](https://github.com/wicanr2/isaac-sim-study/issues/10)、[#1](https://github.com/wicanr2/isaac-sim-study/issues/1) | 部分完成:#8–#10 完成;#7 realtime 閒時 C9 3/5、未達五次全綠,開放 |

GOAL 1–3 的檔案產生時放在 `docs/hil/`,2026-09-17 集中到這個目錄;issue 留言裡引用的舊路徑沒有改寫。
