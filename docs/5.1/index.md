# Isaac Sim 5.1

本 repo 的實測環境以 Isaac Sim 5.1.x 為主(部分 API 對照回溯到 4.5),這一區收「只在 5.x 命名空間下成立」的內容——主要是程式碼。兩版通用的機制與方法論在[共通區](../common/),與 6.0.1 的差異見[版本速查表](../version-matrix.md)。

## 這一區只有兩篇的原因

5.1 與 6.0 之間,絕大多數機制沒有改變:物理引擎怎麼算一步、碰撞近似損掉什麼、質量該掛在哪一層、材質綁定何時 fallback,這些在兩版是同一套,因此留在共通區而不是複製兩份。真正綁死版本的是 **Python 模組路徑**——6.0 起 `isaacsim.core.api` / `isaacsim.core.prims` / `isaacsim.core.utils` 整組移到 `isaacsim.core.experimental.*`,凡是貼了可執行程式碼的篇章都會因此失效。這一區收的就是那兩篇。

## 篇章

- **[07 · 最小可跑範例](07-minimal-example/README.md)** —— 三個由小到大的 standalone 範例:方塊落地、載入官方倉庫場景、在官方場景裡讀機器人位姿。全部不開 GUI 也能跑。程式碼依官方文件與 `standalone_examples` 組合,**尚未在本 repo 環境實機驗證**,資產路徑在不同文件版本間基準點不一,執行時用文中的 `is_file()` 檢查確認。
- **[11 · 讀取即時位姿與放置精度驗收](11-live-pose-and-accuracy/README.md)** —— 四種讀位姿的方法只有一種讀得到模擬當下的值,另外三種各有各的騙法;唯讀的觀測 API 反而會弄壞控制鏈;後半把「放得準不準」變成可量測、可驗收的管線。四元數順序在 `omni.physx`(x,y,z,w)與 `isaacsim.core.prims`(w,x,y,z)之間分裂,這一條在 6.0 改了模組路徑之後仍要留意。

## 從零開始的閱讀動線

前面幾篇在共通區,順序是:

1. [01 安裝與執行模式](../common/01-install-and-run-modes/README.md) —— GUI / headless / streaming 是同一核心的三種前端。**選版本前先看 §3**:5.1 配 595 世代驅動會在啟動約 65 秒必 segfault。
2. [02 不碰 UI:用 Python 操作](../common/02-python-no-ui/README.md) —— `--exec` 啟動腳本、ScriptNode、UDP 遠端命令通道三層做法。
3. [03 模型格式與匯入](../common/03-model-import/README.md) → [04 建立物理世界](../common/04-physics-world/README.md) —— 一切先轉 USD;PLAYING 才有物理。
4. [09 物理模擬基礎](../common/09-physics-simulation-fundamentals/README.md) → [13 接觸與抓握的第一性原理](../common/13-contact-and-grasp-first-principles/README.md) —— 引擎內部怎麼算,以及為什麼調摩擦常常是錯的第一步。
5. 動手跑 [07](07-minimal-example/README.md)。
6. 要建一個能跑物理搬運的場景,接著讀 [10 場景資產的物理結構](../common/10-scene-physics-authoring/README.md) → [11](11-live-pose-and-accuracy/README.md) → [12 長跑維運](../common/12-long-run-operations/README.md)。

接 ROS 2 或要遠端看畫面,另外讀 [05 ROS2 橋接](../common/05-ros2-bridge/README.md) 與 [06 WebRTC 串流](../common/06-webrtc-streaming/README.md),兩篇的實測都在 5.1 上做的。

## 準備升到 6.0

先讀 [15 物理層變動](../6.0.1/15-physics-backend-5.1-to-6.0/README.md) 與 [14 ROS 2 Bridge 架構重組](../6.0.1/14-ros2-bridge-6.0-architecture/README.md),再看 [08 遷移風險調查](../6.0.1/08-migration-5.1-to-6.0-oom-risk/README.md)。整區入口在 [6.0.1](../6.0.1/)。
