# CONTEXT — 術語表

> 本 repo 的統一用語(ubiquitous language)。首次出現的術語在文中當場一句話翻譯,這裡收長期定義。

| 術語 | 定義 |
|---|---|
| Isaac Sim | NVIDIA 的機器人模擬平台,建立在 Omniverse / USD 之上。 |
| USD | Universal Scene Description,Pixar 開源的 3D 場景描述格式;Isaac Sim 的原生場景格式。 |
| Stage | USD 的場景樹(一份開啟中的場景),所有 Prim 都掛在 Stage 上。 |
| Prim | USD 場景樹的節點(primitive),可以是 Mesh、Xform、Light、Physics Scene 等。 |
| headless | 不開 GUI 視窗的執行模式,適合遠端伺服器與自動化。 |
| standalone script | 用 Isaac Sim 內附 python 直接跑的腳本,由腳本自己啟動 SimulationApp,不經 UI。 |
| WebRTC streaming | Isaac Sim 把畫面經 WebRTC 送到遠端瀏覽器/client 的串流機制。 |
| ROS2 bridge | Isaac Sim 與 ROS2 之間的橋接(topic / TF / 服務),讓外部節點控制或讀取模擬。 |
| articulation | PhysX 的關節樹:多個剛體以 joint 串接,joint 可帶 drive(馬達)接受位置/速度目標。 |
| OmniGraph | Kit 的視覺化節點圖框架;ROS2 pub/sub 節點與 ScriptNode 都掛在其上。 |
| carb settings | Kit 的全域設定樹,GUI 快捷鍵與 `--/path=value` 啟動參數都是改這棵樹。 |
| WHIP / WHEP | WebRTC 推流 / 拉流的標準 HTTP 訊令協定;mediamtx 用它收一路、發多路。 |
| teleport | 直接改物件狀態(位姿/關節值)而不經物理 drive 追蹤;與 PD 目標控制語意相對。 |
