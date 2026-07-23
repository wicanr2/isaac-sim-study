# CONTEXT — 術語表

> 本 repo 的統一用語(ubiquitous language)。首次出現的術語在文中當場一句話翻譯,這裡收長期定義。

| 術語 | 定義 |
|---|---|
| Isaac Sim | NVIDIA 的機器人模擬平台,建立在 Omniverse / USD 之上。 |
| Omniverse Kit | NVIDIA 的應用框架(引擎 + extension 系統);Isaac Sim 是搭在 Kit 上的一個應用。 |
| SimulationApp | standalone Python 工作流的入口類別;實例化它之後才能 import 其他 `omni.*`/`isaacsim.*` 模組。 |
| OptiX | NVIDIA 的 RTX 光線追蹤引擎,Isaac Sim 渲染層的底層;對 GPU 驅動 ABI 敏感。 |
| ECC | Error-Correcting Code,記憶體糾錯;資料中心 GPU 預設開啟,實戰上與 RTX 渲染間歇 crash 相關。 |
| DDS | Data Distribution Service,ROS2 底層的訊息中介層,負責節點彼此發現與資料傳輸。 |
| MDL(.mdl) | NVIDIA Material Definition Language,Omniverse 的材質格式;USD 場景常外部引用材質庫。 |
| DOF | degree of freedom,自由度;一個關節提供一個(或多個)自由度。 |
| PD 控制 | 比例-微分控制;joint drive 以剛度(P)/阻尼(D)把關節拉向目標值。 |
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
| Newton | NVIDIA 6.0 起引入的實驗性物理後端,與 PhysX 並列可切換;USD schema 上對應 `NewtonSceneAPI`/`NewtonArticulationRootAPI`/`NewtonMimicAPI` 等 token,取代部分 `Physx*API`。 |
| texture streaming budget | Isaac Sim 控制貼圖串流佔用 GPU 記憶體上限的設定(`/rtx-transient/resourcemanager/texturestreaming/memoryBudget`),預設吃 60% GPU 記憶體容量。 |
| Nucleus | Omniverse 的資產伺服器/版本控管系統;Isaac Sim 官方資產包掛在其下,路徑依版本命名空間化(如 `.../Isaac/6.0`)。 |
| crate 格式(USD) | USD 的二進位序列化格式(`.usd` 副檔名但內容是 crate binary),以 `PXR-USDC` 開頭,欄位/schema 名稱存在字串 token 表,可用 `strings` 粗略比對版本差異。 |
| contact offset | 碰撞偵測開始產生接觸約束的距離門檻,不管 rest offset 設多少都適用;太小則偵測太晚(抖動/穿透),太大則約束變多變貴。 |
| rest offset | 拿來膨脹或縮小碰撞幾何體的緩衝值,補視覺網格與碰撞近似體積之間的落差。 |
| CCD(Continuous Collision Detection) | 連續碰撞偵測,對物體移動路徑做掃掠測試(sweep test),補「一步移動太快、離散取樣點漏抓」造成的穿透;只對有速度的連續運動有效,對每步直接覆寫位姿(teleport)無效。 |
| PGS(Projected Gauss-Seidel) | PhysX 的一種疊代求解器:每次疊代對整個 timestep 只修一次位置,長鏈末端收斂較慢。 |
| TGS(Temporal Gauss-Seidel) | PhysX 5.1 起建議且預設的疊代求解器:把 timestep 內部切小段疊代並逐段推進位置,長鏈更穩;solver iteration count(position/velocity)決定疊代精細度。 |
| kinematic actor | PhysX 裡以官方 `setKinematicTarget()` 逐步設定目標姿態的剛體:對外表現為無限質量,會正常推開 dynamic 物體(有物理互動);與 teleport(`setGlobalPose`/直接改 world pose)不同——teleport 不觸發碰撞響應,會直接穿過其他物體。 |
| solver 內部狀態 | PhysX 求解器自己維護的殘留量(積分速度、接觸快取、articulation 矩陣),不存在 USD 裡,teleport 類操作碰不到;要清除必須靠 `timeline.stop()`→`play()` 銷毀重建整個物理場景。 |
