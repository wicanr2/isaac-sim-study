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
| gap function `g` | 兩物體最近點的**有號距離**:分開為正、恰好接觸為 0、穿透為負。接觸求解的基本量。 |
| Signorini 條件 | 單邊接觸的三條約束:`g ≥ 0`、`λₙ ≥ 0`、`g·λₙ = 0`。第三條(互補)意謂「分開就沒有力,有力就一定貼著」。 |
| 摩擦錐 | 庫倫摩擦的可行域:切向力受限於 `‖f_t‖ ≤ μ·λₙ`,幾何上是半頂角 `arctan(μ)` 的圓錐。`λₙ = 0` 時退化成一個點。 |
| 碰撞近似(collision approximation) | 把 mesh 編碼成便於求 `g` 的代理形狀(convexHull / convexDecomposition / sdf / boundingCube …)。**有損**,損掉什麼決定哪些任務做不成。 |
| 凸包 convexHull | 含所有頂點的最小凸集。**定義上無法表達凹特徵**——孔洞內每一點都是孔周頂點的凸組合,必被填實。 |
| SDF(signed distance field) | 體素化的有號距離場 `φ(x)`,內負外正。純量場、對拓撲無限制,能表達孔洞與凹槽;代價是記憶體 `O(res³)`。 |
| contact offset | 碰撞偵測開始產生接觸約束的距離門檻,不管 rest offset 設多少都適用;太小則偵測太晚(抖動/穿透),太大則約束變多變貴。 |
| rest offset | 拿來膨脹或縮小碰撞幾何體的緩衝值,補視覺網格與碰撞近似體積之間的落差。 |
| CCD(Continuous Collision Detection) | 連續碰撞偵測,對物體移動路徑做掃掠測試(sweep test),補「一步移動太快、離散取樣點漏抓」造成的穿透;只對有速度的連續運動有效,對每步直接覆寫位姿(teleport)無效。 |
| PGS(Projected Gauss-Seidel) | PhysX 的一種疊代求解器:每次疊代對整個 timestep 只修一次位置,長鏈末端收斂較慢。 |
| TGS(Temporal Gauss-Seidel) | PhysX 5.1 起建議且預設的疊代求解器:把 timestep 內部切小段疊代並逐段推進位置,長鏈更穩;solver iteration count(position/velocity)決定疊代精細度。 |
| kinematic actor | PhysX 裡以官方 `setKinematicTarget()` 逐步設定目標姿態的剛體:對外表現為無限質量,會正常推開 dynamic 物體(有物理互動);與 teleport(`setGlobalPose`/直接改 world pose)不同——teleport 不觸發碰撞響應,會直接穿過其他物體。 |
| solver 內部狀態 | PhysX 求解器自己維護的殘留量(積分速度、接觸快取、articulation 矩陣),不存在 USD 裡,teleport 類操作碰不到;要清除必須靠 `timeline.stop()`→`play()` 銷毀重建整個物理場景。 |
| material purpose | USD 材質綁定的用途分類(如 `allPurpose` / `physics`);同一個 prim 可同時綁渲染材質與物理材質,用 purpose 區分。查詢 `physics` purpose 時可能沿 fallback 回傳渲染材質,故判準是「回傳的是不是 PhysicsMaterial」而非「有沒有回傳」。 |
| simulation view | PhysX tensor API 的批次存取控制代碼;建立新的 view 會使既有 view 失效。`XFormPrim.get_world_poses()` 這類批次張量 API 背後會建立 view,因而可能靜默作廢 ActionGraph 正在用的 view。 |
| authored 位姿 | USD Stage 上寫下的位姿(模型檔存的初始值),不隨模擬前進而改變;與 PhysX 內部狀態、Fabric 快取並列為場景狀態的三個所在地。 |
| soft-pass 容差 | 控制器在主要容差達不到時退用的次要(較寬)容差。把主要容差設得太緊會讓系統幾乎每次都走這條退路,實際精度反而由退路決定。 |
| framesDecoded | W3C WebRTC 統計欄位(`RTCInboundRtpStreamStats`),累計已解碼的視訊幀數;連續數次不增長即可判定串流卡死,比連線狀態更能證明「真的在工作」。 |
| 狀態分歧 | 模擬器持有的物理狀態與外部系統持有的帳面狀態不一致。單獨重啟其中一方即會產生,且表現形式通常是流程「成功」空轉而非報錯。 |
| umbrella package(門面 extension) | 自己不含實作、只在 `[dependencies]` 列出實作者的 extension。Kit 會遞迴解析依賴,所以 enable 門面等於 enable 整套。6.0 的 `isaacsim.ros2.bridge` 即為此形態。 |
| active physics engine | 當前實際參與模擬的物理引擎(`physx` 或 `newton`)。由 `isaacsim.core.simulation_manager` 的 `default_engine` 與 `isaacsim.physics.newton` 的 `auto_switch_on_startup` 共同決定,後者預設 true 會搶成 Newton。 |
| `auto_switch_on_startup` | `isaacsim.physics.newton` 的設定,預設 `true`:只要該 extension 被啟用,啟動時就把 active engine 切成 Newton,即使 `default_engine` 仍寫 `physx`。 |
| rclpy fallback 鏈 | Isaac Sim 啟動 ROS 2 時「先試系統 rclpy、失敗才載內建」的順序。啟動前 source 系統 ROS 會讓第一步成功,載進為系統 Python 編譯的 C extension,造成 ABI 不匹配且不報乾淨的錯。 |
| deprecated 而非移除 | 官方把 extension 從 `source/extensions/` 移到 `source/deprecated/`:仍可用、仍隨版本散佈,只是不再建議。**目錄結構的增減不能直接讀成功能存廢。** |
| 無聲丟棄(schema 不匹配) | 場景 USD 帶著某後端專屬的 schema token(如 `Newton*API`),但實際跑的是另一個後端。schema extension 有載入所以讀得進去、不報錯,設定卻不被採用。 |
