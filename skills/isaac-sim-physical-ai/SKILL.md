---
name: isaac-sim-physical-ai
description: 在 Isaac Sim(4.5 / 5.1 / 6.0.x)上做 physical AI —— 讓模擬裡的接觸、抓握、搬運「物理上真的成立」,而不是帳面成功。核心命題兩條:①接觸力學決定調參順序,摩擦係數 μ 是一個乘數,乘在一個可能為零的法向力上,所以幾何永遠先於摩擦;②物理求解器沒有「非法狀態」的概念,永遠給得出答案,所以「沒報錯」攜帶零資訊,只能主動量測。附版本差異矩陣(含 6.0 的 PhysX 110 換代與 Newton 後端判定)與 headless 觀測手法。觸發:「夾爪/叉齒抓不住、物件滑掉、插不進去」「調摩擦 / 碰撞近似 / 接觸參數 / 質量」「Isaac 車不動但沒錯誤」「模擬跑得動但實體沒動」「headless Isaac 怎麼除錯」「Isaac 版本升級 / 5.1→6.0 遷移」「sim-to-sim 把場景搬到另一台」「6.0 用 PhysX 還是 Newton / 後端怎麼選」「MassAPI / Newton schema 靜默失效」。
---

# Isaac Sim Physical AI

做 physical AI 不是「讓機器人在畫面上把東西搬走」,是**讓接觸力在物理上真的成立**。
帳面成功而物理失敗是這個領域的預設失效模式,而且**全程不會有錯誤訊息**。

三層結構:**版本無關的第一性原理**(§1–§4,最重要)、**版本差異矩陣**(§5)、**專案實例**(references,按需讀)。

---

## 1. 第一性原理:為什麼「調摩擦」常常是錯的第一步

### 1.1 不穿透怎麼變成方程式

定義 gap function `g(q)` = 兩物體最近點的**有號距離**(分開為正、穿透為負),`λₙ` = 法向接觸力大小。
單邊接觸(unilateral contact)必須同時滿足 **Signorini 條件**:

```
g  ≥ 0        不穿透
λₙ ≥ 0        接觸力只能推、不能拉(這就是「單邊」的意思)
g · λₙ = 0    互補:兩者不能同時非零
```

第三條說的是:**分開就沒有力,有力就一定貼著**。這是所有剛體接觸求解器的骨架
(線性互補問題 LCP;PhysX / Bullet / MuJoCo 求的都是它的某種變形)。

> 替代路線是罰函數法:`λₙ = k·max(0, −g)`,好寫但 `k` 要很大才不明顯穿透,而大 `k` 讓系統變剛性、
> 需要極小 timestep。互補條件是硬約束,不必調 `k`,代價是要解 LCP。理解 Signorini 就理解了主流那條。

### 1.2 摩擦錐

庫倫摩擦把切向力限制在一個以法線為軸、半頂角 `arctan(μ)` 的錐內:

```
黏著(v_t = 0):  ‖f_t‖ ≤ μ_s · λₙ
滑動(v_t ≠ 0):  f_t = −μ_d · λₙ · v̂_t
```

### 1.3 串起來 —— 本 skill 最重要的一行

```
g > 0  ⟹  λₙ = 0  ⟹  ‖f_t‖ ≤ μ · 0 = 0
```

**沒有接觸點,摩擦力恆為零,μ 設多大都一樣。**

幾何上,`λₙ = 0` 時摩擦錐退化成原點一個點,可行域只剩零向量。
**μ 控制錐的張角,不控制錐的存在。**

### 1.4 因此責任分三層,順序不可交換

| 層 | 決定什麼 | 由誰決定 | 典型症狀 |
|---|---|---|---|
| **幾何** | `λₙ` 存不存在、在哪、法線朝哪 | 碰撞近似 + 位姿 | **完全插不進去、物件從未被抬起** |
| **摩擦** | `f_t` 的上界 | 材質 μ + **綁定** | 抬得起來但移動中滑掉、放下歪 |
| **求解** | 前兩者的數值精度 | solver 迭代、質量比、timestep | 抖動、下陷、接觸處震盪 |

**第一列的情況下調 μ 不是「通常沒用」,是數學上不可能有用。**

調參順序因此是必然的:**幾何 → 質量/慣量 → 接觸參數(offset)→ 摩擦 → solver 迭代**。
順序錯了會得到一堆「有效但說不出為什麼」的魔數。

---

## 2. `g` 從哪來:碰撞近似是有損編碼

`g` **不是從原始 mesh 算的**。原始 mesh 動輒數萬三角形,逐步做三角形相交測試代價不可接受。
引擎先把 mesh **編碼成便於求 `g` 的代理形狀**,之後所有接觸判定都在代理上做。

**這個編碼有損。損掉什麼,決定了哪些任務做不成。**

| 近似 | 代理是什麼 | 能表達凹陷/孔洞 | 代價 |
|---|---|---|---|
| `convexHull` | 含所有頂點的最小凸集 | ❌ **定義上不能** | 低 |
| `convexDecomposition` | N 個凸塊的聯集 | 部分(取決於 N) | 中 |
| `sdf` | 體素化有號距離場 `φ(x)` | ✅ | 記憶體 `O(res³)` |
| `boundingCube` / `boundingSphere` | 外接盒 / 球 | ❌ | 極低 |
| `triangleMesh`(原始三角網) | 三角形 | ✅ 但**只能當靜態碰撞體** | 高 |

**凸包為什麼對「帶孔薄板」必死**——直接從定義推:

```
conv(V) = { Σ λᵢvᵢ | λᵢ ≥ 0, Σλᵢ = 1, vᵢ ∈ V }
```

孔洞內任一點都可寫成孔周頂點的**凸組合** → **必然落在 `conv(V)` 內** → **孔必被填實**。
這不是精度誤差,是定義的直接後果。於是「叉齒插叉孔」變成「兩個實心體互相擠壓」,`g < 0`,
solver 產生巨大推開力 —— 症狀是**插不進去**,不是「夾不緊」。

**SDF 為什麼能救**:`φ(x)` 是**純量場**不是凸集,對拓撲沒有限制,孔洞/凹槽/薄殼都能表達;
`g` 直接查值 + 三線性插值。代價是記憶體 `O(res³)` 與載入時體素化。

**判準一句話:這個零件的功能是否依賴某個凹特徵?**
叉齒插孔、夾爪夾槽、插銷進洞 → 要 SDF(或夠細的凸分解);箱子、輪子、地板 → 凸包就夠。

### 2.1 `contactOffset` / `restOffset`:「剛好接觸」在數值上不存在

離散積分下 `g` 精確等於 0 幾乎不可能命中,所以引擎在物體外包兩層殼:
`g < contactOffset` 就生成接觸約束;靜止平衡時 `g` 收斂到 `restOffset`。必須 `d_c > d_r`。

- `d_c` 太小 → 高速時兩步之間跨過整個殼層,約束**根本沒被生成** → 穿模;
- `d_c` 太大 → 物體看起來浮著、行為變軟。

**所以穿模不是靠加大 solver 迭代解決的**:迭代求解的是**已建立**的約束,約束沒建立時迭代一萬次也是零。

### 2.2 質量比是隱藏參數

接觸約束交給迭代式 solver(PGS / TGS)逐個投影求解,是**收斂中的近似解**。
兩接觸體的質量比越懸殊,系統條件數越差、同樣迭代次數下殘差越大。

實測參考:承載件 50 kg 對被搬物 20 kg(2.5:1)可穩定;被搬物提到 100 kg 以上而迭代不變,
症狀是「抬得起來但一轉彎就滑掉」—— 那是殘差,不是摩擦不足。

---

## 3. 第一性原理:為什麼模擬器不會告訴你哪裡錯了

編譯器有「合法程式」這個規格,所以能指出你違反了它。
**物理求解器沒有「非法狀態」這個概念** —— 它的契約是「給定 `t` 的狀態,回傳 `t+Δt` 的狀態」。
`NaN` 是合法 float,`10⁶` 公尺是合法座標,`λₙ = 0` 是合法解。

> **推論:在物理模擬裡,「沒有報錯」攜帶零資訊。**

所以除錯只能靠**主動量測**,而且要量對層:

| 要回答 | 量什麼 | 常見錯法 |
|---|---|---|
| 有沒有接觸 | 碰撞近似型別、接觸點數 | 看畫面(視覺 mesh ≠ 碰撞代理) |
| 咬住後有沒有滑 | 被搬物**相對承載件**的位姿(不變量),**整趟連續取樣** | 只在動作結束後量一次 |
| 位姿對不對 | 剛體查詢的**即時**世界座標 vs 真值 | 讀到 authored 值還以為是即時值 |
| 誤差來自哪一層 | 拆成 車體定位 / 插入深度 / 橫向滑移 / 偏航 四項 | 只看一個總誤差 |

**「相對承載件的位姿」是關鍵不變量**:絕對座標在載具行進時本來就一直變,
拿絕對座標看不出「東西有沒有相對夾持面滑掉」。整趟不變 = 沒滑;單調漂移 = 真的在滑。

⚠ 另一個陷阱:**查詢回傳「成功」不代表值可用**。對非剛體查剛體變換,某些引擎會回成功但內容是
未初始化記憶體(位置量級 `1e9` 或 `1e-41`、四元數范數 ≈ 0)。每次取完都要過合理性檢查
(有限、在場景尺度內、四元數范數接近 1)。

---

## 4. 三個會反覆咬人的結構性問題

### 4.1 三層真值,彼此沒有一致性保證

| 層 | 內容 | 何時變 |
|---|---|---|
| **authored** | 模型檔(USD)裡寫的 | 只有改檔案時 |
| **simulated** | 物理引擎內部狀態 | 每個 timestep |
| **business** | 業務系統(WMS/DB)登記 | 事件驅動 |

**沒有任何機制保證三者一致。** reset 讓 authored 覆蓋 simulated,但 business 不會跟著回去 →
「任務兩分鐘就完成、車完全沒動」。

推論:**宣稱搬運成功必須同時驗 simulated 與 business 兩軌**,任一軌單獨都說過謊。
真實機器人裡對應 CAD 模型 / 感測位姿 / WMS 記錄 —— **這個結構是 sim-to-real 共通的**。

### 4.2 開環致動對上可移動目標 = 結構性發散

固定行程插入,若目標已偏深 `δ`,接觸後仍走完剩餘行程 → 再推深一點。離散動態系統:

```
x_{k+1} = x_k + Δ(x_k),  Δ > 0  ⟹  單調發散
```

**沒有負回饋項,所以這不是 bug,是開環控制的必然。** 三種修法:加感知回授(治本)、
週期性重置(止血)、讓目標不可移動(改變問題)。

⚠ **這個發散在真實世界也會發生** —— 真堆高機也會把棧板越推越深。模擬忠實重現了它,
這是模擬對的地方,不是缺陷。把它當 bug 去調物理參數會白費很多時間。

### 4.3 有效場景 = authored ⊕ runtime mutations

啟動腳本常在 runtime 改碰撞近似、補綁材質、隱藏幾何。所以:

- **「複製場景檔」≠「複製場景」** —— 版本控制只管到 authored 那一半;
- 要重現一個行為,必須同時固定兩半;
- **runtime mutation 應是暫時的診斷手段,驗證通過就寫回 authored**,否則每次部署都是賭博。

實例:把 `.usd` 搬到另一台主機後抓握失敗,查很久才發現 SDF 是啟動腳本 runtime 套的,
新主機的腳本沒有那段 —— **換錯了變數**,結論還被寫成「換場景檔沒用」。

---

## 5. 版本差異矩陣

**同一份 Python 在不同版本會 import 失敗或行為不同。動手前先確認手上是哪一版。**

| 面向 | 4.5 | 5.1 | 6.0.x |
|---|---|---|---|
| Kit | 105 | 106 | 110 |
| **物理引擎** | PhysX | PhysX | PhysX **或 Newton**(要顯式選,見 §5.6) |
| **PhysX 版本**(實測) | — | `omni.physx-107.3.26` | `omni.physx-110.1.13`(**跨三個大版號**) |
| 核心 API 命名空間 | `omni.isaac.*` | `isaacsim.core.api` / `.prims` / `.utils` | 整組移至 `isaacsim.core.experimental.*`(**breaking**);舊的搬到 `source/deprecated/`,仍可用 |
| 關節目標設定 | `set_joint_position_targets` | 同左 | **已移除** → 用 `apply_action`(PD 物理驅動) |
| 典型部署 | — | native(`isaac-sim.streaming.sh` + `isaacsim.exp.full.streaming.kit`) | 容器 `nvcr.io/nvidia/isaac-sim:6.0.1` |
| ROS2 bridge 內附 DDS | humble FastDDS 2.6.11 | 同左 | **jazzy FastDDS 2.14.6** |
| 驅動相容(實測) | — | 555 世代穩;**595 / CUDA 13 → 啟動約 65 秒 segfault** | 595 正常 |

### 5.1 跨版本最貴的坑:DDS 共享記憶體靜默丟資料

6.0.x 的 bridge 載 **jazzy FastDDS 2.14.6**;若同機 ROS 2 是 **humble(2.6.11)**,在 `--ipc=host` 下
**共享記憶體的 user data 佈局不相容**:

- discovery 正常 → `ros2 topic list` / `node list` 都看得到;
- **資料面靜默全丟** → 訂閱端永遠收到空的;
- **沒有任何錯誤訊息**(這正是 §3 的具體案例)。

症狀常被誤診成「OmniGraph 不 tick」。**修法一行**:給 ROS 2 那側掛 UDP-only 的 FastDDS profile
(`FASTRTPS_DEFAULT_PROFILES_FILE`),Isaac 端零改動。

### 5.2 選版本的順序

**先確認機器的 GPU 驅動版本,再選 Isaac Sim 版本。** 雲端 GPU instance 的 AMI 常預裝最新驅動;
降驅動(DKMS 重編 kernel module)風險高於換 Isaac Sim 版本(只是換 container tag)。
遇到不相容 → **優先升 Isaac Sim**。

### 5.3 5.1 → 6.0 遷移 checklist

1. 依上表改 import 與關節控制呼叫;
2. 場景載入後**逐項驗證**:碰撞近似型別、物理材質綁定、關節 drive 增益、TF 是否更新;
3. 不要假設「同一份 USD 在新版行為相同」 —— 先 dump 現況再比對(§4.3);
4. DDS 版本檢查(§5.1);
5. 記憶體:大場景在 6.0 有 OOM 風險,遷移前先量 VRAM 基線。

### 5.4 ROS2 TF graph:6.0 必壞的一層

6.0 把 TF 的 prim 解析從 `ROS2PublishTransformTree` 拆到新的 `IsaacComputeTransformTree`
(`isaacsim.core.nodes`),舊的 `targetPrims` 直連被標 deprecated。
**deprecated 的表現形式是靜默產出空 topic + 以 30 Hz 洗版的 warning**,不是啟動失敗:

```
[Warning] [isaacsim.ros2.nodes] [PoseTree] target getObjectType eInvalid for '<prim>'
```

實測同一份 USD:5.1 上這個 warning **0 次**,6.0.1 上 **20,532,664 次**(約 125 MB/小時)。

**判定手法**:同輸入、換版本、數同一個訊號出現幾次。這種定量對照比任何推論都有說服力,
五分鐘就能拿到,而且能一次區分「版本回歸」與「設定寫錯」。

**修法**:在驅動腳本裡 runtime 重建 graph(不改 USD),版本判斷用
「該節點型別**註冊得到與否**」而非版本字串 —— 這樣同一份腳本兩台通吃。

---

## 5.5 跨主機移植:失效多半在接縫上,不在被移植的東西裡

一次 5.1 → 6.0.1 的實戰移植裡,真正卡住整合的**沒有一個屬於 Isaac 本身**。
更麻煩的是它們全部符合同一個模式:**載入成功、狀態正常、就是不會動。**

### 5.5.1 USD 內嵌的絕對路徑 reference

reference 斷鏈**不會中斷載入、不會報錯**,只讓該 prim 變成 typeless
(`GetTypeName()` 回 `''`)、子節點數 0 —— 沒有 mesh、沒有剛體。

實例:場景五顆棧板裡四顆寫死來源機的絕對路徑 `file:/home/<user>/<dir>/pallet.usd`,
搬到新主機全滅,只有用相對路徑的那顆倖存。症狀表現成「叉車搬不動貨」。

**移植前必做的兩個檢查(五分鐘)**:

```bash
strings scene.usd | grep -oE '[A-Za-z0-9_./-]*\.usd[a-z]?' | sort -u
python3 -c "from pxr import Usd; Usd.Stage.Open('scene.usd')" 2>&1 | grep "Could not open"
```

容器內若 `from pxr import Usd` 失敗(6.0 的 `python.sh` 不帶 kit 的 PYTHONPATH),
改用 `/isaac-sim/kit/python/bin/python3` 並把 `PYTHONPATH`/`LD_LIBRARY_PATH`
指到 `extscache/omni.usd.libs-*`。

**修法用 `Sdf.Layer` 層級改寫**(不需解析,所以不受斷鏈影響):

```python
dst = Sdf.Layer.CreateNew(DST); dst.TransferContent(Sdf.Layer.FindOrOpen(SRC))
dst.GetPrimAtPath(p).referenceList.prependedItems = [Sdf.Reference('../pallet.usd')]
dst.Save()
```

> 打包工具(Omniverse Collector)只收它認得的相依,**手動加的 reference 不會被收進去**——
> 所以「Collected 包看起來自足」是假象。而且來源機上那條絕對路徑永遠是對的,
> **在原機做任何驗證都測不出來**。

### 5.5.2 素材目錄權限 vs 容器 uid

貼圖目錄是 `drwxrwx---`(770,owner uid 1000),Isaac 容器跑在 `uid=1234` → 讀不到。
主場景檔是 644 所以載得進去,於是「載入成功」,只是材質缺一半。`chmod -R o+rX` 解決。

`tar` 會保留權限位;來源機上 owner 對得起來所以沒事,落到容器裡 uid 不同就全擋。
**跨主機搬資產,權限跟檔案一樣要驗。**

### 5.5.3 時間基準衝突:兩個 `/tf` 發布者

```
/tf Publisher count: 2
  → Isaac 的 ROS2PublishTransformTree(發 sim time)
  → 某支 Python bridge     (發 wall time)
```

訂閱端若 `use_sim_time=False` 且沒人發 `/clock`,wall time 那份先進 TF buffer,
sim time 那份就成了「遠古資料」被整批丟棄:

```
Warning: TF_OLD_DATA ignoring data from the past for frame base_link at time 339.750000
```

下游拿不到任何有效位姿 → 判定無法執行 → 任務被裝置端取消(看起來像「車壞了」)。

**先數發布者再查內容**:`ros2 topic info /tf -v`。頻率正常、topic 有資料,
不代表資料**能用** —— 兩個發布者打架時 `topic hz` 反而更高。

### 5.5.4 自動化腳本的 fallback 會把你打回舊軌

殘留節點常常不是「沒殺乾淨」,而是**被某個 watchdog / listener 重新拉起來的**。
判別:看進程的啟動時間 —— 若晚於你的清理動作,就是有東西在拉它。

實例:一支 MQTT listener 用「讀場景檔找標記字串」決定重啟哪條軌的 ROS 節點,
新軌沒登記標記 → fallback 到舊軌 → 把舊 bridge 拉回來污染 `/tf`。
而它的原始碼註解**早就寫了**「若之後基線改變要同步更新」。

> **註解裡的警語不是機制。** 要嘛讓它自動偵測,要嘛讓它在偵測不到時**拒絕動作**,
> 而不是 fallback 到某個當時看起來合理的猜測。

### 5.5.5 `pkill -f` 在 `docker exec` 裡會自殺

`docker exec <c> bash -c "pkill -9 -f my_node; echo done"` —— `pkill` 的 pattern
會匹配到**自己那行 shell 命令**,先殺掉自己,後面的 `echo` 從來不會執行
(這也是判別徵兆:預期的輸出憑空消失)。用明確 PID `kill -9`,或 `[m]y_node` 括號技巧。

### 5.5.6 移植的驗收條件

**不能是「有沒有錯誤訊息」。** 上面五個問題全部「沒有錯誤訊息」。
唯一有效的驗收是**端到端跑一趟真實任務**,並且用物理量而非帳面狀態判定:

| 驗收項 | 量法 | 判準 |
|---|---|---|
| 抓握 | 貨物相對取放機構的距離,全程取樣 | 恆定(轉彎中變化 < 2 cm)= 咬住沒滑 |
| 不飛 | 放下後連續取樣位置 | 多個取樣點完全不變、高度歸零 |
| 落點 | 終點座標 vs 目標端點座標 | 平面誤差在容差內 |
| 帳面一致 | 業務 DB 的貨物位置 | 與物理位置指向同一個儲位 |

**在調物理參數之前,先確認你要調的東西真的存在、而且真的有被驅動。**
那次移植若一開始就去調摩擦係數,會在一個根本沒有剛體的 prim 上調到天亮。

---

## 5.6 6.0 的第零步:先確認 active 物理後端

6.0 起 PhysX 不再是唯一選擇。**在調任何接觸參數之前,先確定跑的是哪個引擎** —— 兩者的求解器不同,
參數語意與調參經驗不能互換。

判定機制:`isaacsim.core.simulation_manager` 的 `default_engine` 預設 `physx`,**但**
`isaacsim.physics.newton` 的 `auto_switch_on_startup` 預設 `true` —— 只要該 extension 被啟用,
啟動時就會把 active engine 搶成 Newton,`default_engine` 寫什麼都沒用。

```
app 有沒有 enable isaacsim.physics.newton ?
├─ 沒有 → PhysX
└─ 有 → auto_switch_on_startup(預設 true)→ Newton
```

**判定三法**(由弱到強):

| 方法 | 怎麼做 | 注意 |
|---|---|---|
| 讀 app 檔 | grep `/isaac-sim/apps/<你用的>.kit` 有無 `isaacsim.physics.newton` | 最直接的靜態證據 |
| 看 log | 找 `[ext: isaacsim.physics.newton-…] startup` | **見下方陷阱** |
| 執行期查詢 | `SimulationManager.get_active_physics_engine()` | 最權威 |

⚠ **陷阱:log 出現 newton 字樣不代表 Newton 在跑。** 一台實測跑 PhysX 的 6.0.1:

```
[ext: isaacsim.pip.newton-0.6.1] startup          ← pip 套件,標準安裝就有
[ext: omni.usd.schema.newton-1.2.1] startup       ← USD schema,認得 token 而已
[ext: omni.physx-110.1.13] startup                ← 真正在跑的
```

關鍵是**有沒有** `isaacsim.physics.newton`(引擎本體)。只有它 startup 才代表 Newton 參與模擬。

**選哪個**:官方對「來自 5.x 的既有 PhysX 場景」的建議是**留在 PhysX**;Newton 的定位是 RL 大規模平行、
可微分模擬、軟體/布料。做版本遷移的情境,通常不該切過去。

⚠ 另一個組合風險:**6.0 的 URDF/MJCF 匯入器預設套 Newton schema**
(`NewtonArticulationRootAPI` 取代 `PhysxArticulationAPI`、`NewtonMimicAPI` 取代 `PhysxMimicJointAPI`),
而 `MassAPI` 改成**只在設定非預設密度時才授權**。若資產帶 Newton schema 卻跑在 PhysX 上,
那些設定會被**靜默丟棄**(schema extension 有載入,所以讀得進去、不報錯)。
**跨版本搬場景時,質量屬於「要重新量、不能假設沿用」的那一類。**

---

## 6. 快速體檢清單

東西不動 / 沒被搬走 / 數字很怪,照這個順序問:

0. **(6.0 起)active 物理後端是哪個** — PhysX 還是 Newton?決定後面所有參數語意(§5.6)。
1. **進程與 log** — 元件在不在?log 最後寫入距今幾秒?(從 `/proc/<pid>/fd/1` 反查真檔,**不要猜檔名**)
2. **載入的是哪份場景** — `/proc/<pid>/maps` 反查 + 對 md5。這是唯一可信的答案。
3. **位姿來源** — 拿到的是**即時值**還是 authored fallback?值合理嗎(§3)?
4. **物理 API** — RigidBody / Collision / Mass 在不在?
5. **碰撞近似** — 是不是 `convexHull`?(§2)
6. **材質綁定** — 材質存在 ≠ 綁上了;看 `ComputeBoundMaterial("physics")` 的解算結果。
7. **中介層** — TF 有沒有更新?控制器有沒有送出軌跡?
8. **雙軌對照** — 物理座標 vs 業務 DB(§4.1)。

前七項全綠但東西還是沒動 → 懷疑傳輸層靜默丟資料(§5.1)。

**剛換過主機或版本**,再補三項(§5.5):

9. **東西還在嗎** — dump prim 數與剛體清單,對照來源機。斷鏈的 reference 會讓 prim
   靜默變成 typeless 空殼;**md5 相同只證明檔案相同,不證明場景相同**。
10. **有幾個發布者** — `ros2 topic info /tf -v`。兩個發布者用不同時間基準比沒有發布者更難查。
    ⚠ **問「有幾個」要問 topic,不要數進程**:`pgrep -x` 與 `ps -e -o comm` 會把 zombie 一起算進去
    (zombie 的 `comm` 還在、`cmdline` 是空的)。實測一台 151 個同名進程裡 150 個是 zombie。
    非數進程不可時用 `ps -ef | grep <完整 cmdline>`,有疑慮讀 `/proc/<pid>/status` 的 `State`。
11. **殘留是誰拉起來的** — 看進程啟動時間;晚於你的清理動作 = 有 watchdog/listener 在拉。

---

## When NOT to apply

- 只跑既有場景、不碰物理參數也不除錯 → 照 runbook 走。
- 有 GUI 且問題肉眼可見 → 先看畫面,不必先架探針。
- 問題明確在業務層(排程、派工、API)且物理已驗過 → 這裡的方法幫不上。
- 純運動學 / 路徑規劃(不涉接觸)→ §1–§2 不適用。

---

## 延伸閱讀

- **完整推導 + 圖解**:公開教學 repo [`wicanr2/issac-sim-study`](https://github.com/wicanr2/issac-sim-study)
  —— 第 13 篇「接觸與抓握的第一性原理」(摩擦錐、凸包 vs SDF 兩張 SVG)、
  09 篇(timestep/CCD/PD/solver)、10 篇(材質綁定、質量比)、11 篇(讀位姿與放置精度)、
  08 篇(5.1→6.0 遷移風險)。
- **專案實例(按需讀)**:[`references/circ-ai-playbook.md`](references/circ-ai-playbook.md) ——
  headless Isaac 5.1 的 UDP 探針命令表、連續取樣腳本、漂移門檻的取值理由、A/B 紀律。
  **只有在需要可直接照抄的實作時才讀。**
