---
name: isaac-sim-60
description: >
  Isaac Sim 6.0.x 特有的行為、陷阱與操作。涵蓋:物理後端判定(PhysX vs Newton,
  auto_switch_on_startup 預設 true 會搶成 Newton,但 log 有 newton 不代表 Newton 在跑);
  PhysX 107→110 換代唯一改變的 schema 預設值(physxJoint:maxJointVelocity 從 1e6 變 inf,
  只作用於 articulation —— 用關節去推/夾/叉東西的場景升上 6.0 後暴衝,先查這個);
  ROS 2 Bridge 拆成五個 extension 但設定鍵命名空間沒跟著搬;三個標 Deprecated 並指向
  Newton 的物理屬性(timeStepsPerSecond / contactOffset / restOffset);物理參數的四個
  無聲失效條件;容器內沒有 usdcat 時怎麼讀寫 USD。觸發:「Isaac Sim 6.0 / 6.0.1」
  「5.1 升 6.0 之後行為變了」「PhysX 還是 Newton / 後端怎麼選」「6.0 的 ROS2 bridge」
  「參數設了沒作用」「USD crate 怎麼改 / 沒有 usdcat」「MassAPI / Newton schema 靜默失效」。
  版本無關的接觸力學與調參順序見 isaac-sim-physical-ai skill,兩者互補。
---

# Isaac Sim 6.0.x 的特有行為

本 skill 只收 **6.0 相對 5.x 真正不一樣的地方**,以及據此該改變的判斷。
版本無關的第一性原理(為什麼幾何先於摩擦、為什麼模擬器不報錯)在 `isaac-sim-physical-ai`。

所有預設值與引文取自 Isaac Sim 6.0.1 實機(`omni.usd.schema.physx-110.1.13`)與
`isaac-sim/IsaacSim` GitHub 的 tag 快照,對照組是 5.1 native(`omni.physx-107.3.26`)。

---

## 0. 第零步:確認 active 物理後端

**在調任何物理參數之前做這件事。** 6.0 起 PhysX 不再是唯一後端,兩者參數語意不同。

```
app 有沒有 enable isaacsim.physics.newton ?
├─ 沒有 → PhysX
└─ 有 → auto_switch_on_startup(預設 true)→ Newton
         即使 default_engine 仍寫 physx
```

判定三法(由弱到強):

| 方法 | 做法 |
|---|---|
| 讀 app 檔 | grep `/isaac-sim/apps/<你用的>.kit` 有無 `isaacsim.physics.newton` |
| 看 log | 找 `[ext: isaacsim.physics.newton-…] startup` |
| 執行期 | `SimulationManager.get_active_physics_engine()` |

⚠ **log 出現 newton 字樣不代表 Newton 在跑。** 一台實測跑 PhysX 的 6.0.1:

```
[ext: isaacsim.pip.newton-0.6.1] startup          ← pip 套件,標準安裝就有
[ext: omni.usd.schema.newton-1.2.1] startup       ← USD schema,認得 token 而已
[ext: omni.physx-110.1.13] startup                ← 真正在跑的
```

關鍵是**有沒有** `isaacsim.physics.newton`(引擎本體)。

6.0.1 實機實測:`isaacsim.exp.full.streaming.kit` 與 `isaacsim.exp.full.kit` **都沒有** newton
條目 → PhysX;只有 `isaacsim.exp.full.newton.kit` 有(含 `auto_switch_on_startup = true`)。

> ⚠ 官方 `skills/physics-simulation/SKILL.md` 寫「the standard `isaacsim.exp.full.kit` does
> [enable newton]」,但 v6.0.1 與 main 的實檔都沒有。**官方敘述與官方 repo 實檔是兩個獨立
> 真值來源,會互相矛盾** —— 決定性問題要讀自己機器上那支 app 檔。

**選哪個**:官方對「來自 5.x 的既有 PhysX 場景」建議**留在 PhysX**。Newton 的定位是大規模
RL、可微分模擬、軟體/布料。做版本遷移通常不該切過去。

---

## 1. PhysX 107 → 110:唯一改變的 schema 預設值

逐項比對兩版 `PhysxSchema/generatedSchema.usda`(107.3.26 共 235 條、110.1.13 共 190 條),
**同名屬性的預設值只有一項不同**:

```
physxJoint:maxJointVelocity      107.3.26: 1000000        110.1.13: inf
```

官方 doc:

> Maximum joint velocity. **Only applies to joints that are part of an articulation**
> (see PhysicsArticulationRootAPI).

**5.x 時代所有 articulation 關節都有一道 1e6 速度上限,6.0 拿掉了。** 求解器在接觸不穩定時
算出的異常關節速度,5.x 會被夾住,6.0 原樣生效。

**症狀**:機械臂/夾爪/叉齒這類**用 articulation 去接觸別的物體**的場景,升上 6.0 後被接觸
的物體突然飛出去,而且沒有任何錯誤訊息。

**處置**:顯式設回有限值,其餘不動,跑同一段模擬對照。成本極低、機制明確。

旁證 —— 110 同時**新增**兩個場景層屬性,其中一個是同一組機制:

```
physxScene:solveArticulationContactLast    doc 明確提到 "articulation joint
                                           maximum velocity constraints" 的求解順序
physxScene:disableSleeping
```

**這一區在 107 → 110 之間被整體動過,不是單一數值微調。**

> ⚠ 比對範圍限定 **USD schema 曝露的預設值**。PhysX SDK 內部不透過 schema 曝露的預設值
> 沒有涵蓋,C++ 層仍可能有其他變更。

另外 110 相對 107 **移除 45 條**屬性,全在 `physxDeformable*` / `physxParticle*` /
`physxAutoParticleCloth*` / `physxAutoAttachment*` —— 對應官方 release notes 的
「粒子與可變形體整組移除」,剛體場景不受影響。

---

## 2. 物理參數的四個無聲失效條件

參數設了沒作用,幾乎都是這四條之一。**四條都不報錯。**

| # | 條件 | 檢查 |
|---|---|---|
| 1 | **貼在缺少對應 API 的 prim 上**(孤兒設定) | 設碰撞參數前確認同一 prim 有 `CollisionAPI`;設剛體參數前確認有 `RigidBodyAPI` |
| 2 | **後端不吃** | `physx*` 由 PhysX 讀、`newton:*` 由 Newton 讀,設錯會被靜默忽略 |
| 3 | **被 runtime patch 覆蓋** | 場景檔的 authored 值 ≠ 跑起來的有效值,啟動腳本可能改過 |
| 4 | **combine mode 稀釋** | 摩擦/彈性是兩側材質合成,mode 決定怎麼合 |

### 2.1 三個標了 Deprecated 並指向 Newton 的物理屬性

```
physxScene:timeStepsPerSecond   → "Deprecated: use newton:timeStepsPerSecond"
physxCollision:contactOffset    → "Deprecated: use newton:contactGap"      預設 -inf
physxCollision:restOffset       → "Deprecated: use newton:contactMargin"   預設 -inf
```

這三個恰好是搬運場景最常設的。**合理推斷是「PhysX 下照舊有效,Newton 下改用 `newton:*`」**
—— 屬性仍在 schema、仍有預設值、PhysX 仍是官方支援後端。

⚠ **但這是推斷不是實測。** 追「參數調了沒反應」的問題時,這條要優先排除:設極端值
(如 `timeStepsPerSecond` 30 對 480)看行為有無差異。

> `-inf` 是「未設定,由系統推導」的哨兵值,不是真的負無限大。

### 2.2 combine mode:同一綁定狀態,四種結論

| `frictionCombineMode` | 一側 μ=5.0、另一側預設 0.5 |
|---|---|
| `average`(**schema 預設**) | 2.75 |
| `min` | **0.5 —— 綁了等於沒綁** |
| `max` | **5.0 —— 單側綁定就足夠** |
| `multiply` | 2.5 |

**看到「只有一側綁了物理材質」不要直接下結論。先讀 combine mode。**

另外 `ComputeBoundMaterial("physics")` **幾乎不會回 `None`** —— 找不到物理材質會沿 fallback
回傳**渲染材質**。判準是**回傳型別是不是 PhysicsMaterial**,不是有沒有回傳。

---

## 3. 關鍵預設值速查(110.1.13)

亂飛/不穩相關的:

| 屬性 | 預設 | 註 |
|---|---|---|
| `physxJoint:maxJointVelocity` | **`inf`** | 5.x 是 1e6(§1) |
| `physxRigidBody:maxLinearVelocity` | **`inf`** | 不設限 |
| `physxRigidBody:maxAngularVelocity` | 5729.58 | 度/秒 |
| `physxRigidBody:maxDepenetrationVelocity` | **3** | 初始穿透暴衝的主控點 |
| `physxRigidBody:solverPositionIterationCount` | 16 | |
| `physxRigidBody:solverVelocityIterationCount` | **1** | |
| `physxArticulation:solverPositionIterationCount` | 32 | |
| `physxArticulation:solverVelocityIterationCount` | **1** | 官方對複雜關節建議 **16** |
| `physxScene:timeStepsPerSecond` | **60** | 緊密接觸官方建議 240 |
| `physxScene:enableCCD` | 0 | |
| `physxScene:enableStabilization` | 0 | 開啟會破壞自由旋轉物體的角動量 |
| `physxScene:solverType` | `TGS` | |
| `physxSDFMeshCollision:sdfResolution` | **256** | **體素間距 = 最長 AABB 邊 ÷ 此值** |
| `physxMaterial:frictionCombineMode` | `average` | |

**SDF 解析度的算法**(schema 原文:"The spacing of the uniformly sampled SDF is equal to the
largest AABB extent of the mesh, divided by the resolution"):

```
體素邊長   = 最長 AABB 邊 ÷ resolution
反推需求   resolution = 最長邊 ÷ (最小特徵尺寸 ÷ 想要的取樣點數)
```

官方:「too low SDF resolution can lead to situations where very thin parts of the mesh
don't collide」。用了 `sdf` 不等於孔就表達得出來,還要看解析度。

---

## 4. ROS 2 Bridge 在 6.0 的三個判讀陷阱

介面幾乎完全向後相容,**要留意的不是「什麼壞了」而是「什麼看起來變了其實沒變」**。

| 陷阱 | 真相 |
|---|---|
| bridge 拆成五個 extension,是不是要改 `--enable`? | **不用**。`bridge` 變 umbrella,依賴會遞迴帶起 core/nodes/examples/ui |
| 設定宣告搬到 `core` 的 toml,鍵名是不是也改了? | **沒改**,仍是 `exts."isaacsim.ros2.bridge".*`。宣告位置 ≠ 命名空間 |
| bridge 的 `[package] version` 是 5.1.2,這是 5.1 的東西? | **不是**。extension 版號與產品版號脫鉤,5.1.2 是 6.0.1 隨附的 bridge |

**rclpy 載入順序**:先試系統的、失敗才用內建的。

```
Attempting to load system rclpy
Could not import system rclpy: No module named 'rclpy'
Attempting to load internal rclpy for ROS Distro: humble
rclpy loaded
```

這解釋了「**啟動前不要 `source /opt/ros/<distro>`**」:一旦 source,第一步會成功,載進為
系統 Python 編譯的 C extension,ABI 不匹配且不報乾淨的錯。**看到 `Attempting to load
internal rclpy` 才是走內建。**

**節點用法的 deprecation**(節點型別仍在,場景不會壞,只是 warning):

| 資料 | 5.1 | 6.0 建議 |
|---|---|---|
| TF | `ROS2PublishTransformTree.targetPrims` | `isaacsim.core.nodes.IsaacComputeTransformTree` → publisher |
| JointState | `ROS2PublishJointState.targetPrim` | `Isaac Read Joint State` → publisher |

改接過程中會出現 `[Error] Please specify at least one valid target prim for the ROS pose
tree component` —— 那是清空 `targetPrims` 之後、compute 節點接上之前的**中間狀態**,不是故障。

**驗收**:只有 `ros2 topic hz /tf` 量到穩定頻率才是資料面通了。extension startup 只證明載入了。

---

## 5. 6.0 的其他 breaking change

- **`MassAPI` 授權規則改了**:只在設定了非預設密度時才套用。同一匯入流程在兩版產出的
  質量可能不同,**且不報警**。跨版本搬場景時,質量屬於「要重新量、不能假設沿用」。
- **匯入器預設套 Newton schema**:`NewtonArticulationRootAPI` 取代 `PhysxArticulationAPI`、
  `NewtonMimicAPI` 取代 `PhysxMimicJointAPI`。**帶 Newton schema 卻跑 PhysX 後端 = 靜默丟棄。**
- **`isaacsim.core.api` / `.prims` / `.utils` 搬到 `source/deprecated/`**,仍可用。
  **目錄結構的增減不能直接讀成功能存廢。**
- **`isaacsim.sensors.physx*` deprecated** → `isaacsim.sensors.experimental.physics`。

---

## 6. 容器內沒有 usdcat:怎麼讀寫 USD

實測 `find /isaac-sim -name "usdcat*"` 回空,而 `pxr` 藏在 extension 快取裡。要接起來
**兩個環境變數都要設**:

```bash
USDLIB=$(ls -d /isaac-sim/extscache/omni.usd.libs-*/ | head -1)
export PYTHONPATH=${USDLIB}:$PYTHONPATH
export LD_LIBRARY_PATH=${USDLIB}bin:$LD_LIBRARY_PATH    # 少這行 → ImportError: libusd_tf.so
/isaac-sim/kit/python/bin/python3 -c "from pxr import Usd; print('pxr OK')"
```

讀大場景:

```python
stage = Usd.Stage.Open(path, load=Usd.Stage.LoadNone)   # 只讀結構,不載幾何
prim = stage.GetPrimAtPath("/target_pallet"); prim.Load()
attr.HasAuthoredValue()      # 區分「刻意設定」與「吃 schema 預設」——這個區分很關鍵
```

⚠ **絕不要對大場景用 `stage.Flatten()`**:126 MB 的 crate 攤平後是數 GB 文字。要匯出局部
用 `Sdf.CopySpec` 只複製子樹。

`.usd` 檔頭是 `PXR-USDC` = crate 二進位,編輯器打不開;`.usda` 是文字。

**改場景檔還是改啟動腳本**:結構性錯誤(貼紙貼錯層)在源頭修;調參性質(解析度、offset、
摩擦)用啟動腳本 `--exec` 在載入後套用 —— 原檔不動、可版控、同一份場景能餵給不同版本的機器。

---

## 7. 診斷順序

```
0. 確認 active 後端(§0)                        ← 決定以下全部是否適用
1. 結構檢查(離線讀檔,不必跑模擬)
   ├─ RigidBody 與 Collision 在同一層嗎
   ├─ 質量寫在剛體層還是子層,值合理嗎
   ├─ 材質綁定回傳的型別是 PhysicsMaterial 嗎
   └─ 有沒有孤兒設定(參數貼在缺 API 的 prim 上)
2. 幾何檢查
   ├─ 初始有沒有重疊
   └─ 最小特徵 ÷ 體素邊長 = 幾個取樣點
3. articulation 場景:先查 physxJoint:maxJointVelocity(§1)
4. 這一步之前不要碰摩擦係數
```

**1 和 2 全是靜態檢查,離線讀檔即可,秒級完成。跑一次完整任務要好幾分鐘。能靜態量的就別派任務。**

**證明一個參數真的生效,只有一種方法:設極端值看行為有無可觀察差異。**
grep 得到、讀得出來、探針讀回來,都只證明「值在那裡」。

---

## 參考

- `omni.usd.schema.physx-110.1.13/plugins/PhysxSchema/resources/generatedSchema.usda`
- `isaac-sim/IsaacSim` @ v6.0.1:`source/apps/*.kit`、`source/extensions/isaacsim.ros2.*`、
  `skills/physics-simulation/SKILL.md`
- Isaac Sim 6.0.0 release notes
- 完整推導與圖:`isaac-sim-study` 的 [14](../../docs/14-ros2-bridge-6.0-architecture/README.md)、
  [15](../../docs/15-physics-backend-5.1-to-6.0/README.md)、
  [16](../../docs/16-model-tuning-for-6.0/README.md)、
  [17](../../docs/17-physics-parameter-tuning-6.0/README.md)
