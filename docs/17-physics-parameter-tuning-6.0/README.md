# 17 · Isaac Sim 6.0 的物理調參:入口、生效條件、與完整參數表

前一篇([16](../16-model-tuning-for-6.0/README.md))講的是**結構**——貼紙貼在哪一層。這一篇講**數值**:6.0 有哪些物理參數可以調、從哪裡調、以及一個參數要滿足什麼條件才會真的生效。

核心問題只有一個,而它在這個系統裡出乎意料地難回答:

> **我設了這個值,它到底有沒有作用?**

底下先講五個生效條件(每一個都可以無聲地讓設定失效),再給完整參數表。

依據來自 Isaac Sim 6.0.1 隨附的 `PhysxSchema` 定義檔實測(`omni.usd.schema.physx-110.1.13`),路徑與引文都標了。

延伸閱讀:[16 模型結構調校](../16-model-tuning-for-6.0/README.md)、[15 物理層變動](../15-physics-backend-5.1-to-6.0/README.md)、[09 物理模擬基礎](../09-physics-simulation-fundamentals/README.md)。

---

## 1. 三個調參入口

同一個參數可能有三種設法,作用時機與持久性都不同。

| 入口 | 形式 | 何時生效 | 存在哪 |
|---|---|---|---|
| **USD 屬性** | `physxRigidBody:maxLinearVelocity = 30` | 場景載入時 | 場景檔(持久) |
| **啟動參數(carb setting)** | `--/persistent/physics/...=X` | 程序啟動時 | 命令列(不持久) |
| **Runtime API** | `PhysxSchema.PhysxRigidBodyAPI.Apply(prim)` 後設屬性 | 呼叫當下 | 記憶體(重載即失) |

**物理參數的絕大多數是第一種。** 第二種主要管全域行為與 UI 預設,第三種是啟動腳本 patch 的做法(見 [16 篇 §3.2](../16-model-tuning-for-6.0/README.md))。

Runtime 改的典型寫法:

```python
from pxr import PhysxSchema, UsdPhysics

prim = stage.GetPrimAtPath("/target_pallet/target_pallet")
api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)     # 沒有就貼上,有就取得
api.CreateMaxLinearVelocityAttr().Set(30.0)
```

⚠ `Apply()` 是冪等的,可以重複呼叫;`CreateXxxAttr()` 在屬性不存在時建立、存在時取得。**兩者都不會報「你貼錯 prim 了」。**

---

## 2. 五個生效條件

一個屬性設下去卻沒作用,幾乎都是踩到這四條之一。四條都**不報錯**。

### 2.1 條件一:貼在有對應 API 的 prim 上

`physxSDFMeshCollision:sdfResolution` 只有在該 prim 具備碰撞能力時才有意義。貼在一個沒有 `CollisionAPI` 的 prim 上,它就是個**孤兒設定**:grep 得到、讀得出來、完全不生效。

實例(來自一個真實場景):

```
fork_tilt         [RigidBody, Mass=100]  physxSDFMeshCollision:sdfResolution = 256   ← 孤兒
  └─ fork_tilt_01 [Collision, approx=sdf]                                            ← 碰撞其實在這
```

`fork_tilt` 沒有 `CollisionAPI`,那個 `sdfResolution` 什麼也沒做。真正該設的是子層的 `fork_tilt_01`。

**檢查法**:設任何碰撞相關參數前,先確認同一個 prim 上有 `CollisionAPI`;設剛體相關參數前,先確認有 `RigidBodyAPI`。

### 2.2 條件二:後端要吃這個屬性

6.0 有 PhysX 與 Newton 兩個後端([15 篇](../15-physics-backend-5.1-to-6.0/README.md))。`physx*` 命名空間的屬性由 PhysX 讀,`newton:*` 由 Newton 讀。**設了另一個後端的屬性,會被靜默忽略。**

6.0 的 schema 裡有三個屬性標了 Deprecated 並指向 Newton,而它們恰好是搬運場景最常設的:

```
physxScene:timeStepsPerSecond
  doc = "Deprecated: use newton:timeStepsPerSecond (NewtonSceneAPI) instead."

physxCollision:contactOffset      預設 -inf
  doc = "Deprecated: use newton:contactGap (NewtonCollisionAPI) instead."

physxCollision:restOffset         預設 -inf
  doc = "Deprecated: use newton:contactMargin (NewtonCollisionAPI) instead."
```

> **怎麼解讀這個 Deprecated**:這些屬性在 6.0 仍然存在於 `PhysxSchema` 裡、仍有預設值、PhysX 後端仍是官方支援的後端。**合理推斷是「PhysX 下照舊有效,Newton 下改用 newton:* 對應項」**,而不是「6.0 起一律失效」。
>
> ⚠ **但這是推斷,不是實測。** 要確認,最直接的方式是設一個極端值(例如 `timeStepsPerSecond = 30` 對 `= 480`)跑同一段模擬,看行為有沒有差。**在有這個實測之前,不要把「我設了 timestep」當成已知事實** —— 尤其當你正在追一個「參數調了沒反應」的問題時,這條要優先排除。

### 2.3 條件三:場景檔的值可能被 runtime patch 覆蓋

啟動腳本可以在場景載入後改任何屬性。於是**檔案裡的值與實際跑的值可以不同**。

實例:某場景檔裡叉齒是 `physics:approximation = convexHull`,但跑起來探針讀到的是 `sdf` + `sdfResolution=512` —— 啟動腳本改的。

**兩個值都要查,而且要分清楚**:

| 問題 | 查哪個 |
|---|---|
| 這個場景檔本身怎麼設的 | 離線讀檔的 **authored value** |
| 現在跑的模擬實際用什麼 | 對跑著的程序發**探針**讀即時值 |
| 兩者為什麼不同 | 讀啟動腳本 |

只查檔案會漏掉 patch;只查 runtime 會分不清「原本就這樣」和「腳本改的」。

### 2.5 條件五:**runtime 寫入不一定會被採用,而且剛體與關節不一樣**

§2.3 說「啟動腳本可以在場景載入後改任何屬性」。**那句要加一個但書:寫得進去,不代表 PhysX 會收下。**

實測(Isaac Sim 6.0.1-rc.7 / PhysX 110.1.13,2026-08-01):

| 對象 | runtime 寫入 | 回讀 | PhysX 實際採用 |
|---|---|---|---|
| **剛體屬性**(`physxRigidBody:maxLinearVelocity` 等) | 成功 | 成功 | ❌ **不採用** |
| **關節 drive**(`drive:linear:physics:maxForce`) | 成功 | 成功 | ✅ **採用** |

兩者走的是**不同路徑**,不要互相推論。剛體那組的 runtime 寫入只碰到 USD 層,
沒有進到模擬;關節 drive 則會即時反映在行為上。

#### 怎麼證明:彈道正對照

「回讀成功」完全不是證據(§5 已經說過)。要證明一次 runtime 寫入有沒有被採用,
用一個**方向明確、只要生效就必然改變畫面**的量:

```
給一個靜止的剛體 physics:velocity = (0, 0, 10)
若被採用 → 1.5 秒後應上升約 4 公尺(扣掉重力)
實測 Δz = +0.0000  →  沒被採用
```

這個方法可重用:**挑一個「生效就會大幅改變、不生效就完全不動」的量**,
而不是去比較兩個都很小的數字。

對照組同樣要做:關節 drive 的 `maxForce` 用門檻掃描
(1000 → 2000,行為在中間翻轉)證明那條路徑是通的。
**只驗失敗的那一側,分不出「這個旋鈕沒用」與「我的寫入方式沒用」。**

#### 後果

要真的改剛體屬性只有兩條路,**都不是 runtime**:

1. 寫進 USD 資產(改場景檔)
2. 在啟動腳本裡用 Isaac 的 API 設定(需重啟)

⚠ 這條的代價很具體:曾經有 40 輪實驗把「剛體速度上限」當成一個變因在比較,
而那個變因**從頭到尾沒有真的施加過**。四小時的資料全部無效,
而且過程中沒有任何錯誤訊息。

### 2.4 條件四:combine mode —— 兩側材質怎麼合成

摩擦與彈性是**兩個接觸面**材質合成的結果,合成方式由 combine mode 決定。**同一個綁定狀態,不同 mode 給出完全不同的有效值。**

| `frictionCombineMode` | 合成 | 一側 μ=5.0、另一側預設 0.5 |
|---|---|---|
| `average`(**schema 預設**) | 平均 | 2.75 |
| `min` | 取小 | **0.5 —— 綁了等於沒綁** |
| `max` | 取大 | **5.0 —— 單側綁定就足夠** |
| `multiply` | 相乘 | 2.5 |

`restitutionCombineMode`、`dampingCombineMode` 同理,預設都是 `average`。

**所以「只有一側綁了物理材質」這件事本身不足以下任何結論。** 必須先讀 combine mode。

---

## 3. 完整參數表(Isaac Sim 6.0.1 / PhysX 110.1.13)

只列與剛體搬運相關的類別。預設值全部來自實機的 `generatedSchema.usda`。

### 3.1 `PhysxSceneAPI` — 場景層(貼在 PhysicsScene 上)

最常調的:

| 屬性 | 預設 | 說明 |
|---|---|---|
| `timeStepsPerSecond` | **60** | 物理步頻。一般剛體 60–120、緊密接觸建議 240 ⚠ 標 Deprecated(§2.2) |
| `solverType` | `TGS` | `TGS` / `PGS`。TGS 對長關節鏈更穩 |
| `enableCCD` | **0** | 連續碰撞偵測,防高速穿透 |
| `enableStabilization` | **0** | 額外穩定化;**會破壞自由旋轉物體的角動量** |
| `enableGPUDynamics` | 1 | GPU 動力學管線 |
| `broadphaseType` | `GPU` | 粗篩演算法 |
| `collisionSystem` | `PCM` | 碰撞偵測系統 |
| `bounceThreshold` | 0 | 相對速度低於此值不彈跳 |
| `frictionOffsetThreshold` | 0.04 | 接觸分離距離超過此值不算摩擦 |
| `frictionCorrelationDistance` | 0.025 | 接觸點合併成 patch 的距離 |
| `frictionType` | `patch` | 摩擦模型 |
| `maxBiasCoefficient` | `inf` | 約束求解器的最大偏置係數 |
| `minPositionIterationCount` / `max` | 1 / 255 | 全場景 position 疊代上下限 |
| `minVelocityIterationCount` / `max` | 0 / 255 | 全場景 velocity 疊代上下限 |
| `enableExternalForcesEveryIteration` | 0 | 提高 TGS 穩定性 |
| `solveArticulationContactLast` | **0** | **110 新增**:把 articulation 接觸與關節最大速度約束排到最後解 |
| `disableSleeping` | **0** | **110 新增**:全場景禁止睡眠 |
| `enableEnhancedDeterminism` | 0 | 以效能換確定性 |

GPU 容量類(場景大時才需要動):`gpuCollisionStackSize` 64MB、`gpuHeapCapacity` 64MB、`gpuMaxRigidContactCount` 524288、`gpuMaxRigidPatchCount` 81920、`gpuFoundLostPairsCapacity` 262144、`gpuFoundLostAggregatePairsCapacity` 1024、`gpuTotalAggregatePairsCapacity` 1024、`gpuTempBufferCapacity` 16MB、`gpuMaxNumPartitions` 8。

### 3.2 `PhysxRigidBodyAPI` — 剛體(貼在有 RigidBodyAPI 的 prim 上)

**「東西亂飛」的主要控制面在這一區。**

| 屬性 | 預設 | 說明 |
|---|---|---|
| `maxLinearVelocity` | **`inf`** | 線速度上限。**預設不設限** |
| `maxAngularVelocity` | **5729.58** | 角速度上限(度/秒),約 16 轉/秒 |
| `maxDepenetrationVelocity` | **3** | solver 推開穿透時的最大速度。**初始穿透造成暴衝的主要控制點** |
| `maxContactImpulse` | `inf` | 單一接觸可施加的衝量上限 |
| `linearDamping` | 0 | 線性阻尼 |
| `angularDamping` | 0.05 | 角阻尼 |
| `solverPositionIterationCount` | **16** | 該剛體的 position 疊代 |
| `solverVelocityIterationCount` | **1** | 該剛體的 velocity 疊代 |
| `enableCCD` | 0 | 該剛體的掃掠積分 |
| `enableSpeculativeCCD` | 0 | 依速度動態調整 contact offset |
| `enableGyroscopicForces` | 1 | 陀螺力 |
| `sleepThreshold` | 5e-5 | 質量正規化動能低於此值可睡眠 |
| `stabilizationThreshold` | 1e-5 | 低於此值參與穩定化 |
| `cfmScale` | 0.025 | 弱化約束響應,可穩定 articulation |
| `contactSlopCoefficient` | 0 | 接觸角度容差,改善滾動行為 |
| `disableGravity` | 0 | 關閉重力 |
| `retainAccelerations` | 0 | 跨幀保留力/加速度 |
| `solveContact` | 1 | 是否在 solver 中處理其接觸 |
| `lockedPosAxis` / `lockedRotAxis` | 0 / 0 | 位元旗標,鎖定特定軸的移動/旋轉 |

### 3.3 `PhysxCollisionAPI` — 碰撞

| 屬性 | 預設 | 說明 |
|---|---|---|
| `contactOffset` | **`-inf`** | 開始產生接觸約束的距離 ⚠ Deprecated → `newton:contactGap` |
| `restOffset` | **`-inf`** | 休息間隙,可縮放實際碰撞體積 ⚠ Deprecated → `newton:contactMargin` |
| `torsionalPatchRadius` | 0 | 扭轉摩擦的接觸 patch 半徑 |
| `minTorsionalPatchRadius` | 0 | 同上,下限 |

> `-inf` 是「未設定,用系統推導值」的哨兵值,不是真的負無限大。

### 3.4 `PhysxSDFMeshCollisionAPI` — SDF 碰撞

| 屬性 | 預設 | 說明 |
|---|---|---|
| `sdfResolution` | **256** | **體素間距 = 最長 AABB 邊 ÷ 此值**(見 [16 篇 §4.4](../16-model-tuning-for-6.0/README.md)) |
| `sdfSubgridResolution` | 6 | >0 啟用稀疏 SDF;0 為稠密 |
| `sdfBitsPerSubgridPixel` | `BitsPerPixel16` | 8 / 16 / 32,降低可省記憶體但損精度 |
| `sdfMargin` | 0.01 | SDF 相對 bbox 對角線的擴張比例 |
| `sdfNarrowBandThickness` | 0.01 | 表面附近高解析取樣的帶寬 |
| `sdfEnableRemeshing` | 0 | 計算前先重新網格化 |
| `sdfTriangleCountReductionFactor` | 1 | 保留輸入三角形的比例 |

### 3.5 `PhysxMaterialAPI` — 物理材質

| 屬性 | 預設 | 說明 |
|---|---|---|
| `frictionCombineMode` | **`average`** | 兩側摩擦合成方式(§2.4) |
| `restitutionCombineMode` | **`average`** | 彈性合成方式 |
| `dampingCombineMode` | **`average`** | 阻尼合成方式 |
| `compliantContactStiffness` | 0 | 柔順接觸的隱式彈簧剛度 |
| `compliantContactDamping` | 0 | 柔順接觸阻尼 |
| `compliantContactAccelerationSpring` | 0 | 切換成加速度式柔順接觸 |

摩擦值本身是 `UsdPhysics.MaterialAPI` 的 `physics:staticFriction` / `dynamicFriction` / `restitution`。

### 3.6 `PhysxArticulationAPI` / `PhysxJointAPI` — 關節與 articulation

| 屬性 | 預設 | 說明 |
|---|---|---|
| `physxArticulation:solverPositionIterationCount` | **32** | articulation 的 position 疊代(比剛體的 16 高) |
| `physxArticulation:solverVelocityIterationCount` | **1** | velocity 疊代。**官方對複雜關節建議 16,預設遠低於此** |
| `physxArticulation:sleepThreshold` | 5e-5 | |
| `physxArticulation:stabilizationThreshold` | 1e-5 | |
| `physxArticulation:articulationEnabled` | 1 | |
| `physxArticulation:enabledSelfCollisions` | 1 | ⚠ Deprecated → `newton:selfCollisionEnabled` |
| **`physxJoint:maxJointVelocity`** | **`inf`** | **關節速度上限。5.x 時代預設是 1e6,6.0 拿掉了**(見 [15 篇 §4.5](../15-physics-backend-5.1-to-6.0/README.md)) |
| `physxJoint:jointFriction` | 0 | 關節摩擦 |
| `physxJoint:armature` | 0 | 致動器的等效慣性,可穩定高增益驅動 |

---

## 4. 調參順序(接續 13 篇的推導)

物理層面的順序由接觸力學決定,不能交換:

```
幾何(λₙ 存不存在)→ 質量/慣性 → 碰撞近似精度 → offset → 致動 → 摩擦
```

在這之上,**穩定性問題**(抖動、暴衝、下陷)有另一組旋鈕,順序建議:

| 順序 | 症狀 | 先動 |
|---|---|---|
| 1 | 一啟動就飛 | 檢查初始穿透;`maxDepenetrationVelocity`(預設 3) |
| 2 | 接觸後暴衝 | `maxLinearVelocity`(預設 inf)、`maxAngularVelocity`;articulation 場景加 `physxJoint:maxJointVelocity` |
| 3 | 抖動 / 下陷 | 提高 `solverPositionIterationCount`(剛體 16 → 32,articulation 32 → 64) |
| 4 | 高速穿透 | `enableCCD` |
| 5 | 接觸解不穩 | 提高 `timeStepsPerSecond`(120 → 240) |
| 6 | 仍不穩 | `enableStabilization` ⚠ 會破壞自由旋轉物體的角動量 |

**一次只動一個。** 同時改兩項,無論結果好壞都無法歸因。

---

## 5. 怎麼確認一個參數真的生效

按可信度由低到高:

| # | 方法 | 能證明 |
|---|---|---|
| 1 | 在場景檔裡 grep 得到 | 只證明**寫下來了** |
| 2 | 離線讀出 authored value | 證明**值正確且格式沒錯** |
| 3 | 對跑著的模擬用探針讀回 | 證明**載入後仍是這個值**(排除 runtime patch 覆蓋) |
| 4 | **設極端值,行為有可觀察的差異** | **證明它真的參與計算** |

**只有第 4 項能證明生效。** 前三項都可能踩到 §2 的四個條件之一。

第 4 項的操作:選一個**方向明確**的極端值(例如把 `maxLinearVelocity` 從 30 設成 0.1),跑同一段模擬,量同一個數字。有差 → 生效;沒差 → 這個參數對當前情境無作用,或設定沒被採用。

> ⚠ 極端值測試要挑**無副作用**或**易回復**的參數,而且要事先想好回退方式。不要拿會改變狀態的操作當煙霧測試。

---

## 6. 一頁摘要

| 問題 | 答案 |
|---|---|
| 參數從哪設 | USD 屬性(主要)、啟動參數、runtime API |
| 為什麼我設了沒用 | 貼錯 prim(缺對應 API)/ 後端不吃 / 被 runtime patch 覆蓋 / combine mode 稀釋 / **runtime 寫入沒被採用(剛體屬性)** |
| 東西亂飛先動什麼 | `maxDepenetrationVelocity` → `maxLinearVelocity` → articulation 的 `maxJointVelocity` |
| 抖動下陷先動什麼 | solver iteration(剛體預設 16/1、articulation 32/1,都偏低) |
| 緊密接觸場景的 timestep | 官方建議 240,schema 預設只有 60 |
| 單側綁物理材質夠不夠 | **看 `frictionCombineMode`**:`max` 夠、`min` 完全不夠、`average` 打對折 |
| 怎麼證明參數生效 | 設極端值看行為差異。grep 得到、讀得出來都不算 |

---

## 參考

- `omni.usd.schema.physx-110.1.13/plugins/PhysxSchema/resources/generatedSchema.usda` — 本篇所有預設值與 doc 引文的來源,取自 Isaac Sim 6.0.1 實機
- 官方 `skills/physics-simulation/SKILL.md` @ v6.0.1 — Hz 與 solver iteration 的建議表
- [13 接觸與抓握的第一性原理](../13-contact-and-grasp-first-principles/README.md) — 為什麼調參順序是幾何優先
- [15 物理層變動](../15-physics-backend-5.1-to-6.0/README.md) §4.5 — `maxJointVelocity` 在 107 → 110 的變更
