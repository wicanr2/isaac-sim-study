# 15 · 5.1 到 6.0 的物理層變動:PhysX 換代與 Newton 後端的加入

從 Isaac Sim 5.1 升到 6.0,物理層同時發生了兩件獨立的事:**PhysX 自己跨了三個大版號**,以及**多了一個叫 Newton 的後端可以選**。這兩件事常被混談成「6.0 改用 Newton 了」,但它們的影響範圍與觸發條件完全不同——多數場景升上 6.0 之後跑的仍然是 PhysX,而升級帶來的行為差異主要來自 PhysX 換代,不是來自 Newton。

分不清這兩者,排查方向會整個偏掉:以為在調 Newton 的參數,實際跑的是 PhysX;或者反過來,把 PhysX 時代的調參經驗套在一個已經自動切成 Newton 的環境上。

本篇先把「怎麼確定自己這台跑的是哪一個」講清楚,再分別談兩者的變動。

延伸閱讀:[09 物理模擬基礎](../09-physics-simulation-fundamentals/README.md)(timestep、solver、contact/rest offset、CCD)、[13 接觸與抓握的第一性原理](../13-contact-and-grasp-first-principles/README.md)(為什麼調摩擦常是錯的第一步)、[08 5.1 → 6.0.1 遷移風險調查](../08-migration-5.1-to-6.0-oom-risk/README.md)(場景與記憶體面)、[14 ROS 2 Bridge 在 6.0 的架構重組](../14-ros2-bridge-6.0-architecture/README.md)。

---

## 1. 兩件事,不是一件

| | 變動 | 觸發條件 | 影響 |
|---|---|---|---|
| **A** | PhysX `107.3.x` → `110.1.x` | **升級即發生**,無法迴避 | 求解行為、移除的功能、schema 授權規則 |
| **B** | 新增 Newton 後端 | **只在啟用對應 extension 時發生** | 換一整套求解器,PhysX 調參經驗不直接適用 |

實測兩台機器的 `omni.physx` 版本(同一套場景、同一個專案):

```
Isaac Sim 5.1 (native)    omni.physx-107.3.26+107.3.3
Isaac Sim 6.0.1 (容器)     omni.physx-110.1.13
```

`107 → 110` 不是修補版號的推進。**「同一份場景在 5.1 能跑、搬到 6.0 行為變了」這件事,在還沒碰到 Newton 之前就已經有充分的解釋來源了。**

---

## 2. 先確定自己跑的是哪一個後端

這是所有物理排查的第零步。判斷依據按可信度排列。

### 2.1 機制:誰決定 active engine

`isaacsim.core.simulation_manager` 負責註冊物理引擎並挑出 active 的那個。它的 `default_engine` 預設是 `physx`。

但 `isaacsim.physics.newton` 這個 extension 有一個 `auto_switch_on_startup` 設定,**預設為 true**——只要它被啟用,啟動時就會把 active engine 搶成 Newton,即使 `default_engine` 還寫著 `physx`。

於是判斷邏輯是:

```
app 有沒有 enable isaacsim.physics.newton ?
├─ 沒有 → PhysX
└─ 有 → 看它的 auto_switch_on_startup
         ├─ true(預設)→ Newton
         └─ false      → 依 default_engine
```

顯式指定的啟動參數:

```bash
--/exts/isaacsim.core.simulation_manager/default_engine=physx    # 或 =newton
--/exts/isaacsim.physics.newton/auto_switch_on_startup=false     # 關掉自動切換
```

執行期查詢:

```python
from isaacsim.core.simulation_manager import SimulationManager
print(SimulationManager.get_active_physics_engine())

from isaacsim.physics.newton import get_available_physics_engines
print(get_available_physics_engines())
```

### 2.2 靜態判斷:讀 kit app 檔

在 6.0.1 實機上 grep 三支 app,結果分明:

| `/isaac-sim/apps/…` | `isaacsim.physics.newton` | `auto_switch_on_startup` | 後端 |
|---|---|---|---|
| `isaacsim.exp.full.streaming.kit` | 未列 | — | **PhysX** |
| `isaacsim.exp.full.kit` | 未列 | — | **PhysX** |
| `isaacsim.exp.full.newton.kit` | 列了三個 newton extension | `= true` | **Newton** |

```
# isaacsim.exp.full.newton.kit
"isaacsim.physics.newton" = {}
"isaacsim.physics.newton.tensors" = {}
"isaacsim.physics.newton.ui" = {}
exts."isaacsim.physics.newton".auto_switch_on_startup = true
exts."isaacsim.physics.newton".capture_graph_physics_step = true
```

**Newton 在 6.0.1 是要顯式選的**:用 `isaacsim.exp.full.newton.kit` 這支專屬 app,或自己把 extension 加進去。

> ⚠ 官方 `skills/physics-simulation/SKILL.md` 寫「the standard `isaacsim.exp.full.kit` does [enable newton]」。**這句與同一個 tag 底下的實際檔案不符**——`v6.0.1` 與 `main`(VERSION `6.0.1-rc.7`)的 `source/apps/isaacsim.exp.full.kit` 都沒有任何 newton 條目,6.0.1 容器內的實檔也沒有。它可能描述的是更後面的版本規劃。
>
> 教訓不在於哪一方對,而在於**官方 skill / 文件與官方 repo 的實際檔案是兩個獨立的真值來源,會互相矛盾**。判斷「我這台會不會自動切 Newton」這種決定性問題,要讀自己機器上那支 app 檔,不要只讀敘述。

### 2.3 日誌佐證:區分「載入」與「啟用」

啟動 log 裡出現 newton 字樣,不代表 Newton 是 active 的。實測一台**跑 PhysX** 的 6.0.1:

```
[13.182s] [ext: isaacsim.pip.newton-0.6.1] startup          ← pip 套件,只是裝著
[14.129s] [ext: omni.usd.schema.newton-1.2.1] startup       ← USD schema,認得 token 而已
[14.444s] [ext: omni.physx.foundation-110.1.13] startup     ← 真正在跑的是這些
[14.452s] [ext: omni.physx-110.1.13] startup
[15.297s] [ext: omni.physx.tensors-110.1.13] startup
```

關鍵在於**沒有** `[ext: isaacsim.physics.newton-…] startup` 這一行。

- `isaacsim.pip.newton` = Python 套件依賴
- `omni.usd.schema.newton` = USD schema 定義(讓 stage 認得 `Newton*API` token)
- `isaacsim.physics.newton` = **引擎本體,只有這個 startup 才代表 Newton 真的參與模擬**

前兩者在標準 6.0.1 安裝裡本來就會載入。看到 newton 就下結論,是這裡最容易犯的錯。

---

## 3. Newton 是什麼,什麼時候該用

官方定位是 experimental,與 PhysX 並列可切換:

> The isaacsim.physics.newton extension integrates Newton physics simulation into Isaac Sim, providing an alternative physics engine to PhysX with support for advanced solvers including XPBD and MuJoCo backends.
>
> — `source/extensions/isaacsim.physics.newton/docs/Overview.md` @ v6.0.1

它不是一個求解器,而是一組:

| 求解器 | 座標 | 可微分 | 適用 |
|---|---|---|---|
| `SolverFeatherstone` | 廣義座標 | 是 | 關節機器人(機械臂、足式)預設 |
| `SolverMuJoCo` | 廣義座標 | 是 | 對標 MuJoCo 基線、移植 MuJoCo policy |
| `XPBD` | 位置式 | — | 剛體 + 軟體 |
| `VBD` | — | — | 軟體 / 布料 |

官方給的選擇建議裡,有一條直接決定了多數既有專案的走向:

| 需求 | 用 |
|---|---|
| 上千環境的 RL 訓練 | Newton |
| 可微分模擬 | Newton |
| 軟體、布料、可變形 | Newton |
| **來自 Isaac Sim 5.x 的既有 PhysX 場景** | **PhysX** |

— `skills/physics-simulation/SKILL.md` @ v6.0.1

**把 5.x 場景搬上 6.0 的情境,官方建議留在 PhysX。** 這也是為什麼「6.0 = Newton」這個印象在實務上多半不成立:做遷移的人本來就不該切過去。

兩個後端共用 `UsdPhysics.*` 這層標準 schema,許多 `PhysxSchema.*` 屬性在 Newton 下仍被採用,Newton 另外透過 `omni.usd.schema.newton` 讀自己的求解器設定。ROS 2 那一側則不受影響:

> ROS 2 Bridge handles the Newton backend similarly to PhysX. All OmniGraph nodes are compatible with Newton and PhysX.
>
> — Isaac Sim 6.0.0 release notes

---

## 4. PhysX 110 拿掉了什麼

跟 Newton 無關、升級就會遇到的部分。

**移除的功能**——粒子與可變形體整組:布料(cloth)、deformable body、particle physics 從 PhysX 移除,對應的 standalone 範例與 API 標為 deprecated。這類場景要繼續跑,路徑是切到 Newton 的 XPBD / VBD。

**變成 no-op 的屬性**:`physxPBDMaterial:lift`、`physxPBDMaterial:drag` 保留但不再作用,只會發 deprecation warning。

**改名**:GPU deformable 接觸數上限改名為 `GpuMaxDeformableVolumeContacts`。

其中最需要留意的是第二類。**屬性還在、設得進去、不報錯,但不再有任何效果**——這是無聲失效的標準形狀,與 [13 篇](../13-contact-and-grasp-first-principles/README.md) §5 談的是同一個問題:模擬器永遠給得出答案,沒有錯誤訊息不攜帶任何正確性資訊。

---

## 4.5 唯一改變的 schema 預設值:關節速度上限沒了

逐項比對兩版 `PhysxSchema/resources/generatedSchema.usda` 裡所有帶預設值的屬性(107.3.26 共 235 條、110.1.13 共 190 條),**同名屬性的預設值只有一項不同**:

```
physxJoint:maxJointVelocity      107.3.26: 1000000        110.1.13: inf
```

官方 doc:

> Maximum joint velocity. **Only applies to joints that are part of an articulation**
> (see PhysicsArticulationRootAPI). All joint axes will use the same maximum joint velocity value.

也就是說,**5.x 時代所有 articulation 的關節都有一道 1e6 的速度上限,6.0 把它拿掉了**。求解器在接觸不穩定時算出的異常關節速度,在 5.x 會被夾住,在 6.0 原樣生效。

對機械臂、夾爪、堆高機叉齒這類**由 articulation 驅動去接觸別的物體**的場景,這是一條需要留意的行為變更:關節暴衝會把巨大動量傳給被接觸的物體,表現為「東西突然飛出去」,而且**不會有任何錯誤訊息**。

旁證是 110 同時**新增**的兩個場景層屬性,其中一個正是同一組機制:

```
physxScene:solveArticulationContactLast   ← 110 新增
  doc: Order articulation contact constraints and articulation joint
       maximum velocity constraints so that they are solved after all
       other constraints in the solver.
physxScene:disableSleeping                ← 110 新增
```

「articulation joint **maximum velocity constraints**」——110 不只改了預設值,還為這組約束加了求解順序的開關。**這一區在 107 → 110 之間被整體動過。**

**遷移時的處置**:如果你的場景依賴 articulation 去推、夾、叉東西,升上 6.0 後行為變得不穩,**先顯式把 `physxJoint:maxJointVelocity` 設回一個有限值**再談其他調參。這是一個成本極低、機制明確的對照實驗。

> ⚠ 這個比對只涵蓋 **USD schema 曝露的預設值**。PhysX SDK 內部不透過 schema 曝露的預設值沒有比對到,C++ 層仍可能有其他變更。「只有一項不同」限定在 schema 這一層。

另外,110 相對 107 **移除了 45 條**屬性,全部集中在 `physxDeformable*` / `physxParticle*` / `physxAutoParticleCloth*` / `physxAutoAttachment*`——與 §4 講的「粒子與可變形體整組移除」一致,剛體搬運類場景不受影響。

---

## 5. Schema 授權規則的變動:匯入器改寫 Newton schema

6.0 的 URDF / MJCF 匯入器**預設對匯入資產套用 Newton schema**,可在匯入時選後端。具體變動:

| 對象 | 5.x | 6.0 |
|---|---|---|
| Articulation root | `PhysxArticulationAPI` | `NewtonArticulationRootAPI` + `newton:selfCollisionEnabled` |
| Mimic joint | `PhysxMimicJointAPI` | `NewtonMimicAPI` |
| `MassAPI` | 照常授權 | **只在設定了非預設密度時才授權** |

第三條對接觸行為有直接影響。質量是接觸力計算裡的隱藏參數——質量比失衡會讓求解器算出來的法向力不足以支撐([10 篇](../10-scene-physics-authoring/README.md) 談過質量比是隱藏參數)。授權規則改變意謂**同一個匯入流程在兩版產出的資產,質量可能不一樣**,而這不會有任何警告。

跨版本搬場景時,質量屬於「要重新量、不能假設沿用」的那一類。

> 這裡還有一個組合風險:**場景帶了 `Newton*API` 的 schema,但實際跑的是 PhysX 後端**。schema extension 有載入,所以 stage 讀得進去、不報錯,但 PhysX 不吃這些 token,設定被靜默丟棄。從 6.0 匯入器產出資產、卻在 PhysX 下跑,就會落進這個組合。判斷方式是先確定後端(§2),再確認場景實際帶的是哪一套 API token。

---

## 6. API 搬家:deprecated 不等於刪除

比對兩個 tag 的 `source/extensions/` 會看到三個 core extension「消失」:

```
6.0 相對 5.1 少了:  isaacsim.core.api   isaacsim.core.prims   isaacsim.core.utils
6.0 相對 5.1 多了:  isaacsim.core.experimental.actuators
                    isaacsim.core.experimental.primdata
                    isaacsim.core.rendering_manager
```

**它們沒有被刪除,是搬到了 `source/deprecated/`**:

```
source/deprecated/isaacsim.core.api/
source/deprecated/isaacsim.core.prims/
source/deprecated/isaacsim.core.utils/
```

仍然可用,既有腳本不會立刻壞。遷移方向:

| Deprecated | 改用 |
|---|---|
| `isaacsim.core.api` | `isaacsim.core.experimental.*` |
| `isaacsim.sensors.physics` / `isaacsim.sensors.physx` | `isaacsim.sensors.experimental.physics` |
| `isaacsim.robot.manipulators` 及相關 motion generation | 見官方 migration guide |

`isaacsim.sensors.physx*` 三個 extension 在 6.0 的 `source/extensions/` 底下確實不見了,對照官方 release notes 是 deprecated 而非移除——**目錄結構的增減不能直接讀成功能的存廢**,要對照 `source/deprecated/` 與 release notes 才有結論。

---

## 7. 對調參的實務影響

假設情境:5.x 的搬運場景搬上 6.0.1,後端維持 PhysX,某個抓取/承載動作變得不穩。

**不該做的第一步**是調摩擦。理由在 [13 篇](../13-contact-and-grasp-first-principles/README.md) 已完整推導:`g > 0 ⟹ λₙ = 0 ⟹ ‖f_t‖ ≤ μ·0 = 0`,沒有接觸點時 μ 設多大都是零。

跨版本情境下,排查順序建議如下:

| 順序 | 檢查 | 為什麼排在這 |
|---|---|---|
| 0 | **確認後端**(§2) | 決定後面所有知識適不適用 |
| 1 | 幾何是否真的接觸 | 決定 `λₙ` 存不存在 |
| 2 | 質量是否跨版本改變(§5) | `MassAPI` 授權規則變了,且不報警 |
| 3 | 碰撞近似是否仍表達得出凹特徵 | 凸包填實孔洞是定義的後果 |
| 4 | contact / rest offset | 補幾何落差 |
| 5 | 摩擦 μ | 最後 |

一個有用的反向判據:**當 μ 已經被調到遠離物理合理範圍、問題卻沒有改善時,那本身就是「瓶頸不在摩擦」的證據。** 官方給的參考值可以當標尺:

| 材質配對 | 靜摩擦 μ | 動摩擦 μ |
|---|---|---|
| 混凝土 / 混凝土 | 0.6 | 0.5 |
| 鋼 / 鋼 | 0.74 | 0.57 |
| 橡膠 / 混凝土 | 1.0 | 0.8 |
| 木 / 木 | 0.5 | 0.3 |
| 紙板 / 鋼 | 0.4 | 0.3 |

— `skills/physics-simulation/SKILL.md` @ v6.0.1

日常工程材料的 μ 大致落在 0.3–1.0。看到場景裡掛著 μ = 5,那不是一個「摩擦不夠」的場景,是一個**曾經有人往摩擦方向調過而且沒有解決問題**的場景。這種數值本身就是排查證據。

另外兩條與版本無關、但在跨版本排查時特別容易忽略的規則(同樣來自官方 skill):

- **`RigidBodyAPI` 與 `CollisionAPI` 必須在同一個 prim 上。** 拆到父/子層會造成間歇性的碰撞失效——間歇性正是最難歸因的失效形態。
- **帶 scale 的靜態碰撞體要用「父 xform 放位置、子 mesh 放 scale」**,直接對套了 `CollisionAPI` 的 Cube 做 scale,PhysX 會用錯碰撞邊界(典型症狀是物件穿過地面)。

---

## 8. 一頁摘要

| 問題 | 答案 |
|---|---|
| 6.0 是不是改用 Newton 了? | 否。要顯式啟用 `isaacsim.physics.newton`(或用 `full.newton.kit`)才會是 Newton |
| 那 6.0 的物理為什麼跟 5.1 不一樣? | PhysX 自己從 107.3.x 換到 110.1.x |
| 5.x 場景搬上來該用哪個? | 官方建議 PhysX |
| 怎麼確定我這台跑哪個? | 讀那支 kit app 檔有沒有 newton;log 找 `isaacsim.physics.newton` startup;或執行期 `get_active_physics_engine()` |
| log 有 newton 就是 Newton 嗎? | 否。`pip.newton` 與 `usd.schema.newton` 在 PhysX 環境也會載入 |
| 舊 API 會不會壞? | 短期不會,搬到 `source/deprecated/` 仍可用 |
| 升級最該重新量的是什麼? | 質量(`MassAPI` 授權規則變了且不報警) |
| **articulation 推東西變得會亂飛?** | **`physxJoint:maxJointVelocity` 預設從 1e6 變成 inf,先設回有限值試(§4.5)** |

---

## 參考

官方(`github.com/isaac-sim/IsaacSim`,標明 tag):

- `source/apps/isaacsim.exp.full.kit` / `isaacsim.exp.full.newton.kit` @ v6.0.1、main — 後端啟用的決定性依據
- `source/extensions/isaacsim.physics.newton/docs/Overview.md` @ v6.0.1 — Newton 定位與設定類別
- `skills/physics-simulation/SKILL.md` @ v6.0.1 — 後端選擇建議、摩擦參考表、prim 設定規則(§2.2 記錄了它與實檔的一處矛盾)
- Isaac Sim 6.0.0 release notes — Newton 實驗性支援、PhysX 移除項、schema 變更、API deprecation
- `source/deprecated/` @ v6.0.1 — 三個 core extension 的實際去向

實測:Isaac Sim 5.1 native 與 `nvcr.io/nvidia/isaac-sim:6.0.1` 容器各一台,2026-07-29。`omni.physx` 版本、extension startup 序列、三支 kit app 的 newton 條目均為該兩機實際輸出。
