# CAD 資產的判讀與轉換:從 IGES 檔到可用的碰撞幾何

一個累積幾年的資產庫裡,同一個物件常常有好幾份:CAD 原始檔、從它轉出的 USD、
還有一份長得很像但來源完全不同的美術資產。**哪一份現在被場景用著、哪一份的幾何品質更好,
不會寫在檔名上**,而判斷錯的代價是實質的——把一份現成可用的資產判定成「轉換失敗」,
或者更糟,把美術資產當成工程模型去調物理,然後花好幾天追一個根本不存在的建模瑕疵。

這篇處理三件事:怎麼在不開 CAD 軟體的前提下判讀 IGES 原始檔、怎麼正確數出一份 USD 裡
到底有沒有幾何(這一項有一個會讓人完全看錯的陷阱)、以及 Isaac Sim 6.0 內建轉換鏈的
實際呼叫方式。

前置定位見 [03 模型格式與匯入](../03-model-import/README.md);本篇是它 §2「CAD → USD」
那一節的操作展開。

---

## 1. 三種版本,三種幾何品質

同一個物件在資產庫裡常見的三種形態:

| 形態 | 幾何來源 | 典型特徵 |
|---|---|---|
| **CAD 原始檔**(`.IGS` / `.STEP` / `.SLDPRT`) | 機構設計 | 曲面(NURBS / trimmed surface),不是三角網格 |
| **CAD 轉出的 USD** | tessellate 上面那份 | 零件階層完整、面數低、外觀樸素 |
| **美術資產** | 資產庫 / 掃描 / 手工建模 | 面數高、有 PBR 貼圖、幾何刻意做出「真實感」 |

一個實測對照(同一種標準棧板):

| | 美術資產 | CAD 轉出 |
|---|---|---|
| 三角形 | 11,870 | **5,152**(43%) |
| 側面平整度 | 1.6~41.5 mm 凹凸,缺角達 4 cm 級 | 設計值 |
| 零件結構 | 單一 mesh | 4 個 prototype × 多實例 |

**那 4 cm 的凹凸不是建模瑕疵,是設計意圖** —— 那份資產叫「回收木棧板」,
它就該看起來破舊。問題在於它同時被拿去當物理接觸面用,
而接觸面的平整度直接決定摩擦、翻滾力矩與求解器能不能收斂
(見 [13 接觸與抓握的第一性原理](../13-contact-and-grasp-first-principles/README.md))。

### 1.1 怎麼辨識美術資產

看檔案裡有沒有這些東西,任一命中就幾乎確定:

| 訊號 | 為什麼 |
|---|---|
| `DomeLight` / `DistantLight` / `Camera` / `RenderProduct` / `RenderSettings` | **那是一個資產展示場景**,不是零件模型 |
| 材質是 PBR 三件組(`*_Albedo` / `*_Normal` / `*_ORM` 貼圖 + `.mdl`) | 為了渲染真實感而做 |
| mesh 名稱形如 `SM_<名稱>_<兩位數變體>` | `SM` = Static Mesh,商業資產庫的命名慣例 |
| 單一高密度 mesh、沒有零件階層 | 建模時就合併了 |

### 1.2 怎麼辨識 CAD 轉出的 USD

| 訊號 | 為什麼 |
|---|---|
| prim 名稱有 `tn__` 前綴 | USD 對非 ASCII 名稱的編碼(translated name)。CAD 零件常是中文或帶特殊字元命名 |
| 名稱含 `DE<數字>` | IGES 的 **D**irectory **E**ntry 編號,轉換器直接拿來命名 |
| 名稱含 `STEP1` 之類的來源標記 | 從 STEP 檔轉來 |
| `stage.GetPrototypes()` 非空 | 轉換器預設用 instancing(見 §3) |
| 材質是 `UsdPreviewSurface`,沒有貼圖 | CAD 沒有材質資訊,轉換器只給預設 |

---

## 2. 判讀 IGES 原始檔(不需要 CAD 軟體)

IGES 是**固定 80 欄的純文字格式**,用一支 Python 就能讀出關鍵資訊。

### 2.1 檔案結構

每一行的**第 73 欄**是 section code:

| Code | Section | 內容 |
|---|---|---|
| `S` | Start | 人可讀的說明 |
| `G` | Global | **單位、精度、產生系統、作者** |
| `D` | Directory Entry | 每個實體 2 行:型別、狀態、指向 P section 的指標 |
| `P` | Parameter Data | 實體的實際數值(控制點、節點向量……) |
| `T` | Terminate | 各 section 的行數統計 |

### 2.2 實體型別統計:判斷需不需要 tessellation

Directory Entry 每個實體佔 2 行,**第 1 行的欄位 1–8 是型別編號**:

```python
with open(path, errors="replace") as f:
    de = [ln for ln in f if len(ln) >= 73 and ln[72] == "D"]
for i in range(0, len(de) - 1, 2):
    entity_type = int(de[i][0:8])
```

常見型別與判讀:

| 型別 | 意義 | 對轉換的含意 |
|---|---|---|
| 110 / 100 / 126 | 直線 / 圓弧 / NURBS 曲線 | 曲線,多半是曲面的邊界 |
| **128** | NURBS 曲面 | **要 tessellation** |
| **144** | Trimmed surface(修剪曲面) | **要 tessellation**,最常見的實體 |
| 120 / 122 | 旋轉面 / 拉伸面 | 解析曲面,要 tessellation |
| **186 / 510 / 512 / 514** | B-rep 實體 / Face / Shell | 要 B-rep tessellation,比曲面更重 |
| **106** | Copious data | **已經是離散點**,不需 tessellation |
| **308 / 408** | Subfigure 定義 / 實例 | **零件階層**——轉出的 USD 會用它做 instancing |

一份典型的機構件 IGES(標準棧板)實測:

```
1552 × 126  NURBS 曲線      384 × 144  Trimmed surface
1352 × 110  直線            292 × 128  NURBS 曲面
 768 × 102  複合曲線         92 × 120  旋轉面
 384 × 142  曲面上的曲線      28 × 408  Subfigure 實例
```

→ **全部是曲面,沒有任何離散幾何**。所以轉換結果的面數與品質**完全由
tessellation 參數決定**,不是檔案內容決定。要調品質就去調轉換器的 LOD(§4.3)。

### 2.3 單位:Global section 的 Hollerith 陷阱

Global section 用逗號分隔,單位資訊在第 14/15 個參數。但**不能直接用逗號 split** ——
IGES 的字串是 Hollerith 格式(`2HMM` 表示「長度 2 的字串 MM」),
**字串內容裡的逗號會破壞欄位切分**,讓後面所有欄位偏移。

實務上最省事的做法是直接找 `<數字>H` 開頭的 token:

```python
raw = "".join(ln[:72] for ln in g_lines)
# 找 Hollerith:如 "2HMM" → 單位是 mm
```

實測值得留意的是,同一份檔案裡 Global section 也常帶著**原始檔名與零件編號**,
那往往是尺寸的直接線索(例如編號裡就寫著標稱長寬高)。

### 2.4 blank status:一個看起來很有解釋力、但通常不是原因的欄位

Directory Entry 第 1 行的**欄位 65–72** 是 status number,格式 `BBSSAAHH`:

| 欄 | 意義 |
|---|---|
| `BB` | Blank status:`00` 可見 / `01` 隱藏 |
| `SS` | Subordinate entity switch:`00` 獨立 / `01` 被其他實體參照 |
| `AA` | Entity use flag:`00` 幾何 / `05` 結構 / `06` 其他 |
| `HH` | Hierarchy |

看到「轉出來沒有幾何」時,很容易懷疑是所有實體都被標成 blanked 而被轉換器略過
(轉換器確實有 `omitHiddenOnLoad` 這個預設開啟的選項)。

⚠ **但實測三份不同的 CAD 檔,blank 分布是同型的**:

| 檔案 | blanked / 總實體 | 轉換結果 |
|---|---|---|
| 棧板 | 4,408 / 5,478(80%) | 有幾何 |
| 貨架總成 | 28,856 / 36,378(79%) | 有幾何 |
| 滑軌底座 | 660 / 765(86%) | 有幾何 |

**「大多數實體 blanked」是 CAD 匯出的常態**(被 Subfigure 參照的子實體通常都標 blanked),
不是異常訊號。把它當成失敗原因會走進死路——這一條是實際踩過才寫下來的。

---

## 3. 判讀轉出的 USD:instancing 會讓你完全看錯

**這是本篇最重要的一節。**

數一份 USD 有多少 mesh,最直覺的寫法是:

```python
meshes = [p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
```

**這對 CAD 轉出的資產會回傳 0**,而檔案裡明明有幾百個 mesh。

原因:`Usd.Stage.Traverse()` 的預設 predicate **不走訪 instance proxy**。
而 Omniverse 的 CAD 轉換器**預設就用 instancing**
(`instancingStyle = eInstanceableReference`)——CAD 組件裡大量重複的螺栓、
橫樑、墊塊,用 instance 存一份原型再引用是正確的做法。

三種讀法的實測對照(同一份棧板 USD,148 KB):

| 讀法 | 結果 |
|---|---|
| `stage.Traverse()` | **0** |
| `Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate))` | 3,684 |
| `stage.GetPrototypes()` 內的 mesh | **384**(唯一幾何:5,920 頂點 / 5,152 三角形) |

### 3.1 正確的數法

```python
from pxr import Usd, UsdGeom

def count_geometry(stage):
    """回傳 (唯一幾何的 mesh 數, 展開後的實例數)。"""
    protos = stage.GetPrototypes()
    if protos:                                   # ← 先問這一句
        uniq = [q for pr in protos
                for q in Usd.PrimRange(pr, Usd.PrimAllPrimsPredicate)
                if q.IsA(UsdGeom.Mesh)]
        rng = Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies(
            Usd.PrimAllPrimsPredicate))
        return len(uniq), len([p for p in rng if p.IsA(UsdGeom.Mesh)])
    plain = [p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
    return len(plain), len(plain)
```

**判準:數 mesh 之前先問 `stage.GetPrototypes()` 有幾個。非 0 就必須換讀法。**

兩個數字的用途不同:**唯一幾何**決定記憶體與 cook 成本、也是你要檢查的幾何本體;
**展開後的實例數**才是場景裡實際存在幾個物件。

### 3.2 為什麼正對照沒有擋住這個錯

這個錯誤實際發生過,而且當時是做過對照的:用同一段程式讀一份美術資產,
得到的三角形數與另一份文件記錄的場上值完全吻合——對照通過了。

**但美術資產的 `GetPrototypes()` 是空的**(沒有 instancing),CAD 資產全部有。
對照組剛好落在方法有效的那一側,於是驗證了一個並不成立的推廣。

> **正對照要挑「與待測對象同類」的樣本。**
> 拿一份沒有 instancing 的檔去驗證「我數得到 mesh」,
> 不能保證對有 instancing 的檔也成立。

這是 [19 調參實驗方法論](../19-tuning-experiment-methodology/README.md) 的
「極端值正對照」在**查詢方法**上的對應形式:對照組要能區分「方法有效」與「方法無效」,
落在同一側的對照不提供資訊。

---

## 4. 轉換管線:Isaac Sim 6.0.1 的實際呼叫方式

Isaac Sim 6.0.1 內建完整的 CAD 轉換鏈,不需要另外裝 Omniverse 桌面版:

| Extension | 角色 |
|---|---|
| `omni.kit.converter.cad` | 聚合層(依賴下面三個) |
| `omni.kit.converter.hoops` / `hoops_core` | **HOOPS Exchange**,負責 IGES / STEP / SLDPRT / CATPart / X_T / PRT |
| `omni.kit.converter.dgn` / `jt` | MicroStation DGN / Siemens JT |

### 4.1 正確的呼叫方式

官方自己的做法是**啟動一個只載入轉換器的獨立 kit 進程**
(見 `omni.kit.converter.hoops/delegate.py` 的 `launch_kit_app`),照抄即可:

```bash
LAUNCH=$(ls -d $EXTSCACHE/omni.kit.converter.hoops-*/)omni/kit/converter/hoops/process/launch_hoops_app.py
echo '{}' > /tmp/cfg.json          # 空 config = 用轉換器預設

$ISAAC_SIM/kit/kit \
  --ext-folder $ISAAC_SIM/exts \
  --ext-folder $ISAAC_SIM/extscache \
  --ext-folder $ISAAC_SIM/apps \
  --enable omni.kit.converter.hoops_core \
  --exec "$LAUNCH --input-path <in.IGS> --output-path <out.usd> --config-path /tmp/cfg.json" \
  --no-window --info
```

實測耗時:1.9 MB / 5,478 實體的棧板 **15 秒**;12 MB / 36,378 實體的貨架總成 **19 秒**
(含 kit 啟動)。

### 4.2 三條走不通的路

| 做法 | 結果 |
|---|---|
| 用 `omni.kit.asset_converter` | `UNSUPPORTED_IMPORT_FORMAT`。**它只吃 `.gltf` / `.fbx` / `.obj`**,CAD 是完全另一條管線 |
| 在 `SimulationApp` 進程裡 `import omni.kit.converter.hoops_core` | `libTD_DbCore.so: undefined symbol: _ZN13OdConstStringC1EPKw` —— ODA library 的載入順序在完整 app 環境下不對 |
| 掛載其他 kit 進程正在使用的 cache volume | `Failed to acquire exclusive lock to data store`。**轉換用的臨時容器要用自己的 cache** |

第一條特別值得記:兩個 extension 名字都叫 "converter",支援格式卻不重疊,
而錯誤訊息 `UNSUPPORTED_IMPORT_FORMAT` 很容易被讀成「這個檔案有問題」。

### 4.3 可調參數

`HoopsOptions` 的欄位(透過 config JSON 傳入),影響幾何品質的是前幾項:

| 參數 | 預設 | 作用 |
|---|---|---|
| `tessLOD` | 2 | **tessellation 細緻度**(0~4)。§2.2 說過面數完全由這裡決定 |
| `accurateTessellation` | false | 更貼近原始曲面,面數更高 |
| `accurateSurfaceCurvatures` | true | 曲率處理 |
| `instancingStyle` | `eInstanceableReference` | **產生 instance**——就是 §3 那個陷阱的來源 |
| `dedup` | true | 相同幾何去重 |
| `convertPhysicsData` | false | CAD 若帶物理資訊是否轉入 |
| `omitHiddenOnLoad` | true | 略過 blanked 實體(§2.4) |
| `upAxis` | file default | 座標系 |

---

## 5. 驗證轉換結果:三層,缺一層就會誤判

| 層 | 看什麼 | 陷阱 |
|---|---|---|
| **轉換器自述** | log 裡的 `Total Meshes in USD = N` / `Total Triangles in USD = N` | 訊息在冗長 log 的中段,**不要用管線截斷** |
| **USD 結構** | prototypes 數 + 唯一幾何 mesh 數(§3.1) | `Traverse()` 對 instanced 資產回 0 |
| **幾何本體** | bbox、頂點數分布、沿高度軸的佔用剖面 | 見 §6 的座標與單位陷阱 |

第一層最容易被跳過,而它是**最直接的答案**。實際發生過的情況:轉換器在 log 中段
明寫 `Total Meshes in USD = 384`,而那段正好落在被 `head -40` / `tail -70` 截掉的區間,
於是往「檔案有問題」「license 有問題」猜了兩輪。

> **診斷用的輸出寫進檔案,不要走管線截斷。**
> 「跑完了但沒產出」的第一步永遠是看完整輸出,不是猜成因。

第三層的「佔用剖面」值得一提:把 bbox 沿高度軸切 N 段,數每段有幾個 mesh 相交,
可以在不開 GUI 的情況下看出結構。一份 CAD 檔的實測剖面:

```
 −59.6 ~  133.0 mm   79→477→1688→798→688→2285 個 mesh   ← 密集,零件本體
 133.0 ~ 1225.0 mm   固定 12 個                          ← 稀疏,貫穿全高
```

那 12 個固定值一路貫穿到頂,追下去是 2 個實例 × 6 面的純盒子(24 頂點 / 12 三角形)
—— **CAD 檔裡混著參考包絡框**,不是零件。沒有這個剖面,整體 bbox 會把包絡框算進去,
得到一個比實際零件大得多的尺寸。

---

## 6. 讀幾何尺寸時的座標與單位陷阱

三個都會產生「看起來合理但錯誤」的數字。

### 6.1 world AABB 對有傾角的物件會被撐大

軸對齊包圍盒(AABB)不會跟著物件旋轉。一個 1060 mm 寬、真實高度 121 mm 的板件,
安裝時有 2.62° 傾角,它的 world AABB 高度會量到 **168.8 mm**:

```
1060 mm × sin(2.62°) = 48.5 mm     121 + 48.5 ≈ 169 ✓
```

照那個數字往下算(例如「上下板之間的淨空」),會得到完全虛構的結果。

> **物件有傾角時,world AABB 只能用來比對同一物件的相對變化,
> 不能拿來讀尺寸,也不能拿來比較不同位置的高度差。**
> 要讀尺寸就讀局部座標(`xformOp:translate` / `xformOp:scale`),
> 或用 `ComputeUntransformedBound`。

### 6.2 軸向不能假設

「高度軸是 Z」在 CAD 轉出的資產上經常不成立——CAD 軟體的座標慣例、
轉換器的 `upAxis` 設定、以及各層 xform 疊加後,實際的高度軸可能是任一軸。

穩健的做法是**從資料本身判別**:

```python
hz = min(range(3), key=lambda k: ext[k])      # 最薄的那軸 = 板件的高度軸
```

但這只對「明顯扁平」的物件有效。近正方形的物件(長寬比 < 1.1)用這招會判錯,
要用別的線索(例如零件的排列方向),而且**判錯不會報錯**。

### 6.3 單位不能假設,而且同一份 stage 裡可能不一致

`UsdGeom.GetStageMetersPerUnit()` 只給 stage 層級的宣告值。
**各子樹的 xform scale 會讓局部座標的實際單位不同**——實測同一份場景裡,
一個子樹的局部座標是公分、另一個是公釐。

> **用一個已知真值反推單位。**
> 量到 `1120000` 這種數量級不合理的值,就是單位錯的訊號——但前提是你把它印出來看。

---

## 7. 檢查清單

拿到一份不熟悉的資產時:

- [ ] `stage.GetPrototypes()` 有幾個?非 0 就換讀法(§3.1)
- [ ] 檔案裡有 `Camera` / `DomeLight` / `RenderSettings` 嗎?有 → 是美術展示場景(§1.1)
- [ ] prim 名稱有 `tn__` / `DE<數字>` 嗎?有 → 是 CAD 轉出的(§1.2)
- [ ] 材質是 PBR 貼圖組還是 `PreviewSurface`?
- [ ] 整體 bbox 與「一個零件應該多大」差很多嗎?差很多 → 檔案裡混著別的東西,做佔用剖面(§5)
- [ ] 要讀尺寸:讀的是局部座標還是 world AABB?物件有傾角嗎?(§6.1)
- [ ] 高度軸是從資料判出來的,還是假設的?(§6.2)
- [ ] 單位用已知真值反推過了嗎?(§6.3)

要跑轉換時:

- [ ] 用的是 `hoops_core` 那條管線,不是 `asset_converter`(§4.2)
- [ ] 獨立 kit 進程,不在 `SimulationApp` 裡 import(§4.2)
- [ ] 臨時容器用自己的 cache volume(§4.2)
- [ ] 完整 log 寫進檔案(§5)
- [ ] 驗證三層都做了,不是只看檔案大小(§5)

---

## 8. 延伸閱讀

- [03 模型格式與匯入](../03-model-import/README.md) —— 匯入路徑的整體定位、依賴解析、資產授權
- [10 場景資產的物理結構](../10-scene-physics-authoring/README.md) —— 轉出幾何之後怎麼加碰撞與物理
- [13 接觸與抓握的第一性原理](../13-contact-and-grasp-first-principles/README.md) —— 為什麼接觸面的平整度是物理結果的上游
- [19 調參實驗方法論](../19-tuning-experiment-methodology/README.md) —— 正對照的設計原則(§3.2 是它在查詢方法上的對應)
- 官方文件:Omniverse CAD Converter、HOOPS Exchange 支援格式清單
