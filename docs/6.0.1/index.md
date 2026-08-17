# Isaac Sim 6.0.1

這一區收「只在 6.0.x 成立」的內容:extension 架構重組、PhysX 換代與 Newton 後端、從 5.1 搬場景的風險,以及 6.0 的物理調參。兩版通用的機制與方法論在[共通區](../common/),差異對照見[版本速查表](../version-matrix.md)。

## 從 5.1 升上來

升級這件事本身有一條建議順序,因為前兩篇會決定後面所有排查的方向:

1. **[15 · 5.1 → 6.0 的物理層變動](15-physics-backend-5.1-to-6.0/README.md)** —— PhysX 換代(107→110)與 Newton 後端加入是**兩件獨立的事**,常被混談成「6.0 改用 Newton 了」。先確定自己這台跑的是哪一個後端(log 裡有 newton ≠ Newton 在跑),否則整個排查方向會偏掉。含兩版 schema 逐項比對的結果。
2. **[14 · ROS 2 Bridge 的架構重組](14-ros2-bridge-6.0-architecture/README.md)** —— 一個 extension 拆成五個,而外部介面沒變。三個「看起來變了其實沒變」的判讀陷阱,加上 rclpy 的 system→internal fallback 與「啟動前不要 source ROS」的機制。
3. **[08 · 5.1 場景搬進 6.0.1 的 OOM / 異常風險](08-migration-5.1-to-6.0-oom-risk/README.md)** —— 調查報告,不是教學。結論分「官方出處」與「推測」兩級,尚未實機重現的部分有明確標註。附遷移 SOP。

## 場景在 6.0 跑不對

手上有一個物理行為不對的場景要修,而且不熟 Isaac Sim,從 16 篇開始——它從「一個會被搬動的箱子由哪些東西組成」講起,不預設前置知識。

- **[16 · 把 5.x 場景調到 6.0 能跑,東西不會亂飛](16-model-tuning-for-6.0/README.md)** —— 先處理**結構**:剛體與碰撞怎麼分層、質量掛在哪一層、材質綁定為什麼會 fallback 回渲染材質、SDF 解析度不足以表達孔洞。四個問題的共同特徵是設得進去但不生效,而且不報錯。含東西亂飛的成因排序與診斷決策樹。
- **[17 · 6.0 的物理調參:入口、生效條件、完整參數表](17-physics-parameter-tuning-6.0/README.md)** —— 再處理**數值**:三個調參入口、五個讓設定無聲失效的條件,以及取自 6.0.1 實機 schema 的七類完整預設值表。核心問題是「我設了這個值,它到底有沒有作用」,而只有設極端值看行為差異能回答它。

## 接著往共通區走

6.0 專屬的部分講完之後,真正決定成敗的多半是版本無關的東西:

- 夾不住、叉不起來、轉彎會滑 → [13 接觸與抓握的第一性原理](../common/13-contact-and-grasp-first-principles/README.md)(為什麼調摩擦常常是錯的第一步)
- 尺寸都對但件插不進去 → [22 幾何的量測紀律](../common/22-geometry-and-measurement-discipline/README.md)
- 要跑幾十輪調參 → [19 調參實驗的方法論](../common/19-tuning-experiment-methodology/README.md) 與 [20 用 Claude Code 跑調參](../common/20-claude-code-driven-tuning/README.md)
- 症狀怎麼都追不到源頭 → [23 物理模擬不可以偷懶](../common/23-no-shortcuts-in-physics-sim/README.md)

## 這一區沒有的東西

6.0.1 環境下實機驗證過的**最小可跑範例**目前沒有。[07 篇](../5.1/07-minimal-example/README.md)的程式碼以 4.5–5.1.x API 寫成,搬到 6.0 需要把 `isaacsim.core.*` 改成 `isaacsim.core.experimental.*`,但改完的版本尚未在本 repo 的環境跑過,因此不另立一篇冒充驗證過。

## 搭配的 Claude Code skill

[`skills/isaac-sim-60/SKILL.md`](../../skills/isaac-sim-60/SKILL.md) 是這一區的濃縮版,收 6.0.x 特有的行為與陷阱:物理後端判定、`maxJointVelocity` 從 1e6 變 inf、參數的五個無聲失效條件、關節名≠世界軸、容器內沒有 usdcat 時怎麼讀寫 USD、關鍵預設值速查。
