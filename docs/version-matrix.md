# 5.1 與 6.0.1 的差異速查

跨版本排查時最花時間的不是「哪裡不一樣」,而是「這個症狀該不該歸給版本」。下表把本 repo 實測或查證過的差異集中在一處,每一列都標出處篇章;沒列進來的,就是本 repo 沒有證據,不要當成「兩版相同」。

| 項目 | 5.1 | 6.0.1 | 出處 |
|---|---|---|---|
| Python API 命名空間 | `isaacsim.core.api` / `isaacsim.core.prims` / `isaacsim.core.utils` | 整組移至 `isaacsim.core.experimental.*`(breaking change) | [01 §3](common/01-install-and-run-modes/README.md) |
| PhysX 版本 | 107.3.x | 110.1.x(升級即發生,無法迴避) | [15 §1](6.0.1/15-physics-backend-5.1-to-6.0/README.md) |
| 可選物理後端 | PhysX | PhysX 或 Newton;**5.x 場景官方建議留在 PhysX** | [15 §2–3](6.0.1/15-physics-backend-5.1-to-6.0/README.md) |
| `physxJoint:maxJointVelocity` 預設 | `1000000` | `inf` —— 兩版 schema 逐項比對後**唯一**改變的預設值 | [15 §4.5](6.0.1/15-physics-backend-5.1-to-6.0/README.md) |
| ROS 2 bridge extension | `isaacsim.ros2.bridge` 一個 extension 全包 | 拆成門面 + `core`/`nodes`/`examples`/`ui` 四個實作;`--enable isaacsim.ros2.bridge` 照舊帶起整套 | [14 §1](6.0.1/14-ros2-bridge-6.0-architecture/README.md) |
| ROS 2 外部介面(啟動參數、設定鍵、OmniGraph 節點型別名) | — | **沒變** | [14](6.0.1/14-ros2-bridge-6.0-architecture/README.md) |
| 匯入器授權的 articulation schema | `PhysxArticulationAPI` | `NewtonArticulationRootAPI` + `newton:selfCollisionEnabled` | [15 §5](6.0.1/15-physics-backend-5.1-to-6.0/README.md) |
| 驅動相容(實測組合) | 555 世代穩定;**595 世代啟動約 65 秒必 segfault** | 595 正常 | [01 §3](common/01-install-and-run-modes/README.md) |
| 5.1 場景搬進新版的記憶體風險 | — | 有 OOM/異常的現場觀察,機轉分「官方出處」與「推測」兩級,尚未直接重現 | [08](6.0.1/08-migration-5.1-to-6.0-oom-risk/README.md) |
| 官方資產包分片數 | 3 片 | 5 片 | [25 §2](common/25-offline-assets-deployment/README.md) |
| 資產解壓後的目錄 | `Assets/Isaac/5.1` | `Assets/Isaac/6.0`(**不是 `6.0.1`**,檔名帶修訂號而目錄名不帶) | [25 §3](common/25-offline-assets-deployment/README.md) |
| 離線環境的 extension registry | 需自行關掉 | 官方文件稱由 Kit SDK 自動管理(本 repo 未實測) | [25 §5](common/25-offline-assets-deployment/README.md) |

## 這張表怎麼用

`maxJointVelocity` 那一列是升級後最容易誤判的一條。5.x 時代所有 articulation 的關節都有一道 1e6 的速度上限,求解器在接觸不穩定時算出的異常速度會被夾住;6.0 拿掉之後同樣的數值原樣生效,表現成「東西突然飛出去」,而且不報錯。場景若靠 articulation 去推、夾、叉東西,升版後行為變不穩時,先把這個屬性顯式設回有限值當對照實驗,再談其他調參。

ROS 2 那兩列要一起看:結構換了一輪而外部介面沒動,是最容易產生錯誤判斷的一種升級——照著新架構去改啟動參數和設定鍵,反而會把原本能跑的接法改壞。

驅動那一列的方向性值得記住:降驅動要 DKMS 重編 kernel module,風險高於換 Isaac Sim 版本(只是換 container tag)。遇到不相容,優先升 Isaac Sim。

## 沒有進表的東西

- **6.0 的最小可跑範例**:本 repo 的 [07 篇](5.1/07-minimal-example/README.md)程式碼以 4.5–5.1.x API 寫成,在 6.0 需要改 import 路徑,但改完的版本尚未實機驗證,因此不另立一篇。
- **WebRTC 串流在 6.0 的行為**:[06 篇](common/06-webrtc-streaming/README.md)的單 client 限制與分流架構實測於 5.1,6.0 未驗證。
- 兩版共通的機制與方法論不在此表,見[共通區](common/)。
