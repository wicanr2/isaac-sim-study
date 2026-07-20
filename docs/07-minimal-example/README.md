# 最小可跑範例:從零到第一個會動的模擬

前六篇講完了機制,這篇補上「第一步到底怎麼踏」。目標是三個由小到大的可跑範例:物理 hello world(方塊落地)、載入官方倉庫場景、在官方場景裡讀機器人位姿。全部走 standalone Python(不開 GUI 也能跑),程式碼以 NVIDIA 官方教學與 `standalone_examples` 為底組合。

> 版本與驗證狀態:API 以 Isaac Sim 4.5–5.1.x 為準(6.0 起 `isaacsim.core.*` 移至 `isaacsim.core.experimental.*`,見 [01 篇](../01-install-and-run-modes/README.md) §3)。本篇程式碼依官方文件與官方範例組合而成,**尚未在本 repo 的環境實機驗證**;資產路徑在不同文件版本間基準點不一,執行時務必用文中的 `is_file()` 檢查確認。

## 0. 執行方式:一定要用 Isaac Sim 自帶的 Python

standalone script 不是用系統 Python 跑,而是用安裝目錄裡的 `python.sh`(它會把 Isaac Sim 的直譯器與函式庫環境架好):

```bash
cd <isaac-sim 安裝目錄>
./python.sh /path/to/my_script.py
```

還有一條**鐵則,官方文件反覆強調**:所有 `omni.*` / `isaacsim.*`(除了 `SimulationApp` 本身)的 import,**必須寫在 `SimulationApp(...)` 實例化之後**——因為那些模組要等 Kit 應用起來才存在。把 import 全放檔案開頭是這裡最常見的錯誤。

## 1. 物理 hello world:一顆方塊落地

改寫自官方 Core API 教學(Hello World),`headless` 開關就是 [01 篇](../01-install-and-run-modes/README.md)三種模式在 standalone 下的對應:

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})   # False = 開 GUI 視窗

# ---- 以下的 import 必須在 SimulationApp 之後 ----
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

world = World()
world.scene.add_default_ground_plane()      # 地面(帶 collision)
cube = world.scene.add(
    DynamicCuboid(                          # 動態剛體:受重力、會碰撞
        prim_path="/World/my_cube",
        name="my_cube",
        position=np.array([0.0, 0.0, 1.0]), # 從 1 m 高處落下
        scale=np.array([0.5, 0.5, 0.5]),
        color=np.array([0.0, 0.0, 1.0]),
    ))
world.reset()                               # 初始化物理(必要)

for i in range(500):
    position, orientation = cube.get_world_pose()
    if i % 100 == 0:
        print(f"step {i}: z = {position[2]:.3f}")
    world.step(render=True)                 # 推進一個物理步(headless 下 render 仍驅動更新)

simulation_app.close()
```

跑起來會看到 z 從 1.0 一路降到約 0.25(方塊半高)後停住——**這一行輸出就是「物理世界真的在算」的證據**,對應 [04 篇](../04-physics-world/README.md)的所有概念:Physics Scene、地面、剛體、PLAYING(`world.step()` 內部處理了播放)。

`World` 是官方的高層封裝(場景管理 + 物理 context + 常用物件);[02 篇](../02-python-no-ui/README.md)的 `--exec` 走的是更底層的路徑,兩者最後操作的是同一個 stage。

## 2. 載入官方倉庫場景

官方資產(倉庫環境、範例機器人)從 asset root 下載(見 [03 篇](../03-model-import/README.md) §2),不需要自備任何模型檔。官方 `standalone_examples` 的 `load_stage.py` 模式:

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import carb
from isaacsim.core.utils.stage import open_stage
from isaacsim.storage.native import get_assets_root_path   # 5.x;版本不同時搜尋 get_assets_root_path
import omni.usd

assets_root = get_assets_root_path()        # 例:https://omniverse-content-production.s3.../Assets/Isaac/5.1
if assets_root is None:
    carb.log_error("拿不到 asset root——檢查網路與 asset_root 設定(01 篇 §2)")
    simulation_app.close(); raise SystemExit

scene_path = assets_root + "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
ok = open_stage(scene_path)
print(f"open_stage -> {ok}")

# 等場景與其引用的資產載完再往下走
import isaacsim.core.utils.stage as stage_utils
while stage_utils.is_stage_loading():
    simulation_app.update()

import omni.timeline
omni.timeline.get_timeline_interface().play()
for _ in range(300):
    simulation_app.update()
simulation_app.close()
```

倉庫場景在同目錄還有幾個變體:`warehouse.usd`(空倉)、`warehouse_with_forklifts.usd`、`warehouse_multiple_shelves.usd`。**路徑基準點在不同版本文件裡不一致**(有的含 `/Isaac/` 前綴、有的不含),所以拼完路徑先驗證再開:

```python
from isaacsim.core.utils.nucleus import is_file   # 部分版本在 isaacsim.storage.native
print(is_file(scene_path))                        # False 就是路徑拼錯,調整前綴再試
```

## 3. 官方機器人放進場景 + 讀位姿

把官方 Nova Carter(輪式機器人)引用進目前 stage,跑幾步物理、讀它的世界位姿——這就是 [05 篇](../05-ros2-bridge/README.md)裡「回報 pose」鏈路最源頭的那個讀值:

```python
# ...接在 §2 open_stage 之後(或 §1 的空世界亦可)
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleArticulation

robot_usd = assets_root + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
add_reference_to_stage(usd_path=robot_usd, prim_path="/World/Carter")

robot = SingleArticulation("/World/Carter")
world.reset()                # 或確保 timeline 已 play,articulation 才能初始化
robot.initialize()

for i in range(120):
    world.step(render=True)
pos, quat = robot.get_world_pose()
print(f"Carter 位姿:{pos}, joint 數:{robot.num_dof}")
```

讀到位姿之後,[04 篇](../04-physics-world/README.md) §4 的兩條控制路徑(`set_joint_positions` 瞬移 / `apply_action` PD 目標)就都接得上了。

## 4. 這三個範例分別驗證了什麼

| 範例 | 驗證的事 | 對應篇 |
|---|---|---|
| 方塊落地 | Python 環境對、物理引擎在算 | 01、04 |
| 開倉庫場景 | 網路與 asset root 通、場景載入流程對 | 01 §2、03 |
| 機器人讀位姿 | articulation 初始化成功、可進入控制 | 04、05 |

依序跑,哪一步失敗就回對應篇查症狀表。三步都通,就具備接 [02 篇](../02-python-no-ui/README.md) UDP 通道或 [05 篇](../05-ros2-bridge/README.md) ROS2 橋接的基礎。

## 5. 延伸閱讀

- 官方 Core API Tutorial:Hello World(本篇 §1 的出處)
- 官方 repo `source/standalone_examples/api/isaacsim.simulation_app/`:`load_stage.py`、`livestream.py` 等(§2 的出處)
- 回到索引:[README](../../README.md)
