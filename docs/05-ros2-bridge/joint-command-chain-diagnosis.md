# 關節命令送不到:把驅動鏈路切成四段來查

> 症狀:ROS2 節點一路印「命令已送出」,Isaac 裡的關節卻文風不動,而且**沒有任何錯誤訊息**。
> 這篇講怎麼在四段之間定位斷點,不靠猜。

## 為什麼要切段

從「程式決定要抬升」到「關節真的動了」,中間有四段獨立的機制,任何一段斷掉都是同一個外觀 ——
命令看起來發了、關節沒反應、沒有錯誤:

```
①  控制器內部狀態          ②  ROS2 topic            ③  ActionGraph              ④  關節
   (它以為現在幾公分)   →   /joint_command      →   Subscribe → Controller  →   PhysX drive
```

單看任一端都會誤判。控制器印的「current=0.732」可能只是它自己記的數字;
關節停著也可能是命令根本沒出門。**必須兩端都量,而且要能單獨戳中間。**

## 第一性原理:開環控制器的 `current` 不是量測值

先分清一件事,否則後面全錯。控制器 log 印的

```
FORK_COMMAND current=(z2=0.732) target=(z2=1.147) feedback=disabled published=16.00s
```

裡的 `current`,在 `feedback=disabled` 時**是它自己維護的內部狀態**,不是從 `/joint_states`
讀回來的真值。開環控制器的職責只到「把命令發出去」,它沒有義務、也沒有能力知道關節有沒有動。

所以 `published=16.00s` 只證明「發了 16 秒」,不證明「關節收到」,更不證明「關節動了」。
**要判斷物理有沒有發生,唯一可信的是 `/joint_states` 或 PhysX 剛體位姿。**

## 四段的查法

### ④ 關節本身:繞過所有上游,直接戳

最有價值的一步,而且最快。自己發一則只帶目標關節的 `JointState`,看它動不動:

```python
import rclpy, time
from sensor_msgs.msg import JointState

rclpy.init()
n = rclpy.create_node("poke")
pub = n.create_publisher(JointState, "/joint_command", 10)
st = {"pos": None}

def cb(m):
    d = dict(zip(m.name, m.position))
    if "z2" in d:
        st["pos"] = d["z2"]

n.create_subscription(JointState, "/joint_states", cb, 10)
for _ in range(20):
    rclpy.spin_once(n, timeout_sec=0.1)
print("起始 =", st["pos"])

msg = JointState()
msg.name = ["z2"]
msg.position = [1.0]
t0 = time.time()
while time.time() - t0 < 20:
    msg.header.stamp = n.get_clock().now().to_msg()
    pub.publish(msg)
    rclpy.spin_once(n, timeout_sec=0.05)
    time.sleep(0.03)
print("20 秒後 =", st["pos"])
```

**要持續發**,不是發一次 —— `ROS2SubscribeJointState` 是每 tick 取最新值,單則訊息容易錯過 tick。

動了 → ③④ 都正常,斷點在 ①②,往上查。
不動 → 往下查 ③ 與關節的 drive 設定。

### ③ ActionGraph:節點在不在、接沒接、收到幾個關節

Isaac 端要有這條鏈:`ROS2SubscribeJointState` → `IsaacArticulationController`。
dump 出來確認三件事:

```
NODE .../SubscriberJointState [isaacsim.ros2.bridge.ROS2SubscribeJointState] disabled=False
  outputs:jointNames     = len=4      ← 最近一則命令帶了幾個關節
  outputs:positionCommand = len=4
NODE .../ArticulationController [isaacsim.core.nodes.IsaacArticulationController] disabled=False
  inputs:jointNames      <- [...SubscriberJointState.outputs:jointNames]
  inputs:positionCommand <- [...SubscriberJointState.outputs:positionCommand]
  inputs:execIn          <- [...OnPlaybackTick.outputs:tick]
```

- `disabled=False` —— 節點沒被關掉
- `inputs:xxx <- [...]` —— 連線真的接上了(沒接的話這裡是空的)
- `len=N` —— 最近收到的命令帶幾個關節。**這個數字會透露上游發了什麼**

`execIn` 接 `OnPlaybackTick` 而不是 subscriber 的 `execOut`,代表 controller 每 tick 都跑、
用 subscriber 的最新值 —— 所以 `execOut = 0` 不代表壞掉。

### ② topic:攔截實際送出的內容

同時訂閱 `/joint_command`(上游發什麼)與 `/joint_states`(關節怎麼回應),印出來對照:

```python
def ccb(m):
    print("CMD", {k: round(v, 3) for k, v in zip(m.name, m.position)})
```

比對控制器 log 宣稱的 target 與這裡實際看到的值。**兩者不一致就是 ① 的 bug。**

常見的錯位寫法:

```python
msg.name = controlled_joints
msg.position = [
    float(target_positions[self.joint_names.index(name)])   # ← 索引取自另一個清單
    for name in controlled_joints
]
```

`target_positions` 與 `joint_names` 的長度或順序一旦不一致,值就悄悄錯位到別的關節上 ——
name 對、值錯,而且完全不報錯。

### ① 控制器:只剩它了

前三段都正常就往這裡查。重點不是它印了什麼,是它**實際塞進 message 的值**。

## 兩個會讓你查錯方向的陷阱

**`ros2` CLI 可能整組壞掉。** 這台環境 `ros2 topic echo` / `ros2 topic info` 會噴
xmlrpc daemon 的 traceback,但 python 直接訂閱完全正常。CLI 不通不代表 topic 不通 ——
**別用 CLI 的失敗當證據**,換 python 再判斷。

**確認任務真的下發了,再解讀「零則命令」。** 監聽期間一則命令都沒有,可能是上游根本沒被觸發。
我就因為上游任務被業務邏輯擋掉(而且回傳的是描述性訊息、不是錯誤),
把「零則命令」誤讀成「控制器不發命令」,白繞一圈。
**先確認觸發成功,再讀監聽結果。**

## 一句話

四段獨立,外觀相同。**先戳最下游那段**(直接發 topic 給關節)——
一次測試就能把嫌疑範圍砍半,比從上游逐行讀程式快得多。
