# ROS2 话题桥接（二连杆机械臂肌肉骨骼仿真）

> 项目类型：机器人课程设计 / 技术复盘
> 平台：Ubuntu 22.04（VMware 虚拟机）· 终端操作
> 上一篇：[二连杆机械臂肌肉骨骼建模与 MuJoCo 仿真](./muscle_arm_mujoco_simulation.md)

在上一篇 MuJoCo 肌肉骨骼二连杆臂（`muscle_arm.xml`，4 条肌腱驱动肌肉）的基础上，
用 ROS2（Humble）把仿真状态发布成标准话题，让 rviz / robot_state_publisher 等
ROS2 标准工具可以直接订阅。

---

## 1 依赖环境

| 类别 | 版本 / 说明 |
|------|-------------|
| 操作系统 | Ubuntu 22.04（VMware 虚拟机） |
| ROS2 | Humble（`/opt/ros/humble`） |
| 物理引擎 | mujoco 3.2.3（`pip install mujoco`，清华源） |
| Python | 3.10（系统自带 `python3`） |
| 模型文件 | `muscle_arm.xml`（同上一篇文档的模型） |

> 排错记录：conda 环境里的 Python 3.14 与 `rclpy` 不兼容（`import rclpy` 直接失败），
> 因此本节点改用系统自带的 `python3`（3.10）运行，conda 环境只用来跑纯 MuJoCo / 绘图脚本。

---

## 2 话题设计

| 话题 | 消息类型 | 方向 | 内容 |
|------|----------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 发布 | 两个关节的角度、角速度（**ROS2 标准话题**） |
| `/muscle_state` | `std_msgs/Float64MultiArray` | 发布 | 4 条肌肉的激活度（0~1） |
| `/muscle_force` | `std_msgs/Float64MultiArray` | 发布 | 4 条肌肉的执行器力（N） |

关节状态特意使用标准话题名 `/joint_states`、标准消息 `sensor_msgs/JointState`，
这样 `ros2 topic echo`、rviz 的 TF/关节显示、`robot_state_publisher` 都能直接订阅，
不需要再写自定义消息或额外转换节点。

---

## 3 源码

桥接节点源码随文档一起提交：**`docs/muscle_ros_publisher.py`**。

本节点使用标准消息类型，因此**不需要 colcon 构建**，直接用 `python3` 运行即可；
（如果希望用 `ros2 run` 启动，需要先把脚本放进一个 ROS2 功能包并 `colcon build`，
本作业为文档 + 单文件脚本，未提交功能包，故用 `python3` 直接运行。）

完整代码：

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""muscle_ros_publisher.py

把 MuJoCo 肌肉骨骼二连杆臂（muscle_arm.xml，4 条肌腱驱动肌肉）的仿真状态
实时发布到 ROS2 话题：

  /joint_states    sensor_msgs/JointState      关节角度、角速度（标准话题）
  /muscle_state    std_msgs/Float64MultiArray  4 条肌肉的激活度
  /muscle_force    std_msgs/Float64MultiArray  4 条肌肉的执行器力

运行：python3 muscle_ros_publisher.py
依赖：mujoco 3.2.3、rclpy（ROS2 Humble）、numpy
"""

import numpy as np
import mujoco
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

XML_PATH = "muscle_arm.xml"
DT = 0.002  # 仿真步长 2 ms（名义 500 Hz）


class MuscleArmBridge(Node):
    def __init__(self):
        super().__init__("muscle_arm_bridge")
        self.model = mujoco.MjModel.from_xml_path(XML_PATH)
        self.data = mujoco.MjData(self.model)
        self.n_joint = self.model.njnt

        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.ms_pub = self.create_publisher(Float64MultiArray, "/muscle_state", 10)
        self.mf_pub = self.create_publisher(Float64MultiArray, "/muscle_force", 10)

        self.t = 0.0
        self.create_timer(DT, self.step)
        self.get_logger().info(
            "muscle_arm.xml 已加载：%d 个关节、%d 个肌肉执行器"
            % (self.model.njnt, self.model.nu))

    def ctrl(self, t):
        """前 1.5 s 激励伸肌使手臂伸展，之后切到屈肌使手臂屈曲"""
        c = np.zeros(self.model.nu)
        if t < 1.5:
            c[1::2] = 0.8  # 伸肌
        else:
            c[0::2] = 0.8  # 屈肌
        return c

    def step(self):
        self.data.ctrl[:] = self.ctrl(self.t)
        mujoco.mj_step(self.model, self.data)
        self.t += DT

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.model.joint(i).name for i in range(self.n_joint)]
        js.position = [
            self.data.qpos[self.model.jnt_qposadr[i]] for i in range(self.n_joint)]
        js.velocity = [
            self.data.qvel[self.model.jnt_dofadr[i]] for i in range(self.n_joint)]
        self.js_pub.publish(js)

        act = Float64MultiArray()
        act.data = [float(self.data.act[i]) for i in range(self.model.na)]
        self.ms_pub.publish(act)

        force = Float64MultiArray()
        force.data = [float(v) for v in self.data.actuator_force]
        self.mf_pub.publish(force)


def main():
    rclpy.init()
    node = MuscleArmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

---

## 4 运行步骤

```bash
# 0. 载入 ROS2 环境
source /opt/ros/humble/setup.bash

# 1. 准备模型与脚本（把上一篇的 muscle_arm.xml 放在同一目录）
ls muscle_arm.xml muscle_ros_publisher.py

# 2. 启动桥接节点（无需 colcon 构建）
python3 muscle_ros_publisher.py
```

另开一个终端（同样先 `source /opt/ros/humble/setup.bash`）：

```bash
# 3. 查看话题列表，应能看到 3 个话题
ros2 topic list

# 4. 查看关节状态（value 会随仿真持续变化）
ros2 topic echo /joint_states

# 5. 查看肌肉激活度（伸展阶段接近 0.6 与 0.45，切换后回到 0，交替变化）
ros2 topic echo /muscle_state

# 6. 查看发布频率（发布在仿真步进后同步触发，DT = 0.002 s）
ros2 topic hz /joint_states
```

---

## 5 发布频率与延迟说明

- 话题发布挂在仿真步进上（每 `mj_step` 之后发一次），仿真步长 `DT = 0.002 s`，
  因此名义上对应 500 Hz 的发布节拍。
- 实际频率受机器性能、Python 回调开销影响，**请用 `ros2 topic hz` 在本机实测**，
  本文不给出未经测量的延迟数字。

---

## 6 注意事项

- **通信范围**：本节点默认在本机通过**环回地址（loopback）**通信，不涉及跨机 / 局域网场景；
  多终端同时运行只要 `ROS_DOMAIN_ID` 一致即可互相发现，`ROS_LOCALHOST_ONLY=1` 也能正常工作。
- 首次运行前务必 `source /opt/ros/humble/setup.bash`，否则 `ros2 topic` 相关命令不可用。
- 若 `import rclpy` 失败，先确认当前 `python3` 是系统 3.10 而不是 conda 的 3.14。
- 节点与 `muscle_arm.xml` 必须放在同一工作目录，否则 `from_xml_path` 会报找不到文件。

---

## 7 验证结果

- `ros2 topic list` 中出现 `/joint_states`、`/muscle_state`、`/muscle_force` 三个话题；
- `ros2 topic echo /joint_states` 的角度值随仿真持续变化；
- `ros2 topic echo /muscle_state` 的激活度在伸展 / 屈曲两个阶段之间切换；
- 停止节点（Ctrl+C）后话题消失，节点退出正常。

| 文件 | 说明 |
|------|------|
| `docs/muscle_ros_publisher.py` | ROS2 桥接节点源码 |
| `muscle_arm.xml` | MuJoCo 肌肉骨骼模型（见上一篇文档） |
