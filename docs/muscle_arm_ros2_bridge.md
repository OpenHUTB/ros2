# ROS2 话题桥接（二连杆机械臂肌肉骨骼仿真）

> 项目类型：机器人课程设计 / 技术复盘
> 平台：Ubuntu 22.04（VMware 虚拟机）· 终端操作
> 前一份 MuJoCo 肌肉骨骼仿真（二连杆臂建模）见 PR #538；本文所用模型文件已随本 PR 提交为
> `docs/muscle_arm.xml`，无需另行获取。

在 MuJoCo 肌肉骨骼二连杆臂（`muscle_arm.xml`，4 条肌腱驱动肌肉）的基础上，
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
| 模型文件 | `docs/muscle_arm.xml`（随本 PR 提交） |

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
这样 `ros2 topic echo`、rviz 的关节/TF 显示、`robot_state_publisher` 都能直接订阅，
不需要额外写自定义消息或转换节点。`/joint_states` 里的关节名为 `sh_pitch`、`el_pitch`。

---

## 3 源码与模型

| 文件 | 说明 |
|------|------|
| `docs/muscle_ros_publisher.py` | ROS2 桥接节点源码（随本 PR 提交） |
| `docs/muscle_arm.xml` | MuJoCo 肌肉骨骼模型（MJCF，随本 PR 提交） |

本节点使用标准消息类型，因此**不需要 colcon 构建**，直接用 `python3` 运行即可；
（若希望用 `ros2 run` 启动，需要先把脚本放进一个 ROS2 功能包并 `colcon build`，
本作业为文档 + 单文件脚本，未提交功能包，故用 `python3` 直接运行。）

`docs/muscle_arm.xml` 的内容（4 条肌肉：`sh_flex`/`sh_ext`/`el_flex`/`el_ext`）：

```xml
<mujoco model="muscle_arm">
  <option timestep="0.002"/>
  <compiler angle="radian"/>

  <worldbody>
    <light diffuse="0.8 0.8 0.8" pos="0 -1 2" dir="0 1 -1"/>
    <geom name="ground" type="plane" size="1.5 1.5 0.1" rgba="0.9 0.9 0.9 1"/>
    <body name="shoulder" pos="0 0 0">
      <joint name="sh_pitch" type="hinge" axis="0 1 0" pos="0 0 0" range="-3.0 3.0"/>
      <geom name="upperarm" type="capsule" fromto="0 0 0 0.40 0 0" size="0.05" mass="2.0"/>
      <body name="elbow" pos="0.40 0 0">
        <joint name="el_pitch" type="hinge" axis="0 1 0" pos="0 0 0" range="-3.0 3.0"/>
        <geom name="forearm" type="capsule" fromto="0 0 0 0.35 0 0" size="0.04" mass="1.2"/>
        <geom name="hand" type="sphere" pos="0.35 0 0" size="0.06" mass="0.4"/>
      </body>
    </body>
  </worldbody>

  <tendon>
    <fixed name="t_sh_flex"><joint joint="sh_pitch" coef="-1"/></fixed>
    <fixed name="t_sh_ext" ><joint joint="sh_pitch" coef="1"/></fixed>
    <fixed name="t_el_flex"><joint joint="el_pitch" coef="-1"/></fixed>
    <fixed name="t_el_ext" ><joint joint="el_pitch" coef="1"/></fixed>
  </tendon>

  <actuator>
    <muscle name="sh_flex" tendon="t_sh_flex" force="1500" lengthrange="0.05 1.2"/>
    <muscle name="sh_ext"  tendon="t_sh_ext"  force="1500" lengthrange="0.05 1.2"/>
    <muscle name="el_flex" tendon="t_el_flex" force="500" lengthrange="0.05 1.2"/>
    <muscle name="el_ext"  tendon="t_el_ext"  force="500" lengthrange="0.05 1.2"/>
  </actuator>
</mujoco>
```

桥接节点完整代码（与 `docs/muscle_ros_publisher.py` 一致）：

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""muscle_ros_publisher.py

把 MuJoCo 肌肉骨骼二连杆臂（muscle_arm.xml，4 条肌腱驱动肌肉）的仿真状态
实时发布到 ROS2 话题：

  /joint_states    sensor_msgs/JointState      关节角度、角速度（标准话题）
  /muscle_state    std_msgs/Float64MultiArray  4 条肌肉的激活度
  /muscle_force    std_msgs/Float64MultiArray  4 条肌肉的执行器力

运行方式（不需 colcon 构建，全部使用标准消息类型）：

    python3 docs/muscle_ros_publisher.py

依赖：mujoco 3.2.3、rclpy（ROS2 Humble）、numpy
模型：docs/muscle_arm.xml（随本 PR 一起提交，与脚本放在同一目录）
"""

import os
import numpy as np
import mujoco
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

HERE = os.path.dirname(os.path.abspath(__file__))
XML_PATH = os.environ.get("MUSCLE_ARM_XML", os.path.join(HERE, "muscle_arm.xml"))
DT = 0.002  # 仿真步长 2 ms（名义 500 Hz）


class MuscleArmBridge(Node):
    def __init__(self):
        super().__init__("muscle_arm_bridge")
        if not os.path.isfile(XML_PATH):
            raise FileNotFoundError(
                "找不到模型文件 %s，请把 docs/muscle_arm.xml 放在脚本同一目录，"
                "或用环境变量 MUSCLE_ARM_XML 指定路径" % XML_PATH)
        self.model = mujoco.MjModel.from_xml_path(XML_PATH)
        self.data = mujoco.MjData(self.model)
        self.n_joint = self.model.njnt

        self.js_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.ms_pub = self.create_publisher(Float64MultiArray, "/muscle_state", 10)
        self.mf_pub = self.create_publisher(Float64MultiArray, "/muscle_force", 10)

        self.t = 0.0
        self.create_timer(DT, self.step)
        self.get_logger().info(
            "模型 %s 已加载：%d 个关节、%d 个肌肉执行器"
            % (os.path.basename(XML_PATH), self.model.njnt, self.model.nu))
        # 打印关节名，方便与 /joint_states.name 对照
        names = [self.model.joint(i).name for i in range(self.n_joint)]
        self.get_logger().info("关节名：%s" % ", ".join(names))

    def ctrl(self, t):
        """前 1.5 s 激励伸肌使手臂伸展，之后切到屈肌使手臂屈曲"""
        c = np.zeros(self.model.nu)
        if t < 1.5:
            c[1::2] = 0.8  # sh_ext / el_ext（伸肌）
        else:
            c[0::2] = 0.8  # sh_flex / el_flex（屈肌）
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

# 1. 确认模型与脚本都在（仓库里已经自带，无需自己再造模型）
ls docs/muscle_arm.xml docs/muscle_ros_publisher.py

# 2. 启动桥接节点（无需 colcon 构建）
python3 docs/muscle_ros_publisher.py
```

启动时终端会打印已加载的关节名，例如：

```
[INFO] [muscle_arm_bridge]: 模型 muscle_arm.xml 已加载：2 个关节、4 个肌肉执行器
[INFO] [muscle_arm_bridge]: 关节名：sh_pitch, el_pitch
```

另开一个终端（同样先 `source /opt/ros/humble/setup.bash`）：

```bash
# 3. 查看话题列表，应能看到 3 个话题
ros2 topic list

# 4. 查看关节状态（sh_pitch / el_pitch 的角度会随仿真持续变化）
ros2 topic echo /joint_states

# 5. 查看肌肉激活度（伸展 / 屈曲两个阶段交替变化）
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
- 脚本默认在同目录找 `muscle_arm.xml`，模型放在别处时用环境变量指定：
  `MUSCLE_ARM_XML=/path/to/muscle_arm.xml python3 docs/muscle_ros_publisher.py`。

---

## 7 验证结果

- `ros2 topic list` 中出现 `/joint_states`、`/muscle_state`、`/muscle_force` 三个话题；
- `ros2 topic echo /joint_states` 的 `sh_pitch`、`el_pitch` 角度值随仿真持续变化；
- `ros2 topic echo /muscle_state` 的激活度在伸展 / 屈曲两个阶段之间切换；
- 停止节点（Ctrl+C）后话题消失，节点退出正常。
