# humanoid_keyboard_control

人形机器人（双足）MuJoCo 物理仿真与键盘运动控制 ROS2 功能包。

对应课程任务：**① 人形机器人 / 肌肉骨骼人 / 车辆 / 水下机器人 / 无人机物理仿真 + 键盘运动控制**。

## 功能概述

- 自建简化双足人形 MJCF 模型（15 关节 + 髋/踝 roll 抗侧倾加强），包含前视相机与场景障碍物（红柱/蓝块）；
- **站姿控制**：全关节 PD 伺服回直立位，4 s 站立质心零漂移（仿真 3 万+ step/s）；
- **运动控制**：
  - 前进/后退 —— 速度级控制（按偏航角将指令速度分解到世界系 vx/vy，沿机器人当前朝向移动，位移 ≈ v×t）；
  - 转向 —— 姿态级控制（直接旋转底盘四元数），避免双足原地旋转脚底摩擦滑移导致的侧倾失稳；
- 发布 `/humanoid/odom`、`/joint_states`、`/tf`（标准 `tf2_ros.TransformBroadcaster`），订阅 `/cmd_vel`；
- 无 ROS 独立运行（`main.py` 键盘控制 / 自动演示），与 ROS2 节点共用同一仿真核心 `sim_core.py`。

> **能力边界**：移动控制为演示级运动学控制（直接改写 freejoint 速度与根部四元数），
> 不包含双足步态；位移/转角指标仅验证指令执行准确性，不代表真实步行控制或
> 动力学准确性，与静态站姿的物理稳定性分开评价（详见仓库 docs 文档 4.5 节）。

## 环境要求

- Ubuntu 22.04 + ROS2 Humble（或 Windows + Python 直接运行 standalone 模式）
- Python 依赖安装：
  ```bash
  pip3 install -r requirements.txt        # mujoco、numpy；录制 GIF 另需 imageio
  ```

## 运行方式

### 1. standalone（无需 ROS，最快验证）

```bash
python3 main.py            # 交互：W/S 前进后退，A/D 转向，空格急停，ESC 退出（弹出可视化窗口）
python3 main.py --no-viewer # 无图形环境：仅命令行输出
python3 main.py --demo     # 自动演示动作序列，输出 demo_assets/humanoid_demo.gif
python3 main.py --test     # 自动化测试（站姿/驱动/转向/渲染/性能）
```

> 交互模式与演示模式默认打开 **MuJoCo 可视化窗口**，可直接看到人形机器人在场景中
> 站立、前进与转向。窗口需要图形界面：SSH 远程请用 `ssh -X` 转发 X11；
> 无图形环境加 `--no-viewer`（自动退化为无窗口运行，仿真与测试不受影响）。

### 2. ROS2 launch（Humble）

```bash
# 源码放入工作空间
mkdir -p ~/ros2_ws/src
cp -r <本包路径> ~/ros2_ws/src/

# 构建并启动
cd ~/ros2_ws && colcon build --symlink-install --packages-select humanoid_keyboard_control
source install/setup.bash
ros2 launch humanoid_keyboard_control main.launch.py

# 无图形界面时关闭可视化窗口
ros2 launch humanoid_keyboard_control main.launch.py visualize:=false
```

启动后在本终端按键盘控制，另开终端可用：

```bash
ros2 topic echo /humanoid/odom      # 查看里程计
ros2 topic echo /joint_states       # 查看关节状态
ros2 run tf2_ros tf2_echo world humanoid/base_link   # 验证 TF 可读取
```

## 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/cmd_vel` | geometry_msgs/Twist | 速度指令（linear.x 前进，angular.z 转向） |
| `/humanoid/odom` | nav_msgs/Odometry | 里程计（world 系位姿与速度） |
| `/joint_states` | sensor_msgs/JointState | 15 个人形关节位置 |
| `/tf` | tf2_msgs/TFMessage | world → humanoid/base_link（TransformBroadcaster 发布） |

## 文件结构

```
humanoid_keyboard_control/
├── humanoid_keyboard_control/
│   ├── sim_core.py              # 仿真核心（模型加载/站姿PD/运动控制/渲染，无 ROS）
│   ├── mujoco_sim_node.py       # ROS2 仿真节点
│   └── keyboard_teleop_node.py  # ROS2 键盘遥操作节点
├── models/humanoid_simple.xml   # 自建双足人形 MJCF 模型
├── launch/main.launch.py        # launch 启动
├── config/params.yaml           # 仿真与控制参数
├── test/test_standalone.py      # 自动化测试
├── main.py / main.sh            # 模块主入口
└── README.md
```

## AI 声明

- 自建人形 MJCF 模型、站姿 PD 与运动控制策略、键盘遥操作节点均为本作业原创实现；
- 参考了 MuJoCo 官方文档（freejoint / motor / Renderer 用法）与仓库内已有模块 `rov_mujoco` 的 ROS2 包结构惯例。
