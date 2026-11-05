# humanoid_human_following

人形机器人人体目标跟随：自建简化双足人形在 MuJoCo 场景中实时检测并跟随一个移动目标（红色球体），把目标保持在设定距离内，目标移动 → 转向前进 → 追近减速 → 目标停下机器人停稳。

这是 `waypoint_publisher`（静态路径点）的自然进阶——把「静态路径」升级为「实时更新目标的动态跟随」，复用感知 + 规划 + 控制三段闭环。

## 运行环境

- 操作系统：Ubuntu 22.04（虚拟机）
- ROS 版本：ROS 2 Humble
- 依赖：`mujoco>=3.0`、`numpy`（动图录制另需 `Pillow`、`imageio`）

## standalone（无需 ROS，最快验证）

```
cd src/humanoid/humanoid_human_following
pip3 install -r requirements.txt
python3 main.py                   # 弹出可视化窗口，红球走 L 形，人形跟随
python3 main.py --headless        # 无窗口
python3 main.py --gif             # 录制演示动图 doc/humanoid_human_following.gif
python3 main.py --test            # 自动化测试
```

## ROS2 Humble

```
cd ~/ros2_ws/src
cp -r src/humanoid/humanoid_human_following ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --symlink-install --packages-select humanoid_human_following
source install/setup.bash
ros2 launch humanoid_human_following main.launch.py
```

查看话题：

```
ros2 topic echo /humanoid/follow/target
ros2 topic echo /humanoid/follow/status
ros2 topic echo /humanoid/odom
```

## 核心设计

- **距离保持**：不「追到底」，而是通过比例控制把距离收敛到期望值 `D_KEEP=0.8 m`，避免机器人贴上目标球。
- **运动学驱动**：直接覆盖根位置与朝向（每周期覆盖抵消重力），机器人稳定跟随，不依赖真实步态。
- **感知为理想化**：直接读取目标红球世界坐标；如需真实感知可接 `humanoid_lidar_nn_perception` 的目标检测输出。

## 能力边界

- 不含真实步态与平衡控制，只验证「跟随」逻辑。
- 场景无障碍物，避障不在本模块范围。
- 目标感知为理想化，速度未加平滑（建议实际部署加低通）。

## 目录

```
src/humanoid/humanoid_human_following/
 ├── humanoid_human_following/
 │   ├── __init__.py
 │   ├── sim_core.py            # 控制器核心（standalone 与 ROS 共用）
 │   └── mujoco_sim_node.py     # ROS2 仿真节点
 ├── config/params.yaml
 ├── launch/main.launch.py
 ├── models/humanoid_following.xml
 ├── resource/humanoid_human_following
 ├── test/test_standalone.py
 ├── main.py                    # standalone 入口（含 gif/测试）
 ├── package.xml
 ├── requirements.txt
 ├── setup.cfg
 └── setup.py
```
