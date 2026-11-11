# drone_charging_nest

基于 **MuJoCo** 的无人机自主充电/机巢仿真模块（ROS 2 ament_python 包）：四旋翼从机巢起飞，
飞往巡检点，再返航降落到充电机巢，自动充电至设定电量后再次起飞，最后降落完成。

## 模块组成

| 文件 | 说明 |
|---|---|
| `drone_charging_nest/app.py` | 仿真入口：四旋翼动力学、级联 PID、任务状态机、HUD 与双相机录制 |
| `drone_charging_nest/ros_node.py` | ROS 2 节点入口（rclpy 生命周期内运行演示） |
| `main.py` | 兼容入口：源码目录可直接 `python main.py` |
| `models/charging_nest.xml` | MuJoCo 场景：四旋翼、充电机巢、巡检目标、主相机/机巢相机 |
| `launch/main.launch.py` | ROS 2 launch：启动 `drone_charging_nest` 可执行文件 |
| `resource/drone_charging_nest` | ament 资源索引标记 |
| `test/test_packaging.py` | 包导入与模型文件存在性测试 |
| `scripts/make_gif.py` | 渲染帧 → GIF / MP4 |
| `scripts/plot_mission.py` | 由 `mission_log.json` 绘制高度/电量任务曲线 |
| `scripts/record_demo.sh` | 一键录制 + 生成截图/GIF/曲线 |
| `package.xml` / `setup.py` / `setup.cfg` | ament_python 包配置 |

## 环境要求

- Ubuntu 20.04（VMware 虚拟机，4 vCPU / 4 GB RAM，无 GPU 均可）
- Python 3.8
- MuJoCo 2.3.7（CPU 版）、Pillow、Matplotlib
- 渲染：MuJoCo GLFW 后端 + Mesa 软件渲染（需要图形会话）
- ROS 2 环境（`colcon build` / `ros2 launch` 需要；本地 VM 为 ROS Noetic，Python 入口已实测）

## 安装

### 方式 1：源码 / venv（本 VM 实测方式）

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

### 方式 2：ROS 2 工作区（ament_python）

```bash
source /opt/ros/humble/setup.bash   # 按实际 ROS 2 发行版
colcon build --symlink-install --packages-select drone_charging_nest
source install/setup.bash
```

> `ros2 launch` 使用 ROS 2 环境的 `python3` 执行安装入口，该解释器也需要能
> `import mujoco / PIL`：请给同一解释器安装依赖（如
> `python3 -m pip install --user -r requirements.txt`），或把 ROS 2 环境建在同一 venv 中。

## 运行

```bash
# 源码方式：仅仿真
python3 -m drone_charging_nest.app

# 兼容入口
python main.py

# ROS 2 launch
ros2 launch drone_charging_nest main.launch.py

# 录制：640x480 主视图 + 机巢相机画中画 + HUD
bash scripts/record_demo.sh /tmp/dc_frames
# 产物：results/01_standby.png ... 08_complete.png、demo_charging_nest.gif/.mp4、mission_curve.png
```

> 渲染环境变量（虚拟机桌面）：`DISPLAY=:0`、`XAUTHORITY`（按当前 UID 自动查找）、
> `MUJOCO_GL=glfw`。

## 任务流程

```
STANDBY(1s) → TAKEOFF(2s) → CRUISE(3s) → INSPECT(1.2s) → RETURN(3s)
→ LANDING(2.6s) → CHARGING(7s) → TAKEOFF2(1.6s) → LAND2(2.2s) → DONE(1.2s)
```

- 机巢位置 (0, 0)，巡检目标 (1.75, 1.15)，巡航高度 1.15 m；
- 飞行中电量按 1.8 %/s 下降；降落后进入 CHARGING，电量按 2.6 %/s 回升，充电至 95% 截止；
- HUD 实时显示任务状态、电量条、高度与水平位置；机巢相机画中画观察降落/充电过程。

## 实现说明

1. **四旋翼模型**：自建 MuJoCo 刚体（机身、机臂、4 旋翼、起落架、free joint），
   旋翼推力通过 `data.xfrc_applied` 在四个旋翼位置合成总力/力矩施加。
2. **控制**：位置环 PID 生成期望推力向量 → 由推力向量反解期望姿态 →
   姿态误差 PD 生成期望力矩 → 4 旋翼推力分配（含偏航阻尼项）。
3. **充电机巢**：静态机巢 + 绿色环形指示灯 + 十字对准标记；电池/充电为演示模型。
4. **录制**：主视图 + 机巢相机两路渲染，PIL 叠加 HUD 后保存帧序列；GIF/MP4 与
   任务曲线由脚本生成。

## 实测结果（VMware，无 GPU）

| 指标 | 数值 |
|---|---|
| 任务时长 | 24.8 s |
| 巡航高度 | 1.15 m（INSPECT/RETURN 实测 1.1485~1.1522 m，跨度 0.0037 m） |
| 巡检点悬停位置 | 均值 (1.807, 1.187) m（相对目标 (1.75,1.15) 偏差约 (0.057,0.037) m），std (0.037,0.024) m |
| 降落位置误差 | < 0.002 m（机巢中心） |
| 电量 | 起飞 100% → 降落 78.8% → 充电 95% → 结束 88.2% |
| 渲染 | 20 fps，496 帧；640×480 主视图 + 机巢画中画 |
| GIF 大小 | 约 2.2 MB（< 5 MB） |

## 已知限制

- 四旋翼为简化动力学模型（无螺旋桨气动、无真实电调/电池模型）；
- 机巢充电为状态机模拟，不包含真实触点对齐、接触电阻或充电电路；
- 未实现真实 ROS 话题/服务（节点仅作为 launch 生命周期入口）；
- 场景、巡检目标与机巢布局为演示自定义。

## 大模型使用声明

本模块代码、参数调试与文档由 AI 助手辅助完成；所有仿真、截图、GIF、曲线与数据均在
Ubuntu 20.04 + MuJoCo 2.3.7（CPU）环境中实际运行生成，作者已核对并对内容负责。
