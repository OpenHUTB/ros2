# mujoco_car_follow

MuJoCo 双车跟车仿真：蓝色引导车在 **±3 m** 范围内往返前进、横向正弦摆动走出 S 形轨迹，
红色跟随车通过 **前馈 + PD** 控制自动调整位置，与引导车保持 **0.5 m 的真实车心距**跟随。

## 运行环境

- Ubuntu 20.04 及以上
- Python 3.8 及以上
- MuJoCo 2.3.7
- ROS 2 Humble（仅 ROS 2 启动方式需要）

## 安装依赖

```bash
pip install mujoco==2.3.7 numpy
```

ROS 2 方式另外需要 `ament_index_python`（Humble 桌面版默认已带）：

```bash
sudo apt install ros-humble-ament-index-python
```

## 方式一：直接运行（不依赖 ROS）

```bash
cd src/ground/mujoco_car_follow
pip install mujoco==2.3.7 numpy
python3 scripts/main.py
```

脚本会自己找到同包目录下的 `scene.xml`，无需设置环境变量。

## 方式二：ROS 2 编译运行

```bash
# 1. 把包放进 ROS 2 工作空间
mkdir -p ~/ros2_ws/src
cp -r src/ground/mujoco_car_follow ~/ros2_ws/src/

# 2. 编译
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mujoco_car_follow

# 3. 加载环境并启动
source install/setup.bash
ros2 launch mujoco_car_follow main.launch.py
```

`scene.xml` 会安装到 `share/mujoco_car_follow/`，脚本优先通过
`ament_index_python.get_package_share_directory()` 定位它。

## 无显示器环境

`mujoco.viewer.launch_passive` 需要窗口，纯服务器上可以用虚拟显示运行：

```bash
sudo apt install xvfb
xvfb-run -a python3 scripts/main.py
```

## 控制参数

| 参数 | 含义 | 默认值 |
| --- | --- | --- |
| `TARGET` | 目标纵向车心距（真实中心距） | 0.5 m |
| `LEAD_X_FORCE` | 引导车纵向推力 | 0.2 N |
| `LEAD_Y_AMP` / `LEAD_Y_FREQ` | 横向正弦振幅 / 角频率 | 0.3 m / 2.0 rad/s |
| `KP_X` / `KD_X` | 纵向 PD 增益 | 25.0 / 2.0 |
| `KP_Y` / `KD_Y` | 横向 PD 增益 | 15.0 / 1.5 |
| `FMAX` | 单电机输出限幅 | 1.0 N |
| `X_LIMIT` | 引导车往返边界 | ±3.0 m |
| `FF_RATE` | 前馈力每步最大变化量 | 0.02 N |
| `LOG_INTERVAL` | 日志打印间隔 | 0.5 s |

## 距离口径

`dx` 是**两车车身中心（车心）之间的纵向距离**，按两车 body 的世界坐标计算：

```
世界坐标 = body 初始位置 + 关节位移 qpos
dx = lead_x - follow_x
```

车身长 0.30 m，因此 0.5 m 车心距对应前保险杠间隙约 **0.20 m**。
横向目标是两车横向对齐（偏差 0），实际残差见运行统计输出。

## 控制原理

1. 引导车：`ctrl_x = 0.2 * direction`（到 ±3 m 折返），`ctrl_y = 0.3*cos(2t)`
2. 跟随车纵向：`ctrl = ff + KP_X*(dx - 0.5) - KD_X*v`
   - 前馈项 `ff` 跟随引导车当前推力并做变化率限幅，抵消两车相同的阻尼负载，消除匀速段稳态误差
   - PD 项负责收敛距离、抑制超调
3. 跟随车横向：`ctrl = ctrl_lead_y + KP_Y*dy - KD_Y*v`，同速跟踪正弦

## 运行输出

每 0.5 s 打印一行，避免每步都刷屏；关闭窗口后输出整段统计：

```
[t= 12.50s] 真实车心距=0.503 m  横向残差=+0.008 m | 引导(+1.230,+0.150) 跟随(+0.727,+0.142)

===== 统计（去掉前 3 s 过渡段） =====
纵向车心距：均值 0.497 m，最大偏差 0.053 m
横向残差：均值 +0.000 m，最大偏差 0.032 m
```

以上为 120 s 无头实测（`t=0.01 s`，含 ±3 m 折返瞬态）；匀速段偏差明显小于折返瞬间。

## AI 辅助声明

本模块的代码整理、文档撰写与调试排错在 **豆包（Doubao）** 辅助下完成；
控制方案设计、仿真模型结构由作者完成，作者对全部内容负责。
