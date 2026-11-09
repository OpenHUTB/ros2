# 悬停抗风能力核算（倾角 / 推重比余量 / 蒲福风级）

给定整机质量、阻力面积 $C_dA$ 与倾角上限，算清楚四件事：

1. 悬停时能扛多大的风（m/s，以及蒲福几级）；
2. 需要为姿态预留多少推重比（$T/W=1/\cos\theta$）；
3. 阵风再大一点时需要额外倾斜多少度；
4. 如果实际可用推重比达不到 $1/\cos\theta_{max}$，有效倾角会被 $ \arccos(1/(T/W))$ 截断到多少。

公式、倾角预算、敏感性与适用范围见[完整说明](../../../docs/air/uav_hover_wind_limit.md)。

## 环境与运行（离线）

Python 3.8 及以上，只需 NumPy 与 Matplotlib，不需要 ROS：

```bash
cd src/air/uav_hover_wind_limit
python3 -m pip install -r requirements.txt
python3 main.py --output outputs
python3 main.py --mass 3.0 --drag-area 0.20 --max-tilt 30 --altitude 1500
python3 main.py --tw 1.05          # 实际可用推重比只有 1.05 时
```

默认参数：2 kg 四旋翼、$C_dA=0.15\ \mathrm{m^2}$、倾角上限 25°、海平面标准大气。输出
`outputs/wind_vs_tilt.png`、`outputs/capability.png` 与 `outputs/report.json`（`outputs/`
已进 `.gitignore`）。

实测环境：Windows 11 / Python 3.12.14 / NumPy 2.3.5 / Matplotlib 3.11.2。控制台输出只用
`m/s`、`deg`、`m^2` 这类 ASCII 单位，并在入口把 stdout/stderr 的 `errors` 放宽为 `replace`，
Windows 默认 GBK 代码页下也不会抛 `UnicodeEncodeError`。

## 运行（ROS 2）

模块同时提供 `package.xml` / `setup.py` / `launch`，`colcon build` 后可以直接起节点：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2
colcon build --packages-select uav_hover_wind_limit
source install/setup.bash
ros2 launch uav_hover_wind_limit main.launch.py
ros2 topic echo /uav/wind_limit        # std_msgs/String, 内容与 report.json 相同
```

节点参数：`mass`、`drag_area`、`max_tilt_deg`、`altitude_m`、`available_thrust_to_weight`
（0 表示不提供）、`period_s`（发布周期）。发布用 transient local，晚加入的订阅者也能拿到。

## 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--mass` | 2.0 | 整机质量 kg |
| `--drag-area` | 0.15 | 阻力面积 $C_dA$，m² |
| `--max-tilt` | 25.0 | 倾角上限 deg，必须在 (0, 90) |
| `--altitude` | 0.0 | 海拔 m，用 ISA 对流层近似换算空气密度（-500~11000） |
| `--tw` | 无 | 实际可用推重比；给定时有效倾角取 $\min(\theta_{max},\arccos(1/TW))$ |
| `--output` | `outputs` | 输出目录 |

## 输出字段

`report.json`：`parameters`、`tilt_budget`（有效倾角 / 可达倾角 / 谁的限制 / 所需推重比）、
`weight_n`、`max_wind_m_s`、`max_wind_beaufort`、`thrust_to_weight`、`thrust_increase_percent`、
`tilt_sweep`（倾角-风速表）、`gust_test`（阵风增量与总倾角）、`notes`（海拔前提等说明）。

## 测试

```bash
python3 -m pip install -r requirements-dev.txt
python3 -m pytest test -v      # 19 项
```

覆盖 $\tan\theta=D/(mg)$ 恒等式、倾角与风速互逆、$V(\theta)$ 闭式解、$T/W=1/\cos\theta$、
质量与阻力面积缩放律、海拔对密度与抗风的影响、推重比耦合（$T/W=1.05$ 时有效倾角 17.75°、
抗风 8.27 m/s）、`tilt_budget` 字段、蒲福风级边界、阵风附加倾角、参数校验、GBK 打印、
`main.py` 冒烟，以及用假 `rclpy` 替身验证 ROS 节点的「参数 → 计算 → 发布 JSON」接线。

## 适用范围

准静态模型：只算气动阻力与姿态倾角，不含姿态环动态、电机响应、旋翼入流遮挡、机体升力与
风切变；$C_dA$ 与实际可用推重比需要实测或厂家数据。结论用于选型与作业窗口评估，不代替
整机气动仿真或试飞。ROS 节点未在本机运行（本机无 ROS），`colcon build` / launch / 话题
收发见 `validation/ros_checklist.md`。

## 使用声明

使用 DeepSeek Harness（deepseek-flash）辅助实现、测试与文档；作者对提交内容负责。
