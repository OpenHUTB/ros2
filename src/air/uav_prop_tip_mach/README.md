# 旋翼桨尖马赫数与桨盘载荷核算

选桨时两个最容易忽略、又最影响噪声与效率的量：**桨尖马赫数**（桨尖会不会提前进入压缩性）和
**桨盘载荷**（悬停诱导速度与功率的大小）。本模块用三个课本公式一次算清楚，**只依赖标准库**，
不需要 NumPy、Matplotlib 或 ROS。

公式、阈值与适用范围见[完整说明](../../../docs/air/uav_prop_tip_mach.md)。

## 环境与运行

Python 3.8 及以上，运行时不依赖任何第三方库：

```bash
cd src/air/uav_prop_tip_mach
python3 main.py --output outputs
python3 main.py --mass 1.2 --diameter 0.127 --rotor-count 4 --hover-rpm 12000 --max-rpm 16000
```

默认参数：2 kg 四旋翼、单桨直径 0.254 m、悬停 6500 rpm、最大 9000 rpm、15 ℃。输出
`outputs/report.json`（`outputs/` 已进 `.gitignore`）。

实测环境：Windows 11 / Python 3.12.14。控制台只用 `m/s`、`rpm`、`m^2` 这类 ASCII 单位，并在入口
把 stdout/stderr 的 `errors` 放宽为 `replace`，Windows 默认 GBK 代码页下不会抛 `UnicodeEncodeError`。

## 参数

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--mass` | 2.0 | 整机质量 kg |
| `--diameter` | 0.254 | 单桨直径 m |
| `--rotor-count` | 4 | 旋翼数量 |
| `--hover-rpm` | 6500 | 悬停转速 rpm |
| `--max-rpm` | 9000 | 最大转速 rpm |
| `--temperature` | 15 | 环境温度 ℃，影响声速 |
| `--output` | `outputs` | 输出目录 |

## 输出字段

`report.json`：`parameters`、`speed_of_sound_m_s`、`weight_n`、`thrust_per_rotor_n`、
`total_disk_area_m2`、`tip_speed_hover_m_s`、`tip_mach_hover`、`tip_mach_max`、`tip_mach_level`、
`disk_loading_n_m2`、`disk_loading_kg_m2`、`disk_loading_level`、`induced_velocity_m_s`、
`limits`（马赫 0.6 / 0.7 对应转速、桨盘载荷 100 N/m² 对应桨径）与 `notes`。

## 测试

```bash
python3 -m pip install -r requirements-dev.txt
python3 -m pytest test -v      # 13 项
```

覆盖声速、桨尖速度与马赫数、桨盘面积与载荷的缩放关系、诱导速度、三级阈值、两个反解与正解互逆、
report 自洽、参数校验、GBK 打印与 `main.py` 冒烟。

## ROS 2 说明

本模块是纯离线计算工具，不参与飞行控制，因此**没有 rclpy 节点**；按仓库约定提供了一个
`launch/main.launch.py`（用 `ExecuteProcess` 直接跑安装后的命令行入口）以及 `package.xml` /
`setup.py`，`colcon build` 后可以 `ros2 launch uav_prop_tip_mach main.launch.py`。
如果维护者希望它变成一个发布话题的节点，我可以另开 PR 补（计算已在 `core` 里，节点只是外壳）。

## 使用声明

使用 DeepSeek Harness（deepseek-flash）辅助实现、测试与文档；作者对提交内容负责。
