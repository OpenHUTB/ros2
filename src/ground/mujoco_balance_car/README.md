# mujoco_balance_car —— MuJoCo 两轮自平衡小车（双环 PID）

> 机器人操作系统及应用 · 地面载具选题
>
> 基于 MuJoCo 物理引擎的两轮倒立摆小车：**位置环 → 期望倾角 → 倾角环 → 电机力矩**
> 的双环级联 PID。核心特点不只是"能站起来"，而是**把控制品质变成可复算的指标**
> （稳定时间 / 超调 / 稳态误差 / 抗扰恢复 / 可恢复倾角边界）。

## 1. 与仓库已有模块的区别

| 模块 | 物理引擎 | 控制算法 | 是否有离线指标 | 是否需要 ROS |
|---|---|---|---|---|
| `balance_bot`（两轮自平衡机器人） | MuJoCo | 倾角 PD + MLP 策略克隆 | 是（另有指标口径） | 是（rclpy 节点） |
| **本模块** | **MuJoCo** | **双环级联 PID（位置 + 倾角）** | **是（见 §5）** | **否**（`launch` 只是多一种启动方式） |
| `mujoco_car_avoidance` | MuJoCo | 射线感知 + 分级减速 | 是 | 是 |

本模块的定位：**用最小依赖（mujoco + numpy）复现经典倒立摆 PID 控制，并给出可复算的
控制品质指标**；`--gui` 只是可选的可视化外壳，评测链路完全不依赖图形界面。

## 2. 目录结构

```
src/ground/mujoco_balance_car/
├── main.py                        # 课程约定入口（薄壳，转发到包内 cli）
├── main.sh / main.bat             # 一键运行脚本
├── balance_car.xml                # MuJoCo 模型（底盘 + 两轮 + 摆杆）
├── pid_controller.py              # 向后兼容 shim（老用法 import pid_controller 仍可用）
├── mujoco_balance_car/
│   ├── cli.py                     # 命令行入口：headless / gui / selftest / launch
│   ├── config.py                  # PID 与场景参数（集中一处便于调参）
│   ├── env.py                     # 模型封装：复位、步进、扰动、状态读取
│   ├── pid.py                     # PID（积分限幅、微分对测量值）
│   ├── controller.py              # 双环级联控制
│   ├── metrics.py                 # 稳定时间 / 超调 / 稳态误差（纯函数，可单测）
│   ├── runner.py                  # 三个场景 + 倾角扫描 + CSV 导出
│   └── plotting.py                # matplotlib 可选出图
├── launch/main.launch.py          # ros2 launch 入口
├── test/test_balance_logic.py     # 单元测试（22 项；无 mujoco 时跳过仿真部分）
├── package.xml / setup.py / setup.cfg / resource/
└── requirements.txt
```

## 3. 运行环境（实测）

| 组件 | 实测版本 / 说明 |
|---|---|
| 操作系统 | Windows 11 原生（Linux/macOS 同样可用；`main.sh` 已做 Python 探测） |
| Python | 3.12.14 |
| MuJoCo | **3.2.7**（见下方"为什么锁定版本"） |
| numpy | 2.3.5 |
| matplotlib | 3.11.2（**可选**，只用于出图；缺失时只写 CSV，退出码仍为 0） |
| 图形界面 | **不需要**（默认 `--headless`）；`--gui` 才需要 `mujoco-python-viewer` + `glfw` |
| ROS | 不需要；`ros2 launch` 是可选启动方式 |

> **为什么锁定 MuJoCo 3.2.7**：本机开启了 Windows Smart App Control，`mujoco 3.17.0`
> 的 `mujoco.dll` 被应用程序控制策略拦截（`OSError: [WinError 4551]`），
> 而 3.1.6 / 3.2.7 可正常加载。因此 `requirements.txt` 写的是 `mujoco>=3.0.0`，
> 实测与文档口径统一用 **3.2.7**。若你的机器上 3.17 能加载，也可直接用新版本
> （本模块没有用到 3.17 的新 API）。

## 4. 安装与运行

```bash
cd src/ground/mujoco_balance_car
pip install -r requirements.txt          # mujoco + numpy（必需）
pip install matplotlib                   # 可选：需要 PNG 曲线时
```

**一键运行（默认离线批跑，导出 CSV/PNG）**

```bash
python3 main.py                          # 等价于 --headless；输出到 ./shots
python3 main.py --save_dir shots --duration 8 --lean 0.1
bash main.sh                             # Linux/macOS 包装脚本
main.bat                                 # Windows 包装脚本
```

**其它入口**

```bash
python3 main.py --selftest               # 22 项单元测试（约 1 s）
python3 main.py --gui                    # 可视化窗口（R 复位 / ↑↓ 扰动 / ESC 退出）
ros2 launch mujoco_balance_car main.launch.py duration:=8 lean:=0.15
```

产物（默认 `shots/`）：

| 文件 | 内容 |
|---|---|
| `balance_metrics.csv` | 全部场景指标（一行一个 metric，可直接复算/对比） |
| `balance_series.csv` / `position_step_series.csv` / `disturbance_series.csv` | 时间序列：`t, angle, x, target_lean, torque` |
| `balance_responses.png` | 三场景的倾角与位置曲线 |
| `lean_sweep.png` | 初始倾角 → 稳定时间（失败档用红带标出） |

## 5. 实测指标（本机实测，2026-11，60 s 数据可复算）

口径：判据带宽取 **倾角 0.01 rad（0.57°）、位置 0.02 m**；数据由 `balance_metrics.csv`
与各 `*_series.csv` 复算，脚本不依赖随机数（MuJoCo 定步长、无随机扰动）。

| 场景 | 指标 | 数值 |
|---|---|---|
| 平衡恢复（初始 0.05 rad / 2.9°） | 稳定时间 | **0.194 s** |
| | 最大倾角 | 0.050 rad（即初值，未超调） |
| | 末态倾角 / 位置误差 | −1.1e−5 rad / 1.2e−5 m |
| 位置阶跃 1.0 m（第 1 s 起） | 到位时间 | **3.752 s** |
| | 超调 | **3.23%** |
| | 稳态位置误差 | 0.004 m |
| | 过程中最大倾角 | 0.192 rad |
| 速度扰动 1.5 m/s（第 2 s 推一把） | 最大位移偏离 | 0.706 m |
| | 位置恢复时间 | **3.86 s** |
| | 倾角恢复时间 | 3.37 s |
| | 过程最大倾角 | 0.253 rad |
| 可恢复初始倾角 | 成功率 | **8/8（0.02~0.30 rad，即 1.1°~17.2°）** |
| | 稳定时间范围 | 0.036 ~ 0.996 s（随初始倾角近似线性） |

![三场景闭环响应](../../docs/img/ground/mujoco_balance_car_responses.png)

![初始倾角扫描](../../docs/img/ground/mujoco_balance_car_lean_sweep.png)

## 6. 调参记录（为什么不是仓库里原来那组 12 / 3.5）

| 阶段 | 现象 | 原因与处理 |
|---|---|---|
| 原参数 `kp_angle=12, kd=3.5, kp_pos=0.5, kd_pos=0.1` | 平衡场景 **不收敛**：倾角停在 −0.086 rad，位置缓慢振荡 | 只闭合了倾角环，外环增益与内环不匹配；`a_s` 为 nan |
| 加大外环 `kp_pos` 到 2~8 | 出现**周期约 10 s、幅度逐渐增大**的慢振荡 | 位置控制是非最小相位系统（想后退要先向前推），外环必须显著慢于内环 |
| 内环微分改为**对测量值求导** | 振荡消失 | 原实现对"误差"求导，外环目标倾角的变化率混进微分项，产生力矩尖峰 |
| 四维增益网格搜索（等效全状态 PD） | 找到 `kθ=30, kω=3.5, kx=6, kv=6` | 展开式 `ctrl = kθ·θ + kω·θ̇ + kx·(x−x*) + kv·ẋ`，反解为 `kp_angle=30, kd_angle=3.5, kp_pos=0.2, kd_pos=0.2` |

> 关键认识：级联 PID **代数上等价于全状态 PD**（推导见 `config.py` 注释），
> 所以调参不必在 `(kp_pos, kd_pos)` 上盲试——直接对 `(kθ, kω, kx, kv)` 搜索，
> 再按 `kp_pos = kx/kθ`、`kd_pos = kv/kθ` 反解回级联形式即可。

## 7. 常见问题

**Q1：`pip install mujoco` 后 `import mujoco` 报 `WinError 4551`？**
Windows 的 Smart App Control 拦截了该版本 DLL。换用 `pip install "mujoco==3.2.7"`
即可（本机实测通过）；或到"Windows 安全中心 → 应用和浏览器控制 → 智能应用控制"关闭它
（该开关不可逆，请自行评估）。

**Q2：`--gui` 报缺少 `mujoco_viewer` / `glfw`？**
GUI 是可选依赖：`pip install mujoco-python-viewer glfw`。无图形界面的虚拟机请直接用
`python3 main.py`（默认 headless）。

**Q3：`ros2 launch` 报找不到可执行文件？**
需要先 `colcon build --packages-select mujoco_balance_car` 并 `source install/setup.bash`。
只想验证算法时不需要 ROS：`python3 main.py` 即可。

**Q4：为什么默认不带窗口，和仓库原文档的"一键运行弹窗"不一样？**
评测机/CI 常无图形界面，`python3 main.py` 必须能直接跑通（仓库约定）。可视化改由
`--gui` 提供，功能与原来一致（R 复位、↑/↓ 扰动、ESC 退出）。

## 8. 已知限制

- 只做**一维**平衡（底盘沿 x 滑移），没有转向自由度，因此不是完整的双轮差速机器人。
- 未建模电机动态（把力矩当作理想输入）、未建模轮胎打滑；实车需再加电流环与滑移补偿。
- 指标只在 MuJoCo 定步长 0.002 s 的仿真里成立，真实系统上稳定时间会明显变长。
- 可恢复倾角只在 0.30 rad（17.2°）以内验证；更大初值会因力矩饱和（±2 N·m）直接倒。
