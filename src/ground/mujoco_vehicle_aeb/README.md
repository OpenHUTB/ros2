# mujoco_vehicle_aeb —— MuJoCo 自动紧急制动（AEB / TTC）

> 地面载具选题：两车纵向场景下的 **TTC 型自动紧急制动**。
> 重点不只是"刹住了"，而是给出**可复算的安全指标**：最小净间距、是否碰撞、
> 碰撞瞬间速度、制动距离、最小 TTC，以及"初速度 × 增益"的覆盖范围扫描。

## 1. 与仓库已有模块的区别

| 模块 | 关注点 | 是否给量化指标 | 本机可跑（无仿真器） |
|---|---|---|---|
| `mujoco_car_avoidance` | 射线感知 + 分级减速避障 | 是 | 是（MuJoCo） |
| `mujoco_car_park` / `carla_aeb_braking` | 泊车 / CARLA 侧 AEB | 视模块而定 | CARLA 侧需要仿真器 |
| **本模块** | **TTC 比例制动 + 制动保持，两车纵向** | **是（§5）** | **是（纯 MuJoCo，无 ROS/仿真器）** |

## 2. 目录结构

```
src/ground/mujoco_vehicle_aeb/
├── main.py                    # 课程约定入口（转发到 scripts/cli.py）
├── main.bat                   # Windows 一键脚本
├── models/aeb_car.xml         # 两车纵向模型（自车 freejoint + 前车滑移关节）
├── scripts/
│   ├── main.py / main.sh      # rosrun 兼容入口 / Linux 一键脚本
│   ├── cli.py                 # 命令行：headless（默认）/ --gui / --selftest
│   ├── aeb_controller.py      # TTC 计算 + 比例制动 + 制动保持（纯逻辑，可单测）
│   ├── aeb_env.py             # MuJoCo 封装：按关节名取地址、施加制动、驻车保持
│   ├── aeb_metrics.py         # 指标：最小间距/碰撞/制动距离/最小 TTC（纯函数）
│   ├── runner.py              # 三场景 + 工况扫描 + CSV
│   └── plotting.py            # matplotlib 可选出图
├── launch/main.launch         # roslaunch 入口（参数全部透传，gui:=true 开窗口）
├── test/test_aeb_logic.py     # 单元测试 + 仿真集成
├── CMakeLists.txt / package.xml / requirements.txt
└── README.md
```

## 3. 运行环境（实测）

| 组件 | 实测版本 / 说明 |
|---|---|
| 操作系统 | Windows 11 原生（Linux/macOS 同样可用） |
| Python | 3.12.14 |
| MuJoCo | **3.2.7**（3.17.0 的 DLL 在本机被 Windows Smart App Control 拦截，见 FAQ） |
| numpy | 2.3.5 |
| matplotlib | 3.11.2（**可选**，只用于出图；缺失时只写 CSV，退出码仍为 0） |
| ROS | 不需要（`launch` 只是多一种启动方式） |
| 图形界面 | **不需要**（默认离线批跑）；`--gui` 才需要 GLFW |

## 4. 安装与运行

```bash
cd src/ground/mujoco_vehicle_aeb
pip install -r requirements.txt      # mujoco + numpy
pip install matplotlib               # 可选：需要 PNG 曲线时
```

**一键运行（默认离线批跑）**

```bash
python3 main.py                      # 三场景 + 工况扫描，产物写到 ./shots
bash scripts/main.sh                 # Linux/macOS 包装脚本
main.bat                             # Windows 包装脚本
```

**其它入口**

```bash
python3 main.py --selftest           # 25 项单元测试
python3 main.py --gui                # 打开 MuJoCo 窗口（无 GUI 环境请用默认模式）
python3 main.py --v-ego 12 --ttc 6 --gain 2.0   # 改工况/阈值/增益
python3 main.py --no-latch           # 复现"纯比例 TTC 律"（贴脸才刹）
ros2 launch mujoco_vehicle_aeb main.launch gui:=false v_ego:=8
```

产物（默认 `shots/`）：

| 文件 | 内容 |
|---|---|
| `aeb_metrics.csv` | 三场景全部指标 + 扫描网格 |
| `aeb_baseline_series.csv` / `aeb_no_aeb_series.csv` / `aeb_high_speed_series.csv` | 时间序列：`t, ego_x, ego_v, gap, ttc, brake, force` |
| `aeb_curves.png` | 四联图：速度 / 净间距 / TTC / 制动量 |
| `aeb_sweep.png` | 最小净间距 vs 初速度（按增益分组，0 线以下为碰撞） |

## 5. 实测指标（本机实测，可由 CSV 逐点复算）

工况：自车 5 m/s、前车静止在 35 m 处、TTC 阈值 5 s、增益 1.4、制动力上限 6000 N。

| 场景 | 最小净间距 | 碰撞 | 碰撞速度 | 最小 TTC | 制动介入 | 制动距离 |
|---|---|---|---|---|---|---|
| `baseline`（AEB 开） | **18.72 m** | 否 | — | 4.99 s | 1.59 s | **13.88 m** |
| `no_aeb`（增益 0，对照组） | −35.04 m | **是** | **4.95 m/s** | −86.29 s | — | — |
| `high_speed`（12 m/s） | 10.69 m | 否 | — | 2.71 s | 0.00 s | 21.91 m |

"初速度 × 增益"覆盖扫描（数字为最小净间距 m，`撞` = 发生碰撞）：

| v (m/s) \ gain | 0（关闭） | 0.7 | 1.4 | 2.1 |
|---|---|---|---|---|
| 3 | −11.96 撞 | 12.67 | 12.67 | 12.67 |
| 5 | −35.04 撞 | 18.72 | 18.72 | 18.72 |
| 8 | −35.06 撞 | 17.21 | 17.21 | 17.30 |
| 12 | −35.09 撞 | 3.10 | 10.69 | 13.82 |
| 16 | −35.12 撞 | **−12.15 撞** | 6.70 | 9.33 |

读法：增益为 0 时**每一档都撞**（这正是 AEB 的价值）；增益 0.7 在 16 m/s 已经失守；
1.4 与 2.1 能覆盖到 16 m/s，且间距随增益增大——即"刹得更狠、留得更多"。

![AEB 闭环曲线](../../docs/img/ground/mujoco_vehicle_aeb_curves.png)

![覆盖范围扫描](../../docs/img/ground/mujoco_vehicle_aeb_sweep.png)

## 6. 调参记录（这版改了什么、为什么）

| 阶段 | 现象 | 原因 / 处理 |
|---|---|---|
| 原实现 | **一跑就 `IndexError`**：`data.qpos[11]` / `data.qvel[10]`，而模型只有 11 个 qpos、10 个 qvel | 前车是直接挂在 worldbody 上的刚体（没有关节），qpos 里根本没有它的状态。改为给前车加 `lead_slide` 滑移关节，**按关节名取地址**，不再硬编码下标 |
| 原实现 | 只能在有窗口的机器上跑（`mujoco.viewer`） | 主入口改为离线批跑；`--gui` 才开窗口 |
| 纯比例律 | 制动力长期只有 0.15，12 s 仍未停住，贴着前车蹭 | 制动抬高 TTC，TTC 抬高又压低制动力（负反馈）。加**制动保持**（触发后不低于 `latch_floor`），减速变单调 |
| 保持逻辑 | 车以 0.42 m/s 一直滑行，`stop_distance` 算不出来 | 低速段 TTC 被"间距/极小速度"放大，刚触发就解除保持。改为**只有不再接近前车时才允许解除** |
| 制动方向 | 恒定制动力让车停下后**反向加速**（12 s 时 v=−19.5 m/s） | 模型没有滚动阻力。制动改为**逆着运动方向**；并加低速驻车保持（检查 `hold_still`），文档中注明这一近似 |
| 对照组 | `no_aeb` 与 `baseline` 指标完全相同 | `gain=0` 时仍被置位 `triggered`，保持下限补出了制动力。改为**比例项 > 0 才触发**，并用单测锁死 |

## 7. 常见问题

**Q1：`import mujoco` 报 `OSError: [WinError 4551] 应用程序控制策略阻止了此文件`？**
Windows Smart App Control 拦截了该版本 DLL。换 `pip install "mujoco==3.2.7"` 即可
（本机实测通过）；或自行关闭智能应用控制（该开关不可逆）。

**Q2：`--gui` 打不开窗口？**
GUI 需要图形界面与 GLFW；无显示环境请用默认的离线批跑（同样会导出全部指标）。

**Q3：为什么"无 AEB"场景会显示间距 −35 m？**
那只表示自车**穿过**了静止前车（模型只做纵向、没有碰撞响应），用来对照"没刹车会怎样"。

**Q4：为什么刹车后车不会倒着走？**
真实制动器的力始终与运动方向相反。若像早期实现那样恒定沿 −x 施力，模型没有滚动阻力，
车停下后会继续反向加速——所以 `apply_brake` 按速度符号取方向。

**Q5：指标里的"最小 TTC"为什么会出现负数？**
碰撞后两车重叠、相对速度仍为正，按定义算出的 TTC 就是负的。它只在"已撞"时出现，
可当作"撞得多深"的粗略指示。

## 8. 已知限制

- 模型是**纵向一维**简化：没有转向、没有悬架、没有轮胎滑移模型，碰撞也没有响应。
- 没有滚动阻力与静摩擦，因此低速段用"驻车保持"把速度清零（近似真实车辆的停车保持）。
- 前车匀速、不减速：不覆盖"前车急刹"这类更难的工况。
- 制动力上限 6000 N（≈0.6 g）是按乘用车量级取的，未做实车标定。
- 本机未安装 ROS，`roslaunch` / `catkin_make` 路径**未实测**（launch 与 CMakeLists 按
  同仓库其它 catkin 模块的写法编写）。
