# humanoid_passage_width_sweep

人形在 MuJoCo 窄通道里绕障通行，回答一个问题：**人形到底需要多宽的通道**？
做法是 2x2 单变量对比 —— 同一个规划算法 × 同一套场景，只改「代价函数里的膨胀半径」：

| | 膨胀 0.38 m（足迹 0.30 + 裕度 0.08） | 膨胀 0.18 m（质点假设） |
|---|---|---|
| **MPPI**（采样型 MPC） | `mppi_footprint` | `mppi_point` |
| **反应式斥力法** | `reactive_footprint` | `reactive_point` |

## 1. 结论

7 档门宽（0.62～1.10 m）× 2 次重复 = 每变体 14 次：

| 变体 | 通过 | 成功率 |
|---|---|---|
| MPPI（足迹感知） | 12/14 | **86%** |
| MPPI（质点假设） | 0/14 | 0% |
| 斥力法（足迹感知） | 0/14 | 0% |
| 斥力法（质点假设） | 0/14 | 0% |

- **同一算法只改膨胀半径，通过率从 86% 掉到 0%**：忽略人形体宽（0.60 m 足迹按质点处理）
  会让规划器把「贴着自己身体擦过去」判成可行解 —— 这是本模块最硬的一条结论。
- MPPI 只在 **0.62 m** 那档失败：该档单侧净空仅 **1 cm**，几何上已接近不可通行。
- 反应式斥力法在两种膨胀下都失败，说明**局部斥力缺少前瞻**：0.22 m 的圆柱要提前约
  0.5 m 让开，纯反应式来不及（详见第 5 节）。

## 2. 效果图

| 出发 | 绕过第一个障碍 |
|---|---|
| ![](../../docs/img/humanoid/humanoid_passage_width_sweep/scene_start.png) | ![](../../docs/img/humanoid/humanoid_passage_width_sweep/scene_obstacle.png) |

| 穿过窄门 | 到达目标 |
|---|---|
| ![](../../docs/img/humanoid/humanoid_passage_width_sweep/scene_gate.png) | ![](../../docs/img/humanoid/humanoid_passage_width_sweep/scene_done.png) |

| 通过率 vs 门宽 | 最小净空 vs 门宽 |
|---|---|
| ![](../../docs/img/humanoid/humanoid_passage_width_sweep/success_vs_width.png) | ![](../../docs/img/humanoid/humanoid_passage_width_sweep/clearance_vs_width.png) |

0.70 m 门口的完整轨迹（绕过两个圆柱并穿门）：

![](../../docs/img/humanoid/humanoid_passage_width_sweep/trajectory.png)

## 3. 运行环境

| 项 | 要求 |
|---|---|
| 系统 | Ubuntu 22.04 / Windows 10+ |
| ROS | Humble（只在 `ros2 launch` 时需要；独立入口不需要） |
| Python | 3.10+ |
| MuJoCo | >= 3.3 |
| 依赖 | `mujoco`、`numpy`，出图另需 `matplotlib`、`pillow` |
| 磁盘 | < 1 MB（纯 XML 建模，无网格资源） |

## 4. 安装与运行

```bash
cd src/humanoid/humanoid_passage_width_sweep
python3 -m pip install -r requirements.txt

python3 main.py --version              # 版本与关键参数
python3 main.py --test                 # 10 项自检
python3 main.py --headless             # 宽度扫描 + CSV + 图
python3 main.py --snapshots --width 0.86   # 四张场景快照
python3 main.py --gui                  # 交互窗口
python3 main.py --export-model         # 重新生成 models/corridor_scene.xml
```

ROS 2：

```bash
colcon build --packages-select humanoid_passage_width_sweep
source install/setup.bash
ros2 launch humanoid_passage_width_sweep main.launch.py
ros2 launch humanoid_passage_width_sweep main.launch.py width:=0.78 variant:=mppi_point
ros2 topic echo /passage_sweep_node/state --once
ros2 topic pub --once /passage_sweep_node/reset std_msgs/Empty
```

## 5. 能力边界

1. **人形是运动学驱动的**：直接积分位姿，不受力、不做平衡，评价的是规划层。
   碰撞判定用 MuJoCo 真实接触（足迹圆柱 vs 墙/柱），**碰撞即终止**。
   > 早期版本没有终止，规划器会穿墙而过、把失败伪装成 `reached=True`（实测 contact 了 502 步）。
2. **反应式斥力法在本场景全失败**，我试过：切向滑移分量（k_tang 0.9）、
   提高斥力增益、两种膨胀半径 —— 仍然刮蹭。这是「局部反应式缺少前瞻」的证据，
   不是没调参。
3. **0.62 m 门槛未通过**：单侧净空 1 cm，属于几何极限，不宣称能过。
4. **纯平面、无动态障碍**：速度上限 0.6 m/s，障碍静止。
5. 只用 64 束激光、量程 4 m；大于 4 m 的障碍只能靠走进量程才发现。

## 6. 测试

```bash
python3 test/test_standalone.py                        # 纯 Python：场景几何 / 规划器 / 指标
python3 ../../../../tests/test_humanoid_passage_width_sweep_local.py   # 需要 MuJoCo
```

CI：`.github/workflows/humanoid_passage_width_sweep_tests.yml`。

## 7. 第三方与许可

场景、规划器、代码、图均为本模块原创，无外部模型/权重。MPPI 与 APF 属公开方法，
为独立实现。许可：MIT。