# humanoid_local_planner_bench —— 人形足迹下的局部避障算法对比

MuJoCo 场景：8 m 走廊 + **0.90 m 窄门** + 两个圆柱障碍。足迹半径 0.28 m（肩宽 0.45 m）的
人形从起点走到 8 m 外终点，对比 **DWA** 与 **APF** 两种局部规划器。

**人形与轮式车的差别不在速度，在宽度**：0.56 m 的足迹塞进 0.90 m 门道，单侧净空只有 0.17 m。

## 效果

| 全景 | 过门 |
|---|---|
| ![](https://raw.githubusercontent.com/OpenHUTB/ros2/master/docs/img/humanoid/humanoid_local_planner_bench/scene_persp.png) | ![](https://raw.githubusercontent.com/OpenHUTB/ros2/master/docs/img/humanoid/humanoid_local_planner_bench/scene_door.png) |

完整截图与动图见 [`docs/humanoid/humanoid_local_planner_bench.md`](../../../docs/humanoid/humanoid_local_planner_bench.md)。

## 实测结果

Ubuntu 22.04 + MuJoCo 3.15.0，控制频率 20 Hz，到达判定半径 0.35 m。

| 规划器 | 到达 | 时间 (s) | 路径 (m) | 最小间距 (m) | 碰撞步 | 平均 \|ω\| (rad/s) |
|---|---|---|---|---|---|---|---|
| DWA | 是 | 38.00 | 17.15 | 0.167 | 0 | 0.25 |
| APF | 是 | **17.35** | 17.39 | 0.140 | 0 | 0.83 |

- 两者都能通过 0.90 m 窄门且零碰撞——足迹宽度确实进了判定。
- APF 快 2.2 倍，但转向抖动大（平均 |ω| 是 DWA 的 3.3 倍）。
- DWA 慢而平滑：硬约束逐条筛掉会撞的轨迹，安全余量更大。

## 环境（已验证）

| 组件 | 版本 |
|---|---|
| 系统 | Ubuntu 22.04（VMware 虚拟机）/ Windows 10、11（standalone） |
| Python | 3.10（22.04）/ 3.13（Windows） |
| MuJoCo | 3.3.0 / 3.15.0（Ubuntu）/ 3.16.0（Windows）；**下界 3.3**，3.2.x 不可用 |
| 依赖 | `mujoco`、`numpy`（pip 安装）；出图另需 `matplotlib`、`pillow` |
| ROS | Humble（仅 ROS 节点与 launch 需要） |
| 图形 | 无需独显；`--gif` 需 OpenGL 上下文，无桌面会话时用 `xvfb-run` |

## 安装与运行

```bash
sudo apt update && sudo apt install -y python3-pip patchelf libgl1 libglfw3
python3 -m pip install -r requirements.txt

cd src/humanoid/humanoid_local_planner_bench
python3 main.py --version              # 1.0.0
python3 main.py --test                 # 自检 10 项，失败退出码 1
python3 main.py --headless             # 对比表 + CSV + 曲线
python3 main.py --gif                  # 演示动图（需 DISPLAY 或 xvfb-run）
python3 main.py --demo --planner apf   # 视窗回放
```

## 边界

- **不含平衡控制**：底座运动学驱动，只评价规划层，不评价"人形能否真的走过去"。
- **步态仅用于可视化**：腿是 `contype=0` 的视觉几何，不参与碰撞、不产生推进力。
- **碰撞一律按足迹圆柱**：大幅摆腿时腿部可能与障碍视觉穿插，两者口径不同。
- **速度指令未限幅**：DWA/APF 输出有抖动，实际部署需加低通或速度斜坡。
- **仅单一场景**：一条走廊 + 一道窄门，结论不外推到开阔地或动态障碍。
- **Bug2 未纳入**：跟墙逻辑按质点设计，0.56 m 足迹在 0.90 m 门道必然刮蹭，属算法前提不匹配。

## 故障排查

| 现象 | 原因 | 处理 |
|---|---|---|
| `No module named 'mujoco'` | 未装依赖或 pip 与解释器不一致 | 用 `python3 -m pip install` |
| `--gif` 报 OpenGL 上下文错误 | 无 `DISPLAY` | `export DISPLAY=:0` 或 `xvfb-run -a python3 main.py --gif` |
| 激光最小距离接近 0 | 射线打到机器人自身 | 确认机器人几何体在 `group=1`、射线 `geomgroup=[1,0,1,1,1,1]` |
| 规划器原地不动 | 激光被自身遮挡，全部报"贴脸" | 同上；`--test` 看"激光测墙距离"是否为 1.54 |
| `MuJoCo 3.2.x 过低，需要 >= 3.3.0` | 3.2.x 的 `mj_geomDistance` 在「圆柱 vs box」上偏小 | `python3 -m pip install -U 'mujoco>=3.3'` |
