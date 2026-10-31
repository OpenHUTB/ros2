# rooftop_landing_nn

**屋顶降落（学习型落点选择）**：机载 LiDAR 点云栅格化成高度图（高度 / 掩码 / 粗糙度三通道），
用一个小卷积网络在高度图上直接回归**最安全的屋顶落点**，再控制无人机飞抵其上方并垂直降落。

它与同选题的 [`rooftop_landing`](../rooftop_landing/README.md)（RANSAC 平面 → 凸包 → polylabel
最大内切圆）是两条技术路线，**话题接口完全一致**，可以互相替换或并排运行。

详细的动机、方法、消融实验与参数说明见文档页
[`docs/air/rooftop_landing_nn.md`](../../../docs/air/rooftop_landing_nn.md)。

---

## 1. 结果速览

同一批场景、同一份 ROI 内点云，主指标 = 「落在屋面内且落点净空 ≥ 0.6 m」的比例：

| 屋顶保留点数 | 本模块 | 几何法（上游默认） | 几何法（按上游建议调参） |
|---|---|---|---|
| 100%   | **100%** | 50% | 60% |
| 50%    | **100%** | 55% | 60% |
| 25%    | **98%**  | 58% | 57% |
| 12.5%  | **93%**  | 23% | 22% |
| 6.25%  | **95%**  | 18% | 15% |

分形状（保留 35% 点）：矩形屋顶两者都是 100%（不夸大）；**U 形 / L 形 / 带天井**
几何法降到 0%~67%（凸包把凹口填平），本模块保持 100%。

## 2. 依赖与编译

```bash
# 客户机（Ubuntu 20.04 + ROS Noetic），唯一额外依赖是 numpy
sudo apt install -y python3-numpy

cd ~/uav_ws/src
cp -r <本包> air/            # 目录结构为 src/air/rooftop_landing_nn
cd ~/uav_ws && catkin_make
source devel/setup.bash
```

**不需要 torch**：权重要么用 `tools/train_landing_cnn.py` 在开发机上训练（PyTorch + GPU），
要么直接用仓库里的 `models/landing_cnn.npz`；numpy 前向在 `scripts/landing_cnn.py` 里逐层复刻，
训练脚本结束时会自动做 numpy / torch 对齐检查（argmax 一致率 100%）。

## 3. 运行

```bash
# 终端 1：桥接层（见 docs/air/carlair_ros_bridge.md）
roslaunch carlair_ros_bridge main.launch host:=<仿真器地址>

# 终端 2：本模块（作业要求的 launch 启动）
roslaunch rooftop_landing_nn main.launch
# 等价前台运行：rosrun rooftop_landing_nn main.py
```

节点状态机：`DETECT`（累积点云 → 推理落点）→ `APPROACH`（飞抵屋面上方安全高度）
→ `DESCEND`（垂直下降）→ `LANDED`；超时进入 `ABORT`（悬停待命）。

常用参数（全部可被 roslaunch 覆盖，详见 `config/rooftop_landing_nn.yaml`）：

```bash
roslaunch rooftop_landing_nn main.launch safe_altitude:=6.0
roslaunch rooftop_landing_nn main.launch z_up:=false            # 桥接层是 NED 下为正
roslaunch rooftop_landing_nn main.launch use_rviz:=false
roslaunch rooftop_landing_nn main.launch rough_max:=-1          # 关掉"平坦度"硬约束
roslaunch rooftop_landing_nn main.launch accumulate_frames:=5   # 点云更稀时多累积几帧
```

## 4. 测试

```bash
# 4.1 本地回归（无需 ROS / 仿真器 / GPU / torch），预期结尾 "结果：46/46 通过"
python3 tests/test_rooftop_landing_nn_local.py

# 4.2 闭环冒烟（假桥接层 + U 形屋顶，不含仿真器）
#     终端 1: roslaunch rooftop_landing_nn main.launch use_rviz:=false
#     终端 2:
python3 tests/test_rooftop_landing_nn_ros_sim.py
```

`tests/test_rooftop_landing_nn_ros_sim.py` 最后会核对落点是否落在 U 形屋面内、
以落点为心 0.6 m 的圆是否完全在屋面内（几何法在这张图上会把落点选到缺口上方）。

4.2 已在 **Ubuntu 20.04 + ROS Noetic** 实机跑通（`roscore` + `rosrun rooftop_landing_nn main.py`
+ 测试脚本），输出：`DETECT -> APPROACH -> DESCEND -> LANDED`，落点 (0.12, -1.16)、
屋面内 + 0.6 m 净空均为「是」，`结论：通过`。录屏见
`docs/img/air/rooftop_landing_nn/closed_loop_ros_noetic.gif`（左：节点实时日志，
右：RViz 点云与落点标记），单帧见同目录 `closed_loop_ros_noetic.png`。

4.3 **真实 UE 5.5 + Cosys-AirSim 上也跑通过一次完整降落**（检出落点 → 飞抵 → 下降 → LANDED，
落点置信度 1.00），录屏见 `docs/img/air/rooftop_landing_nn/airsim_landing.gif`。
注意：下降走 `/uav/cmd_vel`，桥接层的 `control/cmd_duration` 必须先设成 0.5 s，
否则 10 Hz 下发时命令会被下一条顶掉、飞机不下高度（实测 4 s 只降 0.11 m）。

## 5. 重新训练 / 复现实验（开发机，需要 PyTorch）

```bash
# 训练：约 4 分钟（RTX 5060），产出 models/landing_cnn.npz + results/ + 训练曲线
python tools/train_landing_cnn.py --epochs 60 --batches-per-epoch 40

# 对比实验：产出 results/*.csv 与 docs/img/air/rooftop_landing_nn/*.png
python tools/eval_landing_nn.py --scenes 60
```

## 6. 目录

```
rooftop_landing_nn/
├─ scripts/
│   ├─ main.py                    # 入口（main.* 约定）
│   ├─ rooftop_landing_nn_node.py # ROS 节点：状态机 / 话题 / RViz marker
│   ├─ heightmap.py               # 点云 → 高度图（纯 numpy，训练与部署共用）
│   └─ landing_cnn.py             # numpy 前向 + 落点提取 + 两条硬约束
├─ tools/
│   ├─ scene_dataset.py           # 合成屋顶场景 / 点云退化 / 真值（最大净空中心）
│   ├─ train_landing_cnn.py       # PyTorch 训练 + 导出 npz + 对齐检查
│   └─ eval_landing_nn.py         # 与几何法对比的消融实验 + 出图
├─ models/landing_cnn.npz         # 权重（约 200 KB，入库）
├─ results/                       # 消融实验 CSV / metrics.json
├─ config/ launch/ rviz/ package.xml CMakeLists.txt
```

## 附：大模型使用声明

本模块代码、训练脚本、实验与文档在编写过程中使用了 AI 大模型辅助（方案讨论、代码与文档起草、
实验脚本整理）。全部结论均来自仓库内可复现的运行结果：本地回归测试 46 项断言、
`tools/eval_landing_nn.py` 的三组消融实验（CSV 与图见 `results/` 与 `docs/img/air/rooftop_landing_nn/`）。
作者对提交内容的正确性与完整性负全部责任。
