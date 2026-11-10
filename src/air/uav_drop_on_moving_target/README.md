# uav_drop_on_moving_target —— 给行驶中的车队投送物资

无人机沿道路追上一辆行驶中的接收车，把物资投到它头顶。难点不在"扔得准"，
而在**车在动**：货从 30 m 高度落地要 2.47 s，这段时间车队又走了 14.8 m——
瞄准车辆"此刻"的位置，落点必然偏出接受半径。本模块把这件事拆成三层：

1. **弹道**（`core/ballistics.py`）：平抛解析解，落点由释放时的真实三维状态
   决定——水平速度给前抛量，竖直速度参与解下落时间，风按拖曳系数折算横偏；
2. **车辆模型**（`core/vehicle.py`）：折线路径 + 分段恒速 + 指定航点停留，
   任意未来时刻的位置/速度解析给出（含"中途停车 6 s"这类情形）；
3. **释放时刻解算**（`core/release.py`）：在无人机标称航迹上扫出使落点误差
   最小的松手时刻，并同步给出两条对照基线的误差。

## 快速开始

```bash
# 依赖：Python 3.10+，numpy / mujoco / matplotlib / pytest
cd src/air/uav_drop_on_moving_target
python main.py                 # 三个算例各真飞一次（MuJoCo 闭环）
python main.py --no-sim        # 只跑算法层，秒级
python main.py --case convoy_crosswind
python -m pytest test/ -q      # 28 项测试全过
python tools/make_gif.py       # 重录文档配图
```

ROS 2 环境下可 colcon 构建：

```bash
colcon build --packages-select uav_drop_on_moving_target
source install/setup.bash
ros2 run uav_drop_on_moving_target main        # 算法层 + 闭环
ros2 run uav_drop_on_moving_target drop_node   # 在线判定节点
```

## 算例与结果

| 算例 | 场景 | 释放时刻 | 闭环落点误差 | 基线一：瞄准当前位置 | 基线二：按匀速规划 |
| --- | --- | --- | --- | --- | --- |
| convoy_straight | 直行车队 6 m/s，无风 | 51.28 s | 0.01 m | 14.91 m | — |
| convoy_crosswind | 同上 + 6 m/s 侧风 | 48.31 s | 0.04 m | 14.84 m | — |
| convoy_stopgo | 中途停车 6 s 再起步 | 45.03 s | 0.03 m | 0.23 m（恰在停车窗口） | 36.00 m |

- 30 m 高度、10 m/s 巡航：下落 2.47 s，前抛 24.7 m；6 m/s 侧风横飘 11.9 m
  （风偏系数 0.8），比 6 m 的接受半径大一倍，必须反向补偿；
- 基线一说明"瞄准当前位置"的误差 ≈ 车速 × 下落时间，与无人机怎么飞无关；
- stopgo 算例里基线一反而准，因为释放发生在停车窗口内——真正会翻车的是
  基线二：忽略停留的匀速规划偏出 36 m，这就是车辆模型必须含停留的原因。

闭环里落点误差是 MuJoCo 刚体积分出的真实释放状态喂给同一套弹道模型、与
车队真实位置对比得来的；`results/<case>_result.json` 里有逐项机读记录。

## 与仓库已有模块的关系

- `uav_mujoco_sim`：物理底座（同一份 `quadrotor.xml` + `GeometricController`）；
- `uav_emergency_drop`（PR #1013）：投给**固定**需求点，本模块的目标在移动，
  核心差了一层"车辆未来位置预测"；
- `uav_moving_target_follow`（PR #1074）：跟住移动目标，本模块在跟上的基础上
  完成"对时对位"的投放。

## 已知局限

- 弹道仍按无阻力质点平抛 + 常值风，散布 σ = 0.15h 只是工程近似；
- ROS 节点对车队的未来位置用当前速度直线外推（预测窗口 30 s），航迹已知时
  应换成带 waypoints 的模型；
- 本机为 Windows + Python 3.12 开发环境、没有 ROS 2，launch 链路未实跑，
  README 中的构建/launch 步骤可供有环境时验证。
