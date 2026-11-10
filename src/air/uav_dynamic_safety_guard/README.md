# uav_dynamic_safety_guard — 动态障碍下的时序神经安全层

对应作业任务（2）**传感器感知**与任务（3）**导航控制**。

在标称速度指令后面加一道安全层：输入 360° 激光的三帧距离栅格，输出修正后的
速度指令。几何安全层用障碍的真值轨迹做 0.6 s 外推，作为教师生成示范；神经
网络只看到雷达历史，必须自己把“障碍在动”从三帧栅格里读出来。

依赖 [uav_mujoco_sim](../uav_mujoco_sim) 提供 `/lidar/points`、`/uav/odom` 与
`/uav/cmd_vel`。

---

## 1. 与课程已有模块的区别

| 已有模块 | 区别 |
|---|---|
| `uav_neural_safety_filter` | 单帧 3×36 距离栅格 + 静态障碍，教师用场景真值 SDF 外推；运动障碍在单帧里只会让回波“变近”，网络看不到切向速度 |
| `uav_ppo_nav` / `uav_depth_gate_racing_e2e` | 端到端策略，避障编进训练目标，不能叠加在别人的控制器上 |
| `octree_uav_3d_pathfinding` | 几何规划，位姿已知，不处理底层速度指令 |
| `uav_formation_flight` | 编队保持，不含指令级安全约束 |

本模块的差异是**动态障碍 + 时序观测**：场景里有绕圈运动的柱子，观测是 3 帧
栅格；同一套代码、同一份数据可以只喂单帧（消融）或三帧（本模块），所以
“时序有没有用”是可测的，而不是靠说的。

## 2. 方法

```
/lidar/points ──→ 3×36 距离栅格 ──→ 3 帧历史 ─┐
/uav/odom ─────→ 机头系速度 ──────────────────┼─→ 332 维观测 ──┬─→ 几何教师（真值外推）
/uav/goal ─────→ 目标方向 ────────────────────┘   + 标称指令   └─→ 神经安全层（部署）
```

**几何教师**：对 86–87 个候选速度按环境的参考动力学（加速度限幅 4 m/s²、一阶
低通 τ=0.12 s、航向跟随 1.5 rad/s）外推 0.6 s，每步用障碍在该时刻的真值位置
计算间距，取整段路径最小值 $q(u)$；约束 $q \ge$ margin，可行候选里取离标称
指令最近的一个。无可行候选时取 $q$ 最大的动作（停下来只会被运动障碍撞）。

**神经安全层**：MLP 335→192→192→3（纯 numpy，102147 个参数），输入 332 维
观测 + 3 维标称指令，一次前向输出修正指令。

**为什么要时序**：单帧栅格无法区分“静止的柱子”和“正横穿过来的柱子”——两者
在同一帧里都只是一个近距离回波。三帧历史给出回波距离的变化率，相当于把
障碍的径向速度暴露给网络。

## 3. 环境

MuJoCo 四旋翼（`uav_mujoco_sim` 底座）+ 18 个静止盒/柱 + 5 个绕圈运动的柱子。

| 项 | 值 |
|---|---|
| 场景 | ±9 m，18 静止障碍（高 1.2–2.8 m）+ 5 运动障碍 |
| 运动障碍 | 半径 1.4–3.2 m 的圆周，速度 0.4–1.0 m/s，柱高 1.2–2.0 m |
| 雷达 | 36 方位 × 3 俯仰（−15°/0°/+15°），8 m |
| 控制频率 | 20 Hz（物理步长 0.01 s，每帧 5 步） |
| 标称控制器 | 目标吸引 + 栅格斥力，**不做避障**，1.8 m/s |
| 参考动力学 | 加速度限幅 4 m/s²、速度低通 0.12 s、航向跟随 1.5 rad/s |
| 训练场景 | seed 0–19（每个 8 集） |
| 测试场景 | seed 100–111（12 个未见场景） |

运动障碍高度 ≥1.2 m，保证三层激光都能打到；这一点与静态版本一致，避免把
“雷达层间盲区”混进动态障碍的结论里。

## 4. 数据与训练

`--collect-imitation` 让几何教师在动态场景里闭环飞行，逐帧记录
(三帧栅格, 速度, 目标方向, 距离/高度, 标称指令, 教师指令)。示范时教师用
**1.0 m 的间距余量**（比评测时更保守），留出学生回归误差的缓冲。

左镜像增广（方位角翻转 + y 分量取反）后按 episode 切 12% 验证集，MSE + Adam
（lr 2e-3，weight decay 1e-4），120 轮，保留验证最优权重。

## 5. 实测结果

12 个未见场景（seed 100–111，5 个运动障碍，20 s，1.8 m/s）：

| 过滤器 | 碰撞率 | 到达率 | 推进率 | 最小间距 | 延迟 |
|---|---|---|---|---|---|
| 无过滤 | 100% | 0% | 36% | −0.04 m | 0.002 ms |
| 刹车 | 42% | 8% | 39% | 0.13 m | 0.005 ms |
| 几何教师 | 0% | 92% | 93% | 0.82 m | 2.464 ms |
| **时序神经层** | 8% | 92% | 94% | 0.42 m | 0.058 ms |
| 单帧神经层（消融） | 33% | 58% | 71% | 0.30 m | 0.040 ms |

时序层把碰撞率从 100%（无过滤）压到 8%，只比几何教师差 8 个百分点，到达率
与推进率和教师持平（92% / 94%），延迟是教师的 1/42（0.058 ms vs 2.46 ms）。

把运动障碍拿掉、其余设置不变（消融对照）：

| 过滤器 | 碰撞率 | 到达率 | 推进率 | 最小间距 | 延迟 |
|---|---|---|---|---|---|
| 无过滤 | 58% | 42% | 56% | 0.45 m | 0.002 ms |
| 刹车 | 8% | 42% | 56% | 0.57 m | 0.005 ms |
| 几何教师 | 0% | 100% | 94% | 0.92 m | 2.117 ms |
| **时序神经层** | 0% | 100% | 95% | 0.75 m | 0.055 ms |
| 单帧神经层（消融） | 8% | 92% | 88% | 0.81 m | 0.040 ms |

静态场景里单帧与三帧都贴着教师（8% 与 0%），说明动态场景里的差距来自
“看不见障碍在动”，而不是网络容量。

测距噪声 0.08 m + 15% 丢点（动态场景）：

| 过滤器 | 碰撞率 | 到达率 | 推进率 | 最小间距 | 延迟 |
|---|---|---|---|---|---|
| 无过滤 | 100% | 0% | 36% | −0.04 m | 0.002 ms |
| 刹车 | 50% | 0% | 41% | 0.08 m | 0.005 ms |
| 几何教师 | 0% | 92% | 93% | 0.82 m | 2.431 ms |
| **时序神经层** | 25% | 75% | 84% | 0.30 m | 0.058 ms |
| 单帧神经层（消融） | 33% | 67% | 80% | 0.18 m | 0.041 ms |

时序差分对噪声敏感：碰撞率 8% → 25%。单帧模型本来就没用差分，噪声下
几乎不变（33%）。这是最明显的短板，也是下一步要补的。

![场景与轨迹](../img/air/uav_dynamic_safety_guard/01_scene.png)

![指标](../img/air/uav_dynamic_safety_guard/04_metrics.png)

![时序](../img/air/uav_dynamic_safety_guard/05_timeline.png)

![噪声](../img/air/uav_dynamic_safety_guard/06_noise.png)


## 6. 复现

```bash
# 仿真侧（Python ≥3.10，需要 mujoco 与 numpy）
python main.py --collect-imitation --out data/dynamic_imitation.npz \
               --seeds 20 --episodes 8 --mirror 1 --teacher-margin 1.0
python main.py --train-policy --data data/dynamic_imitation.npz \
               --out data/dynamic_policy_seq.npz --temporal 1
python main.py --train-policy --data data/dynamic_imitation.npz \
               --out data/dynamic_policy_single.npz --temporal 0 --hidden 128

# 任意机器（只需 numpy）
python main.py --selftest
python main.py --evaluate --eval-episodes 12 --moving 5 --max-time 20 \
               --out results.json
python main.py --figures  --results results.json \
               --outdir ../../docs/img/air/uav_dynamic_safety_guard
```

ROS2 在线运行见第 7 节。

## 7. ROS2 接口

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/lidar/points` | sensor_msgs/PointCloud2 | 相对机体中心、世界轴向 |
| 订阅 | `/uav/odom` | nav_msgs/Odometry | 位姿与世界系速度 |
| 订阅 | `/uav/cmd_vel_nominal` | geometry_msgs/Twist | 机头系标称指令 |
| 订阅 | `/uav/goal` | geometry_msgs/Point | 目标点（构特征用） |
| 发布 | `/uav/cmd_vel` | geometry_msgs/Twist | 机体系，已抵消姿态 |
| 发布 | `/guard/status` | std_msgs/String | 推理次数、延迟、指令 |

```bash
ros2 launch uav_dynamic_safety_guard guard.launch.py with_sim:=true
# 只起安全层（假定点云/里程计/标称指令已在别处运行）
ros2 launch uav_dynamic_safety_guard guard.launch.py with_sim:=false nominal:=false
```

键盘遥控接法：`ros2 run uav_mujoco_sim keyboard --ros-args -r /uav/cmd_vel:=/uav/cmd_vel_nominal`。

## 8. 文件结构

```
uav_dynamic_safety_guard/
├── main.py                          入口（自动定位底座 uav_mujoco_sim）
├── uav_dynamic_safety_guard/
│   ├── cli.py                       采集 / 训练 / 评测 / 出图 / 自检
│   ├── selftest.py                  37 项自检（只需 numpy）
│   ├── plotting.py                  六张评测图
│   ├── core/
│   │   ├── nn.py                    Linear / MLP / Adam
│   │   ├── geometry.py              盒·柱·球有符号距离与按时外推
│   │   ├── field.py                 动态障碍场与圆周运动障碍
│   │   ├── ranges.py                3×36 栅格、单帧 116 维 / 时序 332 维特征
│   │   ├── env.py                   MuJoCo 环境（geom_pos 逐帧改写）
│   │   ├── dataset.py               教师闭环示范采集
│   │   ├── samples.py               镜像增广、特征拼接、训练用 XY
│   │   ├── network.py               安全层网络与存取
│   │   ├── training.py              行为克隆训练与离线指标
│   │   ├── filter.py                候选集、几何教师、神经层
│   │   └── experiment.py            闭环评测
│   └── ros/                         guard_node / nominal_node
├── config/guard.yaml
├── launch/guard.launch.py
├── data/                            示范数据与两组权重
└── test/                            pytest 50 项
```

## 9. 已知限制

- 学生的碰撞率仍显著高于教师：教师有障碍真值轨迹，学生只有三帧雷达，在
  近距遭遇运动障碍时回归误差足够让动作越过安全边界。
- 运动障碍是匀速圆周运动，参数对教师可见；换成随机折返、变速或人群场景
  需要重新采集，教师也需换成能处理非匀速的模型。
- 三帧历史只给出 0.1 s 的回波变化，横穿速度的估计在近距离仍然粗糙；
  更长历史或显式的速度估计器应能进一步缩小与教师的差距。
- 雷达噪声与丢点会削弱时序差分，碰撞率上升。
- 过滤器视界 0.6 s，被完全封死的通道会停下；ROS 节点只做了语法检查，
  未在本机 colcon 构建（本机无 ROS），与 `uav_mujoco_sim` 的联调留待有
  ROS 环境的机器验证。
