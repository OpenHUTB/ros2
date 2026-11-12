# joint_traj_retime

关节轨迹的时间缩放重定时：给定各关节速度/加速度上限，求**最大常数时间缩放** α，
把"形状不错但太慢"的 `JointTrajectory` 压到贴着上限跑。

```
关键帧 ──自然三次样条──▶ q(t), q̇(t), q̈(t) ──约束求 α──▶ t/α 的新时间轴
```

## 1. 依赖

```bash
pip3 install numpy        # core 只需要 numpy；画图另需 matplotlib
colcon build --packages-select joint_traj_retime --symlink-install
source install/setup.bash
```

## 2. 不需要 ROS 就能跑

```bash
cd src/humanoid/joint_traj_retime
python3 main.py              # 示例：6 关节步态周期，输出三张表
python3 main.py --vmax 3.0   # 改全局速度上限
python3 main.py --json out.json
python3 -m unittest discover -s test -p "test_*.py" -v    # 22 passed
```

## 3. ROS 2

```bash
ros2 launch joint_traj_retime main.launch.py
# 发一条 JointTrajectory 进去，出来的是压缩后的时间轴
ros2 topic echo /joint_trajectory_retimed
```

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 订阅 | 原始关键帧 |
| `/joint_trajectory_retimed` | `trajectory_msgs/JointTrajectory` | 发布 | 时间轴已压缩（速度 ×α、加速度 ×α²） |

## 4. 参数

`config/params.yaml`：

| 参数 | 默认 | 说明 |
|---|---|---|
| `dt` | 0.001 s | 插值采样步长 |
| `default_v_max` | 2.0 rad/s | 速度上限（节点内各关节统一） |
| `default_a_max` | 8.0 rad/s² | 加速度上限 |
| `speed_scale` | 1.0 | 额外保守系数（<1 更慢） |

## 5. 验证边界

| 内容 | 状态 |
|---|---|
| `core.py` / 22 项测试 | ✅ 实跑通过（Windows 11 + Python 3.12 + NumPy 2.5） |
| `main.py` 三张表 | ✅ 实跑，见文档 |
| 两张图 | ✅ 由 `tools/make_figures.py` 实际生成 |
| ROS 2 节点 | ⚠️ 语法与桩导入通过，**未做 colcon build / launch 实跑** |

## 6. 已知限制

- 只做**全局常数**时间缩放，不是逐点时间最优（TOP）：拐角处不能单独减速；
- 无力矩约束（需要逆动力学与整机模型），力矩可行性要由控制器单独校验；
- 不检查碰撞、关节限位与路径几何可行性——本模块只动时间轴，不动形状；\n- 关键帧时间戳必须**严格递增**：非单调或重复会直接 `ValueError`（详见 `build_trajectory`），\n  不会静默算出有限的 α；节点侧的时间戳换算用整数纳秒拆分（`split_duration`），避免 `nanosec = 1e9` 越界。
