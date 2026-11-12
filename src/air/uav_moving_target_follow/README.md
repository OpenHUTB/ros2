# uav_moving_target_follow —— 移动地面目标的后方等距跟随

无人机跟着一辆车走这件事，难点不在"飞到它旁边"，而在三个咬在一起的问题：
车在动，追的时候得预测它能到哪；跟着飞的时候参考点该放在哪（正上方会随目标
急转甩来甩去）；以及转角处偏移方向一转就是 90°，参考点瞬间跳 17 m，位置环必
然大幅超调。本模块补的就是这一层：解析截获预测 + standoff 参考生成（偏移方向
限速旋转）+ MuJoCo 真飞验证。

仓库里 `uav_waypoint_tracking` 跟的是静止航点，`uav_moving_platform_landing`
要落到平台上，`uav_poi_orbit` 绕的是固定点——"保持与移动目标等距飞行"这一层
还没有模块覆盖，本模块与它们互补。

## 三个问题

1. **目标在动，截获点在哪？**——固定点迭代：t 时刻的 standoff 参考点位置是
   目标轨迹的解析函数，无人机"爬升 + 巡航"可达时刻与之相等处即最早截获时刻。
2. **参考点放哪？**——目标速度反方向 standoff 米（正后方）、固定高度；目标
   停车时保持上一次的偏移方向（不跳到随机一侧）。
3. **转角参考点跳变怎么办？**——偏移方向按 15°/s 限速旋转，参考点绕目标走
   圆弧而不是 17 m 跳变；旋转期间的切向线速度计入参考速度前馈。

## 运行环境

- 系统：Ubuntu 20.04 / Windows 11
- Python：3.8+（本机 3.13.14）
- 依赖：numpy、mujoco、matplotlib、pytest
- ROS 2：Humble（ROS 侧节点）

## 快速开始

```bash
pip install numpy mujoco matplotlib pytest

cd src/air/uav_moving_target_follow
python main.py                    # 三个算例各真飞一次
python main.py --case town        # 单个算例
python main.py --no-sim           # 仅算法层（截获预测 + 参考几何审计）
python -m pytest test/ -q         # 14 项测试
python tools/make_gif.py          # 重录四个图

# ROS 2 侧（Humble）
colcon build --packages-select uav_moving_target_follow
source install/setup.bash
ros2 launch uav_moving_target_follow main.launch.py
# 节点订阅 /uav/odom 与 /target/odom，在
# /uav_moving_target_follow/setpoint 发 JSON 参考点（p_ref/v_ref/yaw_d）
```

## 结果（MuJoCo 真飞，Quadrotor + GeometricController，dt = 0.01 s）

| 算例 | 目标路径 | 目标速度 | 截获时刻 | 跟随误差 mean / max | 转角最大 | 朴素"正上方"转角最大 |
| --- | --- | --- | --- | --- | --- | --- |
| `road` | 300 m 直线 | 6 m/s | 14.76 s | 0.06 / 0.07 m | 0.07 m | 12.27 m |
| `town` | 420 m 含两个 90° 弯 | 6 m/s | 14.76 s | 0.41 / 4.61 m | 4.61 m | 12.61 m |
| `s_curve` | 400 m 含 3 个折弯 | 5 m/s | 11.10 s | 0.13 / 1.00 m | 0.78 m | 12.19 m |

- 跟随误差 = 无人机真实位置与 standoff 参考点的距离，统计窗口为截获后 3 s
  起至目标停止行驶；目标停车瞬间速度前馈从 6 m/s 骤降为 0，有一次 3~4 m 的
  减速超调（results JSON 里单列为 `stop_transient_max_m`），1.5 s 内收敛；
- "朴素正上方"列用同一条真实轨迹、把参考点换成目标正上方复算——转角处误差
  是 standoff 参考的 2.6~15 倍，这就是 standoff 几何 + 偏移限速的收益；
- `results/case_<name>_result.json` 有截获预测、参考几何审计与逐时刻误差的
  机读记录。

## 模块结构

```
uav_moving_target_follow/
├── core/target.py       GroundTarget：折线路径 + 恒速，任意 t 的解析 pos/vel
├── core/follow.py       follow_reference（standoff 参考生成、偏移限速）、
│                        plan_intercept（固定点迭代求最早截获时刻）
├── core/flyer.py        MuJoCo 闭环：爬升 → 按解析截获直线飞行 → standoff 跟随
├── ros/follow_node.py   薄壳节点：订阅 /uav/odom + /target/odom，发布参考点
├── config/…yaml         standoff_m / alt_m / cruise_speed / 话题名
├── test/                9 项纯算法 + 5 项 MuJoCo 冒烟
└── tools/make_gif.py    朴素 vs standoff 同帧对比动图
```

## 已知限制

- 目标模型是恒速折线（里程计接入后即真实目标），未建模目标机动逃逸；
- 偏移限速 15°/s 是按巡航 9 m/s 扫出来的（45°/s 时转角切向速度超出巡航能力，
  误差反而变大），换机型需要重新标定；
- 能耗/续航未纳入截获决策——追不上时只给"提高巡航速度或缩小 standoff"的建议。

## 大模型使用声明

使用了 AI 大模型辅助进行代码编写与自查。作者对提交内容负责。
