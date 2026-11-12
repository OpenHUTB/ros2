# uav_tiltrotor_transition_corridor

倾转旋翼无人机的**过渡走廊**计算与倾转角调度。纯 Python + numpy，无 ROS 也能跑完整仿真与测试。

## 核心内容

* `aero.py` —— ISA 大气、机翼升阻极曲线、失速后升力衰减
* `rotor.py` —— 定距桨推力模型：动量理论功率限制 + 前进比线性衰减，两者取小
* `corridor.py` —— 走廊求解（二分，非网格扫描）、解析升力下边界、倾角保护
* `control.py` —— 推力矢量分配 + 速度/高度回路 + 走廊保护
* `world.py` —— 纵向三自由度动力学（含执行机构滞后与速率限制）
* `sim/` —— 场景、实验、作图

## 快速开始

```bash
pip install -r requirements.txt

python main.py                      # 全部场景 + 指标表
python main.py --scenario nominal --plots ./fig
python -m pytest tests -q           # 38 项测试
bash ci/run_ci.sh                   # 四阶段自检
```

## ROS 2

```bash
colcon build --packages-select uav_tiltrotor_transition_corridor
ros2 launch uav_tiltrotor_transition_corridor transition.launch.py corridor_altitude:=4000.0
```

`corridor_altitude` 必须填**机场密度高度**。用海平面的走廊表去飞高原，下边界会被低估 13° 以上，
这是本模块最容易踩的坑。

## 结论摘要

| 场景 | 中止 | 高度误差 | 最大下沉 |
| --- | --- | --- | --- |
| 标称过渡（30 m，0→20 m/s→0） | 否 | 0.18 m | 0.08 m/s |
| 高原过渡（4000 m） | 否 | 0.23 m | 0.06 m/s |
| 推力降级 15% | 是（掉高 9.5 m） | — | — |
| 粗糙调度 15°，无走廊保护 | 是（掉高 8.45 m） | — | 3.48 m/s |
| 同一条调度，开走廊保护 | 否 | 0.24 m | 0.10 m/s |

详细推导与图表见 `docs/air/uav_tiltrotor_transition_corridor/index.md`。
