# torque_vectoring_yaw_control

四轮独立驱动车辆的**扭矩矢量横摆控制**：把"要多少总驱动力、要多少横摆力矩"翻译成四个轮子的纵向力，
每步都在附着椭圆约束下求解。纯 Python + numpy，无 ROS 也能跑完整仿真与测试。

## 核心内容

* `vehicle.py` —— 整车参数、线性轮胎 + `μFz` 截断、附着椭圆给出的纵向力额度
* `allocator.py` —— 加权最小二乘分配（`min Σ(F_i/c_i)²`），饱和时用有效集逐轮钉边界
* `control.py` —— 参考横摆率（含 `μg/u` 截断）、速率前馈 + PI、抗积分饱和
* `world.py` —— 纵向/侧向/横摆三自由度动力学，含转向速率限制与载荷转移
* `node.py` —— ROS 2 节点，上层给总驱动力，本节点只负责横摆力矩与四轮分配
* `sim/` —— 场景、实验、作图

## 快速开始

```bash
pip install -r requirements.txt

python main.py                      # 四个场景 × 有/无扭矩矢量
python main.py --scenario over_limit
python main.py --plots ./fig        # 另存五张图
python -m pytest tests -q           # 41 项测试
bash ci/run_ci.sh                   # 四阶段自检
```

## ROS 2

```bash
colcon build --packages-select torque_vectoring_yaw_control
ros2 launch torque_vectoring_yaw_control torque_vectoring.launch.py mu:=0.8 enable_tv:=true
```

节点订阅 `/vehicle/velocity`、`/vehicle/steer`、`/vehicle/drive_force_cmd`，
发布 `/vehicle/wheel_force_cmd` 与 `/vehicle/wheel_torque_cmd`（左前/右前/左后/右后）。
**总纵向力由上游给**，节点不再闭环车速——否则会和驱动/制动控制抢权限。

## 结论摘要

| 场景 | 峰值附着利用率 | 横摆 RMS 改善 | 侧偏角 |
| --- | --- | --- | --- |
| 阶跃转向 4° @ 20 m/s | 0.11 → 0.50 | 58.7% | 1.4° → 1.4° |
| 正弦扫掠 5° @ 0.5 Hz | 0.12 → 0.52 | 75.9% | 1.8° → 1.7° |
| 右后电机失效 @ 22 m/s | 0.20 → 1.00 | 56.8% | 1.9° → 1.9° |
| 超附着极限 5° @ 30 m/s | 0.17 → 1.00 | 32.9% | 33.6° → 25.2° |

前三个工况都在附着圆内（利用率 ≤ 0.52、侧偏角 < 2°），比的是控制质量；最后一个工况当场失控，
留着是为了划清"扭矩矢量能做什么、不能做什么"的边界。

详细推导与图表见 `docs/ground/torque_vectoring_yaw_control/index.md`。
