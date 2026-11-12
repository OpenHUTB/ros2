# humanoid_mujoco_slam_nn_nav

MuJoCo 双足人形机器人：物理仿真、键盘遥控、激光 SLAM 建图导航、
神经网络感知/规划/控制与端到端视觉驾驶（ROS Noetic + 纯 NumPy 网络）。

- 详细教程与实验数据：`docs/humanoid/humanoid_mujoco_slam_nn_nav.md`
- 一键入口：`./main.sh --mode test|teleop|nav|e2e|benchmark`
- 环境：Ubuntu 20.04 + ROS Noetic + Python 3.8 + MuJoCo 3.x

## 测试

`tests/*.py` 是 `main()` 脚本而不是 `unittest.TestCase`，因此没有注册
`catkin_add_nosetests`；请显式运行：

```bash
python3 main.py --mode test              # 4/4
# 或
PYTHONPATH=. python3 tests/run_all.py     # 4/4
```
