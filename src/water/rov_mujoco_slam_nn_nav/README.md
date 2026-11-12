# rov_mujoco_slam_nn_nav

MuJoCo ROV：物理仿真、6-DOF 键盘遥控、声呐 SLAM 建图导航、神经网络感知/规划/控制与端到端声呐图航行（ROS Noetic + 纯 NumPy 网络）。

- 详细教程与实验数据：`docs/water/rov_mujoco_slam_nn_nav/README.md`
- 一键入口：`./main.sh --mode test|teleop|nav|e2e|eval`
- 环境：Ubuntu 20.04 + ROS Noetic（需 `sudo apt install ros-noetic-tf`）+ Python 3.8 + MuJoCo 3.x
- 场景：`pool` / `pipe_field` / `trench`；闭环评测：`python3 main.py --mode eval --seed 0`
- 编码自检：`bash tools/check_encoding.sh`（阻止 BOM / CRLF 回归）
