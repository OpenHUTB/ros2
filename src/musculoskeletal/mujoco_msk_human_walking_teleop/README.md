# mujoco_msk_human_walking_teleop

MuJoCo 平面肌肉骨骼人（22 条 Hill 型肌肉驱动 13 DoF）键盘遥操作：站立、行走、踢腿、蹲起，支持平地 / 5° 坡道 / 负载 10 kg 三场景。

完整教程（含原理公式、算法流程、源码解析、性能评价与 GIF 演示）见仓库文档：`docs/musculoskeletal/msk_human_walking_teleop.md`。

## 运行环境

- Ubuntu 20.04 + ROS Noetic（rospy）
- Python 3.8：`pip3 install mujoco==3.2.3 numpy pyyaml`
- `sudo apt install xterm`

## 运行步骤

```shell
cd ~/catkin_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="mujoco_msk_human_walking_teleop"
source devel/setup.bash

# 一键启动：MuJoCo 仿真窗口 + xterm 键盘窗口
roslaunch mujoco_msk_human_walking_teleop main.launch
# 场景切换：scene:=flat|slope5|load，无显示器加 viewer:=false
```

键盘：`1` 站立 / `2` 踢腿 / `3` 蹲起 / `4` 行走 / `0` 放松，`+`/`-` 步频调节，
`Q/W A/S Z/X`（左腿）、`U/I J/K N/M`（右腿）、`R/T F/G`（双臂）手动肌肉兴奋，`H` 帮助，`ESC` 退出。

无 ROS 离线运行：

```shell
python3 main.py check    # 模型自检
python3 main.py eval     # 多场景性能评测
```
