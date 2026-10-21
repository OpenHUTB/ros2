# mujoco_msk_hand_teleop

MuJoCo 肌肉骨骼灵巧手：键盘遥操作 + 神经网络肌肉激活控制（ROS Noetic）。

- 8 自由度手部（3 指 + 2 自由度腕），11 条 Hill 型肌肉经空间腱驱动；
- 3 个场景：自由到达（NN 逆模型）/ 门铃式按键 / 拨球离台；
- 纯 numpy MLP 逆模型：指尖目标位置 → 11 维肌肉激活，平均到达误差 7.3 mm
  （线性基线 22.4 mm）。

## 快速开始

```bash
# 依赖: Ubuntu 20.04 + ROS Noetic
sudo apt install -y xterm        # keyboard_teleop.launch 用 xterm -e 弹窗读键
# 或用 rosdep 安装 package.xml 中声明的依赖（含 xterm）
rosdep install --from-paths src --ignore-src -r -y
pip install mujoco==3.2.3 numpy imageio matplotlib Pillow

catkin_make && source devel/setup.bash
roslaunch mujoco_msk_hand_teleop main.launch scene:=button   # 主入口
roslaunch mujoco_msk_hand_teleop keyboard_teleop.launch scene:=ball
roslaunch mujoco_msk_hand_teleop nn_reach.launch
```

无 ROS 环境：

```bash
python3 main.py check          # 模型自检
python3 main.py train          # 训练 NN 逆模型（约 5 分钟）
python3 main.py demo --scene button
```

按键映射：q/a 食指伸/屈，w/s 中指伸/屈，e/d 拇指伸/屈，r 拇指对掌，
u/j 腕屈/伸，k/l 腕左/右摆，空格放松，ESC 退出。

详细文档：`docs/musculoskeletal/msk_hand_teleop.md`（GitHub Pages 可见）。
