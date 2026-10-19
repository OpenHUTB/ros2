# mjc_rov_current_teleop

动态洋流扰动下的 **6-DOF 水下 ROV MuJoCo 物理仿真** + 键盘运动控制 + **MLP 神经网络巡航策略**（行为克隆）。

- 完全自包含：不依赖仓库其他包（不 import rov_mujoco 等）
- MJCF 模型只用 box/plane/cylinder 基本几何体，无外部 mesh/纹理
- 离线链路（collect/train/test）纯 Python，无 GPU 也能几分钟跑通
- ROS 2 Humble 节点支持 `ros2 launch` 一键启动

## 运行环境

- Ubuntu 20.04.6 LTS（无独立显卡）
- ROS 2 Humble（`/opt/ros/humble`）
- Python 3.8.10
- 依赖：`pip3 install -i https://mirrors.aliyun.com/pypi/simple/ -r requirements.txt`
- torch 用 CPU 版：
  ```bash
  pip3 install torch==2.4.1+cpu torchvision==0.19.1+cpu \
    -i https://mirrors.aliyun.com/pypi/simple/ \
    --extra-index-url https://download.pytorch.org/whl/cpu
  ```
- MuJoCo <3.2（Py3.8 最高支持 3.1.x），OSMesa 离屏渲染：
  ```bash
  sudo apt install libosmesa6
  pip3 install "mujoco<3.2" PyOpenGL
  ```

## 运行步骤

```bash
# 1. 编译（在 ros2_ws 根目录）
colcon build --symlink-install --packages-select mjc_rov_current_teleop
source install/setup.bash

# 2. 不带参数跑通全链路：采集 -> 训练 -> 评估
bash main.sh

# 3. 或分步
python3 main.py collect    # 采集专家数据
python3 main.py train      # 训练 MLP，产出 weights/cruise_mlp.pt
python3 main.py test       # 闭环评估，打印 MAE / 成功率

# 4. ROS 实时仿真
ros2 launch mjc_rov_current_teleop main.launch.py
# 另开终端：
ros2 run mjc_rov_current_teleop keyboard_node    # 键盘控制
ros2 run mjc_rov_current_teleop cruise_node      # MLP 自动驾驶
ros2 topic hz /rov_teleop/odom                   # 验证话题
```

## 话题

| 话题 | 类型 | 方向 |
|---|---|---|
| `/rov_teleop/cmd_vel` | geometry_msgs/Twist | 输入（键盘/巡航节点发） |
| `/rov_teleop/odom` | nav_msgs/Odometry | 位姿+速度 |
| `/rov_teleop/image_raw` | sensor_msgs/Image(rgb8) | 前视相机，160x120 |
| `/rov_teleop/status` | std_msgs/String | 状态文本 |

## 算法说明

- 动力学：代码层 qfrc_applied 施加 浮力补偿 + 黏性阻尼 + 横滚/俯仰自稳 + 时变洋流扰动
- 神经网络：MLP(6->64->32->2) 行为克隆，输入 [ex,ey,eyaw,vx,vy,wz]，输出 [vx_world, vy_world]（世界系期望速度）
- 权重不入库，`python3 main.py train` 本地复现
