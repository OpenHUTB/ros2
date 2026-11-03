# ROS 环境下基于 MuJoCo 的机器人强化学习与仿真验证

## 一、项目介绍

本实验在 Ubuntu 和 ROS Noetic 环境下，利用 MuJoCo 物理引擎搭建虚拟仿真环境。通过 Python 脚本加载四足机器人的 MJCF 模型，使用 `mujoco.mj_step` 实现物理引擎的步进运算，并通过 `rospy` 节点实时输出仿真时间与机器人基座位置。实验旨在验证 ROS 节点与 MuJoCo 引擎数据交互的可行性，为后续接入强化学习算法（PPO）打下工程基础。

## 二、运行环境

- 操作系统：Ubuntu 20.04 (VMware 虚拟机)
- ROS 版本：ROS Noetic
- 编程语言：Python 
- 物理引擎：MuJoCo
- 深度学习框架：PyTorch / Stable-Baselines3
- 依赖功能包：mujoco (python)

## 三、项目结构

```text
src/quadruped_mujoco_rl/
├── launch/
│   └── mujoco_quadruped.launch  # 启动 MuJoCo 仿真与 RL 控制节点
├── config/
│   └── mujoco_params.yaml       # 仿真与 RL 超参数配置
├── scripts/
│   ├── main_train.py            # 训练脚本（入口）
│   └── main_test.py             # 测试与渲染脚本
├── models/        
|   ├─ a1.xml
│   ├─ quadruped.xml             # 四足机器人 MJCF 模型文件
│   └─ assets/
│   ├─ calf.obj
│   ├─ hip.obj
│   ├─ thigh.obj
│   ├─ thigh_mirror.obj
│   ├─ trunk.obj
│   └─ trunk_A1.png
├── package.xml                  # ROS 包清单
├── CMakeLists.txt               # 编译文件
└── README.md                    # 运行说明
```
(注：原本设计的 mujoco_quadruped.launch 因外部依赖包 mujoco_ros_pkgs 失效，改用直接运行 main_test.py 脚本的方式完成仿真验证。)
四、核心算法与源码解析

## 四、核心算法与源码解析

1. **物理引擎加载**：代码通过 `mujoco.MjModel.from_xml_path` 加载四足机器人 MJCF 文件，初始化物理模型和数据结构 `MjData`。
2. **闭环仿真流程**：`main_test.py` 节点启动后，通过循环以 100Hz 的频率调用 `mujoco.mj_step(model, data)` 推进物理仿真。
3. **状态数据输出**：在循环中，通过 `rospy.loginfo_throttle` 限制日志频率，实时打印仿真时间（`data.time`）和机器人基座位置（`data.qpos[:3]`），验证仿真环境连通性。
4. **参数配置**：后续可通过 `config/mujoco_params.yaml` 统一管理仿真步长与强化学习超参数，便于模型训练阶段的调参。
## 五、运行步骤

### 1. 安装 MuJoCo 与 ROS 接口依赖：
   ```bash
   pip install mujoco
   ```
### 2. 准备四足机器人的 MJCF 模型文件：
   从 MuJoCo 官方模型库（mujoco_menagerie）下载四足机器人模型，将获取的 quadruped.xml 和 assets 文件夹放置在功能包 models/ 目录下。
   *(注：如果网络受限无法从 GitHub 克隆，可在浏览器中下载 ZIP 包并解压)*
   
### 3. 编译工作空间并刷新环境（确保当前位于工作空间根目录）：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 4. 启动 ROS Master 与 MuJoCo 仿真节点（因原定的 roslaunch 依赖包失效，改用终端直接启动）：

打开一个新的终端，输入以下命令（保持运行，不要关闭）：

```bash
roscore
```

### 5. 另开一个新的终端，启动四足机器人仿真测试节点：

```bash
cd ~/catkin_ws
source devel/setup.bash
rosrun quadruped_mujoco_rl main_test.py
```

(注：如果是虚拟机运行，建议在执行上述命令前，先在终端执行 export MUJOCO_GL=osmesa 防止渲染闪退。运行成功后，终端将持续打印仿真时间和机器人基座位置。)
## 六、运行效果
![运行效果图](https://github.com/user-attachments/assets/424ffe9c-b756-4da7-8b9b-e2f0065db49a)

![运行效果图](https://github.com/user-attachments/assets/f38e0ccf-6e57-44f5-9c78-a46d55afc5ac)
## 七、实验总结

本次实验成功在 ROS 框架下配置了 MuJoCo 物理引擎环境，加载了四足机器人 MJCF 模型，实现了基于 Python 节点的物理仿真步进。

实验证明了 ROS 节点能够与 MuJoCo 物理引擎进行有效的数据交互，通过终端能够实时输出机器人的仿真时间与基座位置，验证了物理仿真回路的连通性。该实验为后续接入强化学习（PPO）算法及进一步实现机器人自主步态控制奠定了坚实的工程基础，达到了阶段性实验目的。
