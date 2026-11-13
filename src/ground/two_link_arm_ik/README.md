# two_link_arm_ik

包定位说明：本模块是基于 pygame 的独立 2D 仿真演示，不创建 rclpy 节点，也不发布 ROS 话题。

二连杆机械臂正向与逆向运动学 2D 仿真。基座固定，给定末端目标点，用解析逆解算出两个关节角，机械臂逐帧插值过去，末端留下运动轨迹。

对应文档：docs/ground/two_link_arm_ik.md

## 目录结构

```
two_link_arm_ik/
├── main.py                     # 入口，转发到 two_link_arm_ik.main
├── main.bat                    # Windows 启动脚本
├── main.sh                     # Linux 和 macOS 启动脚本
├── package.xml                 # 包声明，ament_python
├── requirements.txt            # 依赖 pygame 和 pytest
├── setup.py                    # 打包与安装路径
├── setup.cfg
├── README.md
├── resource/two_link_arm_ik
├── launch/main.launch.py       # ROS 2 launch 占位入口
├── two_link_arm_ik/
│   ├── __init__.py
│   ├── config.py               # 连杆长度、关节限位与目标点
│   ├── kinematics.py           # 正解、逆解、可达性判定与关节限位裁剪
│   ├── arm.py                  # 机械臂状态机，逐帧插值并按序切换目标
│   └── main.py                 # pygame 渲染与主循环
└── test/test_arm_kinematics.py
```

## 环境

- Windows 11，Python 3.13.16，pygame 2.6.1
- 安装依赖：

```bash
pip install -r requirements.txt
```

## 运行

```bash
cd src/ground/two_link_arm_ik
python main.py
```

Windows 用 main.bat，Linux 和 macOS 用 main.sh。

## 测试

```bash
cd src/ground/two_link_arm_ik
python -m pytest test/test_arm_kinematics.py
```

覆盖：全伸展姿态末端位置、直角姿态末端位置、正向解的逆解回代、工作空间外目标返回空解、边界可达性判定、关节限位裁剪，以及依次抵达全部目标点。

## 实测结果

本地复跑：177 步依次抵达全部 4 个目标点，末端最终误差 0.00 px。

## 限制

- 纯二维几何演示，没有动力学与力矩，不考虑重力、惯量与碰撞。
- 插值按固定比例推进，不是时间最优轨迹。
- 目标点写在 config 中，未做鼠标点选交互。
