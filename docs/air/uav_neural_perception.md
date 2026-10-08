# uav_neural_perception：无人机神经网络视觉感知与运动控制

> 第二次作业（任务②）：无人机机载前视 RGB 摄像头图像 → CNN 神经网络感知 → 运动控制。

## 系统架构
前视摄像头图像 → /camera/image_raw → CNN推理[vx,vy,yaw] → /uav/cmd_vel → 无人机运动

## 算法原理
采用行为克隆（模仿学习）思路：
1. 数据采集：手动键盘控制无人机飞行，同步录制前视图像与控制指令
2. 网络结构：3层卷积层提取视觉特征，2层全连接回归3维控制量
3. 部署推理：订阅/camera/image_raw，预处理→CNN推理→安全限幅→发布/uav/cmd_vel

网络结构：Conv2d(3→16) → Conv2d(16→32) → Conv2d(32→64) → Linear(1024→128) → Linear(128→3)

## 运行环境
- Ubuntu 20.04 + ROS Noetic
- PyTorch（CPU）、OpenCV、cv_bridge

## 运行步骤
1. 编译：cd ~/catkin_ws && catkin_make
2. 启动桥接：roslaunch carlair_ros_bridge main.launch
3. 采集数据：rosrun uav_neural_perception dataset.py
4. 训练：python3 scripts/train.py --csv ~/perception_dataset/data.csv --epochs 30
5. 部署：roslaunch uav_neural_perception main.launch

## 本地测试（无需仿真器）
python3 scripts/model.py --selfcheck
python3 tests/test_local.py

## 声明
本项目使用大模型辅助撰写，作者对全部提交内容负全部责任。
