# vehicle_pygame_teleop
基于 pygame 的 ROS 车辆键盘遥操作功能包。

## 功能
- 键盘方向键控制车辆线速度和角速度
- 空格键紧急停车
- 30Hz 稳定输出控制指令

## 运行环境
- ROS Noetic
- Python 3
- pygame

## 安装依赖
```bash
pip3 install pygame
```

## 运行方式
```bash
roslaunch vehicle_pygame_teleop main.launch
```

## 控制说明
- 方向键 ↑：前进
- 方向键 ↓：后退
- 方向键 ←：左转
- 方向键 →：右转
- 空格键：立即停车
