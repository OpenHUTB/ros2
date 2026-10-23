# 空域载具（Air）模块

本目录为无人机（空域载具）相关模块的文档集合。

## 无人机键盘飞行控制

### 环境

- Ubuntu 16.04
- ROS Kinetic
- AirSim 无人机仿真

### 安装依赖

```bash
pip install pynput
```

### 运行步骤

1. 启动无人机仿真环境
2. 编译工作空间：

   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```
3. 一键启动：

   ```bash
   roslaunch drone_keyboard_control drone_keyboard_control.launch
   ```

### 按键说明

| 按键 | 动作 |
| :--- | :--- |
| W | 前进 |
| S | 后退 |
| A | 左转 |
| D | 右转 |
| Q | 上升 |
| E | 下降 |

> 详细说明见 [无人机键盘控制](./drone_keyboard_control.md)。
