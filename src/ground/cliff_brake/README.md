# cliff_brake —— 悬崖边缘紧急制动仿真

基于 **MuJoCo** 物理引擎与 **ROS Noetic** 的地面无人车防跌落安全仿真：车头测距 → 滞回决策 → 力控制动，构成三节点 ROS 闭环。底盘带 z 方向跌落自由度，越过悬崖边缘会真实坠落。

## 依赖

- Ubuntu 20.04 + ROS Noetic（`rospy`、`std_msgs` 为 ROS 自带）
- Python 3.8+，Python 依赖见 `requirements.txt`

```bash
pip3 install -r requirements.txt
```

## 编译与运行

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch cliff_brake main.launch
```

或直接使用入口脚本（注意切到模块目录）：

```bash
cd ~/catkin_ws/src/ground/cliff_brake
./main.sh
```

无需 ROS 与图形界面的定量制动测试（安全包络 + 坠落对照）：

```bash
cd ~/catkin_ws/src/ground/cliff_brake
python3 brake_test.py
```

## 文档

详细原理、参数说明与实测数据见 [docs/ground/cliff_brake.md](../../../docs/ground/cliff_brake.md)。
