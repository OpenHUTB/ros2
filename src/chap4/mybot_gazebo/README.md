# mybot_gazebo

本功能包用于第 4 章 Gazebo 机器人平台搭建实验，包含：

- `urdf/mybot.urdf.xacro`：两轮差速移动机器人、激光雷达和 RGB 相机模型；
- `worlds/mybot_empty.world`：最小 Gazebo 世界；
- `launch/mybot_gazebo.launch`：启动 Gazebo、加载模型并发布 TF；
- `scripts/move_square.py`：让机器人按正方形轨迹运动的最小示例。

## 运行

推荐直接使用 `main.sh` 入口：

```bash
./main.sh
```

也可以手动启动：

```bash
mkdir -p ~/catkin_ws/src
cp -r src/chap4/mybot_gazebo ~/catkin_ws/src/   # 在仓库根目录执行
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch mybot_gazebo mybot_gazebo.launch
```

另开终端运行：

```bash
source ~/catkin_ws/devel/setup.bash
rosrun mybot_gazebo move_square.py
```

验证话题：

```bash
rostopic list
rostopic echo /odom
rostopic echo /scan
rosrun tf tf_echo odom base_link
```
