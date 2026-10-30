# ROS Noetic Web远程遥控小车

本模块在Gazebo仿真环境中搭建差速小车模型，搭载RGB相机；通过`rosbridge\_server`实现网页与ROS的WebSocket通信，`web\_video\_server`推送相机图像流，网页端支持W/A/S/D键盘遥控小车，实时查看车载相机画面。


## 运行环境

- Ubuntu 20.04
- ROS Noetic
- Python 3.8
- 依赖：`rospy`、`rosbridge\_server`、`web\_video\_server`、`xacro`、`gazebo\_ros`


## 构建

```bash

mkdir -p \~/ros1\_ws/src

cp -r <仓库路径>/src/web\_remote\_car \~/ros1\_ws/src/

cd \~/ros1\_ws

source /opt/ros/noetic/setup.bash

catkin\_make

source devel/setup.bash

```

## 启动
```
roslaunch web\_remote\_car start\_all.launch

```

## 查看同步结果

```

rostopic hz /camera/image\_raw

rostopic echo /cmd\_vel

```
浏览器打开地址：`http://127.0.0.1:8000`
 控制按键：W 前进、S 后退、A 左转、D 右转
默认配置下，相机图像输出约 30Hz，`/cmd\_vel`由按键事件按需发布。


## 参数

参数位于`launch/params.yaml`：

表格
| 参数 | 默认值 | 说明 |

| --- | --- | --- |

| `rosbridge\_port` | `9090` | rosbridge WebSocket 端口 |

| `video\_server\_port` | `8080` | web\_video\_server 端口 |

| `camera\_topic` | `/camera/image\_raw` | 相机图像话题 |

| `linear\_speed` | `0.5` | 小车线速度 |

| `angular\_speed` | `0.8` | 小车角速度 |


## 测试
```

cd \~/ros1\_ws

source /opt/ros/noetic/setup.bash

source devel/setup.bash

catkin\_make run\_tests\_web\_remote\_car

catkin\_test\_results build/test\_results/web\_remote\_car

```