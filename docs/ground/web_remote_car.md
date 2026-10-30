\# 基于Web的远程驾驶与多视角监控

本项目在Gazebo仿真环境搭建搭载RGB相机的差速小车模型，通过 `rosbridge\_server` 实现ROS与前端JS通信，`web\_video\_server` 推送相机图像流；网页端支持键盘控制小车运动，带按键按下可视化反馈，同步展示小车相机视角画面。



源码位置：`src/ground/web\_remote\_car`



\## 1. 项目目标

\- 在Gazebo中搭建搭载RGB相机的差速小车XACRO模型

\- 使用 `rosbridge\_server` 建立ROS与网页前端的WebSocket通信

\- 使用 `web\_video\_server` 发布相机图像流

\- 网页前端实现W/A/S/D键盘遥控小车，按键高亮反馈

\- 网页实时渲染相机画面，实现远程可视化监控



\## 2. 数据流

```text

键盘按键事件 → JS → rosbridge\_server → /cmd\_vel → 小车运动

相机图像话题 /camera/image\_raw → web\_video\_server → HTTP图像流 → 网页展示



\## 3. 运行环境

表格



| 项目 | 说明 |

| --- | --- |

| 操作系统 | Ubuntu 20.04 |

| ROS | ROS Noetic |

| Python | Python 3.8 |

| 依赖 | `rosbridge\_server`、`web\_video\_server`、`xacro`、`gazebo\_ros` |



本项目依赖 Gazebo 仿真环境，不需要真实硬件小车与实体相机。

\## 4. 构建



```

mkdir -p \~/ros1\_ws/src

cp -r <仓库路径>/src/ground/web\_remote\_car \~/ros1\_ws/src/

cd \~/ros1\_ws

source /opt/ros/noetic/setup.bash

catkin\_make

source devel/setup.bash

```



\## 5. 启动



```

roslaunch web\_remote\_car start\_all.launch

```



节点会同时启动 Gazebo 仿真、小车模型生成、`rosbridge\_server`、`web\_video\_server`。



\## 6. 查看结果



新开终端：



```

source /opt/ros/noetic/setup.bash

source \~/ros1\_ws/devel/setup.bash

rostopic echo /cmd\_vel

rostopic hz /camera/image\_raw

```



打开浏览器访问：



```

http://127.0.0.1:8000

```



默认控制按键：W 前进、S 后退、A 左转、D 右转。



默认话题与服务端口：



表格



| 话题 / 服务 | 说明 |

| --- | --- |

| `/cmd\_vel` | 小车速度控制指令 |

| `/camera/image\_raw` | 相机原始图像 |

| 9090 | rosbridge WebSocket 端口 |

| 8080 | web\_video\_server 服务端口 |



控制与相机画面演示：
<img width="1619" height="824" alt="camera_demo1" src="https://github.com/user-attachments/assets/00fdacbe-233d-4f5c-87cb-8a2c3b17ba24" />
<img width="1619" height="824" alt="camera_demo2" src="https://github.com/user-attachments/assets/578d526a-04dc-43c3-ba48-fb4cdfb415e7" />

\## 7. 参数调整



默认启动参数位于 `launch/web\_bridge.launch`：



表格



| 参数 | 默认值 | 说明 |

| --- | --- | --- |

| `port` | `9090` | rosbridge 端口 |

| `video\_port` | `8080` | web\_video\_server 端口 |

| `camera\_topic` | `/camera/image\_raw` | 订阅的相机话题 |



修改端口或者相机话题后，需要同步修改前端 `index.html` 内对应的地址，重启 launch 生效。



\## 8. 实测结果



测试环境：Ubuntu 20.04.6、ROS Noetic 1.17.4、Python 3.8.10。

构建和测试通过：



\- `catkin\_make` 成功；

\- Gazebo 正常加载小车模型，相机画面正常发布；

\- 网页端按键响应流畅，图像流无明显卡顿。



话题实测频率如下：



表格



| 话题 | 实测频率 |

| --- | --- |

| `/camera/image\_raw` | 约 30 Hz |

| `/cmd\_vel` | 按键触发，按需发布 |



\## 9. 测试



```

cd \~/ros1\_ws

source /opt/ros/noetic/setup.bash

source devel/setup.bash

catkin\_make run\_tests\_web\_remote\_car

catkin\_test\_results build/test\_results/web\_remote\_car

```



测试覆盖模型加载、话题发布、通信链路基础检查。



\## 10. 常见问题



表格



| 现象 | 原因 | 处理 |

| --- | --- | --- |

| 找不到 `rosbridge\_server` | Noetic 未安装该包 | `sudo apt install ros-noetic-rosbridge-server` |

| 网页看不到相机画面 | web\_video\_server 未启动 / 话题名不匹配 | 检查 launch，核对 html 里的图像地址 |

| 按键控制小车不动 | `/cmd\_vel` 没有消息 / 小车驱动失效 | 终端 `rostopic echo /cmd\_vel` 排查消息 |

| 修改 launch 后参数不生效 | 节点残留 | 关闭所有 roslaunch，重新启动 |



\## 11. 小结

本项目基于 ROS Noetic 搭建 Web 远程监控遥控系统，`rosbridge` 打通 ROS 后端与前端，`web\_video\_server` 完成图像推流。适合仿真小车远程操控、车载相机可视化场景；可以在此基础上扩展雷达数据可视化、自动避障、AEB 制动算法等功能。

