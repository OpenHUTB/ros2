# 基于 ROS 2 的禁飞区（电子围栏）动态检测与安全预警系统

## 一、 项目简介
本项目是一个运行于 ROS 2 框架下的独立安全监控层模块。系统旨在为无人机（Air）等移动机器人提供实时的禁飞区（电子围栏）检测。它不仅支持静态围栏配置，还支持在运行时动态接收禁飞区多边形数据。当检测到载具接近或侵入禁飞区时，系统将输出分级安全预警，并可通过底层控制指令接管，强制载具悬停、刹停或返航（RTL），以保障飞行安全。

## 二、 代码目录结构
项目采用标准 ROS 2 工作空间结构，核心算法封装在独立的 Package 中：

```text
src/
├── air_safety_geofence/          # ★ 禁飞区核心功能包
│   ├── config/
│   │   └── geofence_params.yaml  # 配置文件（安全距离、响应动作等）
│   ├── launch/
│   │   └── geofence.launch.py    # 启动脚本
│   ├── msg/                      # 自定义报警消息（等级、距离等）
│   ├── src/
│   │   ├── geofence_node.cpp     # 核心检测节点（C++）
│   │   └── polygon_checker.cpp   # 几何算法（射线法判断）
│   └── package.xml               # 依赖声明
```
## 三、 核心算法与逻辑
本系统主要采用射线法（Ray Casting）进行空间拓扑判断，并结合距离计算实现分级预警：

1.数据获取：订阅 /odom（载具实时位姿）与 /geofence（动态围栏边界）。

2.空间判断：判断当前位姿是否在多边形内部；同时计算当前位置到各边界的最短距离。

3.安全响应策略：

 SAFE（安全）：载具在安全区域。

 WARN（警告）：载具进入安全缓冲区（如距边界 5 米内），触发降速或预警提示。

 CRITICAL（危险）：载具侵入禁飞区内部，立即发布零速指令或触发紧急返航。
核心伪代码：
```python
def odom_callback(odom_msg):
    x, y = odom_msg.pose.position.x, odom_msg.pose.position.y
    if not geofence_points: return
    
    is_inside = point_in_polygon(x, y, geofence_points)
    distance = min_distance_to_polygon(x, y, geofence_points)
    
    if is_inside:
        publish_alert("CRITICAL", "已侵入禁飞区！")
        publish_cmd_vel(0.0, 0.0, 0.0) # 紧急制动
    elif distance < SAFE_MARGIN:
        publish_alert("WARN", "接近禁飞区边界")
    else:
        publish_alert("SAFE", "安全")
```

## 四、 测试与验证过程

已在 Ubuntu + ROS 2 Humble 环境中完成最小闭环验证：

1.启动禁飞区监控节点。

2.模拟发布载具位置话题 /odom 与矩形禁飞区话题 /geofence。

3.验证结果：载具位于 (0,0) 时系统输出 [SAFE]；通过指令将载具移动至围栏内 (3,3) 时，系统成功触发 [CRITICAL] 闯入报警，验证了动态检测与安全逻辑的有效性。

## 五、 未来扩展 (Roadmap)
□ 引入轨迹预测，对规划路径进行预检。
□ 支持 RViz 可视化，直观展示围栏与载具足迹。
□ 适配不同飞控（如 PX4 / MAVROS），进行实机联调。

![运行效果]<img width="2358" height="1360" alt="ROS Noetic Humble Gazebo 11-2026-10-18-09-46-14" src="https://github.com/user-attachments/assets/27102189-acaf-4f51-8d6a-e948fdbe6fdf" />
<img width="2358" height="1360" alt="ROS Noetic Humble Gazebo 11-2026-10-18-09-47-35" src="https://github.com/user-attachments/assets/2450422d-350c-4176-b594-c593f949576d" />

