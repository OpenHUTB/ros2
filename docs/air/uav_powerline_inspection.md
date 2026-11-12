# 无人机输电线路双侧巡检航线预览

该示例根据输电杆塔的局部 ENU 坐标，为每座杆塔生成左右两个观察航点。相机偏航角始终指向杆塔，飞行高度由杆塔高度与安全余量共同确定。相邻杆塔的观察顺序交替，减少跨越线路的无效移动。

## 功能

- 校验杆塔坐标、杆塔高度、安全距离和飞行速度；
- 根据线路局部方向计算左右法向观察点；
- 输出观察侧、三维坐标、相机偏航角和对应杆塔编号；
- 汇总航点数量、三维航线长度与理想飞行时间；
- 离线生成 JSON 预览，不连接或解锁真实飞行器。

## 运行

```bash
colcon build --packages-select uav_powerline_inspection
source install/setup.bash
ros2 run uav_powerline_inspection preview \
  --pole-count 6 --pole-spacing 45 --pole-height 20 \
  --bearing-deg 20 --standoff 8 --height-margin 2 --speed 4
```

默认结果写入 `powerline_route.json`，可在接入 AirSim 或真实飞控前检查每个观察点。

## 与区域勘测的区别

区域勘测使用平行往返航线覆盖矩形区域；本示例围绕离散杆塔生成双侧近距观察点，并为每个航点计算面向目标的相机方向，适用于线性基础设施巡检。
