# drone_teleop_sim

无人机遥操作与轨迹记录模拟程序。

## 功能

- 模拟多旋翼无人机在 NED 坐标系下的飞行动力学
- 模拟键盘遥控飞行指令流（起飞→巡航→转弯→悬停→降落）
- 实时记录逐帧飞行数据（位置、速度、高度、状态）
- 自动生成轨迹图、高度曲线、速度曲线、状态时间线

## 运行

```bash
python3 drone_teleop_sim.py
```

依赖：numpy, matplotlib

## 输出

- `flight_log.csv` — 逐帧飞行数据
- `trajectory.png` — XY 平面飞行轨迹俯视图
- `altitude.png` — 高度随时间变化曲线
- `velocity.png` — 三轴速度分量曲线
- `state_timeline.png` — 飞行状态时间线
