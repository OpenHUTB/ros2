# 输电线路无人机巡检任务预检

该工具在巡检任务进入 AirSim、PX4 或真实飞控前读取任务 JSON，检查航点结构、飞行高度、相邻航段距离、杆塔双侧覆盖和相机偏航角，并生成独立的预检报告。

## 检查内容

- 航点必须包含杆塔编号、观察侧、三维坐标和相机偏航角；
- 坐标必须是有限数值，杆塔编号必须是正整数；
- 飞行高度必须处于配置的安全范围内；
- 相机偏航角必须处于 `[-180, 180]`；
- 过长航段和缺少单侧观察点会生成警告；
- 报告包含可执行性、错误、警告、航点数、杆塔数、总航程和最长航段。

## 运行

```bash
colcon build --packages-select powerline_mission_preflight
source install/setup.bash
ros2 run powerline_mission_preflight check \
  src/air/powerline_mission_preflight/config/demo_mission.json \
  --min-altitude 5 --max-altitude 120 --max-leg-distance 80
```

默认报告写入 `preflight_report.json`。工具只读取任务文件，不连接、解锁或控制飞行器。
