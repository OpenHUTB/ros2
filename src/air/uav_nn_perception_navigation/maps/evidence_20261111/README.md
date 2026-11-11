# 2026 年 11 月 11 日 SLAM 地图采集记录

使用 AirSim 1.8.1 Blocks、ROS Noetic / Ubuntu 20.04，配置文件为本包 `config/settings.json`：`Drone_1` 的 `lidar_1` 水平视场为 -90° 到 +90°。使用提交 `5ccc7e6` 的 `slam.launch`，传入 `airsim_ip`、`altitude:=4`、`goal_x:=0`、`goal_y:=8`，先调用 `/uav/takeoff`，随后采样 `/uav/lidar/scan`、`/uav/odom`、`/map_metadata` 并以 `rosrun map_server map_saver -f .../uav_map` 保存地图。

`slam.launch` 的 gmapping 初始边界为 ±60 m，分辨率为 0.1 m，粒子数为 30。记录的地图为 1184×1184 格，origin 为 (-60, -60, 0)，PGM 共 1,401,910 字节，SHA-256 为 `8309e36152820c6f1d8b209ba9188d1200e31e18695da40dae77bf716726fce2`。按 map_saver PGM 像素值统计：占据 0 共 1801 格、未知 205 共 1151729 格、空闲 254 共 248326 格。旁边的文本文件为采集到的一次消息样本，不构成完整轨迹或全程日志。

运行过程中出现过 scan matching failed 并回退里程计，不能以此地图证明整个回合匹配稳定；尚未保存完整的逐帧匹配分数或与基准地图的误差评估。每帧点云以查询时的估计位置和偏航补偿，并非逐点去畸变。未命中像素被填为最大量程，不代表无障碍。测试结果只适用于该次仿真，不保证独立复跑得到相同地图。
