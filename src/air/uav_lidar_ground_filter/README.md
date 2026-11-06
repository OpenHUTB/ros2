# uav_lidar_ground_filter

订阅 `/lidar/points`，RANSAC 分离地面与障碍：

- `/lidar/ground`：地面点；
- `/lidar/obstacles`：障碍点。

```bash
roslaunch uav_lidar_ground_filter main.launch distance_threshold:=0.2
```

测试：`python3 test/test_ground_filter_local.py`。
