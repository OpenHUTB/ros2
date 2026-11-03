# uav_camera_frustum_viz

RViz 可视化：相机视锥 + 飞行轨迹 + 无人机位置。

```bash
roslaunch uav_camera_frustum_viz main.launch near:=0.5 far:=8.0
```

订阅 `/uav/odom`、`/camera/camera_info`，发布 `/uav/camera_frustum`（`MarkerArray`）。
测试：`python3 test/test_frustum_local.py`。
