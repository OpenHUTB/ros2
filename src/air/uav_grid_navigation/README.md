# 低空无人机二维栅格路径规划

本模块在预先给定的二维占据栅格上使用四邻域 A* 寻路，并按离散格点逐步推进路线。障碍物由输入栅格提供，结果包含航点序列、离散步数和是否到达目标。

这是路径规划算法示例。它不读取 MuJoCo 位姿或传感器数据，也不执行物理飞行、激光建图或 SLAM。

```bash
python navigation.py
python -m pip install -r requirements-dev.txt
python -m pytest -q test_navigation.py
```

默认运行会将路线指标写入 `output/grid_navigation.json`。测试覆盖障碍绕行、不可达目标、输入校验和路线终点。

English: The example plans a four-connected A* route on a supplied occupancy grid and advances along discrete cells. It does not model flight dynamics or consume simulated sensor data.
