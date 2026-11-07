# Bug2 反应式避障（bug2_wall_following）

演示经典 Bug2 算法在 2D 平面上绕过圆形障碍到达目标。

## 分层

- `geometry.py`：点到线段距离、线段相交、障碍表面最近点。
- `bug2.py`：状态机 GOAL_SEEKING / WALL_FOLLOWING / REACHED，记录 hit_point。
- `main.py`：pygame 可视化，绿色起点、红色目标、灰色障碍、青色轨迹、橙色沿墙段。

## 运行

见包内 README。
