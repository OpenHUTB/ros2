# bug2_wall_following

Bug2 反应式避障 2D 仿真。机器人从起点直线朝目标走，碰到圆形障碍时切换到沿墙跟随模式，绕障碍一周后再次穿过"起点-目标"连线（Lm）、且该点比碰墙点更靠近目标时，切回直线朝目标，直到到达终点。

## 运行

```bash
cd src/ground/bug2_wall_following
python main.py        # Windows 用 main.bat，Linux/macOS 用 main.sh
```

依赖：`pip install -r requirements.txt`（仅 pygame）。

## 测试

```bash
python -m pytest test/test_bug2_logic.py
```
覆盖：初始状态、无遮挡时朝目标移动、被挡进入沿墙、最终到达目标、几何工具。
