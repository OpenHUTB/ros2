# 差速小车走迷宫

MuJoCo差速小车穿越L型迷宫作业，从起点(-2,-2)出发，绕过墙壁到达绿色终点(2,2)。

## 效果演示
![小车走迷宫](../../assets/diff_drive_maze.gif)

## 运行方法
```bash
# 可视化模式，打开仿真窗口
python3 maze_solver.py

# 无头模式，不打开窗口，仅终端输出
python3 maze_solver.py --headless
