# MuJoCo 红绿灯启停小车仿真

## 项目简介
基于 MuJoCo 物理引擎的红绿灯响应小车仿真项目。小车沿直线行驶，到达停止线后根据信号灯状态做出停车、减速或继续行驶的决策。

## 环境依赖
- Python 3.8+
- MuJoCo
- NumPy

## 安装依赖
```bash
pip install mujoco numpy
```

## 运行方式
```bash
cd src/ground/mujoco_traffic_light_stop
python3 main.py
```

## 功能说明
- 红绿灯自动循环：红灯5s → 黄灯2s → 绿灯5s
- 小车从左侧起点向右行驶
- 到达停止线前1米开始提前减速
- 到达停止线后：
  - 红灯：刹车停车
  - 黄灯：减速慢行
  - 绿灯：正常前进
- 终端实时打印运行日志（时间、灯状态、位置、速度、推力）

## 运行效果
仿真运行界面：
![红绿灯启停仿真运行界面](../../img/ground/mujoco_traffic_light_stop_running.png)

动态效果：
![红绿灯启停仿真效果](../../img/ground/mujoco_traffic_light_stop_demo.gif)

## 文件结构
```
mujoco_traffic_light_stop/
├── main.py      # 主程序，控制逻辑+日志输出
├── scene.xml    # MuJoCo 场景模型
└── README.md    # 说明文档
```

## 参数调整
| 参数 | 位置 | 说明 |
|------|------|------|
| T_RED | main.py | 红灯时长（秒） |
| T_YELLOW | main.py | 黄灯时长（秒） |
| T_GREEN | main.py | 绿灯时长（秒） |
| stop_line | main.py | 停止线x坐标 |
| slow_down_distance | main.py | 提前减速距离 |
| normal_force | main.py | 正常行驶推力 |
| brake_force | main.py | 刹车推力 |
| slow_force | main.py | 黄灯减速推力 |
