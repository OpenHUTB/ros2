# MuJoCo 两轮自平衡小车 PID 控制仿真

## 项目介绍
基于 MuJoCo 物理引擎实现的两轮自平衡小车仿真项目，采用双环 PID 控制算法实现倒立摆动态平衡与位置保持。纯 Python 实现，无 ROS 依赖，可直接在 Ubuntu 虚拟机运行。

## 环境依赖
- Ubuntu 22.04（虚拟机兼容）
- Python 3.10+
- 依赖库：mujoco、mujoco-python-viewer、numpy

## 运行方式

### 一键运行
```bash
chmod +x main.sh
./main.sh
```

### 手动运行

```bash
pip install -r requirements.txt
python3 main.py
```

## 操作说明

| 按键 | 功能 |
|------|------|
| R | 重置仿真状态 |
| 方向键 ↑| 给小车施加向前速度扰动 |
| 方向键 ↓| 给小车施加向后速度扰动 |
| ESC| 关闭仿真窗口 |

## 效果说明

1. 启动后摆杆从微小偏移自动恢复垂直，保持动态平衡。
2. 施加前后扰动后，小车可自动回到初始位置，摆杆维持直立。
3. 可通过调整 main.py 中 PID 参数优化平衡响应速度与稳定性。
