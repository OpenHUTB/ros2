# MuJoCo小车多目标导航

## 项目说明
小车自动依次走到三个预设目标点，到达后打印目标坐标，全部走完后自动停车。

## 运行环境
- Ubuntu 20.04
- MuJoCo 2.3.7
- Python 3.8

## 运行步骤
```bash
pip install mujoco==2.3.7
python3 main.py
```

## 运行效果
<img width="1516" height="991" alt="Peek 2026-11-08 19-09" src="https://github.com/user-attachments/assets/d287b3e1-b50b-4718-8e5a-83dbadb95392" />
小车从原点出发，依次走到三个目标点，终端打印当前位置和到达提示。
