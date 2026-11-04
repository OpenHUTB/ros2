# MuJoCo 双自由度机械臂画正方形

## 功能

2R平面机械臂通过逆运动学控制，末端执行器画出正方形轨迹。

## 运行方式

### 有显示器（可视化窗口）
```bash
python3 two_dof_arm_square.py
python3 two_dof_arm_square.py --headless

## 作业要求

- 机械臂：2自由度平面关节臂
- 任务：末端执行器画出正方形轨迹
- 带红色轨迹标记点

## 文件说明

| 文件 | 说明 |
|------|------|
| `two_dof_arm_square.py` | 控制脚本 |
| `two_dof_arm_square.xml` | MuJoCo模型文件 |
