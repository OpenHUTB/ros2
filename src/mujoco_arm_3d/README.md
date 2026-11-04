# mujoco_arm_3d：三自由度机械臂空间螺旋线仿真

基于 MuJoCo 的三自由度机械臂（肩方位 yaw + 肩俯仰 shoulder + 肘俯仰 elbow）仿真：
末端在 3D 空间中沿**圆柱螺旋线**运动（水平画圆 + 垂直匀速上升），
解析空间 IK 实时解算三个关节角，PD 位置控制驱动关节，
渲染合成 demo_arm3d.gif 效果图。

## 算法

- **空间逆运动学（ik3）**：由末端位置 (x, y, z) 先求水平方位角 q0 = atan2(y, x)，
  再在臂平面（r-z）内用两连杆解析解求肩角 q1、肘角 q2；
  yaw 角做解缠处理保证多圈连续旋转（1.5 圈 / 540°）。
- **正运动学（fk3）**：q0、q1、q2 -> 末端世界坐标，用于与仿真器互查验证。
- **PD 位置控制**：三关节独立 PD（KP=40, KD=12），每帧按目标关节角输出力矩。
- **透视投影**：按相机参数（azimuth/elevation/distance/fovy）手算 lookAt + 透视矩阵，
  把末端世界坐标投影为像素，叠加黄色轨迹线。

## 文件

| 文件 | 说明 |
|---|---|
| `mujoco_arm_3d_single.py` | 主程序（XML 内嵌，单文件可直接运行） |
| `verify_arm3d.py` | 验证脚本：`ik`（IK 精度）与 `gui`（glfw 实时渲染） |
| `docs/mujoco_arm_demo.gif` | 演示效果图（虚拟机实测录屏） |
| `docs/mujoco_arm_3d.md` | 模型、算法与验证详细说明 |

## 运行

```bash
pip install mujoco pillow numpy
python3 mujoco_arm_3d_single.py        # 生成 demo_arm3d.gif
python3 verify_arm3d.py ik             # IK 数值精度验证
python3 verify_arm3d.py gui            # glfw 实时渲染验证（弹出窗口）
```

无图形界面环境（虚拟机）用 osmesa：

```bash
MUJOCO_GL=osmesa python3 mujoco_arm_3d_single.py
```

## 验证结果（本机实测）

| 项 | 结果 |
|---|---|
| IK 数值精度 | 螺旋线 8 个空间点 逆解→正解互查，最大误差 **0.0000 mm** |
| 实时渲染 | glfw 查看器完整播放 **216 帧 / 9.0 秒** |
| 效果图 | docs/mujoco_arm_demo.gif（虚拟机实测：半径 0.15 m，高度 0.20→0.40 m，1.5 圈螺旋） |
