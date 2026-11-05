# 项目 13：机械臂拧螺丝（MuJoCo 仿真）

三自由度空间机械臂（肩方位 + 肩俯仰 + 肘俯仰）末端夹持竖直螺丝刀，
对准桌面木板上的螺丝后连续旋转 2.5 圈（900°），把螺丝按螺距拧入
桌面螺孔（下沉 40 mm，螺丝顶面拧到板面以下）。
3/4 视角相机渲染，画面上方实时显示「旋转圈数 / 拧入深度」状态条，
叠加黄色末端轨迹与青色刀头螺旋轨迹，输出 `demo_armscrew.gif` 效果图。

## 文件

| 文件 | 说明 |
|---|---|
| `mujoco_arm_screw_single.py` | 主程序（单文件，模型 XML 内嵌） |
| `verify_armscrew.py` | 验证脚本（IK 精度 / 全流程贴合与下沉 / glfw 实时渲染） |
| `README.md` | 本项目说明 |
| `docs/mujoco_arm_screw.md` | 设计与验证文档 |
| `docs/mujoco_arm_demo.gif` | 效果图（拧螺丝动画，450 帧 / 15 秒） |

## 运行

```bash
# 本机（Windows glfw 后端）
python mujoco_arm_screw_single.py

# 虚拟机 / 无显卡环境（osmesa 离屏渲染）
# 若系统缺 libOSMesa：sudo apt-get update && sudo apt-get install -y libosmesa6
MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa python3 mujoco_arm_screw_single.py
```

运行结束在同目录生成效果图 `demo_armscrew.gif`（450 帧 / 15.0 秒）。

## 验证

```bash
python3 verify_armscrew.py ik      # IK 精度：三个目标点逆解 -> 正解互查
python3 verify_armscrew.py clear   # 全流程：刀头贴合、转角 900°、下沉 40 mm
python3 verify_armscrew.py gui     # glfw 实时渲染：完整播放拧螺丝
```

本机验证结果：

- IK 互查最大误差 **0.00 mm**（三个目标点全部 PASS）
- 对位+拧入阶段（frame 110–350）刀头-螺丝顶最大间隙 **0.0000 m**（全程贴住）
- 最终转角 **900.0°（2.50 圈）**，螺丝下沉 **40.0 mm**
- 螺丝终了顶面高度 **0.003 m**（板面 0.020 m，完全拧入板内）
- 结果：**PASS**

## 设计要点

- 三自由度空间臂：肩方位 yaw + 肩俯仰 pitch1 + 肘俯仰 pitch2，
  杆长 L1 = L2 = 0.30 m，肩俯仰中心高度 0.06 m
- **构型选择**：目标靠近桌面（d ≈ 0.21 m < 臂长 0.6 m）时二连杆必须大幅
  折叠，IK 取 **负解（q2 < 0，肘抬到肩上方）**，让 L2 从上往下伸到螺丝，
  形成"臂从上方伸下来拧螺丝"的构型；正解会让肘沉到地面以下，杆件整段
  被地面遮住
- 空间 IK：方位角 `yaw = atan2(y, x)`（带跨 ±π 解缠）+ 臂平面内二连杆闭合解
- 螺丝刀：竖直悬挂于末端下方 0.11 m（刀头对准螺丝顶），转角与螺丝同步
- 螺丝：十字槽圆柱，绕 z 轴旋转 theta，同时按螺距匀速下沉
  `sz = 初始高度 − 40 mm × (theta / 900°)`
- 时间线：0–70 帧移向螺丝上方 → 70–110 帧下压对位 → 110–350 帧旋转拧入
  （2.5 圈 / 0.96 s）→ 350–420 帧抬起 → 420–450 帧停留展示
- 臂运动学插值驱动：每帧 qpos 向 IK 目标角逼近（最大 0.07 rad/帧），
  用 `mj_forward` 渲染而不积分动力学（臂模型轻、重力角加速度大，
  动态 PD 无法收敛；运动学驱动保证刀头精确贴合螺丝顶）
- 相机：lookat (0.11, 0.04, 0.14)、distance 0.98、azimuth 60°、elevation −30°，
  黄色末端轨迹与青色刀头轨迹用手算 lookAt 透视投影叠加（fovy = 45°）
