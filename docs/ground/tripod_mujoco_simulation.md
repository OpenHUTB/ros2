# 三足仿生机器人 MuJoCo 物理仿真

---

## 1. 概述与目标

本模块基于 MuJoCo 物理仿真引擎，搭建三足仿生机器人的多刚体动力学仿真环境，验证三足机构的站立稳定性与关节运动特性。

### 核心任务目标
1. **三足多刚体建模**：构建包含中心躯干、三条腿（每条腿含髋关节与膝关节）的完整多连杆模型。
2. **地面接触碰撞仿真**：启用 MuJoCo 接触求解器，模拟足端与地面的摩擦与碰撞动力学。
3. **仿真视频自动渲染**：通过离屏渲染（Offscreen Rendering）自动生成仿真过程 MP4 视频。

---

## 2. 模型结构

### 2.1 整体构型
- **躯干**：球体 + 圆柱体组合，总质量约 1.5 kg
- **腿部数量**：3 条，呈 120° 均匀分布
- **每条腿自由度**：3 个（髋关节侧摆、髋关节旋转、膝关节屈伸）

### 2.2 关节参数
| 关节名称 | 类型 | 运动范围 |
|---------|------|---------|
| 髋关节侧摆 (hip_abd) | 旋转关节 | ±30° |
|髋关节旋转 (hip_rot) | 旋转关节 | ±45° |
| 膝关节 (knee) | 旋转关节 | -90° ~ +10° |

---

## 3. 运行方式

### 3.1 环境依赖
- Python >= 3.10
- MuJoCo 3.x
- imageio + imageio-ffmpeg（用于视频编码）

### 3.2 安装依赖
```bash
cd src/ground/tripod_mujoco
pip install -r requirements.txt
```

### 3.3 运行仿真
```bash
cd src/ground/tripod_mujoco
python3 sim_render.py
```

运行成功后，仿真视频将自动保存至 `output/sim_video.mp4`。

---

## 4. 文件结构

```
tripod_mujoco/
├── model/
│   └── model.xml      # MuJoCo 模型定义文件
├── output/
│   └── sim_video.mp4  # 生成的仿真视频
├── sim_render.py      # 仿真与渲染主脚本
├── requirements.txt   # Python 依赖
└── README.md          # 项目说明
```
