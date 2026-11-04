# MuJoCo 肌肉驱动下肢：从单关节到负重踏步

用 **Hill 肌肉**（而不是电机）驱动下肢，覆盖六级递进实验：单关节摆动、拮抗肌对跟踪、
共激活刚度、双关节耦合、负重站立、原地踏步。

MuJoCo 只负责刚体动力学与地面接触，肌肉模型（激活、力-长度、力-速度、被动力）
由 Python 实现，因此肌肉部分完全可单元测试，也不会受 MuJoCo 内置 muscle 执行器
归一化方式的版本差异影响。

## 运行环境

| 依赖 | 版本 | 说明 |
|---|---|---|
| Python | 3.8+ | 实测 3.14.3 |
| MuJoCo | 3.0+ | 实测 3.16.0，`pip install mujoco` |
| numpy | 1.20+ | 实测 2.5.4 |
| Pillow | 9+ | 仅 `gif` 子命令需要 |

本模块**不依赖 ROS 运行**；仓库内提供 `launch/` 与 `package.xml` 只是为了与其他
模块统一管理，`main.py` 可独立执行。

## 运行

```bash
cd src/musculoskeletal/mujoco_muscle_leg_gait

python3 main.py check                 # 四个模型自检（单关节/双关节/单腿/双腿）
python3 main.py run --scene l3        # 跑单个实验，打印指标 JSON
python3 main.py bench --out out       # 跑全部六级，导出 out/metrics.json
python3 main.py gif --out gif         # 渲染 L1/L2/L5/L6 动图
```

单元测试（不需要 ROS、不需要显卡）：

```bash
python3 -m unittest discover -s tests -v
```

实测 17 项全部通过，耗时约 0.4 s。

## 六级实验

| 级别 | 内容 | 关键指标 |
|---|---|---|
| L1 | 单条肌肉阶跃 | 峰值角 0.828 rad，峰值肌力 32.2 N |
| L2 | 拮抗肌对 + PD 跟踪 | 目标 ±0.30 rad，末态 0.277 / −0.220 |
| L3 | 共激活 → 关节刚度 | 上升段 0.011→0.212 N·m/rad，下降段 0.327→0.033 |
| L4 | 双关节腘绳肌耦合 | 髋峰值误差：单关节 0.120 / 朴素 0.331 / 联合QP 0.204 rad |
| L5 | 单腿负重站立（100% 重力） | 骨盆 −0.015 m，膝 0.21 rad，峰值激活 0.24 |
| L6 | 双腿原地踏步（35% 体重支撑） | 骨盆 +0.007 m，抬脚 0.180 m，峰值激活 0.38 |

## 文件结构

```
mujoco_muscle_leg_gait/
├── main.py                 # 统一入口（check/run/bench/gif）
├── main.sh
├── package.xml / setup.py  # ament_python 包装（可选）
├── launch/main.launch.py
├── resource/
├── mujoco_muscle_leg_gait/
│   ├── core.py             # Hill 肌肉、MJCF 生成、肌肉分配
│   ├── runs.py             # L1-L6 实验与录制
│   └── cli.py              # 命令行实现
└── tests/
    ├── test_core.py        # 肌肉模型、分配、QP
    └── test_runs.py        # L3 解析互验 + L1/L5/L6 集成冒烟
```

## 已知限制

1. **L6 使用 35% 体重支撑**。骨盆被两个滑轨（水平 + 竖直）约束，没有躯干俯仰与
   重心转移控制，完全负重会失稳。这一步只验证"节律与承重"，不等于自由行走。
2. **单关节阶段用解析矩臂**（`L = L0 ∓ rθ`）。真实肌肉靠肌腱缠绕获得随构型变化的
   力臂，这在 L4 之后的多关节场景才必要。
3. **肌肉参数为缩放模型**（总质量约 11 kg、`F_max=600 N`），不对应具体人体数据。
4. **未接入 ROS 话题**。当前状态下 ROS 只是启动包装，没有发布关节状态。

## 参考

- A. V. Hill, *The heat of shortening and the dynamic constants of muscle*, 1938
- F. E. Zajac, *Muscle and tendon: properties, models, scaling, and application to
  biomechanics and motor control*, 1989
- D. G. Thelen, *Adjustment of muscle mechanics model parameters to simulate dynamic
  contractions in older adults*, 2003
