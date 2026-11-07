# 人形机器人手臂逆运动学作业

> 7 自由度人形机械臂正运动学、三种逆解算法、Yoshikawa 可操作性分析
> 纯 Python 实现，仅依赖 numpy + matplotlib

## 作业内容

实现一个 7 自由度（肩3 + 肘1 + 腕3）人形机械臂的逆运动学求解器，包含：

1. **正运动学**：DH 参数建模，输入关节角输出末端位姿
2. **几何雅可比**：数值差分计算 J = ∂p/∂q
3. **三种逆解算法**：
   - 伪逆法（pseudoinverse）
   - 阻尼最小二乘法（damped LS / Levenberg-Marquardt 风格）
   - 零空间投影（主任务追末端 + 零空间优化关节偏好）
4. **Yoshikawa 可操作性分析**：可操作性指标 w、椭圆半轴、条件数
5. **单元测试**：6 项测试全部通过
6. **可视化**：收敛曲线对比、可操作性椭圆、手臂轨迹

## 目录结构

```
humanoid_ik_assignment/
├── ik/
│   ├── __init__.py
│   ├── robot.py            # 7-DOF DH 模型 + 正运动学
│   ├── jacobian.py          # 几何雅可比（数值差分）
│   ├── solver.py            # 三种逆解：伪逆/阻尼LS/零空间
│   └── manipulability.py   # Yoshikawa 可操作性椭圆
├── tests/
│   └── test_ik.py          # 6 项单元测试
├── output/                  # 生成的图像
│   ├── convergence_compare.png    三种方法收敛曲线对比
│   ├── manipulability_ellipse.png  可操作性椭圆 3D
│   ├── arm_trajectory.png         手臂构型与末端轨迹
│   └── manipulability_vs_q.png  可操作性 vs 关节角
├── demo.py                  # 跑 demo 生成图像
└── README.md
```

## 运行

```bash
# 跑单元测试
python tests/test_ik.py

# 生成可视化图像
python demo.py
```

## 实验结果

| 方法 | 收敛步数 | 最终误差 |
|---|---|---|
| 伪逆法 | 15 步 | ~1e-5 m |
| 阻尼最小二乘 | 62 步 | ~1e-5 m |
| 零空间投影 | 15 步 | ~1e-5 m |

零空间投影法在收敛速度与伪逆法相当的同时，最终关节角更接近偏好位姿（距离 0.354 vs 伪逆 0.398）。

## 理论要点

- **伪逆法**：Δq = J⁺ e，收敛快但奇异点附近数值不稳定
- **阻尼最小二乘**：Δq = Jᵀ (JJᵀ + λ²I)⁻¹ e，奇异点附近更稳定，收敛略慢
- **零空间投影**：Δq = J⁺ e + (I - J⁺J) q_pref，主任务追末端，零空间优化次级目标（避关节限位）
- **Yoshikawa 指标**：w = √det(JJᵀ)，w 越大该方向可操作性越好；w→0 表示接近奇异位形
