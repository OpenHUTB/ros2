# 人形机器人落地冲击缓冲：膝关节阻抗与刚性保持对比

双足人形机器人从一定高度自由跌落后，膝关节采用**阻抗控制（虚拟弹簧-阻尼力矩）**与**刚性位置保持**两种策略的落地冲击对比实验。模块通过 PyBullet 物理仿真发布落地过程采样，评估两种策略在地面反力、屈膝行程、姿态偏移与恢复时间上的差异。

## 原理

- 机器人以蹲姿（髋 = -θ/2，膝 = θ，踝 = -θ/2，脚掌保持水平）从指定高度释放，自由跌落后双脚触地。
- **刚性保持**：髋/膝/踝全部位置控制在蹲姿角，膝关节不承担缓冲功能。
- **阻抗缓冲**：髋/踝保持位置控制，膝关节切换为力矩控制 `τ = -kp·(q-q₀) - kd·q̇`（按力矩上限截断），触地后膝关节被动屈曲吸收能量，再缓慢回弹到蹲姿。
- 采样话题 `landing_sample`（`std_msgs/Float64MultiArray`）：`[t, grf, base_vz, base_z, base_pitch, knee_flexion]`。

### 指标定义

| 指标 | 含义 |
| --- | --- |
| 首峰 GRF | 触地瞬间最大地面反力，由接触力学与足端质量主导（两种策略接近，属预期现象） |
| 主动峰 GRF | 触地 50 ms 后的最大地面反力，反映身体减速过程，是缓冲策略的主要评价指标 |
| 冲量 | 触地后 0.3 s 内 GRF 的梯形积分，用于动量守恒校验（两种策略应一致） |
| 最大屈膝 | 落地过程中的最大膝关节角，反映缓冲行程 |
| 最大俯仰 | 底座俯仰角最大偏移，反映姿态稳定性 |
| 恢复时间 | 触地后最后一次超出稳定判据（速度/GFR 偏离静重）的时刻 |

## 运行环境

- Ubuntu 20.04 + ROS 2 Foxy
- Python 3.8+，`numpy`、`matplotlib`
- `pybullet` 需单独安装：`pip3 install pybullet`（或 `sudo apt install python3-pybullet`）
- 无显示器环境自动使用 DIRECT 模式，`use_gui:=true` 可打开图形窗口

## 运行步骤

```bash
# 编译
cd ~/ros2_ws
colcon build --packages-select humanoid_landing_impedance
source install/setup.bash

# 单元测试（新版 setuptools 不再暴露 tests_require，需显式指定 pytest 框架）
colcon test --packages-select humanoid_landing_impedance --python-testing pytest
colcon test-result --test-result-base build/humanoid_landing_impedance

# 若本机 ~/.local 的 pytest 插件与系统 pytest 冲突，可加环境变量后再执行：
#   export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1

# 单次实验（可换 controller_mode:=stiff、drop_height、use_gui:=true）
ros2 launch humanoid_landing_impedance main.launch.py \
  controller_mode:=impedance drop_height:=0.05

# 完整对比实验（两种控制器 × 三个跌落高度，生成 CSV 与对比图）
cd src/humanoid/humanoid_landing_impedance
./run_eval.sh
```

输出文件（`results/`，已加入 `.gitignore`）：

- `samples_<mode>_<drop>.csv`：每次落地的完整采样序列
- `metrics.csv`：指标汇总表
- `effect.png`：GRF 曲线 + 主动峰/屈膝行程对比柱状图

## 运行结果

以下为 `run_eval.sh` 在 Ubuntu 20.04 + ROS 2 Foxy 虚拟机中实际运行的结果（两种控制器 × 三个跌落高度）：

| 跌落高度(m) | 控制器 | 首峰GRF(N) | 主动峰GRF(N) | 冲量(N·s) | 最大屈膝(rad) | 最大俯仰(rad) | 恢复时间(s) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 0.03 | 阻抗缓冲 | 729.26 | 46.24 | 10.04 | 0.34 | 0.04 | 0.15 |
| 0.03 | 刚性保持 | 772.94 | 144.80 | 9.96 | 0.30 | 0.00 | 0.15 |
| 0.05 | 阻抗缓冲 | 912.60 | 88.58 | 10.26 | 0.34 | 0.04 | 0.18 |
| 0.05 | 刚性保持 | 957.07 | 259.23 | 10.17 | 0.30 | 0.00 | 0.20 |
| 0.10 | 阻抗缓冲 | 1004.08 | 60.36 | 11.14 | 0.34 | 0.03 | 0.20 |
| 0.10 | 刚性保持 | 1048.59 | 68.29 | 11.05 | 0.30 | 0.01 | 0.20 |

主要结论：

- 0.05 m 跌落时，阻抗缓冲将主动峰 GRF 从 259.23 N 降到 88.58 N，降低约 **66%**；0.03 m 时降低约 68%。
- 首峰 GRF 两种策略接近（由接触力学主导），冲量基本一致（动量守恒校验通过）。
- 阻抗缓冲通过约 0.04 rad 的额外屈膝行程吸收冲击能量，姿态俯仰保持在 0.04 rad 以内。

![落地缓冲对比效果](../../../../../docs/img/humanoid/humanoid_landing_impedance/effect.png)

## 已知限制

- 首峰 GRF 主要由接触刚度与足端质量决定，两种策略差异很小；缓冲效果体现在主动峰与后续过程。
- PyBullet 关节默认带目标速度为 0 的隐藏速度电机，进行 `TORQUE_CONTROL` 前必须先用 `VELOCITY_CONTROL(force=0)` 关闭，否则力矩完全无效（本模块已在代码中处理）。
- 与 `humanoid_zmp_walking` 的同名模型相比，URDF 关节速度上限由 1.0 提升到 4.0 rad/s：落地缓冲需要更快的膝关节屈曲速度，否则冲击能量全被接触模型吸收，关节缓冲来不及参与。
