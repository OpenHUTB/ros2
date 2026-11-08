# 无人机载荷质量估计

本页记录一个可复现的载荷质量估计实验：用飞行中的**推力指令**与**竖直速度响应**，估计无人机当前搭载物的质量（kg）。同一个推力下，载荷越重、竖直加速越慢——模型要学的就是这个关系。

实验在 MuJoCo 里做：把同一架四旋翼的机身质量改成"空机 + 载荷"，用固定参数的控制器飞出一段高度轨迹，记录推力与速度，再用神经网络和物理公式分别反推载荷。

![ROS2 回放中的质量估计：三条曲线对应三次飞行，虚线是真实载荷](../img/air/payload_mass_estimation/demo.gif)

上图的每一帧都来自真实运行：`payload_estimator` 节点订阅回放话题，逐条发布 `/payload/estimate`，共 663 条估算。三次飞行的真实载荷分别是 0.207、0.219、0.368 kg。

---

## 1. 任务定义与设计取舍

**输入**：最近 20 个采样点（20 Hz，约 0.95 秒）的推力指令与竖直速度。

**输出**：载荷质量估计值（kg），范围 0～0.4 kg。

**边界**：只做**水平姿态下的竖直飞行**，且空机质量与推力标定已知。这里的"载荷"是附加质量，与着陆时的接地力不是一回事。

三个关键取舍：

| 取舍 | 选择 | 理由 |
|---|---|---|
| 特征 | 推力序列 + 速度差分加速度序列 | 动力学上，推力与加速度之比直接给出质量；网络不必自己学微分 |
| 网络 | 小 MLP（38→48→24→1） | 输入已是物理量，样本只有 90 次飞行；深层网络没有增益 |
| 对照 | 同一窗口的冲量物理基线 | 这个问题有闭式解，**必须**与它比较，否则无法说明神经网络的价值 |

### 1.1 与社区模型的关系

四旋翼模型复用 OpenHUTB 社区 [PR #178](https://github.com/OpenHUTB/ros2/pull/178)，版本与许可见 `models/source.json` 与 `THIRD_PARTY_NOTICES.md`。**质量变化实验、数据采集、训练与 ROS2 估计程序都是本项目新增的**。

### 1.2 功能与入口

| 阶段 | 入口 | 输出 |
|---|---|---|
| 自检 | `bash main.sh doctor` | `artifacts/environment.json` |
| 数据 | `bash main.sh prepare` | 解压语料到 `data/` |
| 采集 | `bash main.sh collect` | 180 次 MuJoCo 飞行（另存目录） |
| 训练 | `bash main.sh train` | `models/<name>/model.pt`、`meta.json`、`loss.csv` |
| 评估 | `bash main.sh evaluate` | `results/metrics.json`、逐窗口 CSV、2 张图 |
| 测试 | `bash main.sh test` | 8 项单元测试 |
| 构建 | `bash main.sh build` | `install/`、`build/`、`log/` |
| ROS2 | `bash main.sh demo` | 两个话题 + 订阅曲线 |
| 仿真图 | `python render_validation.py` | `results/simulation.png` |

---
