# 诊断脚本（diag/）

本目录保存排查 `humanoid_e2e_recovery` 闭环失稳时用过的 15 个诊断脚本。
它们**不是**主流程的一部分，作用是把 README 与 `docs/humanoid/humanoid_e2e_recovery.md`
里的实测数值变成可复现的：每个脚本只回答一个问题，输出可直接与下表对照。

## 前置条件

| 需要 | 说明 |
|---|---|
| 运行环境 | Linux + MuJoCo 4.0.0 + Python 3.12（EGL 离屏渲染，64×64 一帧约 65.6 ms） |
| 数据集 | `data/e2e_dataset.npz`：`python3 main.py --collect --episodes 8`，约 166 s，得 4 272 条 |
| 权重 | `models_trained/e2e_policy.npz`（随仓库提供） |

`data/` 被 `.gitignore` 排除，数据集要自己采；探针数据（`02`）也写进 `data/`。
所有路径都走 `--dataset / --weights / --out`，默认指向仓库内位置，换机器不用改代码。

```bash
cd src/humanoid/humanoid_e2e_recovery        # 模块根目录
python3 diag/03_correlation_ceiling.py --help
```

## 脚本清单

期望输出一列是本轮（云主机 MuJoCo 4.0.0）实跑的值；与文档历史值的差异都注明了原因。

| 编号 | 文件 | 回答的问题 | 命令 | 期望输出 |
|---|---|---|---|---|
| 01 | `01_feature_dynamics.py` | 姿态在变时图像变了多少，池化丢没丢信息 | `python3 diag/01_feature_dynamics.py` | 16×16 逐维 std **0.00277**、64×64 原始 **0.00354**（只差 1.3 倍）；同期倾角只变 2.58°、骨盆移 3.2 cm → 瓶颈是"画面上看不见"，不是池化 |
| 02 | `02_probe_strong_push.py` | 加大推力能否拿到有动态的探针数据 | `python3 diag/02_probe_strong_push.py --episodes 3` | (60,150) N：X 逐维 std **0.01081**、Y 踝 pitch std **105.81**（按 `TORQUE_LIMIT=300` 裁剪后；未裁剪时代为 84.88）；(40,90) 只有 0.00106/0.23；(100,250) 发散 |
| 03 | `03_correlation_ceiling.py` | 单帧图像的信息量够不够 | `python3 diag/03_correlation_ceiling.py --dataset data/diag_probe.npz` | 单像素 max\|corr\| **0.805**（X 维 49；历史值 0.840）；岭回归 **R² = +0.999**（预测 std 105.74 / 目标 105.81），Y[0/4/5/9] R² **+0.910** |
| 04 | `04_std_floor_scan.py` | `X_std` 兜底下限取多少才学得动 | `python3 diag/04_std_floor_scan.py --dataset data/diag_probe.npz` | 0.001→**0.5360**、0.003→0.5356、0.005→0.8466、0.01→0.9656、**0.02→0.9809（最优）**、0.05→0.9738（历史值 0.3011/0.3775/0.7243/0.9657/0.9817/0.9271） |
| 05 | `05_floor_train_loop.py` | 离线达标了，闭环为什么还倒 | `python3 diag/05_floor_train_loop.py --dataset data/diag_probe.npz` | 离线踝 pitch 预测 std 7.2 / 目标 8.5，闭环 **跌倒 2.69 s** → 训练集不含稳态样本 |
| 06 | `06_quick_train_loop.py` | 用正式训练入口重训，离线/闭环差多少 | `python3 diag/06_quick_train_loop.py --dataset data/diag_probe.npz` | 离线 55.8/6.1（目标 55.9/7.6），闭环 **跌倒 2.58 s** |
| 07 | `07_closedloop_probe.py` | 网络跟专家差在哪；常数力矩会不会"假装站稳" | `python3 diag/07_closedloop_probe.py` | 专家踝 pitch **55.43/5.30**（0–124.97）；新权重网络 **67.78/34.08**；旧权重网络 **2.67/0.00（恒定）**；常数 +55 站 10 s/5.5°、−55 跌倒 1.19 s、0 跌倒 1.31 s |
| 08 | `08_trace_vs_expert.py` | 闭环里网络是否真的在跟专家 | `python3 diag/08_trace_vs_expert.py` | 网络 **61.0/20.15**（8.8–107.7），专家 **56.6/22.80**（−1.7–128.4）；20 ms 处 34.73 vs 29.85，80 ms 处 96.05 vs 95.16；340 ms 内倾角只到 1.62° → 缓慢漂移 |
| 09 | `09_blend_sweep.py` | 补多少专家力矩（=阻尼）才站得住 | `python3 diag/09_blend_sweep.py --betas 0 0.3 0.5 --seconds 10` | β=0 **跌倒 3.53 s**；β=0.3 站满 10 s/3.6°；β=0.5 站满 10 s/3.8° |
| 10 | `10_render_delay.py` | 是不是控制延迟造成的 | `python3 diag/10_render_delay.py --every 1 2 --seconds 6` | 2 ms → **3.58 s**、4 ms → **3.53 s**，几乎无差 → 与延迟无关 |
| 11 | `11_frame_diff.py` | 帧间差分能否补上速度通道（失败路线） | `python3 diag/11_frame_diff.py --dataset data/e2e_dataset.npz` | 差分逐维 std **0.00038**（单帧 0.00250）；独立 1e-4 兜底 → 训练 **nan**，闭环 **0.58 s** 跌倒 |
| 12 | `12_zero_bias.py` | 减掉稳态常数偏置能否变好（失败路线） | `python3 diag/12_zero_bias.py` | 稳态首帧输出 **9.04 N·m**；无校正 3.53 s 倒，零偏校正 **1.38 s**（更差） |
| 13 | `13_dagger.py` | DAgger 一轮能否消除误差累积（失败路线） | `python3 diag/13_dagger.py --dataset data/e2e_dataset.npz --episodes 6` | 新增 1 530 条、合并 5 802 条，纯网络仍 **1.95 s** 跌倒（数据量不是瓶颈） |
| 14 | `14_pipeline_check.py` | 一条命令复现入库权重的全流程 | `python3 diag/14_pipeline_check.py --episodes 8` | 4 272 条、8/8 回合站住；X 逐维 std 均值 **0.00250**、X_mean 0.3675、X_std 均值 0.0200（大量格子被 floor 钳住）；Y 踝 pitch **58.0/33.8**（−124~300）；离线预测 std 28.27 / 目标 27.74；稳态首帧网络 **9.04** vs 专家 0.00；空站 **3.53 s** 跌倒 63.6°（单帧权重口径） |
| 15 | `15_pool_temporal_ablation.py` | 同一批数据上，32 格 / 双帧 6 ms / 双帧 102 ms 到底哪个能站住 | `python3 diag/15_pool_temporal_ablation.py --episodes 8` | 一次采集 32 格数据后四档对照：A 16×16 单帧 **2.63 s**、B 32×32 单帧 **1.72 s**、C 双帧 6 ms **2.85 s**、D 双帧 102 ms **站满 10 s**；离线 val_mse 1.48/1.42 好看不等于闭环能站 → 只有 D 采纳入库 |

> 时间窗口提醒：闭环类脚本的 `--seconds` 必须长于预期失稳时间，否则会把"还没倒"读成"站稳"。
> 例如 β=0 实际在 3.53 s 才倒，用 `--seconds 3` 会显示"站稳 3.0s / 16.0°"。上表的闭环数字默认 10 s 窗口。

## 旧权重怎么取（复现"输出恒定 +55"的关键）

`07` 里"旧权重网络 2.67 / std 0.00"需要修复前（`afb05a539`）的那份权重，仓库里已替换成新版，
可用下面两行取回来（已验证 343 766 B、可复现 2.67）：

```bash
curl -sL -o /tmp/old_e2e_policy.npz \
  https://raw.githubusercontent.com/OpenHUTB/ros2/afb05a539/src/humanoid/humanoid_e2e_recovery/models_trained/e2e_policy.npz
python3 diag/07_closedloop_probe.py --weights /tmp/old_e2e_policy.npz
```

## 证据链怎么串

```
01 特征动态 ─┐
02 探针数据 ─┼─> 03 信息量上限(R²=0.999) ──> 推翻"图像信息不足"
             └─> 04 兜底扫描(0.02 最优) ──> 05/06 离线好了闭环仍倒
                                                      │
07 闭环探针(旧权重恒 +55 = 常数巧合) ────────────────┤
08 逐帧对照(网络在跟专家, corr 与专家同号同量级) ──────┤
09 混合扫描(β=0.3 就能站 10 s) ──> 定位缺口: 无速度阻尼
10 刷新率 / 11 差分 / 12 零偏 / 13 DAgger ──> 排除性实验, 均无效
14 全链路 ──> 入库权重 e2e_policy.npz 的生成过程
```

## 只跑一个的最小组合

复现"为什么纯视觉只能 3.5 s"这条主线，只需三步：

```bash
python3 diag/02_probe_strong_push.py --episodes 3                      # 约 90 s
python3 diag/03_correlation_ceiling.py --dataset data/diag_probe.npz    # 秒级
python3 diag/09_blend_sweep.py --betas 0 0.3 --seconds 10               # 约 4 min
```

## 注意

- 渲染约 65.6 ms/帧，闭环脚本（07/09/10/12/13/14）耗时 ≈ 仿真时长 × 500 帧/秒 × 65.6 ms，
  10 s 闭环约 2 min，采一次 8 回合数据约 166 s。
- 探针数据与 `X_mean/X_std` 绑定当前渲染后端；换后端（EGL → Windows 原生等）必须重采重训，
  否则会重现"输入整体偏移 → 第一层 ReLU 全死 → 输出恒定"的故障。
- 07 的"常数 ±55"对照组不依赖任何权重文件，可单独用来判断当前判据有没有判别力。
- 11 在旧版 1 回合数据集（`Y` 近乎恒定）上会退化成 nan 以外的假"站稳"，务必用 8 回合数据集跑。
