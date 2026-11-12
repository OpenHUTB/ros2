# muscle_synergy_lab

肌肉协同（muscle synergy）降维：用非负矩阵分解（NMF）回答"**需要几个协同才能重建这段 EMG**"，
并检验另一个常被滥用的指标——**与真值协同的子空间夹角**——到底能说明什么。

```
EMG 激活矩阵 E ──NMF(k)──▶ W·H ──▶ VAF（重建率）与相邻增量（选 k 的依据）
                        └──▶ 与真值子空间的夹角（只能验证"没跑偏"，不能选 k）
```

## 1. 依赖

```bash
pip3 install numpy                      # 运行（core / main.py / 测试）只需要 numpy
pip3 install "matplotlib>=3.5"          # 仅 tools/make_figures.py 出图需要
colcon build --packages-select muscle_synergy_lab --symlink-install
```

## 2. 不需要 ROS

```bash
cd src/musculoskeletal/muscle_synergy_lab
python3 main.py                        # 默认 8 肌肉 / 真值 3 协同 / 噪声 3%
python3 main.py --noise 0.10           # 加大噪声
python3 main.py --muscles 12 --synergies 4
python3 -m unittest discover -s test -p "test_*.py" -v    # 16 passed
```

## 3. ROS 2

```bash
ros2 launch muscle_synergy_lab main.launch.py k:=3 window:=200
ros2 topic pub /emg/activation std_msgs/msg/Float64MultiArray "{data: [0.3, 0.1, 0.2, 0.05, 0.4, 0.1, 0.2, 0.05]}"
ros2 topic echo /emg/synergy_vaf
```

| 话题 | 类型 | 说明 |
|---|---|---|
| `/emg/activation` | `std_msgs/Float64MultiArray` | 订阅：n 块肌肉的激活值 |
| `/emg/synergy_vaf` | `std_msgs/Float64` | 发布：窗口内重建率 |
| `/emg/synergy_matrix` | `std_msgs/Float64MultiArray` | 发布：k×n 协同权重 |

## 4. 主要结果（合成数据，真值 3 协同，噪声 3%）

| 协同数 k | VAF | 相邻增量 | 与真值子空间夹角 |
|---|---|---|---|
| 1 | 76.5% | — | 0.24° |
| 2 | 89.4% | +13.0 | 0.25° |
| **3** | **99.9%** | +10.5 | 0.49° |
| 4 | 99.97% | +0.06 | 0.58° |
| 5 | 99.97% | +0.01 | 0.25° |

- 选 k 靠**增量断崖**：2→3 有 10.5 个百分点，3→4 只有 0.06；
- **夹角不能选 k**：1–5 维全在 0.6° 以内（子空间嵌套：少选时落在内部、多选时包含真值），
  但对"打乱结构"能读出 65°——它只负责区分"完全不同的结构"。

## 5. 验证边界

| 内容 | 状态 |
|---|---|
| `core.py` / 16 项测试 | ✅ 实跑通过（Windows 11 + Python 3.12 + NumPy 2.5） |
| `main.py` 三张表 | ✅ 实跑，见文档 |
| 图 | ✅ 由 `tools/make_figures.py` 实际生成；CI 里也会跑一次（无显示器，`Agg` 后端） |
| ROS 2 节点 | ⚠️ 语法与桩导入通过，**未做 colcon build / launch 实跑** |
| 真实 EMG | ⚠️ **未使用**：全部结论基于"已知协同"的合成数据（有真值可比） |

## 6. 已知限制

- NMF 是非凸的，不同初始化会落到不同局部最优：`scan_k` 用 warm start 减轻抖动；
- VAF 随 k 必然上升，直到 k = 肌肉数，因此**不能只用 VAF 的绝对值**选协同数；
- 子空间夹角需要真值，真实场景只能做跨条件一致性检验；
- 合成数据的协同结构是"分簇"的，真实肌肉协同可能有更复杂的重叠。
