# 拮抗肌共收缩与关节刚度核算

一对拮抗肌跨过同一个关节：伸肌给正向力矩，屈肌给反向力矩。净力矩只取决于两侧肌力之差，
而**关节刚度取决于两侧肌力之和**——这就是共收缩的意义：同样的净力矩，两侧都多出一点力，
关节就"硬"一点，扰动下的偏转就小一点。本模块把这两条关系写成可以手算核对的公式，
只依赖 Python 标准库。

## 默认样机结论

默认参数：目标净力矩 8 N·m、肌力臂 0.03 m、最大等长力 600 N、肌肉静息长度 0.25 m、
肌刚度系数 q=25、共收缩水平 0.3、扰动力矩 2 N·m。

| 指标 | 数值 | 说明 |
| --- | --- | --- |
| 伸肌力 / 激活度 | 313.3 N / 0.52 | 基线 180 N + 净力矩所需的一半 |
| 屈肌力 / 激活度 | 46.7 N / 0.08 | 基线 180 N − 净力矩所需的一半 |
| 力矩上限（c=0.3） | 10.8 N·m | 2·c·F_max·r，超过它屈肌侧归零 |
| 关节刚度 | 32.40 N·m/rad | r²·q·(F_ago+F_ant)/L |
| 2 N·m 扰动偏转 | 3.54° | Δθ = τ_d/K |

两条值得注意的结论：

- **刚度与净力矩解耦（线性模型下）。** 共收缩水平固定时，两侧肌力之和不变，所以不管你要
  8 N·m 还是 4 N·m，关节刚度都是 32.4 N·m/rad——共收缩决定"硬不硬"，净力矩决定"朝哪使劲"，
  两笔账互不干扰。
- **刚度要翻倍，代价是肌肉总力翻倍。** c 从 0.3 提到 0.6，K 从 32.4 涨到 64.8 N·m/rad，
  但两侧总力从 360 N 涨到 720 N，代谢代价同步上升。这就是"紧绷着稳"与"省着力活"的取舍，
  也是肌肉疲劳模块关心的输入。

## 1. 运行环境

只要 Python 3.8 及以上，**运行时不依赖任何第三方库**：

```bash
cd src/musculoskeletal/msk_cocontraction_stiffness
python3 main.py --output outputs
python3 main.py --torque 10 --coactivation 0.5 --output outputs
```

实测环境：Windows 11 `10.0.26200`、Python `3.12.14`；`python3 main.py` 实际输出：

```text
==> msk_cocontraction_stiffness
    参数: 目标力矩 8.0 N*m, 力臂 0.03 m, F_max 600 N, 静息长 0.25 m, q=25, 共收缩 0.30
    肌力: 伸肌 313.3 N (激活 0.52), 屈肌 46.7 N (激活 0.08), 上限 10.8 N*m (正常)
    关节刚度: 32.40 N*m/rad, 2.0 N*m 扰动 -> 3.54 deg 偏转
```

结果写入 `outputs/report.json`（`outputs/` 已进 `.gitignore`）。

## 2. 公式

```text
F_m = a * F_max                       线性肌力模型（忽略力-长度/力-速度）
tau = r * (F_ago - F_ant)             净关节力矩
k_m = q * F_m / L_slack               单肌刚度，q 为无量纲系数（文献典型 10~30）
K   = r^2 * (k_ago + k_ant)
    = r^2 * q * (F_ago + F_ant) / L_slack

F_ago = c*F_max + tau/(2r)            反解：c 为共收缩水平 c=(F_ago+F_ant)/(2*F_max)
F_ant = c*F_max - tau/(2r)
可行条件: tau <= 2 * c * F_max * r
delta_theta = tau_disturbance / K     扰动下的稳态角位移
```

## 3. 判定阈值

| 量 | 阈值 | 含义 |
| --- | --- | --- |
| 目标力矩 / 共收缩上限 | > 90% → 注意 | 拮抗侧肌力快归零，再大的力矩就只能降共收缩 |
| 目标力矩 / 共收缩上限 | > 100% → 超限 | 该共收缩水平下物理不可行 |

阈值是工程经验值：拮抗侧留 10% 的力，避免扰动一来就"拉穿"。

## 4. 适用范围与限制

- 线性肌力模型，不含力-长度与力-速度关系，只算准静态共收缩；
- q 系数取常数（默认 25），真实值随肌纤维类型、激活状态与长度变化；
- 两侧肌力臂取相同的 r，真实关节的屈/伸肌力臂不对称；
- 关节刚度是"肌源性"部分，不含肌腱串行弹性与韧带/软骨贡献。

## 5. 测试与验证

```bash
cd src/musculoskeletal/msk_cocontraction_stiffness
python3 test/test_msk_cocontraction_stiffness.py     # 12 项，无 pytest 也能跑
python3 -m pytest test -v                             # CI 用
```

12 项检查全部通过，期望值都能手算核对：线性肌力与激活度边界、反解的差/和恒等式
（差等于 τ/r、和等于 2cF_max）、上限边界（τ=10.8 拮抗侧归零，10.81 判不可行，
c=0 时任何力矩不可行）、关节刚度 32.4 N·m/rad 与共收缩/q 系数的比例关系、
刚度与净力矩解耦、扰动偏转 3.54° 与刚度翻倍减半、默认报告、高利用率判注意、
超限判不可行、非法参数拒绝（q 出 10~30、共收缩出 [0,1] 等）、GBK 控制台打印与 CLI 冒烟。

CI：`.github/workflows/msk_cocontraction_stiffness_tests.yml` 在 Ubuntu + Python 3.8 上跑 pytest、
无 pytest 直跑、`main.py` 冒烟与 `PYTHONIOENCODING=gbk` 复跑；另有 Windows 任务再跑一遍。

## 使用声明

使用 DeepSeek Harness（deepseek-flash）辅助实现、测试与文档；作者对提交内容负责。
默认参数取自人体肘/膝拮抗肌对量级，不是某一受试者的实测值，核算前请替换为自己的参数。
