# whole_body_ik

人形机器人全身逆运动学与可操作性分析。

给定末端位姿反解关节角，并在奇异位形附近保持稳定；顺带用可操作性椭球量化
"这个位形还好不好动"。

核心逻辑**只用 Python 标准库**，不需要 numpy / MuJoCo 也能跑测试。

## 内容

```
main.py                     便捷入口（等价于 scripts/main.py）
.gitignore                  忽略 demo_out 与 __pycache__
LICENSE_apache_2_0.txt      Apache-2.0 许可原文
main.sh                     shell 入口
scripts/main.py             真正的入口（roslaunch 用这个）
scripts/linalg.py           3D 旋转、4x4 位姿、解方程、Jacobi 特征分解
scripts/kinematics.py       关节、串联链、正运动学、几何雅可比
scripts/ik_solver.py        伪逆 / 阻尼最小二乘 / 零空间 / 自适应阻尼
scripts/manipulability.py   Yoshikawa 指标、奇异值、椭球、限位代价
scripts/simple_plot.py      手写 PNG 折线图（只用 zlib）
scripts/animate.py          线框图动图（需要 Pillow）
test/test_ik.py             93 个单元测试
```

## 运行

```bash
cd src/humanoid/whole_body_ik
python3 main.py            # 完整演示，出 6 张曲线
python3 main.py --quick    # 跳过画圆跟踪
python3 main.py --gif      # 额外生成动图（需 pip install pillow）
./main.sh                  # 等价的 shell 入口
```

## 测试

```bash
cd src/humanoid/whole_body_ik
python3 -m unittest discover -s test
```

跑完应显示 `Ran 93 tests ... OK`。

## 用 launch 启动

```bash
roslaunch whole_body_ik main.launch     # 完整
roslaunch whole_body_ik quick.launch    # 快速
```

## 实测结果

| 项目 | 结果 |
|---|---|
| 正解 vs 解析式（2R） | 偏差 5.55e-17 |
| 雅可比 vs 数值差分 | 偏差 1.00e-09 |
| 可操作性 vs 解析 \|sin q1\| | 偏差 2.78e-16 |
| 2R 逆解收敛 | 5~9 次迭代，误差 ~1e-7 |
| 7 自由度画圆跟踪（竖直圆） | 最大误差 9.93e-07 m，平均 17.6 次迭代 |
| 位姿逆解（6 维） | 38 次迭代，误差 1.28e-11 |
| 奇异点 q1=0.005 | 伪逆步长 4.472，DLS 0.0006 |
| 零空间投影 \|J·Δq\| | 1.50e-16 |

详细说明见站点文档：`docs/humanoid/whole_body_ik.md`
