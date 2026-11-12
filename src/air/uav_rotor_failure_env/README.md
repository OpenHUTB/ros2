# uav_rotor_failure_env

多旋翼**单桨失效后的悬停可行域**：给定桨布局与单桨推力上限，算出"零姿态力矩下最多能产生多少总推力"，
从而回答三个工程问题——掉一个桨还能不能悬停、还能带多少载、要不要自旋。

```
桨布局 + 失效桨 ──▶ 线性约束（Mx=My=0，可选 Mz=0）+ 顶点枚举 ──▶ 最大总推力 / 载重上限 / 是否必须自旋
```

## 1. 依赖

```bash
pip3 install numpy        # core 只用 numpy；画图另需 matplotlib
colcon build --packages-select uav_rotor_failure_env --symlink-install
```

## 2. 不需要 ROS

```bash
cd src/air/uav_rotor_failure_env
python3 main.py                        # 三种机型对比 + quad 明细
python3 main.py --detail hexa          # 看六旋翼逐桨
python3 main.py --payload 1.5          # 判断带 1.5 kg 载荷还能不能悬停
python3 -m unittest discover -s test -p "test_*.py" -v    # 12 passed
```

## 3. ROS 2

```bash
ros2 launch uav_rotor_failure_env main.launch.py config:=quad failed_rotor:=0
ros2 topic echo /uav/allocation_report
ros2 topic echo /uav/allocation_margin
```

| 话题 | 类型 | 说明 |
|---|---|---|
| `/uav/allocation_margin` | `std_msgs/Float64MultiArray` | `[正常, 失效·自由偏航, 失效·零偏航, 载重上限 kg]` |
| `/uav/allocation_report` | `std_msgs/String` | 一行结论（是否必须自旋） |

## 4. 主要结论（默认参数：单桨 10 N、2 kg 机重）

| 机型 | 正常 | 单桨失效（可自旋） | 单桨失效（零偏航） |
|---|---|---|---|
| quad-X | 100% | 50% | **0%（必须自旋）** |
| hexa | 100% | 66.7% | 66.7% |
| octo | 100% | 75% | 70.7% |

## 5. 验证边界

| 内容 | 状态 |
|---|---|
| `core.py` / 12 项测试 | ✅ 实跑通过（Windows 11 + Python 3.12 + NumPy 2.5） |
| `main.py` 三张表 | ✅ 实跑，见文档 |
| 图 | ✅ 由 `tools/make_figures.py` 实际生成 |
| ROS 2 节点 | ⚠️ 语法与桩导入通过，**未做 colcon build / launch 实跑** |

## 6. 已知限制

- 静止悬停模型：不含姿态倾角、气动干扰、地面效应与瞬态响应；
- 偏航力矩用 `κ·f` 的线性近似，真实桨的扭矩-推力关系随工况变化；
- 顶点枚举针对"总推力最大"这一目标，等推力解里优先取 |Mz| 最小者（不自旋优先）；
- 不判断控制律能否实现该分配——这里给的是**可行域**，不是控制器设计。
