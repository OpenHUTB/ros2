# brake_load_transfer

载重偏置对制动距离与跑偏的影响（纯 numpy 计算 + ROS 2 包装）。

回答一个很实际的问题：**货装偏了，刹车距离差多少、车会不会跑偏。**

```
载重位置 ──▶ 合成质心 ──▶ 轴荷分配 ──▶ 固定 bias 下的抱死顺序 ──▶ 制动距离 / 与理想 EBD 的差距
                    └──▶ 左右载荷差 ──▶ 制动力不均衡 ──▶ 横摆力矩 ──▶ 跑偏量
```

## 1. 依赖与构建

```bash
pip3 install numpy                     # 只有 core 需要；画图另需 matplotlib
colcon build --packages-select brake_load_transfer --symlink-install
source install/setup.bash
```

## 2. 直接用（不需要 ROS）

```bash
cd src/ground/brake_load_transfer
python3 main.py                 # 空载基准
python3 main.py --case --payload 300 --payload-x 2.4
python3 main.py --sweep         # 载重前后位置扫描
python3 main.py --bias-sweep    # 制动力分配扫描
python3 main.py --lateral       # 左右偏置 → 跑偏
python3 main.py --all --json out.json
```

## 3. ROS 2

```bash
ros2 launch brake_load_transfer main.launch.py payload:=300.0 payload_x:=2.4
# 另开终端
ros2 topic pub --times 1 /vehicle/brake_cmd std_msgs/msg/Float64 "{data: 1.0}"
ros2 topic echo /vehicle/brake_state
```

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/vehicle/brake_cmd` | `std_msgs/Float64` | 订阅 | 踏板开度 0~1 |
| `/vehicle/brake_state` | `std_msgs/Float64MultiArray` | 发布 | `[车速, 已行驶, 减速度, 后轴抱死?]` |

## 4. 参数

`config/params.yaml`：

| 参数 | 默认 | 说明 |
|---|---|---|
| `mass` | 1500 kg | 整备质量 |
| `payload` / `payload_x` | 0 / 1.35 m | 载重与其质心距前轴距离 |
| `brake_bias` | 0.62 | 前轴制动力占比 |
| `mu` | 0.90 | 路面附着系数 |
| `v0_kmh` | 100 | 制动初速 |

## 5. 测试

```bash
python3 -m unittest discover -s src/ground/brake_load_transfer/test -p "test_*.py" -v
```

23 项测试覆盖：轴荷守恒、动态转移量、后轴不抬轮、bias 分配、μ 截断、
理想距离闭式、**对称工况零跑偏**、左右偏置单调性与方向对称性，以及
**抱死顺序**（空载→后轴、β=0.70→前轴）与自洽迭代后的距离基准值。

## 6. 验证边界

| 内容 | 状态 |
|---|---|
| `core.py` / 23 项测试 | ✅ 实跑通过（Windows 11 + Python 3.12 + NumPy 2.5） |
| `main.py --all` | ✅ 实跑，见文档表格 |
| 两张图 | ✅ 由 `tools/make_figures.py` 实际生成 |
| ROS 2 节点 | ⚠️ 语法与桩导入通过，**未做 colcon build / launch 实跑**（本机无 ROS 2） |

## 7. 已知限制

- 纵向不含 ABS 循环控制，抱死后按 μ·Fz 恒力处理；
- 轮胎用 μ 上限而非 μ–滑移曲线，因此"峰值附着/滑动附着"的差别不在模型里；
- 横向用线性轮胎 + 准静态轴荷，只给量级，不做操稳仿真；
- 不含侧倾/俯仰动力学与悬架，因此极限工况的载荷再分配会低估。
