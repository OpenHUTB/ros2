# trajectory_accuracy_eval —— 轨迹 / 里程计精度评估

回答一个很具体的问题：**你车上的里程计（或 SLAM）跑完一条轨迹，到底偏了多少？**

仓库里已经有跑里程计、跑 SLAM、跑导航的模块，但缺少一个把它们"跑得准不准"变成
可比较数字的工具。本模块用三个标准指标补上这一环：

| 指标 | 含义 | 适用场景 |
|---|---|---|
| **ATE**（绝对轨迹误差） | 整条轨迹刚体对齐后的位置误差 RMSE | 比较两条轨迹整体贴合程度 |
| **RPE**（相对位姿误差） | 固定间隔内**相对运动**的误差 | 判断局部精度（尺度、漂移） |
| **漂移率** | 终点误差 / 轨迹长度 × 100% | 直观看"跑 100 m 偏多少" |

核心算法**纯 NumPy**，不需要 ROS、不需要 Gazebo/CARLA、不需要 GPU，
因此在任何装了 numpy 的机器上都能复现全部数字；同时提供 ROS 1 (Noetic)
在线评估节点与演示用数据源。

## 1. 为什么需要它

轮式里程计的典型误差是**系统性**的：轮半径差 2%，跑 100 m 就会少算 2 m。
这种误差不会因为换地图、换控制器而消失，只能用标定量化。本模块把
"标定前 / 标定后"的差别变成 ATE 和漂移率两个数字，便于写实验报告，也便于回归。

## 2. 快速开始（不需要 ROS）

```bash
cd src/ground/trajectory_accuracy_eval

# 生成一条 figure8 轨迹 + 带 2% 轮半径误差的里程计，并评估
python3 main.py --mode demo

# 换轨迹 / 换误差
python3 main.py --mode demo --kind circle --scale 1.05
python3 main.py --mode demo --kind straight --yaw-bias 0.1

# 扫描尺度误差对精度的影响（输出表格 + 图）
python3 main.py --mode sweep

# 评估自己的两个 CSV（格式：t,x,y,yaw）
python3 main.py --mode eval --gt ground_truth.csv --est odometry.csv
```

输出示例（`figure8`、1.5 m/s、40 s、2% 尺度误差）：

```
轨迹长度            : 60.00 m
ATE RMSE            : 0.3301 m  (mean 0.2876 / max 0.5709)
ATE 朝向 RMSE       : 0.002 °
RPE(delta=1) 平移   : RMSE 0.0015 m
RPE(delta=10) 平移  : RMSE 0.0152 m
RPE(delta=50) 平移  : RMSE 0.0756 m
终点漂移率(原始)    : 1.897 %
终点漂移率(对齐后)  : 0.951 %
```

这三行 RPE 可以直接手算验证：每步前进 `1.5 m/s × 0.05 s = 0.075 m`，
尺度误差 2% 即每步 `0.0015 m`，因此 `RPE(10) = 10 × 0.0015 = 0.015 m`、
`RPE(50) = 0.075 m`。**指标算得对不对是可以用纸笔核对的**，这也是本模块的价值之一。

## 3. 尺度误差的影响（`--mode sweep` 实测）

真值轨迹 `figure8`、1.5 m/s、40 s（长度 60 m）：

| 轮半径尺度系数 | 尺度误差 | ATE RMSE / m | ATE max / m | 终点漂移率 / % |
|---|---|---|---|---|
| 0.96 | −4% | 0.6601 | 1.1397 | 3.793 |
| 0.98 | −2% | 0.3302 | 0.5710 | 1.898 |
| 0.99 | −1% | 0.1655 | 0.2868 | 0.950 |
| 1.00 | 0% | 0.0119 | 0.0173 | 0.055 |
| 1.01 | +1% | 0.1653 | 0.2866 | 0.949 |
| 1.02 | +2% | 0.3301 | 0.5709 | 1.897 |
| 1.04 | +4% | 0.6599 | 1.1397 | 3.793 |

结论很直接：**漂移率与尺度误差近似成正比**，与符号无关（多算和少算对称）。
1.00 那一行残留的 0.055% 来自仿真积分的一阶离散误差，不是算法缺陷（见第 7 节）。
用数据文件：`results/sweep_scale.json`。

## 4. ROS 1（Noetic）在线评估

不需要装任何仿真器，launch 里自带"真值 + 带误差里程计"的数据源：

```bash
mkdir -p ~/catkin_ws/src
cp -r src/ground/trajectory_accuracy_eval ~/catkin_ws/src/
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch trajectory_accuracy_eval main.launch
```

终端里每 3 秒打印一次：

```
配对 401 组 | ATE RMSE 0.1665 m (max 0.2845) | RPE(10) 0.0152 m
```

换轨迹 / 换误差：

```bash
roslaunch trajectory_accuracy_eval main.launch kind:=circle
roslaunch trajectory_accuracy_eval main.launch wheel_scale:=1.01
```

观察指标话题：

```bash
rostopic echo /traj_eval/metrics    # [ATE_RMSE, ATE_MAX, RPE_RMSE, 配对数]
```

`Ctrl+C` 退出后，配对好的两条轨迹会写到 `~/traj_eval_out/`，可再用
`python3 main.py --mode eval --gt ... --est ...` 离线复核。

节点接口：

| 方向 | 话题 | 类型 |
|---|---|---|
| 订阅 | `/ground_truth` | `nav_msgs/Odometry` |
| 订阅 | `/odom` | `nav_msgs/Odometry` |
| 订阅 | `/traj_eval/reset` | `std_msgs/Empty`（清空缓冲，开始新一轮评估） |
| 发布 | `/traj_eval/metrics` | `std_msgs/Float64MultiArray` |

换成自己的数据源，只要把 `/ground_truth` 与 `/odom` 接上即可，
例如 `rosbag play` 或真实定位输出。

## 5. 文件结构

```
src/ground/trajectory_accuracy_eval/
├── main.py                     # 统一入口：demo / eval / sweep / ros
├── traj_eval/
│   ├── se2.py                  # SE2 位姿运算 + Umeyama 对齐
│   ├── metrics.py              # ATE / RPE / 漂移率
│   ├── io.py                   # 轨迹 CSV 读写
│   ├── simulate.py             # 真值轨迹 + 里程计误差模型
│   ├── viz.py                  # 轨迹对比 / 误差曲线 / 敏感性图
│   ├── ros_node.py             # ROS 1 在线评估节点
│   └── ros_sim_publisher.py    # ROS 1 演示数据源
├── scripts/                    # roslaunch 的两个入口
├── launch/main.launch
├── config/params.yaml
├── test/test_traj_eval.py      # 自洽测试：41 项检查
├── test/test_known_answer.py   # 已知答案对拍：7 项（含暴力搜索交叉验证）
├── results/                    # 实测输出的指标（JSON）
└── requirements.txt
```

### 5.1 测试

```bash
python3 test/test_traj_eval.py        # 自洽测试（41 项）
python3 test/test_known_answer.py     # 已知答案对拍（7 项）
python3 -m pytest test/ -q            # 或一次跑完（15 passed）
```

> **Windows 用户**：默认控制台是 GBK，无法编码 `∘`/`⁻¹` 等符号。
> 两个脚本启动时已把输出流的 `errors` 设为 `replace`，并对这些符号做 ASCII 等价替换
> （`∘`→`o`、`⁻¹`→`^-1`），因此不会因 `UnicodeEncodeError` 中断；GBK 与 UTF-8 下结果一致。

自洽测试只保证"实现与自己一致"，查不出"公式整体写错但内部自洽"的问题，
因此 `test_known_answer.py` 用**解析解**与**暴力搜索**两种独立答案源对拍：
同心圆半径差的 ATE 必须严格等于半径差、匀速直线的 RPE 必须等于 `(k−1)·ds·Δ`、
已知高斯噪声下 ATE 应接近 `σ√2`，以及与完全不用 SVD 的暴力搜索求出的
最优对齐 RMSE 一致。详见文档第 7 节。


## 6. 与仓库已有模块的区别

* `src/chap6`、`docs/chapter/chap6.md` 等讲的是**怎么跑 SLAM/导航**，不评估精度；
* `src/ground/mujoco_diff_drive_slip_compensation` 关注**轮子打滑的补偿控制**，
  本模块关注**事后精度评估**，两者互补（可用本模块量化其补偿效果）；
* 仓库中 `绝对轨迹误差`、`相对位姿误差`、`里程计精度`、`umeyama` 等关键词此前均无实现，
  本模块是首个把 ATE/RPE 标准指标落成可运行工具的地方。

## 7. 已知限制

1. **对齐不估计尺度**（默认 `with_scale=False`）：尺度误差本身就是要评估的对象，
   对齐全吸收掉会让 ATE 失去意义。需要时可传 `with_scale=True` 单独观察估计出的尺度。
2. **零误差时仍有约 0.01 m 的 ATE 残留**：仿真里用一阶（Euler）方式把相对运动
   重新积分成里程计轨迹，与真值的解析积分存在离散差异。可通过减小 `--dt` 降低。
3. **只处理平面运动（SE2）**：地面车辆足够；空中/水下三维轨迹需要扩展到 SE3。
4. **要求两条轨迹逐点对应**：本模块按时间戳配对，不做轨迹重采样。
   若采样率不同，请先插值到统一时间轴。
5. ROS 节点用最近邻时间戳配对，容差由 `~match_tolerance` 控制；
   采样率差异很大时应调小容差或先做时间对齐。

## 8. 引用与参考

* Sturm, Engelhard, Endres, Burgard, Cremers. *A Benchmark for the Evaluation of
  RGB-D SLAM Systems*. IROS 2012.（ATE / RPE 的定义）
* Umeyama, S. *Least-squares estimation of transformation parameters between two
  point patterns*. IEEE TPAMI, 1991.（相似变换闭式解）
