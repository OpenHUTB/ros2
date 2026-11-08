# uav_right_of_way

两架无人机在空中近距离相遇时，谁让谁、怎么让。这个模块做的事：

1. 用相对运动解析算 CPA（最近接近点），预测 8 s 内两机距离会小于 40 m 就触发决策；
2. 按四条规则给出让路动作：优先级高的先走 → 交叉左让右 → 对头各右转 → 追越机从左侧过；
3. 每次决策都写一条带理由的字符串进 JSON，方便事后对账；
4. 威胁过去之后，按余弦剖面慢慢回到计划航路，恢复期间继续盯着，有二次冲突就重新让。

整个过程不改飞行计划，和 `uav_deconflict`（飞之前改起飞时刻/高度/航路）是两层东西：
那边管计划层，这边管飞行中临场的让路。

## 效果

三个场景（对头 / 交叉 / 追越），每架机用 `uav_mujoco_sim` 的刚体模型真飞，
下面是全程机对间最小三维距离（越小越危险）：

| 场景 | 不开规则 | 开规则 | 决策次数 |
|---|---|---|---|
| 对头 | 1.68 m | 14.56 m | 6 |
| 交叉 | 1.19 m | 33.44 m | 8 |
| 追越 | 0.42 m | 19.98 m | 4 |

代价是航迹偏离计划的平均值从 2~3 m 涨到 4~8 m，属于让路的正常代价。

动图（左栏不开规则、右栏开规则，同一时刻对比）：

![对头相遇](../../docs/img/air/uav_right_of_way/headon.gif)

另两个场景的动图和三场景轨迹总览图在 `docs/img/air/uav_right_of_way/`，
用 `tools/make_gif.py` 重跑生成。

一条决策日志长这样（存在 `results/<场景>_result.json` 的 `decision_log` 里）：

```text
规则②交叉相遇左让右：UAV-2 位于本机右侧（叉积 -44.1），右转30°让行（CPA 40m@8.0s）
规则④追越：从左侧追越 UAV-1，左偏20m保持间隔（正后方，CPA 25m@8.0s）
```

## 怎么让的

让路动作一共四种，都作用在参考速度上，不动控制器本身：

- 右转 30°：参考航向叠加偏置
- 降高 10 m：参考高度下移
- 减速 30%：参考速度乘 0.7
- 左侧追越：给参考点加一个横向偏置目标

规则只看两机当时的状态（位置/速度/航向 + 任务优先级），几何分类用航向差：
大于 150° 算对头，小于 10° 算追越，其余是交叉；左右侧用航向叉积符号判。

威胁消失后先观察 3 s 再开始恢复，恢复是余弦剖面 `(1 + cos πs)/2`：
起点接住当前偏移、终点归零、两端速度为零，不会拿航向阶跃去激励姿态环。
恢复期间每个决策周期重新检测，出现二次冲突立即中止恢复重新决策。

## 运行

```bash
# 仅算法层（不需要 mujoco，几秒跑完）
python main.py --no-sim

# 三场景 MuJoCo 闭环，基线 + 规则共 6 次真飞（本机实测约 70 s）
python main.py

# 只跑一个场景
python main.py --scenario overtake

# 录 gif / 总览图
python tools/make_gif.py
```

装包之后：

```bash
colcon build --packages-select uav_right_of_way
source install/setup.bash
ros2 run uav_right_of_way main
ros2 launch uav_right_of_way main.launch.py   # 在线决策节点
```

节点订阅 `<prefix><n>/odom`，预测到相遇就往 `/uav_right_of_way/commands`
发 JSON（让路动作 + 决策理由）。判定逻辑都在 `core/`，节点只是个收发壳。

依赖：Python 3.10+，`pip install numpy mujoco matplotlib pytest`；
ROS 节点部分需要 ROS 2 Humble；仿真底座是同仓库的 `uav_mujoco_sim`
（源码运行时自动挂 `sys.path`）。

## 目录

```
uav_right_of_way/
├── main.py                       入口
├── package.xml / setup.py / setup.cfg
├── config/uav_right_of_way.yaml  判定与规则参数
├── launch/main.launch.py
├── uav_right_of_way/
│   ├── core/
│   │   ├── detect.py    CPA 解析、侵入判定
│   │   ├── rules.py     四条规则（动作 + 理由字符串）
│   │   ├── recover.py   余弦恢复 + 恢复期监视
│   │   ├── scenarios.py headon / cross / overtake
│   │   ├── sim.py       基线 vs 规则闭环
│   │   └── pipeline.py  结果汇总、JSON、报告
│   ├── cli.py
│   └── ros/row_node.py  在线决策节点
├── test/        48 项测试（没装 mujoco 的机器也能全过，闭环用例自动退化）
├── tools/make_gif.py
└── results/     每场景的机读结果
```

## 已知局限

- 判定假设两机短时间匀速直线，急机动的一方预测会不准（8 s 窗口内影响有限，没专门验证）；
- 只处理两机相遇，三机以上的组合让路没有做（规则是两两算的，多机同时触发时的优先序靠优先级字段兜底）；
- 让路动作的幅度（30°/10 m/30%）写死在参数里，没有按相遇几何自适应调整。
