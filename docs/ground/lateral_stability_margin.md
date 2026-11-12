# 不足转向梯度与侧向稳定裕度：`lateral_stability_margin`

## 模块概述

`lateral_stability_margin` 把「一台车推头还是甩尾」这件事算成四个能直接读的数：
不足转向梯度 `K`、特征车速 / 临界车速、稳态转向角 `δ`、转向灵敏度 `a_y/δ`。
它只做准静态的圆周运动，不模拟纵向载荷转移，也不需要 CARLA 或显示器——
在任意一台装了 ROS 的机器上都能跑通并通过离线测试。

适合用来：对比两套轮胎调校对转向手感的影响、判断某车速下是否已经进入不稳定区、
给转向控制器校核「该打多少方向盘」。

## 构建与运行

```bash
# ① 把包放进工作空间（结构必须是 src/ground/<包名>/...）
mkdir -p ~/catkin_ws20/src/ground
cp -r lateral_stability_margin ~/catkin_ws20/src/ground/
ls ~/catkin_ws20/src/ground          # 只应打印一行 lateral_stability_margin

# ② 修 shebang（提交版是 python3，Melodic 需要 python2）
cd ~/catkin_ws20/src/ground/lateral_stability_margin/scripts
sed -i '1s|^#!/usr/bin/env python$|#!/usr/bin/env python2|' main.py stability_core.py stability_node.py
head -2 stability_core.py            # 第 1 行 shebang、第 2 行 coding，两行都要在
chmod +x main.py stability_core.py stability_node.py

# ③ 编译
cd ~/catkin_ws20 && catkin_make

# ④ 跑
source /opt/ros/melodic/setup.bash
source ~/catkin_ws20/devel/setup.bash
roslaunch lateral_stability_margin main.launch

# ⑤ 反例：换成硬前软后的甩尾车
roslaunch lateral_stability_margin main.launch front_cornering:=160000 rear_cornering:=40000
```

终端 A 会打印整份报告（参数、轴荷、梯度、车速扫描表、四条断言、结论），
终端 B 用 `rosparam get` / `rostopic echo` 核参数与话题。

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `/vehicle/mass` | 1500.0 | 整车质量 kg |
| `/vehicle/wheelbase` | 2.7 | 轴距 m |
| `/vehicle/cg_to_front` | 1.2 | 质心到前轴距离 m（必须小于轴距） |
| `/vehicle/gravity` | 9.81 | 重力加速度 m/s² |
| `/tire/front_cornering` | 80000.0 | 前轴侧偏刚度 N/rad |
| `/tire/rear_cornering` | 95000.0 | 后轴侧偏刚度 N/rad |
| `/scene/radius` | 50.0 | 转弯半径 m |
| `/scene/speed_kmh` | 40.0 | 稳态圆周车速 km/h |
| `/check/understeer_min` | 0.0 | 不足转向梯度下限 rad/(m/s²) |
| `/check/ackermann_speed_kmh` | 2.0 | 校验阿克曼角时用的低速 km/h |
| `/check/ackermann_tolerance` | 0.02 | 低速转向角与 `L/R` 的相对偏差上限 |
| `/rate/publish_hz` | 1.0 | 保活重发频率 Hz |

## 话题

| 话题 | 类型 | 内容 |
|---|---|---|
| `/stability/report` | `std_msgs/String` | 整份报告 |
| `/stability/cornering` | `std_msgs/Float64MultiArray` | `[K, a_y, δ, 阿克曼角, 超出量]` |
| `/stability/limit` | `std_msgs/Float64MultiArray` | `[特征车速, 临界车速]`（不存在的给 −1） |

三条都是 latched（`latch=True`），所以 `rostopic echo` 一上来就能拿到最后一帧，
不用怕「订阅晚了错过消息」。

## 实测输出（默认参数）

```
整车：质量 1500.0 kg，轴距 2.70 m，质心到前轴 1.20 m（到后轴 1.50 m）
轮胎：前轮侧偏刚度 80000 N/rad，后轮 95000 N/rad
轴荷：前 8175.0 N，后 6540.0 N（合计 14715.0 N）
梯度：K = 0.003399 rad/(m/s²)，特性「不足转向」
特征车速 28.184 m/s（101.5 km/h）
工况：半径 50.0 m，车速 40.0 km/h → a_y 2.469 m/s²，δ 0.062393 rad（3.57°），
      阿克曼角 0.054000 rad（3.09°），超出 0.008393 rad
转向灵敏度 a_y/δ = 39.5740 1/s
车速(km/h)  a_y(m/s²)   δ(rad)   δ(°)   灵敏度(1/s)
       20      0.617   0.0561    3.21     11.0036
       40      2.469   0.0624    3.57     39.5740
       60      5.556   0.0729    4.18     76.2246
       80      9.877   0.0876    5.02    112.7825
      100     15.432   0.1065    6.10    144.9628
      120     22.222   0.1295    7.42    171.5524
断言：4 通过 / 0 失败（共 4 条）
  [PASS] 轴荷分配  前轴 8175.0 N + 后轴 6540.0 N = 14715.0 N（相对偏差 0.00e+00）
  [PASS] 不足转向裕度  K = 0.003399 rad/(m/s²)，特性「不足转向」，特征车速 28.184 m/s（101.5 km/h）
  [PASS] 低速阿克曼  车速 2.0 km/h 时转向角 0.054021 rad，阿克曼角 0.054000 rad（相对偏差 0.0004）
  [PASS] 稳态转向角  δ = 0.062393 rad → 反算 a_y = 2.469136 m/s²（相对偏差 1.80e-16）
结论：PASSED
```

## 两个反例

**反例一：硬前软后 → 甩尾车**

```bash
roslaunch lateral_stability_margin main.launch front_cornering:=160000 rear_cornering:=40000
```

```
梯度：K = -0.011458 rad/(m/s²)，特性「过多转向」
临界车速 15.350 m/s（55.3 km/h）
[FAIL] 不足转向裕度  K = -0.011458 rad/(m/s²)（下限 0.0000），
                    特性「过多转向」，临界车速 15.350 m/s（55.3 km/h）
断言：3 通过 / 1 失败（共 4 条）
结论：FAILED
```

后轴太软，后轮先到附着极限，梯度转负、临界车速只有 55.3 km/h。四条断言只翻
「不足转向裕度」这一条 —— 其余三条（轴荷、低速阿克曼、稳态转向角的闭环）
与梯度符号无关，所以照旧通过。

**反例二：把阿克曼校验放在 60 km/h**

```bash
roslaunch lateral_stability_margin main.launch ackermann_speed_kmh:=60
```

```
[FAIL] 低速阿克曼  车速 60.0 km/h 时转向角 0.072884 rad，阿克曼角 0.054000 rad
                  （相对偏差 0.3498，容差 0.020）
```

60 km/h 时 `K·a_y` 已经占了转向角的 26%，再用「阿克曼角」当参考就完全不成立。
这条断言的意义正在于此：它只在低速（`K·a_y → 0`）时才有资格叫“阿克曼”。

## 离线测试

```bash
cd ~/catkin_ws20
python3 tests/test_lateral_stability_margin_local.py
```

覆盖 A 几何与轴荷、B 梯度与转向特性、C 稳态圆周、D 四条断言、E 车速扫描、
F 字符图、G 参数读取、H 报告排版、I 闭式核对、J 工程文件约定、K 编码声明回归。

> K 组是专门为「Python 2.7 找不到编码声明」这个坑加的回归：它会检查三个脚本的
> **第 1 行是 shebang、第 2 行是 coding**。手册里那条 `sed` 用的是锚定替换
> `1s|^#!/usr/bin/env python$|#!/usr/bin/env python2|`，只会改 shebang，不会
> 把编码声明顶掉。

## 结构约定

```
lateral_stability_margin/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/lateral_stability_margin.yaml      # 12 个默认值
├── launch/main.launch                        # 12 个带 doc 的 arg + 同名 param
├── scripts/
│   ├── main.py                               # 入口，转发到 node
│   ├── stability_core.py                     # 纯标准库，离线可测
│   └── stability_node.py                     # rospy 侧：参数、发布、保活
└── tests/                                    # 由仓库根的 tests/ 放
```

## 常见问题

**1. 特征车速算出来只有 30 km/h？**
`K` 忘了除以 `g`。`W/C` 的量纲是 rad，而公式里 `K` 的单位是 rad/(m/s²)。

**2. `特stable` 报 `UnicodeEncodeError`？**
节点里给 rospy 的日志必须是 byte str。本模块所有日志都走 `_log()`，它会先
`encode("utf-8")`。自己加日志时别直接 `rospy.loginfo(u"中文")`。

**3. `rosparam get /tire/rear_cornering` 还是 95000？**
`<arg>` 只对 launch 内部可见，带斜杠的名字必须落成同名 `<param>` 才能被
`rosparam get` 读到；带斜杠的 `roslaunch pkg file.launch tire/rear_cornering:=…`
会被**静默忽略**，要写裸名 `rear_cornering:=…`。

## 大模型声明

本模块的代码编写、测试编写与文档整理使用了 AI 编程助手辅助，提交内容由本人负责。
