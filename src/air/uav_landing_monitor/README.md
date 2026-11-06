# uav_landing_monitor

无人机降落过程监控：在线校验「速度指令 vs 实际高度变化」，发现"喊了不动 / 掉得没人管 / 超时没落地"
就告警并落 CSV。**只监不听**，不发任何控制指令，可以挂在任何桥接层与降落模块旁边。

## 1. 一眼看懂

![故障轮](../../../docs/img/air/uav_landing_monitor/stuck_run.png)

上图是真实 UE + AirSim 运行（桥接层 `cmd_duration=0.1`）：飞机卡在屋面上方 0.39 m 下不去，
当时版本逐帧报了 7 条 `CMD_NO_EFFECT`，最后由 `LANDING_TIMEOUT` 收尾，结论 `fail`；
当前版本改为上升沿触发，同样一段持续故障只报 1 条（有对应回归用例）。
把桥接层改成 `cmd_duration=0.5` 后同一条链路安静落地，只报一条 `LANDED`（零误报）。

## 2. 事件

| 代码 | 级别 | 含义 |
| --- | --- | --- |
| `CMD_NO_EFFECT` | error | 窗口内指令积分很大，但高度几乎不动（本次联调抓到的静默故障） |
| `DESCEND_STALL` | warn | 下降途中指令消失、高度也不再下降，且没到目标（不依赖 `DESCEND` 字样） |
| `FREE_FALL` | error | 高度掉得比指令快（失控 / 掉高） |
| `Z_REACHED` | info | 高度进入目标 ± 容差并稳定（物理上到位置了） |
| `LANDED` | info | 到位置 + 上游已释放下降指令 |
| `CMD_NOT_RELEASED` | warn | 到位置了，上游还在发下降指令 |
| `OVERSHOOT` | error | 低于目标高度过多（可重复触发） |
| `LANDING_TIMEOUT` | error | 下降超时仍未到位置 |
| `INPUT_STALE` | error | 里程计断流 |
| `TARGET_UNKNOWN` | warn | 迟迟拿不到落地目标高度 |

语义要点：期望位移用**指令积分** `∫|−vz|dt`；告警**上升沿触发**（持续故障只报一条）；
落地/超时后**自动复位**，可连续监控多次降落；`frame/z_up:=false` 时高度/指令/目标三处一起换算。

## 3. 运行

```bash
roslaunch carlair_ros_bridge main.launch host:=192.168.49.1     # 终端 1：桥接层
roslaunch rooftop_landing_nn main.launch                        # 终端 2：被监控的降落模块
roslaunch uav_landing_monitor main.launch csv:=/tmp/landing.csv # 终端 3：本模块
rosrun   uav_landing_monitor main.py                           # 等价的前台运行
```

输出：

* `/uav/landing_monitor/alert`（`std_msgs/String`，一条告警一行 JSON，含数值证据）
* `/uav/landing_monitor/state`（`std_msgs/String`，2 Hz 状态 JSON，含仍在持续中的告警）
* CSV（`monitor/csv` 参数）：`t,phase,z,target_z,vz_cmd,status,alerts`
* 退出或落地时打印中文报告：结论 / 阶段 / 时长 / 告警统计 / 逐条明细

## 4. 参数

见 `config/uav_landing_monitor.yaml`（启动时会打印整份生效阈值，拼错的键会 WARN）。最常改的：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `target/mode` | `auto` | 目标高度来源：`landing`（落点话题）→ `goal_offset`（目标点 − 安全高度）→ `fixed` |
| `thresholds/descend_grace_s` | 1.5 | 刚下发下降指令的起动宽限期（只保护一致性判据，掉高判据照常） |
| `thresholds/landed_cmd_grace_s` | 5.0 | 到位置后上游多久还不释放下降指令就告警 |
| `thresholds/timeout_s` | 90.0 | 从开始下降到放弃的时限 |

## 5. 测试

```bash
# 纯 python 本地回归（不需要 ROS / 仿真器 / GPU），预期结尾 "结果：67/67 通过"
python3 tests/test_uav_landing_monitor_local.py

# catkin 工作空间里也可以： catkin_make run_tests（走 catkin_add_nosetests）
# 把监控 CSV 画成证据图
python3 tools/plot_report.py landing.csv --out landing.png
```

## 6. 目录

```text
uav_landing_monitor/
├── scripts/
│   ├── main.py                  # 统一入口（roslaunch / rosrun 都走它）
│   ├── landing_monitor_node.py  # ROS 节点：订阅、告警、CSV、报告
│   └── monitor_core.py          # 纯逻辑判据（无 ROS 依赖，可单测）
├── tools/plot_report.py         # CSV -> 证据图（同类告警标注次数）
├── tests/test_uav_landing_monitor_local.py
├── launch/main.launch
├── config/uav_landing_monitor.yaml
├── CMakeLists.txt / package.xml
└── README.md
```

## 附：大模型使用声明

代码、测试与文档在编写过程中使用了大模型辅助（判据设计讨论、代码与文档起草、绘图脚本整理、
按评审意见重构判据与补充回归用例）。结论均来自可复现的真实运行：本地 67 项断言 +
UE 5.5 + Cosys-AirSim 上两轮真实降落（故障轮 / 正常轮，见文档第 4 节）。
作者对提交内容的正确性与完整性负全部责任。
