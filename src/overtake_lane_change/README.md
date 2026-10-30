# Overtake（超车变道）仿真作业

ROS 超车变道仿真：三车道直道，本车巡航接近慢速前车后，打左转灯、向左变道、加速超越、再向右变回原车道并恢复巡航。

## 场景
- 三车道直道，车道宽 3.5 m；
- 前车：初始 x=80 m，以 8 m/s 匀速慢行（红色）；
- 本车：初始 x=0 m，以 15 m/s 巡航（蓝色）；
- 超车触发：保险杠间距 < 40 m → 打左转灯 1 s → 向左变道（y: 0 → 3.5 m）→ 加速至 17 m/s 超越 → 超过前车 15 m 后向右变回（y: 3.5 → 0 m）→ 恢复 15 m/s 巡航。

## 控制律
- 状态机：`CRUISE → SIGNAL → CHANGING → OVERTAKING → CHANGING_BACK → CRUISE`；
- 纵向：速度比例控制（限幅加减速 a_max=2.5, d_max=4.0 m/s²）；打灯/变道期间跟住前车速度，保持间距；
- 横向：比例控制横向速度（限幅 ±2.0 m/s），平滑完成车道变换；
- 安全兜底：任何状态下前车保险杠间距过近立即全力制动；
- 车距语义：保险杠间距 = 中心距 − 车长 4.5 m。

## 文件结构
```
overtake/
├── scripts/
│   ├── vehicle_model.py       # 车辆运动学模型
│   ├── overtake_controller.py # 超车控制律（状态机）
│   ├── headless_test.py       # 无界面自动验证
│   ├── overtake_sim.py        # Pygame 交互仿真 + GIF 录制
│   └── main.py                # ROS Noetic 节点
├── launch/main.launch
├── package.xml
├── CMakeLists.txt
├── README.md
└── docs/overtake.md           # 设计文档
```

## 运行（无需 ROS 也可直接跑）
```bash
cd scripts
python3 headless_test.py                        # 自动验证
python3 overtake_sim.py                         # 交互窗口（需 pygame）
python3 overtake_sim.py --record demo.gif --duration 26   # 录演示动图（只需 Pillow）
```

## 验证结果（headless_test.py）
| 项目 | 结果 |
|---|---|
| 超车成功（超过前车 20m+） | PASS |
| 回到原车道（\|y\|<0.1） | PASS |
| 全程未超速（≤18 m/s） | PASS |
| 变道平滑（无瞬移） | PASS |
| 全程不碰撞（保险杠间距>0.5m） | PASS |
| 恢复巡航（≈15 m/s） | PASS |
| 状态机完整走完 | PASS |

## ROS 运行
```bash
catkin_make && source devel/setup.bash
rosrun overtake main.py
# 或 roslaunch overtake main.launch
```
