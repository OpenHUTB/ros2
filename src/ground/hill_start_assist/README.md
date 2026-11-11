# hill_start_assist

ROS 1（Noetic）+ MuJoCo 的**差速 AGV 坡道起步辅助**：
车停在 12° 斜坡上，松开刹车后会在重力分力下后溜；本模块用
**四轮轮角保持 PD + 坡度前馈**把它稳住。

## 与仓库已有模块的区别

| 模块 | 场景 | 控制对象 |
|---|---|---|
| `diff_drive_odom_calib` | 平地 | 里程计系数标定 |
| `balance_bot` | 平地 | 车身倾角（倒立摆） |
| **本模块** | **12° 斜坡** | **轮子转角（驻车）** |

仓库没有坡道/驻车相关模块。

## 目录结构

```
├── models/slope_car.xml        12° 斜坡 + 4 轮两轴车辆（nq=11, nu=4, 7.0 kg）
├── src/hill_start_assist/
│   ├── config.py               几何/坡度/增益常量
│   └── control.py              轮角保持 PD + 坡度前馈
├── tools/check_hill.py         自检：有/无辅助对比（含俯仰与接触判据）
├── tools/make_gif.py           渲染对比动图
└── test/test_hill_start.py     5 项纯逻辑单测（不需要 ROS / MuJoCo）
```

## 依赖

```bash
sudo apt install -y python3-numpy
pip3 install mujoco==3.2.3
# 仅渲染动图需要：
sudo apt install -y libosmesa6 libosmesa6-dev
pip3 install -U "PyOpenGL>=3.1.6"
```

## 运行

```bash
python3 main.py                  # 模块入口：一条命令跑完自检
python3 main.py --gif            # 自检 + 渲染动图
./main.sh                        # 同上，自动处理无 DISPLAY 的渲染后端
roslaunch hill_start_assist hill_start.launch   # ROS 环境

# 单独跑某一步
python3 tools/check_hill.py
python3 -m unittest discover -s test             # 5 项
python3 tools/make_gif.py                        # 写到 docs/img/hill_start_drive.gif
```

## 实测（Ubuntu 20.04 + Python 3.8.10 + mujoco 3.2.3）

| 方案 | 松刹车后 3 s 沿坡位移 | 末俯仰 | 最小接触 |
|---|---|---|---|
| 无辅助 | **−2.179 m** | −12.0° | 4 |
| 有辅助 | **−0.008 m** | −12.0° | 4 |

位移减少 **99.65%**；两组起点初速均为 0.00 m/s，全程未翻（末俯仰 −12.0° = 坡度）。

## 已知限制

- 坡度是场景给定常数，未做 IMU 在线估坡
- 未做神经网络、ROS 节点与键盘遥控
