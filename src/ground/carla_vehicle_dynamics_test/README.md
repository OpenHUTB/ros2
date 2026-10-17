# carla_vehicle_dynamics_test：CARLA 地面载具动力性测试

在 CARLA 中对地面载具做三项**动力性（纵向与转向性能）测试**，并支持多车型横向对比：

| 测试 | 方法 | 输出指标 |
|---|---|---|
| **加速** | 全油门从静止加速到目标车速 | 0→v 时间、平均加速度 $a=\Delta v/\Delta t$ |
| **制动** | 加速到目标车速后全力刹车至停稳 | 制动距离、平均减速度 $a=v^2/(2s)$ |
| **转向** | 固定转向角 + 恒定车速的稳态圆周 | 实测转弯半径（最小二乘圆拟合）vs 阿克曼理论值 $R=L/\tan\delta$ |

结果输出为**终端对比表 + CSV + 速度曲线图**，便于把"实测"和"理论"放在一起核对。

## 1. 运行环境

| 项目 | 版本 / 说明 |
|---|---|
| 宿主机 | Windows 10/11，CARLA 0.9.16（Windows 发行版解压即用，双击其中的 `CarlaUE4.exe`） |
| 虚拟机 | Ubuntu 22.04 LTS（VMware），ROS 2 Humble |
| Python | 3.10（CARLA 0.9.16 的 wheel 提供 cp310/cp311/cp312） |
| Python 依赖 | `carla==0.9.16`、`numpy`，`matplotlib`（可选，用于画曲线） |

## 2. 运行步骤

```bash
# ① Windows 上双击启动 CARLA 服务端，用 ipconfig 记下宿主机 IP
#    ！下面命令里的 192.168.148.1 要换成你自己查到的地址（通常也是 192.168.x.1）

# ② 虚拟机中安装依赖（carla 客户端 + numpy，matplotlib 可选）
cd src/ground/carla_vehicle_dynamics_test
python3 -m pip install --user -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple

# ③ 离线单元测试（不需要 CARLA，先跑这个最省时间）
cd src/ground/carla_vehicle_dynamics_test
python3 test/test_metrics.py          # 预期：[结果] 30 项全部通过
python3 test/test_physics_parse.py    # 预期：[结果] 18 项全部通过

# ④ 单车型三项全测
python3 main.py --mode all --host 192.168.148.1

# ⑤ 多车型对比（结果写入 data/dynamics_results.csv 与 data/dynamics_curves.png）
python3 main.py --mode all --host 192.168.148.1 --vehicles all

# ⑥ 也可以用 ROS 2 launch 启动（同一套参数）
cd ~/ros2_ws && source install/setup.bash
ros2 launch carla_vehicle_dynamics_test main.launch.py host:=192.168.148.1 vehicles:=all
```

## 3. 常用参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--mode` | `all` | `accel` / `brake` / `turn` / `all` |
| `--host` | `127.0.0.1` | Windows 宿主机 IP |
| `--vehicle` | `vehicle.tesla.model3` | 单个车型 blueprint |
| `--vehicles` | - | 写 `all` 则循环内置三种车型对比 |
| `--v-target-kmh` | `60` | 目标车速（km/h） |
| `--steer` | `0.3` | 转向测试的归一化转向指令（-1~1） |
| `--turn-speed` | `8.0` | 转向测试目标车速（m/s） |
| `--out-dir` | `data` | CSV 与曲线图输出目录 |

## 4. 目录说明

```
main.py                       统一入口（check 参数 → 连接 CARLA → 逐车型测试 → 出表）
main.sh / main.bat            一键入口
carla_vehicle_dynamics_test/
├── metrics.py                纯数学函数（加速度/制动距离/圆拟合/理论转弯半径），可离线单测
├── carla_utils.py            CARLA 连接、同步步进、按车型生成车辆、跟随镜头、销毁
├── runner.py                 三项测试的测量流程
├── app.py                    命令行参数、对比表、CSV、曲线图
└── dynamics_node.py          ROS 2 节点：跑测试并把结果发布到 /dynamics/result
test/test_metrics.py          纯函数单元测试（30 项，不需要 CARLA）
test/test_physics_parse.py    车辆物理参数解析测试（18 项，假对象，不需要 CARLA）
launch/main.launch.py         ROS 2 启动入口
```

## 5. 结果如何核对

* **加速**：实测 0→60 km/h 时间与平均加速度一栏可直接和车型规格对比；
* **制动**：程序会用实测减速度反推匀减速理论距离 $s=v^2/(2a)$，两者应当接近；
* **转向**：用最小二乘圆拟合实测轨迹得到半径，与阿克曼理论值 $R=L/\tan\delta$
  （$L$ 从车辆物理参数读取）对照，二者差异反映轮胎侧滑等非理想因素。

完整推导、算法流程图与实测数据见在线文档：`docs/ground/carla_vehicle_dynamics_test.md`。
