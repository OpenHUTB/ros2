# carla_nn_car_following

基于神经网络的自适应跟车（ACC）仿真模块：在 CARLA 中生成自车与前车，用**前向毫米波雷达**感知，
由**感知网络**估计前车距离与相对速度、**控制网络**输出油门与刹车，实现自动跟车；
同时保留**经典 ACC 控制器**作为训练标签来源与性能基线。

* 在线文档：<https://openhutb.github.io/ros2/ground/carla_nn_car_following/>
* 完整说明（公式、算法流程、源码解析、性能评价、修复记录）见 `docs/ground/carla_nn_car_following.md`

## 1. 运行环境

| 项目 | 版本 |
|---|---|
| 宿主机 | Windows 10/11，CARLA 0.9.16（Windows 发行版解压即用，双击其中的 `CarlaUE4.exe`） |
| 虚拟机 | Ubuntu 22.04 LTS（VMware），ROS 2 Humble |
| Python | 3.10.12（CARLA 0.9.16 的 wheel 只提供 cp310 / cp311 / cp312） |
| Python 依赖 | `carla==0.9.16`（`pip install --user`）、`numpy`；`matplotlib` 可选 |

> Ubuntu 20.04 自带的 Python 3.8 **装不上** CARLA 0.9.16 的官方 wheel，因此本模块使用 22.04 + Humble。

## 2. 运行步骤

```bash
# ① 在 Windows 上双击启动 CARLA 服务端，并用 ipconfig 记下宿主机 IP
#    本模块实测：VMware NAT 下为 VMnet8 的地址 192.168.148.1
#    ！注意：下面的 192.168.148.1 都要换成你自己查到的地址（通常也是 192.168.x.1）

# ② 虚拟机中安装 CARLA Python 客户端
python3 -m pip install --user carla==0.9.16 -i https://pypi.tuna.tsinghua.edu.cn/simple

# ③ 编译本功能包
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone --depth 1 https://github.com/OpenHUTB/ros2.git
cp -r ros2/src/ground/carla_nn_car_following ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select carla_nn_car_following --symlink-install
source install/setup.bash

cd ~/ros2_ws/src/carla_nn_car_following

# ④ 环境自检
python3 check_env.py --host 192.168.148.1 --town Town03

# ⑤ 离线测试（不需要 CARLA，先跑这个最省时间）
python3 test/test_reference_acc.py       # 24 项
python3 test/test_nn_numpy.py            # 17 项

# ⑥ 经典 ACC 跟车（基线）
python3 main.py --mode scene --host 192.168.148.1 --scenario decelerate

# ⑦ 采集训练数据（四种工况）
python3 main.py --mode collect --host 192.168.148.1 --scenario all

# ⑧ 训练两个网络（不需要 CARLA）
python3 main.py --mode train

# ⑨ 神经网络控制器在线跟车
python3 main.py --mode run --host 192.168.148.1 --scenario decelerate --controller nn

# ⑩ 用 ROS 2 launch 启动
ros2 launch carla_nn_car_following main.launch.py \
    host:=192.168.148.1 town:=Town03 scenario:=decelerate controller:=nn

# ⑪ 另开终端观察状态
ros2 topic echo /acc/state

# ⑫ 多场景性能评测
python3 main.py --mode eval --host 192.168.148.1 --scenario all
```

## 3. 目录说明

```
main.py                 统一入口：check / scene / collect / train / run / eval
check_env.py            环境自检（连接、自车、雷达、ROS 2）
diagnose_drive.py       诊断工具：逐个出生点试验"车能不能开动"
main.sh / main.bat      一键入口
carla_nn_car_following/
  carla_client.py       连接 CARLA、生成自车与前车、挂载雷达、出生点预检
  scenario.py           四种前车工况 + 纯追踪/面包屑横向控制
  reference_acc.py      经典 ACC 控制器（标签来源与基线）
  nn_numpy.py           纯 NumPy 的 MLP（前向、反向、Adam）
  train.py              训练感知网络与控制网络
  nn_controller.py      神经网络控制器（感知 → 控制 级联）
  acc_scene_node.py     ROS 2 节点，发布 /acc/state
launch/main.launch.py   ROS 2 启动入口（controller:=classic | nn）
config/params.yaml      运行参数
test/                   24 + 17 项离线测试（都不需要 CARLA）
```

## 4. 单元测试（不需要 CARLA）

```bash
cd src/ground/carla_nn_car_following
python3 test/test_reference_acc.py      # 经典 ACC 公式与四种工况
python3 test/test_nn_numpy.py           # MLP、标准化器、端到端训练流程
```

## 5. 常见问题

| 现象 | 解决 |
|---|---|
| 连接超时 | 用 `ipconfig` 确认宿主机 IP；确认 `CarlaUE4.exe` 已启动；防火墙放行该程序 |
| 雷达一帧数据都没有 | 传感器句柄被回收：必须保存为 `self.sensor` |
| 前车生成在几十米外 | 新生成 actor 的位置数据未到达客户端，生成后需 `settle()`（本模块已修复） |
| 车不动 / 撞路沿 | 出生点被挡住或缺少横向控制，参见文档第 9 章"修复记录" |
| 车辆出生即逆行 | 使用 `map.get_spawn_points()` 自带的朝向 |
| 退出后 CARLA 卡住 | 在 `finally` 中恢复 `world.apply_settings(original_settings)` |
| `No module named 'carla'` | 安装与运行必须使用同一个解释器（`python3`） |
