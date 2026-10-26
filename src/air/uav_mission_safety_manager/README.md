# 无人机任务管理与安全状态机

ROS 2 的 `ament_python` 包，支持单航点、暂停/继续、返航自动降落与安全超时检测；原有 AirSim 流程使用 Ubuntu 20.04 / Foxy 验证。

新增：**多航点任务队列**，在 Ubuntu 22.04 / ROS 2 Humble 下构建与验证，支持最多 100 点顺序执行、进度反馈、暂停恢复和取消。详见[Humble 构建与本地演示](../../../docs/air/uav_mission_safety_manager/overview.md#waypoint-queue)。以下 Foxy 安装步骤保留供历史 AirSim 用法参考，Humble 演示请使用上述文档中的多航点章节。

## 文档入口

- [模块概览](../../../docs/air/uav_mission_safety_manager/overview.md)
- [构建与运行教程](../../../docs/air/uav_mission_safety_manager/usage.md)：依赖、WSL 地址、命令话题、服务、离线与实际模拟器复测命令。
- [AirSim 实测及已知限制](../../../docs/air/uav_mission_safety_manager/airsim_validation.md)

以上链接用于仓库源码视图；安装后的 share 目录可通过项目文档站 https://openhutb.github.io/ros2/ 阅读教程。

## 目录结构

```text
uav_mission_safety_manager/
├── README.md
├── LICENSE
├── package.xml
├── requirements.txt
├── setup.py
├── setup.cfg
├── config/mission.yaml
├── launch/main.launch.py
├── resource/uav_mission_safety_manager
├── uav_mission_safety_manager/
│   ├── __init__.py
│   ├── main.py                 # ROS 节点与 main() 入口
│   ├── controller.py           # 非阻塞控制器
│   ├── state_machine.py
│   ├── airsim_adapter.py
│   └── mock_adapter.py
└── tests/
    ├── test_adapter.py
    ├── test_controller.py
    ├── test_state_machine.py
    ├── ros_smoke.py
    ├── live_airsim_validation.py
    └── ros_airsim_validation.py
```

教程位于 `docs/air/uav_mission_safety_manager/`。
Python 文件使用英文下划线命名，文件名不包含日期；README、LICENSE 和 ROS 约定文件名保持通用写法。

## 快速构建与检查

在仓库根目录执行。ROS 依赖通过 `rosdep` 安装；实际 AirSim 模式还需初始化 pip 依赖：

```bash
source /opt/ros/foxy/setup.bash
rosdep install --from-paths src/air/uav_mission_safety_manager --ignore-src --rosdistro foxy -r -y
# AirSim 安装脚本提前导入这两项，首次安装需先准备构建工具和这两项。
python3 -m pip install --user "setuptools>=45.2.0,<76" "wheel<0.46" numpy==1.17.4 msgpack-rpc-python==0.4.1
python3 -m pip install --user --no-build-isolation -r src/air/uav_mission_safety_manager/requirements.txt
```

`requirements.txt` 面向 Python 3.8，包含 AirSim 客户端及其兼容依赖；仅离线运行可跳过两条 pip 命令。然后构建并检查：

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select uav_mission_safety_manager
source install/setup.bash
python3 -m unittest discover -s src/air/uav_mission_safety_manager/tests -v
ROS_DOMAIN_ID=87 python3 src/air/uav_mission_safety_manager/tests/ros_smoke.py
```

先用默认离线模式检查节点启动：

```bash
ros2 launch uav_mission_safety_manager main.launch.py use_airsim:=false
```

安装的可执行名保持 `mission_manager`，入口为 `uav_mission_safety_manager.main:main`。
直接运行与 launch 二选一，避免重复节点：

```bash
ros2 run uav_mission_safety_manager mission_manager --ros-args -p use_airsim:=false
```

实际 AirSim 模式、依赖及飞行操作请遵循完整教程。状态机启动不会自动连接、解锁或起飞。
实际模拟器测试结论与限制见上述验证文档。

## 已知限制

起飞 RPC 可能返回 False 但遥测显示已离地，原因未确定。FAILSAFE 不保证自动降落，停止节点不会停止模拟器飞行任务。
坐标为 AirSim NED；普通航点容差为 1 米。屋顶落地兼容逻辑仅限记录起点，不代表任意地形或真机验证。
多航点队列的本次验证使用本地运动演示适配器，尚未执行 AirSim 队列飞行。未实现 ROS2 Action、避障或实际电子围栏。详见新教程与历史实测记录。

## 维护与许可证

维护者：LuLuforeverx2（1357170941@qq.com）。许可证：[Apache-2.0](LICENSE)。
