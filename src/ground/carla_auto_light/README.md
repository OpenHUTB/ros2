# carla_auto_light：CARLA 地面载具灯光自动控制

在 CARLA 中根据**环境亮度与驾驶输入**自动控制车辆灯光，覆盖四类灯：

| 灯 | 触发条件 | 说明 |
|---|---|---|
| 近光灯 | 太阳高度角 < 5°（黄昏/夜间）或降水 ≥ 30 | 自动大灯 |
| 雾灯 | 降水 ≥ 30 | 与近光灯一起开 |
| 转向灯 | 方向盘绝对值 > 0.15 | **回正到 0.05 以内自动熄灭**（状态机） |
| 刹车灯 | 刹车量 > 0.05 | 跟随驾驶输入 |

演示时按 **白天 → 黄昏 → 夜间 → 暴雨** 四个阶段切换天气，每个阶段内依次执行
"巡航 → 点刹 → 打右转向 → 回正"，把四类灯全部演一遍。

## 1. 运行环境

| 项目 | 版本 / 说明 |
|---|---|
| 宿主机 | Windows 11，CARLA 0.9.16（解压后双击 `CarlaUE4.exe`） |
| 虚拟机 | Ubuntu 22.04 LTS（VMware），ROS 2 Humble，Python 3.10.12 |
| Python 依赖 | `carla==0.9.16`（见 `requirements.txt`） |

## 2. 运行步骤

```bash
# ① Windows 上启动 CARLA 服务端，用 ipconfig 记下宿主机 IP
#    下面的 192.168.148.1 要换成你自己查到的地址

# ② 虚拟机中安装依赖
cd src/ground/carla_auto_light
python3 -m pip install --user -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple

# ③ 离线单元测试（不需要 CARLA，先跑这个最省时间）
python3 test/test_light_logic.py      # 预期：[结果] 53 项全部通过

# ④ 演示
python3 main.py --host 192.168.148.1

# ⑤ 或通过 ROS 2 launch 启动（状态发布到 /vehicle/lights）
cd ~/ros2_ws && source install/setup.bash
ros2 launch carla_auto_light main.launch.py host:=192.168.148.1
```

## 3. 常用参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--host` | `127.0.0.1` | Windows 宿主机 IP |
| `--vehicle` | `vehicle.tesla.model3` | 车型 blueprint |
| `--town` | `Town03` | 地图 |
| `--phase-seconds` | `6` | 每个天气阶段时长（秒） |
| `--spawn-index` | `-1` | 出生点编号，-1 自动挑选 |
| `--fast` | 关闭 | 关闭实时节奏（默认按仿真时间 1:1 播放，便于观察灯光切换） |
| `--no-follow` | 关闭 | 关闭第三人称跟随镜头（默认跟随车辆） |

## 4. 目录说明

```
main.py                       统一入口
main.sh / main.bat            一键入口
carla_auto_light/
├── light_logic.py            纯决策逻辑（阈值表 + 转向灯状态机），可离线单测
├── carla_utils.py            CARLA 连接、同步、天气切换、灯光位掩码
├── light_node.py             ROS 2 节点：发布 /vehicle/lights
└── app.py                    命令行入口与四阶段演示流程
test/test_light_logic.py      纯逻辑单元测试（53 项，不需要 CARLA）
launch/main.launch.py         ROS 2 启动入口
```

## 5. 结果如何核对

演示结束会打印四阶段汇总表（阶段 / 太阳高度角 / 降水 / 灯光）。核对方法：

* 白天阶段应当**只有点刹与打转向时**才亮灯，其余为「无」；
* 黄昏与夜间阶段近光灯**常亮**；
* 暴雨阶段近光灯与雾灯**同时亮**；
* 转向灯在"回正"动作后应当**自动熄灭**（而不是一直亮）。

完整原理、阈值推导与实测记录见在线文档：`docs/ground/carla_auto_light.md`。
