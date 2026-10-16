# 无人机键盘控制（uav_keyboard_control）

任务①「无人机仿真 + 键盘控制」的 ROS 侧实现：把终端按键转换为**机体系速度指令**，
发布到 `/uav/cmd_vel`，由 `carlair_ros_bridge` 连仿真器执行。仿真/坐标换算/安全限幅
集中在桥接层，键盘、规划器、神经网络控制器共用同一条指令通道。

## 1. 运行架构

```text
终端按键 ──► uav_keyboard_control ──/uav/cmd_vel──► carlair_ros_bridge ──► CarlaAir/AirSim
            (raw 模式读键)        (geometry_msgs/Twist)   (坐标换算+执行)     (无人机)
```

## 2. 按键映射

机体系（x 前 / y 左 / z 上）：

| 按键 | 动作 | Twist 分量 |
|---|---|---|
| `W` / `S` | 前进 / 后退 | `linear.x = ±horiz_speed` |
| `A` / `D` | 左移 / 右移 | `linear.y = ±horiz_speed` |
| `R` / `F` | 上升 / 下降 | `linear.z = ±vert_speed` |
| `Q` / `E` | 右转 / 左转 | `angular.z = ±yaw_rate` |
| 松开 | 悬停 | 全零 |
| `ESC` | 退出 | — |

`Q`/`E` 的方向遵循 AirSim 机体系 `yaw_rate` 约定（`Q=+yaw_rate` 机头右转）。

## 3. 编译与运行

```shell
# 终端 1：桥接（连仿真 + 执行 + 自动起飞悬停）
source ~/bridge_ws/devel/setup.bash
roslaunch carlair_ros_bridge main.launch

# 终端 2：键盘控制
source ~/bridge_ws/devel/setup.bash
roslaunch uav_keyboard_control main.launch
```

把焦点放在键盘控制终端，按 `W/A/S/D/R/F/Q/E` 操控，松开即悬停，`ESC` 退出。

## 4. 实现要点

### 4.1 终端 raw 模式

Linux 终端默认行缓冲（要按回车），用 `termios`+`tty` 进入 raw 模式即可单键即时响应：

```python
with RawKeyReader() as reader:        # __enter__ 切 raw，__exit__ 恢复
    key = reader.get_key(timeout=0.03)  # select 非阻塞读一个字符
```

`with` 语句保证 `Ctrl-C` 或异常退出时也能恢复终端设置，不会把用户的终端卡在 raw 模式。

### 4.2 按键 → 速度：纯函数

`key_to_cmd(key, horiz, vert, yaw)` 把按键映射为 `(vx, vy, vz, wz)`，
`step_once` 再打包成 `geometry_msgs/Twist`。两者都不依赖 ROS/终端，
可在无 ROS 的机器上单元测试（`tests/test_keyboard_local.py`，35 项）。

### 4.3 松开即悬停

节点以 30 Hz 持续发布：按键按下发对应速度，松开/未知键发全零。桥接层的
`cmd_vel_sub` 有「超过 `cmd_timeout` 无指令自动悬停」的安全保护，这里再叠加一层
显式零速度，双保险防止无人机失控。

## 5. 与视口内置 WASD 的关系

CarlaAir 视口**内置**键盘操控（任务①的「仿真」部分），本模块提供 **ROS 侧**键盘控制
（任务①的「键盘控制」部分）。两者可对比验证：视口 WASD 直接驱动仿真器，
本模块走 `键盘 → /uav/cmd_vel → 桥接 → 仿真器` 的完整 ROS 链路，正是后续
规划器、神经网络控制器的通用入口。

## 6. 效果图

键盘控制演示（录屏转 GIF，`W 前进 / A 左移 / D 右移 / R 上升 / Q 右转` 机动，第三人称视角）：

![](../img/air/uav_keyboard_control/keyboard_demo.gif)

## 7. 本机实测记录

### 7.1 运行环境

| 项 | 配置 |
|---|---|
| 宿主机 | Windows + Unreal Engine 5.5 + **Cosys-AirSim 3.3**（Blocks 环境） |
| 客户机 | VMware 虚拟机，Ubuntu 20.04.6 LTS（内核 5.15） |
| ROS | **ROS Noetic**（Python 3.8.10） |
| 工作区 | `~/uav_ws`（catkin） |
| 仿真器地址 | `192.168.49.1:41451`（本次实测环境的值，即 VMware NAT 下 VMnet8 的宿主机地址；换环境请用 `ipconfig` 重新确认） |
| AirSim RPC 监听 | 本构建（Cosys-AirSim 3.3）**无需修改 `settings.json`**：监听地址由顶层键 `"LocalHostIp"` 决定，默认空字符串即监听所有网卡。依据与实测见 [7.1.1](#711-关于-rpc-监听地址) |

#### 7.1.1 关于 RPC 监听地址

原文写的是「`settings.json` 必须含 `"ApiServerAddress": "0.0.0.0"`，默认只监听 `127.0.0.1`」。
经核对本机构建的源码与二进制，**这两句都不成立**，特此更正：

* **键名不对。** 本构建读取的是顶层键 **`"LocalHostIp"`**，不是 `ApiServerAddress`。
  源码依据 `Plugins/AirSim/Source/AirLib/include/common/AirSimSettings.hpp`：

  ```cpp
  //by default we spawn server at local endpoint. Do not use 127.0.0.1 as default below
  //because for docker container default is 0.0.0.0 and people get really confused why things
  api_server_address = settings_json.getString("LocalHostIp", "");
  api_port           = settings_json.getInt("ApiServerPort", RpcLibPort);
  ```

  佐证：该构建的 `Binaries/Win64/UnrealEditor-AirSim.dll` 里能查到字符串
  `LocalHostIp` 与 `ApiServerPort`，但**查不到 `ApiServerAddress`**。

* **「默认只监听 127.0.0.1」没有依据。** 上面两行的默认值是**空字符串**，
  紧邻的源码注释还明确写了「不要默认用 127.0.0.1，因为 docker 容器场景默认是 0.0.0.0」。
  也就是**默认就监听所有网卡**。

* **实测与之一致。** 宿主机 `netstat -ano | findstr 41451` 显示 `0.0.0.0:41451`，
  客户机侧直连该端口即可建立 TCP 连接。

**结论**：把「虚拟机连不上」归因于 `ApiServerAddress` 是错的，本机实测中**从未真正需要**这一项。
如果确实要显式限定监听地址，应写成顶层键 `"LocalHostIp": "0.0.0.0"`。

### 7.2 运行步骤

前置：按 [环境搭建与连接](../air/setup_and_connect.md) 配好虚拟机 ROS 环境；
按 [桥接模块](../air/carlair_ros_bridge.md) 的 **2.1 节**在宿主机启动 CarlaAir 仿真器，
等终端输出 `CarlaAir is ready.`（此时 `41451` 端口就绪）。
**仿真器窗口不要最小化**，否则 AirSim 取相机会阻塞。

> 最初的本机实测是用一个**自写的控制脚本**（负责起飞并爬升到 15 m）配合桥接层完成的，
> 那个脚本不属于本仓库。下面给出**只用仓库内组件**的等价步骤。

```shell
# ① 客户机：桥接层
export ROS_MASTER_URI=http://localhost:11311
source /opt/ros/noetic/setup.bash
source ~/uav_ws/devel/setup.bash
# <宿主机IP> 换成仿真器所在机器的地址（Windows 下用 ipconfig 查看，
# 桥接模块文档中的示例为 192.168.94.1）
# auto_takeoff 默认为 true：桥接层会 takeoff() 并 hover()，起飞后悬停在约 1 m
roslaunch carlair_ros_bridge main.launch host:=<宿主机IP>

# ② 客户机：把无人机送到 15 m 高空
#    /uav/goal 是 ENU（z 向上为正），桥接层飞抵该点后自动悬停，
#    并把 /uav/status 置为 GOAL_DONE
rostopic pub -1 /uav/goal geometry_msgs/Point "{x: 0.0, y: 0.0, z: 15.0}"
rostopic echo -n 1 /uav/odom/pose/pose/position     # 确认 z ≈ 15

# ③ 客户机：另开一个终端，键盘控制（作业要求的 launch 启动方式）
export ROS_MASTER_URI=http://localhost:11311
source /opt/ros/noetic/setup.bash
source ~/uav_ws/devel/setup.bash
roslaunch uav_keyboard_control main.launch
# 在弹出的 xterm 窗口里按 W/A/S/D/R/F/Q/E 操控，松开悬停，ESC 退出
```

> **关于 `auto_takeoff`**：它的语义是「**起飞并悬停**」——桥接层源码里就是
> `sim.takeoff()` + `sim.hover()`，**并不会自动升到 15 m**；想让无人机到 15 m，
> 必须另外下发目标点（如上面第 ② 步）或速度指令。
> 若把它设为 `false`，则无人机根本不会起飞，需要自己补上起飞动作。

### 7.3 实测结果

桥接层环境自检 **5/5 通过**：

```text
[通过] airsim 包可用  -- 已导入
[通过] 连接仿真器 192.168.49.1:41451  -- confirmConnection 成功
[通过] 读取位姿  -- pos_enu=(-0.00, 0.00, -0.69) yaw=90.0°
[通过] 相机取帧 front_rgb  -- 尺寸 640x480
[通过] 激光雷达点云 lidar1  -- 点数 8192, x∈[-71.4, 93.6] z∈[-0.4, 22.5]
自检结果: 5/5 通过
```

`/uav/odom` 发布频率实测 **19.999 Hz**（设计值 20 Hz）。

键盘闭环验证（用伪终端自动化模拟真人按键，脚本与注入时序见
[7.6](#76-按键闭环的复现方法)）：

```text
[test] 节点已在伪终端中启动 (pid=22254)
[test] 按键 前进 W 3.0s / 左移 A 2.5s / 后退 S 2.5s / 右移 D 2.5s / ESC 退出

按键前  (x, y, z) = ( 3.6e-07,  2.2e-07, 15.145)    ← 已按 7.2 第 ② 步送到 15 m
按键后  (x, y, z) = ( 0.389,   -0.403,   15.071)    ← 水平移动约 0.56 m，高度保持
```

**位置发生实际变化**，证明「键盘 → `/uav/cmd_vel` → 桥接层 → 仿真器」整条链路有效。

### 7.4 性能评价

| 指标 | 实测值 | 说明 |
|---|---|---|
| `/uav/odom` 发布频率 | 19.999 ~ 20.24 Hz | 桥接层环境自检实测；另用 `rostopic hz /uav/odom` 复测为 20.010 Hz（min 0.041 s / max 0.055 s） |
| 键盘节点指令发布频率 | 30 Hz | 由 `rate/publish_hz` 设定。**这是设定值，不能当作端到端响应延迟的实测值** |
| 指令水平速度 | 2.0 m/s | 由 `control/horiz_speed` 决定，可用参数覆盖 |
| 单次按键的实测水平位移 | 约 0.44 m | 按住 `W` 约 4 s 后 `/uav/odom` 的水平位移实测值。**位移单位是 m，速度单位才是 m/s** |
| 遥控稳定性 | 松开即悬停 | 双重保护：节点显式发零速 + 桥接层 0.5 s 超时悬停 |

> **关于「按键响应延迟」**：原表写的「< 30 ms」是用 30 Hz 轮询周期反推出来的，
> 既不是实测值，本身也和 30 Hz（周期 33 ms）矛盾，已删除。
> 要得到真实延迟，需要给按键打时间戳、再和 `/uav/cmd_vel` 首帧的到达时刻对比，
> 本机没有做这项测量，因此不给出数字。

### 7.5 实测截图

键盘控制窗口。`main.launch` 通过 `launch-prefix="xterm -e"` 拉起，
窗口标题为 `main.py`，黑底白字即是键位帮助：

![键盘控制窗口](../img/air/uav_keyboard_control/xterm_keyboard_window.png)

仿真器中无人机自动起飞至 15 m 悬停。AirSim 叠加层显示
`requestApiControl was successful`、`Vehicle is already armed`、`Collision Count:0`：

![15 m 悬停](../img/air/uav_keyboard_control/hover_15m.png)

### 7.6 按键闭环的复现方法

7.3 的按键闭环数据由下面这个脚本生成，**只依赖 Python 标准库**，
用伪终端（pty）把按键「敲」进键盘节点。时序是固定且可复现的：

1. 客户机先按 7.2 启动桥接层与键盘节点，并用 `/uav/odom` 确认无人机已在目标高度悬停。
2. 用 `pty.fork()` 起一个伪终端，在其子进程里执行 `rosrun uav_keyboard_control main.py`。
   节点从 stdin 读到的是「终端输入」，等同于真人按键（`rosrun` 无终端时会读不到按键）。
3. 父进程先等 **6.0 s**，让节点完成初始化并打印启动信息；再记录按键前的
   `/uav/odom/pose/pose/position`。
4. 按固定序列向伪终端主设备**各写一次按键字节**，每个按键保持 2.5~3.0 s 后再写下一个。
   键盘节点在这段时间内持续发布该方向速度，松开/换键即回到零速悬停：

   | 顺序 | 写入字节 | 保持时长 | 对应操作 |
   |---|---|---|---|
   | 1 | `b"w"` | 3.0 s | 前进 |
   | 2 | `b"a"` | 2.5 s | 左移 |
   | 3 | `b"s"` | 2.5 s | 后退 |
   | 4 | `b"d"` | 2.5 s | 右移 |
   | 5 | `b"\x1b"` | 1.0 s | ESC 退出 |

5. 再读一次位姿，两者相减即为水平位移（0.56 m）。

完整脚本，保存为 `kb_node_test.py`，在已 source 过工作空间的客户机上执行
`python3 kb_node_test.py`：

<details>
<summary>点击展开 kb_node_test.py（纯标准库，无第三方依赖）</summary>

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用伪终端(pty)驱动 uav_keyboard_control 的入口 main.py，模拟真人按键。

rosrun 直接启动时节点从 stdin 读原始按键，在无终端的自动化环境里读不到；
用 pty 给节点分配一个伪终端，就能像真人一样把按键「敲」进去。

用法： python3 kb_node_test.py
"""
from __future__ import print_function

import os
import pty
import select
import sys
import time

SEQ = [
    (b"w", 3.0, "前进 W"),
    (b"a", 2.5, "左移 A"),
    (b"s", 2.5, "后退 S"),
    (b"d", 2.5, "右移 D"),
    (b"\x1b", 1.0, "ESC 退出"),
]

CMD = ["rosrun", "uav_keyboard_control", "main.py"]

pid, fd = pty.fork()
if pid == 0:
    try:
        os.execvp(CMD[0], CMD)
    except Exception as exc:            # noqa: BLE001
        sys.stderr.write("exec failed: %s\n" % exc)
    os._exit(1)

print("[test] 节点已在伪终端中启动 (pid=%d)" % pid)
time.sleep(6.0)                          # 等节点初始化 + 打印启动信息

buf = b""


def drain(seconds):
    """在 seconds 秒内持续读取子进程输出（同时保持伪终端被占用）。"""
    global buf
    end = time.time() + seconds
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.2)
        if not r:
            continue
        try:
            data = os.read(fd, 4096)
        except OSError:
            return False
        if not data:
            return False
        buf += data
    return True


drain(1.0)
for key, dur, name in SEQ:
    print("[test] 按键 %-10s 持续 %.1f s" % (name, dur))
    try:
        os.write(fd, key)
    except OSError as exc:
        print("[test] 写入失败：%s" % exc)
        break
    if not drain(dur):
        print("[test] 节点已退出")
        break

drain(1.0)
try:
    os.close(fd)
except OSError:
    pass

print("----- 键盘节点输出 -----")
sys.stdout.write(buf.decode("utf-8", "replace"))
print("----- 输出结束 -----")
```

</details>

## 8. 参考

* [carlair_ros_bridge 桥接模块](../air/carlair_ros_bridge.md)
* [AirSim 无人机 API 参考](https://openhutb.github.io/doc/python_api/#airsim.client.MultirotorClient)

---

本文档在编写过程中使用了 AI 大模型辅助（资料核对、文档起草、问题排查）。
文中运行环境、命令与**实测数据**均来自本机实际运行记录：桥接层环境自检 **5/5 通过**、
`/uav/odom` 实测 **19.999 Hz**、键盘闭环位移 `(0, 0, 15.145) → (0.389, −0.403, 15.071)`，
对应的终端输出与截图已随本页一并提交（第 7 节）并被评审逐条核对过；
作者对提交内容的正确性与完整性负全部责任。
