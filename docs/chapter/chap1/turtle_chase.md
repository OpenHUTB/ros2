# 双海龟追逐实验

## 一、实验目的与环境说明

### 1.1 实验目的

掌握 ROS 话题通信机制:逃亡者、追逐者两个节点分别向 `/turtle1/cmd_vel`、`/turtle2/cmd_vel` 话题发布 `geometry_msgs/Twist` 速度消息,同时订阅 `/turtle1/pose`、`/turtle2/pose` 话题的 `turtlesim/Pose` 位姿消息,实时读取两只海龟的坐标与朝向;

掌握 ROS 服务通信机制:调用 `/spawn` 服务动态生成第二只小海龟(追逐者)、`/turtleN/set_pen` 服务设置画笔颜色与粗细、`/clear` 服务清空画布——这是与画正方形等单海龟实验相比新增的通信方式;

利用实时位姿反馈实现闭环控制:逃亡者采用纯跟踪(Pure Pursuit)法沿圆形轨迹逃跑,追逐者以"转向 P 控制 + 速度 P 控制"双闭环追赶,并用状态机管理"追逐—抓捕—跟随"三个阶段;

通过 turtlesim 仿真器,让两只小海龟完成"追逐—逃亡"协同对抗任务:追逐者从左下角出发,追上沿绿色圆周逃跑的逃亡者,终端打印抓捕用时与抓捕点,随后切换为跟随模式。

### 1.2 实验环境

| 项目 | 配置 |
| --- | --- |
| 操作系统 | Ubuntu 18.04(VMware 虚拟机) |
| ROS 发行版 | ROS Melodic(实测) |
| 仿真器 | turtlesim |
| 编程语言 | Python 2.7.17(Ubuntu 18.04 + Melodic 的 rospy 默认解释器,实测可运行);代码未使用 Python 2 专有语法,兼容 Python 3.6 及以上(已通过 Python 3.14 语法检查),在 Noetic(Ubuntu 20.04,Python 3.8)下仅需把脚本 shebang 改为 `#!/usr/bin/env python3` 即可运行 |
| 功能包 | turtle_chase |

功能包目录结构如下:

```
turtle_chase/
├── main.sh            # 一键运行脚本: 自动拷贝/编译/启动, 一条命令跑通整个实验
├── main.py            # 一键启动入口: 自动检测 roscore、启动仿真器与两个节点
├── package.xml        # 包清单,声明依赖 rospy、std_srvs、geometry_msgs、turtlesim
├── CMakeLists.txt     # 编译配置,把脚本安装到 bin 目录
├── README.md          # 运行环境与运行步骤说明
└── scripts/
    ├── runner.py      # 逃亡者节点:纯跟踪画圆逃跑(绿色轨迹)
    └── chaser.py      # 追逐者节点:spawn 生成 + 双 P 控制追赶(红色→洋红轨迹)
```

最新源码同步保存在本书仓库 `src/chap1/turtle_chase/`,可直接查看与下载。

## 二、核心控制原理与算法解析

### 2.1 话题通信机制

turtlesim 仿真器启动后,海龟 turtle1 会一直订阅名为 `/turtle1/cmd_vel` 的话题。我们编写的逃亡者节点 turtle_runner 作为发布者,向该话题发布 Twist 消息控制海龟 1 运动;与此同时,海龟通过 `/turtle1/pose` 话题持续广播自己的位姿(`turtlesim.msg.Pose`,包含坐标 x、y 和朝向角 theta),逃亡者节点订阅该话题,实时读取位姿作为反馈。

追逐者 turtle2 由 `/spawn` 服务动态生成,生成后与 turtle1 完全对等:拥有自己的 `/turtle2/cmd_vel`、`/turtle2/pose` 话题。追逐者节点 turtle_chaser 同时订阅两只海龟的位姿,一边控制自己追,一边"盯"着对方跑,这是双海龟协同的关键。

Twist 消息中本实验只用到两个分量:

- `linear.x`:线速度,控制海龟前进的快慢(单位 m/s);
- `angular.z`:角速度,控制海龟转头的快慢(单位 rad/s),取正值绕 z 轴逆时针旋转,即左转。

Pose 消息中用到三个分量:

- `x`、`y`:海龟在平面上的坐标(单位 m);
- `theta`:海龟的朝向角(单位 rad,取值范围 \([-\pi, \pi]\))。

注意:turtlesim 内置 0.5 秒看门狗——只要超过 0.5 秒没有收到新的 cmd_vel 消息,它就会自动把海龟速度清零(强制刹车)。所以两个节点的控制循环都以 `rospy.Rate(50)` 以 50 Hz 的频率采样位姿并重发当前指令,直到动作到位。

### 2.2 服务通信机制(双海龟的生成)

与单海龟实验不同,本实验的第二只海龟不是仿真器自带的,而是追逐者节点启动时调用 `/spawn` 服务动态生成的:

- `/spawn`(类型 `turtlesim/Spawn`):在指定坐标 \((2.0,\ 2.0)\) 以指定朝向 \(\pi/4\) 生成名为 turtle2 的海龟,服务响应返回实际海龟名;
- `/turtleN/set_pen`(类型 `turtlesim/SetPen`):设置画笔的 r、g、b 颜色、线宽 width 和 off 开关。本实验把逃亡者画笔设为绿色 (0,255,0)、追逐者设为红色 (255,0,0),抓住后换成洋红色粗笔 (255,0,255, 线宽 6) 标记跟随轨迹;
- `/clear`(类型 `std_srvs/Empty`):清空画布上的历史轨迹,逃亡者节点启动时调用一次,保证每次实验从干净的画布开始。

由于重复运行时 turtle2 可能已存在,/spawn 调用用 try/except 兼容:生成失败(海龟已存在)时直接复用,不影响实验继续。

### 2.3 逃亡者:纯跟踪法画圆逃跑

逃亡者的目标是沿画布中心 \((c_x, c_y) = (5.544,\ 5.544)\)(turtlesim 画布为 \(11 \times 11\))半径 \(R = 2.5\ \mathrm{m}\) 的圆周匀速逃跑。采用纯跟踪思想:在圆周上放置一个匀速转动的"虚拟目标点"(carrot point),相位 \(\varphi(t) = \varphi_0 + \omega t\),其中初始相位 \(\varphi_0\) 由海龟当前所在方位角确定,保证启动瞬间目标点就在前方附近;角速度 \(\omega = 0.42\ \mathrm{rad/s}\) 与线速度 \(v = 1.2\ \mathrm{m/s}\) 相匹配,使虚拟点"跑得动、追得上"。

每个控制周期(50 Hz)执行:

1. 计算虚拟目标点坐标:\(t_x = c_x + R\cos\varphi\),\(t_y = c_y + R\sin\varphi\);
2. 计算期望航向:\(\theta_d = \operatorname{atan2}(t_y - y,\ t_x - x)\);
3. 航向误差 \(\Delta\theta = \theta_d - \theta\),先归一化到 \([-\pi, \pi]\) 再做 P 控制:\(\omega = k_\omega \Delta\theta\)(\(k_\omega = 5.0\));
4. 偏差较小时全速逃跑(\(v = 1.2\ \mathrm{m/s}\));偏差过大(\(|\Delta\theta| > 0.8\ \mathrm{rad}\))时减速到 0.4 倍,先把车头摆正再加速,保证轨迹平滑收敛。

由于虚拟目标点始终在圆周前方,海龟会自然地沿圆形轨迹稳定"逃跑",无需显式计算圆弧。

### 2.4 追逐者:双闭环 P 控制与状态机

追逐者同时订阅两只海龟的位姿,把逃亡者位置 \((x_1, y_1)\) 换算到自己的极坐标系下:距离 \(d = \sqrt{(x_1 - x_2)^2 + (y_1 - y_2)^2}\),方位差 \(\Delta\theta = \operatorname{atan2}(y_1 - y_2,\ x_1 - x_2) - \theta_2\)(归一化到 \([-\pi, \pi]\))。控制器分为两个闭环:

- **转向环(角度 P 控制)**:\(\omega = k_\omega \Delta\theta\)(\(k_\omega = 6.0\)),始终把车头对准逃亡者当前位置,这是"追得上"的前提;
- **速度环(距离 P 控制)**:追逐阶段全速 \(v = 1.7\ \mathrm{m/s}\)(偏差过大时先降速转向);抓住后切换为跟随模式,\(v = \operatorname{clamp}\left(k_v (d - d_0),\ 0,\ v_{\max}\right)\),其中 \(d_0 = 0.5\ \mathrm{m}\) 为期望跟随距离,\(k_v = 4.0\)——距离越远追得越快,恰好保持 \(d_0\) 时自然停下,实现稳定"尾随"而不冲过。

抓捕判定构成一个三态状态机:

1. **追逐**(\(d \ge 0.6\ \mathrm{m}\)):全速接近;
2. **抓捕**(\(d < 0.6\ \mathrm{m}\),只触发一次):终端打印 `>>> 抓住逃亡者! 用时 X s, 抓捕点 (x, y)`,画笔换成洋红色粗线;
3. **跟随**(抓住之后):按速度环保持 \(d_0\) 距离,随逃亡者一起沿圆周环绕。

工程细节:两个位姿回调在不同线程执行,共享位姿数据用 `threading.Lock` 加锁保护;节点退出时(`rospy.on_shutdown`)再发一次零速度,保证 Ctrl+C 后海龟立即停住。

### 2.5 算法流程

一键运行脚本 main.sh:自动把功能包拷贝到 `~/catkin_ws/src/` → `catkin_make` 编译 → source 工作空间 → 检测并启动 roscore → 启动 turtlesim 仿真器 → 依次拉起逃亡者与追逐者节点;Ctrl+C 时统一清理全部子进程;

一键启动入口 main.py:探测 roscore(解析 ROS_MASTER_URI 并试连端口)→ 未运行则先启动 roscore → 启动 turtlesim 仿真器 → 依次拉起逃亡者与追逐者节点;Ctrl+C 时按进程组清理全部子进程;

逃亡者 runner.py:初始化节点、发布者、订阅者、50 Hz 频率对象 → 调用 /clear 清屏、/turtle1/set_pen 设绿色画笔 → 等待第一帧位姿,由当前方位角确定虚拟目标点初始相位 \(\varphi_0\) → 循环:计算目标点 → 纯跟踪 P 控制发布速度;

追逐者 chaser.py:初始化节点与话题 → 调用 /spawn 生成 turtle2、设红色画笔 → 等待双方位姿 → 循环:极坐标解算距离与方位差 → 抓捕判定(触发一次打印与换笔)→ 转向环 + 速度环发布控制指令;

循环内外均通过 `rospy.is_shutdown()` 和 try/except 做保护,节点被 Ctrl+C 中断时海龟也能安全停下。

## 三、完整源码展示

### 3.1 一键运行脚本 main.sh

最新源码同步保存在本书仓库 `src/chap1/turtle_chase/main.sh`,可直接查看与下载。

```bash
#!/usr/bin/env bash
# 双海龟"追逐—逃亡"协同控制实验 —— 一键运行脚本 main.sh
#
# 用法: bash main.sh    (在本模块目录 src/chap1/turtle_chase/ 下执行)
# 功能: 自动拷贝功能包到 catkin 工作空间、编译并 source, 检测并启动
#       roscore 与 turtlesim 仿真器, 再拉起逃亡者 runner.py 和追逐者
#       chaser.py; 按 Ctrl+C 退出时自动清理全部子进程。

set -u

MODULE_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$HOME/catkin_ws"

# 0. ROS 环境未加载时自动 source
if [ -z "${ROS_DISTRO:-}" ]; then
    source /opt/ros/*/setup.bash
fi

# 1. 拷贝功能包到工作空间(从工作空间内部运行时跳过, 避免目录自拷贝)
if [ "$MODULE_DIR" != "$WS_DIR/src/turtle_chase" ]; then
    mkdir -p "$WS_DIR/src"
    echo "[main.sh] 拷贝功能包到 $WS_DIR/src/"
    cp -r "$MODULE_DIR" "$WS_DIR/src/"
fi

# 2. 编译并 source
echo "[main.sh] 编译工作空间..."
(cd "$WS_DIR" && catkin_make) || { echo "[main.sh] catkin_make 失败, 请检查报错"; exit 1; }
source "$WS_DIR/devel/setup.bash"

# 3. 检测 roscore, 没有就启动一个
if ! rostopic list >/dev/null 2>&1; then
    echo "[main.sh] 启动 roscore..."
    roscore &
    sleep 3
fi

# 4. 启动仿真器与两个节点
rosrun turtlesim turtlesim_node &
TURTLE_PID=$!
sleep 2
rosrun turtle_chase runner.py &
RUNNER_PID=$!
sleep 1
rosrun turtle_chase chaser.py &
CHASER_PID=$!

cleanup() {
    echo "[main.sh] 正在退出, 清理全部子进程..."
    kill $CHASER_PID $RUNNER_PID $TURTLE_PID 2>/dev/null
    wait 2>/dev/null
    echo "[main.sh] 已退出"
}
trap cleanup EXIT INT TERM

echo "[main.sh] 实验运行中, 按 Ctrl+C 一键退出"
wait
```

### 3.2 一键启动入口 main.py

最新源码同步保存在本书仓库 `src/chap1/turtle_chase/main.py`,可直接查看与下载。

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
双海龟追逐实验 —— 一键启动入口(main.py)

功能: 自动检测 roscore 与 turtlesim 仿真器, 依次拉起
     逃亡者节点 runner.py 和追逐者节点 chaser.py;
     Ctrl+C 退出时自动清理全部子进程。

运行(需先 source 工作空间):
    rosrun turtle_chase main.py
或
    python main.py
"""

import os
import signal
import socket
import subprocess
import time


def master_alive():
    """通过 ROS_MASTER_URI 探测 roscore 是否已经在运行"""
    uri = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
    try:
        host = uri.split('//')[-1].split(':')[0].split('/')[0]
        port = int(uri.split(':')[-1].split('/')[0])
        s = socket.socket()
        s.settimeout(1.0)
        try:
            s.connect((host, port))
            return True
        finally:
            s.close()
    except Exception:
        return False


def start(cmd):
    """以独立进程组启动子进程, 便于退出时整组清理"""
    print('启动: ' + cmd)
    return subprocess.Popen(cmd, shell=True, preexec_fn=os.setsid)


def main():
    procs = []

    if master_alive():
        print('检测到 roscore 已在运行, 跳过启动')
    else:
        procs.append(start('roscore'))
        time.sleep(3)

    procs.append(start('rosrun turtlesim turtlesim_node'))
    time.sleep(2)                                    # 等仿真窗口弹出
    procs.append(start('rosrun turtle_chase runner.py'))
    time.sleep(1)
    procs.append(start('rosrun turtle_chase chaser.py'))

    print('实验运行中, 按 Ctrl+C 一键退出')
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print('正在退出, 清理全部子进程...')
    finally:
        for p in reversed(procs):
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception:
                pass
        time.sleep(1)
        print('已退出')


if __name__ == '__main__':
    main()
```

### 3.3 逃亡者节点 scripts/runner.py

最新源码同步保存在本书仓库 `src/chap1/turtle_chase/scripts/runner.py`,可直接查看与下载。

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
双海龟追逐实验 —— 逃亡者节点(turtle1)

原理: 在画布中心的圆周上放一个匀速转动的"虚拟目标点",
     逃亡者按照纯跟踪(Pure Pursuit)思想, 始终朝虚拟目标点转向,
     从而沿圆形轨迹稳定地"逃跑"; 画笔设置为绿色。
"""

import math

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from std_srvs.srv import Empty


def normalize_angle(angle):
    """把角度差归一化到 [-pi, pi], 防止跨 +-pi 时转向跳变"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class Runner:
    def __init__(self):
        rospy.init_node('turtle_runner')

        # ---------- 可调参数(rosparam, 可用 _参数名:=值 覆盖) ----------
        self.speed = rospy.get_param('~speed', 1.2)          # 逃跑线速度 m/s
        self.radius = rospy.get_param('~radius', 2.5)        # 圆形轨迹半径 m
        self.carrot_w = rospy.get_param('~carrot_w', 0.42)   # 虚拟目标点角速度 rad/s
        self.turn_gain = rospy.get_param('~turn_gain', 5.0)  # 转向P控制增益
        self.rate = rospy.Rate(50)                           # 50Hz发布, 规避看门狗

        self.cx, self.cy = 5.544445, 5.544445   # 画布中心(turtlesim画布11x11)
        self.pose = None                        # 最新位姿, 由回调更新

        self.cmd_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_cb, queue_size=10)

        # 调用服务: 清空画布轨迹, 逃亡者画笔设为绿色
        rospy.wait_for_service('/clear')
        rospy.ServiceProxy('/clear', Empty)()
        rospy.wait_for_service('/turtle1/set_pen')
        rospy.ServiceProxy('/turtle1/set_pen', SetPen)(0, 255, 0, 3, 0)
        rospy.loginfo('逃亡者就绪: 沿半径 %.1f m 的圆周逃跑', self.radius)

    def pose_cb(self, msg):
        self.pose = msg

    def publish_cmd(self, v, w):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def run(self):
        rospy.on_shutdown(lambda: self.publish_cmd(0, 0))

        # 等待第一帧位姿, 用当前位置确定虚拟目标点的初始相位
        while not rospy.is_shutdown() and self.pose is None:
            self.rate.sleep()
        p = self.pose
        phi0 = math.atan2(p.y - self.cy, p.x - self.cx)
        t0 = rospy.get_time()

        while not rospy.is_shutdown():
            p = self.pose
            # 虚拟目标点("胡萝卜")沿圆周匀速转动
            phi = phi0 + self.carrot_w * (rospy.get_time() - t0)
            tx = self.cx + self.radius * math.cos(phi)
            ty = self.cy + self.radius * math.sin(phi)
            # 期望航向 = 指向虚拟目标点的方向, 对航向误差做P控制
            err = normalize_angle(math.atan2(ty - p.y, tx - p.x) - p.theta)
            v = self.speed if abs(err) < 0.8 else 0.4 * self.speed  # 偏差大时先减速
            self.publish_cmd(v, self.turn_gain * err)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        Runner().run()
    except rospy.ROSInterruptException:
        pass
```

### 3.4 追逐者节点 scripts/chaser.py

最新源码同步保存在本书仓库 `src/chap1/turtle_chase/scripts/chaser.py`,可直接查看与下载。

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
双海龟追逐实验 —— 追逐者节点(turtle2)

原理: 1) 调用 /spawn 服务动态生成第二只小海龟作为追逐者;
     2) 同时订阅两只海龟的位姿话题, 实时计算相对距离与方位;
     3) 转向P控制对准目标 + 速度P控制(抓住后保持跟随距离);
     4) 距离小于阈值判定"抓住", 变换画笔颜色并打印抓捕信息,
        随后切换为跟随模式, 稳定地跟在逃亡者身后。
"""

import math
import threading

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, SetPen


def normalize_angle(angle):
    """把角度差归一化到 [-pi, pi], 防止跨 +-pi 时转向跳变"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class Chaser:
    def __init__(self):
        rospy.init_node('turtle_chaser')

        # ---------- 可调参数(rosparam, 可用 _参数名:=值 覆盖) ----------
        self.max_speed = rospy.get_param('~max_speed', 1.7)      # 最大线速度 m/s
        self.turn_gain = rospy.get_param('~turn_gain', 6.0)      # 转向P控制增益
        self.catch_dist = rospy.get_param('~catch_dist', 0.6)    # 抓捕判定距离 m
        self.follow_dist = rospy.get_param('~follow_dist', 0.5)  # 跟随保持距离 m
        self.rate = rospy.Rate(50)                               # 50Hz, 规避看门狗

        self.lock = threading.Lock()  # 两个位姿回调在不同线程, 加锁保护共享数据
        self.my_pose = None           # 追逐者 turtle2 的位姿
        self.target = None            # 逃亡者 turtle1 的位姿
        self.caught = False           # 是否已抓住
        self.t_start = None           # 开始追逐的时刻

        self.cmd_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/turtle2/pose', Pose, self.my_cb, queue_size=10)
        rospy.Subscriber('/turtle1/pose', Pose, self.target_cb, queue_size=10)

        self.spawn_turtle()
        rospy.loginfo('追逐者就绪: 最大速度 %.1f m/s', self.max_speed)

    def my_cb(self, msg):
        with self.lock:
            self.my_pose = msg

    def target_cb(self, msg):
        with self.lock:
            self.target = msg

    def spawn_turtle(self):
        """调用 /spawn 服务在左下角生成追逐者, 画笔设为红色"""
        try:
            rospy.wait_for_service('/spawn', timeout=5.0)
            spawn = rospy.ServiceProxy('/spawn', Spawn)
            name = spawn(2.0, 2.0, math.pi / 4, 'turtle2').name
            rospy.loginfo('已生成追逐者: %s', name)
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logwarn('生成 turtle2 失败(可能已存在, 将直接使用): %s', e)
        try:
            rospy.wait_for_service('/turtle2/set_pen', timeout=5.0)
            rospy.ServiceProxy('/turtle2/set_pen', SetPen)(255, 0, 0, 3, 0)
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logwarn('设置追逐者画笔失败: %s', e)

    def publish_cmd(self, v, w):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def run(self):
        rospy.on_shutdown(lambda: self.publish_cmd(0, 0))

        while not rospy.is_shutdown():
            with self.lock:
                me, tgt = self.my_pose, self.target
            if me is None or tgt is None:   # 还没收到位姿, 原地等待
                self.rate.sleep()
                continue
            if self.t_start is None:
                self.t_start = rospy.get_time()

            # 相对距离与方位(极坐标)
            dx, dy = tgt.x - me.x, tgt.y - me.y
            dist = math.hypot(dx, dy)
            err = normalize_angle(math.atan2(dy, dx) - me.theta)

            # ---- 抓捕判定(只触发一次) ----
            if not self.caught and dist < self.catch_dist:
                self.caught = True
                rospy.loginfo('>>> 抓住逃亡者! 用时 %.1f s, 抓捕点 (%.2f, %.2f)',
                              rospy.get_time() - self.t_start, me.x, me.y)
                try:  # 换洋红色粗画笔, 标记抓捕后的跟随轨迹
                    rospy.ServiceProxy('/turtle2/set_pen', SetPen)(255, 0, 255, 6, 0)
                except rospy.ServiceException:
                    pass

            # ---- 速度控制: 追逐阶段全速, 跟随阶段按距离P控制 ----
            if self.caught:
                v = max(0.0, min(self.max_speed, 4.0 * (dist - self.follow_dist)))
            else:
                v = self.max_speed if abs(err) < 0.9 else 0.5
            self.publish_cmd(v, self.turn_gain * err)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        Chaser().run()
    except rospy.ROSInterruptException:
        pass
```

### 3.5 构建文件 CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(turtle_chase)

## 查找catkin和我们用到的依赖(rospy、std_srvs、geometry_msgs和turtlesim)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_srvs
  geometry_msgs
  turtlesim
)

## 声明这个catkin包,因为包里面只有python代码,所以不用导出库
catkin_package()

## 把三个脚本装到bin目录下,装好之后就可以用
## rosrun turtle_chase main.py 直接运行了
catkin_install_python(PROGRAMS
  main.py
  scripts/runner.py
  scripts/chaser.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### 3.6 包清单 package.xml

```xml
<?xml version="1.0"?>
<!-- turtle_chase 功能包清单: 双海龟"追逐—逃亡"协同控制实验 -->
<package format="3">
  <name>turtle_chase</name>
  <version>1.0.0</version>
  <description>双海龟追逐实验: 逃亡者纯跟踪圆周逃跑, 追逐者P控制追赶并跟随</description>
  <maintainer email="student@example.com">student</maintainer>
  <license>MIT</license>

  <!-- 构建工具依赖 -->
  <buildtool_depend>catkin</buildtool_depend>

  <!-- 运行依赖: rospy负责节点通信, std_srvs提供/clear服务类型,
       geometry_msgs提供Twist消息, turtlesim提供仿真器与Spawn/SetPen服务 -->
  <depend>rospy</depend>
  <depend>std_srvs</depend>
  <depend>geometry_msgs</depend>
  <depend>turtlesim</depend>
</package>
```

## 四、运行与验证

### 4.1 编译功能包

```bash
cp -r src/chap1/turtle_chase/ ~/catkin_ws/src/
cd ~/catkin_ws              # 进入工作空间根目录
catkin_make                 # 编译
source devel/setup.bash     # 刷新环境
chmod +x ~/catkin_ws/src/turtle_chase/main.sh ~/catkin_ws/src/turtle_chase/main.py ~/catkin_ws/src/turtle_chase/scripts/*.py   # 确保可执行权限
```

### 4.2 启动节点

推荐方式:运行一键脚本 main.sh,自动完成拷贝、编译、source,并检测启动 roscore、turtlesim 与两个节点,一条命令跑通整个实验:

```bash
cd ~/catkin_ws/src/turtle_chase
bash main.sh
```

也可以在编译完成后用一键启动入口 main.py(不再重复拷贝与编译):

```bash
rosrun turtle_chase main.py
```

还可以分四个终端手动启动(注意先启动逃亡者再启动追逐者,因为逃亡者启动时会清空画布并设置画笔):

```bash
# 终端1:启动 ROS 主节点
roscore

# 终端2:启动小海龟仿真器
rosrun turtlesim turtlesim_node

# 终端3:运行逃亡者节点,海龟1开始沿绿色圆周逃跑
rosrun turtle_chase runner.py

# 终端4:运行追逐者节点,自动生成海龟2并开始追赶
rosrun turtle_chase chaser.py
```

重复实验前先重置:`rosservice call /reset`。

### 4.3 话题/服务订阅与发布验证

在节点运行的同时,另开终端依次执行:

```bash
rostopic list                        # 应看到 /turtle1/cmd_vel /turtle2/cmd_vel /turtle1/pose /turtle2/pose
rostopic info /turtle1/cmd_vel       # 速度话题类型,应为 geometry_msgs/Twist
rostopic info /turtle2/cmd_vel       # 发布者应为 turtle_chaser
rostopic info /turtle1/pose          # 位姿话题类型,应为 turtlesim/Pose
rostopic echo /turtle2/cmd_vel       # 实时打印追逐者发出的速度消息
rosnode list                         # 确认 /turtle_runner /turtle_chaser 已注册
rosservice list | grep -E "spawn|clear|set_pen"   # 确认 /spawn /clear /turtleN/set_pen 服务存在
rqt_graph                            # 查看两节点—四话题的计算图
```

节点运行时 `rostopic echo /turtle2/cmd_vel` 的输出节选如下:追逐阶段 `linear.x` 为全速 1.7,转弯修正时 `angular.z` 出现较大值;抓住后进入跟随阶段,`linear.x` 随相对距离在 0~1.7 之间自动调节。由于控制循环以 50 Hz 持续发布,同一阶段会连续刷出内容相近的消息。

```
linear:
  x: 1.7
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 2.31
---
```

### 4.4 预期结果

节点运行后,在 turtlesim 窗口中可以依次看到:海龟 1 沿绿色圆形轨迹匀速逃跑;左下角自动出现海龟 2(由 /spawn 服务生成),拖着红色轨迹直线冲刺追赶;约 4~5 秒后追上,追逐者终端同步打印 `已生成追逐者: turtle2`、`追逐者就绪: 最大速度 1.7 m/s`、`>>> 抓住逃亡者! 用时 4.5 s, 抓捕点 (7.94, 5.66)`(实测数据);随后海龟 2 轨迹变为洋红色粗线,以约 \(d_0 = 0.5\ \mathrm{m}\) 的距离稳定跟随海龟 1 沿圆周环绕,直到按 Ctrl+C 退出。

小海龟追逐实验效果图(红色轨迹为追逐过程,洋红色为抓捕后的跟随轨迹,绿色为逃亡者的圆周轨迹):

![小海龟追逐实验效果图](../../img/chapter/turtle_chase.png)

结果分析:抓捕用时约 4.5 s,主要取决于追逐者与逃亡者的速度差(\(1.7 - 1.2 = 0.5\ \mathrm{m/s}\))以及追逐者的出生点 \((2.0,\ 2.0)\) 到逃亡圆周的直线距离。若用 `_max_speed:=1.3` 把追逐者降速到与逃亡者接近,可观察到抓捕用时明显变长,验证速度差决定抓捕时间的分析;跟随阶段洋红轨迹与绿色圆周基本重合,说明距离 P 控制的速度环收敛良好。

## 五、总结

本实验通过编写逃亡者、追逐者两个节点,综合运用了 ROS 的话题通信(双海龟的 cmd_vel 发布与 pose 订阅)和服务通信(/spawn 动态生成海龟、/set_pen 设置画笔、/clear 清屏),实现了双海龟"追逐—逃亡—跟随"的协同对抗任务。实验中掌握了纯跟踪法轨迹跟踪与"转向 P 控制 + 速度 P 控制"双闭环的设计方法,理解了抓捕判定的三态状态机,并练习了 50 Hz 高频循环规避 turtlesim 0.5 秒看门狗、角度差 ±π 跨界归一化、多线程回调加锁、服务调用容错、main.sh 一键运行脚本与进程清理等工程技巧。与画正方形等单海龟实验相比,本实验更贴近真实机器人系统中"多节点分布式协作 + 对抗性任务"的场景,加深了对 ROS 分布式通信架构和闭环反馈控制的理解。

## 六、声明

本实验的代码与文档在 ZCode 智能体辅助下编写,本人已逐一测试验证全部内容可正常运行(运行环境:Ubuntu 18.04 + ROS Melodic),并对提交内容负全部责任。
