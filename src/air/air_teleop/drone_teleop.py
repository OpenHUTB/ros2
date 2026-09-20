#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# 无人机终端键盘遥控器：drone_teleop
# 思路：W/S/A/D 控制前后左右平移，Space/Ctrl+P 控制垂直升降，J/L 控制原地偏航，
# 按键松开对应轴速度立即归零；T 起飞、G 降落、H 紧急悬停、ESC 安全退出。
#
# 坐标与通信：通过 AirSim RPC（默认 41451 端口）连接多旋翼，
# 在机体坐标系（Body Frame）下周期性下发速度指令，频率 10 Hz。
# 注意 AirSim 使用 NED 坐标系：z 轴指向地面，所以上升对应 vz < 0、下降对应 vz > 0。
#
# 载具名称：--vehicle 显式传入时直接使用；否则初始化时通过 client.listVehicles()
# 自动获取第一架载具名称，查询失败或列表为空时回退默认名 'SimpleFlight'。
# 所有 AirSim API 调用均显式传入 vehicle_name。

import math
import signal
import sys
import time

import airsim

# 终端按键：Windows 用 msvcrt 读取无缓冲按键，类 Unix 平台用 termios 切换终端模式
try:
    import msvcrt
except ImportError:      # Linux / macOS
    msvcrt = None
    import termios
    import tty


class DroneTeleop:
    """终端键盘遥控无人机：机体速度指令 + 实时 HUD + 安全退出"""

    # ---- 标称速度：类属性，按键对照表与速度指令共用同一份参数 ----
    LINEAR_SPEED = 3.0                            # 水平标称线速度 3.0 m/s
    VERTICAL_SPEED = 2.0                          # 垂直标称速度 2.0 m/s
    YAW_RATE_DEG = 30.0                           # 偏航角速度 30 度/秒

    # ---- 按键映射表：按键字节 -> 说明（ESC 单独处理，用于退出）----
    KEYMAP = {
        'w': '前进', 's': '后退',
        'a': '左移', 'd': '右移',
        ' ': '上升', '\x10': '下降',      # \x10 即 Ctrl+P（终端无法单独捕获 Shift 修饰键）
        'j': '左偏航', 'l': '右偏航',
        't': '起飞', 'g': '降落', 'h': '紧急悬停',
    }

    def __init__(self, ip="127.0.0.1", port=41451, vehicle_name=""):
        # 连接 AirSim RPC 服务，建立与仿真器之间的通信
        self.client = airsim.MultirotorClient(ip=ip, port=port)
        self.client.confirmConnection()
        # 载具名称：显式传入优先，否则 listVehicles 自动获取第一架载具
        self.vehicle_name = self._resolve_vehicle_name(self.client, vehicle_name)

        # 控制参数
        self.ctrl_freq = 10.0                     # 控制频率 10 Hz
        self.period = 1.0 / self.ctrl_freq        # 控制周期 0.1 s
        self.cmd_duration = self.period * 2       # 指令时长略长于周期，保证指令之间不断档
        self.hud_freq = 5.0                       # HUD 刷新频率 5 Hz

        self.yaw_rate = math.radians(self.YAW_RATE_DEG)   # 偏航角速度，换算成弧度/秒
        self.watchdog_timeout = 0.5               # 控制循环超时阈值，超时自动悬停

        # 运行状态
        self._old_term = None                     # 原终端属性，退出时还原
        self.flying = False                       # 是否已经起飞
        self.hovering = False                     # 是否处于紧急悬停
        self.exit_requested = False               # 收到一次退出信号
        self.pressed = set()                      # 当前仍然按住的按键集合
        self.takeoff_z = 0.0                      # 起飞点的 NED z，用于把高度显示成相对高度
        self.last_loop_at = time.time()           # 上一次控制循环的时刻，供看门狗判断
        self.last_hud_at = 0.0                    # 上一次刷新 HUD 的时刻
        self.last_warn_at = 0.0                   # 上一次打印看门狗告警的时刻

    # ---------------- 连接与载具 ----------------

    @staticmethod
    def _resolve_vehicle_name(client, requested=''):
        """确定载具名称：显式传入优先；否则 listVehicles 取第一架，失败回退 'SimpleFlight'"""
        if requested:
            return requested
        try:
            vehicles = client.listVehicles()
        except Exception as exc:
            print('[警告] listVehicles 查询失败：%s，回退默认载具名 SimpleFlight' % exc)
            return 'SimpleFlight'
        if vehicles:
            print('listVehicles 自动获取载具: %s' % vehicles)
            return vehicles[0]
        print('[警告] listVehicles 返回空列表，回退默认载具名 SimpleFlight')
        return 'SimpleFlight'

    # ---------------- 终端按键 ----------------

    @staticmethod
    def setup_stdout():
        """把标准输出切成 UTF-8，避免中文 HUD 在 Windows 终端里编码报错"""
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    def setup_keyboard(self):
        """切换到无缓冲按键模式，使 getch 立即返回每一个按键"""
        if msvcrt is None:                        # Linux / macOS
            self._old_term = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

    def teardown_keyboard(self):
        """还原终端属性，把终端交还给用户"""
        if self._old_term is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_term)
            self._old_term = None

    @staticmethod
    def read_key():
        """读一个按键并转成小写；没有按键时返回 None（不阻塞）"""
        if msvcrt is not None:                    # Windows
            if not msvcrt.kbhit():
                return None
            ch = msvcrt.getwch()
        else:                                     # Linux / macOS
            if not sys.stdin.isatty():
                return None
            import select
            if not select.select([sys.stdin], [], [], 0)[0]:
                return None
            ch = sys.stdin.read(1)

        if ch in ('\x00', '\xe0'):                # 方向键/功能键的前导码，再读一字节后丢弃
            if msvcrt is not None:
                msvcrt.getwch()
            return None
        ch = ch.lower()
        return ch if ch in DroneTeleop.KEYMAP or ch == '\x1b' else None

    def setup_signals(self):
        """注册退出信号处理：Ctrl+C / IPC 中断都走同一套安全退出流程"""
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                signal.signal(sig, self.on_interrupt)
            except (ValueError, OSError):         # 非主线程或平台不支持时忽略
                pass

    def on_interrupt(self, _signum, _frame):
        """收到中断信号：只置标志位，由控制循环负责安全收尾"""
        self.exit_requested = True

    # ---------------- 飞行控制 ----------------

    def takeoff(self):
        """起飞：解锁电机并爬升到安全高度"""
        if self.flying:
            return
        self.client.enableApiControl(True, vehicle_name=self.vehicle_name)
        self.client.armDisarm(True, vehicle_name=self.vehicle_name)
        # 起飞前先记下起点的 NED z，之后 HUD 显示的就是相对起飞点的高度
        self.takeoff_z = self.client.getMultirotorState(
            vehicle_name=self.vehicle_name).kinematics_estimated.position.z_val
        self.client.takeoffAsync(timeout_sec=10, vehicle_name=self.vehicle_name).join()
        self.client.hoverAsync(vehicle_name=self.vehicle_name).join()
        self.flying = True
        self.hovering = False

    def land(self, timeout_sec=15):
        """降落：先平稳悬停，再垂直降落，最后落到地面"""
        if not self.flying:
            return
        # 悬停一会儿，等速度衰减下来再降落，避免带着水平速度砸地
        self.client.hoverAsync(vehicle_name=self.vehicle_name).join()
        time.sleep(0.3)
        self.client.landAsync(timeout_sec=timeout_sec, vehicle_name=self.vehicle_name).join()
        self.flying = False
        self.hovering = False

    def release(self):
        """安全释放：切断机载电机动力并交还 API 控制权"""
        try:
            self.client.armDisarm(False, vehicle_name=self.vehicle_name)
            self.client.enableApiControl(False, vehicle_name=self.vehicle_name)
        except Exception as exc:                  # 补偿动作失败也不能影响退出
            print("[警告] 释放 API 控制权时出错：%s" % exc)

    def stop_and_hover(self):
        """紧急悬停：立即把三个轴的速度打成 0，原地保持高度"""
        if not self.flying:
            return
        self.client.hoverAsync(vehicle_name=self.vehicle_name)
        self.hovering = True

    # ---------------- 状态与 HUD ----------------

    def state(self):
        """读一次无人机状态，返回 (相对起飞点的高度 m, 线速度 m/s, 线速度大小 m/s)"""
        try:
            kinematics = self.client.getMultirotorState(
                vehicle_name=self.vehicle_name).kinematics_estimated
        except Exception:                         # 仿真器卡顿或连接抖动时沿用上一帧
            return None
        # NED 坐标系下 z 向下为正：取负号得到绝对高度，再减去起飞点才是相对高度
        altitude = -(kinematics.position.z_val - self.takeoff_z)
        vx = kinematics.linear_velocity.x_val
        vy = kinematics.linear_velocity.y_val
        vz = kinematics.linear_velocity.z_val
        if not all(map(math.isfinite, (altitude, vx, vy, vz))):
            return None
        return altitude, (vx, vy, vz), math.sqrt(vx * vx + vy * vy + vz * vz)

    @staticmethod
    def print_help():
        """启动时打印按键对照表"""
        print("=" * 62)
        print("             无人机终端键盘遥控器 · CarlaAir / AirSim")
        print("=" * 62)
        print("  W / S      前进 / 后退            (±%.1f m/s 机体 x 轴)" % DroneTeleop.LINEAR_SPEED)
        print("  A / D      左移 / 右移            (±%.1f m/s 机体 y 轴)" % DroneTeleop.LINEAR_SPEED)
        print("  Space      垂直上升               (NED vz = -%.1f m/s)" % DroneTeleop.VERTICAL_SPEED)
        print("  Ctrl+P     垂直下降               (NED vz = +%.1f m/s)" % DroneTeleop.VERTICAL_SPEED)
        print("  J / L      原地左偏航 / 右偏航    (±%.0f 度/秒)" % DroneTeleop.YAW_RATE_DEG)
        print("  T 起飞      G 降落                H 紧急悬停")
        print("  ESC        安全退出：悬停 -> 降落 -> 释放控制权")
        print("-" * 62)
        print("  提示：按键松开后对应轴速度自动归零；起飞前请先按 T 解锁电机")
        print("=" * 62)

    def render_hud(self, velocity, speed):
        """用 \r 原地刷新 HUD，显示实时高度与三轴线速度"""
        if velocity is None:
            self.hud("高度 --.-- m | 线速度 --.-- m/s")
            return
        vx, vy, vz = velocity
        # 用固定宽度对齐，避免数值抖动导致这一行左右跳
        self.hud("高度 %6.2f m | 线速度 vx %+6.2f  vy %+6.2f  vz %+6.2f  |v| %5.2f m/s"
                 % (-vz, vx, vy, vz, speed))

    @staticmethod
    def hud(text):
        """单行原地刷新：先补空格清掉上一次的残留字符，再回到行首"""
        width = 88
        sys.stdout.write("\r" + text.ljust(width))
        sys.stdout.flush()

    # ---------------- 控制循环 ----------------

    def frame_velocity(self, pressed):
        """把当前按下的按键集合翻译成机体坐标系下的速度指令（NED）与偏航角速度

        返回 (vx, vy, vz, yaw_rate)：vx 前、vy 右、vz 下（负值上升）、yaw_rate 弧度/秒。
        """
        vx = vy = vz = 0.0
        yaw = 0.0
        if 'w' in pressed:                        # 前进
            vx += self.LINEAR_SPEED
        if 's' in pressed:                        # 后退
            vx -= self.LINEAR_SPEED
        if 'd' in pressed:                        # 右移
            vy += self.LINEAR_SPEED
        if 'a' in pressed:                        # 左移
            vy -= self.LINEAR_SPEED
        if ' ' in pressed:                        # 上升：NED 下为负
            vz -= self.VERTICAL_SPEED
        if '\x10' in pressed:                     # 下降：NED 下为正
            vz += self.VERTICAL_SPEED
        if 'j' in pressed:                        # 左偏航（逆时针，正角速度）
            yaw += self.yaw_rate
        if 'l' in pressed:                        # 右偏航（顺时针，负角速度）
            yaw -= self.yaw_rate
        return vx, vy, vz, yaw

    def send_velocity(self, vx, vy, vz, yaw):
        """在机体坐标系下下发速度指令（异步，不阻塞控制循环）"""
        self.client.moveByVelocityBodyFrameAsync(
            vx, vy, vz, self.cmd_duration,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=math.degrees(yaw)),
            vehicle_name=self.vehicle_name)

    def handle_key(self, key):
        """处理一个按键；返回 False 表示需要退出主循环"""
        if key == '\x1b':                         # ESC
            print("\r\n[退出] 收到 ESC，开始安全降落")
            return False
        if key == 't':                            # 起飞
            self.takeoff()
            print("\r\n[起飞] 已到达安全高度，开始遥控")
        elif key == 'g':                          # 降落
            print("\r\n[降落] 正在降落，请稍候……")
            self.land()
            print("[降落] 已落地，按 T 可重新起飞")
        elif key == 'h':                          # 紧急悬停
            self.stop_and_hover()
            print("\r\n[悬停] 已紧急悬停，按方向键继续操控")
        elif key in 'wasd jl\x10':                # 方向键：解除悬停状态
            self.hovering = False
        return True

    def run(self):
        """主循环：10 Hz 读按键 -> 计算速度 -> 下发指令 -> 刷新 HUD"""
        self.setup_stdout()
        self.print_help()
        self.setup_keyboard()
        self.setup_signals()

        print("\r\n[连接] 已连接 AirSim 服务，按 T 起飞")
        try:
            while not self.exit_requested:
                loop_start = time.time()

                # 1) 读出本周期内所有按键：检测到就记入，本周期没再出现的视为已松开
                current = set()
                while True:
                    key = self.read_key()
                    if key is None:
                        break
                    if not self.handle_key(key):
                        self.exit_requested = True
                        break
                    current.add(key)
                self.pressed = current

                # 2) 看门狗：控制循环被阻塞太久时先悬停保底
                if loop_start - self.last_loop_at > self.watchdog_timeout and self.flying:
                    self.stop_and_hover()
                    if loop_start - self.last_warn_at >= 1.0:
                        self.last_warn_at = loop_start
                        print("\r\n[看门狗] 控制循环停顿超过 %.1f s，已自动悬停"
                              % self.watchdog_timeout)
                self.last_loop_at = loop_start

                # 3) 计算并下发指令：只有起飞后才允许操控，悬停时保持原地
                if self.flying and not self.hovering:
                    vx, vy, vz, yaw = self.frame_velocity(self.pressed)
                    self.send_velocity(vx, vy, vz, yaw)

                # 4) 刷新 HUD（限频，避免终端刷新过快看不清）
                if loop_start - self.last_hud_at >= 1.0 / self.hud_freq:
                    self.last_hud_at = loop_start
                    snapshot = self.state()
                    if snapshot is None:
                        self.render_hud(None, 0.0)
                    else:
                        altitude, velocity, speed = snapshot
                        self.render_hud(velocity, speed)

                # 5) 按控制周期补齐剩余时间，把频率稳定在 10 Hz
                sleep_time = self.period - (time.time() - loop_start)
                if sleep_time > 0:
                    time.sleep(sleep_time)
        finally:
            self.shutdown()

    def shutdown(self):
        """安全退出：平稳悬停 -> 降落 -> 释放 API 控制权 -> 还原终端"""
        print("\r\n[收尾] 正在安全退出……")
        try:
            if self.flying:
                self.stop_and_hover()
                time.sleep(0.5)
                self.land()
        except Exception as exc:
            print("[警告] 降落过程出错：%s" % exc)
        finally:
            self.release()
            self.teardown_keyboard()
            self.hud("已安全退出，控制权已释放")
            print()


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description="CarlaAir / AirSim 无人机终端键盘遥控器")
    parser.add_argument('--ip', default='127.0.0.1', help='AirSim 服务地址，默认本机')
    parser.add_argument('--port', type=int, default=41451, help='AirSim RPC 端口，默认 41451')
    parser.add_argument('--vehicle', default='', help='载具名称，默认取仿真器里的第一架')
    args = parser.parse_args()

    teleop = DroneTeleop(ip=args.ip, port=args.port, vehicle_name=args.vehicle)
    try:
        teleop.run()
    except KeyboardInterrupt:
        # Ctrl+C 已经由信号处理器接管，这里兜底处理极端情况
        teleop.exit_requested = True
