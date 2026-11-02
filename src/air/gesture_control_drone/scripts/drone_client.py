#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AirSim 多旋翼控制封装（本模块唯一与仿真器打交道的地方）。

能力清单
--------
| 分类 | 接口 |
|---|---|
| 连接 | :meth:`connect` / :meth:`close`（退出时先悬停、再交还 API 控制） |
| 起飞 | :meth:`_takeoff`：解锁 → ``takeoffAsync`` → 定高 ``moveToZAsync`` → 悬停 |
| 落地 | :meth:`_land` |
| 姿态 | :meth:`_hover`、:meth:`_yaw`（偏航速率） |
| 位移 | :meth:`_move_body`（机体系速度）、:meth:`_up`、:meth:`_down`（带高度限幅） |
| 返航 | :meth:`_home`（``goHomeAsync``，老版本回退到 ``moveToPositionAsync``） |
| 急停 | :meth:`_estop`：``hover`` / ``land`` / ``disarm`` 三选一，默认悬停 |
| 状态 | :meth:`state` → :class:`DroneState`（NED→ENU 后的位姿/速度/落地状态/碰撞） |

设计要点
--------
* **连接复用**：只建一次并接管 API 控制（原实现每条手势都新建客户端）。
* **地址可配**：``ip``/``port``/``vehicle_name`` 都来自 ROS 参数；跨机部署
  （模拟器在宿主 Windows、ROS 在客户机 Ubuntu）时填宿主 IP。
* **指令超时保护**：``*Async().join()`` 在仿真器卡住/掉线时会永久阻塞，这里把每次
  RPC 调用放到工作线程并加超时，超时视为失败（:class:`CommandTimeout`）。
* **安全限幅**：速度不超过 ``max_speed``，升高不超过 ``max_altitude``，
  下降不低于 ``min_altitude``（避免一路降到地面）。超限直接拒绝并说明原因。
* **可测试**：不 import rospy、``airsim`` 延迟导入、客户端可注入
  （``client_factory``）、时钟可注入，因此整套逻辑可离线单测。
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Optional, Tuple

import gesture_map as gm

ESTOP_ACTIONS = ("hover", "land", "disarm")

#: 官方 ``airsim.types.LandedState``：``Landed = 0``，``Flying = 1``
#: （旧版本这里写成 0=Unknown / 1=Landed，与官方相反，已修正）
LANDED_STATE_NAMES = {0: "Landed", 1: "Flying"}

#: 服务端裸 RPC 需要的默认参数：``DrivetrainType.MaxDegreeOfFreedom`` + 不控偏航
#: （与官方客户端 ``moveByVelocityBodyFrameAsync`` 的默认值等价）
RAW_DRIVETRAIN = 0
RAW_YAW_MODE = {"is_rate": True, "yaw_or_rate": 0}


class CommandTimeout(RuntimeError):
    """RPC 调用超过 ``command_timeout`` 秒未返回。"""


def ned_to_enu(x, y, z):
    """AirSim(NED: 北/东/下) -> ROS 常用(ENU: 东/北/上)。交换 x/y 并对 z 取负。"""
    return (float(y), float(x), -float(z))


def enu_to_ned(x, y, z):
    """ENU -> NED（与 :func:`ned_to_enu` 是同一个变换，自逆）。"""
    return (float(y), float(x), -float(z))


def landed_state_name(value):
    """把 ``MultirotorState.landed_state`` 变成可读名字。"""
    name = getattr(value, "name", None)
    if isinstance(name, str):
        return name
    try:
        return LANDED_STATE_NAMES.get(int(value), "Unknown(%s)" % value)
    except (TypeError, ValueError):
        return str(value)


@dataclass
class DroneState(object):
    """一次状态快照（坐标已转成 ENU，单位 m / m/s）。"""

    position_enu: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity_enu: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    landed_state: str = "Unknown"
    api_control: bool = False
    collision: bool = False
    stamp: float = field(default=0.0)

    @property
    def altitude(self):
        """离原点高度（ENU 的 z，向上为正）。"""
        return self.position_enu[2]

    def summary(self):
        return ("pos_enu=(%.2f, %.2f, %.2f) alt=%.2f m state=%s api=%s%s"
                % (self.position_enu[0], self.position_enu[1],
                   self.position_enu[2], self.altitude, self.landed_state,
                   "ON" if self.api_control else "OFF",
                   "  ⚠碰撞" if self.collision else ""))


class AirSimDrone(object):
    """把 :mod:`gesture_map` 的指令 token 翻译成 AirSim 多旋翼动作。"""

    def __init__(self, ip="127.0.0.1", port=41451, vehicle_name="",
                 estop_action="hover", speed=1.0, move_duration=1.0,
                 takeoff_altitude=3.0, max_speed=3.0, max_altitude=30.0,
                 min_altitude=0.5, yaw_rate=30.0, command_timeout=10.0,
                 takeoff_timeout=60.0, land_timeout=60.0,
                 state_lock_timeout=0.1,
                 connect_retries=3, log=None, client_factory=None,
                 clock=None):
        self.ip = str(ip)
        self.port = int(port)
        self.vehicle_name = str(vehicle_name or "")
        self.estop_action = str(estop_action).lower()
        if self.estop_action not in ESTOP_ACTIONS:
            raise ValueError("estop_action 必须是 %s 之一，收到 %r"
                             % (list(ESTOP_ACTIONS), estop_action))
        self.speed = max(0.05, abs(float(speed)))
        self.max_speed = max(self.speed, abs(float(max_speed)))
        self.move_duration = max(0.05, abs(float(move_duration)))
        self.takeoff_altitude = abs(float(takeoff_altitude))
        self.max_altitude = abs(float(max_altitude))
        if self.takeoff_altitude > self.max_altitude:
            raise ValueError("takeoff_altitude(%.1f m) 不能超过 max_altitude(%.1f m)"
                             % (self.takeoff_altitude, self.max_altitude))
        self.min_altitude = max(0.0, abs(float(min_altitude)))
        self.yaw_rate = abs(float(yaw_rate))
        self.command_timeout = max(0.1, float(command_timeout))
        # 起飞/降落是慢动作：3 m 高度按 ~0.22 m/s 下降要约 14 s，
        # 用全局 command_timeout(10 s) 会误判超时，所以单独给上限。
        self.takeoff_timeout = max(self.command_timeout, float(takeoff_timeout))
        self.land_timeout = max(self.command_timeout, float(land_timeout))
        # msgpack-rpc 客户端不是线程安全的（状态定时器与指令工作线程共用同一连接），
        # 所有对主客户端的 RPC 都串行化。
        self._api_lock = threading.RLock()
        # 状态读取抢不到锁时最多等这么久就放弃（不能把 5 Hz 的 odom 卡在
        # 一次 20 s 的降落后面）。
        self.state_lock_timeout = max(0.0, float(state_lock_timeout))
        self.connect_retries = max(1, int(connect_retries))

        self._log = log if log is not None else (lambda msg: None)
        self._client_factory = client_factory
        self._clock = clock if clock is not None else time.monotonic

        self.client = None
        self.connected = False
        self.api_control = False
        self.armed = False
        self.api_mode = "unknown"      # "async" / "sync" / "unknown"（连上后探测）
        self.last_command = gm.CMD_NONE
        self.last_error = ""
        self.counts = {}
        self.last_state = DroneState()
        #: 最近一次 state() 是否真的读到了数据；节点据此决定是否发布 odom
        self.state_valid = False

        self._handlers = {
            gm.CMD_TAKEOFF: self._takeoff,
            gm.CMD_LAND: self._land,
            gm.CMD_HOVER: self._hover,
            gm.CMD_FORWARD: self._forward,
            gm.CMD_UP: self._up,
            gm.CMD_DOWN: self._down,
            gm.CMD_YAW_LEFT: self._yaw_left,
            gm.CMD_YAW_RIGHT: self._yaw_right,
            gm.CMD_HOME: self._home,
            gm.CMD_ESTOP: self._estop,
        }
        missing = [c for c in gm.ALL_COMMANDS if c not in self._handlers]
        if missing:      # 词表加了指令却忘了实现 —— 启动即失败，别等到现场
            raise RuntimeError("gesture_map 里有指令没有实现：%s" % missing)

    # ------------------------------------------------------------------ #
    # 连接
    # ------------------------------------------------------------------ #
    def _kw(self):
        """需要传 vehicle_name 的调用用它；空值时什么都不传（兼容老版本客户端）。"""
        return {"vehicle_name": self.vehicle_name} if self.vehicle_name else {}

    def _make_client(self):
        if self._client_factory is not None:
            return self._client_factory(ip=self.ip, port=self.port)
        try:
            import airsim
        except ImportError:
            # Windows 上官方 airsim 包要编译 msgpack-python(0.5.6，需 MSVC)，
            # 很多机器装不上；Cosys-AirSim 的客户端 RPC 协议一致，可直接顶上。
            try:
                import cosysairsim as airsim
            except ImportError as exc:
                raise RuntimeError(
                    "缺少 AirSim 客户端：pip install airsim（Linux/macOS 推荐）"
                    " 或 pip install cosysairsim（Windows 无需编译器）；"
                    "两者 RPC 协议一致") from exc
        return airsim.MultirotorClient(ip=self.ip, port=self.port)

    def connect(self):
        """建立连接并接管 API 控制；成功返回 True。"""
        if self.connected:
            return True
        for attempt in range(1, self.connect_retries + 1):
            try:
                self.client = self._make_client()
                with self._api_lock:
                    connected = self.client.confirmConnection()
                # 不同客户端版本行为不同：airsim 1.8.1 的 confirmConnection() 只打印
                # "Connected!" 而**不返回值**（None），所以只有明确 False 才算失败。
                if connected is False:
                    raise RuntimeError("confirmConnection() 返回 False"
                                       "（仿真器没开 / IP 不对 / 41451 被防火墙拦）")
                with self._api_lock:
                    self.client.enableApiControl(True, **self._kw())
                self._detect_api_mode()
                self.connected = True
                self.api_control = True
                self.last_error = ""
                self._log("Connected! AirSim %s:%d%s"
                          % (self.ip, self.port,
                             (" 载具 %s" % self.vehicle_name
                              if self.vehicle_name else "")))
                self._ensure_unpaused()
                return True
            except Exception as exc:
                self.last_error = str(exc)
                self.connected = False
                self.api_control = False
                self._log("连接 AirSim %s:%d 失败（第 %d/%d 次）：%s"
                          % (self.ip, self.port, attempt,
                             self.connect_retries, exc))
                if attempt < self.connect_retries:
                    time.sleep(0.5)
        return False

    def close(self):
        """安全退出：先悬停，再交还 API 控制（**不要**在空中锁桨）。"""
        if self.client is None:
            return
        try:
            if self.connected:
                try:
                    self._hover(self.client)
                except Exception:
                    pass
                try:
                    with self._api_lock:
                        self.client.enableApiControl(False, **self._kw())
                except Exception:
                    pass
        finally:
            self.connected = False
            self.api_control = False
            self._log("已交还 API 控制并断开 AirSim")

    # ------------------------------------------------------------------ #
    # 超时保护
    # ------------------------------------------------------------------ #
    def _call(self, fn, what, timeout=None):
        """在工作线程里执行一次 RPC 调用，超时抛 :class:`CommandTimeout`。

        * 持有 :attr:`_api_lock`：同一个 msgpack-rpc 连接不能被两个线程同时用
          （状态定时器 5 Hz 读状态 + 指令工作线程执行动作），否则响应会错位。
        * ``timeout`` 默认取 ``command_timeout``；起飞/降落传各自的上限。
        """
        limit = self.command_timeout if timeout is None else float(timeout)
        with self._api_lock:
            box = {}

            def _run():
                try:
                    box["value"] = fn()
                except BaseException as exc:   # 原样带回调用线程
                    box["error"] = exc

            worker = threading.Thread(target=_run, name="airsim_call", daemon=True)
            worker.start()
            worker.join(limit)
            if worker.is_alive():
                raise CommandTimeout("%s 超过 %.1fs 未返回（仿真器可能卡住/掉线）"
                                     % (what, limit))
            if "error" in box:
                raise box["error"]
            return box.get("value")

    # ------------------------------------------------------------------ #
    # 兼容层：客户端能力不一样，而**不是**服务端不一样
    #   * 官方客户端（airsim / cosysairsim）：提供 *Async 包装，
    #     内部等价于 call_async('takeoff', ...)；
    #   * 精简/自写客户端：可能只有同步方法，或只暴露底层 client.call。
    # 服务端在所有版本都只有 takeoff / land / hover / moveByVelocityBodyFrame …。
    # 这里做能力探测 + 回退，保证同一份代码在两种客户端上都能用。
    # ------------------------------------------------------------------ #
    @staticmethod
    def _supports(client, name):
        return callable(getattr(client, name, None))

    def _detect_api_mode(self):
        """连上后探测这个 build 提供异步接口还是只有同步接口。"""
        if self._supports(self.client, "takeoffAsync"):
            self.api_mode = "async"
            self._log("AirSim API：异步接口（takeoffAsync/landAsync/hoverAsync）")
        elif self._supports(self.client, "takeoff") or self._has_raw_rpc():
            self.api_mode = "sync"
            self._log("AirSim API：客户端未提供 *Async（服务端方法名本来就只有 "
                      "takeoff/land/hover）—— 使用同步回退")
        else:
            self.api_mode = "unknown"
            self._log("⚠ 无法识别 AirSim API 模式（既没有 takeoffAsync 也没有 takeoff），"
                      "动作可能失败")
        return self.api_mode

    def _has_raw_rpc(self):
        """客户端是否暴露底层 msgpack-rpc（airsim 客户端的 .client.call）。"""
        raw = getattr(self.client, "client", None)
        return raw is not None and callable(getattr(raw, "call", None))

    def _sync_rpc(self, client, method, *args, vehicle_name=None,
                  raw_args=None):
        """调用同步方法；客户端里没有该方法时退回底层 RPC。

        两条路的参数**不是一回事**：

        * **客户端方法**（如 ``hover(vehicle_name)``、
          ``rotateByYawRate(rate, duration, vehicle_name)``）：用 ``args`` +
          ``vehicle_name`` 关键字；旧客户端没有该形参时退回纯位置调用。
        * **服务端裸 RPC**（``client.call(...)``）：参数个数是固定的，而且往往
          比客户端方法多。例如 ``moveByVelocityBodyFrame`` 要 7 个::

              vx, vy, vz, duration, drivetrain, yaw_mode, vehicle_name

          只补一个 ``vehicle_name`` 仍然会
          ``Function ... was called with an invalid number of arguments``。
          这类方法由调用方用 ``raw_args`` 把**完整**参数列表给进来。
        """
        fn = getattr(client, method, None)
        if callable(fn):
            if vehicle_name is None:
                return fn(*args)
            try:
                return fn(*args, vehicle_name=vehicle_name)
            except TypeError:              # 旧客户端没有 vehicle_name 形参
                return fn(*args)
        if self._has_raw_rpc():
            raw = getattr(client, "client")
            if raw_args is not None:       # 调用方给了服务端的完整参数
                return raw.call(method, *raw_args)
            if vehicle_name is None:
                return raw.call(method, *args)
            return raw.call(method, *args, vehicle_name)
        raise RuntimeError("客户端没有 %s()，也没有底层 call()" % method)

    def _ensure_unpaused(self):
        """仿真被暂停时物理不推进，takeoff 会永远等不到高度 —— 主动取消。

        实测：暂停状态下 takeoff 高度恒为 0，且同步 takeoff 请求永远不返回。
        """
        probe = getattr(self.client, "simIsPaused", None)
        if not callable(probe):
            return None
        try:
            paused = bool(probe())
        except Exception as exc:
            self._log("查询暂停状态失败：%s" % exc)
            return None
        if paused:
            self._log("⚠ 仿真处于暂停状态，自动 simPause(False) 后继续")
            try:
                self.client.simPause(False)
            except Exception as exc:
                self._log("取消暂停失败：%s" % exc)
        return paused

    def _fire_and_forget(self, method, *args):
        """用一条**独立连接**把同步命令发出去，后台线程等它，自己立即返回。

        两个原因：
        1. 同步 ``takeoff`` / ``land`` 要等动作做完才返回（3 m 降落约 14 s），
           留在调用线程里会把 ROS 回调/定时器堵住；
        2. 同一个 msgpack-rpc 连接不能被两个线程并发使用，独立连接天然隔离，
           不会和状态轮询抢同一个 socket。

        状态仍由调用方轮询 :meth:`state` 观察。
        """
        client = self._make_client()
        try:
            client.confirmConnection()
        except Exception:
            pass
        holder = {"client": client}

        def _run():
            try:
                self._sync_rpc(holder["client"], method, *args)
            except Exception as exc:
                self._log("同步命令 %s 在后台返回异常（动作可能已执行）：%s"
                          % (method, exc))
            finally:
                for closer in ("close", "sock"):
                    obj = holder["client"]
                    target = getattr(obj, closer, None)
                    if closer == "sock":
                        target = getattr(target, "close", None)
                    if callable(target):
                        try:
                            target()
                            break
                        except Exception:
                            continue

        thread = threading.Thread(target=_run, name="airsim_sync_cmd", daemon=True)
        thread.start()
        self._log("已下发同步命令 %s%s（不等待返回，状态靠轮询）"
                  % (method, list(args)))
        return thread

    # ------------------------------------------------------------------ #
    # 指令
    # ------------------------------------------------------------------ #
    def execute(self, command):
        """执行一条指令 token；成功返回 True，失败记录 :attr:`last_error`。"""
        if not gm.is_command(command):
            self._log("忽略未知指令：%r（可用：%s）"
                      % (command, list(gm.ALL_COMMANDS)))
            return False
        if not self.connect():
            return False
        self._log("执行：%s（%s）" % (gm.command_cn(command), command))
        try:
            self._handlers[command](self.client)
            self.last_command = command
            self.last_error = ""
            self.counts[command] = self.counts.get(command, 0) + 1
            return True
        except Exception as exc:
            self.last_error = str(exc)
            self._log("指令 %s 执行失败：%s" % (command, exc))
            return False

    # -- 各动作 --------------------------------------------------------- #
    # 每个动作都按 api_mode 分支：新版走 *Async().join()，
    # 老版（只有同步接口）走同步调用；会阻塞的同步命令一律"发完即弃"。
    def _takeoff(self, client):
        altitude = self.takeoff_altitude
        if altitude > self.max_altitude:
            raise RuntimeError("起飞高度 %.1f m 超过上限 %.1f m"
                               % (altitude, self.max_altitude))
        self._call(lambda: client.armDisarm(True, **self._kw()), "解锁")
        self.armed = True

        if self.api_mode == "async":
            self._call(lambda: client.takeoffAsync(**self._kw()).join(), "起飞",
                       timeout=self.takeoff_timeout)
            # takeoffAsync 只到默认高度，再定高过去
            self._call(lambda: client.moveToZAsync(-altitude, self.speed,
                                                  **self._kw()).join(),
                       "升到 %.1f m" % altitude,
                       timeout=self.takeoff_timeout)
            self._call(lambda: client.hoverAsync(**self._kw()).join(), "悬停")
            return

        # 老 build：同步 takeoff 会阻塞到动作完成（实测 >120 s 才返回），
        # 因此用独立连接发完即弃；高度由仿真器默认值决定，不做定高。
        self._fire_and_forget("takeoff", 20.0, self.vehicle_name)
        self._log("同步模式：起飞已下发，爬升到仿真器默认高度"
                  "（此 build 不支持定高到 %.1f m）" % altitude)

    def _land(self, client):
        if self.api_mode == "async":
            self._call(lambda: client.landAsync(**self._kw()).join(), "降落",
                       timeout=self.land_timeout)
            return
        self._fire_and_forget("land", 60.0, self.vehicle_name)

    def _hover(self, client):
        if self.api_mode == "async":
            self._call(lambda: client.hoverAsync(**self._kw()).join(), "悬停")
            return
        self._call(lambda: self._sync_rpc(
            client, "hover", vehicle_name=self.vehicle_name), "悬停")

    def _forward(self, client):
        self._move_body(client, +1.0, 0.0, 0.0, "前进")

    def _up(self, client):
        state = self.state(client=client)
        headroom = self.max_altitude - state.altitude
        duration = self._clamped_duration(headroom, "已到最大高度 %.1f m" % self.max_altitude)
        self._move_body(client, 0.0, 0.0, -1.0, "上升", duration=duration)

    def _down(self, client):
        state = self.state(client=client)
        headroom = state.altitude - self.min_altitude
        duration = self._clamped_duration(
            headroom, "已到最低高度 %.1f m，请用降落" % self.min_altitude)
        self._move_body(client, 0.0, 0.0, +1.0, "下降", duration=duration)

    def _yaw(self, client, rate, what):
        if self.api_mode == "async":
            self._call(lambda: client.rotateByYawRateAsync(
                rate, self.move_duration, **self._kw()), what)
            return
        try:
            self._call(lambda: self._sync_rpc(
                client, "rotateByYawRate", rate, self.move_duration,
                vehicle_name=self.vehicle_name), what)
        except Exception as exc:
            raise RuntimeError(
                "rotateByYawRate 调用失败（%s）；偏航不可用，"
                "可改用 moveByVelocityBodyFrame" % exc)

    def _yaw_left(self, client):
        # NED 下偏航正方向是顺时针（右转），左转取负
        self._yaw(client, -self.yaw_rate, "左转")
        self._brake(client)

    def _yaw_right(self, client):
        self._yaw(client, +self.yaw_rate, "右转")
        self._brake(client)

    def _home(self, client):
        if self.api_mode == "async" and callable(getattr(client, "goHomeAsync", None)):
            # goHomeAsync(timeout_sec, vehicle_name)：第一个位置参数是**超时**
            # 不是速度（之前误传 self.speed，等于给返航设了 1 秒超时）
            self._call(lambda: client.goHomeAsync(
                60.0, **self._kw()).join(), "返航")
        elif self.api_mode == "async":
            self._call(lambda: client.moveToPositionAsync(
                0.0, 0.0, -self.takeoff_altitude, self.speed,
                **self._kw()).join(), "返航（回退到 moveToPositionAsync）")
        else:
            # 同步分支同理：goHome(timeout_sec, vehicle_name)
            self._fire_and_forget("goHome", 60.0, self.vehicle_name)
            return
        self._brake(client)

    def _estop(self, client):
        if self.estop_action == "hover":
            self._brake(client, duration=0.8)
            self._log(">>> 紧急停止：已刹停 <<<")
        elif self.estop_action == "land":
            self._land(client)
            self._log(">>> 紧急停止：立即降落 <<<")
        else:                                    # disarm
            self._call(lambda: client.armDisarm(False, **self._kw()), "锁桨")
            self.armed = False
            self._log(">>> 紧急停止：已锁桨（空中执行会坠落，"
                      "恢复需重新 armDisarm(True)）<<<")

    # -- 通用工具 ------------------------------------------------------- #
    def _clamped_duration(self, headroom_m, refuse_msg):
        """把一次位移限制在剩余空间内；没空间则拒绝执行。"""
        if headroom_m <= 0.05:
            raise RuntimeError(refuse_msg)
        max_step = self.speed * self.move_duration
        if headroom_m >= max_step:
            return self.move_duration
        return headroom_m / self.speed

    def _brake(self, client, duration=0.5):
        """用零速度指令把飞机刹住。

        为什么不直接 ``hover()``：实测这个 build 在**移动中**调用 ``hover`` 会让
        飞机持续掉高度（0.85 m → 落地）；而静止时 ``hover`` 是安全的。
        零速度刹车在两种状态下都表现稳定（实测停住后抖动 ±0.05 m）。
        """
        desc = "刹车"
        dur = max(0.1, duration)
        if self.api_mode == "async":
            self._call(lambda: client.moveByVelocityBodyFrameAsync(
                0.0, 0.0, 0.0, dur, **self._kw()).join(), desc)
        else:
            # 服务端裸 RPC 要 7 个参数：vx, vy, vz, duration, drivetrain,
            # yaw_mode, vehicle_name —— 只补 vehicle_name 会少 2 个。
            self._call(lambda: self._sync_rpc(
                client, "moveByVelocityBodyFrame", 0.0, 0.0, 0.0, dur,
                vehicle_name=self.vehicle_name,
                raw_args=(0.0, 0.0, 0.0, dur, RAW_DRIVETRAIN, RAW_YAW_MODE,
                          self.vehicle_name)), desc)

    def _move_body(self, client, sx, sy, sz, what, duration=None):
        """机体系速度控制：sx 前、sy 右、sz 下（NED），单位向量乘速度。"""
        duration = self.move_duration if duration is None else duration
        v = min(self.speed, self.max_speed)
        vx, vy, vz = sx * v, sy * v, sz * v
        desc = "%s（v=%.2f m/s, %.2f s）" % (what, v, duration)

        if self.api_mode == "async":
            self._call(lambda: client.moveByVelocityBodyFrameAsync(
                vx, vy, vz, duration, **self._kw()).join(), desc)
        else:
            try:
                # 服务端裸 RPC 要 7 个参数（见 _sync_rpc 的说明）
                self._call(lambda: self._sync_rpc(
                    client, "moveByVelocityBodyFrame", vx, vy, vz, duration,
                    vehicle_name=self.vehicle_name,
                    raw_args=(vx, vy, vz, duration, RAW_DRIVETRAIN,
                              RAW_YAW_MODE, self.vehicle_name)), desc)
            except Exception as exc:
                raise RuntimeError(
                    "moveByVelocityBodyFrame 调用失败（%s）；"
                    "机体系速度控制不可用" % exc)
        self._brake(client)

    # ------------------------------------------------------------------ #
    # 状态
    # ------------------------------------------------------------------ #
    def state(self, client=None):
        """读取一次载具状态并转成 ENU；失败时返回上一帧并记录错误。

        **抢不到 ``_api_lock`` 就直接放弃这次读取**（``state_lock_timeout``，默认
        0.1 s）：起飞/降落是在锁内 ``join()`` 的，实测 ``land`` 要 20 s 量级，
        如果这里干等，5 Hz 的 ``/drone/odom`` 会整整停 20 s —— RViz 里表现为
        飞机"冻住"再突然跳到地面。放弃时 ``state_valid`` 置 False，节点就不发布。
        """
        client = client or self.client
        if client is None:
            self.state_valid = False
            return self.last_state
        if not self._api_lock.acquire(timeout=self.state_lock_timeout):
            self.state_valid = False
            return self.last_state
        try:
            try:
                raw = self._call(lambda: client.getMultirotorState(**self._kw()),
                                 "读取状态")
                k = raw.kinematics_estimated
                pos = ned_to_enu(k.position.x_val, k.position.y_val, k.position.z_val)
                vel = ned_to_enu(k.linear_velocity.x_val, k.linear_velocity.y_val,
                                 k.linear_velocity.z_val)
                collision = False
                try:
                    info = self._call(
                        lambda: client.simGetCollisionInfo(**self._kw()), "读取碰撞")
                    collision = bool(getattr(info, "has_collided", False))
                except Exception:
                    collision = False
                try:
                    api = bool(self._call(
                        lambda: client.isApiControlEnabled(**self._kw()),
                        "读取 API 状态"))
                except Exception:
                    api = self.api_control
                self.last_state = DroneState(
                    position_enu=pos, velocity_enu=vel,
                    landed_state=landed_state_name(getattr(raw, "landed_state", 0)),
                    api_control=api, collision=collision, stamp=self._clock())
                self.state_valid = True
            except Exception as exc:
                self.state_valid = False
                self.last_error = str(exc)
                self._log("读取状态失败：%s" % exc)
        finally:
            self._api_lock.release()
        return self.last_state
