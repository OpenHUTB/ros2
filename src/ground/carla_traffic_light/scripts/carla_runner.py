# -*- coding: utf-8 -*-
"""CARLA 集成：把信号灯决策与纵向控制接到仿真里。

流程
----
1. 连接 CARLA，找一盏有停止线的信号灯
2. **把它强制设成红灯并冻结**，这样每次运行都能稳定复现"停车"场景
3. 沿车道往回退一段距离，在那里生成自车，朝向信号灯
4. 每帧：
   * 读自车位置与速度
   * 找前方最近的信号灯，读灯色
   * 用自车到停止线的前向投影距离作为 ``d``
   * 交给 traffic_light_logic.decide 得出动作与目标速度
   * 交给 longitudinal 控制器得出油门/刹车
   * 用纯跟踪法算转向，让车不跑出车道
   * 写回车辆控制
5. 按间隔存截图

几个曾经踩过的坑
----------------
* **不能在停止线上生成自车**。最初直接用 ``stop_waypoints[0].transform`` 生成，
  结果距离一上来就是 0，全程"已越过停止线"，什么也测不到。要沿车道往回退。
* **必须给转向**。只控制油门刹车的话车会直直冲出路口撞墙，最后卡死在那里，
  速度一直是 0。
* **只考虑前方的灯**。车开过去之后如果还盯着身后那盏，距离会一直是个负数。
"""

from __future__ import annotations

import math
import os
import time

from traffic_light_logic import (
    GO,
    LightParams,
    STOP,
    decide,
)
from longitudinal import LongitudinalController

try:
    import carla
    _HAS_CARLA = True
except ImportError:  # pragma: no cover
    carla = None
    _HAS_CARLA = False


def _require_carla():
    if not _HAS_CARLA:
        raise RuntimeError("没有找到 carla 客户端库，请先安装（Python 3.12 + carla 0.9.16）")


# ---------------------------------------------------------------------------
def connect(host="127.0.0.1", port=2000, timeout=10.0):
    """连接 CARLA，返回 (client, world)。"""
    _require_carla()
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    world = client.get_world()
    return client, world


def waypoint_back(wp, distance, step=2.0):
    """沿车道往回退 distance 米，返回退到的路径点。"""
    cur = wp
    acc = 0.0
    guard = 0
    while acc < distance and guard < 2000:
        guard += 1
        prev = cur.previous(step)
        if not prev:
            break
        nxt = prev[0]
        acc += nxt.transform.location.distance(cur.transform.location)
        cur = nxt
    return cur


def signed_forward_distance(ego_transform, target_location):
    """目标点在自车坐标系下的「前方距离」，负值表示在车后。"""
    fwd = ego_transform.get_forward_vector()
    loc = ego_transform.location
    dx = target_location.x - loc.x
    dy = target_location.y - loc.y
    return dx * fwd.x + dy * fwd.y


def find_traffic_light_ahead(ego_location, ego_transform, traffic_lights,
                             max_range=120.0):
    """在自车**前方**找最近的信号灯。

    只保留前向投影距离为正的候选，避免车开过去之后还盯着身后那盏。
    返回 (traffic_light, stop_location, forward_distance) 或 None。
    """
    best = None
    for tl in traffic_lights:
        try:
            stop_wps = tl.get_stop_waypoints()
        except Exception:
            continue
        if not stop_wps:
            continue
        for wp in stop_wps:
            loc = wp.transform.location
            fwd = signed_forward_distance(ego_transform, loc)
            if fwd <= 0.0:
                continue                    # 在车后，忽略
            if fwd > max_range:
                continue
            if best is None or fwd < best[2]:
                best = (tl, loc, fwd)
    return best


def light_state_string(tl):
    """把 CARLA 的灯色转成逻辑层用的字符串。"""
    try:
        st = tl.get_state()
    except Exception:
        return None
    name = getattr(st, "name", None) or str(st)
    if "Red" in name:
        return "Red"
    if "Yellow" in name:
        return "Yellow"
    if "Green" in name:
        return "Green"
    return "Off"


def lane_steer(ego_transform, map_obj, lookahead=10.0, gain=1.6):
    """纯跟踪式的车道保持转向。

    取前方 lookahead 处的车道中心点，算它在自车坐标系里的方位角，
    按比例转成方向盘量。
    """
    try:
        wp = map_obj.get_waypoint(ego_transform.location)
        ahead = wp.next(lookahead)
        if not ahead:
            return 0.0
        target = ahead[0].transform.location
    except Exception:
        return 0.0

    fwd = ego_transform.get_forward_vector()
    loc = ego_transform.location
    dx = target.x - loc.x
    dy = target.y - loc.y
    cross = fwd.x * dy - fwd.y * dx          # 右正
    dot = fwd.x * dx + fwd.y * dy
    angle = math.atan2(cross, dot)
    steer = gain * angle
    return max(-1.0, min(1.0, steer))


# ---------------------------------------------------------------------------
class TrafficLightRunner(object):
    """CARLA 信号灯响应实验。"""

    def __init__(self, params=None, ctrl=None, host="127.0.0.1", port=2000,
                 max_speed=8.0):
        _require_carla()
        self.params = params if params is not None else LightParams()
        self.ctrl = ctrl if ctrl is not None else LongitudinalController()
        # 巡航限速。停车曲线在离停止线很远时允许很高速度（sqrt(2ad)），
        # 50 m 外能算出 14 m/s，没必要为此猛加速，也容易刹不住。
        self.max_speed = float(max_speed)
        self.client, self.world = connect(host, port)
        self.map = self.world.get_map()
        self.ego = None
        self.camera = None
        self.frames = []
        self.save_dir = None
        self.target_light = None
        # 同步模式相关。异步模式下客户端按自己的节奏发指令，和服务器跑的帧
        # 对不齐，车动起来会一顿一顿；同步模式让服务器只在 tick() 时前进一帧。
        self.sync_mode = False
        self._orig_settings = None

    # -- 同步模式 -------------------------------------------------------
    def set_synchronous(self, dt=0.05):
        """切到同步模式，让服务器只在 world.tick() 时前进。

        异步模式下服务器按自己的帧率跑（30~60 FPS），客户端 20 Hz 发一次控制
        指令，两者对不齐，车看起来就是一顿一顿的。同步模式把物理步长固定成
        dt，客户端调用一次 tick() 就精确前进一帧，动作立刻变平滑。
        """
        try:
            self._orig_settings = self.world.get_settings()
            s = self.world.get_settings()
            s.synchronous_mode = True
            s.fixed_delta_seconds = float(dt)
            self.world.apply_settings(s)
            self.sync_mode = True
            return True
        except Exception as e:
            print("    无法切到同步模式（继续用异步）: %s" % e)
            self.sync_mode = False
            return False

    # -- 信号灯 ---------------------------------------------------------
    def pick_light(self):
        """挑一盏带停止线的信号灯，并强制设为红灯。"""
        lights = list(self.world.get_actors().filter("traffic.traffic_light"))
        if not lights:
            return None
        for tl in lights:
            try:
                if not tl.get_stop_waypoints():
                    continue
            except Exception:
                continue
            # 冻结成红灯，保证每次运行都能复现停车场景
            try:
                tl.set_state(carla.TrafficLightState.Red)
                tl.freeze(True)
            except Exception as e:
                print("    警告：无法设置灯色: %s" % e)
            self.target_light = tl
            return tl
        return None

    # -- 车辆 -----------------------------------------------------------
    def spawn_ego(self, distance_before=50.0):
        """在停止线前 distance_before 米处生成自车，朝向信号灯。"""
        tl = self.pick_light()
        if tl is None:
            raise RuntimeError("世界里没有带停止线的信号灯")

        stop_wp = tl.get_stop_waypoints()[0]
        spawn_wp = waypoint_back(stop_wp, distance_before)

        bp_lib = self.world.get_blueprint_library()
        bp = bp_lib.filter("vehicle.*")[0]

        transform = spawn_wp.transform
        transform.location.z += 0.3

        self.ego = self.world.try_spawn_actor(bp, transform)
        if self.ego is None:
            # 位置被占用，往前挪一点再试
            for shift in (5.0, 10.0, 15.0):
                alt = waypoint_back(stop_wp, distance_before + shift)
                t2 = alt.transform
                t2.location.z += 0.3
                self.ego = self.world.try_spawn_actor(bp, t2)
                if self.ego is not None:
                    transform = t2
                    break
        if self.ego is None:
            raise RuntimeError("生成自车失败：附近没有空闲位置")

        self.ego.set_autopilot(False)
        print("    自车已生成，距停止线约 %.1f m"
              % transform.location.distance(stop_wp.transform.location))
        return self.ego

    def attach_camera(self, save_dir, width=800, height=450):
        """挂一个前视相机，用于存截图。"""
        os.makedirs(save_dir, exist_ok=True)
        self.save_dir = save_dir
        bp_lib = self.world.get_blueprint_library()
        cam_bp = bp_lib.find("sensor.camera.rgb")
        cam_bp.set_attribute("image_size_x", str(width))
        cam_bp.set_attribute("image_size_y", str(height))
        cam_bp.set_attribute("fov", "90")

        spawn = carla.Transform(carla.Location(x=2.4, z=1.5))
        self.camera = self.world.spawn_actor(cam_bp, spawn, attach_to=self.ego)
        self.camera.listen(self._on_image)
        return self.camera

    def _on_image(self, image):
        self.frames.append(image)

    # -- 主循环 ---------------------------------------------------------
    def run(self, duration=30.0, dt=0.05, save_every=10, verbose=True,
            max_saved=40):
        """跑一段时间，返回统计。"""
        # 先切同步模式，否则车一卡一卡
        self.set_synchronous(dt)

        all_lights = list(self.world.get_actors().filter("traffic.traffic_light"))
        # 只盯着生成时选定的那盏灯。之前用 all_lights 会导致"用车旁的 A 灯
        # 生成，行驶时却盯着更近的 B 灯"，B 灯是绿的所以根本不停车。
        if self.target_light is not None:
            lights = [self.target_light]
        else:
            lights = all_lights
        if verbose:
            print("    世界里信号灯: %d 个（本车只看目标灯）" % len(all_lights))
            if self.target_light is not None:
                try:
                    raw = self.target_light.get_state()
                except Exception as e:
                    raw = "读取失败: %s" % e
                print("    目标灯 id=%d，已强制设为红灯，当前 get_state() = %s"
                      % (self.target_light.id, raw))
            print()
            print("  %6s %8s %8s %8s %7s %6s  %s" % (
                "t(s)", "v(m/s)", "距离(m)", "目标v", "油门", "刹车", "动作"))
            print("  " + "-" * 72)

        t0 = time.time()
        step = 0
        n_saved = 0
        last_print = -1.0
        min_gap = float("inf")
        stats = {"steps": 0, "stopped": 0, "go": 0, "slow": 0,
                 "min_gap": -1.0, "final_speed": 0.0, "final_state": None,
                 "saved_images": 0}

        try:
            while time.time() - t0 < duration:
                ego_tf = self.ego.get_transform()
                vel = self.ego.get_velocity()
                speed = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

                found = find_traffic_light_ahead(
                    ego_tf.location, ego_tf, lights)
                if found is None:
                    state, dist = None, 0.0
                else:
                    tl, stop_loc, dist = found
                    state = light_state_string(tl)

                if state is None:
                    action, v_target, note = GO, 20.0, "前方无信号灯"
                else:
                    action, v_target, note = decide(
                        state, dist, speed, self.params)
                if v_target is None:
                    v_target = self.max_speed
                v_target = min(v_target, self.max_speed)

                a_ff = 0.0 if action == GO else -self.params.comfort_decel
                throttle, brake = self.ctrl.update(
                    speed, v_target, dt, a_feedforward=a_ff)

                steer = lane_steer(ego_tf, self.map)

                ctl = carla.VehicleControl()
                ctl.throttle = float(throttle)
                ctl.brake = float(brake)
                ctl.steer = float(steer)
                self.ego.apply_control(ctl)
                self._spectator_follow()

                if state is not None:
                    min_gap = min(min_gap, dist)
                stats["final_speed"] = speed
                stats["final_state"] = state
                if action == STOP:
                    stats["stopped"] += 1
                elif action == "SLOW":
                    stats["slow"] += 1
                else:
                    stats["go"] += 1
                stats["steps"] += 1

                el = time.time() - t0
                if verbose and el - last_print >= 1.0:
                    gap_txt = ("%.1f" % dist) if state else "—"
                    print("  %6.2f %8.3f %8s %8.3f %7.3f %6.3f  %-4s %s" % (
                        el, speed, gap_txt, v_target, throttle, brake,
                        action, note))
                    last_print = el

                if (self.camera is not None and n_saved < max_saved
                        and step % save_every == 0):
                    n_saved = self._save_latest(n_saved)

                step += 1
                if self.sync_mode:
                    # 同步模式：tick 一次精确前进 fixed_delta_seconds
                    self.world.tick()
                else:
                    self.world.wait_for_tick()
                    time.sleep(max(0.0, dt - 0.02))
        except KeyboardInterrupt:
            print("    用户中断")
        finally:
            self.cleanup()

        stats["min_gap"] = -1.0 if min_gap == float("inf") else min_gap
        stats["saved_images"] = n_saved
        return stats

    def _spectator_follow(self, back=9.0, up=4.0, pitch=-18.0):
        """让 CARLA 窗口的旁观者视角跟住自车。

        不加这个的话，CarlaUE4 窗口停在世界的某个固定位置，车开远了就看不到。
        这里把旁观者放到车后方 ``back`` 米、上方 ``up`` 米，朝向与车一致。
        """
        if self.ego is None:
            return
        try:
            spec = self.world.get_spectator()
            tf = self.ego.get_transform()
            fwd = tf.get_forward_vector()
            loc = tf.location - fwd * back
            loc.z += up
            spec.set_transform(carla.Transform(
                loc, carla.Rotation(pitch=pitch, yaw=tf.rotation.yaw)))
        except Exception:
            pass

    def _save_latest(self, n_saved):
        """保存最新一帧。"""
        if not self.frames or self.save_dir is None:
            return n_saved
        img = self.frames[-1]
        path = os.path.join(self.save_dir, "frame_%04d.png" % n_saved)
        img.save_to_disk(path)
        return n_saved + 1

    # -- 清理 -----------------------------------------------------------
    def cleanup(self):
        for a in (self.camera, self.ego):
            try:
                if a is not None:
                    a.destroy()
            except Exception:
                pass
        self.camera = None
        self.ego = None
        # 恢复原本的世界设置（把同步模式关掉，否则 CARLA 窗口会冻住）
        if self._orig_settings is not None:
            try:
                self.world.apply_settings(self._orig_settings)
            except Exception:
                pass
            self._orig_settings = None
            self.sync_mode = False
        # 恢复灯的正常循环
        if self.target_light is not None:
            try:
                self.target_light.freeze(False)
            except Exception:
                pass
