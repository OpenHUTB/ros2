# -*- coding: utf-8 -*-
"""无人机激光雷达分扇区避障 + 梯形速度巡航。

对应报告：docs/air/uav_lidar_cruise.md

实测要点（OpenHUTB v2.10.0 / Town10HD / airsim.drone）：
  1. 无人机控制接口是 set_target_velocity / set_target_angular_velocity，
     没有 set_velocity。
  2. 点云为 LidarMeasurement.raw_data，每点 4 个 float32(x, y, z, intensity)，
     元素没有 .distance 属性。
  3. 该机型雷达机头轴为 -X，方位角必须用 atan2(y, -x)。
  4. 机身自反射集中在 1.25~3.0 m（实测 53.3% 的点），必须过滤。

运行：python main.py（需先启动 OpenHUTB 仿真）
"""
import math
import os
import struct
import sys
import time

# carla/hutb 模块由 OpenHUTB 的 PythonAPI 提供；若未安装则尝试从常见路径导入
try:
    import carla
except ImportError:  # pragma: no cover
    _cand = [
        os.path.join(os.environ.get("OPENHUTB_ROOT", r"C:\hutb_windows_v2.10.0"),
                     "PythonAPI", "carla", "dist"),
        os.path.join(os.environ.get("CARLA_ROOT", r"C:\carla"), "PythonAPI", "carla", "dist"),
    ]
    for _p in _cand:
        if os.path.isdir(_p):
            for _f in os.listdir(_p):
                if _f.endswith(".whl"):
                    sys.path.insert(0, _p)
                    break
    import carla

# ---------- 参数区 ----------
V_MAX = 2.0          # 巡航速度 m/s
V_TURN = 0.6         # 避障转向时的前进速度 m/s
A_MAX = 1.5          # 最大加速度 m/s^2
DT = 0.1             # 控制周期 s

D_SLOW = 3.5         # 触发减速的前方距离 m
D_STOP = 1.5         # 触发停车的前方距离 m
D_BOXED = 1.2        # 左右均小于该值判定为被围困 m
D_MIN_VALID = 3.2    # 最小有效距离 m（兜底过滤；首选方案是把雷达装高）

YAW_FRONT = 30.0     # 前扇区半角
YAW_SIDE = 120.0     # 侧扇区外边界
YAW_RATE = 0.4       # 转向角速度 rad/s

FAR = 50.0           # 无回波的填充值
LIDAR_RANGE = "50.0"
LIDAR_MOUNT_Z = 2.0  # 实测 2.0m 可消除机身自反射


def parse_lidar(msg):
    """把 LidarMeasurement.raw_data 解析成 (x, y, z) 列表。"""
    raw = msg.raw_data
    n = len(raw) // 16
    vals = struct.unpack("<" + "ffff" * n, raw)
    return [(vals[i], vals[i + 1], vals[i + 2]) for i in range(0, n * 4, 4)]


def sector_distances(points):
    """按机头(-X)划分前/左/右扇区，返回各自最小有效水平距离。"""
    d_front, d_left, d_right = FAR, FAR, FAR
    for x, y, z in points:
        if abs(z) > 3.0:
            continue
        d = math.hypot(x, y)
        if d < D_MIN_VALID or d > FAR:      # 剔除自反射与地面
            continue
        # 实测机头轴为 -X
        yaw = math.degrees(math.atan2(y, -x))

        if abs(yaw) <= YAW_FRONT:
            d_front = min(d_front, d)
        elif YAW_FRONT < yaw <= YAW_SIDE:
            d_left = min(d_left, d)
        elif -YAW_SIDE <= yaw < -YAW_FRONT:
            d_right = min(d_right, d)
    return d_front, d_left, d_right


def ramp(current, target, a_max=A_MAX, dt=DT):
    """梯形速度规划：每帧速度变化不超过 a_max*dt。"""
    dv = a_max * dt
    return min(current + dv, target) if target > current else max(current - dv, target)


class CruiseController:
    """感知—决策—平滑执行三层结构中的后两层。"""

    def __init__(self, trapezoid=True):
        self.v = 0.0
        self.w = 0.0
        self.dists = (FAR, FAR, FAR)
        self.state = "CRUISE"
        self.trapezoid = trapezoid
        self.history = []

    def update(self, dists):
        d_f, d_l, d_r = dists
        self.dists = dists

        if d_f < D_STOP:
            self.state, v_t, w_t = "STOP", 0.0, 0.0
        elif d_f < D_SLOW:
            if max(d_l, d_r) < D_BOXED:
                self.state, v_t, w_t = "BOXED", 0.0, 0.0
            else:
                self.state = "AVOID"
                v_t = V_TURN
                w_t = YAW_RATE if d_l >= d_r else -YAW_RATE
        else:
            self.state, v_t, w_t = "CRUISE", V_MAX, 0.0

        if self.trapezoid:
            self.v = ramp(self.v, v_t)
            self.w = ramp(self.w, w_t, a_max=2.0)
        else:
            self.v, self.w = v_t, w_t        # 阶跃：仅供对照实验

        self.history.append(self.v)
        return self.v, self.w

    def jitter(self):
        """相邻帧速度变化超过 2 倍单帧上限的次数占比。"""
        jumps = sum(1 for a, b in zip(self.history, self.history[1:])
                    if abs(b - a) > 2 * A_MAX * DT)
        return jumps, jumps / max(len(self.history) - 1, 1)


def find_drone(world):
    for actor in world.get_actors():
        if "airsim.drone" in actor.type_id:
            return actor
    return None


def attach_lidar(world, drone):
    bp = world.get_blueprint_library().find("sensor.lidar.ray_cast")
    bp.set_attribute("range", LIDAR_RANGE)
    bp.set_attribute("channels", "32")
    bp.set_attribute("points_per_second", "200000")
    bp.set_attribute("rotation_frequency", "20.0")
    # 实测默认 dropoff_general_rate=0.45 会丢掉 45% 的点
    bp.set_attribute("dropoff_general_rate", "0.0")
    # 实测：z=0.5m 时机身自反射最近到 1.53m；z=2.0m 时自反射消失（最近 4.1m）。
    # 因此把雷达架到机体上方 2.0m，从源头消除自反射，而不是只靠距离过滤兜底。
    return world.spawn_actor(bp, carla.Transform(carla.Location(z=LIDAR_MOUNT_Z)),
                             attach_to=drone)


def main():
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(30.0)
    world = client.get_world()
    print("[OK] 已连接 OpenHUTB：", world.get_map().name)

    drone = find_drone(world)
    if drone is None:
        print("[FAIL] 场景内未找到 airsim.drone")
        return
    print(f"[OK] 无人机 id={drone.id}")

    lidar = attach_lidar(world, drone)
    print(f"[OK] 雷达已挂载 id={lidar.id}")

    state = {"dists": (FAR, FAR, FAR)}

    def on_scan(msg):
        state["dists"] = sector_distances(parse_lidar(msg))

    lidar.listen(on_scan)
    time.sleep(1.0)

    ctrl = CruiseController()
    try:
        while True:
            v, w = ctrl.update(state["dists"])
            fwd = drone.get_transform().get_forward_vector()
            # 注意：不是 set_velocity
            drone.set_target_velocity(carla.Vector3D(fwd.x * v, fwd.y * v, 0.0))
            drone.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, w))

            print(f"[{ctrl.state:6s}] 前 {ctrl.dists[0]:5.2f}m "
                  f"左 {ctrl.dists[1]:5.2f}m 右 {ctrl.dists[2]:5.2f}m "
                  f"-> v={v:4.2f} w={w:+.2f}")
            time.sleep(DT)

    except KeyboardInterrupt:
        print("\n[STOP] 用户中断")
    finally:
        drone.set_target_velocity(carla.Vector3D(0.0, 0.0, 0.0))
        drone.set_target_angular_velocity(carla.Vector3D(0.0, 0.0, 0.0))
        lidar.stop()
        lidar.destroy()

        jumps, rate = ctrl.jitter()
        print(f"[STAT] 帧数={len(ctrl.history)} 跳变={jumps} 抖动率={rate:.1%}")


if __name__ == "__main__":
    main()
