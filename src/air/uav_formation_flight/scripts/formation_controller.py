#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四旋翼编队飞行主节点（Leader-Follower）。

长机按 missions/<任务>/leader 的航点序列飞行；每个僚机的期望位置 =
长机当前位置 + shapes/<队形> 中对应的 ENU 偏移。位置误差经 PID 转成三维速度指令
（moveByVelocityAsync），三架共用同一个 AirSim RPC 连接，用 vehicle_name 区分。

坐标约定：本文件与参数一律使用 ROS ENU（x 向东、y 向北、z 向上），相对起飞点。
与 AirSim NED 世界坐标的换算：
    ned.x = home_ned.x + enu.y
    ned.y = home_ned.y + enu.x
    ned.z = home_ned.z - enu.z

这台 OpenHUTB 构建会让三架载具在**同一个点**生成（settings.json 里的 X/Y/Z 被忽略），
所以起飞流程必须做到：
  1. 起飞前先把三架都落到地面（上一次运行若被中断，飞机可能还悬在空中）；
  2. 每一架起飞后**立刻飞到自己的队形空位，再让下一架起飞**——否则三架会悬停在
     同一坐标上重叠，被物理引擎互相推开，队形永远无法保持。

日志采用边跑边写，即使进程被强杀（roslaunch 的 Ctrl+C 会升级为 SIGKILL）也能留下数据。
"""

import csv
import math
import os
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

import airsim


def enu_to_ned_vec(x, y, z):
    """ROS 的 ENU 向量（位置或速度）-> AirSim 的 NED 向量。

    ENU 是 x 向东、y 向北、z 向上；NED 是 x 向北、y 向东、z 向下，
    因此两个水平分量互换、垂直分量取反。
    """
    return airsim.Vector3r(y, x, -z)


class PID(object):
    """位置误差（米）到速度指令（米/秒）的一维 PID，带积分限幅。"""

    def __init__(self, kp, ki, kd, limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.integral = 0.0
        self.last_err = None

    def reset(self):
        self.integral = 0.0
        self.last_err = None

    def update(self, err, dt):
        self.integral += err * dt
        self.integral = max(-self.limit, min(self.limit, self.integral))
        deriv = 0.0
        if self.last_err is not None and dt > 0:
            deriv = (err - self.last_err) / dt
        self.last_err = err
        return self.kp * err + self.ki * self.integral + self.kd * deriv


class FormationController(object):
    def __init__(self):
        # 下面 get_param 的 fallback 与 config/params.yaml 保持一致：只有不经 launch、
        # 没加载参数文件时才用得到。改参数时两处一起改（launch 里的 kp/ki/kd 另算）。
        self.rate_hz = float(rospy.get_param("/flight/control_rate", 20.0))
        self.takeoff_alt = float(rospy.get_param("/flight/takeoff_altitude", 3.0))
        self.takeoff_interval = float(rospy.get_param("/flight/takeoff_interval", 1.0))
        self.max_speed = float(rospy.get_param("/flight/max_speed", 2.0))
        self.max_vz = float(rospy.get_param("/flight/max_vertical_speed", 1.0))
        self.wp_tol = float(rospy.get_param("/flight/waypoint_tolerance", 0.25))
        self.formation_tol = float(rospy.get_param("/flight/formation_tolerance", 0.30))
        self.wp_timeout = float(rospy.get_param("/flight/waypoint_timeout", 25.0))
        self.settle_time = float(rospy.get_param("/flight/settle_time", 1.0))
        self.deploy_timeout = float(rospy.get_param("/flight/deploy_timeout", 30.0))
        self.max_duration = float(rospy.get_param("/flight/max_duration", 150.0))
        off = rospy.get_param("/flight/deploy_offset", [0.0, 0.0])
        self.deploy_offset = (float(off[0]), float(off[1]))
        clr = rospy.get_param("/flight/leader_clear_offset", [2.5, 0.0])
        self.leader_clear_offset = (float(clr[0]), float(clr[1]))
        self.alt_separation = float(rospy.get_param("/flight/alt_separation", 3.0))
        # 展开阶段用哪种方式就位：position = AirSim 自带位置控制（默认，稳定第一），
        # pid = 我们自己的速度环（用于对比实验）
        self.deploy_mode = str(rospy.get_param("/flight/deploy_mode", "position")).lower()
        # 长机参考点的推进速度（虚拟引导点，米/秒）。航点之间用它匀速推进，
        # 而不是把航点当成阶跃指令直接丢给 PID。
        self.ref_speed = float(rospy.get_param("/flight/leader_ref_speed", 0.7))
        # 高度通道的下发方式：
        #   velocity = moveByVelocityAsync，三维速度都交给我们的 PID（默认，当前配置）；
        #   setpoint = moveByVelocityZAsync，只下发水平速度 + 高度设定值，高度交给 AirSim。
        # 两条路都实飞过：setpoint 在这个构建里不尊重水平速度（指令 0.5 m/s，实测
        # 0.85~1.43 m/s，171%~286%），已弃用；velocity 下 lag_probe.py 的附加判定证明
        # 「边横飞边下降」时垂直分量是执行的（水平 0.43 m/s、高度 -0.53 m/s 都跟随指令），
        # 早期的高度漂移是撞进 5~6 米的树冠/路灯，不是接口问题。
        self.altitude_mode = str(
            rospy.get_param("/flight/altitude_mode", "velocity")).lower()

        # 速度指令的持续时长（秒），取约两个控制周期。
        # 用 lag_probe.py 实测过：0.5 秒与 0.05 秒两种时长下，"指令 -> 实际速度"
        # 的延迟都是 0.65 秒，也就是说延迟不在指令队列里，而在 AirSim 内部的速度
        # 跟踪环节。时长本身保持与控制周期同量级即可（避免排入过期指令）。
        self.cmd_duration = float(rospy.get_param(
            "/flight/cmd_duration", 2.0 / self.rate_hz))
        # 僚机前馈增益：把长机当前速度直接叠加进僚机的速度指令。只靠位置误差
        # 「追」一个移动目标，必然滞后一个回路延迟的时间，队形误差 ≈ 延迟 x 长机速度。
        self.ff_gain = float(rospy.get_param("/flight/leader_velocity_ff", 0.5))
        # 前馈速度的低通时间常数（秒）：长机速度里的抖动不要整幅灌进僚机指令
        self.ff_tau = float(rospy.get_param("/flight/leader_velocity_ff_tau", 0.3))
        self.ff_state = [0.0, 0.0, 0.0]

        self.ip = rospy.get_param("/airsim/ip", "192.168.237.1")
        self.timeout = float(rospy.get_param("/airsim/timeout", 30.0))
        self.leader = rospy.get_param("/airsim/leader", "SimpleFlight")
        self.followers = list(rospy.get_param("/airsim/followers"))

        self.mission = rospy.get_param("/mission", "line_straight")
        self.wps = [tuple(map(float, p))
                    for p in rospy.get_param("/missions/" + self.mission + "/leader")]
        self.schedule = rospy.get_param("/missions/" + self.mission + "/schedule")
        self.shapes = rospy.get_param("/shapes")

        kp = float(rospy.get_param("/pid/kp", 0.4))
        ki = float(rospy.get_param("/pid/ki", 0.02))
        kd = float(rospy.get_param("/pid/kd", 0.05))
        limit = float(rospy.get_param("/pid/integral_limit", 1.0))
        self.gains = (kp, ki, kd)
        self.pids = {}
        for name in [self.leader] + self.followers:
            self.pids[name] = PID(kp, ki, kd, limit)

        self.log_dir = os.path.expanduser(
            rospy.get_param("/log_dir", "~/uav_formation_flight_logs"))
        if not os.path.isdir(self.log_dir):
            os.makedirs(self.log_dir)
        self.tag = "formation_%s_kp%.2f_ki%.2f_kd%.2f" % (
            self.mission, kp, ki, kd)

        self.client = None
        self.home = None
        self.vehicles = [self.leader] + self.followers
        self.path_pubs = {}
        self.paths = {}
        for name in self.vehicles:
            p = Path()
            p.header.frame_id = "world"
            self.paths[name] = p
            self.path_pubs[name] = rospy.Publisher(
                "/formation/%s/path" % name, Path, queue_size=4)

        self.sample_count = 0
        self.metrics = []
        self.deploy_time = float("nan")
        self.leader_err_sq = 0.0
        self.leader_err_n = 0
        self.hold_err = {n: [] for n in self.followers}
        self.switch_records = []
        self.form_time = None
        self.collisions = {}
        self.collision_seen = {}
        self.deploy_failed = []
        self.collision_check_every = int(rospy.get_param(
            "/flight/collision_check_every", 5))
        self.tick = 0

        self.samples_path = os.path.join(self.log_dir, self.tag + ".csv")
        self.fh = None
        self.writer = None

    # ---------------------------------------------------------------- 连接
    def connect(self):
        rospy.loginfo("连接 AirSim %s（超时 %.1f 秒）", self.ip, self.timeout)
        self.client = airsim.MultirotorClient(
            ip=self.ip, timeout_value=self.timeout)
        self.client.confirmConnection()

        # 每次运行前把关卡重置一遍。否则三架是从上一次运行散落的地面位置起飞的
        # （ensure_landed 只是"原地降落"），展开时各自斜穿长机的起飞柱，会出现
        # 「SimpleFlight2 卡住 20 秒不收敛」这类偶发失败。重置后三架回到同一个
        # 生成点，每次运行的初始条件一致。
        if rospy.get_param("/flight/reset_on_start", True):
            try:
                rospy.loginfo("重置模拟器关卡，保证每次运行起点一致")
                self.client.reset()
                time.sleep(2.0)
                self.client.confirmConnection()
            except Exception as exc:                                # noqa: BLE001
                rospy.logwarn("关卡重置失败（继续运行，但起点可能不干净）：%s", exc)

        names = self.client.listVehicles()
        names = getattr(names, "vehicle_names", names)
        names = [str(n) for n in names]
        rospy.loginfo("模拟器载具列表：%s", names)
        missing = [n for n in self.vehicles if n not in names]
        if missing:
            rospy.logerr("以下载具不在模拟器中：%s（检查宿主 settings.json 的 "
                         "Vehicles 段）", missing)
            raise RuntimeError("missing vehicles: %s" % missing)

        # 把关键参数打一遍，便于确认虚拟机上跑的到底是哪一版
        rospy.loginfo("控制参数：kp=%.2f ki=%.2f kd=%.2f | 引导点 %.2f m/s | "
                      "展开方式 %s | 分层 %.1f m | 长机让位 (%.1f, %.1f) | 控制频率 %.0f Hz",
                      self.gains[0], self.gains[1], self.gains[2], self.ref_speed,
                      self.deploy_mode, self.alt_separation,
                      self.leader_clear_offset[0], self.leader_clear_offset[1],
                      self.rate_hz)

        # 上一次运行若被中断，飞机可能还悬在空中；先统统落到地面再记起飞点
        self.ensure_landed()
        time.sleep(1.0)

        state = self.client.getMultirotorState(vehicle_name=self.leader)
        pos = state.kinematics_estimated.position
        self.home = (pos.x_val, pos.y_val, pos.z_val)
        rospy.loginfo("起飞点（AirSim NED 世界坐标）：(%.2f, %.2f, %.2f)",
                      self.home[0], self.home[1], self.home[2])

    def ensure_landed(self):
        """把三架载具都落到地面，保证每次运行都从同一状态开始。"""
        for name in self.vehicles:
            try:
                self.client.enableApiControl(True, vehicle_name=name)
                self.client.armDisarm(True, vehicle_name=name)
                self.client.landAsync(timeout_sec=30, vehicle_name=name).join()
                self.client.armDisarm(False, vehicle_name=name)
                self.client.enableApiControl(False, vehicle_name=name)
            except Exception as exc:                                # noqa: BLE001
                # 刚 reset 过时载具本来就在地面，AirSim 客户端会返回 None 而报错，
                # 属于正常情况，只要它确实在地面就没问题。
                rospy.logwarn("%s 预降落未完成（通常本来就在地面）：%s", name, exc)
            try:
                pos = self.pose_enu(name)
                rospy.loginfo("%s 地面位置 ENU=(%.2f, %.2f)", name, pos[0], pos[1])
            except Exception:                                       # noqa: BLE001
                pass

    def enu_to_ned(self, x, y, z):
        return (self.home[0] + y, self.home[1] + x, self.home[2] - z)

    def pose_vel_enu(self, name):
        """一次 RPC 同时取回位置和速度（ENU，相对起飞点）。"""
        kin = self.client.getMultirotorState(
            vehicle_name=name).kinematics_estimated
        p, v = kin.position, kin.linear_velocity
        return ((p.y_val - self.home[1],          # 东向
                 p.x_val - self.home[0],          # 北向
                 self.home[2] - p.z_val),         # 向上
                (v.y_val, v.x_val, -v.z_val))     # 三维速度

    def pose_enu(self, name):
        """返回该载具相对起飞点的 ENU 位置 (x, y, z)。"""
        return self.pose_vel_enu(name)[0]

    # ---------------------------------------------------------------- 控制
    def send_velocity(self, name, v_enu, z_target=None):
        """把 ENU 三维速度指令（米/秒）发给某架载具。

        altitude_mode=setpoint（默认）时，高度用 moveByVelocityZAsync 以**设定值**
        下发，由 AirSim 自己的高度环维持；只有水平速度走我们的 PID。
        altitude_mode=velocity 时退回三维速度（moveByVelocityAsync），高度也交给 PID。

        时长用 self.cmd_duration（默认 2 个控制周期）。它只是「这条指令的有效期」。
        返回实际下发的 ENU 速度（已限幅），供日志记录。
        """
        v_east = max(-self.max_speed, min(self.max_speed, v_enu[0]))
        v_north = max(-self.max_speed, min(self.max_speed, v_enu[1]))
        v_up = max(-self.max_vz, min(self.max_vz, v_enu[2]))
        if self.altitude_mode == "setpoint" and z_target is not None:
            v_ned = enu_to_ned_vec(v_east, v_north, 0.0)
            self.client.moveByVelocityZAsync(
                v_ned.x_val, v_ned.y_val, self.home[2] - z_target,
                self.cmd_duration,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(False, 0),
                vehicle_name=name)
        else:
            v_ned = enu_to_ned_vec(v_east, v_north, v_up)
            self.client.moveByVelocityAsync(
                v_ned.x_val, v_ned.y_val, v_ned.z_val, self.cmd_duration,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(False, 0),
                vehicle_name=name)
        return (v_east, v_north, v_up)

    def hold_at(self, name, target, timeout=None):
        """用同一套 PID 把某架载具飞到目标 ENU 位置并保持。

        返回 True 表示误差已进入 wp_tol 内。
        """
        timeout = timeout or self.deploy_timeout
        rate = rospy.Rate(self.rate_hz)
        t0 = time.time()
        t_prev = t0
        n_iter = 0
        last_collision_check = 0.0
        last_progress = 0.0
        while not rospy.is_shutdown() and time.time() - t0 < timeout:
            t_now = time.time()
            # PID 的 dt 一律用实测周期。标称值 1/rate 只有在循环真跑满 20 Hz 时才对；
            # 一旦实际跑得更慢，标称 dt 会把微分项放大好几倍 —— 这正是展开阶段疯狂
            # 绕圈（周期约 2 秒、残差 1.5 米）而单机探针（真跑到 20 Hz）稳如磐石的原因。
            dt = min(max(t_now - t_prev, 0.25 / self.rate_hz), 4.0 / self.rate_hz)
            t_prev = t_now
            n_iter += 1
            if self.collision_check_every > 0 and \
                    t_now - last_collision_check > 0.25:
                last_collision_check = t_now
                self.check_collisions(0.0)
            now, vel = self.pose_vel_enu(name)
            err = [target[i] - now[i] for i in range(3)]
            dist = math.sqrt(sum(e * e for e in err))
            if dist < self.wp_tol:
                rospy.loginfo("  %s 已进入 %.2f 米容差（展开用时 %.1f 秒，"
                              "平均控制频率 %.1f Hz）",
                              name, self.wp_tol, t_now - t0, n_iter / max(1e-3, t_now - t0))
                return True
            if t_now - last_progress > 1.0:            # 卡住时能看到它在做什么
                last_progress = t_now
                rospy.loginfo("  %s 展开中 t=%.1f 位置 ENU=(%.2f, %.2f, %.2f) "
                              "目标=(%.2f, %.2f, %.2f) 残差 %.2f 米 速度 %.2f m/s",
                              name, t_now - t0, now[0], now[1], now[2],
                              target[0], target[1], target[2], dist,
                              math.sqrt(sum(v * v for v in vel)))
            v = [self.pids[name].update(err[i], dt) for i in range(3)]
            self.send_velocity(name, v, z_target=target[2])
            rate.sleep()
        rospy.logwarn("  %s 未在 %.0f 秒内收敛（平均控制频率 %.1f Hz）",
                      name, timeout, n_iter / max(1e-3, time.time() - t0))
        return False

    def probe_plant(self, name):
        """原地抽检：状态估计 vs 真值，以及东向 0.5 m/s 常值命令 3 秒的响应。

        单机探针里这个响应是 0.49 / 0.50。若节点里明显偏低，说明命令通道有问题；
        若「估计」和「真值」对不上，说明控制器在追一个假位姿（那类问题会让飞机
        永远收敛不了、高度一直差一截）。
        """
        rate = rospy.Rate(self.rate_hz)
        pos_e, _ = self.pose_vel_enu(name)
        try:
            k = self.client.simGetGroundTruthKinematics(vehicle_name=name)
            p = k.position
            pos_t = (p.y_val - self.home[1], p.x_val - self.home[0],
                     self.home[2] - p.z_val)
            rospy.loginfo("  状态对比 %s：估计=(%.2f, %.2f, %.2f) 真值=(%.2f, %.2f, %.2f)"
                          " 差 %.2f 米", name, pos_e[0], pos_e[1], pos_e[2],
                          pos_t[0], pos_t[1], pos_t[2], math.dist(pos_e, pos_t))
        except Exception as exc:                                    # noqa: BLE001
            rospy.logwarn("  真值状态读取失败：%s", exc)

        # 东向推一段、再反向推同样长一段：净位移接近 0，不会在展开前把飞机
        # 推出好几米（上一轮就是这么把 SimpleFlight2/3 顶偏、展开收不住的）。
        p0 = self.pose_enu(name)
        t0 = time.time()
        while time.time() - t0 < 2.0:
            self.send_velocity(name, (0.5, 0.0, 0.0))
            rate.sleep()
        t_pulse = time.time()
        p1 = self.pose_enu(name)
        t1 = time.time()
        while time.time() - t1 < 2.0:
            self.send_velocity(name, (-0.5, 0.0, 0.0))
            rate.sleep()
        for _ in range(6):
            self.send_velocity(name, (0.0, 0.0, 0.0))
            rate.sleep()
        p2 = self.pose_enu(name)
        dt = max(1e-3, t_pulse - t0)
        v = (p1[0] - p0[0]) / dt
        rospy.loginfo("  执行机构抽检 %s：东向 0.5 m/s 跑 %.1f 秒 -> 实测 %.2f m/s"
                      "（%.0f%%）；反向回来后净位移 %+.2f 米",
                      name, dt, v, v / 0.5 * 100.0, p2[0] - p0[0])
        return v

    def move_to_slot(self, name, slot, timeout=None):
        """用 AirSim 自带的位置控制把飞机挪到队形空位（展开阶段的默认方式）。

        展开只是布场，稳定第一：实测我们自己的速度环在 SimpleFlight2 上会绕圈
        不收敛（同样增益在另外两架上很好），所以把展开和队形控制解耦 ——
        队形任务仍然由我们自己的 PID + 长机速度前馈完成。
        """
        timeout = timeout or self.deploy_timeout
        ned = self.enu_to_ned(slot[0], slot[1], slot[2])
        try:
            self.client.moveToPositionAsync(
                ned[0], ned[1], ned[2], self.max_speed, timeout_sec=timeout,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                yaw_mode=airsim.YawMode(False, 0), vehicle_name=name).join()
        except Exception as exc:                                    # noqa: BLE001
            rospy.logwarn("  %s 位置控制展开超时或出错：%s", name, exc)
        pos = self.pose_enu(name)
        dist = math.dist(pos, slot)
        ok = dist < 0.5
        rospy.loginfo("  %s 展开结束：位置 ENU=(%.2f, %.2f, %.2f) 残差 %.2f 米%s",
                      name, pos[0], pos[1], pos[2], dist, "" if ok else "（未收敛）")
        return ok

    # ---------------------------------------------------------------- 起降
    def takeoff_all(self):
        """依次起飞、逐架让出起飞柱，再到自己的队形空位就位。

        两个实测约束（都来自日志和探针）：
        1) 三架在同一个点生成，起飞柱只有一根 —— 所以必须「上一架让开之后，
           下一架才起飞」，不能让第二架穿过还停在柱子里的第一架；
        2) 横向转移必须走低空。在 5-6 米高度朝 ±y（人行道/行道树那一侧）横移，
           SimpleFlight2 会绕圈 30 秒都进不了空位；而同一动作在 3-4 米高度连做
           两次都是一次成功。所以僚机一律在队形高度平移，长机则爬高并沿马路
           方向（ENU x）让出柱子 —— 沿马路高飞不经过行道树。
        """
        slots = self.offsets(self.shape_at(0))
        n = len(self.vehicles)
        t0 = time.time()
        for i, name in enumerate(self.vehicles):
            if i:
                time.sleep(self.takeoff_interval)
            self.client.enableApiControl(True, vehicle_name=name)
            self.client.armDisarm(True, vehicle_name=name)
            self.client.takeoffAsync(timeout_sec=20, vehicle_name=name).join()

            if i == 0:
                # 长机：爬到队形高度之上，再沿马路方向让出起飞柱
                level = self.takeoff_alt + (n - 1) * self.alt_separation
                base = (self.leader_clear_offset[0], self.leader_clear_offset[1])
            else:
                # 僚机：就在队形高度上平移就位（低空转移）
                level = self.takeoff_alt
                base = (slots[i - 1][0], slots[i - 1][1])
            z = self.home[2] - level
            self.client.moveToZAsync(z, self.max_vz, timeout_sec=25,
                                     vehicle_name=name).join()
            rospy.loginfo("%s 已起飞并稳定在 %.1f 米（相对起飞点）",
                          name, level)

            # 起飞刚结束时当场量一次被控对象（命令 -> 实际速度），
            # 用来区分「命令通道有问题」和「回路参数不对」
            self.probe_plant(name)

            # 注意：队形偏移里的 z 是「相对长机的高度差」，不是绝对高度，
            # 展开阶段必须用 level 作为高度，否则会把飞机往地面压。
            slot = (self.deploy_offset[0] + base[0],
                    self.deploy_offset[1] + base[1],
                    level)
            if self.deploy_mode == "pid":
                ok = self.hold_at(name, slot)
            else:
                ok = self.move_to_slot(name, slot)
            rospy.loginfo("%s 已就位（偏移 %.1f, %.1f）%s", name,
                          slot[0], slot[1], "" if ok else "（未完全收敛）")
            if not ok:
                self.deploy_failed.append(name)
            self.pids[name].reset()
        # 僚机都就位后，长机再回到第一个航点上。这样进入任务循环时三架都站在
        # 各自的目标位置上（开局零误差），长机的引导点也就不需要任何跳变。
        if not self.deploy_failed:
            self.move_to_slot(self.leader, tuple(self.wps[0]))
        self.deploy_time = time.time() - t0
        if self.deploy_failed:
            rospy.logerr("展开未完成：%s 没能在 %.0f 秒内进入队形空位。"
                         "长机的第一个航点在起飞柱正上方，继续飞必然撞上它，"
                         "所以本次任务在这里中止。",
                         "、".join(self.deploy_failed), self.deploy_timeout)
            raise RuntimeError("deployment incomplete: %s" % self.deploy_failed)
        rospy.loginfo("编队成形，用时 %.2f 秒", self.deploy_time)

    def check_column_clear(self):
        """确认长机所在的那根竖直通道里没有别的载具。

        长机的航点都在起飞点的竖直线上，下降时若别的载具还停在柱子里就会被撞到
        （物理引擎把两架同时弹开，表现为乱摆 + 航点全部超时）。
        """
        lp = self.pose_enu(self.leader)
        for name in self.followers:
            p = self.pose_enu(name)
            d = math.hypot(p[0] - lp[0], p[1] - lp[1])
            if d < 1.5:
                rospy.logerr("长机正下方 %.2f 米处仍是 %s，下降会相撞，任务中止。",
                             d, name)
                raise RuntimeError("column not clear: %s at %.2f m" % (name, d))

    def land_all(self):
        for name in reversed(self.vehicles):
            try:
                self.client.landAsync(timeout_sec=30, vehicle_name=name).join()
                self.client.armDisarm(False, vehicle_name=name)
                self.client.enableApiControl(False, vehicle_name=name)
                rospy.loginfo("%s 已降落", name)
            except Exception as exc:                                # noqa: BLE001
                rospy.logwarn("%s 降落失败：%s", name, exc)

    # ------------------------------------------------------------ 队形定义
    def shape_at(self, wp_index):
        """返回长机处于第 wp_index 个航点时应使用的队形名。"""
        name = self.schedule[0][0]
        for shape, idx in self.schedule:
            if wp_index >= int(idx):
                name = shape
        return name

    def offsets(self, shape):
        return [tuple(map(float, o)) for o in self.shapes[shape]]

    # ---------------------------------------------------------------- 记录
    def open_log(self):
        self.fh = open(self.samples_path, "w", newline="")
        self.writer = csv.writer(self.fh)
        self.writer.writerow(["t", "shape", "vehicle",
                              "x_ref", "y_ref", "z_ref",
                              "x", "y", "z", "ex", "ey", "ez", "dist",
                              "u_x", "u_y", "u_z",
                              "pid_x", "pid_y", "pid_z", "ff_x", "ff_y", "ff_z"])

    def log_sample(self, t, shape, name, ref, now, err, u=None, dbg=None):
        """记一行样本。

        u    = 这一拍实际下发的速度指令（已限幅）
        dbg  = (pid_x, pid_y, pid_z, ff_x, ff_y, ff_z)：限幅之前的 PID 输出与前馈项。
               分开记录才能在"误差很小、指令却满幅"这类现象里看出是哪一项在推。
        """
        dist = math.sqrt(sum(e * e for e in err))
        self.writer.writerow(
            ["%.3f" % t, shape, name] +
            ["%.4f" % v for v in list(ref) + list(now) + list(err)] +
            ["%.4f" % dist] +
            (["%.4f" % v for v in u] if u else ["", "", ""]) +
            (["%.4f" % v for v in dbg] if dbg else ["", "", "", "", "", ""]))
        self.sample_count += 1
        if self.sample_count % 5 == 0:
            self.fh.flush()          # 被强杀也能留下数据
        return dist

    def check_collisions(self, t):
        """定期查询 AirSim 的碰撞信息，只在碰撞状态变化时报警。

        simGetCollisionInfo 会给出撞到的对象名（静态物体会给 mesh 名，撞到别的
        载具会给载具名）和穿透深度，用来区分「撞树/撞建筑」和「机间相撞」。
        """
        for name in self.vehicles:
            try:
                info = self.client.simGetCollisionInfo(vehicle_name=name)
            except Exception:                                       # noqa: BLE001
                continue
            if not info.has_collided:
                continue
            key = str(info.object_name)
            # 噪声过滤：停机时和路面/地面 mesh 的持续接触（穿透恒为 0），以及碰撞
            # 标志会保持导致"飞机已经起来了还报着地面碰撞"。树冠、路灯这类擦碰的
            # 穿透同样常常是 0，所以不按穿透过滤，只按对象名过滤。
            if info.penetration_depth < 1e-3 and key.startswith(
                    ("Road_", "Ground", "Landscape")):
                continue
            if self.collision_seen.get(name) == key:
                continue
            self.collision_seen[name] = key
            self.collisions[name] = self.collisions.get(name, 0) + 1
            pos = self.pose_enu(name)
            rospy.logwarn("碰撞  t=%.1f  %s 撞到 %s（穿透 %.3f 米）位置 ENU="
                          "(%.2f, %.2f, %.2f)", t, name, key,
                          info.penetration_depth, pos[0], pos[1], pos[2])

    def publish_path(self, name, enu):
        ned = self.enu_to_ned(enu[0], enu[1], enu[2])
        ps = PoseStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "world"
        ps.pose.position.x = ned[0]
        ps.pose.position.y = ned[1]
        ps.pose.position.z = ned[2]
        ps.pose.orientation.w = 1.0
        self.paths[name].header.stamp = ps.header.stamp
        self.paths[name].poses.append(ps)
        self.path_pubs[name].publish(self.paths[name])

    # ---------------------------------------------------------------- 任务
    def run_mission(self):
        rate = rospy.Rate(self.rate_hz)
        self.open_log()
        self.check_column_clear()

        wp_index = 0
        t = 0.0
        t_start = time.time()
        t_prev = t_start
        n_iter = 0
        # 虚拟引导点：以 leader_ref_speed 沿航点折线匀速推进，而不是把航点当阶跃
        # 指令丢给 PID。0.65 秒执行延迟 + kp=1.0 的阶跃响应必然过冲成极限环
        # （表现为来回摆、高度被横向加速度顶高约 1 米、撞路灯和互相碰撞）。
        # 关键：引导点从长机**当前实际位置**起步。上一轮把它摆在第一个航点上，
        # 开场就等于 2.5 米的阶跃，长机照样冲出去过冲，6.5 秒后撞上 SimpleFlight2。
        carrot = list(self.pose_enu(self.leader))
        wp_enter_t = 0.0
        settle_since = None
        form_since = None
        shape = self.shape_at(0)
        shape_since = None

        try:
            while not rospy.is_shutdown() and t < self.max_duration:
                now_wall = time.time()
                # 同 hold_at：PID 用实测周期，日志时间用真实经过时间
                dt = min(max(now_wall - t_prev, 0.25 / self.rate_hz),
                         4.0 / self.rate_hz)
                t_prev = now_wall
                n_iter += 1
                t = now_wall - t_start
                # 引导点以限速朝当前航点推进；到了就停下等长机跟上
                goal = self.wps[min(wp_index, len(self.wps) - 1)]
                step_len = self.ref_speed * dt
                to_goal = [goal[k] - carrot[k] for k in range(3)]
                dist_c = math.sqrt(sum(v * v for v in to_goal))
                if dist_c <= step_len or dist_c < 1e-6:
                    carrot = list(goal)
                    carrot_at_goal = True
                    carrot_vel = (0.0, 0.0, 0.0)
                else:
                    unit = [to_goal[k] / dist_c for k in range(3)]
                    carrot = [carrot[k] + unit[k] * step_len for k in range(3)]
                    carrot_at_goal = False
                    carrot_vel = tuple(v * self.ref_speed for v in unit)
                ref = tuple(carrot)
                leader_enu, leader_vel = self.pose_vel_enu(self.leader)
                err = (ref[0] - leader_enu[0], ref[1] - leader_enu[1],
                       ref[2] - leader_enu[2])
                dist = math.sqrt(sum(e * e for e in err))
                self.leader_err_sq += dist * dist
                self.leader_err_n += 1

                # 引导点停在航点上、长机也跟上了，才算到达这个航点
                if dist < self.wp_tol and carrot_at_goal:
                    if settle_since is None:
                        settle_since = t
                    elif t - settle_since >= self.settle_time:
                        if wp_index < len(self.wps) - 1:
                            rospy.loginfo("已到达航点 %d（t=%.1f 秒，长机 ENU="
                                          "(%.2f, %.2f, %.2f)）",
                                          wp_index + 1, t, leader_enu[0],
                                          leader_enu[1], leader_enu[2])
                            wp_index += 1
                            wp_enter_t = t
                            settle_since = None
                            self.pids[self.leader].reset()
                        else:
                            rospy.loginfo("全部航点完成（t=%.1f 秒）", t)
                            break
                else:
                    settle_since = None
                    if t - wp_enter_t > self.wp_timeout and wp_index < len(self.wps) - 1:
                        rospy.logwarn("航点 %d 超时，跳过", wp_index + 1)
                        wp_index += 1
                        wp_enter_t = t

                new_shape = self.shape_at(wp_index)
                if new_shape != shape:
                    shape = new_shape
                    self.switch_records.append([shape, t, None])
                    rospy.loginfo("队形切换：%s", shape)
                offs = self.offsets(shape)

                # 长机也走"前馈 + 反馈"：前馈是引导点速度，反馈是 PID 修正。
                # 只靠反馈的话，0.7 m/s 参考配 0.65 秒延迟会有约 0.45 米稳态拖尾。
                pid_lead = [self.pids[self.leader].update(err[i], dt) for i in range(3)]
                v_lead = [carrot_vel[i] + pid_lead[i] for i in range(3)]
                u_lead = self.send_velocity(self.leader, v_lead, z_target=ref[2])
                self.publish_path(self.leader, leader_enu)
                self.log_sample(t, shape, self.leader, ref, leader_enu, err, u_lead,
                                tuple(pid_lead) + tuple(carrot_vel))

                if leader_enu[2] < 1.0:
                    rospy.logerr("长机高度只剩 %.2f 米，中止任务", leader_enu[2])
                    break

                errors = {}
                abort = False
                for i, name in enumerate(self.followers):
                    target = (leader_enu[0] + offs[i][0],
                              leader_enu[1] + offs[i][1],
                              leader_enu[2] + offs[i][2])
                    now = self.pose_enu(name)
                    ferr = (target[0] - now[0], target[1] - now[1],
                            target[2] - now[2])
                    # 长机速度前馈 + 位置误差 PID：前馈抵消「跟住移动目标」的固有滞后，
                    # PID 只负责修正残余偏差。前馈先过 0.3 秒低通，避免长机的速度抖动
                    # 整幅灌进僚机指令（ff_gain 设 0 即退回纯反馈）。
                    alpha = min(1.0, dt / max(self.ff_tau, 1e-3))
                    for j in range(3):
                        self.ff_state[j] += (leader_vel[j] - self.ff_state[j]) * alpha
                    ff = [self.ff_gain * self.ff_state[j] for j in range(3)]
                    pid_f = [self.pids[name].update(ferr[j], dt) for j in range(3)]
                    v = [ff[j] + pid_f[j] for j in range(3)]
                    u = self.send_velocity(name, v, z_target=target[2])
                    errors[name] = self.log_sample(t, shape, name, target, now, ferr, u,
                                                   tuple(pid_f) + tuple(ff))
                    self.publish_path(name, now)
                    if now[2] < 1.0:
                        rospy.logerr("%s 高度只剩 %.2f 米，中止任务", name, now[2])
                        abort = True
                        break
                if abort:
                    break

                ready = all(v < self.formation_tol for v in errors.values())
                if self.form_time is None:
                    if ready:
                        if form_since is None:
                            form_since = t
                        elif t - form_since >= self.settle_time:
                            self.form_time = form_since
                            rospy.loginfo("编队保持稳定（%.2f 秒）", self.form_time)
                    else:
                        form_since = None
                elif ready and self.switch_records and \
                        self.switch_records[-1][2] is None:
                    if shape_since is None:
                        shape_since = t
                    elif t - shape_since >= self.settle_time:
                        self.switch_records[-1][2] = shape_since - self.switch_records[-1][1]
                        rospy.loginfo("%s 队形保持到位，过渡用时 %.2f 秒",
                                      shape, self.switch_records[-1][2])
                        shape_since = None
                elif not ready:
                    shape_since = None

                for name in self.followers:
                    self.hold_err[name].append(errors[name])

                self.tick += 1
                if self.collision_check_every > 0 and self.tick % self.collision_check_every == 0:
                    self.check_collisions(t)

                rate.sleep()
        finally:
            if self.fh:
                self.fh.flush()
                self.fh.close()
            self.finalize_metrics()
            elapsed = time.time() - t_start
            if n_iter > 0 and elapsed > 0:
                self.loop_rate = n_iter / elapsed
                rospy.loginfo("任务循环平均频率 %.1f Hz（%d 拍 / %.1f 秒）",
                              self.loop_rate, n_iter, elapsed)
                self.metrics.append(("loop_rate", "", self.loop_rate, "Hz"))

    # ---------------------------------------------------------------- 输出
    def finalize_metrics(self):
        if self.metrics:
            return
        self.metrics.append(("leader_track_rms", "",
                             math.sqrt(self.leader_err_sq / max(1, self.leader_err_n)),
                             "m"))
        for name in self.followers:
            vals = self.hold_err.get(name) or []
            if vals:
                self.metrics.append((
                    "formation_rms", name,
                    math.sqrt(sum(v * v for v in vals) / len(vals)), "m"))
                self.metrics.append(("formation_max", name, max(vals), "m"))
            else:
                self.metrics.append(("formation_rms", name, float("nan"), "m"))
        self.metrics.append(("formation_setup_time", "", self.deploy_time, "s"))
        for name in self.vehicles:
            self.metrics.append(("collision_count", name,
                                 self.collisions.get(name, 0), ""))
        for shape_name, t0, cost in self.switch_records:
            self.metrics.append(("switch_time", shape_name,
                                 cost if cost is not None else float("nan"), "s"))

    def write_metrics(self):
        mpath = os.path.join(self.log_dir, self.tag + "_metrics.csv")
        with open(mpath, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["metric", "vehicle", "value", "unit"])
            for metric, vehicle, value, unit in self.metrics:
                w.writerow([metric, vehicle, "%.4f" % value, unit])
        rospy.loginfo("采样数据：%s（%d 行）", self.samples_path, self.sample_count)
        rospy.loginfo("指标数据：%s", mpath)
        return mpath

    def report(self):
        print("=" * 62)
        print("编队飞行结果")
        print("=" * 62)
        print("任务：%s    PID 增益：kp=%.2f ki=%.2f kd=%.2f"
              % (self.mission, self.gains[0], self.gains[1], self.gains[2]))
        print("长机：%s    僚机：%s" % (self.leader, ", ".join(self.followers)))
        for metric, vehicle, value, unit in self.metrics:
            label = metric + ("[%s]" % vehicle if vehicle else "")
            print("  %-28s %8.4f %s" % (label, value, unit))
        print("=" * 62)


def main():
    rospy.init_node("formation_controller", anonymous=False)
    ctrl = FormationController()
    try:
        ctrl.connect()
        ctrl.takeoff_all()
        ctrl.run_mission()
    except Exception as exc:                                    # noqa: BLE001
        rospy.logerr("运行出错：%s", exc)
        try:
            ctrl.finalize_metrics()
        except Exception:                                       # noqa: BLE001
            pass
    finally:
        try:
            if ctrl.client is not None:
                ctrl.land_all()
        except Exception as exc:                                # noqa: BLE001
            rospy.logwarn("降落阶段出错：%s", exc)
        if ctrl.sample_count:
            try:
                ctrl.finalize_metrics()
                ctrl.write_metrics()
                ctrl.report()
            except Exception as exc:                            # noqa: BLE001
                rospy.logwarn("写日志出错：%s", exc)


if __name__ == "__main__":
    main()
