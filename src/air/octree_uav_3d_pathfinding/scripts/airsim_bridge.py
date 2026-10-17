#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AirSim → ROS 桥接节点

把宿主 Windows 上跑的 AirSim（OpenHUTB，RPC 41451）接进客户机的 ROS：
轮询多旋翼的状态与激光雷达，转成标准 ROS 消息发布出去。

    AirSim(宿主)  ──RPC 41451──►  本节点  ──►  /cloud_in        sensor_msgs/PointCloud2
                                          ──►  /ground_truth/odom nav_msgs/Odometry
                                          ──►  TF world → base_link → lidar_link
                                          ──►  /uav/status       std_msgs/String
                              /uav/goal  ◄──  geometry_msgs/Point（世界系目标点）

订阅：
    /uav/goal    geometry_msgs/Point   目标点（ROS 世界系 ENU，单位 m）
发布：
    /cloud_in                sensor_msgs/PointCloud2   激光雷达点云（lidar_link 系）
    /ground_truth/odom       nav_msgs/Odometry         位姿与速度（world → base_link）
    /uav/status              std_msgs/String           运行状态
    /tf                      tf2_msgs/TFMessage        world→base_link（动态）、base_link→lidar_link（静态）

运行：
    roslaunch octree_uav_3d_pathfinding main.launch
    或单独跑：rosrun octree_uav_3d_pathfinding airsim_bridge.py _host:=192.168.198.1

坐标系（本节点最容易出错的地方，单独说明）：
    AirSim 用 NED（x 北、y 东、z 地），而且机体系是 FRD（前-右-下）；
    ROS 用 ENU（x 东、y 北、z 天），机体系是 FLU（前-左-上）。两处都要转：

      世界系：  (e, n, u) = ( y_ned,  x_ned, -z_ned )
      机体系：  (x, y, z)_flu = ( x_frd, -y_frd, -z_frd )      ← 点云逐点做这个

    姿态四元数不能只做轴交换，要按基变换复合：
      q_ros = q_Cw ⊗ q_ned ⊗ q_Dx
      其中 q_Cw 是世界 NED→ENU 的基变换（绕 (1,1,0)/√2 转 180°），
           q_Dx 是机体系 FRD→FLU 的基变换（绕 x 轴转 180°）。
"""

import math
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String

try:
    import airsim
except ImportError:
    raise SystemExit('缺少 airsim 客户端：pip3 install numpy msgpack-rpc-python airsim')


SQRT2_2 = math.sqrt(2.0) / 2.0

# 世界 NED → ENU 的基变换，用四元数表示：绕 (1,1,0)/√2 转 180°
Q_WORLD = (0.0, SQRT2_2, SQRT2_2, 0.0)   # (w, x, y, z)
# 机体系 FRD → FLU 的基变换：绕 x 轴转 180°
Q_BODY = (0.0, 1.0, 0.0, 0.0)


def quat_mul(a, b):
    """四元数乘法 a ⊗ b，参数与返回都是 (w, x, y, z)。"""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def quat_normalize(q):
    n = math.sqrt(sum(v * v for v in q))
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(v / n for v in q)


class AirSimBridge(object):
    """把 AirSim 的位姿与激光雷达转成 ROS 话题。"""

    def __init__(self):
        # ---------------- 连接参数 ----------------
        self.host = rospy.get_param('~host', '192.168.198.1')
        self.port = rospy.get_param('~port', 41451)
        self.vehicle = rospy.get_param('~vehicle_name', 'Drone1')
        self.lidar = rospy.get_param('~lidar_name', 'LidarSensor1')
        self.connect_retries = rospy.get_param('~connect_retries', 5)

        # ---------------- 坐标系 / 帧名 ----------------
        self.world_frame = rospy.get_param('~world_frame', 'world')
        self.body_frame = rospy.get_param('~body_frame', 'base_link')
        self.lidar_frame = rospy.get_param('~lidar_frame', 'lidar_link')
        # 雷达在机体系（NED FRD）下的安装位置，需与 settings.json 一致
        self.mount = (rospy.get_param('~lidar_x', 0.0),
                      rospy.get_param('~lidar_y', 0.0),
                      rospy.get_param('~lidar_z', -1.0))
        # 世界系 z 偏移：AirSim 的 NED 原点未必在地面（本机 Town10HD 上原点
        # 比载具高约 25 m），加一个常量偏移把地图挪到 RViz 网格附近，便于观察。
        # 纯平移，不影响寻路结果。
        self.world_z_offset = rospy.get_param('~world_z_offset', 0.0)

        # ---------------- 频率 ----------------
        self.state_rate = rospy.get_param('~state_rate', 20.0)
        self.cloud_rate = rospy.get_param('~cloud_rate', 10.0)
        self.status_period = rospy.get_param('~status_period', 5.0)
        self.goal_velocity = rospy.get_param('~goal_velocity', 3.0)
        self.auto_takeoff = rospy.get_param('~auto_takeoff', True)
        self.takeoff_height = rospy.get_param('~takeoff_height', 3.0)

        # AirSim 的 msgpackrpc 客户端不是线程安全的，
        # 两个定时器共用一个客户端，所有调用都必须持锁
        self.lock = threading.Lock()
        self.client = self._connect()

        # 运行状态
        self.armed = False
        self.pending_goal = None          # ROS 世界系 (x, y, z)
        self.cloud_count = 0
        self.last_cloud_stamp = None
        self.last_status = rospy.get_time()

        # ---------------- ROS 接口 ----------------
        self.cloud_pub = rospy.Publisher('/cloud_in', PointCloud2, queue_size=1)
        self.odom_pub = rospy.Publisher('/ground_truth/odom', Odometry, queue_size=1)
        self.status_pub = rospy.Publisher('/uav/status', String, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        rospy.Subscriber('/uav/goal', Point, self.goal_cb, queue_size=1)

        self.publish_lidar_mount()
        self.state_timer = rospy.Timer(
            rospy.Duration(1.0 / self.state_rate), self.state_cb)
        self.cloud_timer = rospy.Timer(
            rospy.Duration(1.0 / self.cloud_rate), self.cloud_cb)
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo('AirSim 桥接节点已启动')
        rospy.loginfo('  模拟器      : %s:%d', self.host, self.port)
        rospy.loginfo('  载具 / 雷达 : %s / %s', self.vehicle, self.lidar)
        rospy.loginfo('  帧          : %s -> %s -> %s',
                      self.world_frame, self.body_frame, self.lidar_frame)
        rospy.loginfo('  频率        : 状态 %.0f Hz，点云 %.0f Hz',
                      self.state_rate, self.cloud_rate)

    # ------------------------------------------------------------------
    # 连接
    # ------------------------------------------------------------------
    def _connect(self):
        client = airsim.MultirotorClient(ip=self.host, port=self.port)
        for i in range(self.connect_retries):
            try:
                client.confirmConnection()
                rospy.loginfo('已连接 AirSim（%s:%d），载具列表 %s',
                              self.host, self.port, client.listVehicles())
                return client
            except Exception as exc:
                rospy.logwarn('连接 AirSim 失败（第 %d/%d 次）：%s',
                              i + 1, self.connect_retries, exc)
                if i + 1 < self.connect_retries:
                    rospy.sleep(2.0)
        rospy.logfatal('无法连接 AirSim（%s:%d）—— 检查模拟器是否启动、'
                       '宿主防火墙是否放行 41451', self.host, self.port)
        raise RuntimeError('AirSim connection failed')

    # ------------------------------------------------------------------
    # 坐标系换算
    # ------------------------------------------------------------------
    @staticmethod
    def position_to_enu(x, y, z):
        """世界 NED → 世界 ENU。"""
        return y, x, -z

    @staticmethod
    def point_to_flu(x, y, z):
        """机体系 FRD（前-右-下）→ ROS 机体系 FLU（前-左-上）。"""
        return x, -y, -z

    @staticmethod
    def orientation_to_enu(q_airsim):
        """AirSim 姿态（NED，FRD）→ ROS 姿态（ENU，FLU）。"""
        q_ned = quat_normalize((q_airsim.w_val, q_airsim.x_val,
                                q_airsim.y_val, q_airsim.z_val))
        return quat_normalize(quat_mul(quat_mul(Q_WORLD, q_ned), Q_BODY))

    # ------------------------------------------------------------------
    # 静态 TF：base_link -> lidar_link
    # ------------------------------------------------------------------
    def publish_lidar_mount(self):
        mx, my, mz = self.mount
        fx, fy, fz = self.point_to_flu(mx, my, mz)
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.body_frame
        t.child_frame_id = self.lidar_frame
        t.transform.translation.x = fx
        t.transform.translation.y = fy
        t.transform.translation.z = fz
        # 安装姿态全部为 0 时，FRD→FLU 的旋转正好是绕 x 轴 180°，
        # 其四元数为 (w=0, x=1, y=0, z=0)
        t.transform.rotation.w = Q_BODY[0]
        t.transform.rotation.x = Q_BODY[1]
        t.transform.rotation.y = Q_BODY[2]
        t.transform.rotation.z = Q_BODY[3]
        self.static_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------
    # 状态：里程计 + TF（同时负责下发目标点）
    # ------------------------------------------------------------------
    def state_cb(self, _event):
        with self.lock:
            try:
                state = self.client.getMultirotorState(vehicle_name=self.vehicle)
            except Exception as exc:
                rospy.logwarn_throttle(5.0, '读取载具状态失败：%s', exc)
                return

            # 有新目标点时下发（AirSim 是异步执行，不等它返回）
            if self.pending_goal is not None:
                gx, gy, gz = self.pending_goal
                self.pending_goal = None
                try:
                    self._send_goal(gx, gy, gz)
                except Exception as exc:
                    rospy.logerr('下发目标点失败：%s', exc)

        pos = state.kinematics_estimated.position
        vel = state.kinematics_estimated.linear_velocity
        ex, ey, ez = self.position_to_enu(pos.x_val, pos.y_val, pos.z_val)
        ez += self.world_z_offset
        qw, qx, qy, qz = self.orientation_to_enu(
            state.kinematics_estimated.orientation)

        stamp = rospy.Time.now()

        # 里程计
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.body_frame
        odom.pose.pose.position.x = ex
        odom.pose.pose.position.y = ey
        odom.pose.pose.position.z = ez
        odom.pose.pose.orientation = Quaternion(qx, qy, qz, qw)
        # 速度也做 FRD→FLU 的轴变换
        vx, vy, vz = self.point_to_flu(vel.x_val, vel.y_val, vel.z_val)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        self.odom_pub.publish(odom)

        # 动态 TF
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.body_frame
        t.transform.translation.x = ex
        t.transform.translation.y = ey
        t.transform.translation.z = ez
        t.transform.rotation = Quaternion(qx, qy, qz, qw)
        self.tf_broadcaster.sendTransform(t)

        # 周期状态
        now = rospy.get_time()
        if now - self.last_status > self.status_period:
            self.last_status = now
            dist = math.sqrt(ex * ex + ey * ey + ez * ez)
            text = ('位置 ENU (%.2f, %.2f, %.2f)  已收点云 %d 帧  到原点 %.1f m'
                    % (ex, ey, ez, self.cloud_count, dist))
            self.status_pub.publish(String(data=text))
            rospy.loginfo(text)

    def _send_goal(self, x_enu, y_enu, z_enu):
        """ROS 世界系目标点 → AirSim 世界 NED → moveToPositionAsync。"""
        if self.auto_takeoff and not self.armed:
            rospy.loginfo('首次收到目标点，解锁并起飞')
            self.client.enableApiControl(True, vehicle_name=self.vehicle)
            self.client.armDisarm(True, vehicle_name=self.vehicle)
            self.client.takeoffAsync(vehicle_name=self.vehicle)
            self.armed = True

        # ENU → NED： (x_ned, y_ned, z_ned) = (n, e, -u) = (y_enu, x_enu, -z_enu)
        # 注意先把世界系 z 偏移减掉，否则目标点会整体偏高/偏低一个偏移量
        x_ned, y_ned, z_ned = y_enu, x_enu, -(z_enu - self.world_z_offset)
        rospy.loginfo('目标点 ENU (%.2f, %.2f, %.2f) -> NED (%.2f, %.2f, %.2f)，'
                      '速度 %.1f m/s',
                      x_enu, y_enu, z_enu, x_ned, y_ned, z_ned,
                      self.goal_velocity)
        self.client.moveToPositionAsync(
            x_ned, y_ned, z_ned, self.goal_velocity,
            vehicle_name=self.vehicle)

    def goal_cb(self, msg):
        with self.lock:
            self.pending_goal = (msg.x, msg.y, msg.z)
        rospy.loginfo('收到目标点 (%.2f, %.2f, %.2f)', msg.x, msg.y, msg.z)

    # ------------------------------------------------------------------
    # 点云
    # ------------------------------------------------------------------
    def cloud_cb(self, _event):
        with self.lock:
            try:
                data = self.client.getLidarData(
                    lidar_name=self.lidar, vehicle_name=self.vehicle)
            except Exception as exc:
                rospy.logwarn_throttle(5.0, '读取激光雷达失败：%s', exc)
                return

        raw = data.point_cloud
        if not raw:
            return
        # 时间戳没变说明这一帧还没更新，跳过，避免重复发同一帧
        if data.time_stamp == self.last_cloud_stamp:
            return
        self.last_cloud_stamp = data.time_stamp

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.lidar_frame

        # SensorLocalFrame：点已是机体系（NED FRD），逐点转成 FLU
        points = []
        for i in range(0, len(raw) - 2, 3):
            x, y, z = self.point_to_flu(raw[i], raw[i + 1], raw[i + 2])
            points.append((x, y, z))

        msg = point_cloud2.create_cloud_xyz32(header, points)
        self.cloud_pub.publish(msg)

        self.cloud_count += 1
        if self.cloud_count == 1:
            rospy.loginfo('收到首帧点云：%d 个点，frame_id=%s（雷达 %s）',
                          len(points), self.lidar_frame, self.lidar)

    # ------------------------------------------------------------------
    def shutdown(self):
        rospy.loginfo('桥接节点退出')
        with self.lock:
            try:
                if self.armed:
                    self.client.hoverAsync(vehicle_name=self.vehicle)
            except Exception as exc:
                rospy.logwarn('退出前悬停失败：%s', exc)


def main():
    rospy.init_node('airsim_bridge')
    AirSimBridge()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
