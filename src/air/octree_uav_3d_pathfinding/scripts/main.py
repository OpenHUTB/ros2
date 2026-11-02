#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
空域载具八叉树三维寻路 —— 主入口节点（环境与链路自检）

本节点用于验证整个模块的运行环境与链路是否就绪。因为模拟器（AirSim）
跑在宿主 Windows 上、ROS 跑在客户机虚拟机里，"链路"天然分成两层，
所以自检也分两层来做 —— 这样出问题时能一眼看出是模拟器的事还是 ROS 的事：

  1. 输出运行环境信息（ROS 发行版、Python 版本、AirSim 客户端版本、octomap 版本）
  2. 直连 AirSim RPC，确认模拟器在线、能列出载具
  3. 订阅 /cloud_in，确认桥接节点把激光点云送进了 ROS
  4. 查询 TF，确认 world -> base_link 的坐标变换可用
  5. 发布心跳话题 /uav_status，确认节点间通信正常
  6. 运行指定时长后自动退出，便于在终端中直接观察完整输出

运行：
    roslaunch octree_uav_3d_pathfinding main.launch
或：
    rosrun octree_uav_3d_pathfinding main.py
"""

import os
import re
import subprocess
import sys

import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

# 心跳话题名
TOPIC_STATUS = '/uav_status'

# 模块名与版本
MODULE_NAME = 'octree_uav_3d_pathfinding'
MODULE_VERSION = '0.2.0'


def read_octomap_version():
    """读取已安装的 octomap 版本。

    探测顺序（从最直接到最兜底），任何一步成功就返回：
      1. Python 绑定 octomap
      2. ROS 包 octomap 的 package.xml —— 若装了 ros-<distro>-octomap，版本与上游一致
         （注意不能遍历 /opt/ros 下几百个包去找，那样既慢又容易被某个包解析失败打断）
      3. liboctomap-dev 安装的 CMake 版本文件
      4. dpkg 查询
    """
    distro = os.environ.get('ROS_DISTRO', 'noetic')

    # 1) Python 绑定
    try:
        import octomap
        version = getattr(octomap, '__version__', None)
        if version:
            return version
    except ImportError:
        pass

    # 2) ROS 包的 package.xml：直接读这一个文件
    try:
        with open('/opt/ros/%s/share/octomap/package.xml' % distro) as handle:
            match = re.search(r'<version>\s*([^<\s]+)\s*</version>', handle.read())
            if match:
                return match.group(1)
    except (IOError, OSError):
        pass

    # 3) CMake 版本文件（不同发行版放的位置不一样，逐个试）
    for path in ('/usr/share/octomap/octomap-config-version.cmake',
                 '/usr/lib/x86_64-linux-gnu/cmake/octomap/octomap-config-version.cmake',
                 '/usr/lib/octomap/octomap-config-version.cmake',
                 '/usr/local/share/octomap/octomap-config-version.cmake'):
        try:
            with open(path) as handle:
                match = re.search(r'set\(PACKAGE_VERSION\s+"([^"]+)"',
                                  handle.read(), re.IGNORECASE)
                if match:
                    return match.group(1)
        except (IOError, OSError):
            continue

    # 4) dpkg 兜底
    for pkg in ('liboctomap-dev', 'ros-%s-octomap' % distro):
        try:
            out = subprocess.check_output(
                ['dpkg-query', '-W', '-f=${Version}', pkg],
                stderr=subprocess.STDOUT).decode('utf-8', errors='replace').strip()
            if out and 'no packages found' not in out and 'not installed' not in out:
                return out
        except Exception:
            continue

    return '未检测到'


def read_airsim_version():
    """读取 airsim 客户端包的版本。"""
    try:
        import airsim
        return getattr(airsim, '__version__', 'unknown')
    except ImportError:
        return '未安装'


class CloudMonitor(object):
    """订阅 /cloud_in，统计点云帧数、点数与频率。"""

    def __init__(self, topic):
        self.count = 0
        self.last_points = 0
        self.last_frame = ''
        self.first_wall = None
        self.last_wall = None
        self.rate = 0.0
        self.sub = rospy.Subscriber(topic, PointCloud2, self._callback, queue_size=1)

    def _callback(self, msg):
        now = rospy.get_time()
        self.count += 1
        self.last_points = msg.width * msg.height
        self.last_frame = msg.header.frame_id
        self.last_wall = now
        if self.first_wall is None:
            self.first_wall = now

    @property
    def received(self):
        return self.count > 0

    def compute_rate(self):
        if self.count > 1 and self.first_wall is not None and self.last_wall is not None:
            elapsed = self.last_wall - self.first_wall
            if elapsed > 0:
                self.rate = (self.count - 1) / elapsed
        return self.rate

    def reset_stats(self):
        self.count = 0
        self.first_wall = None
        self.last_wall = None
        self.rate = 0.0


class TfCounter(object):
    """挂在 /tf 上统计消息数与出现过的帧名。

    自检失败时最怕"不知道是没发还是查不到"，这个计数器让报错自带结论：
    消息数为 0 说明桥接节点根本没广播；有消息但帧名对不上说明是帧名配置问题。
    """

    def __init__(self):
        self.count = 0
        self.frames = set()
        self.sub = rospy.Subscriber('/tf', TFMessage, self._callback, queue_size=100)

    def _callback(self, msg):
        self.count += 1
        for tr in msg.transforms:
            self.frames.add(tr.header.frame_id)
            self.frames.add(tr.child_frame_id)

    def describe(self):
        if not self.frames:
            return '/tf 上一条消息都没有收到'
        return '/tf 上收到 %d 条消息，出现过的帧：%s' % (
            self.count, ', '.join(sorted(self.frames)))


class HeartbeatMonitor(object):
    """订阅心跳话题，用于确认消息真的能收发一圈。"""

    def __init__(self, topic):
        self.count = 0
        self.sub = rospy.Subscriber(topic, String, self._callback, queue_size=10)

    def _callback(self, _msg):
        self.count += 1


def print_environment(params):
    """打印运行环境信息。"""
    distro = os.environ.get('ROS_DISTRO', 'unknown')

    rospy.loginfo('=' * 64)
    rospy.loginfo('模块: %s  v%s', MODULE_NAME, MODULE_VERSION)
    rospy.loginfo('=' * 64)
    rospy.loginfo('ROS 发行版      : %s', distro)
    rospy.loginfo('Python 版本     : %s', sys.version.split()[0])
    rospy.loginfo('AirSim 客户端   : %s', read_airsim_version())
    rospy.loginfo('octomap 版本    : %s', read_octomap_version())
    rospy.loginfo('模拟器地址      : %s:%d', params['host'], params['port'])
    rospy.loginfo('载具 / 激光雷达 : %s / %s', params['vehicle'], params['lidar'])
    rospy.loginfo('节点名          : %s', rospy.get_name())
    rospy.loginfo('-' * 64)


def check_airsim_rpc(params):
    """[1/4] 直连 AirSim RPC，确认模拟器在线并列出载具。"""
    try:
        import airsim
    except ImportError:
        rospy.logerr('[1/4] 未安装 airsim 客户端')
        rospy.logerr('      执行：pip3 install numpy msgpack-rpc-python airsim')
        return False

    client = airsim.MultirotorClient(ip=params['host'], port=params['port'])
    try:
        client.confirmConnection()
    except Exception as exc:
        rospy.logerr('[1/4] 无法连接 AirSim（%s:%d）：%s',
                     params['host'], params['port'], exc)
        rospy.logerr('      依次检查：')
        rospy.logerr('        1) 宿主 Windows 上模拟器是否已启动（CarlaUE4.exe）')
        rospy.logerr('        2) 宿主防火墙是否放行 %d 端口', params['port'])
        rospy.logerr('        3) host 是否为宿主机的 VMnet8 地址（不是 127.0.0.1）')
        return False

    vehicles = client.listVehicles()
    rospy.loginfo('[1/4] AirSim RPC 正常：载具列表 %s', vehicles)
    if params['vehicle'] and params['vehicle'] not in vehicles:
        rospy.logwarn('      注意：配置的载具 "%s" 不在列表里，'
                      '请核对 config/params.yaml 与模拟器的 settings.json',
                      params['vehicle'])
    return True


def wait_for_cloud(monitor, topic, timeout):
    """[2/4] 等待桥接节点发布的第一帧点云。"""
    start = rospy.get_time()
    while not monitor.received:
        if rospy.get_time() - start > timeout:
            rospy.logerr('[2/4] %s 等待超时（%.1f 秒无数据）', topic, timeout)
            rospy.logerr('      可能原因：桥接节点未启动，或 AirSim 没有返回激光数据')
            return False
        rospy.sleep(0.2)
    rospy.loginfo('[2/4] 点云链路正常：已收到 %d 帧，最新一帧 %d 个点，frame_id=%s',
                  monitor.count, monitor.last_points, monitor.last_frame)
    return True


def check_tf(buffer, counter, world, body, timeout):
    """[3/4] 查询 world -> base_link 的坐标变换。"""
    start = rospy.get_time()
    warned = False
    last_error = None
    while not rospy.is_shutdown():
        try:
            # 注意：tf2_ros 的 Python 版查询用 snake_case（lookup_transform），
            # 驼峰的 lookupTransform 是 C++ 和 tf1 的 API，在这里会抛 AttributeError。
            # 同库的广播却用驼峰 sendTransform —— 这个库自己不一致，容易写错。
            tr = buffer.lookup_transform(world, body, rospy.Time(0),
                                         rospy.Duration(0.5))
            p = tr.transform.translation
            rospy.loginfo('[3/4] TF 正常：%s -> %s，当前位置 (%.2f, %.2f, %.2f)',
                          world, body, p.x, p.y, p.z)
            return True
        except Exception as exc:
            last_error = exc
            if not warned:
                rospy.loginfo('[3/4] 等待 TF %s -> %s ...', world, body)
                warned = True
            if rospy.get_time() - start > timeout:
                rospy.logerr('[3/4] 等不到 TF %s -> %s', world, body)
                rospy.logerr('      最后一次查询报错：%s', last_error)
                rospy.logerr('      诊断：%s', counter.describe())
                return False
            rospy.sleep(0.2)
    return False


def check_heartbeat(pub, monitor, pub_topic, timeout=3.0):
    """[4/4] 发一次心跳并确认自己收到了，验证话题链路真的能通。

    只"发布"是不够的 —— 如果没有任何订阅者，发布本身永远不会报错，
    这个检查就等于什么都没验证。所以这里自己订阅自己，确认消息真的转了一圈。
    """
    start = rospy.get_time()
    while (monitor.count == 0 and not rospy.is_shutdown()
           and rospy.get_time() - start < timeout):
        msg = String()
        msg.data = '{} v{} alive'.format(MODULE_NAME, MODULE_VERSION)
        pub.publish(msg)
        rospy.sleep(0.2)

    if monitor.count > 0:
        rospy.loginfo('[4/4] 话题通信正常：%s 收发成功（%d 条），订阅者 %d 个',
                      pub_topic, monitor.count, pub.get_num_connections())
        return True

    rospy.logerr('[4/4] 话题通信失败：向 %s 发布了但自己没有收到', pub_topic)
    return False


def main():
    rospy.init_node('uav_env_check', anonymous=False)

    # 参数（在 config/params.yaml 中配置，与 airsim_bridge 保持一致）
    params = {
        'host': rospy.get_param('~host', '192.168.198.1'),
        'port': rospy.get_param('~port', 41451),
        'vehicle': rospy.get_param('~vehicle_name', 'Drone1'),
        'lidar': rospy.get_param('~lidar_name', 'LidarSensor1'),
    }
    cloud_topic = rospy.get_param('~cloud_topic', '/cloud_in')
    world_frame = rospy.get_param('~world_frame', 'world')
    body_frame = rospy.get_param('~body_frame', 'base_link')
    check_duration = rospy.get_param('~check_duration', 10.0)
    cloud_timeout = rospy.get_param('~cloud_timeout', 20.0)
    tf_timeout = rospy.get_param('~tf_timeout', 10.0)
    pub_topic = rospy.get_param('~status_topic', TOPIC_STATUS)

    print_environment(params)

    monitor = CloudMonitor(cloud_topic)
    status_pub = rospy.Publisher(pub_topic, String, queue_size=10)
    hb_monitor = HeartbeatMonitor(pub_topic)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)   # noqa: F841 生命周期需保持
    tf_counter = TfCounter()                             # 自检失败时用来说明原因

    results = []
    results.append(('AirSim RPC 连接', check_airsim_rpc(params)))
    results.append(('激光点云链路', wait_for_cloud(monitor, cloud_topic, cloud_timeout)))
    results.append(('TF 坐标变换', check_tf(tf_buffer, tf_counter,
                                            world_frame, body_frame, tf_timeout)))
    results.append(('话题通信', check_heartbeat(status_pub, hb_monitor, pub_topic)))

    # 观察窗口：持续发心跳，统计点云与位置
    monitor.reset_stats()
    start = rospy.get_time()
    rate = rospy.Rate(2)  # 2 Hz
    while not rospy.is_shutdown() and rospy.get_time() - start < check_duration:
        msg = String()
        msg.data = '{} alive | cloud_frames={} | points={}'.format(
            MODULE_NAME, monitor.count, monitor.last_points)
        status_pub.publish(msg)
        rospy.loginfo('运行中... 已收 %d 帧点云（最新 %d 点），'
                      '订阅者 %d 个',
                      monitor.count, monitor.last_points,
                      status_pub.get_num_connections())
        rate.sleep()

    cloud_rate = monitor.compute_rate()

    rospy.loginfo('-' * 64)
    rospy.loginfo('检查结果：')
    all_ok = True
    for name, ok in results:
        rospy.loginfo('  [%s] %s', '通过' if ok else '失败', name)
        all_ok = all_ok and ok
    rospy.loginfo('  %s 平均频率: %.1f Hz（累计 %d 帧）',
                  cloud_topic, cloud_rate, monitor.count)
    rospy.loginfo('-' * 64)

    if all_ok:
        rospy.loginfo('环境与链路自检全部通过，模块可以开始后续开发。')
    else:
        rospy.logwarn('存在未通过的检查项，请根据上方提示排查。')

    rospy.loginfo('自检节点退出（仿真与桥接节点继续运行）。')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
