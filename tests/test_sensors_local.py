#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""传感器桥接本地测试（桩依赖，不需要 ROS / 仿真器 / GPU）.

A. 图像解码：BGRA -> BGR、三通道兼容、深度平面图 float32
B. 图像消息打包：bgr8 / 32FC1 的 height/width/step/data 布局
C. 点云：NED->ENU 轴变换、SensorLocalFrame 位姿变换、环形滤波、体素降采样、
        PointCloud2 二进制布局（point_step=12, FLOAT32 小端）
D. 节点逻辑：ImagePublisher / LidarPublisher 的 step_once 发布行为
E. 工程文件：CMakeLists / main.launch / bridge.yaml / settings.json 一致性

用法: python3 tests/test_sensors_local.py
"""
from __future__ import annotations

import json
import os
import struct
import sys
import types
import xml.etree.ElementTree as ET

import numpy as np

PASS, FAIL = [], []


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "src", "air", "carlair_ros_bridge")
SCRIPTS = os.path.join(PKG, "scripts")
sys.path.insert(0, SCRIPTS)


# ---------------------------------------------------------------- 消息桩
class Header(object):
    def __init__(self):
        self.stamp = None
        self.frame_id = ""
        self.seq = 0


class Image(object):
    def __init__(self):
        self.header = Header()
        self.height = 0
        self.width = 0
        self.encoding = ""
        self.is_bigendian = False
        self.step = 0
        self.data = b""


class PointField(object):
    def __init__(self, name="", offset=0, datatype=0, count=1):
        self.name, self.offset, self.datatype, self.count = name, offset, datatype, count


class PointCloud2(object):
    def __init__(self):
        self.header = Header()
        self.height = 0
        self.width = 0
        self.fields = []
        self.is_bigendian = False
        self.point_step = 0
        self.row_step = 0
        self.data = b""
        self.is_dense = False


class _Resp(object):
    """ImageResponse 的最小替身."""

    def __init__(self, width, height, uint8=None, floats=None):
        self.width, self.height = width, height
        self.image_data_uint8 = [] if uint8 is None else uint8
        self.image_data_float = [] if floats is None else floats


class _Pose(object):
    """LidarData.pose 的最小替身（NED）."""

    def __init__(self, t, q_wxyz):
        self.position = types.SimpleNamespace(x_val=t[0], y_val=t[1], z_val=t[2])
        self.orientation = types.SimpleNamespace(
            w_val=q_wxyz[0], x_val=q_wxyz[1], y_val=q_wxyz[2], z_val=q_wxyz[3])


# ---------------------------------------------------------------- rospy 桩
PARAMS = {
    "sim/host": "127.0.0.1", "sim/airsim_port": 41451, "sim/vehicle_name": "Drone1",
    "rate/image_hz": 20.0, "rate/lidar_hz": 10.0,
    "topic/image": "/camera/image_raw", "topic/depth": "/camera/depth",
    "topic/seg": "/camera/seg", "topic/lidar": "/lidar/points",
    "camera/rgb_name": "front_rgb", "camera/depth_name": "front_depth",
    "camera/seg_name": "front_seg",
    "camera/publish_rgb": True, "camera/publish_depth": True, "camera/publish_seg": False,
    "sensor/lidar_name": "lidar1", "sensor/lidar_frame": "vehicle_inertial",
    "lidar/min_range": 0.3, "lidar/max_range": 60.0, "lidar/voxel_leaf": 0.0,
    "frame/world": "world", "frame/body": "base_link",
}
PUBS = {}
CLOCK = {"t": 100.0}


class _Publisher(object):
    def __init__(self, name, msg_type, queue_size=10):
        self.name = name
        self.msgs = []
        PUBS[name] = self

    def publish(self, msg):
        self.msgs.append(msg)


class _Time(object):
    @staticmethod
    def now():
        return CLOCK["t"]


class _Rate(object):
    def __init__(self, hz):
        self.hz = hz

    def sleep(self):
        pass


def install_stubs():
    m = types.ModuleType("rospy")
    m.get_param = lambda name, default=None: PARAMS.get(name, default)
    m.Publisher = _Publisher
    m.Rate = _Rate
    m.Time = _Time
    m.init_node = lambda *a, **k: None
    m.is_shutdown = lambda: False
    m.loginfo = lambda *a, **k: None
    m.logwarn = lambda *a, **k: None
    m.logwarn_throttle = lambda *a, **k: None
    sys.modules["rospy"] = m

    sm = types.ModuleType("sensor_msgs")
    smm = types.ModuleType("sensor_msgs.msg")
    smm.Image, smm.PointCloud2, smm.PointField = Image, PointCloud2, PointField
    sm.msg = smm
    sys.modules["sensor_msgs"] = sm
    sys.modules["sensor_msgs.msg"] = smm


install_stubs()
import image_pub  # noqa: E402
import lidar_pub  # noqa: E402
import sim_client  # noqa: E402


# ================================================================= A. 图像解码
def test_decode():
    print("== A. 图像解码 ==")
    # 2x2 的 BGRA 图：像素 (B,G,R,A)
    bgra = bytes([1, 2, 3, 255, 4, 5, 6, 255, 7, 8, 9, 255, 10, 11, 12, 255])
    img = sim_client.decode_rgb(_Resp(2, 2, uint8=bgra))
    check(img.shape == (2, 2, 3), "BGRA -> (H, W, 3) 形状正确: %r" % (img.shape,))
    check(tuple(img[0, 0]) == (1, 2, 3), "丢掉 alpha 后为 BGR 顺序: %r" % (tuple(img[0, 0]),))
    check(tuple(img[1, 1]) == (10, 11, 12), "第四像素值正确: %r" % (tuple(img[1, 1]),))
    check(img.dtype == np.uint8 and img.flags["C_CONTIGUOUS"], "dtype=uint8 且内存连续")

    img3 = sim_client.decode_rgb(_Resp(2, 1, uint8=bytes([1, 2, 3, 4, 5, 6])))
    check(img3.shape == (1, 2, 3), "三通道输入也能解析: %r" % (img3.shape,))

    img_list = sim_client.decode_rgb(_Resp(2, 1, uint8=list(range(1, 9))))
    check(img_list.shape == (1, 2, 3), "list 形式的字节流同样可解析")

    try:
        sim_client.decode_rgb(_Resp(2, 2, uint8=bytes(4)))
        check(False, "尺寸非法时应抛异常")
    except ValueError:
        check(True, "字节数与尺寸不匹配时抛 ValueError")

    depth = sim_client.decode_depth(_Resp(3, 2, floats=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]))
    check(depth.shape == (2, 3) and depth.dtype == np.float32, "深度图 -> (H, W) float32")
    check(abs(float(depth[1, 2]) - 6.0) < 1e-6, "深度值按行优先排布: %.1f" % depth[1, 2])

    try:
        sim_client.decode_depth(_Resp(3, 2, floats=[1.0, 2.0]))
        check(False, "深度浮点数个数不符时应抛异常")
    except ValueError:
        check(True, "深度图元素个数不符时抛 ValueError")


# ================================================================= B. 消息打包
def test_image_msg():
    print("== B. sensor_msgs/Image 打包 ==")
    arr = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
    msg = image_pub.make_image_msg(arr, "bgr8", "base_link", 100.0, 7)
    check(msg.height == 2 and msg.width == 3, "height/width 正确: %dx%d" % (msg.height, msg.width))
    check(msg.encoding == "bgr8" and msg.header.frame_id == "base_link", "encoding/frame_id 正确")
    check(msg.step == 3 * 3 * 1, "step = width * channels * itemsize = %d" % msg.step)
    check(len(msg.data) == msg.step * msg.height, "data 长度 = step * height = %d" % len(msg.data))
    check(msg.is_bigendian is False and msg.header.seq == 7, "is_bigendian=False，seq 透传")

    d = np.array([[1.5, 2.5], [3.5, 4.5]], dtype=np.float32)
    dmsg = image_pub.make_image_msg(d, "32FC1", "world", 1.0)
    check(dmsg.step == 2 * 4 and len(dmsg.data) == 2 * 2 * 4, "32FC1: step = width*4")
    back = np.frombuffer(dmsg.data, dtype="<f4").reshape(2, 2)
    check(np.allclose(back, d), "深度字节流可无损还原")

    try:
        image_pub.make_image_msg(np.zeros((1,)), "bgr8", "world", 0.0)
        check(False, "维度非法时应抛异常")
    except ValueError:
        check(True, "一维数组抛 ValueError")


# ================================================================= C. 点云
def test_lidar():
    print("== C. 点云解码与 PointCloud2 ==")
    # vehicle_inertial：NED(1,2,3) -> ENU(2,1,-3)
    p = sim_client.lidar_points_to_enu([1.0, 2.0, 3.0, -4.0, 5.0, -6.0])
    check(p.shape == (2, 3), "点数为 2 的 (N, 3) 数组: %r" % (p.shape,))
    check(np.allclose(p[0], [2.0, 1.0, -3.0]), "NED(1,2,3) -> ENU(2,1,-3): %r" % (p[0],))
    check(np.allclose(p[1], [5.0, -4.0, 6.0]), "NED(-4,5,-6) -> ENU(5,-4,6): %r" % (p[1],))

    empty = sim_client.lidar_points_to_enu([])
    check(empty.shape == (0, 3), "空点云返回 (0, 3)")

    for bad, label in (([1.0, 2.0], "长度不是 3 的倍数"),
                       ([1.0, 2.0, 3.0, 4.0], "4 个浮点数")):
        try:
            sim_client.lidar_points_to_enu(bad)
            check(False, "%s 应抛异常" % label)
        except ValueError:
            check(True, "%s -> ValueError" % label)

    try:
        sim_client.lidar_points_to_enu([1.0, 2.0, 3.0], frame="sensor_local")
        check(False, "sensor_local 缺 pose 时应抛异常")
    except ValueError:
        check(True, "sensor_local 未提供 pose -> ValueError")

    try:
        sim_client.lidar_points_to_enu([1.0, 2.0, 3.0], frame="bogus")
        check(False, "未知坐标系应抛异常")
    except ValueError:
        check(True, "未知 DataFrame -> ValueError")

    # 位姿约定自检：NED 下绕 z 轴 +90°(向右偏航，北->东) 时，机体 +X 应指向东
    q_yaw90 = [np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)]
    R = sim_client.quat_wxyz_to_matrix(q_yaw90)
    check(np.allclose(R.dot([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-9),
          "偏航 +90° 时机体 X 轴指向东 (NED)")

    # 零位姿时 sensor_local 与 vehicle_inertial 等价
    pose0 = _Pose([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])
    a = sim_client.lidar_points_to_enu([1.0, 2.0, 3.0], pose=pose0, frame="sensor_local")
    b = sim_client.lidar_points_to_enu([1.0, 2.0, 3.0], frame="vehicle_inertial")
    check(np.allclose(a, b), "单位位姿下两种 DataFrame 结果一致")

    # 一般位姿：由世界点正向投影到雷达局部系，反解必须回到同一个世界点
    t_ned = np.array([3.0, -2.0, 5.0])
    pose = _Pose(list(t_ned), q_yaw90)
    p_world_ned = np.array([[10.0, 1.0, -20.0], [-4.0, 6.0, -25.5], [0.0, 0.0, 0.0]])
    p_sensor = (p_world_ned - t_ned).dot(R)          # 世界 -> 局部：R^T (p - t)
    got = sim_client.lidar_points_to_enu(p_sensor.ravel(), pose=pose, frame="sensor_local")
    want = p_world_ned.dot(sim_client.C_NED2ENU.T)
    check(np.allclose(got, want, atol=1e-9), "SensorLocalFrame + 雷达位姿可还原世界系 ENU 点")

    # 环形滤波
    pts = np.array([[0.1, 0.0, 0.0], [1.0, 0.0, 0.0], [5.0, 0.0, 0.0], [70.0, 0.0, 0.0]])
    kept = lidar_pub.filter_by_range(pts, 0.3, 60.0)
    check(len(kept) == 2 and np.allclose(kept[:, 0], [1.0, 5.0]),
          "环形滤波保留 1 m 与 5 m 的点，剔除 0.1 m 与 70 m")
    check(len(lidar_pub.filter_by_range(pts, 0.0, 0.0)) == 4, "阈值 0 表示不启用滤波")

    # 体素降采样
    grid = np.array([[0.05, 0.05, 0.05], [0.15, 0.15, 0.15], [2.1, 0.0, 0.0]])
    down = lidar_pub.voxel_downsample(grid, 0.5)
    check(len(down) == 2, "0.5 m 体素把前两点合并: %d -> %d" % (len(grid), len(down)))
    check(np.allclose(down[0], [0.1, 0.1, 0.1]), "合并后取体素内均值: %r" % (down[0],))
    check(len(lidar_pub.voxel_downsample(grid, 0.0)) == 3, "leaf=0 时不降采样")

    # 字节布局
    raw = lidar_pub.pack_xyz32(np.array([[1.0, 2.0, -3.0]]))
    check(len(raw) == 12, "单点 12 字节 (3 x float32)")
    check(struct.unpack("<3f", raw) == (1.0, 2.0, -3.0), "小端 float32 顺序为 x, y, z")

    cloud = lidar_pub.make_pointcloud2(np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]),
                                       "world", 42.0, 3)
    check(cloud.width == 2 and cloud.height == 1, "无序点云 height=1, width=N")
    check(cloud.point_step == 12 and cloud.row_step == 24, "point_step=12, row_step=24")
    check([(f.name, f.offset, f.datatype) for f in cloud.fields]
          == [("x", 0, 7), ("y", 4, 7), ("z", 8, 7)], "字段 x@0 y@4 z@8 均为 FLOAT32(7)")
    check(cloud.header.frame_id == "world" and cloud.header.seq == 3, "frame_id/seq 正确")
    check(cloud.is_dense is True, "全部有限 -> is_dense=True")
    check(len(cloud.data) == 2 * 12, "data 长度 = N * point_step")

    nan_cloud = lidar_pub.make_pointcloud2(np.array([[np.nan, 0.0, 0.0]]), "world", 0.0)
    check(nan_cloud.is_dense is False, "含 NaN -> is_dense=False")

    empty_cloud = lidar_pub.make_pointcloud2(np.zeros((0, 3)), "world", 0.0)
    check(empty_cloud.width == 0 and empty_cloud.data == b"", "空点云也能构造合法消息")


# ================================================================= D. 节点逻辑
class FakeSim(object):
    def __init__(self, resp=None, lidar=None):
        self.resp = resp or {}
        self.lidar = lidar
        self.calls = []

    def connect(self):
        self.calls.append(("connect",))

    def get_images_bundle(self, specs):
        self.calls.append(("bundle", tuple(s[0] for s in specs)))
        return [(s[0], self.resp.get(s[0])) for s in specs]

    def get_lidar_points_enu(self, name="lidar1", frame="vehicle_inertial"):
        self.calls.append(("lidar", name, frame))
        return self.lidar


def test_nodes():
    print("== D. 节点发布逻辑 ==")
    resp = {"front_rgb": _Resp(2, 1, uint8=bytes([1, 2, 3, 255, 4, 5, 6, 255])),
            "front_depth": _Resp(2, 1, floats=[1.0, 2.0])}

    fake_img = FakeSim(resp)
    image_pub.SimClient = lambda **kw: fake_img
    PUB1 = image_pub.ImagePublisher()
    check(PUB1.specs == [("front_rgb", 0, False), ("front_depth", 1, True)],
          "采集清单与开启的相机一致: %r" % (PUB1.specs,))
    PUB1.step_once()
    check(len(PUBS["/camera/image_raw"].msgs) == 1, "/camera/image_raw 发布 1 帧")
    check(len(PUBS["/camera/depth"].msgs) == 1, "/camera/depth 发布 1 帧")
    check(len(PUBS["/camera/seg"].msgs) == 0, "未开启的 /camera/seg 不发布")
    check(PUBS["/camera/image_raw"].msgs[0].encoding == "bgr8", "彩色图 encoding=bgr8")
    check(PUBS["/camera/depth"].msgs[0].encoding == "32FC1", "深度图 encoding=32FC1")
    check(PUB1.seq == 1, "seq 自增")
    PUB1.step_once()
    check(PUB1.seq == 2 and len(PUBS["/camera/image_raw"].msgs) == 2, "第二次调用继续发布")

    fake_lidar = FakeSim(lidar=np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]))
    lidar_pub.SimClient = lambda **kw: fake_lidar
    PUB2 = lidar_pub.LidarPublisher()
    PUB2.step_once()
    msgs = PUBS["/lidar/points"].msgs
    check(len(msgs) == 1 and msgs[0].width == 2, "/lidar/points 发布 2 点")
    check(msgs[0].header.frame_id == "world", "点云 frame_id=world")
    pts = np.frombuffer(msgs[0].data, dtype="<f4").reshape(-1, 3)
    check(np.allclose(pts, [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]), "点云坐标按原样写入字节流")
    check(PUB2.total_points == 2, "统计发布点数 = 2")
    check(("lidar", "lidar1", "vehicle_inertial") in fake_lidar.calls,
          "按 bridge.yaml 的 DataFrame 参数读取点云: %r" % (fake_lidar.calls,))
    check(len(PUB2.process(np.array([[0.1, 0.0, 0.0], [1.0, 0.0, 0.0]]))) == 1,
          "节点内 process 应用了 0.3 m 最小距离滤波")

    fake_lidar2 = FakeSim(lidar=None)
    PUB2.sim = fake_lidar2
    PUB2.step_once()
    check(len(PUBS["/lidar/points"].msgs) == 1, "无点云时不发布空消息")


# ================================================================= E. 工程文件
def test_project_files():
    print("== E. 工程文件一致性 ==")
    cmake = open(os.path.join(PKG, "CMakeLists.txt"), encoding="utf-8").read()
    for name in ("image_pub.py", "lidar_pub.py"):
        check(name in cmake, "CMakeLists 安装 %s" % name)

    launch = ET.parse(os.path.join(PKG, "launch", "main.launch")).getroot()
    types_ = [n.get("type") for n in launch.findall("node")]
    check("image_pub.py" in types_ and "lidar_pub.py" in types_, "main.launch 启动两个新节点")
    args = [a.get("name") for a in launch.findall("arg")]
    check("publish_image" in args and "publish_lidar" in args, "main.launch 暴露开关参数")

    cfg = open(os.path.join(PKG, "config", "bridge.yaml"), encoding="utf-8").read()
    for key in ("image_hz", "lidar_hz", "lidar_name", "voxel_leaf", "publish_depth"):
        check(key in cfg, "bridge.yaml 含参数 %s" % key)

    st = json.load(open(os.path.join(PKG, "config", "settings.json"), encoding="utf-8"))
    lidar = st["Vehicles"]["Drone1"]["Sensors"]["lidar1"]
    check(lidar["DataFrame"] == "VehicleInertialFrame",
          "settings.json 雷达 DataFrame=VehicleInertialFrame（与 lidar_frame 一致）")
    cams = st["Vehicles"]["Drone1"]["Cameras"]
    for cam, itype in (("front_rgb", 0), ("front_depth", 1), ("front_seg", 5)):
        check(cams[cam]["CaptureSettings"][0]["ImageType"] == itype,
              "相机 %s 的 ImageType=%d" % (cam, itype))


def main():
    test_decode()
    test_image_msg()
    test_lidar()
    test_nodes()
    test_project_files()
    print("\n==================== 结果 ====================")
    print("通过 %d 项，失败 %d 项" % (len(PASS), len(FAIL)))
    if FAIL:
        for f in FAIL:
            print("  FAIL: %s" % f)
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
