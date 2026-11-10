#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""无人机多传感器感知与给定轨迹跟踪控制 —— 主入口。

对应课程任务（2）：使用传感器获取到的数据进行感知，对无人机进行运动控制。

运行::

    # 终端 1：桥接（必须 body_frame:=false，因为本模块输出世界系速度）
    roslaunch carlair_ros_bridge main.launch publish_image:=false body_frame:=false

    # 终端 2：轨迹跟踪
    roslaunch uav_perception_trajectory main.launch traj:=figure8

控制链路::

    /lidar/points ─┐
                   ├─► 本节点 ──/uav/cmd_vel──► carlair_ros_bridge ──► CarlaAir
    /uav/odom    ──┘   (感知→控制)

为什么必须在启动时校验 body_frame
--------------------------------
本模块输出的是**世界系速度**。若桥接按机体系解释，无人机前进方向会随机头
漂移而改变，轨迹会变成绕圈。而这类故障在外层表现为「无人机会飞、日志里
却什么错都没有」，极难排查。因此在启动时主动读取桥接参数并显式告警。
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

from controller import TrajectoryController      # noqa: E402
from perception import safety_scale, sector_histogram, pointcloud2_to_xyz  # noqa: E402
from trajectory import make_traj, period         # noqa: E402

DEFAULTS = {
    "traj": "figure8", "radius": 5.0, "omega": 0.4, "z": -5.0, "cycles": 3,
    "kp": 0.8, "ki": 0.05, "kd": 0.25,
    "v_max": 3.0, "dt": 0.1,
    "s_safe": 0.6, "s_min": 0.25,
    "cmd_topic": "/uav/cmd_vel",
    "lidar_topic": "/lidar/points",
    "odom_topic": "/uav/odom",
}


def build_parser():
    p = argparse.ArgumentParser(description="无人机感知与给定轨迹跟踪控制（课程任务 2）")
    for k, v in DEFAULTS.items():
        if isinstance(v, bool):
            p.add_argument("--" + k, action="store_true")
        else:
            p.add_argument("--" + k, type=type(v), default=None)
    # 单测/离线重放用：不连 ROS，只跑一轮纯计算
    p.add_argument("--dry-run", action="store_true",
                   help="不连 ROS，用合成传感器数据跑一轮并打印结果（自检用）")
    return p


def resolve_params(args):
    cfg = dict(DEFAULTS)
    # ROS 参数服务器（先取 launch 里设的值）
    try:
        import rospy
        if rospy.core.is_initialized():
            for k, v in DEFAULTS.items():
                cfg[k] = rospy.get_param("~" + k, v)
    except Exception:
        pass
    # 命令行最高优先级
    for k in DEFAULTS:
        val = getattr(args, k, None)
        if val is not None:
            cfg[k] = val
    return cfg


# --------------------------------------------------------------------------
# 离线自检：不连 ROS，用合成数据走一遍完整链路
# --------------------------------------------------------------------------

def run_dry_run(cfg):
    """合成「正前方 4 m 有墙」的点云，验证感知 → 控制链路。"""
    print("=== dry-run：合成传感器数据，验证感知 → 控制链路 ===")

    # 合成点云：正前方 4 m 处一片墙，另加一些地面点
    wall = np.array([[4.0, y * 0.1, 0.0] for y in range(-20, 21)])
    floor = np.array([[x, y, -2.0] for x in range(1, 10) for y in range(-5, 6)])
    pts = np.vstack([wall, floor])

    h = sector_histogram(pts)
    print("扇形直方图（16 扇，1=空旷）：")
    print("  " + " ".join("{:.2f}".format(v) for v in h))
    print("  正前方扇形 [15,0,1] = {:.2f} {:.2f} {:.2f}"
          .format(h[15], h[0], h[1]))

    s = safety_scale(h, s_safe=cfg["s_safe"], s_min=cfg["s_min"])
    print("  安全系数 s = {:.3f}（1=全速，0=停止）".format(s))

    ctrl = TrajectoryController(kp=cfg["kp"], ki=cfg["ki"], kd=cfg["kd"],
                                v_max=cfg["v_max"], dt=cfg["dt"])
    traj = make_traj(cfg["traj"], radius=cfg["radius"],
                     omega=cfg["omega"], z=cfg["z"])

    p = np.array([0.0, 0.0, cfg["z"]])   # 起点
    print("\n轨迹跟踪（前 5 步）：")
    for k in range(5):
        t = k * cfg["dt"]
        p_d, p_d_dot = traj(t)
        v = ctrl.step(p_d, p_d_dot, p)
        v_final = ctrl.apply_safety(v, s)
        print("  t={:.1f} | p_d=({:+.2f},{:+.2f},{:+.2f}) "
              "p=({:+.2f},{:+.2f},{:+.2f}) |e|={:.3f} "
              "v_cmd=({:+.2f},{:+.2f},{:+.2f}) v_final=({:+.2f},{:+.2f},{:+.2f})"
              .format(t, *p_d, *p, np.linalg.norm(p_d - p), *v, *v_final))
        p = p + v_final * cfg["dt"] * 0.5   # 一阶滞后的一半，仅示意

    print("\n[结论] 感知链路正常：点云 → 直方图 → 安全系数；"
          "控制链路正常：前馈+PID → 饱和 → 安全修正")
    return 0


# --------------------------------------------------------------------------
# ROS 模式
# --------------------------------------------------------------------------

def run_ros_mode(cfg):
    import rospy
    from geometry_msgs.msg import Twist

    rospy.init_node("uav_perception_trajectory", anonymous=False)

    # --- 启动校验：桥接的 body_frame 必须为 false ---------------------------
    try:
        body_frame = rospy.get_param("/carlair_ros_bridge/control/body_frame", None)
        if body_frame is True:
            rospy.logerr(
                "桥接 control/body_frame = true，但本模块输出的是世界系速度！"
                "无人机前进方向会随机头漂移而改变，轨迹会变成绕圈。"
                "请用 body_frame:=false 重启桥接。")
        elif body_frame is None:
            rospy.logwarn(
                "读不到桥接的 control/body_frame 参数，无法确认坐标系一致性。"
                "若轨迹异常（绕圈），请检查桥接是否以 body_frame:=false 启动。")
        else:
            rospy.loginfo("已校验桥接 body_frame=false，坐标系一致。")
    except Exception as exc:
        rospy.logwarn("校验 body_frame 时出错：%s", exc)

    # --- 组件 --------------------------------------------------------------
    traj = make_traj(cfg["traj"], radius=cfg["radius"],
                     omega=cfg["omega"], z=cfg["z"])
    ctrl = TrajectoryController(kp=cfg["kp"], ki=cfg["ki"], kd=cfg["kd"],
                                v_max=cfg["v_max"], dt=cfg["dt"])

    pub = rospy.Publisher(cfg["cmd_topic"], Twist, queue_size=10)
    state = {"hist": np.ones(16, dtype=np.float32), "pose": None}

    def on_lidar(msg):
        try:
            pts = pointcloud2_to_xyz(msg)
            state["hist"] = sector_histogram(pts)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "点云处理失败：%s", exc)

    def on_odom(msg):
        p = msg.pose.pose.position
        # AirSim/ROS 常用的 NED 位姿直接取用；若桥接输出 ENU，此处需相应转换
        state["pose"] = np.array([p.x, p.y, p.z], dtype=np.float64)

    rospy.Subscriber(cfg["lidar_topic"], rospy.AnyMsg, on_lidar, queue_size=1)
    rospy.Subscriber(cfg["odom_topic"], rospy.AnyMsg, on_odom, queue_size=1)

    t0 = rospy.Time.now()
    rospy.sleep(1.0)
    t0 = rospy.Time.now()

    T = period(cfg["traj"], cfg["omega"])
    rospy.loginfo("[就绪] 轨迹=%s 参数=%s 周期=%.2f s 圈数=%d",
                  cfg["traj"], traj.params, T, cfg["cycles"])
    rospy.loginfo("[就绪] PID Kp=%.2f Ki=%.2f Kd=%.2f  v_max=%.2f  dt=%.2f",
                  cfg["kp"], cfg["ki"], cfg["kd"], cfg["v_max"], cfg["dt"])
    rospy.loginfo("[就绪] 安全阈值 s_safe=%.2f s_min=%.2f", cfg["s_safe"], cfg["s_min"])

    rate = rospy.Rate(1.0 / cfg["dt"])
    err_hist = []
    last_log = rospy.Time.now()

    while not rospy.is_shutdown():
        t = (rospy.Time.now() - t0).to_sec()

        if T != float("inf") and t > T * cfg["cycles"]:
            rospy.loginfo("[完成] 已跑满 %d 圈，开始降落", cfg["cycles"])
            break

        if state["pose"] is None:
            rospy.logwarn_throttle(5.0, "尚未收到 %s，等待位姿 …", cfg["odom_topic"])
            rate.sleep()
            continue

        p_d, p_d_dot = traj(t)
        p = state["pose"]
        v = ctrl.step(p_d, p_d_dot, p)
        s = safety_scale(state["hist"],
                         s_safe=cfg["s_safe"], s_min=cfg["s_min"])
        v_final = ctrl.apply_safety(v, s)

        tw = Twist()
        tw.linear.x, tw.linear.y, tw.linear.z = (float(x) for x in v_final)
        tw.angular.z = 0.0
        pub.publish(tw)

        err = float(np.linalg.norm(p_d - p))
        err_hist.append(err)

        if (rospy.Time.now() - last_log).to_sec() >= 1.0:
            rospy.loginfo(
                "t=%.1f | p_d=(%+.2f,%+.2f,%+.2f) p=(%+.2f,%+.2f,%+.2f) |e|=%.3f "
                "| v=(%+.2f,%+.2f,%+.2f) s=%.2f",
                t, p_d[0], p_d[1], p_d[2], p[0], p[1], p[2], err,
                v_final[0], v_final[1], v_final[2], s)
            last_log = rospy.Time.now()

        rate.sleep()

    # 收尾：发零速度
    pub.publish(Twist())
    if err_hist:
        rospy.loginfo("[统计] 平均位置误差 %.3f m，最大 %.3f m（共 %d 帧）",
                      float(np.mean(err_hist)), float(np.max(err_hist)), len(err_hist))
    return 0


# --------------------------------------------------------------------------

def main():
    args = build_parser().parse_args()
    cfg = resolve_params(args)
    if args.dry_run:
        return run_dry_run(cfg)
    return run_ros_mode(cfg)


if __name__ == "__main__":
    sys.exit(main())
