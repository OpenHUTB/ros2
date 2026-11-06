#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MoveIt 六自由度机械臂批量规划评估。

读取 targets.yaml 中的末端目标位姿，对指定的 OMPL 规划器重复规划，
记录成功率、规划耗时、关节路径长度和末端位姿误差，并写出逐次 CSV 与汇总 CSV。
"""

import argparse
import csv
import math
import os
import sys
import time

import rospy
import yaml
import moveit_commander

from geometry_msgs.msg import Pose
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest


DEFAULT_PLANNERS = [
    "RRTConnectkConfigDefault",
    "RRTstarkConfigDefault",
    "PRMkConfigDefault",
]

DEFAULT_FRAME = "base_link"
DEFAULT_EE_LINK = "grasping_frame"

ATTEMPT_FIELDS = [
    "planner",
    "target",
    "repeat",
    "success",
    "planning_time",
    "wall_time",
    "num_points",
    "joint_path_length",
    "position_error",
    "orientation_error",
    "error_code",
]

SUMMARY_FIELDS = [
    "planner",
    "attempts",
    "successes",
    "success_rate",
    "mean_planning_time",
    "p95_planning_time",
    "mean_joint_path_length",
    "mean_position_error",
    "mean_orientation_error",
]


def parse_args(argv):
    parser = argparse.ArgumentParser(description="MoveIt 六自由度机械臂规划器对比")
    parser.add_argument("--targets", required=True, help="目标位姿 YAML")
    parser.add_argument("--planners", default="", help="逗号分隔的规划器 ID；留空用默认三种")
    parser.add_argument("--planners-file", default="", help="可选：planners.yaml 路径")
    parser.add_argument("--repeats", type=int, default=3, help="每个目标的重复次数")
    parser.add_argument("--timeout", type=float, default=5.0, help="单次规划超时（秒）")
    parser.add_argument("--out-dir", required=True, help="结果输出目录")
    parser.add_argument("--frame", default=DEFAULT_FRAME, help="目标位姿参考坐标系")
    parser.add_argument("--ee-link", default=DEFAULT_EE_LINK, help="末端执行器 link")
    parser.add_argument("--no-fk", action="store_true", help="跳过 /compute_fk，末端误差留空")
    # roslaunch 会附加 __name:=...、__log:=... 等 ROS 参数，
    # 用 parse_known_args 忽略它们，避免 argparse 报 unrecognized arguments。
    args, _ = parser.parse_known_args(argv)
    return args


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def make_pose(item):
    position = item.get("position")
    orientation = item.get("orientation")
    if not position or len(position) != 3:
        raise ValueError("目标 %s 缺少合法的 position" % item.get("name", "?"))
    if not orientation or len(orientation) != 4:
        raise ValueError("目标 %s 缺少合法的 orientation" % item.get("name", "?"))

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = [float(v) for v in position]
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = [float(v) for v in orientation]
    return pose


def to_seconds(value):
    if value is None:
        return None
    if hasattr(value, "to_sec"):
        return float(value.to_sec())
    return float(value)


def parse_plan_result(result):
    """兼容 MoveIt 1 不同版本的 plan() 返回值。"""
    if isinstance(result, tuple):
        if len(result) == 4:
            success, trajectory, planning_time, error_code = result
        elif len(result) == 3:
            trajectory, planning_time, error_code = result
            error_value = getattr(error_code, "val", error_code)
            success = error_code is None or error_value == MoveItErrorCodes.SUCCESS
        elif len(result) == 2:
            trajectory, planning_time = result
            error_code = None
            success = trajectory is not None
        else:
            raise ValueError("plan() 返回了 %d 个值，无法解析" % len(result))
    else:
        trajectory = result
        planning_time = None
        error_code = None
        success = trajectory is not None

    if success and (
        trajectory is None
        or not getattr(trajectory.joint_trajectory, "points", [])
    ):
        success = False
    return bool(success), trajectory, to_seconds(planning_time), error_code


def error_code_value(error_code):
    if error_code is None:
        return ""
    if hasattr(error_code, "val"):
        return error_code.val
    return error_code


def joint_path_length(trajectory):
    points = trajectory.joint_trajectory.points
    total = 0.0
    for previous, current in zip(points, points[1:]):
        total += sum(
            abs(a - b)
            for a, b in zip(previous.positions, current.positions)
        )
    return total


def quaternion_angle(q1, q2):
    dot = abs(
        q1.x * q2.x
        + q1.y * q2.y
        + q1.z * q2.z
        + q1.w * q2.w
    )
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


def make_fk_client(timeout):
    try:
        rospy.wait_for_service("/compute_fk", timeout=timeout)
        return rospy.ServiceProxy("/compute_fk", GetPositionFK)
    except rospy.ROSException:
        rospy.logwarn("没有找到 /compute_fk，末端误差将留空")
        return None


def compute_fk(client, frame, link, joint_names, positions):
    request = GetPositionFKRequest()
    request.header.frame_id = frame
    request.fk_link_names = [link]
    request.robot_state.joint_state.name = list(joint_names)
    request.robot_state.joint_state.position = [float(v) for v in positions]
    try:
        response = client(request)
    except rospy.ServiceException as exc:
        rospy.logwarn("compute_fk 调用失败：%s", exc)
        return None
    if response.error_code.val != MoveItErrorCodes.SUCCESS:
        return None
    if not response.pose_stamped:
        return None
    return response.pose_stamped[0].pose


def percentile(values, ratio):
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * ratio
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - position) + ordered[upper] * (position - lower)


def mean(values):
    return sum(values) / float(len(values)) if values else None


def write_attempts(path, rows):
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=ATTEMPT_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def write_summary(path, rows):
    grouped = {}
    for row in rows:
        item = grouped.setdefault(
            row["planner"],
            {
                "attempts": 0,
                "successes": 0,
                "planning_time": [],
                "joint_path_length": [],
                "position_error": [],
                "orientation_error": [],
            },
        )
        item["attempts"] += 1
        if row["success"]:
            item["successes"] += 1
        for key in ("planning_time", "joint_path_length", "position_error", "orientation_error"):
            value = row.get(key)
            if value not in (None, ""):
                item[key].append(float(value))

    summary = []
    for planner, item in grouped.items():
        summary.append(
            {
                "planner": planner,
                "attempts": item["attempts"],
                "successes": item["successes"],
                "success_rate": item["successes"] / float(item["attempts"]),
                "mean_planning_time": mean(item["planning_time"]),
                "p95_planning_time": percentile(item["planning_time"], 0.95),
                "mean_joint_path_length": mean(item["joint_path_length"]),
                "mean_position_error": mean(item["position_error"]),
                "mean_orientation_error": mean(item["orientation_error"]),
            }
        )

    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        writer.writerows(summary)
    return summary


def main(argv):
    args = parse_args(argv)

    targets_doc = load_yaml(args.targets)
    targets = targets_doc.get("targets") or []
    if not targets:
        raise SystemExit("targets.yaml 里没有 targets 列表")

    if args.planners:
        planners = [item.strip() for item in args.planners.split(",") if item.strip()]
    elif args.planners_file:
        planners = load_yaml(args.planners_file).get("planners") or DEFAULT_PLANNERS
    else:
        planners = list(DEFAULT_PLANNERS)

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("plan_eval", anonymous=True)

    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_pose_reference_frame(args.frame)
    arm.set_end_effector_link(args.ee_link)
    arm.set_planning_time(args.timeout)
    arm.set_num_planning_attempts(10)

    fk_client = None if args.no_fk else make_fk_client(5.0)

    try:
        arm.set_named_target("home")
        arm.go(wait=True)
    except Exception as exc:
        rospy.logwarn("回到 home 失败，继续使用当前状态：%s", exc)
    arm.clear_pose_targets()

    os.makedirs(args.out_dir, exist_ok=True)
    attempts_path = os.path.join(args.out_dir, "plan_attempts.csv")
    summary_path = os.path.join(args.out_dir, "plan_summary.csv")

    rows = []
    for planner in planners:
        arm.set_planner_id(planner)
        rospy.loginfo("开始规划器：%s", planner)

        for target in targets:
            name = target.get("name", "unnamed")
            pose = make_pose(target)

            for repeat in range(1, args.repeats + 1):
                arm.set_start_state_to_current_state()
                arm.set_pose_target(pose)

                started = time.time()
                try:
                    result = arm.plan()
                except Exception as exc:
                    rospy.logwarn("%s / %s #%d 规划异常：%s", planner, name, repeat, exc)
                    result = None
                wall_time = time.time() - started
                arm.clear_pose_targets()

                success, trajectory, planning_time, error_code = parse_plan_result(result)

                row = {
                    "planner": planner,
                    "target": name,
                    "repeat": repeat,
                    "success": int(success),
                    "planning_time": planning_time if planning_time is not None else wall_time,
                    "wall_time": wall_time,
                    "num_points": 0,
                    "joint_path_length": "",
                    "position_error": "",
                    "orientation_error": "",
                    "error_code": error_code_value(error_code),
                }

                if success:
                    trajectory_message = trajectory.joint_trajectory
                    row["num_points"] = len(trajectory_message.points)
                    row["joint_path_length"] = joint_path_length(trajectory)

                    if fk_client is not None and trajectory_message.points:
                        final_pose = compute_fk(
                            fk_client,
                            args.frame,
                            args.ee_link,
                            trajectory_message.joint_names,
                            trajectory_message.points[-1].positions,
                        )
                        if final_pose is not None:
                            dx = final_pose.position.x - pose.position.x
                            dy = final_pose.position.y - pose.position.y
                            dz = final_pose.position.z - pose.position.z
                            row["position_error"] = math.sqrt(dx * dx + dy * dy + dz * dz)
                            row["orientation_error"] = quaternion_angle(
                                final_pose.orientation, pose.orientation
                            )

                rows.append(row)
                rospy.loginfo(
                    "%s / %s #%d success=%s time=%.3f s",
                    planner,
                    name,
                    repeat,
                    success,
                    row["planning_time"],
                )

    write_attempts(attempts_path, rows)
    summary = write_summary(summary_path, rows)

    rospy.loginfo("逐次结果：%s", attempts_path)
    rospy.loginfo("汇总结果：%s", summary_path)
    for item in summary:
        rospy.loginfo(
            "%s: %d/%d, mean %.3f s, p95 %.3f s",
            item["planner"],
            item["successes"],
            item["attempts"],
            item["mean_planning_time"] or 0.0,
            item["p95_planning_time"] or 0.0,
        )

    moveit_commander.roscpp_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
