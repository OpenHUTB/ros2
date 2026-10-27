#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""场景静态检查：不需要 MuJoCo，也不需要图形界面。

用途：在装 MuJoCo 之前先把 XML 里最容易出错的东西查一遍——
    - 名称引用是否齐全（Python 侧按名字取的每个对象是否都存在）；
    - 自车几何体是否都在 geom group 3（射线屏蔽组）；
    - 路面是否确实不参与碰撞；
    - XML 中的几何量与 config.py 的常量是否一致（轮半径、轮距、步长、轮速上限）；
    - "射线会不会打到自车"的几何预判（车轮横向位置 vs 车头射线起点）。

用法：
    python3 tools/check_scene.py
退出码：0 全部通过，1 存在问题。
"""

import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from mujoco_car_avoidance import config  # noqa: E402

# Python 侧按名字引用的对象（改 XML 时若漏改会在这里被拦下）
REQUIRED_BODY = ["car", "obstacle", "roadside_object"]
REQUIRED_GEOM = ["obstacle_geom", "roadside_geom"]
REQUIRED_SITE = [config.LIDAR_SITE_NAME]
REQUIRED_ACTUATOR = ["left_motor", "right_motor"]
REQUIRED_JOINT = ["car_root", "left_wheel_joint", "right_wheel_joint"]


def _mass_points(root):
    """收集显式给了 mass 的几何体在**自车坐标系**下的位置，返回 [(x, mass, name)]。

    仅用于质心位置检查（判断驱动轮是否承担足够重量），不参与其它校验。
    """
    points = []

    def walk(node, offset):
        for child in node:
            if child.tag == "body":
                pos = [float(v) for v in (child.get("pos") or "0 0 0").split()]
                child_offset = [offset[i] + pos[i] for i in range(3)]
                if child.get("name") == "car":
                    child_offset = [0.0, 0.0, 0.0]      # 以自车为参考系
                walk(child, child_offset)
            elif child.tag == "geom":
                mass = child.get("mass")
                if mass is None:
                    continue
                pos = [float(v) for v in (child.get("pos") or "0 0 0").split()]
                points.append((offset[0] + pos[0], float(mass), child.get("name") or "?"))
            else:
                walk(child, offset)

    worldbody = root.find("worldbody")
    if worldbody is not None:
        walk(worldbody, [0.0, 0.0, 0.0])
    return points


def main() -> int:
    scene = config.resolve_scene_path()
    if not scene.is_file():
        print("[FAIL] 找不到场景文件：{}".format(scene))
        return 1

    tree = ET.parse(str(scene))
    root = tree.getroot()
    problems = []
    notes = []

    # ---------------------------------------------------------------- 收集
    bodies = {b.get("name"): b for b in root.iter("body") if b.get("name")}
    # 自由关节写作 <freejoint>，铰链关节写作 <joint>，两者都要收进来
    joints = {j.get("name"): j for j in root.iter()
              if j.tag in ("joint", "freejoint") and j.get("name")}
    geoms = {g.get("name"): g for g in root.iter("geom") if g.get("name")}
    sites = {s.get("name"): s for s in root.iter("site") if s.get("name")}

    # 注意：<actuator> 是容器元素，执行器是它的子元素
    actuator_root = root.find("actuator")
    actuator_list = list(actuator_root) if actuator_root is not None else []
    actuators = {a.get("name"): a for a in actuator_list if a.get("name")}

    print("[场景] {}".format(scene))
    print("[统计] body={} joint={} geom={} site={} actuator={}".format(
        len(bodies), len(joints), len(geoms), len(sites), len(actuator_list)))

    # ---------------------------------------------------------------- 名称
    for name in REQUIRED_BODY:
        if name not in bodies:
            problems.append("缺少 body '{}'".format(name))
    for name in REQUIRED_GEOM:
        if name not in geoms:
            problems.append("缺少 geom '{}'".format(name))
    for name in REQUIRED_SITE:
        if name not in sites:
            problems.append("缺少 site '{}'".format(name))
    for name in REQUIRED_JOINT:
        if name not in joints:
            problems.append("缺少 joint '{}'".format(name))
    for name in REQUIRED_ACTUATOR:
        if name not in actuators:
            problems.append("缺少 actuator '{}'".format(name))

    # 重名检查：MuJoCo 不接受重名对象（会直接拒绝加载模型）。
    # 这一条能拦住"插入新几何体时忘记删掉旧的那条"这类编辑事故。
    name_groups = {
        "body": [n.get("name") for n in root.iter("body")],
        "joint": [n.get("name") for n in root.iter("joint")],
        "freejoint": [n.get("name") for n in root.iter("freejoint")],
        "geom": [n.get("name") for n in root.iter("geom")],
        "site": [n.get("name") for n in root.iter("site")],
        "actuator": [a.get("name") for a in actuator_list],
    }
    for tag, names in name_groups.items():
        present = [n for n in names if n]
        duplicates = sorted({n for n in present if present.count(n) > 1})
        if duplicates:
            problems.append("<{}> 中存在重名对象：{}（MuJoCo 会拒绝加载）".format(
                tag, "、".join(duplicates)))

    # ---------------------------------------------------------------- 执行器
    joint_names = set(joints)
    for actuator in actuator_list:
        target = actuator.get("joint")
        if target not in joint_names:
            problems.append("actuator '{}' 引用了不存在的 joint '{}'".format(
                actuator.get("name"), target))
    for name in REQUIRED_ACTUATOR:
        node = actuators.get(name)
        if node is None:
            continue
        if not node.get("ctrlrange"):
            problems.append("actuator '{}' 未设置 ctrlrange（MuJoCo 默认只有 ±1）".format(name))

        # 出力必须限幅：不限幅时阶跃指令会让伺服瞬间输出几百牛，
        # 实测 0.05 s 内把车加速到 2.77 m/s 并掀翻（upright=-1，车轮悬空 100% 滑移）。
        force_range = node.get("forcerange")
        if not force_range:
            problems.append(
                "actuator '{}' 未设置 forcerange：阶跃指令会把车弹飞并掀翻，"
                "建议 ±{:.0f} N".format(name, config.MAX_WHEEL_FORCE_N))
        else:
            upper = float(force_range.split()[1])
            if upper > config.MAX_WHEEL_FORCE_N + 1e-6:
                problems.append(
                    "actuator '{}' 的 forcerange 上限 {:.1f} N 大于 config.MAX_WHEEL_FORCE_N={:.1f} N".format(
                        name, upper, config.MAX_WHEEL_FORCE_N))
            else:
                notes.append("actuator '{}' 出力限幅 ±{:.0f} N（两个轮合计 {:.0f} N）".format(
                    name, upper, 2 * upper))

    # ---------------------------------------------------------------- group
    # 自车几何体的 group：允许由 <default class="ego"> 继承，但显式值不能是别的组
    for name, node in geoms.items():
        if node.get("class") == "ego" and node.get("group") not in (None, "3"):
            problems.append("geom '{}' 的显式 group={}，与自车屏蔽组 3 冲突".format(
                name, node.get("group")))

    default_ego = None
    for node in root.iter("default"):
        if node.get("class") == "ego":
            default_ego = node
    if default_ego is None:
        problems.append("缺少 <default class=\"ego\">，自车几何体无法统一进入 geom group 3")
    else:
        inner = default_ego.find("geom")
        if inner is None or inner.get("group") != "3":
            problems.append("<default class=\"ego\"> 中的 geom 未设置 group=\"3\"")

    # 射线可见性必须与 config.LIDAR_GEOM_GROUPS 自洽：
    # 障碍物要"可见"，地面/路面要"被屏蔽"，否则会出现假目标（打到路面）或丢目标（打到自车）。
    mask = config.LIDAR_GEOM_GROUPS
    expectations = [
        ("obstacle_geom", 0, True),    # 障碍物：必须能被射线看到
        ("roadside_geom", 0, True),    # 路侧干扰物：要能被看到，再靠本车道判定滤除
        ("floor", 1, False),           # 地面：必须屏蔽
        ("road", 1, False),            # 路面：必须屏蔽（俯仰时会被打到）
    ]
    for name, group, should_see in expectations:
        node = geoms.get(name)
        if node is None:
            continue
        actual = node.get("group")
        if actual != str(group):
            problems.append("geom '{}' 的 group 应为 {}（当前 {}）".format(name, group, actual))
            continue
        visible = bool(mask[group])
        if visible != should_see:
            problems.append(
                "geom '{}' 在 group {}，但 config.LIDAR_GEOM_GROUPS[{}]={}，"
                "射线可见性与预期（{}）不符".format(
                    name, group, group, mask[group], "可见" if should_see else "屏蔽"))

    road = geoms.get("road")
    if road is None:
        problems.append("缺少 geom 'road'")
    elif road.get("contype") != "0" or road.get("conaffinity") != "0":
        problems.append("路面未设置 contype=conaffinity=0，会与车轮发生碰撞")

    # 支撑接触**不应**设 margin：margin 会让接触在"尚未接触"时就产生力，
    # 车身会悬浮在弹性气垫上（实测静置 z 比轮半径高约 7 mm），一给驱动指令就被弹飞。
    # 接触稳定改由两条保证：出生时轮胎已有 2 mm 穿透 + 驱动轮出力限幅。
    floor = geoms.get("floor")
    if floor is not None:
        margin = floor.get("margin")
        try:
            margin_value = float(margin) if margin is not None else 0.0
        except ValueError:
            margin_value = 0.0
        if margin_value > 0.001:
            problems.append(
                "floor 设置了 margin={}：支撑接触不应有 margin（会让车身悬浮在弹性接触上、"
                "被驱动指令弹飞），请删除该属性".format(margin))
        else:
            notes.append("floor 未设 margin：支撑接触不应有 margin，"
                         "接触稳定由出生穿透 + 驱动轮出力限幅保证")

    # ---------------------------------------------------------------- 几何一致性
    wheel = geoms.get("left_wheel_geom")
    if wheel is not None:
        radius = float(wheel.get("size").split()[0])
        if abs(radius - config.WHEEL_RADIUS_M) > 1e-6:
            problems.append("车轮半径 XML={} 与 config.WHEEL_RADIUS_M={} 不一致".format(
                radius, config.WHEEL_RADIUS_M))

        car_body = bodies.get("car")
        if car_body is not None:
            car_z = float(car_body.get("pos").split()[2])
            penetration = radius - car_z
            if penetration < 0.0:
                problems.append(
                    "car 出生高度 {:.4f} 高于车轮半径 {:.4f}：出生时轮胎悬空，"
                    "接触会断续并引起空转/偏航".format(car_z, radius))
            else:
                notes.append("car 出生高度 {:.4f} -> 轮胎初始穿透 {:.1f} mm".format(
                    car_z, penetration * 1000))

    # ---------------------------------------------------------------- 支撑与牵引
    # 驱动轮必须承担足够的法向力。四点支撑（前后支撑轮都着地）是**静不定**的：
    # 求解器可能把重量几乎全给支撑轮，驱动轮几乎没有法向力 ——
    # 实测表现为车轮空转（滑移 75%）、车速只有指令的 25%。
    points = _mass_points(root)
    if points:
        total_mass = sum(mass for _x, mass, _name in points)
        com_x = sum(x * mass for x, mass, _name in points) / total_mass
        notes.append("自车质心（车体系）x={:+.3f} m，统计质量 {:.2f} kg".format(com_x, total_mass))
        if com_x < 0.015:
            problems.append(
                "自车质心 x={:+.3f} m 过于靠后（驱动轮轴在 x=0）：静定的三点支撑要求质心"
                "落在轮轴前方，否则驱动轮可能不承担重量、牵引力不足".format(com_x))

    car_body = bodies.get("car")
    if car_body is not None:
        car_z = float(car_body.get("pos").split()[2])
        for name, expect_airborne in (("caster", False), ("caster_rear", True)):
            node = geoms.get(name)
            if node is None:
                continue
            pos = [float(v) for v in node.get("pos").split()]
            radius = float(node.get("size").split()[0])
            clearance = car_z + pos[2] - radius
            if expect_airborne:
                if clearance < 0.002:
                    problems.append(
                        "'{}' 底面离地仅 {:.1f} mm：与驱动轮同时着地会形成静不定四点支撑，"
                        "建议抬高到 5 mm 以上".format(name, clearance * 1000))
                else:
                    notes.append("'{}' 底面离地 {:.1f} mm：静止时不承重，仅作防后仰保护".format(
                        name, clearance * 1000))
            elif clearance > 0.0:
                problems.append(
                    "'{}' 底面离地 {:.1f} mm：三点支撑缺少前支点，车体会前倾蹭地".format(
                        name, clearance * 1000))
            else:
                notes.append("'{}' 着地（穿透 {:.1f} mm），与两驱动轮构成三点支撑".format(
                    name, -clearance * 1000))

    left, right = bodies.get("left_wheel"), bodies.get("right_wheel")
    if left is not None and right is not None:
        separation = abs(float(left.get("pos").split()[1]) - float(right.get("pos").split()[1]))
        if abs(separation - config.WHEEL_SEPARATION_M) > 1e-6:
            problems.append("轮距 XML={} 与 config.WHEEL_SEPARATION_M={} 不一致".format(
                separation, config.WHEEL_SEPARATION_M))

    option = root.find("option")
    if option is None or option.get("timestep") is None:
        problems.append("<option> 缺少 timestep")
    elif abs(float(option.get("timestep")) - config.TIMESTEP) > 1e-9:
        problems.append("timestep XML={} 与 config.TIMESTEP={} 不一致".format(
            option.get("timestep"), config.TIMESTEP))

    for name in REQUIRED_ACTUATOR:
        node = actuators.get(name)
        if node is not None and node.get("ctrlrange"):
            limit = float(node.get("ctrlrange").split()[1])
            if abs(limit - config.MAX_WHEEL_OMEGA) > 1e-6:
                problems.append("执行器 {} 的 ctrlrange 上限 {} 与 config.MAX_WHEEL_OMEGA={} 不一致"
                                .format(name, limit, config.MAX_WHEEL_OMEGA))

    # ---------------------------------------------------------------- 射线几何预判
    if "car" in bodies and config.LIDAR_SITE_NAME in sites:
        car = bodies["car"]
        car_z = float(car.get("pos").split()[2])
        site_local = [float(v) for v in sites[config.LIDAR_SITE_NAME].get("pos").split()]
        sensor_x = site_local[0]
        if left is not None:
            wheel_x = float(left.get("pos").split()[0])
            if wheel_x < sensor_x:
                notes.append("车轮位于车头射线起点之后（x={} < {}），侧向射线不会打到自车".format(
                    wheel_x, sensor_x))
            else:
                problems.append("车轮位于射线起点之前，侧向射线可能打到自车")
        notes.append("射线起点离地高度约 {:.3f} m；水平射线不接触路面（路面顶面 z≈0）".format(
            car_z + site_local[2]))

    # ---------------------------------------------------------------- 输出
    for note in notes:
        print("  [NOTE] {}".format(note))
    if problems:
        print("\n[结果] 发现 {} 个问题：".format(len(problems)))
        for problem in problems:
            print("  [FAIL] {}".format(problem))
        return 1

    print("\n[结果] 场景静态检查通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
