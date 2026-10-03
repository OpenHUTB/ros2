#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
沿道路飞行：从 OpenHUTB 场景的 OpenDRIVE 地图（*.xodr）里取一条道路，
采样成航点，导出为本包的任务格式（供 pid_tracker.py 跟踪）。

为什么不用 CARLA 的 Python 客户端直接问路：本版 OpenHUTB（hutb v2.10.0）
在 AIR 游戏模式下 CARLA 的 episode 不可用，在 CARLA 游戏模式下响应里又含
非 UTF-8 字符串，客户端解码会失败。直接读地图文件最稳，也正好说明
"道路来自 OpenDrive 文件"这句话的字面含义。

用法（宿主机或虚拟机都可以，只依赖 Python 标准库）：

  # 1) 看看地图里有哪些路、哪条最长
  python3 road_mission.py --xodr .../Town10HD.xodr --list

  # 2) 检验几种坐标换算：无人机起飞点在 NED (0,0)，哪种换算能让它落在道路上
  python3 road_mission.py --xodr .../Town10HD.xodr --probe-home -0.02 1.51

  # 3) 导出某条道路的航点（写成本包 missions 格式的 YAML）
  python3 road_mission.py --xodr .../Town10HD.xodr --road 12 \
      --mapping swap --altitude 3.0 --home "-0.02 1.51" \
      --out ../../config/missions/road_town10hd.yaml --name road_town10hd

坐标说明：
  OpenDRIVE 用 x 向东、y 向北（右手系）；AirSim 的 NED 用 x 向北、y 向东。
  两者要么 (x, y) 直通，要么交换（--mapping direct / swap），符号用
  --flip-north / --flip-east 微调，先用 --probe-home 让它自己挑。
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET


# ------------------------------------------------------------------ 解析 xodr
def sample_geometry(geom, x0, y0, hdg, length, step=2.0):
    """把一段 geometry 采样成折线点，支持 line / arc，其余按直线近似"""
    n = max(2, int(math.ceil(length / step)) + 1)
    pts = []

    arc = geom.find("arc")
    curvature = float(arc.get("curvature")) if arc is not None else 0.0

    if abs(curvature) < 1e-9:
        for i in range(n):
            s = length * i / (n - 1)
            pts.append((x0 + s * math.cos(hdg), y0 + s * math.sin(hdg)))
        return pts

    # 圆弧：OpenDRIVE 的 hdg 是起点切线方向，曲率逆时针为正
    r = abs(1.0 / curvature)
    radius_dir = hdg + math.pi / 2.0 if curvature > 0 else hdg - math.pi / 2.0
    cx = x0 + r * math.cos(radius_dir)
    cy = y0 + r * math.sin(radius_dir)
    a0 = math.atan2(y0 - cy, x0 - cx)
    for i in range(n):
        a = a0 + (length * i / (n - 1)) * curvature
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def parse_xodr(path):
    """返回 {road_id: {'length': L, 'points': [(x, y), ...]}}（OpenDRIVE 坐标）"""
    root = ET.parse(path).getroot()
    roads = {}
    for road in root.findall("road"):
        rid = int(road.get("id"))
        pts = []
        # geometry 嵌在 <planView> 里，用 iter 递归取
        for geom in road.iter("geometry"):
            pts.extend(sample_geometry(geom,
                                       float(geom.get("x")),
                                       float(geom.get("y")),
                                       float(geom.get("hdg")),
                                       float(geom.get("length"))))
        roads[rid] = {"length": float(road.get("length")), "points": pts}
    return roads


# ------------------------------------------------------------- 坐标换算候选
def make_mapping(name, flip_north=False, flip_east=False):
    """OpenDRIVE (x 向东, y 向北) -> AirSim NED (x 向北, y 向东)"""
    def convert(px, py):
        if name == "swap":
            north, east = py, px        # odr 的 y 是北，x 是东
        else:
            north, east = px, py        # 直通
        if flip_north:
            north = -north
        if flip_east:
            east = -east
        return north, east
    return convert


def probe_home(roads, home_north, home_east, stride=4):
    """对每个候选换算，算起飞点到最近道路的距离（越接近 0 越说明换算是对的）"""
    results = []
    for name in ("direct", "swap"):
        for fn in (False, True):
            for fe in (False, True):
                conv = make_mapping(name, fn, fe)
                best_d = float("inf")
                for _rid, road in roads.items():
                    for px, py in road["points"][::stride]:
                        north, east = conv(px, py)
                        d = math.hypot(north - home_north, east - home_east)
                        if d < best_d:
                            best_d = d
                results.append((best_d, name, fn, fe))
    results.sort(key=lambda r: r[0])
    return results


def select_route(roads, conv, home):
    """挑离起飞点最近的 road，并从最近的那个点开始、朝剩余更长的一侧走"""
    home_north, home_east = home
    best = None
    for rid, road in roads.items():
        for idx, (px, py) in enumerate(road["points"][::4]):
            north, east = conv(px, py)
            d = math.hypot(north - home_north, east - home_east)
            if best is None or d < best[0]:
                best = (d, rid, idx * 4)
    dist, rid, idx = best
    pts = roads[rid]["points"]
    # 从最近点开始；朝较长的一侧延伸（保证航线尽量长）
    if len(pts) - idx >= idx:
        route = pts[idx:]
    else:
        route = list(reversed(pts[:idx + 1]))
    return rid, route, dist


# ------------------------------------------------------------------ 导出任务
def export_mission(roads, road_id, conv, home, altitude, name, out_path,
                   spacing=5.0, offset=(0.0, 0.0)):
    """offset=(北向偏移, 东向偏移)：标定起飞点与地图原点之间的残差"""
    road = roads[road_id]
    waypoints = []
    last = None
    acc = 0.0
    for px, py in road["points"]:
        north, east = conv(px, py)
        if last is None:
            waypoints.append((north, east))
            last = (north, east)
            continue
        acc += math.hypot(north - last[0], east - last[1])
        if acc >= spacing:
            waypoints.append((north, east))
            last = (north, east)
            acc = 0.0
    if last is not None and waypoints[-1] != last:
        waypoints.append(last)

    home_north, home_east = home
    lines = [
        "# 由 road_mission.py 从 OpenDRIVE 地图生成：%s" % name,
        "# road_id=%d，共 %d 个航点（每 %.0f 米一个），高度 %.1f 米"
        % (road_id, len(waypoints), spacing, altitude),
        "missions:",
        "  %s:" % name,
    ]
    for north, east in waypoints:
        # 任务航点用 ENU（x 东、y 北、z 上），且相对起飞点
        lines.append("    - [%.2f, %.2f, %.1f]"
                     % (east - home_east + offset[1],
                        north - home_north + offset[0], altitude))
    text = "\n".join(lines) + "\n"
    if out_path:
        with open(out_path, "w", encoding="utf-8") as fh:
            fh.write(text)
        print("已写出 %s（%d 个航点）" % (out_path, len(waypoints)))
    else:
        print(text)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xodr", required=True, help="OpenDRIVE 地图文件")
    ap.add_argument("--list", action="store_true", help="列出道路并按长度排序")
    ap.add_argument("--probe-home", nargs=2, type=float, metavar=("NORTH", "EAST"),
                    help="给出起飞点的 NED 坐标，检验各候选换算是否落在道路上")
    ap.add_argument("--road", type=int, help="要导出的 road_id")
    ap.add_argument("--nearest-to-home", action="store_true",
                    help="自动选离起飞点最近的道路，并从最近点开始")
    ap.add_argument("--mapping", default="swap", choices=("direct", "swap"))
    ap.add_argument("--flip-north", action="store_true")
    ap.add_argument("--flip-east", action="store_true")
    ap.add_argument("--home", default="0 0", help='起飞点 NED，如 "0 0"')
    ap.add_argument("--altitude", type=float, default=3.0)
    ap.add_argument("--spacing", type=float, default=5.0)
    ap.add_argument("--offset-north", type=float, default=0.0,
                    help="北向标定偏移（米）")
    ap.add_argument("--offset-east", type=float, default=0.0,
                    help="东向标定偏移（米）")
    ap.add_argument("--name", default="road")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    offset = (args.offset_north, args.offset_east)

    roads = parse_xodr(args.xodr)
    print("解析 %s：%d 条道路" % (args.xodr, len(roads)))

    if args.list:
        order = sorted(roads.items(), key=lambda kv: -kv[1]["length"])
        print("%-6s %-10s %s" % ("road", "长度(m)", "起点(odr x, y)"))
        for rid, road in order[:20]:
            x, y = road["points"][0]
            print("%-6d %-10.1f (%.1f, %.1f)" % (rid, road["length"], x, y))

    if args.probe_home:
        north, east = args.probe_home
        print("\n起飞点 NED=(%.2f, %.2f) 到最近道路的距离（越小越对）："
              % (north, east))
        for d, name, fn, fe in probe_home(roads, north, east):
            print("  %-6s flip_north=%-5s flip_east=%-5s -> %.2f m"
                  % (name, fn, fe, d))

    if args.nearest_to_home:
        conv = make_mapping(args.mapping, args.flip_north, args.flip_east)
        home = tuple(float(v) for v in args.home.split())
        rid, route, dist = select_route(roads, conv, home)
        print("\n离起飞点最近的道路 road_id=%d（直线距离 %.1f m，"
              "航线 %d 个原始采样点）" % (rid, dist, len(route)))
        export_mission({rid: {"length": 0.0, "points": route}}, rid, conv,
                       home, args.altitude, args.name, args.out, args.spacing,
                       offset)

    if args.road is not None:
        conv = make_mapping(args.mapping, args.flip_north, args.flip_east)
        home = tuple(float(v) for v in args.home.split())
        export_mission(roads, args.road, conv, home, args.altitude,
                       args.name, args.out, args.spacing, offset)

    return 0


if __name__ == "__main__":
    sys.exit(main())
