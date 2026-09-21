# -*- coding: utf-8 -*-
r"""
================================================================================
              仿真车选车与键盘遥控器  --  SELECT YOUR VEHICLE
================================================================================
连上正在运行的 OpenHUTB / AirSim 地面车辆模拟器，列出全部可驾驶车型，
你挑一辆（可带「备选车池」），随后自动打开第三人称键盘驾驶窗口。

用法示例:
    python main.py                    交互式选车（推荐）
    python main.py --list             只看车型清单
    python main.py --vehicle seal     直接开「比亚迪 海豹」
    python main.py -v mustang --pool cybertruck,tesla.model3
    python main.py --port 2000 --res 1600x900
    python main.py --dry-run          只选车，不启动驾驶窗口

说明:
    --vehicle 支持「关键词」，可以是蓝图 id 片段，也可以是中文名片段。
    没写 --vehicle 时进入菜单，输入编号选择；输入关键词可搜索。
    选中后可选若干「备选车」，开车途中按 Backspace 在它们之间轮换。

模拟器查找顺序: --simulator 参数 -> 环境变量 CARLA_ROOT ->
本目录向上若干级中的 CarlaUE4 安装。找不到且 2000 端口未开放时，
请先自行启动模拟器再运行本脚本。
================================================================================
"""

import argparse
import json
import os
import re
import socket
import subprocess
import sys
import tempfile
import time
import unicodedata

MODULE_DIR = os.path.dirname(os.path.abspath(__file__))
MANUAL_CONTROL = os.path.join(MODULE_DIR, "car_manual_control.py")
# 运行期数据放到系统临时目录，避免污染源码树
DATA_DIR = os.path.join(tempfile.gettempdir(), "car_keyboard_teleop")
CACHE_FILE = os.path.join(DATA_DIR, "vehicle_cache.json")
CHOICE_FILE = os.path.join(DATA_DIR, "vehicle_choice.json")
LIST_FILE = os.path.join(DATA_DIR, "vehicle_list.txt")

try:
    os.makedirs(DATA_DIR, exist_ok=True)
except OSError:
    pass

try:
    sys.stdout.reconfigure(errors="replace")
except Exception:
    pass

LINE = "=" * 78

# ------------------------------------------------------------------ 中文车名
CN_NAMES = {
    # ---- 四轮 ----
    "vehicle.audi.a2": "奥迪 A2",
    "vehicle.audi.etron": "奥迪 e-tron",
    "vehicle.audi.tt": "奥迪 TT",
    "vehicle.bmw.grandtourer": "宝马 2系 Gran Tourer",
    "vehicle.bmw.isetta": "宝马 Isetta 蛋车",
    "vehicle.byd.seal": "比亚迪 海豹",
    "vehicle.carlacola": "Carlacola 送货卡车",
    "vehicle.carlamotors.carlacola": "Carlacola 送货卡车",
    "vehicle.carlamotors.european_hgv": "欧洲重型卡车 HGV",
    "vehicle.carlamotors.firetruck": "消防车",
    "vehicle.chevrolet.impala": "雪佛兰 Impala",
    "vehicle.citroen.c3": "雪铁龙 C3",
    "vehicle.dodge.charger_2020": "道奇 Charger 2020",
    "vehicle.dodge.charger_police": "道奇 Charger 警车",
    "vehicle.dodge.charger_police_2020": "道奇 Charger 警车 2020",
    "vehicle.ford.ambulance": "福特 救护车",
    "vehicle.ford.crown": "福特 皇冠维多利亚",
    "vehicle.ford.mustang": "福特 野马 Mustang",
    "vehicle.jeep.wrangler_rubicon": "Jeep 牧马人 Rubicon",
    "vehicle.lincoln.mkz_2017": "林肯 MKZ 2017",
    "vehicle.lincoln.mkz_2020": "林肯 MKZ 2020",
    "vehicle.mercedes.coupe": "奔驰 Coupe",
    "vehicle.mercedes.coupe_2020": "奔驰 Coupe 2020",
    "vehicle.mercedes.sprinter": "奔驰 Sprinter 厢货",
    "vehicle.micro.microlino": "Microlino 微型电动车",
    "vehicle.mini.cooper_s": "Mini Cooper S",
    "vehicle.mini.cooper_s_2021": "Mini Cooper S 2021",
    "vehicle.mitsubishi.fusorosa": "三菱 Fuso Rosa 中巴",
    "vehicle.nissan.micra": "日产 Micra",
    "vehicle.nissan.patrol": "日产 途乐 Patrol",
    "vehicle.nissan.patrol_2021": "日产 途乐 Patrol 2021",
    "vehicle.seat.leon": "西雅特 Leon",
    "vehicle.tesla.cybertruck": "特斯拉 Cybertruck",
    "vehicle.tesla.model3": "特斯拉 Model 3",
    "vehicle.toyota.prius": "丰田 普锐斯 Prius",
    "vehicle.volkswagen.t2": "大众 T2 面包车",
    "vehicle.volkswagen.t2_2021": "大众 T2 2021",
    # ---- 两轮 ----
    "vehicle.bh.crossbike": "BH 越野自行车",
    "vehicle.diamondback.century": "Diamondback 公路自行车",
    "vehicle.gazelle.omafiets": "Gazelle 城市自行车",
    "vehicle.harley-davidson.low_rider": "哈雷 Low Rider 摩托",
    "vehicle.kawasaki.ninja": "川崎 Ninja 摩托",
    "vehicle.vespa.zx125": "Vespa ZX125 踏板摩托",
    "vehicle.yamaha.yzf": "雅马哈 YZF 摩托",
    # ---- DReyeVR 眼动仪车辆 ----
    "vehicle.dreyevr.egovehicle": "DReyeVR 自车",
    "vehicle.dreyevr.tesla.model3": "DReyeVR 特斯拉 Model 3",
    "vehicle.dreyevr.jeep.wrangler_rubicon": "DReyeVR Jeep 牧马人",
    "vehicle.dreyevr.ford.mustang": "DReyeVR 福特野马",
    "vehicle.dreyevr.vespa": "DReyeVR Vespa 踏板车",
}

GROUP_ORDER = [
    ("motorbike", "两轮 · 摩托车"),
    ("bicycle", "两轮 · 自行车"),
    ("car", "四轮 · 汽车"),
    ("van", "四轮 · 厢式货车"),
    ("truck", "四轮 · 卡车 / 特种车"),
    ("", "其他"),
]


# ------------------------------------------------------------------ 小工具
def cjk_width(text):
    """中文字符按 2 列宽计算。"""
    return sum(2 if unicodedata.east_asian_width(ch) in "WF" else 1 for ch in text)


def pad(text, width):
    return text + " " * max(0, width - cjk_width(text))


def pretty_name(vid):
    if vid in CN_NAMES:
        return CN_NAMES[vid]
    parts = [p for p in vid.split(".") if p]
    if len(parts) > 1:
        return " ".join(x.replace("_", " ").title() for x in parts[1:])
    return vid


def ask(prompt):
    try:
        return input(prompt).strip()
    except (EOFError, KeyboardInterrupt):
        print()
        return "0"


def port_is_open(host, port, timeout=1.0):
    sock = socket.socket()
    sock.settimeout(timeout)
    try:
        sock.connect((host, port))
        return True
    except Exception:
        return False
    finally:
        try:
            sock.close()
        except Exception:
            pass


# ------------------------------------------------------------------ 启动模拟器
SIM_EXE_REL = (
    os.path.join("CarlaUE4", "Binaries", "Win64", "CarlaUE4-Win64-Shipping.exe"),
    "CarlaUE4.exe",
)


def _check_exe(path):
    return path if path and os.path.isfile(path) else None


def find_simulator_exe(explicit=""):
    """按优先级查找模拟器可执行文件，找不到返回 None。"""
    found = _check_exe(explicit)
    if found:
        return found
    # 1) 环境变量 CARLA_ROOT
    roots = [os.environ.get("CARLA_ROOT", "")]
    # 2) 从本目录向上最多 5 级查找（兼容仓库与模拟器同盘的常见布局）
    cur = MODULE_DIR
    for _ in range(5):
        cur = os.path.dirname(cur)
        if not cur or cur == os.path.dirname(cur):
            break
        roots.append(cur)
    for root in roots:
        if not root:
            continue
        for rel in SIM_EXE_REL:
            found = _check_exe(os.path.join(root, rel))
            if found:
                return found
    return None


def start_simulator(port, width=1280, height=720, exe=""):
    """启动模拟器进程（不等待）。返回 (是否成功, 说明)。"""
    exe = find_simulator_exe(exe)
    if exe is None:
        return False, (
            "未找到模拟器可执行文件。请先手动启动 CarlaUE4 模拟器，"
            "或设置环境变量 CARLA_ROOT 指向模拟器安装目录，"
            "也可以用 --simulator 显式指定 CarlaUE4-Win64-Shipping.exe 路径。"
        )

    sim_root = os.path.dirname(exe)
    args = [exe]
    if exe.lower().endswith("carlaue4-win64-shipping.exe"):
        args.append("/Game/Carla/Maps/Town10HD")
    args += ["-windowed", "-ResX=%d" % width, "-ResY=%d" % height, "-carla-rpc-port=%d" % port]

    flags = 0
    if os.name == "nt":
        flags = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
    try:
        subprocess.Popen(args, cwd=sim_root, creationflags=flags,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as exc:
        return False, "启动失败：%s" % exc
    return True, exe


def ensure_server(host, port, wait_seconds=240, width=1280, height=720, exe=""):
    """确保模拟器在线：不在线就启动并等它就绪。"""
    if port_is_open(host, port):
        return True

    print()
    print("[启动] 端口 %d 上没有模拟器，尝试自动打开……" % port)
    ok, info = start_simulator(port, width, height, exe)
    if not ok:
        print("       [错误] %s" % info)
        return False
    print("       已启动：%s" % info)
    print("       首次加载要 1-3 分钟，黑屏属正常，别关那个窗口。")
    print()

    waited = 0
    while waited < wait_seconds:
        time.sleep(2)
        waited += 2
        if port_is_open(host, port, 0.6):
            print("[等待] 端口就绪（等了约 %d 秒），再给它几秒初始化……" % waited)
            time.sleep(5)
            return True
        if waited % 20 == 0:
            print("[等待] 已等 %d 秒……" % waited)

    print("[超时] %d 秒内端口 %d 没能就绪。" % (wait_seconds, port))
    print("       可降低分辨率（如 -ResX=640 -ResY=360）手动启动模拟器，等画面出来后再运行本程序。")
    return False


# ------------------------------------------------------------------ 采集车型
def collect_blueprints(library):
    items, seen = [], set()
    for bp in library.filter("vehicle.*"):
        vid = bp.id
        if ".wheel" in vid or ".light" in vid:
            continue
        if vid in seen:
            continue
        seen.add(vid)

        wheels, btype = 4, ""
        try:
            wheels = int(bp.get_attribute("number_of_wheels").as_int())
        except Exception:
            pass
        try:
            btype = bp.get_attribute("base_type").as_string().strip().lower()
        except Exception:
            btype = ""
        if not btype:
            btype = "motorbike" if wheels <= 2 else "car"

        items.append({"id": vid, "type": btype, "wheels": wheels})
    return items


def sort_items(items):
    order = [g[0] for g in GROUP_ORDER]
    return sorted(items, key=lambda x: (order.index(x["type"]) if x["type"] in order else 99, x["id"]))


def fetch_from_server(host, port, timeout):
    import logging

    logging.getLogger().setLevel(logging.WARNING)
    import carla

    client = carla.Client(host, port)
    client.set_timeout(timeout)
    world = client.get_world()
    items = sort_items(collect_blueprints(world.get_blueprint_library()))
    try:
        map_name = world.get_map().name
    except Exception:
        map_name = "?"
    meta = {"map": map_name, "time": time.strftime("%Y-%m-%d %H:%M:%S"), "online": True}
    return items, meta


def save_cache(items, meta):
    try:
        with open(CACHE_FILE, "w", encoding="utf-8") as fp:
            json.dump({"items": items, "meta": meta}, fp, ensure_ascii=False, indent=2)
    except Exception:
        pass


def load_cache():
    try:
        with open(CACHE_FILE, "r", encoding="utf-8") as fp:
            data = json.load(fp)
        items = data.get("items") or []
        if items:
            meta = dict(data.get("meta") or {})
            meta["online"] = False
            return items, meta
    except Exception:
        pass
    return None, None


def save_choice(vehicle_id, pool, res, host, port):
    try:
        with open(CHOICE_FILE, "w", encoding="utf-8") as fp:
            json.dump(
                {
                    "primary": vehicle_id,
                    "pool": pool,
                    "res": res,
                    "host": host,
                    "port": port,
                    "time": time.strftime("%Y-%m-%d %H:%M:%S"),
                },
                fp,
                ensure_ascii=False,
                indent=2,
            )
    except Exception:
        pass


def load_choice():
    try:
        with open(CHOICE_FILE, "r", encoding="utf-8") as fp:
            return json.load(fp)
    except Exception:
        return {}


def write_list_file(items):
    try:
        with open(LIST_FILE, "w", encoding="utf-8-sig") as fp:
            fp.write("OpenHUTB / AirSim 可驾驶车型清单（共 %d 种）\n" % len(items))
            fp.write("生成时间: %s\n\n" % time.strftime("%Y-%m-%d %H:%M:%S"))
            for btype, title in GROUP_ORDER:
                group = [x for x in items if x["type"] == btype]
                if not group:
                    continue
                fp.write("【%s】%d 种\n" % (title, len(group)))
                for it in group:
                    fp.write("    %s  %s  (%d 轮)\n" % (pad(pretty_name(it["id"]), 30), it["id"], it["wheels"]))
                fp.write("\n")
    except Exception:
        pass


# ------------------------------------------------------------------ 筛选 / 匹配
def matches(item, keyword):
    kw = keyword.lower()
    return kw in item["id"].lower() or kw in pretty_name(item["id"]).lower()


def resolve(items, spec):
    """把关键词解析成候选列表。"""
    s = (spec or "").strip().lower()
    if not s:
        return []
    exact = [x for x in items if x["id"].lower() == s]
    if exact:
        return exact
    return [x for x in items if matches(x, s)]


# ------------------------------------------------------------------ 显示
def show_menu(items, keyword="", note=""):
    view = [it for it in items if matches(it, keyword)] if keyword else items
    index_of = {it["id"]: i for i, it in enumerate(items, 1)}

    print()
    print(LINE)
    print("              车 型 选 择 器   （共 %d 种可驾驶车辆）" % len(items))
    if note:
        print("              %s" % note)
    print(LINE)

    if not view:
        print("\n    没有匹配「%s」的车型，换个关键词试试。" % keyword)
        print(LINE)
        return

    if keyword:
        print("    搜索「%s」→ 命中 %d 种：" % (keyword, len(view)))
        print()

    for btype, title in GROUP_ORDER:
        group = [it for it in view if it["type"] == btype]
        if not group:
            continue
        print("  【%s】 %d 种" % (title, len(group)))
        for it in group:
            print("  %3d  %s  %s  %d轮" % (index_of[it["id"]], pad(pretty_name(it["id"]), 28), pad(it["id"], 38), it["wheels"]))
        print()
    print(LINE)


# ------------------------------------------------------------------ 选车
def choose_from_menu(items):
    last = load_choice()
    last_id = last.get("primary", "")
    while True:
        show_menu(items, keyword="", note="")
        print("  输入 编号 选车 ｜ 输入 关键词 搜索 ｜ 回车 = 上次(%s) ｜ 0 = 退出"
              % (pretty_name(last_id) if last_id else "无"))
        raw = ask("  >> ")

        if raw == "":
            if last_id and any(x["id"] == last_id for x in items):
                return next(x for x in items if x["id"] == last_id)
            print("  [提示] 还没有上次选择，请输入编号。")
            continue
        if raw in ("0", "q", "Q"):
            return None
        if raw.isdigit():
            idx = int(raw)
            if 1 <= idx <= len(items):
                return items[idx - 1]
            print("  [提示] 编号超出范围（1 - %d）。" % len(items))
            continue
        # 关键词搜索
        hit = resolve(items, raw)
        if not hit:
            print("  [提示] 没有找到匹配「%s」的车型。" % raw)
            continue
        if len(hit) == 1:
            return hit[0]
        show_menu(items, keyword=raw, note="")
        pick = ask("  命中 %d 种，输入编号选一个（回车返回）: " % len(hit))
        if pick.isdigit():
            idx = int(pick)
            if 1 <= idx <= len(items):
                return items[idx - 1]


def choose_pool(items, primary):
    pool = []
    ans = ask("  要不要再加几辆「备选车」？（开车时按 Backspace 在它们之间轮换）[y/N]: ")
    if not ans.lower().startswith("y"):
        return pool
    while True:
        raw = ask("  输入备选编号（空格/逗号分隔，可多个，回车结束）: ")
        if not raw:
            break
        added = 0
        for token in re.split(r"[ ,，、]+", raw):
            if not token.isdigit():
                continue
            idx = int(token)
            if 1 <= idx <= len(items):
                vid = items[idx - 1]["id"]
                if vid != primary["id"] and vid not in pool:
                    pool.append(vid)
                    added += 1
        print("  已加入 %d 辆，当前备选车池：%s" % (added, "、".join(pretty_name(v) for v in pool) or "空"))
        if len(pool) >= 8:
            print("  备选车池已满（最多 8 辆）。")
            break
    return pool


# ------------------------------------------------------------------ 启动
def launch(vehicle, pool, res, host, port, dry_run=False):
    head = [sys.executable, "-u", MANUAL_CONTROL, "--host", host, "--port", str(port), "--res", res]
    if pool:
        cmd = head + ["--vehicle-pool", ",".join([vehicle["id"]] + pool)]
    else:
        cmd = head + ["--vehicle", vehicle["id"]]

    print()
    print(LINE)
    print("  你的车   : %s   (%s)" % (pretty_name(vehicle["id"]), vehicle["id"]))
    if pool:
        print("  备选车池 : %s" % "、".join(pretty_name(v) for v in pool))
    print("  分辨率   : %s      服务器: %s:%s" % (res, host, port))
    print(LINE)
    print()

    if dry_run:
        print("  [dry-run] 未打开驾驶窗口。命令如下：")
        print("  " + " ".join('"%s"' % c if " " in c else c for c in cmd))
        return 0

    print("  正在启动驾驶窗口…… 关闭该窗口即结束驾驶。")
    print()
    try:
        return subprocess.call(cmd, cwd=os.path.dirname(MANUAL_CONTROL))
    except FileNotFoundError:
        print("  [错误] 找不到 manual_control.py：%s" % MANUAL_CONTROL)
        return 3


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(
        description="OpenHUTB 仿真车选车与键盘遥控器：列出车型 -> 选车 -> 打开键盘驾驶窗口",
        add_help=True)
    ap.add_argument("--host", default="127.0.0.1", help="模拟器地址 (默认 127.0.0.1)")
    ap.add_argument("--port", type=int, default=2000, help="CARLA RPC 端口 (默认 2000)")
    ap.add_argument("--res", default="1280x720", help="驾驶窗口分辨率 (默认 1280x720)")
    ap.add_argument("-v", "--vehicle", default="", help="直接指定车型：完整 id 或关键词/中文名")
    ap.add_argument("--pool", default="", help="备选车：逗号分隔的 id 或关键词，Backspace 轮换")
    ap.add_argument("--simulator", default="", help="模拟器可执行文件路径（缺省取 CARLA_ROOT 或自动查找）")
    ap.add_argument("--timeout", type=float, default=15.0, help="连接模拟器的超时秒数 (默认 15)")
    ap.add_argument("--list", action="store_true", help="只打印车型清单，不启动")
    ap.add_argument("--refresh", action="store_true", help="强制重新拉取车型（忽略缓存）")
    ap.add_argument("--no-autostart", action="store_true", help="模拟器没开时不要自动启动它")
    ap.add_argument("--dry-run", action="store_true", help="只选车，不启动驾驶窗口")
    args = ap.parse_args()

    if not os.path.isfile(MANUAL_CONTROL):
        print("[错误] 找不到键盘驾驶脚本：%s" % MANUAL_CONTROL)
        return 3

    # ---- 0. 确保模拟器在线（不在线就自动启动并等待）----
    try:
        sim_w, sim_h = [int(x) for x in args.res.lower().split("x")]
    except Exception:
        sim_w, sim_h = 1280, 720

    if not args.no_autostart and not port_is_open(args.host, args.port):
        if not ensure_server(args.host, args.port, width=sim_w, height=sim_h, exe=args.simulator):
            print()
            print(LINE)
            print("  模拟器没起来，没法选车。你可以：")
            print("    1) 手动启动 CarlaUE4 模拟器，等城市画面出现；")
            print("    2) 设置环境变量 CARLA_ROOT 指向模拟器安装目录后重试；")
            print("    3) 用 --simulator 指定 CarlaUE4-Win64-Shipping.exe 的完整路径。")
            print(LINE)
            return 2

    # ---- 1. 取车型列表 ----
    items, meta = None, None
    if not args.refresh:
        cached_items, cached_meta = load_cache()
        if cached_items and port_is_open(args.host, args.port):
            cached_items = None  # 端口通，还是去实时拉最新
        elif cached_items:
            items, meta = cached_items, cached_meta

    if items is None:
        print("[连接] 正在连接模拟器 %s:%d ..." % (args.host, args.port))
        last_exc = None
        for attempt in range(1, 4):
            try:
                items, meta = fetch_from_server(args.host, args.port, args.timeout)
                break
            except Exception as exc:
                last_exc = exc
                if attempt < 3:
                    print("       第 %d 次连接失败（%s），5 秒后重试……" % (attempt, exc))
                    time.sleep(5)

        if items is not None:
            save_cache(items, meta)
            write_list_file(items)
            print("       连接成功，地图 = %s，共 %d 种车型。" % (meta["map"], len(items)))
        else:
            cached_items, cached_meta = load_cache()
            if cached_items:
                print("       连接失败（%s）。" % last_exc)
                print("       改用本地缓存的车单（%s 拉取），车型可能与当前版本略有差异。"
                      % cached_meta.get("time", "?"))
                items, meta = cached_items, cached_meta
            else:
                print()
                print(LINE)
                print("  [错误] 连不上模拟器，也没有本地缓存可用的车单。")
                print()
                print("  请先启动模拟器：")
                print("     - 运行模拟器安装目录下的 CarlaUE4.exe（或 CarlaUE4/Binaries/Win64/ 下的主程序）；")
                print("     - 或设置 CARLA_ROOT 环境变量后重新运行本脚本。")
                print()
                print("  等城市画面出现、RPC 端口 %d 开放后，再运行本程序。" % args.port)
                print(LINE)
                return 2

    # ---- 2. 只列清单 ----
    if args.list:
        note = "实时读取（地图 %s）" % meta.get("map", "?") if meta.get("online") else \
               "离线缓存（%s）" % meta.get("time", "?")
        show_menu(items, note=note)
        print("  车型清单已写入：%s" % LIST_FILE)
        return 0

    # ---- 3. 选车 ----
    note = "" if meta.get("online") else "⚠ 离线缓存清单（%s），实际车型以在线为准" % meta.get("time", "?")
    pool_ids = []

    if args.vehicle:
        hit = resolve(items, args.vehicle)
        if not hit:
            print("[错误] 没有找到匹配「%s」的车型。" % args.vehicle)
            show_menu(items, note=note)
            return 4
        if len(hit) > 1:
            print("[提示] 「%s」匹配到 %d 种，请从下面挑一个（或用完整 id）。" % (args.vehicle, len(hit)))
            show_menu(items, keyword=args.vehicle, note=note)
            return 4
        vehicle = hit[0]
        if args.pool:
            for token in re.split(r"[ ,，]+", args.pool):
                sub = resolve(items, token)
                if len(sub) == 1 and sub[0]["id"] != vehicle["id"] and sub[0]["id"] not in pool_ids:
                    pool_ids.append(sub[0]["id"])
                elif len(sub) > 1:
                    print("[提示] 备选车「%s」匹配到多种，已跳过。" % token)
    else:
        show_menu(items, note=note)
        vehicle = choose_from_menu(items)
        if vehicle is None:
            print("已取消。")
            return 0
        print()
        print("  √ 已选：%s  (%s)" % (pretty_name(vehicle["id"]), vehicle["id"]))
        pool_ids = choose_pool(items, vehicle) if not args.dry_run else []

    save_choice(vehicle["id"], pool_ids, args.res, args.host, args.port)
    return launch(vehicle, pool_ids, args.res, args.host, args.port, args.dry_run)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n已取消。")
        sys.exit(0)
