#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""妯″潡缁熶竴鍏ュ彛銆?
    python3 main.py --list                 鍒楀嚭鍏ㄩ儴妯″紡
    python3 main.py --mode keyboard        浠诲姟1锛氶敭鐩橀仴鎺?    python3 main.py --mode perception      浠诲姟2锛氭縺鍏夋劅鐭?+ 鍙嶅簲寮忛伩闅?    python3 main.py --mode nav             浠诲姟3锛氭爡鏍煎缓鍥?+ A* 瀵艰埅
    python3 main.py --mode nn              浠诲姟4锛氱缁忕綉缁滅瓥鐣ラ琛?    python3 main.py --mode record          浠诲姟4锛氶噰闆?(鎵弿, 鍔ㄤ綔) 鏁版嵁
    python3 main.py --mode train           浠诲姟4锛氳缁冪綉缁滐紙绂荤嚎锛屼笉闇€瑕佹ā鎷熷櫒锛?
闄?`train` 澶栵紝鍏朵綑妯″紡閮戒細鍚姩 airsim_bridge锛岄渶瑕?OpenHUTB 宸茶繍琛屻€?"""

import argparse
import os
import subprocess
import sys

PKG = "uav_openhutb_lidar_nn_nav"

MODES = {
    "keyboard": ("浠诲姟1 閿洏閬ユ帶", "launch/main.launch", "mode:=keyboard"),
    "perception": ("浠诲姟2 婵€鍏夋劅鐭?+ 鍙嶅簲寮忛伩闅?, "launch/main.launch", "mode:=perception"),
    "nav": ("浠诲姟3 鏍呮牸寤哄浘 + A* 瀵艰埅", "launch/main.launch", "mode:=nav"),
    "nn": ("浠诲姟4 绁炵粡缃戠粶绛栫暐椋炶", "launch/main.launch", "mode:=nn"),
    "record": ("浠诲姟4 閲囬泦璁粌鏁版嵁", "launch/record.launch", ""),
}


def show_list():
    print("=" * 62)
    print(" uav_openhutb_lidar_nn_nav 鈥斺€?OpenHUTB 鏃犱汉鏈轰豢鐪熸ā鍧?)
    print("=" * 62)
    for k, (name, launch, _) in MODES.items():
        print(" --mode %-11s %s" % (k, name))
    print(" --mode train        浠诲姟4 璁粌缃戠粶锛堢绾匡級")
    print("")
    print(" 绀轰緥锛歱ython3 main.py --mode keyboard")
    print("=" * 62)


def run_train(args):
    here = os.path.dirname(os.path.abspath(__file__))
    cmd = [sys.executable, os.path.join(here, "scripts", "nn_train.py")] + args
    return subprocess.call(cmd)


def main():
    ap = argparse.ArgumentParser(description="OpenHUTB 鏃犱汉鏈烘ā鍧楀叆鍙?)
    ap.add_argument("--mode", choices=list(MODES.keys()) + ["train"])
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--backend", default="openhutb", choices=["openhutb", "mock"],
                    help="openhutb=杩炴帴瀛︽牎妯℃嫙鍣紱mock=鏈湴鍑犱綍浠跨湡锛堟棤闇€瀹夎妯℃嫙鍣級")
    ap.add_argument("--airsim-ip", default=None, help="OpenHUTB 鎵€鍦ㄤ富鏈?IP")
    ap.add_argument("--dry-run", action="store_true", help="鍙墦鍗板懡浠?)
    args, extra = ap.parse_known_args()

    if args.list or not args.mode:
        show_list()
        return 0

    if args.mode == "train":
        return run_train(extra)

    name, launch, mode_arg = MODES[args.mode]
    cmd = ["roslaunch", PKG, launch]
    if mode_arg:
        cmd.append(mode_arg)
    cmd.append("backend:=%s" % args.backend)
    if args.airsim_ip:
        cmd.append("airsim_ip:=%s" % args.airsim_ip)
    cmd += extra

    print("[main] 妯″紡: %s" % name)
    print("[main] 鎵ц: %s" % " ".join(cmd))
    if args.dry_run:
        return 0
    try:
        return subprocess.call(cmd)
    except FileNotFoundError:
        sys.stderr.write("[main] 鎵句笉鍒?roslaunch锛岃鍏?source /opt/ros/noetic/setup.bash\n")
        return 127


if __name__ == "__main__":
    sys.exit(main())
