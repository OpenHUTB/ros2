#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
航点跟踪实验的连接自检（host/客户机侧运行，无需 roscore）

把“连不上模拟器”拆成几段分别判定，避免在“端口通”和“能控制飞机”之间误判：

  1. 读参数：默认读 config/params.yaml 的 airsim/ip、airsim/vehicle_name、
     airsim/timeout —— 与 pid_tracker.py / compare_builtin.py 用的是同一套参数；
  2. 测 TCP：socket.connect_ex 连 ip:41451，区分“超时/包被丢弃”（多为防火墙）、
     “连接被拒绝”（模拟器没起来或没进 AirSim 无人机模式）与“连通”；
  3. 发 RPC：用 airsim 客户端调一次 getMultirotorState（超时取 airsim/timeout），
     并打印这次调用实际耗时的秒数 —— `Request timed out` 就出现在这一步；
  4. 核对载具名，并用两次位置采样判断模拟器是否在推进时间（被 P 暂停时不会）。

退出码：全部通过为 0，任何一项不通过为 1，可直接当脚本的前置检查。

用法：
  python3 check_link.py                       # 用 config/params.yaml
  python3 check_link.py --ip 192.168.237.1    # 临时覆盖地址
  python3 check_link.py --params /path/params.yaml --json
"""

import argparse
import io
import json
import os
import socket
import sys
import time

if sys.version_info < (3, 7):        # Python 3.6 上保证中文输出不乱码
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8",
                                  errors="replace")
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8",
                                  errors="replace")

RPC_PORT = 41451
"""AirSim / OpenHUTB 模拟器的 RPC 端口"""

PARAMS_DEFAULT = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir, "config", "params.yaml")

TIMEOUT_EXC = ()
"""msgpackrpc 的超时异常，导入 airsim 之后填充"""


# ------------------------------------------------------------------ 输出小工具
class Report(object):
    """收集每一步的结论，最后一次性汇总"""

    def __init__(self):
        self.steps = []

    def ok(self, step, text):
        self.steps.append({"step": step, "ok": True, "text": text})
        print("[%s] %s" % (step, text))

    def bad(self, step, text, advice):
        self.steps.append({"step": step, "ok": False, "text": text,
                           "advice": advice})
        print("[%s] %s" % (step, text))
        print("      → %s" % advice)

    def note(self, text, indent=6):
        print(" " * indent + text)

    @property
    def passed(self):
        return all(step["ok"] for step in self.steps)

    @property
    def failed(self):
        return [step for step in self.steps if not step["ok"]]


# ------------------------------------------------------------------ 参数读取
def load_params(path):
    """读 config/params.yaml 的 airsim 段；读不到就返回空字典，走默认值"""
    path = os.path.abspath(path)
    if not os.path.isfile(path):
        return {}, "参数文件不存在：%s" % path
    try:
        import yaml
    except ImportError:
        return {}, "没有装 PyYAML，无法读取 %s（用 --ip/--vehicle 直接指定）" % path
    try:
        with open(path, "r") as fh:
            data = yaml.safe_load(fh) or {}
    except Exception as exc:                                  # noqa: BLE001
        return {}, "参数文件解析失败：%s" % exc
    airsim = data.get("airsim") or {}
    return airsim, None


# ------------------------------------------------------------------ 单项检查
def check_tcp(ip, port, report):
    """第 2 步：TCP 端口通不通，返回 (是否连通, 耗时秒)"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(3.0)
    t0 = time.time()
    try:
        code = sock.connect_ex((ip, port))
    except socket.gaierror as exc:
        report.bad("2/4", "地址无法解析：%s" % exc,
                   "检查 airsim/ip 是否写成了主机名或写错；"
                   "宿主机 VMnet8 地址用 ip route | grep default 查看")
        return False, time.time() - t0
    except OSError as exc:
        report.bad("2/4", "连接出错：%s" % exc,
                   "确认客户机与宿主机在同一网段（VMware NAT/仅主机模式）")
        return False, time.time() - t0
    finally:
        sock.close()

    cost = time.time() - t0
    if code == 0:
        report.ok("2/4", "TCP 端口连通（%s:%d，耗时 %.2f 秒）" % (ip, port, cost))
        return True, cost
    if code == 11 or code == 10060:
        report.bad("2/4", "TCP 连接超时（errno %d）：包被丢弃，通常是防火墙" % code,
                   "在宿主机放行 41451：New-NetFirewallRule -DisplayName "
                   "\"AirSim RPC 41451\" -Direction Inbound -Protocol TCP "
                   "-LocalPort 41451 -Action Allow -Profile Any；"
                   "若曾误点“取消”而生成 Block 规则，要先把该规则停用")
    elif code == 111 or code == 10061:
        report.bad("2/4", "TCP 连接被拒绝（errno %d）：该端口没有进程监听" % code,
                   "模拟器没启动，或停在 CARLA 车辆模式；"
                   "在模拟器窗口点一下让它获得焦点后按 Ctrl+V 切到 AirSim "
                   "无人机模式，再用 netstat -ano | findstr 41451 确认端口在听")
    else:
        report.bad("2/4", "TCP 连接失败（errno %d）" % code,
                   "核对 airsim/ip 是否为宿主机 VMnet8 地址、模拟器是否已启动")
    return False, cost


def check_rpc(ip, vehicle, timeout, report):
    """第 3 步：发一次 RPC（getMultirotorState）。

    返回 (状态对象, 客户端, 可用标志)；客户端建立失败时状态为 None。
    """
    try:
        import airsim
    except ImportError as exc:
        report.bad("3/4", "无法导入 airsim 客户端：%s" % exc,
                   "在客户机上装客户端（顺序不能反）：pip3 install numpy → "
                   "pip3 install msgpack-rpc-python → pip3 install airsim")
        return None, None, False
    except Exception as exc:                                  # noqa: BLE001
        report.bad("3/4", "airsim 客户端导入异常：%s" % exc,
                   "见 setup_and_connect.md 的客户端安装步骤")
        return None, None, False

    global TIMEOUT_EXC
    try:
        from msgpackrpc.error import TimeoutError as _TimeoutError
        TIMEOUT_EXC = (_TimeoutError,)
    except ImportError:
        TIMEOUT_EXC = ()

    report.note("airsim 客户端版本：%s" % getattr(airsim, "__version__", "未知"))
    client = airsim.MultirotorClient(ip=ip, timeout_value=timeout)

    t0 = time.time()
    try:
        state = client.getMultirotorState(vehicle_name=vehicle)
    except Exception as exc:                                  # noqa: BLE001
        cost = time.time() - t0
        if TIMEOUT_EXC and isinstance(exc, TIMEOUT_EXC):
            report.bad("3/4", "RPC 超时：%.1f 秒内没有应答（超时设置 %.1f 秒）"
                              % (cost, timeout),
                       "TCP 通但 RPC 不应答，按这个顺序试：① 切到无人机模式后"
                       "等几十秒，场景还在加载；② 在模拟器窗口按 P 恢复"
                       "（左上角 FPS 数字应持续跳动）；③ 重启模拟器，"
                       "上一次实验的 RPC 可能还堵着；"
                       "④ 把 config/params.yaml 的 airsim/timeout 再调大")
        else:
            report.bad("3/4", "RPC 调用失败（耗时 %.2f 秒）：%s" % (cost, exc),
                       "载具名不对会报 Vehicle API ... is not available，"
                       "用 listVehicles 看到的载具名覆盖 vehicle:=；"
                       "其余错误见第 13 节常见问题")
        return None, client, True
    cost = time.time() - t0
    report.ok("3/4", "RPC 应答正常（getMultirotorState 耗时 %.2f 秒，"
                     "超时设置 %.1f 秒）" % (cost, timeout))
    return state, client, True


def check_vehicle(client, state, vehicle, report):
    """第 4 步：核对载具名，并用两次采样判断模拟器是否在推进"""
    names = None
    try:
        raw = client.listVehicles()
        names = getattr(raw, "vehicle_names", raw)
        names = [str(n) for n in names]
    except Exception as exc:                                  # noqa: BLE001
        report.note("无法列出载具名（%s），跳过这一项核对" % exc)
    if names:
        if vehicle in names:
            report.ok("4/4", "载具名核对通过：模拟器载具列表 %s" % names)
        else:
            report.bad("4/4", "载具名对不上：模拟器里是 %s，参数里是 '%s'"
                              % (names, vehicle),
                       "用 vehicle:=%s 覆盖载具名（OpenHUTB 模拟器默认 "
                       "SimpleFlight，AirSim Blocks 场景是 Drone1）" % names[0])

    # 两次位置采样：被 P 暂停时位置完全不变
    pos = getattr(getattr(state, "kinematics_estimated", None), "position", None)
    if pos is None:
        return None
    first = (pos.x_val, pos.y_val, pos.z_val)
    time.sleep(0.5)
    try:
        state2 = client.getMultirotorState(vehicle_name=vehicle)
        pos2 = state2.kinematics_estimated.position
        second = (pos2.x_val, pos2.y_val, pos2.z_val)
    except Exception as exc:                                  # noqa: BLE001
        report.note("第二次位置采样失败（%s）" % exc)
        return first
    moved = sum((a - b) ** 2 for a, b in zip(first, second)) ** 0.5
    if moved < 1e-6:
        report.ok("4/4", "模拟器状态：悬停中，0.5 秒内位置变化 %.3f 米"
                         "（若按了 P 暂停会一直是 0）" % moved)
    else:
        report.ok("4/4", "模拟器状态：正常，0.5 秒内位置变化 %.3f 米" % moved)
    return first


# ------------------------------------------------------------------ 主流程
def main(argv=None):
    parser = argparse.ArgumentParser(
        description="航点跟踪实验的连接自检（host/客户机侧，无需 roscore）")
    parser.add_argument("--params", default=PARAMS_DEFAULT,
                        help="参数文件路径，默认 config/params.yaml")
    parser.add_argument("--ip", default=None, help="模拟器地址（宿主机 VMnet8）")
    parser.add_argument("--port", type=int, default=RPC_PORT, help="RPC 端口")
    parser.add_argument("--vehicle", default=None, help="载具名")
    parser.add_argument("--timeout", type=float, default=None,
                        help="单次 RPC 超时（秒）")
    parser.add_argument("--json", action="store_true", help="以 JSON 输出结论")
    args = parser.parse_args(argv)

    params, warning = load_params(args.params)
    ip = args.ip or params.get("ip") or "192.168.237.1"
    vehicle = args.vehicle or params.get("vehicle_name") or "SimpleFlight"
    timeout = args.timeout or float(params.get("timeout") or 30.0)

    report = Report()
    if not args.json:
        print("=" * 68)
        print("航点跟踪实验 · 连接自检")
        print("=" * 68)
        print("[1/4] 目标 %s:%d    载具 %s    RPC 超时 %.1f 秒"
              % (ip, args.port, vehicle, timeout))
        if warning:
            report.note("提示：%s" % warning)

    connected, _ = check_tcp(ip, args.port, report)
    state = None
    client = None
    if connected:
        state, client, _has_client = check_rpc(ip, vehicle, timeout, report)
    else:
        report.bad("3/4", "跳过：TCP 端口不通，RPC 不可能成功",
                   "先按上一行的结论处理端口问题")

    first = None
    if client is not None:
        first = check_vehicle(client, state, vehicle, report)
    elif state is not None:
        report.note("载具名与模拟器状态核对：跳过（没有可用的 airsim 客户端）")

    if not args.json:
        print("-" * 68)
        if report.passed:
            print("连接自检通过，可以运行 roslaunch uav_waypoint_tracking "
                  "main.launch")
        else:
            print("连接自检未通过：%d 项待处理（见上面的 → 提示）"
                  % len(report.failed))
        print("=" * 68)

    if args.json:
        print(json.dumps({
            "ip": ip,
            "port": args.port,
            "vehicle": vehicle,
            "timeout": timeout,
            "position_ned": ({"x": first[0], "y": first[1], "z": first[2]}
                             if first else None),
            "passed": report.passed,
            "steps": report.steps,
        }, ensure_ascii=False, indent=2))

    return 0 if report.passed else 1


if __name__ == "__main__":
    sys.exit(main())
