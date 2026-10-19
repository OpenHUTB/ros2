#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""并发回归：第二个连接所在线程渲染时 OpenGL 上下文创建失败（单客户端测不出）。

    IndexError: index 4188928 is out of bounds for axis 0 with size 59
    python -m pytest test/test_concurrency.py -v
"""
import os
import subprocess
import sys
import threading
import time

import numpy as np
import pytest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG_ROOT)

from uav_mujoco_sim.sim import SimClient                    # noqa: E402
from uav_mujoco_sim.sim.client import wait_for_server       # noqa: E402

PORT = 41523
HAS_DISPLAY = bool(os.name == "nt" or os.environ.get("DISPLAY")
                   or os.environ.get("WAYLAND_DISPLAY"))


@pytest.fixture(scope="module")
def server():
    if not HAS_DISPLAY:
        pytest.skip("无显示环境，无法创建渲染器")
    env = dict(os.environ, PYTHONPATH=PKG_ROOT)
    p = subprocess.Popen(
        [sys.executable, "-m", "uav_mujoco_sim.sim", "--port", str(PORT),
         "--scene", "urban_block", "--seed", "3",
         "--width", "320", "--height", "240"],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, cwd=PKG_ROOT)
    try:
        wait_for_server("127.0.0.1", PORT, timeout=60)
        yield p
    finally:
        p.terminate()
        try:
            p.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            p.kill()


def _valid_seg(seg):
    """语义类别须落在 0~8；天文数字说明渲染上下文错乱。"""
    ids = [int(i) for i in np.unique(seg)]
    return all(0 <= i <= 8 for i in ids), ids


def test_second_client_can_render_segmentation(server):
    """第二个连接请求分割不应崩溃（原 bug 的精确触发条件）。"""
    c1 = wait_for_server("127.0.0.1", PORT, timeout=30)
    errs = []
    stop = threading.Event()

    def keep_rendering():
        while not stop.is_set():
            try:
                c1.frame(n=1, want=["odom", "rgb", "depth"])
            except Exception as e:                        # noqa: BLE001
                errs.append(repr(e))
                return

    t = threading.Thread(target=keep_rendering, daemon=True)
    t.start()
    time.sleep(2)

    try:
        c2 = SimClient("127.0.0.1", PORT)
    except OSError as e:
        stop.set()
        c1.close()
        pytest.fail("第二个客户端连不上：%s" % e)

    try:
        for k in range(5):
            r = c2.frame(n=1, want=["odom", "rgb", "depth", "seg", "lidar"])
            ok, ids = _valid_seg(r["seg"])
            assert ok, "第 %d 次分割出现非法类别：%s" % (k + 1, ids)
        assert not errs, "常驻客户端报错：%s" % errs[:2]
    finally:
        stop.set()
        t.join(timeout=15)
        c1.close()
        c2.close()


def test_three_clients_concurrently(server):
    """三个客户端并发读写，全部请求都应有正确响应。"""
    clients = [SimClient("127.0.0.1", PORT) for _ in range(3)]
    wants = [["odom", "rgb", "depth"],
             ["odom", "seg"],
             ["odom", "rgb", "depth", "seg", "lidar"]]
    errors = []

    def spam(c, want, tag):
        for _ in range(15):
            try:
                r = c.frame(n=1, want=want)
                if "seg" in want:
                    ok, ids = _valid_seg(r["seg"])
                    if not ok:
                        errors.append((tag, "非法类别", ids))
                        return
            except Exception as e:                        # noqa: BLE001
                errors.append((tag, type(e).__name__, str(e)[:60]))
                return

    threads = [threading.Thread(target=spam, args=(c, w, "c%d" % i),
                                daemon=True)
               for i, (c, w) in enumerate(zip(clients, wants), start=1)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=180)
    for c in clients:
        c.close()
    assert not errors, errors[:3]


def test_protocol_roundtrip_arrays():
    """协议编解码须逐元素一致且 dtype 不变（曾漏读二进制段致图像全空）。"""
    from uav_mujoco_sim.sim.protocol import encode, decode, blob_array
    blobs = {
        "rgb": np.arange(3 * 4 * 5, dtype=np.uint8).reshape(4, 5, 3),
        "depth": np.linspace(0.1, 9.9, 20, dtype=np.float32).reshape(4, 5),
        "empty": np.zeros((0, 3), dtype=np.float32),
    }
    head, payload = decode(encode({"ok": True, "blobs": dict(blobs)}))
    for k, v in blobs.items():
        got = blob_array(head, payload, k)
        assert np.array_equal(got, v), k
        assert got.dtype == v.dtype, k
