#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仿真服务端客户端：把 TCP 往返封装成 Python 调用。

只依赖 numpy + 标准库，因此 ROS 侧的 Python 3.8 也能直接用。
"""
from __future__ import annotations

import json
import socket
import struct
from typing import Any, Dict, Optional, Sequence

import numpy as np

from .protocol import blob_array, encode


class SimClient:
    """同步阻塞式客户端（一条连接一次一个请求）。"""

    def __init__(self, host: str = "127.0.0.1", port: int = 41460,
                 timeout: float = 30.0, connect_timeout: float = 10.0):
        self.host, self.port = host, port
        self.timeout = timeout
        self.sock = socket.create_connection((host, port), timeout=connect_timeout)
        self.sock.settimeout(timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    # ---------------------------------------------------------------- 底层
    def _send(self, req: Dict[str, Any]) -> Any:
        clean = {}
        for k, v in req.items():
            clean[k] = v.tolist() if isinstance(v, np.ndarray) else v
        self.sock.sendall(encode(clean))
        return self._recv()

    def _recv(self) -> tuple:
        """收一条完整消息：4 字节头长 + JSON 头 + 二进制载荷。

        载荷长度不在消息最前面，由 JSON 头的 blob 描述推出（``max(offset+nbytes)``）。
        """
        hdr = self._recv_exact(4)
        (n,) = struct.unpack(">I", hdr)
        head = json.loads(self._recv_exact(n).decode("utf-8"))
        blobs = head.get("blobs") or {}
        size = max((int(b["offset"]) + int(b["nbytes"]) for b in blobs.values()),
                   default=0)
        payload = self._recv_exact(size) if size > 0 else b""
        return head, payload

    def _recv_exact(self, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("仿真服务端已关闭连接")
            buf.extend(chunk)
        return bytes(buf)

    # ---------------------------------------------------------------- 接口
    def ping(self) -> bool:
        head, _ = self._send({"op": "ping"})
        return bool(head.get("ok"))

    def reset(self, pos: Optional[Sequence[float]] = None, yaw: float = 0.0,
              scene: Optional[str] = None, seed: Optional[int] = None) -> Dict[str, Any]:
        req: Dict[str, Any] = {"op": "reset", "yaw": float(yaw)}
        if pos is not None:
            req["pos"] = list(pos)
        if scene is not None:
            req["scene"] = scene
        if seed is not None:
            req["seed"] = int(seed)
        head, _ = self._send(req)
        return head

    def frame(self, n: int = 1, cmd_vel=None, goal=None, mode: Optional[str] = None,
              rotor=None, want: Sequence[str] = ("odom", "imu")) -> Dict[str, Any]:
        """推进 n 个控制周期并取一帧观测。

        图像/点云以 numpy 数组放在 ``rgb``/``depth``/``seg``/``lidar`` 键下，未请求的为 None。
        """
        req: Dict[str, Any] = {"op": "frame", "n": int(n), "want": list(want)}
        if cmd_vel is not None:
            req["cmd_vel"] = list(map(float, cmd_vel))
        if goal is not None:
            req["goal"] = list(map(float, goal))
        if mode is not None:
            req["mode"] = mode
        if rotor is not None:
            req["rotor"] = list(map(float, rotor))
        head, payload = self._send(req)
        for name in ("rgb", "depth", "seg", "lidar"):
            if name in want:
                head[name] = blob_array(head, payload, name)
        return head

    def observe(self, want: Sequence[str] = ("odom", "imu")) -> Dict[str, Any]:
        head, _ = self._send({"op": "observe", "want": list(want)})
        return head

    def render(self, want: Sequence[str] = ("rgb", "depth")) -> Dict[str, Any]:
        head, payload = self._send({"op": "render", "want": list(want)})
        for name in want:
            head[name] = blob_array(head, payload, name)
        return head

    def goto(self, goal: Sequence[float]) -> None:
        """设定目标点，后续 ``frame`` 持续朝目标飞行。"""
        self._send({"op": "frame", "n": 1, "goal": list(map(float, goal)),
                    "want": ["odom"]})

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


def wait_for_server(host: str = "127.0.0.1", port: int = 41460,
                    timeout: float = 20.0, interval: float = 0.3) -> SimClient:
    """轮询等待服务端就绪并返回已连接客户端（``ros2 launch`` 启动顺序不定）。"""
    import time
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        try:
            c = SimClient(host, port, connect_timeout=2.0)
            if c.ping():
                return c
            c.close()
        except OSError as e:
            last = e
            time.sleep(interval)
    raise TimeoutError(f"{timeout}s 内连不上仿真服务端 {host}:{port}（最后错误：{last}）")
