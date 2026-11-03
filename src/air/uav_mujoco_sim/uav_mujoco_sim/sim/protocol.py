#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仿真服务端线协议：客户端/服务端共用的编解码。

线格式 ``[4 字节大端 JSON 长度][JSON 头][二进制载荷]``；图像/点云等大数组走二进制载荷，
JSON 头记录 ``dtype / shape / offset / nbytes``，避免 base64 膨胀与浮点文本化精度损失。
"""
from __future__ import annotations

import json
import struct
from typing import Any, Dict, Tuple

import numpy as np

# 允许传输的 dtype 白名单，避免 pickle 反序列化风险
_DTYPE_MAP = {
    "uint8": np.uint8,
    "int32": np.int32,
    "int64": np.int64,
    "float32": np.float32,
    "float64": np.float64,
}


def encode(msg: Dict[str, Any]) -> bytes:
    """把含 numpy 数组的响应编码成字节流（原地修改 ``msg``）。

    ``msg["blobs"]`` 为 ``{名字: ndarray}``，数组写入二进制段并在头部记录
    ``dtype/shape/offset/nbytes``，其余键走 JSON。
    """
    blobs: Dict[str, np.ndarray] = msg.pop("blobs", {}) or {}
    meta: Dict[str, Dict[str, Any]] = {}
    payload = bytearray()
    for name, arr in blobs.items():
        a = np.ascontiguousarray(arr)
        meta[name] = {
            "dtype": a.dtype.name,
            "shape": list(a.shape),
            "offset": len(payload),
            "nbytes": int(a.nbytes),
        }
        payload.extend(a.tobytes())
    msg["blobs"] = meta
    head = json.dumps(msg, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    return struct.pack(">I", len(head)) + head + bytes(payload)


def decode(buf: bytes) -> Tuple[Dict[str, Any], bytes]:
    """从字节流解出 (JSON 头, 二进制载荷)。调用方自行按 offset 切片。"""
    if len(buf) < 4:
        raise ValueError("消息太短")
    (n,) = struct.unpack(">I", buf[:4])
    head = json.loads(buf[4:4 + n].decode("utf-8"))
    return head, buf[4 + n:]


def blob_array(head: Dict[str, Any], payload: bytes, name: str) -> np.ndarray | None:
    """按头部描述把指定二进制块还原成 numpy 数组；不存在返回 None。"""
    meta = (head.get("blobs") or {}).get(name)
    if not meta:
        return None
    dt = _DTYPE_MAP.get(meta["dtype"])
    if dt is None:
        raise ValueError(f"不支持的 dtype：{meta['dtype']}")
    o, nb = int(meta["offset"]), int(meta["nbytes"])
    return np.frombuffer(payload[o:o + nb], dtype=dt).reshape(meta["shape"]).copy()
