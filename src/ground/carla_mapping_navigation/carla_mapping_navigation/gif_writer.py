#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""纯 Python GIF89a 编码器（不依赖 Pillow）。

作业三/作业四的模块本身是纯 numpy 的（连 PNG 都是手写 zlib 导出的），
所以动图也自己写，保持模块零第三方依赖。

用法：
    frames = [np.zeros((H, W), dtype=np.uint8), ...]   # 调色板下标
    palette = [(0, 0, 0), (255, 255, 255), ...]        # 长度 ≤ 256
    write_gif("out.gif", frames, palette, delay_cs=8)
"""
import struct

__all__ = ["lzw_encode", "write_gif"]


def lzw_encode(indices, min_code_size):
    """GIF 的 LZW 编码，变长码、低位在前（LSB first）。

    字典键用 (前缀码, 下一个字节) 而不是元组字符串，避免长前缀反复哈希。
    """
    clear = 1 << min_code_size
    end = clear + 1

    out = bytearray()
    state = {"buf": 0, "cnt": 0, "code_size": min_code_size + 1}

    def emit(code):
        state["buf"] |= code << state["cnt"]
        state["cnt"] += state["code_size"]
        while state["cnt"] >= 8:
            out.append(state["buf"] & 0xFF)
            state["buf"] >>= 8
            state["cnt"] -= 8

    def reset():
        state["code_size"] = min_code_size + 1
        return end + 1

    table = {}
    next_code = reset()

    emit(clear)
    prefix = int(indices[0])
    for ch in indices[1:]:
        ch = int(ch)
        key = (prefix, ch)
        got = table.get(key)
        if got is not None:
            prefix = got
            continue
        emit(prefix)
        if next_code < 4096:
            table[key] = next_code
            next_code += 1
            # 注意是 `>` 而不是 `==`：解码端每读一个码才建一条表项，
            # 且第一个码不建表，所以它总比编码端慢一条。等号写法会让解码端
            # 在边界上少读一位，PIL 报 "image file is truncated"。
            if next_code > (1 << state["code_size"]) and state["code_size"] < 12:
                state["code_size"] += 1
        else:
            # 码表满：发 clear 让解码端一起重置
            emit(clear)
            table.clear()
            next_code = reset()
        prefix = ch

    emit(prefix)
    emit(end)
    if state["cnt"]:
        out.append(state["buf"] & 0xFF)
    return bytes(out)


def _sub_blocks(data):
    """把字节流切成 GIF 的子块（每块 ≤255 字节，最后跟一个 0x00 结束）。"""
    out = bytearray()
    for i in range(0, len(data), 255):
        chunk = data[i:i + 255]
        out.append(len(chunk))
        out += chunk
    out.append(0)
    return bytes(out)


def write_gif(path, frames, palette, delay_cs=8, loop=0, transparent=None):
    """写一个 GIF89a。

    frames    : 可迭代，每项是 (H,W) 的调色板下标数组（0..len(palette)-1）
    palette   : [(r,g,b), ...]，长度 ≤ 256，会自动补齐到 2 的幂
    delay_cs  : 每帧间隔，单位 1/100 秒（GIF 规范就是这么存的）
    loop      : 0 = 无限循环
    """
    frames = list(frames)
    if not frames:
        raise ValueError("至少要有一帧")

    n = len(palette)
    if n > 256:
        raise ValueError(f"调色板最多 256 色，给了 {n} 色")
    # 全局色表大小必须是 2 的幂，且至少 2
    bits = 1
    while (1 << bits) < n:
        bits += 1
    table_size = 1 << bits

    pal = bytearray()
    for i in range(table_size):
        r, g, b = palette[i] if i < n else (0, 0, 0)
        pal += bytes((r & 0xFF, g & 0xFF, b & 0xFF))

    h, w = len(frames[0]), len(frames[0][0])
    min_code_size = max(2, bits)

    gif = bytearray(b"GIF89a")
    # 逻辑屏幕描述符：GCT 存在 | 颜色深度 | 无排序 | GCT 大小
    gif += struct.pack("<HHBBB", w, h, 0xF0 | (bits - 1), 0, 0)
    gif += pal

    # Netscape 扩展：循环次数
    gif += b"\x21\xFF\x0BNETSCAPE2.0\x03\x01"
    gif += struct.pack("<H", loop)
    gif += b"\x00"

    for fr in frames:
        if len(fr) != h or len(fr[0]) != w:
            raise ValueError(f"每帧尺寸必须一致：期望 {h}x{w}")
        # 图形控制扩展：disposal=1（保留），无透明
        packed = 0x04 if transparent is None else 0x05
        gif += b"\x21\xF9\x04" + bytes((packed,)) + struct.pack("<H", delay_cs)
        gif += bytes((0 if transparent is None else transparent, 0))
        # 图像描述符
        gif += b"\x2C" + struct.pack("<HHHHB", 0, 0, w, h, 0)
        gif += bytes((min_code_size,))
        flat = []
        for row in fr:
            flat.extend(int(v) for v in row)
        gif += _sub_blocks(lzw_encode(flat, min_code_size))

    gif += b"\x3B"

    with open(path, "wb") as f:
        f.write(gif)
    return path
