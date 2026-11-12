"""CAN 报文校验：CRC8(SAE J1850) 与滚动计数器。"""

CHECK_BYTES = b'123456789'
# CRC8/SAE-J1850 的标准校验值：对 b'123456789' 计算应得 0x4B
CHECK_VALUE = 0x4B


def crc8_sae_j1850(data: bytes, init: int = 0xFF) -> int:
    """CRC8/SAE-J1850：poly=0x1D, init=0xFF, xorout=0xFF，不反转。"""
    crc = init
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x1D) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc ^ 0xFF


def xor_checksum(data: bytes) -> int:
    """逐字节异或，部分国产 ECU 用这种简化的帧校验。"""
    result = 0
    for byte in data:
        result ^= byte
    return result


def counter_step_ok(previous: int, current: int, width: int = 4) -> bool:
    """滚动计数器是否按 +1 递增（到顶回绕）。``previous`` 为 None 时视为首帧。"""
    if previous is None:
        return True
    if not 0 <= previous < (1 << width) or not 0 <= current < (1 << width):
        raise ValueError(f'计数器超出 {width} 位范围')
    return current == (previous + 1) % (1 << width)


def counter_jump(previous: int, current: int, width: int = 4) -> int:
    """返回计数器跳变的步数（正常为 1；丢帧时会大于 1）。"""
    if previous is None:
        return 0
    return (current - previous) % (1 << width)
