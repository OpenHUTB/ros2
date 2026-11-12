"""CAN 信号定义与位域编解码。

位编号约定（与 Python 整数位序一致）：
- 把 8 字节报文看作一个 64 位整数，byte0 的最低位是第 0 位；
- ``little``（Intel）：信号从 ``start_bit`` 开始，向高位连续取 ``length`` 位；
- ``big``（Motorola / DBC 风格）：``start_bit`` 是信号的**最高位**，
  之后按「同字节内位号递减、跨字节跳到下一字节的最高位」的锯齿顺序取位。

只做位运算，不依赖 ROS 与第三方库。
"""

from dataclasses import dataclass, field
from typing import Iterable

BYTE_COUNT = 8
_MAX_BITS = BYTE_COUNT * 8
ID_BYTES = 4
MESSAGE_BYTES = ID_BYTES + BYTE_COUNT


def unpack_message(data) -> tuple:
    """把 ROS 消息的字节数组拆成 ``(frame_id, payload)``；长度不符返回 None。

    约定：前 ``ID_BYTES`` 字节是帧 ID（小端 uint32），后 8 字节是载荷。
    """
    if len(data) != MESSAGE_BYTES:
        return None
    frame_id = int.from_bytes(bytes(data[:ID_BYTES]), 'little')
    return frame_id, bytes(data[ID_BYTES:])


def pack_message(frame_id: int, payload: bytes) -> list:
    """``(frame_id, payload)`` 打包成 ROS 消息可用的字节列表。"""
    if len(payload) != BYTE_COUNT:
        raise ValueError(f'载荷必须是 {BYTE_COUNT} 字节，收到 {len(payload)}')
    return list(frame_id.to_bytes(ID_BYTES, 'little') + bytes(payload))


def payload_to_int(payload: bytes) -> int:
    """把 8 字节载荷转成 64 位整数（byte0 为最低字节）。"""
    if len(payload) != BYTE_COUNT:
        raise ValueError(f'载荷必须是 {BYTE_COUNT} 字节，收到 {len(payload)}')
    value = 0
    for index, byte in enumerate(payload):
        value |= byte << (8 * index)
    return value


def int_to_payload(value: int) -> bytes:
    """把 64 位整数转回 8 字节载荷。"""
    if value < 0 or value >= 1 << _MAX_BITS:
        raise ValueError('数值超出 8 字节范围')
    return bytes((value >> (8 * index)) & 0xFF for index in range(BYTE_COUNT))


def motorola_positions(start_bit: int, length: int) -> list:
    """按 DBC 的 Motorola 顺序返回每一位在 64 位整数中的下标记（首项是 MSB）。"""
    positions = []
    position = start_bit
    for _ in range(length):
        if position < 0 or position >= _MAX_BITS:
            raise ValueError('Motorola 信号越界')
        positions.append(position)
        if position % 8 == 0:
            position += 15
        else:
            position -= 1
    return positions


@dataclass(frozen=True)
class SignalSpec:
    """一个信号在报文中的位置与物理量换算。"""

    name: str
    start_bit: int
    length: int
    byte_order: str = 'little'
    signed: bool = False
    scale: float = 1.0
    offset: float = 0.0
    unit: str = ''
    minimum: float = None
    maximum: float = None

    def __post_init__(self):
        if self.byte_order not in ('little', 'big'):
            raise ValueError(f'{self.name}: byte_order 只能是 little/big')
        if not 1 <= self.length <= _MAX_BITS:
            raise ValueError(f'{self.name}: length 必须在 1..64')
        if self.scale == 0.0:
            raise ValueError(f'{self.name}: scale 不能为 0')

    @property
    def mask(self) -> int:
        return (1 << self.length) - 1

    def raw_from_payload(self, payload: bytes) -> int:
        """从载荷里取出原始整数值（已按补码转成有符号）。"""
        frame = payload_to_int(payload)
        if self.byte_order == 'little':
            raw = (frame >> self.start_bit) & self.mask
        else:
            raw = 0
            for position in motorola_positions(self.start_bit, self.length):
                raw = (raw << 1) | ((frame >> position) & 1)
        if self.signed and raw & (1 << (self.length - 1)):
            raw -= 1 << self.length
        return raw

    def raw_into_payload(self, payload: int, raw: int) -> int:
        """把原始整数写进报文的 64 位整型表示。"""
        if self.signed:
            if not -(1 << (self.length - 1)) <= raw < (1 << (self.length - 1)):
                raise ValueError(f'{self.name}: 有符号原始值越界')
            raw &= self.mask
        elif not 0 <= raw <= self.mask:
            raise ValueError(f'{self.name}: 原始值越界 0..{self.mask}')
        if self.byte_order == 'little':
            return (payload & ~(self.mask << self.start_bit)) | (raw << self.start_bit)
        cleared = payload
        positions = motorola_positions(self.start_bit, self.length)
        for position in positions:
            cleared &= ~(1 << position)
        result = cleared
        for index, position in enumerate(positions):
            bit = (raw >> (self.length - 1 - index)) & 1
            result |= bit << position
        return result

    def physical(self, raw: int) -> float:
        return raw * self.scale + self.offset

    def raw(self, physical_value: float) -> int:
        return int(round((physical_value - self.offset) / self.scale))

    def in_range(self, physical_value: float) -> bool:
        if self.minimum is not None and physical_value < self.minimum - 1e-9:
            return False
        if self.maximum is not None and physical_value > self.maximum + 1e-9:
            return False
        return True


@dataclass
class FrameSpec:
    """一条报文的信号集合。"""

    frame_id: int
    name: str
    signals: list = field(default_factory=list)
    cycle_ms: float = 10.0

    def __post_init__(self):
        if not 0 <= self.frame_id < (1 << 11):
            raise ValueError(f'{self.name}: 标准帧 ID 必须 < 0x800')
        self._check_overlap()

    def _check_overlap(self):
        used = {}
        for signal in self.signals:
            if signal.byte_order == 'little':
                positions = range(signal.start_bit, signal.start_bit + signal.length)
            else:
                positions = motorola_positions(signal.start_bit, signal.length)
            for position in positions:
                if position in used:
                    raise ValueError(
                        f'{self.name}: 信号 {signal.name} 与 {used[position]} 位重叠')
                used[position] = signal.name

    def encode(self, values: dict) -> bytes:
        """按物理量字典打包成 8 字节载荷，缺省信号写 0。"""
        payload = 0
        for signal in self.signals:
            physical = values.get(signal.name, signal.offset)
            payload = signal.raw_into_payload(payload, signal.raw(physical))
        return int_to_payload(payload)

    def decode(self, payload: bytes) -> dict:
        """解包成 ``{信号名: 物理值}``。"""
        return {signal.name: signal.physical(signal.raw_from_payload(payload))
                for signal in self.signals}

    def raw_view(self, payload: bytes) -> dict:
        return {signal.name: signal.raw_from_payload(payload) for signal in self.signals}


def iter_signals(frames: Iterable) -> Iterable:
    for frame in frames:
        for signal in frame.signals:
            yield frame, signal
