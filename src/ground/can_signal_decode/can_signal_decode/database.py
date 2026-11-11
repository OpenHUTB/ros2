"""两条示例报文的信号数据库，以及带 CRC / 计数器的编解码封装。

报文 0x100 VCU_Status（小端信号）
    车速 VehicleSpeed     bit0   len16  0.01 km/h
    电机扭矩 MotorTorque   bit16  len16  0.1 Nm，有符号
    挡位 Gear             bit32  len3
    制动压力 BrakePressure bit35  len8   0.5 bar
    —— byte7 为 CRC8

报文 0x101 Steering_Status（大端/Motorola 信号）
    方向盘转角 SteeringAngle  start7 len16 0.1 deg，有符号（DBC 的 Motorola 写法）
    转角角速度 YawRate?      不用
    保留保留
    滚动计数器 AliveCounter  bit32 len4
    —— byte7 为 CRC8
"""

from .checksum import counter_jump, counter_step_ok, crc8_sae_j1850
from .frame import BYTE_COUNT, FrameSpec, SignalSpec

CRC_BYTE = 7


def default_frames():
    """返回本模块使用的报文定义列表。"""
    vcu = FrameSpec(
        frame_id=0x100,
        name='VCU_Status',
        cycle_ms=20.0,          # 50 Hz 基频下每帧都发
        signals=[
            SignalSpec('VehicleSpeed', start_bit=0, length=16, scale=0.01,
                       unit='km/h', minimum=0.0, maximum=250.0),
            SignalSpec('MotorTorque', start_bit=16, length=16, signed=True,
                       scale=0.1, unit='Nm', minimum=-300.0, maximum=300.0),
            SignalSpec('Gear', start_bit=32, length=3, scale=1.0, unit=''),
            SignalSpec('BrakePressure', start_bit=35, length=8, scale=0.5,
                       unit='bar', minimum=0.0, maximum=127.5),
            SignalSpec('AliveCounter', start_bit=43, length=4, scale=1.0),
        ],
    )
    steering = FrameSpec(
        frame_id=0x101,
        name='Steering_Status',
        cycle_ms=60.0,          # 50 Hz 基频下每 3 帧发一次
        signals=[
            SignalSpec('SteeringAngle', start_bit=7, length=16, byte_order='big',
                       signed=True, scale=0.1, unit='deg',
                       minimum=-780.0, maximum=780.0),
            SignalSpec('AliveCounter', start_bit=32, length=4, scale=1.0),
        ],
    )
    return [vcu, steering]


class FrameCodec:
    """在 ``FrameSpec`` 上叠加「CRC 占 byte7 + 滚动计数器」的整车报文约定。"""

    def __init__(self, spec: FrameSpec, counter_name: str = None):
        self.spec = spec
        self.counter_name = counter_name
        if counter_name is not None:
            names = [signal.name for signal in spec.signals]
            if counter_name not in names:
                raise ValueError(f'{counter_name} 不在报文 {spec.name} 的信号表里')

    @property
    def frame_id(self):
        return self.spec.frame_id

    @property
    def name(self):
        return self.spec.name

    def build(self, values: dict, counter: int = None) -> bytes:
        """打包并盖章（写计数器与 CRC）。"""
        if counter is not None and self.counter_name is not None:
            values = dict(values)
            values[self.counter_name] = float(counter)
        payload = bytearray(self.spec.encode(values))
        payload[CRC_BYTE] = crc8_sae_j1850(bytes(payload[:CRC_BYTE]))
        return bytes(payload)

    def crc_ok(self, payload: bytes) -> bool:
        if len(payload) != BYTE_COUNT:
            return False
        expected = crc8_sae_j1850(bytes(payload[:CRC_BYTE]))
        return payload[CRC_BYTE] == expected

    def decode(self, payload: bytes, previous: bytes = None,
               expected_step: int = 1) -> dict:
        """解码一帧，返回物理量 + 校验结论。

        ``expected_step`` 是「相对上一帧通过 CRC 的报文，计数器应该前进了几步」。
        中间夹了 k 个 CRC 失败的帧时，接收端会传 ``1 + k``，这样坏帧既不会被当作
        计数器基准，也不会让下一帧被误判成跳变。
        """
        result = self.spec.decode(payload)
        result['frame_id'] = self.spec.frame_id
        result['frame_name'] = self.spec.name
        result['crc_ok'] = self.crc_ok(payload)

        violated = [signal.name for signal in self.spec.signals
                    if not signal.in_range(result[signal.name])]
        result['range_violations'] = violated

        if self.counter_name is None or not result['crc_ok']:
            # CRC 都没过的一帧，计数器位本身也可能被打坏，拿它做连续性判定没有意义：
            # 记为「不适用」，避免一次误码被重复计成两帧异常。
            result['counter_ok'] = True
            result['counter_jump'] = 0
        else:
            current = int(round(result[self.counter_name]))
            previous_counter = None
            if previous is not None:
                previous_counter = int(round(
                    self.spec.decode(previous)[self.counter_name]))
            step = counter_jump(previous_counter, current)
            result['counter_step'] = step
            result['counter_ok'] = (previous_counter is None
                                    or step == expected_step)
            result['counter_jump'] = step
        result['valid'] = bool(result['crc_ok'] and result['counter_ok']
                               and not violated)
        return result
