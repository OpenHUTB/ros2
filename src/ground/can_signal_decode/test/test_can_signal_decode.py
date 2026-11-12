"""can_signal_decode 的离线单元测试（不依赖 ROS 2）。"""

import pytest

from can_signal_decode.checksum import (CHECK_BYTES, CHECK_VALUE,
                                        counter_jump, counter_step_ok,
                                        crc8_sae_j1850, xor_checksum)
from can_signal_decode.database import CRC_BYTE, FrameCodec, default_frames
from can_signal_decode.frame import (FrameSpec, SignalSpec, int_to_payload,
                                     motorola_positions, pack_message,
                                     payload_to_int, unpack_message)


def test_crc_check_value():
    assert crc8_sae_j1850(CHECK_BYTES) == CHECK_VALUE


def test_crc_detects_every_single_bit_flip():
    base = bytes([0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77])
    reference = crc8_sae_j1850(base)
    for bit in range(len(base) * 8):
        damaged = bytearray(base)
        damaged[bit // 8] ^= 1 << (bit % 8)
        assert crc8_sae_j1850(bytes(damaged)) != reference, f'bit {bit} 未被发现'


def test_xor_checksum():
    assert xor_checksum(bytes([0x0F, 0xF0])) == 0xFF
    assert xor_checksum(b'') == 0


def test_counter_rules():
    assert counter_step_ok(None, 7) is True
    assert counter_step_ok(7, 8) is True
    assert counter_step_ok(15, 0) is True
    assert counter_step_ok(7, 9) is False
    assert counter_jump(7, 9) == 2
    assert counter_jump(15, 0) == 1


def test_payload_int_round_trip():
    payload = bytes([1, 2, 3, 4, 5, 6, 7, 8])
    assert int_to_payload(payload_to_int(payload)) == payload
    with pytest.raises(ValueError):
        int_to_payload(1 << 64)


def test_little_endian_speed_round_trip():
    spec = FrameSpec(0x100, 'test', [SignalSpec('Speed', 0, 16, scale=0.01)])
    for physical in (0.0, 35.5, 85.25, 250.0):
        payload = spec.encode({'Speed': physical})
        assert spec.decode(payload)['Speed'] == pytest.approx(physical, abs=0.005)


def test_signed_torque_round_trip():
    spec = FrameSpec(0x100, 'test',
                     [SignalSpec('Torque', 16, 16, signed=True, scale=0.1)])
    for physical in (-300.0, -12.3, 0.0, 120.5, 300.0):
        decoded = spec.decode(spec.encode({'Torque': physical}))['Torque']
        assert decoded == pytest.approx(physical, abs=0.05)


def test_motorola_positions_are_sawtooth():
    assert motorola_positions(7, 16) == [7, 6, 5, 4, 3, 2, 1, 0,
                                         15, 14, 13, 12, 11, 10, 9, 8]


def test_motorola_and_little_layouts_differ():
    big = FrameSpec(0x101, 'big', [SignalSpec('A', 7, 16, byte_order='big')])
    little = FrameSpec(0x101, 'little', [SignalSpec('A', 0, 16)])
    payload = big.encode({'A': 0x1234})
    assert big.decode(payload)['A'] == 0x1234
    assert little.decode(payload)['A'] != 0x1234


def test_overlapping_signals_are_rejected():
    with pytest.raises(ValueError):
        FrameSpec(0x100, 'bad', [
            SignalSpec('A', 0, 8),
            SignalSpec('B', 4, 8),
        ])


def test_range_violation_is_reported():
    codec = FrameCodec(default_frames()[0], counter_name='AliveCounter')
    payload = codec.build({'VehicleSpeed': 300.0, 'MotorTorque': 0.0,
                           'Gear': 3.0, 'BrakePressure': 0.0}, counter=0)
    decoded = codec.decode(payload, previous=None)
    assert decoded['crc_ok'] is True
    assert decoded['range_violations'] == ['VehicleSpeed']
    assert decoded['valid'] is False


def test_codec_accepts_clean_frame_and_flags_corruption():
    codec = FrameCodec(default_frames()[1], counter_name='AliveCounter')
    payload = codec.build({'SteeringAngle': 123.4}, counter=5)
    good = codec.decode(payload, previous=None)
    assert good['crc_ok'] and good['counter_ok'] and good['valid']

    damaged = bytearray(payload)
    damaged[CRC_BYTE] ^= 0x01
    bad = codec.decode(bytes(damaged), previous=payload)
    assert bad['crc_ok'] is False
    assert bad['valid'] is False


def test_message_pack_unpack():
    payload = bytes(range(8))
    message = pack_message(0x101, payload)
    assert len(message) == 12
    assert unpack_message(message) == (0x101, payload)
    assert unpack_message([1, 2, 3]) is None
    with pytest.raises(ValueError):
        pack_message(0x100, b'\x00')


def test_crc_failed_frame_does_not_break_next_counter():
    """坏帧不能作为下一帧的计数器基准：一次误码只罚一帧，不连累下一帧。"""
    from can_signal_decode.main import run

    codec = FrameCodec(default_frames()[0], counter_name='AliveCounter')
    values = {'VehicleSpeed': 60.0, 'MotorTorque': 10.0, 'Gear': 4.0,
              'BrakePressure': 0.0}
    # 真实场景：第 6 帧在总线上被干扰（计数器原本是 6），第 7 帧正常到达。
    first = codec.build(values, counter=5)
    damaged = bytearray(codec.build(values, counter=6))
    damaged[6] ^= 0x03                     # 打在载荷高位，计数器位保持可读
    third = codec.build(values, counter=7)

    stream = [
        (0.00, codec, first, values, ()),
        (0.02, codec, bytes(damaged), values, ('crc',)),
        (0.04, codec, third, values, ()),
    ]
    stats, rows = run(stream)

    assert rows[1][2]['crc_ok'] is False
    assert rows[1][2]['valid'] is False             # 坏帧整体判为无效
    assert rows[1][2]['counter_jump'] == 0          # 但不拿它的计数器做判定
    assert rows[2][2]['crc_ok'] is True
    assert rows[2][2]['counter_ok'] is True         # 但下一帧不应被连累
    assert rows[2][2]['valid'] is True


def test_cycle_ms_matches_simulation_rates():
    """报文周期要与 simulate() 的实际发送频率一致（50 Hz 基频）。"""
    vcu, steering = default_frames()
    assert vcu.cycle_ms == pytest.approx(20.0)        # 每帧都发
    assert steering.cycle_ms == pytest.approx(60.0)   # 每 3 帧发一次
