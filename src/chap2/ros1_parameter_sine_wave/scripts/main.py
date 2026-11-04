#!/usr/bin/env python3
"""发布正弦信号，并使用 dynamic_reconfigure 在线调整参数."""

import math
import threading

import rospy
from dynamic_reconfigure.server import Server
from std_msgs.msg import Float64

from ros1_parameter_sine_wave.cfg import ParamsConfig
from ros1_parameter_sine_wave.math_utils import compute_sine


class SineWaveNode:
    """周期性发布正弦信号并支持运行时参数热更新."""

    def __init__(self):
        """创建发布者、动态参数服务器和发布定时器."""
        self._lock = threading.Lock()
        self._config = None
        self._timer = None
        self._server = None
        self._sample_count = 0
        self._start_time = rospy.get_time()

        self._publisher = rospy.Publisher(
            'sine_wave/value', Float64, queue_size=10)
        self._server = Server(ParamsConfig, self._configure)

        rospy.loginfo('正弦信号节点已启动')

    def _configure(self, config, level):
        """接收动态参数，并在采样率变化时重建定时器."""
        del level

        with self._lock:
            previous_rate = (
                self._config.publish_rate_hz
                if self._config is not None
                else None
            )
            self._config = config

        rate_changed = (
            previous_rate is None
            or not math.isclose(
                previous_rate,
                config.publish_rate_hz,
                rel_tol=1e-9,
                abs_tol=1e-9,
            )
        )
        if rate_changed:
            self._replace_timer(config.publish_rate_hz)

        rospy.loginfo(
            '参数已更新: amplitude=%.3f, frequency_hz=%.3f, '
            'publish_rate_hz=%.3f, phase_rad=%.3f, enabled=%s',
            config.amplitude,
            config.frequency_hz,
            config.publish_rate_hz,
            config.phase_rad,
            str(config.enabled).lower(),
        )
        return config

    def _replace_timer(self, publish_rate_hz):
        """按新发布频率重建定时器."""
        if self._timer is not None:
            self._timer.shutdown()

        period = rospy.Duration(1.0 / publish_rate_hz)
        self._timer = rospy.Timer(period, self._publish_sample)
        rospy.loginfo('发布频率已更新为 %.3f Hz', publish_rate_hz)

    def _publish_sample(self, event):
        """计算并发布当前时刻的正弦值."""
        del event

        with self._lock:
            config = self._config

        if config is None or not config.enabled:
            return

        elapsed_sec = rospy.get_time() - self._start_time
        value = compute_sine(
            config.amplitude,
            config.frequency_hz,
            elapsed_sec,
            config.phase_rad,
        )
        self._publisher.publish(Float64(value))

        self._sample_count += 1
        log_interval = max(1, int(round(config.publish_rate_hz)))
        if self._sample_count % log_interval == 0:
            rospy.loginfo('t=%.3fs value=%.6f', elapsed_sec, value)

    def shutdown(self):
        """停止定时器并关闭动态参数服务器."""
        if self._timer is not None:
            self._timer.shutdown()
            self._timer = None
        if self._server is not None:
            self._server.shutdown()
            self._server = None


def main():
    """启动正弦信号节点，直到收到 Ctrl+C."""
    rospy.init_node('sine_wave_node')
    node = SineWaveNode()
    rospy.on_shutdown(node.shutdown)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('正弦信号节点被手动终止')
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
