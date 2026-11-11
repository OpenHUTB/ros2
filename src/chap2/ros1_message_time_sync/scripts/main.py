#!/usr/bin/env python3
"""发布两路模拟传感器话题，并输出按时间近似配对的同步结果."""

import math

import message_filters
import rospy
from sensor_msgs.msg import RelativeHumidity, Temperature
from std_msgs.msg import String

from ros1_message_time_sync.sensor_math import periodic_sensor_value


class SensorSyncNode:
    """生成两路不同频率的传感器数据并提供时间同步结果."""

    def __init__(self):
        """读取参数、创建话题、同步器和定时器."""
        self._temperature_rate = self._positive_parameter(
            'temperature_rate_hz', 10.0)
        self._humidity_rate = self._positive_parameter(
            'humidity_rate_hz', 7.0)
        self._sync_slop = self._positive_parameter('sync_slop_sec', 0.14)
        self._queue_size = self._positive_integer_parameter(
            'sync_queue_size', 30)

        self._temperature_base = float(
            rospy.get_param('~temperature_base_c', 22.0))
        self._temperature_amplitude = float(
            rospy.get_param('~temperature_amplitude_c', 1.5))
        self._temperature_period = self._positive_parameter(
            'temperature_period_sec', 20.0)
        self._humidity_base = float(
            rospy.get_param('~humidity_base_percent', 45.0))
        self._humidity_amplitude = float(
            rospy.get_param('~humidity_amplitude_percent', 8.0))
        self._humidity_period = self._positive_parameter(
            'humidity_period_sec', 30.0)

        self._temperature_publisher = rospy.Publisher(
            '/sensors/temperature', Temperature, queue_size=20)
        self._humidity_publisher = rospy.Publisher(
            '/sensors/humidity', RelativeHumidity, queue_size=20)
        self._pair_publisher = rospy.Publisher(
            '/sensors/sync_pair', String, queue_size=20)

        self._temperature_subscriber = message_filters.Subscriber(
            '/sensors/temperature', Temperature)
        self._humidity_subscriber = message_filters.Subscriber(
            '/sensors/humidity', RelativeHumidity)
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self._temperature_subscriber, self._humidity_subscriber],
            queue_size=self._queue_size,
            slop=self._sync_slop,
        )
        self._synchronizer.registerCallback(self._sync_callback)

        self._start_time = rospy.get_time()
        self._temperature_sequence = 0
        self._humidity_sequence = 0
        self._sync_count = 0
        self._difference_sum = 0.0
        self._maximum_difference = 0.0

        self._temperature_timer = rospy.Timer(
            rospy.Duration(1.0 / self._temperature_rate),
            self._publish_temperature,
        )
        self._humidity_timer = rospy.Timer(
            rospy.Duration(1.0 / self._humidity_rate),
            self._publish_humidity,
        )

        rospy.loginfo(
            '时间同步节点已启动: temperature=%.1f Hz, humidity=%.1f Hz, '
            'slop=%.3f s',
            self._temperature_rate,
            self._humidity_rate,
            self._sync_slop,
        )

    def _positive_parameter(self, name, default):
        """读取并校验正数参数."""
        value = float(rospy.get_param(f'~{name}', default))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} 必须是大于 0 的有限数值')
        return value

    def _positive_integer_parameter(self, name, default):
        """读取并校验正整数参数."""
        value = self._positive_parameter(name, default)
        if not value.is_integer():
            raise ValueError(f'{name} 必须是正整数')
        return int(value)

    def _publish_temperature(self, event):
        """发布一路模拟温度消息."""
        del event
        elapsed_sec = rospy.get_time() - self._start_time
        message = Temperature()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = 'simulated_environment'
        message.header.seq = self._temperature_sequence
        message.temperature = periodic_sensor_value(
            elapsed_sec,
            self._temperature_base,
            self._temperature_amplitude,
            self._temperature_period,
        )
        message.variance = 0.01
        self._temperature_sequence += 1
        self._temperature_publisher.publish(message)

    def _publish_humidity(self, event):
        """发布一路模拟湿度消息."""
        del event
        elapsed_sec = rospy.get_time() - self._start_time
        message = RelativeHumidity()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = 'simulated_environment'
        message.header.seq = self._humidity_sequence
        message.relative_humidity = periodic_sensor_value(
            elapsed_sec,
            self._humidity_base,
            self._humidity_amplitude,
            self._humidity_period,
        )
        message.variance = 0.04
        self._humidity_sequence += 1
        self._humidity_publisher.publish(message)

    def _sync_callback(self, temperature_message, humidity_message):
        """处理时间戳近似相同的温湿度消息对."""
        difference = abs(
            (temperature_message.header.stamp
             - humidity_message.header.stamp).to_sec())
        pair = String()
        pair.data = (
            f'temperature_c={temperature_message.temperature:.3f} '
            f'humidity_percent={humidity_message.relative_humidity:.3f} '
            f'timestamp_diff_sec={difference:.6f} '
            f'temperature_seq={temperature_message.header.seq} '
            f'humidity_seq={humidity_message.header.seq}'
        )
        self._pair_publisher.publish(pair)

        self._sync_count += 1
        self._difference_sum += difference
        self._maximum_difference = max(
            self._maximum_difference, difference)
        if self._sync_count % 10 == 0:
            average_difference = self._difference_sum / self._sync_count
            rospy.loginfo(
                '已同步 %d 对: average_dt=%.6f s, max_dt=%.6f s',
                self._sync_count,
                average_difference,
                self._maximum_difference,
            )

    def shutdown(self):
        """停止生成消息的定时器."""
        self._temperature_timer.shutdown()
        self._humidity_timer.shutdown()


def main():
    """启动时间同步节点，直到收到 Ctrl+C."""
    rospy.init_node('sensor_sync_node')
    node = SensorSyncNode()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()


if __name__ == '__main__':
    main()
