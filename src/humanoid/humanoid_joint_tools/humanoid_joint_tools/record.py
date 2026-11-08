import json
import os
import tempfile

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

DEFAULT_FILENAME = '~/.ros/joint_traj.json'

class JointRecorder(Node):
    def __init__(self):
        super().__init__('joint_recorder')
        self.declare_parameter('input_topic', '/joint_states_raw')
        self.declare_parameter('filename', DEFAULT_FILENAME)
        topic = self.get_parameter('input_topic').value
        self.subscription = self.create_subscription(JointState, topic, self.joint_callback, 20)
        self.start_srv = self.create_service(Trigger, '/start_record', self.start_callback)
        self.stop_srv = self.create_service(Trigger, '/stop_record', self.stop_callback)

        self.is_recording = False
        self.recorded_data = []
        self.get_logger().info(
            f"Recorder ready. Recording {topic} into '{self.target_path()}'. "
            'Call /start_record to begin.'
        )

    def target_path(self):
        return os.path.expanduser(self.get_parameter('filename').value)

    def start_callback(self, request, response):
        # 防重入：录制中再次调用 /start_record 直接拒绝，避免静默清空已录数据
        if self.is_recording:
            response.success = False
            response.message = 'Recording already in progress. Call /stop_record first.'
            self.get_logger().warn('=== /start_record ignored: recording already in progress ===')
            return response

        self.is_recording = True
        self.recorded_data = []
        response.success = True
        response.message = 'Recording started.'
        self.get_logger().info('=== Recording Started ===')
        return response

    def stop_callback(self, request, response):
        was_recording = self.is_recording
        self.is_recording = False
        if not was_recording:
            response.success = False
            response.message = 'No recording in progress.'
            return response
        if not self.recorded_data:
            response.success = False
            response.message = 'No data recorded.'
            return response

        path = self.target_path()
        try:
            overwritten = self.save(self.recorded_data, path)
        except OSError as e:
            response.success = False
            response.message = f'Failed to save {path}: {e}'
            self.get_logger().error(f'Failed to save {path}: {e}')
            return response

        note = ' (previous file overwritten)' if overwritten else ''
        response.success = True
        response.message = f'Saved {len(self.recorded_data)} frames to {path}.{note}'
        self.get_logger().info(f'=== Recording Stopped. Saved to {path}{note} ===')
        return response

    def joint_callback(self, msg):
        if not self.is_recording:
            return
        self.recorded_data.append({
            'time': self.get_clock().now().nanoseconds / 1e9,
            'name': list(msg.name),
            'position': list(msg.position),
        })

    @staticmethod
    def save(frames, path):
        directory = os.path.dirname(path) or '.'
        os.makedirs(directory, exist_ok=True)
        overwritten = os.path.exists(path)
        fd, tmp_path = tempfile.mkstemp(dir=directory, prefix='.joint_traj_', suffix='.tmp')
        try:
            with os.fdopen(fd, 'w') as f:
                json.dump(frames, f)
            os.replace(tmp_path, path)
        except BaseException:
            if os.path.exists(tmp_path):
                os.unlink(tmp_path)
            raise
        return overwritten

def main(args=None):
    rclpy.init(args=args)
    node = JointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
