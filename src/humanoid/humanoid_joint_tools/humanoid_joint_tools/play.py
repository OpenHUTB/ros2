import json
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

DEFAULT_FILENAME = '~/.ros/joint_traj.json'
STEP_PERIOD = 0.01

class JointPlayer(Node):
    def __init__(self):
        super().__init__('joint_player')
        self.declare_parameter('filename', DEFAULT_FILENAME)
        self.declare_parameter('output_topic', '/joint_states')
        self.declare_parameter('auto_play', False)
        self.declare_parameter('check_period', 1.0)
        topic = self.get_parameter('output_topic').value
        self.publisher_ = self.create_publisher(JointState, topic, 20)
        self.start_srv = self.create_service(Trigger, '/start_play', self.start_play_callback)

        self.data = []
        self.current_idx = 0
        self.is_playing = False
        self.last_signature = None
        self.start_time = 0.0
        self.wall_start = 0.0

        self.step_timer = self.create_timer(STEP_PERIOD, self.play_callback)
        self.check_timer = self.create_timer(
            float(self.get_parameter('check_period').value), self.check_and_play
        )
        self.get_logger().info(
            f"Player ready. Publishing on {topic} from '{self.target_path()}'. "
            'Call /start_play to replay.'
        )

    def target_path(self):
        return os.path.expanduser(self.get_parameter('filename').value)

    def start_play_callback(self, request, response):
        path = self.target_path()
        if self.is_playing:
            response.success = False
            response.message = 'Playback already running.'
            return response
        if not os.path.exists(path):
            response.success = False
            response.message = f'{path} not found. Record a trajectory first.'
            return response
        frames = self.load(path)
        if not frames:
            response.success = False
            response.message = f'{path} contains no valid frames.'
            return response

        self.last_signature = self.signature(path)
        self.begin(frames)
        response.success = True
        response.message = f'Playing {len(frames)} frames from {path}.'
        return response

    def check_and_play(self):
        if self.is_playing or not bool(self.get_parameter('auto_play').value):
            return
        path = self.target_path()
        if not os.path.exists(path):
            return
        current_signature = self.signature(path)
        if current_signature == self.last_signature:
            return
        frames = self.load(path)
        if not frames:
            return
        self.last_signature = current_signature
        self.get_logger().info(f'New recording detected: {len(frames)} frames. Starting playback.')
        self.begin(frames)

    def begin(self, frames):
        self.data = frames
        self.current_idx = 0
        self.start_time = frames[0]['time']
        self.wall_start = self.now()
        self.is_playing = True

    def play_callback(self):
        if not self.is_playing:
            return
        elapsed = self.now() - self.wall_start
        while self.current_idx < len(self.data):
            frame = self.data[self.current_idx]
            if frame['time'] - self.start_time > elapsed:
                break
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = frame['name']
            msg.position = frame['position']
            self.publisher_.publish(msg)
            self.current_idx += 1
        if self.current_idx >= len(self.data):
            self.is_playing = False
            self.get_logger().info(
                f'=== Playback Finished ({len(self.data)} frames, {elapsed:.2f} s) ==='
            )

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def signature(path):
        stat = os.stat(path)
        return stat.st_mtime_ns, stat.st_size

    @staticmethod
    def load(path):
        try:
            with open(path, 'r') as f:
                frames = json.load(f)
        except (OSError, json.JSONDecodeError):
            return []
        if not isinstance(frames, list):
            return []
        valid = []
        for i, frame in enumerate(frames):
            if not isinstance(frame, dict) or 'name' not in frame or 'position' not in frame:
                continue
            frame = dict(frame, time=float(frame.get('time', i * 0.05)))
            frame['name'] = list(frame['name'])
            frame['position'] = [float(v) for v in frame['position']]
            valid.append(frame)
        return valid

def main(args=None):
    rclpy.init(args=args)
    node = JointPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
