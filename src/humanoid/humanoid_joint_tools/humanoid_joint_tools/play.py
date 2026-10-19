import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import json
import os

class JointPlayer(Node):
    def __init__(self):
        super().__init__('joint_player')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.filename = 'recorded_traj.json'
        self.data = []
        self.current_idx = 0
        self.is_playing = False
        
        self.check_timer = self.create_timer(1.0, self.check_and_play)
        self.play_timer = None
        self.get_logger().info("Player ready. Waiting for recorded_traj.json...")

    def check_and_play(self):
        if not self.is_playing and os.path.exists(self.filename):
            try:
                with open(self.filename, 'r') as f:
                    self.data = json.load(f)
                if len(self.data) > 0:
                    self.get_logger().info(f"Loaded {len(self.data)} frames. Starting playback...")
                    self.is_playing = True
                    self.current_idx = 0
                    if self.play_timer:
                        self.play_timer.cancel()
                    self.play_timer = self.create_timer(0.05, self.play_callback)
            except Exception as e:
                self.get_logger().error(f"Failed to load file: {e}")

    def play_callback(self):
        if self.current_idx >= len(self.data):
            self.get_logger().info("=== Playback Finished ===")
            self.play_timer.cancel()
            self.is_playing = False
            return
            
        frame = self.data[self.current_idx]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = frame['name']
        msg.position = frame['position']
        
        self.publisher_.publish(msg)
        self.current_idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = JointPlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
