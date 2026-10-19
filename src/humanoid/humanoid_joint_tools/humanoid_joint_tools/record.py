import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import json
import time

class JointRecorder(Node):
    def __init__(self):
        super().__init__('joint_recorder')
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.start_srv = self.create_service(Trigger, '/start_record', self.start_callback)
        self.stop_srv = self.create_service(Trigger, '/stop_record', self.stop_callback)
        
        self.is_recording = False
        self.recorded_data = []
        self.get_logger().info("Recorder ready. Call /start_record to begin.")

    def start_callback(self, request, response):
        self.is_recording = True
        self.recorded_data = []
        response.success = True
        response.message = "Recording started."
        self.get_logger().info("=== Recording Started ===")
        return response

    def stop_callback(self, request, response):
        self.is_recording = False
        if len(self.recorded_data) == 0:
            response.success = False
            response.message = "No data recorded."
            return response
            
        filename = 'recorded_traj.json'
        with open(filename, 'w') as f:
            json.dump(self.recorded_data, f)
            
        response.success = True
        response.message = f"Saved {len(self.recorded_data)} frames to {filename}."
        self.get_logger().info(f"=== Recording Stopped. Saved to {filename} ===")
        return response

    def joint_callback(self, msg):
        if self.is_recording:
            frame = {
                'time': time.time(),
                'name': list(msg.name),
                'position': list(msg.position)
            }
            self.recorded_data.append(frame)

def main(args=None):
    rclpy.init(args=args)
    node = JointRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
