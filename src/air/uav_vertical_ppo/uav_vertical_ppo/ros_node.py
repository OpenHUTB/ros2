"""以 50 Hz 执行 MuJoCo 与 PPO 闭环并发布高度状态."""
import json

from .environment import VerticalHoverEnv
from .paths import resource


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from stable_baselines3 import PPO

    class HoverNode(Node):
        def __init__(self):
            super().__init__('uav_vertical_ppo')
            self.declare_parameter('scenario', 'calm')
            self.declare_parameter('target_height', 1.5)
            self.declare_parameter('model_path', str(resource('data/hover_ppo.zip')))
            self.env = VerticalHoverEnv(self.get_parameter('scenario').value,
                                        self.get_parameter('target_height').value)
            from .policy import load_policy
            self.policy = load_policy(self.get_parameter('model_path').value, self.env)
            self.obs, _ = self.env.reset(seed=101)
            self.publisher = self.create_publisher(String, '/uav/hover_state', 10)
            self.timer = self.create_timer(0.02, self.tick)
            self.get_logger().info('定高闭环已启动，状态话题 /uav/hover_state')

        def tick(self):
            action, _ = self.policy.predict(self.obs, deterministic=True)
            self.obs, _, done, timeout, info = self.env.step(action)
            message = String()
            message.data = json.dumps(info)
            self.publisher.publish(message)
            if done or timeout:
                self.obs, _ = self.env.reset(seed=101)

    rclpy.init(args=args)
    node = None
    try:
        node = HoverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.env.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
