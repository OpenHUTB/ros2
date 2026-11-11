"""语音指令 -> 速度指令（决策/控制层）。

订阅 ``/voice_intent``，按意图产生 ``/cmd_vel``：

* ``forward`` / ``backward`` / ``turn_left`` / ``turn_right`` —— 按当前速度档位
  下发速度，保持 ``hold_time`` 秒后自动停车（语音只有一句话，不能像键盘那样
  「按住」；自动停车也避免识别错误让机器人一直跑）。
* ``stop`` —— 立即清零并取消保持。
* ``faster`` / ``slower`` —— 切换速度档位，正在执行的指令立刻按新档位生效。
* ``wave`` —— 不产生速度，向 ``/humanoid/gesture`` 发一次上身动作请求。

速度档位与基准速度都来自 ``config/intents.yaml``。
"""

import os

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

MOTION_INTENTS = ('forward', 'backward', 'turn_left', 'turn_right')


class CommandNode(Node):
    """意图 -> /cmd_vel。"""

    def __init__(self):
        super().__init__('command_node')

        self.declare_parameter('config', '')
        self.declare_parameter('hold_time', 3.0)      # 单条指令保持时长 (s)
        self.declare_parameter('publish_rate', 20.0)

        cfg = self.get_parameter('config').value or self._default_config()
        self.speed, self.levels = self._load_config(cfg)
        self.hold_time = float(self.get_parameter('hold_time').value)
        rate = float(self.get_parameter('publish_rate').value)

        self.level_name = 'normal'
        self.active_intent = ''
        self.deadline = 0.0

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_gesture = self.create_publisher(String, '/humanoid/gesture', 10)
        self.create_subscription(String, '/voice_intent', self._intent_callback, 10)
        self.create_timer(1.0 / rate, self._timer_callback)

        self.get_logger().info(
            f'指令节点已启动 | 保持时长 {self.hold_time:.1f}s | '
            f'基准速度 v_fwd={self.speed["v_forward"]:.2f} '
            f'v_bwd={self.speed["v_backward"]:.2f} w={self.speed["w_turn"]:.2f}')

    # ------------------------------------------------------------------
    @staticmethod
    def _default_config() -> str:
        try:
            from ament_index_python.packages import get_package_share_directory
            return os.path.join(get_package_share_directory('humanoid_voice_command'),
                                'config', 'intents.yaml')
        except Exception:                                  # noqa: BLE001
            here = os.path.dirname(os.path.abspath(__file__))
            return os.path.abspath(os.path.join(here, '..', 'config', 'intents.yaml'))

    @staticmethod
    def _load_config(path: str):
        speed = {'v_forward': 0.30, 'v_backward': 0.20, 'w_turn': 0.60}
        levels = {'slow': 0.6, 'normal': 1.0, 'fast': 1.5}
        if path and os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as fh:
                data = yaml.safe_load(fh) or {}
            speed.update({k: float(v) for k, v in (data.get('speed') or {}).items()})
            levels.update({k: float(v) for k, v in (data.get('levels') or {}).items()})
        return speed, levels

    # ------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _intent_callback(self, msg: String):
        intent = msg.data.strip()
        if intent == 'stop':
            self.active_intent = ''
            self.deadline = 0.0
            self.pub_cmd_vel.publish(Twist())
            self.get_logger().info('收到 stop，已停车')
            return

        if intent == 'wave':
            self.pub_gesture.publish(String(data='wave'))
            self.get_logger().info('收到 wave，触发上身动作')
            return

        if intent in ('faster', 'slower'):
            order = ['slow', 'normal', 'fast']
            i = order.index(self.level_name)
            i = min(len(order) - 1, i + 1) if intent == 'faster' else max(0, i - 1)
            self.level_name = order[i]
            self.get_logger().info(
                f'速度档位 -> {self.level_name} (×{self.levels[self.level_name]:.2f})')
            # 正在执行的动作立刻按新档位生效
            if self.active_intent:
                self.deadline = self._now() + self.hold_time
            return

        if intent in MOTION_INTENTS:
            self.active_intent = intent
            self.deadline = self._now() + self.hold_time
            self.get_logger().info(
                f'执行 {intent}，保持 {self.hold_time:.1f}s（档位 {self.level_name}）')
            return

        self.get_logger().debug(f'忽略意图 {intent!r}')

    def _twist_for(self, intent: str) -> Twist:
        scale = self.levels[self.level_name]
        msg = Twist()
        if intent == 'forward':
            msg.linear.x = self.speed['v_forward'] * scale
        elif intent == 'backward':
            msg.linear.x = -self.speed['v_backward'] * scale
        elif intent == 'turn_left':
            msg.angular.z = self.speed['w_turn'] * scale
        elif intent == 'turn_right':
            msg.angular.z = -self.speed['w_turn'] * scale
        return msg

    def _timer_callback(self):
        # 无活动指令时也持续发零速：一是让下游能判断控制器在线，
        # 二是避免 /cmd_vel 断流后下游保持最后一条非零指令。
        if not self.active_intent:
            self.pub_cmd_vel.publish(Twist())
            return
        if self._now() > self.deadline:
            self.get_logger().info(f'{self.active_intent} 保持时间到，停车')
            self.active_intent = ''
            self.pub_cmd_vel.publish(Twist())
            return
        self.pub_cmd_vel.publish(self._twist_for(self.active_intent))


def main(args=None):
    rclpy.init(args=args)
    node = CommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd_vel.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
