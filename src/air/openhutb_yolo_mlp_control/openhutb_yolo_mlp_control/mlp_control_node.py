import csv
import math
import os
import time

import numpy as np
import rclpy
import torch
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool

from .mlp_model import MLPController


class MLPControlNode(Node):
    """仅通过 ROS 消息工作的八字形轨迹 MLP 控制器。

    输入：
      /openhutb/odom   -> nav_msgs/Odometry
      /openhutb/target -> geometry_msgs/PointStamped

    输出：
      /openhutb/cmd_vel -> geometry_msgs/Twist

    本节点不会直接调用 AirSim，只有 airsim_bridge 节点直接连接模拟器。
    """

    def __init__(self):
        super().__init__('mlp_controller')

        self.declare_parameter(
            'model_path',
            'mlp_controller.pth',
        )
        self.declare_parameter(
            'control_dt',
            0.15,
        )
        self.declare_parameter(
            'odom_topic',
            '/openhutb/odom',
        )
        self.declare_parameter(
            'target_topic',
            '/openhutb/target',
        )
        self.declare_parameter(
            'cmd_vel_topic',
            '/openhutb/cmd_vel',
        )
        self.declare_parameter(
            'done_topic',
            '/openhutb/trajectory_done',
        )
        self.declare_parameter(
            'result_dir',
            'results',
        )
        self.declare_parameter(
            'use_vertical_control',
            True,
        )
        self.declare_parameter(
            'vertical_speed_limit',
            1.0,
        )

        model_path = os.path.abspath(
            str(
                self.get_parameter(
                    'model_path'
                ).value
            )
        )

        if not os.path.isfile(model_path):
            raise FileNotFoundError(
                f'MLP checkpoint not found: {model_path}'
            )

        self.control_dt = float(
            self.get_parameter(
                'control_dt'
            ).value
        )
        self.use_vertical_control = bool(
            self.get_parameter('use_vertical_control').value
        )
        self.vertical_speed_limit = abs(float(
            self.get_parameter('vertical_speed_limit').value
        ))

        self.device = torch.device(
            'cuda'
            if torch.cuda.is_available()
            else 'cpu'
        )

        self.get_logger().info(
            f'Loading MLP checkpoint: {model_path}'
        )

        checkpoint = torch.load(
            model_path,
            map_location=self.device,
            weights_only=False,
        )

        required_keys = {
            'model_state_dict',
            'input_scale',
            'output_scale',
        }

        missing = required_keys.difference(
            checkpoint.keys()
        )

        if missing:
            raise KeyError(
                'MLP checkpoint is missing keys: '
                + ', '.join(sorted(missing))
            )

        self.model = MLPController().to(
            self.device
        )

        self.model.load_state_dict(
            checkpoint['model_state_dict']
        )
        self.model.eval()

        self.input_scale = np.asarray(
            checkpoint['input_scale'],
            dtype=np.float32,
        )
        self.output_scale = np.asarray(
            checkpoint['output_scale'],
            dtype=np.float32,
        )

        self.get_logger().info(
            f'MLP ready; device={self.device}'
        )

        self.odom = None
        self.target = None
        self.done = False

        self._received_odom = False
        self._received_target = False
        self._control_started = False

        self.start_time = time.time()

        self.cmd_pub = self.create_publisher(
            Twist,
            str(
                self.get_parameter(
                    'cmd_vel_topic'
                ).value
            ),
            10,
        )

        # 桥接节点使用传感器数据 QoS 发布里程计。
        self.odom_sub = self.create_subscription(
            Odometry,
            str(
                self.get_parameter(
                    'odom_topic'
                ).value
            ),
            self.odom_callback,
            qos_profile_sensor_data,
        )

        self.target_sub = self.create_subscription(
            PointStamped,
            str(
                self.get_parameter(
                    'target_topic'
                ).value
            ),
            self.target_callback,
            10,
        )

        self.done_sub = self.create_subscription(
            Bool,
            str(
                self.get_parameter(
                    'done_topic'
                ).value
            ),
            self.done_callback,
            10,
        )

        self.timer = self.create_timer(
            self.control_dt,
            self.control_step,
        )

        result_dir = os.path.abspath(
            str(
                self.get_parameter(
                    'result_dir'
                ).value
            )
        )

        os.makedirs(
            result_dir,
            exist_ok=True,
        )

        self.csv_path = os.path.join(
            result_dir,
            'eight_trajectory.csv',
        )

        self.csv_file = open(
            self.csv_path,
            'w',
            newline='',
            encoding='utf-8',
        )

        self.csv_writer = csv.writer(
            self.csv_file
        )

        self.csv_writer.writerow([
            'time',
            'target_x',
            'target_y',
            'target_z',
            'actual_x',
            'actual_y',
            'actual_z',
            'error',
            'vx_cmd',
            'vy_cmd',
            'vz_cmd',
        ])

        self.get_logger().info(
            f'Result CSV: {self.csv_path}'
        )

    def odom_callback(
        self,
        msg: Odometry,
    ):
        self.odom = msg

        if not self._received_odom:
            self._received_odom = True
            self.get_logger().info(
                'Receiving /openhutb/odom.'
            )

    def target_callback(
        self,
        msg: PointStamped,
    ):
        self.target = msg

        if not self._received_target:
            self._received_target = True
            self.get_logger().info(
                'Receiving /openhutb/target.'
            )

    def done_callback(
        self,
        msg: Bool,
    ):
        self.done = bool(
            msg.data
        )

        if self.done:
            self.publish_stop()

            self.get_logger().info(
                'Figure-eight trajectory done; '
                f'result CSV: {self.csv_path}'
            )

    def publish_stop(self):
        self.cmd_pub.publish(
            Twist()
        )

    def control_step(self):
        if self.done:
            self.publish_stop()
            return

        if (
            self.odom is None
            or self.target is None
        ):
            return

        if not self._control_started:
            self._control_started = True
            self.get_logger().info(
                'MLP control loop started.'
            )

        p = self.odom.pose.pose.position
        v = self.odom.twist.twist.linear
        target = self.target.point

        ex = float(
            target.x - p.x
        )
        ey = float(
            target.y - p.y
        )
        # 在 MLP 输入中保留高度误差，以便修正缓慢的高度漂移。
        # 轨迹目标将 z 保持在起始飞行高度，因此 ez 通常接近零，但不能丢弃。
        ez = float(target.z - p.z)

        distance = math.sqrt(
            ex * ex
            + ey * ey
            + ez * ez
        )

        input_state = np.array(
            [
                ex,
                ey,
                ez,
                float(v.x),
                float(v.y),
                float(v.z),
            ],
            dtype=np.float32,
        )

        normalized = (
            input_state
            / self.input_scale
        )

        tensor = torch.tensor(
            normalized,
            dtype=torch.float32,
            device=self.device,
        ).unsqueeze(0)

        with torch.no_grad():
            output = (
                self.model(tensor)
                .cpu()
                .numpy()[0]
            )

        command = (
            output
            * self.output_scale
        )

        vx_cmd, vy_cmd, vz_cmd = [
            float(x)
            for x in command
        ]
        if not self.use_vertical_control:
            vz_cmd = 0.0
        elif self.vertical_speed_limit > 0.0:
            vz_cmd = max(
                -self.vertical_speed_limit,
                min(self.vertical_speed_limit, vz_cmd),
            )

        twist = Twist()
        twist.linear.x = vx_cmd
        twist.linear.y = vy_cmd
        twist.linear.z = vz_cmd

        self.cmd_pub.publish(
            twist
        )

        self.csv_writer.writerow([
            time.time()
            - self.start_time,
            float(target.x),
            float(target.y),
            float(target.z),
            float(p.x),
            float(p.y),
            float(p.z),
            distance,
            vx_cmd,
            vy_cmd,
            vz_cmd,
        ])

        self.csv_file.flush()

    def destroy_node(self):
        try:
            self.publish_stop()
            self.csv_file.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = MLPControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
