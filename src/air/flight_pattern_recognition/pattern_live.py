"""Read-only AirSim bridge. Never arms, takes off, or sends flight commands."""
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    import airsim
    rclpy.init()
    node = Node('flight_pattern_live')
    node.declare_parameter('host', '192.168.239.1')
    node.declare_parameter('vehicle', 'PredictionDrone')
    client = airsim.MultirotorClient(ip=node.get_parameter('host').value, timeout_value=2)
    vehicle = node.get_parameter('vehicle').value
    pub = node.create_publisher(String, '/uav/pattern/telemetry', 100)

    def tick():
        try:
            state = client.getMultirotorState(vehicle_name=vehicle)
            k = state.kinematics_estimated
            payload = dict(timestamp_ns=int(state.timestamp), segment_id='live')
            for prefix, vector in [('p', k.position), ('v', k.linear_velocity), ('a', k.linear_acceleration)]:
                for axis in 'xyz':
                    payload[prefix + axis] = float(getattr(vector, axis + '_val'))
            pub.publish(String(data=json.dumps(payload, allow_nan=False)))
        except Exception as error:
            node.get_logger().warning('AirSim read failed: %s' % error)

    node.create_timer(.05, tick)
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
