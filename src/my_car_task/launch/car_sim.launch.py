from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    urdf_file = "/home/user/ros2/src/my_car_task/urdf/car.urdf"

    return LaunchDescription([
        ExecuteProcess(
            cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so"],
            output="screen"
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-file", urdf_file, "-entity", "my_car"],
            output="screen"
        ),
        Node(
            package="my_car_task",
            executable="car_controller",
            output="screen"
        )
    ])
