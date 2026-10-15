from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    model_path = LaunchConfiguration("model_path")
    airsim_ip = LaunchConfiguration("airsim_ip")
    airsim_port = LaunchConfiguration("airsim_port")
    show_window = LaunchConfiguration("show_window")
    start_topmost = LaunchConfiguration("start_topmost")

    return LaunchDescription([

        DeclareLaunchArgument(
            "model_path",
            default_value="yolo11n.pt",
            description="Path to YOLO model"
        ),
        DeclareLaunchArgument(
            "airsim_ip",
            default_value="127.0.0.1",
            description="OpenHUTB/AirSim RPC server address",
        ),
        DeclareLaunchArgument(
            "airsim_port",
            default_value="41451",
            description="OpenHUTB/AirSim RPC server port",
        ),
        DeclareLaunchArgument(
            "show_window",
            default_value="true",
            description="Show the OpenCV YOLO window",
        ),
        DeclareLaunchArgument(
            "start_topmost",
            default_value="true",
            description="Request an always-on-top YOLO window when supported.",
        ),

        # =====================================================
        # OpenHUTB / AirSim 到 ROS 的桥接
        # =====================================================
        Node(
            package="openhutb_yolo_mlp_control",
            executable="airsim_bridge",
            name="airsim_bridge",
            output="screen",

            parameters=[{
                "airsim_ip": ParameterValue(
                    airsim_ip,
                    value_type=str,
                ),
                "airsim_port": ParameterValue(
                    airsim_port,
                    value_type=int,
                ),

                "camera_name": "0",

                "image_hz": 5.0,
                "state_hz": 20.0,

                # 感知启动文件不接管无人机。
                "enable_api_control": False,
                "arm_on_start": False,
                "auto_takeoff": False,

                "command_duration": 0.15,
                "command_timeout": 0.60,

                "camera_topic": "/openhutb/camera/rgb",
                "odom_topic": "/openhutb/odom",
                "cmd_vel_topic": "/openhutb/cmd_vel",

                "capture_metric_topic":
                    "/openhutb/metrics/airsim_capture_ms",
            }]
        ),

        # =====================================================
        # ROS 图像到 YOLO 检测
        # =====================================================
        Node(
            package="openhutb_yolo_mlp_control",
            executable="yolo_detector",
            name="yolo_detector",
            output="screen",

            parameters=[{
                "model_path": model_path,

                "confidence": 0.30,
                "image_size": 640,

                "input_image_topic":
                    "/openhutb/camera/rgb",

                "detections_topic":
                    "/openhutb/detections",

                "annotated_image_topic":
                    "/openhutb/yolo/image",

                "capture_metric_topic":
                    "/openhutb/metrics/airsim_capture_ms",

                "show_window": ParameterValue(
                    show_window,
                    value_type=bool,
                ),
                "start_topmost": ParameterValue(
                    start_topmost,
                    value_type=bool,
                ),

                "window_name":
                    "OpenHUTB + YOLO",

                "output_dir":
                    "output",
            }]
        ),
    ])
