import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE = 'openhutb_yolo_mlp_control'
PACKAGE_SHARE = get_package_share_directory(PACKAGE)
DEFAULT_CONFIG_PATH = os.path.join(
    PACKAGE_SHARE,
    'config',
    'openhutb_ros.yaml',
)
DEFAULT_MLP_MODEL_PATH = os.path.join(
    PACKAGE_SHARE,
    'models',
    'mlp_controller.pth',
)


def generate_launch_description():
    # OpenHUTB / AirSim connection.
    airsim_ip = LaunchConfiguration('airsim_ip')
    airsim_port = LaunchConfiguration('airsim_port')
    auto_takeoff = LaunchConfiguration('auto_takeoff')

    # Bridge rates and command dispatch.
    image_hz = LaunchConfiguration('image_hz')
    state_hz = LaunchConfiguration('state_hz')
    command_duration = LaunchConfiguration('command_duration')
    command_timeout = LaunchConfiguration('command_timeout')
    min_dispatch_distance = LaunchConfiguration('min_dispatch_distance')
    max_dispatch_distance = LaunchConfiguration('max_dispatch_distance')

    # YOLO perception.
    yolo_model_path = LaunchConfiguration('yolo_model_path')
    yolo_confidence = LaunchConfiguration('yolo_confidence')
    yolo_image_size = LaunchConfiguration('yolo_image_size')
    show_window = LaunchConfiguration('show_window')
    start_topmost = LaunchConfiguration('start_topmost')
    yolo_output_dir = LaunchConfiguration('yolo_output_dir')

    # Figure-eight trajectory generation.
    figure8_scale = LaunchConfiguration('figure8_scale')
    figure8_points = LaunchConfiguration('figure8_points')
    waypoint_tolerance = LaunchConfiguration('waypoint_tolerance')
    max_waypoint_time = LaunchConfiguration('max_waypoint_time')

    # MLP controller.
    mlp_model_path = LaunchConfiguration('mlp_model_path')
    control_dt = LaunchConfiguration('control_dt')
    use_vertical_control = LaunchConfiguration('use_vertical_control')
    vertical_speed_limit = LaunchConfiguration('vertical_speed_limit')
    result_dir = LaunchConfiguration('result_dir')

    return LaunchDescription([
        # ---------------------------------------------------------
        # Connection and bridge arguments
        # ---------------------------------------------------------
        DeclareLaunchArgument(
            'airsim_ip',
            default_value='127.0.0.1',
            description='OpenHUTB/AirSim RPC server address.',
        ),
        DeclareLaunchArgument(
            'airsim_port',
            default_value='41451',
            description='OpenHUTB/AirSim RPC server port.',
        ),
        DeclareLaunchArgument(
            'auto_takeoff',
            default_value='true',
            description=(
                'Automatically take off before trajectory control. '
                'Set to false when the vehicle is already airborne.'
            ),
        ),
        DeclareLaunchArgument(
            'image_hz',
            default_value='5.0',
            description='Camera publishing rate used by YOLO, in Hz.',
        ),
        DeclareLaunchArgument(
            'state_hz',
            default_value='20.0',
            description='Odometry publishing rate, in Hz.',
        ),
        DeclareLaunchArgument(
            'command_duration',
            default_value='0.05',
            description='Velocity-to-displacement integration interval.',
        ),
        DeclareLaunchArgument(
            'command_timeout',
            default_value='1.50',
            description='Watchdog timeout for incoming velocity commands.',
        ),
        DeclareLaunchArgument(
            'min_dispatch_distance',
            default_value='0.0',
            description='Minimum accumulated displacement before dispatch.',
        ),
        DeclareLaunchArgument(
            'max_dispatch_distance',
            default_value='0.15',
            description='Maximum displacement sent in one position command.',
        ),

        # ---------------------------------------------------------
        # YOLO arguments
        # ---------------------------------------------------------
        DeclareLaunchArgument(
            'yolo_model_path',
            default_value='yolo11n.pt',
            description=(
                'Path to the YOLO model. The repository does not include '
                'yolo11n.pt, so an absolute local path is recommended.'
            ),
        ),
        DeclareLaunchArgument(
            'yolo_confidence',
            default_value='0.30',
            description='YOLO confidence threshold.',
        ),
        DeclareLaunchArgument(
            'yolo_image_size',
            default_value='640',
            description='YOLO inference image size.',
        ),
        DeclareLaunchArgument(
            'show_window',
            default_value='true',
            description='Show the OpenCV YOLO result window.',
        ),
        DeclareLaunchArgument(
            'start_topmost',
            default_value='true',
            description='Request an always-on-top YOLO window when supported.',
        ),
        DeclareLaunchArgument(
            'yolo_output_dir',
            default_value='output',
            description='Directory used by the YOLO node for runtime output.',
        ),

        # ---------------------------------------------------------
        # Figure-eight trajectory arguments
        # ---------------------------------------------------------
        DeclareLaunchArgument(
            'figure8_scale',
            default_value='10.0',
            description='Figure-eight trajectory scale in metres.',
        ),
        DeclareLaunchArgument(
            'figure8_points',
            default_value='160',
            description='Number of segments used to discretize the trajectory.',
        ),
        DeclareLaunchArgument(
            'waypoint_tolerance',
            default_value='0.35',
            description='Waypoint reach tolerance in metres.',
        ),
        DeclareLaunchArgument(
            'max_waypoint_time',
            default_value='3.0',
            description='Maximum time allowed for each waypoint, in seconds.',
        ),

        # ---------------------------------------------------------
        # MLP controller arguments
        # ---------------------------------------------------------
        DeclareLaunchArgument(
            'mlp_model_path',
            default_value=DEFAULT_MLP_MODEL_PATH,
            description='Path to the packaged MLP controller checkpoint.',
        ),
        DeclareLaunchArgument(
            'control_dt',
            default_value='0.05',
            description='MLP control period in seconds.',
        ),
        DeclareLaunchArgument(
            'use_vertical_control',
            default_value='true',
            description='Allow the MLP controller to correct altitude drift.',
        ),
        DeclareLaunchArgument(
            'vertical_speed_limit',
            default_value='1.0',
            description='Maximum absolute vertical MLP command in m/s.',
        ),
        DeclareLaunchArgument(
            'result_dir',
            default_value='results',
            description='Directory used to store trajectory CSV results.',
        ),

        # ---------------------------------------------------------
        # OpenHUTB / AirSim bridge
        # ---------------------------------------------------------
        Node(
            package=PACKAGE,
            executable='airsim_bridge',
            name='airsim_bridge',
            output='screen',
            additional_env={'PYTHONUNBUFFERED': '1'},
            parameters=[
                DEFAULT_CONFIG_PATH,
                {
                    'airsim_ip': ParameterValue(airsim_ip, value_type=str),
                    'airsim_port': ParameterValue(airsim_port, value_type=int),
                    'camera_name': '0',
                    'image_hz': ParameterValue(image_hz, value_type=float),
                    'state_hz': ParameterValue(state_hz, value_type=float),
                    'enable_api_control': True,
                    'arm_on_start': True,
                    'auto_takeoff': ParameterValue(
                        auto_takeoff,
                        value_type=bool,
                    ),
                    'command_duration': ParameterValue(
                        command_duration,
                        value_type=float,
                    ),
                    'command_timeout': ParameterValue(
                        command_timeout,
                        value_type=float,
                    ),
                    'min_dispatch_distance': ParameterValue(
                        min_dispatch_distance,
                        value_type=float,
                    ),
                    'max_dispatch_distance': ParameterValue(
                        max_dispatch_distance,
                        value_type=float,
                    ),
                    'command_smoothing': 0.35,
                    'pose_max_speed': 1.2,
                    'control_backend': 'auto',
                    'camera_topic': '/openhutb/camera/rgb',
                    'odom_topic': '/openhutb/odom',
                    'cmd_vel_topic': '/openhutb/cmd_vel',
                    'capture_metric_topic': (
                        '/openhutb/metrics/airsim_capture_ms'
                    ),
                },
            ],
        ),

        # ---------------------------------------------------------
        # YOLO detector
        # ---------------------------------------------------------
        Node(
            package=PACKAGE,
            executable='yolo_detector',
            name='yolo_detector',
            output='screen',
            additional_env={'PYTHONUNBUFFERED': '1'},
            parameters=[
                DEFAULT_CONFIG_PATH,
                {
                    'model_path': ParameterValue(
                        yolo_model_path,
                        value_type=str,
                    ),
                    'confidence': ParameterValue(
                        yolo_confidence,
                        value_type=float,
                    ),
                    'image_size': ParameterValue(
                        yolo_image_size,
                        value_type=int,
                    ),
                    'input_image_topic': '/openhutb/camera/rgb',
                    'detections_topic': '/openhutb/detections',
                    'annotated_image_topic': '/openhutb/yolo/image',
                    'capture_metric_topic': (
                        '/openhutb/metrics/airsim_capture_ms'
                    ),
                    'show_window': ParameterValue(
                        show_window,
                        value_type=bool,
                    ),
                    'start_topmost': ParameterValue(
                        start_topmost,
                        value_type=bool,
                    ),
                    'window_name': 'OpenHUTB + YOLO',
                    'output_dir': ParameterValue(
                        yolo_output_dir,
                        value_type=str,
                    ),
                },
            ],
        ),

        # ---------------------------------------------------------
        # Figure-eight target generator
        # ---------------------------------------------------------
        Node(
            package=PACKAGE,
            executable='trajectory_target',
            name='trajectory_target',
            output='screen',
            additional_env={'PYTHONUNBUFFERED': '1'},
            parameters=[
                DEFAULT_CONFIG_PATH,
                {
                    'figure8_scale': ParameterValue(
                        figure8_scale,
                        value_type=float,
                    ),
                    'figure8_points': ParameterValue(
                        figure8_points,
                        value_type=int,
                    ),
                    'waypoint_tolerance': ParameterValue(
                        waypoint_tolerance,
                        value_type=float,
                    ),
                    'max_waypoint_time': ParameterValue(
                        max_waypoint_time,
                        value_type=float,
                    ),
                    'odom_topic': '/openhutb/odom',
                    'target_topic': '/openhutb/target',
                    'done_topic': '/openhutb/trajectory_done',
                },
            ],
        ),

        # ---------------------------------------------------------
        # MLP trajectory controller
        # ---------------------------------------------------------
        Node(
            package=PACKAGE,
            executable='mlp_controller',
            name='mlp_controller',
            output='screen',
            additional_env={'PYTHONUNBUFFERED': '1'},
            parameters=[
                DEFAULT_CONFIG_PATH,
                {
                    'model_path': ParameterValue(
                        mlp_model_path,
                        value_type=str,
                    ),
                    'control_dt': ParameterValue(
                        control_dt,
                        value_type=float,
                    ),
                    'use_vertical_control': ParameterValue(
                        use_vertical_control,
                        value_type=bool,
                    ),
                    'vertical_speed_limit': ParameterValue(
                        vertical_speed_limit,
                        value_type=float,
                    ),
                    'odom_topic': '/openhutb/odom',
                    'target_topic': '/openhutb/target',
                    'cmd_vel_topic': '/openhutb/cmd_vel',
                    'done_topic': '/openhutb/trajectory_done',
                    'result_dir': ParameterValue(
                        result_dir,
                        value_type=str,
                    ),
                },
            ],
        ),
    ])
