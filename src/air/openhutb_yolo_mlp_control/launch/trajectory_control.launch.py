import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE = 'openhutb_yolo_mlp_control'
DEFAULT_MODEL_PATH = os.path.join(
    get_package_share_directory(PACKAGE),
    'models',
    'mlp_controller.pth',
)


def generate_launch_description():
    airsim_ip = LaunchConfiguration(
        'airsim_ip'
    )
    airsim_port = LaunchConfiguration(
        'airsim_port'
    )
    model_path = LaunchConfiguration(
        'model_path'
    )
    figure8_scale = LaunchConfiguration(
        'figure8_scale'
    )
    figure8_points = LaunchConfiguration(
        'figure8_points'
    )
    waypoint_tolerance = LaunchConfiguration(
        'waypoint_tolerance'
    )
    result_dir = LaunchConfiguration(
        'result_dir'
    )
    auto_takeoff = LaunchConfiguration(
        'auto_takeoff'
    )
    control_dt = LaunchConfiguration(
        'control_dt'
    )
    command_duration = LaunchConfiguration(
        'command_duration'
    )
    command_timeout = LaunchConfiguration(
        'command_timeout'
    )
    use_vertical_control = LaunchConfiguration(
        'use_vertical_control'
    )
    vertical_speed_limit = LaunchConfiguration(
        'vertical_speed_limit'
    )
    min_dispatch_distance = LaunchConfiguration(
        'min_dispatch_distance'
    )
    max_dispatch_distance = LaunchConfiguration(
        'max_dispatch_distance'
    )

    return LaunchDescription([
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
            'model_path',
            default_value=DEFAULT_MODEL_PATH,
        ),
        DeclareLaunchArgument(
            'figure8_scale',
            default_value='10.0',
        ),
        DeclareLaunchArgument(
            'figure8_points',
            default_value='160',
        ),
        DeclareLaunchArgument(
            'waypoint_tolerance',
            default_value='0.35',
        ),
        DeclareLaunchArgument(
            'result_dir',
            default_value='results',
        ),
        DeclareLaunchArgument(
            'auto_takeoff',
            default_value='true',
            description=(
                'Automatically take off before starting trajectory control. '
                'Set auto_takeoff:=false when the vehicle is already airborne.'
            ),
        ),
        DeclareLaunchArgument(
            'control_dt',
            default_value='0.05',
            description='MLP control period in seconds.',
        ),
        DeclareLaunchArgument(
            'command_duration',
            default_value='0.05',
            description='Displacement integration interval in seconds.',
        ),
        DeclareLaunchArgument(
            'command_timeout',
            default_value='1.50',
            description='Watchdog timeout for incoming velocity commands.',
        ),
        DeclareLaunchArgument(
            'use_vertical_control',
            default_value='true',
            description='Allow the MLP to correct altitude drift.',
        ),
        DeclareLaunchArgument(
            'vertical_speed_limit',
            default_value='1.0',
            description='Maximum MLP vertical command in m/s.',
        ),
        DeclareLaunchArgument(
            'min_dispatch_distance',
            default_value='0.0',
            description='Minimum accumulated displacement before dispatch.',
        ),
        DeclareLaunchArgument(
            'max_dispatch_distance',
            default_value='0.15',
            description='Maximum displacement in one position dispatch.',
        ),

        Node(
            package=PACKAGE,
            executable='airsim_bridge',
            name='airsim_bridge',
            output='screen',
            additional_env={
                'PYTHONUNBUFFERED': '1',
            },
            parameters=[{
                'airsim_ip': ParameterValue(
                    airsim_ip,
                    value_type=str,
                ),
                'airsim_port': ParameterValue(
                    airsim_port,
                    value_type=int,
                ),
                'camera_name': '0',

                # 验证轨迹控制时降低图像 RPC 负载，避免影响控制周期。
                'image_hz': 0.1,
                'state_hz': 20.0,

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
                'command_smoothing': 0.35,
                'pose_max_speed': 1.2,
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
                'control_backend': 'auto',

                'camera_topic':
                    '/openhutb/camera/rgb',
                'odom_topic':
                    '/openhutb/odom',
                'cmd_vel_topic':
                    '/openhutb/cmd_vel',
                'capture_metric_topic':
                    '/openhutb/metrics/'
                    'airsim_capture_ms',
            }],
        ),

        Node(
            package=PACKAGE,
            executable='trajectory_target',
            name='trajectory_target',
            output='screen',
            additional_env={
                'PYTHONUNBUFFERED': '1',
            },
            parameters=[{
                'figure8_scale':
                    ParameterValue(
                        figure8_scale,
                        value_type=float,
                    ),
                'figure8_points':
                    ParameterValue(
                        figure8_points,
                        value_type=int,
                    ),
                'waypoint_tolerance':
                    ParameterValue(
                        waypoint_tolerance,
                        value_type=float,
                    ),
                'max_waypoint_time':
                    3.0,

                'odom_topic':
                    '/openhutb/odom',
                'target_topic':
                    '/openhutb/target',
                'done_topic':
                    '/openhutb/'
                    'trajectory_done',
            }],
        ),

        Node(
            package=PACKAGE,
            executable='mlp_controller',
            name='mlp_controller',
            output='screen',
            additional_env={
                'PYTHONUNBUFFERED': '1',
            },
            parameters=[{
                'model_path':
                    ParameterValue(
                        model_path,
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

                'odom_topic':
                    '/openhutb/odom',
                'target_topic':
                    '/openhutb/target',
                'cmd_vel_topic':
                    '/openhutb/cmd_vel',
                'done_topic':
                    '/openhutb/'
                    'trajectory_done',

                'result_dir':
                    ParameterValue(
                        result_dir,
                        value_type=str,
                    ),
            }],
        ),
    ])
