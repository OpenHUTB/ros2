"""Archimedean-spiral search mission; preview-only by default.

复用 openhutb_yolo_mlp_control 的 AirSim 桥接节点（airsim_bridge），
叠加本包的螺旋搜索节点；任一进程退出时整体退出。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from uav_spiral_search.spiral_search import Settings as SearchSettings
from uav_spiral_search.target_detector import Settings as DetectorSettings


def generate_launch_description():
    bridge_package = 'openhutb_yolo_mlp_control'
    arguments = {
        'airsim_ip': ('127.0.0.1', str),
        'airsim_port': (41451, int),
        'camera_name': ('0', str),
        'dry_run': (True, bool),
        'auto_takeoff': (False, bool),
        'show_window': (True, bool),
        'control_backend': ('auto', str),
        'control_dt': (0.05, float),
        'max_dispatch_distance': (0.2, float),
    }
    for name, value in vars(SearchSettings()).items():
        arguments[name] = (value, type(value))
    for name, value in vars(DetectorSettings()).items():
        arguments['det_' + name] = (value, type(value))

    declarations = [DeclareLaunchArgument(name, default_value=str(value).lower()
                    if isinstance(value, bool) else str(value))
                    for name, (value, _) in arguments.items()]
    param = lambda name: ParameterValue(LaunchConfiguration(name),
                                        value_type=arguments[name][1])

    bridge = Node(package=bridge_package, executable='airsim_bridge',
                  name='airsim_bridge', output='screen', parameters=[{
                      'airsim_ip': param('airsim_ip'),
                      'airsim_port': param('airsim_port'),
                      'camera_name': param('camera_name'),
                      'enable_api_control': ParameterValue(PythonExpression([
                          "'", LaunchConfiguration('dry_run'), "'.lower() == 'false'"
                      ]), value_type=bool),
                      'auto_takeoff': param('auto_takeoff'),
                      'arm_on_start': param('auto_takeoff'),
                      'hold_heading': True,
                      'command_smoothing': 1.0,
                      'async_image_capture': True,
                      'command_duration': param('control_dt'),
                      'command_timeout': 0.5,
                      'min_dispatch_distance': 0.0,
                      'max_dispatch_distance': param('max_dispatch_distance'),
                      'control_backend': param('control_backend'),
                      'image_hz': 5.0,
                      'state_hz': 20.0,
                  }])

    searcher = Node(package='uav_spiral_search', executable='spiral_search',
                    name='spiral_search', output='screen', parameters=[{
                        name: param(name) for name in
                        [*vars(SearchSettings()),
                         *('det_' + n for n in vars(DetectorSettings())),
                         'dry_run', 'control_dt', 'show_window']
                    }])

    # Register handlers before starting processes, including startup failures.
    exits = [RegisterEventHandler(OnProcessExit(target_action=node,
              on_exit=[EmitEvent(event=Shutdown(reason='Spiral search process exited'))]))
             for node in (bridge, searcher)]
    return LaunchDescription(declarations + exits + [bridge, searcher])
