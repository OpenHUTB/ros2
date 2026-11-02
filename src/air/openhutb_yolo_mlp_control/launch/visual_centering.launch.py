"""Independent YOLO horizontal-centering mode; preview-only by default."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from openhutb_yolo_mlp_control.visual_centering import Settings


def generate_launch_description():
    package = 'openhutb_yolo_mlp_control'
    arguments = {
        'airsim_ip': ('127.0.0.1', str),
        'airsim_port': (41451, int),
        'camera_name': ('0', str),
        'yolo_model_path': ('yolo11n.pt', str),
        'show_window': (True, bool),
        'start_topmost': (True, bool),
        'dry_run': (True, bool),
        'control_backend': ('auto', str),
        'control_dt': (0.05, float),
        'max_dispatch_distance': (0.2, float),
    }
    for name, value in vars(Settings()).items():
        arguments[name] = (value, type(value))
    # More responsive simulation preset, with bounded acceleration.
    arguments.update(gain=(1.8, float), max_speed=(0.8, float),
                     max_acceleration=(1.2, float))
    declarations = [DeclareLaunchArgument(name, default_value=str(value).lower()
                    if isinstance(value, bool) else str(value))
                    for name, (value, _) in arguments.items()]
    param = lambda name: ParameterValue(LaunchConfiguration(name),
                                        value_type=arguments[name][1])
    bridge = Node(package=package, executable='airsim_bridge', name='airsim_bridge',
                  output='screen', parameters=[{
                      'airsim_ip': param('airsim_ip'), 'airsim_port': param('airsim_port'),
                      'camera_name': param('camera_name'),
                      'enable_api_control': ParameterValue(PythonExpression([
                          "'", LaunchConfiguration('dry_run'), "'.lower() == 'false'"
                      ]), value_type=bool),
                      'auto_takeoff': False, 'arm_on_start': False,
                      'hold_heading': True, 'command_smoothing': 1.0,
                      'async_image_capture': True,
                      'command_duration': param('control_dt'), 'command_timeout': 0.5,
                      'min_dispatch_distance': 0.0,
                      'max_dispatch_distance': param('max_dispatch_distance'),
                      'control_backend': param('control_backend'),
                      'image_hz': 5.0, 'state_hz': 20.0,
                  }])
    detector = Node(package=package, executable='yolo_detector', name='yolo_detector',
                    output='screen', parameters=[{
                        'model_path': param('yolo_model_path'),
                        'show_window': param('show_window'),
                        'start_topmost': param('start_topmost'),
                        'confidence': param('confidence'),
                    }])
    controller = Node(package=package, executable='visual_centering',
                      name='visual_centering', output='screen', parameters=[{
                          name: param(name) for name in
                          [*vars(Settings()), 'dry_run', 'control_dt']
                      }])
    # Register handlers before starting processes, including startup failures.
    exits = [RegisterEventHandler(OnProcessExit(target_action=node,
              on_exit=[EmitEvent(event=Shutdown(reason='Visual centering process exited'))]))
             for node in (bridge, detector, controller)]
    return LaunchDescription(declarations + exits + [bridge, detector, controller])
