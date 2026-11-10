"""Gazebo 差速驱动小车 —— 模块统一入口（ROS 2）。

仓库约定「每个模块必须支持 launch 启动，入口为 main.xxx」，因此本模块
以本文件作为统一入口，通过 ``mode`` 参数调度到对应功能节点：

    teleop  键盘遥控（物理仿真 + 键盘运动控制）
    laser   激光雷达避障（感知 → 控制闭环）
    camera  深度相机避障（感知 → 控制闭环）
    idle    仅加载仿真环境，不启动控制算法

启动方式（三者等价）：
    ros2 launch gazebo_diff_drive_car main.launch.py mode:=laser
    ros2 run    gazebo_diff_drive_car car_main --ros-args -p mode:=laser
    python3 main.py --ros-args -p mode:=laser
"""

import sys

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from gazebo_diff_drive_car.depth_camera_avoid import DepthCameraAvoidNode
from gazebo_diff_drive_car.laser_avoid import LaserAvoidNode
from gazebo_diff_drive_car.teleop_keyboard import TeleopKeyboardNode


#: 支持的运行模式
SUPPORTED_MODES = ('teleop', 'laser', 'camera', 'idle')


class ModeDispatcher(Node):
    """读取 ``mode`` 参数的调度节点。"""

    def __init__(self):
        """声明并读取 mode / use_sim_time 参数。"""
        super().__init__('car_main')
        self.declare_parameter('mode', 'teleop')
        self.declare_parameter('use_sim_time', True)
        self.mode = str(self.get_parameter('mode').value)
        self.use_sim_time = bool(self.get_parameter('use_sim_time').value)


def create_worker(mode, use_sim_time=True):
    """按模式创建对应功能节点。

    :param mode: 运行模式字符串
    :param use_sim_time: 是否使用仿真时钟（透传给 worker 节点，
        否则避障状态机里 get_clock().now() 会走系统时钟，计时失效）
    :return: 节点实例；idle 模式返回 None
    """
    if mode == 'idle':
        return None
    if mode == 'teleop':
        worker = TeleopKeyboardNode()
    elif mode == 'laser':
        worker = LaserAvoidNode()
    elif mode == 'camera':
        worker = DepthCameraAvoidNode()
    else:
        raise ValueError('未知模式：%s' % mode)

    # 透传 use_sim_time：worker 是在本进程内新建的独立节点，
    # 不会自动继承 car_main 的参数，需要显式声明并设置。
    worker.declare_parameter('use_sim_time', False)
    worker.set_parameters([
        Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time),
    ])
    return worker


def main(args=None):
    """程序入口。"""
    rclpy.init(args=args)

    dispatcher = ModeDispatcher()
    mode = dispatcher.mode
    use_sim_time = dispatcher.use_sim_time
    dispatcher.destroy_node()

    if mode not in SUPPORTED_MODES:
        print('未知模式 %r，支持：%s' % (mode, ', '.join(SUPPORTED_MODES)),
              file=sys.stderr)
        rclpy.shutdown()
        return

    print('=' * 58)
    print('  Gazebo 差速驱动小车 —— 统一入口节点（ROS 2）')
    print('  运行模式：%s   use_sim_time=%s' % (mode, use_sim_time))
    print('=' * 58)

    worker = create_worker(mode, use_sim_time)

    if worker is None:
        print('idle 模式：仅加载仿真环境，不启动控制算法。')
        rclpy.shutdown()
        return

    if mode == 'teleop':
        worker.start_terminal()

    try:
        rclpy.spin(worker)
    except KeyboardInterrupt:
        pass
    finally:
        if mode == 'teleop':
            worker.restore_terminal()
        worker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
