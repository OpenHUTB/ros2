"""启动对比实验节点。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = 'uav_bc_bench'


def generate_launch_description():
    """构造 launch 描述。"""
    params_file = os.path.join(get_package_share_directory(PACKAGE), 'config', 'params.yaml')

    dataset_dir = LaunchConfiguration('dataset_dir')
    output_dir = LaunchConfiguration('output_dir')
    epochs = LaunchConfiguration('epochs')
    seeds = LaunchConfiguration('seeds')
    structures = LaunchConfiguration('structures')
    synthetic = LaunchConfiguration('synthetic')

    node = Node(
        package=PACKAGE,
        executable='main',
        name='uav_bc_bench',
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
        parameters=[params_file, {
            'dataset_dir': ParameterValue(dataset_dir, value_type=str),
            'output_dir': ParameterValue(output_dir, value_type=str),
            'epochs': ParameterValue(epochs, value_type=int),
            'seeds': ParameterValue(seeds, value_type=int),
            'structures': ParameterValue(structures, value_type=str),
            'synthetic': ParameterValue(synthetic, value_type=int),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('dataset_dir', default_value=os.path.expanduser('~/.ros/uav_depth_track_nn/data'),
                              description='包含 controls.csv 与 frames/ 的数据集目录'),
        DeclareLaunchArgument('output_dir', default_value=os.path.expanduser('~/.ros/uav_bc_bench'),
                              description='指标、表格与图表的输出目录'),
        DeclareLaunchArgument('epochs', default_value='30', description='每个配置的训练轮数'),
        DeclareLaunchArgument('seeds', default_value='3', description='每个配置重复的随机种子数'),
        DeclareLaunchArgument('structures', default_value='mlp,cnn,cnn_wide', description='参与对比的网络结构'),
        DeclareLaunchArgument('synthetic', default_value='0',
                              description='大于 0 时现场生成等量合成样本，不需要真实数据集'),
        node,
    ])
