import os,sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
def generate_launch_description():
    share=get_package_share_directory("uav_wind_triangle")
    return LaunchDescription([ExecuteProcess(cmd=[sys.executable,"-m","uav_wind_triangle.main","--input",os.path.join(share,"sample_data","wind_profile.json")],output="screen")])
