import os,sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
def generate_launch_description():
 s=get_package_share_directory("uav_radio_link_budget")
 return LaunchDescription([ExecuteProcess(cmd=[sys.executable,"-m","uav_radio_link_budget.main","--input",os.path.join(s,"sample_data","radio_profile.json")],output="screen")])
