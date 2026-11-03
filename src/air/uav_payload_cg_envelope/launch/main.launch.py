import os,sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
def generate_launch_description():
 s=get_package_share_directory("uav_payload_cg_envelope")
 return LaunchDescription([ExecuteProcess(cmd=[sys.executable,"-m","uav_payload_cg_envelope.main","--input",os.path.join(s,"sample_data","cg_profile.json")],output="screen")])
