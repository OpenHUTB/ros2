@echo off


if not exist "ros2" (
    git clone https://OpenHUTB:T8w6TYB_r71gGTP3A02B@git.code.tencent.com/OpenHUTB/ros2.git   &&  cd ros2  && git lfs pull
) else (
    cd ros2
    git reset --hard HEAD
    git clean -fdx
    git pull
    git lfs pull
)


rem Usage: vmware [OPTION ...] [--] [configuration file(s)]
rem where OPTIONS are:
rem -v             Show program versionPower on when a virtual machine is
rem -x             Same as -x but also go into full screen mode
rem -p             Start the virtual machine paused
rem -q             Close virtual machine at rem power off
rem -s NAME=VALUE
rem -f
rem Set variable NAME to VALUEOpen a new windowStart in rem full screen mode
rem Console connection:-H hostname-U username-P password
rem HostUser name
rem Password for remote connections


rem modify .vmx file to skip confirmation dialog of "Move/Copy"
rem uuid.action = "create"
start "" VMware\vmware.exe -x ros_noetic_humble_gazebov11_linux_win_v1\ros-noetic-humble-gazebo11-linux-win-v1.vmx
ping -n 31 127.0.0.1 >nul

WindowsNoEditor\CarlaUE4.exe

rem sshpass pass password to ssh command to avoid password prompt
rem https://github.com/xhcoding/sshpass-win32/releases/download/v1.0.7/sshpass.exe
sshpass -p password ssh -o StrictHostKeyChecking=no user@192.168.40.133 "export SVGA_VGPU10=0 && export ROS_IP=$(hostname -I | tr -d [:blank:])  && export ROS_MASTER_URI=http://$ROS_IP:11311 && /bin/bash -c 'source /opt/ros/noetic/setup.bash; roscore ' &"


rem manual control
rem source ~/carla-ros-bridge/catkin_ws/devel/setup.bash
rem python -m pip install numpy==1.23.1
rem roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch host:=172.21.108.47 timeout:=60000 town:='Town03' spawn_point:=-25,-134,0.5,0,0,-90


rem exit

cd ..
