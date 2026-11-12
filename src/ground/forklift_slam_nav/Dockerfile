FROM mujoco-forklift-pallet-docking:latest
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y ros-humble-slam-toolbox ros-humble-nav2-bringup && apt-get clean
COPY src/ground/forklift_slam_nav/requirements.txt /tmp/forklift-slam-nav-requirements.txt
RUN python3 -m pip install --no-cache-dir -r /tmp/forklift-slam-nav-requirements.txt
WORKDIR /ws
COPY src/ground/forklift_slam_nav /ws/src/forklift_slam_nav
RUN bash -lc 'source /opt/ros/humble/setup.bash && colcon build --merge-install --symlink-install'
CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && export AMENT_PREFIX_PATH=/ws/install:$AMENT_PREFIX_PATH && ros2 launch forklift_slam_nav bringup.launch.py"]
