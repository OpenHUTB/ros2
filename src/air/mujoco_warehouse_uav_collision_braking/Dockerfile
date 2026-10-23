FROM ros:humble-ros-base-jammy
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip libgl1 libosmesa6 && apt-get clean
WORKDIR /workspace/src/mujoco_warehouse_uav_collision_braking
COPY requirements.txt .
RUN python3 -m pip install --no-cache-dir -r requirements.txt
COPY . .
WORKDIR /workspace
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select mujoco_warehouse_uav_collision_braking
ENV MUJOCO_GL=osmesa
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash", "-lc", "source /workspace/install/setup.bash && ros2 launch mujoco_warehouse_uav_collision_braking main.launch.py"]
