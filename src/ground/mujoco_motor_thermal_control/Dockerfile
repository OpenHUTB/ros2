FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y python3-pip
RUN python3 -m pip install --no-cache-dir "numpy<2" "mujoco>=3.2,<4" pytest matplotlib pillow

WORKDIR /workspace
COPY src/ground/mujoco_motor_thermal_control /workspace/src/mujoco_motor_thermal_control
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select mujoco_motor_thermal_control"

CMD ["bash"]
