FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-pip ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install --no-cache-dir --upgrade packaging 'numpy<2' pillow mujoco \
    && python3 -m pip install --no-cache-dir --index-url https://download.pytorch.org/whl/cpu torch

WORKDIR /workspace
COPY . /workspace
RUN chmod +x /workspace/src/air/mujoco_abandoned_park_uav_visual_steering/visual_steering_node
RUN . /opt/ros/humble/setup.sh \
    && colcon build --packages-select mujoco_abandoned_park_uav_visual_steering

ENTRYPOINT ["/bin/bash", "-lc"]
CMD ["source /opt/ros/humble/setup.bash && export AMENT_PREFIX_PATH=/workspace/install/mujoco_abandoned_park_uav_visual_steering:$AMENT_PREFIX_PATH && ros2 launch mujoco_abandoned_park_uav_visual_steering main.launch.py"]
