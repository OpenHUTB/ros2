FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive \
    MUJOCO_GL=egl \
    AMENT_PREFIX_PATH=/workspace/ros_ws/install/mujoco_uav_moving_pad_landing:/opt/ros/humble \
    MUJOCO_LANDING_MODEL=/workspace/output/landing_controller.joblib \
    MUJOCO_LANDING_IMAGE_MODEL=/workspace/output/image_controller.joblib \
    PYTHONUNBUFFERED=1

RUN sed -i 's|http://archive.ubuntu.com/ubuntu|https://archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list \
    && apt-get update -o Acquire::Retries=5 \
    && apt-get -o Acquire::Retries=5 install -y --no-install-recommends \
      python3-pip libegl1 libgl1 libglvnd0 libgl1-mesa-dri libglib2.0-0 \
    && apt-get clean

WORKDIR /workspace
COPY requirements.txt ./requirements.txt
RUN python3 -m pip install --no-cache-dir -r requirements.txt
COPY . ./mujoco_uav_moving_pad_landing
RUN mkdir -p /workspace/ros_ws/src \
    && ln -s /workspace/mujoco_uav_moving_pad_landing /workspace/ros_ws/src/mujoco_uav_moving_pad_landing \
    && . /opt/ros/humble/setup.sh \
    && cd /workspace/ros_ws \
    && colcon build --packages-select mujoco_uav_moving_pad_landing

ENV PYTHONPATH=/workspace/mujoco_uav_moving_pad_landing
CMD ["bash", "-lc", ". /opt/ros/humble/setup.bash && . /workspace/ros_ws/install/setup.bash && python3 /workspace/mujoco_uav_moving_pad_landing/main.py demo --output /workspace/output"]
