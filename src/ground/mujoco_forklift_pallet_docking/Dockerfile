FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV MUJOCO_GL=egl

RUN --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    fonts-noto-cjk libegl1 libgl1 libglfw3 python3-colcon-common-extensions python3-pip python3-pytest

WORKDIR /ros_ws
COPY requirements.txt src/mujoco_forklift_pallet_docking/requirements.txt
RUN python3 -m pip install --no-cache-dir -r src/mujoco_forklift_pallet_docking/requirements.txt
COPY . src/mujoco_forklift_pallet_docking
RUN /bin/bash -lc "source /opt/ros/humble/setup.bash && colcon build --merge-install --packages-select mujoco_forklift_pallet_docking && source install/setup.bash && ros2 pkg prefix mujoco_forklift_pallet_docking"

CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch mujoco_forklift_pallet_docking main.launch.py"]
