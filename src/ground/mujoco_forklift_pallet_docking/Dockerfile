FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV MUJOCO_GL=egl

RUN apt-get update && apt-get install -y --no-install-recommends \
    libegl1 libgl1 libglfw3 python3-colcon-common-extensions python3-pip python3-pytest

WORKDIR /ros_ws
COPY . src/mujoco_forklift_pallet_docking
RUN python3 -m pip install --no-cache-dir -r src/mujoco_forklift_pallet_docking/requirements.txt
RUN /bin/bash -lc "source /opt/ros/humble/setup.bash && colcon build --merge-install --packages-select mujoco_forklift_pallet_docking && source install/setup.bash && export AMENT_PREFIX_PATH=/ros_ws/install:\$AMENT_PREFIX_PATH && export PYTHONPATH=/ros_ws/install/lib/python3.10/site-packages:\$PYTHONPATH && python3 -m pytest -q src/mujoco_forklift_pallet_docking/test && ros2 pkg prefix mujoco_forklift_pallet_docking"

CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && source install/setup.bash && export AMENT_PREFIX_PATH=/ros_ws/install:$AMENT_PREFIX_PATH && export PYTHONPATH=/ros_ws/install/lib/python3.10/site-packages:$PYTHONPATH && ros2 launch mujoco_forklift_pallet_docking main.launch.py"]
