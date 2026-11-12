FROM ros:humble-ros-base-jammy
ENV DEBIAN_FRONTEND=noninteractive MUJOCO_GL=egl
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-pytest python3-colcon-common-extensions libegl1 libgl1 libglfw3 fonts-wqy-zenhei
WORKDIR /ws
COPY . src/mujoco_muscle_gripper
RUN python3 -m pip install --no-cache-dir numpy==1.26.4 mujoco==3.3.7 Pillow==11.3.0 matplotlib==3.9.4
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --merge-install --packages-select mujoco_muscle_gripper && source install/setup.bash && ros2 pkg prefix mujoco_muscle_gripper"
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 launch mujoco_muscle_gripper main.launch.py mode:=bench out:=/results"]
