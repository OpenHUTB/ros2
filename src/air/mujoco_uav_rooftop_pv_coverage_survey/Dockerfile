FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /workspace/ros2

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-venv python3-colcon-common-extensions \
    libgl1-mesa-glx libegl1 libosmesa6 libosmesa6-dev \
    && apt-get clean

COPY requirements.txt /tmp/requirements.txt
RUN sed '/^torch/d' /tmp/requirements.txt > /tmp/requirements-docker.txt \
    && python3 -m pip install --no-cache-dir -r /tmp/requirements-docker.txt \
    && python3 -m pip install --no-cache-dir --index-url https://download.pytorch.org/whl/cpu torch==2.5.1+cpu

COPY . /workspace/ros2/mujoco_uav_rooftop_pv_coverage_survey
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --base-paths /workspace/ros2/mujoco_uav_rooftop_pv_coverage_survey --install-base /opt/rooftop_pv"

ENTRYPOINT ["/bin/bash", "-lc"]
CMD ["source /opt/ros/humble/setup.bash && source /opt/rooftop_pv/setup.bash && ros2 launch mujoco_uav_rooftop_pv_coverage_survey rooftop_pv_workflow.launch.py mode:=all output:=/workspace/ros2/artifacts/day2"]
