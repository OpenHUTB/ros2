FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /workspace/ros2

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-venv libgl1-mesa-glx libegl1 \
    && apt-get clean

COPY requirements.txt /tmp/requirements.txt
RUN python3 -m pip install --no-cache-dir -r /tmp/requirements.txt

COPY . /workspace/ros2/mujoco_uav_rooftop_pv_coverage_survey
RUN python3 -m pip install --no-cache-dir -e /workspace/ros2/mujoco_uav_rooftop_pv_coverage_survey

ENTRYPOINT ["/bin/bash", "-lc"]
CMD ["source /opt/ros/humble/setup.bash && python3 /workspace/ros2/mujoco_uav_rooftop_pv_coverage_survey/main.py --mode all --output /workspace/ros2/artifacts/day2"]
