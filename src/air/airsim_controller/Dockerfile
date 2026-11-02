# Docker 运行环境（Ubuntu 20.04 + ROS Noetic + AirSim 客户端）
# 注意：AirSim 仿真器本身是带图形界面的 Unreal 可执行文件，容器内只跑控制/ROS 节点；
# 仿真器跑在宿主机（Windows/Ubuntu），容器通过网络 192.168.x.x:41451 连接。
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-opencv python3-pil python3-pynput \
      git && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /app/requirements.txt
RUN pip3 install --no-cache-dir -r /app/requirements.txt torch --extra-index-url https://download.pytorch.org/whl/cpu

WORKDIR /app
COPY . /app

# 连接宿主机 AirSim（在 -e AIRSIM_HOST=192.168.x.x 时由客户端读取）
ENV AIRSIM_HOST=host.docker.internal
CMD ["bash"]
