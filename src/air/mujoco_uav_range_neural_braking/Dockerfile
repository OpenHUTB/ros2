FROM ros:humble-ros-base-jammy
RUN sed -i 's|http://archive.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g; s|http://security.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g' /etc/apt/sources.list
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip libgl1 libegl1 libglfw3
COPY requirements.txt /tmp/uav_requirements.txt
RUN python3 -m pip install --no-cache-dir -r /tmp/uav_requirements.txt
WORKDIR /work/module
