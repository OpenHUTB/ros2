@echo off


if not exist "ros2" (
    git clone https://OpenHUTB:T8w6TYB_r71gGTP3A02B@git.code.tencent.com/OpenHUTB/ros2.git   &&  cd ros2  && git lfs pull
) else (
    cd ros2  && git pull
    git lfs pull
)
