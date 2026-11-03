"""PX4 + ROS2 + OpenCV 无人机视觉目标追踪功能包。

包含三个可执行节点：
- image_pub：发布合成相机图像（/camera/image_raw）
- color_tracker：HSV 颜色阈值检测，发布目标偏移与面积（/vision/target）
- offboard_control：把图像偏差转成 PX4 Offboard 速度指令
"""
