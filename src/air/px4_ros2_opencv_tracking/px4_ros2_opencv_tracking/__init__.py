"""PX4 + ROS2 + OpenCV 视觉检测与 Offboard 速度控制演示（合成图像输入）。

包含三个可执行节点：
- image_pub：发布合成相机图像（/camera/image_raw，非真实相机）
- color_tracker：HSV 颜色阈值检测，发布 /vision/target 与调试图 /vision/debug_image
- offboard_control：图像偏差 -> PX4 Offboard 速度指令（状态机）
"""
