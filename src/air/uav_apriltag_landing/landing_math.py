#!/usr/bin/env python3
"""AprilTag 降落模块纯计算函数 (不依赖 ROS/OpenCV)."""

ST_SEARCH = "SEARCH"
ST_ALIGN = "ALIGN"
ST_DESCEND = "DESCEND"
ST_LANDED = "LANDED"


def pixel_to_ground(dx_px, dy_px, height, fx, fy):
    """像素偏差 (px) 换算为地面平移量 (m)。

    参数:
        dx_px, dy_px: 标签中心相对图像光心的像素偏差
        height: 无人机相对地面的高度 (m)
        fx, fy: 相机焦距 (px)
    返回:
        (dx_m, dy_m): 地面平移量 (m)
    """
    return dx_px * height / fx, dy_px * height / fy
