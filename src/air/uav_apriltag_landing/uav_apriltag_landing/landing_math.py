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


def decide_next_state(current_state, tag_detected, dx_px, dy_px,
                      align_tol, height, landed_h):
    """根据当前状态和传感器输入, 返回下一状态。纯函数, 可独立测试。"""
    if current_state == ST_SEARCH:
        return ST_ALIGN if tag_detected else ST_SEARCH
    if current_state == ST_ALIGN:
        if not tag_detected:
            return ST_SEARCH
        if abs(dx_px) < align_tol and abs(dy_px) < align_tol:
            return ST_DESCEND
        return ST_ALIGN
    if current_state == ST_DESCEND:
        if height <= landed_h + 0.05:
            return ST_LANDED
        if not tag_detected:
            return ST_ALIGN
        return ST_DESCEND
    return ST_LANDED


def compute_align_target(current_target, dx_px, dy_px, height, fx, fy, hover_height):
    """ALIGN 状态: 计算新目标位置 (开环累加)。"""
    gx, gy = pixel_to_ground(dx_px, dy_px, height, fx, fy)
    tx = current_target[0] + gx
    ty = current_target[1] + gy
    return (tx, ty, hover_height)


def compute_descend_target(current_target, dx_px, dy_px, height, fx, fy,
                           descend_step, landed_h):
    """DESCEND 状态: 计算新目标位置, 带高度反馈防止连续下压。"""
    gx, gy = pixel_to_ground(dx_px, dy_px, height, fx, fy)
    tx = current_target[0] + gx * 0.5
    ty = current_target[1] + gy * 0.5
    cur_z = current_target[2]
    if height >= cur_z - 0.15:
        new_z = max(cur_z - descend_step, landed_h)
    else:
        new_z = cur_z
    return (tx, ty, new_z)


def timeout_target(current_target, height, landed_h):
    """传感器超时: 保持当前水平位置, 高度取当前实际高度 (悬停)。"""
    return (current_target[0], current_target[1], max(height, landed_h))
