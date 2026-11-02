#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
safety.py —— 部署安全层：权重加载校验 + 输出合法性校验（不依赖 rospy，便于本地回归测试）。

安全约定：
1. 部署节点在权重缺失或加载失败时必须安全停止，绝不允许随机初始化模型驱动无人机；
2. 随机权重只允许在离线自检场景由调用方显式开启（allow_random=True）；
3. 模型输出出现 NaN / inf 等非法值时，禁止下发速度指令。
"""
import os
import math


class WeightMissingError(RuntimeError):
    """权重文件不存在。部署节点应安全停止，不得使用随机模型继续飞行。"""


class WeightLoadError(RuntimeError):
    """权重文件存在但加载/解析/匹配失败（损坏、格式不符、结构不匹配等）。"""


def _clip(value, lower, upper):
    """纯 Python 限幅，避免在安全层引入第三方数值库依赖。"""
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value


def load_trained_model(model_path, device, build_model_fn, allow_random=False, log=None):
    """加载训练好的 CNN 权重。

    参数:
        model_path: 权重文件路径。
        device: torch 设备（仅在真正加载或回退随机模型时使用）。
        build_model_fn: 无参可调用对象，返回随机初始化模型；
                        仅用于离线自检回退与构建待加载结构。
        allow_random: 是否允许权重缺失时回退到随机模型。
                      部署节点必须保持 False；仅离线自检可显式传 True。
        log: 可选日志对象，需提供 info/warn/error 方法；为 None 时静默。

    返回:
        已加载权重并处于 eval 模式的模型。

    抛出:
        WeightMissingError: 文件不存在且未允许随机权重（部署侧必须安全停止）。
        WeightLoadError: 文件存在但加载 / 匹配失败。
    """
    def _log(level, msg):
        if log is not None and hasattr(log, level):
            getattr(log, level)(msg)

    if not os.path.exists(model_path):
        if allow_random:
            _log("warn",
                 "权重 %s 缺失，自检模式下使用随机权重。"
                 % model_path)
            model = build_model_fn().to(device)
            model.eval()
            return model
        raise WeightMissingError(
            "权重文件不存在：%s。部署节点将安全停止，请先运行 train.py 训练并生成权重；"
            "禁止在缺失权重时使用随机模型驱动无人机。" % model_path)

    # 文件存在：尝试加载，任何异常都视为加载失败并安全停止，不静默继续。
    try:
        import torch
        model = build_model_fn().to(device)
        ckpt = torch.load(model_path, map_location=device)
        state_dict = ckpt.get("state_dict", ckpt) if isinstance(ckpt, dict) else ckpt
        # strict=True：键缺失 / 多余 / 形状不符立即报错，杜绝静默加载错模型。
        model.load_state_dict(state_dict, strict=True)
        model.eval()
        _log("info", "模型权重已加载：%s" % model_path)
        return model
    except WeightMissingError:
        raise
    except Exception as exc:  # 覆盖 torch.load、load_state_dict 等一切加载失败
        raise WeightLoadError(
            "权重文件 %s 加载失败：%s。部署节点将安全停止，请检查权重是否损坏"
            "或与当前模型结构匹配。" % (model_path, exc)) from exc


def sanitize_action(action, max_vx, max_vy, max_yaw):
    """校验并限幅模型输出。

    参数:
        action: 长度为 3 的序列 [vx, vy, yaw]（numpy 数组 / list / tuple 均可）。
        max_vx, max_vy, max_yaw: 三轴限幅（正数）。

    返回:
        (vx, vy, yaw) 已限幅的 float 元组；输出非法（None、长度不对、NaN、inf、
        不可转 float）时返回 None，调用方不得下发速度指令。
    """
    if action is None or len(action) != 3:
        return None

    values = []
    for v in action:
        try:
            fv = float(v)
        except (TypeError, ValueError):
            return None
        # NaN 或无穷大视为非法输出，禁止下发。
        if math.isnan(fv) or math.isinf(fv):
            return None
        values.append(fv)

    vx, vy, yaw = values
    vx = _clip(vx, -max_vx, max_vx)
    vy = _clip(vy, -max_vy, max_vy)
    yaw = _clip(yaw, -max_yaw, max_yaw)
    return (vx, vy, yaw)
