"""安全层：权重加载校验 + 输出合法性校验（纯 Python，不依赖 ROS/torch，可单测）。

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
    """权重文件存在但加载/解析/匹配失败。"""


def _clip(value, lower, upper):
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value


def load_trained_model(model_path, device, build_model_fn, allow_random=False, log=None):
    """加载训练好的 CNN 权重。

    权重缺失且 allow_random=False 时抛 WeightMissingError，部署侧安全停止。
    文件存在但 torch.load / load_state_dict 失败时抛 WeightLoadError。
    """
    def _log(level, msg):
        if log is not None and hasattr(log, level):
            getattr(log, level)(msg)

    if not os.path.exists(model_path):
        if allow_random:
            _log("warn", "权重 %s 缺失，自检模式下使用随机权重。" % model_path)
            model = build_model_fn().to(device)
            model.eval()
            return model
        raise WeightMissingError(
            "权重文件不存在：%s。部署节点将安全停止；"
            "禁止在缺失权重时使用随机模型驱动无人机。" % model_path)

    try:
        import torch
        model = build_model_fn().to(device)
        ckpt = torch.load(model_path, map_location=device)
        state_dict = ckpt.get("state_dict", ckpt) if isinstance(ckpt, dict) else ckpt
        model.load_state_dict(state_dict, strict=True)
        model.eval()
        _log("info", "模型权重已加载：%s" % model_path)
        return model
    except WeightMissingError:
        raise
    except Exception as exc:
        raise WeightLoadError(
            "权重文件 %s 加载失败：%s" % (model_path, exc)) from exc


def sanitize_action(action, max_vx, max_vy, max_yaw):
    """校验并限幅模型输出。

    返回 (vx, vy, yaw) float 元组；非法（None、长度不对、NaN、inf、不可转 float）返回 None。
    """
    if action is None or len(action) != 3:
        return None
    values = []
    for v in action:
        try:
            fv = float(v)
        except (TypeError, ValueError):
            return None
        if math.isnan(fv) or math.isinf(fv):
            return None
        values.append(fv)
    vx, vy, yaw = values
    return (_clip(vx, -max_vx, max_vx),
            _clip(vy, -max_vy, max_vy),
            _clip(yaw, -max_yaw, max_yaw))
