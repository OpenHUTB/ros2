# -*- coding: utf-8 -*-
"""controllers.py —— 三种跟踪控制器：几何专家 / 两阶段(感知网+控制网) / 端到端。

- ExpertController：传统几何视觉伺服（投影 + PI 律），作为**示教数据来源**
  与性能基线，不使用任何神经网络；
- TwoStageController：感知网络输出目标像素，再由控制网络输出速度指令；
- E2EController：图像直接映射到速度指令（课程任务点 5*1 的端到端形式）。

三者的特征定义完全一致（含两个积分状态），保证对比公平：纯比例律跟匀速
目标会留有稳态偏差，因此专家里加了对"横向像素误差"和"视在尺寸误差"的
积分项，而积分状态也一并喂给学习控制器，否则克隆出来的网络无法复现积分作用。
"""
import numpy as np

from . import params as P
from .nn import MLP

FEAT_DIM = 7


# ---------------- 特征 ----------------
def pixel_errors(det):
    """归一化像素误差：ex 向右为正，ey 向下为正(远离中心)。

    对地面目标，ey 是距离的单调函数（越远越靠上），因此距离环直接用 ey。
    """
    ex = (det['u'] - (P.IMG_W - 1) / 2.0) / (P.IMG_W / 2.0)
    ey = (det['v'] - (P.IMG_H - 1) / 2.0) / (P.IMG_H / 2.0)
    return float(ex), float(ey)


class ServoState:
    """只保留一个积分状态：图像纵坐标误差的积分。

    对地面目标，"目标在画面里的纵向位置"本身就是距离的单调函数，因此纵坐标
    误差可以直接作为距离环的误差量；积分项用小权限、仅在小误差区间内作用，
    用来消掉匀速跟飞时的稳态偏差，同时避免机动阶段积分饱和。
    """

    def __init__(self):
        self.ie = 0.0

    def update(self, ey, dt):
        if abs(ey) < P.I_BAND:
            self.ie = float(np.clip(self.ie + ey * dt / P.TAU_I,
                                    -P.I_LIM, P.I_LIM))

    def features(self, ex, ey, z, vx_prev, vy_prev, visible):
        return np.array([
            ex, ey,
            vx_prev / P.V_MAX, vy_prev / P.V_MAX,
            (P.H_DES - z) / 3.0,
            1.0 if visible else 0.0,
            self.ie,
        ], dtype=float)


def detection_from_gt(gt):
    """把几何真值包装成与感知网络输出同构的字典。"""
    return {'u': gt['u'], 'v': gt['v'],
            'vis': 1.0 if gt['visible'] else 0.0}


def pointing_yaw(quad_yaw, ex, lost_steps):
    """机头指向律：目标在画面右侧就右转，使光轴始终对准目标。

    对所有控制器一致（几何律，不属于被学习的策略），否则横向误差会
    与距离环互相拉扯，比较也失去公平。

    丢失目标时先"记忆保持"约 2 s（机头不动，被遮挡的目标通常从原方位重新
    出现，此时无需转动即可重新捕获），仍未捕获再以 1.2 rad/s 扫描整周。
    """
    if lost_steps > 0:
        if lost_steps <= 40:
            return quad_yaw
        return quad_yaw + 0.06
    return quad_yaw + float(np.clip(1.2 * ex, -0.6, 0.6))


def expert_command(feat, visible):
    """几何专家（IBVS 比例+纵坐标积分）：光轴指向由偏航律负责，
    这里只算速度：纵向误差+视在尺寸共同决定前飞速度，垂直保持定高。
    """
    ex, ey, dz, ie = feat[0], feat[1], feat[4], feat[6]
    if not visible:
        # 丢失后悬停等待：继续前飞会靠近遮挡物，反而让目标被挡更久
        return np.zeros(3), True
    vx = np.clip(P.KP_EY * (-ey) + P.KI_EY * ie, -P.V_MAX, P.V_MAX)
    vy = np.clip(P.KP_LAT * ex, -P.V_MAX, P.V_MAX)
    vz = np.clip(0.9 * dz * 3.0, -P.VZ_MAX, P.VZ_MAX)
    return np.array([vx, vy, vz]), False


class BaseController:
    name = 'base'
    label = 'base'                 # 图表里用的英文标签（避免缺字形）
    kind = 'learned'
    # 该控制器的输入是否直接是图像（决定 DAgger 是否需要收集图像样本）
    image_input = False

    def reset(self):
        self.v_prev = np.zeros(3)
        self.lost_steps = 0
        self.servo = ServoState()
        self.ex = 0.0

    def _post(self, cmd):
        cmd = np.asarray(cmd, dtype=float)
        cmd[0:2] = np.clip(cmd[0:2], -P.V_MAX, P.V_MAX)
        cmd[2] = np.clip(cmd[2], -P.VZ_MAX, P.VZ_MAX)
        self.v_prev = cmd
        return cmd

    def yaw_des(self, quad, lost):
        """机头指向目标；丢失时先记忆保持再慢速扫描。"""
        return pointing_yaw(quad.yaw, self.ex, self.lost_steps)

    def step(self, img, quad, dt, gt=None):
        raise NotImplementedError


# ---------------- 1) 几何专家 ----------------
class ExpertController(BaseController):
    name = '几何视觉伺服(专家)'
    label = 'geometric servo (expert)'
    kind = 'baseline'

    def step(self, img, quad, dt, gt=None):
        det = detection_from_gt(gt)
        ex, ey = pixel_errors(det)
        self.ex = ex
        self.servo.update(ey, dt)
        feat = self.servo.features(ex, ey, quad.p[2],
                                   self.v_prev[0], self.v_prev[1],
                                   det['vis'] >= 0.5)
        cmd, lost = expert_command(feat, det['vis'] >= 0.5)
        self.lost_steps = self.lost_steps + 1 if lost else 0
        return self._post(cmd)


# ---------------- 2) 两阶段：感知网络 + 控制网络 ----------------
class ControllerNet:
    """控制网络：9 维特征 -> 3 维归一化速度指令。"""

    def __init__(self, seed=0):
        self.net = MLP([FEAT_DIM] + list(P.CTRL_HIDDEN) + [3], seed=seed)

    def fit(self, X, Y, epochs=120, lr=2e-3, verbose=False):
        return self.net.fit(X, Y, epochs=epochs, batch=64, lr=lr,
                            verbose=verbose)

    def predict(self, feat):
        out = self.net.predict(feat)[0]
        return np.array([out[0] * P.V_MAX, out[1] * P.V_MAX, out[2] * P.VZ_MAX])

    def save(self, path):
        self.net.save(path)

    @classmethod
    def load(cls, path):
        obj = cls()
        obj.net = MLP.load(path)
        return obj


class TwoStageController(BaseController):
    name = '两阶段(感知网+控制网)'
    label = 'two-stage (perception + control net)'
    kind = 'learned'

    def __init__(self, detector, control):
        self.detector = detector
        self.control = control
        self.reset()

    def step(self, img, quad, dt, gt=None):
        det = self.detector.detect(img)
        ex, ey = pixel_errors(det)
        self.ex = ex
        self.servo.update(ey, dt)
        if det['vis'] < 0.5:
            self.lost_steps += 1
            self.last_feat = None
            return self._post(np.zeros(3))
        self.lost_steps = 0
        feat = self.servo.features(ex, ey, quad.p[2],
                                   self.v_prev[0], self.v_prev[1], True)
        self.last_feat = feat          # 供 DAgger 重新标注
        return self._post(self.control.predict(feat))

    def features_from_img(self, img, v_prev=None):
        """两阶段不直接吃图像，这里只为接口兼容而保留（忽略 v_prev）。"""
        return self.detector.features(img)


# ---------------- 3) 端到端：图像 -> 指令 ----------------
class E2ENet:
    def __init__(self, seed=0):
        n_in = P.E2E_DS * P.E2E_DS * 3 + (3 if P.E2E_CMD_FEEDBACK else 0)
        self.net = MLP([n_in] + list(P.E2E_HIDDEN) + [3], seed=seed)

    def fit(self, X, Y, epochs=80, lr=1.5e-3, verbose=False):
        return self.net.fit(X, Y, epochs=epochs, batch=64, lr=lr,
                            verbose=verbose)

    def predict_img(self, feat):
        out = self.net.predict(feat)[0]
        return np.array([out[0] * P.V_MAX, out[1] * P.V_MAX, out[2] * P.VZ_MAX])

    def save(self, path):
        self.net.save(path)

    @classmethod
    def load(cls, path):
        obj = cls()
        obj.net = MLP.load(path)
        return obj


class E2EController(BaseController):
    name = '端到端(图像->指令)'
    label = 'end-to-end (image -> command)'
    kind = 'learned'
    image_input = True

    def __init__(self, net, detector_features=None):
        self.net = net
        self.features = detector_features
        self.reset()

    def features_from_img(self, img, v_prev=None):
        """端到端输入：图像特征(+上一步指令，提供速度记忆)。"""
        f = self.features(img)
        if P.E2E_CMD_FEEDBACK:
            v = np.zeros(3) if v_prev is None else np.asarray(v_prev, float)
            f = np.concatenate([f, v / np.array([P.V_MAX, P.V_MAX, P.VZ_MAX])])
        return f

    def step(self, img, quad, dt, gt=None):
        if self.features is not None:
            feat = self.features_from_img(img, self.v_prev)
        else:
            feat = np.zeros(P.E2E_DS * P.E2E_DS * 3)
        self.last_feat = feat          # 供 DAgger 重新标注
        cmd = self.net.predict_img(feat)
        if float(np.abs(cmd).sum()) < 1e-3:
            self.lost_steps += 1
        else:
            self.lost_steps = 0
        return self._post(cmd)
