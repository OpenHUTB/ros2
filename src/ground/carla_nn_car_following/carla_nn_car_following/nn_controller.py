# -*- coding: utf-8 -*-
"""神经网络控制器：感知网络估计车距与相对速度，控制网络输出油门与刹车。

与经典控制器（``reference_acc.ReferenceACC``）的区别：

* 经典控制器：直接使用 CARLA 给出的**真值**车距与相对速度（相当于"上帝视角"）；
* 本控制器：只使用**雷达原始回波统计量**（``FrontRadar.features``），
  先由感知网络估计车距与相对速度，再由控制网络给出油门与刹车。

两者的接口保持一致（都提供 ``labels(...)``），便于在同一场景里做对比实验。

部署时需要保证输入顺序与训练时一致，见 ``train.py`` 的 ``PERCEPTION_INPUTS``。
"""

import os

import numpy as np

from .nn_numpy import MLP, Normalizer


def candidate_models_dirs(models_dir):
    """把 ``models_dir`` 展开成一组候选目录（按优先级排序）。

    为什么需要它：``ros2 launch`` 的工作目录是**启动它的那个目录**，而不是功能包目录，
    所以把 ``models_dir`` 默认写成相对路径 ``models`` 时，从别处启动就会找不到模型。
    这里做一个最小可预测的搜索：当前工作目录 → 本功能包源码目录 → ``CARLA_NN_MODELS_DIR``。

    ``__file__`` 用 ``realpath`` 解析，这样即使 ``colcon build --symlink-install``
    把文件软链到 ``install/`` 下，也能回溯到**源码目录**。
    """
    expanded = os.path.expanduser(os.path.expandvars(models_dir))
    if os.path.isabs(expanded):
        return [expanded]

    candidates = [os.path.abspath(expanded)]
    # 本文件位于 <包根>/carla_nn_car_following/nn_controller.py
    package_root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    candidates.append(os.path.join(package_root, expanded))
    env_dir = os.environ.get('CARLA_NN_MODELS_DIR')
    if env_dir:
        candidates.append(os.path.join(os.path.expanduser(env_dir), expanded))
    return candidates


class NNController:
    """感知网络 + 控制网络 的级联控制器。"""

    def __init__(self, perception, perception_meta, control, control_meta):
        self.perception = perception
        self.control = control
        self.p_in = Normalizer(perception_meta['in_mean'], perception_meta['in_std'])
        self.p_out = Normalizer(perception_meta['out_mean'], perception_meta['out_std'])
        self.c_in = Normalizer(control_meta['in_mean'], control_meta['in_std'])
        self.c_out = Normalizer(control_meta['out_mean'], control_meta['out_std'])

        # 训练集里各输入/输出的取值范围。部署时把输入裁剪回该范围，
        # 避免 MLP 在训练分布之外"外推"出离谱结果（不裁剪时实测估计误差
        # 会从 ±1 m 恶化到 −21 m，进而导致自车几乎追尾）。
        self.p_in_min = perception_meta.get('in_min')
        self.p_in_max = perception_meta.get('in_max')
        self.p_out_min = perception_meta.get('out_min')
        self.p_out_max = perception_meta.get('out_max')
        self.c_in_min = control_meta.get('in_min')
        self.c_in_max = control_meta.get('in_max')

    @staticmethod
    def _clip(values, low, high):
        """把数值裁剪到 [low, high]；缺省（旧模型）时不做处理。"""
        if low is None or high is None:
            return values
        return np.minimum(np.maximum(values, low), high)

    # ---------------- 载入 ----------------
    @classmethod
    def load(cls, models_dir):
        """载入两个网络的权重。

        ``models_dir`` 支持三种写法（见 :func:`candidate_models_dirs`）：

        1. **绝对路径** —— 最稳妥，推荐在 `ros2 launch` 时传入；
        2. 相对路径 —— 依次在「当前工作目录」「本功能包源码目录」「``CARLA_NN_MODELS_DIR`` 环境变量」
           下查找，因此从任意目录启动都能找到（``ros2 launch`` 的工作目录是启动它的那个目录）；
        3. ``~`` 与环境变量会被展开。

        全部找不到时抛出 :class:`FileNotFoundError`，并把**尝试过的目录逐个列出**，
        便于使用者定位问题（也便于评审独立核验）。
        """
        tried = []
        for directory in candidate_models_dirs(models_dir):
            tried.append(directory)
            perception_path = os.path.join(directory, 'perception.npz')
            control_path = os.path.join(directory, 'control.npz')
            if os.path.isfile(perception_path) and os.path.isfile(control_path):
                print('[模型] 已从 %s 载入神经网络权重' % directory)
                perception, perception_meta = MLP.load(perception_path)
                control, control_meta = MLP.load(control_path)
                return cls(perception, perception_meta, control, control_meta)

        raise FileNotFoundError(
            '未找到模型文件 perception.npz / control.npz，已尝试以下目录：\n  - %s\n'
            '请先训练模型（python3 main.py --mode collect 然后 --mode train），'
            '或把 models_dir 参数写成**绝对路径**，例如：\n'
            '  ros2 launch carla_nn_car_following main.launch.py controller:=nn '
            'models_dir:=$HOME/ros2_ws/src/carla_nn_car_following/models'
            % '\n  - '.join(tried))

    # ---------------- 感知 ----------------
    def estimate(self, features, v_ego):
        """由雷达特征估计 (车距, 相对速度)。

        ``features`` 必须来自 ``carla_client.FrontRadar.features()``，
        顺序与 ``train.PERCEPTION_INPUTS`` 一致。
        """
        vector = np.array([[features['d_radar'], features['azimuth'], features['closing_rate'],
                            features['mean_depth'], features['n_points'], v_ego]],
                          dtype=np.float64)
        vector = self._clip(vector, self.p_in_min, self.p_in_max)      # 防外推
        output = self.p_out.inverse(self.perception.predict(self.p_in.transform(vector)))[0]
        output = self._clip(output, self.p_out_min, self.p_out_max)    # 估计值限制在训练范围内
        return float(output[0]), float(output[1])

    # ---------------- 控制 ----------------
    def labels(self, features, v_ego):
        """返回 ``(throttle, brake, d_hat, v_rel_hat)``。"""
        gap_hat, v_rel_hat = self.estimate(features, v_ego)
        vector = np.array([[gap_hat, v_rel_hat, v_ego]], dtype=np.float64)
        vector = self._clip(vector, self.c_in_min, self.c_in_max)      # 防外推
        output = self.c_out.inverse(self.control.predict(self.c_in.transform(vector)))[0]

        throttle = float(np.clip(output[0], 0.0, 1.0))
        brake = float(np.clip(output[1], 0.0, 1.0))
        # 训练标签中油门与刹车是互斥的，这里做同样的后处理，避免同时踩下
        if throttle >= brake:
            brake = 0.0
        else:
            throttle = 0.0
        return throttle, brake, gap_hat, v_rel_hat
