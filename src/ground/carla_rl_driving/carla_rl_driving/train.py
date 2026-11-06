# -*- coding: utf-8 -*-
"""
训练入口：train（任务二/四 + 对比实验）

两条对比轴：
  * 同模型不同 RL 算法 ：--algo ppo | sac      （同 CNN 骨干，换 on/off-policy 算法）
  * 同 RL 算法不同结构 ：--arch cnn | cnn_lstm  （同 PPO，换纯 CNN vs CNN+GRU 时序结构）

底层算法实现调用 Stable-Baselines3（PPO / SAC），只把我们自己的网络骨干
（models.CNNBackbone）作为自定义 feature extractor 插进去。

典型命令：
    python -m carla_rl_driving.train --algo ppo --arch cnn        --scenario town01
    python -m carla_rl_driving.train --algo ppo --arch cnn_lstm  --scenario town01
    python -m carla_rl_driving.train --algo sac --arch cnn        --scenario town02
"""

import argparse
import os


def make_features_extractor(arch):
    """返回一个 SB3 BaseFeaturesExtractor 子类，骨干由 --arch 决定（结构消融）。"""
    from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
    import torch.nn as nn
    from .models import CNNBackbone

    class _Extractor(BaseFeaturesExtractor):
        def __init__(self, observation_space, features_dim=256):
            super().__init__(observation_space, features_dim)
            self.backbone = CNNBackbone(in_channels=3)
            self._arch = arch

        def forward(self, observations):
            # observations: (B,3,H,W)
            if self._arch == 'cnn_lstm':
                # 单帧推理时把时间维补 1；批量训练里可在 env 侧堆叠多帧
                x = observations.unsqueeze(1)          # (B,1,3,H,W)
                b, t = x.shape[0], x.shape[1]
                f = self.backbone(x.reshape(b * t, *x.shape[2:])).reshape(b, t, -1)
                return f[:, -1, :]
            return self.backbone(observations)

    _Extractor.__name__ = 'Extractor_%s' % arch
    return _Extractor


def build_env(scenario):
    from .env import CarlaRLEnv
    town_map = {
        'town01': 'Town01',      # 直道 + 环岛
        'town02': 'Town02',     # 窄路弯道
        'town04': 'Town04',      # 多车道匝道/路口
    }
    town = town_map.get(scenario, 'Town01')
    return CarlaRLEnv(town=town, target_speed=5.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--algo', default='ppo', choices=['ppo', 'sac'])
    ap.add_argument('--arch', default='cnn', choices=['cnn', 'cnn_lstm'])
    ap.add_argument('--scenario', default='town01',
                    choices=['town01', 'town02', 'town04'])
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=2000)
    ap.add_argument('--steps', type=int, default=200000)
    ap.add_argument('--out', default='models')
    args = ap.parse_args()

    from stable_baselines3 import PPO, SAC
    from stable_baselines3.common.callbacks import CheckpointCallback

    env = build_env(args.scenario)
    extractor = make_features_extractor(args.arch)
    policy_kwargs = dict(features_extractor_class=extractor,
                         features_extractor_kwargs=dict(features_dim=256))

    os.makedirs(args.out, exist_ok=True)
    tag = '%s_%s_%s' % (args.algo, args.arch, args.scenario)

    if args.algo == 'ppo':
        model = PPO('CnnPolicy', env, policy_kwargs=policy_kwargs,
                    verbose=1, n_steps=256, batch_size=64,
                    learning_rate=3e-4, gamma=0.99, device='auto')
    else:
        model = SAC('CnnPolicy', env, policy_kwargs=policy_kwargs,
                    verbose=1, learning_rate=3e-4, gamma=0.99,
                    buffer_size=50000, device='auto')

    ckpt = CheckpointCallback(save_freq=5000, save_path=args.out, name_prefix=tag)
    model.learn(total_timesteps=args.steps, callback=ckpt)
    save_path = os.path.join(args.out, tag + '.zip')
    model.save(save_path)
    print('已保存训练权重：%s' % save_path)


if __name__ == '__main__':
    main()
