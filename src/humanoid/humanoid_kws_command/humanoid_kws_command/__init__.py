# -*- coding: utf-8 -*-
"""人形机器人离线语音指令控制：log-mel + 小型 MLP 的端到端关键词识别（KWS）。

不接 ASR 引擎、不下外部预训练权重，模型参数约 5.9k（< 50 KB）。
"""

from humanoid_kws_command.config import VERSION

__version__ = VERSION

__all__ = ["VERSION", "__version__"]