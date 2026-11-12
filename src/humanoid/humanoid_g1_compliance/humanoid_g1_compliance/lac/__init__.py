"""从上游 lac-humanoid/lac-code vendored 的推理侧模块（MIT）。

只保留部署路径需要的部分：网络结构、观测构建、动作映射、权重加载。
上游的 ``inference.py`` / ``relay.py`` / ``stiffness_control.py`` 依赖
``unitree_hg`` DDS 消息与 Xbox 手柄，这里没有搬运——对应功能由
``humanoid_g1_compliance`` 自己的 ``policy.py`` / 节点重写。

各文件的原始内容未做逻辑修改，仅补了本说明。来源与许可见仓库根
``THIRD_PARTY.md``。
"""
