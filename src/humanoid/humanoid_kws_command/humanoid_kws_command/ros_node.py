# -*- coding: utf-8 -*-
"""ROS 2 节点：离线 WAV -> KWS -> 指令话题。

节点不接麦克风、不做流式解码：给定一个 WAV 文件，按固定周期重复「识别并发布」，
便于在没有声卡的环境（容器/CI）里验证整条链路。识别与前端都在不依赖 rclpy 的
模块里，因此没有 ROS 也能跑 main.py --test。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

import humanoid_kws_command.config as config
import humanoid_kws_command.corpus as corpus
from humanoid_kws_command.kws import MLP

# 指令 -> 底盘速度指令 [线速度 m/s, 角速度 rad/s]
ACTION_TABLE = {
    "forward": (0.30, 0.0), "backward": (-0.20, 0.0),
    "turn_left": (0.0, 0.60), "turn_right": (0.0, -0.60),
    "stop": (0.0, 0.0), "faster": (0.45, 0.0), "slower": (0.15, 0.0),
    "wave": (0.0, 0.0), "crouch": (0.0, 0.0), "stand_up": (0.0, 0.0),
    "unknown": (0.0, 0.0),
}


class KwsNode(Node):
    """把 WAV 文件当成「离线语音流」反复识别。"""

    def __init__(self):
        super().__init__("kws_command_node")
        self.declare_parameter("wav_path", "")
        self.declare_parameter("model_path", "")
        self.declare_parameter("repeat_hz", 0.5)

        wav = str(self.get_parameter("wav_path").value)
        model_path = str(self.get_parameter("model_path").value)
        hz = max(0.1, float(self.get_parameter("repeat_hz").value))

        self.pub_cmd = self.create_publisher(String, "~/command", 10)
        self.pub_act = self.create_publisher(Float64MultiArray, "~/base_command", 10)

        self.model = MLP.load(model_path) if model_path else None
        self.feat = None
        if wav and self.model is not None:
            self.feat = corpus.features_from_wav(wav)[None, :]
            self.get_logger().info(f"已载入 WAV {wav}，模型 {model_path}")
        else:
            self.get_logger().warn("未提供 wav_path / model_path，节点只发 unknown")

        self.create_timer(1.0 / hz, self._on_timer)

    def _on_timer(self):
        label, conf = "unknown", 0.0
        if self.model is not None and self.feat is not None:
            proba = self.model.proba(self.feat)[0]
            idx = int(proba.argmax())
            label = corpus.LABELS[idx]
            conf = float(proba[idx])

        msg = String()
        msg.data = label
        self.pub_cmd.publish(msg)

        act = Float64MultiArray()
        v, w = ACTION_TABLE.get(label, (0.0, 0.0))
        act.data = [v, w, conf]
        self.pub_act.publish(act)
        self.get_logger().info(f"{label}  conf={conf:.2f}  -> v={v:+.2f} w={w:+.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = KwsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()