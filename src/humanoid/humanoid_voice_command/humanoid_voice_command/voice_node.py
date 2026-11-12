"""语音识别节点：口令音频 -> 文本 -> 运动意图。

节点职责（感知层）：
  1. 取一段口令音频（wav 文件 / 目录轮询 / 麦克风）；
  2. Whisper tiny 离线识别成文本；
  3. 意图分类（规则 + 意图网络）；
  4. 发布 ``/voice_text``（识别文本）、``/voice_intent``（意图名）、
     ``/voice_command``（JSON，含置信度与判定来源，便于调试与录制）。

音频来源三选一：
  * ``--wav path.wav``        只处理一个文件后退出（最常用，便于写测试）
  * ``--audio-dir dir``       按文件名顺序轮流播放目录下的 wav，模拟连续口令
  * ``--mic``                 从麦克风采集（需要 sounddevice，缺省不用）
"""

import json
import os
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from humanoid_voice_command.asr import WhisperASR, default_cache_dir
from humanoid_voice_command.intent import (
    INTENTS,
    IntentClassifier,
    load_lexicon,
)


class VoiceNode(Node):
    """音频 -> 文本 -> 意图。"""

    def __init__(self):
        super().__init__('voice_node')

        self.declare_parameter('model', 'tiny')
        self.declare_parameter('model_path', '')          # 本地 Whisper 权重，空则用缓存/下载
        self.declare_parameter('language', 'zh')
        self.declare_parameter('wav', '')
        self.declare_parameter('audio_dir', '')
        self.declare_parameter('use_mic', False)
        self.declare_parameter('interval', 2.5)          # 目录轮询时两条口令的间隔 (s)
        self.declare_parameter('loop', True)             # 目录轮询是否循环
        self.declare_parameter('weights', '')            # 意图网络路径，空则用包内权重
        self.declare_parameter('config', '')             # 词表 yaml，空则用包内配置
        self.declare_parameter('record_json', '')        # 把识别结果写成 json 便于出图

        p = self.get_parameter
        self.model_name = p('model').value
        self.model_path = p('model_path').value
        self.language = p('language').value
        self.wav = p('wav').value
        self.audio_dir = p('audio_dir').value
        self.use_mic = bool(p('use_mic').value)
        self.interval = float(p('interval').value)
        self.loop = bool(p('loop').value)
        self.record_json = p('record_json').value

        share = self._share_dir()
        weights = p('weights').value or os.path.join(share, 'weights', 'intent_mlp.pt')
        cfg = p('config').value or os.path.join(share, 'config', 'intents.yaml')
        self.lexicon = load_lexicon(cfg)
        self.classifier = IntentClassifier(weights_path=weights, lexicon=self.lexicon)

        # ---- 发布者 ----
        self.pub_text = self.create_publisher(String, '/voice_text', 10)
        self.pub_intent = self.create_publisher(String, '/voice_intent', 10)
        self.pub_cmd = self.create_publisher(String, '/voice_command', 10)

        # ---- 识别器（延迟到首次使用时加载，避免空跑时占内存）----
        self.asr: Optional[WhisperASR] = None
        self._records: List[dict] = []
        self._files: List[str] = []
        self._idx = 0

        if self.wav:
            self.create_timer(0.5, self._once_callback)
            self.get_logger().info(f'单文件模式：{self.wav}')
        elif self.audio_dir:
            self._files = sorted(
                os.path.join(self.audio_dir, f) for f in os.listdir(self.audio_dir)
                if f.lower().endswith('.wav'))
            if not self._files:
                self.get_logger().error(f'目录里没有 wav：{self.audio_dir}')
            else:
                self.get_logger().info(f'目录轮询模式：{len(self._files)} 条口令，'
                                       f'间隔 {self.interval:.1f}s')
            self.create_timer(self.interval, self._dir_callback)
        elif self.use_mic:
            self.create_timer(1.0, self._mic_callback)
            self.get_logger().info('麦克风模式')
        else:
            self.get_logger().warn('未指定音频来源，节点空转（可用 --wav / --audio-dir / --mic）')

        self.get_logger().info(
            f'语音识别节点已启动 | 模型 whisper-{self.model_name} | 语言 {self.language} | '
            f'意图类别 {len(INTENTS)}')

    # ------------------------------------------------------------------
    @staticmethod
    def _share_dir() -> str:
        try:
            from ament_index_python.packages import get_package_share_directory
            return get_package_share_directory('humanoid_voice_command')
        except Exception:                                  # noqa: BLE001
            here = os.path.dirname(os.path.abspath(__file__))
            return os.path.abspath(os.path.join(here, '..'))

    def _ensure_asr(self) -> WhisperASR:
        if self.asr is None:
            prompt = ', '.join(
                w for intent in INTENTS for w in self.lexicon.get(intent, []))
            t0 = time.time()
            self.asr = WhisperASR(
                model_name=self.model_name,
                model_path=self.model_path or None,
                download_root=default_cache_dir(),
                language=self.language,
                initial_prompt=prompt if self.language.startswith('zh') else None)
            self.get_logger().info(
                f'Whisper {self.model_name} 加载完成，耗时 {time.time() - t0:.2f}s'
                f'（权重 {self.asr.source}）')
        return self.asr

    # ------------------------------------------------------------------
    def _publish_result(self, wav: str, text: str, latency: float):
        intent, conf, source = self.classifier.classify(text)

        self.pub_text.publish(String(data=text))
        self.pub_intent.publish(String(data=intent))
        payload = {
            'wav': os.path.basename(wav),
            'text': text,
            'intent': intent,
            'confidence': round(conf, 4),
            'source': source,
            'asr_latency': round(latency, 4),
            'stamp': round(time.time(), 3),
        }
        self.pub_cmd.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.get_logger().info(
            f'"{os.path.basename(wav)}" -> "{text}" -> {intent} '
            f'({source}, {conf:.2f}, {latency * 1000:.0f}ms)')

        if self.record_json:
            self._records.append(payload)
            try:
                with open(self.record_json, 'w', encoding='utf-8') as fh:
                    json.dump(self._records, fh, ensure_ascii=False, indent=2)
            except OSError as exc:
                self.get_logger().warn(f'写 json 失败：{exc}')

    def _once_callback(self):
        """单文件模式：识别一次后不再触发。"""
        if not self.wav:
            return
        wav, self.wav = self.wav, ''
        asr = self._ensure_asr()
        res = asr.transcribe(wav)
        self._publish_result(wav, str(res['text']), float(res['latency']))

    def _dir_callback(self):
        """目录轮询：按顺序播放，可循环。"""
        if not self._files:
            return
        if self._idx >= len(self._files):
            if not self.loop:
                return
            self._idx = 0
        wav = self._files[self._idx]
        self._idx += 1
        asr = self._ensure_asr()
        res = asr.transcribe(wav)
        self._publish_result(wav, str(res['text']), float(res['latency']))

    def _mic_callback(self):
        """麦克风模式：录 2 s 音频后识别（需要 sounddevice）。"""
        try:
            import sounddevice as sd
        except ImportError:
            self.get_logger().error('未安装 sounddevice，无法使用麦克风模式')
            self.use_mic = False
            return
        asr = self._ensure_asr()
        self.get_logger().info('开始录音 2.0s ...')
        audio = sd.rec(int(2.0 * 16000), samplerate=16000, channels=1, dtype='float32')
        sd.wait()
        tmp = os.path.join('/tmp', 'voice_mic.wav')
        import soundfile as sf
        sf.write(tmp, audio[:, 0], 16000)
        res = asr.transcribe(tmp)
        self._publish_result(tmp, str(res['text']), float(res['latency']))


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
