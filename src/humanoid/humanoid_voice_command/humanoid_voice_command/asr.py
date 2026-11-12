"""离线语音识别封装（OpenAI Whisper，tiny 模型）。

设计要点
--------
* **权重来源**：仓库不携带 72 MB 的 Whisper 权重，默认按名字加载
  （``whisper.load_model('tiny')``），权重缓存在 ``~/.cache/whisper``，
  首次运行联网下载一次，之后完全离线。若本机已有副本（``model_path`` 参数 /
  ``HUMANOID_VOICE_WHISPER`` 环境变量 / 包内 ``weights/whisper/tiny.pt``），
  优先使用本地文件，断网也能跑。
* **短口令的重复幻觉**：Whisper 对 1~2 s 的口令音频容易在尾部反复输出同一个词
  （``停止, 停止, 停止, ...``）。原因是 30 s 补零段被当成语音继续解码。
  关闭时间戳输出（``without_timestamps=True``）可消除该现象，同时再对结果做一次
  重复片段折叠作为兜底。
* **词表提示**：把口令词表塞进 ``initial_prompt``，让解码器偏向这些词，
  对 tiny 模型的中文识别提升明显。
"""

import os
import time
from typing import Dict, Optional

import numpy as np

TARGET_SR = 16000


def _require(name: str, hint: str):
    try:
        return __import__(name)
    except ImportError as exc:                            # pragma: no cover
        raise ImportError(f'缺少依赖 {name}：{hint}') from exc


def load_wav_16k(path: str) -> np.ndarray:
    """读 wav（任意采样率/声道）并线性插值重采样到 16 kHz 单声道 float32。

    不依赖 ffmpeg：whisper 自带的 ``load_audio`` 需要系统 ffmpeg，
    而口令音频本来就是 wav，用 soundfile 直接读更省事。
    """
    sf = _require('soundfile', 'pip install soundfile')
    data, sr = sf.read(path, dtype='float32', always_2d=False)
    if data.ndim > 1:
        data = data.mean(axis=1)
    if sr != TARGET_SR:
        n = int(round(len(data) * TARGET_SR / sr))
        if n <= 1:
            return np.zeros(TARGET_SR, dtype=np.float32)
        idx = np.linspace(0.0, len(data) - 1.0, n)
        data = np.interp(idx, np.arange(len(data)), data).astype(np.float32)
    peak = float(np.max(np.abs(data))) if data.size else 0.0
    if peak > 1e-6:
        data = (data / peak * 0.95).astype(np.float32)
    return data


def dedup_repeats(text: str) -> str:
    """折叠 ``停止, 停止, 停止`` / ``左转左转左转`` 这类整段重复。"""
    parts = [p.strip() for p in text.replace('，', ',').split(',') if p.strip()]
    if len(parts) > 1 and len(set(parts)) == 1:
        return parts[0]
    clean = text.strip()
    for size in range(1, len(clean) // 2 + 1):
        if len(clean) % size:
            continue
        unit = clean[:size]
        if unit * (len(clean) // size) == clean:
            return unit
    return clean


class WhisperASR:
    """Whisper tiny 的轻量封装。"""

    def __init__(self, model_name: str = 'tiny',
                 model_path: Optional[str] = None,
                 download_root: Optional[str] = None,
                 language: str = 'zh',
                 initial_prompt: Optional[str] = None):
        whisper = _require('whisper', 'pip install openai-whisper')
        self.model_name = model_name
        self.language = language
        self.initial_prompt = initial_prompt
        t0 = time.time()
        local = model_path or find_local_model(model_name)
        if local:
            # 本地权重：whisper 接受文件路径，直接加载，不联网
            self.model = whisper.load_model(local)
            self.source = local
        else:
            kwargs = {}
            root = download_root or default_cache_dir()
            os.makedirs(root, exist_ok=True)
            kwargs['download_root'] = root
            self.model = whisper.load_model(model_name, **kwargs)
            self.source = os.path.join(root, f'{model_name}.pt')
        self.load_latency = time.time() - t0

    def transcribe(self, wav_path: str) -> Dict[str, object]:
        """识别单个 wav，返回文本、语言、时长与推理耗时。"""
        audio = load_wav_16k(wav_path)
        t0 = time.time()
        lang = None if self.language == 'auto' else self.language
        result = self.model.transcribe(
            audio,
            language=lang,
            fp16=False,
            temperature=0.0,
            condition_on_previous_text=False,
            without_timestamps=True,
            initial_prompt=self.initial_prompt,
        )
        latency = time.time() - t0
        raw = str(result.get('text', ''))
        return {
            'text': dedup_repeats(raw),
            'raw_text': raw,
            'language': result.get('language', lang or ''),
            'duration': len(audio) / TARGET_SR,
            'latency': latency,
            'wav': wav_path,
        }


def default_cache_dir() -> str:
    """whisper 的默认权重缓存目录（``~/.cache/whisper``）。"""
    base = os.environ.get('XDG_CACHE_HOME') or os.path.join(os.path.expanduser('~'), '.cache')
    return os.path.join(base, 'whisper')


def bundled_model_path(model_name: str = 'tiny') -> str:
    """包内可选的自带权重路径（仓库不提供，放进去就会被优先使用）。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('humanoid_voice_command')
    except Exception:                                     # noqa: BLE001
        here = os.path.dirname(os.path.abspath(__file__))
        share = os.path.abspath(os.path.join(here, '..'))
    return os.path.join(share, 'weights', 'whisper', f'{model_name}.pt')


def find_local_model(model_name: str = 'tiny') -> Optional[str]:
    """按优先级找现成的本地权重：环境变量 -> 包内 -> 缓存目录。找不到返回 None。"""
    candidates = [
        os.environ.get('HUMANOID_VOICE_WHISPER', ''),
        bundled_model_path(model_name),
        os.path.join(default_cache_dir(), f'{model_name}.pt'),
    ]
    for path in candidates:
        # 小于 1 MB 的当成没下完/占位文件，忽略
        if path and os.path.isfile(path) and os.path.getsize(path) > 1024 * 1024:
            return path
    return None
