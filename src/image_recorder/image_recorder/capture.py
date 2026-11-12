"""录制文件的命名与目录工具（纯函数，便于测试）。

原始实现把图片固定存成当前目录下的 ``capture.jpg``，多次保存会互相覆盖，
输出位置也不可配置。这里把命名/建目录抽出来：

* 文件名带时间戳（毫秒）与可选序号，避免覆盖；
* 输出目录可配置，不存在时创建；
* 扩展名白名单，防止传入奇怪的后缀。
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

ALLOWED_EXTENSIONS = {"jpg", "jpeg", "png"}


def normalize_topic(topic: str) -> str:
    """校验话题名：非空、去掉首尾空白、必须以 ``/`` 开头。"""
    if not isinstance(topic, str):
        raise TypeError("话题名必须是字符串")
    text = topic.strip()
    if not text:
        raise ValueError("话题名不能为空")
    if not text.startswith("/"):
        text = "/" + text
    return text


def ensure_output_dir(output_dir) -> Path:
    """创建（必要时递归创建）输出目录并返回 ``Path``。"""
    path = Path(output_dir).expanduser()
    if path.exists() and not path.is_dir():
        raise ValueError(f"输出路径已存在且不是目录：{path}")
    path.mkdir(parents=True, exist_ok=True)
    return path


def make_capture_path(output_dir, stamp: datetime | None = None,
                      extension: str = "jpg", index: int | None = None) -> Path:
    """生成 ``capture_YYYYmmdd_HHMMSS_fff[_NNN].jpg`` 形式的路径。"""
    ext = extension.lower().lstrip(".")
    if ext not in ALLOWED_EXTENSIONS:
        raise ValueError(f"不支持的图片格式：{extension!r}（允许 {sorted(ALLOWED_EXTENSIONS)}）")
    when = stamp if stamp is not None else datetime.now()
    name = f"capture_{when.strftime('%Y%m%d_%H%M%S_%f')[:-3]}"
    if index is not None:
        if index < 0:
            raise ValueError("index 不能为负")
        name += f"_{index:03d}"
    return ensure_output_dir(output_dir) / f"{name}.{ext}"
