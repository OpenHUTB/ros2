#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""weather_adaptive_perception_replay 的离线测试（不需要 CARLA / ROS / GPU / 联网）。

覆盖
----
1. 模型清单：每个权重都带期望字节数与 sha256，校验函数能把「代理返回 HTML」「内容不符」判死
2. ensure_model：断网、下载损坏时给出可执行提示（含 --model-dir），且不留 .part 残文件
3. 几何合理性判据：面积过小、跨屏大框、极端长宽比
4. 检测后处理：IoU、NMS、按类别 + IoU 匹配
5. 天气判别：用文档里的实测特征复现 ClearNoon / HardRainNoon 两级
6. 端到端：模型目录已校验通过时跑完整流程，断言 summary.json 与文档口径一致

1~5 组只用标准库与 numpy，cv2 / matplotlib 缺失时照常执行；第 6 组缺依赖或缺模型自动 skip。
在仓库根目录执行：python3 -m pytest tests/test_weather_adaptive_perception_replay_local.py -v
"""
from __future__ import annotations

import hashlib
import importlib.util
import json
import os
import shutil
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(_HERE)
MODULE_PATH = os.path.join(ROOT, "src", "ground", "weather_adaptive_perception_replay",
                           "main.py")


def _load_module():
    """按路径加载模块：仓库里 main.py 同名文件很多，不能靠 sys.path 直接 import。"""
    spec = importlib.util.spec_from_file_location(
        "weather_adaptive_perception_replay_main", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)      # 模块把 cv2 / matplotlib 做成可选导入
    return module


MOD = _load_module()


def test_module_is_importable_without_optional_deps():
    assert os.path.isfile(MODULE_PATH)
    assert MOD.MODEL_FILES, "模型清单不能为空"


@pytest.mark.parametrize("name", sorted(MOD.MODEL_FILES))
def test_model_manifest_declares_size_and_sha256(name):
    meta = MOD.MODEL_FILES[name]
    urls = meta["urls"]
    assert urls, "每个权重至少给一个下载源"
    for url in urls:
        assert url.startswith("https://"), "权重必须走 HTTPS"
        assert url.rstrip("/").endswith(name), "下载地址的末段应与文件名一致"
    assert meta["size"] > 1024
    assert len(meta["sha256"]) == 64
    int(meta["sha256"], 16)          # 必须是合法十六进制
    # 国内网络直连 raw.githubusercontent 常被拖死，大文件必须有镜像兜底
    assert len(urls) >= 2, "23 MB 的权重只给一个源，国内网络基本下不下来"


def test_requirements_pin_opencv_below_5():
    """opencv-python 5.x 移除了 Caffe 导入器，必须钉在 4.x（CI 曾因此红过一次）。"""
    req = os.path.join(ROOT, "src", "ground", "weather_adaptive_perception_replay",
                       "requirements.txt")
    with open(req, encoding="utf-8") as fh:
        text = fh.read().replace(" ", "")
    assert "opencv-python>=4.5,<5" in text


def test_file_matches_rejects_bad_size_and_bad_digest(tmp_path):
    data = b"hello weather"
    digest = hashlib.sha256(data).hexdigest()
    path = tmp_path / "deploy.prototxt"
    path.write_bytes(data)

    assert MOD.file_matches(str(path), len(data), digest) is True
    # 字节数不符：代理把文件换成了 HTML 错误页
    assert MOD.file_matches(str(path), len(data) + 1, digest) is False
    # 字节数相符、内容不符：下载被截断后补齐或换了包
    assert MOD.file_matches(str(path), len(data), "0" * 64) is False
    assert MOD.file_matches(str(tmp_path / "missing.prototxt"), 10, "0" * 64) is False


def test_ensure_model_fails_soft_when_offline(tmp_path, monkeypatch):
    def boom(*args, **kwargs):
        raise OSError("network unreachable")

    monkeypatch.setattr(MOD.urllib.request, "urlopen", boom)
    with pytest.raises(SystemExit) as exc:
        MOD.ensure_model(str(tmp_path))
    message = str(exc.value)
    assert "模型下载失败" in message, "断网应给一句人话，而不是裸 traceback"
    assert "--model-dir" in message, "应提示可指向本地模型目录"
    assert not list(tmp_path.glob("*.part")), "失败后不应残留半截文件"


def test_ensure_model_rejects_corrupted_payload(tmp_path, monkeypatch):
    """字节数对得上、内容对不上（典型的「HTML 页面被补齐到同样大小」）：必须判死。"""
    name = "deploy.prototxt"
    meta = MOD.MODEL_FILES[name]
    payload = (b"<html><body>502 Bad Gateway</body></html>").ljust(meta["size"], b"x")

    class _FakeResponse:
        headers = {"Content-Length": str(meta["size"])}

        def __init__(self):
            self._sent = False

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

        def read(self, n=-1):
            if self._sent:
                return b""
            self._sent = True
            return payload

    monkeypatch.setattr(MOD.urllib.request, "urlopen", lambda *a, **k: _FakeResponse())
    with pytest.raises(SystemExit) as exc:
        MOD.ensure_model(str(tmp_path))
    message = str(exc.value)
    assert "模型下载失败" in message
    assert "校验失败" in message, "内容不符要说明是校验没过，而不是只说下载失败"
    assert "--model-dir" in message
    assert not list(tmp_path.glob("*.part"))
    assert not list(tmp_path.glob("*.prototxt")), "坏文件不能被留下冒充可用权重"


def test_download_model_rejects_truncated_stream(tmp_path, monkeypatch):
    """源一直吐不满字节数（被代理截断）：不能留下半截 .part 文件。"""
    class _Resp:
        headers = {"Content-Length": "20"}

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

        def read(self, n=-1):
            return b"01234"          # 只有 5 字节

    monkeypatch.setattr(MOD.urllib.request, "urlopen", lambda *a, **k: _Resp())
    meta = {"urls": ["https://mirror.example.bad/deploy.prototxt"],
            "size": 20, "sha256": hashlib.sha256(b"01234567890123456789").hexdigest()}
    with pytest.raises(SystemExit) as exc:
        MOD.download_model("deploy.prototxt", meta, str(tmp_path))
    assert "模型下载失败" in str(exc.value)
    assert not list(tmp_path.glob("*.part"))


def test_download_model_falls_back_to_next_source(tmp_path, monkeypatch):
    """第一个源读超时、第二个源正常：应换源成功，而不是直接失败。"""
    name = "deploy.prototxt"
    meta = {"urls": ["https://dead.example.invalid/deploy.prototxt",
                     "https://mirror.example.ok/deploy.prototxt"],
            "size": 20, "sha256": hashlib.sha256(b"01234567890123456789").hexdigest()}
    calls = {"dead": 0, "ok": 0}

    class _Resp:
        headers = {"Content-Length": "20"}

        def __init__(self, payload):
            self._payload = payload
            self._sent = False

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

        def read(self, n=-1):
            if self._sent:
                return b""
            self._sent = True
            return self._payload

    def fake_urlopen(url, timeout=None):
        if "dead" in url:
            calls["dead"] += 1
            raise TimeoutError("read timed out")
        calls["ok"] += 1
        return _Resp(b"01234567890123456789")

    monkeypatch.setattr(MOD.urllib.request, "urlopen", fake_urlopen)
    path = MOD.download_model(name, meta, str(tmp_path))
    assert os.path.getsize(path) == 20
    assert calls == {"dead": 1, "ok": 1}
    assert not list(tmp_path.glob("*.part"))


def test_box_filter_rejects_oversized_and_extreme_ratio():
    w = h = 900
    assert MOD.box_is_plausible(40, 30, w, h) is True
    assert MOD.box_is_plausible(10, 10, w, h) is False, "面积过小"
    assert MOD.box_is_plausible(200, 700, w, h) is False, "框高超过画面 55%"
    assert MOD.box_is_plausible(400, 100, w, h) is False, "框宽超过画面 35%"
    assert MOD.box_is_plausible(20, 300, w, h) is False, "长宽比过窄"
    assert MOD.box_is_plausible(300, 20, w, h) is False, "长宽比过宽"


def test_iou_and_nms():
    assert MOD.iou((0, 0, 10, 10), (0, 0, 10, 10)) == pytest.approx(1.0)
    assert MOD.iou((0, 0, 10, 10), (100, 100, 110, 110)) == pytest.approx(0.0)

    strong = {"cls": 7, "name": "car", "score": 0.9, "box": (0, 0, 100, 100)}
    weak = {"cls": 7, "name": "car", "score": 0.5, "box": (5, 5, 105, 105)}
    keep = MOD.nms([weak, strong])
    assert len(keep) == 1
    assert keep[0]["score"] == 0.9, "同类重叠框应保留置信度高的那个"

    other_class = {"cls": 15, "name": "person", "score": 0.4, "box": (0, 0, 100, 100)}
    assert len(MOD.nms([strong, other_class])) == 2, "不同类别不应互相抑制"


def test_match_requires_same_class_and_iou():
    gt = [{"cls": 7, "name": "car", "box": (0, 0, 100, 100)},
          {"cls": 15, "name": "person", "box": (200, 200, 260, 320)}]
    dets = [{"cls": 7, "name": "car", "box": (0, 0, 100, 100)},
            {"cls": 6, "name": "bus", "box": (200, 200, 260, 320)}]   # 位置对、类别错
    hit_gt, hit_det = MOD.match(dets, gt)
    assert hit_gt == {0}
    assert hit_det == {0}, "类别不一致的框不能算命中"


def test_classify_weather_reproduces_documented_labels():
    # 取自文档「实验结果」里的实测特征
    clear = {"brightness": 73.08, "contrast": 43.47, "saturation": 74.31,
             "sharpness": 429.25, "dark_channel_mean": 44.06}
    rain = {"brightness": 136.07, "contrast": 24.20, "saturation": 19.79,
            "sharpness": 87.09, "dark_channel_mean": 121.02}
    assert MOD.classify_weather(clear) == ("ClearNoon", pytest.approx(0.80))
    assert MOD.classify_weather(rain) == ("HardRainNoon", pytest.approx(0.80))


def test_lidar_branch_is_deterministic_and_weather_agnostic():
    gt = [{"cls": 7, "name": "car", "score": 0.3, "box": (0, 0, 100, 120)}]
    assert MOD.lidar_branch(gt) == MOD.lidar_branch(gt), "固定种子，结果必须可复现"
    candidates = MOD.lidar_branch(gt)
    assert all(d["src"] == "lidar" for d in candidates)


def test_imread_imwrite_work_in_non_ascii_dir(tmp_path):
    """Windows 上用户名常含中文，cv2.imread/imwrite 会直接读不到文件，必须走 imdecode/imencode。"""
    cv2 = pytest.importorskip("cv2", reason="需要 opencv-python")
    np = pytest.importorskip("numpy")
    folder = tmp_path / "中文 目录"
    folder.mkdir()
    img = np.zeros((40, 60, 3), dtype=np.uint8)
    img[:, :, 1] = 200
    path = folder / "帧.png"
    assert MOD.imwrite_any(str(path), img) is True
    assert path.stat().st_size > 0
    back = MOD.imread_any(str(path))
    assert back is not None and back.shape == img.shape
    assert MOD.imread_any(str(folder / "不存在.png")) is None


def _verified_model_dir():
    model_dir = os.environ.get("WEATHER_AP_REPLAY_MODEL_DIR",
                               os.path.join(MOD.BASE_DIR, "model"))
    for name, meta in MOD.MODEL_FILES.items():
        if not MOD.file_matches(os.path.join(model_dir, name), meta["size"], meta["sha256"]):
            return None
    return model_dir


def test_load_caffe_net_works_from_non_ascii_dir(tmp_path):
    """模型放在含中文的目录里也能加载：cv2 的 Caffe 层读不了非 ASCII 路径，需要切目录兜底。"""
    cv2 = pytest.importorskip("cv2", reason="需要 opencv-python")
    if not hasattr(cv2.dnn, "readNetFromCaffe"):
        pytest.skip("当前 opencv-python 没有 Caffe 导入器（需 4.x）")
    model_dir = _verified_model_dir()
    if model_dir is None:
        pytest.skip("模型未就绪：设 WEATHER_AP_REPLAY_MODEL_DIR 指向已校验的模型目录")
    folder = tmp_path / "模型 目录"
    folder.mkdir()
    for name in MOD.MODEL_FILES:
        shutil.copyfile(os.path.join(model_dir, name), str(folder / name))
    net = MOD.load_caffe_net(str(folder / "deploy.prototxt"),
                            str(folder / "mobilenet_iter_73000.caffemodel"))
    assert net is not None


def test_end_to_end_matches_documented_numbers(tmp_path, monkeypatch):
    pytest.importorskip("cv2", reason="需要 opencv-python")
    pytest.importorskip("matplotlib", reason="需要 matplotlib")
    model_dir = _verified_model_dir()
    if model_dir is None:
        pytest.skip("模型未就绪：设 WEATHER_AP_REPLAY_MODEL_DIR 指向已校验的模型目录")

    monkeypatch.setattr(MOD, "RESULT_DIR", str(tmp_path))
    monkeypatch.setattr(sys, "argv", ["main.py", "--model-dir", model_dir])
    MOD.main()

    out_png = tmp_path / "adaptive_perception.png"
    assert out_png.stat().st_size > 100000, "应产出非空的对比图"

    summary = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))
    assert summary["gt_count"] == 2
    assert summary["modes"]["camera_clear"]["recall"] == pytest.approx(1.00)
    assert summary["modes"]["camera_rain"]["recall"] == pytest.approx(0.50)
    assert summary["modes"]["fusion_clear"]["recall"] == pytest.approx(1.00)
    assert summary["modes"]["fusion_rain"]["recall"] == pytest.approx(1.00)
    assert summary["weather"]["clear"]["label"] == "ClearNoon"
    assert summary["weather"]["rain"]["label"] == "HardRainNoon"
