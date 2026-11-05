# -*- coding: utf-8 -*-
"""MuJoCo 机器人仿真公共库：GL 后端选择、相机注入、仿真循环、视频输出。

用法：
    每个机器人脚本先调用 setup_gl()（必须在 import mujoco 之前），
    再用 load_with_camera() 加载带自定义相机的模型，
    用 run_sim() 跑物理仿真并把画面输出为 mp4。
"""
import os
import platform
import re
from pathlib import Path

def setup_gl(force=None):
    """在 import mujoco 之前调用，选择渲染后端。

    - 已有 MUJOCO_GL 环境变量：尊重用户设置
    - Windows：wgl（MuJoCo 在 Windows 上支持 wgl）
    - macOS：cgl（系统原生 OpenGL 上下文，离屏可用）
    - Linux：默认 osmesa（软件渲染，虚拟机/SSH 无显示也能跑）；
      有独显可手动设 MUJOCO_GL=egl 提速
    """
    cur = os.environ.get("MUJOCO_GL", "").strip().lower()
    if force:
        os.environ["MUJOCO_GL"] = force
        return force
    if cur:
        return cur
    system = platform.system()
    if system == "Windows":
        backend = "wgl"
    elif system == "Darwin":  # macOS
        backend = "cgl"
    else:  # Linux 及其他 Unix
        backend = "osmesa"
    os.environ["MUJOCO_GL"] = backend
    return backend


def _expand_includes(text, base_dir, depth=0):
    """递归内联 MJCF 的 <include>。

    去掉子文件的 <mujoco> 根标签，并把子文件的 meshdir / texturedir 改成
    相对子文件目录的绝对路径。这样拼出来的完整 XML 放到任意目录都能编译，
    不依赖原始目录层级（menagerie 的 scene.xml 只 include 一个机器人定义）。
    """
    if depth > 5:
        raise RuntimeError("MJCF include 嵌套过深")

    def repl(m):
        inc = (base_dir / m.group(1)).resolve()
        sub = inc.read_text(encoding="utf-8", errors="replace")
        sub = re.sub(r"<mujoco[^>]*>", "", sub, count=1).replace("</mujoco>", "")
        sub = re.sub(
            r'(meshdir|texturedir)="([^"]+)"',
            lambda mm: '%s="%s"' % (
                mm.group(1), (inc.parent / mm.group(2)).resolve().as_posix()),
            sub,
        )
        return _expand_includes(sub, inc.parent, depth + 1)

    return re.sub(r'<include\s+file="([^"]+)"\s*/>', repl, text)


def _with_offscreen(text, width, height):
    """把离屏渲染分辨率写进 <visual><global offwidth offheight/>。

    MuJoCo 的 Renderer 需要模型里声明离屏帧缓冲尺寸；menagerie 的 scene.xml
    没有这一段，所以这里补上（已有 <global> 时合并属性，不覆盖原有设置）。
    """
    gw = 'offwidth="%d" offheight="%d"' % (width, height)

    def merge(m):
        inner = m.group(1)
        return m.group(0) if "offwidth" in inner else "<global %s %s/>" % (inner.strip(), gw)

    text, n = re.subn(r"<global\s+([^/>]*)/>", merge, text, count=1)
    if n:
        return text
    if "<visual>" in text:
        return text.replace("<visual>", "<visual>\n    <global %s/>" % gw, 1)
    m = re.search(r"<mujoco[^>]*>", text)
    insert = "\n  <visual>\n    <global %s/>\n  </visual>" % gw
    return text[:m.end()] + insert + text[m.end():]


def _camera_tag(cam):
    """把相机参数字典渲染成一行 MJCF <camera> 标签。"""
    def vec(values):
        return " ".join("%g" % float(v) for v in values)

    attrs = ['name="%s"' % cam.get("name", "view0"), 'pos="%s"' % vec(cam["pos"])]
    if "xyaxes" in cam:
        attrs.append('xyaxes="%s"' % vec(cam["xyaxes"]))
    elif "quat" in cam:
        attrs.append('quat="%s"' % vec(cam["quat"]))
    if cam.get("fovy") is not None:
        attrs.append('fovy="%g"' % float(cam["fovy"]))
    if cam.get("mode") in ("track", "targetbody"):
        attrs.append('mode="%s"' % cam["mode"])
        attrs.append('target="%s"' % cam["target"])
    return "<camera %s/>" % " ".join(attrs)


def _model_dir(robot):
    """定位 menagerie 模型目录。

    优先用 mujoco_menagerie 包（它会按需下载到本地缓存）。该包的下载逻辑用
    tarfile.extractall(..., filter="data")，而 filter 参数要 Python 3.11.4 以上
    才有；Python 3.10（Ubuntu 22.04 默认）上会 TypeError。遇到这种情况不直接崩，
    改为扫描本地已有缓存目录复用。
    """
    import mujoco_menagerie as mj

    reason = None
    try:
        return Path(mj.get(robot).path())
    except Exception as exc:  # 网络、解包、权限等任何失败都退到本地缓存扫描
        reason = exc

    roots = []
    if os.environ.get("MUJOCO_MENAGERIE_CACHE"):
        roots.append(Path(os.environ["MUJOCO_MENAGERIE_CACHE"]))
    if os.environ.get("LOCALAPPDATA"):
        roots.append(Path(os.environ["LOCALAPPDATA"]) / "mujoco_menagerie")
    if os.environ.get("XDG_CACHE_HOME"):
        roots.append(Path(os.environ["XDG_CACHE_HOME"]) / "mujoco_menagerie")
    roots += [Path.home() / ".cache" / "mujoco_menagerie",
              Path.home() / ".local" / "share" / "mujoco_menagerie"]

    for root in roots:
        for models in (root / "cache" / "models", root / "models", root):
            if not models.is_dir():
                continue
            for d in sorted(models.iterdir()):
                if d.is_dir() and (d.name == robot or d.name.startswith(robot + "-")):
                    print("复用本地缓存模型:", d)
                    return d

    raise RuntimeError(
        "未找到 %s 的模型：mujoco-menagerie 取模型失败（%r），本地缓存中也没有可复用目录。"
        "Python %s 下 tarfile.extractall 的 filter 参数需 >= 3.11.4；可升级 Python，"
        "或把模型手动放到 %s" % (robot, reason, platform.python_version(), roots[0])
    ) from reason


def load_with_camera(robot, entry, cam, width=960, height=544):
    """加载 mujoco_menagerie 模型，并把自定义相机注入 MJCF 后编译。

    相机注入方式：读出 menagerie 的入口 XML，递归内联 <include>、把 meshdir /
    texturedir 改成绝对路径、在 <worldbody> 开头插入 <camera> 标签，写到系统
    临时目录（tempfile）后编译，加载完立即删除临时文件。

    menagerie 安装目录全程只读，不会残留任何文件（旧实现会往安装目录写
    scene_view.xml，在只读安装或共享环境下会 PermissionError 并留垃圾）。
    这里不用 MjSpec：它在 3.1 才引入，且 3.2.x 的相机属性（mode/targetbody）
    尚不完整，直接用 MjModel.from_xml_path 兼容性最好。

    cam 为字典：
        name   相机名（默认 "view0"）
        pos    相机位置 [x, y, z]
        xyaxes 或 quat  相机姿态（二选一）
        fovy   视场角（可选）
        mode   "track" 时为跟踪模式，配合 target（body 名）
    """
    import tempfile

    import mujoco

    src = (_model_dir(robot) / f"{entry}.xml").resolve()
    text = _expand_includes(src.read_text(encoding="utf-8", errors="replace"), src.parent)
    text = _with_offscreen(text, width, height)

    m = re.search(r"<worldbody[^>]*>", text)
    if not m:
        raise RuntimeError("%s 中没有 <worldbody>，无法注入相机" % src)
    text = text[:m.end()] + "\n    " + _camera_tag(cam) + text[m.end():]

    fd, tmp = tempfile.mkstemp(prefix="mjscene_", suffix=".xml")
    os.close(fd)
    try:
        with open(tmp, "w", encoding="utf-8") as f:
            f.write(text)
        return mujoco.MjModel.from_xml_path(tmp)
    finally:
        try:
            os.remove(tmp)
        except OSError:
            pass


def run_sim(model, controller, duration=6.0, fps=30, video_path=None,
            preview_dir=None, height=544, width=960):
    """跑仿真并渲染。

    video_path 给定时边渲染边写 MP4，长视频不会把上千帧全留在内存里；
    preview_dir 给定时额外保存首/中/末三张 PNG 用于检查取景（这时才需要 Pillow）。
    返回 preview 保存的帧；没有 preview 时返回空列表。
    """
    import mujoco

    data = mujoco.MjData(model)
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    controller(data, 0.0)

    dt = model.opt.timestep
    sub = max(1, round(1.0 / fps / dt))
    nframes = int(duration * fps)
    if nframes <= 0:
        raise ValueError("duration=%r 太短，一帧都生成不了" % duration)

    writer = None
    if video_path:
        import imageio.v2 as iio

        os.makedirs(os.path.dirname(os.path.abspath(video_path)), exist_ok=True)
        writer = iio.get_writer(video_path, fps=fps, codec="libx264", quality=7,
                                macro_block_size=1)
    if preview_dir:
        os.makedirs(preview_dir, exist_ok=True)

    frames = []
    t = 0.0
    renderer = None
    try:
        renderer = mujoco.Renderer(model, height, width)
        for fi in range(nframes):
            for _ in range(sub):
                controller(data, t)
                mujoco.mj_step(model, data)
                t += dt
            renderer.update_scene(data, camera="view0")
            frame = renderer.render()
            if writer is not None:
                writer.append_data(frame)
            if preview_dir and fi in {0, nframes // 2, nframes - 1}:
                from PIL import Image  # 只有预览模式需要 Pillow

                path = os.path.join(preview_dir, "preview_%03d.png" % fi)
                Image.fromarray(frame).save(path)
                frames.append(frame.copy())
                print("预览帧:", path)
    finally:
        if renderer is not None:
            renderer.close()
        if writer is not None:
            writer.close()

    if video_path:
        print("视频已保存: %s  (%d 帧, %d fps)" % (video_path, nframes, fps))
    return frames


def _positive_seconds(text):
    """--duration 只接受正数秒，避免 0/负数生成空视频。"""
    import argparse

    try:
        value = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(
            "--duration 需要数字（秒）：%s" % text
        ) from None
    if value <= 0:
        raise argparse.ArgumentTypeError("--duration 必须为正数（秒）：%s" % text)
    return value


def parse_args():
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--preview", action="store_true",
                    help="只跑 1.5 秒，输出 3 张 PNG 检查取景")
    ap.add_argument("--out", default=None, help="视频输出路径")
    ap.add_argument("--duration", type=_positive_seconds, default=None,
                    help="完整视频的仿真时长（秒），不填则用模块默认值")
    # roslaunch 会追加 __name:=、__log:= 等私有参数，须用 parse_known_args 忽略
    args, _ = ap.parse_known_args()
    return args
