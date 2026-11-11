#!/usr/bin/env python3
"""路径解析：产物目录 + 模型文件，源码树与 devel/install 布局都能找到。

产物目录：`--out` > `$IMPEDANCE_OUT_DIR` > 仓库 `docs/img/chap8` > `~/.cache/<pkg>/output` > `./impedance_output`
模型文件：`--model`/`$IMPEDANCE_MODEL_PATH` > rospkg 包路径 > `<脚本目录>/models` > `<包目录>/models`
          > `<ws>/share/<pkg>/models` > `~/.cache/<pkg>/models`

注意脚本目录在两种布局下含义不同：
- 源码树：<repo>/src/chap8/<pkg>/scripts/，包目录是它的上一级；
- catkin_install_python 安装后：<ws>/lib/<pkg>/，脚本目录**就是**包目录，
  上一级是 <ws>/lib。所以候选里两种都试，share 目录则按上两级去找 <ws>/share。
"""
import os

PKG = "mujoco_2link_impedance_control"
ENV_VAR = "IMPEDANCE_OUT_DIR"
MODEL_ENV = "IMPEDANCE_MODEL_PATH"
MODEL_NAME = "two_link_soft_arm.xml"


def _probe(path):
    """建目录并写一个探针文件，确认真的可写后返回该目录。"""
    os.makedirs(path, exist_ok=True)
    probe = os.path.join(path, ".write_probe")
    with open(probe, "w") as f:
        f.write("")
    os.remove(probe)
    return path


def resolve_out_dir(pkg_root, cli_out=None):
    """顺序：--out > $IMPEDANCE_OUT_DIR > <repo>/docs/img/chap8 > ~/.cache/<pkg>/output > ./impedance_output。

    `pkg_root` 是包目录；它在源码树里的位置是 <repo>/src/chap8/<pkg>，向上三级即仓库根。
    装到 devel/install 后这个位置是 <ws>/lib/<pkg>，向上三级既不是仓库也可能不可写，
    探测失败就继续往后退。
    """
    cands = []
    first = cli_out or os.environ.get(ENV_VAR)
    if first:
        cands.append(first)
    # 显式指定的目录也不可写时，继续按默认顺序退（而不是直接报错）
    repo = os.path.dirname(os.path.dirname(os.path.dirname(pkg_root)))
    # 只有确认这里是仓库才往仓库里写：仓库里 docs/img/chap8 或 docs/index.md 至少有一个。
    # 这样 devel/install 布局（<ws>/lib/<pkg>，向上三级是 <ws>，通常没有 docs/）不会凭空
    # 造出 <ws>/docs/img/chap8（旧版就是这样把产物写到 <ws>/lib/docs/img 的）；
    # 稀疏检出只拉了 docs/img/chap8 而没有 index.md 时也仍能认出来。
    docs = os.path.join(repo, "docs")
    if (os.path.isdir(os.path.join(docs, "img", "chap8"))
            or os.path.isfile(os.path.join(docs, "index.md"))):
        cands.append(os.path.join(docs, "img", "chap8"))
    cands.append(os.path.expanduser(os.path.join("~", ".cache", PKG, "output")))
    cands.append(os.path.join(os.getcwd(), "impedance_output"))
    for c in cands:
        try:
            return _probe(c)
        except OSError:
            continue
    raise SystemExit("找不到可写输出目录，请用 --out 指定")


def resolve_model_path(script_dir, cli_model=None):
    """按源码树 / devel / install 三种布局依次找模型，找不到时列出所有候选再退出。

    旧版三个离线脚本用 `dirname(dirname(__file__))/models/...` 拼路径，装到
    devel/lib/<pkg>/ 之后就指向 <ws>/lib/models/two_link_soft_arm.xml（不存在），
    `mujoco.MjModel.from_xml_path` 直接抛 mjParseXML resource not found。
    """
    cands = []
    first = cli_model or os.environ.get(MODEL_ENV)
    if first:
        cands.append(os.path.abspath(first))
    # rospkg 在源码空间（package.xml 就在包目录）与 devel/install 空间都能给出包根
    try:
        import rospkg
        cands.append(os.path.join(rospkg.RosPack().get_path(PKG), "models", MODEL_NAME))
    except Exception:
        pass
    # 安装布局：script_dir 就是 <ws>/lib/<pkg>；源码树：上一级才是包目录
    cands.append(os.path.join(script_dir, "models", MODEL_NAME))
    cands.append(os.path.join(os.path.dirname(script_dir), "models", MODEL_NAME))
    # <ws>/lib/<pkg> 与 <ws>/lib/<pkg>/scripts 两种起点都往上找 <ws>/share/<pkg>/models
    for up in (os.path.dirname(script_dir),
               os.path.dirname(os.path.dirname(script_dir))):
        cands.append(os.path.join(up, "share", PKG, "models", MODEL_NAME))
    cands.append(os.path.expanduser(os.path.join("~", ".cache", PKG, "models", MODEL_NAME)))

    seen = []
    for c in cands:
        if c not in seen:
            seen.append(c)
    for c in seen:
        if os.path.isfile(c):
            return c
    raise SystemExit("找不到模型 %s，候选：\n  %s" % (MODEL_NAME, "\n  ".join(seen)))
