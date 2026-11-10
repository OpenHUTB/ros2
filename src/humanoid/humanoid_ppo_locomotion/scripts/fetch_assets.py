#!/usr/bin/env python3
"""下载 G1 机器人模型资源（STL 网格 + MJCF）。

仓库只保存文本（XML），网格文件（约 32 MB）通过本脚本从官方仓库获取。

用法：
    python scripts/fetch_assets.py                       # 从 GitHub 下载
    python scripts/fetch_assets.py --from-local D:/path/to/unitree_mujoco

来源：unitreerobotics/unitree_mujoco（BSD-3-Clause），路径 unitree_robots/g1。
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
import tempfile
import urllib.request
import zipfile

REPO_ZIP = "https://codeload.github.com/unitreerobotics/unitree_mujoco/zip/refs/heads/main"
SUBDIR = "unitree_robots/g1"
TARGET = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                      "models", "g1")


def from_local(src: str) -> int:
    src_g1 = os.path.join(src, SUBDIR) if os.path.isdir(os.path.join(src, SUBDIR)) else src
    if not os.path.isdir(src_g1):
        print(f"[fetch] 找不到 {src_g1}")
        return 1
    os.makedirs(TARGET, exist_ok=True)
    shutil.copytree(src_g1, TARGET, dirs_exist_ok=True)
    print(f"[fetch] 已从本地复制到 {TARGET}")
    return 0


def from_github() -> int:
    os.makedirs(TARGET, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        zip_path = os.path.join(tmp, "unitree_mujoco.zip")
        print(f"[fetch] 下载 {REPO_ZIP}（约 116 MB）")
        urllib.request.urlretrieve(REPO_ZIP, zip_path)
        with zipfile.ZipFile(zip_path) as zf:
            prefix = None
            for name in zf.namelist():
                if name.endswith(f"/{SUBDIR}/") or f"/{SUBDIR}/" in name:
                    prefix = name.split(SUBDIR)[0] + SUBDIR + "/"
                    break
            if prefix is None:
                print("[fetch] 压缩包结构异常，未找到 g1 目录")
                return 1
            members = [n for n in zf.namelist() if n.startswith(prefix)]
            for m in members:
                rel = m[len(prefix):]
                if not rel:
                    continue
                dest = os.path.join(TARGET, rel)
                if m.endswith("/"):
                    os.makedirs(dest, exist_ok=True)
                else:
                    os.makedirs(os.path.dirname(dest), exist_ok=True)
                    with zf.open(m) as src, open(dest, "wb") as dst:
                        shutil.copyfileobj(src, dst)
            print(f"[fetch] 解压 {len(members)} 个文件到 {TARGET}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--from-local", default="", help="本地已有的 unitree_mujoco 仓库路径")
    args = ap.parse_args()

    if os.path.exists(os.path.join(TARGET, "scene_29dof.xml")):
        print(f"[fetch] 模型已存在：{TARGET}（如需重新下载请先删除该目录）")
        return 0

    if args.from_local:
        rc = from_local(args.from_local)
    else:
        rc = from_github()

    if rc == 0:
        print("[fetch] 完成。可用以下命令验证：")
        print(f'  python -c "import mujoco; mujoco.MjModel.from_xml_path('
              f"r'{os.path.join(TARGET, 'scene_29dof.xml')}')\"")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
