#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Download the Franka Emika Panda model assets used by this module.

The robot description comes from MuJoCo Menagerie
(https://github.com/google-deepmind/mujoco_menagerie, Apache-2.0).  Only the
mesh files are downloaded here; ``panda_pick.xml`` (this module) and
``scene_hand_eye.xml`` (this module) are shipped in the repository.

Usage:
    python scripts/setup_model.py
"""
import os
import re
import sys
import time
import urllib.request

PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
MODULE_ROOT = os.path.dirname(PACKAGE_DIR)


def _share_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory("hand_eye_calibration")
    except Exception:
        return MODULE_ROOT


MODEL_DIR = os.path.join(_share_dir(), "models", "franka_emika_panda")
# Pin upstream MuJoCo Menagerie to a known commit for reproducible assets.
BASE = ("https://raw.githubusercontent.com/google-deepmind/"
        "mujoco_menagerie/370c43965323c0944b433b202aac5058b5f907a9/franka_emika_panda")


def download(url, dest):
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    if os.path.exists(dest) and os.path.getsize(dest) > 0:
        return True
    for i in range(4):
        try:
            tmp = dest + ".part"
            urllib.request.urlretrieve(url, tmp)
            os.replace(tmp, dest)
            return True
        except Exception as e:  # noqa: BLE001
            print("  retry %d for %s: %s" % (i + 1, os.path.basename(dest), e))
            time.sleep(1.5)
    return False


def main():
    panda_pick = os.path.join(MODEL_DIR, "panda_pick.xml")
    if not os.path.exists(panda_pick):
        sys.exit("missing " + panda_pick)
    text = open(panda_pick, encoding="utf-8").read()
    assets = sorted(set(re.findall(r'file="([^"]+)"', text)))
    print("assets referenced by panda_pick.xml:", len(assets))

    ok, fail = 0, []
    for name in assets:
        dest = os.path.join(MODEL_DIR, "assets", name)
        if download("%s/assets/%s" % (BASE, name), dest):
            ok += 1
        else:
            fail.append(name)
    # upstream robot XML + license, for reference/attribution
    download("%s/panda.xml" % BASE, os.path.join(MODEL_DIR, "panda.xml"))
    download("%s/LICENSE" % BASE, os.path.join(MODEL_DIR, "LICENSE"))

    print("downloaded %d/%d assets" % (ok, len(assets)))
    if fail:
        print("failed:", fail)
        sys.exit(1)
    print("done. try:  python main.py --record /tmp/he_frames")


if __name__ == "__main__":
    main()
