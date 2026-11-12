#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Download the Franka Emika Panda mesh assets used by this module.

The robot description comes from MuJoCo Menagerie
(https://github.com/google-deepmind/mujoco_menagerie, Apache-2.0).  Only the
mesh files are downloaded here; ``panda_pick.xml`` (this module) and
``scene_pickplace.xml`` (this module) are shipped in the repository.

The upstream commit is pinned and every file is verified against
``models/franka_emika_panda/assets.sha256`` (same content as the pinned commit),
so the download is reproducible.  Set ``PI0_MENAGERIE_BASE`` to use a mirror
(e.g. ``https://ghfast.top/https://raw.githubusercontent.com/google-deepmind/
mujoco_menagerie/<sha>/franka_emika_panda``).

Usage:
    python scripts/setup_model.py
"""
import hashlib
import os
import sys
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.normpath(os.path.join(HERE, "..", "models", "franka_emika_panda"))
ARCHIVE = os.path.join(MODEL_DIR, "assets.sha256")

# Pinned upstream commit (MuJoCo Menagerie main, 2026-11-10).  Assets are
# verified against assets.sha256 before use.
MENAGERIE_COMMIT = "370c43965323c0944b433b202aac5058b5f907a9"
BASE = ("https://raw.githubusercontent.com/google-deepmind/"
        "mujoco_menagerie/%s/franka_emika_panda" % MENAGERIE_COMMIT)


def read_manifest():
    with open(ARCHIVE, encoding="utf-8") as f:
        entries = []
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            digest, name = line.split(None, 1)
            entries.append((digest.lower(), name.strip()))
    return entries


def sha256_of(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def download(url, dest):
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    req = urllib.request.Request(url, headers={"User-Agent": "pi0-mujoco-grasp-setup/1.0"})
    tmp = dest + ".part"
    with urllib.request.urlopen(req, timeout=60) as resp, open(tmp, "wb") as out:
        while True:
            chunk = resp.read(1 << 20)
            if not chunk:
                break
            out.write(chunk)
    os.replace(tmp, dest)


def main():
    if not os.path.isfile(ARCHIVE):
        sys.exit("missing asset manifest: " + ARCHIVE)
    entries = read_manifest()
    print("assets in manifest:", len(entries))

    base = os.environ.get("PI0_MENAGERIE_BASE", BASE).rstrip("/")
    seen = set()
    ok = fail = skip = 0
    for digest, name in entries:
        dest = os.path.join(MODEL_DIR, "assets", name)
        seen.add(name)
        if os.path.exists(dest) and sha256_of(dest) == digest:
            skip += 1
            continue
        good = False
        for attempt in range(4):
            try:
                download("%s/assets/%s" % (base, name), dest)
                if sha256_of(dest) == digest:
                    good = True
                    break
                print("  checksum mismatch, retry %d: %s" % (attempt + 1, name))
            except Exception as exc:  # noqa: BLE001
                print("  retry %d for %s: %s" % (attempt + 1, name, exc))
                time.sleep(1.5)
        if good:
            ok += 1
        else:
            fail += 1
            print("  FAILED:", name)

    print("downloaded %d, verified-existing %d, failed %d (total %d)"
          % (ok, skip, fail, len(entries)))
    if fail:
        sys.exit(1)
    print("done. try:  python main.py --record /tmp/pi0_frames")


if __name__ == "__main__":
    main()
