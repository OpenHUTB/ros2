#!/usr/bin/env python3
"""Build a clean source archive (used for submission and VM transfer)."""

from __future__ import annotations

import argparse
import os
import zipfile

SKIP_DIRS = {"__pycache__", ".git", "runs", "weights_test", "results", ".venv",
             "dist", "site", "report", ".github", ".dsh-office"}
SKIP_FILES = {"train_log.txt", "train_e2e_log.txt"}
SKIP_SUFFIX = (".pyc",)
EXCLUDE_DATA = True


def build(src: str, dst: str, with_data: bool = False) -> str:
    src = os.path.abspath(src)
    dst = os.path.abspath(dst)
    dst_dir = os.path.dirname(dst)
    count = 0
    with zipfile.ZipFile(dst, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as zf:
        for root, dirs, files in os.walk(src):
            dirs[:] = [d for d in dirs if d not in SKIP_DIRS
                       and os.path.abspath(os.path.join(root, d)) != dst_dir]
            for name in files:
                path = os.path.join(root, name)
                if os.path.abspath(path) == dst:
                    continue
                rel = os.path.relpath(path, src)
                if name in SKIP_FILES or name.endswith(SKIP_SUFFIX):
                    continue
                if name.startswith("debug_") and name.endswith(".png"):
                    continue
                if not with_data and rel.startswith("data" + os.sep) and name.endswith(".npz"):
                    continue
                zf.write(path, os.path.join(os.path.basename(src), rel))
                count += 1
    print(f"{dst}: {count} files, {os.path.getsize(dst) / 1024 / 1024:.2f} MB")
    return dst


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=".")
    ap.add_argument("--dst", default="dist/neural_ca_drone.zip")
    ap.add_argument("--with-data", action="store_true")
    a = ap.parse_args()
    os.makedirs(os.path.dirname(os.path.abspath(a.dst)), exist_ok=True)
    build(a.src, a.dst, a.with_data)
