#!/usr/bin/env python3
"""Add reproducible training metadata to the model JSON files.

The weights themselves live in ``models/*.npz``; the companion ``*.json``
files already carried the training metrics but not enough information to
reproduce a particular weight file.  This script adds a ``meta`` block with:

    arch          network architecture (from the model file)
    lr, epochs    optimiser settings
    train_seconds wall time of the training run
    dataset_hash  SHA1 of the committed results/dataset_*.json files
    config_hash   SHA1 of humanoid_walk_nn/config.py
    git_sha       current git commit (or "unversioned")
    created_utc   when this meta block was written

Run from the repository root::

    python tools/add_model_meta.py
"""

from __future__ import annotations

import glob
import hashlib
import io
import json
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
MODELS = os.path.join(ROOT, "models")
RESULTS = os.path.join(ROOT, "results")
CONFIG = os.path.join(ROOT, "humanoid_walk_nn", "config.py")


def _sha1_files(paths):
    digest = hashlib.sha1()
    for path in sorted(paths):
        with open(path, "rb") as fh:
            digest.update(os.path.basename(path).encode("utf-8"))
            digest.update(fh.read())
    return digest.hexdigest()


def _git_sha() -> str:
    try:
        out = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT,
            stderr=subprocess.DEVNULL, timeout=10)
        return out.decode("ascii").strip()
    except Exception:
        return "unversioned"


def _dataset_hash() -> str:
    files = glob.glob(os.path.join(RESULTS, "dataset_*.json"))
    files += glob.glob(os.path.join(RESULTS, "dataset_*.npz"))
    if not files:
        return "no-dataset-files"
    return _sha1_files(files)


def _config_hash() -> str:
    if not os.path.exists(CONFIG):
        return "no-config"
    with open(CONFIG, "rb") as fh:
        return hashlib.sha1(fh.read()).hexdigest()


def main() -> int:
    git_sha = _git_sha()
    dataset_hash = _dataset_hash()
    config_hash = _config_hash()
    created = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    updated = 0
    for path in sorted(glob.glob(os.path.join(MODELS, "*.json"))):
        with open(path, "r", encoding="utf-8-sig") as fh:
            payload = json.load(fh)
        meta = {
            "arch": payload.get("architecture", payload.get("input_dim")),
            "lr": payload.get("lr"),
            "epochs": payload.get("epochs"),
            "batch": payload.get("batch"),
            "train_seconds": payload.get("seconds"),
            "train_samples": payload.get("train_samples", payload.get("samples")),
            "dataset_hash": dataset_hash,
            "config_hash": config_hash,
            "git_sha": git_sha,
            "created_utc": created,
        }
        payload["meta"] = meta
        with open(path, "w", encoding="utf-8", newline="\n") as fh:
            json.dump(payload, fh, indent=2, sort_keys=True)
            fh.write("\n")
        updated += 1
        print("meta ->", os.path.relpath(path, ROOT))
    print("updated %d model json files" % updated)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
