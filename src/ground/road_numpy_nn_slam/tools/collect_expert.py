#!/usr/bin/env python3
"""Collect expert demonstration episodes from the road-vehicle simulator.

Writes ``results/dataset_<scene>.npz`` (pose, 72-beam lidar, clean lidar,
camera images, expert (throttle, steer) commands, reactive behaviour-cloning
labels, traffic-light features) plus a per-scene metadata JSON.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

from road_numpy_nn_slam import config as C
from road_numpy_nn_slam.datasets import collect, dataset_metadata, save_dataset


def parse_scenes(text: str):
    return [s.strip() for s in text.split(",") if s.strip()]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenes", default=",".join(C.SCENES))
    parser.add_argument("--seconds", type=float, default=60.0,
                        help="maximum simulated seconds per episode")
    parser.add_argument("--episodes", type=int, default=3,
                        help="episodes collected per scene")
    parser.add_argument("--seed", type=int, default=C.DEFAULT_SEED)
    parser.add_argument("--sample-hz", type=float, default=10.0)
    parser.add_argument("--noise", action="store_true", default=True,
                        help="sensor noise on (default)")
    parser.add_argument("--no-noise", dest="noise", action="store_false")
    parser.add_argument("--data-dir", default=C.RESULTS_DIR)
    parser.add_argument("--force", action="store_true",
                        help="re-collect even if a dataset already exists")
    parser.add_argument("--prefer-nav", action="store_true",
                        help="use road_numpy_nn_slam.control.MissionController as expert")
    parser.add_argument("--no-random-start", dest="random_start",
                        action="store_false",
                        help="always start each episode at the scene start pose")
    parser.set_defaults(random_start=True)
    args = parser.parse_args()

    scenes = parse_scenes(args.scenes)
    os.makedirs(args.data_dir, exist_ok=True)
    t_start = time.time()
    summary = {}
    total_samples = 0
    for scene_name in scenes:
        path = os.path.join(args.data_dir, "dataset_%s.npz" % scene_name)
        if os.path.exists(path) and not args.force:
            print("[skip] %s (dataset exists, use --force to re-collect)"
                  % path)
            continue
        t0 = time.time()
        data = collect(scene_name, seconds=args.seconds, seed=args.seed,
                       noise=args.noise, sample_hz=args.sample_hz,
                       episodes=args.episodes,
                       prefer_nav_expert=args.prefer_nav,
                       random_start=args.random_start)
        save_dataset(data, path)
        meta = dataset_metadata(data)
        meta["seconds_wall"] = round(time.time() - t0, 2)
        with open(os.path.join(args.data_dir,
                               "dataset_%s.json" % scene_name), "w",
                  encoding="utf-8") as fh:
            json.dump(meta, fh, indent=2, sort_keys=True)
        summary[scene_name] = meta
        total_samples += int(meta["samples"])
        print("[ok] %s: %d samples, success %.0f%%, collisions %d, red-light "
              "%d, pedestrians hit %d, off-road %.1f s, %.1f s wall"
              % (scene_name, meta["samples"],
                 100.0 * meta["episode_success_rate"], meta["collisions"],
                 meta["red_light_violations"], meta["pedestrians_hit"],
                 meta["off_road_time"], meta["seconds_wall"]))
    elapsed = time.time() - t_start
    print("total: %d scenes, %d samples, %.1f s wall"
          % (len(summary), total_samples, elapsed))
    with open(os.path.join(args.data_dir, "dataset_summary.json"), "w",
              encoding="utf-8") as fh:
        json.dump({"scenes": summary, "total_samples": total_samples,
                   "beams": C.LIDAR_BEAMS,
                   "seconds_wall": round(elapsed, 2)}, fh, indent=2,
                  sort_keys=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
