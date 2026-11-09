# -*- coding: utf-8 -*-
"""Entry point: DVS event-camera background-activity suppression + tracking.

Run the whole module with no arguments (offline replay of measured CARLA
data, zero third-party dependencies)::

    cd src/ground/dvs_event_motion_tracking
    python main.py

Other modes::

    python main.py --self-test          # deterministic synthetic regression
    python main.py --no-plot            # skip PNG generation (stdlib only)
    python main.py --collect            # re-collect data from a live CARLA server
"""

import argparse
import os
import sys

import dvs_core

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "dvs_events.json")
REPO_ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
IMG_DIR = os.path.join(REPO_ROOT, "docs", "img", "dvs_event_motion_tracking")
OUT_DIR = os.path.join(HERE, "data")


def repo_img_dir():
    return IMG_DIR


def print_report(meta, records, stats):
    print("=" * 66)
    print("DVS event-camera background activity filter / motion tracking")
    print("=" * 66)
    print("simulator            : %s" % meta.get("simulator", "offline dataset"))
    print("image / fov          : %s x %s px / %s deg" %
          (meta.get("width"), meta.get("height"), meta.get("fov_deg")))
    print("frames (static/move) : %d (%d / %d)" %
          (stats["frames"], stats["static_frames"], stats["moving_frames"]))
    print("static noise / frame : %s events" % stats["static_noise_per_frame"])
    print("background removed   : %.1f%%" % (stats["background_suppression"] * 100))
    print("detection rate       : %.1f%%" % (stats["detection_rate"] * 100))
    print("centroid MAE / max   : %s px / %s px" %
          (stats["centroid_mae_px"], stats["centroid_max_px"]))
    print("velocity est / gt    : %.2f / %.2f px/s (rel.err %s)" %
          (stats["velocity_est_px_s"], stats["velocity_gt_px_s"],
           stats["velocity_rel_error"]))
    print("false alarms         : %d / %d holdout static frames" %
          (stats["false_alarms"], stats["holdout_frames"]))
    print("raw events / RGB vol : %d / %d (%.3f%%)" %
          (stats["total_events_raw"], stats["rgb_pixel_volume"],
           stats["event_sparsity_ratio"] * 100))
    print("-" * 66)
    print("%5s %6s %8s %8s %8s %8s %6s" %
          ("frame", "t/s", "gt_u", "u_hat", "events", "kept", "det"))
    show = [r for r in records if r["detected"]][:6] + \
        ([records[-1]] if records else [])
    seen = set()
    for r in show:
        if r["i"] in seen:
            continue
        seen.add(r["i"])
        print("%5d %6.2f %8s %8s %8d %8d %6s" %
              (r["i"], r["t"],
               "-" if r["gt_u"] is None else "%.1f" % r["gt_u"],
               "-" if r["u"] is None else "%.1f" % r["u"],
               r["n_events"], r["n_kept"], "YES" if r["detected"] else "no"))
    print("-" * 66)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", default=DATA, help="offline event dataset JSON")
    ap.add_argument("--self-test", action="store_true",
                    help="run synthetic regression tests and exit")
    ap.add_argument("--collect", action="store_true",
                    help="collect a new dataset from a live CARLA server")
    ap.add_argument("--host", default="127.0.0.1", help="CARLA host")
    ap.add_argument("--port", type=int, default=2000, help="CARLA port")
    ap.add_argument("--no-plot", action="store_true", help="do not generate PNG")
    ap.add_argument("--out", default=OUT_DIR, help="directory for CSV/JSON results")
    ap.add_argument("--cell", type=int, default=None, help="BAF grid cell, px")
    ap.add_argument("--resid-k", type=float, default=None,
                    help="background residual sigma multiplier")
    ap.add_argument("--min-mass", type=float, default=None,
                    help="minimum residual mass of a detection")
    args = ap.parse_args(argv)

    if args.self_test:
        print("=== synthetic self-test ===")
        s = dvs_core.self_test()
        print("  PASS  BAF suppressed %.1f%% of background events" %
              (s["background_suppression"] * 100))
        print("  PASS  detection %.1f%%  centroid MAE %.2f px  "
              "false alarms %d" %
              (s["detection_rate"] * 100, s["centroid_mae_px"],
               s["false_alarms"]))
        print("  PASS  velocity %.1f px/s vs ground truth %.1f px/s" %
              (s["velocity_est_px_s"], s["velocity_gt_px_s"]))
        print("  PASS  DVS packed record stride = 13 bytes")
        print("self-test passed. Run `python main.py` for the measured replay.")
        return 0

    if args.collect:
        from dvs_collect import collect
        os.makedirs(IMG_DIR, exist_ok=True)
        print("=== collecting DVS events from live CARLA ===")
        collect(host=args.host, port=args.port, out_json=args.data,
                out_rgb=os.path.join(IMG_DIR, "scene_rgb.png"))
        print("collection finished. Run `python main.py` to replay.")
        return 0

    if not os.path.isfile(args.data):
        print("dataset not found: %s" % args.data, file=sys.stderr)
        print("run `python main.py --collect` with a live CARLA server, or "
              "use --data to point at a dataset.", file=sys.stderr)
        return 2

    cfg = dict(dvs_core.DEFAULTS)
    if args.cell is not None:
        cfg["cell"] = args.cell
    if args.resid_k is not None:
        cfg["resid_k"] = args.resid_k
    if args.min_mass is not None:
        cfg["min_cluster_weight"] = args.min_mass

    meta, frames = dvs_core.load_dataset(args.data)
    records, stats, detections = dvs_core.run_pipeline(meta, frames, cfg)
    print_report(meta, records, stats)

    os.makedirs(args.out, exist_ok=True)
    csv_path = os.path.join(args.out, "dvs_track.csv")
    json_path = os.path.join(args.out, "dvs_metrics.json")
    dvs_core.save_csv(records, csv_path)
    dvs_core.save_json({"meta": meta, "stats": stats}, json_path)
    print("saved: %s" % csv_path)
    print("saved: %s" % json_path)

    if not args.no_plot:
        try:
            from dvs_plot import plot_analysis
            png = os.path.join(repo_img_dir(), "event_analysis.png")
            plot_analysis(meta, frames, records, stats, detections, png, cfg)
            print("saved: %s" % png)
        except ImportError:
            print("Pillow not available; skipped PNG (re-run without --no-plot "
                  "after installing Pillow).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
