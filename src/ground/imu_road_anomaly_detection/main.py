"""IMU-based road anomaly detection — entry point.

Detects road surface anomalies (potholes, speed bumps, rough patches) from
vertical accelerometer data collected in CARLA simulation.

Modes
-----
- ``--self-test`` : synthetic signal validation (no CARLA needed)
- ``--simulate``  : generate a labelled quarter-car dataset, then detect
- ``--collect``   : live CARLA data collection (IMU/GNSS/RGB) + detection
- (default)       : offline analysis of pre-collected JSON data

Two datasets ship with the module:

* ``data/imu_drive.json``   — labelled physics simulation (7 ground-truth
  events, used for the quantitative metrics)
* ``data/carla_drive.json`` — real live CARLA Town10HD recording with **6
  ground-truth excitation events**, also used for quantitative evaluation
  (one start-up pitch transient produces a reproducible false alarm)

Examples
--------

::

    # self test (no third-party dependencies)
    python main.py --self-test

    # quantitative evaluation on the labelled physics dataset
    python main.py --data data/imu_drive.json --results sim_results.json

    # quantitative evaluation on the labelled CARLA recording (6 GT events)
    python main.py --data data/carla_drive.json --results carla_results.json

    # full pipeline: collect from CARLA + detect + plot
    python main.py --collect --host 127.0.0.1 --port 2000 --duration 40
"""

from __future__ import annotations

import argparse
import json
import os
import sys

# ensure local imports work when run from any directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from imu_anomaly import (
    DEFAULTS,
    load_samples_with_gps,
    gravity_align,
    high_pass_residual,
    detect_events,
    summarize,
    self_test as _self_test,
)


_MODULE_DIR = os.path.dirname(os.path.abspath(__file__))


def _disp(path: str) -> str:
    """Show paths relative to the module so terminal output stays compact."""
    try:
        return os.path.relpath(path, _MODULE_DIR).replace("\\", "/")
    except ValueError:
        return path


def run_offline(data_path: str, output_dir: str, make_plot: bool = True,
                verbose: bool = True,
                results_name: str = "detection_results.json",
                plot_name: str = "imu_analysis.png") -> dict:
    """Process pre-collected IMU data and produce results."""
    if not os.path.exists(data_path):
        print(f"  Data file not found: {data_path}")
        return {}

    samples, gps = load_samples_with_gps(data_path)
    if not samples:
        print("  No IMU samples in data file.")
        return {}

    cfg = dict(DEFAULTS)
    # auto-detect sample rate from the median timestamp gap (robust to a
    # missing sample in the first frame, which CARLA sometimes produces)
    if len(samples) > 10:
        import statistics
        gaps = [samples[i + 1].t - samples[i].t
                for i in range(min(200, len(samples) - 1))]
        gaps = [g for g in gaps if g > 0]
        if gaps:
            cfg["sample_rate"] = int(round(1.0 / statistics.median(gaps)))
    # Honour the dataset's declared rate only when no timestamps are
    # available.  Real CARLA recordings can be slower than the requested
    # sensor_tick, so the measured spacing is authoritative.
    if len(samples) <= 10:
        with open(data_path, "r", encoding="utf-8") as f:
            _meta = json.load(f)
        declared = _meta.get("scene", {}).get("sample_rate_hz")
        if declared:
            cfg["sample_rate"] = int(declared)

    print(f"  Loaded {len(samples)} IMU samples "
          f"({len(gps)} GPS points) from {_disp(data_path)}")
    print(f"  Sample rate: ~{cfg['sample_rate']} Hz")
    print(f"  Duration: {samples[-1].t - samples[0].t:.1f} s")

    gx, gy, gz = gravity_align(samples, cfg["baseline_frames"])
    print(f"  Gravity estimate: ({gx:.3f}, {gy:.3f}, {gz:.3f}) m/s^2")

    residual = high_pass_residual(
        samples, gx, gy, gz, cfg["hp_window"],
        cfg.get("glitch_limit", 200.0))

    # compute baseline noise stats
    baseline_std = 0.0
    if len(residual) > cfg["baseline_frames"]:
        import statistics
        baseline_std = statistics.stdev(
            residual[:cfg["baseline_frames"] * 2]
        )
    print(f"  Baseline noise std: {baseline_std:.4f} m/s^2")

    events = detect_events(residual, samples, cfg, gps)

    summary = summarize(events)
    print(f"\n  Detected {summary['total']} anomalies:")
    for label, count in summary.get("labels", {}).items():
        print(f"    {label}: {count}")
    if events:
        print(f"  Mean peak amplitude: {summary['mean_amp']:.2f} m/s^2")
        print(f"  Mean duration:       {summary['mean_dur']:.4f} s")
        print(f"  Mean energy:         {summary['mean_energy']:.2f}")
        centroid = summary.get("mean_centroid")
        print("  Mean centroid freq:  %s" %
              ("n/a (no resolvable event windows; coverage %s)"
               % summary["centroid_coverage"] if centroid is None
               else "%.1f Hz (coverage %s)"
               % (centroid, summary["centroid_coverage"])))
        print(f"  Mean confidence:     {summary['mean_confidence']:.2f}")

    if verbose:
        print("\n  Event details:")
        for i, ev in enumerate(events):
            loc_str = ""
            if ev.lat is not None:
                loc_str = f"  ({ev.lat:.6f}, {ev.lon:.6f})"
            centroid = ("n/a" if ev.spectral_centroid is None
                        else "%.1fHz" % ev.spectral_centroid)
            print(f"    [{i+1}] t={ev.t_peak:.2f}s  amp={ev.peak_az:.2f} m/s^2  "
                  f"dur={ev.duration:.3f}s  f={centroid}  "
                  f"label={ev.label}  conf={ev.confidence:.2f}{loc_str}")

    # save results
    os.makedirs(output_dir, exist_ok=True)
    results = {
        "summary": summary,
        "events": [
            {
                "t_start": ev.t_start, "t_end": ev.t_end,
                "t_peak": ev.t_peak, "peak_az": ev.peak_az,
                "duration": ev.duration, "energy": ev.energy,
                "spectral_centroid": ev.spectral_centroid,
                "label": ev.label, "confidence": ev.confidence,
                "crest_factor": ev.crest_factor,
                "eff_duration": ev.eff_duration,
                "lat": ev.lat, "lon": ev.lon,
            }
            for ev in events
        ],
        "stats": {
            "n_samples": len(samples),
            "n_gps": len(gps),
            "sample_rate": cfg["sample_rate"],
            "duration_s": round(samples[-1].t - samples[0].t, 2),
            "gravity": {"x": round(gx, 4), "y": round(gy, 4), "z": round(gz, 4)},
            "baseline_noise_std": round(baseline_std, 6),
        },
    }
    results_path = os.path.join(output_dir, results_name)
    with open(results_path, "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=2)
    print(f"\n  Results saved to {_disp(results_path)}")

    # ground-truth evaluation (when the dataset carries labels)
    try:
        from imu_eval import evaluate, print_report
        metrics = evaluate(data_path, results_path)
        if "error" not in metrics:
            results["evaluation"] = metrics
            with open(results_path, "w", encoding="utf-8") as f:
                json.dump(results, f, ensure_ascii=False, indent=2)
            print_report(metrics)
    except Exception as e:
        print(f"  Evaluation skipped: {e}")

    # generate plot
    if make_plot:
        try:
            from imu_plot import generate_plot
            plot_path = os.path.join(output_dir, plot_name)
            generate_plot(data_path, plot_path, events)
        except Exception as e:
            print(f"  Plot generation skipped: {e}")

    return results


def run_collect(host: str, port: int, duration: float, output_dir: str,
                results_name: str, plot_name: str,
                make_plot: bool = True, inject_anomalies: bool = True) -> dict:
    """Collect data from CARLA then run detection."""
    from imu_collect import collect
    # live CARLA data must never overwrite the labelled simulation dataset
    data_path = os.path.join(output_dir, "carla_drive.json")
    print(f"  Starting CARLA collection ({duration:.0f}s)...")
    collect(host=host, port=port, duration=duration, output=data_path,
            inject_anomalies=inject_anomalies)
    return run_offline(data_path, output_dir, make_plot=make_plot,
                       results_name=results_name, plot_name=plot_name)


def main():
    parser = argparse.ArgumentParser(
        description="IMU-based road anomaly detection",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--self-test", action="store_true",
                        help="Run synthetic signal validation")
    parser.add_argument("--collect", action="store_true",
                        help="Collect live data from CARLA")
    parser.add_argument("--simulate", action="store_true",
                        help="Generate physics-based IMU data (no CARLA needed)")
    parser.add_argument("--host", default="127.0.0.1",
                        help="CARLA server host")
    parser.add_argument("--port", type=int, default=2000,
                        help="CARLA server port")
    parser.add_argument("--duration", type=float, default=44.0,
                        help="Recording / simulation duration in seconds")
    parser.add_argument("--no-inject", action="store_true",
                        help="CARLA: record smooth road without excitation")
    parser.add_argument("--data", default="data/imu_drive.json",
                        help="Path to pre-collected IMU JSON")
    parser.add_argument("--output", default="data",
                        help="Output directory for results")
    parser.add_argument("--no-plot", action="store_true",
                        help="Skip plot generation (standard library only)")
    parser.add_argument("--results", default="detection_results.json",
                        help="Result JSON filename inside --output")
    parser.add_argument("--plot-name", default="imu_analysis.png",
                        help="Analysis PNG filename inside --output")
    args = parser.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    data_path = os.path.join(here, args.data)
    output_dir = os.path.join(here, args.output)

    if args.self_test:
        print("Running self-test...")
        ok = _self_test()
        sys.exit(0 if ok else 1)

    if args.collect:
        print("Collecting + detecting...")
        run_collect(args.host, args.port, args.duration, output_dir,
                    args.results, args.plot_name, make_plot=not args.no_plot,
                    inject_anomalies=not args.no_inject)
        return

    if args.simulate:
        print("Generating physics-based IMU data...")
        from imu_sim_gen import generate_dataset
        data_path = os.path.join(output_dir, "imu_drive.json")
        generate_dataset(data_path, duration=args.duration)
        print("Running detection on simulated data...")
        run_offline(data_path, output_dir, make_plot=not args.no_plot,
                    results_name=args.results, plot_name=args.plot_name)
        return

    if os.path.exists(data_path):
        print("Running offline analysis...")
        run_offline(data_path, output_dir, make_plot=not args.no_plot,
                    results_name=args.results, plot_name=args.plot_name)
    else:
        print("No pre-collected data found; running self-test...")
        ok = _self_test()
        sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
