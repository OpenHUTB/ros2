"""IMU road anomaly detection core.

Detects road surface anomalies (potholes, speed bumps, rough patches) from
vertical accelerometer data collected by a CARLA IMU sensor.

Pipeline
--------
1.  Gravity alignment – estimate gravity direction from a static baseline and
    project it out so the residual is pure road-induced vibration.
2.  High-pass residual – subtract a rolling mean to remove slow drift.
3.  Sliding-window energy – compute the vertical acceleration energy in each
    window.  Anomalies produce energy spikes that are *k* standard deviations
    above the windowed baseline.
4.  Event segmentation – merge adjacent flagged windows, extract peak amplitude,
    duration and spectral centroid for each event.
5.  Classification – rule-based classifier maps the feature vector to one of
    {pothole, speed_bump, rough_patch} using amplitude, duration and frequency
    ratios learned from the labelled CARLA scenes.

The entire module uses only the Python standard library so it can run in any
environment without installing numpy / scipy.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# ── data structures ──────────────────────────────────────────────────────

@dataclass
class IMUSample:
    """One IMU measurement at time *t* (seconds)."""
    t: float
    ax: float          # m/s^2
    ay: float
    az: float
    gx: float = 0.0    # rad/s (gyro, unused in v1 but kept for future)
    gy: float = 0.0
    gz: float = 0.0


@dataclass
class AnomalyEvent:
    """A detected road anomaly."""
    t_start: float
    t_end: float
    t_peak: float
    peak_az: float          # m/s^2 (gravity-removed vertical residual)
    duration: float          # seconds
    energy: float            # sum of squared residual in window
    spectral_centroid: float # Hz
    label: str               # pothole | speed_bump | rough_patch
    confidence: float        # 0..1
    crest_factor: float = 0.0    # peak / RMS of the event window
    eff_duration: float = 0.0    # FWHM of the impact (seconds)
    lat: Optional[float] = None
    lon: Optional[float] = None


# ── configurable defaults ────────────────────────────────────────────────

DEFAULTS = {
    "sample_rate": 100,          # Hz (CARLA IMU sensor_tick)
    "gravity": 9.81,             # m/s^2
    "baseline_frames": 50,       # frames for gravity estimation
    "hp_window": 30,             # high-pass rolling mean window (frames)
    "detect_window": 10,         # detection sliding window (frames)
    "detect_stride": 3,          # stride between detection windows
    "sigma_k": 3.0,             # anomaly threshold = k * std
    "min_event_gap": 0.05,      # merge events closer than this (seconds)
    "min_event_duration": 0.02, # discard events shorter than this (seconds)
    "min_span_s": 0.10,         # minimum event window around a sharp impact
                                # (seconds, not frames: the two datasets run
                                #  at 100 Hz and 50 Hz respectively)
    "settling_guard": 0.5,      # ignore first 0.5 s (sensor/filter settling)
    "rough_merge_gap": 1.5,     # merge low-amp events within 1.5 s (one patch)
    # classification thresholds (tuned on the labelled simulation dataset)
    "pothole_amp_min": 5.0,     # m/s^2 — above this is a discrete impact
    "bump_dur_min": 0.12,       # s — wide excitation = speed bump
    "rough_amp_max": 5.0,       # m/s^2 — sustained low-amplitude vibration
    "rough_dur_min": 0.30,      # s
}


# ── helpers ──────────────────────────────────────────────────────────────

def _mean(vals: List[float]) -> float:
    return sum(vals) / len(vals) if vals else 0.0


def _std(vals: List[float]) -> float:
    if len(vals) < 2:
        return 0.0
    m = _mean(vals)
    return math.sqrt(sum((v - m) ** 2 for v in vals) / (len(vals) - 1))


def _rolling_mean(vals: List[float], window: int) -> List[float]:
    """Centered rolling mean for high-pass filtering."""
    n = len(vals)
    half = window // 2
    out = [0.0] * n
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        out[i] = _mean(vals[lo:hi])
    return out


def _goertzel_power(samples: List[float], freq: float, fs: float) -> float:
    """Single-bin DFT power via Goertzel algorithm (no scipy needed)."""
    n = len(samples)
    if n == 0:
        return 0.0
    k = int(round(freq * n / fs))
    w = 2.0 * math.pi * k / n
    coeff = 2.0 * math.cos(w)
    s_prev, s_prev2 = 0.0, 0.0
    for x in samples:
        s = x + coeff * s_prev - s_prev2
        s_prev2 = s_prev
        s_prev = s
    power = s_prev2 ** 2 + s_prev ** 2 - coeff * s_prev * s_prev2
    return power / (n * n)


def _spectral_centroid(samples: List[float], fs: float,
                       max_freq: float = 30.0) -> float:
    """Approximate spectral centroid using Goertzel bins up to *max_freq*."""
    if not samples:
        return 0.0
    n = len(samples)
    freqs = [f for f in range(1, int(max_freq) + 1)]
    powers = [_goertzel_power(samples, f, fs) for f in freqs]
    total = sum(powers)
    if total < 1e-12:
        return 0.0
    return sum(f * p for f, p in zip(freqs, powers)) / total


# ── core pipeline ─────────────────────────────────────────────────────────

def load_samples(path: str) -> List[IMUSample]:
    """Load IMU samples from a JSON file produced by ``imu_collect.py``."""
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return [IMUSample(
        t=d["t"], ax=d["ax"], ay=d["ay"], az=d["az"],
        gx=d.get("gx", 0), gy=d.get("gy", 0), gz=d.get("gz", 0),
    ) for d in data["samples"]]


def load_samples_with_gps(path: str) -> Tuple[List[IMUSample], List[dict]]:
    """Load IMU samples and GPS positions from a collection JSON."""
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    samples = [IMUSample(
        t=d["t"], ax=d["ax"], ay=d["ay"], az=d["az"],
        gx=d.get("gx", 0), gy=d.get("gy", 0), gz=d.get("gz", 0),
    ) for d in data["samples"]]
    gps = data.get("gps", [])
    return samples, gps


def _median(vals: List[float]) -> float:
    s = sorted(vals)
    n = len(s)
    if n == 0:
        return 0.0
    return s[n // 2] if n % 2 else (s[n // 2 - 1] + s[n // 2]) / 2.0


def reject_glitches(raw_z: List[float],
                    limit: float = 200.0) -> Tuple[List[float], int]:
    """Replace unphysical single-sample IMU spikes.

    CARLA occasionally emits an isolated invalid accelerometer sample when a
    vehicle is spawned / teleported (one reading of ~ -12 000 m/s^2 was seen).
    Genuine road impacts reach ~80 m/s^2, so only values further than
    ``limit`` m/s^2 from the global median are treated as sensor glitches and
    replaced by that median.  Returns ``(clean_signal, n_replaced)``.
    """
    med = _median(raw_z)
    out = [med if abs(v - med) > limit else v for v in raw_z]
    return out, sum(1 for a, b in zip(raw_z, out) if a != b)


def gravity_align(samples: List[IMUSample],
                  baseline_frames: int = 50) -> Tuple[float, float, float]:
    """Estimate the gravity vector robustly.

    A CARLA IMU needs a short warm-up (~0.5 s) before it reports gravity.
    We therefore skip the first ``baseline_frames`` samples and take the
    **median** of the remaining signal: the median is insensitive to both
    isolated sensor glitches and the transient acceleration caused by road
    anomalies (which occupy only a small fraction of the recording).
    """
    stable = samples[baseline_frames:] or samples
    bx = _median([s.ax for s in stable])
    by = _median([s.ay for s in stable])
    bz = _median([s.az for s in stable])
    return bx, by, bz


def high_pass_residual(samples: List[IMUSample],
                       gx: float, gy: float, gz: float,
                       hp_window: int = 30,
                       glitch_limit: float = 200.0) -> List[float]:
    """Remove gravity and slow drift; return the vertical residual signal.

    The vertical axis is *z* after gravity subtraction.  Unphysical single
    sample spikes are rejected first (see :func:`reject_glitches`), then a
    rolling-mean high-pass removes low-frequency drift from road grade.
    """
    raw_z = [s.az - gz for s in samples]
    raw_z, _ = reject_glitches(raw_z, glitch_limit)
    baseline = _rolling_mean(raw_z, hp_window)
    return [r - b for r, b in zip(raw_z, baseline)]


def detect_events(residual: List[float], samples: List[IMUSample],
                  cfg: dict, gps: Optional[List[dict]] = None) -> List[AnomalyEvent]:
    """Detect anomaly events from the vertical residual signal."""
    fs = cfg.get("sample_rate", 100)
    win = cfg.get("detect_window", 15)
    stride = cfg.get("detect_stride", 5)
    k = cfg.get("sigma_k", 3.0)
    min_gap = cfg.get("min_event_gap", 0.08)
    min_dur = cfg.get("min_event_duration", 0.02)

    n = len(residual)
    if n < win * 3:
        return []

    # sliding-window energy
    energies = []
    for i in range(0, n - win + 1, stride):
        chunk = residual[i:i + win]
        energies.append((i, sum(x * x for x in chunk)))

    # global baseline (exclude top 10% to avoid anomaly bias)
    sorted_e = sorted(e for _, e in energies)
    cut = int(len(sorted_e) * 0.9)
    baseline_e = _mean(sorted_e[:cut]) if cut > 0 else _mean(sorted_e)
    std_e = _std(sorted_e[:cut]) if cut > 1 else _std(sorted_e)

    threshold = baseline_e + k * std_e

    # find flagged windows
    flagged: List[Tuple[int, int]] = []
    for start, e in energies:
        if e > threshold:
            flagged.append((start, start + win))

    if not flagged:
        return []

    # merge overlapping / adjacent windows
    merged: List[Tuple[int, int]] = []
    for s, e in flagged:
        if merged and s - merged[-1][1] < stride:
            merged[-1] = (merged[-1][0], e)
        else:
            merged.append((s, e))

    # convert to events
    events: List[AnomalyEvent] = []
    # noise floor for trimming (use residual std of baseline region)
    baseline_region = residual[:min(len(residual), cfg["baseline_frames"] * 2)]
    baseline_std = _std(baseline_region) if len(baseline_region) > 1 else 0.01
    for s_idx, e_idx in merged:
        # trim boundaries using FWHM of the peak
        s_trim = s_idx
        e_trim = min(e_idx, n)
        chunk_full = residual[s_trim:e_trim]
        if len(chunk_full) > 2:
            # find the peak in the full window
            pk_idx = max(range(len(chunk_full)), key=lambda i: abs(chunk_full[i]))
            pk_val = abs(chunk_full[pk_idx])
            half_max = pk_val * 0.5
            # trim to where |val| > 50% of peak
            above = [i for i, v in enumerate(chunk_full)
                     if abs(v) > half_max]
            if above:
                lo = s_idx + above[0]
                hi = s_idx + above[-1] + 1
            else:
                lo = hi = s_idx + pk_idx
                hi += 1
            # A single-sample impact (a real pothole strike) has a one-frame
            # FWHM.  Keep a minimum span around the peak, otherwise the
            # min_event_duration check below would silently drop it.
            # Express the minimum span in *seconds*, then convert to frames:
            # 10 frames is 0.1 s at 100 Hz but 0.2 s at 50 Hz, and using a
            # frame count would make sharp 50 Hz impacts look like long
            # speed-bump events.
            min_span = max(1, int(round(
                cfg.get("min_span_s", 0.10) * fs)))
            if hi - lo < min_span:
                centre = (lo + hi) // 2
                lo = centre - min_span // 2
                hi = lo + min_span
                if lo < 0:
                    lo, hi = 0, min_span
                if hi > n:
                    lo, hi = n - min_span, n
            s_trim, e_trim = max(0, lo), min(n, hi)

        t_start = samples[s_trim].t
        t_end = samples[min(e_trim - 1, n - 1)].t
        if t_end - t_start < min_dur:
            continue
        chunk = residual[s_trim:e_trim]
        peak_local = max(range(len(chunk)), key=lambda i: abs(chunk[i]))
        peak_idx = s_trim + peak_local
        peak_az = abs(chunk[peak_local])
        energy = sum(x * x for x in chunk)
        centroid = _spectral_centroid(chunk, fs)

        # RMS of the chunk (for crest factor)
        rms_chunk = math.sqrt(energy / max(1, len(chunk)))
        crest_factor = peak_az / max(rms_chunk, 0.001)

        # effective duration = FWHM of the |residual| peak
        half_max = peak_az / 2.0
        left = peak_local
        while left > 0 and abs(chunk[left]) > half_max:
            left -= 1
        right = peak_local
        while right < len(chunk) - 1 and abs(chunk[right]) > half_max:
            right += 1
        eff_duration = (right - left + 1) / fs

        # total window duration (for classification)
        win_duration = t_end - t_start

        label, conf = _classify(
            peak_az, win_duration, centroid, cfg,
            crest_factor=crest_factor, eff_duration=eff_duration,
        )

        # attach GPS if available
        lat = lon = None
        if gps:
            # find closest GPS sample by time
            best = min(gps, key=lambda g: abs(g["t"] - samples[peak_idx].t))
            lat, lon = best.get("lat"), best.get("lon")

        events.append(AnomalyEvent(
            t_start=t_start, t_end=t_end,
            t_peak=samples[peak_idx].t,
            peak_az=peak_az,
            duration=round(win_duration, 4),
            energy=round(energy, 4),
            spectral_centroid=round(centroid, 2),
            label=label,
            confidence=round(conf, 3),
            crest_factor=round(crest_factor, 3),
            eff_duration=round(eff_duration, 4),
            lat=lat, lon=lon,
        ))

    # ── post-processing ────────────────────────────────────────────────
    # 1) settling guard: the high-pass filter and suspension model need a
    #    short warm-up; discard detections inside the first `settling_guard`s.
    t0 = samples[0].t
    guard = cfg.get("settling_guard", 0.5)
    events = [e for e in events if e.t_peak - t0 >= guard]
    if not events:
        return []

    # 2) merge: consecutive events that belong to one physical excitation.
    rough_gap = cfg.get("rough_merge_gap", 1.5)
    ra_max = cfg.get("rough_amp_max", 5.0)
    merged_events: List[AnomalyEvent] = [events[0]]
    for ev in events[1:]:
        prev = merged_events[-1]
        # a rough patch produces several low-amplitude detections; merge them
        is_rough_pair = (ev.peak_az < ra_max and prev.peak_az < ra_max
                         and ev.t_start - prev.t_end < rough_gap)
        is_close = (ev.t_start - prev.t_end < min_gap)
        if is_rough_pair or is_close:
            new_dur = ev.t_end - prev.t_start
            new_peak = prev if prev.peak_az >= ev.peak_az else ev
            new_label, _ = _classify(
                max(prev.peak_az, ev.peak_az), new_dur,
                (prev.spectral_centroid + ev.spectral_centroid) / 2, cfg,
                crest_factor=max(prev.crest_factor, ev.crest_factor),
                eff_duration=max(prev.eff_duration, ev.eff_duration),
            )
            merged_events[-1] = AnomalyEvent(
                t_start=prev.t_start, t_end=ev.t_end,
                t_peak=new_peak.t_peak,
                peak_az=max(prev.peak_az, ev.peak_az),
                duration=round(new_dur, 4),
                energy=round(prev.energy + ev.energy, 4),
                spectral_centroid=round(
                    (prev.spectral_centroid + ev.spectral_centroid) / 2, 2),
                label=new_label,
                confidence=round(max(prev.confidence, ev.confidence), 3),
                crest_factor=max(prev.crest_factor, ev.crest_factor),
                eff_duration=max(prev.eff_duration, ev.eff_duration),
                lat=ev.lat, lon=ev.lon,
            )
        else:
            merged_events.append(ev)

    return merged_events


def _classify(amp: float, dur: float, centroid: float,
              cfg: dict, crest_factor: float = 0.0,
              eff_duration: float = 0.0) -> Tuple[str, float]:
    """Classify an anomaly from its physical signature.

    Three rules, in priority order, based on how a quarter-car suspension
    responds to different road excitations:

    1. **rough_patch** – low-amplitude (< ``rough_amp_max``) but long
       (>= ``rough_dur_min``) excitation: sustained unevenness, not a
       discrete impact.
    2. **speed_bump** – strong (>= ``pothole_amp_min``) *and* wide
       (>= ``bump_dur_min``) excitation: the vehicle rides over a long hump.
    3. **pothole** – strong *and* narrow excitation: a sharp dip/rebound.

    Confidence grows with the margin above the corresponding threshold.
    """
    pa_min = cfg.get("pothole_amp_min", 5.0)
    bd_min = cfg.get("bump_dur_min", 0.12)
    ra_max = cfg.get("rough_amp_max", 5.0)
    rd_min = cfg.get("rough_dur_min", 0.30)

    if amp < ra_max and dur >= rd_min:
        conf = min(1.0, 0.5 + dur / (rd_min * 3) * 0.5)
        return "rough_patch", round(conf, 3)

    if amp < ra_max:
        conf = 0.4 + 0.2 * min(1.0, dur / max(rd_min, 0.01))
        return "rough_patch", round(conf, 3)

    if amp >= pa_min and dur >= bd_min:
        conf = min(1.0, 0.6 + 0.4 * min(1.0, (dur - bd_min) / bd_min + 0.5))
        return "speed_bump", round(conf, 3)

    if amp >= pa_min:
        conf = min(1.0, 0.6 + 0.4 * min(1.0, (pa_min / max(amp, 0.01)) * (amp / pa_min)))
        return "pothole", round(min(0.98, conf), 3)

    return "pothole", 0.4


# ── summary ──────────────────────────────────────────────────────────────

def summarize(events: List[AnomalyEvent]) -> dict:
    """Return a summary dict for reporting."""
    if not events:
        return {"total": 0, "labels": {}, "mean_amp": 0, "mean_dur": 0}
    labels = {}
    for e in events:
        labels[e.label] = labels.get(e.label, 0) + 1
    return {
        "total": len(events),
        "labels": labels,
        "mean_amp": round(_mean([e.peak_az for e in events]), 3),
        "mean_dur": round(_mean([e.duration for e in events]), 4),
        "mean_energy": round(_mean([e.energy for e in events]), 3),
        "mean_centroid": round(_mean([e.spectral_centroid for e in events]), 2),
        "mean_confidence": round(_mean([e.confidence for e in events]), 3),
    }


# ── self test ────────────────────────────────────────────────────────────

def self_test() -> bool:
    """Run synthetic test: inject a pothole-like spike into clean signal."""
    fs = 100
    n = 500
    cfg = dict(DEFAULTS)
    cfg["sample_rate"] = fs

    # generate clean signal + gravity
    samples = []
    for i in range(n):
        t = i / fs
        az = 9.81 + 0.1 * math.sin(2 * math.pi * 0.5 * t)  # gravity + slow drift
        # inject pothole at t=2.0s (short sharp negative spike)
        if 1.98 <= t <= 2.04:
            az -= 12.0 * math.exp(-((t - 2.01) ** 2) / 0.0002)
        # inject speed bump at t=4.0s (longer, strong positive bump)
        if 3.95 <= t <= 4.20:
            az += 12.0 * math.sin(math.pi * (t - 3.95) / 0.25)
        samples.append(IMUSample(t=t, ax=0, ay=0, az=az))

    gx, gy, gz = gravity_align(samples, cfg["baseline_frames"])
    residual = high_pass_residual(samples, gx, gy, gz, cfg["hp_window"])
    events = detect_events(residual, samples, cfg)

    if len(events) < 2:
        print(f"  self-test FAIL: expected >=2 events, got {len(events)}")
        return False

    # check at least one pothole near t=2.0 and one speed_bump near t=4.0
    has_pothole = any(
        abs(e.t_peak - 2.0) < 0.1 and e.label == "pothole" for e in events
    )
    has_bump = any(
        abs(e.t_peak - 4.0) < 0.2 and e.label == "speed_bump" for e in events
    )
    if not has_pothole:
        print(f"  self-test FAIL: no pothole detected near t=2.0")
        print(f"  events: {[(e.t_peak, e.label, e.peak_az) for e in events]}")
        return False
    if not has_bump:
        print(f"  self-test FAIL: no speed_bump detected near t=4.0")
        print(f"  events: {[(e.t_peak, e.label, e.peak_az) for e in events]}")
        return False

    print(f"  self-test PASS: {len(events)} events detected")
    for e in events:
        print(f"    t={e.t_peak:.2f}s amp={e.peak_az:.2f} dur={e.duration:.3f}s "
              f"centroid={e.spectral_centroid:.1f}Hz label={e.label} "
              f"conf={e.confidence:.2f}")
    return True


if __name__ == "__main__":
    self_test()
