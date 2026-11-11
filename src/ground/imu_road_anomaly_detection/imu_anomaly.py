"""IMU road anomaly detection core.

Detects road surface anomalies (potholes, speed bumps, rough patches) from
vertical accelerometer data collected by a CARLA IMU sensor.

Pipeline
--------
1.  Gravity removal – estimate a **fixed** three-axis gravity vector from the
    warm-up-excluded recording median, project every sample onto it, then
    subtract the projected median.  This is a static-attitude estimate only:
    dynamic pitch/roll is *not* compensated (that needs attitude angles or
    gyroscope integration), which is exactly why the 41.00 s start-up pitch
    transient in the CARLA recording remains a false alarm.
2.  High-pass residual – subtract a rolling mean to remove slow drift.
3.  Sliding-window energy – compute the vertical acceleration energy in each
    window.  Anomalies produce energy spikes that are *k* standard deviations
    above the windowed baseline.
4.  Event segmentation – merge adjacent flagged windows, extract peak amplitude,
    duration, crest factor, effective duration and (only when the window is
    long enough to resolve them) spectral centroid.
5.  Classification – rule-based classifier maps the feature vector to one of
    {pothole, speed_bump, rough_patch} using amplitude, duration, crest factor
    and effective duration.  The spectral centroid is reported as a diagnostic
    feature only when the event window has sufficient frequency resolution.

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
    spectral_centroid: Optional[float]  # Hz; None when window too short to resolve
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
    "glitch_limit": 200.0,      # m/s^2 — unphysical single-sample IMU spikes
    "crest_impulse_min": 2.0,   # peak/RMS: sharp discrete impact vs. sustained hump
    "centroid_max_freq": 30.0,  # Hz — upper bound of the Goertzel scan
    "centroid_min_res_hz": 2.0, # Hz — require resolution at least this fine
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
    """DFT power at an arbitrary (fractional-bin) frequency via Goertzel.

    The original implementation rounded ``k = round(freq * n / fs)`` to an
    integer bin.  For a short event window (e.g. 10 samples at 100 Hz) the
    resolution is 10 Hz, so 1 Hz / 2 Hz / ... all collapsed onto k=0 and the
    "spectral centroid" became a weighted average of rounding noise.  Keeping
    ``k`` fractional evaluates the requested frequency directly instead.
    """
    n = len(samples)
    if n == 0:
        return 0.0
    w = 2.0 * math.pi * freq / fs
    coeff = 2.0 * math.cos(w)
    s_prev, s_prev2 = 0.0, 0.0
    for x in samples:
        s = x + coeff * s_prev - s_prev2
        s_prev2 = s_prev
        s_prev = s
    power = s_prev2 ** 2 + s_prev ** 2 - coeff * s_prev * s_prev2
    return power / (n * n)


def _spectral_centroid(samples: List[float], fs: float,
                       max_freq: float = 30.0,
                       min_res_hz: float = 2.0) -> Optional[float]:
    """Spectral centroid, or ``None`` when the window cannot resolve it.

    Resolution of a length-*n* window is ``fs / n``.  A 10-sample window at
    100 Hz resolves only 10 Hz, so reporting a centroid (let alone a 1 Hz
    grid) is misleading.  We require ``fs / n <= min_res_hz`` and sample the
    spectrum on that actual resolution grid instead of pretending 1 Hz bins
    exist.  Checking only ``n < fs / max_freq`` would be far too permissive.
    """
    if not samples or fs <= 0:
        return None
    n = len(samples)
    res = fs / n
    if res > min_res_hz:
        return None
    freqs = [res * i for i in range(1, int(max_freq / res) + 1)]
    if not freqs:
        return None
    powers = [_goertzel_power(samples, f, fs) for f in freqs]
    total = sum(powers)
    if total < 1e-12:
        return None
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
    """Estimate a *fixed* gravity vector robustly.

    A CARLA IMU needs a short warm-up (~0.5 s) before it reports gravity.
    We therefore skip the first ``baseline_frames`` samples and take the
    **median** of the remaining signal: the median is insensitive to both
    isolated sensor glitches and the transient acceleration caused by road
    anomalies (which occupy only a small fraction of the recording).

    This is a **static** attitude estimate, not a per-sample attitude
    solution: ``high_pass_residual`` projects onto this one fixed vector.
    Dynamic pitch/roll still couples longitudinal/lateral acceleration into
    the vertical channel; removing that needs attitude angles or gyro data.
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

    Gravity is estimated as the fixed vector ``(gx, gy, gz)`` by
    :func:`gravity_align`.  Every sample is projected onto its normalised
    direction and the median of that projection is subtracted, so all three
    components are actually used (the previous version only did ``az - gz``).
    Unphysical single-sample spikes are rejected first, then a rolling-mean
    high-pass removes low-frequency drift from road grade.

    Limitation: a fixed vector cannot follow dynamic pitch/roll.  The 41.00 s
    CARLA start-up transient remains a false alarm by design; full removal
    requires attitude angles / gyroscope integration.
    """
    g_norm = math.sqrt(gx * gx + gy * gy + gz * gz)
    if g_norm < 1e-6:
        # Degenerate gravity estimate: fall back to the sensor's z axis.
        ux, uy, uz = 0.0, 0.0, 1.0
    else:
        ux, uy, uz = gx / g_norm, gy / g_norm, gz / g_norm
    projected = [s.ax * ux + s.ay * uy + s.az * uz for s in samples]
    g_med = _median(projected)
    raw_z = [p - g_med for p in projected]
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
        centroid = _spectral_centroid(
            chunk, fs,
            max_freq=cfg.get("centroid_max_freq", 30.0),
            min_res_hz=cfg.get("centroid_min_res_hz", 2.0))

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
            spectral_centroid=(round(centroid, 2)
                               if centroid is not None else None),
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
            centroids = [c for c in (prev.spectral_centroid,
                                     ev.spectral_centroid) if c is not None]
            merged_centroid = (_mean(centroids) if centroids else None)
            new_label, _ = _classify(
                max(prev.peak_az, ev.peak_az), new_dur,
                merged_centroid, cfg,
                crest_factor=max(prev.crest_factor, ev.crest_factor),
                eff_duration=max(prev.eff_duration, ev.eff_duration),
            )
            merged_events[-1] = AnomalyEvent(
                t_start=prev.t_start, t_end=ev.t_end,
                t_peak=new_peak.t_peak,
                peak_az=max(prev.peak_az, ev.peak_az),
                duration=round(new_dur, 4),
                energy=round(prev.energy + ev.energy, 4),
                spectral_centroid=(round(merged_centroid, 2)
                                   if merged_centroid is not None else None),
                label=new_label,
                confidence=round(max(prev.confidence, ev.confidence), 3),
                crest_factor=max(prev.crest_factor, ev.crest_factor),
                eff_duration=max(prev.eff_duration, ev.eff_duration),
                lat=ev.lat, lon=ev.lon,
            )
        else:
            merged_events.append(ev)

    return merged_events


def _classify(amp: float, dur: float, centroid: Optional[float],
              cfg: dict, crest_factor: float = 0.0,
              eff_duration: float = 0.0) -> Tuple[str, float]:
    """Classify an anomaly from its physical signature.

    The **label** is decided by amplitude + total duration (the three rules
    below).  All the extra features are genuinely consumed here to shape the
    **confidence**, so none of ``centroid`` / ``crest_factor`` /
    ``eff_duration`` is a dead parameter any more:

    * ``crest_factor`` (peak/RMS) and ``eff_duration`` (FWHM) separate a
      sharp impulse from a smooth hump and modulate confidence by up to 0.1;
    * ``centroid`` is used only as a weak corroborating term when resolvable
      (``None`` for short windows is explicitly handled, never coerced to 0).

    Crucially the pothole confidence is a strictly increasing function of
    ``amp`` above ``pothole_amp_min`` — the old
    ``(pa_min/amp)*(amp/pa_min) == 1`` identity made every pothole 0.98, and
    a linearly clamped margin merely moved the saturation point to 0.85.
    """
    pa_min = cfg.get("pothole_amp_min", 5.0)
    bd_min = cfg.get("bump_dur_min", 0.12)
    ra_max = cfg.get("rough_amp_max", 5.0)
    rd_min = cfg.get("rough_dur_min", 0.30)
    crest_impulse_min = cfg.get("crest_impulse_min", 2.0)

    # Sharpness evidence from the two previously-unused shape features.
    sharp_bonus = 0.0
    if crest_factor >= crest_impulse_min:
        sharp_bonus += 0.05
    if 0 < eff_duration < bd_min:
        sharp_bonus += 0.05
    # A resolvable high centroid corroborates a high-frequency impulse.
    if centroid is not None and centroid > 10.0:
        sharp_bonus += 0.02

    # 幅值裕度：用 1 - pa_min/amp 而不是截断到 1 的线性裕度。
    # 截断会让 10 / 100 / 1000 m/s² 的马氏裕度都等于 1，置信度全部撞在
    # 同一个 0.85 上——那和原来的常数 0.98 一样失去了"幅值越大越可信"的含义。
    # 饱和型裕度在 amp >= pa_min 时严格单调递增且收敛到 1，既不会溢出也保序。
    amp_margin = 1.0 - pa_min / max(amp, pa_min)

    if amp < ra_max and dur >= rd_min:
        # sustained low-amplitude unevenness; longer = more certain, and a
        # long FWHM (as opposed to one sharp spike) corroborates it.
        conf = 0.5 + 0.4 * min(1.0, dur / (rd_min * 3))
        if eff_duration >= rd_min:
            conf += 0.1
        return "rough_patch", round(min(1.0, conf), 3)

    if amp < ra_max:
        conf = 0.4 + 0.2 * min(1.0, dur / max(rd_min, 0.01))
        return "rough_patch", round(conf, 3)

    if amp >= pa_min and dur >= bd_min:
        # wide strong excitation; a *smooth* profile (low crest factor) is
        # the expected speed-bump signature and raises confidence slightly.
        conf = min(1.0, 0.6 + 0.4 * min(1.0, (dur - bd_min) / bd_min + 0.5))
        if crest_factor < crest_impulse_min:
            conf = min(1.0, conf + 0.05)
        return "speed_bump", round(conf, 3)

    if amp >= pa_min:
        # Strictly increasing in amplitude (saturating margin, no clamp);
        # sharpness adds at most 0.12 on top.
        conf = 0.5 + 0.35 * amp_margin + min(0.12, sharp_bonus)
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
    centroids = [e.spectral_centroid for e in events
                 if e.spectral_centroid is not None]
    result = {
        "total": len(events),
        "labels": labels,
        "mean_amp": round(_mean([e.peak_az for e in events]), 3),
        "mean_dur": round(_mean([e.duration for e in events]), 4),
        "mean_energy": round(_mean([e.energy for e in events]), 3),
        "mean_confidence": round(_mean([e.confidence for e in events]), 3),
        "centroid_coverage": "%d/%d" % (len(centroids), len(events)),
    }
    # Only report a centroid when at least one event window could resolve it;
    # never substitute a 0.0 placeholder for "no frequency resolution".
    if centroids:
        result["mean_centroid"] = round(_mean(centroids), 2)
    return result


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
    residual = high_pass_residual(
        samples, gx, gy, gz, cfg["hp_window"],
        cfg.get("glitch_limit", 200.0))
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
        centroid = ("n/a" if e.spectral_centroid is None
                    else "%.1fHz" % e.spectral_centroid)
        print(f"    t={e.t_peak:.2f}s amp={e.peak_az:.2f} dur={e.duration:.3f}s "
              f"centroid={centroid} label={e.label} "
              f"conf={e.confidence:.2f}")
    return True


if __name__ == "__main__":
    self_test()
