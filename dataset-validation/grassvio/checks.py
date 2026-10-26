"""Section 7 automated validation checks.

Each check returns {"status", "messages", "stats"} where status is one of
"pass", "fail", "warn", "skip".

Thresholds are module constants so the team can tune them in one place;
see README.md for the rationale of each value.
"""

import csv
import glob
import math
import os
import re
from datetime import datetime, timezone

import cv2
import numpy as np

from .schema import validate_meta

# --- thresholds (tunable) -------------------------------------------------
FRAME_COUNT_TOL = 1          # rgb vs depth frame counts must agree within ±1
IMU_COUNT_REL_TOL = 0.02     # imu rows must match meta num_imu_samples within 2%
GAP_FACTOR = 2.0             # flag timestamp gaps larger than 2x nominal dt
DEPTH_VALIDITY_MIN = 0.50    # flag takes with mean depth validity below 50%
GYRO_STATIC_MAX = 0.02       # cell-A: mean |gyro| must be below 0.02 rad/s
FLOW_MEDIAN_MAX_PX = 0.5     # cell-A: median LK flow must be below 0.5 px
NOMINAL_FPS_FALLBACK = 30.0  # used when meta.yaml lacks duration/frame counts
FLOW_MAX_PAIRS = 12          # cap on frame pairs evaluated in the flow smoke test

FRAME_RE = re.compile(r"frame_(\d+)\.png$")


def _check_result(status, messages=None, stats=None):
    return {
        "status": status,
        "messages": messages or [],
        "stats": stats or {},
    }


def _list_frames(subdir):
    paths = sorted(glob.glob(os.path.join(subdir, "frame_*.png")))
    out = []
    for p in paths:
        m = FRAME_RE.search(os.path.basename(p))
        if m:
            out.append((int(m.group(1)), p))
    return out


def _nominal_dt(meta):
    """Nominal frame interval in seconds, from meta or fallback."""
    try:
        dur = float(meta.get("duration_s"))
        n = int(meta.get("num_rgb_frames"))
        if dur > 0 and n > 0:
            return dur / n
    except (TypeError, ValueError):
        pass
    return 1.0 / NOMINAL_FPS_FALLBACK


def _frame_timestamps(seq_dir, frames):
    """Return {frame_index: timestamp_s}.

    Prefers <seq>/timestamps.csv (columns: frame_index,timestamp_s);
    otherwise derives t = index * nominal dt. meta may be None.
    """
    ts_path = os.path.join(seq_dir, "timestamps.csv")
    ts = {}
    if os.path.isfile(ts_path):
        with open(ts_path, newline="") as f:
            reader = csv.DictReader(f)
            if reader.fieldnames and "frame_index" in reader.fieldnames \
                    and "timestamp_s" in reader.fieldnames:
                for row in reader:
                    try:
                        ts[int(row["frame_index"])] = float(row["timestamp_s"])
                    except (ValueError, TypeError):
                        continue
    if not ts and frames:
        # Fallback: derive from frame indices at nominal fps. We cannot know
        # meta here, so use the fallback rate; caller may override.
        for idx, _ in frames:
            ts[idx] = idx / NOMINAL_FPS_FALLBACK
    return ts


def _read_imu(imu_path):
    """Read imu.csv -> (timestamps, gyro (N,3), accel (N,3)).

    Accepts a header row (timestamp,gyro_x,...) or a bare 7-column file.
    Returns (None, None, None) with a message if unreadable.
    """
    if not os.path.isfile(imu_path):
        return None, "imu.csv not found at %s" % imu_path
    rows = []
    with open(imu_path, newline="") as f:
        reader = csv.reader(f)
        first = next(reader, None)
        if first is None:
            return None, "imu.csv is empty"
        # Detect header: first cell not parseable as float.
        try:
            float(first[0])
            has_header = False
        except ValueError:
            has_header = True
        if not has_header:
            rows.append(first)
        for row in reader:
            if row:
                rows.append(row)
    try:
        data = np.array([[float(x) for x in r[:7]] for r in rows], dtype=float)
    except (ValueError, IndexError) as e:
        return None, "imu.csv has malformed rows: %s" % e
    if data.shape[1] < 7:
        return None, "imu.csv needs 7 columns (t,gx,gy,gz,ax,ay,az)"
    return (data[:, 0], data[:, 1:4], data[:, 4:7]), None


# --- individual checks ----------------------------------------------------


def check_frame_counts(seq_dir, meta):
    rgb = _list_frames(os.path.join(seq_dir, "rgb"))
    depth = _list_frames(os.path.join(seq_dir, "depth"))
    imu_path = os.path.join(seq_dir, "imu.csv")
    messages, stats = [], {"n_rgb": len(rgb), "n_depth": len(depth)}

    imu_rows = None
    if os.path.isfile(imu_path):
        with open(imu_path, newline="") as f:
            rows = [r for r in csv.reader(f) if r]
        # discount a header row if present
        try:
            float(rows[0][0])
        except (ValueError, IndexError):
            rows = rows[1:]
        imu_rows = len(rows)
    stats["n_imu"] = imu_rows

    status = "pass"
    if not rgb:
        status, messages = "fail", messages + ["no rgb frames found in rgb/"]
    if not os.path.isdir(os.path.join(seq_dir, "depth")):
        messages.append("depth/ directory missing")
    elif abs(len(rgb) - len(depth)) > FRAME_COUNT_TOL:
        status = "fail"
        messages.append(
            "rgb/depth frame counts disagree: rgb=%d depth=%d (tolerance ±%d)"
            % (len(rgb), len(depth), FRAME_COUNT_TOL)
        )
    if imu_rows is None:
        status = "fail"
        messages.append("imu.csv missing")
    elif meta and isinstance(meta.get("num_imu_samples"), int):
        declared = meta["num_imu_samples"]
        tol = max(2, math.ceil(IMU_COUNT_REL_TOL * declared))
        if abs(imu_rows - declared) > tol:
            status = "fail"
            messages.append(
                "imu sample count %d disagrees with meta num_imu_samples=%d "
                "(tolerance ±%d)" % (imu_rows, declared, tol)
            )
    if status == "pass":
        messages.append(
            "counts agree: rgb=%d depth=%d imu=%s"
            % (len(rgb), len(depth), imu_rows)
        )
    return _check_result(status, messages, stats)


def check_timestamp_gaps(seq_dir, meta):
    messages, stats, status = [], {}, "pass"
    dt_nom = _nominal_dt(meta) if meta else 1.0 / NOMINAL_FPS_FALLBACK
    stats["nominal_dt_s"] = dt_nom
    limit = GAP_FACTOR * dt_nom

    streams = {}
    rgb = _list_frames(os.path.join(seq_dir, "rgb"))
    if rgb:
        ts = _frame_timestamps(seq_dir, rgb)
        # If timestamps were derived at fallback rate but meta gives a better
        # nominal dt, re-derive so the gap test is self-consistent.
        if not os.path.isfile(os.path.join(seq_dir, "timestamps.csv")) and meta:
            ts = {idx: idx * dt_nom for idx, _ in rgb}
        ordered = [ts[i] for i, _ in rgb if i in ts]
        streams["rgb"] = ordered
    imu_data, imu_err = _read_imu(os.path.join(seq_dir, "imu.csv"))
    if imu_data is not None:
        streams["imu"] = list(imu_data[0])

    if not streams:
        return _check_result("skip", ["no timestamped streams available"], stats)

    for name, t in streams.items():
        t = sorted(t)
        if len(t) < 2:
            messages.append("%s: only %d timestamp(s), gap check trivial" % (name, len(t)))
            continue
        diffs = np.diff(np.asarray(t, dtype=float))
        worst = float(np.max(diffs))
        n_bad = int(np.sum(diffs > limit))
        stats["%s_max_gap_s" % name] = worst
        stats["%s_gaps_over_2x" % name] = n_bad
        if n_bad:
            status = "fail"
            messages.append(
                "%s: %d gap(s) exceed 2x nominal dt (%.3fs); worst=%.3fs"
                % (name, n_bad, limit, worst)
            )
    if status == "pass":
        messages.append(
            "no gaps exceed 2x nominal dt (%.3fs) on %s"
            % (limit, ", ".join(sorted(streams)))
        )
    return _check_result(status, messages, stats)


def check_depth_validity(seq_dir, meta):
    depth_dir = os.path.join(seq_dir, "depth")
    frames = _list_frames(depth_dir)
    if not frames:
        return _check_result("fail", ["no depth frames found in depth/"], {"n_depth": 0})
    fracs = []
    for _, p in frames:
        d = cv2.imread(p, cv2.IMREAD_UNCHANGED)
        if d is None:
            continue
        fracs.append(float(np.count_nonzero(d)) / d.size)
    if not fracs:
        return _check_result("fail", ["depth frames unreadable"], {"n_depth": len(frames)})
    mean_v = float(np.mean(fracs))
    min_v = float(np.min(fracs))
    stats = {
        "n_depth": len(frames),
        "depth_validity_mean": round(mean_v, 4),
        "depth_validity_min": round(min_v, 4),
    }
    if mean_v < DEPTH_VALIDITY_MIN:
        return _check_result(
            "fail",
            ["mean depth validity %.1f%% is below the 50%% flag threshold"
             % (100 * mean_v)],
            stats,
        )
    return _check_result(
        "pass",
        ["mean depth validity %.1f%% (min %.1f%%)" % (100 * mean_v, 100 * min_v)],
        stats,
    )


def check_imu_static_bias(seq_dir, meta):
    cell = str((meta or {}).get("matrix_cell", "")).strip()
    if cell != "A":
        return _check_result(
            "skip", ["static-bias sanity applies to cell-A takes only"], {}
        )
    imu_data, err = _read_imu(os.path.join(seq_dir, "imu.csv"))
    if imu_data is None:
        return _check_result("fail", [err], {})
    _, gyro, _ = imu_data
    mag = np.linalg.norm(gyro, axis=1)
    mean_mag = float(np.mean(mag))
    stats = {"gyro_mean_magnitude_rad_s": round(mean_mag, 6),
             "n_imu": int(len(mag))}
    if mean_mag > GYRO_STATIC_MAX:
        return _check_result(
            "fail",
            ["cell-A take but mean |gyro| = %.4f rad/s exceeds %.3f rad/s; "
             "camera was not still or IMU is misbehaving"
             % (mean_mag, GYRO_STATIC_MAX)],
            stats,
        )
    return _check_result(
        "pass",
        ["cell-A static sanity ok: mean |gyro| = %.4f rad/s" % mean_mag],
        stats,
    )


def check_flow_smoke(seq_dir, meta):
    cell = str((meta or {}).get("matrix_cell", "")).strip()
    if cell != "A":
        return _check_result(
            "skip", ["optical-flow smoke test applies to cell-A takes only"], {}
        )
    frames = _list_frames(os.path.join(seq_dir, "rgb"))
    if len(frames) < 2:
        return _check_result("fail", ["need >= 2 rgb frames for flow test"], {})
    # Evenly spaced consecutive pairs, capped for speed.
    n_pairs = min(FLOW_MAX_PAIRS, len(frames) - 1)
    step = max(1, (len(frames) - 1) // n_pairs)
    pair_idx = list(range(0, len(frames) - 1, step))[:n_pairs]

    medians = []
    lk_params = dict(winSize=(21, 21), maxLevel=3,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                               30, 0.01))
    for k in pair_idx:
        g1 = cv2.imread(frames[k][1], cv2.IMREAD_GRAYSCALE)
        g2 = cv2.imread(frames[k + 1][1], cv2.IMREAD_GRAYSCALE)
        if g1 is None or g2 is None:
            continue
        pts = cv2.goodFeaturesToTrack(g1, maxCorners=200, qualityLevel=0.01,
                                      minDistance=7)
        if pts is None:
            continue
        p1, st, _ = cv2.calcOpticalFlowPyrLK(g1, g2, pts, None, **lk_params)
        ok = st.reshape(-1).astype(bool)
        if not np.any(ok):
            continue
        disp = np.linalg.norm(p1.reshape(-1, 2)[ok] - pts.reshape(-1, 2)[ok], axis=1)
        medians.append(float(np.median(disp)))
    if not medians:
        return _check_result(
            "warn", ["no trackable points found; flow test inconclusive"], {}
        )
    med = float(np.median(medians))
    stats = {"flow_median_px": round(med, 4), "pairs_evaluated": len(medians)}
    if med > FLOW_MEDIAN_MAX_PX:
        return _check_result(
            "fail",
            ["cell-A take but median LK flow = %.2f px exceeds %.1f px; "
             "unexpected image motion" % (med, FLOW_MEDIAN_MAX_PX)],
            stats,
        )
    return _check_result(
        "pass",
        ["cell-A flow smoke ok: median flow = %.3f px over %d pairs"
         % (med, len(medians))],
        stats,
    )


def run_all_checks(seq_dir):
    """Run the full Section 7 suite on a sequence directory.

    Returns the report dict (also serializable to the validation JSON).
    """
    seq_dir = os.path.abspath(seq_dir)
    seq_id = os.path.basename(seq_dir.rstrip(os.sep))

    meta_res = validate_meta(os.path.join(seq_dir, "meta.yaml"),
                             seq_dir_name=seq_id)
    meta = meta_res["meta"]

    checks = {
        "meta_schema": _check_result(
            meta_res["status"],
            meta_res["errors"] + meta_res["warnings"],
            {"n_errors": len(meta_res["errors"]),
             "n_warnings": len(meta_res["warnings"])},
        ),
        "frame_counts": check_frame_counts(seq_dir, meta),
        "timestamp_gaps": check_timestamp_gaps(seq_dir, meta),
        "depth_validity": check_depth_validity(seq_dir, meta),
        "imu_static_bias": check_imu_static_bias(seq_dir, meta),
        "flow_smoke": check_flow_smoke(seq_dir, meta),
    }

    statuses = [c["status"] for c in checks.values()]
    if "fail" in statuses:
        overall = "fail"
    elif "warn" in statuses:
        overall = "warn"
    else:
        overall = "pass"

    summary = {
        "sequence_id": seq_id,
        "matrix_cell": str((meta or {}).get("matrix_cell", "")),
        "n_checks": len(checks),
        "n_pass": statuses.count("pass"),
        "n_fail": statuses.count("fail"),
        "n_warn": statuses.count("warn"),
        "n_skip": statuses.count("skip"),
    }
    for name in ("frame_counts", "depth_validity", "imu_static_bias", "flow_smoke"):
        for k, v in checks[name]["stats"].items():
            if k not in summary:
                summary[k] = v

    return {
        "sequence_id": seq_id,
        "validator": "grassvio-validate 0.1.0",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "sequence_dir": seq_dir,
        "overall": overall,
        "checks": checks,
        "summary": summary,
    }
