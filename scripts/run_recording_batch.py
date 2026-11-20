#!/usr/bin/env python3
"""Run reproducible baseline/window-filter checks on local GrassVIO recordings.

Raw inputs and outputs remain on this machine. This script never uploads data.
The reported inlier counts and map coverage are diagnostics, not pose truth.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from statistics import median
from datetime import datetime, timezone


CAMERA_MODES = {"visual_plane_only", "known_stationary_whole_recording", "independent_full_motion"}
REPORT_FIELDS = (
    "recording", "camera_mode", "starts_at_rest", "baseline_status", "filter_status",
    "baseline_frames_used", "filter_frames_used", "baseline_pairs_skipped",
    "filter_pairs_skipped", "baseline_median_inliers", "filter_median_inliers",
    "filter_median_heldout_error_px", "filter_windows",
    "filter_median_unfiltered_heldout_error_px", "filter_paired_heldout_windows",
    "filter_paired_median_error_change_px",
)


def stamp():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def atomic_json(path, obj):
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + ".tmp")
    with temp.open("w", encoding="utf-8") as fp:
        json.dump(obj, fp, indent=2, allow_nan=False)
        fp.write("\n")
    os.replace(temp, path)


def read_json(path):
    with path.open(encoding="utf-8") as fp:
        return json.load(fp)


def config_for(name, manifest, manifest_dir):
    entry = manifest.get("recordings", {}).get(name, {})
    mode = entry.get("camera_mode", "visual_plane_only")
    if mode not in CAMERA_MODES:
        raise ValueError(f"{name}: unsupported camera_mode {mode!r}")
    prior = entry.get("camera_motion")
    if mode == "independent_full_motion" and not prior:
        raise ValueError(f"{name}: independent_full_motion requires camera_motion")
    if mode != "independent_full_motion" and prior:
        raise ValueError(f"{name}: camera_motion is only valid for independent_full_motion")
    if mode == "known_stationary_whole_recording" and not entry.get("stationary_acquisition_verified"):
        raise ValueError(f"{name}: known_stationary_whole_recording requires stationary_acquisition_verified")
    if prior:
        prior = (manifest_dir / prior).resolve()
        if not prior.is_file():
            raise ValueError(f"{name}: camera-motion file not found")
    return {"camera_mode": mode, "starts_at_rest": entry.get("starts_at_rest"),
            "camera_motion": prior}


def source_fingerprint(recording, script, root, config, args):
    # Input metadata is cheap even for multi-gigabyte recordings. Changing
    # the code, configuration or recording invalidates the cached stage.
    sources = [script, root / "stitch-map" / "window_motion.py",
               root / "stitch-map" / "geometry.py",
               root / "stitch-map" / "heldout_diagnostics.py"]
    digest = hashlib.sha256()
    for source in sources:
        if source.is_file():
            digest.update(source.read_bytes())
    prior = config["camera_motion"]
    if prior:
        digest.update(prior.read_bytes())
    payload = {
        "recording_path": str(recording.resolve()),
        "size": recording.stat().st_size,
        "mtime_ns": recording.stat().st_mtime_ns,
        "script_sha256": digest.hexdigest(),
        "camera_mode": config["camera_mode"],
        "settings": {"stride": args.stride, "window": args.window,
                     "threshold_px": args.threshold_px, "max_frames": args.max_frames,
                     "blend": args.blend},
    }
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def stage_command(script, recording, stage, target, config, args):
    cmd = [sys.executable, str(script),
           "--bag" if recording.suffix == ".db3" else "--video", str(recording),
           "--out", str(target), "--blend", args.blend,
           "--max-frames", str(args.max_frames)]
    if stage == "baseline":
        cmd += ["--stride", str(args.stride)]
    else:
        cmd += ["--motion-filter", "--flow-window", str(args.window),
                "--motion-residual-px", str(args.threshold_px)]
        if config["camera_mode"] == "known_stationary_whole_recording":
            cmd.append("--camera-still")
        elif config["camera_mode"] == "independent_full_motion":
            cmd += ["--camera-motion", str(config["camera_motion"])]
    if args.debug_images:
        cmd += ["--debug-out", str(target.with_name(target.stem + "_debug.png"))]
    return cmd


def run_stage(command, output, state_path, signature, rerun=False):
    stats_path = output.with_suffix(".json")
    if not rerun and state_path.is_file() and output.is_file() and stats_path.is_file():
        try:
            previous = read_json(state_path)
            if (previous.get("fingerprint") == signature and
                    previous.get("status") == "passed"):
                return previous, read_json(stats_path), "cached"
        except (ValueError, OSError):
            pass
    result = {"status": "failed", "fingerprint": signature, "updated_at": stamp()}
    output.parent.mkdir(parents=True, exist_ok=True)
    log_path = output.with_suffix(".log")
    # A failed new run must never accidentally reuse last run's JSON.
    if stats_path.exists():
        stats_path.unlink()
    if output.exists():
        output.unlink()
    try:
        with log_path.open("w", encoding="utf-8") as logfile:
            completed = subprocess.run(command, stdout=logfile, stderr=subprocess.STDOUT,
                                       check=False)
        result["exit_code"] = completed.returncode
        if completed.returncode == 0 and output.is_file() and stats_path.is_file():
            stats = read_json(stats_path)
            result["status"] = "passed"
        else:
            stats = {}
            result["error"] = "command failed or output missing; see the local log"
    except (OSError, ValueError) as exc:
        stats = {}
        result["error"] = str(exc)
    result["log_name"] = log_path.name
    atomic_json(state_path, result)
    return result, stats, "executed"


def summarize(recording, config, cases):
    base, filt = cases["baseline"], cases["filtered"]
    bs, fs = base[1], filt[1]
    windows = fs.get("windows", [])
    def finite_error(value):
        return type(value) in (float, int) and math.isfinite(value) and value >= 0

    filtered_errors = [w["heldout_median_error_px"] for w in windows
                       if isinstance(w, dict) and finite_error(w.get("heldout_median_error_px"))]
    unfiltered_errors = [w["heldout_unfiltered_median_error_px"] for w in windows
                         if isinstance(w, dict) and finite_error(w.get("heldout_unfiltered_median_error_px"))]
    paired_changes = [w["heldout_median_error_px"] - w["heldout_unfiltered_median_error_px"]
                      for w in windows if isinstance(w, dict)
                      and finite_error(w.get("heldout_median_error_px"))
                      and finite_error(w.get("heldout_unfiltered_median_error_px"))]
    return {
        "recording": recording.name, "camera_mode": config["camera_mode"],
        "starts_at_rest": config["starts_at_rest"],
        "baseline_status": base[0]["status"], "filter_status": filt[0]["status"],
        "baseline_frames_used": bs.get("frames_used"), "filter_frames_used": fs.get("frames_used"),
        "baseline_pairs_skipped": bs.get("pairs_skipped"), "filter_pairs_skipped": fs.get("pairs_skipped"),
        "baseline_median_inliers": bs.get("median_inliers"),
        "filter_median_inliers": fs.get("median_inliers"),
        "filter_median_heldout_error_px": median(filtered_errors) if filtered_errors else None,
        "filter_windows": len(windows),
        "filter_median_unfiltered_heldout_error_px": median(unfiltered_errors) if unfiltered_errors else None,
        "filter_paired_heldout_windows": len(paired_changes),
        "filter_paired_median_error_change_px": median(paired_changes) if paired_changes else None,
    }


def run_batch(args):
    input_dir = args.input_dir.resolve()
    root = Path(__file__).resolve().parents[1]
    script = (args.stitch_script or (root / "stitch-map" / "stitch_map.py")).resolve()
    if not script.is_file():
        raise ValueError(f"stitch script not found: {script}")
    if not input_dir.is_dir():
        raise ValueError(f"input directory not found: {input_dir}")
    if args.window < 2 or args.stride < 1 or args.threshold_px <= 0 or args.max_frames < 0:
        raise ValueError("window >=2, stride >=1, threshold >0 and max-frames >=0 required")
    manifest = read_json(args.manifest) if args.manifest else {}
    manifest_dir = args.manifest.resolve().parent if args.manifest else input_dir
    recordings = sorted([*input_dir.glob("*.db3"), *input_dir.glob("*.mp4")])
    if args.names:
        requested = set(args.names)
        recordings = [p for p in recordings if p.name in requested]
        missing = requested - {p.name for p in recordings}
        if missing:
            raise ValueError(f"requested files are missing: {sorted(missing)}")
    if not recordings:
        raise ValueError("no .db3 or .mp4 files found")
    configs = {rec.name: config_for(rec.name, manifest, manifest_dir) for rec in recordings}
    report = []
    for recording in recordings:
        config = configs[recording.name]
        label = f"{recording.name}: {config['camera_mode']}"
        if args.dry_run:
            print(f"Would process {label}", flush=True)
            continue
        print(f"Processing {label}", flush=True)
        directory = args.output_dir.resolve() / recording.stem
        signature = source_fingerprint(recording, script, root, config, args)
        cases = {}
        for stage in ("baseline", "filtered"):
            image = directory / (stage + ".png")
            cmd = stage_command(script, recording, stage, image, config, args)
            result, stats, action = run_stage(cmd, image, directory / (stage + "_state.json"),
                                               signature + ":" + stage, args.rerun)
            cases[stage] = (result, stats)
            print(f"  {stage}: {result['status']} ({action})", flush=True)
        report.append(summarize(recording, config, cases))
        atomic_json(args.output_dir.resolve() / "summary.json",
                    {"generated_at": stamp(), "real_recordings_tested": True,
                     "pose_ground_truth_available": False, "camera_still_means_entire_recording": True,
                     "results": report})
        with (args.output_dir.resolve() / "summary.csv").open("w", newline="") as fp:
            writer = csv.DictWriter(fp, fieldnames=REPORT_FIELDS)
            writer.writeheader()
            writer.writerows(report)
    return 1 if any(r["baseline_status"] != "passed" or r["filter_status"] != "passed"
                    for r in report) else 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--manifest", type=Path,
                        help="per-recording camera evidence; never infer from gyro alone")
    parser.add_argument("--names", nargs="*", help="optional basenames to run")
    parser.add_argument("--stitch-script", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--stride", type=int, default=5)
    parser.add_argument("--window", type=int, default=25)
    parser.add_argument("--threshold-px", type=float, default=1.5)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--blend", choices=("average", "best"), default="best")
    parser.add_argument("--debug-images", action="store_true")
    parser.add_argument("--rerun", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)
    try:
        return run_batch(args)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    sys.exit(main())
