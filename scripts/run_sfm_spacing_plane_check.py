#!/usr/bin/env python3
"""Run near/far planar and SfM geometry checks for a folder of recordings."""
import argparse
import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "stitch-map"))
from sfm_spacing_plane_check import run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--multiview-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--names", nargs="*",
                        help="specific .db3 recording names; default is all")
    parser.add_argument("--gaps", type=int, nargs="+", default=[5, 25, 50])
    parser.add_argument("--anchor-step", type=int, default=50)
    parser.add_argument("--min-matches", type=int, default=40)
    args = parser.parse_args()
    if any(g < 1 for g in args.gaps) or args.anchor_step < 1:
        parser.error("frame gaps and anchor step must be positive")

    recordings = sorted(args.input_dir.glob("*.db3"))
    if args.names:
        missing = set(args.names) - {p.name for p in recordings}
        if missing:
            parser.error(f"missing recordings: {sorted(missing)}")
        recordings = [p for p in recordings if p.name in args.names]
    if not recordings:
        parser.error("no recordings found")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    results = []
    failures = []
    for bag in recordings:
        sfm_folder = args.multiview_dir / bag.stem
        dense = sfm_folder / "dense_sfm" / "reconstruction.json"
        ordinary = sfm_folder / "sfm" / "reconstruction.json"
        sfm_path = dense if dense.exists() else ordinary if ordinary.exists() else None
        output = args.output_dir / bag.stem / "spacing_plane.json"
        try:
            result = run(bag, output, sorted(set(args.gaps)), args.anchor_step,
                         sfm_path, args.min_matches)
            diagnostics = (json.loads(sfm_path.read_text()).get("diagnostics", {})
                           if sfm_path else {})
            for gap, summary in result["summary"].items():
                results.append({
                    "recording": bag.name, "gap_frames": int(gap),
                    "sfm_reconstruction": sfm_path.name if sfm_path else "not_available",
                    "multiview_reserved_track_median_px": diagnostics.get("validation_median_px"),
                    **summary,
                })
        except (OSError, ValueError, KeyError) as exc:
            failures.append({"recording": bag.name, "reason": str(exc)})
            print(f"{bag.name}: failed: {exc}", file=sys.stderr, flush=True)

    if results:
        fields = ["recording", "gap_frames", "sfm_reconstruction",
                  "multiview_reserved_track_median_px", "attempted_pairs",
                  "evaluated_pairs", "parallax_eligible_pairs",
                  "strong_plane_pairs", "median_plane_holdout_px",
                  "median_parallax_deg", "sfm_pose_pairs"]
        with (args.output_dir / "summary.csv").open("w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=fields, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(results)
    (args.output_dir / "summary.json").write_text(
        json.dumps({"rows": results, "failed": failures,
                    "metric_ground_truth_available": False},
                   indent=2, allow_nan=False) + "\n")
    lines = [
        "# SfM frame-spacing and plane check",
        "",
        "I compared calibrated SIFT matches on near and farther frames, "
        "fitted homographies using training image tiles, and evaluated them "
        "on held-out tiles. I checked whether the recovered two-view motion "
        "has enough parallax to triangulate 3D structure.",
        "",
        "The plane error is a 2D transfer error. Where reconstructed SfM "
        "poses exist at both frames, the SfM value is a 1D epipolar Sampson "
        "residual and must not be directly ranked against the plane error. "
        "The SfM validation error comes from the existing full-video "
        "reconstruction; it is not independent physical ground truth.",
        "",
        "| Video | Gap (frames) | Evaluated / attempted | Adequate parallax "
        "| Strong plane | Median plane holdout (px) | Median parallax (deg) |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in results:
        def fmt(value):
            return "-" if value is None else (
                f"{value:.3f}" if isinstance(value, float) else str(value))
        lines.append(
            f"| {row['recording']} | {row['gap_frames']} | "
            f"{row['evaluated_pairs']}/{row['attempted_pairs']} | "
            f"{row['parallax_eligible_pairs']} | {row['strong_plane_pairs']} | "
            f"{fmt(row['median_plane_holdout_px'])} | "
            f"{fmt(row['median_parallax_deg'])} |")
    lines.extend([
        "", f"Videos with failed runs: {len(failures)}.",
        "",
        "A longer frame gap can improve triangulation only if sufficient "
        "scene overlap remains and the recovered motion contains translation. "
        "A pure rotation may be many degrees but still provides no baseline. "
        "A low plane error supports a local plane model; it does not prove "
        "that vegetation is rigid or that recovered camera motion is accurate.",
        "",
    ])
    (args.output_dir / "report.md").write_text("\n".join(lines))
    print(json.dumps({"recordings": len(recordings), "table_rows": len(results),
                      "failed": failures}, indent=2), flush=True)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
