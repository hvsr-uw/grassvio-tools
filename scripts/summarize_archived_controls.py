#!/usr/bin/env python3
"""Summarize archived control measurements without publishing motion traces."""
import argparse
import csv
import json
from pathlib import Path


def run(controls_dir, pairs_dir, outdir):
    reports=[json.loads(p.read_text()) for p in sorted(controls_dir.glob('*/controls.json'))]
    if len(reports)!=4:
        raise ValueError('expected four archived control reports')
    pair_report=json.loads((pairs_dir/'20260926_155419/controls.json').read_text())
    rows=[r['summary'] for r in reports]
    pair=pair_report['summary']
    outdir.mkdir(parents=True,exist_ok=True)
    with (outdir/'summary.csv').open('w',newline='') as handle:
        writer=csv.DictWriter(handle,fieldnames=list(rows[0]))
        writer.writeheader();writer.writerows(rows)
    aggregate={'window_frames':25,'recordings':rows,'september_adjacent_pairs':pair,
        'input_files':[r['source'] for r in reports],
        'independent_metric_ground_truth_available':False,
        'original_fig6_plotting_provenance_established':False}
    (outdir/'summary.json').write_text(json.dumps(aggregate,indent=2,allow_nan=False)+'\n')
    def fmt(value,digits=3):
        return f'{value:.{digits}f}' if value is not None else 'unavailable'
    lines=['# Archived recording controls','',
        'I measured the September 26 video and three October controls using the recorded calibration. I retained every requested interval and marked unavailable synchronized depth explicitly. I did not assign zero camera motion or a physical wind label to any video.','',
        '## Pairing and calibration','',
        '| Recording | Duration | RGB-D windows / requested | First colour/depth offset | Quiet-window mean gyro | Low-motion candidate |',
        '|---|---:|---:|---:|---:|---|']
    for r in rows:
        lines.append(f"| {r['recording']} | {r['duration_s']:.3f} s | {r['rgbd_windows']} / {r['windows']} | {r['first_depth_offset_ms']:.3f} ms | {r['quiet_window_mean_gyro_rad_s']:.3f} rad/s | {'yes' if r['quiet_window_is_low_motion_candidate'] else 'no'} |")
    lines+=['',
        'The depth pairing limit is 20 ms. The first colour frames of 155419 and 080033 exceed it. I keep those RGB-D intervals unavailable and continue the later comparisons. I do not increase the pairing limit or replace missing measurements with an identity transform. The quiet-window test checks a 0.5 s interval, using mean gyro magnitude below 0.03 rad/s and accelerometer-magnitude MAD below 0.05 m/s^2. Passing these thresholds is a heuristic and does not prove stationary camera velocity.','',
        '## Filtering on separate image points','',
        'I use 25-frame windows, including the shorter final window, and a 1.5 px temporal residual threshold. Each filter fits only checkerboard training tiles and scores the same held-out tiles. The colour cue uses its existing 0.35 likelihood threshold and a stricter 1.125 px residual threshold. The scores below are medians of available per-window median pixel errors. They measure image consistency across all held-out tracks, including moving objects; they are not wind-detection accuracy.','',
        '| Recording | Unfiltered error | Temporal error | Temporal + colour error | Colour candidates | Additional colour rejections |',
        '|---|---:|---:|---:|---:|---:|']
    for r in rows:
        lines.append(f"| {r['recording']} | {fmt(r['unfiltered_heldout_median_px'])} px | {fmt(r['temporal_heldout_median_px'])} px | {fmt(r['colour_heldout_median_px'])} px | {r['vegetation_candidate_tracks']} | {r['semantic_extra_rejections']} |")
    rejections=sum(r['semantic_extra_rejections'] for r in rows)
    lines+=['',f'The colour cue makes {rejections} additional rejections across these {sum(r["windows"] for r in rows)} windows. These measurements do not establish a colour-cue improvement. A visually fitted plane can follow a coherently moving majority, so these scores also do not prove that the filter separated camera motion from grass motion.','',
        '## Same-source September frame comparisons','',
        f"I also measured every adjacent frame pair in 155419. The recording contains {pair['frames']} colour frames over {pair['duration_s']:.3f} s, giving {pair['windows']} adjacent intervals. {pair['rgbd_windows']} RGB-D intervals are available; the first interval fails depth pairing. The valid source indices are 1 through 53 and the targets are 2 through 54. The matching count of 53 does not establish that these were the exact endpoints used in the original Fig. 6.", '',
        f"The median RGB-D translation magnitude is {pair['rgbd_translation_median_m']*1000:.3f} mm per adjacent interval, and its median rotation magnitude is {pair['rgbd_rotation_median_deg']:.3f} degrees. With raw gyro and zero clock shift, the median relative rotation disagreement against RGB-D is {pair['gyro_vs_rgbd_median_deg']:.3f} degrees. These are model comparisons, not ground-truth errors.", '',
        f"The quietest 0.5 s window still has a mean gyro magnitude of {pair['quiet_window_mean_gyro_rad_s']:.3f} rad/s, so it fails the low-motion gate. I do not subtract the video-wide mean gyro as if it were sensor bias. The raw-IMU translation diagnostic carries velocity across intervals but starts at assumed zero velocity and uses an offline gravity heuristic. It is not a validated camera trajectory.", '',
        'The raw September recording is now available for analysis. The original report and exact plotting implementation are still needed to identify the quantity behind its Fig. 6 curves. These new measurements do not settle that provenance.','',
        '## Short-recording initialization','',
        'The October 30 video does not supply enough disjoint prefix and later rotation windows for the existing gyro fit. The linear inertial initializer now returns `unavailable_gyro_calibration` and `initialization_usable=false` instead of raising a missing-key error. This reports an unavailable estimate; it does not make the initialization pass.','',
        '## Reproduce','',
        '```bash',
        'python scripts/inspect_archived_controls.py --input-dir recordings --output-dir local-results/archived-controls --names 20260926_155419.db3 20261007_080033.db3 20261026_173701.db3 20261030_174745.db3',
        'python scripts/inspect_archived_controls.py --input-dir recordings --output-dir local-results/fig6-frame-pairs --window 1 --names 20260926_155419.db3',
        'python scripts/summarize_archived_controls.py --controls-dir local-results/archived-controls --pairs-dir local-results/fig6-frame-pairs --outdir analysis/archived-controls',
        'python -m pytest -q tests',
        '```','',
        'Each local `controls.json` contains the input SHA-256, full interval list, depth offsets, relative transforms, filter counts and scoring definitions. The repository summary contains aggregates and input hashes. Recordings and detailed motion traces remain outside the repository.','',
        'Independent motion accuracy and physical wind validation remain unresolved. I still need independently measured camera motion and labelled stationary/moving vegetation controls to make those claims.','']
    (outdir/'report.md').write_text('\n'.join(lines),encoding='utf-8')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('controls-dir','pairs-dir','outdir'):
        p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();run(a.controls_dir,a.pairs_dir,a.outdir)
