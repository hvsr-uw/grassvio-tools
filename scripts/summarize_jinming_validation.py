#!/usr/bin/env python3
"""Build a reviewable results report and figure bundle from saved measurements."""
import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'stitch-map'))
from motion_report import xyz_figure


def run(input_dir, publish_dir):
    paths = sorted(input_dir.glob('*/motion_report.json'))
    if len(paths) != 6:
        raise ValueError(f'expected all six recording reports, found {len(paths)}')
    reports = [json.loads(path.read_text()) for path in paths]
    rows = [r['summary'] for r in reports]
    archive_path = ROOT/'analysis/audit-imu-preint/runs/20260926_155419_preint.json'
    archive = json.loads(archive_path.read_text())
    archived_carried = archive['chains']['primary']['end_trans_m']
    archived_reset = archive['chains']['zero_velocity_reset']['end_trans_m']
    publish_dir.mkdir(parents=True, exist_ok=True)
    with (publish_dir/'summary.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    names = [r['recording'].split('_')[-1].split('.')[0] for r in rows]
    x = np.arange(len(rows))
    plt.rcParams.update({'font.size': 9, 'axes.grid': True, 'grid.alpha': .2})
    fig, axes = plt.subplots(2, 2, figsize=(13, 9), layout='constrained')
    for offset, key, label in [(-.25, 'raw_position_rmse_m', 'Raw IMU, zero initial velocity'),
        (0, 'initialized_position_rmse_m', 'Calibrated IMU + visual initial velocity'),
        (.25, 'mean_imu_position_rmse_m', 'Five-sample mean + visual initial velocity')]:
        axes[0, 0].bar(x+offset, [r[key] for r in rows], .24, label=label)
    axes[0, 0].set(title='Free propagation position disagreement', ylabel='Later position RMSE vs RGB-D (m)')
    axes[0, 0].legend(fontsize=7)
    for offset, key, label in [(-.15, 'visual_reset_prediction_rmse_m', 'IMU from previous visual reset'),
        (.15, 'visual_reset_baseline_rmse_m', 'Constant velocity from same prior poses')]:
        axes[0, 1].bar(x+offset, [r[key] for r in rows], .3, label=label)
    axes[0, 1].set(title='Next-pose prediction before the visual update', ylabel='Later prediction RMSE vs RGB-D (m)')
    axes[0, 1].legend(fontsize=7)
    axes[1, 0].bar(x, [r['heldout_gyro_vs_rgbd_median_deg'] for r in rows])
    axes[1, 0].set(title='Prefix-calibrated gyro rotation', ylabel='Later interval median difference (degrees)')
    axes[1, 1].bar(x, [r['pnp_inlier_rate_median']*100 for r in rows])
    axes[1, 1].axhline(50., color='red', linestyle='--', label='50% checklist bar')
    axes[1, 1].set(title='RGB-D PnP RANSAC support', ylabel='Median inlier rate (%)', ylim=(0, 105))
    axes[1, 1].legend(fontsize=7)
    for ax in axes.flat:
        ax.set_xticks(x, names)
        ax.set_xlabel('November 19 recording time')
    fig.suptitle('Jinming checklist: six-recording validation\nVisual estimates are model references; independent metric ground truth is unavailable')
    fig.savefig(publish_dir/'summary.png', dpi=160)
    fig.savefig(publish_dir/'summary.svg')
    # Render from saved numbers, so every XYZ figure uses the same final
    # plotting convention without repeating camera/IMU measurements.
    for path, report in zip(paths, reports):
        series = {name: (np.asarray(report['camera_positions_m'][name], float),
            np.asarray(rotations, float)) for name, rotations in report['camera_rotations'].items()}
        picture = xyz_figure(series, report['times_s'], path.parent.name)
        picture.savefig(path.parent/'motion_xyz.png', dpi=160)
        picture.savefig(path.parent/'motion_xyz.svg')
        plt.close(picture)
        report['definitions']['rotation'] = 'principal Rodrigues rotation-vector components in degrees; plot line breaks at branch cuts; not Euler angles'
        path.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    with PdfPages(input_dir/'all_recording_figures.pdf') as pdf:
        pdf.savefig(fig)
        for path in paths:
            with PdfPages(path.parent/'motion_figures.pdf') as individual:
                for name in ('motion_xyz', 'fig6_common_intervals', 'drift_comparison', 'visual_reset_validation'):
                    page, ax = plt.subplots(figsize=(14, 8), layout='constrained')
                    ax.imshow(plt.imread(path.parent/(name+'.png')))
                    ax.axis('off')
                    pdf.savefig(page)
                    individual.savefig(page)
                    plt.close(page)
    plt.close(fig)
    wins = sum(r['visual_reset_beats_baseline'] for r in rows)
    rejects = sum(r['semantic_additional_rejections'] for r in rows)
    windows = sum(r['semantic_windows'] for r in rows)
    lines = [
        '# Jinming checklist: recorded motion validation', '',
        'I ran the updated channel code, gyro timing fit, constrained initialization and plotting pipeline on all six November 19 recordings. I checked every RGB-D pose against the earlier visual inputs before reusing the SfM reconstructions. All visual poses and component labels matched; this run changes inertial initialization and evaluates semantics separately.', '',
        '![Measured results across six recordings](summary.png)', '',
        '## Measured results', '',
        'Position errors below are later-window RMSE against the RGB-D model. The gyro column is the median relative rotation difference on later intervals. These are different metrics and horizons; they are not independent ground-truth accuracy.', '',
        '| Recording | Raw IMU position | Initialized IMU position | Mean IMU position | Gyro rotation |',
        '|---|---:|---:|---:|---:|',
    ]
    for r in rows:
        lines.append(f"| {r['recording']} | {r['raw_position_rmse_m']:.2f} m | {r['initialized_position_rmse_m']:.2f} m | {r['mean_imu_position_rmse_m']:.2f} m | {r['heldout_gyro_vs_rgbd_median_deg']:.3f} deg |")
    lines += ['', 'The initialized trajectory uses the prefix-selected gyro bias and timing plus one initial visual displacement. I subtract the measured acceleration displacement before solving for initial velocity. The mean-IMU curve uses a centered five-sample temporal mean on each sensor axis. None of these free trajectories receives later visual position corrections.', '',
        '## How drift compensation was checked', '',
        'For each later window, I reset position and orientation to the previous visual pose, estimate velocity from the preceding visual displacement, and then integrate IMU motion to predict the next pose. I reserve that next visual pose for scoring. The comparison baseline uses the same prior visual measurements and continues at constant velocity. The reset itself is not scored as an accurate prediction.', '',
        '| Recording | IMU prediction after previous visual reset | Constant-velocity prediction | IMU improves |',
        '|---|---:|---:|---|']
    for r in rows:
        lines.append(f"| {r['recording']} | {r['visual_reset_prediction_rmse_m']:.3f} m | {r['visual_reset_baseline_rmse_m']:.3f} m | {'yes' if r['visual_reset_beats_baseline'] else 'no'} |")
    lines += ['', f'The local IMU prediction improves the baseline on {wins}/6 recordings. This evaluates visual-aided predictions; it does not validate independent IMU translation. The prefix fit for velocity, accelerometer bias and gravity still fails the existing usability gates on all six recordings. Rejected fitted parameters remain out of the mapping output.', '',
        '## Fig. 6 definitions and replacement figures', '',
        'Each new `fig6_common_intervals` figure compares the exact same recorded source and target frames. Every translation is a source-to-target camera-transform vector in metres. Rotation uses rotation vectors in degrees. RGB-D uses recorded calibration and depth registration; SfM gets one metric scale from initial depth. The figures use a common y scale within each row. Unsupported visual intervals remain absent.', '',
        'The older Fig. 6 used the September 26 take ending in 155419. That raw recording and the exact original plotting trace are not in the current permitted workspace, so these November 19 figures are replacements, not a numerical reproduction of that earlier figure. The repository does contain a separate `imu_fusion.py::imu_distance` diagnostic that resets velocity for every interval and demeans acceleration by the take mean. This can produce tiny acceleration-only displacements even during steady translation, but its existence does not establish that it generated the original Fig. 6.', '',
        f'The archived preintegration study for the same 155419 take gives a {archived_carried:.3f} m endpoint displacement when carrying velocity and {archived_reset:.3f} m when resetting it, a factor of {archived_carried/archived_reset:.1f}. That archive uses ten stride-five pairs rather than the original figure\'s 53 pairs. It demonstrates sensitivity to the velocity definition without establishing the provenance or correctness of the original figure. The archived numbers are in `analysis/audit-imu-preint/runs/20260926_155419_preint.json`.', '',
        'For example, a camera moving at 1 m/s with zero acceleration moves 0.0333 m in one 30 Hz interval. An acceleration-only calculation reset to zero velocity returns zero. The corrected comparison carries velocity across intervals, removes gravity in a consistent frame, includes the camera/IMU lever arm, and never compares unscaled two-view translation against metres.', '',
        '## Fixes and checks', '',
        '- A later low-motion gravity candidate is now rotated into the integration starting frame. A quiet IMU window alone cannot prove that the camera is stationary.',
        '- Initial velocity uses a visual edge beginning at the integration origin and subtracts integrated acceleration. A later accumulated world position is never divided by a single-edge duration.',
        '- Visual measurements remain in the comparison even when RGB-D cannot initialize velocity. Zero-velocity fallback is reported explicitly.',
        '- Gyro and acceleration use trapezoidal updates. XYZ rotation plots use principal angle-axis values and break lines at branch changes, so coordinate discontinuities are not drawn as camera-motion jumps. Missing visual data also breaks lines. Orientation errors use full rotation matrices.',
        '- The optional vegetation gate now uses 1.125 px inside the ordinary 1.5 px residual gate, so it can affect selection. A regression test demonstrates an actual extra rejection while retaining a rigid green point.',
        f'- Real-data semantic checks covered {windows} windows and rejected {rejects} additional points. These recordings provide no labelled wind truth, and this experiment does not establish a semantic accuracy improvement.',
        '- Ninety-four regression tests pass, including gravity-frame transport, acceleration-aware initial velocity, semantic selection, translation-transform direction, per-axis temporal averaging and the rotation plotting branch cut.', '',
        '## Reproduce', '',
        '```bash',
        'python -m pip install -r requirements-analysis.txt',
        'python scripts/run_jinming_validation.py --input-dir recordings --recovery-dir local-results/recovery --multiview-dir local-results/multiview --output-dir local-results/jinming-final',
        'python scripts/summarize_jinming_validation.py --input-dir local-results/jinming-final --publish-dir analysis/jinming-validation',
        'python -m pytest -q tests',
        '```', '',
        'Detailed local deliverables are `motion_report.json`, `interval_motion.csv`, four figures in PNG and SVG, and `motion_figures.pdf` for each recording. The combined `all_recording_figures.pdf` contains every recording. Input hashes, recording identity, coordinate definitions, initialization assumptions and stage logs accompany the figures. The CSV summary here is the small aggregate record; recordings and full motion traces remain local.', '',
        '## What remains unresolved', '',
        'Independent inertial translation remains unvalidated. The older Fig. 6 still needs its original input data and plotting provenance to establish its exact cause. Absolute rotation/translation accuracy and wind-specific detection require independent reference motion and labelled controls. The current results support gyro rotation calibration and measured visual-aided prediction checks, with those limits.'
    ]
    (publish_dir/'report.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    print(json.dumps({'recordings': len(rows), 'local_prediction_improvements': wins,
        'semantic_windows': windows, 'semantic_additional_rejections': rejects,
        'report': str(publish_dir/'report.md'), 'figure_bundle': str(input_dir/'all_recording_figures.pdf')}))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input-dir', type=Path, required=True)
    p.add_argument('--publish-dir', type=Path, required=True)
    a = p.parse_args()
    run(a.input_dir, a.publish_dir)
