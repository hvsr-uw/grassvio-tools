#!/usr/bin/env python3
"""Recheck recorded motion and produce synchronized XYZ and drift figures.

Reuse measured visual tracks and SfM only when the newly measured RGB-D
poses agree with the previous visual reconstruction inputs. Raw data stays
local. Every subprocess writes a log and the runner checkpoints each stage.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def read(path):
    return json.loads(path.read_text())


def run(args):
    summary = []
    bags = sorted(args.input_dir.glob('*.db3'))
    if args.names:
        bags = [b for b in bags if b.name in args.names]
    if not bags:
        raise ValueError('no matching recordings')
    for bag in bags:
        old = args.recovery_dir/bag.stem
        sfm_dir = args.multiview_dir/bag.stem
        sfm = sfm_dir/'dense_sfm/reconstruction.json'
        if not sfm.exists():
            sfm = sfm_dir/'sfm/reconstruction.json'
        folder = args.output_dir/bag.stem
        folder.mkdir(parents=True, exist_ok=True)
        channels, timing = folder/'channels.json', folder/'gyro_timing.json'
        state = {'recording': bag.name, 'stages': {}}
        for name, module, params, expected in [
            ('channels', 'motion_channels', ['--bag', bag, '--atlas-dir', old/'atlas', '--outdir', folder], channels),
            ('timing', 'gyro_timing_refinement', ['--bag', bag, '--channels', channels, '--out', timing], timing),
            ('inertial_fit', 'constrained_inertial_initialization', ['--bag', bag, '--channels', channels,
                '--gyro-fit', timing, '--out', folder/'constrained_initialization.json'], folder/'constrained_initialization.json'),
            ('plots', 'motion_report', ['--bag', bag, '--channels', channels, '--gyro-fit', timing,
                '--sfm', sfm, '--fit', folder/'constrained_initialization.json', '--outdir', folder], folder/'motion_report.json'),
        ]:
            if name == 'plots':
                previous, current = read(old/'channels/channels.json'), read(channels)
                if previous['rgbd_pose_indices'] != current['rgbd_pose_indices']:
                    raise ValueError('visual frame indices changed; rerun multiview reconstruction')
                if previous['rgbd_component_ids'] != current['rgbd_component_ids'] or not np.allclose(
                    previous['rgbd_poses_to_component'], current['rgbd_poses_to_component'], atol=1e-8, rtol=0):
                    raise ValueError('visual poses changed; rerun multiview reconstruction')
                state['visual_sfm_reused_after_unchanged_rgbd_check'] = True
            command = [sys.executable, '-u', str(ROOT/'stitch-map'/(module+'.py')), *map(str, params)]
            print(f'{bag.name}: {name} starting', flush=True)
            with (folder/(name+'.log')).open('w', encoding='utf-8') as log:
                result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
            state['stages'][name] = {'exit_code': result.returncode, 'output_exists': expected.exists()}
            (folder/'pipeline_state.json').write_text(json.dumps(state, indent=2)+'\n')
            if result.returncode or not expected.exists():
                raise RuntimeError(f'{bag.name}: {name} failed; see {folder/(name+".log")}')
            print(f'{bag.name}: {name} completed', flush=True)
        result = read(folder/'motion_report.json')['summary']
        summary.append(result)
        args.output_dir.joinpath('summary.json').write_text(json.dumps({'recordings': summary}, indent=2)+'\n')
        print(json.dumps(result), flush=True)
    return summary


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input-dir', type=Path, required=True)
    p.add_argument('--recovery-dir', type=Path, required=True)
    p.add_argument('--multiview-dir', type=Path, required=True)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--names', nargs='*')
    run(p.parse_args())
