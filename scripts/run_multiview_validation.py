#!/usr/bin/env python3
"""Run timing refinement, initialization, multiview SfM, comparison and mapping.

Each measurement module caches against its source and input dependencies.
Failed commands and incomplete scientific checks remain separate.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys

from run_recording_batch import atomic_json


ROOT = Path(__file__).resolve().parents[1]


def run(args):
    inputs = sorted(args.input_dir.glob('*.db3'))
    if args.names:
        missing = set(args.names)-{p.name for p in inputs}
        if missing:
            raise ValueError(f'recordings missing: {sorted(missing)}')
        inputs = [p for p in inputs if p.name in args.names]
    if not inputs:
        raise ValueError('no recordings found')
    summary = []
    for bag in inputs:
        folder = args.output_dir/bag.stem
        folder.mkdir(parents=True, exist_ok=True)
        original = args.recovery_dir/bag.stem
        channels = original/'channels/channels.json'
        timing = folder/'gyro_timing.json'
        initialization = folder/'constrained_initialization.json'
        sfm = folder/'sfm/reconstruction.json'
        comparison = folder/'channel_comparison.json'
        stages = [
            ('timing', 'gyro_timing_refinement.py', ['--bag', bag, '--channels', channels, '--out', timing], timing),
            ('inertial', 'constrained_inertial_initialization.py', ['--bag', bag, '--channels', channels,
                '--gyro-fit', timing, '--out', initialization], initialization),
            ('sfm', 'multiview_sfm.py', ['--bag', bag, '--atlas', original/'atlas/atlas.json',
                '--channels', channels, '--outdir', folder/'sfm'], sfm),
            ('comparison', 'multiview_channel_check.py', ['--bag', bag, '--sfm', sfm, '--channels', channels,
                '--gyro-fit', timing, '--out', comparison], comparison),
            ('sfm_map', 'sfm_registered_map.py', ['--bag', bag, '--sfm', sfm,
                '--outdir', folder/'sfm_map'], folder/'sfm_map/registered_map.json')]
        state = {'stages': {}}
        failed = False

        def execute(name, module, parameters, expected):
            before = expected.stat().st_mtime_ns if expected.exists() else None
            command = [sys.executable, '-u', str(ROOT/'stitch-map'/module), *map(str, parameters)]
            with (folder/(name+'.log')).open('a', encoding='utf-8') as log:
                code = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=False).returncode
            if code == 0 and not expected.exists():
                code = 1
            reused = code == 0 and expected.stat().st_mtime_ns == before
            state['stages'][name] = {'exit_code': code, 'cached': reused}
            atomic_json(folder/'pipeline_state.json', state)
            print(f"{bag.name}: {name} exit={code} ({'cached' if reused else 'executed'})", flush=True)
            return code

        for name, module, parameters, expected in stages:
            if execute(name, module, parameters, expected):
                failed = True
                break
            if name == 'sfm' and args.adaptive_gaps:
                coarse = json.loads(sfm.read_text())
                if coarse['unregistered_frames']:
                    dense = folder/'dense_sfm/reconstruction.json'
                    if execute('sfm_dense', 'adaptive_sfm.py', ['--bag', bag, '--coarse', sfm,
                               '--outdir', folder/'dense_sfm'], dense):
                        failed = True
                        break
                    for _, _, later_parameters, _ in stages:
                        for index, value in enumerate(later_parameters):
                            if value == sfm:
                                later_parameters[index] = dense
                    sfm = dense
        result = {'recording': bag.name, 'commands_completed': not failed}
        if not failed:
            s, g, i, c = [json.loads(path.read_text()) for path in (sfm, timing, initialization, comparison)]
            mapping = json.loads((folder/'sfm_map/registered_map.json').read_text())
            result.update(sfm_cameras=len(s['registered_frames']), sfm_requested=len(s['selected_frames']),
                original_cameras_registered=sum(frame in s['registered_frames']
                    for frame in s.get('original_selected_frames', s['selected_frames'])),
                original_cameras_requested=len(s.get('original_selected_frames', s['selected_frames'])),
                sfm_validation_median_px=s['diagnostics'].get('validation_median_px'),
                sfm_validation_p90_px=s['diagnostics'].get('validation_p90_px'),
                metric_scale_usable=s['metric_scale'].get('usable', False),
                sfm_map_frames=len(mapping['frames_used']),
                clock_shift_ms=g['selected_hypothesis']['shift_ms'],
                gyro_vs_rgbd_median_deg=g['heldout_calibrated_median_deg'],
                gyro_vs_sfm_median_deg=c['summary']['gyro_vs_sfm_rotation_deg']['median'],
                inertial_initialization_usable=i['initialization_usable'],
                three_channel_agreement_established=False, metric_ground_truth_available=False)
        summary.append(result)
        atomic_json(args.output_dir/'summary.json', {'recordings': summary})
        print(json.dumps(result), flush=True)
    return 1 if any(not row['commands_completed'] for row in summary) else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-dir', type=Path, required=True)
    parser.add_argument('--recovery-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--names', nargs='*')
    parser.add_argument('--adaptive-gaps', action='store_true', help='retry missing SfM views with closer keyframes and five-frame feature support')
    args = parser.parse_args()
    try:
        return run(args)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == '__main__':
    sys.exit(main())
