import importlib.util
import json
from pathlib import Path
import textwrap
from types import SimpleNamespace
import pytest

PATH = Path(__file__).resolve().parents[1] / 'scripts' / 'run_recording_batch.py'
spec = importlib.util.spec_from_file_location('run_recording_batch', PATH)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def test_stationary_must_be_confirmed_for_entire_recording(tmp_path):
    with pytest.raises(ValueError, match='stationary_acquisition_verified'):
        mod.config_for('x.db3', {'recordings': {'x.db3': {
            'camera_mode': 'known_stationary_whole_recording', 'starts_at_rest': True}}}, tmp_path)


def test_moving_mode_requires_independent_prior(tmp_path):
    with pytest.raises(ValueError, match='requires camera_motion'):
        mod.config_for('x.db3', {'recordings': {'x.db3': {
            'camera_mode': 'independent_full_motion'}}}, tmp_path)


def test_unchanged_stages_are_skipped_but_configuration_changes_rerun(tmp_path):
    folder = tmp_path / 'recordings'
    folder.mkdir()
    (folder / 'a.db3').write_bytes(b'placeholder')
    fake = tmp_path / 'fake_stitch.py'
    fake.write_text(textwrap.dedent('''
        import argparse, json
        from pathlib import Path
        p=argparse.ArgumentParser()
        p.add_argument('--bag'); p.add_argument('--out'); p.add_argument('--blend')
        p.add_argument('--max-frames'); p.add_argument('--stride')
        p.add_argument('--flow-window'); p.add_argument('--motion-residual-px')
        p.add_argument('--motion-filter',action='store_true')
        p.add_argument('--camera-still',action='store_true')
        p.add_argument('--camera-motion')
        p.add_argument('--debug-out')
        a=p.parse_args()
        Path(a.out).write_bytes(b'png')
        Path(a.out).with_suffix('.json').write_text(json.dumps({'frames_used': 2, 'pairs_skipped': 0,
          'median_inliers': 40, 'windows': [{'heldout_median_error_px': 1.2}]}))
    '''))
    args = SimpleNamespace(input_dir=folder, output_dir=tmp_path / 'results', manifest=None,
        names=None, stitch_script=fake, stride=5, window=25, threshold_px=1.5, max_frames=0,
        blend='best', debug_images=False, rerun=False, dry_run=False)
    assert mod.run_batch(args) == 0
    log = args.output_dir / 'a' / 'baseline.log'
    first = log.stat().st_mtime_ns
    assert mod.run_batch(args) == 0
    assert log.stat().st_mtime_ns == first
    args.threshold_px = 2.0
    assert mod.run_batch(args) == 0
    assert log.stat().st_mtime_ns >= first
    report = json.loads((args.output_dir / 'summary.json').read_text())
    assert report['pose_ground_truth_available'] is False
    assert report['results'][0]['filter_median_heldout_error_px'] == 1.2


def test_missing_names_fail_without_partial_run(tmp_path):
    d=tmp_path/'recordings'; d.mkdir(); (d/'a.db3').write_bytes(b'a')
    args = SimpleNamespace(input_dir=d, output_dir=tmp_path/'out', manifest=None,
        names=['missing.db3'], stitch_script=PATH, stride=5, window=25,
        threshold_px=1.5, max_frames=0, blend='best', debug_images=False,
        rerun=False, dry_run=False)
    with pytest.raises(ValueError, match='missing'):
        mod.run_batch(args)
    assert not args.output_dir.exists()


def test_heldout_summary_uses_medians_and_only_paired_finite_changes():
    windows = [
        {'heldout_median_error_px': 1., 'heldout_unfiltered_median_error_px': 3.},
        {'heldout_median_error_px': 2., 'heldout_unfiltered_median_error_px': 3.},
        {'heldout_median_error_px': 100., 'heldout_unfiltered_median_error_px': 10.},
        {'heldout_median_error_px': None, 'heldout_unfiltered_median_error_px': 4.},
        {'heldout_median_error_px': float('nan')},
        {'heldout_median_error_px': float('inf')},
        {'heldout_median_error_px': True},
    ]
    state = {'status': 'passed'}
    result = mod.summarize(Path('x.db3'),
        {'camera_mode': 'visual_plane_only', 'starts_at_rest': None},
        {'baseline': (state, {}), 'filtered': (state, {'windows': windows})})
    assert result['filter_median_heldout_error_px'] == 2.
    assert result['filter_median_unfiltered_heldout_error_px'] == 3.5
    assert result['filter_paired_heldout_windows'] == 3
    assert result['filter_paired_median_error_change_px'] == -1.


def test_rejected_render_keeps_diagnostics_and_can_resume(tmp_path):
    import sys
    fake = tmp_path / 'reject.py'
    fake.write_text(textwrap.dedent('''
        import json, sys
        from pathlib import Path
        Path(sys.argv[1]).write_text(json.dumps({'render_status':'rejected',
            'render_error':'canvas exceeds render budget','frames_total':901,
            'frames_used':181,'pairs_skipped':0}))
        sys.exit(1)
    '''))
    output = tmp_path / 'map.png'
    state = tmp_path / 'state.json'
    command = [sys.executable, str(fake), str(output.with_suffix('.json'))]
    first, stats, action = mod.run_stage(command, output, state, 'same-code')
    assert first['status'] == 'failed'
    assert first['diagnostics_available'] is True
    assert stats['frames_used'] == 181
    assert action == 'executed'
    second, cached, action = mod.run_stage(command, output, state, 'same-code')
    assert action == 'cached'
    assert cached == stats
    assert second['status'] == 'failed'
    assert not output.exists()


def test_partial_map_is_not_reported_as_complete_placement():
    state = {'status': 'passed'}
    stats = {'frames_total': 101, 'frames_selected': 5, 'frames_used': 2,
        'pairs_skipped': 3, 'selected_frames': [0,25,50,75,100],
        'transforms_to_first': [[[1,0,0],[0,1,0],[0,0,1]],
                                [[1,0,2],[0,1,0],[0,0,1]],None,None,None]}
    result = mod.summarize(Path('x.db3'),
        {'camera_mode': 'visual_plane_only', 'starts_at_rest': None},
        {'baseline': (state, stats), 'filtered': (state, stats)})
    assert result['filter_status'] == 'passed'
    assert result['filter_complete_placement'] is False
    assert result['filter_last_placed_frame'] == 25
