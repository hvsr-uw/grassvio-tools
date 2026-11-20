import numpy as np

from gyro_calibration import integrate_rotation, samples_between
from gyro_timing_refinement import search_timing


def test_wider_prefix_search_recovers_shift_outside_old_range():
    times = np.arange(1001, dtype=np.int64)*2_000_000
    t = times/1e9
    clean = np.column_stack((.3*np.sin(7*t), .2*np.cos(5*t), .15*np.sin(9*t)))
    bias = np.array([.003, -.005, .002])
    shift_ms = -75.
    training = []
    for start, end in [(300_000_000, 600_000_000), (650_000_000, 950_000_000), (1_000_000_000, 1_400_000_000)]:
        reference = integrate_rotation(samples_between(times, clean, start+shift_ms*1e6, end+shift_ms*1e6), np.zeros(3), np.eye(3))
        pose = np.eye(4)
        pose[:3, :3] = reference
        training.append({'start': start, 'end': end, 'T_rgbd_source_to_target': pose})

    def sample(row, shift):
        return samples_between(times, clean+bias, row['start']+shift*1e6, row['end']+shift*1e6)

    best, _ = search_timing(training, sample, np.eye(3))
    assert best['shift_ms'] == shift_ms
    np.testing.assert_allclose(best['bias'], bias, atol=1e-6)
