import importlib.util
from pathlib import Path
import sqlite3
import struct

import numpy as np

PATH = Path(__file__).resolve().parents[1] / 'scripts' / 'inspect_recordings.py'
spec = importlib.util.spec_from_file_location('inspect_recordings', PATH)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def test_nearest_timestamp_pairing_uses_both_neighbours():
    np.testing.assert_array_equal(mod.nearest_offsets_ns([9, 18, 29], [0, 10, 20]), [1, 2, 9])


def test_quiet_startup_does_not_become_verified_rest(tmp_path):
    path = tmp_path / 'quiet recording.db3'
    with sqlite3.connect(path) as db:
        db.execute('CREATE TABLE topics (id INTEGER,name TEXT,type TEXT)')
        db.execute('CREATE TABLE messages (topic_id INTEGER,timestamp INTEGER,data BLOB)')
        for tid, name in enumerate(mod.STREAMS.values(), 1):
            db.execute('INSERT INTO topics VALUES (?,?,?)', (tid, name, 'sensor_msgs/msg/Imu'))
            for stamp in [0, 5_000_000, 10_000_000]:
                data = bytearray(324)
                struct.pack_into('<iI', data, 4, 0, stamp)
                struct.pack_into('<I', data, 12, 1)
                values = [0.] * 37
                values[3] = 1.
                values[26] = -9.8
                struct.pack_into('<37d', data, 28, *values)
                db.execute('INSERT INTO messages VALUES (?,?,?)', (tid, stamp, data))
    result = mod.inspect_recording(path)
    assert result['integrity'] == ['ok']
    assert result['all_required_streams_present'] is True
    assert result['streams']['gyro']['first_second_median_norm'] == 0.
    assert result['starts_at_rest_independently_verified'] is False
    assert result['three_channel_agreement_tested'] is False
    assert result['color_depth_nearest_offset_ms']['max'] == 0.


def test_missing_streams_are_reported_without_invented_measurements(tmp_path):
    path = tmp_path / 'empty.db3'
    with sqlite3.connect(path) as db:
        db.execute('CREATE TABLE topics (id INTEGER,name TEXT,type TEXT)')
        db.execute('CREATE TABLE messages (topic_id INTEGER,timestamp INTEGER,data BLOB)')
    result = mod.inspect_recording(path, check_integrity=False)
    assert result['all_required_streams_present'] is False
    assert result['streams']['gyro'] == {'present': False}
    assert result['integrity'] is None
