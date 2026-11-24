import json

from validation_cache import completed, fingerprint


def test_cache_invalidates_changed_recording_or_measurement_and_requires_outputs(tmp_path):
    bag = tmp_path/'recording.db3'
    bag.write_bytes(b'recording')
    measurement = tmp_path/'channels.json'
    measurement.write_text('{"value":1}')
    key = fingerprint(bag, [measurement], [])
    output = tmp_path/'result.json'
    output.write_text(json.dumps({'source_fingerprint': key, 'value': 42}))
    assert completed(output, key)['value'] == 42
    assert completed(output, key, [tmp_path/'missing.npz']) is None
    measurement.write_text('{"value":2}')
    assert completed(output, fingerprint(bag, [measurement], [])) is None
    measurement.write_text('{"value":1}')
    bag.write_bytes(b'a different recording')
    assert completed(output, fingerprint(bag, [measurement], [])) is None
