import sqlite3
import numpy as np
import cv2
import stitch_map


def test_bag_loader_caps_database_rows_before_decoding(tmp_path, monkeypatch):
    path = tmp_path / 'recording with spaces.db3'
    with sqlite3.connect(path) as connection:
        connection.execute('create table topics (id integer, name text)')
        connection.execute('create table messages (topic_id integer, timestamp integer, data blob)')
        connection.execute('insert into topics values (?, ?)', (1, stitch_map.COLOR))
        for i in range(4):
            connection.execute('insert into messages values (?, ?, ?)', (1, i, bytes([i])))
    seen = []
    def fake_image(blob):
        seen.append(blob[0])
        image = np.full((4, 6, 3), blob[0], np.uint8)
        return {'width': 6, 'height': 4, 'data': image.tobytes()}
    monkeypatch.setattr(stitch_map, 'parse_image', fake_image)
    limited = stitch_map.load_frames_from_bag(str(path), max_frames=2)
    assert len(limited) == 2
    assert seen == [0, 1]
    assert limited[1].shape == (4, 6, 3)
    assert len(stitch_map.load_frames_from_bag(str(path), max_frames=0)) == 4


def test_image_folder_loader_caps_before_reading(tmp_path, monkeypatch):
    for i in range(4):
        cv2.imwrite(str(tmp_path / f'{i:03}.png'), np.full((8, 9, 3), i * 25, np.uint8))
    seen = []
    original = stitch_map.cv2.imread
    def spy(path):
        seen.append(path)
        return original(path)
    monkeypatch.setattr(stitch_map.cv2, 'imread', spy)
    assert len(stitch_map.load_frames_from_folder(str(tmp_path), max_frames=2)) == 2
    assert len(seen) == 2
