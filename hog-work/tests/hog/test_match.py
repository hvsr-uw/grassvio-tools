"""HOG feature matching test.

Mirrors the shared interface test used by the other feature methods:
detect + compute on two views of one scene, match, and check that the
recovered correspondences agree with the known geometric shift.
"""

import cv2
import numpy as np

from src.features.hog.method import detect_and_compute, match


def _synthetic_scene(seed=7, size=256):
    rng = np.random.default_rng(seed)
    # grass-like texture: dark base with many small bright strokes
    img = np.zeros((size, size), dtype=np.uint8)
    for _ in range(900):
        x, y = rng.integers(0, size, 2)
        length = rng.integers(4, 14)
        angle = rng.uniform(0, np.pi)
        x1 = int(np.clip(x + length * np.cos(angle), 0, size - 1))
        y1 = int(np.clip(y + length * np.sin(angle), 0, size - 1))
        cv2.line(img, (x, y), (x1, y1), int(rng.integers(120, 255)), 1)
    return img


def test_match():
    img1 = _synthetic_scene()
    dx, dy = 12, -7
    warp = np.float32([[1, 0, dx], [0, 1, dy]])
    img2 = cv2.warpAffine(img1, warp, img1.shape[::-1])

    kp1, desc1 = detect_and_compute(img1)
    kp2, desc2 = detect_and_compute(img2)
    assert desc1 is not None and desc2 is not None
    assert len(kp1) > 10 and len(kp2) > 10

    good = match(desc1, desc2)
    assert len(good) >= 8, f"too few good matches: {len(good)}"

    # recovered displacement should match the known shift
    est = np.array(
        [
            [kp2[m.trainIdx].pt[0] - kp1[m.queryIdx].pt[0],
             kp2[m.trainIdx].pt[1] - kp1[m.queryIdx].pt[1]]
            for m in good
        ]
    )
    median = np.median(est, axis=0)
    assert abs(median[0] - dx) < 3.0, f"dx off: {median[0]}"
    assert abs(median[1] - dy) < 3.0, f"dy off: {median[1]}"
