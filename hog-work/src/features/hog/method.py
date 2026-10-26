"""HOG descriptor method for the GrassVIO shared feature interface.

Follows the same layout as the other feature methods:
  - detect_and_compute(img) -> (keypoints, descriptors)
  - match(desc1, desc2)      -> list of good cv2.DMatch, sorted by distance

Notes on the design:
  - HOG itself has no keypoint detector, so keypoints come from
    Shi-Tomasi corners (cv2.goodFeaturesToTrack) and a HOG descriptor
    is computed on a fixed window around each corner.
  - Descriptors are float32; matching uses a Brute-Force L2 matcher
    with Lowe's ratio test, mirroring the SIFT setup.
  - Requires opencv-python < 5 (cv2.HOGDescriptor was dropped from
    the 5.x main package). Pin: opencv-python-headless<5.
"""

import cv2
import numpy as np

# HOG window geometry. winSize must be a multiple of blockStride;
# blockStride a multiple of cellSize (OpenCV requirement).
WIN_SIZE = (64, 64)
BLOCK_SIZE = (16, 16)
BLOCK_STRIDE = (8, 8)
CELL_SIZE = (8, 8)
NBINS = 9

# Shi-Tomasi corner detection settings.
MAX_CORNERS = 500
QUALITY_LEVEL = 0.01
MIN_DISTANCE = 8

# Lowe ratio test threshold.
RATIO = 0.75


def _hog():
    return cv2.HOGDescriptor(
        WIN_SIZE, BLOCK_SIZE, BLOCK_STRIDE, CELL_SIZE, NBINS
    )


def _to_gray(img):
    if img.ndim == 3:
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img


def detect_and_compute(img):
    """Detect keypoints and compute HOG descriptors.

    Returns (keypoints, descriptors). descriptors is a float32
    numpy array of shape (N, D) or None when nothing was found.
    """
    gray = _to_gray(img)
    corners = cv2.goodFeaturesToTrack(
        gray,
        maxCorners=MAX_CORNERS,
        qualityLevel=QUALITY_LEVEL,
        minDistance=MIN_DISTANCE,
    )
    if corners is None:
        return [], None

    hog = _hog()
    half = WIN_SIZE[0] // 2
    h, w = gray.shape
    keypoints = []
    descs = []
    for c in corners:
        x, y = float(c[0][0]), float(c[0][1])
        x0, y0 = int(round(x - half)), int(round(y - half))
        if x0 < 0 or y0 < 0 or x0 + WIN_SIZE[0] > w or y0 + WIN_SIZE[1] > h:
            continue  # patch would run off the image edge
        patch = gray[y0 : y0 + WIN_SIZE[1], x0 : x0 + WIN_SIZE[0]]
        d = hog.compute(patch).ravel().astype(np.float32)
        keypoints.append(cv2.KeyPoint(x, y, WIN_SIZE[0]))
        descs.append(d)

    if not descs:
        return [], None
    return keypoints, np.stack(descs)


def match(desc1, desc2, ratio=RATIO):
    """Match two descriptor sets with BFMatcher + Lowe ratio test.

    Returns a list of cv2.DMatch sorted by ascending distance.
    """
    if desc1 is None or desc2 is None:
        return []
    if len(desc1) < 2 or len(desc2) < 2:
        return []
    bf = cv2.BFMatcher(cv2.NORM_L2)
    pairs = bf.knnMatch(desc1, desc2, k=2)
    good = [m for m, n in pairs if m.distance < ratio * n.distance]
    return sorted(good, key=lambda m: m.distance)
