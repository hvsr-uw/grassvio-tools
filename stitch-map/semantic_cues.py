"""Lightweight semantic cues for vegetation-aware feature filtering.

This module deliberately does not claim to be a learned semantic
segmentation model. It produces an auditable vegetation likelihood from
colour, then combines it with the temporal motion residual.
The cue is only allowed to reject a point when both signals agree: a point
must look vegetation-like and violate the fitted camera model. This keeps a
green but rigid surface usable and avoids treating colour alone as proof of
wind-driven motion.
"""

from __future__ import annotations

import cv2
import numpy as np


def vegetation_likelihood(
    image_bgr: np.ndarray,
    *,
    hue_range: tuple[int, int] = (25, 100),
    min_saturation: int = 30,
    min_value: int = 20,
    blur_radius: int = 3,
) -> np.ndarray:
    """Return a float likelihood in [0, 1] for vegetation-like pixels."""
    image = np.asarray(image_bgr)
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("expected an HxWx3 BGR image")
    if image.dtype != np.uint8:
        raise ValueError("semantic cue input must be uint8")
    lo, hi = hue_range
    if not (0 <= lo <= hi <= 180):
        raise ValueError("hue_range must be inside OpenCV's [0, 180] range")
    if not (0 <= min_saturation <= 255 and 0 <= min_value <= 255):
        raise ValueError("saturation and value thresholds must be uint8 values")

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue, saturation, value = [hsv[..., i].astype(np.float32) for i in range(3)]
    in_hue = (hue >= float(lo)) & (hue <= float(hi))
    chromatic = np.clip(
        (saturation - min_saturation) / max(1.0, 255.0 - min_saturation), 0.0, 1.0
    )
    brightness = np.clip(
        (value - min_value) / max(1.0, 255.0 - min_value), 0.0, 1.0
    )
    score = np.where(in_hue, chromatic * brightness, 0.0).astype(np.float32)
    if blur_radius > 1:
        if blur_radius % 2 == 0:
            raise ValueError("blur_radius must be odd")
        score = cv2.GaussianBlur(score, (blur_radius, blur_radius), 0)
    return np.clip(score, 0.0, 1.0)


def sample_likelihood(likelihood: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Sample a likelihood image at floating-point (x, y) points."""
    image = np.asarray(likelihood, dtype=np.float32)
    pts = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    if image.ndim != 2:
        raise ValueError("likelihood must be a two-dimensional image")
    result = np.zeros(len(pts), dtype=np.float32)
    h, w = image.shape
    finite = np.isfinite(pts).all(axis=1)
    xy = np.rint(pts[finite]).astype(int)
    inside = (xy[:, 0] >= 0) & (xy[:, 0] < w) & (xy[:, 1] >= 0) & (xy[:, 1] < h)
    valid_indices = np.flatnonzero(finite)[inside]
    result[valid_indices] = image[xy[inside, 1], xy[inside, 0]]
    return result


def semantic_motion_keep(
    vegetation_scores: np.ndarray,
    motion_residuals_px: np.ndarray,
    *,
    vegetation_threshold: float = 0.35,
    residual_threshold_px: float = 1.5,
) -> tuple[np.ndarray, np.ndarray]:
    """Reject vegetation-like points with incompatible motion.

    Returns (keep, dynamic_scores). dynamic_scores is the product of
    normalized vegetation likelihood and normalized motion residual.
    """
    vegetation = np.asarray(vegetation_scores, dtype=float).reshape(-1)
    residual = np.asarray(motion_residuals_px, dtype=float).reshape(-1)
    if len(vegetation) != len(residual):
        raise ValueError("vegetation and residual arrays must have equal length")
    if not (0.0 <= vegetation_threshold <= 1.0) or residual_threshold_px <= 0:
        raise ValueError("invalid semantic cue thresholds")
    finite = np.isfinite(vegetation) & np.isfinite(residual)
    vegetation = np.clip(vegetation, 0.0, 1.0)
    normalized_residual = np.clip(residual / residual_threshold_px, 0.0, 1.0)
    dynamic_scores = vegetation * normalized_residual
    reject = finite & (vegetation >= vegetation_threshold) & (residual > residual_threshold_px)
    return finite & ~reject, dynamic_scores.astype(np.float32)
