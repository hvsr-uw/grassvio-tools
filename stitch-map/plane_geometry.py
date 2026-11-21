"""Fit a dominant ground plane and construct its map coordinates."""

import numpy as np


def fit_plane(points, threshold_m=.04):
    points = np.asarray(points, float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 100:
        raise ValueError('too few measured points for the initial plane')
    rng = np.random.default_rng(60)
    sample = points[rng.choice(len(points), min(len(points), 12000), replace=False)]
    best = None
    for _ in range(150):
        a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        if np.linalg.norm(normal) < 1e-8:
            continue
        normal /= np.linalg.norm(normal)
        distance = normal @ a
        mask = np.abs(sample @ normal - distance) < threshold_m
        if best is None or mask.sum() > best.sum():
            best = mask
    if best is None or best.mean() < .4:
        raise ValueError('initial dominant plane not supported')
    selected = sample[best]
    center = selected.mean(axis=0)
    _, _, vt = np.linalg.svd(selected - center, full_matrices=False)
    normal = vt[-1]
    distance = float(normal @ center)
    if distance < 0:
        normal, distance = -normal, -distance
    return normal, distance, {
        'sample_points': len(sample), 'inlier_fraction': float(best.mean()),
        'median_plane_error_m': float(np.median(np.abs(selected @ normal - distance))),
    }


def plane_basis(normal):
    normal = np.asarray(normal, float) / np.linalg.norm(normal)
    horizontal = np.array([1., 0, 0]) - normal * normal[0]
    if np.linalg.norm(horizontal) < .1:
        horizontal = np.array([0., 0, 1]) - normal * normal[2]
    horizontal /= np.linalg.norm(horizontal)
    return np.column_stack((horizontal, np.cross(normal, horizontal)))
