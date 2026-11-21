"""Refine plane homographies using an overlapping frame graph near the pose prior."""

import cv2
import numpy as np
from scipy import sparse
from scipy.sparse.linalg import lsmr

from recording_camera import COLOR, rays


def plane_features(recording, indices, forwards):
    detector = cv2.SIFT_create(nfeatures=1250)
    features = []
    for index, forward in zip(indices, forwards):
        gray = cv2.cvtColor(recording.image(COLOR, index), cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = detector.detectAndCompute(gray, None)
        image_xy = np.asarray([point.pt for point in keypoints]).reshape(-1, 2)
        source_rays = rays(image_xy, recording.color)
        camera_to_map = np.linalg.inv(forward)
        flat = source_rays @ camera_to_map.T
        valid = (flat[:, 2] >= 1 / 3) & (flat[:, 2] < 1 / .05)
        map_xy = flat[valid, :2] / flat[valid, 2:3]
        features.append((image_xy[valid], map_xy, descriptors[valid] if descriptors is not None else None))
    return features


def measure_edge(features, matcher, first, second):
    source_pixels, source_plane, source_descriptors = features[first]
    _, target_plane, target_descriptors = features[second]
    if source_descriptors is None or target_descriptors is None or len(target_descriptors) < 2:
        return None
    matches = [a for a, b in matcher.knnMatch(source_descriptors, target_descriptors, k=2)
               if a.distance < .72 * b.distance]
    if len(matches) < 40:
        return None
    source_ids = np.array([m.queryIdx for m in matches])
    target_ids = np.array([m.trainIdx for m in matches])
    a, b = source_plane[source_ids], target_plane[target_ids]
    train = np.floor(source_pixels[source_ids] / 40).astype(int).sum(axis=1) % 2 == 0
    if train.sum() < 25:
        return None
    model, inliers = cv2.estimateAffinePartial2D(a[train], b[train], method=cv2.RANSAC,
                                                ransacReprojThreshold=3., maxIters=2000)
    if model is None or inliers.sum() < 25:
        return None
    scale = np.linalg.norm(model[:, 0])
    if not .87 < scale < 1.15 or np.linalg.norm(model[:, 2]) > 100:
        return None
    accepted = np.linalg.norm(a @ model[:, :2].T + model[:, 2] - b, axis=1) <= 3
    train = train[accepted]
    a, b = a[accepted], b[accepted]
    if (~train).sum() < 12:
        return None
    return first, second, a, b, train


def similarity_corrections(edges, count, pixels_per_m):
    rows, cols, values, target = [], [], [], []
    rng = np.random.default_rng(49)
    for first, second, a, b, train in edges:
        chosen = np.flatnonzero(train)
        if len(chosen) > 120:
            chosen = rng.choice(chosen, 120, replace=False)
        for p, q in zip(a[chosen], b[chosen]):
            u, v = p / pixels_per_m
            x, y = q / pixels_per_m
            for coeffs, axis in (((u, -v, 1., 0., -x, y, -1., 0.), 0),
                                 ((v, u, 0., 1., -y, -x, 0., -1.), 1)):
                rows.extend([len(target)] * 8)
                cols.extend(range(4 * first, 4 * first + 4))
                cols.extend(range(4 * second, 4 * second + 4))
                values.extend(coeffs)
                target.append(q[axis] - p[axis])
    observations = len(target)
    for i in range(count * 4):
        rows.append(len(target))
        cols.append(i)
        values.append(500. if i < 4 else (2. if i % 4 < 2 else .4))
        target.append(0.)

    matrix = sparse.csr_matrix((values, (rows, cols)), shape=(len(target), count * 4))
    target = np.asarray(target)
    weights = np.ones(len(target))
    for _ in range(3):
        solution = lsmr(matrix.multiply(weights[:, None]), target * weights,
                        atol=2e-6, btol=2e-6, maxiter=400)[0]
        residuals = np.linalg.norm((matrix @ solution - target)[:observations].reshape(-1, 2), axis=1)
        weights[:observations] = np.repeat(np.sqrt(np.minimum(1, 2.5 / np.maximum(residuals, 1e-8))), 2)
    result = []
    for a, b, tx, ty in solution.reshape(-1, 4):
        result.append(np.array([[1 + a / pixels_per_m, -b / pixels_per_m, tx],
                                [b / pixels_per_m, 1 + a / pixels_per_m, ty], [0, 0, 1.]]))
    return result


def refine(recording, indices, forwards, pixels_per_m):
    """Fit with alternate source tiles and audit the other tiles of each pair."""
    features = plane_features(recording, indices, forwards)
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    count = len(indices)
    train_pairs = {(i, i + gap) for gap in (1, 4, 10) for i in range(count - gap)}
    train_pairs.add((0, count - 1))
    test_pairs = {(i, i + gap) for gap in (7, 19) for i in range(0, count - gap, 4)}
    cv2.setRNGSeed(49)
    edges = [match for a, b in sorted(train_pairs) if a < b
             if (match := measure_edge(features, matcher, a, b)) is not None]
    tests = [match for a, b in sorted(test_pairs)
             if (match := measure_edge(features, matcher, a, b)) is not None]
    if not edges or not tests:
        raise ValueError('insufficient overlapping image pairs for refinement and a held-out check')
    corrections = similarity_corrections(edges, count, pixels_per_m)
    if any(np.linalg.norm(matrix[:2, 2]) > 100 or not .85 < np.linalg.norm(matrix[:2, 0]) < 1.18
           for matrix in corrections):
        raise ValueError('refined homographies left the calibrated pose neighborhood')

    def score(pairs, common_pair):
        scored = []
        for first, second, a, b, train in pairs:
            p, q = a[~train], b[~train]
            first_correction, second_correction = corrections[first], corrections[second]
            corrected_p = p @ first_correction[:2, :2].T + first_correction[:2, 2]
            corrected_q = q @ second_correction[:2, :2].T + second_correction[:2, 2]
            after = corrected_p - corrected_q
            scored.append(dict(first=indices[first], second=indices[second], points=len(p),
                               before_px=float(np.median(np.linalg.norm(p - q, axis=1))),
                               after_px=float(np.median(np.linalg.norm(after, axis=1))),
                               pair_used_in_fit=common_pair))
        return scored

    rows = score(edges, True) + score(tests, False)
    refined = [forward @ np.linalg.inv(correction) for forward, correction in zip(forwards, corrections)]
    return refined, rows
