"""Shared pixel-space homography fitting and safe mosaic rendering."""
import cv2
import numpy as np


def apply_transform(H, points):
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    q = np.column_stack((p, np.ones(len(p)))) @ np.asarray(H).T
    with np.errstate(divide="ignore", invalid="ignore"):
        return q[:, :2] / q[:, 2:3]


def hartley_transform(points):
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    center = p.mean(axis=0)
    distance = np.linalg.norm(p - center, axis=1).mean()
    if not np.isfinite(distance) or distance < 1e-10:
        raise ValueError("coincident or non-finite homography points")
    scale = np.sqrt(2) / distance
    return np.array([[scale, 0, -scale * center[0]],
                     [0, scale, -scale * center[1]], [0, 0, 1.]])


def fit_homography(src, dst, normalize=True, threshold_px=3., min_inliers=4):
    """Map src to dst; threshold and returned consensus are ALWAYS pixels.

    RANSAC uses the destination normalization scale, then a normalized DLT
    refit is accepted only if it does not increase pixel reprojection error.
    Recompute the mask in pixels, including after refitting. Bad geometry
    returns (None, None); it never becomes an identity pose.
    """
    src = np.asarray(src, dtype=float).reshape(-1, 2)
    dst = np.asarray(dst, dtype=float).reshape(-1, 2)
    if threshold_px <= 0 or min_inliers < 4:
        raise ValueError("positive threshold and at least four inliers required")
    if len(src) != len(dst):
        raise ValueError("source and destination lengths differ")
    if len(src) < min_inliers or not np.isfinite([src, dst]).all():
        return None, None
    if any(np.linalg.matrix_rank(p - p.mean(axis=0)) < 2 for p in (src, dst)):
        return None, None
    try:
        A = hartley_transform(src) if normalize else np.eye(3)
        B = hartley_transform(dst) if normalize else np.eye(3)
        Hn, mask = cv2.findHomography(
            apply_transform(A, src), apply_transform(B, dst), cv2.RANSAC,
            threshold_px * B[0, 0], maxIters=5000, confidence=.999)
        if Hn is None or mask is None:
            return None, None
        H = np.linalg.inv(B) @ Hn @ A
        if not np.isfinite(H).all() or abs(H[2, 2]) < 1e-12:
            return None, None
        H /= H[2, 2]
        errors = np.linalg.norm(apply_transform(H, src) - dst, axis=1)
        keep = np.isfinite(errors) & (errors <= threshold_px)
        if keep.sum() < min_inliers:
            return None, None
        # Normalize inlier-only DLT as well, so distant pixel coordinates
        # do not spoil the refinement after a correct robust consensus.
        C, D = hartley_transform(src[keep]), hartley_transform(dst[keep])
        refined, _ = cv2.findHomography(apply_transform(C, src[keep]),
                                       apply_transform(D, dst[keep]), 0)
        if refined is not None:
            candidate = np.linalg.inv(D) @ refined @ C
            if abs(candidate[2, 2]) > 1e-12:
                candidate /= candidate[2, 2]
                err = np.linalg.norm(apply_transform(candidate, src) - dst, axis=1)
                if np.isfinite(err[keep]).all() and np.mean(err[keep] ** 2) <= np.mean(errors[keep] ** 2):
                    H, errors = candidate, err
        keep = np.isfinite(errors) & (errors <= threshold_px)
        if keep.sum() < min_inliers or np.linalg.matrix_rank(H) < 3:
            return None, None
        return H, keep.astype(np.uint8).reshape(-1, 1)
    except (cv2.error, np.linalg.LinAlgError, ValueError):
        return None, None


def projected_corners(H, shape):
    """Reject a projective horizon inside the image before any allocation."""
    h, w = shape[:2]
    corners = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=float)
    H = np.asarray(H, dtype=float)
    scale = np.linalg.norm(H)
    if H.shape != (3, 3) or not np.isfinite(H).all() or scale == 0:
        raise ValueError("invalid frame transform")
    H = H / scale
    den = np.column_stack((corners, np.ones(4))) @ H[2]
    if not (np.all(den > 1e-10) or np.all(den < -1e-10)):
        raise ValueError("projective horizon crosses the frame")
    p = apply_transform(H, corners)
    if not np.isfinite(p).all() or not cv2.isContourConvex(p.astype(np.float32)):
        raise ValueError("invalid warped frame footprint")
    return p


def render_mosaic(frames, transforms, blend="average", max_side=8000,
                  max_pixels=16_000_000):
    """Warp original frames once. None poses are omitted, never duplicated.

    Average uses premultiplied interpolated colors (no second mask
    multiplication at boundaries). Best selects the most central source
    sample per pixel, avoiding averaging misaligned texture. It does not
    repair bad geometry. Both renderers preserve true black source pixels.
    """
    if blend not in ("average", "best"):
        raise ValueError("blend must be average or best")
    valid = [(f, T) for f, T in zip(frames, transforms) if T is not None]
    if not valid:
        raise ValueError("no placed frames")
    points = np.concatenate([projected_corners(T, f.shape) for f, T in valid])
    lo, hi = np.floor(points.min(axis=0)), np.ceil(points.max(axis=0))
    extent = hi - lo
    if np.any(extent <= 0) or max(extent) > max_side or np.prod(extent) > max_pixels:
        raise ValueError(f"canvas {extent[0]:.0f}x{extent[1]:.0f} exceeds render budget")
    w, h = map(int, extent)
    shift = np.array([[1., 0, -lo[0]], [0, 1, -lo[1]], [0, 0, 1]])
    colors = np.zeros((h, w, 3), dtype=np.float32)
    weights = np.zeros((h, w), dtype=np.float32)
    for f, T in valid:
        sh, sw = f.shape[:2]
        M = shift @ T
        support = cv2.warpPerspective(np.ones((sh, sw), np.float32), M, (w, h))
        warped = cv2.warpPerspective(f.astype(np.float32), M, (w, h))
        if blend == "average":
            colors += warped
            weights += support
        else:
            yy, xx = np.mgrid[:sh, :sw]
            central = np.minimum.reduce((xx + 1, sw - xx, yy + 1, sh - yy)).astype(np.float32)
            central /= central.max()
            score = cv2.warpPerspective(central, M, (w, h))
            take = (support > .99) & (score > weights)
            colors[take] = warped[take] / support[take, None]
            weights[take] = score[take]
    if blend == "average":
        np.divide(colors, weights[..., None], out=colors, where=weights[..., None] > 1e-6)
    image = np.clip(np.rint(colors), 0, 255).astype(np.uint8)
    return image, {"canvas_width_px": w, "canvas_height_px": h,
                   "coverage_fraction": float(np.mean(weights > 1e-6)),
                   "blend": blend, "canvas_shift": shift.tolist()}
