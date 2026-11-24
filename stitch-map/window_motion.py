"""Anchor detection, frame-to-frame flow, and temporal pre-RANSAC filtering.

This detects motion incompatible with a camera/plane model, not wind as a
physical cause. Without an independent full-motion prior, a visually fitted
dominant plane can follow coherently moving grass. Gyro rotation alone is
not a complete prediction when the camera translates.
"""
import cv2
import numpy as np

from geometry import apply_transform, fit_homography, projected_corners
from heldout_diagnostics import evaluate_heldout_window
from semantic_cues import sample_likelihood, semantic_motion_keep, vegetation_likelihood


def validate_camera_motion(data, frame_count):
    """Input: frame-0-to-frame-k pixel homographies, full rigid-plane motion."""
    if data.get("translation_compensated") is not True:
        raise ValueError("camera-motion prior must include translation; gyro-only flow is ambiguous")
    H = np.asarray(data.get("image_transforms"), dtype=float)
    if H.shape != (frame_count, 3, 3) or not np.isfinite(H).all():
        raise ValueError("camera-motion needs one finite 3x3 image_transform per input frame")
    if any(np.linalg.matrix_rank(T) != 3 for T in H):
        raise ValueError("singular camera-motion prior")
    return H


def track_window(grays, max_features=2000, fb_threshold_px=1., initial_points=None, return_ids=False):
    """Detect only at the first anchor; surviving tracks retain their IDs."""
    starts = (cv2.goodFeaturesToTrack(grays[0], maxCorners=max_features,
                                    qualityLevel=.01, minDistance=7, blockSize=7)
              if initial_points is None else np.asarray(initial_points,np.float32).reshape(-1,1,2))
    if starts is None or not len(starts):
        empty=(np.empty((len(grays), 0, 2)),0)
        return (*empty,np.empty(0,dtype=int)) if return_ids else empty
    n = len(starts)
    history = np.full((len(grays), n, 2), np.nan)
    history[0] = starts.reshape(-1, 2)
    ids = np.arange(n)
    points = starts
    lk = dict(winSize=(21, 21), maxLevel=3,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, .01))
    for k in range(1, len(grays)):
        if not len(ids):
            break
        nxt, status, _ = cv2.calcOpticalFlowPyrLK(grays[k-1], grays[k], points, None, **lk)
        if nxt is None or status is None:
            break
        # Never send NaNs or failed forward tracks into the backward tracker.
        forward = status.ravel().astype(bool) & np.isfinite(nxt.reshape(-1, 2)).all(axis=1)
        ids, points, nxt = ids[forward], points[forward], nxt[forward]
        if not len(ids):
            break
        back, back_status, _ = cv2.calcOpticalFlowPyrLK(grays[k], grays[k-1], nxt, None, **lk)
        if back is None or back_status is None:
            break
        h, w = grays[k].shape
        q = nxt.reshape(-1, 2)
        good = (back_status.ravel().astype(bool)
                & np.isfinite(back.reshape(-1, 2)).all(axis=1)
                & (np.linalg.norm(back - points, axis=2).ravel() <= fb_threshold_px)
                & (q[:, 0] >= 0) & (q[:, 0] < w)
                & (q[:, 1] >= 0) & (q[:, 1] < h))
        ids, points = ids[good], nxt[good]
        history[k, ids] = points.reshape(-1, 2)
    # A window verdict requires observations throughout the whole window.
    complete = np.isfinite(history).all(axis=(0, 2))
    result=(history[:, complete],n)
    return (*result,np.flatnonzero(complete)) if return_ids else result


def temporal_filter(history, camera_models=None, residual_threshold_px=1.5):
    """Median camera-compensated residual per persistent track BEFORE RANSAC.

    camera_models[k] maps anchor pixels into frame k. If unavailable, use
    LMedS (not RANSAC) for a provisional plane at each offset. A full camera
    prior or known stationary acquisition is needed to disambiguate a
    moving grass majority. The median across offsets catches oscillation
    even when a blade returns to its starting point at the final anchor.
    """
    if residual_threshold_px <= 0:
        raise ValueError("motion residual threshold must be positive")
    steps, n, _ = history.shape
    residuals = []
    for k in range(1, steps):
        if camera_models is not None:
            model = camera_models[k]
        elif n >= 4:
            model, _ = cv2.findHomography(history[0], history[k], cv2.LMEDS)
        else:
            model = None
        if model is None:
            return np.zeros(n, bool), np.full(n, np.inf)
        predicted = apply_transform(model, history[0])
        residuals.append(np.linalg.norm(predicted - history[k], axis=1))
    scores = np.median(residuals, axis=0) if residuals else np.full(n, np.inf)
    return np.isfinite(scores) & (scores <= residual_threshold_px), scores


def flow_pairs(grays, window=25, residual_threshold_px=1.5, camera_still=False,
               camera_motion=None, min_inliers=20, normalize=True,
               reject_full_frame_horizon=True, track_observer=None,
               semantic_images=None, semantic_vegetation_threshold=0.35,
               semantic_residual_ratio=0.75):
    """Yield next->anchor H and diagnostics, once per completed window.

    Consumes every intermediate frame for flow and re-detects at anchors.
    The final partial window is included and its length is reported.
    A failed/empty window is explicit; no unfiltered retry is allowed.
    """
    if window < 2:
        raise ValueError("flow window must be at least 2 frames")
    if camera_still and camera_motion is not None:
        raise ValueError("choose known-still acquisition or a camera-motion file")
    if camera_motion is not None and len(camera_motion) != len(grays):
        raise ValueError("camera motion length differs from input frames")
    if semantic_images is not None and len(semantic_images) != len(grays):
        raise ValueError("semantic image sequence length differs from input frames")
    if not 0 < semantic_residual_ratio <= 1:
        raise ValueError("semantic residual ratio must be in (0, 1]")
    mode = "known_stationary" if camera_still else ("independent_full_motion" if camera_motion is not None else "visual_plane_only")
    for start in range(0, len(grays) - 1, window):
        end = min(start + window, len(grays) - 1)
        history, detected = track_window(grays[start:end+1])
        models = None
        if camera_still:
            models = np.repeat(np.eye(3)[None], end - start + 1, axis=0)
        elif camera_motion is not None:
            models = camera_motion[start:end+1] @ np.linalg.inv(camera_motion[start])
        keep, scores = temporal_filter(history, models, residual_threshold_px)
        semantic_rejected = 0
        if semantic_images is not None and len(history[0]):
            likelihood = vegetation_likelihood(semantic_images[start])
            vegetation = sample_likelihood(likelihood, history[0])
            semantic_keep, _ = semantic_motion_keep(
                vegetation, scores,
                vegetation_threshold=semantic_vegetation_threshold,
                residual_threshold_px=residual_threshold_px*semantic_residual_ratio)
            semantic_rejected = int(np.count_nonzero(keep & ~semantic_keep))
            keep &= semantic_keep
        H, mask = fit_homography(history[-1, keep], history[0, keep],
                                 normalize=normalize, min_inliers=min_inliers)
        if H is not None and reject_full_frame_horizon:
            try:
                projected_corners(H, grays[end].shape)
            except ValueError:
                H, mask = None, None
        finite = scores[np.isfinite(scores)]
        stats = {"start_frame": start, "end_frame": end,
                 "window_frames": end - start, "features_detected": detected,
                 "complete_tracks": int(history.shape[1]),
                 "prefilter_kept": int(keep.sum()),
                 "prefilter_rejected": int(len(keep) - keep.sum()),
                 "semantic_rejected": semantic_rejected,
                 "semantic_residual_threshold_px":(residual_threshold_px*semantic_residual_ratio
                     if semantic_images is not None else None),
                 "median_temporal_residual_px": float(np.median(finite)) if len(finite) else None,
                 "inliers": int(mask.sum()) if mask is not None else 0,
                 "camera_evidence": mode, "ok": H is not None,
                 "wind_cause_verified": False}
        stats.update(evaluate_heldout_window(
            history, models, residual_threshold_px,
            temporal_filter, fit_homography, apply_transform,
            min_inliers=min_inliers))
        if track_observer is not None:
            track_observer(start, end, history, keep, mask)
        yield start, end, H, stats
