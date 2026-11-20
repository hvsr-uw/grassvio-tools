"""Measure homography fit on spatially separated tracks not used for fitting.

This tests image consistency only, not physical wind or metric camera pose.
"""
import numpy as np


def evaluate_heldout_window(history, camera_models, threshold_px,
                            score_tracks, fit_model, project,
                            min_inliers=20, tile_px=40):
    """Compare unfiltered/filtered fits on unseen checkerboard tiles.

    The diagnostic does NOT alter the full-data homography used by the
    stitcher. Filter priors fitted from video must only see training tracks.
    """
    history = np.asarray(history, dtype=float)
    if (history.ndim != 3 or history.shape[0] < 2 or history.shape[2] != 2 or
            not np.isfinite(history).all() or threshold_px <= 0 or tile_px <= 0):
        raise ValueError("need complete finite tracked points and positive thresholds")
    n = history.shape[1]
    output = {"heldout_train_tracks": 0, "heldout_test_tracks": 0,
              "heldout_train_kept": 0,
              "heldout_unfiltered_median_error_px": None,
              "heldout_median_error_px": None,
              "heldout_status": "too_few_spatially_separated_tracks"}
    if n < min_inliers * 2:
        return output
    # Separate areas rather than alternating neighbouring points. A patch
    # that moves coherently cannot be used for both train and test in the
    # same tile. These remain tracked image observations, not ground truth.
    tile = np.floor(history[0] / tile_px).astype(np.int64)
    holdout = (tile[:, 0] + tile[:, 1]) % 2 == 1
    train = ~holdout
    output["heldout_train_tracks"] = int(train.sum())
    output["heldout_test_tracks"] = int(holdout.sum())
    if train.sum() < min_inliers or holdout.sum() < min_inliers:
        return output
    tracking_train = history[:, train]
    keep, _ = score_tracks(tracking_train, camera_models, threshold_px)
    keep = np.asarray(keep, dtype=bool)
    if len(keep) != len(tracking_train[0]):
        raise ValueError("temporal filter returned wrong number of flags")
    output["heldout_train_kept"] = int(keep.sum())
    original = history[0, holdout]
    last = history[-1, holdout]

    def error(src, dst):
        model, _ = fit_model(src, dst, min_inliers=min_inliers)
        if model is None:
            return None
        predicted = np.asarray(project(model, last), dtype=float)
        if predicted.shape != original.shape or not np.isfinite(predicted).all():
            return None
        return float(np.median(np.linalg.norm(predicted - original, axis=1)))

    output["heldout_unfiltered_median_error_px"] = error(tracking_train[-1], tracking_train[0])
    if keep.sum() >= min_inliers:
        output["heldout_median_error_px"] = error(tracking_train[-1, keep], tracking_train[0, keep])
    output["heldout_status"] = ("scored" if output["heldout_median_error_px"] is not None
                                else "filtered_fit_unavailable")
    return output
