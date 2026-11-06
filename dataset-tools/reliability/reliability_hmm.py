#!/usr/bin/env python3
"""Per-feature reliability as a two-state hidden Markov model (GrassVIO).

Model, in plain words: every tracked feature is either reliable (it sits
on something rigid, so it moves exactly the way the camera moved) or
unreliable (wind-blown grass, a bad match, something that moves on its
own). We never see the state directly. Each frame pair gives one piece
of evidence: the feature's reprojection residual under the pairwise
rigid fit, in pixels. Reliable features cluster near zero residual;
unreliable ones spread widely. A first-order Markov chain says the
state usually stays what it was, but can change. A forward filter
(exact Bayesian inference, the same recursion BANSAC uses for its
inlier probabilities) turns the residual history into P(reliable).

Design choices, all written down so they can be argued with:
  - states: R (reliable), U (unreliable); prior P(R) = 0.5 (no prior
    information, as in BANSAC).
  - transition (1st-order Markov; BANSAC's ablation found 1st order
    most accurate): P(R|R)=0.95, P(U|R)=0.05, P(R|U)=0.30, P(U|U)=0.70.
  - likelihoods are half-normal on the residual r (pixels):
    P(r|R) with sigma_R = 0.5 px, P(r|U) with sigma_U = 3.0 px.
    sigma_R comes from the wind-free control 102631c: pooled median
    residual 0.185 px, p90 0.458 px. sigma_U is a broad spread for
    features that do not follow the rigid motion.
  - track length is not a separate input: a track's evidence count is
    its length, so long consistent tracks end near P(R) = 1 and long
    inconsistent tracks end near 0. Short tracks stay near the prior.
  - the take-level wind label from the feature table is deliberately
    NOT used as a prior: it is the held-out label the model is tested
    against, so using it would be circular.

Usage:
    python3 reliability_hmm.py --bag <take>.db3 --out <prefix> [--stride 5]

Writes <prefix>_tracks.json (per-track reliability, length, median
residual), <prefix>_hist.png (reliability histogram) and
<prefix>_curve.png (mean belief per pair index). Reads bags read-only.
"""
import argparse
import json
import math
import os
import sqlite3
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "bag-exploration"))
from explore_bag import parse_image, COLOR  # noqa: E402

STRIDE = 5
LOWE = 0.75
RANSAC_PX = 3.0

# HMM parameters (see module docstring for where each number comes from).
P_RR, P_UR = 0.95, 0.05   # from R: stay reliable / turn unreliable
P_RU, P_UU = 0.30, 0.70   # from U: recover / stay unreliable
SIGMA_R = 0.5
SIGMA_U = 3.0


def half_normal(r, sigma):
    """Density of |N(0, sigma)| at residual r >= 0."""
    return math.sqrt(2.0 / math.pi) / sigma * math.exp(-(r * r) / (2.0 * sigma * sigma))


def forward_filter(residuals):
    """Exact forward inference. Returns final P(reliable) and the belief
    after each step (mean belief history is the reliability curve)."""
    b_r, b_u = 0.5, 0.5
    history = [b_r]
    for r in residuals:
        # predict through the Markov transition, then update with the
        # residual likelihood (Bayes rule, normalized).
        p_r = P_RR * b_r + P_RU * b_u
        p_u = P_UR * b_r + P_UU * b_u
        l_r = half_normal(r, SIGMA_R)
        l_u = half_normal(r, SIGMA_U)
        tot = l_r * p_r + l_u * p_u
        if tot <= 0:
            b_r, b_u = p_r, p_u
        else:
            b_r, b_u = l_r * p_r / tot, l_u * p_u / tot
        history.append(b_r)
    return b_r, history


def load_frames(bag_path):
    """Decode all colour frames of a .db3 bag, in timestamp order."""
    db = sqlite3.connect(f"file:{bag_path}?mode=ro", uri=True)
    db.execute("PRAGMA query_only=ON")
    tid = db.execute("select id from topics where name=?", (COLOR,)).fetchone()[0]
    rows = db.execute(
        "select data from messages where topic_id=? order by timestamp",
        (tid,)).fetchall()
    db.close()
    frames = []
    for (blob,) in rows:
        p = parse_image(blob)
        rgb = np.frombuffer(p["data"], dtype=np.uint8).reshape(
            p["height"], p["width"], 3)
        frames.append(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    return frames


def build_tracks(grays, stride):
    """Chain SIFT matches across consecutive chosen frames.

    Returns a list of tracks; each track is a list of residuals (one per
    pair the feature survived) plus its length in pairs.
    """
    sift = cv2.SIFT_create(nfeatures=10000)
    matcher = cv2.BFMatcher()
    chosen = list(range(0, len(grays), stride))
    feats = []
    for idx in chosen:
        kp, des = sift.detectAndCompute(grays[idx], None)
        feats.append((kp, des))
    # track id per (frame position, keypoint index); tracks hold
    # (global pair index, residual) pairs, so the belief curve below is
    # mean belief vs time along the take, not vs track age.
    owner = {}
    tracks = []  # each: {"obs": [(pair_idx, residual), ...]}
    n_pairs = 0
    pair_inliers = []
    for a in range(len(chosen) - 1):
        kp_i, des_i = feats[a]
        kp_j, des_j = feats[a + 1]
        if des_i is None or des_j is None:
            continue
        matches = matcher.knnMatch(des_i, des_j, k=2)
        good = []
        for m_n in matches:
            if len(m_n) != 2:
                continue
            m, n = m_n
            if m.distance < LOWE * n.distance:
                good.append(m)
        if len(good) < 8:
            continue
        src = np.float32([kp_i[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst = np.float32([kp_j[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, RANSAC_PX)
        if H is None:
            continue
        proj = cv2.perspectiveTransform(src, H).reshape(-1, 2)
        resid = np.linalg.norm(dst.reshape(-1, 2) - proj, axis=1)
        inl = int(mask.sum()) if mask is not None else 0
        pair_inliers.append(inl)
        n_pairs += 1
        # Link matches into tracks, best (smallest distance) first so a
        # keypoint is claimed once.
        order = sorted(range(len(good)), key=lambda k: good[k].distance)
        used_j = set()
        for k in order:
            m = good[k]
            if m.trainIdx in used_j:
                continue
            used_j.add(m.trainIdx)
            r = float(resid[k])
            key = (a, m.queryIdx)
            if key in owner:
                tracks[owner[key]]["obs"].append((a, r))
                owner[(a + 1, m.trainIdx)] = owner[key]
            else:
                owner[(a + 1, m.trainIdx)] = len(tracks)
                tracks.append({"obs": [(a, r)]})
    return tracks, n_pairs, pair_inliers


def take_stamp(bag_path):
    """Take stamp from a bag filename: last 15 chars before .db3."""
    name = os.path.basename(bag_path)
    if name.endswith(".db3"):
        name = name[:-4]
    return name[-15:] if len(name) > 15 else name


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--stride", type=int, default=STRIDE)
    args = ap.parse_args()

    frames = load_frames(args.bag)
    grays = [cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) for f in frames]
    tracks, n_pairs, pair_inliers = build_tracks(grays, args.stride)

    # Score every track. Tracks of length 1 have a single residual, so
    # their belief stays close to the prior; that is the model working,
    # not a bug.
    scored = []
    belief_by_step = {}
    for t in tracks:
        residuals = [r for _, r in t["obs"]]
        rel, hist = forward_filter(residuals)
        med = float(np.median(residuals)) if residuals else 0.0
        scored.append({"reliability": rel, "length": len(residuals),
                       "median_residual": med})
        # hist[0] is the prior before any observation; hist[i] is the
        # belief after the i-th observation, which happened at the
        # global pair index stored alongside it.
        for (pair_idx, _), b in zip(t["obs"], hist[1:]):
            belief_by_step.setdefault(pair_idx, []).append(b)
    rels = np.array([s["reliability"] for s in scored]) if scored else np.array([])
    meds = np.array([s["median_residual"] for s in scored]) if scored else np.array([])
    lens = np.array([s["length"] for s in scored]) if scored else np.array([])

    summary = {
        "take": take_stamp(args.bag),
        "frames": len(frames),
        "pairs": n_pairs,
        "pair_inliers_median": float(np.median(pair_inliers)) if pair_inliers else 0.0,
        "n_tracks": len(scored),
        "median_track_length": float(np.median(lens)) if len(lens) else 0.0,
        "median_final_reliability": float(np.median(rels)) if len(rels) else 0.0,
        "frac_reliable_gt_0.8": float(np.mean(rels > 0.8)) if len(rels) else 0.0,
        "frac_unreliable_lt_0.2": float(np.mean(rels < 0.2)) if len(rels) else 0.0,
        "median_of_median_residual": float(np.median(meds)) if len(meds) else 0.0,
        # Plain threshold rule, for comparison: fraction of tracks whose
        # median residual is under 0.5 px. If the HMM does not beat this
        # at separating takes, it adds nothing.
        "frac_median_resid_lt_0.5": float(np.mean(meds < 0.5)) if len(meds) else 0.0,
        "belief_curve": {str(k): float(np.mean(v)) for k, v in sorted(belief_by_step.items())},
    }
    with open(args.out + "_tracks.json", "w") as f:
        json.dump({"summary": summary, "tracks": scored}, f, indent=1)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.hist(rels, bins=20, range=(0, 1), color="steelblue", edgecolor="white")
    ax.set_xlabel("final reliability P(reliable)")
    ax.set_ylabel("tracks")
    ax.set_title(take_stamp(args.bag))
    fig.tight_layout()
    fig.savefig(args.out + "_hist.png", dpi=110)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 4.5))
    xs = sorted(int(k) for k in summary["belief_curve"])
    ys = [summary["belief_curve"][str(k)] for k in xs]
    ax.plot(xs, ys, marker="o", ms=3)
    ax.set_ylim(0, 1)
    ax.set_xlabel("pair index (stride-5 steps along the take)")
    ax.set_ylabel("mean belief P(reliable)")
    ax.set_title(take_stamp(args.bag) + " — reliability over time")
    fig.tight_layout()
    fig.savefig(args.out + "_curve.png", dpi=110)
    plt.close(fig)

    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
