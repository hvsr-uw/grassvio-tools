#!/usr/bin/env python3
"""Pose-graph optimization of the chained-homography panorama, take 080033.

Control experiment: does adding the visual loop-closure edge as a real
pose-graph constraint fix the far-end stretch of the normalized chained
panorama, or does it only move the same bias around?

Method (Gauss-Newton / Levenberg-Marquardt pose graph over homographies):
  - Nodes: one homography T_k per chosen frame, mapping frame k into
    frame-0 coordinates. T_0 is fixed to the identity.
  - Edges: the 88 chained neighbour homographies plus ONE loop edge, the
    measured end-to-start homography (best of last-5 vs first-5 SIFT
    matches). Each edge contributes a transfer-error residual: the
    predicted relative transform T_i^-1 T_j applied to a 3x3 grid of
    sample points in frame j, minus the measured homography applied to
    the same points, in pixels.
  - The solver minimizes the sum of squared residuals over ALL edges
    jointly. This differs from the linear distribution tried before in
    three ways: (1) no correction is prescribed in advance -- the
    correction each frame gets is whatever best satisfies all edges at
    once; (2) the loop edge constrains only its two endpoint frames
    directly, and the interior frames move only as far as the chain
    edges force them to; (3) chain edges resist the correction, so the
    optimizer must trade loop error against chain error instead of
    dragging every frame along a fixed ramp. If the pairwise
    homographies are systematically biased (wrong motion model for a
    translating camera over 3D grass), the optimizer can only find a
    self-consistent wrong solution, and the stretch should survive --
    that is the hypothesis under test.

All pair homographies come from stitch_map.pair_homography in
normalized (Hartley) mode, stride 5, SIFT, Lowe 0.75, RANSAC 3.0 px.
stitch_map.py is not modified. Rendering reuses the same
warp-and-average accumulation as stitch_map.stitch, in a local
function, because stitch() computes its own chain internally and
accepts no precomputed transforms.

Bags are read read-only. Nothing is uploaded.

Usage:
    python3 pose_graph_080033.py --bag <take>.db3 --out-dir <folder>
"""
import argparse
import json
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "..", "dataset-tools",
                                "stitch-map"))
import stitch_map  # noqa: E402

W, H = 640, 480
STRIDE = 5
GRID = np.float32([[x, y] for y in (0, 240, 480) for x in (0, 320, 640)])


def apply_h(H, pts):
    """Apply a 3x3 homography to (N, 2) points."""
    h = np.concatenate([pts, np.ones((len(pts), 1))], axis=1) @ H.T
    return h[:, :2] / h[:, 2:3]


def cond(H):
    """Condition number of a homography (scale-normalized, H[2,2] = 1)."""
    Hn = H / H[2, 2]
    return float(np.linalg.cond(Hn))


def frame_squash(T):
    """Vertical and horizontal extent of a frame after transform T,
    as a multiple of the original 640x480 frame size. Returns
    (left-edge length / 480, bounding-box height / 480,
    top-edge length / 640, bounding-box width / 640)."""
    corners = np.float32([[0, 0], [W, 0], [W, H], [0, H]])
    c = apply_h(T, corners)
    height = float(np.linalg.norm(c[3] - c[0]))
    width = float(np.linalg.norm(c[1] - c[0]))
    bb_h = float(c[:, 1].max() - c[:, 1].min())
    bb_w = float(c[:, 0].max() - c[:, 0].min())
    return height / H, bb_h / H, width / W, bb_w / W


def canvas_size(Ts):
    corners = np.float32([[0, 0], [W, 0], [W, H], [0, H]]).reshape(-1, 1, 2)
    allc = np.concatenate(
        [cv2.perspectiveTransform(corners, T) for T in Ts], axis=0)
    x0, y0 = allc.min(axis=0)[0]
    x1, y1 = allc.max(axis=0)[0]
    return float(x1 - x0), float(y1 - y0), float(x0), float(y0)


def pack(Ts):
    """Free nodes (k >= 1) as an 8-vector each, H[2,2] fixed to 1."""
    vals = []
    for T in Ts[1:]:
        Tn = T / T[2, 2]
        vals.append(np.array([Tn[0, 0], Tn[0, 1], Tn[0, 2],
                              Tn[1, 0], Tn[1, 1], Tn[1, 2],
                              Tn[2, 0], Tn[2, 1]]))
    return np.concatenate(vals)


def unpack(x, n):
    Ts = [np.eye(3)]
    for k in range(n - 1):
        h = x[8 * k:8 * k + 8]
        Ts.append(np.array([[h[0], h[1], h[2]],
                            [h[3], h[4], h[5]],
                            [h[6], h[7], 1.0]]))
    return Ts


def edge_residuals(Ts, edges):
    """Stacked transfer-error residuals (pixels) for all edges.

    edges: list of (i, j, H_meas, weight). Residual for one edge is
    (T_i^-1 T_j - H_meas) applied to the 3x3 sample grid.
    """
    out = []
    for i, j, Hm, w in edges:
        pred = np.linalg.inv(Ts[i]) @ Ts[j]
        r = (apply_h(pred, GRID) - apply_h(Hm, GRID)).reshape(-1)
        out.append(w * r)
    return np.concatenate(out)


def edge_rms(Ts, edges, which=None):
    """Per-edge-group RMS transfer error in pixels."""
    vals = {}
    for tag, group in (("chain", edges[:-1]), ("loop", edges[-1:])):
        if which is not None and tag != which:
            continue
        r = edge_residuals(Ts, group)
        vals[tag] = float(np.sqrt(np.mean(r ** 2)))
    return vals


def optimize(Ts_init, edges, max_iter=60, tol=1e-7):
    """Levenberg-Marquardt over the free nodes (pure numpy, numeric
    Jacobian). Returns (Ts, info)."""
    n = len(Ts_init)
    x = pack(Ts_init)
    r0 = edge_residuals(Ts_init, edges)
    cost = float(r0 @ r0)
    lam = 1e-3
    iters = 0
    converged = False
    for it in range(max_iter):
        iters = it + 1
        Ts = unpack(x, n)
        r = edge_residuals(Ts, edges)
        cost = float(r @ r)
        # Forward-difference Jacobian.
        J = np.empty((len(r), len(x)))
        for c in range(len(x)):
            step = 1e-6 * max(1.0, abs(x[c]))
            xp = x.copy()
            xp[c] += step
            J[:, c] = (edge_residuals(unpack(xp, n), edges) - r) / step
        # LM step: solve [J; sqrt(lam) I] dx = -[r; 0].
        A = np.vstack([J, np.sqrt(lam) * np.eye(len(x))])
        b = np.concatenate([-r, np.zeros(len(x))])
        dx, *_ = np.linalg.lstsq(A, b, rcond=None)
        x_new = x + dx
        r_new = edge_residuals(unpack(x_new, n), edges)
        cost_new = float(r_new @ r_new)
        if cost_new < cost:
            rel = (cost - cost_new) / max(cost, 1e-12)
            x, cost = x_new, cost_new
            lam = max(lam * 0.3, 1e-9)
            if rel < tol:
                converged = True
                break
        else:
            lam *= 10.0
            if lam > 1e9:
                break
    Ts = unpack(x, n)
    return Ts, {"iterations": iters, "converged": converged,
                "final_cost": cost,
                "final_rms_px": float(np.sqrt(cost / len(r0))),
                "lambda": lam}


def render(frames, Ts, out_path, max_side=4000):
    """Same warp-and-average accumulation as stitch_map.stitch.

    Returns (width, height) or None when the canvas is too large.
    """
    bw, bh, x0, y0 = canvas_size(Ts)
    if bw > max_side or bh > max_side:
        print(f"render skipped: canvas {bw:.0f} x {bh:.0f} px too large",
              flush=True)
        return None
    cw, ch = int(np.ceil(bw)), int(np.ceil(bh))
    shift = np.array([[1, 0, -x0], [0, 1, -y0], [0, 0, 1.0]])
    acc = np.zeros((ch, cw, 3), dtype=np.float64)
    cover = np.zeros((ch, cw), dtype=np.float64)
    ones = np.ones((H, W), dtype=np.float64)
    for f, T in zip(frames, Ts):
        M = shift @ T
        warped = cv2.warpPerspective(f, M, (cw, ch))
        mask = cv2.warpPerspective(ones, M, (cw, ch))
        acc += warped.astype(np.float64) * mask[..., None]
        cover += mask
    cover[cover == 0] = 1
    cv2.imwrite(out_path, (acc / cover[..., None]).astype(np.uint8))
    print(f"wrote {out_path} {cw}x{ch}", flush=True)
    return (cw, ch)


def render_preview(frames, Ts, out_path, long_side=2000):
    """Downscaled preview of a too-large canvas, for visual judgement
    only. The transforms are scaled, not recomputed."""
    bw, bh, x0, y0 = canvas_size(Ts)
    s = long_side / max(bw, bh)
    S = np.array([[s, 0, 0], [0, s, 0], [0, 0, 1.0]])
    Ts_s = [S @ T for T in Ts]
    return render(frames, Ts_s, out_path, max_side=long_side + 2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out-dir", required=True)
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    frames = stitch_map.load_frames_from_bag(args.bag)
    print(f"loaded {len(frames)} frames from bag", flush=True)
    detector = stitch_map.make_detector("sift")
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    chosen = list(range(0, len(frames), STRIDE))
    feats = {idx: detector.detectAndCompute(
        cv2.cvtColor(stitch_map.downscale(frames[idx]),
                     cv2.COLOR_BGR2GRAY), None) for idx in chosen}
    n = len(chosen)
    print(f"{n} chosen frames (stride {STRIDE})", flush=True)

    # Chain edges: H_k maps frame k into frame k-1 (stitch_map convention).
    chain_H, chain_inliers, chain_cond = [], [], []
    skipped = 0
    for k in range(1, n):
        Hm, inl = stitch_map.pair_homography(
            feats[chosen[k - 1]], feats[chosen[k]], matcher,
            ratio=0.75, normalize=True)
        if Hm is None:
            skipped += 1
            chain_H.append(np.eye(3))
            chain_inliers.append(0)
            chain_cond.append(1.0)
        else:
            chain_H.append(Hm)
            chain_inliers.append(inl)
            chain_cond.append(cond(Hm))
        if k % 20 == 0:
            print(f"chain pair {k}/{n - 1}", flush=True)
    print(f"chain pairs: skipped {skipped}, inliers median "
          f"{int(np.median(chain_inliers))} min {min(chain_inliers)}",
          flush=True)
    print(f"chain H condition numbers: median {np.median(chain_cond):.2e} "
          f"max {max(chain_cond):.2e}", flush=True)

    Ts_chain = [np.eye(3)]
    for Hm in chain_H:
        Ts_chain.append(Ts_chain[-1] @ Hm)
    bw0, bh0, _, _ = canvas_size(Ts_chain)
    print(f"normalized chain canvas: {bw0:.0f} x {bh0:.0f} px", flush=True)

    # Loop edge: last 5 chosen frames vs first 5, best by inliers.
    loop = []
    for a in range(5):
        for b in range(n - 5, n):
            Hm, inl = stitch_map.pair_homography(
                feats[chosen[a]], feats[chosen[b]], matcher,
                ratio=0.75, normalize=True)
            if Hm is not None:
                loop.append((inl, a, b, Hm))
    loop_inliers = sorted([x[0] for x in loop], reverse=True)
    inl, a, b, H_loop = max(loop, key=lambda x: x[0])
    loop_cond = cond(H_loop)
    print(f"loop pairs: {len(loop)} ok, inliers {loop_inliers}", flush=True)
    print(f"best loop pair: chosen frame {b} -> chosen frame {a}, "
          f"inliers {inl}, cond {loop_cond:.2e}", flush=True)
    print(f"loop H maps last frame onto first frame; chain end squash "
          f"(left-edge/480, bbox-h/480, top-edge/640, bbox-w/640) = "
          f"{tuple(round(v, 3) for v in frame_squash(Ts_chain[-1]))}",
          flush=True)
    chain_T_cond = [cond(T) for T in Ts_chain[1:]]
    print(f"chained-transform condition numbers: median "
          f"{np.median(chain_T_cond):.2e} max {max(chain_T_cond):.2e}",
          flush=True)

    # Correction size, for comparison with the linear-distribution run.
    # Chain-implied relative homography mapping frame b into frame a is
    # T_a^-1 @ T_b; D measures how far the measured loop edge is from it.
    H_chain_ab = np.linalg.inv(Ts_chain[a]) @ Ts_chain[b]
    D = H_loop @ np.linalg.inv(H_chain_ab)
    D = D / D[2, 2]
    print(f"loop error D (H_loop vs chain-implied) Frobenius norm "
          f"{np.linalg.norm(D - np.eye(3)):.1f}", flush=True)

    chain_edges = [(k - 1, k, chain_H[k - 1], 1.0)
                   for k in range(1, n)]
    rms_before = edge_rms(Ts_chain, chain_edges + [(a, b, H_loop, 1.0)])
    print(f"transfer-error RMS before optimization: chain "
          f"{rms_before['chain']:.2f} px, loop {rms_before['loop']:.2f} px",
          flush=True)

    results = {}
    for w_loop, tag in ((1.0, "w1"), (10.0, "w10")):
        edges = chain_edges + [(a, b, H_loop, w_loop)]
        Ts_opt, info = optimize(Ts_chain, edges)
        bw, bh, _, _ = canvas_size(Ts_opt)
        rms = edge_rms(Ts_opt, chain_edges + [(a, b, H_loop, 1.0)])
        sq = frame_squash(Ts_opt[-1])
        print(f"[{tag}] loop weight {w_loop}: iters {info['iterations']} "
              f"converged {info['converged']} final RMS "
              f"{info['final_rms_px']:.2f} px; canvas {bw:.0f} x {bh:.0f} px; "
              f"chain RMS {rms['chain']:.2f} px, loop RMS {rms['loop']:.2f} px; "
              f"last-frame squash (left-edge/480, bbox-h/480, "
              f"top-edge/640, bbox-w/640) = "
              f"{tuple(round(v, 2) for v in sq)}",
              flush=True)
        results[tag] = {"loop_weight": w_loop, "info": info,
                        "canvas_px": [round(bw, 1), round(bh, 1)],
                        "chain_rms_px": rms["chain"],
                        "loop_rms_px": rms["loop"],
                        "last_frame_squash": [round(v, 3) for v in sq]}
        if tag == "w1":
            Ts_w1 = Ts_opt
            np.save(os.path.join(args.out_dir, "Ts_posegraph_w1.npy"),
                    np.array(Ts_opt))

    # Render the primary (loop weight 1) result.
    out_png = os.path.join(args.out_dir, "map_posegraph_080033.png")
    got = render(frames, Ts_w1, out_png)
    rendered = "full"
    if got is None:
        got = render_preview(frames, Ts_w1, out_png)
        rendered = "preview-downscaled"
    results["rendered"] = rendered
    results["rendered_size_px"] = list(got) if got else None

    stats = {
        "bag": os.path.basename(args.bag),
        "stride": STRIDE, "n_chosen_frames": n,
        "pairs_skipped": skipped,
        "pair_inliers_median": int(np.median(chain_inliers)),
        "pair_inliers_min": int(min(chain_inliers)),
        "chain_H_cond_median": float(np.median(chain_cond)),
        "chain_H_cond_max": float(max(chain_cond)),
        "chained_T_cond_median": float(np.median(chain_T_cond)),
        "chained_T_cond_max": float(max(chain_T_cond)),
        "chain_canvas_px": [round(bw0, 1), round(bh0, 1)],
        "loop_pair_inliers": loop_inliers,
        "best_loop": {"first_chosen_frame": a, "last_chosen_frame": b,
                      "inliers": int(inl), "cond": loop_cond},
        "loop_error_D_fro": float(np.linalg.norm(D - np.eye(3))),
        "rms_before_px": rms_before,
        "last_frame_squash_chain": [round(v, 3)
                                    for v in frame_squash(Ts_chain[-1])],
        "posegraph": results,
    }
    with open(os.path.join(args.out_dir, "posegraph_080033.json"),
              "w") as f:
        json.dump(stats, f, indent=2)
    print(json.dumps(stats, indent=2))
    print("done", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
