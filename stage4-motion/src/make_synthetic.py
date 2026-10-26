"""
Synthetic lawn video generator for CS566 Stage IV (Motion / Visual Odometry).

Clean pinhole model: downward-facing camera (like the PX4FLOW / optical-flow
sensor discussed in the proposal) at fixed height over a textured lawn with
3D boxes (objects on the lawn). Camera moves forward with a gentle weave and
yaw, giving pure optical-flow-style motion.

Saves (into --out):
  frame_%04d.png      - rendered frames (lossless PNG; mp4 artifacts hurt flow)
  groundtruth.npz     - K, per-frame camera centers (N,3),
                        world->camera rotations (N,3,3)

Conventions: world x=right, y=forward, z=up. Ground plane z=0.

Usage:
  python make_synthetic.py --profile gentle --out data/gentle
  python make_synthetic.py --profile aggressive --out data/aggressive
  python make_synthetic.py --profile gentle --stripes off --out data/plain
  python make_synthetic.py --profile gentle --stripes on --stripe-strength 3.0 \\
      --seed 7 --out data/striped
"""
import argparse
import os

import numpy as np
import cv2

ap = argparse.ArgumentParser(
    description="Generate a synthetic downward-camera lawn sequence.")
ap.add_argument("--out", default=None,
                help="output directory for frames + groundtruth.npz "
                     "(default: data/<profile>/, or data/ with no profile)")
ap.add_argument("--profile", choices=["gentle", "aggressive"], default=None,
                help="motion profile: gentle (weave 0.15) or aggressive (weave 0.8)")
ap.add_argument("--weave", type=float, default=None,
                help="lateral weave amplitude in meters (overrides --profile and WEAVE_AMP)")
ap.add_argument("--stripes", choices=["on", "off"], default="on",
                help="render mowing stripes (default: on)")
ap.add_argument("--stripe-strength", type=float, default=1.0,
                help="multiplier on stripe brightness delta (default: 1.0)")
ap.add_argument("--seed", type=int, default=7,
                help="rng seed; same seed+profile => same trajectory/texture/boxes")
ap.add_argument("--frames", type=int, default=240,
                help="number of frames to render (default: 240)")
args = ap.parse_args()

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if args.out:
    OUT = args.out
elif args.profile:
    OUT = os.path.join(BASE, "data", args.profile)
else:
    OUT = os.path.join(BASE, "data")  # legacy default location
os.makedirs(OUT, exist_ok=True)

# ---------------- Camera ----------------
W, H = 640, 480
FX = FY = 520.0
CX, CY = 320.0, 240.0
K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1.0]])
CAM_H = 1.0  # height above lawn (m)

# ---------------- Lawn texture ----------------
TEX_SIZE = 2048
WORLD_SIZE = 30.0
PX_PER_M = TEX_SIZE / WORLD_SIZE
rng = np.random.default_rng(args.seed)

tex = np.zeros((TEX_SIZE, TEX_SIZE, 3), dtype=np.float32)
tex[..., 0], tex[..., 1], tex[..., 2] = 45, 110, 55  # BGR lawn green
for _ in range(250):  # soft blotches (absolute varied greens)
    x, y = rng.integers(0, TEX_SIZE, 2)
    r = int(rng.integers(40, 160))
    s = float(rng.uniform(-25, 25))
    cv2.circle(tex, (int(x), int(y)), r,
               (45 + s * 0.4, 110 + s, 55 + s * 0.5), -1)
if args.stripes == "on":  # mowing stripes along Y
    g = float(args.stripe_strength)
    for i in range(0, TEX_SIZE, 128):
        if (i // 128) % 2 == 0:
            tex[:, i:i + 64] += (6 * g, 18 * g, 6 * g)
tex += rng.normal(0, 9, tex.shape).astype(np.float32)  # grain
for _ in range(4000):  # speckles (absolute varied greens)
    x, y = rng.integers(0, TEX_SIZE, 2)
    c = float(rng.uniform(-40, 40))
    cv2.circle(tex, (int(x), int(y)), int(rng.integers(1, 4)),
               (45 + c * 0.4, 110 + c, 55 + c * 0.5), -1)
for _ in range(60):  # lawn marks -> strong corners
    x, y = rng.integers(0, TEX_SIZE, 2)
    col = (230, 230, 230) if rng.random() < 0.5 else (40, 220, 230)
    k = rng.integers(0, 3)
    if k == 0:
        cv2.circle(tex, (int(x), int(y)), int(rng.integers(4, 10)), col, -1)
    elif k == 1:
        s = int(rng.integers(6, 16))
        cv2.rectangle(tex, (int(x) - s, int(y) - s), (int(x) + s, int(y) + s), col, -1)
    else:
        cv2.line(tex, (int(x), int(y)),
                 (int(x + rng.integers(-30, 30)), int(y + rng.integers(-30, 30))), col, 3)
tex = np.clip(tex, 0, 255).astype(np.uint8)

# ---------------- 3D boxes ----------------
boxes = []
for _ in range(14):
    boxes.append(dict(x=float(rng.uniform(-6, 6)), y=float(rng.uniform(-2, 10)),
                      sx=float(rng.uniform(0.25, 0.6)), sy=float(rng.uniform(0.25, 0.6)),
                      h=float(rng.uniform(0.15, 0.7)),
                      color=tuple(int(c) for c in rng.integers(60, 255, 3))))

def box_corners(b):
    x, y, sx, sy, h = b["x"], b["y"], b["sx"], b["sy"], b["h"]
    return np.array([[x + dx, y + dy, z]
                     for dx in (-sx / 2, sx / 2) for dy in (-sy / 2, sy / 2)
                     for z in (0, h)])

FACES = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4),
         (2, 3, 7, 6), (1, 3, 7, 5), (0, 2, 6, 4)]

# ---------------- Camera path ----------------
# Lateral weave amplitude: --weave > --profile > WEAVE_AMP env > 0.8 default.
# gentle profile = 0.15, aggressive profile = 0.8 (matches prior WEAVE_AMP use).
if args.weave is not None:
    _WEAVE = float(args.weave)
elif args.profile == "gentle":
    _WEAVE = 0.15
elif args.profile == "aggressive":
    _WEAVE = 0.8
else:
    _WEAVE = float(os.environ.get("WEAVE_AMP", "0.8"))
N = int(args.frames)
t = np.arange(N, dtype=float)
py = 0.035 * t                              # forward ~8.4 m at 240 frames
px = _WEAVE * np.sin(0.05 * t) + 0.15 * _WEAVE * np.sin(0.13 * t)  # weave
pz = np.full(N, CAM_H)
centers = np.stack([px, py, pz], axis=1)

dpy = np.gradient(py)
dpx = np.gradient(px)
yaw = np.arctan2(dpx, dpy)                  # heading of travel

R_base = np.array([[1., 0, 0], [0, -1, 0], [0, 0, -1]])  # nadir: cam z = -world z
Rcw_list = []
for yw in yaw:
    cy, sy = np.cos(yw), np.sin(yw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Rcw_list.append(Rz @ R_base)             # world -> camera
Rcw_list = np.array(Rcw_list)

# ---------------- Render ----------------
# Lossless PNG sequence (mp4v compression artifacts hurt optical flow).
# Frames live directly in OUT as frame_%04d.png (matches prior data layout).
s = PX_PER_M
M = np.array([[1 / s, 0, -WORLD_SIZE / 2],
              [0, 1 / s, -WORLD_SIZE / 2],
              [0, 0, 0]])                   # texture px -> world (x, y, 0)
n = np.array([0., 0, 1])                    # ground normal

for i in range(N):
    R = Rcw_list[i]
    C = centers[i]
    tc = -R @ C
    # plane-induced homography, texture -> image: p ~ K (R M [u,v,1] + tc)
    Hm = K @ (R @ M)
    Hm[:, 2] += K @ tc
    Hm /= Hm[2, 2]
    frame = cv2.warpPerspective(tex, Hm, (W, H), flags=cv2.INTER_LINEAR,
                                borderMode=cv2.BORDER_CONSTANT,
                                borderValue=(45, 110, 55))
    # boxes, back to front
    for b in sorted(boxes, key=lambda b: -np.linalg.norm(
            np.array([b["x"], b["y"], 0.0]) - C)):
        pc = (R @ (box_corners(b) - C).T).T
        if np.any(pc[:, 2] < 0.2):
            continue
        pp = (K @ pc.T).T
        pp = (pp[:, :2] / pp[:, 2:3]).astype(np.int32)
        for f in FACES:
            shade = 0.75 + 0.25 * (f[0] % 2)
            col = tuple(int(min(255, c * shade)) for c in b["color"])
            cv2.fillConvexPoly(frame, pp[list(f)], col)
        cv2.polylines(frame, [pp[[1, 3, 7, 5]]], True, (20, 20, 20), 1)

    frame = np.clip(frame.astype(np.float32) +
                    rng.normal(0, 2.0, frame.shape).astype(np.float32),
                    0, 255).astype(np.uint8)
    cv2.imwrite(os.path.join(OUT, f"frame_{i:04d}.png"), frame)

np.savez(os.path.join(OUT, "groundtruth.npz"), K=K,
         positions=centers, rotations=Rcw_list.transpose(0, 2, 1),  # R_wc
         width=W, height=H, fps=30.0, frames_dir=os.path.basename(OUT) or ".")
print(f"wrote {N} frames to {OUT}; path length "
      f"{np.linalg.norm(np.diff(centers, axis=0), axis=1).sum():.2f} m")
