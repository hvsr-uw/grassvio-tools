"""Show SfM scene points and compare camera paths in the first-camera frame."""

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import proj3d
import numpy as np


def read_colored_points(path):
    with path.open(encoding="ascii") as file:
        for line in file:
            if line.strip() == "end_header":
                break
        else:
            raise ValueError(f"missing PLY header terminator in {path}")
        vertices = np.loadtxt(file, ndmin=2)
    if not len(vertices) or vertices.shape[1] != 6:
        raise ValueError("expected ASCII PLY vertices with x, y, z, red, green and blue")
    return vertices[:, :3], vertices[:, 3:] / 255


def render(sfm_path, cloud_path, orb_path, output):
    sfm = json.loads(sfm_path.read_text(encoding="utf-8"))
    orb = json.loads(orb_path.read_text(encoding="utf-8"))
    frames = sfm["registered_frames"]
    path = np.asarray(sfm["poses_camera_to_world"])[:, :3, 3]
    reference_path = np.asarray(orb["absolute_poses"])[frames, :3, 3]
    origin = path[0]
    if frames[0] != 0 or not np.allclose([origin, reference_path[0]], 0.):
        raise ValueError("both camera paths must start at the frame 0 origin")
    points, colors = read_colored_points(cloud_path)
    clip = Path(sfm["recording"]).stem.split("_")[-1]
    blue, orange, gray = "#24599a", "#db581b", "#727987"

    fig = plt.figure(figsize=(13.2, 6.6))
    fig.subplots_adjust(left=.045, right=.975, bottom=.15, top=.84, wspace=.28)
    fig.suptitle(f"{clip} | Structure from motion", y=.96, fontsize=21, weight="bold")

    scene = fig.add_subplot(121, projection="3d")
    scene.set_title("Reconstructed scene and camera path", fontsize=14, pad=12)
    scene.scatter(*points.T, color=colors, s=8, depthshade=False, alpha=.75)
    scene.plot(*path.T, color=blue, marker=".", markersize=4, linewidth=1.8)
    scene.scatter(*origin, s=72, color=orange, edgecolor="white", linewidth=1.25, depthshade=False)
    depth_guide = np.median(points[:, 2])
    scene.plot([origin[0]] * 2, [origin[1]] * 2, [origin[2], depth_guide],
               linestyle="--", color=orange, linewidth=1.2, alpha=.8)
    scene.set(xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)")
    scene.set_box_aspect(np.ptp(np.concatenate((points, path), axis=0), axis=0))
    scene.view_init(elev=21, azim=-69)

    motion = fig.add_subplot(122)
    motion.set_title("Camera paths in the X–Z plane", fontsize=14, pad=12)
    motion.plot(*(reference_path[:, [0, 2]].T * 100), color=gray, label="ORB-SLAM3 RGB-D",
                linestyle="--", linewidth=1.2, marker="o", markersize=3.6)
    motion.plot(*(path[:, [0, 2]].T * 100), color=blue, label="SfM",
                linewidth=1.6, marker="o", markersize=3.6)
    motion.scatter(*(origin[[0, 2]] * 100), s=62, color=orange, edgecolor="white", zorder=5)
    motion.annotate(f"Frame {frames[0]}", origin[[0, 2]] * 100, xytext=(9, 8),
                    textcoords="offset points", color=orange, weight="bold")
    motion.annotate(f"Frame {frames[-1]}", path[-1, [0, 2]] * 100, xytext=(7, -17),
                    textcoords="offset points", color=blue)
    motion.set(xlabel="X (cm)", ylabel="Z (cm)", aspect="equal")
    motion.grid(alpha=.26)
    motion.margins(x=.17, y=.10)
    motion.legend(loc="lower left", bbox_to_anchor=(0.012, .025), framealpha=.95)

    fig.canvas.draw()
    x, y, _ = proj3d.proj_transform(*origin, scene.get_proj())
    scene.annotate(f"Frame {frames[0]}\n(0, 0, 0) m", (x, y), xytext=(-60, -19),
                   textcoords="offset points", ha="right", va="center", color=orange,
                   weight="bold", arrowprops={"arrowstyle": "-", "color": orange, "linewidth": 1.3})
    fig.text(.06, .047, "Positive Z is forward from the starting camera. The dashed orange line marks that direction.",
             fontsize=10, color="#3a4048")
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=170, facecolor="white")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sfm", type=Path, required=True)
    parser.add_argument("--cloud", type=Path, required=True)
    parser.add_argument("--orb", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    render(args.sfm, args.cloud, args.orb, args.output)


if __name__ == "__main__":
    main()
