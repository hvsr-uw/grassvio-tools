"""Plot world-frame paths or per-recording SfM residuals."""

import argparse
import csv
import json
from pathlib import Path
from textwrap import fill

import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np

from plot_sfm_reconstruction import read_colored_points

BLUE = "#235e9a"
ORANGE = "#a65021"
PURPLE = "#77549a"
GRAY = "#607183"
AXES = "XYZ"


def plot_world(path, output):
    data = json.loads(path.read_text(encoding="utf-8"))
    inputs = data["world_frame_inputs"]
    frames = data["registered_frames"]
    sfm = np.asarray(data["sfm_poses_camera_to_world"])[:, :3, 3]
    rgbd_poses = data["rgbd_poses_camera_to_world"]
    rgbd = np.asarray([np.asarray(pose)[:3, 3] if pose is not None else [np.nan] * 3
                       for pose in rgbd_poses]) if rgbd_poses is not None else None
    points_file = path.with_name("world-points.ply")
    clip = Path(data["recording"]).stem.rsplit("_", 1)[-1]
    preview = bool(inputs.get("preview"))
    fig = plt.figure(figsize=(12, 7.5), layout="constrained")
    grid = fig.add_gridspec(2, 3, height_ratios=[1.25, 1])
    scene = fig.add_subplot(grid[0, :], projection="3d")
    if points_file.exists():
        points, colors = read_colored_points(points_file)
        scene.scatter(*points.T, color=colors, s=5, alpha=.6, depthshade=False)
    scene.plot(*sfm.T, color=BLUE, marker=".", label="SfM")
    if rgbd is not None:
        scene.plot(*rgbd.T, color=GRAY, linestyle="--", label=data["rgbd_reference"])
    scene.plot([0, 0], [0, 0], [0, sfm[0, 2]], color=ORANGE, linestyle=":", linewidth=1.4)
    scene.scatter(0, 0, 0, marker="x", color=ORANGE, s=45, label="Ground origin")
    scene.scatter(*sfm[0], color=ORANGE, s=25)
    scene.set(xlabel="World X (m)", ylabel="World Y (m)", zlabel="World Z (m)")
    scene.view_init(elev=24, azim=-63)
    scene.legend(loc="upper left", bbox_to_anchor=(-.37, .94), frameon=False)
    provenance = ("Preview inputs\n" + inputs["gravity_source"] + "\n" + inputs["feature_depth_source"]
                  if preview else f"Frame 0 height: {sfm[0, 2]:.3f} m\n{len(frames)} registered camera views")
    fig.text(.745, .75, "\n".join(fill(line, width=35) for line in provenance.splitlines()),
             va="top", color=GRAY, fontsize=9)
    combined = np.concatenate((sfm, rgbd)) if rgbd is not None else sfm
    low = np.nanmin(combined, axis=0) * 100
    high = np.nanmax(combined, axis=0) * 100
    mid = (low + high) / 2
    radius = max((high - low).max() * .61, 1)

    for col, (i, j) in enumerate(((0, 1), (0, 2), (1, 2))):
        ax = fig.add_subplot(grid[1, col])
        ax.plot(sfm[:, i] * 100, sfm[:, j] * 100, color=BLUE, marker=".", linewidth=1.8)
        if rgbd is not None:
            ax.plot(rgbd[:, i] * 100, rgbd[:, j] * 100, color=GRAY, linestyle="--", linewidth=1.5)
        ax.scatter(sfm[0, i] * 100, sfm[0, j] * 100, color=ORANGE, s=32, zorder=3)
        ax.annotate("Frame 0", sfm[0, [i, j]] * 100, xytext=(4, 7),
                    textcoords="offset points", color=ORANGE, fontsize=8)
        ax.set(xlabel=f"World {AXES[i]} (cm)", ylabel=f"World {AXES[j]} (cm)", aspect="equal",
               title=f"{AXES[i]}–{AXES[j]} camera path")
        ax.grid(alpha=.25)
        ax.set_xlim(mid[i] - radius, mid[i] + radius)
        ax.set_ylim(mid[j] - radius, mid[j] + radius)
    title = "World-frame preview" if preview else "SfM in the gravity-aligned world frame"
    fig.suptitle(f"{clip}  |  {title}\nX horizontal forward · Y left · Z up",
                 fontsize=15, fontweight="bold")
    fig.savefig(output, dpi=165, facecolor="white")
    plt.close(fig)


def plot_residuals(path, output):
    with path.open(newline="", encoding="utf-8-sig") as file:
        rows = list(csv.DictReader(file))
    names = sorted({row["recording"] for row in rows})
    if not names:
        raise ValueError("the residual CSV is empty")

    groups = (
        ("Translation difference (mm)", 1000, (
            ("translation_vector_difference_m", "Vector", BLUE),
            ("translation_magnitude_difference_m", "Magnitude", ORANGE))),
        ("Translation direction difference (°)", 1, (
            ("translation_direction_difference_deg", "SfM vs. RGB-D", BLUE),)),
        ("Rotation difference (°)", 1, (
            ("sfm_rgbd_rotation_deg", "SfM vs. RGB-D", BLUE),
            ("sfm_imu_rotation_deg", "SfM vs. IMU gyro", PURPLE))),
    )
    with PdfPages(output) as pdf:
        for name in names:
            clip = sorted((row for row in rows if row["recording"] == name),
                          key=lambda row: int(row["source_frame"]))
            x = [float(row["start_s"]) + float(row["duration_s"]) / 2 for row in clip]
            common = sum(bool(row["sfm_rgbd_rotation_deg"] and row["sfm_imu_rotation_deg"]) for row in clip)
            fig, axs = plt.subplots(3, 1, figsize=(9.5, 6.6), sharex=True)
            fig.subplots_adjust(left=.13, right=.97, bottom=.14, top=.83, hspace=.48)
            fig.suptitle(f"{name.rsplit('_', 1)[-1]}  |  Motion differences over time",
                         x=.13, ha="left", y=.96, fontsize=17, fontweight="bold")
            fig.text(.13, .895, f"{common} of {len(clip)} later intervals have all three estimates.",
                     color=GRAY, fontsize=10)
            for ax, (label, factor, series) in zip(axs, groups, strict=True):
                for key, title, color in series:
                    values = [factor * float(row[key]) if row[key] else np.nan for row in clip]
                    ax.plot(x, values, color=color, marker="o", markersize=3.3, linewidth=1.4, label=title)
                ax.set_title(label, loc="left", fontsize=10, weight="bold")
                ax.set_ylim(bottom=0)
                ax.grid(axis="y", alpha=.25)
                ax.spines[["top", "right"]].set_visible(False)
                if len(series) > 1:
                    ax.legend(loc="upper right", frameon=False, ncol=len(series), fontsize=8)
            axs[-1].set_xlabel("Time from frame 0 (s), measured at the center of each interval")
            fig.text(.13, .039, "Direction requires both visual displacements to exceed 5 mm. "
                     "Gaps show unavailable values.", color=GRAY, fontsize=8)
            fig.text(.13, .015, "These curves compare estimates on identical intervals.",
                     color=GRAY, fontsize=8)
            pdf.savefig(fig)
            plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--world-frame", type=Path)
    group.add_argument("--residuals", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.world_frame is not None:
        plot_world(args.world_frame, args.output)
    else:
        plot_residuals(args.residuals, args.output)


if __name__ == "__main__":
    main()
