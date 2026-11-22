"""Plot signed IMU and RGB-D translation for recordings that start from rest."""

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np

from item5_analysis import load_shift_ms
from posthoc import onset_scores, series


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("extract_dir", type=Path)
    parser.add_argument("timing", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()

    clips = []
    for path in sorted(args.extract_dir.glob("*.npz")):
        shift = load_shift_ms(args.timing, path.stem)
        _, rest, motion = series(path, shift)
        if motion is not None:
            clips.append((path.stem.rsplit("_", 1)[-1], rest, motion))
    if not clips:
        parser.error("no verified-rest recordings found")

    rgbd_color, imu_color, error_color = "#2166A6", "#B64A18", "#70448C"
    fig, axes = plt.subplots(len(clips), 4, figsize=(12, 2.1 * len(clips) + 1.45),
                             sharex=True, squeeze=False)
    for (clip, rest, motion), row in zip(clips, axes):
        t = motion["t"]
        keep = t <= 6.05
        vis, imu = motion["vis"], motion["imu"]
        onset = onset_scores(motion)["onset_s"]
        for axis, panel in enumerate(row[:3]):
            panel.plot(t[keep], vis[keep, axis], color=rgbd_color, lw=2)
            panel.plot(t[keep], imu[keep, axis], color=imu_color, lw=1.8, ls="--")
            panel.axhline(0, color="#CBD2DC", lw=0.7, zorder=0)
        error = np.linalg.norm(imu - vis, axis=1)
        row[3].plot(t[keep], error[keep], color=error_color, lw=2)
        row[3].set_ylim(bottom=0)
        row[0].set_ylabel(f"{clip}\n{rest['visual_s']:.2f} s at rest", fontsize=10)
        for panel in row:
            panel.axvspan(0, 1, color="#E8EDF3", zorder=0)
            if onset is not None:
                panel.axvline(onset, color="#535F70", ls=":", lw=1.3)
            panel.set_xlim(0, 6)
            panel.set_xticks(range(7))
            panel.grid(axis="y", color="#DFE4EA", lw=0.65)
            panel.spines[["top", "right"]].set_visible(False)
            panel.tick_params(labelsize=9)

    for panel, title in zip(axes[0], ("X (m)", "Y (m)", "Z (m)", "Position difference (m)")):
        panel.set_title(title, fontsize=11, loc="left")
    fig.suptitle("IMU and RGB-D translation from a verified stationary start",
                 x=0.10, ha="left", y=0.98, fontsize=17, fontweight="bold")
    legend = [Line2D([], [], color=rgbd_color, lw=2, label="RGB-D"),
              Line2D([], [], color=imu_color, lw=2, ls="--", label="IMU"),
              Line2D([], [], color=error_color, lw=2, label="IMU–RGB-D difference"),
              Patch(color="#E8EDF3", label="First second after rest window"),
              Line2D([], [], color="#535F70", ls=":", lw=1.3,
                     label="RGB-D displacement reaches 1 cm")]
    fig.legend(handles=legend, loc="upper left", bbox_to_anchor=(0.093, 0.928),
               ncol=len(legend), frameon=False, fontsize=9)
    fig.supxlabel("Time since the verified rest window ended (s)", y=0.025, fontsize=10)
    fig.tight_layout(rect=(0.025, 0.065, 0.99, 0.86), h_pad=1.2, w_pad=1.65)
    fig.savefig(args.output, dpi=175)
    plt.close(fig)


if __name__ == "__main__":
    main()
