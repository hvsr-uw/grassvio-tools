#!/usr/bin/env python3
"""Make matched map crops and an original-frame comparison for a plane panorama."""

import argparse
from contextlib import closing
import csv
import json
from pathlib import Path
import sys

import cv2
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'stitch-map'))
from recording_camera import COLOR, Recording, project


def load_image(path, kind=cv2.IMREAD_COLOR):
    image = cv2.imread(str(path), kind)
    if image is None:
        raise OSError(f'could not open {path}')
    return image


def laplacian_rms(image, mask):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detail = cv2.Laplacian(gray, cv2.CV_32F)[mask]
    return float(np.sqrt(np.mean(detail**2)))


def show(axis, bgr, title):
    axis.imshow(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    axis.set_title(title, loc='left', fontsize=12)
    axis.set_axis_off()


def crop_near(owner, target, size):
    half = size // 2
    support = cv2.boxFilter((owner > 0).astype(np.float32), -1, (size, size),
                            normalize=False, borderType=cv2.BORDER_CONSTANT)
    yy, xx = np.where(support >= size**2 - .5)
    if not len(xx):
        raise ValueError('no fully covered ROI of the requested size')
    winner = np.argmin((xx - owner.shape[1] * target[0])**2 + (yy - owner.shape[0] * target[1])**2)
    return int(xx[winner] - half), int(yy[winner] - half), size, size


def projected_source(recording, stats, owner_crop, box):
    x, y, w, h = box
    width, height = recording.color['width'], recording.color['height']
    corners = np.array([[x, y], [x + w - 1, y], [x + w - 1, y + h - 1], [x, y + h - 1]])
    corners += stats['origin']
    indices, counts = np.unique(owner_crop[owner_crop > 0], return_counts=True)
    for rank in np.argsort(-counts):
        index = int(indices[rank]) - 1
        forward = np.asarray(stats['map_to_camera'][str(index)])
        camera = np.column_stack((corners, np.ones(4))) @ forward.T
        if np.any(camera[:, 2] <= .05):
            continue
        uv = project(camera, recording.color)
        if np.all(uv >= 0) and np.all(uv < (width, height)):
            return index, uv, float(counts[rank] / owner_crop.size)
    raise ValueError(f'no source frame contains the full crop at {(x, y)}')


def map_crops(recording, inputs, output, geometry_only):
    stats = json.loads((inputs / 'metrics.json').read_text(encoding='utf-8'))
    rendered = load_image(inputs / 'panorama.png')
    average = load_image(inputs / 'average.png')
    hard = load_image(inputs / 'hard_joins.png')
    owner = load_image(inputs / 'owners.png', cv2.IMREAD_UNCHANGED)
    observations = load_image(inputs / 'observations.png', cv2.IMREAD_UNCHANGED)
    comparison = average
    comparison_title = 'Average of overlapping frames'
    if geometry_only:
        comparator = json.loads((geometry_only / 'metrics.json').read_text(encoding='utf-8'))
        if any(comparator[key] != stats[key] for key in ('size', 'origin', 'map_to_camera')):
            raise ValueError('the geometry-only comparison must use the same map grid and warps')
        comparison = load_image(geometry_only / 'panorama.png')
        comparison_title = 'Geometry-only source selection'
        before_owner = load_image(geometry_only / 'owners.png', cv2.IMREAD_UNCHANGED)
        if not np.array_equal(before_owner > 0, owner > 0):
            raise ValueError('the geometry-only comparison has different coverage')
        detail = stats['source_laplacian_rms']
        threshold = .65 * np.median(list(detail.values()))
        soft = [int(frame) + 1 for frame, value in detail.items() if value < threshold]
        stats['blur_ablation'] = dict(
            frames=[i - 1 for i in soft], cutoff_rms=float(threshold),
            geometry_only_pixels=int(np.count_nonzero(np.isin(before_owner, soft))),
            blur_aware_pixels=int(np.count_nonzero(np.isin(owner, soft))),
        )
    name = Path(stats['recording']).stem.rsplit('_', 1)[-1]

    fig, axes = plt.subplots(2, 3, figsize=(10.9, 7.3))
    fig.subplots_adjust(left=.035, right=.975, bottom=.08, top=.88, wspace=.09, hspace=.22)
    fig.suptitle(f'{name} — original frames and matching panorama crops', fontsize=16, y=.98)
    fig.text(.05, .028, 'Boxes mark the same ground patch in a contributing RGB frame. Map crops use the same pixel scale.',
             fontsize=9, color='#4b5661')
    colors = ('#eea336', '#55bdc8')
    boxes = []
    for row, target, color in zip(axes, ((.50, .53), (.72, .61)), colors):
        x, y, w, h = crop_near(owner, target, 224)
        roi = np.s_[y:y + h, x:x + w]
        frame, uv, fraction = projected_source(recording, stats, owner[roi], (x, y, w, h))
        original = recording.image(COLOR, frame)
        if not cv2.imwrite(str(output / f'{name}_original_frame_{frame:04}.png'), original):
            raise OSError(f'could not save the original RGB frame {frame}')
        show(row[0], original, f'Original RGB · frame {frame}')
        row[0].add_patch(Polygon(uv, fill=False, edgecolor=color, linewidth=2))
        show(row[1], comparison[roi], comparison_title)
        final_title = 'Blur-aware source selection' if geometry_only else 'Single-source render + thin seams'
        show(row[2], rendered[roi], final_title)
        for axis in row[1:]:
            axis.add_patch(Rectangle((.002, .002), .996, .996, transform=axis.transAxes,
                                     fill=False, linewidth=1.8, edgecolor=color, clip_on=False))
        boxes.append(dict(box_xywh=[x, y, w, h], frame=frame, owner_fraction=fraction))
    fig.savefig(output / f'{name}_original_frames_and_map_crops.png', dpi=148)
    plt.close(fig)

    kernel = np.ones((5, 5), np.uint8)
    owner_values = owner.astype(np.float64)
    interior = cv2.erode((observations >= 2).astype(np.uint8), kernel, borderType=cv2.BORDER_CONSTANT) != 0
    interior &= cv2.erode(owner_values, kernel) == cv2.dilate(owner_values, kernel)
    if not interior.any():
        raise ValueError('no multiply observed interior pixels to score')
    stats['matched_crops'] = boxes
    stats['texture_comparison'] = {
        'scored_pixels': int(interior.sum()),
        'average_laplacian_rms': laplacian_rms(average, interior),
        'hard_laplacian_rms': laplacian_rms(hard, interior),
        'thin_seam_laplacian_rms': laplacian_rms(rendered, interior),
    }
    return name, stats, rendered, boxes, colors


def overview(name, original, reference, rendered, boxes, colors, output):
    fig, axes = plt.subplots(1, 3, figsize=(12, 4.8))
    fig.subplots_adjust(left=.025, right=.975, bottom=.12, top=.83, wspace=.08)
    fig.suptitle(f'{name} — panorama comparison', fontsize=16, y=.96)
    show(axes[0], original, 'Original RGB-D map')
    show(axes[1], reference, 'Offline mesh reference')
    show(axes[2], rendered, 'Refined ground-plane panorama')
    for entry, color in zip(boxes, colors):
        x, y, w, h = entry['box_xywh']
        axes[2].add_patch(Rectangle((x, y), w, h, fill=False, linewidth=1.6, edgecolor=color))
    fig.text(.028, .07, 'Maps use different projections. The outlined areas are shown at the same scale in the crop comparison.',
             fontsize=9, color='#4b5661')
    fig.savefig(output / f'{name}_panorama_comparison.png', dpi=150)
    plt.close(fig)


def alignment(name, audit, output):
    rows = [row for row in audit if not row['pair_used_in_fit']]
    if not rows:
        return
    before = np.array([row['before_px'] for row in rows])
    after = np.array([row['after_px'] for row in rows])
    limit = max(before.max(), after.max()) * 1.075
    fig, axis = plt.subplots(figsize=(6.2, 5.15), layout='constrained')
    axis.set_title(f'{name} — alignment on held-out frame pairs', loc='left', fontsize=13, pad=12)
    axis.plot([0, limit], [0, limit], color='#77808a', linestyle='--', linewidth=1, label='Same error')
    axis.scatter(before, after, color='#2a6d9e', s=34, alpha=.9, edgecolors='white', linewidths=.45,
                 label=f'{len(rows)} frame pairs')
    axis.set(xlabel='Before refinement (px)', ylabel='After refinement (px)',
             xlim=(0, limit), ylim=(0, limit), aspect='equal')
    axis.grid(color='#e0e4e8', linewidth=.7)
    axis.set_axisbelow(True)
    axis.legend(frameon=False, loc='upper left')
    fig.savefig(output / f'{name}_held_out_alignment.png', dpi=160)
    plt.close(fig)
    with (output / f'{name}_alignment_pairs.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(audit[0]))
        writer.writeheader()
        writer.writerows(audit)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--render', type=Path, required=True)
    parser.add_argument('--geometry-only-render', type=Path)
    parser.add_argument('--original-map', type=Path, required=True)
    parser.add_argument('--offline-reference', type=Path, required=True)
    parser.add_argument('--outdir', type=Path, required=True)
    args = parser.parse_args()
    args.outdir.mkdir(parents=True, exist_ok=True)
    with closing(Recording(args.bag)) as recording:
        name, stats, rendered, boxes, colors = map_crops(recording, args.render, args.outdir, args.geometry_only_render)
    overview(name, load_image(args.original_map), load_image(args.offline_reference), rendered,
             boxes, colors, args.outdir)
    alignment(name, stats['alignment_audit'], args.outdir)
    (args.outdir / f'{name}_panorama_metrics.json').write_text(json.dumps(stats, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
