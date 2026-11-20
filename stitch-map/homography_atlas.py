#!/usr/bin/env python3
"""Keep homography tracking separate from bounded local map projections.

An invertible motion link is not discarded because the full-frame warp
crosses a projective horizon. Local maps still obey the original geometry
and allocation guards. These maps are not one metric orthomosaic.
"""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from geometry import projected_corners, render_mosaic
from stitch_map import load_frames_from_bag
from window_motion import flow_pairs


def normalized_product(a, b):
    result = np.asarray(a) @ np.asarray(b)
    if not np.isfinite(result).all() or np.linalg.matrix_rank(result) != 3:
        raise ValueError("non-finite or singular homography chain")
    return result / np.linalg.norm(result)


def track_edges(frames, window=25, threshold_px=1.5, adaptive_min=5):
    grays = [cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) for frame in frames]
    edges, observations = [], {}

    def measure(start, end):
        saved = {}
        def observe(a, b, history, keep, mask):
            if mask is not None:
                good = mask.ravel() != 0
                saved["source"] = history[0, keep][good]
                saved["target"] = history[-1, keep][good]
        _, _, h, stats = next(flow_pairs(grays[start:end+1], window=end-start,
            residual_threshold_px=threshold_px, reject_full_frame_horizon=False,
            track_observer=observe))
        if h is None and end-start > adaptive_min:
            middle = start + (end-start)//2
            # Both retries remain temporal filtered; no descriptor fallback.
            if middle-start >= adaptive_min and end-middle >= adaptive_min:
                measure(start, middle)
                measure(middle, end)
                return
        stats.update(start_frame=start, end_frame=end, adaptive_window=end-start != window)
        index = len(edges)
        edges.append(dict(stats, H_target_to_source=h.tolist() if h is not None else None))
        if saved:
            observations[f"source_{index}"] = saved["source"]
            observations[f"target_{index}"] = saved["target"]
        print(f"window {start}->{end}: fit={h is not None}, inliers={stats['inliers']}", flush=True)

    for start in range(0, len(frames)-1, window):
        measure(start, min(start+window, len(frames)-1))
    return edges, observations


def chain_edges(edges):
    indices, transforms, components = [0], [np.eye(3)], [0]
    for edge in edges:
        if edge["start_frame"] != indices[-1]:
            raise ValueError("non-contiguous edge indices")
        h = edge["H_target_to_source"]
        if h is None:
            components.append(components[-1]+1)
            transforms.append(np.eye(3))
        else:
            components.append(components[-1])
            transforms.append(normalized_product(transforms[-1], h))
        indices.append(edge["end_frame"])
    return indices, transforms, components


def chart_groups(frames, edges, max_side=8000, max_pixels=16_000_000):
    """Start a new overlapping chart when an old chart becomes unrenderable."""
    charts, chosen, poses = [], [0], [np.eye(3)]
    def permitted(indices, transforms):
        points = np.concatenate([projected_corners(t, frames[i].shape)
                                 for i,t in zip(indices, transforms)])
        extent = np.ceil(points.max(axis=0))-np.floor(points.min(axis=0))
        if max(extent) > max_side or np.prod(extent) > max_pixels:
            raise ValueError("local canvas exceeds render budget")
    for edge in edges:
        start, end, h = edge["start_frame"], edge["end_frame"], edge["H_target_to_source"]
        if h is None:
            charts.append((chosen, poses))
            chosen, poses = [end], [np.eye(3)]
            continue
        candidate = normalized_product(poses[-1], h)
        try:
            permitted(chosen+[end], poses+[candidate])
            chosen.append(end); poses.append(candidate)
        except ValueError:
            charts.append((chosen, poses))
            try:
                permitted([start,end], [np.eye(3),np.asarray(h)])
                chosen, poses = [start,end], [np.eye(3),np.asarray(h)]
            except ValueError:
                # The local motion link remains in the graph even if the
                # pair cannot share a finite full-image chart.
                chosen, poses = [end], [np.eye(3)]
    charts.append((chosen, poses))
    return charts


def run_recording(bag, outdir, window=25, threshold_px=1.5, adaptive_min=5):
    bag, outdir = Path(bag), Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    cache = outdir / "edges.json"
    signature = hashlib.sha256()
    for source in (Path(__file__), Path(__file__).with_name("window_motion.py"),
                   Path(__file__).with_name("geometry.py"), Path(__file__).with_name("heldout_diagnostics.py"),
                   Path(__file__).with_name("stitch_map.py")):
        signature.update(source.read_bytes())
    config = {"path":str(bag.resolve()),"size":bag.stat().st_size,"mtime_ns":bag.stat().st_mtime_ns,
              "window":window,"threshold_px":threshold_px,"adaptive_min":adaptive_min,
              "code":signature.hexdigest()}
    frames = load_frames_from_bag(bag)
    if cache.exists() and json.loads(cache.read_text())["config"] == config:
        edges = json.loads(cache.read_text())["edges"]
    else:
        edges, points = track_edges(frames, window, threshold_px, adaptive_min)
        np.savez_compressed(outdir/"tracks.npz", **points)
        cache.write_text(json.dumps({"config":config,"edges":edges}, indent=2)+"\n")
    indices, transforms, components = chain_edges(edges)
    maps = []
    for k,(selected,poses) in enumerate(chart_groups(frames,edges)):
        image, stats = render_mosaic([frames[i] for i in selected], poses, blend="best")
        name = f"local_map_{k:02}.png"
        if not cv2.imwrite(str(outdir/name), image):
            raise OSError("could not save local map")
        maps.append(dict(stats,image=name,selected_frames=selected,
                         transforms_to_chart=[h.tolist() for h in poses]))
    result = {"recording":bag.name,"frames_total":len(frames),"selected_frames":indices,
              "edges_total":len(edges),"edges_valid":sum(e["H_target_to_source"] is not None for e in edges),
              "components":components,"connected_to_first":sum(c==0 for c in components),
              "homographies_to_component":[h.tolist() for h in transforms],
              "complete_homography_path":len(set(components))==1,
              "metric_pose_available":False,"single_global_map":False,
              "maps":maps,"source_config":config}
    (outdir/"atlas.json").write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps({k:v for k,v in result.items() if k not in ("maps","homographies_to_component","source_config")}), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag",type=Path,required=True)
    parser.add_argument("--outdir",type=Path,required=True)
    args = parser.parse_args()
    run_recording(args.bag,args.outdir)


if __name__ == "__main__":
    main()
