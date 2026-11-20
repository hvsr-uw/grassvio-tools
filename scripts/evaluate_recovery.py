#!/usr/bin/env python3
"""Compare retained homography and RGB-D endpoints on the same image matches.

Both predictions map the final camera view into the first image. Only
direct-loop inliers with measured final-frame depth enter either score.
This is image consistency, not metric pose ground truth.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'stitch-map'))
from geometry import fit_homography,apply_transform
from recording_camera import Recording,project
from motion_channels import sample_cloud
from check_recording_loops import load_selected_color,match_loop


def run(bag,folder):
    atlas=json.loads((folder/'atlas/atlas.json').read_text())
    channel=json.loads((folder/'channels/channels.json').read_text())
    end=atlas['selected_frames'][-1]
    images=load_selected_color(bag,[0,end])
    source,target=match_loop(images[0],images[end])
    direct,mask=fit_homography(source,target,min_inliers=20)
    if direct is None:raise ValueError('no supported direct image loop')
    recording=Recording(bag)
    try:
        cloud,_=recording.cloud_for_color(end)
        points=sample_cloud(cloud,source)
        cells=np.floor(target/40.).astype(int)
        heldout=(cells[:,0]+cells[:,1])%2==1
        valid=np.isfinite(points).all(axis=1)&(mask.ravel()!=0)&heldout
        if channel['rgbd_pose_indices'][-1]!=end or channel['rgbd_component_ids'][-1]!=0:
            raise ValueError('no RGB-D endpoint connected to the first camera')
        pose=np.asarray(channel['rgbd_poses_to_component'][-1])
        world=points[valid]@pose[:3,:3].T+pose[:3,3]
        visible=world[:,2]>.05
        source,target,world=source[valid][visible],target[valid][visible],world[visible]
        homography=np.asarray(atlas['homographies_to_component'][-1])
        herror=np.linalg.norm(apply_transform(homography,source)-target,axis=1)
        derror=np.linalg.norm(project(world,recording.color)-target,axis=1)
        result={'recording':bag.name,'endpoint_frame':end,'common_matches':len(source),
            'evaluation_tiles':'first_image_odd_checkerboard_40px_not_used_for_loop_fit',
            'comparison_direction':'end_to_first','homography_median_error_px':float(np.median(herror)),
            'rgbd_median_error_px':float(np.median(derror)),
            'rgbd_start_to_end_distance_m':float(np.linalg.norm(pose[:3,3])),
            'metric_ground_truth_available':False}
        (folder/'endpoint_evaluation.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result),flush=True)
        return result
    finally:recording.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-dir',type=Path,required=True)
    parser.add_argument('--results-dir',type=Path,required=True)
    args=parser.parse_args()
    inputs=sorted(args.input_dir.glob('*.db3'))
    if not inputs:parser.error('no recordings found')
    summary=[run(bag,args.results_dir/bag.stem) for bag in inputs]
    (args.results_dir/'endpoint_summary.json').write_text(json.dumps(summary,indent=2)+'\n')


if __name__=='__main__':main()
