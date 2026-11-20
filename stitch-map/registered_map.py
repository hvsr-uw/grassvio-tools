#!/usr/bin/env python3
"""Render registered depth through metric RGB-D poses into a ground-plane grid.

The initial measured plane sets the viewing direction. This does not make
the RGB-D trajectory ground truth or correct its accumulated pose errors.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from recording_camera import Recording,COLOR


def fit_plane(points,threshold_m=.04):
    points=np.asarray(points,float).reshape(-1,3)
    points=points[np.isfinite(points).all(axis=1)]
    if len(points)<100:raise ValueError("too few measured points for the initial plane")
    rng=np.random.default_rng(60)
    sample=points[rng.choice(len(points),min(len(points),12000),replace=False)]
    best=None
    for _ in range(150):
        a,b,c=sample[rng.choice(len(sample),3,replace=False)]
        normal=np.cross(b-a,c-a)
        if np.linalg.norm(normal)<1e-8:continue
        normal/=np.linalg.norm(normal)
        distance=normal@a
        mask=np.abs(sample@normal-distance)<threshold_m
        if best is None or mask.sum()>best.sum():best=mask
    if best is None or best.mean()<.4:raise ValueError("initial dominant plane not supported")
    selected=sample[best];center=selected.mean(axis=0)
    _,_,vt=np.linalg.svd(selected-center,full_matrices=False)
    normal=vt[-1];distance=float(normal@center)
    if distance<0:normal=-normal;distance=-distance
    return normal,distance,{"sample_points":len(sample),"inlier_fraction":float(best.mean()),
                            "median_plane_error_m":float(np.median(np.abs(selected@normal-distance)))}


def plane_basis(normal):
    normal=np.asarray(normal,float)/np.linalg.norm(normal)
    horizontal=np.array([1.,0,0])-normal*normal[0]
    if np.linalg.norm(horizontal)<.1:
        horizontal=np.array([0.,0,1])-normal*normal[2]
    horizontal/=np.linalg.norm(horizontal)
    return np.column_stack((horizontal,np.cross(normal,horizontal)))


def render(bag,channels_path,outpath,pixels_per_m=200.):
    recording=Recording(bag)
    try:
        channels=json.loads(Path(channels_path).read_text())
        first,offset=recording.cloud_for_color(0)
        h,w=first.shape[:2]
        plane_points=first[h//2:,:,:][::3,::3].reshape(-1,3)
        normal,distance,plane_stats=fit_plane(plane_points)
        basis=plane_basis(normal)
        coordinates=[];colors=[];priorities=[];used=[];skipped=[]
        yy,xx=np.mgrid[:h,:w]
        central=np.minimum.reduce((xx+1,w-xx,yy+1,h-yy)).astype(float).ravel()
        for index,component,pose in zip(channels["rgbd_pose_indices"],channels["rgbd_component_ids"],channels["rgbd_poses_to_component"]):
            if component!=0:
                skipped.append(index);continue
            cloud,pair_offset=recording.cloud_for_color(index)
            bgr=recording.image(COLOR,index)
            points=cloud.reshape(-1,3)
            good=np.isfinite(points).all(axis=1)&(points[:,2]>.05)&(points[:,2]<3.)
            points=points[good];pose=np.asarray(pose)
            world=points@pose[:3,:3].T+pose[:3,3]
            coordinates.append(world@basis*pixels_per_m)
            colors.append(bgr.reshape(-1,3)[good])
            priorities.append(central[good]/points[:,2]**2)
            used.append(index)
        xy=np.concatenate(coordinates);bgr=np.concatenate(colors);priority=np.concatenate(priorities)
        lo=np.floor(xy.min(axis=0));hi=np.ceil(xy.max(axis=0))+1
        width,height=(hi-lo).astype(int)
        if max(width,height)>8000 or width*height>16_000_000:
            raise ValueError("registered orthographic map exceeds render budget")
        indices=np.rint(xy-lo).astype(int)
        pixel=indices[:,1]*width+indices[:,0]
        # Pick the best original colour sample at each cell, without smoothing.
        order=np.argsort(-priority,kind="stable")
        cells,first=np.unique(pixel[order],return_index=True)
        image=np.zeros((height*width,3),np.uint8)
        image[cells]=bgr[order[first]]
        image=image.reshape(height,width,3)
        if not cv2.imwrite(str(outpath),image):raise OSError("could not write registered map")
        stats={"recording":Path(bag).name,"camera_pose_source":"registered_RGBD_PnP",
               "metric_ground_truth_available":False,"pixels_per_m":pixels_per_m,
               "initial_plane_normal_color":normal.tolist(),"initial_plane_distance_m":distance,
               "initial_plane_fit":plane_stats,"frames_used":used,"frames_skipped_unconnected":skipped,
               "canvas_width_px":int(width),"canvas_height_px":int(height),
               "coverage_fraction":float(len(cells)/(width*height)),
               "depth_registration":channels["depth_registration"]}
        Path(outpath).with_suffix('.json').write_text(json.dumps(stats,indent=2)+"\n")
        print(json.dumps(stats),flush=True)
        return stats
    finally:recording.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag',type=Path,required=True)
    parser.add_argument('--channels',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    render(args.bag,args.channels,args.out)


if __name__=='__main__':main()
