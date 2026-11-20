#!/usr/bin/env python3
"""Compare calibrated RGB-D, inertial and two-view geometry on common windows.

The inertial translation assumes zero initial camera velocity and a quiet
startup gravity estimate. Those assumptions are reported, not verified.
Two-view translation has unknown scale and is compared by direction only.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from recording_camera import Recording,COLOR,GYRO,ACCEL,rays
from sfm_keyframes import assess_two_view
from relocalization import filtered_sequence_features,match_supported


def angle(a,b):
    return float(np.degrees(np.arccos(np.clip((np.trace(np.asarray(a).T@np.asarray(b))-1)/2,-1,1))))


def estimate_pnp(points3d,target_rays,focal):
    points3d,target_rays=np.asarray(points3d,float),np.asarray(target_rays,float)
    good=np.isfinite(points3d).all(axis=1)&np.isfinite(target_rays).all(axis=1)
    points3d,target_rays=points3d[good],target_rays[good]
    if len(points3d)<30:return None,{"status":"insufficient_registered_depth","depth_matches":len(points3d)}
    cv2.setRNGSeed(0)
    ok,rv,tv,inliers=cv2.solvePnPRansac(points3d,target_rays[:,:2],np.eye(3),None,
        iterationsCount=500,reprojectionError=3./focal,confidence=.999)
    if not ok or inliers is None or len(inliers)<20:
        return None,{"status":"pnp_fit_failed","depth_matches":len(points3d)}
    indices=inliers.ravel()
    ok,rv,tv=cv2.solvePnP(points3d[indices],target_rays[indices,:2],np.eye(3),None,
                        rv,tv,True,flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:return None,{"status":"pnp_refine_failed","depth_matches":len(points3d)}
    r=cv2.Rodrigues(rv)[0]
    t=np.eye(4);t[:3,:3]=r;t[:3,3]=tv.ravel()
    predicted=(points3d[indices]@r.T+tv.ravel())
    err=np.linalg.norm(predicted[:,:2]/predicted[:,2:]-target_rays[indices,:2],axis=1)*focal
    return t,{"status":"estimated","depth_matches":len(points3d),"inliers":len(indices),
              "median_reprojection_error_px":float(np.median(err))}


def sample_cloud(cloud,pixels):
    """Use a measured neighbour within one pixel; never fill a large hole."""
    result=np.full((len(pixels),3),np.nan)
    h,w=cloud.shape[:2]
    for i,(u,v) in enumerate(np.rint(pixels).astype(int)):
        choices=[]
        for dy,dx in ((0,0),(-1,0),(0,-1),(0,1),(1,0)):
            x,y=u+dx,v+dy
            if 0<=x<w and 0<=y<h and np.isfinite(cloud[y,x]).all():
                choices.append(cloud[y,x])
                break
        if choices:result[i]=choices[0]
    return result


def reconnect_component(poses,component_ids,pose_sources,component,world_from_component):
    recovered=[]
    for index,identity in enumerate(component_ids):
        if identity==component:
            poses[index]=world_from_component@poses[index]
            component_ids[index]=0
            pose_sources[index]="connected_by_later_relocalization"
            recovered.append(index)
    return recovered


def fit_anchor_loop(points3d,target_rays,anchor_pixels,focal):
    """Fit on even spatial tiles and check the other tiles before using a loop."""
    cells=np.floor(np.asarray(anchor_pixels)/40.).astype(int)
    train=(cells[:,0]+cells[:,1])%2==0
    test=~train
    transform,stats=estimate_pnp(points3d[train],target_rays[train],focal)
    if transform is None:return None,stats
    finite=test&np.isfinite(points3d).all(axis=1)
    if finite.sum()<20:return None,{"status":"insufficient_spatial_loop_holdout"}
    target=points3d[finite]@transform[:3,:3].T+transform[:3,3]
    if np.any(target[:,2]<=0):return None,{"status":"loop_behind_camera"}
    error=np.linalg.norm(target[:,:2]/target[:,2:]-target_rays[finite,:2],axis=1)*focal
    stats.update(loop_training_tracks=int(train.sum()),loop_heldout_tracks=int(finite.sum()),
                 loop_heldout_median_error_px=float(np.median(error)))
    accepted=(stats.get('inliers',0)>=30 and stats['inliers']>=.5*stats['depth_matches']
              and stats['median_reprojection_error_px']<=1. and np.median(error)<=1.5)
    return (transform if accepted else None),stats


def estimate_static_window(gts, gyro, ats, accel, window_seconds=0.5, min_samples=20):
    """Choose the quietest measured window for gravity initialization.

    A recording does not necessarily begin at rest. We score candidate
    windows using gyro magnitude and accelerometer-magnitude variation, then
    use the lowest-score window instead of blindly using the first second.
    This is an initialization heuristic, not proof that the camera was still.
    """
    gts, ats = np.asarray(gts, dtype=np.int64), np.asarray(ats, dtype=np.int64)
    gyro, accel = np.asarray(gyro, dtype=float), np.asarray(accel, dtype=float)
    if gts.ndim != 1 or ats.ndim != 1 or gyro.shape != (len(gts), 3) or accel.shape != (len(ats), 3):
        raise ValueError("sensor timestamps and samples have incompatible shapes")
    if window_seconds <= 0 or min_samples < 2:
        raise ValueError("window_seconds and min_samples must be positive")
    if not (len(gts) and len(ats)):
        raise ValueError("sensor streams are empty")
    duration = int(round(window_seconds * 1e9))
    lower = max(int(gts[0]), int(ats[0]))
    upper = min(int(gts[-1]), int(ats[-1])) - duration
    if upper < lower:
        raise ValueError("sensor streams are shorter than the static window")
    starts = np.unique(np.r_[gts[(gts >= lower) & (gts <= upper)],
                             ats[(ats >= lower) & (ats <= upper)]])
    best = None
    for start in starts:
        end = int(start + duration)
        gm = np.linalg.norm(gyro[(gts >= start) & (gts <= end)], axis=1)
        am = np.linalg.norm(accel[(ats >= start) & (ats <= end)], axis=1)
        if len(gm) < min_samples or len(am) < min_samples:
            continue
        gyro_score = float(np.median(gm))
        accel_mad = float(np.median(np.abs(am - np.median(am))))
        candidate = (gyro_score + accel_mad, int(start), end,
                     gyro_score, accel_mad, len(gm), len(am))
        if best is None or candidate < best:
            best = candidate
    if best is None:
        raise ValueError("no static-window candidate has enough samples")
    score, start, end, gyro_score, accel_mad, gyro_count, accel_count = best
    return {"start_ns": start, "end_ns": end, "duration_s": window_seconds,
            "score": float(score), "median_gyro_norm": gyro_score,
            "accel_norm_mad": accel_mad, "gyro_samples": gyro_count,
            "accel_samples": accel_count}


def inertial_path(gts,gyro,ats,accel,query_times,rotation_cg,rotation_ca,imu_origin_in_color,gyro_bias,\n                  gravity_window=None, initial_velocity=None):
    query_times=np.asarray(query_times,dtype=np.int64)
    first,last=query_times[0],query_times[-1]
    if first<max(gts[0],ats[0]) or last>min(gts[-1],ats[-1]):
        raise ValueError("requested inertial interval is outside measured sensor coverage")
    grid=np.unique(np.r_[gts[(gts>=first)&(gts<=last)],query_times])
    omega=np.column_stack([np.interp(grid,gts,gyro[:,k]-gyro_bias[k]) for k in range(3)])@rotation_cg.T
    force=np.column_stack([np.interp(grid,ats,accel[:,k]) for k in range(3)])@rotation_ca.T
    if gravity_window is None:
        gravity_window = estimate_static_window(gts, gyro, ats, accel)
    gravity_start = int(gravity_window["start_ns"])
    gravity_end = int(gravity_window["end_ns"])
    startup=accel[(ats>=gravity_start)&(ats<=gravity_end)]
    if len(startup)<30:raise ValueError("insufficient static-window accelerometer samples")
    gravity=-(startup@rotation_ca.T).mean(axis=0)
    r=np.eye(3);p=np.zeros(3)
    velocity = (np.zeros(3) if initial_velocity is None else
                np.asarray(initial_velocity, dtype=float).reshape(3).copy())
    velocity += np.cross(omega[0],imu_origin_in_color)
    output={}
    wanted=set(int(t) for t in query_times)
    if int(grid[0]) in wanted:output[int(grid[0])]=(r.copy(),p.copy())
    previous_accel=force[0]+gravity
    for k in range(len(grid)-1):
        dt=float(grid[k+1]-grid[k])/1e9
        r=r@cv2.Rodrigues((omega[k]+omega[k+1])*.5*dt)[0]
        current_accel=r@force[k+1]+gravity
        amid=(previous_accel+current_accel)*.5
        p+=velocity*dt+.5*amid*dt*dt
        velocity+=amid*dt
        previous_accel=current_accel
        if int(grid[k+1]) in wanted:
            # Transform the IMU-origin displacement to the colour-camera origin.
            output[int(grid[k+1])]=(r.copy(),p+imu_origin_in_color-r@imu_origin_in_color)
    return output,{"initial_camera_velocity":
                   "assumed_zero_not_verified" if initial_velocity is None else "provided_visual_reference",
                   "gravity_from":"selected_low_motion_window",
                   "gravity_window":gravity_window,
                   "gravity_vector":gravity.tolist(),"accelerometer_bias_separately_estimated":False,
                   "gyro_bias_used":np.asarray(gyro_bias).tolist()}


def summarize(records,variant):
    def metric(key):
        values=[r[key] for r in records if isinstance(r.get(key),(int,float)) and np.isfinite(r[key])]
        return {"count":len(values),"median":float(np.median(values)) if values else None,
                "p90":float(np.percentile(values,90)) if values else None}
    return {"imu_variant":variant,"windows":len(records),
            "rgbd_windows":sum(r.get("rgbd",{}).get("status")=="estimated" for r in records),
            "sfm_eligible_windows":sum(r.get("sfm",{}).get("eligible",False) for r in records),
            "gyro_vs_rgbd_deg":metric("gyro_vs_rgbd_deg"),
            "sfm_vs_rgbd_deg":metric("sfm_vs_rgbd_deg"),
            "sfm_translation_direction_vs_rgbd_deg":metric("sfm_translation_direction_vs_rgbd_deg"),
            "imu_translation_disagreement_m":metric("imu_translation_disagreement_m")}


def run_recording(bag,atlas_dir,outdir):
    bag,atlas_dir,outdir=Path(bag),Path(atlas_dir),Path(outdir)
    outdir.mkdir(parents=True,exist_ok=True)
    edges=json.loads((atlas_dir/"edges.json").read_text())["edges"]
    tracks=np.load(atlas_dir/"tracks.npz")
    recording=Recording(bag)
    try:
        gts,gyro=recording.imu(GYRO);ats,accel=recording.imu(ACCEL)
        indices=sorted({0,*[edge["end_frame"] for edge in edges]})
        # Retain only intervals covered by both recorded IMU streams.
        indices=[i for i in indices if max(gts[0],ats[0])<=recording.color_times[i]<=min(gts[-1],ats[-1])]
        times=recording.color_times[indices]
        static_window = estimate_static_window(gts, gyro, ats, accel)
        static_gyro = gyro[(gts >= static_window["start_ns"]) & (gts <= static_window["end_ns"])]
        biases={"raw_gyro":np.zeros(3),
                "conditional_static_window_demean":static_gyro.mean(axis=0)}
        # Delay inertial propagation until the first valid RGB-D edge. This
        # gives the IMU an observed visual initial velocity when the take did
        # not begin at rest, instead of silently forcing velocity to zero.
        inertial={};initializations={}
        variants={name:[] for name in biases}
        poses=[np.eye(4)];pose_indices=[0];component_ids=[0];pose_sources=["initial_anchor"]
        next_component=1
        anchor_features=None;anchor_cloud=None
        focal=float((recording.color["fx"]+recording.color["fy"])*.5)
        for k,edge in enumerate(edges):
            start,end=edge["start_frame"],edge["end_frame"]
            if f"source_{k}" not in tracks:
                raise ValueError("missing saved visual observations; rerun the atlas measurement")
            source,target=tracks[f"source_{k}"],tracks[f"target_{k}"]
            a,b=rays(source,recording.color),rays(target,recording.color)
            cloud,offset=recording.cloud_for_color(start)
            points3d=sample_cloud(cloud,source)
            pnp,pnp_stats=estimate_pnp(points3d,b,focal)
            kcam=recording.color["K"]
            source_undistorted=a[:,:2]*[kcam[0,0],kcam[1,1]]+[kcam[0,2],kcam[1,2]]
            target_undistorted=b[:,:2]*[kcam[0,0],kcam[1,1]]+[kcam[0,2],kcam[1,2]]
            sfm=assess_two_view(source_undistorted,target_undistorted,kcam)
            common={"source_frame":start,"target_frame":end,"rgb_depth_offset_ms":offset,
                    "rgbd":pnp_stats,"sfm":sfm}
            if pnp is not None:
                common["T_rgbd_source_to_target"]=pnp.tolist()
                if "R_source_to_target" in sfm:
                    common["sfm_vs_rgbd_deg"]=angle(sfm["R_source_to_target"],pnp[:3,:3])
                if sfm.get("eligible") and np.linalg.norm(pnp[:3,3])>=.005:
                    vt=np.asarray(sfm["t_unit_source_to_target"])
                    common["sfm_translation_direction_vs_rgbd_deg"]=float(np.degrees(np.arccos(np.clip(
                        np.dot(vt,pnp[:3,3])/np.linalg.norm(pnp[:3,3]),-1,1))))
                next_pose=poses[-1]@np.linalg.inv(pnp)
                next_id=component_ids[-1]
                pose_source="chained_relative"
            else:
                next_pose=np.eye(4);next_id=next_component;next_component+=1
                pose_source="unconnected"
            check_loop=next_id!=0 or end==edges[-1]['end_frame'] or (k>=5 and k%5==0)
            if check_loop:
                if anchor_features is None:
                    anchor_features=filtered_sequence_features([cv2.cvtColor(recording.image(COLOR,i),cv2.COLOR_BGR2GRAY)
                        for i in range(edges[0]['end_frame']+1)])
                    anchor_cloud,_=recording.cloud_for_color(0)
                current=filtered_sequence_features([cv2.cvtColor(recording.image(COLOR,i),cv2.COLOR_BGR2GRAY)
                    for i in range(end,start-1,-1)])
                anchor_pixels,current_pixels=match_supported(anchor_features,current)
                recovery,recovery_stats=fit_anchor_loop(sample_cloud(anchor_cloud,anchor_pixels),
                    rays(current_pixels,recording.color),anchor_pixels,focal)
                if recovery is not None:
                    world_pose=np.linalg.inv(recovery)
                    recovered=(reconnect_component(poses,component_ids,pose_sources,next_id,
                        world_pose@np.linalg.inv(next_pose)) if next_id!=0 else [])
                    next_pose=world_pose;next_id=0
                    pose_source="supported_anchor_relocalization"
                    common['endpoint_relocalization']={"anchor_frame":0,
                        "earlier_frames_reconnected":[pose_indices[i] for i in recovered],**recovery_stats}
            poses.append(next_pose);component_ids.append(next_id);pose_sources.append(pose_source)
            pose_indices.append(end)
            if not inertial and pnp is not None:
                dt = float(recording.color_times[end] - recording.color_times[start]) / 1e9
                visual_velocity = (next_pose[:3, 3] / dt) if dt > 0 else np.zeros(3)
                for name,bias in biases.items():
                    inertial[name],initializations[name]=inertial_path(
                        gts,gyro,ats,accel,times,
                        recording.color_from_gyro[0],recording.color_from_accel[0],
                        recording.color_from_gyro[1],bias,
                        gravity_window=static_window,
                        initial_velocity=visual_velocity)
            for name,path in inertial.items():
                record=dict(common)
                if start not in indices or end not in indices:
                    record["imu_status"]="outside_measured_sensor_coverage"
                    variants[name].append(record)
                    continue
                record["imu_status"]="estimated"
                ra,pa=path[int(recording.color_times[start])]
                rb,pb=path[int(recording.color_times[end])]
                ri,ti=rb.T@ra,rb.T@(pa-pb)
                record["R_imu_source_to_target"]=ri.tolist()
                record["t_imu_source_to_target_m"]=ti.tolist()
                if pnp is not None:
                    record["gyro_vs_rgbd_deg"]=angle(ri,pnp[:3,:3])
                    record["imu_translation_disagreement_m"]=float(np.linalg.norm(ti-pnp[:3,3]))
                variants[name].append(record)
            print(f"{bag.name}: {start}->{end}, RGB-D={pnp_stats['status']}, SfM={sfm['reason']}",flush=True)
        if not inertial:
            # Keep a complete diagnostic file even when no RGB-D edge could
            # initialize a visual velocity reference.
            for name,bias in biases.items():
                inertial[name],initializations[name]=inertial_path(
                    gts,gyro,ats,accel,times,
                    recording.color_from_gyro[0],recording.color_from_accel[0],
                    recording.color_from_gyro[1],bias,
                    gravity_window=static_window)
        result={"recording":bag.name,"clock_shift_ms":0.,"metric_ground_truth_available":False,
                "sfm_metric_translation_available":False,"three_channel_agreement_established":False,
                "static_initialization_window":static_window,
                "depth_registration":"recorded_reference_to_stream_extrinsics_and_sdk_distortion",
                "initializations":initializations,"summaries":{name:summarize(records,name) for name,records in variants.items()},
                "comparisons":variants,"rgbd_pose_indices":pose_indices,
                "rgbd_component_ids":component_ids,"rgbd_pose_sources":pose_sources,
                "rgbd_poses_to_component":[p.tolist() for p in poses]}
        (outdir/"channels.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
        print(json.dumps(result["summaries"]),flush=True)
        return result
    finally:
        recording.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag",type=Path,required=True)
    parser.add_argument("--atlas-dir",type=Path,required=True)
    parser.add_argument("--outdir",type=Path,required=True)
    args=parser.parse_args()
    run_recording(args.bag,args.atlas_dir,args.outdir)


if __name__=="__main__":main()
