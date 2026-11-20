#!/usr/bin/env python3
"""Check prefix-only velocity, accelerometer-bias and gravity initialization.

The linear fit uses RGB-D metric positions as a model reference. Rank,
conditioning, physical plausibility and later-window errors are reported.
No rejected estimate is used to correct a trajectory.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from recording_camera import Recording,GYRO,ACCEL


def design_matrix(times,rotation_integrals):
    return np.vstack([np.column_stack((np.eye(3)*t,-b,np.eye(3)*(.5*t*t)))
                      for t,b in zip(times,rotation_integrals)])


def solve_initialization(times,raw_position,rotation_integrals,visual_position):
    matrix=design_matrix(times,rotation_integrals)
    target=(np.asarray(visual_position)-np.asarray(raw_position)).reshape(-1)
    scaling=np.array([1.]*3+[.1]*3+[10.]*3)
    scaled=matrix*scaling
    solution,_,rank,singular=np.linalg.lstsq(scaled,target,rcond=1e-8)
    parameters=solution*scaling
    condition=float(singular[0]/singular[-1]) if len(singular)==9 and singular[-1]>1e-12 else None
    plausible=(8.8<float(np.linalg.norm(parameters[6:9]))<10.8 and np.linalg.norm(parameters[3:6])<1.)
    observable=int(rank)==9 and condition is not None and condition<1e5
    return parameters,{"rank":int(rank),"unknowns":9,"scaled_condition_number":condition,
                       "velocity_m_s":parameters[:3].tolist(),"accel_bias_m_s2":parameters[3:6].tolist(),
                       "gravity_m_s2":parameters[6:9].tolist(),"physically_plausible":bool(plausible),
                       "observable_by_linear_gate":bool(observable)}


def integrals(gts,gyro,ats,accel,queries,rotation_cg,rotation_ca,initial_rotation,bias):
    first,last=queries[[0,-1]]
    if first<max(gts[0],ats[0]) or last>min(gts[-1],ats[-1]):raise ValueError("inertial coverage missing")
    grid=np.unique(np.r_[gts[(gts>=first)&(gts<=last)],queries])
    w=np.column_stack([np.interp(grid,gts,gyro[:,k]-bias[k]) for k in range(3)])@rotation_cg.T
    f=np.column_stack([np.interp(grid,ats,accel[:,k]) for k in range(3)])
    r=initial_rotation.copy();v=np.zeros(3);p=np.zeros(3);jv=np.zeros((3,3));jp=np.zeros((3,3))
    wanted=set(int(t) for t in queries);out={int(first):(p.copy(),jp.copy())}
    a=r@rotation_ca@f[0];basis=r@rotation_ca
    for k in range(len(grid)-1):
        dt=float(grid[k+1]-grid[k])/1e9
        r=r@cv2.Rodrigues((w[k]+w[k+1])*.5*dt)[0]
        newbasis=r@rotation_ca;newa=newbasis@f[k+1]
        p+=v*dt+.25*(a+newa)*dt*dt;v+=(a+newa)*.5*dt
        jp+=jv*dt+.25*(basis+newbasis)*dt*dt;jv+=(basis+newbasis)*.5*dt
        a,basis=newa,newbasis
        if int(grid[k+1]) in wanted:out[int(grid[k+1])]=(p.copy(),jp.copy())
    return out


def run(bag,channels_path,gyro_fit_path,outpath):
    channel=json.loads(Path(channels_path).read_text());cal=json.loads(Path(gyro_fit_path).read_text())
    recording=Recording(bag)
    try:
        gts,gyro=recording.imu(GYRO);ats,accel=recording.imu(ACCEL)
        hypothesis=cal['selected_hypothesis'];shift=hypothesis['shift_ms']*1e6
        gts=gts-shift;ats=ats-shift
        selected=[(i,np.asarray(p)) for i,c,p in zip(channel['rgbd_pose_indices'],channel['rgbd_component_ids'],channel['rgbd_poses_to_component'])
                  if c==0 and max(gts[0],ats[0])<=recording.color_times[i]<=min(gts[-1],ats[-1])]
        indices=[i for i,_ in selected];poses=[p for _,p in selected]
        queries=recording.color_times[indices]
        values=integrals(gts,gyro,ats,accel,queries,recording.color_from_gyro[0],recording.color_from_accel[0],
                         poses[0][:3,:3],np.array(hypothesis['bias']))
        raw=np.array([values[int(t)][0] for t in queries]);b=np.array([values[int(t)][1] for t in queries])
        elapsed=(queries-queries[0])/1e9
        lever=recording.color_from_gyro[1]
        reference=np.array([p[:3,3]+p[:3,:3]@lever for p in poses]);reference-=reference[0]
        split=recording.color_times[0]+.2*(recording.color_times[-1]-recording.color_times[0])
        train=(queries<=split)&(elapsed>0);test=queries>split
        parameters,diagnostics=solve_initialization(elapsed[train],raw[train],b[train],reference[train])
        predicted=(design_matrix(elapsed,b)@parameters).reshape(-1,3)+raw
        # The comparison baseline estimates constant velocity on the same prefix.
        velocity=np.linalg.lstsq(elapsed[train,None],reference[train],rcond=None)[0][0]
        baseline=elapsed[:,None]*velocity
        def rmse(value):return float(np.sqrt(np.mean(np.sum((value[test]-reference[test])**2,axis=1))))
        prediction_error,baseline_error=rmse(predicted),rmse(baseline)
        usable=(diagnostics['observable_by_linear_gate'] and diagnostics['physically_plausible'] and prediction_error<baseline_error)
        result={"recording":Path(bag).name,"training_fraction":.2,"training_positions":int(train.sum()),
                "validation_positions":int(test.sum()),"fit":diagnostics,
                "heldout_initialized_position_rmse_m":prediction_error,"heldout_constant_velocity_rmse_m":baseline_error,
                "initialization_usable":bool(usable),"metric_ground_truth_available":False,
                "rejected_fit_applied_to_trajectory":False}
        Path(outpath).write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
        print(json.dumps(result),flush=True)
        return result
    finally:recording.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag',type=Path,required=True);p.add_argument('--channels',type=Path,required=True)
    p.add_argument('--gyro-fit',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();run(a.bag,a.channels,a.gyro_fit,a.out)


if __name__=='__main__':main()
