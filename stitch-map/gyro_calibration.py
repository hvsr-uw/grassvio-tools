#!/usr/bin/env python3
"""Fit gyro bias and a clock-shift hypothesis on a prefix, score later windows.

RGB-D rotations are a model reference, not ground truth. No accelerometer
bias is fitted here, and this does not establish inertial translation.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from recording_camera import Recording,GYRO
from motion_channels import angle


def samples_between(times,vectors,start,end):
    if start<times[0] or end>times[-1]:raise ValueError("gyro sample coverage missing")
    grid=np.r_[start,times[(times>start)&(times<end)],end]
    values=np.column_stack([np.interp(grid,times,vectors[:,k]) for k in range(3)])
    return np.diff(grid)/1e9,(values[1:]+values[:-1])*.5


def integrate_rotation(samples,bias,rotation_cg):
    intervals,omega=samples
    d=np.eye(3)
    for dt,w in zip(intervals,omega):d=d@cv2.Rodrigues((w-bias)*dt)[0]
    return rotation_cg@d.T@rotation_cg.T


def fit_bias(samples,reference,rotation_cg):
    def residual(bias):
        return np.concatenate([cv2.Rodrigues(r.T@integrate_rotation(s,bias,rotation_cg))[0].ravel()
                               for s,r in zip(samples,reference)])
    bias=np.zeros(3)
    for _ in range(12):
        r=residual(bias)
        epsilon=1e-5
        jac=np.column_stack([(residual(bias+np.eye(3)[k]*epsilon)-r)/epsilon for k in range(3)])
        step=np.linalg.solve(jac.T@jac+np.eye(3)*1e-8,-jac.T@r)
        if np.linalg.norm(step)<1e-8:break
        accepted=False
        for alpha in (1.,.5,.25,.125):
            candidate=bias+alpha*step
            if np.linalg.norm(residual(candidate))<np.linalg.norm(r):
                bias=candidate;accepted=True;break
        if not accepted:break
    return bias,float(np.sqrt(np.mean(residual(bias)**2)))


def run(bag,channels_path,outpath):
    channels=json.loads(Path(channels_path).read_text())
    recording=Recording(bag)
    try:
        times,gyro=recording.imu(GYRO)
        rotations=recording.color_from_gyro[0]
        first,last=recording.color_times[[0,-1]]
        split=first+.2*(last-first)
        shifts_ms=(-50.,-25.,0.,25.,50.)
        margin=max(abs(v) for v in shifts_ms)*1e6
        candidates=[r for r in channels["comparisons"]["raw_gyro"]
                    if "T_rgbd_source_to_target" in r
                    and recording.color_times[r["source_frame"]]>=times[0]+margin
                    and recording.color_times[r["target_frame"]]<=times[-1]-margin]
        training=[r for r in candidates if recording.color_times[r["target_frame"]]<=split]
        validation=[r for r in candidates if recording.color_times[r["source_frame"]]>=split]
        if len(training)<2 or len(validation)<3:
            result={"recording":Path(bag).name,"status":"insufficient_prefix_or_validation_rotations"}
        else:
            hypotheses=[]
            reference=[np.asarray(r["T_rgbd_source_to_target"])[:3,:3] for r in training]
            def sample(row,shift):
                a,b=recording.color_times[[row["source_frame"],row["target_frame"]]]+shift*1e6
                return samples_between(times,gyro,a,b)
            for shift in shifts_ms:
                measurements=[sample(r,shift) for r in training]
                bias,error=fit_bias(measurements,reference,rotations)
                hypotheses.append({"shift_ms":shift,"bias":bias.tolist(),"training_rmse_rad":error})
            best=min(hypotheses,key=lambda h:h["training_rmse_rad"])
            scored=[]
            for row in validation:
                reference=np.asarray(row["T_rgbd_source_to_target"])[:3,:3]
                before=integrate_rotation(sample(row,0.),np.zeros(3),rotations)
                after=integrate_rotation(sample(row,best["shift_ms"]),np.asarray(best["bias"]),rotations)
                scored.append({"source_frame":row["source_frame"],"target_frame":row["target_frame"],
                               "raw_error_deg":angle(before,reference),"calibrated_error_deg":angle(after,reference)})
            result={"recording":Path(bag).name,"status":"prefix_fit_scored_on_later_windows",
                    "training_fraction":.2,"training_windows":len(training),"validation_windows":len(validation),
                    "clock_shift_definition":"gyro query time equals camera time plus shift",
                    "selected_hypothesis":best,"training_hypotheses":hypotheses,
                    "heldout_raw_median_deg":float(np.median([r["raw_error_deg"] for r in scored])),
                    "heldout_calibrated_median_deg":float(np.median([r["calibrated_error_deg"] for r in scored])),
                    "heldout_raw_p90_deg":float(np.percentile([r["raw_error_deg"] for r in scored],90)),
                    "heldout_calibrated_p90_deg":float(np.percentile([r["calibrated_error_deg"] for r in scored],90)),
                    "metric_ground_truth_available":False,"accelerometer_bias_fitted":False,
                    "validation":scored}
        Path(outpath).write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
        print(json.dumps({k:v for k,v in result.items() if k not in ("validation","training_hypotheses")}),flush=True)
        return result
    finally:recording.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag',type=Path,required=True);p.add_argument('--channels',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();run(a.bag,a.channels,a.out)


if __name__=='__main__':main()
