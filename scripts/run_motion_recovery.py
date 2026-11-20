#!/usr/bin/env python3
"""Run local mapping recovery and channel checks with per-stage checkpoints.

Command completion and scientific validation are reported separately.
All recordings, maps and detailed measurements stay in the local output.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import cv2
import numpy as np

from run_recording_batch import atomic_json


ROOT=Path(__file__).resolve().parents[1]


def run(args):
    inputs=sorted(args.input_dir.glob('*.db3'))
    if args.names:
        inputs=[p for p in inputs if p.name in args.names]
        missing=set(args.names)-{p.name for p in inputs}
        if missing:raise ValueError(f'recordings missing: {sorted(missing)}')
    if not inputs:raise ValueError('no recordings found')
    all_modules=('geometry','window_motion','heldout_diagnostics','stitch_map','sfm_keyframes',
                 'recording_camera','homography_atlas','motion_channels','registered_map',
                 'gyro_calibration','inertial_initialization','relocalization')
    summary=[]
    for bag in inputs:
        folder=args.output_dir/bag.stem
        atlas=folder/'atlas';channel=folder/'channels'
        folder.mkdir(parents=True,exist_ok=True);channel.mkdir(exist_ok=True)
        channels=channel/'channels.json';gyro=channel/'gyro_calibration.json'
        commands=[
            ('atlas','homography_atlas.py',['--bag',bag,'--outdir',atlas],
             [atlas/'atlas.json',atlas/'edges.json',atlas/'tracks.npz']),
            ('channels','motion_channels.py',['--bag',bag,'--atlas-dir',atlas,'--outdir',channel],[channels]),
            ('registered_map','registered_map.py',['--bag',bag,'--channels',channels,'--out',channel/'registered_map.png'],
             [channel/'registered_map.png',channel/'registered_map.json']),
            ('gyro_calibration','gyro_calibration.py',['--bag',bag,'--channels',channels,'--out',gyro],[gyro]),
            ('inertial_initialization','inertial_initialization.py',
             ['--bag',bag,'--channels',channels,'--gyro-fit',gyro,'--out',channel/'inertial_initialization.json'],
             [channel/'inertial_initialization.json'])]
        state_path=folder/'pipeline_state.json'
        previous=json.loads(state_path.read_text()) if state_path.exists() else {}
        state={'version':2,'stages':{}}
        failed=False
        for name,script,parameters,expected in commands:
            command=[sys.executable,'-u',str(ROOT/'stitch-map'/script),*[str(p) for p in parameters]]
            modules=(('homography_atlas','geometry','window_motion','heldout_diagnostics','stitch_map')
                     if name=='atlas' else all_modules)
            signature=hashlib.sha256()
            for module in modules:signature.update((ROOT/'stitch-map'/(module+'.py')).read_bytes())
            signature.update((ROOT/'bag-exploration'/'explore_bag.py').read_bytes())
            dependencies={'atlas':[], 'channels':[atlas/'edges.json',atlas/'tracks.npz'],
                'registered_map':[channels], 'gyro_calibration':[channels],
                'inertial_initialization':[channels,gyro]}[name]
            for dependency in dependencies:signature.update(dependency.read_bytes())
            signature.update(json.dumps({'path':str(bag.resolve()),'size':bag.stat().st_size,
                'mtime':bag.stat().st_mtime_ns,'command':command,'cv2':cv2.__version__,'numpy':np.__version__},
                sort_keys=True).encode())
            fingerprint=signature.hexdigest()
            prior=previous.get('stages',{}).get(name,{})
            if (isinstance(prior,dict) and prior.get('fingerprint')==fingerprint and prior.get('exit_code')==0
                    and all(p.exists() for p in expected)):
                code_result=0;action='cached'
            else:
                with (folder/(name+'.log')).open('w',encoding='utf-8') as log:
                    code_result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=False).returncode
                action='executed'
            state['stages'][name]={'exit_code':code_result,'fingerprint':fingerprint}
            atomic_json(state_path,state)
            print(f'{bag.name}: {name} exit={code_result} ({action})',flush=True)
            if code_result!=0:
                failed=True;break
        if failed:
            result={'recording':bag.name,'commands_completed':False}
        else:
            a=json.loads((atlas/'atlas.json').read_text());c=json.loads(channels.read_text())
            g=json.loads(gyro.read_text());i=json.loads((channel/'inertial_initialization.json').read_text())
            result={'recording':bag.name,'commands_completed':True,
                    'frames_total':a['frames_total'],'homography_nodes':len(a['selected_frames']),
                    'homography_nodes_connected':a['connected_to_first'],
                    'complete_homography_path':a['complete_homography_path'],
                    'rgbd_nodes':len(c['rgbd_pose_indices']),
                    'rgbd_nodes_connected':sum(v==0 for v in c['rgbd_component_ids']),
                    'rgbd_end_connected':c['rgbd_component_ids'][-1]==0,
                    'gyro_heldout_raw_deg':g.get('heldout_raw_median_deg'),
                    'gyro_heldout_calibrated_deg':g.get('heldout_calibrated_median_deg'),
                    'inertial_initialization_usable':i['initialization_usable'],
                    'metric_ground_truth_available':False}
        summary.append(result)
        atomic_json(args.output_dir/'summary.json',{'recordings':summary})
        print(json.dumps(result),flush=True)
    return 1 if any(not r['commands_completed'] for r in summary) else 0


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input-dir',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--names',nargs='*')
    args=p.parse_args()
    try:return run(args)
    except (ValueError,OSError) as exc:p.error(str(exc))


if __name__=='__main__':sys.exit(main())
