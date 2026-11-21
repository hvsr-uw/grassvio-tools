import importlib.util
from pathlib import Path

import numpy as np
import pytest
from recording_camera import DepthPairingError

spec=importlib.util.spec_from_file_location('inspect_archived_controls',
    Path(__file__).resolve().parents[1]/'scripts/inspect_archived_controls.py')
controls=importlib.util.module_from_spec(spec)
spec.loader.exec_module(controls)


def test_missing_depth_stays_unavailable_and_is_never_identity_motion():
    class MissingDepth:
        def cloud_for_color(self,index):
            raise DepthPairingError('colour/depth offset exceeds pairing limit')
    transform,stats=controls.measure_rgbd(MissingDepth(),0,np.zeros((2,40,2)),np.ones(40,bool))
    assert transform is None
    assert stats['status']=='unavailable_depth_pair_over_20_ms'


def test_other_depth_failures_are_not_hidden_as_pairing_gaps():
    class BrokenDepth:
        def cloud_for_color(self,index):
            raise ValueError('invalid image encoding')
    with pytest.raises(ValueError,match='encoding'):
        controls.measure_rgbd(BrokenDepth(),0,np.zeros((2,40,2)),np.ones(40,bool))


def test_colour_evaluation_fits_only_training_tiles(monkeypatch):
    rng=np.random.default_rng(566)
    points=rng.uniform([0,0],[400,300],(300,2))
    history=np.stack([points,points+[2.,1.],points+[4.,2.]])
    observed=[]
    original=controls.temporal_filter
    def spy(tracks,*args):
        observed.append(tracks.shape[1])
        return original(tracks,*args)
    monkeypatch.setattr(controls,'temporal_filter',spy)
    _,report=controls.compare_filters(history,np.full((300,400,3),[0,255,0],np.uint8))
    training=report['temporal']['heldout_train_tracks']
    assert 0<training<len(points)
    assert observed==[training,training,len(points)]
    assert report['temporal']['heldout_test_tracks']==report['temporal_and_colour']['heldout_test_tracks']
    assert report['semantic_extra_rejections']==0
    assert report['wind_cause_verified'] is False
