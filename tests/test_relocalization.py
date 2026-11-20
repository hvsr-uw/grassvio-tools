import cv2
import numpy as np
from relocalization import supported_features,match_supported,filtered_sequence_features


def test_no_unfiltered_relocalization_when_temporal_support_is_empty():
    gray=np.random.default_rng(65).integers(0,256,(160,220),dtype=np.uint8)
    keypoints,descriptors=supported_features(gray,np.empty((0,2)))
    assert not keypoints and descriptors is None


def test_relocalization_features_stay_near_supported_track_sites():
    gray=np.random.default_rng(66).integers(0,256,(180,240),dtype=np.uint8)
    detected,_=cv2.SIFT_create().detectAndCompute(gray,None)
    sites=np.array([k.pt for k in detected[:40]],dtype=np.float32)
    keys,descriptors=supported_features(gray,sites)
    assert len(keys)>20
    for key in keys:assert np.linalg.norm(sites-np.array(key.pt),axis=1).min()<=2.
    a,b=match_supported((keys,descriptors),(keys,descriptors))
    assert len(a)>20
    np.testing.assert_allclose(a,b)


def test_recovery_uses_temporally_tracked_sift_points():
    gray=np.random.default_rng(67).integers(0,256,(180,240),dtype=np.uint8)
    keys,descriptors=filtered_sequence_features([gray]*6)
    assert len(keys)>20
    assert len(descriptors)==len(keys)
    a,b=match_supported((keys,descriptors),(keys,descriptors))
    assert len(a)>20
    np.testing.assert_allclose(a,b)
