"""Relocalize an endpoint using only temporally supported feature sites."""
import cv2
import numpy as np
from window_motion import track_window,temporal_filter


def filtered_sequence_features(grays):
    """Track actual SIFT sites before admitting their descriptors to recovery."""
    keys,descriptors=cv2.SIFT_create(nfeatures=3000).detectAndCompute(grays[0],None)
    if descriptors is None:return [],None
    points=np.float32([key.pt for key in keys])
    history,_,identities=track_window(grays,initial_points=points,return_ids=True)
    keep,_=temporal_filter(history,residual_threshold_px=1.5)
    indices=identities[keep]
    return [keys[i] for i in indices],descriptors[indices] if len(indices) else None


def supported_features(gray,supported_points,radius_px=2.):
    keypoints,descriptors=cv2.SIFT_create(nfeatures=10000).detectAndCompute(gray,None)
    sites=np.asarray(supported_points,np.float32).reshape(-1,2)
    if descriptors is None or not len(sites):return [],None
    coordinates=np.float32([key.pt for key in keypoints])
    matcher=cv2.BFMatcher()
    reverse={m.queryIdx:m.trainIdx for m in matcher.match(sites,coordinates)}
    indices=[m.queryIdx for m in matcher.match(coordinates,sites)
             if m.distance<=radius_px and reverse.get(m.trainIdx)==m.queryIdx]
    return [keypoints[i] for i in indices],descriptors[indices] if indices else None


def match_supported(anchor,current):
    ka,da=anchor;kb,db=current
    if da is None or db is None or min(len(da),len(db))<2:
        return np.empty((0,2)),np.empty((0,2))
    matcher=cv2.BFMatcher()
    reverse={m.queryIdx:m.trainIdx for m in matcher.match(db,da)}
    matches=[p[0] for p in matcher.knnMatch(da,db,k=2)
             if len(p)==2 and p[0].distance<.75*p[1].distance and reverse.get(p[0].trainIdx)==p[0].queryIdx]
    return np.array([ka[m.queryIdx].pt for m in matches]).reshape(-1,2),np.array([kb[m.trainIdx].pt for m in matches]).reshape(-1,2)
