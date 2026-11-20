import numpy as np
import pytest
from homography_atlas import chain_edges, chart_groups
from geometry import projected_corners


def edge(a,b,h):
    return {'start_frame':a,'end_frame':b,'H_target_to_source':h}


def test_render_horizon_does_not_break_a_valid_motion_chain():
    h = np.array([[1.,0,0],[0,1,0],[-.01,0,1.]])
    with pytest.raises(ValueError,match='horizon'):
        projected_corners(h,(200,200))
    indices,poses,components=chain_edges([edge(0,25,h),edge(25,50,np.linalg.inv(h))])
    assert components == [0,0,0]
    np.testing.assert_allclose(poses[-1]/poses[-1][2,2],np.eye(3),atol=1e-10)


def test_missing_motion_starts_an_explicit_component():
    indices,poses,components=chain_edges([edge(0,25,None),edge(25,50,np.eye(3))])
    assert components == [0,1,1]


def test_local_charts_keep_budget_guards_and_cover_selected_frames():
    frames=[np.zeros((20,20,3),np.uint8) for _ in range(51)]
    h=np.array([[1.,0,18.],[0,1,0],[0,0,1.]])
    charts=chart_groups(frames,[edge(0,25,h),edge(25,50,h)],max_side=45)
    assert len(charts)==2
    assert set(i for indices,_ in charts for i in indices)=={0,25,50}
    assert charts[1][0]==[25,50]
