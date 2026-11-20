import pytest

from adaptive_sfm import densify_gaps


def test_dense_sampling_brackets_missing_views_and_keeps_existing_anchors():
    assert densify_gaps([0, 25, 50, 75], {50}) == [0, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75]
    assert densify_gaps([0, 25, 50], set()) == [0, 25, 50]
    with pytest.raises(ValueError, match='positive'):
        densify_gaps([0, 25], {25}, step=0)
