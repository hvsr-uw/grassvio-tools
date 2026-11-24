from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from run_sfm_spacing_synthetic_controls import run_case


def test_wider_baseline_reduces_known_pose_triangulation_error():
    flat = run_case(0.)
    assert [r["gap_frames"] for r in flat] == [5, 25, 50]
    assert flat[2]["median_true_parallax_deg"] > 2.
    assert flat[0]["median_true_parallax_deg"] < 1.
    assert (flat[2]["median_known_pose_triangulation_error_m"]
            < flat[0]["median_known_pose_triangulation_error_m"] / 5)


def test_one_homography_fits_flat_grass_but_not_depth_varying_surface():
    flat = run_case(0.)
    uneven = run_case(.4)
    assert all(r["median_plane_holdout_px"] < .6 for r in flat)
    assert (uneven[2]["median_plane_holdout_px"]
            > 3 * uneven[0]["median_plane_holdout_px"])
