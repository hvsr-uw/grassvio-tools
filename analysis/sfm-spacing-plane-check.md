# SfM frame spacing and plane comparison

I added a separate check for the frame-spacing question from our GrassVIO
experiments. Full multiview SfM already registers cameras and refines 3D points.
This check asks whether looking farther ahead produces more usable triangulation
parallax, and whether a single plane still predicts the visible grass well.

## What I measure

I sample each video at regular anchor frames and compare each anchor with
frames 5, 25 and 50 frames later. The actual time gap is also saved, because
the video rate may not be perfectly constant. I use recorded colour calibration
to rectify matched image coordinates and reciprocal SIFT matching with a 0.75
ratio test. The wider gap is not assumed to be better: if fewer sites overlap,
that pair is recorded as unsupported.

I reserve spatial checkerboard image tiles before fitting a homography. I fit
the plane transform on the other tiles using 2.5-pixel RANSAC, then report the
median and 90th-percentile image transfer errors on the reserved tiles. The
training inlier fraction and the held-out error answer different questions.
A plane can fit short-baseline views even when 3D structure exists, and a
small plane error alone does not prove that grass is perfectly flat or still.

For each supported frame pair, I check two-view essential-matrix geometry.
I first reject motion that can be explained by rotation alone. I then require
at least 2 degrees of **rotation-compensated parallax** between viewing rays
to call the pair usable for triangulation. This is different from requiring
2 degrees of camera rotation: rotation by itself gives no 3D baseline.
A highly planar scene can still make the essential decomposition ambiguous,
so eligibility is a diagnostic rather than an accuracy certificate.

When both sampled frames exist in the full SfM reconstruction, I also compute
the epipolar Sampson residual using the SfM camera poses. I score these poses
on the same spatially reserved correspondences. This is a one-dimensional
epipolar residual; the homography error is a two-dimensional image transfer
error, so I do not compare their numbers as if they measured the same thing.
The multiview reconstruction uses the same video, so neither measurement is
independent metric ground truth.

## Reproduce on the recorded videos

First run the existing recovery and multiview stages, if they have not
already completed:

```bash
python -m pip install -r requirements-dev.txt
python scripts/run_motion_recovery.py --input-dir recordings --output-dir local-results/recovery
python scripts/run_multiview_validation.py --input-dir recordings --recovery-dir local-results/recovery --output-dir local-results/multiview --adaptive-gaps
```

Then run this independent comparison on the six November 19 videos:

```bash
python scripts/run_sfm_spacing_plane_check.py \
  --input-dir recordings \
  --multiview-dir local-results/multiview \
  --output-dir local-results/sfm-spacing \
  --names 20261119_073137.db3 20261119_073608.db3 20261119_073711.db3 20261119_074041.db3 20261119_074125.db3 20261119_074200.db3 \
  --gaps 5 25 50 --anchor-step 50
```

The batch script chooses `dense_sfm/reconstruction.json` if a recording
needs its adaptive reconstruction, otherwise the ordinary
`sfm/reconstruction.json`. It writes each video's detailed JSON and CSV,
plus `summary.json`, `summary.csv` and `report.md` across all videos.
It also works without SfM poses, but marks those optional pose comparisons
as unavailable. Existing reconstructions and raw recordings stay unchanged.

For one video without a prior multiview result:

```bash
python stitch-map/sfm_spacing_plane_check.py \
  --bag recordings/20261119_073137.db3 \
  --out local-results/sfm-spacing/single.json \
  --gaps 5 25 50 --anchor-step 50
```

## How I interpret the result

I compare the fraction of attempted pairs that retain enough matches,
the fraction of evaluated pairs with adequate parallax, the held-out
plane-transfer errors, and the measured time and camera motion gaps.
I do not discard failed wide pairs when comparing spacing choices.
A farther frame is useful if it increases usable parallax **without
losing overlap**. A wider gap that produces no match is not a success.

I keep this comparison separate from the image-only full SfM result.
The existing six-video reconstruction registered all 173 original sampled
views, and 203 views after the additional fast-video keyframes. Those
figures and the reserved-track errors are in
[multiview-results.md](multiview-results.md). They do not by themselves
answer the near-versus-far plane question.

The numerical near-versus-far results require rerunning the checker against
the original .db3 videos. I have not added an unrun results table or inferred
those numbers from the existing multiview report.

## Code and checks

- `stitch-map/sfm_spacing_plane_check.py`: calibration, matching, spatial
  holdout, homography fitting, parallax test and optional SfM pose check.
- `scripts/run_sfm_spacing_plane_check.py`: deterministic batch runner and
  combined result tables.
- `tests/test_sfm_spacing_plane_check.py`: synthetic plane and 3D controls,
  rotation-only and tiny-translation rejection, missing matches and pose
  comparison checks.

I keep the recordings, detailed correspondence measurements and locally
generated figures outside the shared repository.
