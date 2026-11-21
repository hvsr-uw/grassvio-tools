# Re-running the 173701 pre-RANSAC wind-filtering experiment

The historical measurements and plots are in [the main report](README.md).
The original Python file that generated those published measurements was not
available in the GitHub branches when this follow-up was prepared.

The new [recording evaluation script](../../../scripts/reproduce_wind_pre_ransac_173701.py)
implements the documented method independently. It is **not** claimed to be
the missing original source file or to have reproduced the original numbers.
Keep the original CSV, figures and report unchanged until the real recording
has been rerun and the newly produced measurements have been compared.

## Run

From the repository root, install Python dependencies:

```bash
python -m pip install -r analysis/check-list-report/panorama-080033/requirements.txt
```

Place the authorized `20261026_173701.db3` recording in a local
`recordings/` directory, or supply its actual path:

```bash
python scripts/reproduce_wind_pre_ransac_173701.py \
  --bag recordings/20261026_173701.db3 \
  --outdir local-results/wind-173701-check \
  --gaps 5 25
```

This is read-only with respect to the recording. No ROS 2 installation is
needed. Outputs are kept under `local-results/`, not the published report
directory, to avoid overwriting historical results.

The generated files are `173701_window_results.csv`,
`173701_inlier_rate_and_alignment.png`,
`173701_match_decisions_frames_195_200.png` (if that pair is available), and
`173701_reproduction_summary.json`.

## Method

1. Match endpoint SIFT features with a 0.75 Lowe ratio check.
2. Track those matches through every intermediate frame using Lucas-Kanade
   optical flow. Require a forward/backward error at most 1 px and endpoint
   agreement with SIFT at most 2 px.
3. Reserve alternate 40-pixel checkerboard tiles from the first image as the
   evaluation set.
4. Estimate an intermediate provisional homography with LMedS using **only
   the non-held-out tracks**. Reject a matched feature if its median temporal
   residual is over 1.5 px, before RANSAC.
5. Use a 3 px homography RANSAC threshold for the raw, flow-only, and filtered
   fits. Calculate inlier rates against each fit's own candidate population.
6. Evaluate all three homographies against the **same flow-confirmed held-out
   points**. Missing homographies remain missing, not silently replaced.

The CSV records attempted matches, filter decisions, raw and filtered
inlier fractions, and held-out alignment errors per window. The default run
covers complete five-frame pairs and up-to-25-frame windows.

## Before claiming replication

Compare the new output CSV against
[`173701_window_results.csv`](173701_window_results.csv). The historical
report gives 87.2% versus 98.3% median inlier fractions over 50 five-frame
pairs, and held-out alignment errors of 1.097 px versus 1.053 px.

Those values are **archived measurements**, not outcomes from this new script.
Differences may reflect the original script's choices of SIFT matching,
feature tracking or provisional plane fitting, which are not fully recoverable
from the published files. If they differ, document and investigate the
discrepancy rather than overwriting the published figures.

The new script is covered by synthetic unit tests, but synthetic testing
cannot validate those historical real-recording values.
