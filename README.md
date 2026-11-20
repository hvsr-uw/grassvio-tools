# GrassVIO tools

I use these scripts for the CS 566 GrassVIO project.

## Motion recovery and channel checks

I keep homography tracking separate from the projection used to draw a
map. A full-image warp that crosses a projective horizon no longer deletes
a valid local motion link in the recovery runner. I save bounded local
maps and retain the connected homography graph. I split a failed tracking
window into shorter filtered windows before declaring a gap.

I also use each recording's calibration to register depth into the colour
camera, estimate metric RGB-D motion and draw an orthographic map. After a
depth gap, I attempt relocalization with SIFT sites that have themselves
passed temporal tracking and filtering. Unsupported gaps stay explicit.

I compare RGB-D, gyro integration and two-view SfM geometry at common
windows. I fit gyro bias and a clock-shift hypothesis on the first 20% of
each video, then score later windows. I separately test accelerometer bias,
gravity and velocity initialization. A completed command does not mean
that this inertial initialization passed its checks.

```bash
python scripts/run_motion_recovery.py --input-dir recordings --output-dir local-results/recovery
```

I save a checkpoint and log for each stage. Repeating the command reuses
completed stages when the recordings, code, dependencies and parameters
match. The full results and remaining limitations are in
[motion-recovery-results.md](analysis/motion-recovery-results.md).

The earlier first-image projection experiments below remain a separate
baseline. The local homography maps are not one metric orthomosaic, and
the RGB-D poses are a model reference rather than ground truth.

## Multiview reconstruction and refined timing

I also run image-only incremental SfM across the sampled views. I carry
temporally filtered feature sites through successive windows, triangulate
landmarks, register cameras with PnP, and jointly refine the cameras and
structure with bundle adjustment. Depth and IMU do not enter that camera
fit. I estimate one metric scale afterward from registered depth in the
initial image. I reserve feature tracks for an image-consistency check.

```bash
python scripts/run_multiview_validation.py --input-dir recordings --recovery-dir local-results/recovery --output-dir local-results/multiview --adaptive-gaps
```

This also widens the prefix-only gyro timing search, tests a constrained
inertial initializer, compares later SfM, RGB-D and gyro motion, and draws
a separate registered-depth map through the labelled SfM poses. The
modules reuse measurements when source and input fingerprints match. I
use closer keyframes and five-frame feature support when the ordinary SfM
run has unsupported views. This is a separate window setting; it does not
show that the original 25-frame setting handled that motion. I
keep the results in [multiview-results.md](analysis/multiview-results.md).
These checks do not supply independent metric ground truth.

## Homography and motion filtering

I fixed the normalized homography threshold so the inlier cutoff stays at
3 pixels. Both stitching entry points now use the same fitter. I also fixed
the loop experiment to use the actual matched endpoints and handle videos
without an endpoint match.

I added a 25-frame motion filter. It detects points at window anchors and
tracks them through the frames between them. I compare each point with the
camera-motion prediction across the window, remove points with a large
median residual, and fit RANSAC to the remaining endpoints.

The filter needs either a recording independently known to be stationary or
a supplied full camera-motion prediction, including translation. Without
one, I use a visual plane fit as a proxy. That proxy can follow moving grass,
so it does not independently separate wind from camera movement. A low gyro
reading alone does not establish that the camera is stationary.

I ran 78 regression checks and four synthetic camera/grass combinations.
The camera prediction in these tests is known exactly. The patch is
synthetic texture rather than a physical grass simulation.

| Synthetic video | Complete tracks | Static control points kept | Moving patch interior rejected | Unfiltered homography error | Filtered homography error |
|---|---:|---:|---:|---:|---:|
| Camera still, grass static | 813 | 100% | n/a | <0.001 px | <0.001 px |
| Camera still, grass moving | 813 | 100% | 100% | 9.762 px | 0.184 px |
| Camera moving, grass static | 800 | 100% | n/a | 0.007 px | 0.007 px |
| Camera moving, grass moving | 788 | 100% | 100% | 9.582 px | 0.234 px |

I measure homography error as the mean distance between the estimated and
true transformed pixels on a fixed 7-by-5 grid. I score patch interiors away
from seams and report surviving tracks. The full counts and settings are in
[synthetic_results.json](analysis/window-filter-validation/synthetic_results.json).
I also ran the September control video; its results are below.

![Synthetic source frames beside their map footprints](analysis/window-filter-validation/synthetic_debug_preview.jpg)

Install and run the checks:

```bash
python -m pip install -r requirements-dev.txt
python -m pytest -q tests
python scripts/validate_window_filter.py --outdir analysis/window-filter-validation
```

Stitch a video and put its source frames beside their warped map footprints:

```bash
python stitch-map/stitch_map.py --video input.mp4 --out map.png \
  --blend best --debug-out debug.png
```

For an independently known stationary recording:

```bash
python stitch-map/stitch_map.py --bag stationary.db3 --out filtered.png \
  --motion-filter --flow-window 25 --camera-still --debug-out debug.png
```

For a moving camera, replace `--camera-still` with
`--camera-motion motion.json`. The file holds one pixel homography per input
frame, mapping input frame 0 into that frame. Use original image coordinates
and include translation. The example below describes two frames, with the
second moving +2 pixels in x and -1 pixel in y:

```json
{
  "translation_compensated": true,
  "image_transforms": [
    [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    [[1, 0, 2], [0, 1, -1], [0, 0, 1]]
  ]
}
```

The 1.5-pixel motion cutoff is an initial setting. I still need to calibrate
it on control videos and evaluate it on separate videos. Window results use
frames only through their end anchor, with 25 frames of latency. I include
the final shorter window and report its length. Flow mode uses
`--flow-window`; descriptor mode uses `--stride`.

I leave unplaced frames out of the map. After a lost flow window, later
relative fits have no known connection to the first frame, so they remain
unplaced. The renderer rejects invalid footprints and oversized canvases
before warping. `--blend best` selects the most central source sample instead
of averaging overlapping textures. It reduces averaging blur but cannot
repair incorrect poses. The JSON includes selected frame indices, missing
poses, transforms and window diagnostics.

`dataset-tools/stitch-map/stitch_map.py` calls the same implementation.
Tree mode matches all selected frame pairs and saves a checkpoint with an
image-content hash. `--loop-check` adds older-frame matching in descriptor
chain mode. Flow mode currently supports chain placement only.

## SfM frame selection

I added a two-view frame-selection check. It waits for enough angular
parallax after camera rotation is removed, checks positive-depth support,
and also fits a plane homography. Pure rotation is not a triangulation
baseline. Strong plane support is flagged because an essential-matrix
pose can be ambiguous on a plane.

I first check whether a rotation alone explains the matched points. This
keeps a degenerate essential-matrix fit from inventing enough translation
to select a pure-rotation pair. I checked zero rotation, larger rotations,
small pixel noise and mismatches, while keeping the translation test.

```bash
python stitch-map/sfm_keyframes.py --video input.mp4 \
  --calibration camera.json --out keyframes.json --min-parallax-deg 2
```

`camera.json` contains `{"K": [[fx,0,cx],[0,fy,cy],[0,0,1]]}` with numeric
values matching the input image resolution. The input images must already
be undistorted. This script selects candidate pairs; it does not run a full
SfM reconstruction or recover metric scale.

## Real recording checks

I added a batch runner for the original camera videos. It runs the
descriptor homography and the 25-frame filtered homography separately, and
records which frames were placed, which failed, and what the fitting
diagnostics showed. I keep the raw videos, maps and logs on my own machine.

I use this command after putting the original recordings in a local folder:

```bash
python scripts/run_recording_batch.py \
  --input-dir recordings --output-dir local-results \
  --manifest recording-evidence.json --debug-images
```

It writes `summary.json` and `summary.csv` plus a separate image,
statistics file, log and restart checkpoint for each recording and method.
If I rerun the same settings, the script skips stages that already finished.
It reruns a stage when its input file, relevant code or settings change.
I can use `--dry-run` to list the files and `--rerun` to ignore checkpoints.

The optional manifest lists the camera evidence for each video. For a video
that only **starts** stationary, I do not label its entire run stationary:

```json
{
  "recordings": {
    "example.db3": {
      "camera_mode": "visual_plane_only",
      "starts_at_rest": true
    }
  }
}
```

The `known_stationary_whole_recording` mode requires independent
confirmation that the camera stayed stationary throughout the full video.
The `independent_full_motion` mode requires a path to one
full camera-motion pixel homography for every frame, including translation.
Without either, the filter fits a visual-plane model from the imagery, so
it cannot independently identify moving grass or measure camera truth.

For each filtered window I fit two **separate diagnostic homographies** from
checkerboard training areas: before and after temporal filtering. I measure
both on tracked points in the other checkerboard areas. The test points do
not enter either diagnostic fit or the temporary plane estimate used for
filtering. I keep the normal full-data transform used for map placement
unchanged. The per-window held-out error is still image consistency rather
than ground-truth camera pose, so lower error does not prove wind detection
or agreement among the motion channels.

I include both unfiltered and filtered held-out errors in the summary and
use the median across windows. I also report the median paired change on
windows where both diagnostic fits succeeded; negative means lower error
after filtering. Missing or non-finite scores do not enter those summaries.

I ran all 173 colour frames from `20260926_102608b.db3`. The descriptor
baseline placed 35 selected frames, and the 25-frame filter placed eight
window anchors across seven windows. Neither run skipped a selected pair.
These methods select different numbers of frames, so the counts are not
a direct accuracy comparison. The median held-out errors before and after
filtering were both 0.05277 pixels, and the median paired change was zero.
This video does not show an improvement from filtering. I used the visual
plane fallback because I have no independent whole-video motion evidence.
The SQLite integrity check passed, and I kept the maps, original-frame
comparisons and logs locally.

I also ran all 4,080 colour frames from six November 19 recordings through
both methods. Neither method placed a complete sequence on any video.
Five filtered runs and two descriptor runs rendered partial maps; the
other maps exceeded the canvas budget. The first lost filtered window in
every video still had 431 to 1,034 inliers, but its composed transform put
a projective horizon inside the frame. Direct first-to-end matches still
found 1,442 to 1,933 inliers at the filtered endpoints. I saved the exact
counts and limitations in
[the November results](analysis/november-recording-validation.md).

I save the placement and window diagnostics even when rendering is
rejected. The command still fails and no new map is written. The batch
runner can reuse those diagnostics on the next run, and it reports map
rendering separately from complete placement. A command that rendered a
partial map does not count as a complete sequence.

I inspect sensor streams and check direct image loops with:

```bash
python scripts/inspect_recordings.py --input-dir recordings --output-dir local-results/sensors
python scripts/check_recording_loops.py --input-dir recordings --results-dir local-results/maps
```

I have not completed the full three-channel comparison. In particular,
a quiet first second can help initialize
velocity, but does not by itself establish accelerometer bias, gravity
orientation or long-term metric translation.

## Windows laptop with UW Box Drive

I use the Windows launcher when the camera recordings are already on my
computer. It avoids rebuilding a temporary work environment each time and
runs the baseline and motion-filter checks without a chat session.

I install [Box Drive](https://support.box.com/hc/en-us/articles/360043695994-Download-Box-Drive-and-Edit),
sign in with my UW account and find the recordings folder in File Explorer.
If the folder is visible and I have the necessary download permission, I
mark just those recordings **Always keep on this device**, then wait for
the offline checkmarks. Box Drive can resume interrupted downloads.

I clone the review branch and run this command from that checkout:

§§§powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_local_windows.ps1 -SourceDir "PATH_TO_OFFLINE_RECORDINGS_FOLDER" -UseSourceInPlace
§§§

This runs from the offline Box Drive folder without copying the whole data
set again. The script never changes the source files. I can instead
download the recordings from the Box website into an ordinary directory
and run the same command without the `-UseSourceInPlace` option. In
that mode the script copies matching files into its own private local
folder before testing them. It selects the November 19 recordings and
the 080033 comparison by filename, and I can override `-NamePattern`
for another set.

The script clones or updates the reviewed code in its own separate
workspace, creates a Python environment, checks the SQLite topic tables,
then runs the image-motion batch script. Outputs stay in that private
workspace. It starts with 300 frames per video to limit memory use.
I can specify `-MaxFrames 0` for full length after checking available RAM.
The batch script stores logs and checkpoints, so identical completed
runs are skipped. I keep my laptop awake while a run is active.

If the folder is available only through a shared link and does not appear
in Box Drive, I use the Box website to download the individual recordings.
Only files I have permission to access are included; no login tokens
are stored, and nothing is uploaded to GitHub or Box by these scripts.

I pass the selected recording names to the batch runner so other recordings
in the same input directory are left out.

I have validated the launcher's syntax with automated checks, but I
have not run a complete Box-to-Windows experiment in this environment.
These image-consistency comparisons do not finish the IMU/SfM
three-channel validation.

## Other scripts and earlier results

I use `bag-exploration/explore_bag.py` to read the ROS 2 SQLite recordings
without a ROS installation.

`imu-integration-filtering/imu_fusion.py` compares accelerometer integration
with visual frame-pair distances. It writes `per_pair_distances.csv`,
`accel_filtered.csv` and `results.json`. It requires the shared features
package on `PYTHONPATH`. The earlier analyses show that IMU translation is
still unreliable; these outputs do not establish three-channel agreement.

`descriptor-benchmark/benchmark.py` compares ORB and HOG on consecutive
frame pairs, and `friday-tasks/friday_tasks.py` contains the earlier
frame-distance, RANSAC and IMU checks. Both need the shared features package.

`dataset-validation/validate.py` checks sequence folders against the schema.
`hog-work/` contains Shi-Tomasi corners with HOG descriptors and a matching
check. `stage4-motion/` contains the synthetic lawn generator, Lucas-Kanade
visual odometry and track statistics.

The earlier SIFT stitching results were 672 by 507 pixels on the September
video and 2204 by 1891 pixels on the October 7 video. The longer video
stretched near its far end. Those numbers belong to the earlier matcher;
they have not been rerun with the corrected fitter and reciprocal matching.
A high pairwise inlier count does not establish an accurate chained path.

I have now tested recovery on six original November 19 videos and added
multiview SfM with a scale estimated from initial depth. Independent IMU
translation still fails validation. I need stronger initialization data,
verified camera motion and a metric reference before claiming agreement
between all three channels.

I keep the related-work findings in
[motion-filter-related-work.md](research/motion-filter-related-work.md).
