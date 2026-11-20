# November 19 recording results

I ran all 4,080 colour frames from six original November 19 recordings.
I used SIFT at stride five for the descriptor baseline, and a 25-frame
window with a 1.5-pixel temporal residual cutoff for the filtered run.
Both used the corrected pixel-space homography fitter and the `best`
blend. I left the 8,000-pixel side and 16-million-pixel canvas limits in
place. I used the visual-plane fallback because I have no independently
verified whole-video camera-motion prior.

## Map coverage

I did not get a complete sequence from either method on any video.
The frame counts below are placed frames divided by selected frames.
Descriptor frames and flow anchors are sampled differently, so their
counts are not a direct accuracy comparison. Frame indices start at zero.

| Video | Colour frames | Descriptor placed | Filtered placed | Last placed descriptor frame | Last placed filtered frame | Descriptor map | Filtered map |
|---|---:|---:|---:|---:|---:|---|---|
| 073137 | 901 | 52/181 | 11/37 | 255 | 250 | Rejected | Rejected |
| 073608 | 818 | 46/164 | 9/34 | 250 | 200 | Rejected | Partial |
| 073711 | 687 | 43/138 | 9/29 | 210 | 200 | Rejected | Partial |
| 074041 | 818 | 45/164 | 9/34 | 220 | 200 | Partial | Partial |
| 074125 | 470 | 43/94 | 9/20 | 210 | 200 | Partial | Partial |
| 074200 | 386 | 27/78 | 6/17 | 130 | 125 | Rejected | Partial |

I rejected five oversized canvases: descriptor 073137 at 27,375 by 5,471,
filtered 073137 at 9,062 by 1,911, descriptor 073608 at 81,763 by 3,609,
descriptor 073711 at 9,287 by 883, and descriptor 074200 at 9,546 by
1,665 pixels. I did not increase the budget or remove frames to make
these full-video runs appear successful.

## First lost window and the return view

I found the same immediate placement failure in all six filtered runs:
the composed transform put a projective horizon inside the full image.
The local pair fit still had hundreds of inliers. After that failure,
the current flow chain has no verified bridge, so later relative fits
remain unplaced.

I also matched the first image directly to the final filtered anchor.
All six return views had strong direct matches, but neither chained
method had an endpoint pose. I kept these direct fits separate from the
maps; they do not repair the chain or supply metric ground truth.

| Video | First lost filtered frame | Local window inliers | Final filtered frame | Direct endpoint inliers | Direct endpoint median error (px) |
|---|---:|---:|---:|---:|---:|
| 073137 | 275 | 988 | 900 | 1442 | 0.230213 |
| 073608 | 225 | 959 | 817 | 1933 | 0.250687 |
| 073711 | 225 | 862 | 686 | 1823 | 0.236389 |
| 074041 | 225 | 1034 | 817 | 1909 | 0.218705 |
| 074125 | 225 | 669 | 469 | 1826 | 0.259101 |
| 074200 | 150 | 431 | 385 | 1855 | 0.210073 |

I could not fit a direct first-image homography at the first lost frame
on any video: those checks had only one to six reciprocal descriptor
matches. That limits what I can infer about the failure. The horizon
rejection alone does not separate accumulated model error from the
limitations of projecting the whole scene into the first camera image.
I still need suitable map geometry and a verified way to reconnect a
lost chain.

## Held-out image consistency

I compared separate unfiltered and filtered fits on spatially held-out
tracks in each window. The table uses medians across windows. Paired
change uses only windows where both fits produced finite scores; a
negative value means lower error after filtering. Errors are rounded
to six decimal places. These scores cover local windows, including
windows that could not be placed in the map.

| Video | Unfiltered median (px) | Filtered median (px) | Median paired change (px) | Paired windows / all windows |
|---|---:|---:|---:|---:|
| 073137 | 0.706732 | 0.697578 | -0.001905 | 36/36 |
| 073608 | 0.822820 | 0.812729 | 0.000000 | 33/33 |
| 073711 | 0.741019 | 0.744161 | 0.000000 | 28/28 |
| 074041 | 0.730214 | 0.727242 | 0.000000 | 33/33 |
| 074125 | 0.905159 | 0.890966 | -0.001694 | 19/19 |
| 074200 | 0.650731 | 0.646362 | 0.000000 | 13/16 |

I do not see evidence here that this filter fixes full-sequence mapping.
Small changes in image residuals do not establish physical wind detection
or correct camera translation.

## Sensors and remaining comparison

I checked SQLite integrity on all six files, and all passed. Every file
has colour, depth, gyro and accelerometer messages with increasing header
timestamps. The median nearest colour/depth offsets range from 6.257 to
6.412 milliseconds. The first-second median gyro magnitudes range from
0.005050 to 0.020804 rad/s. I saved each recording's own camera calibration,
extrinsics, timing and startup sensor measurements.

I have not independently verified zero starting velocity or whole-video
stationarity. I have not used a quiet first second as proof of either.
These experiments do not finish the visual/IMU/SfM comparison. The older
080033 comparison recording was not among the six files I received.

## Repeating the checks

I used these commands with the original recordings in a local folder:

```bash
python scripts/run_recording_batch.py --input-dir recordings --output-dir local-results/maps --max-frames 0 --debug-images
python scripts/inspect_recordings.py --input-dir recordings --output-dir local-results/sensors
python scripts/check_recording_loops.py --input-dir recordings --results-dir local-results/maps
```

I save statistics even when a map is rejected. The batch command returns
a failure status for those runs, and the summary separates rendering
from complete placement. A repeat with unchanged inputs and code reused
all 12 stage results, including the five rejected renders.

I ran 48 Python regression tests and all four synthetic camera/grass
experiments. The synthetic errors reproduced the earlier results. I kept
the original videos, maps, source-frame panels and detailed logs locally.
