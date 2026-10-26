#!/usr/bin/env bash
# One-command end-to-end run for Stage IV (Motion / Visual Odometry):
#   1. generate the gentle + aggressive synthetic lawn datasets
#   2. run the LK+homography VO pipeline on each
#   3. metrics.json + trajectory figures land in results/<profile>/
set -euo pipefail

STAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="$STAGE_DIR/../venv/bin/python"

if [[ ! -x "$PY" ]]; then
  echo "error: venv python not found at $PY" >&2
  echo "create it first: python3 -m venv $STAGE_DIR/../venv && $STAGE_DIR/../venv/bin/pip install opencv-python numpy matplotlib scipy" >&2
  exit 1
fi

cd "$STAGE_DIR"

echo "==> generating gentle dataset"
"$PY" src/make_synthetic.py --profile gentle --out data/gentle

echo "==> generating aggressive dataset"
"$PY" src/make_synthetic.py --profile aggressive --out data/aggressive

echo "==> visual odometry on gentle"
"$PY" src/vo_lk.py --video data/gentle --gt data/gentle/groundtruth.npz --out results/gentle

echo "==> visual odometry on aggressive"
"$PY" src/vo_lk.py --video data/aggressive --gt data/aggressive/groundtruth.npz --out results/aggressive

echo "done."
echo "  metrics: results/gentle/metrics.json  results/aggressive/metrics.json"
