#!/bin/bash
# Run the inlier-rate measurement over the 14-take sample.
# Each take writes its own JSON to results/ as it finishes, so a VM
# restart loses at most the take in progress. Re-running skips takes
# whose result file already exists.
set -u
cd "$(dirname "$0")"
CS="$(cd ../../../.. && pwd)"
PY="$CS/hog-work/.venv/bin/python"
DL="$(dirname "$CS")/browser_downloads"
run() {
  stamp="$1"; bag="$2"
  out="results/${stamp}.json"
  if [ -f "$out" ]; then echo "skip $stamp (done)"; return; fi
  echo "=== $stamp ==="
  "$PY" inlier_rate.py --bag "$bag" --out "$out" --max-pairs 40
}
run 20261007_080033 "$DL/sess-3462837304/browser-download-20261017T230726.262074045Z-1-20261007_080033.db3"
run 20261007_075947 "$DL/sess-3462837304/browser-download-20261017T231319.031772531Z-6-20261007_075947.db3"
run 20260926_155419 "$DL/sess-3462837304/browser-download-20261017T231007.866894491Z-3-20260926_155419.db3"
run 20261026_172532 "$DL/sess-197710181/browser-download-20261027T051452.343670070Z-1-20261026_172532.db3"
run 20261026_172828 "$DL/sess-197710181/browser-download-20261027T054254.650491392Z-3-20261026_172828.db3"
run 20261026_172859 "$DL/sess-1861756418/browser-download-20261027T084815.498833532Z-15-20261026_172859.db3"
run 20260926_102631c "$DL/sess-876766658/browser-download-20261009T144901.767060640Z-1-20260926_102631c.db3"
run 20261030_174745 "$DL/sess-rotation-nov01/box-20261030_174745.db3"
run 20261106_085611 "$DL/sess-2653029502/browser-download-20261106T171330.872403110Z-8-20261106_085611.db3"
run 20261026_173701 "$DL/sess-1861756418/browser-download-20261027T084946.920480102Z-16-20261026_173701.db3"
run 20261026_173807 "$DL/sess-1191256009/browser-download-20261027T065046.915693692Z-1-20261026_173807.db3"
run 20261026_172912 "$DL/sess-197710181/browser-download-20261027T062112.628186968Z-6-20261026_172912.db3"
run 20261026_173250 "$DL/sess-1191256009/browser-download-20261027T073314.009820058Z-5-20261026_173250.db3"
run 20261026_173507 "$DL/sess-1191256009/browser-download-20261027T080549.321812416Z-10-20261026_173507.db3"
echo ALL_DONE
