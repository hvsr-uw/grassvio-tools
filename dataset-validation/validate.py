#!/usr/bin/env python3
"""Validate one GrassVIO sequence folder against the dataset spec.

Usage:
    python validate.py <sequence-dir> [--dataset-root ROOT] [--out PATH] [--quiet]

<sequence-dir> is e.g. grassvio-dataset/sequences/2026-10-04_A_still_still_01.

By default the JSON report is written to
    <dataset-root>/derived/validation/<seq-id>.json
where <dataset-root> is inferred as the grandparent of <sequence-dir>
(i.e. <sequence-dir> must live in <root>/sequences/). Use --dataset-root
to set the root explicitly, or --out to choose the report path directly.

Exit code: 0 if overall result is pass/warn, 1 if fail, 2 on usage errors.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from grassvio.checks import run_all_checks  # noqa: E402


def infer_dataset_root(seq_dir):
    parent = os.path.basename(os.path.dirname(os.path.abspath(seq_dir)))
    if parent == "sequences":
        return os.path.dirname(os.path.dirname(os.path.abspath(seq_dir)))
    return None


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Run GrassVIO Section 7 validation checks on one sequence.")
    ap.add_argument("sequence_dir", help="path to the sequence folder")
    ap.add_argument("--dataset-root", default=None,
                    help="dataset root (contains sequences/ and derived/)")
    ap.add_argument("--out", default=None,
                    help="write the JSON report to this path instead of the default")
    ap.add_argument("--quiet", action="store_true",
                    help="only print the overall result")
    args = ap.parse_args(argv)

    seq_dir = os.path.abspath(args.sequence_dir)
    if not os.path.isdir(seq_dir):
        print("error: not a directory: %s" % seq_dir, file=sys.stderr)
        return 2

    report = run_all_checks(seq_dir)
    seq_id = report["sequence_id"]

    if args.out:
        out_path = os.path.abspath(args.out)
    else:
        root = args.dataset_root or infer_dataset_root(seq_dir)
        if root is None:
            print("error: cannot infer dataset root from %s; "
                  "pass --dataset-root or --out" % seq_dir, file=sys.stderr)
            return 2
        out_path = os.path.join(root, "derived", "validation", seq_id + ".json")

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(report, f, indent=2)

    if args.quiet:
        print(report["overall"])
    else:
        print("sequence : %s" % seq_id)
        print("overall  : %s" % report["overall"].upper())
        for name, chk in report["checks"].items():
            tag = chk["status"].upper().ljust(4)
            first = chk["messages"][0] if chk["messages"] else ""
            print("  [%s] %-15s %s" % (tag, name, first))
        print("report   : %s" % out_path)

    return 0 if report["overall"] in ("pass", "warn") else 1


if __name__ == "__main__":
    sys.exit(main())
