#!/usr/bin/env python3
"""Compare step-2 fits across modes: tests (a1), (a2), (b).

(a1) baseline vs perturbed (same windows): angle between gravity
directions, |delta b_a|, |delta b_g|.
(a2) shifted windows vs nearest primary window by centre pair
(ties -> earlier window): same metrics, median across shifted windows.
(b)  baseline vs weakprior (same windows): same metrics, median
across windows.
Thresholds (pre-specified in PROGRESS.md): 2.0 deg / 0.02 m/s^2 /
0.005 rad/s.
"""
import json
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
TAKES = ["20261007_080033", "20261007_075947", "20260926_155419"]


def load(take, mode):
    with open(os.path.join(_HERE, "runs", f"{take}_step2_{mode}.json")) as f:
        return json.load(f)


def angle_deg(a, b):
    a = np.array(a, dtype=np.float64)
    b = np.array(b, dtype=np.float64)
    return float(np.degrees(np.arccos(np.clip(float(a @ b), -1.0, 1.0))))


def compare(w1, w2):
    return {
        "dg_deg": angle_deg(w1["g_dir"], w2["g_dir"]),
        "dba": float(np.linalg.norm(np.array(w1["b_a_m_s2"])
                                    - np.array(w2["b_a_m_s2"]))),
        "dbg": float(np.linalg.norm(np.array(w1["b_g_rad_s"])
                                    - np.array(w2["b_g_rad_s"]))),
    }


def med(rows, key):
    return float(np.median([r[key] for r in rows]))


def main():
    all_a1, all_a2, all_b = [], [], []
    for take in TAKES:
        base = {w["start_pair"]: w for w in load(take, "baseline")["windows"]}
        pert = {w["start_pair"]: w for w in load(take, "perturbed")["windows"]}
        weak = {w["start_pair"]: w for w in load(take, "weakprior")["windows"]}
        shifted = load(take, "shifted")["windows"]
        print(f"== {take}")
        # (a1)
        rows = []
        for s, w in base.items():
            c = compare(w, pert[s])
            c["window"] = s
            rows.append(c)
            print(f"  (a1) window@{s}: dg={c['dg_deg']:.4f} deg "
                  f"dba={c['dba']:.5f} m/s^2 dbg={c['dbg']:.6f} rad/s")
        all_a1 += rows
        print(f"  (a1) max: dg={max(r['dg_deg'] for r in rows):.4f} deg, "
              f"dba={max(r['dba'] for r in rows):.5f}, "
              f"dbg={max(r['dbg'] for r in rows):.6f}")
        # (a2)
        rows = []
        for w in shifted:
            s = w["start_pair"]
            centre = s + w["n_pairs"] / 2.0
            cands = sorted(base.values(),
                           key=lambda p: (abs(p["start_pair"]
                                              + p["n_pairs"] / 2.0 - centre),
                                          p["start_pair"]))
            nearest = cands[0]
            c = compare(w, nearest)
            c["window"] = s
            c["nearest"] = nearest["start_pair"]
            rows.append(c)
            print(f"  (a2) shifted@{s} vs primary@{nearest['start_pair']}: "
                  f"dg={c['dg_deg']:.3f} deg dba={c['dba']:.4f} "
                  f"dbg={c['dbg']:.5f}")
        all_a2 += rows
        print(f"  (a2) median: dg={med(rows, 'dg_deg'):.3f} deg, "
              f"dba={med(rows, 'dba'):.4f}, dbg={med(rows, 'dbg'):.5f}")
        # (b)
        rows = []
        for s, w in base.items():
            c = compare(w, weak[s])
            c["window"] = s
            rows.append(c)
            print(f"  (b) window@{s}: dg={c['dg_deg']:.3f} deg "
                  f"dba={c['dba']:.4f} m/s^2 dbg={c['dbg']:.5f}")
        all_b += rows
        print(f"  (b) median: dg={med(rows, 'dg_deg'):.3f} deg, "
              f"dba={med(rows, 'dba'):.4f}, dbg={med(rows, 'dbg'):.5f}; "
              f"max dba={max(r['dba'] for r in rows):.4f}")
    print("== ALL WINDOWS")
    for name, rows in (("(a1)", all_a1), ("(a2)", all_a2), ("(b)", all_b)):
        print(f"  {name} n={len(rows)} median dg={med(rows, 'dg_deg'):.3f} "
              f"deg, dba={med(rows, 'dba'):.4f} m/s^2, "
              f"dbg={med(rows, 'dbg'):.5f} rad/s; "
              f"max dg={max(r['dg_deg'] for r in rows):.3f}, "
              f"max dba={max(r['dba'] for r in rows):.4f}, "
              f"max dbg={max(r['dbg'] for r in rows):.5f}")
    # window-to-window spread of the baseline accel bias itself
    print("== BASELINE b_a SPREAD ACROSS WINDOWS (same take)")
    for take in TAKES:
        ws = load(take, "baseline")["windows"]
        ba = np.array([w["b_a_m_s2"] for w in ws])
        g = np.array([w["g_dir"] for w in ws])
        print(f"  {take}: b_a per-axis range "
              f"{np.round(ba.max(axis=0) - ba.min(axis=0), 4)} m/s^2, "
              f"|b_a| range {np.linalg.norm(ba, axis=1).min():.3f}-"
              f"{np.linalg.norm(ba, axis=1).max():.3f}; "
              f"max pairwise g_dir angle "
              f"{max(angle_deg(g[i], g[j]) for i in range(len(g)) for j in range(len(g))):.3f} deg")
    out = {"a1": all_a1, "a2": all_a2, "b": all_b}
    with open(os.path.join(_HERE, "runs", "observability_comparison.json"),
              "w") as f:
        json.dump(out, f, indent=1)


if __name__ == "__main__":
    main()
