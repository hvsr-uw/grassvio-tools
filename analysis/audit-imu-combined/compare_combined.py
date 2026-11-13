#!/usr/bin/env python3
"""Evaluate the combined-correction test against the pre-specified
pass criteria in PROGRESS.md. Reads runs/<take>_combined.json.
Prints the full metric tables and the P1/P2/P3 verdicts."""
import json
import os

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
TAKES = ["20261007_080033", "20261007_075947", "20260926_155419"]
LOOP = TAKES[:2]


def load(take):
    with open(os.path.join(_HERE, "runs", f"{take}_combined.json")) as f:
        return json.load(f)


def vec(a):
    return np.array(a, dtype=np.float64)


def main():
    data = {t: load(t) for t in TAKES}
    print("=== Per-take effective correction (fit segment) ===")
    for t in TAKES:
        print(t)
        for sig in ("0.05", "0.2", "0.8"):
            b = vec(data[t]["per_take_correction"][sig]["b_a_m_s2"])
            print(f"  sig={sig}: b_a={np.round(b, 4)} "
                  f"rms pos {data[t]['per_take_correction'][sig]['rms_pos_m']*1000:.2f} mm")
        d02 = np.linalg.norm(
            vec(data[t]["per_take_correction"]["0.2"]["b_a_m_s2"])
            - vec(data[t]["per_take_correction"]["0.05"]["b_a_m_s2"]))
        d08 = np.linalg.norm(
            vec(data[t]["per_take_correction"]["0.8"]["b_a_m_s2"])
            - vec(data[t]["per_take_correction"]["0.05"]["b_a_m_s2"]))
        print(f"  |db_a| 0.05->0.2 = {d02:.4f} m/s^2, 0.05->0.8 = {d08:.4f}")

    print("\n=== Stability: per-window effective corrections (sig=0.05) ===")
    stab = {}
    for t in TAKES:
        wins = data[t]["stability"]["0.05"]
        b = np.array([w["b_a_m_s2"] for w in wins])
        med = np.median(b, axis=0)
        dev = np.max([np.linalg.norm(row - med) for row in b])
        stab[t] = (med, dev, b)
        print(t, f"n_windows={len(wins)}")
        print(f"  median b_a = {np.round(med, 4)}")
        print(f"  per-component range = {np.round(b.max(axis=0) - b.min(axis=0), 4)}")
        print(f"  max ||b_a - median|| = {dev:.4f} m/s^2")
        for w, row in zip(wins, b):
            print(f"    @{w['start_pair']}: {np.round(row, 4)}")

    print("\n=== Predictions: median / p90 / frac beats CV (sig=0.05) ===")
    for t in TAKES:
        print(t)
        for H in ("H1", "H2", "H6", "H4", "H8"):
            d = data[t]["predictions"]["0.05"][H]
            if d["n_starts"] == 0:
                print(f"  {H}: n=0 (no starts)")
                continue
            print(f"  {H}: n={d['n_starts']} "
                  f"comb med {d['median_imu_combined_m']*1000:.2f} mm "
                  f"p90 {d['p90_imu_combined_m']*1000:.2f} mm | "
                  f"CV med {d['median_cv_m']*1000:.2f} mm "
                  f"p90 {d['p90_cv_m']*1000:.2f} mm | "
                  f"unc med {d['median_imu_uncorrected_m']*1000:.2f} mm | "
                  f"gonly med {d['median_imu_gravity_only_m']*1000:.2f} mm | "
                  f"frac beats CV {d['frac_combined_beats_cv']:.2f}")

    print("\n=== Sanity gate: median 1-pair IMU-combined error < 10 mm ===")
    for t in TAKES:
        m = data[t]["predictions"]["0.05"]["H1"]["median_imu_combined_m"]
        print(f"  {t}: {m*1000:.2f} mm -> "
              f"{'PASS' if m < 0.010 else 'TRIPPED'}")

    print("\n=== P1: beats CV at H=1 and H=2 (median AND frac > 0.5) ===")
    p1 = {}
    for t in LOOP:
        ok = True
        for H in ("H1", "H2"):
            d = data[t]["predictions"]["0.05"][H]
            win = (d["median_imu_combined_m"] < d["median_cv_m"]
                   and d["frac_combined_beats_cv"] > 0.5)
            ok = ok and win
            print(f"  {t} {H}: comb {d['median_imu_combined_m']*1000:.2f} vs "
                  f"CV {d['median_cv_m']*1000:.2f} mm, "
                  f"frac {d['frac_combined_beats_cv']:.2f} -> "
                  f"{'win' if win else 'LOSE'}")
        p1[t] = ok
        print(f"  {t} P1: {'PASS' if ok else 'FAIL'}")

    print("\n=== P2: stability max deviation <= 0.05 m/s^2 ===")
    p2 = {}
    for t in LOOP:
        ok = stab[t][1] <= 0.05
        p2[t] = ok
        print(f"  {t}: {stab[t][1]:.4f} -> {'PASS' if ok else 'FAIL'}")

    print("\n=== P3: prior insensitivity (|db_a| <= 0.02, pred change < 25%) ===")
    p3 = {}
    for t in LOOP:
        d_ba = np.linalg.norm(
            vec(data[t]["per_take_correction"]["0.2"]["b_a_m_s2"])
            - vec(data[t]["per_take_correction"]["0.05"]["b_a_m_s2"]))
        leg1 = d_ba <= 0.02
        leg2 = True
        for H in ("H1", "H2"):
            m05 = data[t]["predictions"]["0.05"][H]["median_imu_combined_m"]
            m02 = data[t]["predictions"]["0.2"][H]["median_imu_combined_m"]
            rel = abs(m02 - m05) / m05
            if rel >= 0.25:
                leg2 = False
            print(f"  {t} {H}: median comb {m05*1000:.2f} -> "
                  f"{m02*1000:.2f} mm ({rel*100:.1f}% change)")
        print(f"  {t}: |db_a| 0.05->0.2 = {d_ba:.4f} "
              f"({'<=' if leg1 else '>'} 0.02) -> "
              f"leg1 {'PASS' if leg1 else 'FAIL'}, "
              f"leg2 {'PASS' if leg2 else 'FAIL'}")
        p3[t] = leg1 and leg2
        print(f"  {t} P3: {'PASS' if p3[t] else 'FAIL'}")

    print("\n=== OVERALL ===")
    for t in LOOP:
        print(f"  {t}: P1 {'PASS' if p1[t] else 'FAIL'}, "
              f"P2 {'PASS' if p2[t] else 'FAIL'}, "
              f"P3 {'PASS' if p3[t] else 'FAIL'}")
    print("  VERDICT: "
          + ("PASSES as a short-term prior"
             if all(p1[t] and p2[t] and p3[t] for t in LOOP) else "NULL"))


if __name__ == "__main__":
    main()
