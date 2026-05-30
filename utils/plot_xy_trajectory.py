#!/usr/bin/env python3
"""Plot GT and estimated TUM trajectories on the XY plane.

The estimated trajectory is aligned to GT with a 2D Sim(2) transform because
monocular SLAM has arbitrary scale. Input files are TUM format:
  timestamp tx ty tz qx qy qz qw
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np


def load_tum(path: Path) -> np.ndarray:
    rows = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            rows.append([float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])])
    if not rows:
        raise ValueError(f"No TUM poses found: {path}")
    return np.asarray(rows, dtype=np.float64)


def match_by_time(gt: np.ndarray, est: np.ndarray, max_dt: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    gt_t = gt[:, 0]
    pairs = []
    dts = []
    for i, t in enumerate(est[:, 0]):
        j = int(np.searchsorted(gt_t, t))
        candidates = []
        if j < len(gt_t):
            candidates.append(j)
        if j > 0:
            candidates.append(j - 1)
        if not candidates:
            continue
        best = min(candidates, key=lambda k: abs(gt_t[k] - t))
        dt = abs(gt_t[best] - t)
        if dt <= max_dt:
            pairs.append((best, i))
            dts.append(dt)
    if not pairs:
        raise ValueError(f"No timestamp matches within max_dt={max_dt}")
    gt_idx = np.asarray([p[0] for p in pairs], dtype=np.int64)
    est_idx = np.asarray([p[1] for p in pairs], dtype=np.int64)
    return gt_idx, est_idx, np.asarray(dts, dtype=np.float64)


def sim2_align(src_xy: np.ndarray, dst_xy: np.ndarray) -> tuple[np.ndarray, float, np.ndarray]:
    """Return R, scale, t such that scale * src @ R.T + t ~= dst."""
    src_mean = src_xy.mean(axis=0)
    dst_mean = dst_xy.mean(axis=0)
    src_c = src_xy - src_mean
    dst_c = dst_xy - dst_mean
    cov = (dst_c.T @ src_c) / len(src_xy)
    u, svals, vt = np.linalg.svd(cov)
    d = np.eye(2)
    if np.linalg.det(u @ vt) < 0:
        d[-1, -1] = -1
    r = u @ d @ vt
    var_src = np.mean(np.sum(src_c * src_c, axis=1))
    if var_src <= 0:
        raise ValueError("Estimated trajectory has zero XY variance; cannot align")
    scale = float(np.trace(np.diag(svals) @ d) / var_src)
    t = dst_mean - scale * (src_mean @ r.T)
    return r, scale, t


def apply_sim2(xy: np.ndarray, r: np.ndarray, scale: float, t: np.ndarray) -> np.ndarray:
    return scale * (xy @ r.T) + t


def set_equal_aspect_with_margin(ax, xy_arrays: list[np.ndarray], margin_ratio: float = 0.06) -> None:
    xy = np.concatenate([a[:, :2] for a in xy_arrays if len(a)], axis=0)
    xmin, ymin = xy.min(axis=0)
    xmax, ymax = xy.max(axis=0)
    span = max(xmax - xmin, ymax - ymin)
    if span <= 0:
        span = 1.0
    cx = 0.5 * (xmin + xmax)
    cy = 0.5 * (ymin + ymax)
    half = 0.5 * span * (1.0 + margin_ratio)
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)
    ax.set_aspect("equal", adjustable="box")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gt-tum", required=True, type=Path)
    parser.add_argument("--est-tum", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--out-csv", type=Path)
    parser.add_argument("--max-dt", type=float, default=0.3)
    parser.add_argument("--title", default="XY trajectory")
    args = parser.parse_args()

    gt = load_tum(args.gt_tum)
    est = load_tum(args.est_tum)
    gt_idx, est_idx, dts = match_by_time(gt, est, args.max_dt)

    gt_match_xy = gt[gt_idx, 1:3]
    est_match_xy = est[est_idx, 1:3]
    r, scale, trans = sim2_align(est_match_xy, gt_match_xy)
    est_xy_aligned = apply_sim2(est[:, 1:3], r, scale, trans)
    est_match_aligned = est_xy_aligned[est_idx]

    err = np.linalg.norm(est_match_aligned - gt_match_xy, axis=1)

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8.5, 8.0), dpi=180)
    ax.plot(gt[:, 1], gt[:, 2], color="#9aa0a6", linewidth=1.6, label=f"GT full ({len(gt)})")
    ax.plot(gt_match_xy[:, 0], gt_match_xy[:, 1], color="#2f7d32", linewidth=1.2, marker="o", markersize=2.2, label=f"GT matched ({len(gt_idx)})")
    ax.plot(est_xy_aligned[:, 0], est_xy_aligned[:, 1], color="#c62828", linewidth=1.5, marker="x", markersize=3.5, label=f"Estimated aligned ({len(est)})")
    ax.set_title(args.title)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(True, linewidth=0.4, alpha=0.35)
    ax.legend(loc="best", fontsize=8)
    ax.text(
        0.01,
        0.01,
        f"Sim(2): scale={scale:.6g}, matched={len(gt_idx)}, max_dt={args.max_dt}s, median_err={np.median(err):.3f}, mean_err={np.mean(err):.3f}",
        transform=ax.transAxes,
        fontsize=8,
        va="bottom",
        ha="left",
        bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "none", "pad": 3},
    )
    set_equal_aspect_with_margin(ax, [gt[:, 1:3], est_xy_aligned])
    fig.tight_layout()
    fig.savefig(args.out)
    plt.close(fig)

    if args.out_csv:
        args.out_csv.parent.mkdir(parents=True, exist_ok=True)
        with args.out_csv.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["gt_time", "est_time", "dt", "gt_x", "gt_y", "est_x_aligned", "est_y_aligned", "xy_error"])
            for gi, ei, dt, e in zip(gt_idx, est_idx, dts, err):
                writer.writerow([gt[gi, 0], est[ei, 0], dt, gt[gi, 1], gt[gi, 2], est_xy_aligned[ei, 0], est_xy_aligned[ei, 1], e])

    print(f"saved_plot={args.out}")
    if args.out_csv:
        print(f"saved_csv={args.out_csv}")
    print(f"gt_poses={len(gt)} est_poses={len(est)} matched={len(gt_idx)}")
    print(f"time_dt_sec min={dts.min():.6f} median={np.median(dts):.6f} max={dts.max():.6f}")
    print(f"xy_error min={err.min():.6f} median={np.median(err):.6f} mean={np.mean(err):.6f} max={err.max():.6f}")
    print(f"sim2_scale={scale:.12g} sim2_translation=({trans[0]:.12g},{trans[1]:.12g})")


if __name__ == "__main__":
    main()
