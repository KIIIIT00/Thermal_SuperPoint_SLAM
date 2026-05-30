#!/usr/bin/env python3
"""Compare SuperPoint YAML feature sets for SLAM-oriented diagnostics."""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def read_yaml(path: Path) -> tuple[np.ndarray, np.ndarray]:
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    keypoints = fs.getNode("keypoints").mat()
    descriptors = fs.getNode("descriptors").mat()
    fs.release()
    if keypoints is None or descriptors is None:
        raise ValueError(f"missing keypoints/descriptors: {path}")
    return keypoints.astype(np.float32, copy=False), descriptors.astype(np.float32, copy=False)


def yaml_files(path: Path) -> list[Path]:
    files = sorted(path.glob("*.yaml"), key=lambda p: int(p.stem))
    if not files:
        raise FileNotFoundError(f"no yaml files in {path}")
    return files


def summarize_counts(
    name: str,
    path: Path,
    width: int,
    height: int,
    grid_x: int,
    grid_y: int,
) -> None:
    counts: list[int] = []
    occupied: list[int] = []
    min_cell_nonzero: list[int] = []
    max_cell: list[int] = []
    score_p50: list[float] = []
    score_p90: list[float] = []

    for f in yaml_files(path):
        kpts, _ = read_yaml(f)
        counts.append(int(kpts.shape[0]))
        if kpts.shape[0] == 0:
            occupied.append(0)
            min_cell_nonzero.append(0)
            max_cell.append(0)
            continue
        xs = np.clip(kpts[:, 0], 0, np.nextafter(float(width), -np.inf))
        ys = np.clip(kpts[:, 1], 0, np.nextafter(float(height), -np.inf))
        cx = np.floor(xs / width * grid_x).astype(np.int32)
        cy = np.floor(ys / height * grid_y).astype(np.int32)
        cell_counts = np.zeros((grid_y, grid_x), dtype=np.int32)
        np.add.at(cell_counts, (cy, cx), 1)
        nz = cell_counts[cell_counts > 0]
        occupied.append(int(nz.size))
        min_cell_nonzero.append(int(nz.min()) if nz.size else 0)
        max_cell.append(int(cell_counts.max()))
        score_p50.append(float(np.percentile(kpts[:, 2], 50)))
        score_p90.append(float(np.percentile(kpts[:, 2], 90)))

    arr = np.asarray(counts)
    print(f"\n[{name}] {path}")
    print(
        "counts "
        f"frames={arr.size} mean={arr.mean():.1f} min={arr.min()} "
        f"p10={np.percentile(arr, 10):.1f} p50={np.percentile(arr, 50):.1f} "
        f"p90={np.percentile(arr, 90):.1f} max={arr.max()}"
    )
    occ = np.asarray(occupied)
    print(
        "grid "
        f"occupied_mean={occ.mean():.1f}/{grid_x * grid_y} "
        f"occupied_min={occ.min()} min_nonzero_cell_mean={np.mean(min_cell_nonzero):.1f} "
        f"max_cell_mean={np.mean(max_cell):.1f}"
    )
    if score_p50:
        print(
            "scores "
            f"p50_mean={np.mean(score_p50):.6f} "
            f"p90_mean={np.mean(score_p90):.6f}"
        )


def cosine_stats(path: Path, frames: int, sample_per_frame: int, seed: int) -> None:
    rng = np.random.default_rng(seed)
    descs: list[np.ndarray] = []
    for f in yaml_files(path)[:frames]:
        _, desc = read_yaml(f)
        if desc.shape[0] > sample_per_frame:
            idx = rng.choice(desc.shape[0], sample_per_frame, replace=False)
            desc = desc[idx]
        descs.append(desc)
    x = np.concatenate(descs, axis=0)
    norms = np.linalg.norm(x, axis=1, keepdims=True)
    x = x / np.maximum(norms, 1e-12)
    if x.shape[0] > 4000:
        idx = rng.choice(x.shape[0], 4000, replace=False)
        x = x[idx]
    sims = x @ x.T
    tri = sims[np.triu_indices(sims.shape[0], k=1)]
    print(
        "cosine "
        f"n={x.shape[0]} mean={tri.mean():.4f} "
        f"p95={np.percentile(tri, 95):.4f} "
        f"p99={np.percentile(tri, 99):.4f}"
    )


def match_stats(path: Path, pair_count: int, step: int, ratio: float) -> None:
    files = yaml_files(path)
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    rows: list[tuple[int, int, int, int, float, float, float, float]] = []
    for i in range(min(pair_count, len(files) - step)):
        k1, d1 = read_yaml(files[i])
        k2, d2 = read_yaml(files[i + step])
        if d1.shape[0] < 2 or d2.shape[0] < 2:
            continue
        knn = matcher.knnMatch(d1, d2, k=2)
        good = [m for m, n in knn if m.distance < ratio * n.distance]
        if not good:
            rows.append((i + 1, i + 1 + step, 0, 0, np.nan, np.nan, np.nan, np.nan))
            continue
        pts1 = np.float32([k1[m.queryIdx, :2] for m in good])
        pts2 = np.float32([k2[m.trainIdx, :2] for m in good])
        flow = np.linalg.norm(pts2 - pts1, axis=1)
        inliers = 0
        if len(good) >= 8:
            try:
                _, mask = cv2.findFundamentalMat(pts1, pts2, cv2.FM_RANSAC, 1.0, 0.99)
                if mask is not None:
                    inliers = int(mask.ravel().astype(bool).sum())
            except cv2.error:
                inliers = 0
        rows.append(
            (
                i + 1,
                i + 1 + step,
                len(good),
                inliers,
                float(np.median(flow)),
                float(np.percentile(flow, 90)),
                float(np.mean(flow < 1.0)),
                float(np.mean(flow < 2.0)),
            )
        )

    arr = np.asarray([r[2:] for r in rows], dtype=np.float64)
    if arr.size == 0:
        print("matches no-pairs")
        return
    print(
        "matches "
        f"pairs={len(rows)} ratio={ratio} step={step} "
        f"good_mean={np.nanmean(arr[:, 0]):.1f} "
        f"inlier_mean={np.nanmean(arr[:, 1]):.1f} "
        f"inlier_ratio_mean={np.nanmean(arr[:, 1] / np.maximum(arr[:, 0], 1)):.3f} "
        f"flow_med_mean={np.nanmean(arr[:, 2]):.2f} "
        f"flow_p90_mean={np.nanmean(arr[:, 3]):.2f} "
        f"frac_flow_lt1={np.nanmean(arr[:, 4]):.3f} "
        f"frac_flow_lt2={np.nanmean(arr[:, 5]):.3f}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("feature_sets", nargs="+", help="NAME=DIR")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--grid-x", type=int, default=8)
    parser.add_argument("--grid-y", type=int, default=6)
    parser.add_argument("--cosine-frames", type=int, default=60)
    parser.add_argument("--sample-per-frame", type=int, default=80)
    parser.add_argument("--match-pairs", type=int, default=80)
    parser.add_argument("--match-step", type=int, default=1)
    parser.add_argument("--ratio", type=float, default=0.8)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    for item in args.feature_sets:
        name, raw_path = item.split("=", 1)
        path = Path(raw_path)
        summarize_counts(name, path, args.width, args.height, args.grid_x, args.grid_y)
        cosine_stats(path, args.cosine_frames, args.sample_per_frame, args.seed)
        match_stats(path, args.match_pairs, args.match_step, args.ratio)


if __name__ == "__main__":
    main()
