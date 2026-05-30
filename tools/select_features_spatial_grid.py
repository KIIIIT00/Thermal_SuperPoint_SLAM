#!/usr/bin/env python3
"""Select SuperPoint YAML features with per-cell top-k spatial quotas."""
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
        raise ValueError(f"Missing keypoints/descriptors: {path}")
    if keypoints.ndim != 2 or keypoints.shape[1] != 3:
        raise ValueError(f"keypoints must be Nx3: {path} has {keypoints.shape}")
    if descriptors.shape[0] != keypoints.shape[0]:
        raise ValueError(
            f"descriptor count mismatch: {path} has "
            f"{keypoints.shape[0]} keypoints and {descriptors.shape[0]} descriptors"
        )
    return keypoints.astype(np.float32, copy=False), descriptors.astype(np.float32, copy=False)


def write_yaml(path: Path, keypoints: np.ndarray, descriptors: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    fs.write("keypoints", keypoints.astype(np.float32, copy=False))
    fs.write("descriptors", descriptors.astype(np.float32, copy=False))
    fs.release()


def spatial_topk_indices(
    keypoints: np.ndarray,
    width: int,
    height: int,
    grid_x: int,
    grid_y: int,
    per_cell: int,
) -> np.ndarray:
    if keypoints.shape[0] == 0:
        return np.empty((0,), dtype=np.int64)

    xs = np.clip(keypoints[:, 0], 0, np.nextafter(float(width), -np.inf))
    ys = np.clip(keypoints[:, 1], 0, np.nextafter(float(height), -np.inf))
    cell_x = np.floor(xs / width * grid_x).astype(np.int32)
    cell_y = np.floor(ys / height * grid_y).astype(np.int32)

    selected: list[np.ndarray] = []
    scores = keypoints[:, 2]
    for gy in range(grid_y):
        for gx in range(grid_x):
            indices = np.flatnonzero((cell_x == gx) & (cell_y == gy))
            if indices.size == 0:
                continue
            order = np.argsort(-scores[indices], kind="stable")
            selected.append(indices[order[:per_cell]])

    if not selected:
        return np.empty((0,), dtype=np.int64)

    keep = np.concatenate(selected)
    # Keep output deterministic and close to image scan order expected by downstream code.
    order = np.lexsort((keypoints[keep, 0], keypoints[keep, 1]))
    return keep[order]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--in-dir", required=True, type=Path)
    parser.add_argument("--out-dir", required=True, type=Path)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--grid-x", type=int, default=8)
    parser.add_argument("--grid-y", type=int, default=6)
    parser.add_argument("--per-cell", type=int, required=True)
    args = parser.parse_args()

    files = sorted(args.in_dir.glob("*.yaml"), key=lambda p: int(p.stem))
    if not files:
        raise FileNotFoundError(f"No YAML files found in {args.in_dir}")

    counts: list[int] = []
    for src in files:
        keypoints, descriptors = read_yaml(src)
        keep = spatial_topk_indices(
            keypoints,
            width=args.width,
            height=args.height,
            grid_x=args.grid_x,
            grid_y=args.grid_y,
            per_cell=args.per_cell,
        )
        write_yaml(args.out_dir / src.name, keypoints[keep], descriptors[keep])
        counts.append(int(keep.size))

    arr = np.asarray(counts)
    print(f"[grid-select] files: {len(files)}")
    print(f"[grid-select] out: {args.out_dir}")
    print(
        "[grid-select] counts: "
        f"mean={arr.mean():.1f} min={arr.min()} max={arr.max()} "
        f"target_max={args.grid_x * args.grid_y * args.per_cell}"
    )


if __name__ == "__main__":
    main()
