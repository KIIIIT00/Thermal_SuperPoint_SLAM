#!/usr/bin/env python3

import argparse
import os
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, FFMpegWriter


def load_tum(path: str) -> Tuple[np.ndarray, np.ndarray]:
    times: List[float] = []
    xyz: List[List[float]] = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            t = float(parts[0])
            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            times.append(t)
            xyz.append([x, y, z])
    return np.asarray(times), np.asarray(xyz)


def match_by_index(
    gt_t: np.ndarray, gt_xyz: np.ndarray, est_t: np.ndarray, est_xyz: np.ndarray
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    n = min(len(gt_t), len(est_t))
    return est_t[:n], gt_xyz[:n], est_xyz[:n]


def match_by_nearest(
    gt_t: np.ndarray,
    gt_xyz: np.ndarray,
    est_t: np.ndarray,
    est_xyz: np.ndarray,
    max_dt: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    gt_idx = 0
    t_out = []
    gt_out = []
    est_out = []
    for i in range(len(est_t)):
        t = est_t[i]
        while gt_idx + 1 < len(gt_t) and abs(gt_t[gt_idx + 1] - t) <= abs(gt_t[gt_idx] - t):
            gt_idx += 1
        if abs(gt_t[gt_idx] - t) <= max_dt:
            t_out.append(t)
            gt_out.append(gt_xyz[gt_idx])
            est_out.append(est_xyz[i])
    return np.asarray(t_out), np.asarray(gt_out), np.asarray(est_out)


def umeyama_alignment(model: np.ndarray, data: np.ndarray, with_scale: bool = True):
    # model, data: (N, 3)
    assert model.shape == data.shape
    n = model.shape[0]
    mean_m = model.mean(axis=0)
    mean_d = data.mean(axis=0)
    xm = model - mean_m
    xd = data - mean_d
    cov = xd.T @ xm / n
    u, s, vt = np.linalg.svd(cov)
    r = u @ vt
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1
        r = u @ vt
    if with_scale:
        var = (xm ** 2).sum() / n
        scale = (s @ np.ones_like(s)) / var
    else:
        scale = 1.0
    t = mean_d - scale * (r @ mean_m)
    return r, t, scale


def apply_alignment(xyz: np.ndarray, r: np.ndarray, t: np.ndarray, scale: float) -> np.ndarray:
    return (scale * (r @ xyz.T)).T + t


def _set_equal_3d_axes(ax, xyz: np.ndarray) -> None:
    mins = xyz.min(axis=0)
    maxs = xyz.max(axis=0)
    centers = (mins + maxs) * 0.5
    radius = 0.5 * np.max(maxs - mins)
    if radius <= 0:
        radius = 1.0
    ax.set_xlim(centers[0] - radius, centers[0] + radius)
    ax.set_ylim(centers[1] - radius, centers[1] + radius)
    ax.set_zlim(centers[2] - radius, centers[2] + radius)


def make_2d_video(
    times: np.ndarray,
    est_xyz: np.ndarray,
    out_path: str,
    plane: str,
    fps: int,
    gt_xyz: np.ndarray = None,
) -> None:
    plane_map = {"xy": (0, 1), "xz": (0, 2), "yz": (1, 2)}
    if plane not in plane_map:
        raise ValueError("plane must be one of: xy, xz, yz")
    i0, i1 = plane_map[plane]

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect("equal", "box")
    ax.set_title(f"Trajectory 2D ({plane.upper()})")
    ax.set_xlabel(plane[0].upper())
    ax.set_ylabel(plane[1].upper())

    if gt_xyz is not None and len(gt_xyz) > 0:
        ax.plot(gt_xyz[:, i0], gt_xyz[:, i1], color="gray", linewidth=1.0, label="GT")
    est_line, = ax.plot([], [], color="red", linewidth=1.7, label="SLAM")
    ax.legend(loc="best")

    all_pts = est_xyz if gt_xyz is None else np.vstack([est_xyz, gt_xyz])
    x_min, y_min = all_pts[:, i0].min(), all_pts[:, i1].min()
    x_max, y_max = all_pts[:, i0].max(), all_pts[:, i1].max()
    pad_x = max(1e-6, (x_max - x_min) * 0.08)
    pad_y = max(1e-6, (y_max - y_min) * 0.08)
    ax.set_xlim(x_min - pad_x, x_max + pad_x)
    ax.set_ylim(y_min - pad_y, y_max + pad_y)

    time_text = ax.text(
        0.02, 0.98, "", transform=ax.transAxes, va="top", ha="left", fontsize=10
    )

    def update(i):
        est_line.set_data(est_xyz[: i + 1, i0], est_xyz[: i + 1, i1])
        time_text.set_text(f"t={times[i]:.3f}")
        return est_line, time_text

    anim = FuncAnimation(fig, update, frames=len(times), interval=1000 / fps, blit=True)
    writer = FFMpegWriter(fps=fps)
    anim.save(out_path, writer=writer)
    plt.close(fig)


def make_3d_video(
    times: np.ndarray,
    est_xyz: np.ndarray,
    out_path: str,
    fps: int,
    gt_xyz: np.ndarray = None,
) -> None:
    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title("Trajectory 3D")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    if gt_xyz is not None and len(gt_xyz) > 0:
        ax.plot(gt_xyz[:, 0], gt_xyz[:, 1], gt_xyz[:, 2], color="gray", linewidth=1.0, label="GT")
    est_line, = ax.plot([], [], [], color="red", linewidth=1.7, label="SLAM")
    ax.legend(loc="best")

    all_pts = est_xyz if gt_xyz is None else np.vstack([est_xyz, gt_xyz])
    _set_equal_3d_axes(ax, all_pts)
    ax.view_init(elev=28, azim=-62)

    def update(i):
        est_line.set_data(est_xyz[: i + 1, 0], est_xyz[: i + 1, 1])
        est_line.set_3d_properties(est_xyz[: i + 1, 2])
        return (est_line,)

    anim = FuncAnimation(fig, update, frames=len(times), interval=1000 / fps, blit=True)
    writer = FFMpegWriter(fps=fps)
    anim.save(out_path, writer=writer)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Create 2D/3D SLAM trajectory videos from TUM files.")
    parser.add_argument("--est_tum", required=True, help="Estimated trajectory in TUM format")
    parser.add_argument("--out_dir", required=True, help="Output directory for videos")
    parser.add_argument("--gt_tum", default="", help="Optional GT trajectory in TUM format")
    parser.add_argument("--plane", default="xz", choices=["xy", "xz", "yz"], help="2D projection plane")
    parser.add_argument("--fps", type=int, default=20, help="Video fps")
    parser.add_argument("--stride", type=int, default=1, help="Take every Nth point")
    parser.add_argument("--sync", default="nearest", choices=["nearest", "index"], help="GT/EST sync mode")
    parser.add_argument("--max_dt", type=float, default=0.05, help="Max dt for nearest sync (sec)")
    parser.add_argument("--no_align", action="store_true", help="Disable Sim(3) alignment to GT")
    parser.add_argument("--prefix", default="slam_traj", help="Output filename prefix")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    est_t, est_xyz = load_tum(args.est_tum)
    if len(est_t) == 0:
        raise RuntimeError(f"No valid TUM entries in est file: {args.est_tum}")

    use_gt = bool(args.gt_tum)
    gt_xyz = None
    t = est_t
    xyz = est_xyz

    if use_gt:
        gt_t, gt = load_tum(args.gt_tum)
        if len(gt_t) == 0:
            raise RuntimeError(f"No valid TUM entries in GT file: {args.gt_tum}")
        if args.sync == "index":
            t, gt_xyz, xyz = match_by_index(gt_t, gt, est_t, est_xyz)
        else:
            t, gt_xyz, xyz = match_by_nearest(gt_t, gt, est_t, est_xyz, args.max_dt)
        if len(t) == 0:
            raise RuntimeError("No matched points between GT and EST. Check timestamps/max_dt.")

        if not args.no_align:
            r, trans, scale = umeyama_alignment(xyz, gt_xyz, with_scale=True)
            xyz = apply_alignment(xyz, r, trans, scale)

    if args.stride > 1:
        t = t[:: args.stride]
        xyz = xyz[:: args.stride]
        if gt_xyz is not None:
            gt_xyz = gt_xyz[:: args.stride]

    out_2d = os.path.join(args.out_dir, f"{args.prefix}_2d.mp4")
    out_3d = os.path.join(args.out_dir, f"{args.prefix}_3d.mp4")

    make_2d_video(t, xyz, out_2d, args.plane, args.fps, gt_xyz=gt_xyz)
    make_3d_video(t, xyz, out_3d, args.fps, gt_xyz=gt_xyz)

    print(f"Wrote {out_2d}")
    print(f"Wrote {out_3d}")


if __name__ == "__main__":
    main()
