#!/usr/bin/env python3
import argparse
import os

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, FFMpegWriter


def load_tum_xyz(path: str):
    t = []
    xyz = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            p = s.split()
            if len(p) < 8:
                continue
            t.append(float(p[0]))
            xyz.append([float(p[1]), float(p[2]), float(p[3])])
    return np.asarray(t), np.asarray(xyz)


def load_xyz(path: str):
    pts = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            p = s.split()
            if len(p) < 3:
                continue
            pts.append([float(p[0]), float(p[1]), float(p[2])])
    return np.asarray(pts)


def set_equal_3d(ax, pts):
    mins = pts.min(axis=0)
    maxs = pts.max(axis=0)
    c = (mins + maxs) * 0.5
    r = 0.5 * np.max(maxs - mins)
    if r <= 0:
        r = 1.0
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)


def make_3d(times, traj, points, out_path, fps):
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title("SLAM Map + Camera Trajectory (3D)")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    if len(points) > 0:
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=1, alpha=0.25, c="gray")

    (line,) = ax.plot([], [], [], color="red", linewidth=1.8)
    (cam,) = ax.plot([], [], [], marker="o", color="blue", markersize=6)

    all_pts = traj if len(points) == 0 else np.vstack([traj, points])
    set_equal_3d(ax, all_pts)
    ax.view_init(elev=26, azim=-62)

    def update(i):
        line.set_data(traj[: i + 1, 0], traj[: i + 1, 1])
        line.set_3d_properties(traj[: i + 1, 2])
        cam.set_data([traj[i, 0]], [traj[i, 1]])
        cam.set_3d_properties([traj[i, 2]])
        return line, cam

    anim = FuncAnimation(fig, update, frames=len(traj), interval=1000 / fps, blit=True)
    anim.save(out_path, writer=FFMpegWriter(fps=fps))
    plt.close(fig)


def make_2d(times, traj, points, out_path, fps, plane):
    idx = {"xy": (0, 1), "xz": (0, 2), "yz": (1, 2)}[plane]
    i0, i1 = idx

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_title(f"SLAM Map + Camera Trajectory (2D:{plane.upper()})")
    ax.set_xlabel(plane[0].upper())
    ax.set_ylabel(plane[1].upper())
    ax.set_aspect("equal", "box")

    if len(points) > 0:
        ax.scatter(points[:, i0], points[:, i1], s=2, alpha=0.25, c="gray")

    (line,) = ax.plot([], [], color="red", linewidth=1.8)
    (cam,) = ax.plot([], [], marker="o", color="blue", markersize=6)

    all_pts = traj if len(points) == 0 else np.vstack([traj, points])
    x_min, y_min = all_pts[:, i0].min(), all_pts[:, i1].min()
    x_max, y_max = all_pts[:, i0].max(), all_pts[:, i1].max()
    px = max(1e-6, (x_max - x_min) * 0.08)
    py = max(1e-6, (y_max - y_min) * 0.08)
    ax.set_xlim(x_min - px, x_max + px)
    ax.set_ylim(y_min - py, y_max + py)

    txt = ax.text(0.02, 0.98, "", transform=ax.transAxes, ha="left", va="top")

    def update(i):
        line.set_data(traj[: i + 1, i0], traj[: i + 1, i1])
        cam.set_data([traj[i, i0]], [traj[i, i1]])
        txt.set_text(f"t={times[i]:.3f}")
        return line, cam, txt

    anim = FuncAnimation(fig, update, frames=len(traj), interval=1000 / fps, blit=True)
    anim.save(out_path, writer=FFMpegWriter(fps=fps))
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="Create SLAM point-cloud + camera videos")
    ap.add_argument("--traj_tum", required=True)
    ap.add_argument("--map_xyz", required=True)
    ap.add_argument("--out_dir", required=True)
    ap.add_argument("--prefix", default="slam_map")
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--stride", type=int, default=1)
    ap.add_argument("--plane", choices=["xy", "xz", "yz"], default="xz")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    t, traj = load_tum_xyz(args.traj_tum)
    pts = load_xyz(args.map_xyz)

    if len(traj) == 0:
        raise RuntimeError("Trajectory is empty")

    if args.stride > 1:
        t = t[:: args.stride]
        traj = traj[:: args.stride]

    out3d = os.path.join(args.out_dir, f"{args.prefix}_map3d.mp4")
    out2d = os.path.join(args.out_dir, f"{args.prefix}_map2d.mp4")

    make_3d(t, traj, pts, out3d, args.fps)
    make_2d(t, traj, pts, out2d, args.fps, args.plane)

    print(f"Wrote {out3d}")
    print(f"Wrote {out2d}")


if __name__ == "__main__":
    main()
