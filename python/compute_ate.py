#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Compute the Absolute Trajectory Error (ATE) between two pose trajectories.

The trajectories are read from a keyframes file with the same 33-column layout
as keyframes.txt (written by Keyframe::Save / read by Keyframe::Load):

    id stamp rtk_heading_valid rtk_valid rtk_inlier
    lidar_pose  : tx ty tz qx qy qz qw
    rtk_pose    : tx ty tz qx qy qz qw
    opti_pose_1 : tx ty tz qx qy qz qw
    opti_pose_2 : tx ty tz qx qy qz qw

ATE is the per-frame norm of the translation difference between the reference
and the estimate trajectories, optionally after a rigid Umeyama alignment.

Usage:
    python3 compute_ate.py keyframes.txt
    python3 compute_ate.py keyframes.txt --ref rtk_pose --est lidar_pose
    python3 compute_ate.py keyframes.txt --align --errors ate_errors.csv
"""

import argparse
import math
import sys
from pathlib import Path

try:
    import numpy as np
except ImportError:
    sys.exit("error: numpy is required (pip install numpy)")

POSE_NAMES = ["lidar_pose", "rtk_pose", "opti_pose_1", "opti_pose_2"]
NUM_SCALAR_FIELDS = 5
NUM_POSE_FIELDS = 7
EXPECTED_COLS = NUM_SCALAR_FIELDS + len(POSE_NAMES) * NUM_POSE_FIELDS  # 33
POSE_OFFSET = {
    name: NUM_SCALAR_FIELDS + i * NUM_POSE_FIELDS
    for i, name in enumerate(POSE_NAMES)
}


def load_keyframes(path: Path):
    """Return every line as a list of floats, in file order."""
    rows = []
    with path.open("r") as fin:
        for line_no, line in enumerate(fin, 1):
            line = line.strip()
            if not line:
                continue
            fields = line.split()
            if len(fields) != EXPECTED_COLS:
                raise ValueError(
                    f"{path}: line {line_no}: expected {EXPECTED_COLS} fields, "
                    f"got {len(fields)}")
            rows.append([float(v) for v in fields])
    return rows


def extract_pose(row, name):
    """Return (t, q) for the given pose, q ordered as (qx, qy, qz, qw)."""
    off = POSE_OFFSET[name]
    t = np.array(row[off:off + 3], dtype=float)
    q = np.array(row[off + 3:off + 7], dtype=float)
    norm = np.linalg.norm(q)
    if norm > 0:
        q /= norm
    return t, q


def quat_to_R(q):
    """Rotation matrix from quaternion (qx, qy, qz, qw)."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def rotation_error_deg(q_ref, q_est):
    """Relative rotation angle between two quaternions, in degrees."""
    R_err = quat_to_R(q_ref).T @ quat_to_R(q_est)
    cos_angle = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
    return math.degrees(math.acos(cos_angle))


def umeyama_rigid(src, dst):
    """Rigid (6-DoF) Umeyama alignment: minimize ||R*src + t - dst||."""
    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)
    src_c = src - mu_src
    dst_c = dst - mu_dst
    cov = dst_c.T @ src_c
    U, _, Vt = np.linalg.svd(cov)
    d = np.sign(np.linalg.det(U @ Vt))
    R = U @ np.diag([1.0, 1.0, d]) @ Vt
    t = mu_dst - R @ mu_src
    return R, t


def summarize(errors, label):
    rmse = float(np.sqrt(np.mean(errors ** 2)))
    mean = float(np.mean(errors))
    median = float(np.median(errors))
    std = float(np.std(errors))
    maximum = float(np.max(errors))
    print(f"  {label:<14}: {rmse:12.6f}")
    print(f"  {'Mean':<14}: {mean:12.6f}")
    print(f"  {'Median':<14}: {median:12.6f}")
    print(f"  {'Std':<14}: {std:12.6f}")
    print(f"  {'Max':<14}: {maximum:12.6f}")
    return rmse, mean, median, std, maximum


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compute ATE between two pose trajectories in a "
                    "keyframes.txt file."
    )
    parser.add_argument("input", nargs="?", default="keyframes.txt",
                        help="input keyframe file (default: keyframes.txt)")
    parser.add_argument("--ref", choices=POSE_NAMES, default="rtk_pose",
                        help="reference (ground truth) pose group "
                             "(default: rtk_pose)")
    parser.add_argument("--est", choices=POSE_NAMES, default="lidar_pose",
                        help="estimated pose group (default: lidar_pose)")
    parser.add_argument("--align", action="store_true",
                        help="rigidly align the estimate to the reference "
                             "with Umeyama before computing the error")
    parser.add_argument("--errors", default=None, metavar="CSV",
                        help="also write per-frame errors to the given CSV")
    args = parser.parse_args()

    in_path = Path(args.input)
    if not in_path.is_file():
        print(f"error: input file not found: {in_path}", file=sys.stderr)
        return 1
    if args.ref == args.est:
        print("error: --ref and --est must be different pose groups",
              file=sys.stderr)
        return 1

    rows = load_keyframes(in_path)
    stamps = np.array([r[1] for r in rows])          # timestamp, field index 1
    t_ref = np.array([extract_pose(r, args.ref)[0] for r in rows])
    t_est = np.array([extract_pose(r, args.est)[0] for r in rows])
    q_ref = np.array([extract_pose(r, args.ref)[1] for r in rows])
    q_est = np.array([extract_pose(r, args.est)[1] for r in rows])

    t_est_used = t_est
    alignment = "none (raw frames)"
    if args.align:
        R, t = umeyama_rigid(t_est, t_ref)
        t_est_used = (R @ t_est.T).T + t
        alignment = "rigid Umeyama (6-DoF)"

    err_xyz = t_ref - t_est_used
    err_norm = np.linalg.norm(err_xyz, axis=1)
    rot_deg = np.array([rotation_error_deg(qr, qe)
                        for qr, qe in zip(q_ref, q_est)])

    print(f"reference : {args.ref}")
    print(f"estimate  : {args.est}")
    print(f"frames    : {len(rows)}")
    print(f"alignment : {alignment}")
    print(f"time range: {stamps.min():.6f} .. {stamps.max():.6f}")
    print()
    print("Absolute trajectory error (translation, meters):")
    summarize(err_norm, "RMSE")
    rmse_xyz = np.sqrt(np.mean(err_xyz ** 2, axis=0))
    print(f"  Per-axis RMSE  : x={rmse_xyz[0]:.6f} "
          f"y={rmse_xyz[1]:.6f} z={rmse_xyz[2]:.6f}")
    rot_rmse = float(np.sqrt(np.mean(rot_deg ** 2)))
    print(f"  Rotation RMSE  : {rot_rmse:.6f} deg")

    if args.errors:
        with open(args.errors, "w", newline="\n") as fout:
            fout.write("stamp,err_x,err_y,err_z,err_norm,rot_deg\n")
            for stamp, err, n, rot in zip(stamps, err_xyz, err_norm, rot_deg):
                fout.write(f"{stamp:.9f},{err[0]:.9f},{err[1]:.9f},"
                           f"{err[2]:.9f},{n:.9f},{rot:.6f}\n")
        print(f"\nper-frame errors written to: {args.errors}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
