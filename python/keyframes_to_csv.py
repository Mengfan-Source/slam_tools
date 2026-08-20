#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Convert keyframes.txt (Keyframe::Save/::Load format) into a PlotJuggler CSV.

Field order follows Keyframe::Load in key_frame.cpp:

    id timestamp rtk_heading_valid rtk_valid rtk_inlier
    lidar_pose_   : tx ty tz qx qy qz qw
    rtk_pose_     : tx ty tz qx qy qz qw
    opti_pose_1_  : tx ty tz qx qy qz qw
    opti_pose_2_  : tx ty tz qx qy qz qw

Total: 5 + 4*7 = 33 columns. The output CSV uses the same style as the
reference file that loads fine in PlotJuggler: a plain comma-separated header
row and the time column (named ``stamp``) placed first.

Usage:
    python3 keyframes_to_csv.py [keyframes.txt] [-o keyframes.csv]
"""

import argparse
import sys
from pathlib import Path

POSE_NAMES = ["lidar_pose", "rtk_pose", "opti_pose_1", "opti_pose_2"]
AXIS_NAMES = ["tx", "ty", "tz", "qx", "qy", "qz", "qw"]

HEADER = ["stamp", "id", "rtk_heading_valid", "rtk_valid", "rtk_inlier"]
for pose in POSE_NAMES:
    HEADER.extend(f"{pose}_{axis}" for axis in AXIS_NAMES)

EXPECTED_COLS = len(HEADER)  # 33


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Convert keyframes.txt to a PlotJuggler-friendly CSV."
    )
    parser.add_argument("input", nargs="?", default="keyframes.txt",
                        help="input keyframe file (default: keyframes.txt)")
    parser.add_argument("-o", "--output", default=None,
                        help="output CSV path (default: <input>.csv)")
    parser.add_argument("--precision", type=int, default=18,
                        help="decimal places for floating point values "
                             "(default: 18, matching the C++ output precision)")
    args = parser.parse_args()

    in_path = Path(args.input)
    if not in_path.is_file():
        print(f"error: input file not found: {in_path}", file=sys.stderr)
        return 1
    out_path = Path(args.output) if args.output else in_path.with_suffix(".csv")

    fmt = f"{{:.{args.precision}f}}"
    written = skipped = 0

    with in_path.open("r") as fin, out_path.open("w", newline="\n") as fout:
        fout.write(",".join(HEADER) + "\n")

        for line_no, line in enumerate(fin, 1):
            line = line.strip()
            if not line:
                continue

            fields = line.split()
            if len(fields) != EXPECTED_COLS:
                print(f"warning: line {line_no}: expected {EXPECTED_COLS} "
                      f"fields, got {len(fields)}; skipped", file=sys.stderr)
                skipped += 1
                continue

            # fields[0]=id, fields[1]=timestamp, fields[2:5]=rtk flags,
            # fields[5:]=the four SE3 poses (tx ty tz qx qy qz qw each).
            row = [fmt.format(float(fields[1])), fields[0]]
            row.extend(fields[2:5])
            row.extend(fmt.format(float(v)) for v in fields[5:])

            fout.write(",".join(row) + "\n")
            written += 1

    print(f"done: wrote {written} rows to {out_path}"
          + (f" ({skipped} lines skipped)" if skipped else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
