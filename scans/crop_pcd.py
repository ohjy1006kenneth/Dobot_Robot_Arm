#!/usr/bin/env python3
"""
crop_pcd.py — Crop a PCD file by radius, bounding box, and/or height above ground.
Just edit the CONFIG block below and run:  python3 crop_pcd.py
"""

# ═══════════════════════════════════════════════════════════════
#  CONFIG  — edit these values, then just run:  python3 crop_pcd.py
# ═══════════════════════════════════════════════════════════════

INPUT_FILE  = "all_raw_points.pcd"   # Input PCD file
OUTPUT_FILE = "cropped.pcd"          # Output PCD file (None = auto: input_cropped.pcd)

RADIUS      = 3.0       # Keep points within this XY radius in metres (None = no radius crop)
ABOVE       = 0.05       # Keep points above this height relative to floor in metres
                         #   0.0  = everything above the floor surface
                         #   0.05 = everything above 5 cm (removes the floor itself)
                         #   None = no lower height limit
BELOW       = 0.1       # Keep points below this height relative to floor in metres
                         #   0.20 = up to 20 cm above floor
                         #   None = no upper height limit
FLOOR_Z     = None       # Override floor Z in metres absolute (None = auto-detect from 2nd percentile)

BOX = None     # XYZ bounding box as [xmin, xmax, ymin, ymax, zmin, zmax]
                         #   e.g. [-1, 1, -1, 1, -0.5, 1.5 ]
                         #   None = no box crop

# ═══════════════════════════════════════════════════════════════

import argparse
import sys
import numpy as np

def main():
    parser = argparse.ArgumentParser(description="Crop a PCD point cloud")
    parser.add_argument("input", nargs="?", default=INPUT_FILE,
                        help=f"Input PCD file (default: {INPUT_FILE})")
    parser.add_argument("--out", default=OUTPUT_FILE,
                        help=f"Output PCD file (default: {OUTPUT_FILE})")
    parser.add_argument("--radius", type=float, default=RADIUS,
                        help=f"Keep only points within this XY radius in metres (default: {RADIUS})")
    parser.add_argument("--box", type=float, nargs=6, metavar=("xmin","xmax","ymin","ymax","zmin","zmax"),
                        default=BOX,
                        help="Keep only points inside this XYZ bounding box")
    parser.add_argument("--above", type=float, default=ABOVE,
                        help=f"Keep points above this height relative to ground in metres (default: {ABOVE})")
    parser.add_argument("--below", type=float, default=BELOW,
                        help=f"Keep points below this height relative to ground in metres (default: {BELOW})")
    parser.add_argument("--floor", type=float, default=FLOOR_Z,
                        help="Override floor Z in metres absolute (default: auto-detect from 2nd percentile)")
    args = parser.parse_args()

    try:
        import open3d as o3d
    except ImportError:
        print("open3d not found. Installing...")
        import subprocess
        subprocess.check_call([sys.executable, "-m", "pip", "install", "open3d"])
        import open3d as o3d

    print(f"Loading {args.input} ...")
    pcd = o3d.io.read_point_cloud(args.input)
    pts = np.asarray(pcd.points)
    print(f"  Loaded {len(pts):,} points")

    if len(pts) == 0:
        print("ERROR: empty point cloud")
        sys.exit(1)

    mask = np.ones(len(pts), dtype=bool)

    # --- XY radius filter ---
    if args.radius is not None:
        xy_dists = np.linalg.norm(pts[:, :2], axis=1)   # XY only — keeps all heights
        mask &= xy_dists <= args.radius
        print(f"  After radius filter ({args.radius} m XY): {mask.sum():,} points")

    # --- Bounding box filter ---
    if args.box is not None:
        xmin, xmax, ymin, ymax, zmin, zmax = args.box
        mask &= (pts[:,0] >= xmin) & (pts[:,0] <= xmax)
        mask &= (pts[:,1] >= ymin) & (pts[:,1] <= ymax)
        mask &= (pts[:,2] >= zmin) & (pts[:,2] <= zmax)
        print(f"  After box filter: {mask.sum():,} points")

    # --- Ground-relative height filter ---
    if args.above is not None or args.below is not None:
        # Detect floor from all points (before other filters corrupt the distribution)
        if args.floor is not None:
            floor_z = args.floor
            print(f"  Floor Z: {floor_z:.4f} m (manual)")
        else:
            floor_z = np.percentile(pts[:, 2], 2)
            print(f"  Floor Z: {floor_z:.4f} m (auto-detected, 2nd percentile)")

        z_rel = pts[:, 2] - floor_z
        if args.above is not None:
            mask &= z_rel >= args.above
        if args.below is not None:
            mask &= z_rel <= args.below

        lo = args.above if args.above is not None else -999
        hi = args.below if args.below is not None else 999
        print(f"  After height filter ({lo*100:.1f} – {hi*100:.1f} cm above floor): {mask.sum():,} points")

    if args.radius is None and args.box is None and args.above is None and args.below is None:
        print("ERROR: specify at least one filter: --radius, --box, --above, or --below")
        sys.exit(1)

    cropped = pcd.select_by_index(np.where(mask)[0])

    out_file = args.out or args.input.replace(".pcd", "_cropped.pcd")
    o3d.io.write_point_cloud(out_file, cropped)
    print(f"  Saved {mask.sum():,} points → {out_file}")

if __name__ == "__main__":
    main()
