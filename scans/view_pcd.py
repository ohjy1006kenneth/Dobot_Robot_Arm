#!/usr/bin/env python3
"""
view_pcd.py — PCD viewer with height colouring and plate layer detection.
Just edit the CONFIG block below and run:  python3 view_pcd.py

Controls in viewer window:
  Mouse drag       : rotate
  Scroll           : zoom
  Shift + drag     : pan
  Q / Escape       : quit
"""

# ═══════════════════════════════════════════════════════════════
#  CONFIG  — edit these values, then just run:  python3 view_pcd.py
# ═══════════════════════════════════════════════════════════════

INPUT_FILE  = "cropped.pcd"   # PCD file to open (relative or absolute path)

RADIUS      = 1.0        # XY crop radius in metres (None = no crop)
FLOOR_Z     = None       # Override floor Z in metres absolute (None = auto-detect)
ABOVE       = 0.0        # Show points above this height relative to floor (metres)
                         #   e.g. 0.0 = everything above floor
                         #        0.05 = everything above 5 cm
BELOW       = 0.20       # Show points below this height relative to floor (metres)
                         #   e.g. 0.20 = up to 20 cm above floor
                         #        None = no upper limit
VOXEL       = 0.003      # Display voxel size in metres (0 = show every point, slow for large clouds)

# ═══════════════════════════════════════════════════════════════

import argparse
import sys
import numpy as np
import open3d as o3d


def find_floor(z, percentile=5):
    """Estimate floor Z as the Nth percentile of all Z values."""
    return np.percentile(z, percentile)


def detect_plate_layers(z_rel, bin_size=0.005, min_gap=0.02, min_count_frac=0.001):
    """
    Find peaks in the Z histogram that correspond to flat plate surfaces.
    z_rel: Z values relative to floor (metres)
    Returns list of (height_m, point_count) for each detected layer.
    """
    in_range = z_rel[(z_rel >= -0.01) & (z_rel <= 0.50)]
    if len(in_range) < 100:
        return []
    bins = np.arange(-0.01, 0.51, bin_size)
    counts, edges = np.histogram(in_range, bins=bins)
    centres = (edges[:-1] + edges[1:]) / 2

    min_count = len(in_range) * min_count_frac
    layers = []
    i = 0
    while i < len(counts):
        if counts[i] >= min_count:
            # Find the peak within this cluster
            j = i
            while j < len(counts) and counts[j] >= min_count:
                j += 1
            peak_idx = i + np.argmax(counts[i:j])
            layers.append((centres[peak_idx], int(counts[peak_idx])))
            i = j
        else:
            i += 1

    # Merge layers closer than min_gap
    merged = []
    for h, c in layers:
        if merged and (h - merged[-1][0]) < min_gap:
            # Keep the one with more points
            if c > merged[-1][1]:
                merged[-1] = (h, c)
        else:
            merged.append([h, c])
    return merged


def height_colormap(z_rel, z_lo, z_hi):
    """Map relative Z to jet-like RGB. z_lo=blue, z_hi=red."""
    z_range = z_hi - z_lo if z_hi != z_lo else 1.0
    t = np.clip((z_rel - z_lo) / z_range, 0, 1)
    colors = np.zeros((len(t), 3))
    colors[:, 0] = np.clip(1.5 - np.abs(t * 4 - 3), 0, 1)  # R
    colors[:, 1] = np.clip(1.5 - np.abs(t * 4 - 2), 0, 1)  # G
    colors[:, 2] = np.clip(1.5 - np.abs(t * 4 - 1), 0, 1)  # B
    return colors


def main():
    parser = argparse.ArgumentParser(description="PCD viewer for plate analysis")
    parser.add_argument("input", nargs="?", default=INPUT_FILE,
                        help=f"Input PCD file (default: {INPUT_FILE})")
    parser.add_argument("--radius", type=float, default=RADIUS,
                        help=f"Crop to this XY radius from origin in metres (default: {RADIUS})")
    parser.add_argument("--floor", type=float, default=FLOOR_Z,
                        help="Override floor Z in metres absolute (default: auto-detect)")
    parser.add_argument("--above", type=float, default=ABOVE,
                        help=f"Only show points above this height relative to floor in metres (default: {ABOVE})")
    parser.add_argument("--below", type=float, default=BELOW,
                        help=f"Only show points below this height relative to floor in metres (default: {BELOW})")
    parser.add_argument("--voxel", type=float, default=VOXEL,
                        help=f"Display voxel size in metres, 0 = off (default: {VOXEL})")
    args = parser.parse_args()

    print(f"Loading {args.input} ...")
    pcd = o3d.io.read_point_cloud(args.input)
    pts = np.asarray(pcd.points)
    print(f"  {len(pts):,} points loaded")

    if len(pts) == 0:
        print("ERROR: empty point cloud — check INPUT_FILE path")
        sys.exit(1)

    # --- Floor detection (do this BEFORE any cropping so percentile is stable) ---
    floor_z = args.floor if args.floor is not None else find_floor(pts[:, 2])
    print(f"\n  Floor Z: {floor_z:.4f} m {'(manual)' if args.floor else '(auto-detected)'}")

    # --- XY-only radius crop (not 3D — avoids cutting points above the sensor) ---
    if args.radius is not None:
        xy_dist = np.linalg.norm(pts[:, :2], axis=1)
        mask = xy_dist <= args.radius
        pcd = pcd.select_by_index(np.where(mask)[0])
        pts = np.asarray(pcd.points)
        print(f"  {len(pts):,} points after XY radius crop ({args.radius} m)")

    if len(pts) == 0:
        print("ERROR: no points left after radius crop — try increasing RADIUS")
        sys.exit(1)

    z = pts[:, 2]
    z_rel = z - floor_z  # height above floor

    # --- Plate layer detection ---
    layers = detect_plate_layers(z_rel)
    print(f"\n  Detected surface layers (possible plate tops):")
    if layers:
        prev_h = 0.0
        for i, (h, c) in enumerate(layers):
            diff = (h - prev_h) * 1000 if i > 0 else 0
            marker = " ← FLOOR" if i == 0 else f" ← +{diff:.0f} mm above previous"
            print(f"    Layer {i}: {h*100:+6.2f} cm above floor  ({c:,} pts){marker}")
            prev_h = h
        if len(layers) >= 2:
            print(f"\n  Thickness differences between consecutive layers:")
            for i in range(1, len(layers)):
                diff_mm = (layers[i][0] - layers[i-1][0]) * 1000
                print(f"    Layer {i-1} → Layer {i}: {diff_mm:.1f} mm")
    else:
        print("    None detected (try scanning from directly above the plates)")

    # --- Height filter for display ---
    lo = args.above if args.above is not None else -0.02
    hi = args.below if args.below is not None else float(z_rel.max())
    mask2 = (z_rel >= lo) & (z_rel <= hi)
    pcd = pcd.select_by_index(np.where(mask2)[0])
    pts = np.asarray(pcd.points)
    z_rel = pts[:, 2] - floor_z   # recompute from pts so it stays aligned
    print(f"\n  Showing {len(pts):,} points between {lo*100:.1f} cm and {hi*100:.1f} cm above floor")
    print(f"  Z_rel range in slice: {z_rel.min()*1000:.1f} mm  to  {z_rel.max()*1000:.1f} mm")

    if len(pts) == 0:
        print("ERROR: no points in the selected height range — adjust ABOVE/BELOW")
        sys.exit(1)

    # --- Voxel downsample for display ---
    if args.voxel > 0 and len(pts) > 300_000:
        pcd = pcd.voxel_down_sample(args.voxel)
        pts = np.asarray(pcd.points)
        z_rel = pts[:, 2] - floor_z   # recompute after downsample
        print(f"  Downsampled to {len(pts):,} points for display (voxel={args.voxel*1000:.0f} mm)")

    # --- Color by height above floor ---
    # Stretch across the actual data range so colours are always visible (never black)
    color_lo = float(z_rel.min())
    color_hi = float(z_rel.max())
    print(f"  Colour range: {color_lo*1000:.1f} mm  →  {color_hi*1000:.1f} mm")
    if color_hi - color_lo < 1e-4:          # degenerate flat slab — add tiny range
        color_lo -= 0.005
        color_hi += 0.005
    colors = height_colormap(z_rel, color_lo, color_hi)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    print(f"\n  Color scale: BLUE = {color_lo*100:.1f} cm  →  RED = {color_hi*100:.1f} cm above floor")
    print("  Opening viewer... (Q to quit)\n")

    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name=f"{args.input}  |  floor={floor_z:.3f}m  |  {color_lo*100:.0f}cm(blue)→{color_hi*100:.0f}cm(red)",
        width=1280, height=800,
    )
    vis.add_geometry(pcd)

    opt = vis.get_render_option()
    opt.background_color = np.array([0.1, 0.1, 0.1])  # dark grey background
    opt.point_size = 1.5                                # slightly larger points
    opt.light_on = False                                # disable lighting — show raw vertex colours

    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()
