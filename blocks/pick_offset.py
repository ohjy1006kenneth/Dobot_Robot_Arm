"""
Interactive Block Offset Picker
================================
Load two point cloud blocks, click matching points on each,
and get the exact X/Y/Z offset to use with merge_blocks.py.

Usage:
    python pick_offset.py block1_real/merged_block1.pcd block2_real/merged_block2.pcd

Controls:
    - Left-click a point on the LEFT view (block1)
    - Left-click a matching point on the RIGHT view (block2)
    - Repeat for more point pairs (more = more accurate average)
    - Press ENTER to compute and print the offset
    - Press R to reset all picks
    - Press Q to quit

The script shows a TOP-DOWN view (X-Y plane) and a SIDE view (Y-Z plane)
side by side for each block so you can pick the same feature from two angles.

Dependencies:
    pip install numpy scipy matplotlib
"""

import sys
import struct
import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.spatial import KDTree

# ── PCD loading (binary + ascii) ─────────────────────────────────────────────

_PCD_TYPE_MAP = {
    ('F', 4): np.float32, ('F', 8): np.float64,
    ('I', 1): np.int8,    ('I', 2): np.int16,   ('I', 4): np.int32,
    ('U', 1): np.uint8,   ('U', 2): np.uint16,  ('U', 4): np.uint32,
}

def _parse_pcd_header(f):
    header = {}
    while True:
        line = f.readline().decode('utf-8', errors='replace').strip()
        if not line or line.startswith('#'):
            continue
        key, *vals = line.split()
        key = key.upper()
        if   key == 'FIELDS': header['fields'] = vals
        elif key == 'SIZE':   header['size']   = [int(v) for v in vals]
        elif key == 'TYPE':   header['type']   = [v.upper() for v in vals]
        elif key == 'COUNT':  header['count']  = [int(v) for v in vals]
        elif key == 'POINTS': header['points'] = int(vals[0])
        elif key == 'DATA':
            header['data'] = vals[0].lower()
            break
    return header

def load_pcd_xyz(path, voxel=0.02):
    """Load PCD and downsample aggressively for interactive display."""
    print(f"  Loading {path} ...")
    with open(path, 'rb') as f:
        hdr = _parse_pcd_header(f)
        raw = f.read()

    fields, sizes, types = hdr['fields'], hdr['size'], hdr['type']
    counts  = hdr.get('count', [1]*len(fields))
    n_pts   = hdr['points']
    fmt     = hdr['data']

    dt_list = []
    for name, tp, sz, cnt in zip(fields, types, sizes, counts):
        np_type = _PCD_TYPE_MAP.get((tp, sz), np.float32)
        dt_list.append((name, np_type) if cnt == 1 else (name, np_type, (cnt,)))
    dtype = np.dtype(dt_list)

    if fmt == 'ascii':
        rows = []
        for line in raw.decode('utf-8', errors='replace').splitlines():
            line = line.strip()
            if line:
                rows.append(tuple(
                    float(v) if t == 'F' else int(float(v))
                    for v, t in zip(line.split(), types)
                ))
        arr = np.array(rows, dtype=dtype)
    elif fmt == 'binary':
        arr = np.frombuffer(raw[:n_pts * dtype.itemsize], dtype=dtype).copy()
    elif fmt == 'binary_compressed':
        import lzf
        comp_size   = struct.unpack_from('<I', raw, 0)[0]
        decomp_size = struct.unpack_from('<I', raw, 4)[0]
        data_bytes  = lzf.decompress(raw[8:8+comp_size], decomp_size)
        arr = np.frombuffer(data_bytes, dtype=dtype).copy()
    else:
        raise ValueError(f"Unknown format: {fmt}")

    arr  = arr[:n_pts]
    xyz  = np.column_stack([arr['x'].astype(float),
                             arr['y'].astype(float),
                             arr['z'].astype(float)])

    # Voxel downsample for display
    if voxel > 0:
        idx = np.floor(xyz / voxel).astype(np.int64)
        idx -= idx.min(axis=0)
        dims = idx.max(axis=0) + 1
        flat = idx[:,0]*dims[1]*dims[2] + idx[:,1]*dims[2] + idx[:,2]
        _, keep = np.unique(flat, return_index=True)
        xyz = xyz[keep]

    print(f"    {n_pts:,} points -> {len(xyz):,} after downsample (voxel={voxel}m)")
    return xyz


# ── Interactive picker ────────────────────────────────────────────────────────

class OffsetPicker:
    def __init__(self, xyz1, xyz2, label1, label2):
        self.xyz1   = xyz1
        self.xyz2   = xyz2
        self.label1 = label1
        self.label2 = label2

        # Picked points: list of (x,y,z) from each cloud
        self.picks1 = []
        self.picks2 = []
        self.active = 1   # which cloud we're picking from next

        self._build_ui()

    def _build_ui(self):
        self.fig = plt.figure(figsize=(16, 9), facecolor='#1a1a2e')
        self.fig.canvas.manager.set_window_title("Block Offset Picker")

        gs = gridspec.GridSpec(2, 2, figure=self.fig,
                               wspace=0.05, hspace=0.1,
                               left=0.04, right=0.96,
                               top=0.92, bottom=0.08)

        ax_col = '#e0e0e0'
        self.axes = {}
        titles = {
            (0,0): f'{self.label1}  —  TOP (X·Y)',
            (0,1): f'{self.label2}  —  TOP (X·Y)',
            (1,0): f'{self.label1}  —  SIDE (Y·Z)',
            (1,1): f'{self.label2}  —  SIDE (Y·Z)',
        }
        for (r,c), title in titles.items():
            ax = self.fig.add_subplot(gs[r, c])
            ax.set_facecolor('#0d0d1a')
            ax.tick_params(colors=ax_col, labelsize=7)
            for spine in ax.spines.values():
                spine.set_edgecolor('#333355')
            ax.set_title(title, color=ax_col, fontsize=8, pad=4)
            self.axes[(r,c)] = ax

        # Plot point clouds
        s = 0.3   # dot size
        a = 0.15  # alpha

        def scatter(ax, pts, xi, yi):
            ax.scatter(pts[:,xi], pts[:,yi], s=s, c='#4fc3f7', alpha=a,
                       linewidths=0, rasterized=True)

        scatter(self.axes[(0,0)], self.xyz1, 0, 1)
        scatter(self.axes[(0,1)], self.xyz2, 0, 1)
        scatter(self.axes[(1,0)], self.xyz1, 1, 2)
        scatter(self.axes[(1,1)], self.xyz2, 1, 2)

        for ax in self.axes.values():
            ax.set_aspect('equal')

        # Status bar
        self.status = self.fig.text(
            0.5, 0.01,
            self._status_text(),
            ha='center', va='bottom',
            color='#ffd54f', fontsize=9,
            fontfamily='monospace',
            bbox=dict(facecolor='#0d0d1a', edgecolor='#333355', pad=4)
        )

        self.fig.canvas.mpl_connect('button_press_event', self._on_click)
        self.fig.canvas.mpl_connect('key_press_event',    self._on_key)

        # Marker collections (updated on each pick)
        self._marker_artists = []
        self._update_markers()

    def _status_text(self):
        n1, n2 = len(self.picks1), len(self.picks2)
        pairs  = min(n1, n2)
        target = f"{'BLOCK1' if self.active == 1 else 'BLOCK2'}"
        return (f"  Picks: block1={n1}  block2={n2}  pairs={pairs}  "
                f"│  Next click → {target}  "
                f"│  ENTER=compute  R=reset  Q=quit  ")

    def _on_click(self, event):
        if event.inaxes is None or event.button != 1:
            return

        ax   = event.inaxes
        x_cl = event.xdata
        y_cl = event.ydata

        # Which view was clicked?
        for (r, c), a in self.axes.items():
            if a is ax:
                row, col = r, c
                break
        else:
            return

        is_block1 = (col == 0)
        xi, yi = (0, 1) if row == 0 else (1, 2)
        zi     = 2 if row == 0 else 0

        # Find nearest point in that cloud
        pts  = self.xyz1 if is_block1 else self.xyz2
        tree = KDTree(pts[:, [xi, yi]])
        dist, idx = tree.query([x_cl, y_cl])
        pt3d = pts[idx]

        if is_block1:
            self.picks1.append(pt3d)
            self.active = 2
            print(f"  block1 pick {len(self.picks1)}: "
                  f"({pt3d[0]:+.4f}, {pt3d[1]:+.4f}, {pt3d[2]:+.4f})")
        else:
            self.picks2.append(pt3d)
            self.active = 1
            print(f"  block2 pick {len(self.picks2)}: "
                  f"({pt3d[0]:+.4f}, {pt3d[1]:+.4f}, {pt3d[2]:+.4f})")

        self._update_markers()
        self.status.set_text(self._status_text())
        self.fig.canvas.draw_idle()

    def _update_markers(self):
        for art in self._marker_artists:
            art.remove()
        self._marker_artists = []

        colors1 = plt.cm.autumn(np.linspace(0.2, 0.9, max(len(self.picks1), 1)))
        colors2 = plt.cm.winter(np.linspace(0.2, 0.9, max(len(self.picks2), 1)))

        def mark(ax, pt, xi, yi, color, idx):
            art = ax.plot(pt[xi], pt[yi], 'o',
                          color=color, markersize=8,
                          markeredgecolor='white', markeredgewidth=0.8,
                          zorder=10)[0]
            txt = ax.text(pt[xi], pt[yi], f' {idx+1}',
                          color='white', fontsize=7, zorder=11)
            self._marker_artists += [art, txt]

        for i, pt in enumerate(self.picks1):
            mark(self.axes[(0,0)], pt, 0, 1, colors1[i], i)
            mark(self.axes[(1,0)], pt, 1, 2, colors1[i], i)

        for i, pt in enumerate(self.picks2):
            mark(self.axes[(0,1)], pt, 0, 1, colors2[i], i)
            mark(self.axes[(1,1)], pt, 1, 2, colors2[i], i)

    def _on_key(self, event):
        if event.key == 'enter':
            self._compute()
        elif event.key == 'r':
            self.picks1.clear()
            self.picks2.clear()
            self.active = 1
            self._update_markers()
            self.status.set_text(self._status_text())
            self.fig.canvas.draw_idle()
            print("  Picks reset.")
        elif event.key == 'q':
            plt.close(self.fig)

    def _compute(self):
        n = min(len(self.picks1), len(self.picks2))
        if n == 0:
            print("  No pairs picked yet.")
            return

        p1 = np.array(self.picks1[:n])
        p2 = np.array(self.picks2[:n])
        offsets = p1 - p2   # how much to shift block2 to match block1

        mean_off = offsets.mean(axis=0)
        std_off  = offsets.std(axis=0) if n > 1 else np.zeros(3)

        print("\n" + "═"*60)
        print(f"  Offset computed from {n} point pair(s):")
        print(f"  dX = {mean_off[0]:+.5f} m  (std={std_off[0]:.5f})")
        print(f"  dY = {mean_off[1]:+.5f} m  (std={std_off[1]:.5f})")
        print(f"  dZ = {mean_off[2]:+.5f} m  (std={std_off[2]:.5f})")
        print()
        print(f"  Use in merge_blocks.py:")
        print(f"    --offset-x {mean_off[0]:.4f} --offset-y {mean_off[1]:.4f}")
        print("═"*60 + "\n")

        if std_off.max() > 0.05:
            print("  !! High std — picked points may not be true correspondences.")
            print("     Try picking more distinctive features (pipe ends, corners).")


def main():
    parser = argparse.ArgumentParser(description="Interactive offset picker for merge_blocks.py")
    parser.add_argument("block1")
    parser.add_argument("block2")
    parser.add_argument("--voxel", type=float, default=0.02,
                        help="Display voxel size in metres (default 0.02 — increase if slow)")
    args = parser.parse_args()

    xyz1 = load_pcd_xyz(args.block1, voxel=args.voxel)
    xyz2 = load_pcd_xyz(args.block2, voxel=args.voxel)

    print("\nInstructions:")
    print("  1. Click a distinctive point on the LEFT panels (block1)")
    print("  2. Click the SAME point on the RIGHT panels (block2)")
    print("  3. Repeat 3-5 times for accuracy")
    print("  4. Press ENTER to compute the offset")
    print("  5. Use the printed --offset-x/y values in merge_blocks.py\n")

    picker = OffsetPicker(xyz1, xyz2,
                          label1=Path(args.block1).stem,
                          label2=Path(args.block2).stem)
    plt.show()


if __name__ == "__main__":
    main()