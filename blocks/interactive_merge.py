"""
Interactive Point Cloud Merger
================================
Load multiple PCD blocks, pick corresponding point pairs visually,
then run GICP to merge them all into one output file.

Usage:
    python interactive_merge.py block1.pcd block2.pcd block3.pcd block4.pcd

Workflow:
    1. All blocks are shown in different colors in the main 3D-like view
    2. Select SOURCE block (the one to move) from the dropdown
    3. Select TARGET block (the one to align to) from the dropdown
    4. Click matching points on each block (3+ pairs recommended)
    5. Click "Align with GICP" — block is moved and merged
    6. Repeat for remaining blocks
    7. Click "Save Merged" to write the output PCD

Controls:
    - Top/Side/Front view buttons to switch projection
    - Scroll to zoom, drag to pan
    - R = reset picks for current pair
    - S = save merged output

Dependencies:
    pip install numpy scipy matplotlib
"""

import sys
import struct
import argparse
import os
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import multiprocessing

import numpy as np
import matplotlib
matplotlib.use('TkAgg' if 'DISPLAY' in os.environ else 'Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Button, RadioButtons
from scipy.spatial import KDTree
from scipy.linalg import svd

N_JOBS = multiprocessing.cpu_count()

# ═══════════════════════════════════════════════════════════════════════════════
#  PCD I/O
# ═══════════════════════════════════════════════════════════════════════════════

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
        if   key == 'FIELDS': header['fields'] = [v.lower() for v in vals]
        elif key == 'SIZE':   header['size']   = [int(v) for v in vals]
        elif key == 'TYPE':   header['type']   = [v.upper() for v in vals]
        elif key == 'COUNT':  header['count']  = [int(v) for v in vals]
        elif key == 'POINTS': header['points'] = int(vals[0])
        elif key == 'DATA':
            header['data'] = vals[0].lower()
            break
    return header

def load_pcd(path, display_voxel=0.02):
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

    arr = arr[:n_pts]
    xyz_full = np.column_stack([arr['x'].astype(float),
                                 arr['y'].astype(float),
                                 arr['z'].astype(float)])
    extra = {name: arr[name].copy() for name in fields if name not in ('x','y','z')}

    # Downsample for display
    xyz_disp = _voxel_downsample(xyz_full, display_voxel)
    print(f"    {n_pts:,} pts  display={len(xyz_disp):,}  path={path}")
    return xyz_full, xyz_disp, extra, hdr

def save_pcd(path, xyz, extra, orig_hdr):
    n      = len(xyz)
    fields = orig_hdr['fields']
    sizes  = orig_hdr['size']
    types  = orig_hdr['type']
    counts = orig_hdr.get('count', [1]*len(fields))

    dt_list = []
    for name, tp, sz, cnt in zip(fields, types, sizes, counts):
        np_type = _PCD_TYPE_MAP.get((tp, sz), np.float32)
        dt_list.append((name, np_type) if cnt == 1 else (name, np_type, (cnt,)))
    dtype = np.dtype(dt_list)
    arr   = np.zeros(n, dtype=dtype)

    for name, tp, sz in zip(fields, types, sizes):
        np_type = _PCD_TYPE_MAP.get((tp, sz), np.float32)
        if   name == 'x': arr['x'] = xyz[:,0].astype(np_type)
        elif name == 'y': arr['y'] = xyz[:,1].astype(np_type)
        elif name == 'z': arr['z'] = xyz[:,2].astype(np_type)
        elif name in extra: arr[name] = extra[name].astype(np_type)

    header = '\n'.join([
        'VERSION 0.7',
        f'FIELDS {" ".join(fields)}',
        f'SIZE {" ".join(map(str,sizes))}',
        f'TYPE {" ".join(types)}',
        f'COUNT {" ".join(map(str,counts))}',
        f'WIDTH {n}', 'HEIGHT 1',
        'VIEWPOINT 0 0 0 1 0 0 0',
        f'POINTS {n}', 'DATA binary',
    ]) + '\n'

    with open(path, 'wb') as f:
        f.write(header.encode())
        f.write(arr.tobytes())
    print(f"  Saved {n:,} pts -> {path}")


# ═══════════════════════════════════════════════════════════════════════════════
#  Geometry helpers
# ═══════════════════════════════════════════════════════════════════════════════

def _voxel_downsample(pts, voxel):
    if voxel <= 0: return pts
    idx  = np.floor(pts / voxel).astype(np.int64)
    idx -= idx.min(axis=0)
    dims = idx.max(axis=0) + 1
    flat = idx[:,0]*dims[1]*dims[2] + idx[:,1]*dims[2] + idx[:,2]
    _, keep = np.unique(flat, return_index=True)
    return pts[keep]

def _voxel_downsample_with_extra(xyz, extra, voxel):
    if voxel <= 0: return xyz, extra
    idx  = np.floor(xyz / voxel).astype(np.int64)
    idx -= idx.min(axis=0)
    dims = idx.max(axis=0) + 1
    flat = idx[:,0]*dims[1]*dims[2] + idx[:,1]*dims[2] + idx[:,2]
    order = np.argsort(flat)
    _, _, inv = np.unique(flat[order], return_index=True, return_inverse=True)
    n_vox    = inv.max() + 1
    counts_v = np.bincount(inv, minlength=n_vox).astype(float)
    xyz_out  = np.zeros((n_vox, 3))
    for d in range(3):
        xyz_out[:,d] = np.bincount(inv, weights=xyz[order,d], minlength=n_vox) / counts_v
    extra_out = {}
    for name, arr in extra.items():
        a = arr[order].astype(float)
        extra_out[name] = (np.bincount(inv, weights=a, minlength=n_vox) / counts_v).astype(arr.dtype)
    return xyz_out, extra_out

def apply_transform(pts, T):
    return (T[:3,:3] @ pts.T).T + T[:3,3]

def _clamp_so3(R):
    U, _, Vt = svd(R)
    return U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt

def _compute_covs_chunk(pts, idx_chunk):
    nb   = pts[idx_chunk]
    diff = nb - nb.mean(axis=1, keepdims=True)
    return np.einsum('nki,nkj->nij', diff, diff) / max(idx_chunk.shape[1]-1, 1)

def _compute_normals_chunk(pts, idx_chunk):
    nb   = pts[idx_chunk]
    diff = nb - nb.mean(axis=1, keepdims=True)
    covs = np.einsum('nki,nkj->nij', diff, diff) / max(idx_chunk.shape[1]-1, 1)
    _, vecs = np.linalg.eigh(covs)
    normals = vecs[:,:,0]
    norms   = np.linalg.norm(normals, axis=1, keepdims=True)
    return normals / np.where(norms < 1e-10, 1.0, norms)

def estimate_covariances(pts, k=20):
    k      = min(k, len(pts)-1)
    _, idx = KDTree(pts).query(pts, k=k, workers=-1)
    chunks = np.array_split(idx, N_JOBS)
    with ThreadPoolExecutor(max_workers=N_JOBS) as ex:
        results = list(ex.map(lambda c: _compute_covs_chunk(pts, c), chunks))
    return np.vstack(results)

def estimate_normals(pts, k=20):
    k      = min(k, len(pts)-1)
    _, idx = KDTree(pts).query(pts, k=k, workers=-1)
    chunks = np.array_split(idx, N_JOBS)
    with ThreadPoolExecutor(max_workers=N_JOBS) as ex:
        results = list(ex.map(lambda c: _compute_normals_chunk(pts, c), chunks))
    return np.vstack(results)

def gicp_step(src, tgt, src_covs, tgt_covs, tgt_normals, tgt_tree, max_corr):
    dists, idx = tgt_tree.query(src, k=1, workers=-1)
    mask = dists < max_corr
    if mask.sum() < 6: return np.eye(4)

    src_m   = src[mask];     tgt_m   = tgt[idx[mask]]
    normals = tgt_normals[idx[mask]]
    Cs      = src_covs[mask]; Ct      = tgt_covs[idx[mask]]

    px, py, pz = src_m[:,0], src_m[:,1], src_m[:,2]
    J_rot = np.stack([
         normals[:,1]*pz - normals[:,2]*py,
        -normals[:,0]*pz + normals[:,2]*px,
         normals[:,0]*py - normals[:,1]*px,
    ], axis=1)
    J   = np.concatenate([J_rot, normals], axis=1)
    C   = Ct + Cs
    nCn = np.einsum('mi,mij,mj->m', normals, C, normals)
    w   = 1.0 / (nCn + 1e-6)
    d   = np.einsum('mi,mi->m', normals, tgt_m - src_m)
    wJ  = w[:,None] * J
    H   = J.T @ wJ
    b   = wJ.T @ d

    try:    xi = np.linalg.solve(H + 1e-6*np.eye(6), b)
    except: return np.eye(4)

    wx, wy, wz = xi[:3]
    dT = np.eye(4)
    dT[:3,:3] = _clamp_so3(np.array([[1,-wz,wy],[wz,1,-wx],[-wy,wx,1]]))
    dT[:3,3]  = xi[3:]
    return dT

def run_gicp(src, tgt, init_T, voxel=0.005, max_corr=0.05, max_iter=100, k=20):
    src_d = _voxel_downsample(src, voxel)
    tgt_d = _voxel_downsample(tgt, voxel)
    src_covs    = estimate_covariances(src_d, k)
    tgt_covs    = estimate_covariances(tgt_d, k)
    tgt_normals = estimate_normals(tgt_d, k)
    tgt_tree    = KDTree(tgt_d)
    T, prev_rmse = init_T.copy(), np.inf
    for it in range(max_iter):
        src_t      = apply_transform(src_d, T)
        src_covs_t = np.einsum('ij,njk,lk->nil', T[:3,:3], src_covs, T[:3,:3])
        dT         = gicp_step(src_t, tgt_d, src_covs_t, tgt_covs,
                               tgt_normals, tgt_tree, max_corr)
        T = dT @ T
        dists, _ = tgt_tree.query(apply_transform(src_d, T), k=1)
        mask     = dists < max_corr
        fitness  = float(mask.mean())
        rmse     = float(dists[mask].mean()) if mask.any() else np.inf
        if abs(prev_rmse - rmse) < 1e-7: break
        prev_rmse = rmse
        if (it+1) % 10 == 0:
            print(f"    iter {it+1}  fitness={fitness:.3f}  rmse={rmse:.5f}m")
    print(f"  GICP done: fitness={fitness:.3f}  rmse={rmse:.5f}m")
    return T, fitness, rmse


# ═══════════════════════════════════════════════════════════════════════════════
#  Main interactive app
# ═══════════════════════════════════════════════════════════════════════════════

COLORS = ['#4fc3f7', '#ff7043', '#66bb6a', '#ffa726',
          '#ab47bc', '#26c6da', '#d4e157', '#ef5350']

VIEWS = {
    'Top  (X·Y)':  (0, 1, 'X', 'Y'),
    'Front(X·Z)':  (0, 2, 'X', 'Z'),
    'Side (Y·Z)':  (1, 2, 'Y', 'Z'),
}

class MergeApp:
    def __init__(self, paths, output, display_voxel, gicp_voxel, max_corr, stages):
        self.output      = output
        self.gicp_voxel  = gicp_voxel
        self.max_corr    = max_corr
        self.stages      = stages

        # Load all blocks
        self.labels = [Path(p).stem for p in paths]
        self.xyz_full = []   # full resolution for GICP
        self.xyz_disp = []   # downsampled for display
        self.extras   = []
        self.hdrs     = []
        self.transforms = [np.eye(4) for _ in paths]  # current transform per block
        self.merged_into = [False] * len(paths)        # block[0] is anchor
        self.merged_into[0] = True

        for p in paths:
            full, disp, extra, hdr = load_pcd(p, display_voxel)
            self.xyz_full.append(full)
            self.xyz_disp.append(disp)
            self.extras.append(extra)
            self.hdrs.append(hdr)

        self.n = len(paths)
        self.view_key = 'Top  (X·Y)'
        self.src_idx  = 1   # block to move
        self.tgt_idx  = 0   # block to align to

        # Pick state
        self.src_picks = []   # list of 3D points from src
        self.tgt_picks = []   # list of 3D points from tgt
        self.next_pick = 'tgt'  # alternate: pick tgt first then src

        self._build_ui()
        self._redraw()

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        self.fig = plt.figure(figsize=(18, 10), facecolor='#111118')
        self.fig.canvas.manager.set_window_title("Interactive Point Cloud Merger")

        gs = gridspec.GridSpec(1, 2, figure=self.fig,
                               left=0.22, right=0.98,
                               top=0.95, bottom=0.05,
                               wspace=0.02)

        # Main view
        self.ax = self.fig.add_subplot(gs[0, 0])
        self.ax.set_facecolor('#0a0a14')
        for sp in self.ax.spines.values(): sp.set_edgecolor('#223')
        self.ax.tick_params(colors='#888', labelsize=7)

        # Overview (all blocks)
        self.ax_all = self.fig.add_subplot(gs[0, 1])
        self.ax_all.set_facecolor('#0a0a14')
        for sp in self.ax_all.spines.values(): sp.set_edgecolor('#223')
        self.ax_all.tick_params(colors='#888', labelsize=7)
        self.ax_all.set_title('All blocks (overview)', color='#aaa', fontsize=8)

        c = '#ccc'
        btn_kw = dict(color='#1a1a2e', hovercolor='#2a2a4e')

        # ── Left panel controls ───────────────────────────────────────────────
        y = 0.93
        def lbl(text, yy, size=8, color='#888'):
            self.fig.text(0.01, yy, text, color=color, fontsize=size,
                          fontfamily='monospace')

        lbl('── VIEW ──', y, color='#4fc3f7')
        y -= 0.06
        ax_view = self.fig.add_axes([0.01, y-0.10, 0.18, 0.12])
        self.rb_view = RadioButtons(ax_view, list(VIEWS.keys()),
                                    activecolor='#4fc3f7')
        ax_view.set_facecolor('#0d0d1a')
        for lbl_obj in self.rb_view.labels:
            lbl_obj.set_color('#ccc'); lbl_obj.set_fontsize(8)
        self.rb_view.on_clicked(self._on_view)
        y -= 0.17

        lbl('── SOURCE (move) ──', y, color='#ff7043')
        y -= 0.05
        ax_src = self.fig.add_axes([0.01, y-0.14, 0.18, 0.16])
        self.rb_src = RadioButtons(ax_src, self.labels, activecolor='#ff7043')
        ax_src.set_facecolor('#0d0d1a')
        for lbl_obj in self.rb_src.labels:
            lbl_obj.set_color('#ccc'); lbl_obj.set_fontsize(8)
        self.rb_src.on_clicked(self._on_src)
        y -= 0.20

        lbl('── TARGET (anchor) ──', y, color='#66bb6a')
        y -= 0.05
        ax_tgt = self.fig.add_axes([0.01, y-0.14, 0.18, 0.16])
        self.rb_tgt = RadioButtons(ax_tgt, self.labels, activecolor='#66bb6a')
        ax_tgt.set_facecolor('#0d0d1a')
        for lbl_obj in self.rb_tgt.labels:
            lbl_obj.set_color('#ccc'); lbl_obj.set_fontsize(8)
        self.rb_tgt.set_active(0)
        self.rb_tgt.on_clicked(self._on_tgt)
        y -= 0.20

        # Buttons
        def make_btn(label, yy, color='#1a1a2e'):
            ax_b = self.fig.add_axes([0.01, yy, 0.18, 0.045])
            btn  = Button(ax_b, label, color=color, hovercolor='#2a3a5e')
            for item in [btn.label]: item.set_color('#eee'); item.set_fontsize(9)
            return btn

        y = max(y - 0.06, 0.28)
        self.btn_align = make_btn('▶  Align with GICP', y, '#0d2040')
        self.btn_align.on_clicked(self._on_align)

        self.btn_reset = make_btn('✕  Reset picks', y-0.06, '#200d0d')
        self.btn_reset.on_clicked(self._on_reset)

        self.btn_save = make_btn('💾  Save merged PCD', y-0.12, '#0d2010')
        self.btn_save.on_clicked(self._on_save)

        # Status text
        self.status_txt = self.fig.text(
            0.5, 0.005, self._status(),
            ha='center', color='#ffd54f', fontsize=8,
            fontfamily='monospace',
            bbox=dict(facecolor='#0d0d1a', edgecolor='#333', pad=3))

        self.fig.canvas.mpl_connect('button_press_event', self._on_click)
        self.fig.canvas.mpl_connect('key_press_event',    self._on_key)
        self.fig.canvas.mpl_connect('scroll_event',       self._on_scroll)

        self._press = None
        self.fig.canvas.mpl_connect('button_press_event',   self._pan_start)
        self.fig.canvas.mpl_connect('button_release_event', self._pan_end)
        self.fig.canvas.mpl_connect('motion_notify_event',  self._pan_move)

    # ── Pan / zoom ────────────────────────────────────────────────────────────

    def _on_scroll(self, event):
        if event.inaxes not in (self.ax, self.ax_all): return
        ax = event.inaxes
        xl, yl = ax.get_xlim(), ax.get_ylim()
        cx = (xl[0]+xl[1])/2; cy = (yl[0]+yl[1])/2
        factor = 0.85 if event.button == 'up' else 1.15
        dx = (xl[1]-xl[0])/2*factor; dy = (yl[1]-yl[0])/2*factor
        ax.set_xlim(cx-dx, cx+dx); ax.set_ylim(cy-dy, cy+dy)
        self.fig.canvas.draw_idle()

    def _pan_start(self, event):
        if event.button == 3 and event.inaxes in (self.ax, self.ax_all):
            self._press = (event.inaxes, event.xdata, event.ydata,
                           event.inaxes.get_xlim(), event.inaxes.get_ylim())

    def _pan_end(self, event):
        self._press = None

    def _pan_move(self, event):
        if self._press is None: return
        ax, x0, y0, xl, yl = self._press
        if event.xdata is None: return
        dx = x0 - event.xdata; dy = y0 - event.ydata
        ax.set_xlim(xl[0]+dx, xl[1]+dx)
        ax.set_ylim(yl[0]+dy, yl[1]+dy)
        self.fig.canvas.draw_idle()

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_view(self, label):
        self.view_key = label
        self._redraw()

    def _on_src(self, label):
        self.src_idx = self.labels.index(label)
        self.src_picks.clear(); self.tgt_picks.clear()
        self.next_pick = 'tgt'
        self._redraw()

    def _on_tgt(self, label):
        self.tgt_idx = self.labels.index(label)
        self.src_picks.clear(); self.tgt_picks.clear()
        self.next_pick = 'tgt'
        self._redraw()

    def _on_reset(self, event=None):
        self.src_picks.clear(); self.tgt_picks.clear()
        self.next_pick = 'tgt'
        self._redraw()

    def _on_key(self, event):
        if event.key == 'r': self._on_reset()
        elif event.key == 's': self._on_save(None)

    def _on_click(self, event):
        if event.inaxes is not self.ax: return
        if event.button != 1: return
        if event.xdata is None: return

        xi, yi, _, _ = VIEWS[self.view_key]
        x_cl, y_cl = event.xdata, event.ydata

        # Pick from src or tgt cloud?
        if self.next_pick == 'tgt':
            pts  = apply_transform(self.xyz_disp[self.tgt_idx],
                                   self.transforms[self.tgt_idx])
            tree = KDTree(pts[:, [xi, yi]])
            _, idx = tree.query([x_cl, y_cl])
            pt3d = pts[idx]
            self.tgt_picks.append(pt3d)
            self.next_pick = 'src'
            print(f"  TGT pick {len(self.tgt_picks)}: "
                  f"({pt3d[0]:+.4f}, {pt3d[1]:+.4f}, {pt3d[2]:+.4f})")
        else:
            pts  = apply_transform(self.xyz_disp[self.src_idx],
                                   self.transforms[self.src_idx])
            tree = KDTree(pts[:, [xi, yi]])
            _, idx = tree.query([x_cl, y_cl])
            pt3d = pts[idx]
            self.src_picks.append(pt3d)
            self.next_pick = 'tgt'
            print(f"  SRC pick {len(self.src_picks)}: "
                  f"({pt3d[0]:+.4f}, {pt3d[1]:+.4f}, {pt3d[2]:+.4f})")

        self._update_status()
        self._draw_picks()

    def _on_align(self, event):
        n = min(len(self.src_picks), len(self.tgt_picks))
        if n < 1:
            print("  Need at least 1 point pair. Click matching points first.")
            self._update_status("Need at least 1 point pair!")
            return

        src_pts = np.array(self.src_picks[:n])
        tgt_pts = np.array(self.tgt_picks[:n])

        # Compute initial transform from picked point pairs
        # Translation: centroid of tgt picks - centroid of src picks
        t_init = tgt_pts.mean(axis=0) - src_pts.mean(axis=0)
        T_init = np.eye(4)
        T_init[:3, 3] = t_init
        print(f"\n  Initial offset from {n} picks: "
              f"dX={t_init[0]:+.4f} dY={t_init[1]:+.4f} dZ={t_init[2]:+.4f}")

        # Apply initial transform to get src near tgt
        T_current = self.transforms[self.src_idx]
        T_init_full = T_init @ T_current

        # Run multi-scale GICP
        src_full = apply_transform(self.xyz_full[self.src_idx], T_current)
        tgt_full = apply_transform(self.xyz_full[self.tgt_idx],
                                   self.transforms[self.tgt_idx])

        print(f"  Running {self.stages}-stage GICP ...")
        T_refine = np.eye(4)
        T_refine[:3, 3] = t_init  # start from picked offset

        for stage in range(self.stages, 0, -1):
            factor   = 2 ** (stage - 1)
            vox      = self.gicp_voxel * factor
            corr     = self.max_corr * factor
            print(f"    Stage {self.stages-stage+1}/{self.stages} "
                  f"voxel={vox:.4f}m corr={corr:.4f}m ...")
            T_refine, fitness, rmse = run_gicp(
                src_full, tgt_full,
                init_T   = T_refine,
                voxel    = vox,
                max_corr = corr,
                max_iter = 50,
                k        = 15,
            )
            if fitness == 0:
                print(f"    !! Zero fitness — picks may be off, try again")
                break

        # Update the stored transform for this block
        self.transforms[self.src_idx] = T_refine
        self.merged_into[self.src_idx] = True

        # Clear picks and redraw
        self.src_picks.clear(); self.tgt_picks.clear()
        self.next_pick = 'tgt'
        self._redraw()
        self._update_status(f"Aligned {self.labels[self.src_idx]} -> "
                            f"{self.labels[self.tgt_idx]}  "
                            f"rmse={rmse:.4f}m")

    def _on_save(self, event):
        print(f"\n  Merging and saving to {self.output} ...")
        all_xyz   = []
        all_names = set()
        for i in range(self.n):
            T   = self.transforms[i]
            xyz = apply_transform(self.xyz_full[i], T)
            all_xyz.append(xyz)
            all_names.update(self.extras[i].keys())

        merged_xyz   = np.vstack(all_xyz)
        merged_extra = {}
        for name in all_names:
            parts = []
            for i in range(self.n):
                if name in self.extras[i]:
                    parts.append(self.extras[i][name])
                else:
                    ref   = next(iter(self.extras[i].values()), None)
                    dtype = ref.dtype if ref is not None else np.float32
                    parts.append(np.zeros(len(self.xyz_full[i]), dtype=dtype))
            merged_extra[name] = np.concatenate(parts)

        save_pcd(self.output, merged_xyz, merged_extra, self.hdrs[0])
        self._update_status(f"Saved {len(merged_xyz):,} pts -> {self.output}")

    # ── Drawing ───────────────────────────────────────────────────────────────

    def _current_axes(self):
        return VIEWS[self.view_key]  # (xi, yi, xlabel, ylabel)

    def _redraw(self):
        xi, yi, xl, yl = self._current_axes()

        # ── Main view: src + tgt only ─────────────────────────────────────────
        self.ax.cla()
        self.ax.set_facecolor('#0a0a14')
        self.ax.set_xlabel(xl, color='#666', fontsize=7)
        self.ax.set_ylabel(yl, color='#666', fontsize=7)
        self.ax.set_title(
            f'SRC={self.labels[self.src_idx]} (red)  '
            f'TGT={self.labels[self.tgt_idx]} (green) '
            f'— click: first TGT then SRC',
            color='#aaa', fontsize=8)

        for i, label in ((self.tgt_idx, 'tgt'), (self.src_idx, 'src')):
            pts = apply_transform(self.xyz_disp[i], self.transforms[i])
            col = '#66bb6a' if label == 'tgt' else '#ff7043'
            self.ax.scatter(pts[:,xi], pts[:,yi], s=0.3, c=col,
                            alpha=0.15, linewidths=0, rasterized=True)

        self.ax.set_aspect('equal')

        # ── Overview: all blocks ──────────────────────────────────────────────
        self.ax_all.cla()
        self.ax_all.set_facecolor('#0a0a14')
        self.ax_all.set_title('All blocks (overview)', color='#aaa', fontsize=8)

        for i in range(self.n):
            pts = apply_transform(self.xyz_disp[i], self.transforms[i])
            col = COLORS[i % len(COLORS)]
            self.ax_all.scatter(pts[:,xi], pts[:,yi], s=0.2, c=col,
                                alpha=0.12, linewidths=0, rasterized=True,
                                label=self.labels[i])

        self.ax_all.legend(loc='upper right', fontsize=7,
                           facecolor='#0d0d1a', labelcolor='white',
                           markerscale=10)
        self.ax_all.set_aspect('equal')
        for sp in self.ax_all.spines.values(): sp.set_edgecolor('#223')

        self._draw_picks()
        self.fig.canvas.draw_idle()

    def _draw_picks(self):
        xi, yi, _, _ = self._current_axes()

        # Remove old pick markers
        for art in getattr(self, '_pick_artists', []):
            try: art.remove()
            except: pass
        self._pick_artists = []

        colors_t = plt.cm.Greens(np.linspace(0.5, 0.9, max(len(self.tgt_picks),1)))
        colors_s = plt.cm.Reds(np.linspace(0.5, 0.9, max(len(self.src_picks),1)))

        for i, pt in enumerate(self.tgt_picks):
            a = self.ax.plot(pt[xi], pt[yi], 'o', color=colors_t[i],
                             ms=9, mew=1, mec='white', zorder=20)[0]
            b = self.ax.text(pt[xi], pt[yi], f' T{i+1}',
                             color='white', fontsize=7, zorder=21)
            self._pick_artists += [a, b]

        for i, pt in enumerate(self.src_picks):
            a = self.ax.plot(pt[xi], pt[yi], 's', color=colors_s[i],
                             ms=9, mew=1, mec='white', zorder=20)[0]
            b = self.ax.text(pt[xi], pt[yi], f' S{i+1}',
                             color='white', fontsize=7, zorder=21)
            self._pick_artists += [a, b]

        self.fig.canvas.draw_idle()

    def _status(self, msg=None):
        if msg:
            return f"  {msg}  "
        n = min(len(self.src_picks), len(self.tgt_picks))
        nxt = 'TARGET' if self.next_pick == 'tgt' else 'SOURCE'
        merged = sum(self.merged_into)
        return (f"  Picks: tgt={len(self.tgt_picks)} src={len(self.src_picks)} "
                f"pairs={n}  │  Next click → {nxt}  "
                f"│  Merged: {merged}/{self.n}  "
                f"│  R=reset  S=save  Scroll=zoom  RightDrag=pan  ")

    def _update_status(self, msg=None):
        self.status_txt.set_text(self._status(msg))
        self.fig.canvas.draw_idle()

    def show(self):
        plt.show()


# ═══════════════════════════════════════════════════════════════════════════════
#  CLI
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Interactive point cloud merger — pick points, run GICP, save.")
    parser.add_argument("files", nargs="+", help="PCD files to merge (first = anchor)")
    parser.add_argument("--output",       default="interactive_merged.pcd")
    parser.add_argument("--display-voxel",type=float, default=0.02,
                        help="Voxel size for display (default 0.02m — increase if slow)")
    parser.add_argument("--gicp-voxel",   type=float, default=0.005,
                        help="Finest GICP voxel size (default 0.005m)")
    parser.add_argument("--max-corr-dist",type=float, default=0.05,
                        help="Finest GICP correspondence distance (default 0.05m)")
    parser.add_argument("--stages",       type=int,   default=3,
                        help="GICP cascade stages (default 3)")
    args = parser.parse_args()

    print(f"\nLoading {len(args.files)} blocks ...")
    app = MergeApp(
        paths        = args.files,
        output       = args.output,
        display_voxel= args.display_voxel,
        gicp_voxel   = args.gicp_voxel,
        max_corr     = args.max_corr_dist,
        stages       = args.stages,
    )

    print("\n── Instructions ───────────────────────────────────────────────")
    print("  1. Select SOURCE block (to move) and TARGET block (anchor)")
    print("  2. Click a point on TARGET cloud (green marker T1)")
    print("  3. Click matching point on SOURCE cloud (red marker S1)")
    print("  4. Repeat 3-5 times for different features")
    print("  5. Click 'Align with GICP' — watches SOURCE snap to TARGET")
    print("  6. Repeat for each remaining block")
    print("  7. Click 'Save merged PCD' or press S")
    print("───────────────────────────────────────────────────────────────\n")

    app.show()

if __name__ == "__main__":
    main()