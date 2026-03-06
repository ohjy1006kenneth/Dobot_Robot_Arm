"""
Block Grid Merger
=================
Merges 4 pre-merged block .pcd files arranged in a 2x2 grid:

    block1 ---1m--- block3
      |                |
     60cm             60cm
      |                |
    block2 ---1m--- block4

Alignment chain (always between direct neighbours):
    Step 1: block2 -> block1  (vertical,   ~60cm)
    Step 2: block3 -> block1  (horizontal, ~1m)
    Step 3: block4 -> block3  (vertical,   ~60cm)

Each block should already be a merged .pcd from merge_pointclouds_gicp.py.

Dependencies:
    pip install numpy scipy

Usage:
    python merge_blocks.py \
        --b1 block1_merged.pcd \
        --b2 block2_merged.pcd \
        --b3 block3_merged.pcd \
        --b4 block4_merged.pcd \
        --output all_blocks_merged.pcd

Optional flags:
    --overlap       FLOAT  Overlap fraction for trimming  (default 0.60)
    --voxel         FLOAT  Finest voxel size, metres      (default 0.003)
    --stages        INT    Cascade stages coarse->fine    (default 5)
    --max-iter      INT    GICP iters per stage           (default 100)
    --max-corr-dist FLOAT  Finest corr distance, metres   (default 0.010)
    --final-voxel   FLOAT  Output voxel size (0=off)      (default 0)
    --sor                  Statistical outlier removal
    --sor-k         INT    SOR neighbours                 (default 20)
    --sor-std       FLOAT  SOR std multiplier             (default 1.0)
    --inspect              Print bounding boxes and exit
"""

import argparse
import struct
import multiprocessing
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
from scipy.spatial import KDTree
from scipy.linalg import svd

N_JOBS = multiprocessing.cpu_count()


# ═══════════════════════════════════════════════════════════════════════════════
#  PCD  I/O  (same as merge_pointclouds_gicp.py)
# ═══════════════════════════════════════════════════════════════════════════════

_PCD_TYPE_MAP = {
    ('F', 4): np.float32, ('F', 8): np.float64,
    ('I', 1): np.int8,    ('I', 2): np.int16,   ('I', 4): np.int32,
    ('U', 1): np.uint8,   ('U', 2): np.uint16,  ('U', 4): np.uint32,
}


def _parse_pcd_header(f):
    header = {}
    while True:
        line_bytes = f.readline()
        line = line_bytes.decode('utf-8', errors='replace').strip()
        if not line or line.startswith('#'):
            continue
        key, *vals = line.split()
        key = key.upper()
        if   key == 'FIELDS': header['fields'] = [v.lower() for v in vals]
        elif key == 'SIZE':   header['size']   = [int(v) for v in vals]
        elif key == 'TYPE':   header['type']   = [v.upper() for v in vals]
        elif key == 'COUNT':  header['count']  = [int(v) for v in vals]
        elif key == 'WIDTH':  header['width']  = int(vals[0])
        elif key == 'HEIGHT': header['height'] = int(vals[0])
        elif key == 'POINTS': header['points'] = int(vals[0])
        elif key == 'DATA':
            header['data'] = vals[0].lower()
            break
    return header


def load_pcd(path: str):
    with open(path, 'rb') as f:
        hdr = _parse_pcd_header(f)
        raw = f.read()

    fields, sizes, types = hdr['fields'], hdr['size'], hdr['type']
    counts   = hdr.get('count', [1] * len(fields))
    n_pts    = hdr['points']
    data_fmt = hdr['data']

    dt_list = []
    for name, tp, sz, cnt in zip(fields, types, sizes, counts):
        np_type = _PCD_TYPE_MAP.get((tp, sz), np.float32)
        dt_list.append((name, np_type) if cnt == 1 else (name, np_type, (cnt,)))
    dtype = np.dtype(dt_list)

    if data_fmt == 'ascii':
        rows = []
        for line in raw.decode('utf-8', errors='replace').splitlines():
            line = line.strip()
            if line:
                rows.append(tuple(
                    float(v) if t == 'F' else int(float(v))
                    for v, t in zip(line.split(), types)
                ))
        arr = np.array(rows, dtype=dtype)
    elif data_fmt == 'binary':
        arr = np.frombuffer(raw[:n_pts * dtype.itemsize], dtype=dtype).copy()
    elif data_fmt == 'binary_compressed':
        try:
            import lzf
            comp_size   = struct.unpack_from('<I', raw, 0)[0]
            decomp_size = struct.unpack_from('<I', raw, 4)[0]
            data_bytes  = lzf.decompress(raw[8:8 + comp_size], decomp_size)
            arr = np.frombuffer(data_bytes, dtype=dtype).copy()
        except ImportError:
            raise RuntimeError("binary_compressed PCD requires: pip install lzf")
    else:
        raise ValueError(f"Unknown PCD data format: {data_fmt!r}")

    arr = arr[:n_pts]
    xyz = np.column_stack([
        arr['x'].astype(np.float64),
        arr['y'].astype(np.float64),
        arr['z'].astype(np.float64),
    ])
    extra = {name: arr[name].copy() for name in fields if name not in ('x','y','z')}
    print(f"  Loaded {n_pts:,} points  <-  {path}")
    return xyz, extra, hdr


def save_pcd(path: str, xyz: np.ndarray, extra: dict, orig_hdr: dict):
    n = len(xyz)
    fields = orig_hdr['fields']
    sizes  = orig_hdr['size']
    types  = orig_hdr['type']
    counts = orig_hdr.get('count', [1] * len(fields))

    # Build structured array for binary output
    dt_list = []
    for name, tp, sz, cnt in zip(fields, types, sizes, counts):
        np_type = _PCD_TYPE_MAP.get((tp, sz), np.float32)
        dt_list.append((name, np_type) if cnt == 1 else (name, np_type, (cnt,)))
    dtype = np.dtype(dt_list)
    arr = np.zeros(n, dtype=dtype)

    for name, tp, sz in zip(fields, types, sizes):
        np_type = _PCD_TYPE_MAP.get((tp, sz), np.float32)
        if   name == 'x': arr['x'] = xyz[:, 0].astype(np_type)
        elif name == 'y': arr['y'] = xyz[:, 1].astype(np_type)
        elif name == 'z': arr['z'] = xyz[:, 2].astype(np_type)
        elif name in extra:
            arr[name] = extra[name].astype(np_type)

    header_lines = [
        'VERSION 0.7',
        f'FIELDS {" ".join(fields)}',
        f'SIZE {" ".join(map(str, sizes))}',
        f'TYPE {" ".join(types)}',
        f'COUNT {" ".join(map(str, counts))}',
        f'WIDTH {n}', 'HEIGHT 1',
        'VIEWPOINT 0 0 0 1 0 0 0',
        f'POINTS {n}', 'DATA binary',
    ]
    with open(path, 'wb') as f:
        f.write(('\n'.join(header_lines) + '\n').encode('utf-8'))
        f.write(arr.tobytes())
    print(f"  Saved  {n:,} points  ->  {path}  (binary)")


# ═══════════════════════════════════════════════════════════════════════════════
#  Inspect
# ═══════════════════════════════════════════════════════════════════════════════

def inspect(label, xyz):
    mn, mx = xyz.min(axis=0), xyz.max(axis=0)
    c = xyz.mean(axis=0)
    print(f"\n  [{label}]  {len(xyz):,} points")
    print(f"    X  [{mn[0]:+.4f} .. {mx[0]:+.4f}]  span={mx[0]-mn[0]:.4f} m")
    print(f"    Y  [{mn[1]:+.4f} .. {mx[1]:+.4f}]  span={mx[1]-mn[1]:.4f} m")
    print(f"    Z  [{mn[2]:+.4f} .. {mx[2]:+.4f}]  span={mx[2]-mn[2]:.4f} m")
    print(f"    centroid ({c[0]:+.4f}, {c[1]:+.4f}, {c[2]:+.4f})")
    return c


# ═══════════════════════════════════════════════════════════════════════════════
#  Voxel helpers
# ═══════════════════════════════════════════════════════════════════════════════

def voxel_downsample_with_extra(xyz, extra, voxel_size):
    if voxel_size <= 0:
        return xyz, extra
    indices  = np.floor(xyz / voxel_size).astype(np.int64)
    mins     = indices.min(axis=0)
    shifted  = indices - mins
    dims     = shifted.max(axis=0) + 1
    flat     = shifted[:,0]*dims[1]*dims[2] + shifted[:,1]*dims[2] + shifted[:,2]
    order    = np.argsort(flat)
    _, _, inv = np.unique(flat[order], return_index=True, return_inverse=True)
    n_vox    = inv.max() + 1
    counts_v = np.bincount(inv, minlength=n_vox).astype(float)
    xyz_out  = np.zeros((n_vox, 3))
    for d in range(3):
        xyz_out[:, d] = np.bincount(inv, weights=xyz[order, d], minlength=n_vox) / counts_v
    extra_out = {}
    for name, arr in extra.items():
        a = arr[order].astype(float)
        extra_out[name] = (np.bincount(inv, weights=a, minlength=n_vox) / counts_v).astype(arr.dtype)
    return xyz_out, extra_out


def voxel_downsample(pts, voxel_size):
    if voxel_size <= 0:
        return pts
    out, _ = voxel_downsample_with_extra(pts, {}, voxel_size)
    return out


# ═══════════════════════════════════════════════════════════════════════════════
#  Normal + covariance estimation
# ═══════════════════════════════════════════════════════════════════════════════

def _compute_normals_chunk(pts, idx_chunk):
    nb   = pts[idx_chunk]
    diff = nb - nb.mean(axis=1, keepdims=True)
    covs = np.einsum('nki,nkj->nij', diff, diff) / max(idx_chunk.shape[1] - 1, 1)
    _, vecs = np.linalg.eigh(covs)
    normals = vecs[:, :, 0]
    norms   = np.linalg.norm(normals, axis=1, keepdims=True)
    return normals / np.where(norms < 1e-10, 1.0, norms)


def _compute_covs_chunk(pts, idx_chunk):
    nb   = pts[idx_chunk]
    diff = nb - nb.mean(axis=1, keepdims=True)
    return np.einsum('nki,nkj->nij', diff, diff) / max(idx_chunk.shape[1] - 1, 1)


def estimate_normals(pts, k=20):
    k      = min(k, len(pts) - 1)
    _, idx = KDTree(pts).query(pts, k=k, workers=-1)
    chunks = np.array_split(idx, N_JOBS)
    with ThreadPoolExecutor(max_workers=N_JOBS) as ex:
        results = list(ex.map(lambda c: _compute_normals_chunk(pts, c), chunks))
    return np.vstack(results)


def estimate_covariances(pts, k=20):
    k      = min(k, len(pts) - 1)
    _, idx = KDTree(pts).query(pts, k=k, workers=-1)
    chunks = np.array_split(idx, N_JOBS)
    with ThreadPoolExecutor(max_workers=N_JOBS) as ex:
        results = list(ex.map(lambda c: _compute_covs_chunk(pts, c), chunks))
    return np.vstack(results)



# ═══════════════════════════════════════════════════════════════════════════════
#  Overlap trimming
# ═══════════════════════════════════════════════════════════════════════════════

def trim_to_overlap(src, tgt, overlap_frac=0.60):
    delta = np.abs(tgt.mean(axis=0) - src.mean(axis=0))
    axis  = int(np.argmax(delta))
    print(f"    Overlap axis: {'XYZ'[axis]}  "
          f"(dX={delta[0]:+.4f} dY={delta[1]:+.4f} dZ={delta[2]:+.4f})")

    src_min, src_max = src[:, axis].min(), src[:, axis].max()
    tgt_min, tgt_max = tgt[:, axis].min(), tgt[:, axis].max()

    if src[:, axis].mean() < tgt[:, axis].mean():
        src_edge = src[src[:, axis] >= src_max - overlap_frac*(src_max-src_min)]
        tgt_edge = tgt[tgt[:, axis] <= tgt_min + overlap_frac*(tgt_max-tgt_min)]
    else:
        src_edge = src[src[:, axis] <= src_min + overlap_frac*(src_max-src_min)]
        tgt_edge = tgt[tgt[:, axis] >= tgt_max - overlap_frac*(tgt_max-tgt_min)]

    print(f"    Trimmed: src {len(src):,}->{len(src_edge):,}  "
          f"tgt {len(tgt):,}->{len(tgt_edge):,}  (frac={overlap_frac:.2f})")
    if len(src_edge) < 50 or len(tgt_edge) < 50:
        print("    !! Too few points in overlap — increase --overlap")
    return src_edge, tgt_edge, axis


# ═══════════════════════════════════════════════════════════════════════════════
#  SO(3) + transform helpers
# ═══════════════════════════════════════════════════════════════════════════════

def _clamp_so3(R):
    U, _, Vt = svd(R)
    return U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt


def apply_transform(pts, T):
    return (T[:3, :3] @ pts.T).T + T[:3, 3]


# ═══════════════════════════════════════════════════════════════════════════════
#  Point-to-plane GICP
# ═══════════════════════════════════════════════════════════════════════════════

def gicp_step_p2plane(src, tgt, src_covs, tgt_covs,
                      tgt_normals, tgt_tree, max_corr_dist):
    dists, idx = tgt_tree.query(src, k=1, workers=-1)
    mask = dists < max_corr_dist
    if mask.sum() < 6:
        return np.eye(4)

    src_m   = src[mask]                    # (M, 3)
    tgt_m   = tgt[idx[mask]]              # (M, 3)
    normals = tgt_normals[idx[mask]]       # (M, 3)
    Cs      = src_covs[mask]              # (M, 3, 3)
    Ct      = tgt_covs[idx[mask]]         # (M, 3, 3)

    # ── Vectorized Jacobian ───────────────────────────────────────────────────
    # J[:, :3] = n @ skew(p),  J[:, 3:] = n
    # skew(p) = [[0, pz, -py], [-pz, 0, px], [py, -px, 0]]
    px, py, pz = src_m[:,0], src_m[:,1], src_m[:,2]
    zeros = np.zeros(len(src_m))
    # skew rows dotted with normal: n @ skew(p) = (M,3) x batched
    # Result shape (M, 3) for rotation part
    J_rot = np.stack([
         normals[:,1]*pz - normals[:,2]*py,   # n·skew col 0
        -normals[:,0]*pz + normals[:,2]*px,   # n·skew col 1
         normals[:,0]*py - normals[:,1]*px,   # n·skew col 2
    ], axis=1)                                # (M, 3)
    J = np.concatenate([J_rot, normals], axis=1)  # (M, 6)

    # ── Vectorized weights ────────────────────────────────────────────────────
    C    = Ct + Cs                                          # (M, 3, 3)
    nCn  = np.einsum('mi,mij,mj->m', normals, C, normals)  # (M,)
    w    = 1.0 / (nCn + 1e-6)                              # (M,)

    # ── Vectorized residuals ──────────────────────────────────────────────────
    d = np.einsum('mi,mi->m', normals, tgt_m - src_m)      # (M,)

    # ── Accumulate H and b ───────────────────────────────────────────────────
    wJ = w[:, None] * J                    # (M, 6)
    H  = J.T @ wJ                          # (6, 6)
    b  = wJ.T @ d                          # (6,)

    try:    xi = np.linalg.solve(H + 1e-6 * np.eye(6), b)
    except: return np.eye(4)

    wx, wy, wz = xi[:3]
    dT = np.eye(4)
    dT[:3, :3] = _clamp_so3(np.array([
        [  1, -wz,  wy],
        [ wz,   1, -wx],
        [-wy,  wx,   1],
    ]))
    dT[:3, 3] = xi[3:]
    return dT


def gicp_p2plane(src, tgt, init_T=None, voxel_size=0.005,
                 k_covs=20, max_iter=100, max_corr_dist=0.01):
    if init_T is None:
        init_T = np.eye(4)

    src_d = voxel_downsample(src, voxel_size)
    tgt_d = voxel_downsample(tgt, voxel_size)

    src_covs    = estimate_covariances(src_d, k=k_covs)
    tgt_covs    = estimate_covariances(tgt_d, k=k_covs)
    tgt_normals = estimate_normals(tgt_d, k=k_covs)
    tgt_tree    = KDTree(tgt_d)

    T, prev_rmse = init_T.copy(), np.inf
    for _ in range(max_iter):
        src_t      = apply_transform(src_d, T)
        src_covs_t = np.einsum("ij,njk,lk->nil", T[:3,:3], src_covs, T[:3,:3])
        dT         = gicp_step_p2plane(src_t, tgt_d, src_covs_t, tgt_covs,
                                       tgt_normals, tgt_tree, max_corr_dist)
        T = dT @ T

        dists, _ = tgt_tree.query(apply_transform(src_d, T), k=1)
        mask     = dists < max_corr_dist
        fitness  = float(mask.mean())
        rmse     = float(dists[mask].mean()) if mask.any() else np.inf
        if abs(prev_rmse - rmse) < 1e-7:
            break
        prev_rmse = rmse

    return T, fitness, rmse


# ═══════════════════════════════════════════════════════════════════════════════
#  PCA initial hint
# ═══════════════════════════════════════════════════════════════════════════════

def pca_axes(pts):
    c    = pts.mean(axis=0)
    diff = pts - c
    cov  = (diff.T @ diff) / len(pts)
    vals, vecs = np.linalg.eigh(cov)
    return c, vecs[:, np.argsort(vals)[::-1]].T


def pca_init_transform(src, tgt, max_rotation_deg=15.0):
    """
    PCA 6-DOF hint with automatic fallback to centroid-only.

    If any Euler angle in the best PCA candidate exceeds max_rotation_deg,
    the hint is considered a bad flip and discarded. A pure translation
    (centroid-to-centroid) is used instead, letting GICP solve rotation
    from a safe starting point.
    """
    src_d = voxel_downsample(src, max(0.005, np.ptp(src, axis=0).max() / 100))
    tgt_d = voxel_downsample(tgt, max(0.005, np.ptp(tgt, axis=0).max() / 100))

    c_src, V_src = pca_axes(src_d)
    c_tgt, V_tgt = pca_axes(tgt_d)
    tgt_tree = KDTree(tgt_d)

    best_T, best_dist = None, np.inf
    for signs in [(s0,s1,s2) for s0 in (1,-1) for s1 in (1,-1) for s2 in (1,-1)]:
        V_src_s = V_src * np.array(signs)[:, None]
        R = V_tgt.T @ V_src_s
        U, _, Vt = svd(R)
        R = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
        t = c_tgt - R @ c_src
        src_t = (R @ src_d.T).T + t
        mean_d = tgt_tree.query(src_t, k=1)[0].mean()
        if mean_d < best_dist:
            best_dist = mean_d
            best_T = np.eye(4)
            best_T[:3,:3] = R
            best_T[:3, 3] = t

    R, t = best_T[:3,:3], best_T[:3,3]
    sy    = np.sqrt(R[0,0]**2 + R[1,0]**2)
    roll  = np.degrees(np.arctan2( R[2,1], R[2,2]))
    pitch = np.degrees(np.arctan2(-R[2,0], sy))
    yaw   = np.degrees(np.arctan2( R[1,0], R[0,0]))
    print(f"    PCA hint  t=({t[0]:+.4f},{t[1]:+.4f},{t[2]:+.4f})  "
          f"roll={roll:+.2f}°  pitch={pitch:+.2f}°  yaw={yaw:+.2f}°  "
          f"mean-NN={best_dist:.5f}m")

    # ── Sanity check — reject bad flips ──────────────────────────────────────
    max_angle = max(abs(roll), abs(pitch), abs(yaw))
    if max_angle > max_rotation_deg:
        print(f"    !! PCA rotation unreasonable (worst={max_angle:.1f}° > "
              f"{max_rotation_deg:.0f}°) — falling back to centroid-only hint")
        t_fallback = tgt_d.mean(axis=0) - src_d.mean(axis=0)
        T_fallback = np.eye(4)
        T_fallback[:3, 3] = t_fallback
        print(f"    Centroid hint  t=({t_fallback[0]:+.4f},{t_fallback[1]:+.4f},"
              f"{t_fallback[2]:+.4f})  (translation only)")
        return T_fallback

    return best_T


# ═══════════════════════════════════════════════════════════════════════════════
#  Multi-scale cascade alignment
# ═══════════════════════════════════════════════════════════════════════════════

def align_pair(src_xyz, tgt_xyz, label,
               finest_voxel, finest_corr_dist,
               n_stages, k_covs, max_iter, overlap_frac):
    print(f"\n-- Aligning {label} " + "-" * max(0, 50 - len(label)))

    # src_xyz has already been pre-offset to its expected grid position,
    # so overlap trimming works correctly here.
    print("    Step 1 - trimming to overlap region ...")
    src_edge, tgt_edge, _ = trim_to_overlap(src_xyz, tgt_xyz,
                                             overlap_frac=overlap_frac)

    print("    Step 2 - PCA initial alignment ...")
    T = pca_init_transform(src_edge, tgt_edge)

    # ── Pre-flight check ──────────────────────────────────────────────────────
    coarsest_corr = finest_corr_dist * 2**(n_stages-1)
    src_hinted    = apply_transform(
        voxel_downsample(src_edge, finest_voxel * 2**(n_stages-1)), T)
    tgt_check     = voxel_downsample(tgt_edge, finest_voxel * 2**(n_stages-1))
    mean_nn       = KDTree(tgt_check).query(src_hinted, k=1, workers=-1)[0].mean()
    print(f"    Pre-flight mean-NN after hint: {mean_nn:.5f}m  "
          f"(coarsest corr_dist={coarsest_corr:.5f}m)")
    if mean_nn > coarsest_corr * 2:
        print(f"    !! mean-NN ({mean_nn:.4f}m) > 2x coarsest corr_dist — "
              f"try increasing --max-corr-dist or check --offset-x/y values")

    scales = [(finest_voxel * 2**i, finest_corr_dist * 2**i)
              for i in range(n_stages - 1, -1, -1)]

    consecutive_zero = 0
    for stage_idx, (vox, corr) in enumerate(scales):
        print(f"    Stage {stage_idx+1}/{n_stages}  "
              f"voxel={vox:.5f}m  corr_dist={corr:.5f}m ...")
        T, fitness, rmse = gicp_p2plane(
            src_edge, tgt_edge, init_T=T,
            voxel_size=vox, k_covs=k_covs,
            max_iter=max_iter, max_corr_dist=corr,
        )
        print(f"      fitness={fitness:.4f}  RMSE={rmse:.6f}m")

        if fitness == 0.0:
            consecutive_zero += 1
            print(f"      !! Zero fitness — try --max-corr-dist {corr*2:.3f}")
            if consecutive_zero >= 2:
                print("      !! Aborting — check --offset-x/y or increase --max-corr-dist")
                break
        else:
            consecutive_zero = 0

    return T


def _summarise(label, T):
    R, t  = T[:3,:3], T[:3,3]
    sy    = np.sqrt(R[0,0]**2 + R[1,0]**2)
    roll  = np.degrees(np.arctan2( R[2,1], R[2,2]))
    pitch = np.degrees(np.arctan2(-R[2,0], sy))
    yaw   = np.degrees(np.arctan2( R[1,0], R[0,0]))
    print(f"  {label}: t=({t[0]:+.4f},{t[1]:+.4f},{t[2]:+.4f})  "
          f"roll={roll:+.3f}°  pitch={pitch:+.3f}°  yaw={yaw:+.3f}°")


# ═══════════════════════════════════════════════════════════════════════════════
#  SOR (overlap-zone only)
# ═══════════════════════════════════════════════════════════════════════════════

def _overlap_mask(xyz, overlap_frac):
    spans = xyz.max(axis=0) - xyz.min(axis=0)
    axis  = int(np.argmax(spans))
    lo, hi = xyz[:, axis].min(), xyz[:, axis].max()
    margin = (1.0 - overlap_frac) / 2.0
    return (xyz[:, axis] >= lo + margin*(hi-lo)) & \
           (xyz[:, axis] <= hi - margin*(hi-lo))


def statistical_outlier_removal(xyz, extra, k=20, std_ratio=1.0, overlap_frac=0.0):
    if overlap_frac > 0:
        in_zone  = _overlap_mask(xyz, overlap_frac)
        out_zone = ~in_zone
    else:
        in_zone  = np.ones(len(xyz), dtype=bool)
        out_zone = np.zeros(len(xyz), dtype=bool)

    xyz_zone   = xyz[in_zone]
    extra_zone = {n: a[in_zone] for n, a in extra.items()}

    tree = KDTree(xyz_zone)
    dists, _ = tree.query(xyz_zone, k=k + 1)
    mean_dists = dists[:, 1:].mean(axis=1)
    threshold  = mean_dists.mean() + std_ratio * mean_dists.std()
    keep       = mean_dists <= threshold

    xyz_out = np.vstack([xyz[out_zone], xyz_zone[keep]])
    extra_out = {n: np.concatenate([a[out_zone], extra_zone[n][keep]])
                 for n, a in extra.items()}

    removed = in_zone.sum() - keep.sum()
    print(f"  SOR: removed {removed:,} points from overlap zone  "
          f"({100*removed/max(in_zone.sum(),1):.1f}%)  "
          f"remaining={len(xyz_out):,}")
    return xyz_out, extra_out


# ═══════════════════════════════════════════════════════════════════════════════
#  Merge helper
# ═══════════════════════════════════════════════════════════════════════════════

def merge_clouds(clouds):
    all_xyz   = np.vstack([c[0] for c in clouds])
    all_names = set()
    for _, extra in clouds:
        all_names.update(extra.keys())
    merged_extra = {}
    for name in all_names:
        parts = []
        for xyz, extra in clouds:
            if name in extra:
                parts.append(extra[name])
            else:
                ref   = next(iter(extra.values()), None)
                dtype = ref.dtype if ref is not None else np.float32
                parts.append(np.zeros(len(xyz), dtype=dtype))
        merged_extra[name] = np.concatenate(parts)
    return all_xyz, merged_extra


# ═══════════════════════════════════════════════════════════════════════════════
#  CLI
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Merge 4 blocks arranged in a 2x2 grid using point-to-plane GICP."
    )
    parser.add_argument("--b1", required=True, help="block1 (top-left anchor)")
    parser.add_argument("--b2", required=True, help="block2 (bottom-left, ~60cm below b1)")
    parser.add_argument("--b3", required=True, help="block3 (top-right, ~1m right of b1)")
    parser.add_argument("--b4", required=True, help="block4 (bottom-right, ~60cm below b3)")
    parser.add_argument("--output",        default="all_blocks_merged.pcd")
    parser.add_argument("--inspect",       action="store_true")
    parser.add_argument("--overlap",       type=float, default=0.60)
    parser.add_argument("--voxel",         type=float, default=0.003)
    parser.add_argument("--stages",        type=int,   default=5)
    parser.add_argument("--gicp-k",        type=int,   default=20)
    parser.add_argument("--max-iter",      type=int,   default=100)
    parser.add_argument("--max-corr-dist", type=float, default=0.010)
    parser.add_argument("--final-voxel",   type=float, default=0.0)
    parser.add_argument("--sor",           action="store_true")
    parser.add_argument("--sor-k",         type=int,   default=20)
    parser.add_argument("--sor-std",       type=float, default=1.0)
    # ── Per-block offsets (from pick_offset.py) ───────────────────────────────
    parser.add_argument("--off2-x", type=float, default=+0.131,
                        help="X offset to apply to block2 (default +0.131)")
    parser.add_argument("--off2-y", type=float, default=-0.032,
                        help="Y offset to apply to block2 (default -0.032)")
    parser.add_argument("--off2-z", type=float, default=+0.020,
                        help="Z offset to apply to block2 (default +0.020)")
    parser.add_argument("--off3-x", type=float, default=-0.050,
                        help="X offset to apply to block3 (default -0.050)")
    parser.add_argument("--off3-y", type=float, default=+1.591,
                        help="Y offset to apply to block3 (default +1.591)")
    parser.add_argument("--off3-z", type=float, default=+0.012,
                        help="Z offset to apply to block3 (default +0.012)")
    args = parser.parse_args()

    # ── Load ──────────────────────────────────────────────────────────────────
    print("\n-- Loading ---------------------------------------------------------")
    xyz1, extra1, hdr1 = load_pcd(args.b1)
    xyz2, extra2, _    = load_pcd(args.b2)
    xyz3, extra3, _    = load_pcd(args.b3)
    xyz4, extra4, _    = load_pcd(args.b4)

    # ── Inspect ───────────────────────────────────────────────────────────────
    if args.inspect:
        print("\n-- Bounding boxes -------------------------------------------------")
        c1 = inspect("block1 (anchor)", xyz1)
        c2 = inspect("block2",          xyz2)
        c3 = inspect("block3",          xyz3)
        c4 = inspect("block4",          xyz4)
        print("\n-- Centroid deltas ------------------------------------------------")
        for a, b, la, lb in [(c1,c2,"b1","b2"),(c1,c3,"b1","b3"),(c3,c4,"b3","b4")]:
            d   = b - a
            dom = 'XYZ'[np.argmax(np.abs(d))]
            print(f"  {lb}-{la}: dX={d[0]:+.4f} dY={d[1]:+.4f} dZ={d[2]:+.4f} m  "
                  f"(dominant: {dom})")
        return

    kw = dict(
        finest_voxel     = args.voxel,
        finest_corr_dist = args.max_corr_dist,
        n_stages         = args.stages,
        k_covs           = args.gicp_k,
        max_iter         = args.max_iter,
        overlap_frac     = args.overlap,
    )

    print(f"\n-- Pre-positioning blocks using measured offsets -------------------")
    print(f"   block2: dX={args.off2_x:+.4f}  dY={args.off2_y:+.4f}  dZ={args.off2_z:+.4f}")
    print(f"   block3: dX={args.off3_x:+.4f}  dY={args.off3_y:+.4f}  dZ={args.off3_z:+.4f}")
    print(f"   block4: dX={args.off3_x:+.4f}  dY={args.off2_y+args.off3_y:+.4f}  dZ={args.off3_z:+.4f}  (block3 offset + block2 Y)")

    def pre_offset(xyz, dx, dy, dz):
        out = xyz.copy()
        out[:, 0] += dx
        out[:, 1] += dy
        out[:, 2] += dz
        return out

    xyz2_pre = pre_offset(xyz2, args.off2_x, args.off2_y, args.off2_z)
    xyz3_pre = pre_offset(xyz3, args.off3_x, args.off3_y, args.off3_z)
    # block4 is below block3, so it gets block3's X/Z offset + block2's Y offset
    xyz4_pre = pre_offset(xyz4, args.off3_x, args.off2_y + args.off3_y, args.off3_z)

    # ── Step 1: block2 -> block1 ──────────────────────────────────────────────
    T2 = align_pair(xyz2_pre, xyz1, "block2 -> block1", **kw)
    xyz2_aligned = apply_transform(xyz2_pre, T2)
    step1_xyz, step1_extra = merge_clouds([(xyz1, extra1), (xyz2_aligned, extra2)])
    save_pcd("debug_step1_b1b2.pcd", step1_xyz, step1_extra, hdr1)

    # ── Step 2: block3 -> block1 ──────────────────────────────────────────────
    T3 = align_pair(xyz3_pre, xyz1, "block3 -> block1", **kw)
    xyz3_aligned = apply_transform(xyz3_pre, T3)
    step2_xyz, step2_extra = merge_clouds([(xyz1, extra1), (xyz2_aligned, extra2), (xyz3_aligned, extra3)])
    save_pcd("debug_step2_b1b2b3.pcd", step2_xyz, step2_extra, hdr1)

    # ── Step 3: block4 -> block3 ──────────────────────────────────────────────
    T4 = align_pair(xyz4_pre, xyz3_aligned, "block4 -> block3 (aligned)", **kw)
    xyz4_aligned = apply_transform(xyz4_pre, T4)

    # ── Merge all four ────────────────────────────────────────────────────────
    print("\n-- Merging all four blocks -----------------------------------------")
    merged_xyz, merged_extra = merge_clouds([
        (xyz1,         extra1),
        (xyz2_aligned, extra2),
        (xyz3_aligned, extra3),
        (xyz4_aligned, extra4),
    ])

    if args.final_voxel > 0:
        merged_xyz, merged_extra = voxel_downsample_with_extra(
            merged_xyz, merged_extra, args.final_voxel)

    if args.sor:
        print("\n-- Statistical Outlier Removal -------------------------------------")
        merged_xyz, merged_extra = statistical_outlier_removal(
            merged_xyz, merged_extra,
            k=args.sor_k, std_ratio=args.sor_std,
            overlap_frac=args.overlap)

    print(f"  Total points : {len(merged_xyz):,}")
    if merged_extra:
        print(f"  Fields       : {list(merged_extra.keys())}")

    save_pcd(args.output, merged_xyz, merged_extra, hdr1)

    print("\n-- Transforms ------------------------------------------------------")
    _summarise("block2 -> block1",          T2)
    _summarise("block3 -> block1",          T3)
    _summarise("block4 -> block3 (aligned)", T4)
    print("\nDone.")


if __name__ == "__main__":
    main()