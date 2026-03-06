"""
GICP Point Cloud Merger  —  pure numpy + scipy, no open3d required
===================================================================
Uses point-to-plane GICP with a multi-scale cascade (loose → tight)
for accurate alignment of scenes with thin geometry (pipes, etc).

── Normal 3-scan merge ───────────────────────────────────────────────────────
    python merge_pointclouds_gicp.py \
        --pc1 s1.pcd --pc2 s2.pcd --pc3 s3.pcd --output merged.pcd

── Debug: scan 1 + 2 only ────────────────────────────────────────────────────
    python merge_pointclouds_gicp.py \
        --pc1 s1.pcd --pc2 s2.pcd --pc3 s3.pcd \
        --output merged_12.pcd --two-only

── Append scan3 onto a pre-merged scan1+scan2 cloud ──────────────────────────
    python merge_pointclouds_gicp.py \
        --pc1 scan1andscan2merged.pcd --pc2 scan3.pcd --pc3 dummy.pcd \
        --output merged_all.pcd --two-only --pc1-far

    --pc1-far  tells the trimmer to cut the FAR end of pc1 (the scan2 side
               of the merged cloud) as the alignment target instead of the
               near/scan1 end that the centroid comparison would normally pick.
               scan3 (pc2) is still trimmed normally on its own near end.

── Inspect ───────────────────────────────────────────────────────────────────
    python merge_pointclouds_gicp.py \
        --pc1 s1.pcd --pc2 s2.pcd --pc3 s3.pcd --inspect

Tuning flags:
    --overlap      FLOAT   Overlap zone fraction         (default 0.60)
    --voxel        FLOAT   Finest voxel size, metres     (default 0.003)
    --stages       INT     Number of cascade stages      (default 5)
    --max-iter     INT     GICP iters per stage          (default 100)
    --max-corr-dist FLOAT  Finest corr distance, metres  (default 0.005)
    --final-voxel  FLOAT   Output voxel size, metres     (default 0=off)
    --two-only             Only merge scan1+scan2
    --pc1-far              Use far end of pc1 for overlap (see above)
"""

import argparse
import struct
from pathlib import Path

import numpy as np
from scipy.spatial import KDTree
from scipy.linalg import svd


# ═══════════════════════════════════════════════════════════════════════════════
#  PCD  I/O
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
    print(f"    fields : {fields}")
    return xyz, extra, hdr


def save_pcd(path: str, xyz: np.ndarray, extra: dict, orig_hdr: dict):
    n = len(xyz)
    fields = orig_hdr['fields']
    sizes  = orig_hdr['size']
    types  = orig_hdr['type']
    counts = orig_hdr.get('count', [1] * len(fields))

    cols, fmt_parts = [], []
    for name, tp, sz in zip(fields, types, sizes):
        if   name == 'x': cols.append(xyz[:, 0])
        elif name == 'y': cols.append(xyz[:, 1])
        elif name == 'z': cols.append(xyz[:, 2])
        elif name in extra:
            cols.append(extra[name].astype(np.float64 if tp == 'F' else np.int64))
        else:
            cols.append(np.zeros(n))
        fmt_parts.append('%.6f' if tp == 'F' else '%d')

    header_lines = [
        'VERSION 0.7',
        f'FIELDS {" ".join(fields)}',
        f'SIZE {" ".join(map(str, sizes))}',
        f'TYPE {" ".join(types)}',
        f'COUNT {" ".join(map(str, counts))}',
        f'WIDTH {n}', 'HEIGHT 1',
        'VIEWPOINT 0 0 0 1 0 0 0',
        f'POINTS {n}', 'DATA ascii',
    ]
    with open(path, 'w') as f:
        f.write('\n'.join(header_lines) + '\n')
        np.savetxt(f, np.column_stack(cols), fmt=fmt_parts)
    print(f"  Saved  {n:,} points  ->  {path}")


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
#  Normal estimation
# ═══════════════════════════════════════════════════════════════════════════════

def estimate_normals(pts: np.ndarray, k: int = 20) -> np.ndarray:
    k    = min(k, len(pts) - 1)
    tree = KDTree(pts)
    _, idx = tree.query(pts, k=k)
    normals = np.zeros_like(pts)
    for i, nbrs in enumerate(idx):
        nb   = pts[nbrs]
        diff = nb - nb.mean(axis=0)
        cov  = (diff.T @ diff) / max(k - 1, 1)
        vals, vecs = np.linalg.eigh(cov)
        normals[i] = vecs[:, 0]
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    norms = np.where(norms < 1e-10, 1.0, norms)
    return normals / norms


def estimate_covariances(pts: np.ndarray, k: int = 20) -> np.ndarray:
    k    = min(k, len(pts) - 1)
    tree = KDTree(pts)
    _, idx = tree.query(pts, k=k)
    covs = np.zeros((len(pts), 3, 3))
    for i, nbrs in enumerate(idx):
        nb   = pts[nbrs]
        diff = nb - nb.mean(axis=0)
        covs[i] = (diff.T @ diff) / max(k - 1, 1)
    return covs


# ═══════════════════════════════════════════════════════════════════════════════
#  Overlap trimming
# ═══════════════════════════════════════════════════════════════════════════════

def trim_to_overlap(src, tgt, overlap_frac=0.60, tgt_use_far_end=False):
    """
    Trim src and tgt to their mutual overlap zone.

    Default (tgt_use_far_end=False): centroid-based — keep the end of each
    cloud that faces the other. Correct for normal side-by-side scan pairs.

    tgt_use_far_end=True (--pc1-far): proximity-based — ignore centroids
    entirely and crop tgt to whichever end is spatially NEAREST to src.
    Robust for pre-merged clouds where the tgt centroid sits in the scan1
    half and the centroid comparison would pick the wrong end.
    """
    delta = np.abs(tgt.mean(axis=0) - src.mean(axis=0))
    axis  = int(np.argmax(delta))
    axis_name = 'XYZ'[axis]
    print(f"    Overlap axis: {axis_name}  "
          f"(dX={delta[0]:+.4f} dY={delta[1]:+.4f} dZ={delta[2]:+.4f})")

    src_min, src_max = src[:, axis].min(), src[:, axis].max()
    tgt_min, tgt_max = tgt[:, axis].min(), tgt[:, axis].max()
    src_centre = (src_min + src_max) / 2.0

    if tgt_use_far_end:
        # Split tgt at its midpoint along the dominant axis.
        # Keep whichever half has the lower mean NN distance to src.
        # This is the only reliable way to isolate the scan2 region of a
        # pre-merged cloud — verified by diagnostic against real data.
        tgt_mid = (tgt_min + tgt_max) / 2.0
        low_half  = tgt[tgt[:, axis] <= tgt_mid]
        high_half = tgt[tgt[:, axis] >  tgt_mid]

        from scipy.spatial import KDTree as _KDTree
        sample = src[::50]
        d_low,  _ = _KDTree(low_half[::10]).query(sample,  k=1)
        d_high, _ = _KDTree(high_half[::10]).query(sample, k=1)

        if d_high.mean() < d_low.mean():
            chosen = high_half
            chosen_min = tgt_mid
            chosen_max = tgt_max
            print(f"    --pc1-far: HIGH half closer "
                  f"(NN {d_high.mean():.4f}m vs {d_low.mean():.4f}m)")
        else:
            chosen = low_half
            chosen_min = tgt_min
            chosen_max = tgt_mid
            print(f"    --pc1-far: LOW half closer "
                  f"(NN {d_low.mean():.4f}m vs {d_high.mean():.4f}m)")

        # Trim further: keep only the overlap_frac of the chosen half that is
        # at its OUTER edge (away from the midpoint / scan1 boundary).
        # HIGH half → keep the top (max) portion  — pure scan2 far edge
        # LOW  half → keep the bottom (min) portion — pure scan2 far edge
        half_span = chosen_max - chosen_min
        if chosen_min == tgt_mid:
            # This is the HIGH half — keep its top (max) end
            trim_lo = chosen_max - overlap_frac * half_span
            tgt_edge = chosen[chosen[:, axis] >= trim_lo]
            print(f"    --pc1-far: HIGH half, keeping outer {overlap_frac:.0%} "
                  f"({axis_name} >= {trim_lo:.4f})")
        else:
            # This is the LOW half — keep its bottom (min) end
            trim_hi = chosen_min + overlap_frac * half_span
            tgt_edge = chosen[chosen[:, axis] <= trim_hi]
            print(f"    --pc1-far: LOW half, keeping outer {overlap_frac:.0%} "
                  f"({axis_name} <= {trim_hi:.4f})")

        # src: use the full scan3 cloud untrimmed.
        # scan3 is already small (~4.5M pts) and already sits in the correct
        # region. Any trim risks cutting away the points that actually overlap
        # with tgt_edge, which kills GICP correspondences entirely.
        src_edge = src

    else:
        # Default centroid-based trimming
        src_is_left = src[:, axis].mean() < tgt[:, axis].mean()
        if src_is_left:
            src_edge = src[src[:, axis] >= src_max - overlap_frac * (src_max - src_min)]
            tgt_edge = tgt[tgt[:, axis] <= tgt_min + overlap_frac * (tgt_max - tgt_min)]
        else:
            src_edge = src[src[:, axis] <= src_min + overlap_frac * (src_max - src_min)]
            tgt_edge = tgt[tgt[:, axis] >= tgt_max - overlap_frac * (tgt_max - tgt_min)]

    print(f"    Trimmed: src {len(src):,}->{len(src_edge):,}  "
          f"tgt {len(tgt):,}->{len(tgt_edge):,}  (frac={overlap_frac:.2f})")
    if len(src_edge) < 50 or len(tgt_edge) < 50:
        print("    !! Too few points — increase --overlap")
    return src_edge, tgt_edge, axis


# ═══════════════════════════════════════════════════════════════════════════════
#  SO(3) helpers
# ═══════════════════════════════════════════════════════════════════════════════

def _clamp_so3(R):
    U, _, Vt = svd(R)
    return U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt


def apply_transform(pts, T):
    return (T[:3, :3] @ pts.T).T + T[:3, 3]


# ═══════════════════════════════════════════════════════════════════════════════
#  Point-to-plane GICP step
# ═══════════════════════════════════════════════════════════════════════════════

def gicp_step_p2plane(src, tgt, src_covs, tgt_covs,
                      tgt_normals, tgt_tree, max_corr_dist,
                      lock_roll_pitch=False):
    dists, idx = tgt_tree.query(src, k=1, workers=-1)
    mask = dists < max_corr_dist
    if mask.sum() < 6:
        return np.eye(4)

    src_m   = src[mask]
    tgt_m   = tgt[idx[mask]]
    normals = tgt_normals[idx[mask]]
    Cs      = src_covs[mask]
    Ct      = tgt_covs[idx[mask]]

    H, b = np.zeros((6, 6)), np.zeros(6)

    for i in range(len(src_m)):
        n = normals[i]
        r = tgt_m[i] - src_m[i]
        d = float(n @ r)
        C   = Ct[i] + Cs[i]
        var = float(n @ C @ n) + 1e-6
        w   = 1.0 / var
        p = src_m[i]
        J = np.zeros(6)
        J[:3] = n @ np.array([
            [    0,  p[2], -p[1]],
            [-p[2],     0,  p[0]],
            [ p[1], -p[0],     0],
        ])
        J[3:] = n
        H += w * np.outer(J, J)
        b += w * d * J

    try:    xi = np.linalg.solve(H + 1e-6 * np.eye(6), b)
    except: return np.eye(4)

    wx, wy, wz = xi[:3]

    if lock_roll_pitch:
        # Zero out roll (wx) and pitch (wy) — only allow yaw (wz) + translation
        wx, wy = 0.0, 0.0

    dT = np.eye(4)
    dT[:3, :3] = _clamp_so3(np.array([
        [  1, -wz,  wy],
        [ wz,   1, -wx],
        [-wy,  wx,   1],
    ]))
    dT[:3, 3] = xi[3:]
    return dT


def gicp_p2plane(src, tgt, init_T=None, voxel_size=0.005,
                 k_covs=20, max_iter=100, max_corr_dist=0.01,
                 lock_roll_pitch=False):
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
                                       tgt_normals, tgt_tree, max_corr_dist,
                                       lock_roll_pitch=lock_roll_pitch)
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


def centroid_init_transform(src, tgt):
    t = tgt.mean(axis=0) - src.mean(axis=0)
    T = np.eye(4)
    T[:3, 3] = t
    print(f"    Centroid hint  t=({t[0]:+.4f},{t[1]:+.4f},{t[2]:+.4f})  "
          f"(translation only, no rotation assumed)")
    return T


def _rotation_to_euler(R):
    sy    = np.sqrt(R[0,0]**2 + R[1,0]**2)
    roll  = np.degrees(np.arctan2( R[2,1], R[2,2]))
    pitch = np.degrees(np.arctan2(-R[2,0], sy))
    yaw   = np.degrees(np.arctan2( R[1,0], R[0,0]))
    return roll, pitch, yaw


def pca_init_transform(src, tgt, max_rotation_deg=15.0):
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
    roll, pitch, yaw = _rotation_to_euler(R)
    print(f"    PCA hint  t=({t[0]:+.4f},{t[1]:+.4f},{t[2]:+.4f})  "
          f"roll={roll:+.2f}°  pitch={pitch:+.2f}°  yaw={yaw:+.2f}°  "
          f"mean-NN={best_dist:.5f}m")

    max_angle = max(abs(roll), abs(pitch), abs(yaw))
    if max_angle > max_rotation_deg:
        print(f"    !! PCA rotation unreasonable (worst axis={max_angle:.1f}° > "
              f"{max_rotation_deg:.0f}°) — falling back to centroid-only hint")
        return centroid_init_transform(src_d, tgt_d)

    return best_T


# ═══════════════════════════════════════════════════════════════════════════════
#  Multi-scale cascade
# ═══════════════════════════════════════════════════════════════════════════════

def align_pair(src_xyz, tgt_xyz,
               finest_voxel, finest_corr_dist,
               n_stages, k_covs, max_iter, overlap_frac,
               tgt_use_far_end=False, init_t=None,
               lock_roll_pitch=False):
    print("    Step 1 - trimming to overlap region ...")
    src_edge, tgt_edge, _ = trim_to_overlap(
        src_xyz, tgt_xyz,
        overlap_frac=overlap_frac,
        tgt_use_far_end=tgt_use_far_end,
    )

    print("    Step 2 - initial alignment ...")
    if tgt_use_far_end and init_t is not None:
        print(f"      (--pc1-far: using provided initial translation hint)")
        T = np.eye(4)
        T[:3, 3] = init_t
    elif tgt_use_far_end:
        print("      (--pc1-far: using identity init)")
        T = np.eye(4)
    else:
        T = pca_init_transform(src_edge, tgt_edge)

    scales = []
    for i in range(n_stages - 1, -1, -1):
        factor = 2 ** i
        scales.append((finest_voxel * factor, finest_corr_dist * factor))

    for stage_idx, (vox, corr) in enumerate(scales):
        print(f"    Stage {stage_idx+1}/{n_stages}  "
              f"voxel={vox:.5f}m  corr_dist={corr:.5f}m ...")
        T, fitness, rmse = gicp_p2plane(
            src_edge, tgt_edge,
            init_T=T,
            voxel_size=vox,
            k_covs=k_covs,
            max_iter=max_iter,
            max_corr_dist=corr,
            lock_roll_pitch=lock_roll_pitch,
        )
        print(f"      fitness={fitness:.4f}  RMSE={rmse:.6f}m")
        if fitness < 0.10:
            print(f"      !! Very low fitness at this scale — "
                  f"try --overlap 0.70 or --max-corr-dist larger")

    return T, fitness, rmse


def _summarise(label, T):
    R, t  = T[:3,:3], T[:3,3]
    sy    = np.sqrt(R[0,0]**2 + R[1,0]**2)
    roll  = np.degrees(np.arctan2( R[2,1], R[2,2]))
    pitch = np.degrees(np.arctan2(-R[2,0], sy))
    yaw   = np.degrees(np.arctan2( R[1,0], R[0,0]))
    print(f"\n  {label}")
    print(f"    t : dX={t[0]:+.6f}  dY={t[1]:+.6f}  dZ={t[2]:+.6f}  m")
    print(f"    R : roll={roll:+.4f}°  pitch={pitch:+.4f}°  yaw={yaw:+.4f}°")


# ═══════════════════════════════════════════════════════════════════════════════
#  Post-processing
# ═══════════════════════════════════════════════════════════════════════════════

def _overlap_mask(xyz: np.ndarray, overlap_frac: float) -> np.ndarray:
    spans = xyz.max(axis=0) - xyz.min(axis=0)
    axis  = int(np.argmax(spans))
    lo    = xyz[:, axis].min()
    hi    = xyz[:, axis].max()
    span  = hi - lo
    margin = (1.0 - overlap_frac) / 2.0
    band_lo = lo + margin * span
    band_hi = hi - margin * span
    return (xyz[:, axis] >= band_lo) & (xyz[:, axis] <= band_hi)


def statistical_outlier_removal(xyz: np.ndarray, extra: dict,
                                 k: int = 20, std_ratio: float = 1.0,
                                 overlap_frac: float = 0.0):
    if overlap_frac > 0:
        in_zone = _overlap_mask(xyz, overlap_frac)
        out_zone = ~in_zone
        xyz_zone   = xyz[in_zone]
        extra_zone = {n: a[in_zone] for n, a in extra.items()}
    else:
        in_zone  = np.ones(len(xyz), dtype=bool)
        out_zone = np.zeros(len(xyz), dtype=bool)
        xyz_zone   = xyz
        extra_zone = extra

    tree = KDTree(xyz_zone)
    dists, _ = tree.query(xyz_zone, k=k + 1)
    mean_dists = dists[:, 1:].mean(axis=1)
    threshold  = mean_dists.mean() + std_ratio * mean_dists.std()
    keep_zone  = mean_dists <= threshold

    xyz_out = np.vstack([xyz[out_zone], xyz_zone[keep_zone]])
    extra_out = {}
    for name, arr in extra.items():
        extra_out[name] = np.concatenate([arr[out_zone], extra_zone[name][keep_zone]])

    removed = in_zone.sum() - keep_zone.sum()
    print(f"  SOR: removed {removed:,} outlier points from overlap zone  "
          f"({100*removed/max(in_zone.sum(),1):.1f}% of zone)  "
          f"threshold={threshold:.5f}m  total remaining={len(xyz_out):,}")
    return xyz_out, extra_out


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
        description="Merge .pcd point clouds — point-to-plane GICP with "
                    "multi-scale cascade (numpy+scipy only)."
    )
    parser.add_argument("--pc1",    required=True)
    parser.add_argument("--pc2",    required=True)
    parser.add_argument("--pc3",    required=True)
    parser.add_argument("--output", default="merged.pcd")
    parser.add_argument("--inspect",  action="store_true")
    parser.add_argument("--two-only", action="store_true")
    parser.add_argument("--overlap",       type=float, default=0.60)
    parser.add_argument("--voxel",         type=float, default=0.003)
    parser.add_argument("--stages",        type=int,   default=5)
    parser.add_argument("--gicp-k",        type=int,   default=20)
    parser.add_argument("--max-iter",      type=int,   default=100)
    parser.add_argument("--max-corr-dist", type=float, default=0.005)
    parser.add_argument("--final-voxel",   type=float, default=0.0)
    parser.add_argument("--sor",     action="store_true")
    parser.add_argument("--sor-k",   type=int,   default=20)
    parser.add_argument("--sor-std", type=float, default=1.0)
    parser.add_argument(
        "--lock-roll-pitch", action="store_true",
        help="Lock roll and pitch during GICP — only allow yaw + translation. "
             "Use when the floor alignment breaks due to GICP over-rotating."
    )
    parser.add_argument(
        "--init-t", type=float, nargs=3, metavar=('DX','DY','DZ'), default=None,
        help="Initial translation hint for --pc1-far, in metres. "
             "Pick one point on the same object in scan3 and merged_12 in CloudCompare "
             "and pass the difference: --init-t DX DY DZ"
    )
    parser.add_argument(
        "--pc1-far", action="store_true",
        help=(
            "Use the far end of pc1 as the overlap target instead of the near end. "
            "Use when pc1 is a pre-merged scan1+scan2 cloud and pc2 is scan3 — "
            "makes scan3 align to the scan2 end of pc1, not the scan1 end. "
            "Example: --pc1 scan1andscan2merged.pcd --pc2 scan3.pcd --pc3 dummy.pcd "
            "--two-only --pc1-far"
        )
    )

    args = parser.parse_args()

    print("\n-- Loading ---------------------------------------------------------")
    xyz1, extra1, hdr1 = load_pcd(args.pc1)
    xyz2, extra2, _    = load_pcd(args.pc2)
    if not args.two_only:
        xyz3, extra3, _ = load_pcd(args.pc3)

    if args.inspect:
        print("\n-- Bounding boxes -------------------------------------------------")
        c1 = inspect("scan1", xyz1)
        c2 = inspect("scan2", xyz2)
        if not args.two_only:
            c3 = inspect("scan3", xyz3)
        print("\n-- Centroid deltas ------------------------------------------------")
        pairs = [(c1,c2,"scan1","scan2")]
        if not args.two_only:
            pairs.append((c2,c3,"scan2","scan3"))
        for a,b,la,lb in pairs:
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

    print("\n-- Aligning pc2 -> pc1 ---------------------------------------------")
    init_t = np.array(args.init_t) if args.init_t else None
    T2, _, _ = align_pair(xyz2, xyz1, tgt_use_far_end=args.pc1_far, init_t=init_t,
                          lock_roll_pitch=args.lock_roll_pitch, **kw)
    xyz2_aligned = apply_transform(xyz2, T2)

    if args.two_only:
        print("\n-- Merging pc1 + pc2 ----------------------------------------------")
        merged_xyz, merged_extra = merge_clouds([
            (xyz1,         extra1),
            (xyz2_aligned, extra2),
        ])
        if args.final_voxel > 0:
            merged_xyz, merged_extra = voxel_downsample_with_extra(
                merged_xyz, merged_extra, args.final_voxel)
        if args.sor:
            print("\n-- Statistical Outlier Removal ----------------------------")
            merged_xyz, merged_extra = statistical_outlier_removal(
                merged_xyz, merged_extra, k=args.sor_k, std_ratio=args.sor_std,
                overlap_frac=args.overlap)
        print(f"  Points: {len(merged_xyz):,}")
        save_pcd(args.output, merged_xyz, merged_extra, hdr1)
        _summarise("T pc2->pc1", T2)
        print("\nDone (--two-only).")
        return

    print("\n-- Aligning scan 3 -> scan 2 ---------------------------------------")
    T3, _, _ = align_pair(xyz3, xyz2_aligned, lock_roll_pitch=args.lock_roll_pitch, **kw)
    xyz3_aligned = apply_transform(xyz3, T3)

    print("\n-- Merging all three -----------------------------------------------")
    merged_xyz, merged_extra = merge_clouds([
        (xyz1,         extra1),
        (xyz2_aligned, extra2),
        (xyz3_aligned, extra3),
    ])
    if args.final_voxel > 0:
        merged_xyz, merged_extra = voxel_downsample_with_extra(
            merged_xyz, merged_extra, args.final_voxel)
    if args.sor:
        print("\n-- Statistical Outlier Removal -------------------------------------")
        merged_xyz, merged_extra = statistical_outlier_removal(
            merged_xyz, merged_extra, k=args.sor_k, std_ratio=args.sor_std,
            overlap_frac=args.overlap)

    print(f"  Points: {len(merged_xyz):,}")
    if merged_extra:
        print(f"  Fields: {list(merged_extra.keys())}")
    save_pcd(args.output, merged_xyz, merged_extra, hdr1)

    _summarise("T scan2->scan1",  T2)
    _summarise("T scan3->merged", T3)
    print("\nDone.")


if __name__ == "__main__":
    main()