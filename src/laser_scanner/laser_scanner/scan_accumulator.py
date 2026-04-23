"""
scan_accumulator.py — Python port of scan_accumulator.cpp

Subscribes to /scanner/single_scan (PointCloud2), accumulates strips via
TF transform + small_gicp GICP refinement, and exposes:
  /scanner/save_merged_cloud  (Trigger) — save posN.pcd
  /scanner/clear_scans        (Empty)   — reset cache
  /scanner/get_scan_count     (Trigger) — report count

Parameters:
  fixed_frame                      (str,   default "base_link")
  output_directory                 (str,   default ~/Dobot_Robot_Arm/scans)
  position_prefix                  (str,   default "pos1")
  auto_publish_interval_ms         (int,   default 1000)
  gicp_max_correspondence_distance (float, default 0.10)
  gicp_max_iterations              (int,   default 200)
  gicp_downsampling_resolution     (float, default 0.005)
  gicp_num_threads                 (int,   default 4)
  floor_band_m                     (float, default 0.020)
"""

import os
import threading
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger, Empty as EmptySrv
import sensor_msgs_py.point_cloud2 as pc2

# GICP thresholds (mirrors C++ defaults)
_AUTO_PUBLISH_INTERVAL_MS = 1000
_GICP_MAX_CORR            = 0.10
_GICP_MAX_ITER            = 200
_GICP_VOXEL               = 0.005
_GICP_THREADS             = 4
_FLOOR_BAND_M             = 0.020
_FLOOR_PERCENTILE         = 0.02
_MAX_GICP_TRANS_M         = 0.15
_MAX_GICP_ROT_DEG         = 10.0


# ── helpers ──────────────────────────────────────────────────────────────────

def _detect_floor_z(pts: np.ndarray, percentile: float) -> float:
    """Return bottom-percentile Z value as the floor reference."""
    z_sorted = np.sort(pts[:, 2])
    idx = int(percentile * len(z_sorted))
    return float(z_sorted[max(0, min(idx, len(z_sorted) - 1))])


def _remove_floor(pts: np.ndarray, floor_z: float, band: float) -> np.ndarray:
    """Return points whose |Z - floor_z| > band (keep structure, remove floor)."""
    mask = np.abs(pts[:, 2] - floor_z) > band
    return pts[mask]


def _ros_msg_to_numpy(msg: PointCloud2) -> np.ndarray:
    """Convert PointCloud2 to Nx3 float32 array; drops NaN / zero-Z points."""
    gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    pts = np.array([[p[0], p[1], p[2]] for p in gen], dtype=np.float32)
    if pts.size == 0:
        return pts.reshape(0, 3)
    valid = (pts[:, 2] != 0.0) & (pts[:, 2] > -0.999)
    return pts[valid]


def _numpy_to_o3d(pts: np.ndarray) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts[:, :3].astype(np.float64))
    return pcd


def _o3d_to_numpy(pcd: o3d.geometry.PointCloud) -> np.ndarray:
    return np.asarray(pcd.points, dtype=np.float32)


def _gicp_align(source_pts: np.ndarray, target_pts: np.ndarray,
                voxel: float, max_corr: float, max_iter: int) -> np.ndarray | None:
    """
    Run generalised-ICP (source → target).
    Returns 4×4 correction matrix on success, None on failure / divergence.
    Both inputs are Nx3 float arrays (structure-only, floor removed).
    """
    src_pcd = _numpy_to_o3d(source_pts)
    tgt_pcd = _numpy_to_o3d(target_pts)

    # Downsample
    src_ds = src_pcd.voxel_down_sample(voxel)
    tgt_ds = tgt_pcd.voxel_down_sample(voxel)

    if len(src_ds.points) < 100 or len(tgt_ds.points) < 100:
        return None

    src_ds.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 4, max_nn=20))
    tgt_ds.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 4, max_nn=20))

    result = o3d.pipelines.registration.registration_generalized_icp(
        src_ds, tgt_ds,
        max_correspondence_distance=max_corr,
        init=np.eye(4),
        estimation_method=o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter),
    )

    T = result.transformation              # 4×4 ndarray
    dt = np.linalg.norm(T[:3, 3])
    # Angle from rotation part
    cos_theta = (np.trace(T[:3, :3]) - 1.0) / 2.0
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    dr_deg = np.degrees(np.arccos(cos_theta))

    if result.fitness > 0.0 and dt < _MAX_GICP_TRANS_M and dr_deg < _MAX_GICP_ROT_DEG:
        return T
    return None


# ── node ─────────────────────────────────────────────────────────────────────

class ScanAccumulatorNode(Node):
    def __init__(self):
        super().__init__("scan_accumulator")

        # ── parameters ──────────────────────────────────────────────────────
        self.declare_parameter("fixed_frame",                      "base_link")
        self.declare_parameter("output_directory",
                               os.path.join(os.environ["HOME"], "Dobot_Robot_Arm", "scans"))
        self.declare_parameter("position_prefix",                  "pos1")
        self.declare_parameter("auto_publish_interval_ms",         _AUTO_PUBLISH_INTERVAL_MS)
        self.declare_parameter("gicp_max_correspondence_distance", _GICP_MAX_CORR)
        self.declare_parameter("gicp_max_iterations",              _GICP_MAX_ITER)
        self.declare_parameter("gicp_downsampling_resolution",     _GICP_VOXEL)
        self.declare_parameter("gicp_num_threads",                 _GICP_THREADS)
        self.declare_parameter("floor_band_m",                     _FLOOR_BAND_M)

        self._fixed_frame   = self.get_parameter("fixed_frame").value
        self._output_dir    = self.get_parameter("output_directory").value
        self._prefix        = self.get_parameter("position_prefix").value
        self._gicp_max_corr = self.get_parameter("gicp_max_correspondence_distance").value
        self._gicp_max_iter = self.get_parameter("gicp_max_iterations").value
        self._gicp_voxel    = self.get_parameter("gicp_downsampling_resolution").value
        self._floor_band    = self.get_parameter("floor_band_m").value

        os.makedirs(self._output_dir, exist_ok=True)

        # ── state ────────────────────────────────────────────────────────────
        self._lock               = threading.Lock()
        self._accumulated: list[np.ndarray] = []   # list of Nx3 arrays in fixed_frame
        self._scan_count         = 0
        self._reference_floor_z  = None
        self._accumulated_pts    = np.zeros((0, 3), dtype=np.float32)

        # ── TF ───────────────────────────────────────────────────────────────
        from tf2_ros import Buffer, TransformListener
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── pub/sub/srv ──────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            PointCloud2, "/scanner/single_scan", self._scan_cb, 10)
        self._pub = self.create_publisher(PointCloud2, "/scanner/merged_cloud", 10)

        self.create_service(Trigger,   "/scanner/save_merged_cloud", self._save_cb)
        self.create_service(EmptySrv,  "/scanner/clear_scans",       self._clear_cb)
        self.create_service(Trigger,   "/scanner/get_scan_count",    self._count_cb)

        pub_ms = self.get_parameter("auto_publish_interval_ms").value
        self.create_timer(pub_ms / 1000.0, self._publish_merged)

        self.get_logger().info(
            f"[ACCUMULATOR] Ready — fixed_frame={self._fixed_frame}  "
            f"prefix={self._prefix}  corr={self._gicp_max_corr:.3f}m  "
            f"floor_band=±{self._floor_band * 1000:.1f}mm")

    # ── TF lookup ────────────────────────────────────────────────────────────

    def _lookup_tf(self, stamp) -> np.ndarray:
        """Return 4×4 transform (laser_frame → fixed_frame) as float32 ndarray."""
        from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
        import tf_transformations
        try:
            ts = self._tf_buffer.lookup_transform(
                self._fixed_frame, "laser_frame", stamp,
                timeout=rclpy.duration.Duration(seconds=0.5))
            t  = ts.transform.translation
            q  = ts.transform.rotation
            T  = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
            T[:3, 3] = [t.x, t.y, t.z]
            return T.astype(np.float32), True
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return np.eye(4, dtype=np.float32), False

    # ── main callback ────────────────────────────────────────────────────────

    def _scan_cb(self, msg: PointCloud2):
        raw = _ros_msg_to_numpy(msg)
        if raw.shape[0] == 0:
            self.get_logger().warn("Empty scan -- skipping")
            return

        # TF: laser_frame → fixed_frame
        T, have_tf = self._lookup_tf(msg.header.stamp)
        # Apply transform (homogeneous)
        ones = np.ones((raw.shape[0], 1), dtype=np.float32)
        pts_h = np.hstack([raw[:, :3], ones])        # Nx4
        scan_in_fixed = (T @ pts_h.T).T[:, :3]      # Nx3

        t_xyz = T[:3, 3]
        self.get_logger().info(
            f"[ACCUMULATOR] Scan #{self._scan_count + 1}: {raw.shape[0]} pts  "
            f"TF {'OK' if have_tf else 'IDENTITY-FALLBACK'}  "
            f"laser→base: ({t_xyz[0]:.3f}, {t_xyz[1]:.3f}, {t_xyz[2]:.3f}) m")

        with self._lock:
            # ── Floor detection ──────────────────────────────────────────────
            if self._reference_floor_z is None:
                self._reference_floor_z = _detect_floor_z(scan_in_fixed, _FLOOR_PERCENTILE)
                self.get_logger().info(
                    f"[ACCUMULATOR] Floor reference: floor_z={self._reference_floor_z:.4f}m")

            floor_z = self._reference_floor_z

            # Re-read prefix (map_merger may have changed it)
            self._prefix = self.get_parameter("position_prefix").value

            # Save individual strip PCD in block folder
            block_folder = os.path.join(self._output_dir, f"block{self._scan_count + 1}")
            os.makedirs(block_folder, exist_ok=True)
            scan_idx = self._scan_count + 1
            ind_path = os.path.join(block_folder, f"block{self._scan_count + 1}_scan{scan_idx}.pcd")
            pcd_ind = _numpy_to_o3d(scan_in_fixed)
            o3d.io.write_point_cloud(ind_path, pcd_ind)
            self.get_logger().info(
                f"[ACCUMULATOR] Saved scan #{scan_idx} → {ind_path} ({scan_in_fixed.shape[0]} pts)")

            # ── First scan: store directly ───────────────────────────────────
            if self._scan_count == 0 or self._accumulated_pts.shape[0] == 0:
                self._accumulated_pts = scan_in_fixed
                self._scan_count += 1
                self.get_logger().info(
                    f"[ACCUMULATOR] Scan #1 stored → Total: {self._accumulated_pts.shape[0]} pts")
                return

            # ── GICP refinement (identity init: points already in world coords) ─
            map_struct  = _remove_floor(self._accumulated_pts, floor_z, self._floor_band)
            scan_struct = _remove_floor(scan_in_fixed,         floor_z, self._floor_band)

            if map_struct.shape[0] < 100 or scan_struct.shape[0] < 100:
                self._accumulated_pts = np.vstack([self._accumulated_pts, scan_in_fixed])
                self._scan_count += 1
                self.get_logger().warn(
                    f"[ACCUMULATOR] Scan #{self._scan_count}: too few structure pts — TF-only → "
                    f"Total: {self._accumulated_pts.shape[0]} pts")
                return

            T_corr = _gicp_align(scan_struct, map_struct,
                                  self._gicp_voxel, self._gicp_max_corr, self._gicp_max_iter)

            if T_corr is not None:
                ones2 = np.ones((scan_in_fixed.shape[0], 1), dtype=np.float32)
                pts_h2 = np.hstack([scan_in_fixed, ones2])
                aligned = (T_corr @ pts_h2.T).T[:, :3].astype(np.float32)
                self._accumulated_pts = np.vstack([self._accumulated_pts, aligned])
                dt = np.linalg.norm(T_corr[:3, 3])
                self.get_logger().info(
                    f"[ACCUMULATOR] Scan #{self._scan_count + 1}: GICP OK  dt={dt:.4f}m → "
                    f"Total: {self._accumulated_pts.shape[0]} pts")
            else:
                self._accumulated_pts = np.vstack([self._accumulated_pts, scan_in_fixed])
                self.get_logger().warn(
                    f"[ACCUMULATOR] Scan #{self._scan_count + 1}: GICP FAILED — TF-only → "
                    f"Total: {self._accumulated_pts.shape[0]} pts")

            # Save merged block after 3 scans
            if self._scan_count == 2:  # After 3rd scan (0-based count)
                merged_path = os.path.join(block_folder, f"block{self._scan_count + 1}.pcd")
                pcd_merged = _numpy_to_o3d(self._accumulated_pts)
                o3d.io.write_point_cloud(merged_path, pcd_merged)
                self.get_logger().info(
                    f"[ACCUMULATOR] Saved merged block → {merged_path} ({self._accumulated_pts.shape[0]} pts)")

            self._scan_count += 1

    # ── periodic publish ──────────────────────────────────────────────────────

    def _publish_merged(self):
        with self._lock:
            if self._accumulated_pts.shape[0] == 0:
                return
            pcd = _numpy_to_o3d(self._accumulated_pts)
        msg = self._pcd_to_ros(pcd, self._fixed_frame)
        self._pub.publish(msg)

    # ── services ─────────────────────────────────────────────────────────────

    def _save_cb(self, req, resp):
        with self._lock:
            if self._accumulated_pts.shape[0] == 0:
                resp.success = False
                resp.message = "No scans accumulated yet"
                return resp
            prefix = self.get_parameter("position_prefix").value
            path   = os.path.join(self._output_dir, f"{prefix}.pcd")
            pcd = _numpy_to_o3d(self._accumulated_pts)
            o3d.io.write_point_cloud(path, pcd)
            n = self._accumulated_pts.shape[0]
        resp.success = True
        resp.message = f"Saved {n} points to {path}"
        self.get_logger().info(f"[ACCUMULATOR] {resp.message}")
        return resp

    def _clear_cb(self, req, resp):
        with self._lock:
            self._accumulated_pts   = np.zeros((0, 3), dtype=np.float32)
            self._scan_count        = 0
            self._reference_floor_z = None
        self.get_logger().info("[ACCUMULATOR] Cache cleared")
        return resp

    def _count_cb(self, req, resp):
        with self._lock:
            n_scans = self._scan_count
            n_pts   = self._accumulated_pts.shape[0]
        resp.success = True
        resp.message = f"Scans: {n_scans}, Points: {n_pts}"
        return resp

    # ── utility ───────────────────────────────────────────────────────────────

    def _pcd_to_ros(self, pcd: o3d.geometry.PointCloud, frame: str) -> PointCloud2:
        pts = np.asarray(pcd.points, dtype=np.float32)
        from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
        from std_msgs.msg import Header
        header = Header()
        header.frame_id = frame
        header.stamp    = self.get_clock().now().to_msg()
        return create_cloud_xyz32(header, pts.tolist())
