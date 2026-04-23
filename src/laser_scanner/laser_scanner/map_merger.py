"""
map_merger.py

Accumulates 3-strip blocks (published on /scanner/merged_cloud) into a
growing world map.

Strategy (mirrors merge_scans.py):
  - Block #1: placed at origin, stored as world map reference.
  - Block #N: initial guess comes from the rover's TF transform
    (odom → base_link) at the time add_block is called.  This replaces the
    old hardcoded positions_x/y/yaw arrays.  GICP then refines on top of
    that TF-based initial guess, exactly like scan_accumulator does for
    individual strips.

TF source priority (set via parameter `odom_frame`):
  1. odom → base_link   (wheel odometry, usually best)
  2. map  → base_link   (if a SLAM node is running)
  Falls back to identity if no TF is available (logs a warning).

Parameters:
  output_directory  (str,   default ~/Dobot_Robot_Arm/scans)
  odom_frame        (str,   default "odom")   parent frame for TF lookup
  robot_frame       (str,   default "base_link")

Services:
  /map_merger/add_block  (Trigger) — place next block using TF + GICP
  /map_merger/save       (Trigger) — save merged_map.pcd
  /map_merger/reset      (Empty)   — clear world map

Topics:
  subscribed: /scanner/merged_cloud  (PointCloud2, frame=base_link)
  published:  /map_merger/world_map  (PointCloud2, frame=odom)
"""

import os
import threading

import numpy as np
import open3d as o3d

import rclpy
import rclpy.duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger, Empty as EmptySrv
import sensor_msgs_py.point_cloud2 as pc2
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_msgs.msg import Header
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

# ── GICP settings ─────────────────────────────────────────────────────────────
_MM_GICP_MAX_CORR     = 0.15   # 150 mm correspondence distance
_MM_GICP_MAX_ITER     = 200
_MM_GICP_VOXEL        = 0.005  # 5 mm downsample
_MM_GICP_MIN_FITNESS  = 0.10   # need ≥10% point overlap to trust GICP result
_MM_GICP_MAX_TRANS_M  = 0.50   # reject correction >50 cm on top of TF init
_MM_GICP_MAX_ROT_DEG  = 15.0   # reject correction >15° on top of TF init


# ── helpers ───────────────────────────────────────────────────────────────────

def _ros_to_numpy(msg: PointCloud2) -> np.ndarray:
    """PointCloud2 → Nx4 float32 array."""
    gen = pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
    pts = np.array([[p[0], p[1], p[2], p[3]] for p in gen], dtype=np.float32)
    return pts.reshape(-1, 4) if pts.size > 0 else np.zeros((0, 4), dtype=np.float32)


def _numpy_to_o3d(pts: np.ndarray) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts[:, :3].astype(np.float64))
    return pcd


def _apply_transform(pts: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Apply 4×4 transform to Nx4 point array, return Nx4 (preserves intensity)."""
    pts_h = np.hstack([pts[:, :3], np.ones((pts.shape[0], 1), dtype=pts.dtype)])  # (N, 4)
    pts_xyz = (T @ pts_h.T).T[:, :3]  # (N, 3)
    pts_out = np.hstack([pts_xyz, pts[:, 3:4]])  # (N, 4) with intensity
    return pts_out.astype(np.float32)


def _tf_stamped_to_matrix(ts) -> np.ndarray:
    """geometry_msgs/TransformStamped → 4×4 float64 ndarray."""
    t = ts.transform.translation
    q = ts.transform.rotation
    # Quaternion → rotation matrix (manual, avoids tf_transformations dep)
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3,  3] = [t.x, t.y, t.z]
    return T


def _gicp_register(source_pts: np.ndarray,
                   target_pts: np.ndarray,
                   init_T: np.ndarray) -> tuple[np.ndarray, float, float, float]:
    """
    GICP: align source onto target starting from init_T.
    Returns (final_T 4×4, fitness, dt_m, dr_deg).
    Mirrors merge_scans.py: estimate_normals → registration_generalized_icp.
    """
    src = _numpy_to_o3d(source_pts).voxel_down_sample(_MM_GICP_VOXEL)
    tgt = _numpy_to_o3d(target_pts).voxel_down_sample(_MM_GICP_VOXEL)

    r = _MM_GICP_VOXEL * 6
    src.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=r, max_nn=20))
    tgt.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=r, max_nn=20))

    result = o3d.pipelines.registration.registration_generalized_icp(
        src, tgt,
        max_correspondence_distance=_MM_GICP_MAX_CORR,
        init=init_T,
        estimation_method=o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=_MM_GICP_MAX_ITER),
    )

    T = result.transformation
    # dt = how much GICP moved the cloud ON TOP of the initial guess
    dt     = float(np.linalg.norm(T[:3, 3] - init_T[:3, 3]))
    # dr = total rotation angle of the final transform (not the delta — used for sanity only)
    ct     = np.clip((np.trace(T[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)
    dr_deg = float(np.degrees(np.arccos(ct)))
    return T.astype(np.float32), float(result.fitness), dt, dr_deg


# ── node ──────────────────────────────────────────────────────────────────────

class MapMergerNode(Node):
    def __init__(self):
        super().__init__("map_merger")

        # ── parameters ──────────────────────────────────────────────────────
        self.declare_parameter("output_directory",
                               os.path.join(os.environ["HOME"], "Dobot_Robot_Arm", "scans"))
        self.declare_parameter("odom_frame",  "odom")
        self.declare_parameter("robot_frame", "base_link")

        self._output_dir  = self.get_parameter("output_directory").value
        self._odom_frame  = self.get_parameter("odom_frame").value
        self._robot_frame = self.get_parameter("robot_frame").value

        os.makedirs(self._output_dir, exist_ok=True)

        # ── TF listener ──────────────────────────────────────────────────────
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── state ────────────────────────────────────────────────────────────
        self._lock              = threading.Lock()
        self._block_count       = 0
        self._world_pts         = np.zeros((0, 4), dtype=np.float32)
        self._latest_block_msg  = None
        # TF of block #1 in world frame (used to express subsequent blocks relative to it)
        self._first_block_T_inv: np.ndarray | None = None

        # ── pub/sub/srv ──────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            PointCloud2, "/scanner/merged_cloud", self._block_cb, 1)
        self._pub = self.create_publisher(PointCloud2, "/map_merger/world_map", 1)

        self.create_service(Trigger,  "/map_merger/add_block", self._add_block_cb)
        self.create_service(Trigger,  "/map_merger/save",      self._save_cb)
        self.create_service(EmptySrv, "/map_merger/reset",     self._reset_cb)

        self._clear_client = self.create_client(EmptySrv, "/scanner/clear_scans")
        self._param_client = self.create_client(
            SetParameters, "/scan_accumulator/set_parameters")

        self.get_logger().info(
            f"[MAP_MERGER] Ready — TF-guided GICP merging\n"
            f"  odom_frame={self._odom_frame}  robot_frame={self._robot_frame}\n"
            f"  /map_merger/add_block → place next block (reads TF + runs GICP)\n"
            f"  /map_merger/save      → save merged_map.pcd\n"
            f"  /map_merger/reset     → clear map")

    # ── block subscription ────────────────────────────────────────────────────

    def _block_cb(self, msg: PointCloud2):
        with self._lock:
            self._latest_block_msg = msg

    # ── TF lookup ─────────────────────────────────────────────────────────────

    def _lookup_rover_tf(self) -> tuple[np.ndarray, bool]:
        """
        Look up odom_frame → robot_frame right now.
        Returns (4×4 float64 matrix, have_tf).
        Falls back to identity with a warning if TF is unavailable.
        """
        try:
            ts = self._tf_buffer.lookup_transform(
                self._odom_frame, self._robot_frame,
                rclpy.time.Time(),                          # latest available
                timeout=rclpy.duration.Duration(seconds=1.0))
            T = _tf_stamped_to_matrix(ts)
            t = T[:3, 3]
            self.get_logger().info(
                f"[MAP_MERGER] TF {self._odom_frame}→{self._robot_frame}: "
                f"({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}) m")
            return T, True
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"[MAP_MERGER] TF lookup failed: {e}  — using identity as initial guess")
            return np.eye(4, dtype=np.float64), False

    # ── add_block service ─────────────────────────────────────────────────────

    def _add_block_cb(self, req, resp):
        # Read TF *before* taking the data lock (TF buffer uses its own lock)
        T_odom_robot, have_tf = self._lookup_rover_tf()

        with self._lock:
            if self._latest_block_msg is None:
                resp.success = False
                resp.message = "No block on /scanner/merged_cloud yet"
                return resp

            block_base = _ros_to_numpy(self._latest_block_msg)
            if block_base.shape[0] == 0:
                resp.success = False
                resp.message = "Block cloud is empty"
                return resp

            self._block_count += 1

            # Save merged block in block folder
            block_folder = os.path.join(self._output_dir, f"block{self._block_count}")
            os.makedirs(block_folder, exist_ok=True)
            merged_path = os.path.join(block_folder, f"block{self._block_count}.pcd")
            o3d.io.write_point_cloud(merged_path, _numpy_to_o3d(block_base))
            self.get_logger().info(
                f"[MAP_MERGER] Saved merged block #{self._block_count} → {merged_path} ({block_base.shape[0]} pts)")

            # ── Block #1: anchor the world frame ──────────────────────────────
            # Store its inverse TF so subsequent blocks are expressed relative to
            # the first rover position (keeps the world map near the origin).
            if self._block_count == 1:
                self._first_block_T_inv = np.linalg.inv(T_odom_robot)
                # Block #1 sits at origin in our world frame — no transform needed
                init_T = np.eye(4, dtype=np.float64)
                block_world = block_base.copy()
                self._world_pts = block_world
                self.get_logger().info(
                    f"[MAP_MERGER] Block #1 → world origin  "
                    f"({block_world.shape[0]} pts)")
                self._latest_block_msg = None
                resp.success = True
                resp.message = f"Block #1 set as world origin ({block_world.shape[0]} pts)"
                # (fall through to cleanup outside the lock)

            else:
                # ── Block #2+: TF-based initial guess + GICP refinement ───────
                #
                # init_T = T_rel = T_block1_inv @ T_blockN
                # This is the same idea as merge_scans.py's trans_init:
                #   we pre-translate the source by how far the rover moved,
                #   then GICP corrects residual error.
                T_rel = self._first_block_T_inv @ T_odom_robot   # relative to block #1
                init_T = T_rel  # 4×4 initial guess for GICP

                # Swap X and Y in TF translation for quick frame workaround
                init_T_swapped = init_T.copy()
                tmp = init_T_swapped[0, 3]
                init_T_swapped[0, 3] = init_T_swapped[1, 3]
                init_T_swapped[1, 3] = tmp

                t_init = init_T_swapped[:3, 3]
                self.get_logger().info(
                    f"[MAP_MERGER] Block #{self._block_count} TF initial guess (X/Y swapped): "
                    f"({t_init[0]:.3f}, {t_init[1]:.3f}, {t_init[2]:.3f}) m  "
                    f"{'(from TF, X/Y swapped)' if have_tf else '(IDENTITY FALLBACK)'}")

                # Run GICP: source = this block in base_link, target = cumulative world map
                # init_T_swapped places the source near where it should be in world frame
                final_T, fitness, dt, dr_deg = _gicp_register(
                    block_base, self._world_pts, init_T_swapped.astype(np.float32))

                self.get_logger().info(
                    f"[MAP_MERGER] Block #{self._block_count} GICP: "
                    f"fitness={fitness:.4f}  correction_dt={dt:.4f}m  dr={dr_deg:.2f}°")

                # Determine rejection reason (if any) for a clear log message
                reject_reason = None
                if fitness < _MM_GICP_MIN_FITNESS:
                    reject_reason = (f"no/low overlap (fitness={fitness:.4f} < "
                                     f"{_MM_GICP_MIN_FITNESS}) — blocks too far apart, "
                                     f"using TF odometry position only")
                elif dt > _MM_GICP_MAX_TRANS_M:
                    reject_reason = (f"GICP correction too large (dt={dt:.3f}m > "
                                     f"{_MM_GICP_MAX_TRANS_M}m) — likely diverged")
                elif dr_deg > _MM_GICP_MAX_ROT_DEG:
                    reject_reason = (f"GICP rotation too large (dr={dr_deg:.1f}° > "
                                     f"{_MM_GICP_MAX_ROT_DEG}°) — likely diverged")

                if reject_reason is None:
                    used_T = final_T
                    gicp_status = "GICP-refined"
                else:
                    used_T = init_T.astype(np.float32)
                    gicp_status = "TF-only"
                    self.get_logger().warn(
                        f"[MAP_MERGER] Block #{self._block_count} GICP rejected: {reject_reason}")

                block_world = _apply_transform(block_base, used_T)
                self._world_pts = np.vstack([self._world_pts, block_world])

                bmin = block_world.min(axis=0); bmax = block_world.max(axis=0)
                self.get_logger().info(
                    f"[MAP_MERGER] Block #{self._block_count} [{gicp_status}] "
                    f"bbox X[{bmin[0]:.3f}..{bmax[0]:.3f}] "
                    f"Y[{bmin[1]:.3f}..{bmax[1]:.3f}] → "
                    f"{self._world_pts.shape[0]} pts total")

                self._latest_block_msg = None
                resp.success = True
                resp.message = (
                    f"Block #{self._block_count} [{gicp_status}], "
                    f"world map: {self._world_pts.shape[0]} pts")

        # ── outside lock: cleanup ─────────────────────────────────────────────
        self._async_clear_scans()
        self._async_set_prefix(f"pos{self._block_count + 1}")
        self._publish_world_map()
        return resp

    # ── save service ──────────────────────────────────────────────────────────

    def _save_cb(self, req, resp):
        with self._lock:
            if self._world_pts.shape[0] == 0:
                resp.success = False
                resp.message = "World map empty"
                return resp
            path = os.path.join(self._output_dir, "merged_map.pcd")
            o3d.io.write_point_cloud(path, _numpy_to_o3d(self._world_pts))
            n = self._world_pts.shape[0]
        resp.success = True
        resp.message = f"Saved {n} pts → {path}"
        self.get_logger().info(f"[MAP_MERGER] {resp.message}")
        return resp

    # ── reset service ─────────────────────────────────────────────────────────

    def _reset_cb(self, req, resp):
        with self._lock:
            self._world_pts         = np.zeros((0, 4), dtype=np.float32)
            self._block_count       = 0
            self._latest_block_msg  = None
            self._first_block_T_inv = None
        self.get_logger().info("[MAP_MERGER] World map reset — TF anchor cleared")
        self._async_set_prefix("pos1")
        return resp

    # ── publish ───────────────────────────────────────────────────────────────

    def _publish_world_map(self):
        with self._lock:
            if self._world_pts.shape[0] == 0:
                return
            pts = self._world_pts.copy()
        header = Header()
        header.frame_id = "odom"
        header.stamp    = self.get_clock().now().to_msg()
        msg = create_cloud_xyz32(header, pts[:, :3].tolist())
        self._pub.publish(msg)

    # ── async helpers ─────────────────────────────────────────────────────────

    def _async_clear_scans(self):
        if self._clear_client.wait_for_service(timeout_sec=1.0):
            self._clear_client.call_async(EmptySrv.Request())
        else:
            self.get_logger().warn("[MAP_MERGER] /scanner/clear_scans unavailable")

    def _async_set_prefix(self, prefix: str):
        if not self._param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f"[MAP_MERGER] scan_accumulator param service not ready — "
                f"next scans may use old prefix. "
                f"Use: ros2 param set /scan_accumulator position_prefix {prefix}")
            return

        req = SetParameters.Request()
        pval = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=prefix)
        req.parameters = [Parameter(name="position_prefix", value=pval)]

        future = self._param_client.call_async(req)

        def _done(f):
            try:
                f.result()
                self.get_logger().info(
                    f"[MAP_MERGER] scan_accumulator position_prefix → {prefix}")
            except Exception as e:
                self.get_logger().warn(f"[MAP_MERGER] Failed to set prefix: {e}")

        future.add_done_callback(_done)
