"""
scanning_coordinator.py — Python port of scanning_coordinator.cpp

Orchestrates a multi-strip scanning sequence:
  1. Subscribe to /start_repair (Bool)
  2. On True: trigger num_scans strips (with delays), then call /map_merger/add_block

Parameters (set via constructor or ros2 param):
  num_scans                (int,   default 3)
  delay_before_first_scan  (float, default 2.4 s)
  delay_between_scans      (float, default 13.1 s)
  delay_before_add_block   (float, default 2.0 s)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class ScanningCoordinatorNode(Node):
    def __init__(self):
        super().__init__("scanning_coordinator")

        # ── parameters ──────────────────────────────────────────────────────
        self.declare_parameter("num_scans",               3)
        self.declare_parameter("delay_before_first_scan", 2.4)
        self.declare_parameter("delay_between_scans",    13.1)
        self.declare_parameter("delay_before_add_block",  2.0)

        self._num_scans              = self.get_parameter("num_scans").value
        self._delay_before_first     = self.get_parameter("delay_before_first_scan").value
        self._delay_between          = self.get_parameter("delay_between_scans").value
        self._delay_before_add_block = self.get_parameter("delay_before_add_block").value

        # ── state ────────────────────────────────────────────────────────────
        self._current_scan    = 0
        self._block_count     = 0
        self._sequence_active = False
        self._delay_timer     = None

        # ── clients ──────────────────────────────────────────────────────────
        self._scan_client      = self.create_client(Trigger, "/scanner/trigger_scan")
        self._add_block_client = self.create_client(Trigger, "/map_merger/add_block")

        # ── subscription ─────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            Bool, "/start_repair", self._trigger_cb, 10)

        self.get_logger().info(
            "[COORDINATOR] Ready — publish /start_repair (True) to begin a "
            f"{self._num_scans}-scan block.\n"
            "  Repeat as many times as needed at any position.\n"
            "  Call /map_merger/save when finished to write merged_map.pcd")

    # ── trigger callback ─────────────────────────────────────────────────────

    def _trigger_cb(self, msg: Bool):
        if not msg.data or self._sequence_active:
            return

        self._block_count += 1
        self._sequence_active = True
        self._current_scan    = 1

        self.get_logger().info(
            f"[COORDINATOR] ▶ Block #{self._block_count} — starting "
            f"{self._num_scans}-scan sequence")

        self._schedule(self._delay_before_first, self._trigger_next_scan)

    # ── sequencing ───────────────────────────────────────────────────────────

    def _trigger_next_scan(self):
        if self._current_scan > self._num_scans:
            # All strips done — wait then add_block
            self._schedule(self._delay_before_add_block, self._add_block)
            return

        self.get_logger().info(
            f"[COORDINATOR] Triggering strip {self._current_scan}/{self._num_scans}...")

        if self._current_scan > 1:
            # Insert inter-strip delay
            self._schedule(self._delay_between, self._trigger_scan)
        else:
            self._trigger_scan()

    def _trigger_scan(self):
        if not self._scan_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("[COORDINATOR] /scanner/trigger_scan unavailable!")
            self._sequence_active = False
            return

        future = self._scan_client.call_async(Trigger.Request())
        future.add_done_callback(self._scan_done)

    def _scan_done(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"[COORDINATOR] Scan service error: {e}")
            self._sequence_active = False
            return

        if resp.success:
            self.get_logger().info(
                f"[COORDINATOR] ✓ Strip {self._current_scan}/{self._num_scans} done")
            self._current_scan += 1
            self._trigger_next_scan()
        else:
            self.get_logger().error(
                f"[COORDINATOR] ✗ Strip {self._current_scan} failed: {resp.message}")
            self._sequence_active = False

    def _add_block(self):
        self.get_logger().info(
            f"[COORDINATOR] Block #{self._block_count} complete — merging into world map...")

        if not self._add_block_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("[COORDINATOR] /map_merger/add_block unavailable!")
            self._sequence_active = False
            return

        future = self._add_block_client.call_async(Trigger.Request())
        future.add_done_callback(self._add_block_done)

    def _add_block_done(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"[COORDINATOR] add_block service error: {e}")
            self._sequence_active = False
            return

        if resp.success:
            self.get_logger().info(
                f"[COORDINATOR] ✓ {resp.message}\n"
                "  → Publish /start_repair again or call /map_merger/save")
        else:
            self.get_logger().error(f"[COORDINATOR] add_block failed: {resp.message}")

        self._sequence_active = False

    # ── timer helper ─────────────────────────────────────────────────────────

    def _schedule(self, delay_s: float, callback):
        """Fire callback once after delay_s seconds, then cancel the timer."""
        if self._delay_timer is not None:
            self._delay_timer.cancel()

        def _wrapper():
            self._delay_timer.cancel()
            callback()

        self._delay_timer = self.create_timer(delay_s, _wrapper)
