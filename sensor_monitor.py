#!/usr/bin/env python3
"""
Sensor Diagnostic Monitor for myvehicle_sensors.py
====================================================
Subscribes to ALL sensor topics published by myvehicle_sensors.py and displays
a live terminal dashboard showing:
  - Per-topic publish frequency (Hz)
  - Latest sensor values
  - Message latency (msg stamp vs. wall clock)
  - Dead / stale topic detection

Usage:
  # Make sure ROS2 env is sourced and myvehicle_sensors.py --ros is running
  python3 sensor_monitor.py

  # Optional args:
  python3 sensor_monitor.py --refresh-rate 2.0   # refresh every 0.5s
  python3 sensor_monitor.py --window 50           # rolling window for Hz calc
"""

import sys
import time
import threading
import argparse
from collections import deque

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
except ImportError:
    print("[ERROR] rclpy not found. Source your ROS2 environment first.")
    sys.exit(1)

from sensor_msgs.msg import PointCloud2, Image, CameraInfo, Imu, NavSatFix


# ─────────────────────────────────────────────────────────────────────────────
# ANSI Color Helpers
# ─────────────────────────────────────────────────────────────────────────────
class C:
    """ANSI terminal colors."""
    RESET   = "\033[0m"
    BOLD    = "\033[1m"
    DIM     = "\033[2m"
    RED     = "\033[91m"
    GREEN   = "\033[92m"
    YELLOW  = "\033[93m"
    CYAN    = "\033[96m"
    WHITE   = "\033[97m"
    BG_RED  = "\033[41m"
    BG_GREEN = "\033[42m"
    BG_YELLOW = "\033[43m"


# ─────────────────────────────────────────────────────────────────────────────
# TopicTracker
# ─────────────────────────────────────────────────────────────────────────────
class TopicTracker:
    """Tracks frequency, latency, and latest values for one topic."""

    def __init__(self, topic_name, msg_type, expected_hz=10.0, window=100):
        self.topic = topic_name
        self.msg_type = msg_type
        self.expected_hz = expected_hz
        self._stamps = deque(maxlen=window)  # wall-clock arrival times
        self._msg_stamps = deque(maxlen=window)  # message header stamps (sim-time)
        self._lock = threading.Lock()
        self.latest_info = "waiting..."
        self.latest_jitter_ms = None   # inter-message jitter (ms)
        self.latest_sim_stamp = None   # latest sim-time stamp (s)
        self.data_warnings = []        # data-level warnings (e.g. 0 points)
        self.msg_count = 0
        self.last_recv_time = None
        self.first_recv_time = None

    def record(self, wall_now, msg_stamp_sec=None, info_str="", data_warnings=None):
        """Called from the subscription callback."""
        with self._lock:
            # Compute inter-message jitter (wall-clock gap vs expected period)
            if len(self._stamps) >= 1 and self.expected_hz > 0:
                actual_gap = wall_now - self._stamps[-1]
                expected_gap = 1.0 / self.expected_hz
                self.latest_jitter_ms = (actual_gap - expected_gap) * 1000.0

            self._stamps.append(wall_now)
            self.msg_count += 1
            self.last_recv_time = wall_now
            if self.first_recv_time is None:
                self.first_recv_time = wall_now
            if info_str:
                self.latest_info = info_str
            if msg_stamp_sec is not None:
                self.latest_sim_stamp = msg_stamp_sec
                self._msg_stamps.append(msg_stamp_sec)
            if data_warnings is not None:
                self.data_warnings = data_warnings

    def get_hz(self):
        """Compute frequency from the rolling window."""
        with self._lock:
            if len(self._stamps) < 2:
                return 0.0
            dt = self._stamps[-1] - self._stamps[0]
            if dt <= 0:
                return 0.0
            return (len(self._stamps) - 1) / dt

    def get_sim_hz(self):
        """Compute frequency from message stamps (sim-time)."""
        with self._lock:
            if len(self._msg_stamps) < 2:
                return 0.0
            dt = self._msg_stamps[-1] - self._msg_stamps[0]
            if dt <= 0:
                return 0.0
            return (len(self._msg_stamps) - 1) / dt

    def get_avg_jitter_ms(self):
        """Average absolute jitter over the window."""
        with self._lock:
            if len(self._stamps) < 2:
                return None
            gaps = []
            expected = 1.0 / self.expected_hz if self.expected_hz > 0 else 0
            stamps_list = list(self._stamps)
            for i in range(1, len(stamps_list)):
                actual = stamps_list[i] - stamps_list[i-1]
                gaps.append(abs(actual - expected) * 1000.0)
            return sum(gaps) / len(gaps) if gaps else None

    def get_status(self):
        """Return (hz, jitter_ms, info, age_sec, color, status, data_warnings, sim_hz)."""
        hz = self.get_hz()
        sim_hz = self.get_sim_hz()
        avg_jitter = self.get_avg_jitter_ms()
        now = time.time()
        age = (now - self.last_recv_time) if self.last_recv_time else None

        if age is None or age > 5.0:
            color = C.RED
            status = "DEAD"
        elif age > 2.0:
            color = C.RED
            status = "STALE"
        elif hz < self.expected_hz * 0.3:
            color = C.YELLOW
            status = "LOW"
        elif self.data_warnings:
            color = C.YELLOW
            status = "WARN"
        else:
            color = C.GREEN
            status = "OK"

        return hz, avg_jitter, self.latest_info, age, color, status, self.data_warnings, sim_hz


# ─────────────────────────────────────────────────────────────────────────────
# Sensor Monitor Node
# ─────────────────────────────────────────────────────────────────────────────
class SensorMonitorNode(Node):
    """ROS2 node that subscribes to all CARLA sensor topics."""

    # All topics from myvehicle_sensors.py
    TOPICS = [
        # (topic, msg_type, expected_hz, label)
        ("/ouster/points",          PointCloud2, 10.0, "Ouster LiDAR"),
        ("/velodyne/points",        PointCloud2, 10.0, "Velodyne LiDAR"),
        ("/zed/left/image_raw",     Image,       10.0, "ZED Left Image"),
        ("/zed/left/camera_info",   CameraInfo,  10.0, "ZED Left CamInfo"),
        ("/zed/right/image_raw",    Image,       10.0, "ZED Right Image"),
        ("/zed/right/camera_info",  CameraInfo,  10.0, "ZED Right CamInfo"),
        ("/imu/imu1/data",          Imu,         20.0, "IMU 1"),
        ("/imu/imu2/data",          Imu,         20.0, "IMU 2"),
        ("/gnss/front_right/fix",   NavSatFix,   10.0, "GNSS Front Right"),
        ("/gnss/rear_right/fix",    NavSatFix,   10.0, "GNSS Rear Right"),
    ]

    def __init__(self, window_size=100):
        super().__init__("sensor_monitor")
        self.trackers = {}

        # Match myvehicle_sensors.py QoS: BEST_EFFORT, KEEP_LAST(1)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        for topic, msg_type, expected_hz, label in self.TOPICS:
            tracker = TopicTracker(topic, msg_type, expected_hz, window_size)
            self.trackers[topic] = (tracker, label)

            # Create subscription with a closure to capture topic
            self.create_subscription(
                msg_type, topic,
                lambda msg, t=topic: self._on_msg(t, msg),
                qos
            )

        self.get_logger().info(f"Monitoring {len(self.TOPICS)} sensor topics...")

    def _stamp_to_sec(self, stamp):
        """Convert ROS2 stamp to float seconds."""
        return stamp.sec + stamp.nanosec * 1e-9

    def _on_msg(self, topic, msg):
        """Generic callback for any topic."""
        tracker, _ = self.trackers[topic]
        now = time.time()

        # Extract timestamp from header
        msg_stamp_sec = None
        if hasattr(msg, 'header') and msg.header.stamp.sec > 0:
            msg_stamp_sec = self._stamp_to_sec(msg.header.stamp)

        # Build human-readable info + data-level warnings
        info, warnings = self._extract_info(topic, msg)
        tracker.record(now, msg_stamp_sec, info, warnings)

    def _extract_info(self, topic, msg):
        """Extract meaningful data and data-level warnings from each message type."""
        warnings = []
        try:
            if isinstance(msg, PointCloud2):
                if msg.width == 0:
                    warnings.append("0 POINTS — sensor may be blocked or misconfigured")
                return f"points={msg.width:,}  fields={len(msg.fields)}  step={msg.point_step}B", warnings

            elif isinstance(msg, Image):
                if msg.width == 0 or msg.height == 0:
                    warnings.append("Empty image")
                return f"{msg.width}x{msg.height}  enc={msg.encoding}  step={msg.step}", warnings

            elif isinstance(msg, CameraInfo):
                return f"{msg.width}x{msg.height}  model={msg.distortion_model}", warnings

            elif isinstance(msg, Imu):
                a = msg.linear_acceleration
                g = msg.angular_velocity
                # Sanity check: stationary vehicle should have accel.z ≈ ±9.81
                if abs(a.z) < 5.0:
                    warnings.append(f"accel.z={a.z:.2f} — expected ~9.81 (gravity missing?)")
                return (f"acc=({a.x:+.2f},{a.y:+.2f},{a.z:+.2f})  "
                        f"gyro=({g.x:+.3f},{g.y:+.3f},{g.z:+.3f})"), warnings

            elif isinstance(msg, NavSatFix):
                if msg.latitude == 0.0 and msg.longitude == 0.0:
                    warnings.append("lat/lon both 0 — no GPS fix")
                return (f"lat={msg.latitude:.7f}  lon={msg.longitude:.7f}  "
                        f"alt={msg.altitude:.2f}  status={msg.status.status}"), warnings

        except Exception as e:
            return f"parse error: {e}", warnings

        return "?", warnings


# ─────────────────────────────────────────────────────────────────────────────
# Terminal Dashboard
# ─────────────────────────────────────────────────────────────────────────────
def clear_screen():
    """ANSI clear + cursor home."""
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()


def render_dashboard(node: SensorMonitorNode, start_time: float):
    """Render a single frame of the terminal dashboard."""
    clear_screen()
    elapsed = time.time() - start_time

    # Header
    print(f"{C.BOLD}{C.CYAN}╔══════════════════════════════════════════════════════════════════════════════════════════════════════╗{C.RESET}")
    print(f"{C.BOLD}{C.CYAN}║  CARLA SENSOR DIAGNOSTIC MONITOR                                        elapsed: {elapsed:8.1f}s       ║{C.RESET}")
    print(f"{C.BOLD}{C.CYAN}╠══════════════════════════════════════════════════════════════════════════════════════════════════════╣{C.RESET}")

    # Column headers
    print(f"{C.BOLD}  {'Status':<8} {'Topic':<30} {'Label':<18} {'WallHz':>7} {'SimHz':>7} {'Jitter':>8} {'Count':>7}  {'Latest Value'}{C.RESET}")
    print(f"  {'─'*8} {'─'*30} {'─'*18} {'─'*7} {'─'*7} {'─'*8} {'─'*7}  {'─'*40}")

    total_hz = 0.0
    total_count = 0
    issues = []

    for topic, msg_type, expected_hz, label in SensorMonitorNode.TOPICS:
        tracker, _ = node.trackers[topic]
        hz, avg_jitter, info, age, color, status, data_warns, sim_hz = tracker.get_status()
        total_hz += hz
        total_count += tracker.msg_count

        # Status badge
        if status == "OK":
            badge = f"{C.BG_GREEN}{C.BOLD} OK   {C.RESET}"
        elif status == "WARN":
            badge = f"{C.BG_YELLOW}{C.BOLD} WARN {C.RESET}"
        elif status == "LOW":
            badge = f"{C.BG_YELLOW}{C.BOLD} LOW  {C.RESET}"
        elif status == "STALE":
            badge = f"{C.BG_RED}{C.BOLD} STALE{C.RESET}"
        else:
            badge = f"{C.BG_RED}{C.BOLD} DEAD {C.RESET}"

        # Format jitter
        if avg_jitter is not None:
            jit_str = f"{avg_jitter:.1f}ms"
            if avg_jitter > 50:
                jit_str = f"{C.RED}{jit_str}{C.RESET}"
            elif avg_jitter > 20:
                jit_str = f"{C.YELLOW}{jit_str}{C.RESET}"
        else:
            jit_str = f"{C.DIM}---{C.RESET}"

        hz_str = f"{hz:.1f}" if hz > 0 else f"{C.DIM}0.0{C.RESET}"
        sim_hz_str = f"{sim_hz:.1f}" if sim_hz > 0 else f"{C.DIM}---{C.RESET}"

        # Truncate info for display
        info_display = info[:55] if info else ""

        print(f"  {badge}  {color}{topic:<30}{C.RESET} {label:<18} {hz_str:>7} {sim_hz_str:>7} {jit_str:>16} {tracker.msg_count:>7}  {C.DIM}{info_display}{C.RESET}")

        # Collect issues
        if status in ("DEAD", "STALE"):
            issues.append(f"  {C.RED}✗ {topic} — {status} (no data for {age:.1f}s){C.RESET}" if age else f"  {C.RED}✗ {topic} — NEVER RECEIVED{C.RESET}")
        elif status == "LOW":
            issues.append(f"  {C.YELLOW}⚠ {topic} — LOW Hz ({hz:.1f}/{expected_hz:.0f}){C.RESET}")
        # Data-level warnings
        for w in data_warns:
            issues.append(f"  {C.YELLOW}⚠ {topic} — {w}{C.RESET}")
        # High jitter warning
        if avg_jitter is not None and avg_jitter > 50:
            issues.append(f"  {C.YELLOW}⚠ {topic} — HIGH JITTER ({avg_jitter:.1f} ms avg){C.RESET}")

    # Separator
    print(f"\n{C.BOLD}{C.CYAN}╠══════════════════════════════════════════════════════════════════════════════════════════════════════╣{C.RESET}")

    # Summary
    print(f"  {C.BOLD}Total messages: {total_count:,}    Combined rate: {total_hz:.1f} Hz{C.RESET}")

    # Issues section
    if issues:
        print(f"\n  {C.BOLD}{C.RED}⚠  ISSUES DETECTED:{C.RESET}")
        for issue in issues:
            print(issue)
    else:
        print(f"\n  {C.BOLD}{C.GREEN}✓  All sensors publishing normally{C.RESET}")

    print(f"\n{C.BOLD}{C.CYAN}╚══════════════════════════════════════════════════════════════════════════════════════════════════════╝{C.RESET}")

    # Help note
    print(f"\n{C.DIM}  WallHz = actual receive rate | SimHz = rate in sim-time | Jitter = delivery consistency")
    print(f"  Jitter > 50ms indicates delivery delays. Data warnings check sensor values for correctness.{C.RESET}")
    print(f"{C.DIM}  Press Ctrl+C to exit.{C.RESET}")


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="CARLA Sensor Diagnostic Monitor")
    parser.add_argument("--refresh-rate", type=float, default=1.0,
                        help="Dashboard refresh interval in seconds (default: 1.0)")
    parser.add_argument("--window", type=int, default=100,
                        help="Rolling window size for Hz calculation (default: 100)")
    parser.add_argument("--no-clear", action="store_true",
                        help="Don't clear screen (useful for logging to file)")
    args = parser.parse_args()

    rclpy.init()
    node = SensorMonitorNode(window_size=args.window)

    # Spin in a background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    start_time = time.time()

    print(f"\n{C.BOLD}[SensorMonitor] Waiting for messages on {len(SensorMonitorNode.TOPICS)} topics...{C.RESET}")
    print(f"{C.DIM}  Make sure myvehicle_sensors.py --ros is running.{C.RESET}\n")
    time.sleep(2.0)  # Give subscriptions time to connect

    try:
        while rclpy.ok():
            if args.no_clear:
                # Simple line-by-line output
                print(f"\n--- [{time.time() - start_time:.1f}s] ---")
                for topic, msg_type, expected_hz, label in SensorMonitorNode.TOPICS:
                    tracker, _ = node.trackers[topic]
                    hz, jitter, info, age, _, status, warns, sim_hz = tracker.get_status()
                    jit_str = f"{jitter:.1f}ms" if jitter is not None else "---"
                    warn_str = f"  ⚠ {'; '.join(warns)}" if warns else ""
                    print(f"  [{status:>5}] {topic:<30} {hz:6.1f} Hz (sim:{sim_hz:5.1f})  jit={jit_str:<10}  cnt={tracker.msg_count:<6}  {info}{warn_str}")
            else:
                render_dashboard(node, start_time)

            time.sleep(args.refresh_rate)

    except KeyboardInterrupt:
        print(f"\n{C.BOLD}[SensorMonitor] Shutting down...{C.RESET}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print(f"{C.GREEN}[SensorMonitor] Done.{C.RESET}")


if __name__ == "__main__":
    main()
