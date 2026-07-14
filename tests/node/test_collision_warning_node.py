"""
MODULE TEST 4 — test_collision_warning_node.py
Test CollisionWarningNode qua ROS2 topic thực tế (cần rclpy).
Inject /scan giả → đọc /cmd_vel_safe → verify scale đúng.

Cần: ROS2 Humble sourced
Chạy:
  source /opt/ros/humble/setup.bash
  python3 tests/node/test_collision_warning_node.py
"""

import sys, os, math, time, threading
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# Thêm đường dẫn để import node gốc nếu cần
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../src/amr_safety/scripts'))


PASS = "\033[92m[PASS]\033[0m"
FAIL = "\033[91m[FAIL]\033[0m"


class TestPublisher(Node):
    """Node test: publish /scan + /cmd_vel, đọc /cmd_vel_safe"""

    def __init__(self):
        super().__init__('test_collision_warning')
        self.scan_pub    = self.create_publisher(LaserScan, '/scan',    10)
        self.cmd_pub     = self.create_publisher(Twist,     '/cmd_vel', 10)
        self.received    = []
        self.warning_log = []

        self.create_subscription(Twist,  '/cmd_vel_safe',      self._on_safe,    10)
        self.create_subscription(String, '/collision_warning',  self._on_warning, 10)

    def _on_safe(self, msg):
        self.received.append(msg)

    def _on_warning(self, msg):
        self.warning_log.append(msg.data)

    def make_scan(self, ranges_deg: dict, n=720):
        """Tạo LaserScan từ dict {angle_deg: dist_m}"""
        msg = LaserScan()
        msg.header.frame_id = 'lidar_link'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.angle_min       = -math.pi
        msg.angle_max       =  math.pi - (2*math.pi/n)
        msg.angle_increment =  2*math.pi / n
        msg.range_min       = 0.12
        msg.range_max       = 12.0
        msg.ranges          = [12.0] * n
        for deg, dist in ranges_deg.items():
            idx = int((math.radians(deg) - msg.angle_min) / msg.angle_increment)
            idx = max(0, min(n-1, idx))
            msg.ranges[idx] = dist
        return msg

    def publish_cmd(self, vx=0.5, vy=0.0, wz=0.0):
        msg = Twist()
        msg.linear.x  = vx
        msg.linear.y  = vy
        msg.angular.z = wz
        self.cmd_pub.publish(msg)

    def wait_for_safe(self, timeout=2.0):
        """Block cho đến khi nhận được /cmd_vel_safe"""
        t0 = time.time()
        before = len(self.received)
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if len(self.received) > before:
                return self.received[-1]
        return None


def check(desc, got, expected, tol=0.02):
    if got is None:
        print(f"  {FAIL}  {desc}  got=None (timeout)")
        return False
    ok = abs(got - expected) < tol
    print(f"  {PASS if ok else FAIL}  {desc}  got={got:.3f}  expected={expected:.3f}")
    return ok


def check_vz(safe):
    """Verify linear.z (vz) is always 0 — Mecanum only uses linear.x/y and angular.z."""
    return check("linear.z = 0.0 (vz unused)", safe.linear.z if safe else None, 0.0)


def run_tests():
    rclpy.init()

    # Khởi động collision_warning_node trong thread riêng
    from collision_warning_node import CollisionWarningNode
    safety_node = CollisionWarningNode()
    spin_thread = threading.Thread(target=lambda: rclpy.spin(safety_node), daemon=True)
    spin_thread.start()

    tester = TestPublisher()
    time.sleep(0.5)  # Chờ node startup

    results = []

    # ── TEST 1: Không vật cản → cmd_vel_safe = cmd_vel gốc ──────────────────
    print("\n[TEST 1] No obstacle → cmd_vel_safe = cmd_vel (scale=1.0)")
    tester.scan_pub.publish(tester.make_scan({}))
    tester.publish_cmd(vx=0.5, vy=0.0, wz=0.0)
    time.sleep(0.15)
    safe = tester.wait_for_safe()
    results.append(check("vx = 0.50", safe.linear.x if safe else None, 0.50))
    results.append(check("vy = 0.00", safe.linear.y if safe else None, 0.00))
    results.append(check_vz(safe))

    # ── TEST 2: Vật cản trước CRITICAL → vx = 0 ─────────────────────────────
    print("\n[TEST 2] Front obstacle 0.18m (CRITICAL) → vx scaled to 0")
    tester.received.clear()
    tester.scan_pub.publish(tester.make_scan({0: 0.18}))
    tester.publish_cmd(vx=0.5, vy=0.0, wz=0.0)
    time.sleep(0.15)
    safe = tester.wait_for_safe()
    results.append(check("vx = 0.0", safe.linear.x if safe else None, 0.0))
    results.append(check_vz(safe))

    # ── TEST 3: Vật cản trước DANGER 0.30m → vx = 0.5*0.20 = 0.10 ──────────
    print("\n[TEST 3] Front obstacle 0.30m (DANGER) → vx = 0.10")
    tester.received.clear()
    tester.scan_pub.publish(tester.make_scan({0: 0.30}))
    tester.publish_cmd(vx=0.5, vy=0.0, wz=0.0)
    time.sleep(0.15)
    safe = tester.wait_for_safe()
    results.append(check("vx = 0.10", safe.linear.x if safe else None, 0.10))
    results.append(check_vz(safe))

    # ── TEST 4: Emergency < 0.15m → tất cả = 0 ──────────────────────────────
    print("\n[TEST 4] Emergency obstacle 0.10m → all velocities = 0")
    tester.received.clear()
    tester.scan_pub.publish(tester.make_scan({0: 0.10}))
    tester.publish_cmd(vx=0.5, vy=0.3, wz=1.0)
    time.sleep(0.15)
    safe = tester.wait_for_safe()
    results.append(check("vx = 0.0 (emergency)", safe.linear.x if safe else None, 0.0))
    results.append(check("vy = 0.0 (emergency)", safe.linear.y if safe else None, 0.0))
    results.append(check("wz = 0.0 (emergency)", safe.angular.z if safe else None, 0.0))
    results.append(check_vz(safe))

    # ── TEST 5: cmd_timeout → zero output ────────────────────────────────────
    print("\n[TEST 5] cmd_timeout (no cmd_vel for 0.6s) → zero output")
    tester.received.clear()
    tester.scan_pub.publish(tester.make_scan({}))
    # Không publish cmd_vel, chờ timeout
    time.sleep(0.7)
    safe = tester.wait_for_safe()
    results.append(check("vx = 0.0 (timeout)", safe.linear.x if safe else None, 0.0))
    results.append(check_vz(safe))

    # ── TEST 6: vy strafe — direction-aware lateral scaling ──────────────────
    # Mecanum: vy > 0 = strafe left (scale_left), vy < 0 = strafe right (scale_rgt)
    # Scenario: obstacle on RIGHT side at 0.30m → DANGER → scale_rgt = 0.20
    #   vy- (strafe toward obstacle) → scaled: -0.5 * 0.20 = -0.10
    #   vy+ (strafe away from obstacle) → unaffected: +0.5 * 1.0 = +0.50
    print("\n[TEST 6] vy strafe direction-aware: right obstacle 0.30m (DANGER)")
    tester.received.clear()
    tester.scan_pub.publish(tester.make_scan({-90: 0.30}))  # obstacle at -90° = right
    time.sleep(0.15)

    print("  [6a] vy- (strafe right, toward obstacle) → vy_safe = -0.10")
    tester.publish_cmd(vx=0.0, vy=-0.5, wz=0.0)
    time.sleep(0.10)
    safe = tester.wait_for_safe()
    results.append(check("vy = -0.10 (DANGER scale 0.20)", safe.linear.y if safe else None, -0.10))
    results.append(check_vz(safe))

    print("  [6b] vy+ (strafe left, away from obstacle) → vy_safe = +0.50 (unaffected)")
    tester.received.clear()
    tester.publish_cmd(vx=0.0, vy=0.5, wz=0.0)
    time.sleep(0.10)
    safe = tester.wait_for_safe()
    results.append(check("vy = +0.50 (left clear, no scaling)", safe.linear.y if safe else None, 0.50))
    results.append(check_vz(safe))

    # Dọn dẹp
    tester.destroy_node()
    safety_node.destroy_node()
    rclpy.shutdown()

    print(f"\n{'='*50}")
    ok = all(results)
    print(f"  Result: {'ALL PASSED' if ok else 'SOME FAILED'}")
    return ok


if __name__ == '__main__':
    sys.exit(0 if run_tests() else 1)
