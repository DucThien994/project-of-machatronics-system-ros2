"""
MODULE TEST 7 — test_nav_goal.py
Test điều hướng Nav2: gửi goal → verify robot đến nơi trong tolerance.

Thuật toán kiểm tra:
  1. NavFn/Dijkstra  — global path planning trên occupancy grid
  2. MPPI controller — sampling-based optimal control (2000 traj × 56 steps)
  3. AMCL           — particle filter localization (saved-map mode)
  4. velocity_smoother — acceleration limiter

Metric: position error < 0.10m, yaw error < 0.15 rad

Cần: Bringup đầy đủ (Gazebo + SLAM/AMCL + Nav2 active)
Chạy:
  source /opt/ros/humble/setup.bash
  python3 tests/integration/test_nav_goal.py
"""

import sys, math, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry

PASS = "\033[92m[PASS]\033[0m"
FAIL = "\033[91m[FAIL]\033[0m"

# Các waypoint test (x, y, yaw_deg) — trong warehouse_v5.world
TEST_GOALS = [
    (2.0,  0.0,   0.0, "Forward 2m"),
    (2.0,  2.0,  90.0, "Forward+left"),
    (0.0,  2.0,  90.0, "Lateral strafe left 2m"),   # test vy direction (Mecanum)
    (0.0,  0.0,   0.0, "Return to origin"),
    (4.0, -1.0, -45.0, "Right quadrant"),
]

# Tolerance
POS_TOL = 0.10   # m
YAW_TOL = 0.15   # rad (~8.6°)
TIMEOUT = 60.0   # s per goal (tăng từ 30s — đủ cho path dài hơn)


class NavTester(Node):

    def __init__(self):
        super().__init__('nav_goal_tester')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._amcl_pose = None   # /amcl_pose — ưu tiên (saved-map mode)
        self._odom_pose = None   # /odom     — fallback (SLAM mode)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

    def _amcl_cb(self, msg):
        self._amcl_pose = msg.pose.pose   # geometry_msgs/Pose

    def _odom_cb(self, msg):
        self._odom_pose = msg.pose.pose

    def _get_current_pose(self):
        """Ưu tiên AMCL (độ chính xác cao hơn trong saved-map mode), fallback odom."""
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._amcl_pose is not None:
                return self._amcl_pose
            if self._odom_pose is not None:
                return self._odom_pose
        return None

    def send_goal(self, x, y, yaw_deg):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_to_pose action server not available")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id    = 'map'
        goal_msg.pose.header.stamp       = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x    = x
        goal_msg.pose.pose.position.y    = y
        yaw_rad = math.radians(yaw_deg)
        goal_msg.pose.pose.orientation.z = math.sin(yaw_rad / 2)
        goal_msg.pose.pose.orientation.w = math.cos(yaw_rad / 2)

        self.get_logger().info(f"Sending goal: ({x:.1f}, {y:.1f}, {yaw_deg:.0f}°)")
        future = self._client.send_goal_async(goal_msg)

        t0 = time.time()
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > 5.0:
                return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        t0 = time.time()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > TIMEOUT:
                self.get_logger().warn(f"Timeout after {TIMEOUT}s")
                return False

        return True

    def check_position(self, target_x, target_y, target_yaw_deg):
        pose = self._get_current_pose()
        if pose is None:
            return False, float('inf'), float('inf')

        dx   = pose.position.x - target_x
        dy   = pose.position.y - target_y
        dist = math.sqrt(dx*dx + dy*dy)

        # Quaternion → yaw (công thức đầy đủ, đúng khi qx/qy ≠ 0)
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w
        yaw_actual = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
        yaw_target = math.radians(target_yaw_deg)
        dyaw = abs(yaw_actual - yaw_target)
        if dyaw > math.pi:
            dyaw = 2*math.pi - dyaw

        ok = dist < POS_TOL and dyaw < YAW_TOL
        return ok, dist, dyaw


def run_tests():
    rclpy.init()
    tester = NavTester()
    time.sleep(1.0)

    results = []

    for (x, y, yaw, desc) in TEST_GOALS:
        print(f"\n[GOAL] {desc}: ({x}, {y}, {yaw}°)")
        ok_nav = tester.send_goal(x, y, yaw)
        if not ok_nav:
            print(f"  {FAIL}  Navigation failed or timed out")
            results.append(False)
            continue

        time.sleep(0.5)  # Dừng lại trước khi đo
        ok_pos, dist, dyaw = tester.check_position(x, y, yaw)
        print(f"  Position error: {dist*100:.1f}cm (limit {POS_TOL*100:.0f}cm)")
        print(f"  Yaw error:      {math.degrees(dyaw):.2f}° (limit {math.degrees(YAW_TOL):.1f}°)")
        mark = PASS if ok_pos else FAIL
        print(f"  {mark}  {desc}")
        results.append(ok_pos)

    tester.destroy_node()
    rclpy.shutdown()

    print(f"\n{'='*50}")
    passed = sum(results)
    total  = len(results)
    ok = passed == total
    print(f"  Result: {passed}/{total} goals PASSED")
    return ok


if __name__ == '__main__':
    sys.exit(0 if run_tests() else 1)
