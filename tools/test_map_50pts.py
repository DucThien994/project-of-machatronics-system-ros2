"""
test_map_50pts.py — Tự động test 50 điểm ngẫu nhiên trên warehouse_v5.world

Chạy:
    source /opt/ros/humble/setup.bash
    source <ws>/install/setup.bash
    python3 test_map_50pts.py

Yêu cầu: Bringup đầy đủ (Gazebo + SLAM/AMCL + Nav2 active)
"""

import sys
import math
import time
import random
import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry

# ═══════════════════════════════════════════════════════════════════════════════
#  CẤU HÌNH TEST — Thay đổi các biến này để điều chỉnh hành vi test
# ═══════════════════════════════════════════════════════════════════════════════

NUM_GOALS         = 50       # Số điểm test
RANDOM_SEED       = 2026     # Seed ngẫu nhiên (None = random mỗi lần chạy)

# Khoảng cách
MIN_SPACING       = 1.5      # Khoảng cách tối thiểu giữa 2 điểm liên tiếp (m)
MAX_SPACING       = 8.0      # Khoảng cách tối đa (m) — None để tắt

# Thời gian
GOAL_TIMEOUT      = 90.0     # Timeout mỗi goal (s)
SETTLE_TIME       = 2.0      # Chờ robot ổn định sau khi nav2 báo done (s)
                             # Robot mecanum cần ~1-2s để dừng rung và AMCL hội tụ

# Dung sai
POS_TOL           = 0.05     # Dung sai vị trí (m)
YAW_TOL           = 0.08     # Dung sai góc (rad ≈ 8.6°)

# Chế độ góc quay (yaw) cho các điểm test
YAW_MODE          = "random_full"
# Các tùy chọn:
#   "zero"           — tất cả yaw = 0° (hướng forward)
#   "random_cardinal" — ngẫu nhiên từ {0, 45, 90, 135, 180, -135, -90, -45}°
#   "random_full"    — ngẫu nhiên hoàn toàn trong [-180, 180]°

# Lưu kết quả CSV
SAVE_CSV          = True
CSV_PATH          = os.path.join(os.path.dirname(__file__), "results_50pts.csv")

# ═══════════════════════════════════════════════════════════════════════════════
#  CẤU HÌNH BẢN ĐỒ warehouse_v5.world
# ═══════════════════════════════════════════════════════════════════════════════

# Giới hạn bản đồ sau khi trừ clearance tường ngoài
WALL_CLEARANCE    = 0.7      # Khoảng cách an toàn từ tường ngoài (m)
OBS_CLEARANCE     = 1.0      # Khoảng cách an toàn từ obstacle (m)
DIV_CLEARANCE     = 0.6      # Khoảng cách an toàn từ tường phân vùng nội bộ (m)

MAP_X_MIN = -12.0 + WALL_CLEARANCE    # -11.3
MAP_X_MAX =  12.0 - WALL_CLEARANCE    #  11.3
MAP_Y_MIN =  -8.0 + WALL_CLEARANCE    #  -7.3
MAP_Y_MAX =   8.0 - WALL_CLEARANCE    #   7.3

# ── Trụ cột (hình tròn): (cx, cy, radius) ─────────────────────────────────
PILLARS = [
    (2.0,  2.0,  0.20),
    (6.0, -2.0,  0.20),
    (10.0, 0.7,  0.20),
]

# ── Obstacle dạng hộp (rectangles): (cx, cy, half_w, half_h) ──────────────
# Kích thước thực + OBS_CLEARANCE
_C = OBS_CLEARANCE
BOX_OBSTACLES = [
    # North Room
    (-9.5,  6.5,  0.65 + _C, 0.35 + _C),   # box_n1
    (-0.5,  6.0,  0.50 + _C, 0.65 + _C),   # box_n2
    (10.0,  6.8,  0.40 + _C, 0.40 + _C),   # box_n3
    # West Room
    (-10.5, 2.0,  1.00 + _C, 0.25 + _C),   # shelf_w1
    (-7.5, -2.5,  0.50 + _C, 0.50 + _C),   # box_w1
    # Central Zone
    ( 3.5, -3.3,  0.65 + _C, 0.25 + _C),   # shelf_c1 (rotated ~90°)
    ( 8.5,  2.5,  0.50 + _C, 0.50 + _C),   # crate_c1
    # South Storage
    (-9.5, -6.0,  0.65 + _C, 0.50 + _C),   # box_s1
    (-4.0, -6.5,  0.50 + _C, 0.50 + _C),   # box_s2
    ( 4.0, -6.0,  0.40 + _C, 0.80 + _C),   # box_s3
    (11.0, -6.2,  1.00 + _C, 0.25 + _C),   # shelf_s1
]

# ── Tường phân vùng nội bộ: (x_min, x_max, y_min, y_max) ─────────────────
_D = DIV_CLEARANCE
DIVIDER_WALLS = [
    # North divider (y ≈ 4), 3 đoạn
    (-12.0, -8.0,  4.0 - _D, 4.0 + _D),   # segment A
    ( -5.0,  3.0,  4.0 - _D, 4.0 + _D),   # segment B
    (  6.0, 12.0,  4.0 - _D, 4.0 + _D),   # segment C
    # West divider (x ≈ -3), 2 đoạn
    (-3.0 - _D, -3.0 + _D, -4.0, -1.0),   # segment D
    (-3.0 - _D, -3.0 + _D,  2.0,  4.0),   # segment E
    # South divider (y ≈ -4), 3 đoạn
    (-12.0, -2.0, -4.0 - _D, -4.0 + _D),  # segment F
    (  2.0,  7.0, -4.0 - _D, -4.0 + _D),  # segment G
    ( 10.0, 12.0, -4.0 - _D, -4.0 + _D),  # segment H
]

PASS = "\033[92m[PASS]\033[0m"
FAIL = "\033[91m[FAIL]\033[0m"
INFO = "\033[94m[INFO]\033[0m"


# ═══════════════════════════════════════════════════════════════════════════════
#  HÀM KIỂM TRA ĐIỂM HỢP LỆ
# ═══════════════════════════════════════════════════════════════════════════════

def _in_bounds(x, y):
    return MAP_X_MIN <= x <= MAP_X_MAX and MAP_Y_MIN <= y <= MAP_Y_MAX


def _hits_pillar(x, y):
    for (cx, cy, r) in PILLARS:
        if math.hypot(x - cx, y - cy) < r + OBS_CLEARANCE:
            return True
    return False


def _hits_box(x, y):
    for (cx, cy, hw, hh) in BOX_OBSTACLES:
        if abs(x - cx) < hw and abs(y - cy) < hh:
            return True
    return False


def _hits_divider(x, y):
    for (xmin, xmax, ymin, ymax) in DIVIDER_WALLS:
        if xmin <= x <= xmax and ymin <= y <= ymax:
            return True
    return False


def is_valid_point(x, y):
    """Kiểm tra điểm (x,y) có hợp lệ (trong map, không va chạm obstacle/tường)."""
    return (
        _in_bounds(x, y)
        and not _hits_pillar(x, y)
        and not _hits_box(x, y)
        and not _hits_divider(x, y)
    )


def pick_yaw():
    """Chọn yaw (degrees) theo YAW_MODE."""
    if YAW_MODE == "zero":
        return 0.0
    elif YAW_MODE == "random_cardinal":
        cardinals = [0, 45, 90, 135, 180, -135, -90, -45]
        return float(random.choice(cardinals))
    else:  # random_full
        return random.uniform(-180.0, 180.0)


def generate_goals(n=NUM_GOALS, seed=RANDOM_SEED):
    """
    Sinh NUM_GOALS điểm ngẫu nhiên hợp lệ trên bản đồ.
    Đảm bảo MIN_SPACING giữa các điểm liên tiếp.
    Trả về list of (x, y, yaw_deg).
    """
    rng = random.Random(seed)
    goals = []
    attempts = 0
    max_attempts = n * 5000  # tránh vòng lặp vô tận

    while len(goals) < n and attempts < max_attempts:
        attempts += 1
        x = rng.uniform(MAP_X_MIN, MAP_X_MAX)
        y = rng.uniform(MAP_Y_MIN, MAP_Y_MAX)

        if not is_valid_point(x, y):
            continue

        # Kiểm tra khoảng cách với điểm trước
        if goals:
            px, py, _ = goals[-1]
            dist = math.hypot(x - px, y - py)
            if dist < MIN_SPACING:
                continue
            if MAX_SPACING is not None and dist > MAX_SPACING:
                continue

        yaw = pick_yaw()
        goals.append((round(x, 3), round(y, 3), round(yaw, 1)))

    if len(goals) < n:
        print(f"  [WARN] Chỉ sinh được {len(goals)}/{n} điểm sau {attempts} lần thử")
        print(f"         Thử giảm MIN_SPACING ({MIN_SPACING}m) hoặc OBS_CLEARANCE ({OBS_CLEARANCE}m)")

    return goals


# ═══════════════════════════════════════════════════════════════════════════════
#  ROS2 NODE
# ═══════════════════════════════════════════════════════════════════════════════

class MapTester(Node):

    def __init__(self):
        super().__init__('map_50pts_tester')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._amcl_pose = None    # /amcl_pose — ưu tiên
        self._odom_pose = None    # /odom      — fallback

        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)

    def _amcl_cb(self, msg):
        self._amcl_pose = msg.pose.pose

    def _odom_cb(self, msg):
        self._odom_pose = msg.pose.pose

    def _get_pose(self, spin_iters=30):
        """Lấy pose hiện tại, ưu tiên AMCL."""
        for _ in range(spin_iters):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._amcl_pose is not None:
                return self._amcl_pose
            if self._odom_pose is not None:
                return self._odom_pose
        return None

    def _quat_to_yaw(self, pose):
        """Quaternion → yaw (rad). Công thức đầy đủ 3D."""
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w
        return math.atan2(2.0 * (qw * qz + qx * qy),
                          1.0 - 2.0 * (qy * qy + qz * qz))

    def send_goal(self, x, y, yaw_deg):
        """Gửi goal và đợi hoàn thành. Trả về True nếu nav2 báo succeeded."""
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_to_pose action server not available")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        yaw_rad = math.radians(yaw_deg)
        goal_msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        future = self._client.send_goal_async(goal_msg)
        t0 = time.time()
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > 5.0:
                return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2")
            return False

        result_future = goal_handle.get_result_async()
        t0 = time.time()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > GOAL_TIMEOUT:
                self.get_logger().warn(f"Timeout {GOAL_TIMEOUT}s")
                return False

        return True

    def check_pose(self, tx, ty, tyaw_deg):
        """
        Đọc pose sau SETTLE_TIME, kiểm tra position + yaw.
        Trả về (ok, dist_m, dyaw_rad, actual_x, actual_y, actual_yaw_deg).
        """
        # Chờ robot ổn định hoàn toàn
        time.sleep(SETTLE_TIME)

        pose = self._get_pose(spin_iters=30)
        if pose is None:
            return False, float('inf'), float('inf'), None, None, None

        dx   = pose.position.x - tx
        dy   = pose.position.y - ty
        dist = math.hypot(dx, dy)

        yaw_act   = self._quat_to_yaw(pose)
        yaw_tgt   = math.radians(tyaw_deg)
        dyaw      = abs(yaw_act - yaw_tgt)
        if dyaw > math.pi:
            dyaw = 2 * math.pi - dyaw

        ok = dist < POS_TOL and dyaw < YAW_TOL
        return (ok, dist, dyaw,
                pose.position.x, pose.position.y,
                math.degrees(yaw_act))


# ═══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════════════════

def run():
    rclpy.init()
    node = MapTester()
    time.sleep(1.0)  # Đợi node warm-up

    # ── Sinh danh sách điểm ──────────────────────────────────────────────────
    print(f"\n{INFO} Generating {NUM_GOALS} random goals (seed={RANDOM_SEED})...")
    goals = generate_goals(NUM_GOALS, RANDOM_SEED)
    print(f"{INFO} {len(goals)} goals generated. MIN_SPACING={MIN_SPACING}m\n")

    # ── Chuẩn bị CSV ─────────────────────────────────────────────────────────
    csv_rows = []
    csv_fields = [
        "trial", "goal_x", "goal_y", "goal_yaw",
        "actual_x", "actual_y", "actual_yaw",
        "error_pos_cm", "error_yaw_deg",
        "duration_s", "nav_ok", "pose_ok", "result"
    ]

    passed = 0
    nav_failed = 0
    pose_failed = 0

    print("=" * 70)
    print(f"  {'#':>3}  {'Goal (x, y, yaw°)':>25}  {'Pos err':>8}  {'Yaw err':>8}  {'Time':>6}  Result")
    print("=" * 70)

    for i, (gx, gy, gyaw) in enumerate(goals):
        trial_num = i + 1
        print(f"\n[{trial_num:>2}/{len(goals)}] Goal: ({gx:6.2f}, {gy:6.2f}, {gyaw:6.1f}°)")

        t_start   = time.time()
        nav_ok    = node.send_goal(gx, gy, gyaw)
        duration  = time.time() - t_start

        if not nav_ok:
            print(f"  {FAIL}  Nav2 timeout/rejected — skip pose check")
            nav_failed += 1
            csv_rows.append([
                trial_num, gx, gy, gyaw,
                "", "", "", "", "", f"{duration:.1f}",
                0, 0, "NAV_FAIL"
            ])
            continue

        pose_ok, dist, dyaw, ax, ay, ayaw = node.check_pose(gx, gy, gyaw)

        dist_cm  = dist * 100.0 if dist != float('inf') else -1
        dyaw_deg = math.degrees(dyaw) if dyaw != float('inf') else -1

        mark = PASS if pose_ok else FAIL
        print(f"  {mark}  pos={dist_cm:.1f}cm  yaw={dyaw_deg:.2f}°  t={duration:.1f}s")

        if pose_ok:
            passed += 1
        else:
            pose_failed += 1

        result_str = "PASS" if pose_ok else "POSE_FAIL"
        csv_rows.append([
            trial_num, gx, gy, gyaw,
            f"{ax:.4f}" if ax is not None else "",
            f"{ay:.4f}" if ay is not None else "",
            f"{ayaw:.2f}" if ayaw is not None else "",
            f"{dist_cm:.2f}",
            f"{dyaw_deg:.3f}",
            f"{duration:.1f}",
            1, int(pose_ok), result_str
        ])

    # ── Tóm tắt ──────────────────────────────────────────────────────────────
    total = len(goals)
    print(f"\n{'='*70}")
    print(f"  TỔNG KẾT: {passed}/{total} PASS  |  Nav fail: {nav_failed}  |  Pose fail: {pose_failed}")
    rate = 100.0 * passed / total if total > 0 else 0.0
    print(f"  Success rate: {rate:.1f}%")
    print(f"  POS_TOL={POS_TOL*100:.0f}cm  YAW_TOL={math.degrees(YAW_TOL):.1f}°  SETTLE={SETTLE_TIME}s  TIMEOUT={GOAL_TIMEOUT}s")
    print(f"{'='*70}\n")

    # ── Lưu CSV ──────────────────────────────────────────────────────────────
    if SAVE_CSV and csv_rows:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path_ts = CSV_PATH.replace(".csv", f"_{ts}.csv")
        with open(csv_path_ts, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(csv_fields)
            writer.writerows(csv_rows)
        print(f"{INFO} Kết quả đã lưu: {csv_path_ts}")

    node.destroy_node()
    rclpy.shutdown()
    return passed == total


if __name__ == '__main__':
    ok = run()
    sys.exit(0 if ok else 1)
