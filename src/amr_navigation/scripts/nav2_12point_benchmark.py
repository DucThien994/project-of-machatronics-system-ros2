#!/usr/bin/env python3
"""
nav2_12point_benchmark.py — Gửi robot tới 12 điểm NGẪU NHIÊN (vị trí lẫn góc
quay random mỗi lần chạy) trong warehouse_v6_double (32x48m), đo sai số x, y,
yaw và quaternion w giữa goal và vị trí cuối cùng thực tế.

FIX (theo yêu cầu): trước đây dùng 12 điểm cố định để so sánh công bằng giữa
các lần tune — nay đổi lại thành random thật (x, y, yaw đều random mỗi lần),
nhưng vẫn đảm bảo:
  - Tránh xa 8 vùng keepout (quanh ShelfF/ShelfE/ShelfD) với margin lớn hơn
    nhiều so với bán kính vật lý của vùng đó (--keepout-margin, mặc định
    1.5m) — vì xe khó luồn lách khi goal ở sát rìa kệ hàng (đường đi hẹp,
    MPPI dễ dao động/mất tốc độ ở đó).
  - Tránh sát tường (--wall-margin, mặc định 1.5m).
  - Các điểm không bắt buộc gần nhau, có thể cách xa tùy random, chỉ ép tối
    thiểu --min-separation (mặc định 2.0m) để không ra 2 điểm trùng nhau.

Cách chạy (sau khi bringup.launch.py + Nav2 đã active):
    ros2 run amr_navigation nav2_12point_benchmark --output run1.csv
    ros2 run amr_navigation nav2_12point_benchmark --seed 42   # lặp lại được
"""
import argparse
import csv
import math
import random
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.utilities import remove_ros_args
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import tf2_ros

# Biên phòng (world warehouse_v6_double.world): x in [-16,16], y in [-24,24].
ROOM_X = (-16.0, 16.0)
ROOM_Y = (-24.0, 24.0)

# 8 vùng keepout (center_x, center_y, width, height) — khớp CHÍNH XÁC với
# KEEPOUT_ZONES trong generate_keepout_mask.py và 8 model "keepout_zone_*"
# trong warehouse_v6_double.world.
KEEPOUT_ZONES = [
    (-13.795143, -12.956635, 2.5, 2.5),
    (-13.795143,  11.043365, 2.5, 2.5),
    (  2.204857, -12.956635, 2.5, 2.5),
    (  2.204857,  11.043365, 2.5, 2.5),
    ( -3.25,     -16.05,     3.1, 11.3),
    ( -3.25,       7.95,     3.1, 11.3),
    ( 12.75,     -16.05,     3.1, 11.3),
    ( 12.75,       7.95,     3.1, 11.3),
]


def too_close_to_keepout(x, y, margin):
    for cx, cy, w, h in KEEPOUT_ZONES:
        half_w, half_h = w / 2.0 + margin, h / 2.0 + margin
        if (cx - half_w) <= x <= (cx + half_w) and (cy - half_h) <= y <= (cy + half_h):
            return True
    return False


def sample_random_pose(rng, existing, wall_margin, keepout_margin, min_sep, max_attempts=2000):
    x_lo, x_hi = ROOM_X[0] + wall_margin, ROOM_X[1] - wall_margin
    y_lo, y_hi = ROOM_Y[0] + wall_margin, ROOM_Y[1] - wall_margin
    for _ in range(max_attempts):
        x = rng.uniform(x_lo, x_hi)
        y = rng.uniform(y_lo, y_hi)
        if too_close_to_keepout(x, y, keepout_margin):
            continue
        if any(math.hypot(x - ex, y - ey) < min_sep for ex, ey in existing):
            continue
        yaw_deg = rng.uniform(-180.0, 180.0)
        return x, y, yaw_deg
    raise RuntimeError(
        "Không tìm được điểm random hợp lệ sau nhiều lần thử — "
        "wall_margin/keepout_margin/min_separation đang đặt quá lớn so với "
        "diện tích phòng còn trống.")


def yaw_to_quat(yaw_deg):
    yaw = math.radians(yaw_deg)
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)  # (z, w)


def quat_to_yaw_deg(z, w):
    return math.degrees(2.0 * math.atan2(z, w))


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--n-points', type=int, default=12, help="Số điểm random (mặc định 12)")
    p.add_argument('--seed', type=int, default=None,
                   help="Seed random — đặt số cụ thể để lặp lại đúng bộ điểm cũ")
    p.add_argument('--wall-margin', type=float, default=1.5,
                   help="Khoảng cách tối thiểu tới tường (m)")
    p.add_argument('--keepout-margin', type=float, default=1.5,
                   help="Khoảng cách tối thiểu tới rìa vùng keepout/kiện hàng (m)")
    p.add_argument('--min-separation', type=float, default=2.0,
                   help="Khoảng cách tối thiểu giữa 2 điểm goal (m)")
    p.add_argument('--timeout', type=float, default=100.0, help="Giây/điểm (mặc định 100s)")
    p.add_argument('--settle-time', type=float, default=1.5,
                   help="Chờ ổn định TF sau khi goal SUCCEEDED trước khi đo (s)")
    p.add_argument('--map-frame', type=str, default='map')
    p.add_argument('--base-frame', type=str, default='base_footprint')
    p.add_argument('--output', type=str, default='nav2_12point_result.csv')
    argv = remove_ros_args(sys.argv)[1:]
    return p.parse_args(argv)


class TwelvePointBenchmark(Node):
    def __init__(self, args):
        super().__init__('nav2_12point_benchmark')
        self.args = args
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def get_current_pose(self, retries=10, delay=0.2):
        for _ in range(retries):
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.args.map_frame, self.args.base_frame, rclpy.time.Time())
                t = tf.transform.translation
                q = tf.transform.rotation
                return t.x, t.y, q.z, q.w
            except Exception:
                rclpy.spin_once(self, timeout_sec=delay)
        return None

    def send_goal_and_wait(self, x, y, yaw_deg):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_to_pose action server không sẵn sàng!")
            return 'SERVER_UNAVAILABLE', None, 0.0

        qz, qw = yaw_to_quat(yaw_deg)
        goal = PoseStamped()
        goal.header.frame_id = self.args.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        msg = NavigateToPose.Goal()
        msg.pose = goal

        send_future = self.client.send_goal_async(msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            # Bị từ chối ngay lập tức -> thường do goal nằm trong vùng lethal
            # (obstacle/keepout) hoặc ComputePathToPose không tìm được đường.
            return 'REJECTED', None, 0.0

        result_future = goal_handle.get_result_async()
        start = time.time()
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.2)
                elapsed = time.time() - start
                if result_future.done():
                    break
                if elapsed > self.args.timeout:
                    cancel_future = goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
                    return 'TIMEOUT', None, elapsed
        except KeyboardInterrupt:
            # Ctrl+C giữa lúc đang chờ goal: hủy goal hiện tại trên Nav2 rồi
            # ném lại để run() dừng vòng lặp và lưu các điểm đã hoàn thành.
            try:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            except Exception:
                pass
            raise

        elapsed = time.time() - start
        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            status_str = 'SUCCEEDED'
        elif status == GoalStatus.STATUS_CANCELED:
            status_str = 'CANCELED'
        else:
            status_str = 'FAILED'

        time.sleep(self.args.settle_time)
        final_pose = self.get_current_pose()
        return status_str, final_pose, elapsed

    def run(self):
        fieldnames = ['label', 'goal_x', 'goal_y', 'goal_yaw_deg', 'goal_w',
                      'final_x', 'final_y', 'final_yaw_deg', 'final_w',
                      'error_x_m', 'error_y_m', 'error_xy_m', 'error_yaw_deg',
                      'error_w', 'duration_sec', 'status']
        rows = []

        rng = random.Random(self.args.seed)
        seed_used = self.args.seed if self.args.seed is not None else 'None (fully random)'
        self.get_logger().info(f"Sinh {self.args.n_points} điểm random (seed={seed_used})...")

        existing = []
        goal_points = []
        for i in range(self.args.n_points):
            x, y, yaw_deg = sample_random_pose(
                rng, existing, self.args.wall_margin,
                self.args.keepout_margin, self.args.min_separation)
            existing.append((x, y))
            goal_points.append((f"P{i+1}", round(x, 3), round(y, 3), round(yaw_deg, 1)))
            self.get_logger().info(
                f"  P{i+1}: x={x:.2f} y={y:.2f} yaw={yaw_deg:.1f}deg")

        f = open(self.args.output, 'w', newline='')
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        f.flush()

        interrupted = False
        try:
            for label, x, y, yaw_deg in goal_points:
                self.get_logger().info(f"--- Goal '{label}': ({x},{y}, yaw={yaw_deg} deg) ---")
                goal_qz, goal_qw = yaw_to_quat(yaw_deg)
                status, final_pose, elapsed = self.send_goal_and_wait(x, y, yaw_deg)

                row = {
                    'label': label, 'goal_x': x, 'goal_y': y,
                    'goal_yaw_deg': yaw_deg, 'goal_w': round(goal_qw, 5),
                    'duration_sec': round(elapsed, 2), 'status': status,
                }
                if final_pose is not None:
                    fx, fy, fqz, fqw = final_pose
                    final_yaw = quat_to_yaw_deg(fqz, fqw)
                    err_x = fx - x
                    err_y = fy - y
                    err_yaw = ((final_yaw - yaw_deg + 180) % 360) - 180  # wrap [-180,180]
                    row.update({
                        'final_x': round(fx, 4), 'final_y': round(fy, 4),
                        'final_yaw_deg': round(final_yaw, 2), 'final_w': round(fqw, 5),
                        'error_x_m': round(err_x, 4), 'error_y_m': round(err_y, 4),
                        'error_xy_m': round(math.hypot(err_x, err_y), 4),
                        'error_yaw_deg': round(err_yaw, 2),
                        'error_w': round(fqw - goal_qw, 5),
                    })
                    self.get_logger().info(
                        f"    -> {status} | error_xy={row['error_xy_m']}m "
                        f"error_yaw={row['error_yaw_deg']}deg t={row['duration_sec']}s")
                else:
                    row.update({k: '' for k in
                                ['final_x', 'final_y', 'final_yaw_deg', 'final_w',
                                 'error_x_m', 'error_y_m', 'error_xy_m',
                                 'error_yaw_deg', 'error_w']})
                    self.get_logger().warn(f"    -> {status} (không lấy được TF cuối)")
                rows.append(row)
                writer.writerow(row)
                f.flush()
        except KeyboardInterrupt:
            interrupted = True
            self.get_logger().warn(
                f"Nhận Ctrl+C — dừng lại, đã lưu {len(rows)}/{len(goal_points)} điểm.")
        finally:
            f.close()

        self._print_summary(rows)
        if interrupted:
            self.get_logger().info(
                f"Đã lưu (một phần, {len(rows)}/{len(goal_points)} điểm): {self.args.output}")
        else:
            self.get_logger().info(f"Đã lưu: {self.args.output}")

    def _print_summary(self, rows):
        ok = [r for r in rows if r['status'] == 'SUCCEEDED' and r['error_xy_m'] != '']
        print(f"\n===== TÓM TẮT ({len(ok)}/{len(rows)} SUCCEEDED) =====")
        for r in rows:
            print(f"  {r['label']:12s} status={r['status']:10s} "
                  f"error_xy={r.get('error_xy_m','-'):>7} m  "
                  f"error_yaw={r.get('error_yaw_deg','-'):>7} deg  "
                  f"error_w={r.get('error_w','-'):>8}  t={r['duration_sec']}s")
        if ok:
            mean_xy = sum(r['error_xy_m'] for r in ok) / len(ok)
            mean_yaw = sum(abs(r['error_yaw_deg']) for r in ok) / len(ok)
            print(f"\nmean error_xy  = {mean_xy:.4f} m")
            print(f"mean |error_yaw| = {mean_yaw:.2f} deg")


def main():
    args = parse_args()
    rclpy.init()
    node = TwelvePointBenchmark(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
