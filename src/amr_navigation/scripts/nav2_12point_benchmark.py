#!/usr/bin/env python3
"""
nav2_12point_benchmark.py — Gửi robot tới 12 điểm CỐ ĐỊNH (không random) trải
đều 4 quadrant của warehouse_v6_double (32x48m), đo sai số x, y, yaw và
quaternion w giữa goal và vị trí cuối cùng thực tế.

TẠI SAO 12 ĐIỂM CỐ ĐỊNH (không phải random như nav2_random_pose_test.py):
Mục đích ở đây là SO SÁNH giữa các lần tune tham số khác nhau (VD trước/sau
khi đổi cost_scaling_factor, hoặc trước/sau khi bật KeepoutFilter) — nếu mỗi
lần test một bộ điểm khác nhau thì không so sánh công bằng được. 12 điểm bên
dưới đã tính toán tránh xa 8 vùng keepout (xem generate_keepout_mask.py) và
tránh mép tường >=1.5m, y hệt mỗi lần chạy.

Cách chạy (sau khi bringup.launch.py + Nav2 đã active):
    ros2 run amr_navigation nav2_12point_benchmark --output run1.csv

Muốn đổi danh sách điểm: sửa trực tiếp GOAL_POINTS bên dưới.
"""
import argparse
import csv
import math
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

# 12 điểm cố định (label, x, y, yaw_deg) — trải đều 3 điểm/quadrant, đã kiểm
# tra tránh 8 vùng keepout (margin >=0.5m) và tránh tường (margin >=1.5m).
GOAL_POINTS = [
    ("Q1_center",  -8.0,  -4.0,    0.0),
    ("Q1_far",     -1.0, -20.0,   90.0),
    ("Q1_corner", -11.0,  -8.0,  180.0),
    ("Q2_center",  -8.0,   4.0,    0.0),
    ("Q2_far",     -1.0,  20.0,  -90.0),
    ("Q2_corner", -11.0,   8.0,   45.0),
    ("Q3_center",   8.0,  -4.0,  180.0),
    ("Q3_far",      1.0, -20.0,  -90.0),
    ("Q3_corner",  10.0,  -8.0,   90.0),
    ("Q4_center",   8.0,   4.0,   90.0),
    ("Q4_far",      1.0,  20.0,    0.0),
    ("Q4_corner",  10.0,   8.0, -135.0),
]


def yaw_to_quat(yaw_deg):
    yaw = math.radians(yaw_deg)
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)  # (z, w)


def quat_to_yaw_deg(z, w):
    return math.degrees(2.0 * math.atan2(z, w))


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
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
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            elapsed = time.time() - start
            if result_future.done():
                break
            if elapsed > self.args.timeout:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
                return 'TIMEOUT', None, elapsed

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

        for label, x, y, yaw_deg in GOAL_POINTS:
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

        with open(self.args.output, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

        self._print_summary(rows)
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
