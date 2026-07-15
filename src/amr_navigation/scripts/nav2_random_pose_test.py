#!/usr/bin/env python3
"""
nav2_random_pose_test.py — Tool do sai so dinh vi/dieu khien cua Nav2.

Muc dich:
  Gui lien tiep N pose goal ngau nhien (vi tri x,y trong vung free-space cua
  global_costmap, goc yaw random deu trong [0, 360) do) toi action server
  'navigate_to_pose' cua Nav2 (bt_navigator). Voi moi goal:
    - Do thoi gian thuc hien (tu luc gui goal den luc ket qua/timeout).
    - Sau khi ket thuc (thanh cong / that bai / timeout), tra cuu TF
      map -> base_footprint de lay pose thuc te xe dat duoc.
    - Tinh sai so x, y, va goc quay (yaw) so voi pose goal da gui.
  Ket qua duoc ghi ra file .csv (ghi tang dan sau moi goal, khong mat du
  lieu neu bi ngat giua chung).

Cach chay (khong can rebuild, chi can source install/setup.bash 1 lan de co
sẵn cac message/action cua Nav2 trong environment):

    python3 nav2_random_pose_test.py --samples 200 --timeout 100 \\
        --x-min -11 --x-max 11 --y-min -7.5 --y-max 7.5 \\
        --output ~/nav2_test_results.csv

Hoac sau khi colcon build, chay qua ros2 run:

    ros2 run amr_navigation nav2_random_pose_test --samples 200

Yeu cau khi chay: Gazebo + robot + Nav2 (map_server/amcl hoac slam_toolbox)
da len day du, TF map->odom->base_footprint da co, action server
'navigate_to_pose' da san sang (bt_navigator).

Cot du lieu trong CSV:
  sample_id, timestamp, goal_x, goal_y, goal_yaw_deg,
  final_x, final_y, final_yaw_deg,
  error_x_m, error_y_m, error_xy_m, error_yaw_deg,
  duration_sec, status

status co the la: SUCCEEDED, ABORTED, CANCELED, TIMEOUT, REJECTED,
ACTION_SERVER_UNAVAILABLE, NO_FREE_SPACE_FOUND, TF_LOOKUP_FAILED.
"""
import argparse
import csv
import math
import os
import random
import sys
import time
from datetime import datetime

import rclpy
from rclpy.utilities import remove_ros_args
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def yaw_deg_to_quat(yaw_deg):
    """Chi xoay quanh truc Z (mat phang) -> quaternion (x,y,z,w)."""
    yaw_rad = math.radians(yaw_deg)
    return 0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


def quat_to_yaw_deg(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def wrap_deg_180(deg):
    """Dua ve khoang (-180, 180]."""
    return (deg + 180.0) % 360.0 - 180.0


def parse_args():
    p = argparse.ArgumentParser(
        description="Test do sai so pose goal Nav2 (random x,y,yaw).")
    p.add_argument('--samples', type=int, default=200,
                    help="So luong pose goal se gui (mac dinh 200).")
    p.add_argument('--timeout', type=float, default=100.0,
                    help="Thoi gian toi da (giay) cho moi lenh (mac dinh 100).")
    p.add_argument('--settle-time', type=float, default=2.0,
                    help="Thoi gian nghi (giay) giua 2 goal lien tiep.")
    p.add_argument('--seed', type=int, default=None,
                    help="Seed cho random, de tai lap ket qua test.")
    # Bien so nay lay theo vi tri 4 buc tuong ngoai cua warehouse_v5.world
    # (wall_north/south o y=+-8.15, wall dai 24.3m -> x~[-12,12]), tru bien an
    # toan ~1m so voi tuong.
    p.add_argument('--x-min', type=float, default=-11.0)
    p.add_argument('--x-max', type=float, default=11.0)
    p.add_argument('--y-min', type=float, default=-7.5)
    p.add_argument('--y-max', type=float, default=7.5)
    p.add_argument('--free-threshold', type=int, default=10,
                    help="Gia tri occupancy (0-100) toi da de coi 1 o la "
                         "free-space (mac dinh 10, thap de tranh vung gradient "
                         "inflation gan vat can).")
    p.add_argument('--max-resample-attempts', type=int, default=200,
                    help="So lan thu lai toi da khi sample 1 vi tri free-space.")
    p.add_argument('--costmap-topic', type=str,
                    default='/global_costmap/costmap')
    p.add_argument('--map-frame', type=str, default='map')
    p.add_argument('--base-frame', type=str, default='base_footprint')
    p.add_argument('--action-name', type=str, default='navigate_to_pose')
    p.add_argument('--output', type=str, default=None,
                    help="Duong dan file .csv xuat ra (mac dinh: "
                         "~/nav2_pose_test_<timestamp>.csv)")
    # loai bo cac argument rieng cua ROS2 (--ros-args ...) truoc khi argparse xu ly
    argv = remove_ros_args(sys.argv)[1:]
    return p.parse_args(argv)


class RandomPoseTester(Node):

    def __init__(self, args):
        super().__init__('nav2_random_pose_test')
        self.args = args
        self.costmap = None
        self._action_client = ActionClient(
            self, NavigateToPose, args.action_name)
        self.create_subscription(
            OccupancyGrid, args.costmap_topic, self._costmap_cb, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _costmap_cb(self, msg):
        self.costmap = msg

    def wait_for_costmap(self, timeout=30.0):
        self.get_logger().info(
            f"Cho du lieu costmap tren '{self.args.costmap_topic}'...")
        start = time.time()
        while rclpy.ok() and self.costmap is None:
            rclpy.spin_once(self, timeout_sec=0.5)
            if time.time() - start > timeout:
                raise RuntimeError(
                    f"Khong nhan duoc costmap sau {timeout}s - kiem tra "
                    f"Nav2/global_costmap da chay chua.")
        self.get_logger().info("Da co costmap.")

    def is_free(self, x, y):
        msg = self.costmap
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        col = int((x - ox) / res)
        row = int((y - oy) / res)
        if col < 0 or col >= msg.info.width or row < 0 or row >= msg.info.height:
            return False
        idx = row * msg.info.width + col
        val = msg.data[idx]
        return 0 <= val <= self.args.free_threshold

    def sample_goal(self):
        a = self.args
        for _ in range(a.max_resample_attempts):
            x = random.uniform(a.x_min, a.x_max)
            y = random.uniform(a.y_min, a.y_max)
            if self.is_free(x, y):
                yaw_deg = round(random.uniform(0.0, 360.0), 2)
                return x, y, yaw_deg
        return None

    def get_current_pose(self, retries=10, delay=0.2):
        for _ in range(retries):
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.args.map_frame, self.args.base_frame, Time())
                t = tf.transform.translation
                q = tf.transform.rotation
                yaw_deg = quat_to_yaw_deg(q.x, q.y, q.z, q.w)
                return t.x, t.y, yaw_deg
            except Exception:
                rclpy.spin_once(self, timeout_sec=delay)
        return None

    def send_goal_and_wait(self, x, y, yaw_deg):
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            return 'ACTION_SERVER_UNAVAILABLE', 0.0

        qx, qy, qz, qw = yaw_deg_to_quat(yaw_deg)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self.args.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        send_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return 'REJECTED', 0.0

        result_future = goal_handle.get_result_async()
        start = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if result_future.done():
                status_code = result_future.result().status
                duration = time.time() - start
                if status_code == GoalStatus.STATUS_SUCCEEDED:
                    return 'SUCCEEDED', duration
                elif status_code == GoalStatus.STATUS_ABORTED:
                    return 'ABORTED', duration
                elif status_code == GoalStatus.STATUS_CANCELED:
                    return 'CANCELED', duration
                else:
                    return f'UNKNOWN_STATUS_{status_code}', duration
            if time.time() - start > self.args.timeout:
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(
                    self, cancel_future, timeout_sec=5.0)
                return 'TIMEOUT', time.time() - start
        # An toan: neu rclpy.ok() tra ve False giua chung (VD Ctrl+C) ma chua
        # kip return o tren, van tra ve 1 gia tri hop le thay vi None (tranh
        # loi unpack "NoneType is not iterable" o noi goi ham).
        return 'NODE_SHUTDOWN', time.time() - start

    def run(self):
        a = self.args
        self.wait_for_costmap()

        output_path = a.output or os.path.expanduser(
            f"~/nav2_pose_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        self.get_logger().info(f"Ket qua se ghi vao: {output_path}")

        fieldnames = [
            'sample_id', 'timestamp', 'goal_x', 'goal_y', 'goal_yaw_deg',
            'final_x', 'final_y', 'final_yaw_deg',
            'error_x_m', 'error_y_m', 'error_xy_m', 'error_yaw_deg',
            'duration_sec', 'status',
        ]

        with open(output_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            f.flush()

            for i in range(a.samples):
                goal = self.sample_goal()
                if goal is None:
                    self.get_logger().warn(
                        f"[{i+1}/{a.samples}] Khong tim duoc vi tri free-space "
                        f"sau {a.max_resample_attempts} lan thu - bo qua mau nay.")
                    writer.writerow({
                        'sample_id': i, 'timestamp': datetime.now().isoformat(),
                        'goal_x': '', 'goal_y': '', 'goal_yaw_deg': '',
                        'final_x': '', 'final_y': '', 'final_yaw_deg': '',
                        'error_x_m': '', 'error_y_m': '', 'error_xy_m': '',
                        'error_yaw_deg': '', 'duration_sec': '',
                        'status': 'NO_FREE_SPACE_FOUND',
                    })
                    f.flush()
                    continue

                gx, gy, gyaw = goal
                self.get_logger().info(
                    f"[{i+1}/{a.samples}] Goal: x={gx:.2f} y={gy:.2f} "
                    f"yaw={gyaw:.2f} deg")

                status, duration = self.send_goal_and_wait(gx, gy, gyaw)
                time.sleep(0.3)  # cho TF on dinh truoc khi doc pose cuoi
                pose = self.get_current_pose()

                if pose is None:
                    final_x = final_y = final_yaw = ''
                    err_x = err_y = err_xy = err_yaw = ''
                    if status == 'SUCCEEDED':
                        status = 'TF_LOOKUP_FAILED'
                else:
                    final_x, final_y, final_yaw = pose
                    err_x = final_x - gx
                    err_y = final_y - gy
                    err_xy = math.hypot(err_x, err_y)
                    err_yaw = wrap_deg_180(final_yaw - gyaw)

                self.get_logger().info(
                    f"    -> status={status} duration={duration:.1f}s "
                    f"err_xy={err_xy if err_xy != '' else 'N/A'}")

                writer.writerow({
                    'sample_id': i,
                    'timestamp': datetime.now().isoformat(),
                    'goal_x': round(gx, 4), 'goal_y': round(gy, 4),
                    'goal_yaw_deg': gyaw,
                    'final_x': round(final_x, 4) if final_x != '' else '',
                    'final_y': round(final_y, 4) if final_y != '' else '',
                    'final_yaw_deg': round(final_yaw, 2) if final_yaw != '' else '',
                    'error_x_m': round(err_x, 4) if err_x != '' else '',
                    'error_y_m': round(err_y, 4) if err_y != '' else '',
                    'error_xy_m': round(err_xy, 4) if err_xy != '' else '',
                    'error_yaw_deg': round(err_yaw, 2) if err_yaw != '' else '',
                    'duration_sec': round(duration, 2),
                    'status': status,
                })
                f.flush()

                time.sleep(a.settle_time)

        self._print_summary(output_path, fieldnames)

    def _print_summary(self, path, fieldnames):
        rows = []
        with open(path, newline='') as f:
            reader = csv.DictReader(f)
            rows = list(reader)

        succeeded = [r for r in rows if r['status'] == 'SUCCEEDED'
                     and r['error_xy_m'] != '']
        n = len(rows)
        n_ok = len(succeeded)
        self.get_logger().info(
            f"===== TONG KET: {n_ok}/{n} thanh cong (co du lieu sai so) =====")
        if n_ok > 0:
            err_xy = [float(r['error_xy_m']) for r in succeeded]
            err_yaw = [abs(float(r['error_yaw_deg'])) for r in succeeded]
            dur = [float(r['duration_sec']) for r in succeeded]
            self.get_logger().info(
                f"error_xy (m): mean={sum(err_xy)/n_ok:.4f} "
                f"max={max(err_xy):.4f} min={min(err_xy):.4f}")
            self.get_logger().info(
                f"error_yaw (deg, abs): mean={sum(err_yaw)/n_ok:.2f} "
                f"max={max(err_yaw):.2f} min={min(err_yaw):.2f}")
            self.get_logger().info(
                f"duration (s): mean={sum(dur)/n_ok:.2f} "
                f"max={max(dur):.2f} min={min(dur):.2f}")
        self.get_logger().info(f"File ket qua: {path}")


def main():
    rclpy.init()
    args = parse_args()
    if args.seed is not None:
        random.seed(args.seed)
    node = RandomPoseTester(args)
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Bi ngat - du lieu da ghi den thoi diem nay van con trong file CSV.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
