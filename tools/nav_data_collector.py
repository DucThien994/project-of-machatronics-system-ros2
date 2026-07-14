#!/usr/bin/env python3
"""
nav_data_collector.py  —  Thu thập dữ liệu điều hướng → CSV chuẩn MATLAB
──────────────────────────────────────────────────────────────────────────────
Cách hoạt động:
  1. Nhận goal từ RViz2 (click "2D Nav Goal")  →  /goal_pose
  2. Gửi goal tới Nav2 action server, chờ kết quả
  3. Đọc pose thực tế từ /amcl_pose (saved-map) hoặc /odom (SLAM)
  4. Ghi 1 dòng vào CSV ngay sau khi robot dừng

Định dạng CSV (MATLAB-friendly):
  - Tất cả cột là số (không có chuỗi văn bản trộn lẫn)
  - Dấu phân cách: dấu phẩy
  - Kết quả: succeeded=1 / failed=0
  - Yaw đơn vị: độ (degree) — dễ đọc trong MATLAB

Cách chạy:
  source /opt/ros/humble/setup.bash
  python3 tools/nav_data_collector.py
  python3 tools/nav_data_collector.py --output ~/data_amr.csv
"""

import argparse, csv, math, os, threading, time
from datetime import datetime

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus

# ── Đường dẫn mặc định ────────────────────────────────────────────────────────
DEFAULT_OUTPUT = os.path.expanduser('~/amr_nav_data.csv')

# ── Header CSV — tên cột theo chuẩn MATLAB (không dấu, không space) ──────────
CSV_HEADER = [
    'trial',            # Số thứ tự lần thử                    [int]
    'time_s',           # Unix timestamp                         [float]
    'goal_x',           # Tọa độ X mục tiêu                    [m]
    'goal_y',           # Tọa độ Y mục tiêu                    [m]
    'goal_yaw',         # Góc quay mục tiêu                    [deg]
    'actual_x',         # Tọa độ X thực tế khi dừng            [m]
    'actual_y',         # Tọa độ Y thực tế khi dừng            [m]
    'actual_yaw',       # Góc quay thực tế khi dừng            [deg]
    'error_pos_cm',     # Sai số vị trí = sqrt(dx²+dy²) × 100  [cm]
    'error_yaw_deg',    # Sai số góc |actual_yaw - goal_yaw|   [deg]
    'duration_s',       # Thời gian hoàn thành goal             [s]
    'succeeded',        # 1 = thành công, 0 = thất bại          [bool]
]

SETTLE_TIME = 0.5   # giây chờ sau khi nav xong để pose ổn định


# ── Tiện ích ──────────────────────────────────────────────────────────────────

def quat_to_yaw(qx, qy, qz, qw) -> float:
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def wrap_pi(angle: float) -> float:
    """Wrap về [-π, π]"""
    while angle >  math.pi: angle -= 2 * math.pi
    while angle < -math.pi: angle += 2 * math.pi
    return angle


# ── Node chính ────────────────────────────────────────────────────────────────

class NavDataCollector(Node):

    def __init__(self, output_path: str):
        super().__init__('nav_data_collector')
        self._output     = output_path
        self._trial      = 0
        self._lock       = threading.Lock()
        self._pose       = (0.0, 0.0, 0.0)   # (x, y, yaw_rad)
        self._pose_src   = 'none'
        self._busy       = False              # chặn goal mới trong khi đang nav

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribers
        self.create_subscription(PoseStamped,
            '/goal_pose',   self._goal_cb,  10)
        self.create_subscription(PoseWithCovarianceStamped,
            '/amcl_pose',   self._amcl_cb,  10)
        self.create_subscription(Odometry,
            '/odom',        self._odom_cb,  10)

        self._init_csv()
        self.get_logger().info(
            f'\n{"="*60}\n'
            f'  NavDataCollector khởi động\n'
            f'  Output : {self._output}\n'
            f'  Nguồn  : /amcl_pose (ưu tiên) hoặc /odom\n'
            f'  Bắt đầu: Click "2D Nav Goal" trong RViz2\n'
            f'{"="*60}')

    # ── CSV ───────────────────────────────────────────────────────────────────

    def _init_csv(self):
        exists = os.path.isfile(self._output)
        self._csv_file = open(self._output, 'a', newline='')
        self._writer   = csv.writer(self._csv_file)
        if not exists:
            self._writer.writerow(CSV_HEADER)
            self._csv_file.flush()
            self.get_logger().info(f'Tạo mới: {self._output}')
        else:
            # Đọc trial cuối để tiếp tục đánh số
            with open(self._output) as f:
                rows = list(csv.reader(f))
            if len(rows) > 1:
                try:
                    self._trial = int(rows[-1][0])
                except Exception:
                    pass
            self.get_logger().info(
                f'Append vào file cũ (trial cuối: {self._trial}): {self._output}')

    def _write_row(self, trial, t_unix, gx, gy, gyaw_rad,
                   ax, ay, ayaw_rad, duration, succeeded):
        ep_cm    = math.sqrt((ax-gx)**2 + (ay-gy)**2) * 100.0
        ey_deg   = abs(math.degrees(wrap_pi(ayaw_rad - gyaw_rad)))
        row = [
            trial,
            round(t_unix, 3),
            round(gx,  4),
            round(gy,  4),
            round(math.degrees(gyaw_rad), 2),
            round(ax,  4),
            round(ay,  4),
            round(math.degrees(ayaw_rad), 2),
            round(ep_cm,  2),
            round(ey_deg, 2),
            round(duration, 2),
            1 if succeeded else 0,
        ]
        with self._lock:
            self._writer.writerow(row)
            self._csv_file.flush()
        return ep_cm, ey_deg

    # ── Pose callbacks ────────────────────────────────────────────────────────

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        yaw = quat_to_yaw(p.orientation.x, p.orientation.y,
                          p.orientation.z, p.orientation.w)
        with self._lock:
            self._pose     = (p.position.x, p.position.y, yaw)
            self._pose_src = 'amcl'

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose
        yaw = quat_to_yaw(p.orientation.x, p.orientation.y,
                          p.orientation.z, p.orientation.w)
        with self._lock:
            if self._pose_src != 'amcl':
                self._pose     = (p.position.x, p.position.y, yaw)
                self._pose_src = 'odom'

    # ── Goal callback ─────────────────────────────────────────────────────────

    def _goal_cb(self, msg: PoseStamped):
        if self._busy:
            self.get_logger().warn('Đang thực hiện goal trước, bỏ qua goal mới.')
            return
        gx   = msg.pose.position.x
        gy   = msg.pose.position.y
        gyaw = quat_to_yaw(msg.pose.orientation.x, msg.pose.orientation.y,
                            msg.pose.orientation.z, msg.pose.orientation.w)
        self.get_logger().info(
            f'[Goal] x={gx:.3f} y={gy:.3f} yaw={math.degrees(gyaw):.1f}°')
        threading.Thread(
            target=self._run_nav, args=(gx, gy, gyaw), daemon=True).start()

    # ── Navigation runner ─────────────────────────────────────────────────────

    def _run_nav(self, gx, gy, gyaw):
        self._busy = True
        try:
            if not self._action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Nav2 action server không sẵn sàng!')
                return

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id    = 'map'
            goal_msg.pose.header.stamp       = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x    = gx
            goal_msg.pose.pose.position.y    = gy
            goal_msg.pose.pose.orientation.z = math.sin(gyaw / 2)
            goal_msg.pose.pose.orientation.w = math.cos(gyaw / 2)

            t_unix = time.time()
            t_start = t_unix

            send_future = self._action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
            if not send_future.done() or not send_future.result().accepted:
                self.get_logger().error('Goal bị từ chối bởi Nav2.')
                return

            result_future = send_future.result().get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
            duration  = time.time() - t_start

            succeeded = (result_future.done() and
                         result_future.result().status == GoalStatus.STATUS_SUCCEEDED)

            time.sleep(SETTLE_TIME)

            with self._lock:
                ax, ay, ayaw = self._pose
                src = self._pose_src

            self._trial += 1
            ep_cm, ey_deg = self._write_row(
                self._trial, t_unix, gx, gy, gyaw, ax, ay, ayaw, duration, succeeded)

            status_str = 'SUCCEEDED' if succeeded else 'FAILED'
            self.get_logger().info(
                f'\n[Ghi Trial #{self._trial}]  {status_str}  [src={src}]\n'
                f'  Goal  : ({gx:.3f}, {gy:.3f}, {math.degrees(gyaw):.1f}°)\n'
                f'  Thực  : ({ax:.3f}, {ay:.3f}, {math.degrees(ayaw):.1f}°)\n'
                f'  Sai số: pos={ep_cm:.1f}cm  yaw={ey_deg:.2f}°  t={duration:.1f}s')
        finally:
            self._busy = False

    def destroy_node(self):
        self._csv_file.close()
        super().destroy_node()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', '-o', default=DEFAULT_OUTPUT)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = NavDataCollector(output_path=args.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'Dừng. File: {args.output}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
