#!/usr/bin/env python3
"""
Script đo sai số góc quay giữa 2 vị trí — 8 lần thử
Gửi goal navigate_to_pose luân phiên giữa vị trí A và B,
so sánh yaw mục tiêu (goal) với yaw thực tế đạt được (AMCL) khi robot dừng.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math, time, csv, os, statistics

AMCL_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

POSE_A = dict(label='A', x=4.424146956836005, y=-3.247192610053417, yaw=math.radians(90))
POSE_B = dict(label='B', x=-4.0, y=2.0, yaw=math.radians(-90))
N_TRIALS = 8
GOAL_TIMEOUT_SEC = 120.0
OUTPUT_CSV = os.path.expanduser('~/final-ros2/amr_ver6.0/logs/rotation_error_log.csv')


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def norm_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class RotationErrorTest(Node):
    def __init__(self):
        super().__init__('rotation_error_test')
        self.amcl_pose = None
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, AMCL_QOS)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def amcl_cb(self, msg):
        self.amcl_pose = msg.pose.pose

    def wait_for_amcl(self):
        print("Đang chờ /amcl_pose ...")
        while self.amcl_pose is None:
            rclpy.spin_once(self, timeout_sec=0.5)

    def send_goal(self, target):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target['x']
        goal_msg.pose.pose.position.y = target['y']
        qx, qy, qz, qw = yaw_to_quat(target['yaw'])
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.client.wait_for_server()
        send_future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=GOAL_TIMEOUT_SEC)
        return result_future.result() is not None


def main():
    rclpy.init()
    node = RotationErrorTest()
    node.wait_for_amcl()

    targets = [POSE_B, POSE_A] * (N_TRIALS // 2)
    rows = []

    with open(OUTPUT_CSV, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['trial', 'target', 'goal_yaw_deg', 'actual_yaw_deg', 'error_deg'])

        for i, target in enumerate(targets, 1):
            print(f"\n--- Lần {i}/{N_TRIALS}: di chuyển đến vị trí {target['label']} "
                  f"({target['x']:.2f}, {target['y']:.2f}), yaw mục tiêu={math.degrees(target['yaw']):+.1f}° ---")
            ok = node.send_goal(target)
            if not ok:
                print(f"  [!] Goal lần {i} không hoàn thành trong {GOAL_TIMEOUT_SEC}s, bỏ qua.")
                continue

            time.sleep(1.5)
            rclpy.spin_once(node, timeout_sec=0.5)

            actual_yaw = quat_to_yaw(node.amcl_pose.orientation)
            goal_yaw = target['yaw']
            err = math.degrees(norm_angle(goal_yaw - actual_yaw))
            rows.append(err)

            writer.writerow([i, target['label'], f"{math.degrees(goal_yaw):.2f}",
                              f"{math.degrees(actual_yaw):.2f}", f"{err:.3f}"])
            f.flush()
            print(f"  Goal yaw={math.degrees(goal_yaw):+.1f}°  Actual yaw={math.degrees(actual_yaw):+.1f}°  "
                  f"Sai số={err:+.2f}°")

    print("\n" + "=" * 60)
    print("THỐNG KÊ SAI SỐ GÓC QUAY (8 lần di chuyển giữa 2 vị trí)")
    print("=" * 60)
    if rows:
        abs_rows = [abs(e) for e in rows]
        print(f"Số lần thành công : {len(rows)}/{N_TRIALS}")
        print(f"Sai số trung bình  : {statistics.mean(rows):+.2f}°")
        print(f"Sai số tuyệt đối TB: {statistics.mean(abs_rows):.2f}°")
        print(f"Độ lệch chuẩn      : {statistics.stdev(rows):.2f}°" if len(rows) > 1 else "")
        print(f"Sai số nhỏ nhất    : {min(abs_rows):.2f}°")
        print(f"Sai số lớn nhất    : {max(abs_rows):.2f}°")
    else:
        print("Không có lần nào thành công.")
    print(f"\nDữ liệu chi tiết đã lưu tại: {OUTPUT_CSV}")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
