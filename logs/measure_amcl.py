#!/usr/bin/env python3
"""
Script đo sai số định vị AMCL — Thí nghiệm 4.2
So sánh AMCL pose với ground truth từ Gazebo
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates
import math, time

class AMCLMeasure(Node):
    def __init__(self):
        super().__init__('amcl_measure')
        self.amcl_x = self.amcl_y = self.amcl_yaw = None
        self.gt_x   = self.gt_y   = self.gt_yaw   = None
        self.converge_start = None
        self.converged = False

        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10)
        self.create_subscription(
            ModelStates, '/gazebo/model_states', self.gazebo_cb, 10)

        self.create_timer(2.0, self.print_status)
        print("Đang theo dõi AMCL... Đặt '2D Pose Estimate' trong RViz2 để bắt đầu.")
        print("-" * 60)

    def amcl_cb(self, msg):
        p = msg.pose.pose
        self.amcl_x = p.position.x
        self.amcl_y = p.position.y
        # Chuyển quaternion → yaw
        q = p.orientation
        siny = 2.0*(q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        self.amcl_yaw = math.atan2(siny, cosy)

        if not self.converged:
            cov = msg.pose.covariance
            # Hội tụ khi covariance position < 0.05 m²
            if cov[0] < 0.05 and cov[7] < 0.05:
                self.converged = True
                elapsed = time.time() - self.converge_start if self.converge_start else 0
                print(f"\n✅ AMCL HỘI TỤ sau {elapsed:.1f} giây")
                print(f"   Covariance x={cov[0]:.4f}  y={cov[7]:.4f}")

    def gazebo_cb(self, msg):
        try:
            idx = msg.name.index('amr_robot')
            p = msg.pose[idx]
            self.gt_x = p.position.x
            self.gt_y = p.position.y
            q = p.orientation
            siny = 2.0*(q.w*q.z + q.x*q.y)
            cosy = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
            self.gt_yaw = math.atan2(siny, cosy)
            if self.converge_start is None:
                self.converge_start = time.time()
        except ValueError:
            pass

    def print_status(self):
        if self.amcl_x is None or self.gt_x is None:
            return
        dx  = abs(self.amcl_x - self.gt_x)
        dy  = abs(self.amcl_y - self.gt_y)
        dyaw = abs(self.amcl_yaw - self.gt_yaw)
        # Normalize yaw error
        if dyaw > math.pi: dyaw = 2*math.pi - dyaw

        print(f"Ground Truth: x={self.gt_x:+.3f}  y={self.gt_y:+.3f}  "
              f"yaw={math.degrees(self.gt_yaw):+.1f}°")
        print(f"AMCL Pose:    x={self.amcl_x:+.3f}  y={self.amcl_y:+.3f}  "
              f"yaw={math.degrees(self.amcl_yaw):+.1f}°")
        print(f"SAI SỐ:       Δx={dx*100:.1f}cm  Δy={dy*100:.1f}cm  "
              f"Δyaw={math.degrees(dyaw):.1f}°")
        print(f"Hội tụ: {'✅ CÓ' if self.converged else '⏳ CHƯA'}")
        print("-" * 60)

def main():
    rclpy.init()
    node = AMCLMeasure()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
