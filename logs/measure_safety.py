#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math, csv, time, os

OUTPUT_FILE = os.path.expanduser('~/final-ros2/amr_ver6.0/logs/safety_log.csv')

THRESH_TRANS = {'CRITICAL': 0.22, 'DANGER': 0.35, 'WARNING': 0.55,  'CAUTION': 0.80}
SCALE_TRANS  = {'SAFE': 1.0, 'CAUTION': 0.75, 'WARNING': 0.50, 'DANGER': 0.20, 'CRITICAL': 0.0}

def classify(dist, thresh):
    if dist < thresh['CRITICAL']: return 'CRITICAL'
    if dist < thresh['DANGER']:   return 'DANGER'
    if dist < thresh['WARNING']:  return 'WARNING'
    if dist < thresh['CAUTION']:  return 'CAUTION'
    return 'SAFE'

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        self.cmd_raw  = Twist()
        self.cmd_safe = Twist()
        self.min_front = self.min_rear = self.min_left = self.min_right = float('inf')

        self.create_subscription(Twist, '/cmd_vel',      self.raw_cb,  10)
        self.create_subscription(Twist, '/cmd_vel_safe', self.safe_cb, 10)
        self.create_subscription(LaserScan, '/scan',     self.scan_cb, 10)
        self.create_timer(0.5, self.log_and_print)

        self.csvfile = open(OUTPUT_FILE, 'w', newline='')
        self.writer = csv.writer(self.csvfile)
        self.writer.writerow([
            'time', 'min_front', 'min_left', 'min_right',
            'vx_raw', 'vx_safe', 'scale_fwd_actual',
            'vy_raw', 'vy_safe', 'scale_lat_actual',
            'level_front', 'level_left', 'level_right',
            'scale_fwd_theory', 'scale_lat_theory'])
        print(f"Ghi log vào: {OUTPUT_FILE}")

    def raw_cb(self,  msg): self.cmd_raw  = msg
    def safe_cb(self, msg): self.cmd_safe = msg

    def scan_cb(self, msg):
        n = len(msg.ranges)
        def min_sector(a_min_deg, a_max_deg):
            idx_min = int((a_min_deg + 180) / 360 * n)
            idx_max = int((a_max_deg + 180) / 360 * n)
            sector = [r for r in msg.ranges[idx_min:idx_max] if msg.range_min < r < msg.range_max]
            return min(sector) if sector else float('inf')

        self.min_front = min_sector(-60, 60)
        self.min_rear  = min(min_sector(120, 180), min_sector(-180, -120))
        self.min_left  = min_sector(30, 150)
        self.min_right = min_sector(-150, -30)

    def log_and_print(self):
        t = time.time()
        vx_r = self.cmd_raw.linear.x
        vy_r = self.cmd_raw.linear.y
        vx_s = self.cmd_safe.linear.x
        vy_s = self.cmd_safe.linear.y

        s_fwd = (vx_s / vx_r) if abs(vx_r) > 0.01 else float('nan')
        s_lat = (vy_s / vy_r) if abs(vy_r) > 0.01 else float('nan')

        lv_f = classify(self.min_front, THRESH_TRANS)
        lv_l = classify(self.min_left,  THRESH_TRANS)
        lv_r = classify(self.min_right, THRESH_TRANS)
        s_fwd_th = SCALE_TRANS[lv_f] if vx_r > 0 else SCALE_TRANS[classify(self.min_rear, THRESH_TRANS)]
        s_lat_th = SCALE_TRANS[lv_l] if vy_r > 0 else SCALE_TRANS[lv_r]

        self.writer.writerow([
            f'{t:.3f}', f'{self.min_front:.3f}', f'{self.min_left:.3f}', f'{self.min_right:.3f}',
            f'{vx_r:.3f}', f'{vx_s:.3f}', f'{s_fwd:.3f}' if not math.isnan(s_fwd) else 'N/A',
            f'{vy_r:.3f}', f'{vy_s:.3f}', f'{s_lat:.3f}' if not math.isnan(s_lat) else 'N/A',
            lv_f, lv_l, lv_r, f'{s_fwd_th:.2f}', f'{s_lat_th:.2f}'])
        self.csvfile.flush()

        print(f"Front: {self.min_front:6.3f}m | Left: {self.min_left:6.3f}m | Right: {self.min_right:6.3f}m -> S_fwd: {s_fwd_th:.2f}, S_lat: {s_lat_th:.2f}")

def main():
    rclpy.init()
    node = SafetyMonitor()
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.csvfile.close()
    rclpy.shutdown()

if __name__ == '__main__': main()
