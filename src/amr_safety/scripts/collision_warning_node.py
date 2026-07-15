#!/usr/bin/env python3
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


# ── Thresholds (meters, LiDAR center → obstacle) ────────────────────────────

# Translation thresholds (vx, vy)
THRESH_TRANS = {
    'CRITICAL': 0.22,
    'DANGER':   0.35,
    'WARNING':  0.55,
    'CAUTION':  0.80,
}

# Rotation thresholds (wz) — wider because robot corners sweep larger arc
THRESH_ROT = {
    'CRITICAL': 0.28,
    'DANGER':   0.40,
    'WARNING':  0.60,
    'CAUTION':  0.85,
}

# Speed scale factors
SCALE_TRANS = {
    'CRITICAL': 0.00,
    'DANGER':   0.20,
    'WARNING':  0.50,
    'CAUTION':  0.75,
    'SAFE':     1.00,
}
SCALE_ROT = {
    'CRITICAL': 0.00,
    'DANGER':   0.30,
    'WARNING':  0.60,
    'CAUTION':  0.85,
    'SAFE':     1.00,
}

# Emergency ALL-STOP: any single LiDAR ray < this value
EMERGENCY_THRESHOLD = 0.15   # m

# ── Directional cone boundaries (radians) ────────────────────────────────────
# ROS frame: angle=0 → +x (forward), angle=+pi/2 → +y (left)
FRONT_HALF = math.pi / 3          # +-60° from forward
REAR_MIN   = math.pi * 2 / 3      # beyond +-120° = rear region
SIDE_MIN   = math.pi / 6          # 30°
SIDE_MAX   = math.pi * 5 / 6      # 150°

# Tolerance for floating-point angle boundary comparisons (LiDAR angle arrays
# are built by repeated float addition, so a ray meant to sit exactly on a
# zone boundary can land a few ULPs to either side).
ANGLE_EPS  = 1e-9

# ── Marker colors ─────────────────────────────────────────────────────────────
MARKER_COLOR = {
    'CRITICAL': (1.0, 0.0, 0.0, 0.60),
    'DANGER':   (1.0, 0.4, 0.0, 0.50),
    'WARNING':  (1.0, 0.9, 0.0, 0.40),
    'CAUTION':  (0.2, 0.8, 1.0, 0.30),
    'SAFE':     (0.0, 1.0, 0.0, 0.20),
}

LEVELS_ORDER = ['SAFE', 'CAUTION', 'WARNING', 'DANGER', 'CRITICAL']
PUBLISH_HZ   = 20.0


def _classify(dist: float, thresh: dict) -> str:
    if dist < thresh['CRITICAL']:
        return 'CRITICAL'
    if dist < thresh['DANGER']:
        return 'DANGER'
    if dist < thresh['WARNING']:
        return 'WARNING'
    if dist < thresh['CAUTION']:
        return 'CAUTION'
    return 'SAFE'


class CollisionWarningNode(Node):

    def __init__(self):
        super().__init__('collision_warning_node')

        self.declare_parameter('publish_rate', PUBLISH_HZ)
        self.declare_parameter('cmd_timeout',  0.5)

        rate             = self.get_parameter('publish_rate').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        # State 
        self.last_cmd      = Twist()
        self.last_cmd_time = self.get_clock().now()

        self.scale_fwd  = 1.0
        self.scale_bwd  = 1.0
        self.scale_left = 1.0
        self.scale_rgt  = 1.0
        self.scale_wz   = 1.0

        self.emergency   = False
        self.global_min  = float('inf')
        self.worst_level = 'SAFE'

        self.level_fwd  = 'SAFE'
        self.level_bwd  = 'SAFE'
        self.level_left = 'SAFE'
        self.level_rgt  = 'SAFE'
        self.level_wz   = 'SAFE'
        self.min_front  = float('inf')
        self.min_rear   = float('inf')
        self.min_left   = float('inf')
        self.min_rgt    = float('inf')

        # QoS 
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        #  Subscribers 
        self.create_subscription(LaserScan, '/scan',    self.scan_callback, sensor_qos)
        self.create_subscription(Twist,     '/cmd_vel', self.cmd_callback,  10)

        # Publishers 
        self.cmd_safe_pub = self.create_publisher(Twist,       '/cmd_vel_safe',      10)
        self.warning_pub  = self.create_publisher(String,      '/collision_warning',  10)
        self.marker_pub   = self.create_publisher(MarkerArray, '/safety/markers',     10)

        self.create_timer(1.0 / rate, self.publish_safe_cmd)

        self.get_logger().info(
            '[collision_warning] Direction-aware Mecanum safety started.\n'
            f'  Trans: CRITICAL={THRESH_TRANS["CRITICAL"]}m | '
            f'DANGER={THRESH_TRANS["DANGER"]}m | '
            f'WARNING={THRESH_TRANS["WARNING"]}m | '
            f'CAUTION={THRESH_TRANS["CAUTION"]}m\n'
            f'  Rot:   CRITICAL={THRESH_ROT["CRITICAL"]}m | '
            f'DANGER={THRESH_ROT["DANGER"]}m\n'
            f'  Emergency all-stop: < {EMERGENCY_THRESHOLD}m'
        )

    def scan_callback(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return

        ranges = np.array(msg.ranges, dtype=np.float32)
        valid  = (np.isfinite(ranges)
                  & (ranges >= msg.range_min)
                  & (ranges <= msg.range_max))
        ranges = np.where(valid, ranges, msg.range_max)

        self.global_min = float(np.min(ranges))
        self.emergency  = self.global_min < EMERGENCY_THRESHOLD

        angles = (msg.angle_min
                  + np.arange(n, dtype=np.float32) * msg.angle_increment)

        #Directional masks
        abs_a = np.abs(angles)
        front_mask = abs_a <= FRONT_HALF + ANGLE_EPS
        rear_mask  = abs_a >= REAR_MIN - ANGLE_EPS
        side_mask  = (abs_a >= SIDE_MIN - ANGLE_EPS) & (abs_a <= SIDE_MAX + ANGLE_EPS)
        left_mask  = side_mask & (angles > 0)
        right_mask = side_mask & (angles < 0)

        def _min(mask):
            s = ranges[mask]
            return float(np.min(s)) if s.size > 0 else float('inf')

        self.min_front = _min(front_mask)
        self.min_rear  = _min(rear_mask)
        self.min_left  = _min(left_mask)
        self.min_rgt   = _min(right_mask)

        # ── Classify & scale 
        self.level_fwd  = _classify(self.min_front,  THRESH_TRANS)
        self.level_bwd  = _classify(self.min_rear,   THRESH_TRANS)
        self.level_left = _classify(self.min_left,   THRESH_TRANS)
        self.level_rgt  = _classify(self.min_rgt,    THRESH_TRANS)
        self.level_wz   = _classify(min(self.min_front, self.min_rear), THRESH_ROT)

        self.scale_fwd  = SCALE_TRANS[self.level_fwd]
        self.scale_bwd  = SCALE_TRANS[self.level_bwd]
        self.scale_left = SCALE_TRANS[self.level_left]
        self.scale_rgt  = SCALE_TRANS[self.level_rgt]
        self.scale_wz   = SCALE_ROT[self.level_wz]

        worst_idx = max(
            LEVELS_ORDER.index(lv)
            for lv in [self.level_fwd, self.level_bwd,
                       self.level_left, self.level_rgt, self.level_wz]
        )
        self.worst_level = LEVELS_ORDER[worst_idx]

        diag = (
            f'{self.worst_level}'
            + (' [EMERGENCY]' if self.emergency else '')
            + f' | min={self.global_min:.2f}m'
            f' | F={self.min_front:.2f}({self.level_fwd})'
            f' B={self.min_rear:.2f}({self.level_bwd})'
            f' L={self.min_left:.2f}({self.level_left})'
            f' R={self.min_rgt:.2f}({self.level_rgt})'
            f' | wz={self.scale_wz:.0%}'
        )
        w = String()
        w.data = diag
        self.warning_pub.publish(w)

        if self.worst_level != 'SAFE' or self.emergency:
            self.get_logger().warn(f'[SAFETY] {diag}')

        self._publish_markers()

    def cmd_callback(self, msg: Twist):
        self.last_cmd      = msg
        self.last_cmd_time = self.get_clock().now()

    def publish_safe_cmd(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        safe    = Twist()

        if elapsed > self.cmd_timeout:
            pass   # cmd timeout → zero
        elif self.emergency:
            pass   # any ray < 0.15m → all-stop
        else:
            vx = self.last_cmd.linear.x
            vy = self.last_cmd.linear.y
            wz = self.last_cmd.angular.z

            # vx: forward → FRONT scale, reverse → REAR scale
            if vx > 0.0:
                safe.linear.x = vx * self.scale_fwd
            elif vx < 0.0:
                safe.linear.x = vx * self.scale_bwd

            # vy: strafe-left → LEFT scale, strafe-right → RIGHT scale
            if vy > 0.0:
                safe.linear.y = vy * self.scale_left
            elif vy < 0.0:
                safe.linear.y = vy * self.scale_rgt

            # wz: rotation uses all-around min scale
            safe.angular.z = wz * self.scale_wz

        self.cmd_safe_pub.publish(safe)

    def _publish_markers(self):
        markers     = MarkerArray()
        current_idx = LEVELS_ORDER.index(self.worst_level)

        zone_radii = [
            ('CRITICAL', THRESH_TRANS['CRITICAL']),
            ('DANGER',   THRESH_TRANS['DANGER']),
            ('WARNING',  THRESH_TRANS['WARNING']),
            ('CAUTION',  THRESH_TRANS['CAUTION']),
        ]

        for idx, (level, radius) in enumerate(zone_radii):
            m = Marker()
            m.header.frame_id    = 'base_footprint'
            m.header.stamp       = self.get_clock().now().to_msg()
            m.ns                 = 'safety_zones'
            m.id                 = idx
            m.type               = Marker.CYLINDER
            m.action             = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x            = radius * 2.0
            m.scale.y            = radius * 2.0
            m.scale.z            = 0.02

            r, g, b, a = MARKER_COLOR[level]
            m.color.r  = r
            m.color.g  = g
            m.color.b  = b
            level_idx  = LEVELS_ORDER.index(level)
            m.color.a  = a if level_idx <= current_idx else 0.05

            m.lifetime.nanosec = int(0.2e9)
            markers.markers.append(m)

        t = Marker()
        t.header.frame_id    = 'base_footprint'
        t.header.stamp       = self.get_clock().now().to_msg()
        t.ns                 = 'safety_text'
        t.id                 = 10
        t.type               = Marker.TEXT_VIEW_FACING
        t.action             = Marker.ADD
        t.pose.position.z    = 0.8
        t.pose.orientation.w = 1.0
        t.scale.z            = 0.20

        r, g, b, _ = MARKER_COLOR[self.worst_level]
        t.color.r = r
        t.color.g = g
        t.color.b = b
        t.color.a = 1.0
        t.text = (
            f'{"[EMRG] " if self.emergency else ""}'
            f'{self.worst_level}\n'
            f'{self.global_min:.2f} m\n'
            f'vx+ {self.scale_fwd:.0%}  vx- {self.scale_bwd:.0%}\n'
            f'vy+ {self.scale_left:.0%}  vy- {self.scale_rgt:.0%}\n'
            f'wz  {self.scale_wz:.0%}'
        )
        t.lifetime.nanosec = int(0.2e9)
        markers.markers.append(t)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionWarningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
