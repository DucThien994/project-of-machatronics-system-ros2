"""
MODULE TEST 3 — test_directional_zones.py
Test logic phân vùng góc LiDAR (FRONT/REAR/LEFT/RIGHT mask).
Thuật toán: boolean masking với numpy trên mảng góc.
Không cần ROS2.
Chạy: python3 tests/unit/test_directional_zones.py
"""

import sys, os, math
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../src/amr_safety/scripts'))

from collision_warning_node import FRONT_HALF, REAR_MIN, SIDE_MIN, SIDE_MAX

PASS = "\033[92m[PASS]\033[0m"
FAIL = "\033[91m[FAIL]\033[0m"

def check(desc, got, expected):
    ok = got == expected
    print(f"  {PASS if ok else FAIL}  {desc}  got={got}")
    return ok

N = 720  # samples

def build_masks(n=N):
    angles = -math.pi + np.arange(n) * (2 * math.pi / n)
    abs_a  = np.abs(angles)
    front_mask = abs_a <= FRONT_HALF
    rear_mask  = abs_a >= REAR_MIN
    left_mask  = (angles >= SIDE_MIN)  & (angles <= SIDE_MAX)
    right_mask = (angles <= -SIDE_MIN) & (angles >= -SIDE_MAX)
    return angles, front_mask, rear_mask, left_mask, right_mask


def angle_to_idx(deg, n=N):
    """Chuyển góc (degree) → index trong mảng 720 samples."""
    rad = math.radians(deg)
    return int((rad - (-math.pi)) / (2 * math.pi / n)) % n


def test_front_zone():
    """0°(forward), ±45°, ±60° phải nằm trong FRONT"""
    print("\n[TEST] FRONT zone: |angle| <= 60°")
    angles, front_mask, rear_mask, left_mask, right_mask = build_masks()
    results = []
    for deg in [0, 30, -30, 59, -59]:
        idx = angle_to_idx(deg)
        results.append(check(f"  {deg}° in FRONT", front_mask[idx], True))
    for deg in [61, -61, 90, -90, 180]:
        idx = angle_to_idx(deg)
        results.append(check(f"  {deg}° NOT in FRONT", front_mask[idx], False))
    return all(results)


def test_rear_zone():
    """180°(backward) và vùng >±120° phải nằm trong REAR"""
    print("\n[TEST] REAR zone: |angle| >= 120°")
    angles, front_mask, rear_mask, left_mask, right_mask = build_masks()
    results = []
    for deg in [121, -121, 150, -150]:
        idx = angle_to_idx(deg)
        results.append(check(f"  {deg}° in REAR", rear_mask[idx], True))
    for deg in [119, -119, 90, -90, 0]:
        idx = angle_to_idx(deg)
        results.append(check(f"  {deg}° NOT in REAR", rear_mask[idx], False))
    return all(results)


def test_left_zone():
    """90°(left) và 30°-150° phải nằm trong LEFT"""
    print("\n[TEST] LEFT zone: 30° <= angle <= 150°")
    angles, front_mask, rear_mask, left_mask, right_mask = build_masks()
    results = []
    for deg in [30, 90, 150]:
        idx = angle_to_idx(deg)
        results.append(check(f"  {deg}° in LEFT", left_mask[idx], True))
    for deg in [29, -90, 151, 0]:
        idx = angle_to_idx(deg)
        results.append(check(f"  {deg}° NOT in LEFT", left_mask[idx], False))
    return all(results)


def test_right_zone():
    """-90°(right) và -30° đến -150° phải nằm trong RIGHT"""
    print("\n[TEST] RIGHT zone: -30° >= angle >= -150°")
    angles, front_mask, rear_mask, left_mask, right_mask = build_masks()
    results = []
    for deg in [-30, -90, -150]:
        idx = angle_to_idx(deg)
        results.append(check(f"  {deg}° in RIGHT", right_mask[idx], True))
    for deg in [-29, 90, -151, 0]:
        idx = angle_to_idx(deg)
        results.append(check(f"  {deg}° NOT in RIGHT", right_mask[idx], False))
    return all(results)


def test_zone_constants():
    """Kiểm tra giá trị hằng số góc đúng spec"""
    print("\n[TEST] Zone constant values")
    results = [
        check("FRONT_HALF = pi/3 (60°)",  abs(FRONT_HALF - math.pi/3)   < 1e-6, True),
        check("REAR_MIN   = 2pi/3 (120°)",abs(REAR_MIN   - math.pi*2/3) < 1e-6, True),
        check("SIDE_MIN   = pi/6 (30°)",  abs(SIDE_MIN   - math.pi/6)   < 1e-6, True),
        check("SIDE_MAX   = 5pi/6 (150°)",abs(SIDE_MAX   - math.pi*5/6) < 1e-6, True),
    ]
    return all(results)


def test_overlap_regions():
    """
    Vùng giao nhau (overlap) giữa các zone:
    - Góc 30°-60°: vừa FRONT vừa LEFT
    - Góc 120°-150°: vừa REAR vừa LEFT
    """
    print("\n[TEST] Overlap regions (expected behavior)")
    angles, front_mask, rear_mask, left_mask, right_mask = build_masks()
    results = []
    idx_45 = angle_to_idx(45)
    results.append(check("45° in FRONT",   front_mask[idx_45], True))
    results.append(check("45° in LEFT",    left_mask[idx_45],  True))
    idx_135 = angle_to_idx(135)
    results.append(check("135° in REAR",   rear_mask[idx_135],  True))
    results.append(check("135° in LEFT",   left_mask[idx_135],  True))
    return all(results)


if __name__ == '__main__':
    ok = all([
        test_zone_constants(),
        test_front_zone(),
        test_rear_zone(),
        test_left_zone(),
        test_right_zone(),
        test_overlap_regions(),
    ])
    print(f"\n{'='*50}")
    print(f"  Result: {'ALL PASSED' if ok else 'SOME FAILED'}")
    sys.exit(0 if ok else 1)
