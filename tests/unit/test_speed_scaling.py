"""
MODULE TEST 2 — test_speed_scaling.py
Test logic tính toán scale hệ số tốc độ theo hướng di chuyển và mức an toàn.
Không cần ROS2. Test thuần Python với numpy.
Chạy: python3 tests/unit/test_speed_scaling.py
"""

import sys, os, math
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../src/amr_safety/scripts'))

from collision_warning_node import (
    _classify, THRESH_TRANS, THRESH_ROT,
    SCALE_TRANS, SCALE_ROT,
    FRONT_HALF, REAR_MIN, SIDE_MIN, SIDE_MAX,
    EMERGENCY_THRESHOLD,
)

PASS = "\033[92m[PASS]\033[0m"
FAIL = "\033[91m[FAIL]\033[0m"

def check(desc, got, expected, tol=1e-9):
    if isinstance(expected, float):
        ok = abs(got - expected) < tol
    else:
        ok = got == expected
    print(f"  {PASS if ok else FAIL}  {desc}  got={got}")
    return ok

# ─── Simulate scan_callback logic (no ROS) ───────────────────────────────────

def compute_scales(ranges_deg: dict, n=720):
    """
    ranges_deg: {angle_deg: distance_m, ...}
    Returns: scale_fwd, scale_bwd, scale_left, scale_rgt, scale_wz, emergency
    """
    ranges = np.full(n, 12.0, dtype=np.float32)
    for angle_deg, dist in ranges_deg.items():
        idx = int((math.radians(angle_deg) - (-math.pi)) / (2*math.pi / n))
        idx = max(0, min(n-1, idx))
        ranges[idx] = dist

    angles = -math.pi + np.arange(n) * (2*math.pi / n)
    abs_a  = np.abs(angles)

    front_mask = abs_a <= FRONT_HALF
    rear_mask  = abs_a >= REAR_MIN
    left_mask  = (angles >= SIDE_MIN) & (angles <= SIDE_MAX)
    right_mask = (angles <= -SIDE_MIN) & (angles >= -SIDE_MAX)

    def _min(mask):
        s = ranges[mask]
        return float(np.min(s)) if s.size > 0 else float('inf')

    min_front = _min(front_mask)
    min_rear  = _min(rear_mask)
    min_left  = _min(left_mask)
    min_rgt   = _min(right_mask)

    level_fwd  = _classify(min_front, THRESH_TRANS)
    level_bwd  = _classify(min_rear,  THRESH_TRANS)
    level_left = _classify(min_left,  THRESH_TRANS)
    level_rgt  = _classify(min_rgt,   THRESH_TRANS)
    level_wz   = _classify(min(min_front, min_rear), THRESH_ROT)

    emergency = float(np.min(ranges)) < EMERGENCY_THRESHOLD

    return (
        SCALE_TRANS[level_fwd],
        SCALE_TRANS[level_bwd],
        SCALE_TRANS[level_left],
        SCALE_TRANS[level_rgt],
        SCALE_ROT[level_wz],
        emergency,
    )


def test_all_clear():
    """Không có vật cản → tất cả scale = 1.0"""
    print("\n[TEST] All clear — no obstacles")
    sf, sb, sl, sr, sw, emg = compute_scales({})
    results = [
        check("scale_fwd  = 1.0", sf, 1.0),
        check("scale_bwd  = 1.0", sb, 1.0),
        check("scale_left = 1.0", sl, 1.0),
        check("scale_rgt  = 1.0", sr, 1.0),
        check("scale_wz   = 1.0", sw, 1.0),
        check("emergency  = False", emg, False),
    ]
    return all(results)


def test_obstacle_front_critical():
    """Vật cản phía trước 0.18m → vx scale=0, wz scale=0, emergency khi <0.15"""
    print("\n[TEST] Front obstacle at 0.18m (CRITICAL zone, not emergency)")
    sf, sb, sl, sr, sw, emg = compute_scales({0: 0.18})
    results = [
        check("scale_fwd  = 0.0 (CRITICAL)", sf, 0.0),
        check("scale_bwd  = 1.0 (rear clear)", sb, 1.0),
        check("scale_wz   = 0.0 (CRITICAL<0.28)", sw, 0.0),
        check("emergency  = False (0.18 > 0.15)", emg, False),
    ]
    return all(results)


def test_emergency_stop():
    """Vật cản < 0.15m → emergency = True"""
    print("\n[TEST] Emergency threshold: obstacle at 0.10m")
    sf, sb, sl, sr, sw, emg = compute_scales({0: 0.10})
    results = [
        check("emergency = True", emg, True),
    ]
    return all(results)


def test_left_obstacle_only():
    """Vật cản bên trái 0.30m → chỉ vy+ bị giảm, vx và vy- không ảnh hưởng"""
    print("\n[TEST] Left obstacle at 0.30m (DANGER zone)")
    sf, sb, sl, sr, sw, emg = compute_scales({90: 0.30})
    results = [
        check("scale_fwd  = 1.0 (front clear)", sf, 1.0),
        check("scale_bwd  = 1.0 (rear clear)",  sb, 1.0),
        check("scale_left = 0.20 (DANGER)",     sl, 0.20),
        check("scale_rgt  = 1.0 (right clear)", sr, 1.0),
        check("scale_wz   = 1.0 (front/rear clear)", sw, 1.0),
    ]
    return all(results)


def test_wz_uses_min_front_rear():
    """FIX BUG-2: wz dùng min(front, rear), không phải global_min"""
    print("\n[TEST] FIX BUG-2: wz scale uses min(front,rear) only")
    # Chỉ có vật bên trái 0.20m (CRITICAL), nhưng front/rear đều xa
    sf, sb, sl, sr, sw, emg = compute_scales({90: 0.20})
    results = [
        check("scale_left = 0.0 (CRITICAL)", sl, 0.0),
        # wz KHÔNG bị ảnh hưởng vì front/rear đều xa
        check("scale_wz  = 1.0 (front/rear clear, only side blocked)", sw, 1.0),
    ]
    return all(results)


def test_scale_values_correct():
    """Kiểm tra tất cả SCALE values đúng theo spec"""
    print("\n[TEST] Scale factor values")
    results = [
        check("SCALE_TRANS CRITICAL = 0.00", SCALE_TRANS['CRITICAL'], 0.00),
        check("SCALE_TRANS DANGER   = 0.20", SCALE_TRANS['DANGER'],   0.20),
        check("SCALE_TRANS WARNING  = 0.50", SCALE_TRANS['WARNING'],  0.50),
        check("SCALE_TRANS CAUTION  = 0.75", SCALE_TRANS['CAUTION'],  0.75),
        check("SCALE_TRANS SAFE     = 1.00", SCALE_TRANS['SAFE'],     1.00),
        check("SCALE_ROT   CRITICAL = 0.00", SCALE_ROT['CRITICAL'],   0.00),
        check("SCALE_ROT   DANGER   = 0.30", SCALE_ROT['DANGER'],     0.30),
        check("SCALE_ROT   WARNING  = 0.60", SCALE_ROT['WARNING'],    0.60),
        check("SCALE_ROT   CAUTION  = 0.85", SCALE_ROT['CAUTION'],    0.85),
    ]
    return all(results)


if __name__ == '__main__':
    ok = all([
        test_all_clear(),
        test_obstacle_front_critical(),
        test_emergency_stop(),
        test_left_obstacle_only(),
        test_wz_uses_min_front_rear(),
        test_scale_values_correct(),
    ])
    print(f"\n{'='*50}")
    print(f"  Result: {'ALL PASSED' if ok else 'SOME FAILED'}")
    sys.exit(0 if ok else 1)
