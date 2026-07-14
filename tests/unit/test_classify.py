"""
MODULE TEST 1 — test_classify.py
Test hàm _classify() trong collision_warning_node.py
Không cần ROS2, không cần Gazebo.
Chạy: python3 tests/unit/test_classify.py
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../src/amr_safety/scripts'))

from collision_warning_node import _classify, THRESH_TRANS, THRESH_ROT

PASS = "\033[92m[PASS]\033[0m"
FAIL = "\033[91m[FAIL]\033[0m"

def check(desc, got, expected):
    ok = got == expected
    print(f"  {PASS if ok else FAIL}  {desc}")
    print(f"         got={got!r}  expected={expected!r}")
    return ok

def test_classify_trans():
    print("\n[TEST] _classify() with THRESH_TRANS")
    results = []
    results.append(check("dist=0.10 → CRITICAL",   _classify(0.10, THRESH_TRANS), 'CRITICAL'))
    results.append(check("dist=0.22 → DANGER",     _classify(0.22, THRESH_TRANS), 'DANGER'))
    results.append(check("dist=0.35 → WARNING",    _classify(0.35, THRESH_TRANS), 'WARNING'))
    results.append(check("dist=0.55 → CAUTION",    _classify(0.55, THRESH_TRANS), 'CAUTION'))
    results.append(check("dist=0.80 → SAFE",       _classify(0.80, THRESH_TRANS), 'SAFE'))
    results.append(check("dist=5.00 → SAFE",       _classify(5.00, THRESH_TRANS), 'SAFE'))
    results.append(check("dist=0.2199 → CRITICAL", _classify(0.2199, THRESH_TRANS), 'CRITICAL'))
    results.append(check("dist=0.2200 → DANGER",   _classify(0.2200, THRESH_TRANS), 'DANGER'))
    return all(results)

def test_classify_rot():
    print("\n[TEST] _classify() with THRESH_ROT")
    results = []
    results.append(check("dist=0.27 → CRITICAL",  _classify(0.27, THRESH_ROT), 'CRITICAL'))
    results.append(check("dist=0.28 → DANGER",    _classify(0.28, THRESH_ROT), 'DANGER'))
    results.append(check("dist=0.40 → WARNING",   _classify(0.40, THRESH_ROT), 'WARNING'))
    results.append(check("dist=0.60 → CAUTION",   _classify(0.60, THRESH_ROT), 'CAUTION'))
    results.append(check("dist=0.85 → SAFE",      _classify(0.85, THRESH_ROT), 'SAFE'))
    return all(results)

def test_boundary_exact():
    """Kiểm tra boundary condition: giá trị đúng bằng threshold là mức CAO HƠN."""
    print("\n[TEST] Boundary conditions (dist == threshold → next higher level)")
    results = []
    # dist < CRITICAL(0.22) → CRITICAL; dist == 0.22 → DANGER
    results.append(check("dist=0.2200 == CRITICAL → DANGER", _classify(0.2200, THRESH_TRANS), 'DANGER'))
    results.append(check("dist=0.3500 == DANGER   → WARNING", _classify(0.3500, THRESH_TRANS), 'WARNING'))
    results.append(check("dist=0.5500 == WARNING  → CAUTION", _classify(0.5500, THRESH_TRANS), 'CAUTION'))
    results.append(check("dist=0.8000 == CAUTION  → SAFE",    _classify(0.8000, THRESH_TRANS), 'SAFE'))
    return all(results)

if __name__ == '__main__':
    ok1 = test_classify_trans()
    ok2 = test_classify_rot()
    ok3 = test_boundary_exact()
    total = all([ok1, ok2, ok3])
    print(f"\n{'='*50}")
    print(f"  Result: {'ALL PASSED' if total else 'SOME FAILED'}")
    sys.exit(0 if total else 1)
