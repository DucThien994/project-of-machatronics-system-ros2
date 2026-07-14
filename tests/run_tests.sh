#!/bin/bash
# run_tests.sh — Chạy toàn bộ test suite theo thứ tự từ nhỏ đến lớn
#
# Thứ tự:
#   1. Unit tests (không cần ROS)       → python3
#   2. Node tests (cần rclpy)           → python3 + ROS2 sourced
#   3. Integration tests (cần Gazebo)  → shell + full bringup
#
# Cách dùng:
#   bash tests/run_tests.sh [unit|node|integration|all]

source /opt/ros/humble/setup.bash 2>/dev/null || true
cd "$(dirname "$0")/.."

MODE="${1:-all}"
PASS="\033[92m[PASS]\033[0m"
FAIL="\033[91m[FAIL]\033[0m"
declare -A RESULTS

run_python() {
    local label="$1"
    local file="$2"
    echo ""
    echo "──────────────────────────────────────────────────────"
    echo "  Running: $label"
    echo "──────────────────────────────────────────────────────"
    python3 "$file"
    RESULTS["$label"]=$?
}

run_bash() {
    local label="$1"
    local file="$2"
    echo ""
    echo "──────────────────────────────────────────────────────"
    echo "  Running: $label"
    echo "──────────────────────────────────────────────────────"
    bash "$file"
    RESULTS["$label"]=$?
}

# ── UNIT TESTS (không cần ROS2) ──────────────────────────────────────────────
if [[ "$MODE" == "unit" || "$MODE" == "all" ]]; then
    echo ""
    echo "════════════════════════════════════════════════════════"
    echo "  UNIT TESTS — Pure Python, không cần ROS2"
    echo "════════════════════════════════════════════════════════"
    run_python "test_classify"          "tests/unit/test_classify.py"
    run_python "test_speed_scaling"     "tests/unit/test_speed_scaling.py"
    run_python "test_directional_zones" "tests/unit/test_directional_zones.py"
fi

# ── NODE TESTS (cần ROS2 sourced) ────────────────────────────────────────────
if [[ "$MODE" == "node" || "$MODE" == "all" ]]; then
    echo ""
    echo "════════════════════════════════════════════════════════"
    echo "  NODE TESTS — cần ROS2 Humble sourced"
    echo "════════════════════════════════════════════════════════"
    if command -v ros2 &> /dev/null; then
        source install/setup.bash 2>/dev/null || true
        run_python "test_collision_warning_node" "tests/node/test_collision_warning_node.py"
    else
        echo "  (skip) ROS2 không có — source /opt/ros/humble/setup.bash trước"
    fi
fi

# ── INTEGRATION TESTS (cần Gazebo + bringup) ─────────────────────────────────
if [[ "$MODE" == "integration" || "$MODE" == "all" ]]; then
    echo ""
    echo "════════════════════════════════════════════════════════"
    echo "  INTEGRATION TESTS — cần Gazebo đang chạy"
    echo "  (ros2 launch amr_bringup bringup.launch.py)"
    echo "════════════════════════════════════════════════════════"
    if ros2 topic list --timeout 2 2>/dev/null | grep -q "/scan"; then
        run_bash   "test_cmd_vel_pipeline" "tests/integration/test_cmd_vel_pipeline.sh"
        run_bash   "test_slam_quality"     "tests/integration/test_slam_quality.sh"
        run_python "test_nav_goal"         "tests/integration/test_nav_goal.py"
    else
        echo "  (skip) /scan không có — khởi động Gazebo bringup trước"
    fi
fi

# ── SUMMARY ──────────────────────────────────────────────────────────────────
echo ""
echo "════════════════════════════════════════════════════════"
echo "  SUMMARY"
echo "════════════════════════════════════════════════════════"
TOTAL=0; PASSED=0
for label in "${!RESULTS[@]}"; do
    code=${RESULTS[$label]}
    TOTAL=$((TOTAL+1))
    if [ $code -eq 0 ]; then
        PASSED=$((PASSED+1))
        echo -e "  $PASS $label"
    else
        echo -e "  $FAIL $label (exit $code)"
    fi
done
echo ""
echo "  $PASSED / $TOTAL passed"
echo "════════════════════════════════════════════════════════"

[ $PASSED -eq $TOTAL ]
