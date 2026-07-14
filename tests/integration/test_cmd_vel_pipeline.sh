#!/bin/bash
# MODULE TEST 5 — test_cmd_vel_pipeline.sh
# Test toàn bộ pipeline: /cmd_vel → collision_warning_node → /cmd_vel_safe → planar_move
#
# Thuật toán kiểm tra: pub/sub ROS2 topic với timeout
# Cần: Gazebo đang chạy + robot đã spawn + collision_warning_node đang chạy
#
# Cách chạy:
#   # Terminal 1: ros2 launch amr_bringup bringup.launch.py nav:=false
#   # Terminal 2 (sau khi t=5s): bash tests/integration/test_cmd_vel_pipeline.sh

source /opt/ros/humble/setup.bash
cd "$(dirname "$0")/../.."
source install/setup.bash 2>/dev/null || true

PASS="\033[92m[PASS]\033[0m"
FAIL="\033[91m[FAIL]\033[0m"
ALL_OK=true

echo ""
echo "========================================================"
echo " TEST: cmd_vel → collision_warning → cmd_vel_safe pipeline"
echo "========================================================"

# ── CHECK 1: Topic /cmd_vel_safe đang publish ─────────────────────────────────
echo ""
echo "[CHECK 1] Topic /cmd_vel_safe is being published"
TOPIC_CHECK=$(ros2 topic list --timeout 3 2>/dev/null | grep "/cmd_vel_safe")
if [ -n "$TOPIC_CHECK" ]; then
    echo -e "  $PASS /cmd_vel_safe exists"
else
    echo -e "  $FAIL /cmd_vel_safe NOT found — collision_warning_node chưa chạy?"
    ALL_OK=false
fi

# ── CHECK 2: Publish cmd_vel, kiểm tra cmd_vel_safe có phản hồi ───────────────
echo ""
echo "[CHECK 2] Publish vx=0.3 to /cmd_vel → check /cmd_vel_safe echoes it"
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
    "{ linear: { x: 0.3, y: 0.0 }, angular: { z: 0.0 } }" > /dev/null 2>&1

SAFE_MSG=$(ros2 topic echo /cmd_vel_safe --once --timeout-sec 2 2>/dev/null)
if echo "$SAFE_MSG" | grep -q "linear"; then
    echo -e "  $PASS /cmd_vel_safe received message"
    echo "    Preview: $(echo "$SAFE_MSG" | grep "x:" | head -1)"
else
    echo -e "  $FAIL /cmd_vel_safe không nhận được message"
    ALL_OK=false
fi

# ── CHECK 3: /collision_warning topic đang publish ────────────────────────────
echo ""
echo "[CHECK 3] /collision_warning topic is active"
WARN_MSG=$(ros2 topic echo /collision_warning --once --timeout-sec 3 2>/dev/null)
if echo "$WARN_MSG" | grep -q "data"; then
    STATUS=$(echo "$WARN_MSG" | grep "data:" | head -1)
    echo -e "  $PASS /collision_warning active: $STATUS"
else
    echo -e "  $FAIL /collision_warning không có message"
    ALL_OK=false
fi

# ── CHECK 4: /odom đang publish (planar_move hoạt động) ──────────────────────
echo ""
echo "[CHECK 4] /odom published by planar_move plugin"
ODOM=$(ros2 topic echo /odom --once --timeout-sec 3 2>/dev/null)
if echo "$ODOM" | grep -q "pose"; then
    echo -e "  $PASS /odom active"
else
    echo -e "  $FAIL /odom không có data — planar_move chưa load?"
    ALL_OK=false
fi

# ── CHECK 5: /scan đang publish (LiDAR hoạt động) ─────────────────────────────
echo ""
echo "[CHECK 5] /scan published by LiDAR plugin (30Hz)"
SCAN_HZ=$(ros2 topic hz /scan --window 20 2>/dev/null &)
sleep 2
SCAN_RATE=$(ros2 topic hz /scan 2>/dev/null | grep "average rate" | awk '{print $3}')
if [ -n "$SCAN_RATE" ]; then
    echo -e "  $PASS /scan rate: ${SCAN_RATE} Hz (expected ~30Hz)"
else
    echo -e "  $FAIL /scan không publish"
    ALL_OK=false
fi

# ── CHECK 6: TF tree map→odom→base_footprint tồn tại ────────────────────────
echo ""
echo "[CHECK 6] TF: map→odom→base_footprint chain"
TF_MAP_ODOM=$(ros2 run tf2_ros tf2_echo map odom --timeout 2.0 2>/dev/null | head -5)
if echo "$TF_MAP_ODOM" | grep -q "Translation"; then
    echo -e "  $PASS map→odom TF exists"
else
    echo -e "  $FAIL map→odom TF missing — SLAM chưa chạy?"
    ALL_OK=false
fi

TF_ODOM_BF=$(ros2 run tf2_ros tf2_echo odom base_footprint --timeout 2.0 2>/dev/null | head -5)
if echo "$TF_ODOM_BF" | grep -q "Translation"; then
    echo -e "  $PASS odom→base_footprint TF exists"
else
    echo -e "  $FAIL odom→base_footprint TF missing"
    ALL_OK=false
fi

# ── SUMMARY ──────────────────────────────────────────────────────────────────
echo ""
echo "========================================================"
if $ALL_OK; then
    echo -e "  $PASS ALL CHECKS PASSED — Pipeline hoạt động đúng"
else
    echo -e "  $FAIL SOME CHECKS FAILED — Xem chi tiết bên trên"
fi
echo "========================================================"
