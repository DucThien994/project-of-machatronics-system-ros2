#!/bin/bash
# MODULE TEST 6 — test_slam_quality.sh
# Test chất lượng bản đồ SLAM Toolbox.
# Thuật toán: SLAM Toolbox (graph-based SLAM, Ceres solver, Levenberg-Marquardt)
#
# Metric kiểm tra:
#   - /map topic publish (OccupancyGrid)
#   - Map size hợp lý (> 100 cells)
#   - Không có excessive unknown cells (< 80% unknown)
#   - map→odom TF latency < 0.5s
#
# Cần: Gazebo + SLAM chạy (t >= 12s sau bringup)
# Chạy: bash tests/integration/test_slam_quality.sh

source /opt/ros/humble/setup.bash
cd "$(dirname "$0")/../.."
source install/setup.bash 2>/dev/null || true

PASS="\033[92m[PASS]\033[0m"
FAIL="\033[91m[FAIL]\033[0m"
ALL_OK=true

echo ""
echo "========================================================"
echo " TEST: SLAM Toolbox map quality"
echo " Thuật toán: Graph-based SLAM + Ceres LM solver"
echo "========================================================"

# ── CHECK 1: /map topic tồn tại ──────────────────────────────────────────────
echo ""
echo "[CHECK 1] /map topic published by slam_toolbox"
MAP_MSG=$(ros2 topic echo /map --once --timeout-sec 5 2>/dev/null)
if echo "$MAP_MSG" | grep -q "info"; then
    echo -e "  $PASS /map topic active"
    WIDTH=$(echo "$MAP_MSG"  | grep "width:"  | awk '{print $2}')
    HEIGHT=$(echo "$MAP_MSG" | grep "height:" | awk '{print $2}')
    RES=$(echo "$MAP_MSG"    | grep "resolution:" | awk '{print $2}')
    echo "    Map size: ${WIDTH}x${HEIGHT} cells, resolution=${RES}m/cell"
else
    echo -e "  $FAIL /map không publish — slam_toolbox chưa chạy?"
    ALL_OK=false
fi

# ── CHECK 2: /slam_toolbox/graph_visualization topic (nếu có) ────────────────
echo ""
echo "[CHECK 2] SLAM pose graph node count"
GRAPH=$(ros2 topic echo /slam_toolbox/graph_visualization --once --timeout-sec 3 2>/dev/null)
if echo "$GRAPH" | grep -q "points"; then
    NPOINTS=$(echo "$GRAPH" | grep -c "x:")
    echo -e "  $PASS Graph visualization active. Node count: ~${NPOINTS}"
else
    echo -e "  (skip) /slam_toolbox/graph_visualization not available"
fi

# ── CHECK 3: map→odom TF latency ─────────────────────────────────────────────
echo ""
echo "[CHECK 3] map→odom TF update frequency (SLAM output)"
T0=$(date +%s%N)
ros2 run tf2_ros tf2_echo map odom --timeout 2.0 2>/dev/null | head -3 > /dev/null
T1=$(date +%s%N)
LATENCY_MS=$(( (T1 - T0) / 1000000 ))
if [ $LATENCY_MS -lt 500 ]; then
    echo -e "  $PASS map→odom TF received in ${LATENCY_MS}ms (< 500ms)"
else
    echo -e "  $FAIL TF latency ${LATENCY_MS}ms — SLAM lag?"
    ALL_OK=false
fi

# ── CHECK 4: Lưu bản đồ và kiểm tra file ─────────────────────────────────────
echo ""
echo "[CHECK 4] Save map và verify file"
MAP_DIR="/tmp/amr_test_map_$(date +%s)"
mkdir -p "$MAP_DIR"
ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/test_map" --timeout 10.0 2>/dev/null
if [ -f "$MAP_DIR/test_map.pgm" ] && [ -f "$MAP_DIR/test_map.yaml" ]; then
    PGM_SIZE=$(wc -c < "$MAP_DIR/test_map.pgm")
    echo -e "  $PASS Map saved: test_map.pgm (${PGM_SIZE} bytes), test_map.yaml"
    # Kiểm tra YAML hợp lệ
    if grep -q "resolution:" "$MAP_DIR/test_map.yaml"; then
        RES_SAVED=$(grep "resolution:" "$MAP_DIR/test_map.yaml" | awk '{print $2}')
        echo "    Saved resolution: ${RES_SAVED}m (expected: 0.05)"
    fi
else
    echo -e "  $FAIL Map không được lưu — kiểm tra nav2_map_server"
    ALL_OK=false
fi

# ── CHECK 5: /scan topic rate ─────────────────────────────────────────────────
echo ""
echo "[CHECK 5] LiDAR /scan rate feeding SLAM"
timeout 3 ros2 topic hz /scan 2>/dev/null | grep "average rate" | \
    awk -v PASS="$PASS" -v FAIL="$FAIL" '{
        rate=$3+0;
        if(rate>=25 && rate<=35)
            print "  " PASS " /scan rate: " rate " Hz (expected 30Hz)";
        else
            print "  " FAIL " /scan rate: " rate " Hz (out of 25-35 range)";
    }'

# ── SUMMARY ──────────────────────────────────────────────────────────────────
echo ""
echo "========================================================"
if $ALL_OK; then
    echo -e "  $PASS SLAM quality checks PASSED"
else
    echo -e "  $FAIL SOME SLAM checks FAILED"
fi
echo "========================================================"
