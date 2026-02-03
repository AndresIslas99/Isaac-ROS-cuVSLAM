#!/bin/bash
# =============================================================================
# SLAM Validation Script — Verify pipeline health and accuracy metrics
# =============================================================================
# Run while the pipeline is active to check all systems.
# Usage: ./validate_slam.sh [duration_seconds]
# =============================================================================

DURATION=${1:-15}
echo "═══════════════════════════════════════════════"
echo "  AGV SLAM Validation (${DURATION}s measurement)"
echo "═══════════════════════════════════════════════"
echo ""

source /opt/ros/humble/setup.bash 2>/dev/null
source ~/agv_slam_ws/install/setup.bash 2>/dev/null

# ── 1. Check all expected nodes are running ──
echo "── Node Check ──"
EXPECTED_NODES=("zed_node" "depth_filter" "visual_slam" "rtabmap" "slam_monitor")
RUNNING_NODES=$(ros2 node list 2>/dev/null)

for node in "${EXPECTED_NODES[@]}"; do
    if echo "$RUNNING_NODES" | grep -q "$node"; then
        echo "  ✓ $node"
    else
        echo "  ✗ $node (MISSING!)"
    fi
done
echo ""

# ── 2. Measure topic rates ──
echo "── Topic Rates (${DURATION}s sample) ──"

declare -A TOPICS=(
    ["RGB"]="/filtered/rgb"
    ["Depth"]="/filtered/depth"
    ["IMU"]="/zed/zed_node/imu/data"
    ["cuVSLAM Odom"]="/visual_slam/tracking/odometry"
    ["RTAB-Map MapData"]="/rtabmap/mapData"
    ["Grid Map"]="/rtabmap/grid_map"
)

for name in "${!TOPICS[@]}"; do
    topic="${TOPICS[$name]}"
    hz=$(timeout ${DURATION}s ros2 topic hz "$topic" --window 50 2>/dev/null | \
         tail -1 | grep -oP '[\d.]+' | head -1)
    if [ -n "$hz" ]; then
        echo "  ${name}: ${hz} Hz"
    else
        echo "  ${name}: NO DATA"
    fi
done &

# Wait for background rate measurement
wait
echo ""

# ── 3. Check TF tree ──
echo "── TF Chain Check ──"
TF_FRAMES=$(ros2 run tf2_ros tf2_echo map base_link --wait-for-server-timeout 3 2>/dev/null | head -5)
if echo "$TF_FRAMES" | grep -q "Translation"; then
    echo "  ✓ map → base_link: OK"
    echo "    $TF_FRAMES" | grep "Translation"
else
    echo "  ✗ map → base_link: BROKEN"
fi

TF_ODOM=$(ros2 run tf2_ros tf2_echo odom base_link --wait-for-server-timeout 3 2>/dev/null | head -5)
if echo "$TF_ODOM" | grep -q "Translation"; then
    echo "  ✓ odom → base_link: OK (cuVSLAM)"
else
    echo "  ✗ odom → base_link: BROKEN (cuVSLAM not publishing TF)"
fi
echo ""

# ── 4. RTAB-Map stats ──
echo "── RTAB-Map Status ──"
RTABMAP_INFO=$(ros2 topic echo /rtabmap/info --once --timeout 5 2>/dev/null)
if [ -n "$RTABMAP_INFO" ]; then
    echo "  Map nodes: $(echo "$RTABMAP_INFO" | grep -oP 'map_size: \K\d+' || echo 'N/A')"
    echo "  Loop closures: $(echo "$RTABMAP_INFO" | grep -oP 'loop_closure_id: \K\d+' || echo 'N/A')"
    echo "  Processing time: $(echo "$RTABMAP_INFO" | grep -oP 'time: \K[\d.]+' || echo 'N/A')s"
else
    echo "  Waiting for RTAB-Map info..."
fi
echo ""

# ── 5. System resources ──
echo "── System Resources ──"
echo "  RAM: $(free -h | awk '/Mem:/ {printf "%s / %s (%.0f%%)", $3, $2, $3/$2*100}')"
echo "  CPU: $(top -bn1 | grep "Cpu(s)" | awk '{print 100-$8"%"}')"

if command -v tegrastats &> /dev/null; then
    GPU_USAGE=$(timeout 2 tegrastats 2>/dev/null | head -1 | grep -oP 'GR3D_FREQ \K\d+%' || echo "N/A")
    echo "  GPU: $GPU_USAGE"
fi
echo ""

echo "═══════════════════════════════════════════════"
echo "  Validation complete"
echo "═══════════════════════════════════════════════"
