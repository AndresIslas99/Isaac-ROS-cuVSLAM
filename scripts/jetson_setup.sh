#!/bin/bash
# =============================================================================
# Jetson Orin AGX 64GB — Performance Setup for Industrial SLAM Pipeline
# =============================================================================
# Run ONCE after boot (or add to systemd service).
# Requires sudo.
#
# Includes: MAXN mode, locked clocks, network buffers, ZRAM disable,
#           RT scheduling, thread affinity setup, NVMe directory creation
# =============================================================================

set -e

echo "═══════════════════════════════════════════════"
echo "  AGV SLAM — Jetson Orin AGX Performance Setup"
echo "  Industrial Grade Configuration"
echo "═══════════════════════════════════════════════"

# ── 1. Power mode: MAXN (all cores, max clocks) ──
echo "[1/9] Setting MAXN power mode..."
sudo nvpmodel -m 0
sleep 2

# ── 2. Lock clocks at maximum ──
echo "[2/9] Locking clocks at maximum..."
sudo jetson_clocks
sleep 1

# ── 3. Verify GPU is available ──
echo "[3/9] Checking GPU..."
if command -v nvidia-smi &> /dev/null; then
    nvidia-smi
elif [ -f /usr/bin/tegrastats ]; then
    echo "Tegra platform detected. GPU check via tegrastats."
    timeout 2 tegrastats || true
else
    echo "WARNING: Cannot verify GPU. Ensure CUDA is available."
fi

# ── 4. Network buffers for large ROS2 messages ──
echo "[4/9] Configuring network buffers (26MB for CycloneDDS)..."
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.core.rmem_default=26214400
sudo sysctl -w net.core.wmem_max=2147483647
sudo sysctl -w net.core.wmem_default=26214400

# ── 5. Disable ZRAM swap (unnecessary with 64GB) ──
echo "[5/9] Disabling ZRAM (64GB RAM = no swap needed)..."
if systemctl is-active --quiet nvzramconfig 2>/dev/null; then
    sudo systemctl stop nvzramconfig
    sudo systemctl disable nvzramconfig
    echo "  ZRAM disabled."
else
    echo "  ZRAM already disabled."
fi

# Free any existing swap
sudo swapoff -a 2>/dev/null || true

# ── 6. Set real-time scheduling limits ──
echo "[6/9] Setting RT scheduling limits..."
if ! grep -q "rtprio 99" /etc/security/limits.d/ros-realtime.conf 2>/dev/null; then
    sudo bash -c 'cat > /etc/security/limits.d/ros-realtime.conf << EOF
# ROS2 real-time scheduling for SLAM pipeline
*    soft    rtprio    99
*    hard    rtprio    99
*    soft    memlock   unlimited
*    hard    memlock   unlimited
*    soft    nice      -20
*    hard    nice      -20
EOF'
    echo "  RT limits set. LOG OUT AND BACK IN for them to take effect."
else
    echo "  RT limits already configured."
fi

# ── 7. Environment variables ──
echo "[7/9] Setting ROS2 environment..."
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CYCLONE_CONFIG="${SCRIPT_DIR}/../config/cyclonedds.xml"

if [ -f "$CYCLONE_CONFIG" ]; then
    export CYCLONEDDS_URI="file://${CYCLONE_CONFIG}"
    echo "  CycloneDDS config: ${CYCLONE_CONFIG}"
else
    echo "  WARNING: CycloneDDS config not found at ${CYCLONE_CONFIG}"
fi

# ── 8. Thread affinity setup script ──
echo "[8/9] Creating thread affinity configuration..."
sudo bash -c 'cat > /usr/local/bin/slam_thread_affinity.sh << "SCRIPT"
#!/bin/bash
# =============================================================================
# Thread Affinity for SLAM Pipeline — Deterministic Core Assignment
# =============================================================================
# Orin AGX 64GB has 12 ARM Cortex-A78AE cores:
#   Cores 0-3:   cuVSLAM (high-priority visual odometry)
#   Cores 4-5:   Depth filter (preprocessing)
#   Cores 6-9:   RTAB-Map (mapping + graph optimization)
#   Cores 10-11: ZED driver + system services
#
# Run this AFTER the SLAM pipeline is launched and all nodes are running.
# Usage: sudo /usr/local/bin/slam_thread_affinity.sh
# =============================================================================

set -e

echo "Setting SLAM thread affinity..."

# Find PIDs by node name
assign_affinity() {
    local node_name="$1"
    local cpuset="$2"
    local priority="$3"

    # Find PID(s) matching the node name
    local pids=$(pgrep -f "$node_name" 2>/dev/null || true)

    if [ -z "$pids" ]; then
        echo "  [SKIP] $node_name not found"
        return
    fi

    for pid in $pids; do
        # Set CPU affinity
        taskset -cp "$cpuset" "$pid" 2>/dev/null || true

        # Set RT priority if requested (SCHED_FIFO)
        if [ "$priority" -gt 0 ] 2>/dev/null; then
            chrt -f -p "$priority" "$pid" 2>/dev/null || true
            echo "  [OK] $node_name (PID $pid) → cores $cpuset, RT priority $priority"
        else
            echo "  [OK] $node_name (PID $pid) → cores $cpuset"
        fi
    done
}

# Assign affinity for each SLAM component
assign_affinity "visual_slam_node" "0-3" 80     # cuVSLAM: perf cores + RT priority
assign_affinity "depth_filter_node" "4-5" 60     # Depth filter: dedicated cores
assign_affinity "rtabmap" "6-9" 40               # RTAB-Map: 4 cores for optimization
assign_affinity "zed_node" "10-11" 0             # ZED: background cores
assign_affinity "pipeline_watchdog" "10-11" 0    # Watchdog: background
assign_affinity "slam_monitor" "10-11" 0         # Monitor: background
assign_affinity "map_quality" "10-11" 0          # Quality: background
assign_affinity "foxglove_bridge" "10-11" 0      # Foxglove: background

echo "Thread affinity configuration complete."
SCRIPT'
sudo chmod +x /usr/local/bin/slam_thread_affinity.sh
echo "  Thread affinity script: /usr/local/bin/slam_thread_affinity.sh"

# ── 9. Create directories ──
echo "[9/9] Creating data directories..."

# NVMe SSD directories for performance
sudo mkdir -p /mnt/ssd/slam_maps
sudo mkdir -p /mnt/ssd/slam_logs
sudo chown -R $(whoami):$(whoami) /mnt/ssd/slam_maps /mnt/ssd/slam_logs 2>/dev/null || true

# Legacy directory (backward compat)
mkdir -p /home/orza/slam_maps 2>/dev/null || true

echo "  Map directory:  /mnt/ssd/slam_maps/"
echo "  Log directory:  /mnt/ssd/slam_logs/"

# ── Summary ──
echo ""
echo "═══════════════════════════════════════════════"
echo "  Setup Complete!"
echo "═══════════════════════════════════════════════"
echo ""
echo "  Power mode:       MAXN (all 12 cores)"
echo "  Clocks:           Locked at max"
echo "  Network buffers:  26MB rmem/wmem"
echo "  ZRAM:             Disabled"
echo "  DDS:              CycloneDDS"
echo "  RT scheduling:    Enabled (rtprio 99)"
echo "  Thread affinity:  /usr/local/bin/slam_thread_affinity.sh"
echo ""
echo "  Free RAM: $(free -h | awk '/Mem:/ {print $4}') available"
echo "  CPU cores: $(nproc)"
echo ""
echo "  Next steps:"
echo "    1. ros2 launch agv_slam agv_slam.launch.py"
echo "    2. Wait 15s for all nodes to start"
echo "    3. sudo /usr/local/bin/slam_thread_affinity.sh"
echo ""
