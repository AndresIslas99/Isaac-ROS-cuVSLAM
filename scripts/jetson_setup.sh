#!/bin/bash
# =============================================================================
# Jetson Orin AGX 64GB — Performance Setup for SLAM Pipeline
# =============================================================================
# Run ONCE after boot (or add to systemd service).
# Requires sudo.
# =============================================================================

set -e

echo "═══════════════════════════════════════════════"
echo "  AGV SLAM — Jetson Orin AGX Performance Setup"
echo "═══════════════════════════════════════════════"

# ── 1. Power mode: MAXN (all cores, max clocks) ──
echo "[1/7] Setting MAXN power mode..."
sudo nvpmodel -m 0
sleep 2

# ── 2. Lock clocks at maximum ──
echo "[2/7] Locking clocks at maximum..."
sudo jetson_clocks
sleep 1

# ── 3. Verify GPU is available ──
echo "[3/7] Checking GPU..."
if command -v nvidia-smi &> /dev/null; then
    nvidia-smi
elif [ -f /usr/bin/tegrastats ]; then
    echo "Tegra platform detected. GPU check via tegrastats."
    timeout 2 tegrastats || true
else
    echo "WARNING: Cannot verify GPU. Ensure CUDA is available."
fi

# ── 4. Network buffers for large ROS2 messages ──
echo "[4/7] Configuring network buffers..."
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.core.rmem_default=8388608
sudo sysctl -w net.core.wmem_max=2147483647
sudo sysctl -w net.core.wmem_default=8388608

# ── 5. Disable ZRAM swap (unnecessary with 64GB) ──
echo "[5/7] Disabling ZRAM (64GB RAM = no swap needed)..."
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
echo "[6/7] Setting RT scheduling limits..."
if ! grep -q "rtprio 99" /etc/security/limits.d/ros-realtime.conf 2>/dev/null; then
    sudo bash -c 'cat > /etc/security/limits.d/ros-realtime.conf << EOF
# ROS2 real-time scheduling for SLAM pipeline
*    soft    rtprio    99
*    hard    rtprio    99
*    soft    memlock   unlimited
*    hard    memlock   unlimited
EOF'
    echo "  RT limits set. LOG OUT AND BACK IN for them to take effect."
else
    echo "  RT limits already configured."
fi

# ── 7. Environment variables ──
echo "[7/7] Setting ROS2 environment..."
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CYCLONE_CONFIG="${SCRIPT_DIR}/../config/cyclonedds.xml"

if [ -f "$CYCLONE_CONFIG" ]; then
    export CYCLONEDDS_URI="file://${CYCLONE_CONFIG}"
    echo "  CycloneDDS config: ${CYCLONE_CONFIG}"
else
    echo "  WARNING: CycloneDDS config not found at ${CYCLONE_CONFIG}"
fi

# ── Summary ──
echo ""
echo "═══════════════════════════════════════════════"
echo "  Setup Complete!"
echo "═══════════════════════════════════════════════"
echo ""
echo "  Power mode:     MAXN"
echo "  Clocks:         Locked at max"
echo "  Network buffers: 2GB rmem/wmem"
echo "  ZRAM:           Disabled"
echo "  DDS:            CycloneDDS"
echo ""
echo "  Free RAM: $(free -h | awk '/Mem:/ {print $4}') available"
echo "  CPU cores: $(nproc)"
echo ""
echo "  Next: ros2 launch agv_slam agv_slam.launch.py"
echo ""

# Create SLAM maps directory
mkdir -p /home/orza/slam_maps
echo "  Map directory: /home/orza/slam_maps/"
echo ""
