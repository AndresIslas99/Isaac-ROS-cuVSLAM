#!/bin/bash
# =============================================================================
# AGV SLAM — One-Command Launch
# =============================================================================
# Launches the full pipeline on the Jetson and opens Foxglove Studio locally
# with the pre-configured layout.
#
# Usage:
#   ./scripts/run_slam.sh                    # mapping mode
#   ./scripts/run_slam.sh --localization     # localization mode
#   ./scripts/run_slam.sh --stop             # stop everything
# =============================================================================

set -e

JETSON_USER="orza"
JETSON_HOST="192.168.55.1"
JETSON_SSH="${JETSON_USER}@${JETSON_HOST}"
FOXGLOVE_URL="foxglove-studio-demo://open?ds=foxglove-websocket&ds.url=ws%3A%2F%2F${JETSON_HOST}%3A8765"
WS_URL="ws://${JETSON_HOST}:8765"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAYOUT_FILE="${SCRIPT_DIR}/../foxglove/agv-slam-layout.json"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log()  { echo -e "${GREEN}[SLAM]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
err()  { echo -e "${RED}[ERR]${NC} $1"; }

# ── Stop mode ──
if [[ "$1" == "--stop" ]]; then
    log "Stopping pipeline on Jetson..."
    ssh "${JETSON_SSH}" 'kill -9 $(ps aux | grep -E "ros2|component_container|rtabmap|depth_filter|slam_monitor|watchdog|foxglove" | grep -v grep | grep -v sshd | awk "{print \$2}") 2>/dev/null; echo "Pipeline stopped"' 2>&1
    pkill -f foxglove-studio 2>/dev/null || true
    log "Done."
    exit 0
fi

# ── Parse args ──
LAUNCH_ARGS=""
if [[ "$1" == "--localization" ]]; then
    LAUNCH_ARGS="localization:=true"
    log "Mode: LOCALIZATION (using existing map)"
else
    log "Mode: MAPPING (building new map)"
fi

# ── Check Jetson connectivity ──
log "Connecting to Jetson at ${JETSON_HOST}..."
if ! ssh -o ConnectTimeout=5 "${JETSON_SSH}" "echo ok" &>/dev/null; then
    err "Cannot reach Jetson at ${JETSON_HOST}"
    exit 1
fi
log "Jetson connected."

# ── Kill any existing pipeline ──
log "Stopping existing pipeline..."
ssh "${JETSON_SSH}" 'kill -9 $(ps aux | grep -E "ros2|component_container|rtabmap|depth_filter|slam_monitor|watchdog|foxglove" | grep -v grep | grep -v sshd | awk "{print \$2}") 2>/dev/null' 2>/dev/null || true
sleep 2

# ── Launch pipeline on Jetson ──
log "Launching SLAM pipeline on Jetson..."
ssh "${JETSON_SSH}" "bash -c '
    source /opt/ros/humble/setup.bash
    source /mnt/ssd/ros2_ws/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    nohup ros2 launch agv_slam agv_slam.launch.py ${LAUNCH_ARGS} > /tmp/agv_slam_launch.log 2>&1 &
    echo \$!
'" &>/dev/null

# ── Wait for pipeline to initialize ──
log "Waiting for pipeline to initialize..."
for i in $(seq 1 30); do
    if ssh "${JETSON_SSH}" "grep -q 'Server listening on port 8765' /tmp/agv_slam_launch.log 2>/dev/null"; then
        break
    fi
    sleep 1
    printf "."
done
echo ""

# ── Verify pipeline health ──
HEALTH=$(ssh "${JETSON_SSH}" 'grep "SLAM Status" /tmp/agv_slam_launch.log 2>/dev/null | tail -1' 2>/dev/null)
if echo "${HEALTH}" | grep -q "OK"; then
    log "Pipeline: ${GREEN}HEALTHY${NC}"
elif echo "${HEALTH}" | grep -q "WARN"; then
    warn "Pipeline: ${YELLOW}WARNING${NC} (still initializing)"
else
    warn "Pipeline starting up, status not yet available..."
fi
echo -e "  ${CYAN}${HEALTH}${NC}"

# ── Check if Foxglove bridge is ready ──
if ssh "${JETSON_SSH}" "grep -q 'Server listening on port 8765' /tmp/agv_slam_launch.log 2>/dev/null"; then
    log "Foxglove bridge: ${GREEN}READY${NC} at ${WS_URL}"
else
    warn "Foxglove bridge not yet ready, waiting..."
    sleep 10
fi

# ── Open Foxglove Studio (desktop app) ──
log "Opening Foxglove Studio..."
killall foxglove-studio 2>/dev/null || true
sleep 1
snap run foxglove-studio &>/dev/null &
disown
sleep 3
log "Foxglove Studio opened."
log "Connect: ${CYAN}Open connection → Foxglove WebSocket → ${WS_URL}${NC}"
if [[ -f "${LAYOUT_FILE}" ]]; then
    log "Import layout: ${CYAN}Layout menu → Import → foxglove/agv-slam-layout.json${NC}"
fi

echo ""
log "════════════════════════════════════════"
log " SLAM Pipeline Running"
log " Foxglove: ${CYAN}${WS_URL}${NC}"
log " Stop:     ${CYAN}./scripts/run_slam.sh --stop${NC}"
log " Logs:     ${CYAN}ssh ${JETSON_SSH} tail -f /tmp/agv_slam_launch.log${NC}"
log "════════════════════════════════════════"
