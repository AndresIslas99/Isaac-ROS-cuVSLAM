#!/bin/bash
# =============================================================================
# Export Session — Package recorded data for transfer to RTX laptop
# =============================================================================
# Validates session, optionally compresses, prints transfer instructions.
#
# Usage:
#   ./export_session.sh /mnt/ssd/sessions/session_20260203_171239
#   ./export_session.sh /mnt/ssd/sessions/session_20260203_171239 --compress
# =============================================================================

set -euo pipefail

SESSION_DIR="${1:-}"
COMPRESS="${2:-}"

if [ -z "$SESSION_DIR" ]; then
    echo "Usage: $0 <session_directory> [--compress]"
    echo "Example: $0 /mnt/ssd/sessions/session_20260203_171239"
    exit 1
fi

if [ ! -d "$SESSION_DIR" ]; then
    echo "ERROR: Directory not found: $SESSION_DIR"
    exit 1
fi

SESSION_NAME=$(basename "$SESSION_DIR")
echo "=== Validating session: $SESSION_NAME ==="

# Check required files
ERRORS=0
check_file() {
    if [ -e "$1" ]; then
        SIZE=$(du -sh "$1" 2>/dev/null | cut -f1)
        echo "  [OK] $2 ($SIZE)"
    else
        echo "  [MISSING] $2: $1"
        ERRORS=$((ERRORS + 1))
    fi
}

check_file "$SESSION_DIR/manifest.json" "Session manifest"
check_file "$SESSION_DIR/trajectory.tum" "TUM trajectory"
check_file "$SESSION_DIR/bag" "ROS2 bag directory"

# Optional files
[ -f "$SESSION_DIR"/*.svo2 ] 2>/dev/null && \
    check_file "$SESSION_DIR"/*.svo2 "SVO2 recording" || \
    echo "  [SKIP] No SVO2 file (optional)"

[ -f "$SESSION_DIR/coverage.ply" ] && \
    check_file "$SESSION_DIR/coverage.ply" "Coverage PLY mesh" || \
    echo "  [SKIP] No PLY file (optional)"

# Validate manifest
if [ -f "$SESSION_DIR/manifest.json" ]; then
    echo ""
    echo "=== Manifest ==="
    python3 -c "
import json, sys
with open('$SESSION_DIR/manifest.json') as f:
    m = json.load(f)
required = ['session_id', 'created', 'frame_count']
for k in required:
    if k not in m:
        print(f'  [WARN] Missing key: {k}')
        sys.exit(1)
print(f'  Session ID: {m.get(\"session_id\", \"N/A\")}')
print(f'  Created: {m.get(\"created\", \"N/A\")}')
print(f'  Duration: {m.get(\"duration_s\", \"N/A\")}s')
print(f'  Frames: {m.get(\"frame_count\", \"N/A\")}')
print(f'  Size: {m.get(\"total_size_gb\", \"N/A\")} GB')
if m.get('checksums'):
    for fname, csum in m['checksums'].items():
        print(f'  Checksum [{fname}]: {csum[:16]}...')
" 2>/dev/null || echo "  [WARN] Manifest parse failed"
fi

# Validate TUM trajectory format
if [ -f "$SESSION_DIR/trajectory.tum" ]; then
    LINES=$(grep -v '^#' "$SESSION_DIR/trajectory.tum" | wc -l)
    echo ""
    echo "=== Trajectory ==="
    echo "  Poses: $LINES"
    # Check first data line has 8 values
    FIRST=$(grep -v '^#' "$SESSION_DIR/trajectory.tum" | head -1)
    COLS=$(echo "$FIRST" | awk '{print NF}')
    if [ "$COLS" = "8" ]; then
        echo "  Format: OK (8 columns: t tx ty tz qx qy qz qw)"
    else
        echo "  [WARN] Expected 8 columns, got $COLS"
        ERRORS=$((ERRORS + 1))
    fi
fi

# Total size
echo ""
TOTAL_SIZE=$(du -sh "$SESSION_DIR" | cut -f1)
echo "=== Total size: $TOTAL_SIZE ==="

if [ $ERRORS -gt 0 ]; then
    echo ""
    echo "WARNING: $ERRORS validation errors found!"
fi

# Compress if requested
if [ "$COMPRESS" = "--compress" ]; then
    ARCHIVE="/mnt/ssd/sessions/${SESSION_NAME}.tar.zst"
    echo ""
    echo "=== Compressing to $ARCHIVE ==="
    if command -v zstd &>/dev/null; then
        tar -C "$(dirname "$SESSION_DIR")" -cf - "$SESSION_NAME" | \
            zstd -T0 -3 -o "$ARCHIVE"
    else
        ARCHIVE="/mnt/ssd/sessions/${SESSION_NAME}.tar.gz"
        tar -czf "$ARCHIVE" -C "$(dirname "$SESSION_DIR")" "$SESSION_NAME"
    fi
    ARCHIVE_SIZE=$(du -sh "$ARCHIVE" | cut -f1)
    echo "  Archive: $ARCHIVE ($ARCHIVE_SIZE)"
fi

# Transfer instructions
echo ""
echo "=== Transfer Instructions ==="
HOSTNAME=$(hostname)
USER=$(whoami)
echo "  From RTX laptop, run:"
echo ""
if [ -n "${COMPRESS:-}" ] && [ -f "${ARCHIVE:-}" ]; then
    echo "  rsync -avP ${USER}@${HOSTNAME}:${ARCHIVE} ~/greenhouse_sessions/"
    echo ""
    echo "  Then extract:"
    echo "  cd ~/greenhouse_sessions && tar -xf ${SESSION_NAME}.tar.*"
else
    echo "  rsync -avP ${USER}@${HOSTNAME}:${SESSION_DIR}/ ~/greenhouse_sessions/${SESSION_NAME}/"
fi
echo ""
echo "=== Done ==="
