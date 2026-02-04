#!/usr/bin/env python3
# =============================================================================
# Storage Monitor — Disk space checker for recording volume
# =============================================================================
# One-shot mode: report current usage and estimated recording time
# Daemon mode:   continuously monitor and warn on low space
#
# Usage:
#   ./check_storage.py                     # One-shot report
#   ./check_storage.py --daemon            # Continuous monitoring
#   ./check_storage.py --path /mnt/ssd     # Custom path
# =============================================================================

import argparse
import os
import shutil
import time
import sys


# Estimated data rates (GB/min) at HD1080 30fps
RATE_BAG_GB_MIN = 2.5     # Full topic set ROS2 bag
RATE_SVO_GB_MIN = 0.8     # H265 SVO2
RATE_TOTAL_GB_MIN = RATE_BAG_GB_MIN + RATE_SVO_GB_MIN


def get_storage_info(path):
    """Retrieve disk usage statistics for a given path."""
    stat = shutil.disk_usage(path)
    return {
        'total_gb': stat.total / (1024 ** 3),
        'used_gb': stat.used / (1024 ** 3),
        'free_gb': stat.free / (1024 ** 3),
        'used_pct': (stat.used / stat.total) * 100,
    }


def count_sessions(path):
    """Count recording sessions and their total size on disk.

    Returns:
        Tuple of (session_count, total_size_gb).
    """
    sessions_dir = os.path.join(path, 'sessions') if not path.endswith('sessions') else path
    if not os.path.exists(sessions_dir):
        return 0, 0.0
    count = 0
    total_size = 0
    for entry in os.scandir(sessions_dir):
        if entry.is_dir() and entry.name.startswith('session_'):
            count += 1
            for dirpath, _, filenames in os.walk(entry.path):
                for f in filenames:
                    total_size += os.path.getsize(os.path.join(dirpath, f))
    return count, total_size / (1024 ** 3)


def estimate_recording_time_hours(path, gb_per_hour=2.0):
    """Estimate how many hours of recording time remain based on free disk space.

    Args:
        path: Filesystem path to check.
        gb_per_hour: Estimated data rate in GB/hour.

    Returns:
        Estimated hours of recording time remaining.
    """
    info = get_storage_info(path)
    if gb_per_hour <= 0:
        return float('inf')
    return info['free_gb'] / gb_per_hour


def print_report(path):
    info = get_storage_info(path)
    n_sessions, session_gb = count_sessions(path)

    est_minutes = info['free_gb'] / RATE_TOTAL_GB_MIN if RATE_TOTAL_GB_MIN > 0 else 0

    print(f'{"="*50}')
    print(f'Storage Report: {path}')
    print(f'{"="*50}')
    print(f'  Total:     {info["total_gb"]:.1f} GB')
    print(f'  Used:      {info["used_gb"]:.1f} GB ({info["used_pct"]:.1f}%)')
    print(f'  Free:      {info["free_gb"]:.1f} GB')
    print(f'  Sessions:  {n_sessions} ({session_gb:.1f} GB)')
    print(f'{"="*50}')
    print(f'  Est. recording time remaining: {est_minutes:.0f} minutes')
    print(f'  (at ~{RATE_TOTAL_GB_MIN:.1f} GB/min: bag + SVO2)')
    print(f'{"="*50}')

    if info['free_gb'] < 5.0:
        print(f'  *** WARNING: LOW DISK SPACE ({info["free_gb"]:.1f} GB free) ***')
    elif info['free_gb'] < 20.0:
        print(f'  Note: Consider archiving old sessions')


def daemon_mode(path, interval=30):
    print(f'Storage monitor running (checking every {interval}s)...')
    print(f'Path: {path}')
    while True:
        info = get_storage_info(path)
        free = info['free_gb']
        est = free / RATE_TOTAL_GB_MIN
        status = 'OK' if free > 20 else 'LOW' if free > 5 else 'CRITICAL'
        print(f'[{time.strftime("%H:%M:%S")}] '
              f'Free: {free:.1f} GB | '
              f'Est. remaining: {est:.0f} min | '
              f'Status: {status}')
        if free < 5.0:
            print(f'  *** WARNING: Only {free:.1f} GB free! ***')
        time.sleep(interval)


def main():
    parser = argparse.ArgumentParser(description='Storage monitor for recording volume')
    parser.add_argument('--path', default='/mnt/ssd', help='Path to monitor')
    parser.add_argument('--daemon', action='store_true', help='Continuous monitoring mode')
    parser.add_argument('--interval', type=int, default=30, help='Check interval in seconds')
    args = parser.parse_args()

    if not os.path.exists(args.path):
        print(f'ERROR: Path not found: {args.path}')
        sys.exit(1)

    if args.daemon:
        daemon_mode(args.path, args.interval)
    else:
        print_report(args.path)


if __name__ == '__main__':
    main()
