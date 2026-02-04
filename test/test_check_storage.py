"""Unit tests for check_storage.py pure functions."""

import os
import sys
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from check_storage import get_storage_info, count_sessions, estimate_recording_time_hours


class TestGetStorageInfo:
    """Test disk usage information retrieval."""

    def test_returns_dict_with_keys(self):
        """get_storage_info returns dict with expected keys."""
        info = get_storage_info("/")
        assert isinstance(info, dict)
        assert 'total_gb' in info
        assert 'used_gb' in info
        assert 'free_gb' in info
        assert 'used_pct' in info

    def test_values_positive(self):
        """All storage values should be positive."""
        info = get_storage_info("/")
        assert info['total_gb'] > 0
        assert info['used_gb'] >= 0
        assert info['free_gb'] >= 0
        assert 0 <= info['used_pct'] <= 100

    def test_used_plus_free_le_total(self):
        """Used + free should not exceed total (difference is reserved blocks)."""
        info = get_storage_info("/")
        assert info['used_gb'] + info['free_gb'] <= info['total_gb'] + 0.01


class TestCountSessions:
    """Test session counting and size calculation."""

    def test_no_sessions_dir(self, tmp_path):
        """Non-existent sessions directory returns (0, 0.0)."""
        count, size = count_sessions(str(tmp_path))
        assert count == 0
        assert size == 0.0

    def test_counts_session_dirs(self, tmp_path):
        """Correctly counts directories starting with 'session_'."""
        sessions = tmp_path / "sessions"
        sessions.mkdir()

        # Create session directories
        for i in range(3):
            s = sessions / f"session_{i:04d}"
            s.mkdir()
            (s / "data.bin").write_bytes(b"\x00" * 1024)

        # Create a non-session directory (should be ignored)
        other = sessions / "other_dir"
        other.mkdir()
        (other / "file.bin").write_bytes(b"\x00" * 1024)

        count, size = count_sessions(str(tmp_path))
        assert count == 3
        assert size > 0


class TestEstimateRecordingTime:
    """Test recording time estimation."""

    def test_positive_estimate(self):
        """Should return a positive number of hours."""
        hours = estimate_recording_time_hours("/", gb_per_hour=2.0)
        assert hours > 0

    def test_zero_rate_returns_inf(self):
        """Zero data rate should return infinity."""
        hours = estimate_recording_time_hours("/", gb_per_hour=0.0)
        assert hours == float('inf')
