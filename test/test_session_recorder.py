"""Unit tests for session_recorder.py pure functions."""

import os
import sys
import tempfile
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from session_recorder import file_checksum, dir_size_gb, build_manifest


class TestFileChecksum:
    """Test SHA256 file checksum computation."""

    def test_deterministic(self, tmp_path):
        """Same content produces same checksum."""
        f = tmp_path / "test.bin"
        f.write_bytes(b"hello world")
        c1 = file_checksum(str(f))
        c2 = file_checksum(str(f))
        assert c1 == c2
        assert len(c1) == 64  # SHA256 hex length

    def test_different_content(self, tmp_path):
        """Different content produces different checksums."""
        f1 = tmp_path / "a.bin"
        f2 = tmp_path / "b.bin"
        f1.write_bytes(b"content A")
        f2.write_bytes(b"content B")
        assert file_checksum(str(f1)) != file_checksum(str(f2))

    def test_nonexistent_file(self):
        """Non-existent file returns None."""
        result = file_checksum("/nonexistent/path/file.bin")
        assert result is None

    def test_empty_file(self, tmp_path):
        """Empty file still produces a valid checksum."""
        f = tmp_path / "empty.bin"
        f.write_bytes(b"")
        c = file_checksum(str(f))
        assert c is not None
        assert len(c) == 64


class TestDirSizeGb:
    """Test directory size calculation."""

    def test_known_size(self, tmp_path):
        """Directory with known file sizes."""
        (tmp_path / "a.bin").write_bytes(b"\x00" * 1024)
        (tmp_path / "b.bin").write_bytes(b"\x00" * 2048)
        size = dir_size_gb(str(tmp_path))
        expected_gb = 3072 / (1024 ** 3)
        assert abs(size - expected_gb) < 1e-9

    def test_empty_dir(self, tmp_path):
        """Empty directory returns 0."""
        assert dir_size_gb(str(tmp_path)) == 0.0

    def test_nonexistent_dir(self):
        """Non-existent directory returns 0."""
        assert dir_size_gb("/nonexistent/dir") == 0.0


class TestBuildManifest:
    """Test manifest JSON structure."""

    def test_manifest_structure(self, tmp_path):
        """Manifest has required keys and correct file count."""
        (tmp_path / "data.bin").write_bytes(b"test data")
        sub = tmp_path / "subdir"
        sub.mkdir()
        (sub / "nested.txt").write_text("nested content")

        manifest = build_manifest(str(tmp_path), session_name="test_session")

        assert manifest['session_name'] == 'test_session'
        assert 'timestamp' in manifest
        assert manifest['total_files'] == 2
        assert manifest['total_size_gb'] >= 0
        assert len(manifest['files']) == 2

        # Each file entry should have path, size_bytes, checksum
        for f_entry in manifest['files']:
            assert 'path' in f_entry
            assert 'size_bytes' in f_entry
            assert 'checksum' in f_entry
            assert f_entry['size_bytes'] > 0
            assert f_entry['checksum'] is not None
