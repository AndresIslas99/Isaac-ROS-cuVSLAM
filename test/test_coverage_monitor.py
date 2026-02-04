"""Unit tests for coverage_monitor.py pure functions."""

import math
import sys
import os
import pytest

# Add scripts directory to path so we can import directly
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from coverage_monitor import yaw_from_quaternion, world_to_grid, CoverageGrid


class TestYawFromQuaternion:
    """Test quaternion-to-yaw conversion."""

    def test_zero_yaw(self):
        """Identity quaternion (0,0,0,1) should give yaw=0."""
        yaw = yaw_from_quaternion(0.0, 0.0, 0.0, 1.0)
        assert abs(yaw) < 1e-9

    def test_90_degrees(self):
        """Quaternion for 90° yaw (around Z)."""
        # q = (0, 0, sin(45°), cos(45°))
        angle = math.pi / 2
        q_z = math.sin(angle / 2)
        q_w = math.cos(angle / 2)
        yaw = yaw_from_quaternion(0.0, 0.0, q_z, q_w)
        assert abs(yaw - math.pi / 2) < 1e-6

    def test_180_degrees(self):
        """Quaternion for 180° yaw."""
        angle = math.pi
        q_z = math.sin(angle / 2)
        q_w = math.cos(angle / 2)
        yaw = yaw_from_quaternion(0.0, 0.0, q_z, q_w)
        assert abs(abs(yaw) - math.pi) < 1e-6

    def test_negative_90(self):
        """Quaternion for -90° yaw."""
        angle = -math.pi / 2
        q_z = math.sin(angle / 2)
        q_w = math.cos(angle / 2)
        yaw = yaw_from_quaternion(0.0, 0.0, q_z, q_w)
        assert abs(yaw - (-math.pi / 2)) < 1e-6


class TestWorldToGrid:
    """Test world-to-grid coordinate conversion."""

    def test_origin(self):
        """World origin maps to grid (0, 0) when x_min=y_min=0."""
        gx, gy = world_to_grid(0.0, 0.0, 0.0, 0.0, 0.05)
        assert gx == 0
        assert gy == 0

    def test_positive_offset(self):
        """1m offset with 5cm resolution = cell 20."""
        gx, gy = world_to_grid(1.0, 2.0, 0.0, 0.0, 0.05)
        assert gx == 20
        assert gy == 40

    def test_negative_origin(self):
        """Grid with negative world origin."""
        gx, gy = world_to_grid(0.0, 0.0, -5.0, -5.0, 0.1)
        assert gx == 50
        assert gy == 50


class TestCoverageGrid:
    """Test coverage grid state transitions and multi-angle tracking."""

    def test_initial_state_unvisited(self):
        grid = CoverageGrid(0.0, 10.0, 0.0, 10.0, 1.0, angles_required=4)
        assert grid.get_state(0, 0) == CoverageGrid.UNVISITED

    def test_partial_after_one_angle(self):
        grid = CoverageGrid(0.0, 10.0, 0.0, 10.0, 1.0, angles_required=4)
        grid.mark_observed(0, 0, angle_bin=0)
        assert grid.get_state(0, 0) == CoverageGrid.PARTIAL

    def test_covered_after_all_angles(self):
        grid = CoverageGrid(0.0, 10.0, 0.0, 10.0, 1.0, angles_required=4)
        for angle_bin in range(4):
            grid.mark_observed(5, 5, angle_bin)
        assert grid.get_state(5, 5) == CoverageGrid.COVERED

    def test_coverage_percent(self):
        grid = CoverageGrid(0.0, 2.0, 0.0, 2.0, 1.0, angles_required=2)
        # Grid is 2x2 = 4 cells. Mark one cell as fully covered.
        grid.mark_observed(0, 0, 0)
        grid.mark_observed(0, 0, 1)
        assert grid.get_state(0, 0) == CoverageGrid.COVERED
        assert abs(grid.coverage_percent() - 25.0) < 0.1
