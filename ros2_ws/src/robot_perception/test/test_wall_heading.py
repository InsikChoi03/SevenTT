"""Unit tests for arena-wall heading correction geometry."""
import math

import pytest

from robot_perception.wall_heading import classify_wall_segments


def _segment(angle_deg: float, length: float = 1.0):
    angle = math.radians(angle_deg)
    return (0.0, 0.0, length * math.cos(angle), length * math.sin(angle))


def test_positive_ten_degree_wall_produces_negative_ten_degree_correction():
    """A positive horizontal-wall tilt must rotate the field estimate negatively."""
    _, estimate = classify_wall_segments([_segment(10.0)])

    assert estimate is not None
    assert math.degrees(estimate.correction_rad) == pytest.approx(-10.0, abs=1e-6)


def test_negative_ten_degree_wall_produces_positive_ten_degree_correction():
    """A negative horizontal-wall tilt must rotate the field estimate positively."""
    _, estimate = classify_wall_segments([_segment(-10.0)])

    assert estimate is not None
    assert math.degrees(estimate.correction_rad) == pytest.approx(10.0, abs=1e-6)


def test_vertical_wall_uses_same_undirected_sign_convention():
    """A vertical undirected wall must use the same correction sign."""
    _, estimate = classify_wall_segments([_segment(100.0)])

    assert estimate is not None
    assert math.degrees(estimate.correction_rad) == pytest.approx(-10.0, abs=1e-6)


def test_angular_variance_reports_segment_disagreement():
    """Angular variance must expose disagreement between accepted segments."""
    _, estimate = classify_wall_segments([_segment(8.0), _segment(12.0)])

    assert estimate is not None
    assert estimate.segment_count == 2
    assert math.degrees(estimate.correction_rad) == pytest.approx(-10.0, abs=1e-6)
    assert estimate.variance_rad2 == pytest.approx(math.radians(2.0) ** 2, rel=1e-6)
