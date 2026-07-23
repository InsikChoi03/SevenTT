"""Regression tests for filtering robot-fixed tray detections from the wide map."""

import math

from robot_perception.nodes.world_model_node import (
    point_inside_axis_aligned_rect,
    point_inside_field_bounds,
)


def test_tray_projection_is_inside_robot_self_exclusion():
    rect = [-0.30, 0.08, -0.28, 0.28]

    assert point_inside_axis_aligned_rect(-0.108, 0.012, rect)


def test_real_floor_objects_outside_chassis_are_not_excluded():
    rect = [-0.30, 0.08, -0.28, 0.28]

    assert not point_inside_axis_aligned_rect(0.19, 0.0, rect)
    assert not point_inside_axis_aligned_rect(0.50, 0.0, rect)
    assert not point_inside_axis_aligned_rect(0.0, 0.50, rect)


def test_field_bounds_reject_projection_glitches_before_track_birth():
    bounds = (-2.0, 2.0, -2.0, 2.0)

    assert point_inside_field_bounds(-2.0, 2.0, bounds)
    assert point_inside_field_bounds(0.4, -1.1, bounds)
    assert not point_inside_field_bounds(-2.89, 0.0, bounds)
    assert not point_inside_field_bounds(0.0, 3.599, bounds)
    assert not point_inside_field_bounds(math.nan, 0.0, bounds)
