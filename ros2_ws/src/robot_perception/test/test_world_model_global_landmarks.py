"""Test global object-landmark matching and 50 cm alias rejection."""

import pytest

from robot_perception.nodes.world_model_node import match_global_landmarks


def _match(observations, anchors):
    return match_global_landmarks(
        observations,
        anchors,
        min_pairs=4,
        residual_gate=0.12,
        max_offset=0.75,
        grid_spacing=0.50,
        alias_tolerance=0.08,
        alias_residual_margin=0.02,
    )


def test_periodic_rectangle_one_row_apart_is_held_as_ambiguous():
    """Hold a rectangle that matches two grid rows equally well."""
    observations = [(0.0, 0.0), (0.5, 0.0), (0.0, 1.0), (0.5, 1.0)]
    anchors = [
        (1, 0.0, 0.0),
        (2, 0.5, 0.0),
        (3, 0.0, 1.0),
        (4, 0.5, 1.0),
        (5, 0.0, 0.5),
        (6, 0.5, 0.5),
        (7, 0.0, 1.5),
        (8, 0.5, 1.5),
    ]

    result = _match(observations, anchors)

    assert result is not None
    assert result.ambiguous
    assert abs(result.alternate_dy - result.dy) == pytest.approx(0.50)


def test_distinctive_global_landmark_resolves_periodic_local_four_points():
    """Use an extra global object to resolve two local grid hypotheses."""
    observations = [
        (0.0, 0.0),
        (0.5, 0.0),
        (0.0, 1.0),
        (0.5, 1.0),
        (1.7, 0.2),
    ]
    anchors = [
        (1, 0.0, 0.0),
        (2, 0.5, 0.0),
        (3, 0.0, 1.0),
        (4, 0.5, 1.0),
        (5, 0.0, 0.5),
        (6, 0.5, 0.5),
        (7, 0.0, 1.5),
        (8, 0.5, 1.5),
        (9, 1.7, 0.2),
    ]

    result = _match(observations, anchors)

    assert result is not None
    assert not result.ambiguous
    assert len(result.pairs) == 5
    assert result.dx == pytest.approx(0.0)
    assert result.dy == pytest.approx(0.0)


def test_duplicate_detections_cannot_count_as_multiple_landmarks():
    """Count each persistent track at most once per camera frame."""
    observations = [
        (-0.18, 0.0),
        (-0.17, 0.01),
        (0.32, 0.0),
        (-0.18, 0.5),
        (0.32, 0.5),
    ]
    anchors = [
        (1, 0.0, 0.0),
        (2, 0.5, 0.0),
        (3, 0.0, 0.5),
        (4, 0.5, 0.5),
    ]

    result = _match(observations, anchors)

    assert result is not None
    assert not result.ambiguous
    assert len(result.pairs) == 4
    assert len({pair[0] for pair in result.pairs}) == 4
    assert result.dx == pytest.approx(0.18, abs=0.02)
