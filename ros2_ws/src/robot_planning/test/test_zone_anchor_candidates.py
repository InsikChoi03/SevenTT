from robot_planning.nodes.mission_fsm_node import (
    _nearest_zone_anchor,
    _parse_zone_anchor_candidates,
)


def test_parse_zone_anchor_candidates_preserves_multiple_points_per_zone():
    candidates = _parse_zone_anchor_candidates(
        [1.0, 1.25, -0.75, 2.0, 0.75, 0.75, 2.0, 1.25, 0.75],
        [],
    )

    assert candidates == {
        1: [(1.25, -0.75)],
        2: [(0.75, 0.75), (1.25, 0.75)],
    }


def test_nearest_zone_anchor_uses_pose_at_zone_entry():
    index, point = _nearest_zone_anchor(
        [(0.75, 0.75), (1.25, 0.75)],
        (1.10, 0.10),
    )

    assert index == 1
    assert point == (1.25, 0.75)
