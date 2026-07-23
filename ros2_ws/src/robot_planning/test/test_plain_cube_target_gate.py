from types import SimpleNamespace

from robot_planning.nodes.mission_fsm_node import MissionFsmNode, plain_cube_approach_invalid
from robot_planning.nodes.target_selector_node import TargetSelectorNode


def _target(label, set_type):
    return SimpleNamespace(class_label=label, set_type=set_type)


def test_plain_cube_approach_continues_only_for_confirmed_cube():
    assert not plain_cube_approach_invalid("cube", 1, False, _target("cube", 1))
    assert not plain_cube_approach_invalid("cube", 1, False, _target("", 0))
    assert plain_cube_approach_invalid(
        "cube", 1, False, _target("fruit_photo_cube", 2)
    )


def test_plain_cube_gate_does_not_cancel_other_mission_modes():
    assert not plain_cube_approach_invalid(
        "dodecahedron", 1, False, _target("fruit_photo_cube", 2)
    )
    assert not plain_cube_approach_invalid(
        "cube", 2, False, _target("fruit_photo_cube", 2)
    )
    assert not plain_cube_approach_invalid(
        "cube", 1, True, _target("fruit_photo_cube", 2)
    )


def test_cube_mission_does_not_explore_unknown_track():
    node = SimpleNamespace(
        set1_label="cube",
        set2_label="banana",
        phase=1,
        set1_base=10.0,
        set2_base=20.0,
        explore_base=5.0,
        _in_active_zone=lambda _obj: True,
    )
    unknown = SimpleNamespace(
        blacklisted=False,
        set_type=0,
        class_label="",
        x=0.5,
        y=0.0,
        confidence=0.9,
    )

    assert TargetSelectorNode._score(node, unknown, 0.0, 0.0) is None


def test_non_cube_mission_keeps_unknown_exploration_behavior():
    node = SimpleNamespace(
        set1_label="dodecahedron",
        set2_label="banana",
        phase=1,
        set1_base=10.0,
        set2_base=20.0,
        explore_base=5.0,
        _in_active_zone=lambda _obj: True,
    )
    unknown = SimpleNamespace(
        blacklisted=False,
        set_type=0,
        class_label="",
        x=0.5,
        y=0.0,
        confidence=0.9,
    )

    assert TargetSelectorNode._score(node, unknown, 0.0, 0.0) is not None


def _stamp(sec):
    return SimpleNamespace(sec=sec, nanosec=0)


def _approach_node():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.set1_label = "cube"
    node.phase = 1
    node._opportunistic_set2_active = False
    node.plain_cube_ambiguous_observations = 5
    node._plain_cube_unknown_count = 0
    node._plain_cube_unknown_last_stamp = None
    return node


def test_plain_cube_approach_holds_then_excludes_five_fresh_unknown_observations():
    node = _approach_node()
    target = SimpleNamespace(set_type=0, class_label="", last_seen=_stamp(1))

    for seq in range(1, 5):
        target.last_seen = _stamp(seq)
        assert node._plain_cube_approach_identity_action(target) == "hold"
    target.last_seen = _stamp(5)
    assert node._plain_cube_approach_identity_action(target) == "exclude"


def test_repeated_world_publish_does_not_count_as_new_unknown_observation():
    node = _approach_node()
    target = SimpleNamespace(set_type=0, class_label="", last_seen=_stamp(1))

    assert node._plain_cube_approach_identity_action(target) == "hold"
    assert node._plain_cube_approach_identity_action(target) == "hold"
    assert node._plain_cube_unknown_count == 1


def test_confirmed_cube_clears_ambiguous_observation_count():
    node = _approach_node()
    unknown = SimpleNamespace(set_type=0, class_label="", last_seen=_stamp(1))
    cube = SimpleNamespace(set_type=1, class_label="cube", last_seen=_stamp(2))

    assert node._plain_cube_approach_identity_action(unknown) == "hold"
    assert node._plain_cube_approach_identity_action(cube) == "continue"
    assert node._plain_cube_unknown_count == 0


def test_definitive_fruit_cube_overrides_earlier_plain_cube_commit():
    node = _approach_node()
    node.global_target_mode = True
    node._global_approach_committed_from_wide = True
    node._appr_tgt_xy = (-1.5, 0.5)
    changed_to_fruit = SimpleNamespace(
        set_type=2,
        class_label="fruit_photo_cube",
        last_seen=_stamp(3),
    )

    assert node._plain_cube_approach_identity_action(changed_to_fruit) == "exclude"


def test_definitive_fruit_cube_at_frozen_position_cancels_plain_cube_commit():
    fruit = SimpleNamespace(
        id=91,
        x=0.50,
        y=-0.50,
        set_type=2,
        class_label="fruit_photo_cube",
    )
    node = SimpleNamespace(
        global_target_mode=True,
        set1_label="cube",
        phase=1,
        _opportunistic_set2_active=False,
        _global_approach_committed_from_wide=True,
        _appr_tgt_xy=(0.50, -0.50),
        global_body_target_match_radius_m=0.18,
        current_target=fruit,
        world=SimpleNamespace(objects=[fruit]),
        _lookup_object=lambda _object_id: fruit,
    )

    assert not MissionFsmNode._committed_global_plain_cube(node)


def test_zone_exclusion_matches_track_id_or_nearby_recreated_track():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.zone_mission_enabled = True
    node.zone_ambiguous_exclusion_radius_m = 0.18
    node._zone_ambiguous_excluded_ids = {15}
    node._zone_ambiguous_excluded_xy = [(-0.5, 1.0)]

    same_id = SimpleNamespace(id=15, x=0.0, y=0.0)
    new_near_id = SimpleNamespace(id=91, x=-0.38, y=1.0)
    new_far_id = SimpleNamespace(id=92, x=-0.20, y=1.0)

    assert node._is_zone_ambiguous_excluded(same_id)
    assert node._is_zone_ambiguous_excluded(new_near_id)
    assert not node._is_zone_ambiguous_excluded(new_far_id)
