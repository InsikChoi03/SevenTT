from robot_perception.nodes.world_model_node import (
    WorldModelNode,
    resolve_recent_yolo_identity,
)


def _resolve(wide_history, body_history=None, required=5):
    body_history = list(body_history or [])
    wide_history = list(wide_history)
    body_votes = {label: float(body_history.count(label)) for label in set(body_history)}
    wide_votes = {label: float(wide_history.count(label)) for label in set(wide_history)}
    return resolve_recent_yolo_identity(
        body_votes,
        wide_votes,
        wide_history + body_history,
        required,
    )


def test_plain_cube_stays_unknown_until_five_clean_observations():
    assert _resolve(["cube"] * 4) == ("", 0)
    assert _resolve(["cube"] * 5) == ("cube", 1)


def test_mixed_plain_and_fruit_photo_labels_stay_unknown():
    assert _resolve(["cube", "cube", "fruit_photo_cube", "cube", "cube"]) == ("", 0)


def test_fruit_photo_cube_requires_five_clean_observations():
    assert _resolve(["fruit_photo_cube"] * 4) == ("", 0)
    assert _resolve(["fruit_photo_cube"] * 5) == ("fruit_photo_cube", 2)


def test_plain_cube_recovers_after_five_new_clean_observations():
    history = ["fruit_photo_cube"] + ["cube"] * 5
    assert _resolve(history) == ("cube", 1)


def test_conflicting_camera_labels_leave_cube_identity_unknown():
    assert _resolve(["cube"] * 5, ["fruit_photo_cube"]) == ("", 0)


def test_non_cube_shape_identity_keeps_existing_vote_behavior():
    assert _resolve(["dodecahedron"]) == ("dodecahedron", 1)


def test_fruit_uses_its_own_threshold_for_recent_veto_evidence():
    node = WorldModelNode.__new__(WorldModelNode)
    node.fruit_cube_sticky_conf_wide = 0.75
    node.fruit_cube_sticky_conf_body = 0.50

    assert node._is_recent_label_evidence("fruit_photo_cube", 0.80, False, 0.85)
    assert not node._is_recent_label_evidence("cube", 0.80, False, 0.85)
