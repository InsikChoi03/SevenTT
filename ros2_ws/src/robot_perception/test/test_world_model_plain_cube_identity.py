from robot_perception.nodes.world_model_node import (
    Track,
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


def test_nonsticky_wide_fruit_decision_is_dominant_but_reversible():
    node = WorldModelNode.__new__(WorldModelNode)
    node.fruit_cube_sticky_enabled = False
    node.plain_cube_confirm_observations = 3
    track = Track(
        id=1,
        x=0.0,
        y=0.0,
        confidence=0.9,
        last_seen_sec=0.0,
        latest_wide_cube_label="fruit_photo_cube",
        label_history=["fruit_photo_cube"],
    )

    WorldModelNode._refresh_identity(node, track)
    assert (track.class_label, track.set_type) == ("fruit_photo_cube", 2)

    track.latest_wide_cube_label = "cube"
    track.label_history = ["cube", "cube", "cube"]
    WorldModelNode._refresh_identity(node, track)
    assert (track.class_label, track.set_type) == ("cube", 1)
