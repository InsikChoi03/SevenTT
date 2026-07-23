from types import SimpleNamespace

from robot_planning.nodes.mission_fsm_node import MissionFsmNode


def _node():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.set1_label = "dodecahedron"
    node.classify_octa_wide_final_enabled = True
    node.classify_wide_confirm_frames = 3
    node.align_distractor_confirm_frames = 5
    node.classify_body_max_frames = 10
    node._body_dets_seq = 0
    node._wide_relative_seq = 0
    node._observed_label = ""
    node._fresh_body_set1_pick_candidate = lambda: (
        ("dodecahedron", 0.96, 0.0, 0.0)
        if node._observed_label == "dodecahedron"
        else None
    )
    node._body_nearest_any = lambda: (
        (node._observed_label, 0.19, 0.0) if node._observed_label else None
    )
    node._reset_set1_final_classification()
    return node


def _new_frame(node, label):
    node._observed_label = label
    node._body_dets_seq += 1
    return node._update_set1_final_classification()


def test_target_requires_five_fresh_stopped_body_frames():
    node = _node()

    for _ in range(4):
        decision, observed, fresh = _new_frame(node, "dodecahedron")
        assert (decision, observed, fresh) == ("pending", "dodecahedron", True)

    assert _new_frame(node, "dodecahedron") == ("target", "dodecahedron", True)


def test_same_distractor_requires_five_votes_before_rejection():
    node = _node()

    for _ in range(4):
        decision, observed, fresh = _new_frame(node, "cube")
        assert (decision, observed, fresh) == ("pending", "cube", True)

    assert _new_frame(node, "cube") == ("distractor", "cube", True)


def test_repeated_control_tick_does_not_count_the_same_body_frame_twice():
    node = _node()

    assert _new_frame(node, "cube") == ("pending", "cube", True)
    assert node._update_set1_final_classification() == ("pending", "", False)
    assert node._classify_body_frame_count == 1


def test_mixed_labels_vote_over_the_window_and_can_end_inconclusive():
    node = _node()

    labels = ["dodecahedron", "cube"] * 4 + ["icosahedron", ""]
    decisions = [_new_frame(node, label)[0] for label in labels]

    assert decisions[-1] == "inconclusive"
    assert node._classify_body_target_count == 4
    assert node._classify_body_other_counts == {"cube": 4, "icosahedron": 1}


def test_wide_positive_plain_cube_uses_body_presence_not_mutable_body_label():
    node = _node()
    node.set1_label = "cube"
    node.phase = 1
    node.global_target_mode = True
    node._global_approach_committed_from_wide = True
    node._appr_tgt_xy = (-1.5, 0.5)
    node._opportunistic_set2_active = False
    node._fresh_body_set1_pick_presence = lambda: (
        "fruit_photo_cube", -0.009, -0.013
    )

    for _ in range(4):
        decision, observed, fresh = _new_frame(node, "fruit_photo_cube")
        assert (decision, observed, fresh) == ("pending", "cube", True)

    assert _new_frame(node, "fruit_photo_cube") == ("target", "cube", True)


def _octa_wide_node():
    node = _node()
    node.set1_label = "octahedron"
    node._wide_label = ""
    node._fresh_body_set1_pick_presence = lambda: ("cube", -0.009, -0.013)
    node._wide_align_target_observation = lambda: (
        SimpleNamespace(label=node._wide_label, confidence=0.97)
        if node._wide_label
        else None
    )
    node.pick_track_conf = 0.5
    node.conf_threshold = 0.7
    node._reset_set1_final_classification()
    return node


def _new_wide_frame(node, label):
    node._wide_label = label
    node._wide_relative_seq += 1
    return node._update_set1_final_classification()


def test_octa_uses_three_fresh_wide_votes_and_ignores_body_cube_label():
    node = _octa_wide_node()

    assert _new_wide_frame(node, "octahedron") == ("pending", "octahedron", True)
    assert _new_wide_frame(node, "octahedron") == ("pending", "octahedron", True)
    assert _new_wide_frame(node, "octahedron") == ("target", "octahedron", True)


def test_octa_wide_distractor_requires_three_wide_votes():
    node = _octa_wide_node()

    assert _new_wide_frame(node, "cube") == ("pending", "cube", True)
    assert _new_wide_frame(node, "cube") == ("pending", "cube", True)
    assert _new_wide_frame(node, "cube") == ("distractor", "cube", True)


def test_octa_wide_identity_does_not_pass_without_body_grab_presence():
    node = _octa_wide_node()
    node.classify_body_max_frames = 3
    node._fresh_body_set1_pick_presence = lambda: None

    decisions = [_new_wide_frame(node, "octahedron")[0] for _ in range(3)]

    assert decisions == ["pending", "pending", "inconclusive"]
    assert node._classify_body_target_count == 0
