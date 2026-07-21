from robot_planning.nodes.mission_fsm_node import MissionFsmNode


def _node():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.align_distractor_confirm_frames = 5
    node._body_dets_seq = 0
    node._align_distractor_label = ""
    node._align_distractor_count = 0
    node._align_distractor_last_seq = 0
    return node


def _new_frame(node, label):
    node._body_dets_seq += 1
    return node._update_align_distractor_confirmation(label)


def test_same_distractor_requires_five_fresh_body_frames():
    node = _node()

    for expected in range(1, 5):
        count, fresh = _new_frame(node, "cube")
        assert fresh
        assert count == expected
        assert count < node.align_distractor_confirm_frames

    assert _new_frame(node, "cube") == (5, True)


def test_repeated_control_tick_does_not_count_the_same_body_frame_twice():
    node = _node()

    assert _new_frame(node, "cube") == (1, True)
    assert node._update_align_distractor_confirmation("cube") == (1, False)


def test_target_reappearance_or_label_change_resets_confirmation():
    node = _node()

    assert _new_frame(node, "cube") == (1, True)
    assert _new_frame(node, "cube") == (2, True)
    assert _new_frame(node, None) == (0, True)
    assert _new_frame(node, "cube") == (1, True)
    assert _new_frame(node, "icosahedron") == (1, True)
