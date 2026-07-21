from types import SimpleNamespace

from robot_planning.nodes.mission_fsm_node import MissionFsmNode


def _node():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node._opportunistic_set2_active = False
    node.phase = 1
    node.set1_label = "icosahedron"
    node.current_target = SimpleNamespace(set_type=1, class_label="icosahedron")
    node.state_enter_s = 10.0
    node._body_dets_stamp_s = 10.1
    node.classify_body_max_age_sec = 0.6
    node.grab_x = 0.19
    node.grab_y = 0.0
    node.align_fwd_tol = 0.02
    node.align_tol = 0.02
    node.pick_track_conf = 0.5
    node.conf_threshold = 0.7
    node._now_s = lambda: 10.2
    node._body_target_base = lambda _label: ("icosahedron", 0.195, -0.01, 0.96)
    return node


def test_fresh_aligned_body_target_survives_world_track_loss():
    node = _node()

    candidate = node._fresh_body_set1_pick_candidate()

    assert candidate is not None
    assert candidate[0] == "icosahedron"
    assert candidate[1] == 0.96


def test_pre_classify_or_stale_body_frame_is_rejected():
    node = _node()
    node._body_dets_stamp_s = 9.9
    assert node._fresh_body_set1_pick_candidate() is None

    node._body_dets_stamp_s = 10.1
    node._now_s = lambda: 10.8
    assert node._fresh_body_set1_pick_candidate() is None


def test_misaligned_or_low_confidence_body_target_is_rejected():
    node = _node()
    node._body_target_base = lambda _label: ("icosahedron", 0.23, 0.0, 0.96)
    assert node._fresh_body_set1_pick_candidate() is None

    node._body_target_base = lambda _label: ("icosahedron", 0.19, 0.0, 0.69)
    assert node._fresh_body_set1_pick_candidate() is None
