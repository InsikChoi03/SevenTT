"""
Wide-cam fruit HINTS must aid routing only — never the pick decision.

The mission FSM guards Set2 picks with (a) the /classification/siglip body-cam CLASSIFY
gate and (b) track-quality thresholds on the world-model Object fields (confidence /
n_obs, e.g. opportunistic_set2_min_conf=0.75 / min_nobs=3). These tests prove a wide
SigLIP hint can reach fruit_label (routing) while leaving every one of those pick-gate
inputs untouched, and that body-cam evidence always outranks and overwrites a hint.
"""

from types import SimpleNamespace

from std_msgs.msg import String

from robot_interfaces.msg import Classification

from robot_perception.nodes.world_model_node import (
    Track,
    WorldModelNode,
    resolve_fruit_label_with_hint,
)
from robot_perception.wide_fruit_hint import build_wide_hint


class _SilentLogger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass


def _set2_track(tid=7, x=1.0, y=1.0):
    return Track(
        id=tid, x=x, y=y, confidence=0.4, last_seen_sec=99.0,
        class_label="fruit_photo_cube", set_type=2, n_obs=2,
        wide_votes={"fruit_photo_cube": 3.0},
        label_history=["fruit_photo_cube"] * 5,
    )


def _node(track, enabled=True, projected=(1.05, 1.0)):
    node = WorldModelNode.__new__(WorldModelNode)
    node.wide_fruit_hint_enabled = enabled
    node.mapping_enabled = True
    node.wide_fruit_hint_assoc_radius = 0.35
    node.wide_fruit_hint_min_margin = 0.10
    node.wide_fruit_hint_max_age = 1.5
    node.plain_cube_confirm_observations = 5
    node.tracks = {track.id: track} if track is not None else {}
    node.get_logger = lambda: _SilentLogger()
    node._now_sec = lambda: 100.0
    node._project_wide_hint_pixel = lambda u, v, stamp_sec: projected
    node._last_body_fruit_id = None
    node._last_body_fruit_sec = 0.0
    node._fruit_attach_window = 1.0
    return node


def _hint_msg(label="apple", margin=0.9, stamp_sec=99.8):
    msg = String()
    msg.data = build_wide_hint(label, margin, 640.0, 480.0, stamp_sec)
    return msg


def _body_siglip(node, track, label="orange", conf=0.8):
    node._last_body_fruit_id = track.id
    node._last_body_fruit_sec = 99.9
    msg = Classification()
    msg.label = label
    msg.confidence = conf
    msg.image_face_visible = True
    node.on_siglip(msg)


# ------------------------------------------------------------------ pure priority rule

def test_body_votes_always_outrank_wide_hints():
    label, source = resolve_fruit_label_with_hint({"apple": 0.5}, {"banana": 9.9}, 2)
    assert (label, source) == ("apple", "body")


def test_wide_hint_fills_only_unlabeled_set2_tracks():
    assert resolve_fruit_label_with_hint({}, {"banana": 1.0}, 2) == ("banana", "wide")
    assert resolve_fruit_label_with_hint({}, {"banana": 1.0}, 1) == ("", "")
    assert resolve_fruit_label_with_hint({}, {}, 2) == ("", "")


# ------------------------------------------------------------------- hint attach path

def test_hint_attaches_as_routing_label_without_touching_pick_gate_fields():
    track = _set2_track()
    node = _node(track)
    node.on_wide_fruit_hint(_hint_msg("apple", 0.9))

    assert track.fruit_label == "apple"            # routing hint visible to the FSM
    assert track.fruit_label_source == "wide"
    assert track.class_label == "fruit_photo_cube"  # identity NOT faked to body grade
    # Pick-gate evidence untouched (FSM thresholds e.g. conf>=0.75 / n_obs>=3):
    assert track.confidence == 0.4
    assert track.n_obs == 2
    assert track.n_body == 0
    assert track.fruit_votes == {}
    assert track.fruit_confidence == 0.0
    assert not track.seen_body
    assert node._last_body_fruit_id is None


def test_many_strong_hints_never_satisfy_pick_gate_thresholds():
    track = _set2_track()
    node = _node(track)
    for _ in range(20):
        node.on_wide_fruit_hint(_hint_msg("apple", 0.99))

    # However many high-margin hints arrive, the FSM's pick conditions stay unmet:
    assert track.confidence == 0.4 < 0.75
    assert track.n_obs == 2 < 3
    assert track.fruit_confidence == 0.0
    assert track.fruit_votes == {}
    assert track.fruit_label_source == "wide"      # still only a hint


def test_body_label_is_never_overwritten_by_a_wide_hint():
    track = _set2_track()
    node = _node(track)
    _body_siglip(node, track, "orange", 0.8)
    assert (track.fruit_label, track.fruit_label_source) == ("orange", "body")

    node.on_wide_fruit_hint(_hint_msg("banana", 0.99))

    assert (track.fruit_label, track.fruit_label_source) == ("orange", "body")
    assert track.wide_fruit_votes == {}            # hint not even recorded


def test_body_read_replaces_an_earlier_wide_hint():
    track = _set2_track()
    node = _node(track)
    node.on_wide_fruit_hint(_hint_msg("banana", 0.9))
    assert (track.fruit_label, track.fruit_label_source) == ("banana", "wide")

    _body_siglip(node, track, "orange", 0.8)

    assert (track.fruit_label, track.fruit_label_source) == ("orange", "body")
    assert track.fruit_confidence == 0.8


def test_hint_respects_margin_age_distance_and_enable_gates():
    for kwargs, msg in (
        (dict(enabled=False), _hint_msg("apple", 0.9)),            # consumer disabled
        (dict(), _hint_msg("apple", 0.05)),                        # below min margin
        (dict(), _hint_msg("apple", 0.9, stamp_sec=90.0)),         # stale capture
        (dict(projected=None), _hint_msg("apple", 0.9)),           # unprojectable pixel
        (dict(projected=(3.0, 3.0)), _hint_msg("apple", 0.9)),     # no track in radius
        (dict(), _hint_msg("dragonfruit", 0.9)),                   # not a known fruit
    ):
        track = _set2_track()
        node = _node(track, **kwargs)
        node.on_wide_fruit_hint(msg)
        assert track.fruit_label == "" and track.wide_fruit_votes == {}


def test_hint_never_binds_to_non_set2_or_blacklisted_tracks():
    shape = Track(
        id=3, x=1.0, y=1.0, confidence=0.9, last_seen_sec=99.0,
        class_label="icosahedron", set_type=1,
        body_votes={"icosahedron": 3.0},
    )
    node = _node(shape)
    node.on_wide_fruit_hint(_hint_msg("apple", 0.9))
    assert shape.fruit_label == "" and shape.wide_fruit_votes == {}

    banned = _set2_track()
    banned.blacklisted = True
    node = _node(banned)
    node.on_wide_fruit_hint(_hint_msg("apple", 0.9))
    assert banned.fruit_label == "" and banned.wide_fruit_votes == {}


def test_demoted_track_drops_its_stale_wide_hint():
    track = _set2_track()
    node = _node(track)
    node.on_wide_fruit_hint(_hint_msg("apple", 0.9))
    assert track.fruit_label == "apple"

    # Later clean plain-cube evidence demotes the track to Set1 -> hint must vanish.
    track.wide_votes = {"cube": 5.0}
    track.label_history = ["cube"] * 5
    node._refresh_identity(track)

    assert track.set_type == 1
    assert track.fruit_label == ""
    assert track.fruit_label_source == ""


def test_mission_state_stub_keeps_on_siglip_face_gate():
    # A faceless body read must not attach body-grade evidence either (regression guard
    # for the shared attach path used by the hint-priority tests above).
    track = _set2_track()
    node = _node(track)
    node._last_body_fruit_id = track.id
    node._last_body_fruit_sec = 99.9
    msg = Classification()
    msg.label = "orange"
    msg.confidence = 0.9
    msg.image_face_visible = False
    node.on_siglip(msg)
    assert track.fruit_votes == {} and track.fruit_label == ""


def test_on_siglip_stub_namespace_isolated():
    # Sanity: harness construction does not leak state between tests.
    a = _node(_set2_track())
    b = _node(_set2_track())
    assert a.tracks is not b.tracks


def test_wide_hint_uses_capture_time_projection_arguments():
    captured = {}
    track = _set2_track()
    node = _node(track)

    def project(u, v, stamp_sec):
        captured["args"] = (u, v, stamp_sec)
        return (1.05, 1.0)

    node._project_wide_hint_pixel = project
    node.on_wide_fruit_hint(_hint_msg("apple", 0.9, stamp_sec=99.8))
    assert captured["args"] == (640.0, 480.0, 99.8)


def _flow_stub():
    return SimpleNamespace(state=" scan ")


def test_namespace_helper_available_for_future_extension():
    # Keeps the SimpleNamespace import exercised alongside the harness style used by the
    # neighbouring world-model tests.
    assert _flow_stub().state.strip().upper() == "SCAN"
