import math
from collections import deque
from types import SimpleNamespace

from robot_perception.nodes.world_model_node import (
    ObjectFlowSample,
    WorldModelNode,
    object_flow_min_pairs_for_state,
    select_translation_velocity_medoid,
)


def _sample(dfwd, dleft, dt=0.1, confidence=0.9):
    return ObjectFlowSample(0.0, dfwd, dleft, dt, confidence)


def test_general_mission_states_keep_existing_object_flow_pair_gate():
    for state in ("", "SCAN", "APPROACH", "ALIGN", "CLASSIFY", "PICK"):
        assert object_flow_min_pairs_for_state(state, 4, 3) == 4


def test_local_anchor_states_use_relaxed_object_flow_pair_gate():
    for state in ("LOCAL_ANCHOR_INSPECTION", "LOCAL_ANCHOR_ALIGN"):
        assert object_flow_min_pairs_for_state(state, 4, 3) == 3


def test_object_flow_pair_gate_normalizes_state_and_clamps_invalid_values():
    assert object_flow_min_pairs_for_state(" local_anchor_align ", 4, 3) == 3
    assert object_flow_min_pairs_for_state("SCAN", 0, 3) == 1


def test_mission_state_callback_normalizes_state_for_flow_profile():
    node = SimpleNamespace(_mission_state="")

    WorldModelNode.on_mission_state(
        node,
        SimpleNamespace(state=" local_anchor_inspection "),
    )

    assert node._mission_state == "LOCAL_ANCHOR_INSPECTION"


class _PublisherSpy:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _SilentLogger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass


def _flow_harness(mission_state):
    return SimpleNamespace(
        _mission_state=mission_state,
        object_flow_min_pairs=4,
        local_anchor_object_flow_min_pairs=3,
        _prev_wide_base=[
            (0.0, 0.0, "cube"),
            (0.5, 0.0, "octahedron"),
            (0.0, 0.5, "dodecahedron"),
        ],
        _prev_wide_base_sec=0.0,
        _object_flow_samples=deque(maxlen=3),
        _object_flow_drive_mode="STOP",
        object_flow=True,
        object_flow_max_dt=0.4,
        object_flow_assoc=0.3,
        object_flow_max_dtheta=0.5,
        object_flow_median_filter_enabled=False,
        object_flow_max_speed=0.0,
        object_flow_position_margin=0.025,
        object_flow_trans_deadband=0.0,
        pub_odom=_PublisherSpy(),
        _now_sec=lambda: 0.1,
        _umeyama_2d=WorldModelNode._umeyama_2d,
        get_logger=lambda: _SilentLogger(),
    )


def test_publish_flow_accepts_three_pairs_only_in_local_anchor_state():
    current = [
        (0.01, 0.0, "cube"),
        (0.51, 0.0, "octahedron"),
        (0.01, 0.5, "dodecahedron"),
    ]
    normal = _flow_harness("SCAN")
    local_anchor = _flow_harness("LOCAL_ANCHOR_INSPECTION")

    WorldModelNode._publish_object_flow(normal, current)
    WorldModelNode._publish_object_flow(local_anchor, current)

    assert normal.pub_odom.messages == []
    assert len(local_anchor.pub_odom.messages) == 1
    assert math.isclose(
        local_anchor.pub_odom.messages[0].data[3], 0.20, abs_tol=1e-6
    )


def test_translation_velocity_medoid_rejects_single_forward_spike():
    samples = [
        _sample(0.012, 0.001),
        _sample(0.240, 0.000),
        _sample(0.013, -0.001),
    ]

    selected = select_translation_velocity_medoid(samples)

    assert selected is samples[0] or selected is samples[2]
    assert math.isclose(selected.vfwd, 0.12, abs_tol=0.01) or math.isclose(
        selected.vfwd, 0.13, abs_tol=0.01
    )


def test_translation_velocity_medoid_compares_velocity_across_variable_dt():
    samples = [
        _sample(0.012, 0.0, dt=0.1),
        _sample(0.024, 0.0, dt=0.2),
        _sample(0.090, 0.0, dt=0.1),
    ]

    selected = select_translation_velocity_medoid(samples)

    assert math.isclose(selected.vfwd, 0.12, abs_tol=1e-9)


def test_translation_velocity_medoid_selects_observed_vector_not_mixed_axes():
    samples = [
        _sample(0.010, 0.000),
        _sample(0.011, 0.001),
        _sample(0.000, 0.080),
    ]

    selected = select_translation_velocity_medoid(samples)

    assert selected in samples[:2]
    assert (selected.dfwd, selected.dleft) in {
        (samples[0].dfwd, samples[0].dleft),
        (samples[1].dfwd, samples[1].dleft),
    }
