import math

from robot_perception.nodes.world_model_node import (
    ObjectFlowSample,
    select_translation_velocity_medoid,
)


def _sample(dfwd, dleft, dt=0.1, confidence=0.9):
    return ObjectFlowSample(0.0, dfwd, dleft, dt, confidence)


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
