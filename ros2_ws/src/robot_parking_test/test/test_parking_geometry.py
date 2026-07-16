import math

from robot_parking_test.parking_geometry import (
    ArrivalObservation,
    PairScoreConfig,
    field_to_base,
    score_floor_pairs,
)


def test_floor_pair_prefers_zone4_hint_and_expected_spacing():
    obs = [
        ArrivalObservation(0, 100.0, 100.0, 0.90, 1.62, 1.80),
        ArrivalObservation(1, 200.0, 100.0, 0.85, 1.98, 1.80),
        ArrivalObservation(2, 300.0, 100.0, 0.95, 0.00, 0.00),
        ArrivalObservation(3, 400.0, 100.0, 0.95, 0.50, 0.00),
    ]
    pairs = score_floor_pairs(obs, PairScoreConfig())
    assert pairs
    assert pairs[0].key == (0, 1)
    assert math.isclose(pairs[0].center_x, 1.80, abs_tol=1e-6)
    assert math.isclose(pairs[0].spacing_m, 0.36, abs_tol=1e-6)


def test_field_to_base_says_corner_goal_is_behind_at_minus_135_deg():
    bx, by = field_to_base(1.835, 1.835, 1.70, 1.70, math.radians(-135.0))
    assert bx < 0.0
    assert abs(by) < 0.01
