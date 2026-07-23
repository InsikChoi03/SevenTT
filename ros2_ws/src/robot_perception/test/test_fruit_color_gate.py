"""Unit tests for the conservative HSV colour-consistency veto (fruit_color_gate)."""

import numpy as np

from robot_perception.fruit_color_gate import (
    ColorGateConfig,
    FRUIT_HUE_RANGES,
    evaluate_color_consistency,
    fruit_hue_fractions,
)

# Solid BGR paints matching each fruit's printed colour.
FRUIT_BGR = {
    "apple": (30, 20, 200),        # deep red
    "orange": (0, 128, 255),       # orange
    "banana": (0, 220, 230),       # yellow
    "pineapple": (19, 69, 139),    # saddle brown
}
WHITE = (255, 255, 255)


def _crop(color_bgr, size=32):
    return np.full((size, size, 3), color_bgr, dtype=np.uint8)


def _mixed_crop(color_a, color_b, size=32):
    crop = _crop(color_a, size)
    crop[:, size // 2:] = color_b
    return crop


def test_every_fruit_colour_passes_its_own_label():
    for fruit, bgr in FRUIT_BGR.items():
        result = evaluate_color_consistency(_crop(bgr), fruit)
        assert not result.veto, f"{fruit} wrongly vetoed: {result}"
        assert result.expected_fraction > 0.5


def test_conflicting_colour_with_clear_dominance_is_vetoed():
    result = evaluate_color_consistency(_crop(FRUIT_BGR["apple"]), "banana")
    assert result.veto
    assert result.dominant_label == "apple"
    assert result.reason == "conflicting_hue_dominant"


def test_orange_crop_under_apple_label_is_vetoed():
    result = evaluate_color_consistency(_crop(FRUIT_BGR["orange"]), "apple")
    assert result.veto
    assert result.dominant_label == "orange"


def test_white_cube_crop_is_never_vetoed():
    for fruit in FRUIT_HUE_RANGES:
        result = evaluate_color_consistency(_crop(WHITE), fruit)
        assert not result.veto
        assert result.reason == "no_color_evidence"


def test_mostly_white_cube_face_with_small_print_is_not_vetoed():
    # The white cube shell dominates the crop; a tiny print area must not trigger a veto
    # against ANY label because the coloured evidence is under min_colored_fraction.
    crop = _crop(WHITE)
    crop[:2, :2] = FRUIT_BGR["apple"]      # 4/1024 pixels < 5% colour evidence
    for fruit in FRUIT_HUE_RANGES:
        assert not evaluate_color_consistency(crop, fruit).veto


def test_banana_pineapple_overlap_never_vetoes_either_direction():
    assert not evaluate_color_consistency(_crop(FRUIT_BGR["banana"]), "pineapple").veto
    assert not evaluate_color_consistency(_crop(FRUIT_BGR["pineapple"]), "banana").veto


def test_mixed_colours_with_expected_hue_present_pass():
    crop = _mixed_crop(FRUIT_BGR["apple"], FRUIT_BGR["banana"])
    assert not evaluate_color_consistency(crop, "banana").veto
    assert not evaluate_color_consistency(crop, "apple").veto


def test_unknown_label_is_never_vetoed():
    result = evaluate_color_consistency(_crop(FRUIT_BGR["apple"]), "fruit_photo_cube")
    assert not result.veto
    assert result.reason == "unknown_label"


def test_empty_and_degenerate_crops_are_never_vetoed():
    empty = np.zeros((0, 0, 3), dtype=np.uint8)
    assert not evaluate_color_consistency(empty, "apple").veto
    gray = np.zeros((8, 8), dtype=np.uint8)     # wrong shape entirely
    assert not evaluate_color_consistency(gray, "apple").veto


def test_hue_fraction_measurement_matches_paint():
    fractions, colored = fruit_hue_fractions(_crop(FRUIT_BGR["orange"]))
    assert colored > 0.99
    assert fractions["orange"] > 0.99
    assert fractions["banana"] == 0.0


def test_loose_threshold_config_suppresses_borderline_veto():
    # With min_expected_fraction raised, even a low expected fraction passes.
    config = ColorGateConfig(min_expected_fraction=0.0)
    result = evaluate_color_consistency(_crop(FRUIT_BGR["apple"]), "banana", config)
    assert not result.veto


def test_veto_requires_dominance_over_expected_fraction():
    # 30% apple / 70% white with label banana: apple fraction of coloured pixels is high,
    # banana absent -> veto fires; but requiring 10x dominance of a large minimum
    # dominant fraction suppresses it when the dominant share is capped.
    config = ColorGateConfig(dominant_min_fraction=1.01)   # unreachable dominance
    result = evaluate_color_consistency(_crop(FRUIT_BGR["apple"]), "banana", config)
    assert not result.veto
    assert result.reason == "no_dominant_conflict"
