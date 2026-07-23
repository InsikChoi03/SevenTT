"""
HSV colour-consistency gate for SigLIP fruit classifications (conservative veto only).

Set2 fruit cubes are WHITE cubes with a printed fruit picture, so the non-white pixels of
a correct crop should contain the expected fruit hue (apple=red, orange=orange,
banana=yellow, pineapple=yellow/brown). This module measures that consistency and returns
a VETO decision that is deliberately conservative: a classification is rejected only when
the expected hue is nearly absent AND a different fruit's hue clearly dominates. Lighting
shifts must not veto a correct read (a wrong veto costs more than a wrong classification
that the body-cam CLASSIFY gate would catch anyway), so every default is loose and every
ambiguous case resolves to "no veto".

Pure numpy only (no cv2/torch): safe to import in dry-run and unit tests.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# OpenCV-convention hue spans (H in [0, 180)) per fruit; a fruit may own several spans
# (red wraps around 180). banana/pineapple overlap on purpose: prints of both are yellow
# dominated, and an overlap can only ever SUPPRESS a veto (never cause one).
FRUIT_HUE_RANGES: dict[str, tuple[tuple[float, float], ...]] = {
    "apple": ((0.0, 9.0), (165.0, 180.0)),
    "orange": ((9.0, 21.0),),
    "banana": ((21.0, 38.0),),
    "pineapple": ((11.0, 38.0),),
}


@dataclass(frozen=True)
class ColorGateConfig:
    """
    Thresholds for the colour-consistency veto (defaults intentionally loose).

    min_expected_fraction: expected-fruit hue fraction below which a veto becomes possible.
    dominant_min_fraction: another fruit's hue must reach this fraction to count as dominant.
    dominance_ratio: and exceed the expected fraction by this factor.
    min_colored_fraction: below this fraction of non-white pixels the crop carries no colour
        evidence at all (white cube face / blank side) and is never vetoed.
    saturation_min / value_min: HSV S/V floor (0-255 scale) for a pixel to count as
        non-white colour evidence (drops the white cube shell and dark shadow).
    """

    min_expected_fraction: float = 0.10
    dominant_min_fraction: float = 0.40
    dominance_ratio: float = 3.0
    min_colored_fraction: float = 0.05
    saturation_min: float = 60.0
    value_min: float = 60.0


@dataclass(frozen=True)
class ColorGateResult:
    """Outcome of the colour gate for one crop/label pair (veto=False means keep)."""

    veto: bool
    label: str
    expected_fraction: float
    dominant_label: str
    dominant_fraction: float
    colored_fraction: float
    reason: str


def _hue_sat_val(crop_bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Convert a BGR uint8 image to OpenCV-convention H [0,180), S [0,255], V [0,255]."""
    arr = np.asarray(crop_bgr, dtype=np.float64)
    b, g, r = arr[..., 0], arr[..., 1], arr[..., 2]
    v = np.maximum(np.maximum(r, g), b)
    mn = np.minimum(np.minimum(r, g), b)
    c = v - mn
    s = np.where(v > 0.0, 255.0 * c / np.where(v > 0.0, v, 1.0), 0.0)
    safe_c = np.where(c > 0.0, c, 1.0)
    h = np.zeros_like(v)
    chroma = c > 0.0
    r_max = chroma & (v == r)
    g_max = chroma & (v == g) & ~r_max
    b_max = chroma & (v == b) & ~r_max & ~g_max
    h = np.where(r_max, 60.0 * (g - b) / safe_c, h)
    h = np.where(g_max, 120.0 + 60.0 * (b - r) / safe_c, h)
    h = np.where(b_max, 240.0 + 60.0 * (r - g) / safe_c, h)
    return np.mod(h, 360.0) / 2.0, s, v


def fruit_hue_fractions(
    crop_bgr: np.ndarray, config: ColorGateConfig = ColorGateConfig()
) -> tuple[dict[str, float], float]:
    """
    Score how much of the crop's colour evidence matches each fruit's hue.

    Returns (per-fruit fraction of the NON-WHITE pixels inside that fruit's hue spans,
    fraction of all pixels that are non-white colour evidence). All zeros when the crop
    is empty or has no coloured pixels.
    """
    arr = np.asarray(crop_bgr)
    if arr.ndim != 3 or arr.shape[-1] < 3 or arr.size == 0:
        return {fruit: 0.0 for fruit in FRUIT_HUE_RANGES}, 0.0
    h, s, v = _hue_sat_val(arr[..., :3])
    colored = (s >= config.saturation_min) & (v >= config.value_min)
    total = float(h.size)
    n_colored = float(np.count_nonzero(colored))
    colored_fraction = n_colored / total if total > 0.0 else 0.0
    if n_colored <= 0.0:
        return {fruit: 0.0 for fruit in FRUIT_HUE_RANGES}, colored_fraction
    hue = h[colored]
    fractions: dict[str, float] = {}
    for fruit, spans in FRUIT_HUE_RANGES.items():
        in_span = np.zeros(hue.shape, dtype=bool)
        for lo, hi in spans:
            in_span |= (hue >= lo) & (hue < hi)
        fractions[fruit] = float(np.count_nonzero(in_span)) / n_colored
    return fractions, colored_fraction


def evaluate_color_consistency(
    crop_bgr: np.ndarray, label: str, config: ColorGateConfig = ColorGateConfig()
) -> ColorGateResult:
    """
    Decide whether the crop's colours contradict the classified fruit label.

    Veto ONLY when the expected hue fraction is under min_expected_fraction AND another
    fruit's hue both reaches dominant_min_fraction and exceeds the expected fraction by
    dominance_ratio. A crop with too little colour evidence (white cube, blank face) is
    never vetoed.
    """
    clean = str(label).strip().lower()
    if clean not in FRUIT_HUE_RANGES:
        return ColorGateResult(False, clean, 0.0, "", 0.0, 0.0, "unknown_label")
    fractions, colored_fraction = fruit_hue_fractions(crop_bgr, config)
    expected = fractions.get(clean, 0.0)
    others = {fruit: frac for fruit, frac in fractions.items() if fruit != clean}
    dominant_label = max(others, key=others.get) if others else ""
    dominant_fraction = others.get(dominant_label, 0.0)
    if colored_fraction < config.min_colored_fraction:
        return ColorGateResult(
            False, clean, expected, dominant_label, dominant_fraction,
            colored_fraction, "no_color_evidence",
        )
    if expected >= config.min_expected_fraction:
        return ColorGateResult(
            False, clean, expected, dominant_label, dominant_fraction,
            colored_fraction, "expected_hue_present",
        )
    # Banana and pineapple prints share yellow and can shift into brown/orange under the
    # arena's warm lighting. Treat strong pineapple-range evidence as ambiguous for a
    # banana prediction; this is a veto-only guard, so ambiguity must fail open.
    if clean == "banana" and fractions.get("pineapple", 0.0) >= config.dominant_min_fraction:
        return ColorGateResult(
            False, clean, expected, dominant_label, dominant_fraction,
            colored_fraction, "banana_pineapple_ambiguous",
        )
    dominance_floor = max(expected, 1e-6) * max(1.0, config.dominance_ratio)
    if dominant_fraction >= config.dominant_min_fraction and dominant_fraction >= dominance_floor:
        return ColorGateResult(
            True, clean, expected, dominant_label, dominant_fraction,
            colored_fraction, "conflicting_hue_dominant",
        )
    return ColorGateResult(
        False, clean, expected, dominant_label, dominant_fraction,
        colored_fraction, "no_dominant_conflict",
    )
