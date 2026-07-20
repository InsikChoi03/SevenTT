"""Pure geometry helpers for arena-wall heading correction."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


@dataclass(frozen=True)
class WallHeadingEstimate:
    """Robust field-axis heading error derived from undirected wall segments."""

    correction_rad: float
    variance_rad2: float
    segment_count: int


def classify_wall_segments(
    segments: Iterable[tuple[float, float, float, float]],
    *,
    min_length_m: float = 0.12,
    axis_tolerance_rad: float = math.radians(35.0),
) -> tuple[
    list[tuple[tuple[float, float, float, float], bool]],
    WallHeadingEstimate | None,
]:
    """Classify field-projected walls and return the rotation that aligns them to field axes.

    Lines are undirected, so their angle is evaluated modulo pi. A wall observed at +10 degrees
    from the horizontal field axis therefore produces a -10 degree pose correction.
    """
    usable: list[tuple[tuple[float, float, float, float], bool]] = []
    errors: list[float] = []
    tolerance = max(0.0, float(axis_tolerance_rad))
    for raw in segments:
        segment = tuple(float(value) for value in raw)
        x0, y0, x1, y1 = segment
        dx, dy = x1 - x0, y1 - y0
        if math.hypot(dx, dy) < float(min_length_m):
            continue
        line_angle = math.atan2(dy, dx) % math.pi
        horizontal = min(line_angle, math.pi - line_angle) <= tolerance
        target_angle = 0.0 if horizontal else math.pi * 0.5
        error = (target_angle - line_angle + math.pi * 0.5) % math.pi - math.pi * 0.5
        if abs(error) > tolerance:
            continue
        usable.append((segment, horizontal))
        errors.append(error)

    if not errors:
        return usable, None
    ordered = sorted(errors)
    middle = len(ordered) // 2
    correction = (
        ordered[middle]
        if len(ordered) % 2
        else 0.5 * (ordered[middle - 1] + ordered[middle])
    )
    variance = sum((error - correction) ** 2 for error in errors) / len(errors)
    return usable, WallHeadingEstimate(correction, variance, len(errors))


def heading_confidence(
    estimate: WallHeadingEstimate,
    *,
    full_count: int = 2,
    max_stddev_rad: float = math.radians(10.0),
) -> float:
    """Return a conservative 0..1 heading confidence from count and angular agreement."""
    count_factor = min(1.0, estimate.segment_count / max(1, int(full_count)))
    stddev = math.sqrt(max(0.0, estimate.variance_rad2))
    spread_factor = max(0.0, 1.0 - stddev / max(1e-6, float(max_stddev_rad)))
    return count_factor * spread_factor
