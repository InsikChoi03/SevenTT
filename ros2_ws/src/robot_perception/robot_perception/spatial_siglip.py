"""Encode the Body detection pixel that produced a SigLIP classification.

``robot_interfaces/Classification`` has no bounding-box field.  Keeping the pixel in
``source`` lets downstream consumers bind a batched Body result to the same physical
object without changing and rebuilding the ROS interface package.
"""
from __future__ import annotations

import math


BODY_SIGLIP_SOURCE = "siglip_body"


def build_body_siglip_source(u_px: float, v_px: float) -> str:
    """Return a compact provenance string carrying a finite Body pixel."""
    u = float(u_px)
    v = float(v_px)
    if not (math.isfinite(u) and math.isfinite(v)):
        raise ValueError("Body SigLIP pixel must be finite")
    return f"{BODY_SIGLIP_SOURCE}:{u:.3f}:{v:.3f}"


def parse_body_siglip_source(source: str) -> tuple[float, float] | None:
    """Read a Body pixel from ``Classification.source`` or return ``None``."""
    parts = str(source).strip().split(":")
    if len(parts) != 3 or parts[0] != BODY_SIGLIP_SOURCE:
        return None
    try:
        u = float(parts[1])
        v = float(parts[2])
    except ValueError:
        return None
    if not (math.isfinite(u) and math.isfinite(v)):
        return None
    return u, v
