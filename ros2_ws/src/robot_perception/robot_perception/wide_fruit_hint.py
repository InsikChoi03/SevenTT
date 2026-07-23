"""
Wide-cam SigLIP fruit-hint payload shared by siglip_gate_node and world_model_node.

The wide (top) camera can pre-read the printed fruit on far Set2 cubes while the robot is
still driving, but robot_interfaces/Classification carries no pixel position and the .msg
files must not be rebuilt mid-competition. The hint therefore travels as a compact JSON
std_msgs/String on WIDE_FRUIT_HINT_TOPIC: siglip_gate_node builds it (build_wide_hint)
with the wide-detection pixel that produced the crop, and world_model_node parses it
(parse_wide_hint), projects that pixel through its existing wide-cam projection and
attaches the fruit to the matching Set2 track as a ROUTING HINT only. Hints are advisory:
they must never satisfy the body-cam pick gate (see world_model_node.on_wide_fruit_hint).

Pure stdlib (json/math only): safe to import in dry-run and unit tests.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Optional

WIDE_FRUIT_HINT_TOPIC = "/classification/siglip_wide_hint"

_REQUIRED_FIELDS = ("label", "confidence", "u_px", "v_px", "stamp_sec")


@dataclass(frozen=True)
class WideFruitHint:
    """One wide-cam SigLIP read: fruit label + the wide-detection pixel it came from."""

    label: str
    confidence: float   # SigLIP margin (best minus runner-up fruit softmax), NOT gate conf
    u_px: float
    v_px: float
    stamp_sec: float    # wide DetectionArray capture stamp (seconds)


def build_wide_hint(
    label: str, confidence: float, u_px: float, v_px: float, stamp_sec: float
) -> str:
    """Serialize one wide-cam fruit hint to the JSON wire format."""
    return json.dumps(
        {
            "label": str(label),
            "confidence": float(confidence),
            "u_px": float(u_px),
            "v_px": float(v_px),
            "stamp_sec": float(stamp_sec),
        }
    )


def parse_wide_hint(payload: str) -> Optional[WideFruitHint]:
    """Parse and validate a hint payload; None for malformed/incomplete/non-finite data."""
    try:
        data = json.loads(payload)
    except (TypeError, ValueError):
        return None
    if not isinstance(data, dict) or any(key not in data for key in _REQUIRED_FIELDS):
        return None
    label = str(data["label"]).strip().lower()
    if not label:
        return None
    try:
        confidence = float(data["confidence"])
        u_px = float(data["u_px"])
        v_px = float(data["v_px"])
        stamp_sec = float(data["stamp_sec"])
    except (TypeError, ValueError):
        return None
    values = (confidence, u_px, v_px, stamp_sec)
    if not all(math.isfinite(value) for value in values):
        return None
    return WideFruitHint(
        label=label,
        confidence=max(0.0, min(1.0, confidence)),
        u_px=u_px,
        v_px=v_px,
        stamp_sec=stamp_sec,
    )
