"""Body SigLIP pixel provenance wire-format tests."""

import pytest

from robot_perception.spatial_siglip import (
    build_body_siglip_source,
    parse_body_siglip_source,
)


def test_body_siglip_source_round_trip():
    source = build_body_siglip_source(319.75, 241.125)
    assert parse_body_siglip_source(source) == (319.75, 241.125)


@pytest.mark.parametrize(
    "source",
    ["", "siglip", "siglip_body:1", "siglip_body:x:2", "other:1:2"],
)
def test_body_siglip_source_rejects_malformed_values(source):
    assert parse_body_siglip_source(source) is None


def test_body_siglip_source_rejects_non_finite_values():
    with pytest.raises(ValueError):
        build_body_siglip_source(float("nan"), 1.0)
