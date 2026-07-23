"""Body SigLIP batches attach each fruit result to its encoded source pixel."""

from robot_interfaces.msg import Classification

from robot_perception.nodes.world_model_node import Track, WorldModelNode
from robot_perception.spatial_siglip import build_body_siglip_source


def _fruit_track(track_id, x, y):
    return Track(
        id=track_id,
        x=x,
        y=y,
        confidence=0.9,
        last_seen_sec=10.0,
        class_label="fruit_photo_cube",
        set_type=2,
        body_votes={"fruit_photo_cube": 1.0},
    )


def test_spatial_body_result_ignores_legacy_primary_and_updates_matching_track():
    first = _fruit_track(1, 1.0, 1.0)
    second = _fruit_track(2, 2.0, 2.0)
    node = WorldModelNode.__new__(WorldModelNode)
    node.tracks = {1: first, 2: second}
    node.body_siglip_assoc_radius = 0.2
    node._fruit_attach_window = 1.0
    node._last_body_fruit_id = 2       # legacy heuristic deliberately points elsewhere
    node._last_body_fruit_sec = 10.0
    node._now_sec = lambda: 10.2
    node._pose_at = lambda _stamp: (0.0, 0.0, 0.0)
    node._project_body_pixel = (
        lambda u, _v, _pose: (1.02, 1.0) if u < 400.0 else (2.02, 2.0)
    )

    msg = Classification()
    msg.header.stamp.sec = 10
    msg.label = "banana"
    msg.confidence = 0.8
    msg.image_face_visible = True
    msg.source = build_body_siglip_source(320.0, 240.0)
    node.on_siglip(msg)

    assert first.fruit_label == "banana"
    assert second.fruit_label == ""
