"""Operator-facing action summaries for the recognition live UI."""

from robot_perception.nodes.recognition_viz_node import action_summary


def test_action_summary_names_target_during_navigation():
    assert action_summary("APPROACH", "banana", 12, 2) == "DRIVING TO banana #12"
    assert action_summary("ALIGN", "dodecahedron", 4, 1) == (
        "FINE ALIGNMENT: dodecahedron #4"
    )


def test_action_summary_explains_set_specific_verification():
    assert action_summary("CLASSIFY", "banana", 12, 2) == (
        "VERIFYING FRUIT WITH BODY SIGLIP: banana #12"
    )
    assert action_summary("CLASSIFY", "dodecahedron", 4, 1) == (
        "VERIFYING SHAPE AT GRIPPER: dodecahedron #4"
    )


def test_action_summary_shows_real_arm_step_and_terminal_states():
    assert action_summary("PICK", "banana", 12, 2, "GRASP") == "ARM GRASP: banana #12"
    assert action_summary("SCAN") == "SEARCHING FIELD FOR TARGETS"
    assert action_summary("END") == "MISSION COMPLETE"
