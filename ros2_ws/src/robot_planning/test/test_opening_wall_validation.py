from robot_planning.nodes.mission_fsm_node import MissionFsmNode


class _Logger:
    def info(self, _message):
        pass


class _OpeningStub:
    _opening_leg = "heading_observe"
    opening_wall_validation_enabled = False
    opening_heading_observe_sec = 1.2
    _opening_wall_heading_valid = True
    _wall_translation_unlocked = False
    state_enter_s = 0.0

    def __init__(self, elapsed):
        self.elapsed = elapsed
        self.mapping_events = []

    def _time_in_state(self):
        return self.elapsed

    def _now_s(self):
        return 10.0

    def _drive(self, *_args):
        pass

    def _set_world_mapping_enabled(self, enabled, reason):
        self.mapping_events.append((enabled, reason))

    def _opening_heading_debug_fresh(self):
        raise AssertionError("disabled wall validation must not wait for wall debug")

    def get_logger(self):
        return _Logger()


def test_disabled_wall_validation_waits_for_stationary_observe_period():
    node = _OpeningStub(elapsed=1.0)

    MissionFsmNode._step_opening(node)

    assert node._opening_leg == "heading_observe"
    assert not any(enabled for enabled, _reason in node.mapping_events)


def test_disabled_wall_validation_releases_opening_without_wall_debug():
    node = _OpeningStub(elapsed=1.2)

    MissionFsmNode._step_opening(node)

    assert node._opening_leg == "settle"
    assert node.state_enter_s == 10.0
    assert node._opening_wall_heading_valid is False
    assert node._wall_translation_unlocked is True
    assert node.mapping_events[-1] == (True, "opening wall validation disabled")


def test_zone_stabilize_reuses_fast_translation_only_wall_correction():
    node = MissionFsmNode.__new__(MissionFsmNode)
    node.state = "ZONE_STABILIZE"
    node._wall_translation_unlocked = True

    assert node._wall_fast_correction_requested()
    assert node._wall_correction_mode_requested() == "TRANSLATION_ONLY"
