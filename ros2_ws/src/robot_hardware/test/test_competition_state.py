from robot_hardware.competition_state import (
    READY,
    RUNNING,
    STANDBY,
    CompetitionStateMachine,
)


def test_start_sequence_advances_only_standby_ready_running():
    machine = CompetitionStateMachine()

    assert machine.state == STANDBY
    assert machine.accept_start(1) == READY
    assert machine.accept_start(2) == RUNNING
    assert machine.accept_start(3) is None
    assert machine.state == RUNNING


def test_duplicate_start_sequence_is_ignored_without_advancing():
    machine = CompetitionStateMachine()

    assert machine.accept_start(7) == READY
    assert machine.accept_start(7) is None
    assert machine.state == READY
    assert machine.accept_start(8) == RUNNING


def test_non_positive_sequence_is_ignored():
    machine = CompetitionStateMachine()

    assert machine.accept_start(0) is None
    assert machine.accept_start(-1) is None
    assert machine.state == STANDBY
