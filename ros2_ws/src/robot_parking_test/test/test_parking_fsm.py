import math

from robot_parking_test.parking_fsm import (
    ArrivalParkingFsm,
    ParkingConfig,
    target_from_marker_center,
)


def _step_pose(pose, cmd, dt):
    x, y, th = pose
    x += (cmd.vx * math.cos(th) - cmd.vy * math.sin(th)) * dt
    y += (cmd.vx * math.sin(th) + cmd.vy * math.cos(th)) * dt
    th = math.atan2(math.sin(th + cmd.omega * dt), math.cos(th + cmd.omega * dt))
    return x, y, th


def test_reverse_parking_never_commands_forward_and_finishes_zero():
    cfg = ParkingConfig(
        latch_stable_sec=0.0,
        settle_hold_sec=0.05,
        approach_standoff_m=0.20,
        max_reverse_speed=0.08,
        min_reverse_speed=0.02,
        reverse_kp=0.8,
    )
    fsm = ArrivalParkingFsm(cfg)
    target = target_from_marker_center(1.8, 1.8, (0, 1), 1.0, cfg)
    pose = (1.70, 1.70, math.radians(-135.0))
    now = 0.0
    dt = 0.05
    fsm.start(now)
    saw_reverse = False
    for _ in range(160):
        cmd = fsm.tick(now, pose, target, True)
        if fsm.state == "STRAIGHT_REVERSE":
            assert cmd.vx <= 1e-9
        if cmd.vx < 0.0:
            saw_reverse = True
        pose = _step_pose(pose, cmd, dt)
        now += dt
        if fsm.state == "SUCCESS":
            break
    assert saw_reverse
    assert fsm.state == "SUCCESS"
    cmd = fsm.tick(now, pose, target, True)
    assert cmd.vx == 0.0
    assert cmd.vy == 0.0
    assert cmd.omega == 0.0


def test_target_in_front_approaches_reverse_start_instead_of_faulting():
    cfg = ParkingConfig(latch_stable_sec=0.0)
    fsm = ArrivalParkingFsm(cfg)
    target = target_from_marker_center(1.8, 1.8, (0, 1), 1.0, cfg)
    pose = (1.95, 1.95, math.radians(-135.0))
    fsm.start(0.0)
    fsm.tick(0.0, pose, target, True)
    fsm.tick(0.1, pose, target, True)
    cmd = fsm.tick(0.2, pose, target, True)
    assert fsm.state == "APPROACH_REVERSE_START"
    assert cmd.vx == 0.0
    cmd = fsm.tick(0.3, pose, target, True)
    assert fsm.state == "APPROACH_REVERSE_START"
    assert cmd.vx > 0.0


def test_acquire_wait_does_not_consume_total_parking_timeout():
    cfg = ParkingConfig(latch_stable_sec=0.0, total_timeout_sec=1.0)
    fsm = ArrivalParkingFsm(cfg)
    target = target_from_marker_center(1.8, 1.8, (0, 1), 1.0, cfg)
    pose = (1.70, 1.70, math.radians(-135.0))

    fsm.start(0.0)
    for now in (0.2, 0.8, 1.5, 3.0):
        cmd = fsm.tick(now, pose, None, False)
        assert fsm.state == "ACQUIRE"
        assert cmd.vx == 0.0
        assert cmd.vy == 0.0
        assert cmd.omega == 0.0

    fsm.tick(3.1, pose, target, True)
    fsm.tick(3.2, pose, target, True)
    assert fsm.state == "READY"
    fsm.tick(3.3, pose, target, True)
    assert fsm.state == "APPROACH_REVERSE_START"
