# robot_parking_test

Independent arrival-marker reverse parking test package.

This package is intentionally separate from the competition mission pipeline. It does not use
`mission_fsm_node`, `target_selector_node`, `explorer_node`, `go_to_goal_node`, or arm nodes.

## Safety Defaults

- `drive_enabled=false`
- `armed=false`
- `auto_start=false`
- `/parking_test/deadman` heartbeat is required before nonzero `/base_command`
- No rear distance sensor is available. Watch the robot and be ready to abort.

`ABORT`:

```bash
ros2 topic pub --once /parking_test/abort std_msgs/Bool "{data: true}"
```

## Observe-Only Zone-4 Test

```bash
ros2 launch robot_parking_test arrival_parking_test.launch.py with_base:=false
```

Watch:

```bash
ros2 topic echo /parking_test/state
ros2 topic echo /parking_test/debug
```

Optional web viewer:

```bash
ros2 launch robot_parking_test parking_web.launch.py
```

Open:

```text
http://127.0.0.1:8090/
```

## Real Base Test

Run only after observe-only confirms the arrival pair and target are correct.

```bash
ros2 launch robot_parking_test arrival_parking_test.launch.py \
  with_base:=true drive_enabled:=true armed:=true auto_start:=true
```

Hold-to-run deadman in another terminal:

```bash
ros2 topic pub /parking_test/deadman std_msgs/Bool "{data: true}" -r 10
```

Manual start/stop gates:

```bash
ros2 topic pub --once /parking_test/arm std_msgs/Bool "{data: true}"
ros2 topic pub --once /parking_test/start std_msgs/Bool "{data: true}"
ros2 topic pub --once /parking_test/start std_msgs/Bool "{data: false}"
```

## Tunable Layout Parameters

The default arrival hint is `center_hint_x=1.8`, `center_hint_y=1.8`.
The default start pose is `initial_x=-1.8`, `initial_y=1.8`, `initial_theta=0.0`
so the robot starts in quadrant 2 facing +x toward the quadrant-1 arrival area.
The floor pair spacing gate is tunable through:

- `floor_pair_spacing_m`
- `floor_pair_spacing_tolerance_m`
- `floor_pair_min_spacing_m`
- `floor_pair_max_spacing_m`
- `parking_depth_offset_m`

These are loaded from `config/arrival_parking_test.yaml`, then optionally overridden by
`robot_bringup/config/motion_tuning.yaml` when launched through `arrival_parking_test.launch.py`.
