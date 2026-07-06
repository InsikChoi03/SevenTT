# Drive Tests

This folder is for low-level base motion experiments that should stay separate from the full
ROS2 bringup stack.

Current recommendation for encoder-based distance tests:

1. Flash the encoder-enabled base firmware first.
   - File: `firmware/base_arm_combined/base_arm_combined.ino`
   - It now emits `<ODOM,fl,fr,rl,rr,t_ms>` from encoder ticks.
   - It also emits `<ENC,e1,e2,e3,e4,t_ms>` raw ticks for mapping checks.
   - Default encoder pins follow the QGPMaker V5.2 map:
     `E1=(8,9), E2=(6,7), E3=(3,2), E4=(5,4)`.

2. Verify encoder packets without ROS2.
   - Use a standalone serial test script from this folder.
   - Keep this stage simple: connect, command a straight move, watch odometry accumulate,
     stop when the target distance is reached.
   - If a wheel moves but the wrong raw encoder changes, adjust `WHEEL_ENCODER`.
   - If a wheel speed sign is reversed, adjust `WHEEL_ENC_DIR`.

3. Only after that, wire the same odometry into ROS2.
   - `robot_hardware/nodes/mcu_bridge_base_node.py` already accepts `<ODOM,...>`.
   - `robot_perception/nodes/localizer_node.py` already integrates `/base/wheel_odom`.

Suggested split of responsibilities:

- Firmware:
  Read encoder ticks, convert to wheel speeds, send `<ODOM,...>`.
- `scripts/drive_tests/`:
  Fast serial-only experiments and calibration.
- ROS2 nodes:
  Reuse the validated odometry stream later for autonomous driving.

Why keep this separate:

- Fewer moving parts while bring-up is unstable.
- Easier to tell whether a failure is in encoder wiring, serial protocol, or localization.
- Safer to test short distance moves without the full mission stack active.
