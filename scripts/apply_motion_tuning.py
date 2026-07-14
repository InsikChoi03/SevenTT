#!/usr/bin/env python3
"""Apply match motion tuning values to ROS parameter YAML files.

The script intentionally edits only known parameter keys and preserves the
surrounding comments in perception.yaml/test_field.yaml.
"""

from __future__ import annotations

import argparse
import datetime as dt
import os
from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TUNING = ROOT / "ros2_ws/src/robot_bringup/config/motion_tuning.yaml"
DEFAULT_TARGETS = [
    ROOT / "ros2_ws/src/robot_bringup/config/perception.yaml",
    ROOT / "ros2_ws/src/robot_bringup/config/test_field.yaml",
]


# tuning.path -> [(ros_node, ros_param), ...]
MAPPING: dict[str, list[tuple[str, str]]] = {
    "targets.set1_label": [("mission_fsm_node", "set1_label"), ("target_selector_node", "set1_label")],
    "targets.set2_label": [("mission_fsm_node", "set2_label"), ("target_selector_node", "set2_label"), ("siglip_gate_node", "set2_label")],
    "targets.shape_target_total": [("mission_fsm_node", "shape_target_total")],
    "targets.fruit_target_total": [("mission_fsm_node", "fruit_target_total")],
    "targets.target_selector.rate_hz": [("target_selector_node", "rate_hz")],
    "targets.target_selector.explore_base": [("target_selector_node", "explore_base")],
    "targets.target_selector.set1_base": [("target_selector_node", "set1_base")],
    "targets.target_selector.set2_base": [("target_selector_node", "set2_base")],
    "targets.target_selector.zone_filter_enabled": [("target_selector_node", "zone_filter_enabled")],
    "opening.enabled": [("mission_fsm_node", "opening_enabled")],
    "opening.startup_warmup_sec": [("mission_fsm_node", "startup_warmup_sec")],
    "opening.forward_speed": [("mission_fsm_node", "opening_speed")],
    "opening.forward_sec": [("mission_fsm_node", "opening_forward_sec")],
    "opening.strafe_right_speed": [("mission_fsm_node", "opening_strafe_speed")],
    "opening.strafe_right_sec": [("mission_fsm_node", "opening_strafe_right_sec")],
    "opening.turn_deg": [("mission_fsm_node", "opening_turn_deg")],
    "opening.turn_omega": [("mission_fsm_node", "opening_turn_omega")],
    "opening.wait_after_turn_sec": [("mission_fsm_node", "opening_wait_after_turn_sec")],
    "scan.search_omega": [("mission_fsm_node", "scan_search_omega")],
    "scan.search_turn_sec": [("mission_fsm_node", "search_turn_sec")],
    "scan.search_look_sec": [("mission_fsm_node", "search_look_sec")],
    "approach.align_start_dist_m": [("mission_fsm_node", "approach_dist_m")],
    "approach.face_tol_rad": [("mission_fsm_node", "approach_face_tol")],
    "approach.speed": [("mission_fsm_node", "approach_speed")],
    "approach.kp_ang": [("mission_fsm_node", "approach_kp_ang")],
    "approach.omega_max": [("mission_fsm_node", "approach_omega_max")],
    "approach.look_sec": [("mission_fsm_node", "approach_look_sec")],
    "approach.move_sec": [("mission_fsm_node", "approach_move_sec")],
    "approach.lost_grace_sec": [("mission_fsm_node", "approach_lost_grace_sec")],
    "approach.standoff_tol_m": [("mission_fsm_node", "approach_standoff_tol")],
    "approach.classify_standoff_sec": [("mission_fsm_node", "classify_standoff_sec")],
    "grab.x_m": [("mission_fsm_node", "grab_x")],
    "grab.min_x_m": [("mission_fsm_node", "grab_min_x")],
    "grab.sonar_range_m": [("mission_fsm_node", "grab_range_m")],
    "grab.sonar_align_enabled": [("mission_fsm_node", "use_sonar_align")],
    "align.settle_sec": [("mission_fsm_node", "align_settle_sec")],
    "align.fwd_duty": [("mission_fsm_node", "align_step_fwd_duty")],
    "align.fwd_sec": [("mission_fsm_node", "align_step_fwd_sec")],
    "align.adaptive_steps_enabled": [("mission_fsm_node", "align_adaptive_steps_enabled")],
    "align.mid_error_m": [("mission_fsm_node", "align_mid_error_m")],
    "align.fwd_mid_sec": [("mission_fsm_node", "align_step_fwd_mid_sec")],
    "align.strafe_duty": [("mission_fsm_node", "align_step_strafe_duty")],
    "align.strafe_sec": [("mission_fsm_node", "align_step_strafe_sec")],
    "align.strafe_mid_sec": [("mission_fsm_node", "align_step_strafe_mid_sec")],
    "align.body_lost_backoff_enabled": [("mission_fsm_node", "align_body_lost_backoff_enabled")],
    "align.body_lost_backoff_speed": [("mission_fsm_node", "align_body_lost_backoff_speed")],
    "align.body_lost_backoff_sec": [("mission_fsm_node", "align_body_lost_backoff_sec")],
    "align.body_lost_backoff_settle_sec": [("mission_fsm_node", "align_body_lost_backoff_settle_sec")],
    "align.settle_pulse_sec": [("mission_fsm_node", "align_settle_pulse_sec")],
    "align.lateral_tol_m": [("mission_fsm_node", "align_tol_m")],
    "align.fwd_tol_m": [("mission_fsm_node", "align_fwd_tol_m")],
    "align.sonar_tol_m": [("mission_fsm_node", "align_sonar_tol_m")],
    "arm.init_pose": [("pick_sequencer_node", "init_pose")],
    "arm.pick_shoulder_wrist": [("pick_sequencer_node", "pick_shoulder_wrist")],
    "arm.place_shoulder_wrist": [("pick_sequencer_node", "place_shoulder_wrist")],
    "arm.grip_open": [("pick_sequencer_node", "grip_open")],
    "arm.grip_closed": [("pick_sequencer_node", "grip_closed")],
    "arm.move_sec": [("pick_sequencer_node", "move_sec")],
    "arm.grasp_sec": [("pick_sequencer_node", "grasp_sec")],
    "arm.reach_grip_open_frac": [("pick_sequencer_node", "reach_grip_open_frac")],
    "zone_mission.enabled": [("mission_fsm_node", "zone_mission_enabled")],
    "zone_mission.order": [("mission_fsm_node", "zone_order")],
    "zone_mission.bounds_m": [
        ("mission_fsm_node", "zone_bounds_m"),
        ("target_selector_node", "zone_bounds_m"),
        ("recognition_viz_node", "zone_bounds_m"),
    ],
    "zone_mission.no_target_advance_sec": [("mission_fsm_node", "zone_no_target_advance_sec")],
    "zone_mission.center_reach_tol_m": [("mission_fsm_node", "zone_center_reach_tol_m")],
    "planner.enabled": [("mission_fsm_node", "planner_enabled")],
    "planner.mode": [("mission_fsm_node", "planner_mode")],
    "planner.grid_spacing_m": [("mission_fsm_node", "grid_spacing_m")],
    "planner.grid_origin_mode": [("mission_fsm_node", "grid_origin_mode")],
    "planner.grid_origin_xy": [("mission_fsm_node", "grid_origin_xy")],
    "planner.phase_tol_m": [("mission_fsm_node", "phase_tol_m")],
    "planner.field_bounds_m": [("mission_fsm_node", "field_bounds_m"), ("go_to_goal_node", "field_bounds_m")],
    "planner.robot_margin_m": [("mission_fsm_node", "robot_margin_m"), ("go_to_goal_node", "robot_margin_m")],
    "planner.lane_block_radius_m": [("mission_fsm_node", "lane_block_radius_m")],
    "planner.comfort_clear_m": [("mission_fsm_node", "comfort_clear_m")],
    "planner.clearance_weight": [("mission_fsm_node", "clearance_weight")],
    "planner.start_connect_k": [("mission_fsm_node", "start_connect_k")],
    "planner.waypoint_reach_tol_m": [("mission_fsm_node", "wp_reach_tol_m")],
    "planner.replan_period_sec": [("mission_fsm_node", "replan_period_sec")],
    "planner.replan_goal_move_m": [("mission_fsm_node", "replan_goal_move_m")],
    "planner.replan_throttle_sec": [("mission_fsm_node", "replan_throttle_sec")],
    "planner.obstacle_min_conf": [("mission_fsm_node", "obstacle_min_conf")],
    "planner.obstacle_min_nobs": [("mission_fsm_node", "obstacle_min_nobs")],
    "planner.exclude_target_radius_m": [("mission_fsm_node", "exclude_target_radius_m")],
    "go_to_goal.kp_lin": [("go_to_goal_node", "kp_lin")],
    "go_to_goal.max_lin_speed": [("go_to_goal_node", "max_lin_speed")],
    "go_to_goal.min_lin_speed": [("go_to_goal_node", "min_lin_speed")],
    "go_to_goal.max_ang_speed": [("go_to_goal_node", "max_ang_speed")],
    "go_to_goal.fine_radius_m": [("go_to_goal_node", "fine_radius_m")],
    "go_to_goal.face_tol_rad": [("go_to_goal_node", "face_tol_rad")],
    "go_to_goal.avoid_radius_m": [("go_to_goal_node", "avoid_radius_m")],
    "go_to_goal.avoid_gain": [("go_to_goal_node", "avoid_gain")],
    "go_to_goal.avoid_goal_skip_m": [("go_to_goal_node", "avoid_goal_skip_m")],
    "go_to_goal.front_stop_m": [("go_to_goal_node", "front_stop_m")],
    "go_to_goal.sonar_timeout_sec": [("go_to_goal_node", "sonar_timeout_sec")],
    "base_controller.wheel_scales": [("base_controller_node", "wheel_scales")],
    "base_controller.strafe_right_scales": [("base_controller_node", "strafe_right_scales")],
    "base_controller.strafe_left_scales": [("base_controller_node", "strafe_left_scales")],
    "base_controller.wheel_min": [("base_controller_node", "wheel_min")],
    "base_controller.wheel_min_rot": [("base_controller_node", "wheel_min_rot")],
    "base_controller.wheel_boost": [("base_controller_node", "wheel_boost")],
    "base_controller.wheel_boost_ms": [("base_controller_node", "wheel_boost_ms")],
    "base_controller.wheel_slew_per_tick": [("base_controller_node", "wheel_slew_per_tick")],
    "base_controller.align_wheel_boost": [("base_controller_node", "align_wheel_boost")],
    "base_controller.align_brake_off": [("base_controller_node", "align_brake_off")],
    "base_controller.wheel_deadband": [("base_controller_node", "wheel_deadband")],
    "base_controller.wheel_brake_ms": [("base_controller_node", "wheel_brake_ms")],
    "base_controller.wheel_brake_scale": [("base_controller_node", "wheel_brake_scale")],
    "world_model.grid_prior_enabled": [("world_model_node", "grid_prior_enabled")],
    "world_model.grid_prior_debug": [("world_model_node", "grid_prior_debug")],
    "world_model.grid_rows": [("world_model_node", "grid_rows"), ("recognition_viz_node", "grid_rows")],
    "world_model.grid_cols": [("world_model_node", "grid_cols"), ("recognition_viz_node", "grid_cols")],
    "world_model.grid_spacing_m": [("world_model_node", "grid_spacing_m"), ("recognition_viz_node", "grid_spacing_m")],
    "world_model.grid_origin_x_m": [("world_model_node", "grid_origin_x_m"), ("recognition_viz_node", "grid_origin_x_m")],
    "world_model.grid_origin_y_m": [("world_model_node", "grid_origin_y_m"), ("recognition_viz_node", "grid_origin_y_m")],
    "world_model.grid_initial_phase_sec": [("world_model_node", "grid_initial_phase_sec")],
    "world_model.grid_initial_snap_radius_m": [("world_model_node", "grid_initial_snap_radius_m")],
    "world_model.grid_new_snap_radius_m": [("world_model_node", "grid_new_snap_radius_m")],
    "world_model.grid_initial_snap_alpha": [("world_model_node", "grid_initial_snap_alpha")],
    "world_model.grid_new_snap_alpha": [("world_model_node", "grid_new_snap_alpha")],
    "world_model.grid_moved_threshold_m": [("world_model_node", "grid_moved_threshold_m")],
    "world_model.body_fov_half_deg": [("world_model_node", "body_fov_half_deg"), ("recognition_viz_node", "body_fov_half_deg")],
    "world_model.body_fov_near_m": [("world_model_node", "body_fov_near_m"), ("recognition_viz_node", "body_fov_near_m")],
    "world_model.body_fov_far_m": [("world_model_node", "body_fov_far_m"), ("recognition_viz_node", "body_fov_far_m")],
    "world_model.body_fov_apex_x": [("world_model_node", "body_fov_apex_x"), ("recognition_viz_node", "body_fov_apex_x")],
    "world_model.wide_fov_forward_m": [("world_model_node", "wide_fov_forward_m"), ("recognition_viz_node", "wide_fov_forward_m")],
    "world_model.wide_fov_lateral_m": [("world_model_node", "wide_fov_lateral_m"), ("recognition_viz_node", "wide_fov_lateral_m")],
    "world_model.wide_fov_center_x": [("world_model_node", "wide_fov_center_x"), ("recognition_viz_node", "wide_fov_center_x")],
    "live_view.show_camera_panels": [("recognition_viz_node", "show_camera_panels")],
    "live_view.write_live_png": [("recognition_viz_node", "write_live_png")],
    "live_view.redraw_rate_hz": [("recognition_viz_node", "redraw_rate_hz")],
    "live_view.show_object_grid_points": [("recognition_viz_node", "show_object_grid_points")],
    "live_view.show_zone_regions": [("recognition_viz_node", "show_zone_regions")],
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tuning", type=Path, default=DEFAULT_TUNING)
    parser.add_argument(
        "--target",
        action="append",
        type=Path,
        help="ROS parameter YAML to update. Repeatable. Defaults to perception.yaml and test_field.yaml.",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print planned changes without writing files.")
    parser.add_argument("--no-history", action="store_true", help="Do not copy the applied tuning into logs/tuning_history.")
    return parser.parse_args()


def get_nested(data: dict[str, Any], dotted: str) -> Any:
    cur: Any = data
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return None
        cur = cur[part]
    return cur


def yaml_value(value: Any) -> str:
    text = yaml.safe_dump(value, default_flow_style=True, sort_keys=False, allow_unicode=False).strip()
    if text.endswith("\n..."):
        text = text[:-4].strip()
    if "\n" in text:
        text = " ".join(line.strip() for line in text.splitlines())
    return text


def split_comment(text: str) -> tuple[str, str]:
    if "#" not in text:
        return text.rstrip(), ""
    before, after = text.split("#", 1)
    return before.rstrip(), "  #" + after.rstrip()


def build_updates(tuning: dict[str, Any]) -> dict[tuple[str, str], Any]:
    updates: dict[tuple[str, str], Any] = {}
    missing: list[str] = []
    for dotted, targets in MAPPING.items():
        value = get_nested(tuning, dotted)
        if value is None:
            missing.append(dotted)
            continue
        for node, param in targets:
            updates[(node, param)] = value
    if missing:
        print("Skipped missing tuning keys:")
        for key in missing:
            print(f"  - {key}")
    return updates


def update_ros_yaml(path: Path, updates: dict[tuple[str, str], Any], dry_run: bool) -> int:
    lines = path.read_text(encoding="utf-8").splitlines()
    out: list[str] = []
    current_node: str | None = None
    in_params = False
    applied = 0

    for line in lines:
        stripped = line.strip()
        indent = len(line) - len(line.lstrip(" "))

        if indent == 0 and stripped.endswith(":") and not stripped.startswith("#"):
            current_node = stripped[:-1]
            in_params = False
            out.append(line)
            continue

        if current_node and indent == 2 and stripped == "ros__parameters:":
            in_params = True
            out.append(line)
            continue

        if in_params and current_node and indent >= 4 and ":" in stripped and not stripped.startswith("#"):
            key = stripped.split(":", 1)[0].strip()
            update_key = (current_node, key)
            if update_key in updates:
                prefix = line[: line.find(key)]
                before_comment, comment = split_comment(line)
                has_comment = bool(comment)
                formatted = yaml_value(updates[update_key])
                new_line = f"{prefix}{key}: {formatted}"
                if has_comment:
                    old_comment = comment.strip()
                    new_line = f"{new_line:<44} {old_comment}"
                out.append(new_line.rstrip())
                applied += 1
                continue

        out.append(line)

    if not dry_run:
        path.write_text("\n".join(out) + "\n", encoding="utf-8")
    return applied


def write_history(tuning_path: Path, tuning: dict[str, Any], dry_run: bool) -> Path | None:
    if dry_run:
        return None
    history_dir = ROOT / "logs/tuning_history"
    history_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y-%m-%d_%H%M%S")
    name = str(tuning.get("metadata", {}).get("name", "motion_tuning")).replace("/", "_")
    dst = history_dir / f"{stamp}_{name}.yaml"
    dst.write_text(tuning_path.read_text(encoding="utf-8"), encoding="utf-8")
    return dst


def main() -> int:
    args = parse_args()
    tuning_path = args.tuning.resolve()
    target_paths = [p.resolve() for p in (args.target or DEFAULT_TARGETS)]

    tuning = yaml.safe_load(tuning_path.read_text(encoding="utf-8"))
    if not isinstance(tuning, dict):
        raise SystemExit(f"Invalid tuning YAML: {tuning_path}")

    updates = build_updates(tuning)
    total = 0
    for target in target_paths:
        count = update_ros_yaml(target, updates, args.dry_run)
        total += count
        verb = "Would update" if args.dry_run else "Updated"
        print(f"{verb} {count} values in {target}")

    if not args.no_history:
        hist = write_history(tuning_path, tuning, args.dry_run)
        if hist:
            print(f"Wrote tuning history: {hist}")

    print(f"Total values {'to update' if args.dry_run else 'updated'}: {total}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
