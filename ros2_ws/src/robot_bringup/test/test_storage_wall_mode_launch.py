import importlib.util
from pathlib import Path


LAUNCH_PATH = Path(__file__).parents[1] / "launch" / "test_field.launch.py"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("test_field_launch", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_storage_wall_mode_default_is_read_from_motion_tuning(tmp_path):
    module = _load_launch_module()
    tuning = tmp_path / "motion_tuning.yaml"
    tuning.write_text(
        "mission_fsm_node:\n"
        "  ros__parameters:\n"
        "    storage_wall_guided_enabled: true\n",
        encoding="utf-8",
    )

    assert module._yaml_bool_parameter(
        str(tuning), "mission_fsm_node", "storage_wall_guided_enabled"
    )


def test_storage_wall_mode_reader_falls_back_safely(tmp_path):
    module = _load_launch_module()
    tuning = tmp_path / "motion_tuning.yaml"
    tuning.write_text("mission_fsm_node: {}\n", encoding="utf-8")

    assert not module._yaml_bool_parameter(
        str(tuning), "mission_fsm_node", "storage_wall_guided_enabled"
    )
