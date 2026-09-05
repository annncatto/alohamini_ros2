from pathlib import Path


WORKSPACE_SRC = Path(__file__).resolve().parents[2]


def _read(package: str, launch_file: str) -> str:
    return (WORKSPACE_SRC / package / "launch" / launch_file).read_text(
        encoding="utf-8"
    )


def test_bringup_is_the_only_full_hardware_composition():
    source = _read("alohamini_bringup", "hardware.launch.py")
    assert source.count('"bridge.launch.py"') == 1
    assert source.count('"description.launch.py"') == 1
    assert source.count('"camera.launch.py"') == 1
    assert source.count('"move_group.launch.py"') == 1
    assert source.count('"teleop.launch.py"') == 1


def test_bringup_defaults_to_receipt_time_and_forwards_state_age_gate():
    source = _read("alohamini_bringup", "hardware.launch.py")
    assert '"state_timestamp_mode", default_value="receipt"' in source
    assert '"camera_timestamp_mode", default_value="receipt"' in source
    assert '"max_state_response_age_sec", default_value="0.25"' in source
    assert source.count('"max_state_response_age_sec": LaunchConfiguration(') == 1


def test_moveit_component_does_not_own_bridge_or_state_publisher():
    for filename in ("move_group.launch.py", "hardware_execution.launch.py"):
        source = _read("alohamini_moveit_config", filename)
        assert "alohamini_lerobot_bridge" not in source
        assert 'package="robot_state_publisher"' not in source


def test_joycon_component_does_not_own_bridge_moveit_or_state_publisher():
    for filename in ("teleop.launch.py", "hardware.launch.py"):
        source = _read("alohamini_joycon_teleop", filename)
        assert "alohamini_lerobot_bridge" not in source
        assert "hardware_execution.launch.py" not in source
        assert 'package="robot_state_publisher"' not in source
