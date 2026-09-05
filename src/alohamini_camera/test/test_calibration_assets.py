from pathlib import Path

import yaml


CALIBRATION = Path(__file__).parents[2] / "alohamini_calibration/config/cameras"


def test_existing_intrinsics_have_consistent_frames_without_fabricating_missing_cameras():
    expected = {
        "forward": "forward_camera_optical",
        "wrist_right": "right_camera_optical",
    }
    for camera_name, frame_id in expected.items():
        document = yaml.safe_load(
            (CALIBRATION / f"intrinsics/{camera_name}.yaml").read_text(encoding="utf-8")
        )
        assert document["camera_name"] == camera_name
        assert document["frame_id"] == frame_id
        assert (document["image_width"], document["image_height"]) == (640, 480)


def test_runtime_configuration_enables_all_five_camera_streams():
    config = yaml.safe_load(
        (Path(__file__).parents[1] / "config/camera.yaml").read_text(encoding="utf-8")
    )["alohamini_camera"]["ros__parameters"]

    assert config["camera_names"] == [
        "forward",
        "backward",
        "chest",
        "wrist_left",
        "wrist_right",
    ]
    assert config["timestamp_mode"] == "receipt"
    assert config["backward"]["frame_id"] == "backward_camera_optical"
    assert config["chest"]["frame_id"] == "chest_camera_optical"


def test_hand_eye_profiles_distinguish_moving_and_fixed_cameras():
    right = yaml.safe_load(
        (CALIBRATION / "hand_eye/wrist_right.yaml").read_text(encoding="utf-8")
    )
    forward = yaml.safe_load(
        (CALIBRATION / "hand_eye/forward.yaml").read_text(encoding="utf-8")
    )

    assert right["calibration_type"] == "eye_in_hand"
    assert right["mount_link"] == "right_camera"
    assert forward["calibration_type"] == "eye_to_hand"
    assert forward["mount_link"] == "front_camera"

    backward = yaml.safe_load(
        (CALIBRATION / "hand_eye/backward.yaml").read_text(encoding="utf-8")
    )
    chest = yaml.safe_load(
        (CALIBRATION / "hand_eye/chest.yaml").read_text(encoding="utf-8")
    )
    assert backward["calibration_type"] == "eye_to_hand"
    assert backward["mount_link"] == "back_camera"
    assert chest["calibration_type"] == "eye_to_hand"
    assert chest["mount_link"] == "chest_camera"


def test_manual_extrinsic_is_never_treated_as_accepted_hand_eye():
    document = yaml.safe_load(
        (CALIBRATION / "extrinsics/wrist_right_manual_candidate.yaml").read_text(
            encoding="utf-8"
        )
    )

    assert not str(document["status"]).startswith("accepted_")
