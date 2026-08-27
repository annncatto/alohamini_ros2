from pathlib import Path

import yaml


CALIBRATION = Path(__file__).parents[2] / "alohamini_calibration/config/cameras"


def test_runtime_camera_names_have_consistent_intrinsic_frames():
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


def test_manual_extrinsic_is_never_treated_as_accepted_hand_eye():
    document = yaml.safe_load(
        (CALIBRATION / "extrinsics/wrist_right_manual_candidate.yaml").read_text(
            encoding="utf-8"
        )
    )

    assert not str(document["status"]).startswith("accepted_")
