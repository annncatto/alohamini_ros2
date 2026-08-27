import json

import pytest

from alohamini_camera.camera_info import (
    load_camera_info_document,
    validate_frame_against_camera_info,
)
from alohamini_camera.protocol import parse_camera_message


def message(camera="forward", sequence=1):
    metadata = {
        "schema_version": 1,
        "camera_name": camera,
        "sequence": sequence,
        "encoding": "jpeg",
        "width": 640,
        "height": 480,
        "capture_monotonic_s": 1.5,
        "capture_unix_ns": 2_000_000_000,
    }
    return [
        f"camera/{camera}".encode(),
        json.dumps(metadata).encode(),
        b"\xff\xd8jpeg",
    ]


def test_camera_protocol_parses_timestamped_jpeg():
    frame = parse_camera_message(message(sequence=4))

    assert frame.camera_name == "forward"
    assert frame.sequence == 4
    assert frame.capture_unix_ns == 2_000_000_000
    assert frame.jpeg == b"\xff\xd8jpeg"


@pytest.mark.parametrize(
    "mutate",
    [
        lambda parts: parts[:2],
        lambda parts: [b"camera/other", *parts[1:]],
        lambda parts: [parts[0], b"{}", parts[2]],
        lambda parts: [parts[0], parts[1], b"not-jpeg"],
    ],
)
def test_camera_protocol_rejects_malformed_messages(mutate):
    with pytest.raises(ValueError):
        parse_camera_message(mutate(message()))


def test_packaged_forward_camera_info_matches_stream():
    path = (
        __import__("pathlib").Path(__file__).parents[2]
        / "alohamini_calibration/config/cameras/intrinsics/forward.yaml"
    )
    document = load_camera_info_document(path)

    validate_frame_against_camera_info(document, "forward", 640, 480)
    with pytest.raises(ValueError):
        validate_frame_against_camera_info(document, "forward", 1280, 720)
