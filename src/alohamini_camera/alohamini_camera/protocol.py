from __future__ import annotations

import json
import math
from dataclasses import dataclass


CAMERA_STREAM_SCHEMA_VERSION = 1


@dataclass(frozen=True)
class CameraFrame:
    camera_name: str
    sequence: int
    width: int
    height: int
    capture_monotonic_s: float
    capture_unix_ns: int
    jpeg: bytes


def parse_camera_message(parts: list[bytes]) -> CameraFrame:
    """Validate one Host ``[topic, metadata, JPEG]`` message."""
    if len(parts) != 3:
        raise ValueError("camera stream message must have exactly three parts")
    topic, metadata_bytes, jpeg = parts
    try:
        metadata = json.loads(metadata_bytes.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("camera metadata is not valid UTF-8 JSON") from error
    if not isinstance(metadata, dict):
        raise ValueError("camera metadata must be an object")
    if metadata.get("schema_version") != CAMERA_STREAM_SCHEMA_VERSION:
        raise ValueError("unsupported camera stream schema_version")
    camera_name = metadata.get("camera_name")
    if not isinstance(camera_name, str) or not camera_name:
        raise ValueError("camera_name must be a non-empty string")
    if topic != f"camera/{camera_name}".encode():
        raise ValueError("camera topic does not match metadata camera_name")
    if metadata.get("encoding") != "jpeg" or not jpeg.startswith(b"\xff\xd8"):
        raise ValueError("camera payload must be a JPEG image")
    try:
        sequence = int(metadata["sequence"])
        width = int(metadata["width"])
        height = int(metadata["height"])
        capture_monotonic_s = float(metadata["capture_monotonic_s"])
        capture_unix_ns = int(metadata["capture_unix_ns"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("camera metadata has invalid numeric fields") from error
    if sequence < 1 or width < 1 or height < 1:
        raise ValueError("camera sequence and dimensions must be positive")
    if not math.isfinite(capture_monotonic_s) or capture_unix_ns <= 0:
        raise ValueError("camera capture timestamps must be finite and positive")
    return CameraFrame(
        camera_name=camera_name,
        sequence=sequence,
        width=width,
        height=height,
        capture_monotonic_s=capture_monotonic_s,
        capture_unix_ns=capture_unix_ns,
        jpeg=jpeg,
    )
