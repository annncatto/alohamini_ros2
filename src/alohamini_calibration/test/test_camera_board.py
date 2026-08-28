from pathlib import Path
import sys

import pytest


cv2 = pytest.importorskip("cv2")
if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "CharucoDetector"):
    pytest.skip("OpenCV 4.7+ with aruco is required", allow_module_level=True)


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))

from camera_board import create_charuco_board, detect_board, load_board  # noqa: E402


BOARD_PATH = PACKAGE / "config/cameras/boards/charuco_9x7_26mm_18p7_ids300_330.yaml"


def test_standard_a4_charuco_geometry_and_detection():
    config = load_board(BOARD_PATH)
    board = create_charuco_board(config)

    assert board.getIds().reshape(-1).tolist() == list(range(300, 331))
    assert board.getChessboardCorners().shape == (48, 3)

    image = board.generateImage((1800, 1400), marginSize=20, borderBits=1)
    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    detection = detect_board(image, config)

    assert detection is not None
    assert detection.point_count == 48
    assert detection.object_points.shape == (48, 1, 3)
    assert detection.image_points.shape == (48, 1, 2)
