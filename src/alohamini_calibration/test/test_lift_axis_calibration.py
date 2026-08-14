import importlib.util
from importlib.machinery import SourceFileLoader
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts/calibrate_lift_axis"
SPEC = importlib.util.spec_from_loader(
    "calibrate_lift_axis", SourceFileLoader("calibrate_lift_axis", str(SCRIPT))
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_fit_recovers_direction_lead_and_home_mapping():
    samples = [
        {"physical_height_mm": 0.0, "raw_tick": 100, "extended_ticks": 500.0},
        {"physical_height_mm": 131.0, "raw_tick": 100, "extended_ticks": -3596.0},
        {"physical_height_mm": 262.0, "raw_tick": 100, "extended_ticks": -7692.0},
    ]

    result = MODULE.fit_calibration(samples, 4096, -0.3)

    assert result["encoder_direction_sign"] == -1
    assert result["measured_lead_mm_per_revolution"] == pytest.approx(131.0)
    assert result["home_extended_ticks"] == 500.0
    assert result["urdf_q_at_home_m"] == -0.3
    assert result["rms_fit_error_mm"] == pytest.approx(0.0, abs=1e-10)


def test_fit_requires_motion_and_two_points():
    with pytest.raises(ValueError, match="at least two"):
        MODULE.fit_calibration([], 4096, -0.3)
    with pytest.raises(ValueError, match="span enough motion"):
        MODULE.fit_calibration(
            [
                {"physical_height_mm": 0.0, "raw_tick": 1, "extended_ticks": 10.0},
                {"physical_height_mm": 1.0, "raw_tick": 1, "extended_ticks": 10.0},
            ],
            4096,
            -0.3,
        )
