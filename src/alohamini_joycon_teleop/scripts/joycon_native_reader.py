#!/usr/bin/env python3
"""Read paired Joy-Cons and publish input-only JSON over localhost ZMQ.

This process deliberately imports neither LeRobot nor ROS. It never connects to
the robot Host and cannot send a robot command.
"""

from __future__ import annotations

import argparse
import json
import signal
import time

import zmq


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="tcp://127.0.0.1:5567")
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument(
        "--sides", nargs="+", choices=("left", "right"), default=("left", "right")
    )
    parser.add_argument(
        "--skip-imu-calibration",
        action="store_true",
        help="Do not run the native two-second stationary IMU calibration.",
    )
    return parser.parse_args()


def button_state(controller, side: str) -> dict[str, bool]:
    joycon = controller.joycon
    common = {
        "stick": bool(
            joycon.get_button_l_stick()
            if side == "left"
            else joycon.get_button_r_stick()
        ),
        "shoulder": bool(
            joycon.get_button_l() if side == "left" else joycon.get_button_r()
        ),
        "trigger": bool(
            joycon.get_button_zl() if side == "left" else joycon.get_button_zr()
        ),
        "sl": bool(
            joycon.get_button_left_sl()
            if side == "left"
            else joycon.get_button_right_sl()
        ),
        "sr": bool(
            joycon.get_button_left_sr()
            if side == "left"
            else joycon.get_button_right_sr()
        ),
    }
    if side == "left":
        common.update(
            {
                "menu": bool(joycon.get_button_minus()),
                "relatch": bool(joycon.get_button_capture()),
                "up": bool(joycon.get_button_up()),
                "down": bool(joycon.get_button_down()),
                "left": bool(joycon.get_button_left()),
                "right": bool(joycon.get_button_right()),
            }
        )
    else:
        common.update(
            {
                "menu": bool(joycon.get_button_plus()),
                "relatch": bool(joycon.get_button_home()),
                "up": bool(joycon.get_button_x()),
                "down": bool(joycon.get_button_b()),
                "left": bool(joycon.get_button_y()),
                "right": bool(joycon.get_button_a()),
            }
        )
    return common


def sample(controller, side: str, sequence: int) -> dict:
    stick_vertical, stick_horizontal, _ = controller.get_stick()
    posture, _, _ = controller.get_control()
    return {
        "schema_version": 1,
        "side": side,
        "sequence": sequence,
        "monotonic_ns": time.monotonic_ns(),
        "stick": [float(stick_horizontal), float(stick_vertical)],
        "orientation_rpy": [float(value) for value in posture[3:6]],
        "buttons": button_state(controller, side),
    }


def main() -> int:
    args = parse_args()
    if args.rate_hz <= 0.0:
        raise ValueError("rate-hz must be positive")
    from joyconrobotics import JoyconRobotics

    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.setsockopt(zmq.LINGER, 0)
    publisher.bind(args.endpoint)
    controllers = {}
    stop = False

    def request_stop(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        for side in args.sides:
            print(
                f"[{side}] opening Joy-Con; native gyro-bias calibration takes "
                "about 2 s. Keep the controller STILL in any orientation "
                "(flat on the desk is fine). The calibration pose does not "
                "set the robot gripper pose: TCP orientation is controlled "
                "relative to the latch point of that controller.",
                flush=True,
            )
            controllers[side] = JoyconRobotics(
                device=side,
                without_rest_init=args.skip_imu_calibration,
            )
        period = 1.0 / args.rate_hz
        sequence = 0
        signatures = {side: None for side in controllers}
        frozen_since = {side: None for side in controllers}
        reconnect_cooldown = {side: 0.0 for side in controllers}
        stick_log_baseline = {side: None for side in controllers}
        stick_log_time = {side: 0.0 for side in controllers}
        while not stop:
            started = time.monotonic()
            sequence += 1
            for side, controller in controllers.items():
                payload = sample(controller, side, sequence)
                signature = (
                    tuple(payload["stick"]),
                    tuple(payload["orientation_rpy"]),
                    tuple(sorted(payload["buttons"].items())),
                )
                now = time.monotonic()
                # Log raw stick values whenever they move, so stick health is
                # visible directly in the terminal (rate-limited per side).
                baseline = stick_log_baseline[side]
                stick = tuple(payload["stick"])
                if baseline is None:
                    stick_log_baseline[side] = stick
                elif (
                    max(
                        abs(stick[0] - baseline[0]),
                        abs(stick[1] - baseline[1]),
                    )
                    > 500
                    and now - stick_log_time[side] > 1.0
                ):
                    stick_log_time[side] = now
                    stick_log_baseline[side] = stick
                    print(
                        f"[{side}] stick H={stick[0]:.0f} V={stick[1]:.0f}",
                        flush=True,
                    )
                if signature == signatures[side]:
                    if frozen_since[side] is None:
                        frozen_since[side] = now
                    elif (
                        now - frozen_since[side] > 1.5
                        and now - reconnect_cooldown[side] > 5.0
                    ):
                        # The HID read thread can die silently on a Bluetooth
                        # hiccup (the native library has no read-loop error
                        # handling), leaving the input report frozen. Reopen
                        # the device so stale sticks/buttons cannot be sent.
                        reconnect_cooldown[side] = now
                        print(
                            f"[{side}] Joy-Con input report frozen; "
                            "reopening the device",
                            flush=True,
                        )
                        try:
                            controller.disconnnect()
                            controllers[side] = JoyconRobotics(
                                device=side, without_rest_init=True
                            )
                            signatures[side] = None
                            frozen_since[side] = None
                            payload = sample(controllers[side], side, sequence)
                        except Exception as exc:
                            print(
                                f"[{side}] reconnect failed ({exc}); "
                                "will retry",
                                flush=True,
                            )
                else:
                    frozen_since[side] = None
                signatures[side] = signature
                publisher.send_string(
                    json.dumps(payload, separators=(",", ":"))
                )
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    finally:
        for controller in controllers.values():
            try:
                controller.disconnnect()
            except Exception:
                pass
        publisher.close()
        context.term()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
