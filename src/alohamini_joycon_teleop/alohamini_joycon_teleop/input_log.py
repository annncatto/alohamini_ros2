from __future__ import annotations

import argparse
import json
import signal
import time
from pathlib import Path

import zmq


def record(endpoint: str, path: Path) -> None:
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")
    socket.connect(endpoint)
    stop = False

    def request_stop(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    last_flush = time.monotonic()
    with path.open("a", encoding="utf-8") as stream:
        while not stop:
            if not socket.poll(100):
                continue
            payload = json.loads(socket.recv_string())
            stream.write(json.dumps(payload, separators=(",", ":")) + "\n")
            now = time.monotonic()
            if now - last_flush >= 1.0:
                stream.flush()
                last_flush = now
    socket.close()
    context.term()


def replay(endpoint: str, path: Path, speed: float) -> None:
    if speed <= 0.0:
        raise ValueError("speed must be positive")
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.LINGER, 0)
    socket.bind(endpoint)
    previous_source_ns = None
    time.sleep(0.2)  # PUB/SUB slow-joiner allowance.
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            payload = json.loads(line)
            source_ns = int(payload.get("monotonic_ns", 0))
            if previous_source_ns is not None and source_ns > previous_source_ns:
                time.sleep(min(0.25, (source_ns - previous_source_ns) / 1.0e9 / speed))
            payload["monotonic_ns"] = time.monotonic_ns()
            socket.send_string(json.dumps(payload, separators=(",", ":")))
            previous_source_ns = source_ns
    socket.close()
    context.term()


def main() -> None:
    parser = argparse.ArgumentParser(description="Record or replay timestamped raw Joy-Con ZMQ samples.")
    subparsers = parser.add_subparsers(dest="command", required=True)
    record_parser = subparsers.add_parser("record")
    record_parser.add_argument("path", type=Path)
    record_parser.add_argument("--endpoint", default="tcp://127.0.0.1:5567")
    replay_parser = subparsers.add_parser("replay")
    replay_parser.add_argument("path", type=Path)
    replay_parser.add_argument("--endpoint", default="tcp://127.0.0.1:5567")
    replay_parser.add_argument("--speed", type=float, default=1.0)
    args = parser.parse_args()
    if args.command == "record":
        record(args.endpoint, args.path.expanduser())
    else:
        replay(args.endpoint, args.path.expanduser(), args.speed)


if __name__ == "__main__":
    main()
