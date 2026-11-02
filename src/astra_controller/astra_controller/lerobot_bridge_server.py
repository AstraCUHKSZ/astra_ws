import argparse
import logging
import pickle
import socketserver
import struct
import threading
import time
from typing import Any

import numpy as np

from astra_controller.astra_controller import AstraController


JOINT_ACTION_DIM = 18
ALLOWED_METHODS = {"connect", "disconnect", "wait_for_reset", "get_observation", "get_action", "send_action"}
JOINT_NAMES = (
    "joint_l1",
    "joint_l2",
    "joint_l3",
    "joint_l4",
    "joint_l5",
    "joint_l6",
    "joint_l7r",
    "joint_r1",
    "joint_r2",
    "joint_r3",
    "joint_r4",
    "joint_r5",
    "joint_r6",
    "joint_r7r",
    "twist_linear",
    "twist_angular",
    "joint_head_pan",
    "joint_head_tilt",
)
CAMERA_NAMES = ("head", "wrist_left", "wrist_right")
OPTIONAL_STATE_DEFAULTS = {
    "twist_linear": 0.0,
    "twist_angular": 0.0,
}
OPTIONAL_COMMAND_DEFAULTS = {
    "twist_linear": 0.0,
    "twist_angular": 0.0,
}


def _recv_exact(sock, size: int) -> bytes:
    chunks = []
    remaining = size
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("Connection closed while receiving data")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def _recv_message(sock) -> Any:
    header = _recv_exact(sock, 4)
    (size,) = struct.unpack("!I", header)
    return pickle.loads(_recv_exact(sock, size))


def _send_message(sock, message: Any) -> None:
    payload = pickle.dumps(message, protocol=pickle.HIGHEST_PROTOCOL)
    sock.sendall(struct.pack("!I", len(payload)) + payload)


def _as_float32(values):
    values = np.asarray(values, dtype=np.float32)
    if not np.isfinite(values).all():
        raise ValueError(f"Non-finite values in bridge payload: {values}")
    return values


def _is_finite_value(value) -> bool:
    if value is None:
        return False
    try:
        return np.isfinite(np.asarray(value, dtype=np.float32)).all()
    except (TypeError, ValueError):
        return False


class AstraLeRobotBridge:
    def __init__(self, space: str = "joint", ready_timeout_s: float = 120.0):
        if space != "joint":
            raise ValueError("Only joint space is supported by the LeRobot bridge.")

        self.controller = AstraController(space=space)
        self.lock = threading.Lock()
        self.ready_timeout_s = ready_timeout_s

    def _fill_optional_defaults(self) -> None:
        for key, value in OPTIONAL_STATE_DEFAULTS.items():
            if self.controller.joint_states.get(key) is None:
                self.controller.joint_states[key] = value
        for key, value in OPTIONAL_COMMAND_DEFAULTS.items():
            if self.controller.joint_commands.get(key) is None:
                self.controller.joint_commands[key] = value

    def _missing_observation_keys(self) -> list[str]:
        self._fill_optional_defaults()
        missing = [f"image:{name}" for name in CAMERA_NAMES if self.controller.images.get(name) is None]
        missing.extend(
            f"state:{name}" for name in JOINT_NAMES if not _is_finite_value(self.controller.joint_states.get(name))
        )
        missing.extend(
            f"state:{name}" for name in ("eef_l", "eef_r", "odom") if not _is_finite_value(self.controller.joint_states.get(name))
        )
        return missing

    def _missing_action_keys(self) -> list[str]:
        self._fill_optional_defaults()
        return [
            f"command:{name}"
            for name in JOINT_NAMES
            if not _is_finite_value(self.controller.joint_commands.get(name))
        ]

    def _wait_until_ready(self, missing_fn, label: str) -> None:
        deadline = time.monotonic() + self.ready_timeout_s
        last_log_t = 0.0
        while True:
            missing = missing_fn()
            if not missing:
                return
            now = time.monotonic()
            if now >= deadline:
                raise TimeoutError(f"Timed out waiting for Astra {label}: missing {missing}")
            if now - last_log_t > 2.0:
                logging.info("Waiting for Astra %s: missing %s", label, missing)
                last_log_t = now
            time.sleep(0.05)

    def connect(self):
        with self.lock:
            self.controller.connect()
        return {"connected": True}

    def disconnect(self):
        with self.lock:
            self.controller.disconnect()
        return {"connected": False}

    def wait_for_reset(self):
        with self.lock:
            self.controller.wait_for_reset()
            self.controller.wait_for_reset()
        return {"reset": True}

    def get_observation(self):
        self._wait_until_ready(self._missing_observation_keys, "observation")
        with self.lock:
            (
                state,
                state_arm_l,
                state_gripper_l,
                state_arm_r,
                state_gripper_r,
                state_base,
                state_eef_l,
                state_eef_r,
                state_odom,
                state_head,
            ) = self.controller.read_present_position()
            images = self.controller.read_cameras()
            done = bool(self.controller.done)
            reset = bool(self.controller.reset)
            rerecord = bool(self.controller.rerecord)
            self.controller.done = False
            self.controller.rerecord = False

        return {
            "state": _as_float32(state),
            "state_arm_l": _as_float32(state_arm_l),
            "state_gripper_l": _as_float32(state_gripper_l),
            "state_arm_r": _as_float32(state_arm_r),
            "state_gripper_r": _as_float32(state_gripper_r),
            "state_base": _as_float32(state_base),
            "state_eef_l": _as_float32(state_eef_l),
            "state_eef_r": _as_float32(state_eef_r),
            "state_odom": _as_float32(state_odom),
            "state_head": _as_float32(state_head),
            "images": images,
            "done": done,
            "reset": reset,
            "rerecord": rerecord,
        }

    def get_action(self):
        self._wait_until_ready(self._missing_action_keys, "action")
        with self.lock:
            (
                action,
                action_arm_l,
                action_gripper_l,
                action_arm_r,
                action_gripper_r,
                action_base,
                action_eef_l,
                action_eef_r,
                action_head,
            ) = self.controller.read_leader_present_position()

        return {
            "action": _as_float32(action),
            "action_arm_l": _as_float32(action_arm_l),
            "action_gripper_l": _as_float32(action_gripper_l),
            "action_arm_r": _as_float32(action_arm_r),
            "action_gripper_r": _as_float32(action_gripper_r),
            "action_base": _as_float32(action_base),
            "action_eef_l": _as_float32(action_eef_l),
            "action_eef_r": _as_float32(action_eef_r),
            "action_head": _as_float32(action_head),
        }

    def send_action(self, action):
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.shape != (JOINT_ACTION_DIM,):
            raise ValueError(f"Expected {JOINT_ACTION_DIM} joint actions, got shape {action.shape}.")
        if not np.isfinite(action).all():
            raise ValueError("Action contains NaN or Inf.")

        with self.lock:
            self.controller.write_goal_position(action.tolist())

        return {"action": action}


class BridgeRequestHandler(socketserver.BaseRequestHandler):
    def handle(self):
        bridge: AstraLeRobotBridge = self.server.bridge
        while True:
            try:
                request = _recv_message(self.request)
            except ConnectionError:
                return

            try:
                method = request["method"]
                params = request.get("params", {})
                if method not in ALLOWED_METHODS:
                    raise ValueError(f"Unknown bridge method: {method}")
                result = getattr(bridge, method)(**params)
                response = {"ok": True, "result": result}
            except Exception as exc:
                logging.exception("Astra LeRobot bridge request failed")
                response = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}

            _send_message(self.request, response)


class ThreadingTCPServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True

    def __init__(self, server_address, request_handler_class, bridge):
        super().__init__(server_address, request_handler_class)
        self.bridge = bridge


def main():
    parser = argparse.ArgumentParser(description="Astra ROS bridge for LeRobot Python 3.12 clients.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--space", default="joint", choices=["joint"])
    parser.add_argument("--ready-timeout-s", type=float, default=120.0)
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    bridge = AstraLeRobotBridge(space=args.space, ready_timeout_s=args.ready_timeout_s)
    server = ThreadingTCPServer((args.host, args.port), BridgeRequestHandler, bridge)
    logging.info("Astra LeRobot bridge listening on %s:%s", args.host, args.port)
    server.serve_forever()


if __name__ == "__main__":
    main()
