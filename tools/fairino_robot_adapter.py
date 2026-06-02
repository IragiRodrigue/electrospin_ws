#!/usr/bin/env python3
"""Fairino robot adapter aligned with the official Python RPC API."""

from __future__ import annotations

from typing import Any, Optional, Sequence, Tuple


def _extract_pose(value: Any) -> Optional[Tuple[float, float, float, float, float, float]]:
    if value is None:
        return None
    if isinstance(value, (list, tuple)):
        if len(value) == 6 and all(isinstance(v, (int, float)) for v in value):
            return tuple(float(v) for v in value)
        for item in value:
            pose = _extract_pose(item)
            if pose is not None:
                return pose
    return None


class FairinoRobotAdapter:
    """Adapter around the official Fairino Python RPC SDK."""

    def __init__(self, robot_ip: str, speed: int = 15, tool_num: int = 0, user_num: int = 0) -> None:
        self.robot_ip = robot_ip
        self.speed = int(speed)
        self.tool_num = int(tool_num)
        self.user_num = int(user_num)
        self.robot = self._connect(robot_ip)

    @staticmethod
    def _connect(robot_ip: str) -> Any:
        errors = []

        try:
            from fairino import Robot as FairinoRobot

            if hasattr(FairinoRobot, "RPC"):
                return FairinoRobot.RPC(robot_ip)
        except Exception as exc:  # pragma: no cover - environment specific
            errors.append(f"fairino.Robot: {exc}")

        try:
            import fairino  # type: ignore

            if hasattr(fairino, "Robot") and hasattr(fairino.Robot, "RPC"):
                return fairino.Robot.RPC(robot_ip)
        except Exception as exc:  # pragma: no cover - environment specific
            errors.append(f"fairino module: {exc}")

        try:
            import frrpc  # type: ignore

            if hasattr(frrpc, "RPC"):
                return frrpc.RPC(robot_ip)
        except Exception as exc:  # pragma: no cover - environment specific
            errors.append(f"frrpc module: {exc}")

        try:
            import Robot  # type: ignore

            if hasattr(Robot, "RPC"):
                return Robot.RPC(robot_ip)
        except Exception as exc:  # pragma: no cover - environment specific
            errors.append(f"Robot module: {exc}")

        raise RuntimeError(
            "Unable to import/connect Fairino SDK. Tried fairino.Robot.RPC(...), frrpc.RPC(...), and Robot.RPC(...). "
            f"Details: {' | '.join(errors) if errors else 'no SDK found'}"
        )

    @staticmethod
    def _split_ret(result: Any) -> Tuple[int, Any]:
        if isinstance(result, (list, tuple)) and len(result) >= 1:
            err = int(result[0])
            if len(result) == 1:
                return err, None
            if len(result) == 2:
                return err, result[1]
            return err, list(result[1:])
        if isinstance(result, (int, float)):
            return int(result), None
        return -1, result

    def get_coords(self) -> Optional[Tuple[float, float, float, float, float, float]]:
        method_names = ["GetActualTCPPose", "GetActualToolFlangePose"]
        for name in method_names:
            method = getattr(self.robot, name, None)
            if callable(method):
                try:
                    result = method()
                    err, data = self._split_ret(result)
                    if err != 0:
                        continue
                    pose = _extract_pose(data)
                    if pose is not None:
                        return pose
                except Exception:
                    continue
        return None

    def send_coords(self, coords: Sequence[float], speed: Optional[int] = None) -> None:
        pose = [float(v) for v in coords]
        vel = int(self.speed if speed is None else speed)
        tool_num = self.tool_num
        user_num = self.user_num

        get_tool_num = getattr(self.robot, "GetActualTCPNum", None)
        if callable(get_tool_num):
            try:
                err, data = self._split_ret(get_tool_num())
                if err == 0 and isinstance(data, (int, float)):
                    tool_num = int(data)
            except Exception:
                pass

        get_user_num = getattr(self.robot, "GetActualWObjNum", None)
        if callable(get_user_num):
            try:
                err, data = self._split_ret(get_user_num())
                if err == 0 and isinstance(data, (int, float)):
                    user_num = int(data)
            except Exception:
                pass

        mode = getattr(self.robot, "Mode", None)
        if callable(mode):
            try:
                mode(0)
            except Exception:
                pass

        attempts = [
            ("MoveCart", (pose, tool_num, user_num, vel, 100.0, 100.0, -1.0, -1)),
            ("MoveCart", (pose, tool_num, user_num, vel, 100.0, 100.0)),
            ("MoveCart", (pose, tool_num, user_num, vel)),
            ("MoveCart", (pose, tool_num, user_num)),
            ("MoveCart", (pose,)),
        ]
        errors = []
        for name, args in attempts:
            method = getattr(self.robot, name, None)
            if not callable(method):
                continue
            try:
                result = method(*args)
                err, data = self._split_ret(result)
                if err != 0:
                    errors.append(f"{name}{args} -> code {err} data {data}")
                    continue
                return
            except TypeError as exc:
                errors.append(f"{name}{args}: {exc}")
            except Exception as exc:
                errors.append(f"{name}{args}: {exc}")
        raise RuntimeError(
            "Could not send Cartesian pose to Fairino robot. "
            f"Tried: {' | '.join(errors) if errors else 'no compatible motion method found'}"
        )

    def close(self) -> None:
        for name in ("CloseRPC", "Disconnect", "close"):
            method = getattr(self.robot, name, None)
            if callable(method):
                try:
                    method()
                except Exception:
                    pass
