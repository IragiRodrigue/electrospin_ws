#!/usr/bin/env python3
"""Best-effort Fairino robot adapter for Cartesian pose control."""

from __future__ import annotations

from typing import Any, Iterable, Optional, Sequence, Tuple


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
    """Adapter around common Fairino Python SDK layouts."""

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
            import Robot  # type: ignore

            if hasattr(Robot, "RPC"):
                return Robot.RPC(robot_ip)
        except Exception as exc:  # pragma: no cover - environment specific
            errors.append(f"Robot module: {exc}")

        raise RuntimeError(
            "Unable to import/connect Fairino SDK. Tried fairino.Robot.RPC(...) and Robot.RPC(...). "
            f"Details: {' | '.join(errors) if errors else 'no SDK found'}"
        )

    def get_coords(self) -> Optional[Tuple[float, float, float, float, float, float]]:
        method_names = [
            "GetActualTCPPose",
            "GetActualToolFlangePose",
            "GetActualTCPNumPose",
            "GetCurrentTCPPose",
            "GetPose",
        ]
        for name in method_names:
            method = getattr(self.robot, name, None)
            if callable(method):
                try:
                    result = method()
                    pose = _extract_pose(result)
                    if pose is not None:
                        return pose
                except Exception:
                    continue
        return None

    def send_coords(self, coords: Sequence[float], speed: Optional[int] = None) -> None:
        pose = [float(v) for v in coords]
        vel = int(self.speed if speed is None else speed)
        attempts = [
            ("MoveCart", (pose, self.tool_num, self.user_num, vel)),
            ("MoveCart", (pose, self.tool_num, self.user_num)),
            ("MoveCart", (pose,)),
            ("MoveL", (pose, self.tool_num, self.user_num, vel)),
            ("MoveL", (pose, self.tool_num, self.user_num)),
            ("MoveL", (pose,)),
        ]
        errors = []
        for name, args in attempts:
            method = getattr(self.robot, name, None)
            if not callable(method):
                continue
            try:
                result = method(*args)
                if isinstance(result, (int, float)) and int(result) not in (0,):
                    errors.append(f"{name}{args} -> code {result}")
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

