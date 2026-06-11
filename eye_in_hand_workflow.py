#!/usr/bin/env python3
"""Launcher for the eye-in-hand workflow assistant."""

from __future__ import annotations

import runpy
from pathlib import Path


if __name__ == "__main__":
    script_path = Path(__file__).resolve().parent / "tools" / "eye_in_hand_workflow.py"
    runpy.run_path(str(script_path), run_name="__main__")
