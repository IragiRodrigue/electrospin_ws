#!/usr/bin/env python3
"""Launcher for the deterministic target configuration UI."""

from __future__ import annotations

import runpy
from pathlib import Path


if __name__ == "__main__":
    script_path = Path(__file__).resolve().parent / "tools" / "target_conf_deterministic.py"
    runpy.run_path(str(script_path), run_name="__main__")
