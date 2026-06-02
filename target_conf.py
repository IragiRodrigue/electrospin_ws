#!/usr/bin/env python3
"""Launcher for the target configuration UI."""

from __future__ import annotations

import runpy
from pathlib import Path


if __name__ == "__main__":
    runpy.run_path(str(Path(__file__).resolve().parent / "tools" / "target_conf.py"), run_name="__main__")
