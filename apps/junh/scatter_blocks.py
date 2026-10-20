#!/usr/bin/env python3
"""Compatibility entry point for the shared MuJoCo block-scattering client."""
from pathlib import Path
import sys

# Keep this repository command usable without sourcing a ROS workspace.
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'sim/piper_mujoco_sim'))
from piper_mujoco_sim.scatter_cli import main  # noqa: E402


if __name__ == '__main__':
    raise SystemExit(main())
