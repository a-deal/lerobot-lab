"""Hardware-free source of truth for the SO-101 joint roster.

The physical validation workflow and the pure pose evaluator both need the
same joint names and ordering.  Keeping that small contract here lets pure
tests import it without importing LeRobot or touching hardware.

This module owns names and order only.  It does not own calibration, home
positions, motion limits, control policy, or evaluation thresholds.
"""

from __future__ import annotations


JOINTS: tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
