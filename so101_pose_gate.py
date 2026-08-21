"""Evaluate one completed SO-101 pose receipt without touching hardware.

Where this module sits
----------------------

``so101_teleoperation_validation`` produces a JSON-compatible record after an
authorized physical pose. This module is the next, hardware-free boundary: it
turns that record's numerical evidence into named metrics and decides whether
those metrics meet a predeclared gate.

The call direction is:

``physical workflow -> pose receipt -> this pure evaluator -> run decision``

This module deliberately does not read JSON or CSV files, import LeRobot,
connect to serial devices, enable torque, send commands, or decide whether a
motion path is collision-safe. A caller supplies ordinary dictionary-like
values that were already extracted from one pose receipt.

Why there are two functions
---------------------------

``calculate_pose_gate_metrics`` answers the measurement question: what were
the average final miss, worst final joint miss, worst leader drift, worst home
return miss, the joints tied for each worst value, and saturated joints?

``evaluate_pose_gate`` answers the policy question: given those measurements,
the operator's visual label, and thresholds declared before the run, which
gate conditions passed or failed?

Keeping measurement separate from policy prevents arithmetic from silently
changing when a threshold changes. It also makes the important limitation
visible: these metrics evaluate target tracking and receipt integrity, not
geometric leader-follower equivalence or physical safety.

Reading order
-------------

1. ``PoseGateThresholds`` names the four predeclared limits.
2. ``PoseGateMetrics`` names the measurements from one pose.
3. ``PoseGateDecision`` carries the final boolean and stable failure reasons.
4. ``calculate_pose_gate_metrics`` is the Andrew-owned measurement logic.
5. ``evaluate_pose_gate`` is the Andrew-owned decision logic.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

from so101_joint_config import JOINTS


@dataclass(frozen=True)
class PoseGateThresholds:
    """Maximum accepted values declared before a physical run."""

    max_final_mae: float
    max_final_joint_error: float
    max_leader_drift: float
    max_home_return_error: float


@dataclass(frozen=True)
class PoseGateMetrics:
    """Hardware-free summary with descriptive, not causal, attribution.

    Each ``*_joints`` tuple names every joint tied for its corresponding
    maximum.  That says where the observed maximum occurred; it does not claim
    why that joint drifted or missed its target.
    """

    final_mae: float
    max_final_joint_error: float
    max_final_joint_error_joints: tuple[str, ...]
    max_leader_drift: float
    max_leader_drift_joints: tuple[str, ...]
    max_home_return_error: float
    max_home_return_error_joints: tuple[str, ...]
    saturated_joints: tuple[str, ...]


@dataclass(frozen=True)
class PoseGateDecision:
    """Whether one pose passed and every reason it did not."""

    passed: bool
    failures: tuple[str, ...]


def _validate_joint_roster(
    *,
    mapping_name: str,
    values: Mapping[str, object],
) -> None:
    actual_joints = set(values)
    expected_joints = set(JOINTS)

    missing_joints = expected_joints - actual_joints
    unexpected_joints = actual_joints - expected_joints

    if missing_joints or unexpected_joints:
        raise ValueError(
            f"{mapping_name} joint roster mismatch: "
            f"missing {sorted(missing_joints)}, "
            f"unexpected {sorted(unexpected_joints)}"
        )


def calculate_pose_gate_metrics(
    *,
    final_signed_error: Mapping[str, float],
    leader_end_drift: Mapping[str, float],
    home_return_observed: Mapping[str, float],
    home_reference: Mapping[str, float],
    absolute_clipped: Mapping[str, bool],
) -> PoseGateMetrics:
    """Calculate the five named measurements from one pose receipt.

    Contract:

    - every input must describe exactly the official six-joint roster;
    - positive and negative errors must not cancel in either the mean or max;
    - home-return error compares the observed return with ``home_reference``;
    - every joint tied for a maximum must be retained in sorted order;
    - saturated joint names must be returned in deterministic sorted order.

    This function calculates evidence only. It does not apply thresholds or
    decide whether the visual comparison passed. Maximum-joint attribution is
    descriptive: causal calibration diagnosis remains a separate workflow.
    """

    for mapping_name, values in (
        ("final_signed_error", final_signed_error),
        ("leader_end_drift", leader_end_drift),
        ("home_return_observed", home_return_observed),
        ("home_reference", home_reference),
        ("absolute_clipped", absolute_clipped),
    ):
        _validate_joint_roster(
            mapping_name=mapping_name,
            values=values,
        )

    final_error_magnitudes = [abs(error) for error in final_signed_error.values()]

    final_mae = sum(final_error_magnitudes) / len(final_error_magnitudes)
    max_final_joint_error = max(final_error_magnitudes)
    max_final_joint_error_joints = tuple(
        sorted(
            joint
            for joint, error in final_signed_error.items()
            if abs(error) == max_final_joint_error
        )
    )

    max_leader_drift = max(abs(drift) for drift in leader_end_drift.values())
    max_leader_drift_joints = tuple(
        sorted(
            joint
            for joint, drift in leader_end_drift.items()
            if abs(drift) == max_leader_drift
        )
    )

    home_return_magnitudes = {
        joint: abs(home_return_observed[joint] - home_reference[joint])
        for joint in JOINTS
    }
    max_home_return_error = max(home_return_magnitudes.values())
    max_home_return_error_joints = tuple(
        sorted(
            joint
            for joint, error in home_return_magnitudes.items()
            if error == max_home_return_error
        )
    )

    saturated_joints = tuple(
        sorted(joint for joint, clipped in absolute_clipped.items() if clipped)
    )

    return PoseGateMetrics(
        final_mae=final_mae,
        max_final_joint_error=max_final_joint_error,
        max_final_joint_error_joints=max_final_joint_error_joints,
        max_leader_drift=max_leader_drift,
        max_leader_drift_joints=max_leader_drift_joints,
        max_home_return_error=max_home_return_error,
        max_home_return_error_joints=max_home_return_error_joints,
        saturated_joints=saturated_joints,
    )


def evaluate_pose_gate(
    *,
    metrics: PoseGateMetrics,
    visual_judgment: str,
    thresholds: PoseGateThresholds,
) -> PoseGateDecision:
    """Apply the predeclared visual, saturation, and numerical gate.

    The workflow's recognized visual labels are ``match``, ``mismatch``, and
    ``unclear``; only ``match`` can pass. Collect every failed condition in
    this stable order so one early failure cannot hide another:

    1. ``visual_judgment``
    2. ``target_saturation``
    3. ``final_mae``
    4. ``final_joint_error``
    5. ``leader_drift``
    6. ``home_return_error``

    A value exactly equal to its maximum is accepted. This function evaluates
    one pose only; a separate workflow-level gate owns the canary and complete
    near/middle/far receipt shapes.
    """

    passed = True
    failures = []

    if visual_judgment != "match":
        passed = False
        failures.append("visual_judgment")

    if metrics.saturated_joints:
        passed = False
        failures.append("target_saturation")

    if metrics.final_mae > thresholds.max_final_mae:
        passed = False
        failures.append("final_mae")

    if metrics.max_final_joint_error > thresholds.max_final_joint_error:
        passed = False
        failures.append("final_joint_error")

    if metrics.max_leader_drift > thresholds.max_leader_drift:
        passed = False
        failures.append("leader_drift")

    if metrics.max_home_return_error > thresholds.max_home_return_error:
        passed = False
        failures.append("home_return_error")

    return PoseGateDecision(
        passed=passed,
        failures=tuple(failures),
    )
