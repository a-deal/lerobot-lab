"""Contracts for the pure SO-101 pose-evidence gate.

These tests use ordinary dictionaries and immutable value objects. They never
import the physical workflow, instantiate a robot, read a receipt file, enable
torque, or send a command.

Read the tests in order. The first five define measurement behavior. The last
three define the decision policy. Together they exercise the completed pure
evaluator without connecting to hardware.
"""

from __future__ import annotations

import unittest
from dataclasses import replace

from so101_joint_config import JOINTS
from so101_pose_gate import (
    PoseGateDecision,
    PoseGateMetrics,
    PoseGateThresholds,
    calculate_pose_gate_metrics,
    evaluate_pose_gate,
)

THRESHOLDS = PoseGateThresholds(
    max_final_mae=1.5,
    max_final_joint_error=5.0,
    max_leader_drift=2.0,
    max_home_return_error=5.0,
)


def joint_values(default: float = 0.0, **overrides: float) -> dict[str, float]:
    """Build one complete six-joint fixture without repetitive literals."""

    return {joint: overrides.get(joint, default) for joint in JOINTS}


def unclipped() -> dict[str, bool]:
    """Return a complete fixture in which no proposed target was clipped."""

    return {joint: False for joint in JOINTS}


class PoseGateMetricTests(unittest.TestCase):
    """Contracts for converting raw pose evidence into comparable metrics."""

    def test_absolute_metrics_do_not_let_opposite_signs_cancel(self) -> None:
        """MAE and maxima measure miss magnitude, not signed direction."""

        metrics = calculate_pose_gate_metrics(
            final_signed_error=joint_values(
                shoulder_pan=2.0,
                shoulder_lift=-2.0,
                elbow_flex=1.0,
                wrist_flex=-1.0,
            ),
            leader_end_drift=joint_values(
                shoulder_pan=-0.75,
                shoulder_lift=0.5,
            ),
            home_return_observed=joint_values(
                wrist_roll=-1.25,
                gripper=0.5,
            ),
            home_reference=joint_values(),
            absolute_clipped=unclipped(),
        )

        self.assertAlmostEqual(metrics.final_mae, 1.0)
        self.assertEqual(metrics.max_final_joint_error, 2.0)
        self.assertEqual(
            metrics.max_final_joint_error_joints,
            ("shoulder_lift", "shoulder_pan"),
        )
        self.assertEqual(metrics.max_leader_drift, 0.75)
        self.assertEqual(metrics.max_leader_drift_joints, ("shoulder_pan",))
        self.assertEqual(metrics.max_home_return_error, 1.25)
        self.assertEqual(metrics.max_home_return_error_joints, ("wrist_roll",))
        self.assertEqual(metrics.saturated_joints, ())

    def test_home_return_error_uses_the_nonzero_reference(self) -> None:
        """Home-return error is the gap from home, not the observed position."""

        metrics = calculate_pose_gate_metrics(
            final_signed_error=joint_values(),
            leader_end_drift=joint_values(),
            home_return_observed=joint_values(shoulder_pan=12.0),
            home_reference=joint_values(shoulder_pan=10.0),
            absolute_clipped=unclipped(),
        )

        self.assertEqual(metrics.max_home_return_error, 2.0)
        self.assertEqual(metrics.max_home_return_error_joints, ("shoulder_pan",))

    def test_metric_inputs_require_one_shared_nonempty_joint_roster(self) -> None:
        """A partial receipt must fail instead of producing plausible metrics."""

        incomplete_home = joint_values()
        incomplete_home.pop("gripper")

        with self.assertRaisesRegex(ValueError, "joint roster"):
            calculate_pose_gate_metrics(
                final_signed_error=joint_values(),
                leader_end_drift=joint_values(),
                home_return_observed=incomplete_home,
                home_reference=joint_values(),
                absolute_clipped=unclipped(),
            )

    def test_matching_but_wrong_joint_rosters_are_rejected(self) -> None:
        """Agreement between inputs cannot replace the official robot roster."""

        wrong_joints = (*JOINTS[:-1], "tool")
        wrong_values = {joint: 0.0 for joint in wrong_joints}
        wrong_clipped = {joint: False for joint in wrong_joints}

        with self.assertRaisesRegex(
            ValueError,
            "final_signed_error joint roster mismatch",
        ):
            calculate_pose_gate_metrics(
                final_signed_error=wrong_values,
                leader_end_drift=wrong_values,
                home_return_observed=wrong_values,
                home_reference=wrong_values,
                absolute_clipped=wrong_clipped,
            )

    def test_saturated_joint_names_are_sorted(self) -> None:
        """Stable evidence ordering must not depend on dictionary insertion."""

        clipped = unclipped()
        clipped["wrist_roll"] = True
        clipped["elbow_flex"] = True

        metrics = calculate_pose_gate_metrics(
            final_signed_error=joint_values(),
            leader_end_drift=joint_values(),
            home_return_observed=joint_values(),
            home_reference=joint_values(),
            absolute_clipped=clipped,
        )

        self.assertEqual(metrics.saturated_joints, ("elbow_flex", "wrist_roll"))


class PoseGateDecisionTests(unittest.TestCase):
    """Contracts for applying the predeclared one-pose acceptance policy."""

    def setUp(self) -> None:
        """Start each test from a clean pose exactly at every allowed maximum."""

        self.clean_boundary_metrics = PoseGateMetrics(
            final_mae=THRESHOLDS.max_final_mae,
            max_final_joint_error=THRESHOLDS.max_final_joint_error,
            max_final_joint_error_joints=("shoulder_pan",),
            max_leader_drift=THRESHOLDS.max_leader_drift,
            max_leader_drift_joints=("shoulder_pan",),
            max_home_return_error=THRESHOLDS.max_home_return_error,
            max_home_return_error_joints=("shoulder_pan",),
            saturated_joints=(),
        )

    def test_match_exactly_at_every_threshold_passes(self) -> None:
        """The gate uses inclusive maxima rather than failing equality."""

        decision = evaluate_pose_gate(
            metrics=self.clean_boundary_metrics,
            visual_judgment="match",
            thresholds=THRESHOLDS,
        )

        self.assertEqual(decision, PoseGateDecision(passed=True, failures=()))

    def test_each_numerical_limit_reports_its_stable_failure(self) -> None:
        """Every limit is independently load-bearing and inspectable."""

        cases = (
            ("final_mae", {"final_mae": 1.5001}),
            # The average can pass while one physically important joint does not.
            ("final_joint_error", {"max_final_joint_error": 5.0001}),
            ("leader_drift", {"max_leader_drift": 2.0001}),
            ("home_return_error", {"max_home_return_error": 5.0001}),
        )

        for expected_failure, changes in cases:
            with self.subTest(expected_failure=expected_failure):
                decision = evaluate_pose_gate(
                    metrics=replace(self.clean_boundary_metrics, **changes),
                    visual_judgment="match",
                    thresholds=THRESHOLDS,
                )

                self.assertEqual(
                    decision,
                    PoseGateDecision(passed=False, failures=(expected_failure,)),
                )

    def test_visual_mismatch_and_saturation_report_both_failures(self) -> None:
        """Good numerical tracking cannot rescue bad physical-pose evidence."""

        decision = evaluate_pose_gate(
            metrics=replace(
                self.clean_boundary_metrics,
                final_mae=0.0,
                max_final_joint_error=0.0,
                max_final_joint_error_joints=(),
                max_leader_drift=0.0,
                max_leader_drift_joints=(),
                max_home_return_error=0.0,
                max_home_return_error_joints=(),
                saturated_joints=("gripper",),
            ),
            visual_judgment="mismatch",
            thresholds=THRESHOLDS,
        )

        self.assertEqual(
            decision,
            PoseGateDecision(
                passed=False,
                failures=("visual_judgment", "target_saturation"),
            ),
        )


if __name__ == "__main__":
    unittest.main()
