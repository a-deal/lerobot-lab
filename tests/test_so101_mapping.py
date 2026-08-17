"""Contract tests for the hardware-free SO-101 mapping layer.

Explain it like I am ten: each test gives the translator a small worksheet and
checks the answer.  Tests call the same public functions that later command
code will call, but they use ordinary numbers instead of physical robots.

We use Python's built-in ``unittest`` framework because this environment does
not currently include pytest.  That keeps this first pure-logic module free of
a new dependency while still using standard discovery and assertion patterns.
"""

from __future__ import annotations

import math
import unittest

from so101_mapping import (
    JointMapping,
    JointTarget,
    final_pose_error,
    map_pose_relative,
    map_scalar_relative,
)


class RelativeMappingTests(unittest.TestCase):
    """A behavior-focused group of mapping contracts.

    ``unittest`` discovers methods whose names begin with ``test_``.  Each name
    states observable behavior rather than mirroring an implementation detail,
    so the tests can survive a future internal refactor.
    """

    def test_scalar_example_uses_fixed_anchors(self) -> None:
        # Arrange, act, and assert fit on one expression for this tiny example.
        # The expected +5 is the same fixed-anchor calculation reviewed aloud.
        self.assertEqual(
            map_scalar_relative(
                leader_now=35.0,
                leader_start=20.0,
                follower_home=-10.0,
            ),
            5.0,
        )

    def test_pose_mapping_applies_sign_gain_and_offset(self) -> None:
        # Arrange + act: create two deliberately different joint contracts and
        # ask the mapper to process them together.  Named arguments make the
        # roles visible and reduce accidental argument-order bugs.
        result = map_pose_relative(
            leader_now={"pan": 50.0, "gripper": 60.0},
            leader_start={"pan": 20.0, "gripper": 40.0},
            follower_home={"pan": -10.0, "gripper": 30.0},
            configuration={
                "pan": JointMapping(
                    sign=1.0,
                    gain=1.0,
                    offset=0.0,
                    minimum=-80.0,
                    maximum=80.0,
                ),
                "gripper": JointMapping(
                    sign=-1.0,
                    gain=0.5,
                    offset=2.0,
                    minimum=5.0,
                    maximum=95.0,
                ),
            },
        )

        # Assert: inspect the public result, not private implementation steps.
        # One test checks several fields because they describe one behavior:
        # an in-range mapping preserves raw == bounded and is not saturated.
        self.assertEqual(result["pan"].raw, 20.0)
        self.assertEqual(result["pan"].bounded, 20.0)
        self.assertFalse(result["pan"].saturated)
        self.assertEqual(result["gripper"].raw, 22.0)
        self.assertEqual(result["gripper"].bounded, 22.0)
        self.assertFalse(result["gripper"].saturated)

    def test_so101_six_joint_contract_maps_every_joint(self) -> None:
        # The smaller unit fixtures isolate individual behaviors.  This
        # contract uses the real SO-101 roster to prove that one call returns
        # one inspectable target for every physical degree of freedom.
        joints = (
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
            "gripper",
        )
        leader_start = {joint: 0.0 for joint in joints}
        leader_now = {
            joint: float(index * 10)
            for index, joint in enumerate(joints, start=1)
        }
        follower_home = {joint: 0.0 for joint in joints}
        configuration = {
            joint: JointMapping(
                minimum=5.0 if joint == "gripper" else -80.0,
                maximum=95.0 if joint == "gripper" else 80.0,
            )
            for joint in joints
        }

        result = map_pose_relative(
            leader_now=leader_now,
            leader_start=leader_start,
            follower_home=follower_home,
            configuration=configuration,
        )

        self.assertEqual(set(result), set(joints))
        for joint in joints:
            self.assertEqual(result[joint].raw, leader_now[joint])
            self.assertEqual(result[joint].bounded, leader_now[joint])
            self.assertFalse(result[joint].saturated)

    def test_mapping_clamps_and_reports_saturation(self) -> None:
        # This edge case pushes far beyond the permitted interval.  The mapper
        # must preserve the proposed 500 while bounding the usable target at 80.
        result = map_pose_relative(
            leader_now={"pan": 500.0},
            leader_start={"pan": 0.0},
            follower_home={"pan": 0.0},
            configuration={
                "pan": JointMapping(minimum=-80.0, maximum=80.0),
            },
        )

        self.assertEqual(result["pan"].raw, 500.0)
        self.assertEqual(result["pan"].bounded, 80.0)
        self.assertTrue(result["pan"].saturated)

    def test_mapping_rejects_mismatched_joint_sets(self) -> None:
        # A missing anchor is a malformed pose, not a joint we should silently
        # skip.  ``assertRaisesRegex`` checks both failure and useful diagnosis.
        with self.assertRaisesRegex(ValueError, "joint"):
            map_pose_relative(
                leader_now={"pan": 10.0},
                leader_start={},
                follower_home={"pan": 0.0},
                configuration={"pan": JointMapping()},
            )

    def test_mapping_rejects_nonfinite_values(self) -> None:
        # NaN means "not a number."  Allowing it through comparisons or motor
        # math would make later behavior undefined, so rejection is the contract.
        with self.assertRaisesRegex(ValueError, "finite"):
            map_pose_relative(
                leader_now={"pan": math.nan},
                leader_start={"pan": 0.0},
                follower_home={"pan": 0.0},
                configuration={"pan": JointMapping()},
            )

    def test_mapping_rejects_nonfinite_configuration(self) -> None:
        # Pose values are not the only numbers crossing the boundary.  A NaN
        # gain would poison the target calculation just as surely as a NaN
        # encoder reading, so configuration fields must be finite too.
        with self.assertRaisesRegex(ValueError, "finite"):
            map_pose_relative(
                leader_now={"pan": 10.0},
                leader_start={"pan": 0.0},
                follower_home={"pan": 0.0},
                configuration={"pan": JointMapping(gain=math.nan)},
            )

    def test_mapping_rejects_reversed_bounds(self) -> None:
        # A minimum above the maximum describes no valid interval.  Silently
        # passing it to clamp would disguise a broken safety configuration.
        with self.assertRaisesRegex(ValueError, "bounds"):
            map_pose_relative(
                leader_now={"pan": 10.0},
                leader_start={"pan": 0.0},
                follower_home={"pan": 0.0},
                configuration={
                    "pan": JointMapping(minimum=80.0, maximum=-80.0),
                },
            )

    def test_final_pose_error_is_target_minus_measured(self) -> None:
        # This is a small call chain: first produce a target, then compare that
        # target with a simulated follower measurement.  It tests the public
        # handoff between mapping and evaluation without involving commands.
        targets = map_pose_relative(
            leader_now={"pan": 50.0},
            leader_start={"pan": 20.0},
            follower_home={"pan": -10.0},
            configuration={"pan": JointMapping()},
        )

        self.assertEqual(
            final_pose_error(targets=targets, follower_measured={"pan": 17.5}),
            {"pan": 2.5},
        )

    def test_final_pose_error_rejects_mismatched_joint_sets(self) -> None:
        # Evaluation must compare like with like.  A missing measurement cannot
        # silently become a zero-error joint or disappear from the report.
        with self.assertRaisesRegex(ValueError, "joint"):
            final_pose_error(
                targets={
                    "pan": JointTarget(
                        raw=20.0,
                        bounded=20.0,
                        saturated=False,
                    ),
                },
                follower_measured={},
            )

    def test_final_pose_error_rejects_nonfinite_measurement(self) -> None:
        # Infinity is not a physical joint coordinate, so it cannot produce a
        # meaningful fidelity measurement.
        with self.assertRaisesRegex(ValueError, "finite"):
            final_pose_error(
                targets={
                    "pan": JointTarget(
                        raw=20.0,
                        bounded=20.0,
                        saturated=False,
                    ),
                },
                follower_measured={"pan": math.inf},
            )


if __name__ == "__main__":
    # This conventional guard runs the tests when the file is executed directly
    # but does nothing when a test runner imports the module for discovery.
    unittest.main()
