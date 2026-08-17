"""Test the clean and legacy target interfaces without touching hardware.

Where this module sits
----------------------

``test_so101_mapping`` proves the translator's numerical rules in isolation.
This module checks the next handoff: whether ``so101_pose_fidelity`` calls that
translator with the correct leader snapshots, follower anchor, and six-joint
configuration.

The first test protects the new interface: ``calculate_joint_targets`` must
return the mapper's ``JointTarget`` dictionary without reshaping it. The second
test protects the temporary legacy interface: ``calculate_targets`` must
unpack those receipts into ``raw``, ``bounded``, and ``clipped`` dictionaries.

``patch`` temporarily substitutes a controllable mapper result. This isolates
adapter wiring from mapping arithmetic. These tests never connect serial
ports, enable torque, command movement, or establish physical pose fidelity.

Read each test as arrange, act, assert: prepare snapshots and receipts, call
the public harness function, then verify both the mapper call and returned
contract.
"""

from __future__ import annotations

import unittest
from unittest.mock import patch

from so101_mapping import JointTarget
from so101_pose_fidelity import (
    HOME,
    JOINTS,
    calculate_joint_targets,
    calculate_targets,
)


class MappingAdapterTests(unittest.TestCase):
    """Prove delegation while preserving the harness's legacy return shape."""

    def test_clean_entrypoint_returns_mapper_joint_targets(self) -> None:
        """The new API returns the mapper's named receipts without reshaping."""

        leader_baseline = {joint: 0.0 for joint in JOINTS}
        leader_captured = {joint: 10.0 for joint in JOINTS}
        mapped_targets = {
            joint: JointTarget(
                raw=float(index),
                bounded=float(index),
                saturated=False,
            )
            for index, joint in enumerate(JOINTS, start=1)
        }

        with patch(
            "so101_pose_fidelity.map_pose_relative",
            return_value=mapped_targets,
        ) as mapper:
            result = calculate_joint_targets(
                leader_captured,
                leader_baseline,
            )

        self.assertIs(result, mapped_targets)
        mapper.assert_called_once()
        call = mapper.call_args.kwargs
        self.assertEqual(call["leader_now"], leader_captured)
        self.assertEqual(call["leader_start"], leader_baseline)
        self.assertEqual(call["follower_home"], HOME)
        self.assertEqual(set(call["configuration"]), set(JOINTS))

    def test_calculate_targets_delegates_and_unpacks_joint_targets(self) -> None:
        leader_baseline = {joint: 0.0 for joint in JOINTS}
        leader_captured = {joint: 10.0 for joint in JOINTS}
        mapped_targets = {}
        for index, joint in enumerate(JOINTS, start=1):
            raw = 100.0 if joint == "gripper" else float(index)
            bounded = 95.0 if joint == "gripper" else raw
            mapped_targets[joint] = JointTarget(
                raw=raw,
                bounded=bounded,
                saturated=(joint == "gripper"),
            )

        # ``patch`` replaces only the pure mapper call.  If the harness keeps
        # doing its own arithmetic, this assertion fails and exposes the
        # duplicate implementation we are trying to remove.
        with patch(
            "so101_pose_fidelity.map_pose_relative",
            return_value=mapped_targets,
        ) as mapper:
            raw, bounded, clipped = calculate_targets(
                leader_captured,
                leader_baseline,
            )

        mapper.assert_called_once()
        call = mapper.call_args.kwargs
        self.assertEqual(call["leader_now"], leader_captured)
        self.assertEqual(call["leader_start"], leader_baseline)
        self.assertEqual(call["follower_home"], HOME)
        self.assertEqual(set(call["configuration"]), set(JOINTS))

        self.assertEqual(
            raw,
            {joint: target.raw for joint, target in mapped_targets.items()},
        )
        self.assertEqual(
            bounded,
            {joint: target.bounded for joint, target in mapped_targets.items()},
        )
        self.assertEqual(
            clipped,
            {joint: target.saturated for joint, target in mapped_targets.items()},
        )


if __name__ == "__main__":
    unittest.main()
