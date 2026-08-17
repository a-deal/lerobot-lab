"""Compatibility tests for the live harness-to-mapper adapter.

The pure mapper returns one ``JointTarget`` object per joint.  The current
hardware harness still expects three parallel dictionaries.  This test fixes
that temporary boundary in place so the calculation engine can change without
silently changing the rest of the motion harness in the same step.

This is a hardware-free contract test.  It does not connect to serial ports,
enable torque, command movement, or establish physical pose fidelity.
"""

from __future__ import annotations

import unittest
from unittest.mock import patch

from so101_mapping import JointTarget
from so101_pose_fidelity import HOME, JOINTS, calculate_targets


class MappingAdapterTests(unittest.TestCase):
    """Prove delegation while preserving the harness's legacy return shape."""

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
