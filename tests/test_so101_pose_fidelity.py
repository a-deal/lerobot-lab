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
The third test protects the first migrated consumer: ``build_pose_preview``
must expose every part of each named receipt before motion is authorized.
The fourth test protects the receipt-to-CSV vocabulary. The fifth proves that
the cycle logger passes every joint through that vocabulary rather than
reconstructing three unrelated target dictionaries.

``patch`` temporarily substitutes a controllable mapper result. This isolates
adapter wiring from mapping arithmetic. These tests never connect serial
ports, enable torque, command movement, or establish physical pose fidelity.

Read each test as arrange, act, assert: prepare snapshots and receipts, call
the public harness function, then verify both the mapper call and returned
contract.
"""

from __future__ import annotations

from typing import Literal
import unittest
from unittest.mock import Mock, patch

from so101_mapping import JointTarget
from so101_pose_fidelity import (
    HOME,
    JOINTS,
    build_pose_preview,
    calculate_joint_targets,
    calculate_targets,
    target_log_fields,
    write_cycle_rows,
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

    def test_pose_preview_reads_named_joint_target_fields(self) -> None:
        """The no-motion receipt exposes proposals, limits, and saturation."""

        leader_baseline = {joint: 0.0 for joint in JOINTS}
        leader_captured = {
            joint: float(index * 10)
            for index, joint in enumerate[Literal['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']](JOINTS, start=1)
        }
        targets = {
            joint: JointTarget(
                raw=100.0 if joint == "gripper" else float(index),
                bounded=95.0 if joint == "gripper" else float(index),
                saturated=(joint == "gripper"),
            )
            for index, joint in enumerate(JOINTS, start=1)
        }

        preview = build_pose_preview(
            pose_name="far",
            leader_captured=leader_captured,
            leader_baseline=leader_baseline,
            targets=targets,
        )

        self.assertEqual(preview["stage"], "pose_preview_no_motion")
        self.assertEqual(preview["pose"], "far")
        self.assertEqual(preview["leader_delta"], leader_captured)
        self.assertEqual(
            preview["raw_target"],
            {joint: target.raw for joint, target in targets.items()},
        )
        self.assertEqual(
            preview["bounded_target"],
            {joint: target.bounded for joint, target in targets.items()},
        )
        self.assertEqual(
            preview["absolute_clipped"],
            {joint: target.saturated for joint, target in targets.items()},
        )

    def test_target_log_fields_preserves_one_receipt(self) -> None:
        """CSV translation preserves both numbers and makes clipping binary."""

        fields = target_log_fields(
            JointTarget(raw=100.0, bounded=80.0, saturated=True)
        )

        self.assertEqual(
            fields,
            {
                "raw_target": 100.0,
                "bounded_target": 80.0,
                "absolute_clipped": 1,
            },
        )

    def test_cycle_logger_reads_target_receipts(self) -> None:
        """Each logged joint gets its target evidence from one named receipt."""

        writer = Mock()
        zeros = {joint: 0.0 for joint in JOINTS}
        false_by_joint = {joint: False for joint in JOINTS}
        targets = {
            joint: JointTarget(
                raw=100.0 if joint == "gripper" else float(index),
                bounded=95.0 if joint == "gripper" else float(index),
                saturated=(joint == "gripper"),
            )
            for index, joint in enumerate(JOINTS, start=1)
        }

        write_cycle_rows(
            writer,
            process_start=0.0,
            pose_name="far",
            phase="approach",
            cycle=1,
            leader_baseline=zeros,
            leader_captured=zeros,
            leader_live=zeros,
            joint_targets=targets,
            commanded=zeros,
            rate_limited=false_by_joint,
            follower_measured=zeros,
            cycle_duration_s=0.02,
        )

        self.assertEqual(writer.writerow.call_count, len(JOINTS))
        rows = [call.args[0] for call in writer.writerow.call_args_list]
        gripper_row = next(row for row in rows if row["joint"] == "gripper")
        self.assertEqual(gripper_row["raw_target"], 100.0)
        self.assertEqual(gripper_row["bounded_target"], 95.0)
        self.assertEqual(gripper_row["absolute_clipped"], 1)


if __name__ == "__main__":
    unittest.main()
