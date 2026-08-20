"""Test the clean target handoffs without touching hardware.

Where this module sits
----------------------

``test_so101_mapping`` proves the translator's numerical rules in isolation.
This module checks the next handoff: whether the teleoperation runner calls that
translator with the correct leader snapshots, follower anchor, and six-joint
configuration.

The first test protects the mapping entrypoint: ``calculate_joint_targets``
must return the mapper's ``JointTarget`` dictionary without reshaping it. The
second test protects the first downstream consumer: ``build_pose_preview``
must expose every part of each named receipt before motion is authorized.
The third test protects the receipt-to-CSV vocabulary. The fourth proves that
the cycle logger passes every joint through that vocabulary rather than
reconstructing three unrelated target dictionaries. The fifth protects the
post-movement summary assembly that a unit test missed during the first live
run.

``patch`` temporarily substitutes a controllable mapper result. This isolates
adapter wiring from mapping arithmetic. These tests never connect serial
ports, enable torque, command movement, or establish physical pose fidelity.

Read each test as arrange, act, assert: prepare snapshots and receipts, call
the public harness function, then verify both the mapper call and returned
contract.
"""

import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from typing import Any, Literal
from unittest.mock import Mock, call, patch

from so101_lifecycle import ArmLifecycle
from so101_mapping import JointTarget
from so101_teleoperation_validation import (
    HOME,
    JOINTS,
    POSE_NAMES,
    UNPOWERED_REST,
    OperationalHomeSnapshot,
    PreflightSnapshot,
    UserAbort,
    approach_and_hold,
    build_pose_preview,
    build_pose_result_record,
    calculate_joint_targets,
    cleanup_connected_arms,
    connect_and_validate_arms,
    pose_within_tolerance,
    prepare_follower_at_operational_home,
    record_cleanup_errors,
    run_pose_validation_trial,
    run_teleoperation_validation,
    target_log_fields,
    write_cycle_rows,
)


class MappingAdapterTests(unittest.TestCase):
    """Prove mapper delegation and each hardware-free receipt handoff."""

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
            "so101_teleoperation_validation.map_pose_relative",
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

    def test_pose_preview_reads_named_joint_target_fields(self) -> None:
        """The no-motion receipt exposes proposals, limits, and saturation."""

        leader_baseline = {joint: 0.0 for joint in JOINTS}
        leader_captured = {
            joint: float(index * 10)
            for index, joint in enumerate[
                Literal[
                    "shoulder_pan",
                    "shoulder_lift",
                    "elbow_flex",
                    "wrist_flex",
                    "wrist_roll",
                    "gripper",
                ]
            ](JOINTS, start=1)
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

        fields = target_log_fields(JointTarget(raw=100.0, bounded=80.0, saturated=True))

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

    def test_pose_result_record_reads_target_receipts(self) -> None:
        """Post-movement summary uses receipts instead of deleted variables."""

        leader_captured = {joint: float(index) for index, joint in enumerate(JOINTS)}
        targets = {
            joint: JointTarget(
                raw=100.0 if joint == "gripper" else float(index),
                bounded=95.0 if joint == "gripper" else float(index),
                saturated=(joint == "gripper"),
            )
            for index, joint in enumerate(JOINTS, start=1)
        }

        record = build_pose_result_record(
            pose_name="near",
            leader_captured=leader_captured,
            targets=targets,
            pose_result={"cycles": 55},
            visual_judgment="match",
        )

        self.assertEqual(record["cycles"], 55)
        self.assertEqual(record["name"], "near")
        self.assertEqual(record["leader_captured"], leader_captured)
        self.assertEqual(record["raw_targets"]["gripper"], 100.0)
        self.assertEqual(record["bounded_targets"]["gripper"], 95.0)
        self.assertTrue(record["absolute_clipped"]["gripper"])
        self.assertEqual(record["visual_judgment"], "match")

    def test_known_unpowered_rest_requires_every_joint_near_reference(self) -> None:
        """Startup recognition accepts the complete rest pose, not a partial match."""

        self.assertTrue(
            pose_within_tolerance(
                dict(UNPOWERED_REST),
                UNPOWERED_REST,
                tolerance=8.0,
            )
        )
        moved = dict(UNPOWERED_REST)
        moved["elbow_flex"] -= 20.0
        self.assertFalse(pose_within_tolerance(moved, UNPOWERED_REST, tolerance=8.0))

    def test_goal_alignment_precedes_torque_enable(self) -> None:
        """Neither arm may enable torque while stale goal positions remain."""

        bus = Mock()
        arm = ArmLifecycle(name="leader", bus=bus, joints=JOINTS)
        pose = {joint: float(index) for index, joint in enumerate(JOINTS)}

        arm.align_goals_and_enable_torque(pose)

        self.assertEqual(
            bus.method_calls,
            [
                call.sync_write("Goal_Position", pose),
                call.enable_torque(list(JOINTS)),
            ],
        )

    def test_pose_within_tolerance_rejects_partial_joint_roster(self) -> None:
        """A partial match is not a valid rest pose."""

        partial_pose = dict(UNPOWERED_REST)

        partial_pose.pop("gripper")

        self.assertFalse(
            pose_within_tolerance(
                partial_pose,
                UNPOWERED_REST,
                tolerance=8.0,
            )
        )

        self.assertFalse(
            pose_within_tolerance(
                partial_pose,
                partial_pose,
                tolerance=8.0,
            )
        )

    def test_cleanup_errors_mark_completed_receipt_failed(self) -> None:
        """A cleanup failure prevents a completed final status."""

        receipt = {
            "status": "completed",
        }
        cleanup_errors = ["leader torque failure"]
        record_cleanup_errors(receipt, cleanup_errors)
        self.assertEqual(receipt["status"], "cleanup_failed")
        self.assertEqual(receipt["cleanup_errors"], cleanup_errors)

    def test_cleanup_errors_preserve_existing_failure(self) -> None:
        "Cleanup evidence does not overwrite the primary experiment failure."
        receipt = {
            "status": "failed",
            "error": "RuntimeError: movement failed",
        }

        cleanup_errors = ["follower disconnect failure"]

        record_cleanup_errors(receipt, cleanup_errors)
        self.assertEqual(receipt["status"], "failed")
        self.assertEqual(receipt["error"], "RuntimeError: movement failed")
        self.assertEqual(receipt["cleanup_errors"], cleanup_errors)

    def test_final_pose_reads_each_arm_once(self) -> None:
        """Final evidence uses one complete snapshot from each arm."""

        leader = Mock()
        follower = Mock()

        commanded = {joint: 0.0 for joint in JOINTS}
        leader_baseline = {joint: 0.0 for joint in JOINTS}
        leader_captured = {joint: 10.0 for joint in JOINTS}
        follower_final = {joint: 1.0 for joint in JOINTS}
        leader_final = {joint: 12.0 for joint in JOINTS}
        targets = {
            joint: JointTarget(raw=0.0, bounded=0.0, saturated=False)
            for joint in JOINTS
        }

        def fake_read_positions(bus: object) -> dict[str, float]:
            if bus is follower.bus:
                return follower_final
            if bus is leader.bus:
                return leader_final
            self.fail("unexpected bus read")

        with patch(
            "so101_teleoperation_validation.read_positions",
            side_effect=fake_read_positions,
        ) as read_positions_mock:
            _, result = approach_and_hold(
                follower=follower,
                leader=leader,
                writer=Mock(),
                process_start=0.0,
                pose_name="near",
                commanded=commanded,
                leader_baseline=leader_baseline,
                leader_captured=leader_captured,
                joint_targets=targets,
                period_s=0.1,
                max_step=2.0,
                hold_s=0.0,
            )

        self.assertEqual(
            read_positions_mock.call_args_list,
            [call(follower.bus), call(leader.bus)],
        )

        self.assertEqual(result["final_measured"], follower_final)
        self.assertEqual(result["leader_final_measured"], leader_final)
        self.assertEqual(
            result["leader_end_drift"],
            {joint: leader_final[joint] - leader_captured[joint] for joint in JOINTS},
        )

    def test_successful_torque_disable_clears_cleanup_obligation(self) -> None:
        """A successful torque-disable clears the torque-off obligation."""

        bus = Mock()
        arm = ArmLifecycle(name="leader", bus=bus, joints=JOINTS)
        pose = {joint: float(index) for index, joint in enumerate(JOINTS)}

        arm.align_goals_and_enable_torque(pose)
        arm.disable_torque_if_required()

        bus.disable_torque.assert_called_once_with(list(JOINTS))
        self.assertFalse(arm.torque_cleanup_required)

    def test_failed_torque_disable_preserves_cleanup_obligation(self) -> None:
        """A failed torque-disable preserves the torque-off obligation."""

        bus = Mock()
        arm = ArmLifecycle(name="leader", bus=bus, joints=JOINTS)
        pose = {joint: float(index) for index, joint in enumerate(JOINTS)}

        arm.align_goals_and_enable_torque(pose)
        bus.disable_torque.side_effect = RuntimeError("partial disable failure")

        with self.assertRaises(RuntimeError):
            arm.disable_torque_if_required()

        self.assertTrue(arm.torque_cleanup_required)

    def test_failed_torque_enable_preserves_cleanup_obligation(self) -> None:
        """A failed torque-enable attempt preserves the torque-off obligation."""

        bus = Mock()
        bus.enable_torque.side_effect = RuntimeError("partial enable failure")
        arm = ArmLifecycle(name="leader", bus=bus, joints=JOINTS)
        pose = {joint: float(index) for index, joint in enumerate(JOINTS)}

        with self.assertRaises(RuntimeError):
            arm.align_goals_and_enable_torque(pose)

        self.assertTrue(arm.torque_cleanup_required)

    def test_cleanup_continues_after_leader_torque_failure(self) -> None:
        """Leader torque failure does not prevent remaining cleanup attempts."""

        leader_bus = Mock()
        follower_bus = Mock()

        leader_bus.is_connected = True
        follower_bus.is_connected = True

        leader_lifecycle = ArmLifecycle(name="leader", bus=leader_bus, joints=JOINTS)
        follower_lifecycle = ArmLifecycle(
            name="follower", bus=follower_bus, joints=JOINTS
        )

        pose = {joint: float(index) for index, joint in enumerate(JOINTS)}
        leader_lifecycle.align_goals_and_enable_torque(pose)
        follower_lifecycle.align_goals_and_enable_torque(pose)

        leader_bus.disable_torque.side_effect = RuntimeError("leader torque failure")

        errors = cleanup_connected_arms(
            leader_lifecycle=leader_lifecycle,
            follower_lifecycle=follower_lifecycle,
        )

        leader_bus.disable_torque.assert_called_once_with(list(JOINTS))
        follower_bus.disable_torque.assert_called_once_with(list(JOINTS))

        leader_bus.disconnect.assert_called_once_with(disable_torque=False)
        follower_bus.disconnect.assert_called_once_with(disable_torque=False)

        self.assertTrue(leader_lifecycle.torque_cleanup_required)
        self.assertFalse(follower_lifecycle.torque_cleanup_required)

        self.assertTrue(
            any("leader" in error for error in errors),
        )

    def test_preflight_rejects_pre_enabled_follower_torque(self) -> None:
        """Preflight refuses follower torque that this run does not own."""
        leader_bus = Mock()
        follower_bus = Mock()

        leader_bus.is_calibrated = True
        follower_bus.is_calibrated = True

        leader_bus.read.return_value = 0
        follower_bus.sync_read.return_value = {joint: 0.0 for joint in JOINTS}
        follower_bus.read.return_value = 1

        leader = ArmLifecycle(name="leader", bus=leader_bus, joints=JOINTS)
        follower = ArmLifecycle(name="follower", bus=follower_bus, joints=JOINTS)

        with self.assertRaisesRegex(RuntimeError, "already enabled"):
            connect_and_validate_arms(
                leader_lifecycle=leader,
                follower_lifecycle=follower,
            )

        follower_bus.disable_torque.assert_not_called()

    def test_follower_startup_decline_sends_no_actuator_commands(self) -> None:
        """Declining startup authorization leaves the follower passive."""

        follower_bus = Mock()
        follower = Mock()

        follower.bus = follower_bus

        follower_lifecycle = ArmLifecycle(
            name="follower", bus=follower_bus, joints=JOINTS
        )

        starting_pose = dict(UNPOWERED_REST)

        with (
            patch("builtins.input", return_value="STOP"),
            self.assertRaisesRegex(UserAbort, "declined recognized-rest startup"),
        ):
            prepare_follower_at_operational_home(
                follower=follower,
                follower_lifecycle=follower_lifecycle,
                starting_pose=starting_pose,
                period_s=0.1,
                max_step=2.0,
            )

        follower_bus.sync_write.assert_not_called()
        follower_bus.enable_torque.assert_not_called()

    def test_pose_trial_quit_before_capture_reads_no_pose_or_commands(self) -> None:
        """Quitting at the capture gate exits before observing or moving either arm."""

        leader_bus = Mock()
        follower_bus = Mock()

        leader = Mock()
        leader.bus = leader_bus

        follower = Mock()
        follower.bus = follower_bus

        leader_lifecycle = ArmLifecycle(name="leader", bus=leader_bus, joints=JOINTS)
        leader_baseline = {joint: 0.0 for joint in JOINTS}
        pose_results: list[dict[str, object]] = []

        with (
            patch("builtins.input", return_value="QUIT"),
            patch("so101_teleoperation_validation.read_positions") as read_positions,
            self.assertRaisesRegex(UserAbort, "operator ended before all poses"),
        ):
            run_pose_validation_trial(
                pose_name="near",
                leader=leader,
                follower=follower,
                leader_lifecycle=leader_lifecycle,
                writer=Mock(),
                flush_cycle_evidence=Mock(),
                process_start=0.0,
                commanded=dict(HOME),
                leader_baseline=leader_baseline,
                pose_results=pose_results,
                period_s=0.1,
                max_step=2.0,
                hold_s=0.0,
            )

        read_positions.assert_not_called()
        leader_bus.sync_write.assert_not_called()
        leader_bus.enable_torque.assert_not_called()
        follower_bus.sync_write.assert_not_called()
        self.assertEqual(pose_results, [])

    def test_pose_trial_quit_after_preview_sends_no_actuator_commands(self) -> None:
        """Quitting after a safe preview leaves both arms unchanged."""

        leader_bus = Mock()
        follower_bus = Mock()

        leader = Mock()
        leader.bus = leader_bus

        follower = Mock()
        follower.bus = follower_bus

        leader_lifecycle = ArmLifecycle(name="leader", bus=leader_bus, joints=JOINTS)

        leader_pose = {joint: 0.0 for joint in JOINTS}

        commanded = dict(HOME)

        pose_results: list[dict[str, object]] = []

        with (
            patch("builtins.input", side_effect=["", "QUIT"]),
            patch(
                "so101_teleoperation_validation.read_positions",
                return_value=leader_pose,
            ),
            patch("builtins.print"),
            self.assertRaisesRegex(UserAbort, "operator ended before all poses"),
        ):
            run_pose_validation_trial(
                pose_name="near",
                leader=leader,
                follower=follower,
                leader_lifecycle=leader_lifecycle,
                writer=Mock(),
                flush_cycle_evidence=Mock(),
                process_start=0.0,
                commanded=commanded,
                leader_baseline=leader_pose,
                pose_results=pose_results,
                period_s=0.1,
                max_step=2.0,
                hold_s=0.0,
            )
        leader_bus.sync_write.assert_not_called()
        leader_bus.enable_torque.assert_not_called()
        follower_bus.sync_write.assert_not_called()

        self.assertEqual(pose_results, [])

    def test_happy_path_records_completed_orchestration_phases(self) -> None:
        """The real orchestrator sequences specialists and records each success."""

        follower_present = {joint: 0.0 for joint in JOINTS}
        follower_torque = {joint: 0 for joint in JOINTS}
        leader_baseline = {joint: 10.0 for joint in JOINTS}

        preflight_snapshot = PreflightSnapshot(
            follower_present=follower_present,
            follower_torque=follower_torque,
        )
        home_snapshot = OperationalHomeSnapshot(
            commanded=dict(HOME),
            observed=dict(HOME),
        )

        leader = Mock()
        leader.bus = Mock()
        follower = Mock()
        follower.bus = Mock()

        def complete_trial(**kwargs: Any) -> dict[str, float]:
            """Simulate one successful trial using its real public contract."""

            pose_name = kwargs["pose_name"]
            kwargs["pose_results"].append({"pose": pose_name, "status": "completed"})

            next_commanded = dict(kwargs["commanded"])
            next_commanded[JOINTS[0]] += 1.0
            return next_commanded

        with TemporaryDirectory() as output_dir:
            args = SimpleNamespace(
                self_test=False,
                period_s=0.1,
                max_step=2.0,
                hold_s=0.1,
                output_dir=output_dir,
                leader_port="mock-leader-port",
                follower_port="mock-follower-port",
                leader_id="mock-leader",
                follower_id="mock-follower",
            )

            with (
                patch(
                    "so101_teleoperation_validation.parse_args",
                    return_value=args,
                ),
                patch(
                    "so101_teleoperation_validation.SO101Leader",
                    return_value=leader,
                ),
                patch(
                    "so101_teleoperation_validation.SO101Follower",
                    return_value=follower,
                ),
                patch(
                    "so101_teleoperation_validation.connect_and_validate_arms",
                    return_value=preflight_snapshot,
                ) as preflight,
                patch(
                    "so101_teleoperation_validation.prepare_follower_at_operational_home",
                    return_value=home_snapshot,
                ) as startup,
                patch(
                    "so101_teleoperation_validation.read_positions",
                    return_value=leader_baseline,
                ) as read_baseline,
                patch(
                    "so101_teleoperation_validation.run_pose_validation_trial",
                    side_effect=complete_trial,
                ) as trial,
                patch(
                    "so101_teleoperation_validation.cleanup_connected_arms",
                    return_value=[],
                ) as cleanup,
                patch("builtins.input", return_value="") as operator_input,
                patch("builtins.print"),
            ):
                phase_calls = Mock()
                phase_calls.attach_mock(preflight, "preflight")
                phase_calls.attach_mock(startup, "startup")
                phase_calls.attach_mock(read_baseline, "baseline")
                phase_calls.attach_mock(trial, "trial")
                phase_calls.attach_mock(cleanup, "cleanup")

                exit_code = run_teleoperation_validation()
                self.assertEqual(exit_code, 0)
            operator_input.assert_called_once()

            self.assertEqual(
                [record[0] for record in phase_calls.mock_calls],
                [
                    "preflight",
                    "startup",
                    "baseline",
                    "trial",
                    "trial",
                    "trial",
                    "cleanup",
                ],
            )

            # Preflight state must become startup input.
            self.assertEqual(
                startup.call_args.kwargs["starting_pose"],
                follower_present,
            )

            # The orchestrator must execute the canonical three-pose order.
            self.assertEqual(
                [record.kwargs["pose_name"] for record in trial.call_args_list],
                list(POSE_NAMES),
            )

            # Each trial must receive the command returned by the prior trial.
            self.assertEqual(
                [
                    record.kwargs["commanded"][JOINTS[0]]
                    for record in trial.call_args_list
                ],
                [HOME[JOINTS[0]] + offset for offset in range(3)],
            )

            summary_paths = list(Path(output_dir).glob("*_summary.json"))
            self.assertEqual(len(summary_paths), 1)
            receipt = json.loads(summary_paths[0].read_text(encoding="utf-8"))

            self.assertEqual(receipt["status"], "completed")
            self.assertEqual(
                receipt["phases_completed"],
                [
                    "connection_preflight",
                    "follower_startup",
                    "leader_baseline",
                    *[f"pose_trial:{pose_name}" for pose_name in POSE_NAMES],
                    "cleanup",
                ],
            )
            self.assertEqual(
                [pose["pose"] for pose in receipt["poses"]],
                list(POSE_NAMES),
            )


if __name__ == "__main__":
    unittest.main()
