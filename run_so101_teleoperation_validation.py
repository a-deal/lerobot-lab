"""Run a bounded three-pose SO-101 leader/follower teleoperation validation.

Where this module sits
----------------------

The human moves the passive leader. This module reads that joint configuration,
asks ``so101_mapping`` for follower targets, previews the proposed movement,
and moves the powered follower only after explicit operator authorization.
After capture, the harness aligns and enables leader torque temporarily so the
comparison pose cannot collapse while the follower moves.

``so101_mapping`` is the translator. It owns input validation and target math
but cannot touch hardware. ``so101_lifecycle`` owns each arm's goal-alignment
and torque transitions. This module conducts the validation. It owns serial
connections, multi-arm cleanup orchestration, incremental commands, human
gates, measurements, evidence files, and shutdown policy.

This is intentionally not free-running teleoperation. Each pose is captured
once, previewed, explicitly authorized, approached through rate-limited
commands, and held long enough to separate mapping error from settling lag.
The follower returns to the same operational home between poses.

Current mapping boundary
------------------------

``calculate_joint_targets`` is the new clean entrypoint. It returns one
``JointTarget`` receipt per joint, keeping the raw proposal, allowed target,
and saturation flag together.

Preview, motion orchestration, logging, final evaluation, and the self-test all
carry those same receipts. The old three-parallel-dictionary interface has
been removed. Only the numerical motor API receives extracted bounded values,
because it needs destinations rather than mapping evidence.

What this module does not prove
-------------------------------

Passing software tests does not prove that either arm is calibrated correctly,
that equal joint coordinates create equal physical poses, or that every
bounded six-joint combination avoids the table or the robot itself. Those
claims require controlled hardware trials and recorded measurements.

Read the functions in this order
--------------------------------

1. ``read_positions`` validates the six-joint hardware state.
2. ``calculate_joint_targets`` calls the pure translator.
3. ``build_pose_preview`` shows the proposed receipts before movement.
4. ``target_log_fields`` translates one receipt into named CSV fields.
5. ``build_pose_result_record`` makes the durable per-pose summary.
6. ``next_commands`` rate-limits one numerical control step.
7. ``approach_and_hold`` executes and evaluates one authorized pose.
8. ``write_cycle_rows`` records the detailed evidence.
9. ``move_home`` restores the common follower anchor.
10. ``run_teleoperation_validation`` connects those pieces into the complete
    operator-gated validation.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so101_leader import SO101Leader, SO101LeaderConfig

from so101_lifecycle import ArmLifecycle
from so101_mapping import (
    JointMapping,
    JointTarget,
    final_pose_error,
    map_pose_relative,
)

JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)

# Operational home proven during the first persistent six-joint run.  This is
# a torque-held working pose, not the calibration midpoint.
HOME = {
    "shoulder_pan": 2.5078369905956066,
    "shoulder_lift": -12.620545073375268,
    "elbow_flex": 23.14049586776858,
    "wrist_flex": 5.565371024734972,
    "wrist_roll": 0.6664889362836561,
    "gripper": 30.92425295343989,
}

# This is the arm's only stable unpowered posture without an external cradle.
# It is a recognition reference, never a commanded destination: shoulder lift
# and elbow flex rest beyond the experiment's normal absolute motion margins.
UNPOWERED_REST = {
    "shoulder_pan": 3.056426332288396,
    "shoulder_lift": -91.9496855345912,
    "elbow_flex": 100.0,
    "wrist_flex": 4.416961130742052,
    "wrist_roll": 0.6664889362836561,
    "gripper": 31.13273106323836,
}
START_REST_TOLERANCE = 8.0

# These are conservative absolute coordinate margins.  They are not a motion
# plan and do not imply that every six-joint combination inside them is safe.
ABSOLUTE_BOUNDS = {
    joint: ((5.0, 95.0) if joint == "gripper" else (-80.0, 80.0)) for joint in JOINTS
}
GAINS = {joint: 1.0 for joint in JOINTS}
OFFSETS = {joint: 0.0 for joint in JOINTS}
SIGNS = {joint: 1.0 for joint in JOINTS}

# Adapter configuration for the pure mapping module.  The live harness still
# exposes its legacy three-dictionary result for now; this value translates the
# harness constants into the mapper's named per-joint contract.
MAPPING_CONFIGURATION = {
    joint: JointMapping(
        sign=SIGNS[joint],
        gain=GAINS[joint],
        offset=OFFSETS[joint],
        minimum=ABSOLUTE_BOUNDS[joint][0],
        maximum=ABSOLUTE_BOUNDS[joint][1],
    )
    for joint in JOINTS
}
POSE_NAMES = ("near", "middle", "far")

CSV_FIELDS = (
    "wall_time_utc",
    "monotonic_s",
    "pose",
    "phase",
    "cycle",
    "joint",
    "leader_baseline",
    "leader_captured",
    "leader_live",
    "leader_delta",
    "raw_target",
    "bounded_target",
    "absolute_clipped",
    "command",
    "rate_limited",
    "follower_measured",
    "command_error",
    "raw_target_error",
    "cycle_duration_s",
)


class UserAbort(RuntimeError):
    """Raised when the operator elects not to continue."""


def utc_now() -> str:
    """Return a timezone-aware timestamp for logs and summary receipts."""

    return datetime.now(UTC).isoformat()


def read_positions(bus: Any) -> dict[str, float]:
    """Read one complete normalized six-joint state from a connected bus.

    The hardware API may return extra channels, missing joints, or unusable
    numerical values. This boundary keeps only the expected joints and fails
    before incomplete state can influence a target or command.
    """

    positions = {
        key: float(value)
        for key, value in bus.sync_read("Present_Position", normalize=True).items()
    }
    missing = [joint for joint in JOINTS if joint not in positions]
    nonfinite = [
        joint
        for joint in JOINTS
        if joint in positions and not math.isfinite(positions[joint])
    ]
    if missing or nonfinite:
        raise RuntimeError(
            f"invalid joint state: missing={missing}, nonfinite={nonfinite}"
        )
    return {joint: positions[joint] for joint in JOINTS}


def clamp(value: float, low: float, high: float) -> float:
    """Keep one control-step value inside an inclusive interval.

    The pure mapper has its own target-bound clamp. This local helper remains
    because the hardware harness also limits how far a command may move during
    one control cycle.
    """

    return max(low, min(high, value))


def pose_within_tolerance(
    measured: dict[str, float],
    reference: dict[str, float],
    tolerance: float,
) -> bool:
    """Return whether every measured joint is near one complete reference.

    This recognizes a known startup state; it does not declare that state safe
    to command. Exact joint sets prevent a partial sensor read from looking
    like a valid rest pose.
    """
    expected_joints = set(JOINTS)

    if (
        set(measured) != expected_joints
        or set(reference) != expected_joints
        or tolerance < 0
    ):
        return False
    return all(
        math.isfinite(measured[joint])
        and math.isfinite(reference[joint])
        and abs(measured[joint] - reference[joint]) <= tolerance
        for joint in reference
    )


def calculate_joint_targets(
    leader_captured: dict[str, float],
    leader_baseline: dict[str, float],
) -> dict[str, JointTarget]:
    """Return the mapper's new named target receipts for all six joints.

    This is the clean socket. Callers put in two leader snapshots and receive
    one complete receipt for every follower joint. New consumers should use
    this function instead of the legacy three piles.

    The function stays hardware-free and does not reshape the mapper's result.
    """

    return map_pose_relative(
        leader_now=leader_captured,
        leader_start=leader_baseline,
        follower_home=HOME,
        configuration=MAPPING_CONFIGURATION,
    )


def build_pose_preview(
    *,
    pose_name: str,
    leader_captured: dict[str, float],
    leader_baseline: dict[str, float],
    targets: dict[str, JointTarget],
) -> dict[str, Any]:
    """Build the no-motion preview directly from named target receipts.

    The preview is the operator's last software-only inspection surface before
    authorizing movement. It must preserve the leader delta plus every target's
    raw proposal, bounded value, and saturation decision. It may format data
    for JSON, but it may not connect hardware or send commands.

    This is the first production consumer to migrate away from the legacy
    parallel dictionaries.
    """

    return {
        "stage": "pose_preview_no_motion",
        "pose": pose_name,
        "leader_delta": {
            joint: leader_captured[joint] - leader_baseline[joint] for joint in JOINTS
        },
        "raw_target": {joint: targets[joint].raw for joint in JOINTS},
        "bounded_target": {joint: targets[joint].bounded for joint in JOINTS},
        "absolute_clipped": {joint: targets[joint].saturated for joint in JOINTS},
    }


def target_log_fields(target: JointTarget) -> dict[str, float | int]:
    """Translate one target receipt into the three fields stored in the CSV.

    The mapper uses meaningful Python attributes: ``raw``, ``bounded``, and
    ``saturated``. The evidence file uses stable column names and stores
    booleans as ``0`` or ``1``. Keeping this tiny translation in one pure
    function prevents preview, logging, and evaluation from inventing
    different interpretations of the same receipt.

    It returns exactly ``raw_target``, ``bounded_target``, and
    ``absolute_clipped`` without changing either numerical value. The
    saturation boolean becomes an integer for the CSV.
    """

    return {
        "raw_target": target.raw,
        "bounded_target": target.bounded,
        "absolute_clipped": int(target.saturated),
    }


def build_pose_result_record(
    *,
    pose_name: str,
    leader_captured: dict[str, float],
    targets: dict[str, JointTarget],
    pose_result: dict[str, Any],
    visual_judgment: str,
) -> dict[str, Any]:
    """Build the durable summary for one completed physical pose.

    The live loop gathers the inputs, but this hardware-free function owns the
    output contract. Keeping summary construction testable prevents a deleted
    compatibility variable from surviving unnoticed in
    ``run_teleoperation_validation`` after the robot has already moved.
    """

    return {
        **pose_result,
        "name": pose_name,
        "leader_captured": leader_captured,
        "raw_targets": {joint: targets[joint].raw for joint in JOINTS},
        "bounded_targets": {joint: targets[joint].bounded for joint in JOINTS},
        "absolute_clipped": {joint: targets[joint].saturated for joint in JOINTS},
        "visual_judgment": visual_judgment,
    }


def next_commands(
    commanded: dict[str, float],
    targets: dict[str, float],
    max_step: float,
) -> tuple[dict[str, float], dict[str, bool]]:
    """Move every current command toward its target by at most ``max_step``.

    The returned boolean dictionary records which joints were rate-limited.
    This is command pacing, not target validation or collision avoidance.
    """

    next_values: dict[str, float] = {}
    limited: dict[str, bool] = {}
    for joint in JOINTS:
        gap = targets[joint] - commanded[joint]
        change = clamp(gap, -max_step, max_step)
        next_values[joint] = commanded[joint] + change
        limited[joint] = abs(gap) > max_step
    return next_values, limited


def write_cycle_rows(
    writer: csv.DictWriter,
    *,
    process_start: float,
    pose_name: str,
    phase: str,
    cycle: int,
    leader_baseline: dict[str, float],
    leader_captured: dict[str, float],
    leader_live: dict[str, float],
    joint_targets: dict[str, JointTarget],
    commanded: dict[str, float],
    rate_limited: dict[str, bool],
    follower_measured: dict[str, float],
    cycle_duration_s: float,
) -> None:
    """Write one evidence row per joint for one approach or hold cycle.

    Keeping source, target receipt, command, measurement, timing, and limiting
    values together makes later failures inspectable instead of reducing the
    run to a single success label. The whole receipt crosses this boundary so
    its raw proposal and safety decision cannot become misaligned parallel
    dictionaries.
    """

    wall_time = utc_now()
    monotonic_s = time.monotonic() - process_start
    for joint in JOINTS:
        target_fields = target_log_fields(joint_targets[joint])
        writer.writerow(
            {
                "wall_time_utc": wall_time,
                "monotonic_s": monotonic_s,
                "pose": pose_name,
                "phase": phase,
                "cycle": cycle,
                "joint": joint,
                "leader_baseline": leader_baseline[joint],
                "leader_captured": leader_captured[joint],
                "leader_live": leader_live[joint],
                "leader_delta": leader_captured[joint] - leader_baseline[joint],
                **target_fields,
                "command": commanded[joint],
                "rate_limited": int(rate_limited[joint]),
                "follower_measured": follower_measured[joint],
                "command_error": commanded[joint] - follower_measured[joint],
                "raw_target_error": joint_targets[joint].raw - follower_measured[joint],
                "cycle_duration_s": cycle_duration_s,
            }
        )


def approach_and_hold(
    *,
    follower: SO101Follower,
    leader: SO101Leader,
    writer: csv.DictWriter,
    process_start: float,
    pose_name: str,
    commanded: dict[str, float],
    leader_baseline: dict[str, float],
    leader_captured: dict[str, float],
    joint_targets: dict[str, JointTarget],
    period_s: float,
    max_step: float,
    hold_s: float,
) -> tuple[dict[str, float], dict[str, Any]]:
    """Approach one authorized target gradually, hold it, and measure error.

    This function owns the repeated control cycles after the operator has
    accepted a preview. It preserves complete receipts for evidence and final
    evaluation, but extracts their bounded numerical values for the existing
    rate limiter and motor command API. It does not decide whether the proposed
    physical path or pose is collision-free.
    """

    bounded_targets = {joint: joint_targets[joint].bounded for joint in JOINTS}
    cycle = 0
    approach_start = time.monotonic()
    rate_limited_cycles = {joint: 0 for joint in JOINTS}
    max_tracking_error = {joint: 0.0 for joint in JOINTS}

    while (
        max(abs(commanded[joint] - bounded_targets[joint]) for joint in JOINTS) > 0.05
    ):
        cycle_start = time.monotonic()
        leader_live = read_positions(leader.bus)
        commanded, rate_limited = next_commands(commanded, bounded_targets, max_step)
        follower.bus.sync_write("Goal_Position", commanded)
        remaining = period_s - (time.monotonic() - cycle_start)
        if remaining > 0:
            time.sleep(remaining)
        follower_measured = read_positions(follower.bus)
        cycle_duration = time.monotonic() - cycle_start
        for joint in JOINTS:
            rate_limited_cycles[joint] += int(rate_limited[joint])
            max_tracking_error[joint] = max(
                max_tracking_error[joint],
                abs(commanded[joint] - follower_measured[joint]),
            )
        write_cycle_rows(
            writer,
            process_start=process_start,
            pose_name=pose_name,
            phase="approach",
            cycle=cycle,
            leader_baseline=leader_baseline,
            leader_captured=leader_captured,
            leader_live=leader_live,
            joint_targets=joint_targets,
            commanded=commanded,
            rate_limited=rate_limited,
            follower_measured=follower_measured,
            cycle_duration_s=cycle_duration,
        )
        cycle += 1

    reached_command_at = time.monotonic()
    hold_start = reached_command_at
    while time.monotonic() - hold_start < hold_s:
        cycle_start = time.monotonic()
        leader_live = read_positions(leader.bus)
        follower_measured = read_positions(follower.bus)
        rate_limited = {joint: False for joint in JOINTS}
        cycle_duration = time.monotonic() - cycle_start
        for joint in JOINTS:
            max_tracking_error[joint] = max(
                max_tracking_error[joint],
                abs(commanded[joint] - follower_measured[joint]),
            )
        write_cycle_rows(
            writer,
            process_start=process_start,
            pose_name=pose_name,
            phase="hold",
            cycle=cycle,
            leader_baseline=leader_baseline,
            leader_captured=leader_captured,
            leader_live=leader_live,
            joint_targets=joint_targets,
            commanded=commanded,
            rate_limited=rate_limited,
            follower_measured=follower_measured,
            cycle_duration_s=cycle_duration,
        )
        cycle += 1
        remaining = period_s - (time.monotonic() - cycle_start)
        if remaining > 0:
            time.sleep(remaining)

    final_measured = read_positions(follower.bus)
    leader_final_measured = read_positions(leader.bus)

    final_error = final_pose_error(
        targets=joint_targets,
        follower_measured=final_measured,
    )
    leader_drift = {
        joint: leader_final_measured[joint] - leader_captured[joint] for joint in JOINTS
    }
    result = {
        "approach_seconds": reached_command_at - approach_start,
        "hold_seconds": time.monotonic() - hold_start,
        "cycles": cycle,
        "rate_limited_cycles": rate_limited_cycles,
        "max_tracking_error": max_tracking_error,
        "final_measured": final_measured,
        "final_signed_error": final_error,
        "leader_final_measured": leader_final_measured,
        "leader_end_drift": leader_drift,
    }
    return commanded, result


def move_home(
    follower: SO101Follower,
    commanded: dict[str, float],
    *,
    period_s: float,
    max_step: float,
) -> dict[str, float]:
    """Rate-limit the follower back to the fixed operational home."""

    while max(abs(commanded[joint] - HOME[joint]) for joint in JOINTS) > 0.05:
        commanded, _ = next_commands(commanded, HOME, max_step)
        follower.bus.sync_write("Goal_Position", commanded)
        time.sleep(period_s)
    return commanded


def prompt_visual_judgment() -> str:
    """Collect the operator's physical-pose label using fixed vocabulary."""

    while True:
        answer = (
            input(
                "While the follower holds: type MATCH, MISMATCH, or UNCLEAR for the physical pose.\n"
            )
            .strip()
            .upper()
        )
        if answer in {"MATCH", "MISMATCH", "UNCLEAR"}:
            return answer.lower()
        print("Expected MATCH, MISMATCH, or UNCLEAR.", flush=True)


def run_self_test() -> None:
    """Exercise mapping and rate limiting with ordinary numbers only.

    This smoke test never constructs a robot or connects to a serial port. It
    catches integration breakage but cannot validate physical behavior.
    """

    baseline = {joint: 0.0 for joint in JOINTS}
    captured = {joint: 10.0 for joint in JOINTS}
    targets = calculate_joint_targets(captured, baseline)
    for joint in JOINTS:
        assert math.isclose(targets[joint].raw, HOME[joint] + 10.0)
        assert math.isclose(targets[joint].bounded, targets[joint].raw)
        assert not targets[joint].saturated

    extreme = dict(captured)
    extreme["shoulder_pan"] = 500.0
    extreme_targets = calculate_joint_targets(extreme, baseline)
    assert extreme_targets["shoulder_pan"].bounded == 80.0
    assert extreme_targets["shoulder_pan"].saturated

    current = {joint: 0.0 for joint in JOINTS}
    target = {joint: 5.0 for joint in JOINTS}
    stepped, limited = next_commands(current, target, 2.0)
    assert all(stepped[joint] == 2.0 for joint in JOINTS)
    assert all(limited.values())
    print(json.dumps({"self_test": "passed"}), flush=True)


def parse_args() -> argparse.Namespace:
    """Build and parse the command-line configuration for one run."""

    parser = argparse.ArgumentParser(
        description="Run three explicitly approved SO-101 teleoperation-validation measurements."
    )
    parser.add_argument("--leader-port", default="/dev/cu.usbmodem5C4C1284061")
    parser.add_argument("--follower-port", default="/dev/cu.usbmodem5C4C1248501")
    parser.add_argument("--leader-id", default="so101_black_leader")
    parser.add_argument("--follower-id", default="so101_white_follower")
    parser.add_argument("--period-s", type=float, default=0.1)
    parser.add_argument("--max-step", type=float, default=2.0)
    parser.add_argument("--hold-s", type=float, default=3.0)
    parser.add_argument(
        "--output-dir",
        default="outputs/hardware/pose_fidelity",
        help="Output directory relative to the current working directory.",
    )
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def cleanup_connected_arms(
    *,
    leader_lifecycle: ArmLifecycle,
    follower_lifecycle: ArmLifecycle,
) -> list[str]:
    """Attempt every applicable cleanup step and return any failures."""

    errors: list[str] = []

    for lifecycle in [leader_lifecycle, follower_lifecycle]:
        if not lifecycle.bus.is_connected:
            continue
        try:
            lifecycle.disable_torque_if_required()
        except Exception as exc:  # noqa: BLE001 - best-effort cleanup boundary
            errors.append(f"{lifecycle.name} torque-off failed: {exc}")

    for lifecycle in [follower_lifecycle, leader_lifecycle]:
        if not lifecycle.bus.is_connected:
            continue
        try:
            lifecycle.bus.disconnect(disable_torque=False)
        except Exception as exc:  # noqa: BLE001 - best-effort cleanup boundary
            errors.append(f"{lifecycle.name} disconnect failed: {exc}")
    return errors


def record_cleanup_errors(receipt: dict[str, Any], cleanup_errors: list[str]) -> None:
    """Record any cleanup failures in the receipt."""
    if receipt["status"] == "completed":
        receipt["status"] = "cleanup_failed"
    receipt["cleanup_errors"] = cleanup_errors


def run_teleoperation_validation() -> int:
    """Run the operator-gated validation and always release owned resources.

    This is the only orchestration layer: it connects both arms, verifies
    torque preconditions, captures anchors and poses, authorizes movement,
    records evidence, and performs shutdown cleanup.
    """

    args = parse_args()
    if args.self_test:
        run_self_test()
        return 0
    if args.period_s <= 0 or args.max_step <= 0 or args.hold_s <= 0:
        raise ValueError("period, max step, and hold duration must be positive")

    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / f"{stamp}_cycles.csv"
    summary_path = output_dir / f"{stamp}_summary.json"
    process_start = time.monotonic()

    leader = SO101Leader(SO101LeaderConfig(port=args.leader_port, id=args.leader_id))
    follower = SO101Follower(
        SO101FollowerConfig(port=args.follower_port, id=args.follower_id)
    )
    leader_lifecycle = ArmLifecycle(name="leader", bus=leader.bus, joints=JOINTS)
    follower_lifecycle = ArmLifecycle(name="follower", bus=follower.bus, joints=JOINTS)
    receipt: dict[str, Any] = {
        "started_at": utc_now(),
        "status": "started",
        "configuration": {
            "leader_port": args.leader_port,
            "follower_port": args.follower_port,
            "leader_id": args.leader_id,
            "follower_id": args.follower_id,
            "period_s": args.period_s,
            "max_step": args.max_step,
            "hold_s": args.hold_s,
            "home": HOME,
            "absolute_bounds": ABSOLUTE_BOUNDS,
            "gains": GAINS,
            "offsets": OFFSETS,
            "signs": SIGNS,
        },
        "poses": [],
        "csv_path": str(csv_path),
        "summary_path": str(summary_path),
    }

    try:
        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            writer.writeheader()

            leader.bus.connect(handshake=True)
            follower.bus.connect(handshake=True)
            if not leader.bus.is_calibrated or not follower.bus.is_calibrated:
                raise RuntimeError("calibration mismatch")
            if any(
                int(leader.bus.read("Torque_Enable", joint, normalize=False))
                for joint in JOINTS
            ):
                raise RuntimeError("leader must remain passive")

            present = read_positions(follower.bus)
            follower_torque = {
                joint: int(follower.bus.read("Torque_Enable", joint, normalize=False))
                for joint in JOINTS
            }
            if any(follower_torque.values()):
                raise RuntimeError(
                    "follower torque was already enabled; stop the other controller first"
                )
            print(
                json.dumps(
                    {
                        "stage": "connected_torque_off",
                        "follower_present": present,
                        "follower_torque": follower_torque,
                    },
                    indent=2,
                ),
                flush=True,
            )
            if pose_within_tolerance(
                present,
                UNPOWERED_REST,
                START_REST_TOLERANCE,
            ):
                answer = (
                    input(
                        "Recognized the known stable unpowered rest pose. Clear the path to "
                        "operational home and type START; anything else aborts.\n"
                    )
                    .strip()
                    .upper()
                )
                if answer != "START":
                    raise UserAbort("operator declined recognized-rest startup")
            else:
                answer = (
                    input(
                        "Follower is outside the known stable rest pose. Support it and clear "
                        "the path to operational home. Type HOLD to continue; anything else aborts.\n"
                    )
                    .strip()
                    .upper()
                )
                if answer != "HOLD":
                    raise UserAbort("operator declined supported startup")
            follower_lifecycle.align_goals_and_enable_torque(present)

            time.sleep(0.4)
            commanded = move_home(
                follower,
                dict(present),
                period_s=args.period_s,
                max_step=args.max_step,
            )
            time.sleep(0.5)
            home_observed = read_positions(follower.bus)
            receipt["home_observed"] = home_observed
            print(
                json.dumps(
                    {
                        "stage": "home_hold",
                        "home_command": HOME,
                        "home_observed": home_observed,
                    },
                    indent=2,
                ),
                flush=True,
            )

            input(
                "Place the passive leader in a visually corresponding, collision-safe baseline pose, "
                "then press ENTER to capture it.\n"
            )
            leader_baseline = read_positions(leader.bus)
            receipt["leader_baseline"] = leader_baseline
            print(
                json.dumps(
                    {"stage": "baseline_captured", "leader_baseline": leader_baseline},
                    indent=2,
                ),
                flush=True,
            )

            for pose_name in POSE_NAMES:
                while True:
                    input(
                        f"Move the passive leader to the {pose_name.upper()} collision-safe pose and hold it. "
                        "Press ENTER for a no-motion preview.\n"
                    )
                    leader_captured = read_positions(leader.bus)
                    joint_targets = calculate_joint_targets(
                        leader_captured,
                        leader_baseline,
                    )
                    preview = build_pose_preview(
                        pose_name=pose_name,
                        leader_captured=leader_captured,
                        leader_baseline=leader_baseline,
                        targets=joint_targets,
                    )
                    print(json.dumps(preview, indent=2), flush=True)
                    if any(target.saturated for target in joint_targets.values()):
                        print(
                            "Rejected: at least one target crossed an absolute margin. "
                            "Reposition the leader; no motion was sent.",
                            flush=True,
                        )
                        continue
                    answer = (
                        input(
                            "Inspect both motion envelopes. Type MOVE to execute this held pose, "
                            "REDO to recapture, or QUIT to shut down.\n"
                        )
                        .strip()
                        .upper()
                    )
                    if answer == "REDO":
                        continue
                    if answer == "QUIT":
                        raise UserAbort("operator ended before all poses")
                    if answer != "MOVE":
                        print("Expected MOVE, REDO, or QUIT.", flush=True)
                        continue
                    break

                leader_lifecycle.align_goals_and_enable_torque(leader_captured)
                time.sleep(0.2)
                print(
                    json.dumps(
                        {
                            "stage": "leader_pose_hold",
                            "pose": pose_name,
                            "leader_captured": leader_captured,
                            "leader_hold_observed": read_positions(leader.bus),
                        },
                        indent=2,
                    ),
                    flush=True,
                )
                commanded, pose_result = approach_and_hold(
                    follower=follower,
                    leader=leader,
                    writer=writer,
                    process_start=process_start,
                    pose_name=pose_name,
                    commanded=commanded,
                    leader_baseline=leader_baseline,
                    leader_captured=leader_captured,
                    joint_targets=joint_targets,
                    period_s=args.period_s,
                    max_step=args.max_step,
                    hold_s=args.hold_s,
                )
                csv_file.flush()
                print(
                    json.dumps(
                        {
                            "stage": "pose_holding",
                            "pose": pose_name,
                            "result": pose_result,
                        },
                        indent=2,
                    ),
                    flush=True,
                )
                pose_result = build_pose_result_record(
                    pose_name=pose_name,
                    leader_captured=leader_captured,
                    targets=joint_targets,
                    pose_result=pose_result,
                    visual_judgment=prompt_visual_judgment(),
                )
                receipt["poses"].append(pose_result)
                leader_lifecycle.disable_torque_if_required()

                print(
                    json.dumps(
                        {"stage": "leader_released", "pose": pose_name},
                        indent=2,
                    ),
                    flush=True,
                )
                commanded = move_home(
                    follower,
                    commanded,
                    period_s=args.period_s,
                    max_step=args.max_step,
                )
                time.sleep(0.5)
                pose_result["home_return_observed"] = read_positions(follower.bus)
                print(
                    json.dumps(
                        {
                            "stage": "returned_home",
                            "pose": pose_name,
                            "home_return_observed": pose_result["home_return_observed"],
                        },
                        indent=2,
                    ),
                    flush=True,
                )

            receipt["status"] = "completed"
    except UserAbort as exc:
        receipt["status"] = "operator_aborted"
        receipt["error"] = str(exc)
    except KeyboardInterrupt:
        receipt["status"] = "keyboard_interrupt"
        receipt["error"] = "operator keyboard interrupt"
    except Exception as exc:
        receipt["status"] = "failed"
        receipt["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        receipt["finished_at"] = utc_now()

        if (
            follower_lifecycle.bus.is_connected
            and follower_lifecycle.torque_cleanup_required
        ):
            try:
                input(
                    "Support the follower, then press ENTER to disable torque and exit.\n"
                )
            except (EOFError, KeyboardInterrupt):
                print("Disabling follower torque now.", flush=True)
        cleanup_errors = cleanup_connected_arms(
            leader_lifecycle=leader_lifecycle,
            follower_lifecycle=follower_lifecycle,
        )

        if cleanup_errors:
            record_cleanup_errors(receipt, cleanup_errors)

        summary_path.write_text(
            json.dumps(receipt, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(
            json.dumps(
                {
                    "stage": "shutdown",
                    "status": receipt["status"],
                    "csv_path": str(csv_path),
                    "summary_path": str(summary_path),
                },
                indent=2,
            ),
            flush=True,
        )
    return 0 if receipt["status"] == "completed" else 1


if __name__ == "__main__":
    sys.exit(run_teleoperation_validation())
