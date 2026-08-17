#!/usr/bin/env python3
"""Conduct a bounded three-pose SO-101 leader/follower fidelity experiment.

Where this module sits
----------------------

The human moves the passive leader. This module reads that joint configuration,
asks ``so101_mapping`` for follower targets, previews the proposed movement,
and moves the powered follower only after explicit operator authorization.

``so101_mapping`` is the translator. It owns input validation and target math
but cannot touch hardware. This module is the experiment conductor. It owns
serial connections, torque lifecycle, incremental commands, human gates,
measurements, evidence files, and shutdown.

This is intentionally not free-running teleoperation. Each pose is captured
once, previewed, explicitly authorized, approached through rate-limited
commands, and held long enough to separate mapping error from settling lag.
The follower returns to the same operational home between poses.

Current migration boundary
--------------------------

``calculate_joint_targets`` is the new clean entrypoint. It returns one
``JointTarget`` receipt per joint, keeping the raw proposal, allowed target,
and saturation flag together.

``calculate_targets`` is the temporary legacy adapter. Existing consumers
still expect three parallel dictionaries named ``raw``, ``bounded``, and
``clipped``. The adapter translates the new receipts into that old shape. It
can be deleted after preview, motion, logging, evaluation, and the self-test
consume ``JointTarget`` objects directly and their replacement tests pass.

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
3. ``legacy_target_views`` and ``calculate_targets`` preserve the temporary
   legacy interface.
4. ``build_pose_preview`` makes the first downstream ``JointTarget`` consumer.
5. ``next_commands`` rate-limits one control step.
6. ``approach_and_hold`` executes and measures one authorized pose.
7. ``write_cycle_rows`` records the detailed evidence.
8. ``move_home`` restores the common follower anchor.
9. ``main`` connects those pieces into the complete operator-gated experiment.
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
from so101_mapping import JointMapping, JointTarget, map_pose_relative


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

# These are conservative absolute coordinate margins.  They are not a motion
# plan and do not imply that every six-joint combination inside them is safe.
ABSOLUTE_BOUNDS = {
    joint: ((5.0, 95.0) if joint == "gripper" else (-80.0, 80.0))
    for joint in JOINTS
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
        joint for joint in JOINTS if joint in positions and not math.isfinite(positions[joint])
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


def calculate_joint_targets(
    leader_captured: dict[str, float],
    leader_baseline: dict[str, float],
) -> dict[str, JointTarget]:
    """Return the mapper's new named target receipts for all six joints.

    This is the clean socket. Callers put in two leader snapshots and receive
    one complete receipt for every follower joint. New consumers should use
    this function instead of the legacy three piles.

    Andrew implements the delegation in this exercise. The function must stay
    hardware-free and must not reshape the mapper's result.
    """

    return map_pose_relative(
        leader_now=leader_captured,
        leader_start=leader_baseline,
        follower_home=HOME,
        configuration=MAPPING_CONFIGURATION,
    )


def legacy_target_views(
    targets: dict[str, JointTarget],
) -> tuple[dict[str, float], dict[str, float], dict[str, bool]]:
    """Split named target receipts into the harness's three legacy views.

    This is a temporary compatibility helper. It contains no mapping math and
    must disappear after preview, motion, logging, evaluation, and self-test
    code all consume ``JointTarget`` objects directly.
    """

    raw = {joint: targets[joint].raw for joint in JOINTS}
    bounded = {joint: targets[joint].bounded for joint in JOINTS}
    clipped = {joint: targets[joint].saturated for joint in JOINTS}
    return raw, bounded, clipped


def calculate_targets(
    leader_captured: dict[str, float],
    leader_baseline: dict[str, float],
) -> tuple[dict[str, float], dict[str, float], dict[str, bool]]:
    """Temporarily translate new target receipts into the legacy tuple.

    Existing harness consumers still expect three parallel dictionaries. This
    wrapper preserves that interface while each consumer migrates to
    ``calculate_joint_targets``. Delete it after no production call sites use
    the legacy tuple and the replacement tests are green.
    """

    targets = calculate_joint_targets(leader_captured, leader_baseline)
    return legacy_target_views(targets)


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
    parallel dictionaries. Andrew implements its returned dictionary in this
    exercise.
    """

    return {
        "stage": "pose_preview_no_motion",
        "pose": pose_name,
        "leader_delta": {
            joint: leader_captured[joint] - leader_baseline[joint]
            for joint in JOINTS
        },
        "raw_target": {
            joint: targets[joint].raw
            for joint in JOINTS
        },
        "bounded_target": {
            joint: targets[joint].bounded
            for joint in JOINTS
        },
        "absolute_clipped": {
            joint: targets[joint].saturated
            for joint in JOINTS
        },
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
    raw_targets: dict[str, float],
    bounded_targets: dict[str, float],
    absolute_clipped: dict[str, bool],
    commanded: dict[str, float],
    rate_limited: dict[str, bool],
    follower_measured: dict[str, float],
    cycle_duration_s: float,
) -> None:
    """Write one evidence row per joint for one approach or hold cycle.

    Keeping source, target, command, measurement, timing, and limiting values
    together makes later failures inspectable instead of reducing the run to a
    single success label.
    """

    wall_time = utc_now()
    monotonic_s = time.monotonic() - process_start
    for joint in JOINTS:
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
                "raw_target": raw_targets[joint],
                "bounded_target": bounded_targets[joint],
                "absolute_clipped": int(absolute_clipped[joint]),
                "command": commanded[joint],
                "rate_limited": int(rate_limited[joint]),
                "follower_measured": follower_measured[joint],
                "command_error": commanded[joint] - follower_measured[joint],
                "raw_target_error": raw_targets[joint] - follower_measured[joint],
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
    raw_targets: dict[str, float],
    bounded_targets: dict[str, float],
    absolute_clipped: dict[str, bool],
    period_s: float,
    max_step: float,
    hold_s: float,
) -> tuple[dict[str, float], dict[str, Any]]:
    """Approach one authorized target gradually, hold it, and measure error.

    This function owns the repeated control cycles after the operator has
    accepted a preview. It does not decide whether the proposed physical path
    or pose is collision-free.
    """

    cycle = 0
    approach_start = time.monotonic()
    rate_limited_cycles = {joint: 0 for joint in JOINTS}
    max_tracking_error = {joint: 0.0 for joint in JOINTS}

    while max(abs(commanded[joint] - bounded_targets[joint]) for joint in JOINTS) > 0.05:
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
            raw_targets=raw_targets,
            bounded_targets=bounded_targets,
            absolute_clipped=absolute_clipped,
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
            raw_targets=raw_targets,
            bounded_targets=bounded_targets,
            absolute_clipped=absolute_clipped,
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
    final_error = {
        joint: bounded_targets[joint] - final_measured[joint] for joint in JOINTS
    }
    leader_drift = {
        joint: read_positions(leader.bus)[joint] - leader_captured[joint]
        for joint in JOINTS
    }
    result = {
        "approach_seconds": reached_command_at - approach_start,
        "hold_seconds": time.monotonic() - hold_start,
        "cycles": cycle,
        "rate_limited_cycles": rate_limited_cycles,
        "max_tracking_error": max_tracking_error,
        "final_measured": final_measured,
        "final_signed_error": final_error,
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
        answer = input(
            "While the follower holds: type MATCH, MISMATCH, or UNCLEAR for the physical pose.\n"
        ).strip().upper()
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
    raw, bounded, clipped = calculate_targets(captured, baseline)
    for joint in JOINTS:
        assert math.isclose(raw[joint], HOME[joint] + 10.0)
        assert math.isclose(bounded[joint], raw[joint])
        assert not clipped[joint]

    extreme = dict(captured)
    extreme["shoulder_pan"] = 500.0
    _, bounded_extreme, clipped_extreme = calculate_targets(extreme, baseline)
    assert bounded_extreme["shoulder_pan"] == 80.0
    assert clipped_extreme["shoulder_pan"]

    current = {joint: 0.0 for joint in JOINTS}
    target = {joint: 5.0 for joint in JOINTS}
    stepped, limited = next_commands(current, target, 2.0)
    assert all(stepped[joint] == 2.0 for joint in JOINTS)
    assert all(limited.values())
    print(json.dumps({"self_test": "passed"}), flush=True)


def parse_args() -> argparse.Namespace:
    """Build and parse the command-line configuration for one run."""

    parser = argparse.ArgumentParser(
        description="Run three explicitly approved, held SO-101 pose-fidelity measurements."
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


def main() -> int:
    """Run the operator-gated experiment and always release owned resources.

    ``main`` is the only orchestration layer: it connects both arms, verifies
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

    leader = SO101Leader(
        SO101LeaderConfig(port=args.leader_port, id=args.leader_id)
    )
    follower = SO101Follower(
        SO101FollowerConfig(port=args.follower_port, id=args.follower_id)
    )
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
    torque_owned = False

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
                joint: int(
                    follower.bus.read("Torque_Enable", joint, normalize=False)
                )
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
            answer = input(
                "Support the follower and clear its path to the known operational home. "
                "Type HOLD to align goals, enable torque, and move home; anything else aborts.\n"
            ).strip().upper()
            if answer != "HOLD":
                raise UserAbort("operator declined torque enable")

            follower.bus.sync_write("Goal_Position", present)
            follower.bus.enable_torque(list(JOINTS))
            torque_owned = True
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
                    answer = input(
                        "Inspect both motion envelopes. Type MOVE to execute this held pose, "
                        "REDO to recapture, or QUIT to shut down.\n"
                    ).strip().upper()
                    if answer == "REDO":
                        continue
                    if answer == "QUIT":
                        raise UserAbort("operator ended before all poses")
                    if answer != "MOVE":
                        print("Expected MOVE, REDO, or QUIT.", flush=True)
                        continue
                    break

                raw_targets, bounded_targets, absolute_clipped = legacy_target_views(
                    joint_targets
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
                    raw_targets=raw_targets,
                    bounded_targets=bounded_targets,
                    absolute_clipped=absolute_clipped,
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
                pose_result.update(
                    {
                        "name": pose_name,
                        "leader_captured": leader_captured,
                        "raw_targets": raw_targets,
                        "bounded_targets": bounded_targets,
                        "visual_judgment": prompt_visual_judgment(),
                    }
                )
                receipt["poses"].append(pose_result)
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
        if follower.bus.is_connected:
            try:
                if torque_owned:
                    try:
                        input(
                            "Support the follower, then press ENTER to disable torque and exit.\n"
                        )
                    except (EOFError, KeyboardInterrupt):
                        print("Disabling follower torque now.", flush=True)
                    follower.bus.disable_torque(list(JOINTS))
            finally:
                follower.bus.disconnect(disable_torque=False)
        if leader.bus.is_connected:
            leader.bus.disconnect(disable_torque=False)
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
    sys.exit(main())
