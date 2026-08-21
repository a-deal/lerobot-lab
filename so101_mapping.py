"""Pure leader-to-follower coordinate mapping for an SO-101 pair.

This file is the robot's translator, not its hands.
It receives numbers describing where the leader moved and returns numbers the
follower could aim for.  A different module owns the dangerous act of sending
those numbers to motors.

That separation is an important software contract: importing or testing this
module cannot connect to serial ports, enable torque, or move hardware.  In
Python, a module is simply one ``.py`` file that groups related names.

Why these entities exist
------------------------

The public contract has four jobs, and each job gets one obvious owner:

1. Describe how one joint maps and what result was produced. ``JointMapping``
   is the immutable input configuration; ``JointTarget`` is the inspectable
   output.  Named value objects prevent a loose pile of numbers from losing
   its meaning as it crosses into logging, command, or evaluation code.
2. Transform coordinates. ``map_scalar_relative`` is the smallest worked
   reference for one joint. ``map_pose_relative`` applies the same rule to a
   complete pose, including optional sign, gain, and offset corrections.
3. Enforce numerical boundaries. ``require_finite`` rejects unusable sensor or
   policy values before arithmetic, while ``clamp`` applies the configured
   absolute interval and lets the result report saturation explicitly.
4. Evaluate the outcome. ``final_pose_error`` compares the requested bounded
   target with the follower's measured position without confusing evaluation
   with command generation.

Why this is sufficient now
--------------------------

The current experiment asks one narrow question: given calibrated leader
coordinates, fixed leader/follower anchors, and a per-joint configuration, what
bounded follower target should be proposed, and how far did the follower finish
from it?  The entities above cover every input, transform, boundary decision,
output, and evaluation value required to answer that question.  They are also
small enough to test with ordinary numbers and no attached robot.

Why this boundary can survive the next stages
---------------------------------------------

Teleoperation currently supplies ``leader_now``.  A replay system or learned
VLA policy may later supply proposed actions instead.  Those sources can share
the same validated target/result vocabulary, or receive their own adapter,
without teaching this module about cameras, language, neural networks, serial
ports, or torque.  Per-joint affine corrections already fit through ``sign``,
``gain``, and ``offset``; additional mapping strategies can implement a new
pure function while preserving the result contract.

Necessary and sufficient here does not mean sufficient for robot safety.  The
command layer must still own rate limits, timing, torque lifecycle, stale-goal
alignment, hardware communication, and emergency shutdown.  A motion-planning
layer must own table and self-collision avoidance.  The recording layer must
own synchronized camera, state, action, and timestamp samples.  Keeping those
responsibilities out is what makes this mapping module understandable,
testable, and reusable rather than another monolith.

For the terminology, call chain, reading order, and implementation sequence,
see ``docs/so101-mapping-learning-guide.md``.
"""

from __future__ import annotations

# ``dataclass`` generates the repetitive constructor, equality, and printable
# representation for small data-shaped classes.  It is idiomatic when an
# object mainly carries named values rather than complex behavior.
from dataclasses import dataclass
import math
# ``Mapping`` says callers may provide any read-only dictionary-like object.
# That is a more flexible input contract than demanding a concrete ``dict``.
from typing import Mapping


@dataclass(frozen=True)
class JointMapping:
    """Configuration for mapping one leader joint onto one follower joint.

    Think of this as the instruction card for one joint.  ``frozen=True``
    makes each card immutable after construction, which prevents a control run
    from silently changing its mapping halfway through.
    """

    sign: float = 1.0
    gain: float = 1.0
    offset: float = 0.0
    minimum: float = -100.0
    maximum: float = 100.0


@dataclass(frozen=True)
class JointTarget:
    """One joint's proposed target and the safety-bound result.

    Keeping ``raw`` and ``bounded`` together preserves evidence.  We can see
    what the mapper wanted, what the boundary allowed, and whether those two
    values differed instead of throwing the original proposal away.
    """

    raw: float
    bounded: float
    saturated: bool


def clamp(value: float, minimum: float, maximum: float) -> float:
    """Return ``value`` constrained to the inclusive interval.

    This tiny pure helper is the numerical equivalent of a doorway: values
    inside pass unchanged; values outside stop at the nearest edge.  Small
    helpers are idiomatic when they give one rule one obvious name.
    """

    return max(minimum, min(maximum, value))


def map_scalar_relative(
    *,
    leader_now: float,
    leader_start: float,
    follower_home: float,
) -> float:
    """Worked one-joint example using fixed anchors and direct sign.

    The three keyword-only arguments make call sites self-documenting: Python
    requires ``leader_now=...`` rather than accepting three easy-to-swap bare
    numbers.  The return annotation documents that this calculation produces
    one floating-point target.
    """

    # Always compare with the original leader anchor and then add that change
    # to the original follower anchor.  We never add a new move to the prior
    # target, so one cycle's tracking miss cannot become accumulated drift.
    return follower_home + (leader_now - leader_start)


def map_pose_relative(
    *,
    leader_now: Mapping[str, float],
    leader_start: Mapping[str, float],
    follower_home: Mapping[str, float],
    configuration: Mapping[str, JointMapping],
) -> dict[str, JointTarget]:
    """Map a complete leader pose into bounded follower targets.

    The signature is the public contract.  Inputs use ``Mapping`` because this
    function only reads them.  The output is a new concrete ``dict`` because
    the function constructs and owns that result.

    Implementation contract:

    1. Validate that every mapping contains exactly the configured joints.
    2. Reject non-finite numbers and invalid bounds.
    3. For each joint, calculate the fixed-anchor relative target with sign,
       gain, and offset.
    4. Clamp to the absolute bounds and report whether saturation occurred.

    This function must remain pure: it may not read hardware state or send a
    command.
    """
    # Validate that every mapping contains exactly the configured joints.
    expected_joints = set(configuration)
    provided_joints = {
        "leader_now": set(leader_now),
        "leader_start": set(leader_start),
        "follower_home": set(follower_home),
    }


    for source_name, actual_joints in provided_joints.items():
        if actual_joints != expected_joints:
            missing_joints = expected_joints - actual_joints
            extra_joints = actual_joints - expected_joints
            raise ValueError(f"Invalid {source_name} joints: {sorted(missing_joints)} missing, {sorted(extra_joints)} extra")


    targets: dict[str, JointTarget] = {}

    for joint_name in expected_joints:
        joint_mapping = configuration[joint_name]

        leader_now_value = require_finite(f"leader_now[{joint_name}]", leader_now[joint_name])
        leader_start_value = require_finite(f"leader_start[{joint_name}]", leader_start[joint_name])
        follower_home_value = require_finite(f"follower_home[{joint_name}]", follower_home[joint_name])

        minimum = require_finite(f"configuration[{joint_name}].minimum", joint_mapping.minimum)
        maximum = require_finite(f"configuration[{joint_name}].maximum", joint_mapping.maximum)

        if minimum > maximum:
            raise ValueError(
                f"Invalid {joint_name} bounds: "
                f"minimum {minimum} exceeds maximum {maximum}"
            )

        sign = require_finite(f"configuration[{joint_name}].sign", joint_mapping.sign)
        gain = require_finite(f"configuration[{joint_name}].gain", joint_mapping.gain)
        offset = require_finite(f"configuration[{joint_name}].offset", joint_mapping.offset)

        leader_change = leader_now_value - leader_start_value

        raw_target = (
            follower_home_value
            + offset
            + sign * gain * leader_change
        )

        bounded_target = clamp(
            raw_target,
            minimum,
            maximum,
        )

        targets[joint_name] = JointTarget(
            raw=raw_target,
            bounded=bounded_target,
            saturated=raw_target != bounded_target,
        )

    return targets

def final_pose_error(
    *,
    targets: Mapping[str, JointTarget],
    follower_measured: Mapping[str, float],
) -> dict[str, float]:
    """Return signed target-minus-measured error for each joint.

    Validate the joint keys and finite
    measured values, then calculate one signed error per joint.
    """
    expected_joints= set(targets)
    actual_joints = set(follower_measured)

    if actual_joints != expected_joints:
        missing_joints = expected_joints - actual_joints
        extra_joints = actual_joints - expected_joints
        raise ValueError(
            f"Invalid follower_measured joints: "
            f"{sorted(missing_joints)} missing, "
            f"{sorted(extra_joints)} extra"
        )

    errors: dict[str, float] = {}

    for joint_name, target in targets.items():
        measured = require_finite(
            f"follower_measured[{joint_name}]",
            follower_measured[joint_name],
        )

        errors[joint_name] = target.bounded - measured

    return errors


def require_finite(name: str, value: float) -> float:
    """Convert one value to ``float`` and reject NaN or infinity.

    Python accepts several numeric-looking types.  Normalizing them at the
    boundary gives the rest of the module one predictable representation.
    NaN and infinity are valid floating-point values mathematically, but they
    are invalid robot coordinates, so the contract rejects them early.
    """

    numeric = float(value)
    if not math.isfinite(numeric):
        raise ValueError(f"{name} must be finite")
    return numeric
