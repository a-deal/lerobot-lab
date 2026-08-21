# SO-101 physical orchestration preflight

Status: **EXECUTED - intentional-abort preflight passed on 2026-08-19**

## Purpose

Test the refactored startup, orchestration, intentional-abort, cleanup, and
receipt path on the real SO-101 pair before running another three-pose
validation.

This is not a no-motion test. The follower will enable torque and move from the
recognized unpowered-rest pose to the configured operational home. The test
aborts before the first teleoperation pose is authorized, so it must not enable
leader torque or command the follower toward a captured trial pose.

## Question answered

Can the real workflow:

1. connect to both calibrated arms with torque initially off;
2. recognize the follower's known unpowered-rest pose;
3. obtain explicit startup authorization and move the follower to operational
   home;
4. capture a passive leader baseline;
5. preview the first pose and honor `QUIT` before trial motion;
6. disable required torque, disconnect both arms, and write an auditable
   intentional-abort receipt?

## Explicitly out of scope

- authorizing `MOVE` for any pose;
- enabling leader torque;
- completing near, middle, or far pose validation;
- evaluating pose fidelity;
- camera capture, dataset recording, replay, or policy execution;
- changing calibration, mapping constants, home, bounds, rate limits, or
  lifecycle code;
- proving physical torque state from software return values alone.

## Fixed software configuration

- Branch: `refactor/pure-mapping-integration`
- Reviewed commit: `371e57ffc701c7f09a3b6ae7924a1f9647c3c629`
- Leader port: `/dev/cu.usbmodem5C4C1284061`
- Follower port: `/dev/cu.usbmodem5C4C1248501`
- Leader calibration ID: `so101_black_leader`
- Follower calibration ID: `so101_white_follower`
- Period: `0.1` seconds
- Maximum command step: `2.0` normalized units
- Hold duration: `3.0` seconds, unused because no trial is authorized

Do not substitute ports, calibration IDs, or control values during the run.
If device enumeration differs, stop and diagnose it as a separate problem.

## Software gate before touching hardware

From the repository root:

```bash
git status --short --branch
git rev-parse HEAD
./.venv/bin/python -m unittest discover -s tests -p 'test_so101*.py' -v
./.venv/bin/python run_so101_teleoperation_validation.py --self-test
```

Expected:

- clean working tree at the reviewed commit;
- 30 passing tests;
- `{"self_test": "passed"}`;
- no process already running this workflow or owning either serial port.

## Physical gate before execution

- Both arms are secured using the previously accepted clamp arrangement.
- The follower begins unpowered and externally supported as needed.
- The follower can travel from its known unpowered rest to operational home
  without contacting the table, either arm, cables, camera hardware, or the
  operator.
- The passive leader has a clear envelope for baseline and preview capture.
- Cables have strain relief and do not cross either motion envelope.
- Both verified 12 V, 5 A supplies and USB connections are routed to the
  correct arm.
- The surge-protector cutoff is reachable without entering an arm's sweep.
- The operator can support the follower during torque-off without reaching
  into commanded motion.
- No abnormal heat, odor, sound, damage, loose fastener, or cable condition is
  present.

If any item is false or uncertain, do not run.

## Connection and power sequence

1. Begin with the surge-protector cutoff off and no process using either arm.
2. Support the follower as needed and confirm both arm envelopes are clear.
3. Connect both USB controllers and verify that the two expected serial ports
   enumerate. Stop if either role maps to a different port.
4. With the cutoff still off, connect each arm only to its verified 12 V, 5 A
   supply.
5. Recheck clamps, cable strain, sweep clearance, and cutoff access.
6. Turn on the surge protector while remaining outside both sweep envelopes.
   Stop for any movement or abnormal sign before starting the program.
7. Run the command below only after both arms are powered, stationary, and
   clear.

## Stop conditions

Stop immediately for:

- unexpected or rapid movement;
- a command or prompt that does not match this card;
- contact or near-contact with the table, another arm, a cable, or the
  operator;
- abnormal sound, odor, heat, smoke, loose mounting, or cable strain;
- an incorrect port, calibration ID, starting state, torque state, or receipt
  path;
- inability to keep the cutoff reachable or support the follower at shutdown.

Keep hands outside both motion envelopes during commanded movement. Use the
reachable power cutoff for unexpected motion rather than reaching into a
moving arm. Treat any cutoff or abnormal stop as a failed preflight requiring
inspection before another run.

## Execution command

Run only after every gate above is reviewed:

```bash
./.venv/bin/python run_so101_teleoperation_validation.py \
  --leader-port /dev/cu.usbmodem5C4C1284061 \
  --follower-port /dev/cu.usbmodem5C4C1248501 \
  --leader-id so101_black_leader \
  --follower-id so101_white_follower \
  --period-s 0.1 \
  --max-step 2.0 \
  --hold-s 3.0 \
  --output-dir outputs/hardware/pose_fidelity
```

## Prompt-by-prompt operator script

1. Confirm the workflow prints the connected, torque-off state.
2. Proceed only if it reports the recognized stable unpowered-rest pose. Type
   `START` only after confirming the path to operational home is clear.
3. If the workflow instead asks for `HOLD`, enter anything other than `HOLD`
   and treat the run as an aborted preflight. Do not expand this run to the
   supported-start branch.
4. Keep hands outside the follower sweep while it moves to operational home.
5. At the leader-baseline prompt, place the passive leader in a corresponding,
   collision-safe baseline pose and press Enter.
6. At the `NEAR` preview prompt, leave the leader at that same safe baseline
   pose and press Enter. The baseline is the neutral reference, so an unchanged
   leader should produce zero relative change and avoid target saturation.
7. Inspect the no-motion preview. At the `MOVE`, `REDO`, or `QUIT` prompt, type
   `QUIT`.
8. Support the follower before acknowledging the shutdown prompt. Expect it to
   lose its torque-enabled hold when cleanup disables torque.
9. After the process exits, switch off arm power using the accepted shutdown
   procedure and inspect both arms and cables.

Do not type `MOVE` anywhere in this preflight.

## Expected result

The command is expected to return exit code `1` because `QUIT` is an intentional
operator abort. That nonzero exit is correct only when the summary receipt says:

```json
{
  "status": "operator_aborted",
  "error": "operator ended before all poses",
  "phases_completed": [
    "connection_preflight",
    "follower_startup",
    "leader_baseline",
    "cleanup"
  ],
  "poses": []
}
```

Also verify:

- exactly one summary JSON and one cycle CSV were created for the run;
- the CSV contains its header but no executed trial-cycle rows;
- neither receipt reports a cleanup error;
- both serial devices can be enumerated again after shutdown;
- no process retains either serial port;
- the operator observed no unexpected physical behavior.

## Interpretation boundary

A green preflight would show that the refactored workflow can reach operational
home, honor an intentional pre-motion abort, attempt cleanup, and preserve the
expected receipt on the real controller path. It would not prove physical
torque-off through independent sensing, safe execution of any trial pose,
leader holding, three-pose fidelity, sustained timing, collision avoidance,
camera synchronization, replay, or policy control.

## Review receipt

- Operator approval: granted for this bounded preflight
- Physical setup inspected: yes
- Exact command approved: yes
- Execution authorized: completed; no further execution authorized by this card
- Result: expected intentional abort with no trial poses or cleanup errors
- Summary: `outputs/hardware/pose_fidelity/20260819T212926Z_summary.json`
- Cycle log: `outputs/hardware/pose_fidelity/20260819T212926Z_cycles.csv` (header only)
