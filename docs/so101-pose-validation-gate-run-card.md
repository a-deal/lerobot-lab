# SO-101 refactored pose-validation gate

Status: **PRE-REGISTERED - NOT YET EXECUTED OR PHYSICALLY AUTHORIZED**

## Purpose

Validate physical pose execution through the refactored SO-101 workflow before
continuous teleoperation, held-out pose coverage, or camera recording.

This card defines two separate runs:

1. a deliberately small near-pose canary followed by an intentional quit; and
2. only after that receipt passes review, a complete near/middle/far anchor run.

The canary's success does not automatically authorize the three-pose run. Stop
after the canary, shut down, inspect its evidence, and obtain fresh live
authorization before starting the second process.

## Question answered

Can the accepted refactored workflow execute and document one small pose, then
all three named anchor poses, while preserving the previously reviewed startup,
preview, exact-command authorization, leader hold, rate-limited follower
approach, measurement, home return, cleanup, and receipt contracts?

## Evidence boundary

The gate separates three claims:

- **Software mapping:** the logged bounded target is the target produced by the
  reviewed fixed-anchor transform.
- **Follower tracking:** the measured follower finishes close to that bounded
  target.
- **Physical imitation:** the operator labels the held leader and follower
  poses `MATCH`, `MISMATCH`, or `UNCLEAR`.

The numerical errors in this card measure follower tracking, not physical
leader-follower similarity. A low tracking error cannot rescue a wrong mapping
or an unsafe path. The visual label is coarse evidence, not a geometric pose
measurement.

## Explicitly out of scope

- continuous leader-follower teleoperation;
- held-out pose coverage;
- camera capture, dataset recording, replay, or policy execution;
- changing calibration, mapping constants, operational home, absolute bounds,
  rate limits, lifecycle behavior, or command vocabulary;
- claiming collision avoidance from joint limits or target tracking;
- proving physical torque-off from a successful software call;
- changing a threshold after seeing a run so that the run passes.

## Frozen executable baseline

- Branch: `refactor/pure-mapping-integration`
- Executable baseline: `cd75bb5` (`Add pure SO-101 pose evaluation gate`)
- Leader port: `/dev/cu.usbmodem5C4C1284061`
- Follower port: `/dev/cu.usbmodem5C4C1248501`
- Leader calibration ID: `so101_black_leader`
- Follower calibration ID: `so101_white_follower`
- Period: `0.1` seconds
- Maximum command step: `2.0` normalized units
- Hold duration: `3.0` seconds

The final run-card commit may place documentation after `cd75bb5`. Before
hardware execution, confirm that the executable Python and test files have not
changed from the reviewed baseline. If executable behavior differs, stop and
run a new code-review and software-validation gate.

## Why these thresholds were chosen

The completed August 17 near/middle/far run provides the planning baseline:

| Pose | Final MAE | Largest final joint error | Largest home-return error | Leader drift note |
|---|---:|---:|---:|---|
| near | 1.108 | 4.327 | 1.237 | Invalid for held-pose comparison: passive leader collapsed |
| middle | 0.746 | 2.444 | 1.148 | 0.456 maximum drift |
| far | 0.754 | 1.958 | 3.949 | 0.000 maximum drift |

The new pass thresholds preserve modest margin around that evidence while
rejecting a materially worse result:

- final mean absolute error (MAE) at or below `1.5` normalized points;
- every absolute final joint error at or below `5.0` normalized points;
- every absolute leader end-drift value at or below `2.0` normalized points;
- every absolute home-return error at or below `5.0` normalized points.

MAE summarizes the typical final joint miss. The maximum-joint threshold stops
one bad joint from hiding inside that average. Leader drift makes the visual
comparison interpretable. Home-return error checks that the workflow restored
the torque-held operational reference after the trial. These are project
tolerances, not universal SO-101 or robotics standards.

Rate-limited cycles are expected command pacing and are not a failure by
themselves. Target saturation is a failure for this gate because it changes the
requested pose before motion.

## Software gate before touching hardware

From the repository root:

```bash
git status --short --branch
git rev-parse HEAD
git diff cd75bb5 -- \
  run_so101_teleoperation_validation.py \
  so101_joint_config.py \
  so101_mapping.py \
  so101_pose_gate.py \
  so101_teleoperation_validation.py \
  tests/test_so101_mapping.py \
  tests/test_so101_pose_gate.py \
  tests/test_so101_teleoperation_validation.py
./.venv/bin/python -m unittest discover -s tests -p 'test_so101*.py' -v
./.venv/bin/python run_so101_teleoperation_validation.py --self-test
```

Expected:

- no executable or test diff from `cd75bb5`;
- 39 passing hardware-free SO-101 tests;
- `{"self_test": "passed"}`;
- no process owns either serial port.

If any expectation fails, do not connect arm power for this gate.

## August 25 workcell prerequisite

Complete the powered-off workbench dry layout before either run:

- inspect bench level, wobble, damage, and fastener security;
- clamp both arms in the accepted positions and mark their bases;
- mark the follower task lane and clear leader and follower sweep envelopes;
- route USB and power cables outside both sweeps with strain relief;
- keep the surge-protector cutoff reachable without entering a sweep;
- establish and mark the externally supported shutdown position;
- place the camera stand provisionally outside both sweeps, but do not connect
  or record the camera;
- photograph and measure the accepted unpowered layout.

Do not use arm power merely to evaluate the furniture layout. Any uncertain
clearance, mounting, cable, cutoff, or shutdown-support condition blocks the
pose gate.

## Physical gate before each run

- Both arms are secure, stationary, and free of damage or loose fasteners.
- Both verified calibration identities and serial roles match this card.
- The follower begins in the recognized unpowered rest and can travel to
  operational home without contact.
- The leader can move manually from its neutral baseline to the requested pose
  without entering the follower sweep.
- The operator can inspect the complete proposed motion envelope before typing
  `MOVE`.
- The cutoff remains reachable and the operator can support the follower at
  shutdown without reaching into commanded motion.
- No abnormal heat, odor, sound, smoke, cable strain, or mounting condition is
  present.

If any item is false or uncertain, do not run.

## Stop conditions

Stop immediately for:

- unexpected, rapid, or uncommanded movement;
- a prompt that differs materially from this card;
- target saturation or a proposed path that is not clearly collision-safe;
- contact or near-contact with the table, either arm, a cable, or the operator;
- leader motion after its torque-enabled hold should have stabilized;
- abnormal sound, odor, heat, smoke, looseness, or cable strain;
- loss of cutoff access or safe shutdown support;
- an incorrect port, calibration ID, starting pose, torque state, or receipt
  path.

For unexpected motion, use the reachable power cutoff rather than reaching
into a moving arm. Treat a cutoff or abnormal stop as a failed run requiring
inspection before another attempt.

## Fixed execution command

Run only after the applicable live authorization:

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

## Stage A: one-pose canary

1. Complete the accepted connection and power sequence from the executed
   preflight card.
2. Authorize follower startup only after confirming the path to operational
   home is clear.
3. Put the passive leader in a collision-safe corresponding pose and press
   Enter to capture that pose as the neutral baseline.
4. At the `NEAR` capture gate, move the leader by one deliberately small,
   collision-safe delta and hold it steady. Press Enter for the read-only
   preview.
5. Confirm no target is saturated and inspect the complete physical path from
   operational home to the proposed follower target.
6. Type exact `MOVE` only if the preview and path pass inspection.
7. Keep hands outside both sweeps while the leader holds and the follower
   approaches. Stop for any stop condition.
8. While both arms hold, enter the honest visual label: `MATCH`, `MISMATCH`, or
   `UNCLEAR`.
9. Confirm the follower returns to operational home and the leader releases.
10. At the `MIDDLE` capture gate, type `QUIT` before moving or reading another
    candidate pose.
11. Support the follower for cleanup, complete shutdown, and inspect the
    receipt before considering Stage B.

The program is expected to exit with code `1` because Stage A intentionally
quits before all three poses. That exit is acceptable only when the receipt
passes every Stage A criterion below.

### Stage A receipt gate

- `status` is `operator_aborted`;
- `error` is `operator ended before all poses`;
- completed phases are exactly connection preflight, follower startup, leader
  baseline, `pose_trial:near`, and cleanup, in that order;
- `poses` contains exactly one completed record named `near`;
- `visual_judgment` is `match`;
- no joint is saturated;
- final MAE is at most `1.5`;
- largest absolute final joint error is at most `5.0`;
- largest absolute leader drift is at most `2.0`;
- largest absolute home-return error is at most `5.0`;
- cleanup errors are absent or empty;
- the cycle CSV contains near-pose approach and hold rows but no middle or far
  rows;
- no physical stop condition occurred and both serial ports are released.

If any criterion fails, Stage A fails. Preserve the receipt and diagnose the
failure without changing a threshold or proceeding to Stage B.

## Stage B: complete anchor gate

Stage B is a fresh process after Stage A shutdown and receipt review. It needs
a second explicit live authorization.

1. Repeat every software and physical precondition.
2. Capture one safe neutral leader baseline.
3. For `NEAR`, `MIDDLE`, and `FAR` in order, reposition the passive leader,
   request the read-only preview, inspect saturation and the physical path,
   type exact `MOVE` only for an accepted preview, label the held physical pose,
   and confirm the follower's home return before advancing.
4. Support the follower for cleanup and complete the accepted shutdown.
5. Inspect the summary and cycle receipts before authorizing continuous
   teleoperation or held-out coverage.

### Stage B receipt gate

- process exit code is `0` and `status` is `completed`;
- completed phases are exactly connection preflight, follower startup, leader
  baseline, `pose_trial:near`, `pose_trial:middle`, `pose_trial:far`, and
  cleanup, in that order;
- `poses` contains exactly `near`, `middle`, and `far`, in that order;
- every pose has `visual_judgment: match` and no saturated joint;
- every pose meets all four numerical thresholds;
- cleanup errors are absent or empty;
- the cycle CSV contains approach and hold evidence for all three poses;
- no physical stop condition occurred and both serial ports are released.

Any failed or unclear pose fails Stage B. Preserve all evidence and stop for
diagnosis rather than repeating until a pass appears.

## Deterministic receipt calculation

Set `SUMMARY` to the new summary JSON, then run:

```bash
SUMMARY=outputs/hardware/pose_fidelity/<timestamp>_summary.json
jq --argjson home \
  '{"shoulder_pan":2.5078369905956066,"shoulder_lift":-12.620545073375268,"elbow_flex":23.14049586776858,"wrist_flex":5.565371024734972,"wrist_roll":0.6664889362836561,"gripper":30.92425295343989}' \
  '
  def absval: if . < 0 then -. else . end;
  .poses[] |
    . as $pose |
    {
      name,
      visual_judgment,
      saturated_joints: [
        .absolute_clipped |
        to_entries[] |
        select(.value == true) |
        .key
      ],
      final_mae: ([.final_signed_error[] | absval] | add / length),
      final_max: ([.final_signed_error[] | absval] | max),
      leader_drift_max: ([.leader_end_drift[] | absval] | max),
      home_return_max: ([
        ($home | to_entries[]) as $entry |
        ($pose.home_return_observed[$entry.key] - $entry.value | absval)
      ] | max)
    }
  ' "$SUMMARY"
```

This calculation was checked against the August 17 receipt and reproduces the
planning values in this card. It summarizes the evidence; it does not inspect
physical safety, validate mapping semantics, or independently measure torque.

## Interpretation and advancement boundary

A green Stage A shows that one deliberately small pose can traverse the
refactored capture, preview, exact `MOVE`, leader-hold, follower-approach,
measurement, home-return, cleanup, and receipt path. It does not validate the
other anchors or continuous control.

A green Stage B shows that all three anchors met the predeclared tracking,
leader-stability, home-return, coarse visual, receipt, and operator-safety
criteria on one accepted workcell setup. It does not prove universal pose
coverage, collision avoidance, geometric pose equivalence, camera
synchronization, replay, or policy control.

Only after both stages pass may the project freeze a separate contract for
continuous six-joint control inside a named collision-safe operational
envelope. Camera recording remains later than that control-and-coverage gate.

## Review and execution receipt

- Threshold contract approved: yes, August 20
- Run-card diff reviewed: pending
- Run-card committed: yes, August 21
- August 25 dry layout accepted: pending
- Stage A live authorization: not granted
- Stage A result: not run
- Stage B live authorization: not granted
- Stage B result: not run
- Continuous teleoperation authorization: not granted
- Camera authorization: not granted
