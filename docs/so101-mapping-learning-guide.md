# SO-101 mapping learning guide

This is the companion for `so101_mapping.py` and
`tests/test_so101_mapping.py`. It teaches the robotics concepts and Python
patterns used to complete the first pure mapping problem without hardware.

## Start here: read the code in this order

Do not read straight from line one to the bottom as if it were prose. Use this
top-down route:

1. **Read the module explanation.** Learn the boundary: translator, not hands.
2. **Read the tests.** They are executable examples of the behavior we promise.
   Calculate their expected values manually before reading implementation.
3. **Read `JointMapping` and `JointTarget`.** These define the input instruction
   card and output receipt.
4. **Read `map_scalar_relative`.** This is the smallest completed example and
   the reference formula for one joint.
5. **Read `require_finite` and `clamp`.** These are the two reusable boundary
   primitives: reject meaningless values; constrain excessive values.
6. **Trace `map_pose_relative`.** Follow how the scalar rule is generalized
   across every configured joint to produce one `JointTarget` per joint.
7. **Trace `final_pose_error`.** Follow how bounded targets are compared with
   measured follower positions after mapping is already understood.

## Call chain for the main mapping test

```text
test prepares named dictionaries and JointMapping cards       ARRANGE
        ↓
test calls map_pose_relative                                  ACT
        ↓
function verifies that all joint-name sets agree
        ↓
function validates finite values and usable bounds
        ↓
for each joint:
    delta = leader_now - leader_start
    raw = follower_home + offset + sign × gain × delta
    bounded = clamp(raw, minimum, maximum)
    saturated = raw differs from bounded
    result[joint] = JointTarget(raw, bounded, saturated)
        ↓
function returns the result dictionary
        ↓
test compares public result fields with expected values       ASSERT
```

## Implementation order

The implementation was completed one behavior at a time rather than by writing
the whole function and debugging a pile of failures.

1. Capture the configured joint-name set.
2. Require `leader_now`, `leader_start`, and `follower_home` to contain exactly
   that same set. Raise `ValueError` if any set differs.
3. For each joint, validate its three input coordinates plus sign, gain, offset,
   minimum, and maximum with `require_finite`.
4. Reject a configuration whose `minimum` is greater than its `maximum`.
5. Calculate delta and raw target from the fixed anchors.
6. Clamp the raw target and record whether it saturated.
7. Return a new dictionary of `JointTarget` objects.
8. Implement `final_pose_error` with the same exact-joint-set and finite-value
   discipline.
9. Add at least one independently stated edge-case test before declaring the
   mapper complete.

## Integration exercise: connect the mapper to the hardware harness

### Look-ahead and definition of done

This exercise sits between isolated mapping logic and physical pose-fidelity
measurement. It does not add a new mapping algorithm or command movement.
It makes the live harness depend on the already-tested mapper.

The bounded definition of done is:

1. integrate one existing pure module, `so101_mapping.py`, into one existing
   hardware workflow, `so101_teleoperation_validation.py`;
2. add one hardware-free adapter test;
3. keep all eleven pure-mapper tests green;
4. make the one adapter test green;
5. pass the harness's existing hardware-free self-test;
6. preserve all serial, torque, rate-limit, prompt, logging, and shutdown
   behavior unchanged.

If this turns green, the next stage is a separate refactor that carries
`JointTarget` objects farther through the harness and removes the temporary
three-dictionary compatibility layer. Keeping those stages separate tells us
whether a failure came from connecting the mapper or redesigning consumers.

**Current status:** those consumer migrations are complete. A later bounded
refactor added `so101_lifecycle.py`, which now owns each arm's goal-alignment
and torque-cleanup transitions. The sequence below preserves the original
mapping exercise while the architecture notes describe the current boundary.

### Module and call-chain map

```text
leader positions captured by the hardware harness
                    ↓
calculate_targets adapter in so101_teleoperation_validation.py
                    ↓
map_pose_relative in so101_mapping.py
                    ↓
dict[joint name, JointTarget]
                    ↓
preview, command, logging, and evaluation consumers
```

`so101_mapping.py` is the translator. It owns validation and target math but
must remain unable to touch hardware. `so101_lifecycle.py` owns each arm's
goal-alignment ordering and torque-cleanup obligation.
`so101_teleoperation_validation.py` conducts the validation. It owns
hardware reads, command timing, operator gates, multi-arm cleanup, evidence,
and receipt policy.

### Legacy surface versus new surface

| Surface | Status | Meaning | Removal condition |
|---|---|---|---|
| `raw`, `bounded`, `clipped` parallel dictionaries | Removed compatibility layer | Three separately keyed views formerly used by the harness | Removal condition satisfied after every consumer accepted `JointTarget` values directly. |
| `clipped` boolean name | Removed legacy vocabulary | Former name for whether an absolute bound altered a raw target | Replaced by the mapper's `saturated` field. |
| `JointMapping` configuration objects | New contract | One named instruction card per joint | Retain unless a later strategy requires a different mapping model. |
| `JointTarget` result objects | New contract | Raw proposal, allowed value, and saturation evidence kept together | Carry end-to-end through preview, logging, command, and evaluation code. |
| `map_pose_relative` | New calculation owner | Validates and maps a complete named joint configuration | Retain as the single affine-mapping implementation. |

### Hardware-harness function map

| Entity | Explain it like I am ten | Responsibility |
|---|---|---|
| `UserAbort` | A named emergency exit requested by the operator. | Separates intentional human cancellation from unexpected failures. |
| `utc_now` | Put the current clock time on the receipt. | Supplies comparable wall-clock timestamps. |
| `read_positions` | Ask all six joints where they are and reject an incomplete answer. | Reads normalized hardware state and validates names and finite values. |
| harness `clamp` | Take only one small step even when the destination is far away. | Supports command rate limiting; it is separate from target-bound clamping. |
| `calculate_joint_targets` | Ask the pure translator for one receipt per joint. | Calls `map_pose_relative` and preserves the returned `JointTarget` objects. |
| `next_commands` | Move each command a little closer to its destination. | Applies the per-cycle maximum-step limit. |
| `write_cycle_rows` | Write down what every joint was asked to do and what it actually did. | Produces the detailed CSV evidence. |
| `approach_and_hold` | Walk to the target carefully, stay there briefly, and measure the result. | Owns the control loop, hold window, measurements, and per-pose summary. |
| `move_home` | Walk the follower back to its known working pose. | Reestablishes the same follower anchor between trials. |
| `prompt_visual_judgment` | Ask the human whether the physical shapes looked alike. | Captures a manual evaluation label. |
| `run_self_test` | Rehearse the arithmetic with pretend numbers and no robot. | Provides a quick hardware-free integration smoke test. |
| `parse_args` | Read the operator's command-line settings. | Builds the runtime configuration. |
| `ArmLifecycle` | Keep one arm's torque-cleanup sticky note with its bus. | Owns goal alignment, torque transitions, and the conservative cleanup obligation. |
| `connect_and_validate_arms` | Inspect both arms before allowing the experiment to start. | Connects both buses, verifies calibration and passive torque state, and returns the validated follower snapshot. |
| `prepare_follower_at_operational_home` | Get permission, power the follower carefully, and move it to the starting line. | Owns startup authorization, goal alignment, torque enablement, rate-limited home movement, and the resulting measurement. |
| `run_pose_validation_trial` | Preview, authorize, measure, and reset one named pose. | Owns one complete trial while recording its result before leader torque release. |
| `run_teleoperation_validation` | Conduct the three-phase validation and publish its final receipt. | Orchestrates preflight, startup, three pose trials, top-level status, and shutdown policy. |

### Python execution and exception flow

This harness is synchronous. Its execution model is the same basic model as
JavaScript's synchronous `try`/`catch`/`finally`: each function call adds a
frame to a stack of paused callers. A normal return removes the current frame
and gives a value back to its caller. An exception interrupts the current path
and searches backward through those callers for a matching handler.

```text
module entrypoint
        ↓ calls
run_teleoperation_validation
        ├─ connect_and_validate_arms
        ├─ prepare_follower_at_operational_home
        ├─ run_pose_validation_trial (once per named pose)
        └─ finally: cleanup_connected_arms
                       ↓ calls
                 ArmLifecycle.disable_torque_if_required
                       ↓ calls
                 bus.disable_torque

An uncaught exception travels back up this stack in the opposite direction.
```

The important Python operations have separate meanings:

| Operation | Meaning |
|---|---|
| `raise SomeError(...)` | Start propagating a new exception. |
| `except SomeError` | Intercept a matching exception at this layer and decide what happens next. |
| bare `raise` inside `except` | Continue propagating the same exception after recording or adding context. |
| `finally` | Run this block before leaving the `try`, whether the path succeeds, returns, or propagates an exception. |

Catching an exception does not automatically mean the program failed, and it
does not automatically mean the program recovered. The handler makes that
policy decision:

- catch and continue when this layer can restore a valid state or choose a
  safe fallback;
- catch and convert the exception into ordinary result data when callers need
  to inspect a degraded outcome;
- catch, record, and re-raise when the layer must preserve evidence but cannot
  safely continue normal work;
- do not catch when a higher caller is the first layer that can make a useful
  decision.

`cleanup_connected_arms` uses the second pattern. It asks each connected
`ArmLifecycle` to discharge its torque obligation, then attempts each
disconnect. Every arm/action pair has its own exception boundary. A failure
becomes a labeled string in `cleanup_errors`, and the helper continues trying
the other independent cleanup actions. This is best-effort continuation, not
proof that torque was physically disabled or that either arm reached a safe
pose.

`run_teleoperation_validation` owns the top-level outcome. Its receipt and
process behavior follow this contract:

| Experiment body | Cleanup | Receipt status | Process behavior |
|---|---|---|---|
| completes | succeeds | `completed` | return `0` |
| completes | reports errors | `cleanup_failed` plus `cleanup_errors` | return `1` |
| operator abort or handled interrupt | any cleanup result | preserve the earlier status and attach any `cleanup_errors` | return `1` |
| raises an unexpected `Exception` | any cleanup result | `failed`, original `error`, plus any `cleanup_errors` | write the receipt, then re-raise the original exception |

The unexpected-exception handler in `run_teleoperation_validation` records the primary experiment
failure and uses bare `raise`. The outer `finally` then runs cleanup and writes
the receipt before that original exception leaves the function. The cleanup error is
not hidden: it remains in the receipt as secondary evidence. The compact
shutdown message prints the final status and receipt path; the receipt file is
the complete record.

The final success rule is therefore:

```text
success = experiment completed AND required cleanup reported no errors
```

When this module is imported by a unit test, Python defines its functions but
does not run `run_teleoperation_validation` because `__name__ != "__main__"`.
The cleanup unit test can
therefore inject failures into mock buses without connecting hardware. Its
current leader-torque-failure case proves that the expected calls are attempted
and later disconnect cleanup continues; it does not prove physical shutdown or
every possible cleanup-failure position.

### Andrew-owned implementation surface

For this exercise Andrew implements only the body of `calculate_targets`.
The signature and legacy return type stay fixed. The adapter must:

1. call `map_pose_relative` with the captured leader positions, fixed leader
   baseline, follower `HOME`, and `MAPPING_CONFIGURATION`;
2. unpack each result's `raw` value into the legacy `raw` dictionary;
3. unpack each result's `bounded` value into the legacy `bounded` dictionary;
4. translate each result's `saturated` boolean into the legacy `clipped`
   dictionary;
5. return those three dictionaries in their existing order.

### Python concepts practiced by the adapter

- **Import boundary:** the harness imports the pure mapper; the pure mapper
  never imports the harness. This prevents circular dependencies and keeps
  hardware out of pure tests.
- **Dictionary comprehension:** construct one complete keyed result by
  transforming every `(joint, target)` pair.
- **Attribute access:** `target.raw`, `target.bounded`, and
  `target.saturated` read named dataclass fields rather than tuple positions.
- **Compatibility adapter:** preserve an old interface while delegating its
  underlying behavior to a new interface.
- **Single source of truth:** one tested function owns the mapping equation;
  the harness no longer keeps a second copy that can drift independently.

### Adapter-test path

```text
test arranges six pretend leader values and six pretend JointTarget receipts
        ↓
patch temporarily replaces map_pose_relative with a controllable test double
        ↓
test calls the public calculate_targets adapter
        ↓
test proves the mapper was called once with the expected inputs
        ↓
test proves raw, bounded, and saturated fields reached the correct legacy outputs
```

The test proves delegation and translation. Combined with the pure mapper
suite, it proves the tested numerical contract is connected to the harness.
It does not prove calibration, physical pose correspondence, command timing,
collision safety, torque behavior, or real leader-follower fidelity.

## Run the red-to-green testing loop

Run the suite from the repository root:

```bash
./.venv/bin/python -m unittest discover -s tests -p 'test_so101_mapping.py' -v
```

Use the result in three stages:

1. **Red:** during development, the unfinished function raised
   `NotImplementedError`. This proved the test reached the intended missing
   behavior.
2. **Green:** implement the smallest rule that satisfies the contract and run
   the same suite again. All relevant tests must pass.
3. **Refactor:** improve names or structure without changing behavior, then run
   the same tests to show the contract survived.

Do not make a failing test green by weakening its expected result unless the
contract itself was wrong and that decision has been made explicitly.

## Stop boundary: what passing tests will not prove

Even a fully green pure-mapping suite will not establish:

- correct calibration;
- safe table or self-collision geometry;
- motor communication;
- torque behavior;
- command timing or rate limiting;
- physical tracking accuracy;
- synchronized camera recording;
- replay or policy competence.

Those belong to later integration and physical-evaluation gates.

## Reference material

## First terminology correction

The six scalar readings are a **joint-space configuration**: one coordinate
for each controlled joint channel. On this SO-101 pair, the software treats
shoulder pan, shoulder lift, elbow flex, wrist flex, wrist roll, and the
gripper as six named coordinates.

That is not quite the same thing as a pose in the most precise robotics usage:

- **Joint-space configuration:** the vector `q` of joint coordinates. It tells
  us the arrangement of the robot's joints.
- **Rigid-body pose:** the position and orientation of a link, usually the end
  effector, relative to a reference frame. Position may be `(x, y, z)` and
  orientation may use a quaternion, rotation matrix, or Euler angles.
- **Robot state:** broader than either. It may include joint positions,
  velocities, effort or torque, timestamps, and sensor observations.

The current name `map_pose_relative` uses “pose” as convenient project
shorthand for a complete set of joint positions. A more formally precise name
would be `map_joint_positions_relative`. We should decide that API name before
the module becomes public or is imported by other code.

`sign`, `gain`, `offset`, and bounds are **not part of the joint configuration**.
They are configuration for the mapping between the two arms:

```text
leader joint configuration
          +
fixed leader and follower anchors
          +
per-joint mapping configuration
          ↓
proposed follower joint targets
```

## Where this module sits

```text
Action source
  human-operated leader now; replay or VLA later
        ↓
Pure mapping layer
  validate → transform → clamp → report saturation
        ↓
Command layer
  rate-limit → manage torque → send motor goals → shut down safely
        ↓
Physical follower
        ↓
Evaluation and recording
  measured state → error → synchronized logs
```

The mapping module owns only the second box. That is why we can test it with
ordinary numbers.

## The overall mosaic

Think of the robot system as a relay team. Each layer receives one kind of
information, performs one job, and hands a clearer result to the next layer.

```text
1. Intention and perception
   human sees the scene, or VLM/VLA receives image + language
                         ↓
2. Action source
   leader arm now; learned policy or replay later
                         ↓
3. Robot representation
   named joint coordinates, state, anchors, and calibration
                         ↓
4. Mapping
   validate → delta → sign/gain/offset → target → clamp
                         ↓
5. Command and safety
   rate limit → torque lifecycle → motor command → emergency cleanup
                         ↓
6. Physical robot
   servos, links, gripper, encoders, environment
                         ↓
7. Measurement and evaluation
   measured state → error → success/failure labels → synchronized record
                         ↓
8. Learning loop
   demonstrations → training → policy → deployment → new evaluation
```

Our current module owns layer 4 and produces evidence used by layer 7. The
glossary below locates every term within this larger relay.

## Glossary: robot and coordinate language

| Term | Explain it like I am ten | Why it matters | Where it sits |
|---|---|---|---|
| Scalar | One number. A shoulder-pan coordinate of `20.0` is a scalar. | Robot software eventually reduces many observations and commands to individual numbers. If one scalar is wrong, one joint may be wrong. | Basic building block used throughout layers 3-7. |
| Joint | A mechanical connection where robot parts move relative to each other. | Joints are where the arm changes shape. Software normally commands or measures them individually. | Physical hardware in layer 6; represented numerically in layer 3. |
| Degree of freedom (DoF) | One independent way something can move, such as rotating left and right. | It tells us how many independent coordinates are needed to describe or control movement. A joint can theoretically have multiple DoFs, although each SO-101 controlled channel contributes one coordinate here. | Connects physical mechanics in layer 6 to representation in layer 3. |
| Gripper channel | The number describing how open or closed the gripper is. | It behaves like the sixth controlled coordinate in our software, even though opening and closing is different from positioning an arm link in space. | Layers 3-6. |
| Joint coordinate | The number used to represent one joint's position within its calibrated range. | It is the shared unit consumed by mapping, commands, measurements, and datasets. | Layer 3, then passed through layers 4-7. |
| Joint-space configuration | All joint coordinates considered together, often written as vector `q`. Our dictionary keeps the joint names attached. | It describes the arm's mechanical arrangement without first calculating where the gripper sits in the room. This is the representation our current mapper accepts. | Layer 3. |
| Configuration space | The set of all possible joint configurations, including combinations that may be unsafe or unreachable in practice. | Per-joint values can each look legal while their combination causes a table or self-collision. This is why independent clamps are not collision avoidance. | Spans layers 3-6; a future motion-planning concern. |
| Reference frame | An agreed origin and set of axes from which position and direction are measured. | “Move right” is meaningless until we know right relative to the robot base, camera, gripper, or room. | Required when moving from joint space to task/Cartesian space. |
| Rigid-body pose | A link's position and orientation relative to a reference frame. Position may use `(x, y, z)`; orientation may use a quaternion, rotation matrix, or Euler angles. | This is the formally precise robotics meaning of pose. It lets us reason about where the gripper is rather than only how each joint is bent. | A task-space representation above layer 3; not calculated by this module. |
| Forward kinematics | The geometry calculation that turns joint coordinates into an estimated link or gripper pose. | Six joint numbers do not directly tell us where the gripper is in the room. Kinematics supplies that bridge. | Between layer 3 joint space and task-space reasoning. |
| Robot state | The information needed to describe the system at an instant: positions and potentially velocity, effort, sensors, and timestamps. | A position alone may not reveal whether the robot is moving quickly, loaded, delayed, or unstable. Policies and evaluators often need more than pose. | Layer 3 representation and layer 7 recording. |
| Observation | Information presented to a policy, such as camera images and robot state. | A learned policy cannot respond to information it never receives. Observation design determines what the policy can condition upon. | Input to layers 1-2 and stored in layer 8 datasets. |
| Action | A proposed change or target sent downstream, such as six desired joint coordinates. | This is the output interface connecting human teleoperation, replay, or a learned policy to the robot-control stack. | Produced in layer 2, translated in layer 4, executed in layer 5. |
| Action chunk | Several future actions predicted together as a short sequence. | Many VLAs generate a small plan rather than deciding only one instant at a time. Every element still needs embodiment, safety, and execution logic. | Layer 2 learned-policy output, consumed downstream. |
| Policy | A rule or learned model that maps observations to actions. | The leader supplies actions today; a trained policy can later replace that source without replacing every deterministic layer below it. | Layer 2 and the learning loop in layer 8. |
| VLM | A vision-language model that connects images and language to produce semantic representations or language outputs. | It can help understand the scene or instruction, but by itself does not define safe motor commands. | Layer 1. |
| VLA | A vision-language-action model that uses visual, linguistic, and often robot-state inputs to predict robot actions. | It adds a learned action source, but still relies on calibration, mapping, command handling, hardware checks, and evaluation. | Layers 1-2, trained and improved in layer 8. |

## Glossary: calibration and mapping language

| Term | Explain it like I am ten | Why it matters | Where it sits |
|---|---|---|---|
| Encoder | A joint sensor that reports a numerical position. | The controller needs feedback about what the joint actually did, not only what it was told to do. | Physical measurement in layer 6, reported to layers 3 and 7. |
| Calibration | Establishing how raw device readings correspond to a useful coordinate range and center. | Two mechanically similar arms may report different raw counts in the same-looking arrangement. Calibration makes each arm internally interpretable. | Precondition for layers 3-7. |
| Normalization | Converting device-specific values into a shared numerical range, such as `-100` to `100`. | It makes comparisons and model interfaces easier, but does not prove that identical numbers mean identical physical poses across two arms. | Layer 3 representation. |
| Leader | The passive arm the human moves to express intended motion. | It is today's action source and the source of demonstration targets. | Layer 2. |
| Follower | The powered arm that receives targets and acts in the environment. | It is where proposed numbers become physical consequences. | Layers 5-6. |
| Anchor | A fixed reference captured at the beginning of a mapping run. | Recomputing from fixed anchors prevents one cycle's tracking miss from becoming the next cycle's starting point. | Layer 3 input to layer 4. |
| Leader start | The leader's joint configuration captured when a relative-mapping run begins. | It defines zero movement for that run. Every leader delta is measured against it. | Layer 3 anchor used by layer 4. |
| Operational home | A collision-checked, torque-held follower configuration used as the follower anchor and return point. | Gravity can collapse an unpowered arm, and a calibration midpoint is only a coordinate. Operational home gives the live controller a usable reference. | Layers 3, 5, and 6. |
| Delta | Change from an anchor: `leader_now - leader_start`. | Relative mapping cares about what the human changed, not whether the two arms began at numerically identical coordinates. | Core intermediate value in layer 4. |
| Sign | Direction correspondence. `+1` preserves numerical direction; `-1` reverses it. | If two joints count opposite physical directions, omitting the sign makes the follower move the wrong way. | Per-joint configuration in layer 4. |
| Gain | Movement scale. A gain of `0.5` maps a 20-point leader change to a 10-point follower change. | It corrects systematic differences in movement magnitude between source and follower. | Per-joint configuration in layer 4. |
| Offset | A fixed correction added to every mapped result after anchoring. | It corrects a consistent target displacement that remains the same across poses. It does not fix a scale error that grows with movement. | Per-joint configuration in layer 4. |
| Affine mapping | A scale-and-shift rule: `home + offset + sign × gain × delta`. | It is the simplest useful correction for consistent per-joint direction, scale, and displacement differences. | Main transform in layer 4. |
| Raw target | The target proposed by the mapping formula before absolute bounds. | Preserving it shows what the mapper wanted and lets us diagnose how much a safety boundary changed the command. | Layer 4 output evidence. |
| Bounds | The permitted numerical minimum and maximum for one joint target. | They prevent the mapper from proposing known-out-of-range values. They do not reason about collisions between joints or the table. | Layer 4 numerical guardrail. |
| Clamp | Replace an out-of-bounds value with the nearest permitted endpoint. | It keeps the downstream target inside a configured interval rather than passing an extreme value unchanged. | Operation in layer 4. |
| Saturation | The raw target exceeded a bound and was clamped. | Heavy saturation means the follower is no longer faithfully imitating the leader, even if the program runs successfully. | Layer 4 receipt used by layer 7 evaluation. |
| Target | The coordinate the follower is being asked to reach. | It is the contract between mapping and command code. A target is an intention, not proof of movement. | Output of layer 4; input to layer 5. |
| Measured position | The encoder coordinate actually reported by the follower. | It lets us distinguish “we sent a number” from “the robot reached the number.” | Layer 6 feedback consumed by layer 7. |
| Signed error | `target - measured`. Positive and negative retain direction instead of reporting only distance. | It reveals whether the follower ended numerically below or above the requested target; absolute error alone loses that information. | Layer 7 evaluation. |
| Tracking error | The difference between the commanded value and measured value during or after motion. | It quantifies physical following quality and may expose lag, load, calibration, or controller limitations. | Layers 5-7. |

## Glossary: Python and software-design language

| Term | Explain it like I am ten | Why it matters | Where it sits |
|---|---|---|---|
| Module | One `.py` file that groups related names. | A clear module boundary keeps pure mapping separate from commands and hardware side effects. | `so101_mapping.py` is the layer-4 module. |
| Function | A named operation that receives inputs and returns an output. | Functions let us isolate, reuse, and test one rule instead of hiding it inside a long script. | `clamp`, `map_scalar_relative`, `map_pose_relative`, and `final_pose_error`. |
| Pure function | A function whose result depends only on its inputs and which changes no outside state. | We can run it repeatedly with pretend data without moving hardware, touching files, or depending on timing. | Core design rule for layer 4. |
| Side effect | A change outside a function's returned value, such as enabling torque, writing a file, or sending a motor command. | Side effects are where real-world risk and nondeterminism enter. Keeping them in separate layers makes them easier to gate. | Deliberately excluded from this module; required in layers 5 and 7. |
| Class | A definition for objects that share named data and behavior. | It gives related values a stable identity and vocabulary rather than passing anonymous tuples or loose dictionaries. | `JointMapping` and `JointTarget`. |
| Object | One concrete instance created from a class. | Each joint gets its own configuration object and each mapped result becomes a target object. | Runtime values inside layer 4. |
| Dataclass | A Python class intended primarily to hold named data, with routine methods generated automatically. | It reduces boilerplate while keeping fields explicit, typed, printable, and comparable in tests. | Both value classes in the mapping module. |
| Immutable | Unable to change after creation. | A `frozen=True` mapping cannot silently change halfway through a control run, which protects repeatability. | `JointMapping` and `JointTarget` contract. |
| Type hint | A declaration of the intended input or output type. | Editors and readers can detect mismatched expectations earlier, although Python usually does not enforce hints by itself at runtime. | Every public function and dataclass field. |
| `Mapping` | A read-only dictionary-like interface supporting keyed lookup. | The function promises only to read caller-owned data and accepts more than one concrete dictionary implementation. | Input types for joint-name-to-value collections. |
| `dict` | Python's concrete mutable key-value container. | The function constructs and owns a new result dictionary keyed by joint name. | Return type of mapping and error functions. |
| Keyword-only argument | An argument that must be supplied by name, such as `leader_now=35`. | It prevents easy-to-miss ordering mistakes among several similar floating-point values. | Public function signatures after `*`. |
| Contract | Observable behavior that callers may rely upon. | It lets implementation change internally without breaking command code or tests, as long as the promised inputs, failures, and outputs remain stable. | The boundary between modules and layers. |
| Validation | Checking that inputs satisfy the contract before calculating. | Missing joints, infinite values, or reversed bounds should fail loudly before producing plausible but unusable robot targets. | First stage inside layer 4. |
| Exception | Python's structured way to stop normal execution and report an invalid condition. | An explicit failure is safer and easier to diagnose than returning an empty or partially valid target set. | `ValueError` for invalid mapping or measurement input. |
| `NotImplementedError` | An exception announcing that a promised behavior has intentionally not been written yet. | It keeps unfinished code from masquerading as success during a red-to-green implementation loop. | Historical scaffold removed after both functions were completed. |
| Helper function | A small function serving a larger operation. | Naming one reusable rule, such as finite validation or clamping, keeps the main transform readable and reduces duplicated mistakes. | `require_finite` and `clamp`. |

## Glossary: testing and learning language

| Term | Explain it like I am ten | Why it matters | Where it sits |
|---|---|---|---|
| Unit test | A small automated experiment for one piece of behavior using controlled values. | It verifies mapping logic without USB devices, torque, timing, or physical risk. | Tests layer 4 in isolation. |
| Integration test | A test of several real components working together. | Pure mapping tests cannot reveal broken imports, command handoffs, serial communication, or logging coordination. | Later tests across layers 4, 5, and 7. |
| Hardware test | A bounded experiment involving the physical robot. | Only hardware can establish actual movement and tracking, but it should come after cheaper deterministic tests pass. | Layers 5-7. |
| Assertion | A statement of the expected result inside a test. | It turns “looks reasonable” into a precise pass/fail contract. | The final check inside each unit test. |
| Arrange, act, assert | Prepare inputs, call the behavior, then inspect the result. | This structure makes each test readable as a miniature experiment rather than a pile of setup and checks. | Test-file organization. |
| Test oracle | The trusted expected answer against which output is compared. | A test is only useful if its expected answer is correct. Our manually derived `+5` example is a small oracle. | Assertions and worked examples. |
| Red | A test fails for the expected missing or incorrect behavior. | It proves the test can detect the gap before we write the implementation. | First completion-problem stage. |
| Green | The smallest correct implementation makes the relevant tests pass. | It provides evidence that the written behavior satisfies the stated examples and edge cases. | Second completion-problem stage. |
| Refactor | Improve code structure without changing observable behavior. | Passing tests let us reorganize confidently while preserving the public contract. | Third completion-problem stage. |
| Edge case | An unusual but validly anticipated input near a boundary or failure condition. | Robots often fail at extremes rather than ordinary values. Explicit edge tests prevent the happy path from becoming the whole definition of correctness. | Validation and saturation tests. |
| Test coverage | Which code paths or behaviors the tests exercise. | Coverage shows what was visited, not whether every expectation is correct or whether physical behavior is safe. | Evidence about the test suite, not proof of the whole system. |
