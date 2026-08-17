# LeRobot Lab

A hands-on robotics learning and evaluation repository built around LeRobot,
PushT, and a paired SO-101 leader/follower setup. The current emphasis is not
just making the arm move: it is separating coordinate mapping, safety bounds,
hardware commands, and observed fidelity into inspectable contracts.

## Start here: what kind of repository is this?

This is a **lab containing several small workflows**, not one application with
one universal `main.py`.

In Python, `main.py` is a convention, not a requirement. Any file can be an
executable entrypoint. `so101_pose_fidelity.py` contains its own `main()` and
can be executed directly. `inspect_sample.py` is another standalone script.
`so101_mapping.py` is different: it is a library module imported by other code
and cannot run hardware.

The training workflow begins with `run-first-train.sh`. That file is a shell
script, not Python. It configures the local environment and calls
`.venv/bin/lerobot-train`, an executable command installed by the LeRobot
package. The training implementation itself therefore lives upstream in
LeRobot; this repository owns the selected arguments and resulting evidence.

## Repository mental model

There are currently two independent learning lanes:

```text
Lane A: imitation learning without physical hardware

inspect_sample.py
  -> loads one PushT dataset sample through LeRobot
  -> shows observation, state, and demonstrated action

run-first-train.sh
  -> configures ffmpeg and MPS for this Mac
  -> invokes the installed lerobot-train CLI
  -> writes generated checkpoints and logs under outputs/


Lane B: SO-101 leader/follower measurement

so101_pose_fidelity.py                    executable experiment entrypoint
  -> imports so101_mapping.py             pure translation library
  -> imports LeRobot SO-101 drivers       hardware communication
  -> reads leader and follower state
  -> previews and operator-gates targets
  -> rate-limits follower commands
  -> records CSV and JSON evidence

tests/test_so101_mapping.py
  -> proves pure mapping behavior with ordinary numbers

tests/test_so101_pose_fidelity.py
  -> proves the harness-to-mapper software boundary without hardware
```

The dependency direction is deliberate:

```text
SO-101 hardware harness -> pure mapping module
tests                  -> public functions in both modules

pure mapping module    -X-> hardware harness
```

The mapper must never import the harness or hardware drivers. That one-way
dependency keeps its numerical rules cheap and safe to test.

## Entrypoints and commands

| Goal | Entry point | Command | Hardware? |
|---|---|---|---|
| Inspect one PushT demonstration sample | `inspect_sample.py` | `./.venv/bin/python inspect_sample.py` | No |
| Train the first local PushT ACT policy | `run-first-train.sh` | `./run-first-train.sh` | No |
| Test the pure mapper | Python unittest discovery | `./.venv/bin/python -m unittest discover -s tests -p 'test_so101_mapping.py' -v` | No |
| Test all SO-101 software contracts | Python unittest discovery | `./.venv/bin/python -m unittest discover -s tests -p 'test_so101*.py' -v` | No |
| Smoke-test the harness integration | `so101_pose_fidelity.py` | `./.venv/bin/python so101_pose_fidelity.py --self-test` | No |
| Run the physical three-pose experiment | `so101_pose_fidelity.py` | `./.venv/bin/python so101_pose_fidelity.py` with verified ports and setup | Yes |

Do not run the physical command merely because the software tests pass. Read
the module-level safety boundary and verify the hardware setup first.

## Current evidence

- Python 3.12 environment with `lerobot==0.4.1` and PyTorch 2.7.1.
- PushT dataset inspection and a local 5,000-step ACT training run.
- A persistent six-joint SO-101 relative-teleoperation harness.
- A hardware-free leader-to-follower mapping module integrated into the live
  pose-fidelity harness.
- Thirteen passing SO-101 software-contract tests at the last committed green
  checkpoint: eleven mapper tests and two harness-boundary tests.

Generated checkpoints, videos, datasets, logs, and local environments are
deliberately excluded from Git.

## Current SO-101 migration

The pure mapper is now connected to the hardware harness. The migration is
deliberately staged:

1. **Complete:** isolate and test leader-to-follower mapping.
2. **Complete:** make the legacy harness call that mapper.
3. **Complete:** expose `dict[str, JointTarget]` as the clean harness boundary.
4. **In progress:** migrate preview, motion, logging, and evaluation consumers
   away from three parallel dictionaries.
5. **Pending:** delete the legacy adapter after all consumers and replacement
   tests are green.
6. **Pending:** run and evaluate the bounded physical three-pose experiment.

The immediate exercise migrates only the no-motion preview. Keeping motion and
logging unchanged gives each contract change one clear failure surface.

## Pure SO-101 mapping layer

`so101_mapping.py` translates leader movement into bounded follower targets
without importing hardware drivers or producing side effects. It validates
joint rosters and numeric inputs, applies per-joint sign/gain/offset mapping,
reports clamping, and calculates signed final-pose error.

Run its contract suite from the repository root:

```bash
./.venv/bin/python -m unittest discover -s tests -p 'test_so101_mapping.py' -v
```

Passing these tests proves the pure numerical contract, not physical safety,
calibration correctness, motor communication, collision avoidance, or tracking
fidelity. Those remain separate integration and physical-evaluation gates.

The companion walkthrough is in `docs/so101-mapping-learning-guide.md`.

## PushT fine-tune

`run-first-train.sh` trains ACT on PushT, the classic "push the T-block into
place" benchmark. This exercises the imitation-learning loop without physical
hardware: demonstrations in, policy out.

```bash
./run-first-train.sh
```

Expect it to be slow on MPS. The point is not a SOTA policy, it's watching loss fall on real demonstration data and understanding every flag. Kill it after a few thousand steps if you want; checkpoints land in outputs/.

## Repository layout

- `so101_mapping.py`: importable pure coordinate-mapping and evaluation
  library; never commands hardware.
- `so101_pose_fidelity.py`: executable, operator-gated physical experiment and
  hardware-free self-test; contains the SO-101 workflow's `main()`.
- `tests/test_so101_mapping.py`: pure mapping unit and six-joint contracts.
- `tests/test_so101_pose_fidelity.py`: hardware-free harness integration and
  migration contracts.
- `inspect_sample.py`: standalone PushT dataset-inspection entrypoint.
- `run-first-train.sh`: shell entrypoint that invokes LeRobot's training CLI.
- `docs/`: deeper concept and implementation reference; modules and this
  README should remain understandable without opening it first.
- `outputs/`: generated training or hardware evidence, ignored by Git.

## Reading order for the current SO-101 work

1. Read this README for repository and workflow taxonomy.
2. Read the top of `so101_pose_fidelity.py` for the live experiment boundary
   and migration status.
3. Read the top of `so101_mapping.py` for the pure translator contract.
4. Read `tests/test_so101_pose_fidelity.py` to see the current handoff examples.
5. Read `tests/test_so101_mapping.py` for the mapper's numerical edge cases.
6. Use `docs/so101-mapping-learning-guide.md` only when deeper terminology or
   historical implementation sequence is useful.

## Notes
- venv: `source .venv/bin/activate` (or call `.venv/bin/python` directly)
- torchcodec needs ffmpeg 4-7; system has 8. Fixed via keg-only `ffmpeg@7` + `DYLD_FALLBACK_LIBRARY_PATH` (baked into run-first-train.sh). The objc "AVFAudioReceiver implemented in both" warning at startup is harmless.
- Pipeline verified 7/16: 2-step smoke run, loss 106→81, checkpoint written. ~0.08s/step on MPS after warmup.
- The live harness currently contains machine-specific development defaults.
  Supply and verify the correct ports, calibration identifiers, bounds, and
  physical setup before any hardware run.
