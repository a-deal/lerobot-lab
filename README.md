# LeRobot Lab

A hands-on robotics learning and evaluation repository built around LeRobot,
PushT, and a paired SO-101 leader/follower setup. The current emphasis is not
just making the arm move: it is separating coordinate mapping, safety bounds,
hardware commands, and observed fidelity into inspectable contracts.

## Current evidence

- Python 3.12 environment with `lerobot==0.4.1` and PyTorch 2.7.1.
- PushT dataset inspection and a local 5,000-step ACT training run.
- A persistent six-joint SO-101 relative-teleoperation harness.
- A hardware-free leader-to-follower mapping module with eleven passing tests.

Generated checkpoints, videos, datasets, logs, and local environments are
deliberately excluded from Git.

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

- `so101_mapping.py`: pure coordinate mapping and evaluation contract.
- `tests/test_so101_mapping.py`: hardware-free unit and six-joint contracts.
- `so101_pose_fidelity.py`: live harness; integration with the pure mapper is
  the next refactor.
- `inspect_sample.py`: inspect one PushT imitation-learning sample.
- `run-first-train.sh`: local ACT training command.
- `docs/`: concept and implementation walkthroughs.

## Notes
- venv: `source .venv/bin/activate` (or call `.venv/bin/python` directly)
- torchcodec needs ffmpeg 4-7; system has 8. Fixed via keg-only `ffmpeg@7` + `DYLD_FALLBACK_LIBRARY_PATH` (baked into run-first-train.sh). The objc "AVFAudioReceiver implemented in both" warning at startup is harmless.
- Pipeline verified 7/16: 2-step smoke run, loss 106→81, checkpoint written. ~0.08s/step on MPS after warmup.
- The live harness currently contains machine-specific development defaults.
  Supply and verify the correct ports, calibration identifiers, bounds, and
  physical setup before any hardware run.
