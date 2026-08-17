# lerobot-lab

Home base for the robot-learning ramp (see hub/projects/job-search/will-thesis-vla-primer-2026-07-16.md).

Set up 2026-07-16: Python 3.12 venv (via uv), lerobot==0.4.1 pinned, torch 2.7.1, MPS verified on the M4 Pro. PushT dataset access verified (206 episodes, 25,650 frames).

## First fine-tune (the Saturday command)

Trains ACT (the standard imitation-learning policy) on PushT, the classic "push the T-block into place" benchmark. This is the full loop minus the physical robot: demos in, policy out.

```bash
cd ~/src/lerobot-lab
./run-first-train.sh
```

Expect it to be slow on MPS. The point is not a SOTA policy, it's watching loss fall on real demonstration data and understanding every flag. Kill it after a few thousand steps if you want; checkpoints land in outputs/.

## After PushT works
1. Swap `--policy.type=diffusion` and compare. Feel the architecture debate from section 3 of the primer.
2. When the SO-101 arms arrive: teleop, record your OWN dataset, train on it. That's the loop that matters.

## Notes
- venv: `source .venv/bin/activate` (or call `.venv/bin/python` directly)
- torchcodec needs ffmpeg 4-7; system has 8. Fixed via keg-only `ffmpeg@7` + `DYLD_FALLBACK_LIBRARY_PATH` (baked into run-first-train.sh). The objc "AVFAudioReceiver implemented in both" warning at startup is harmless.
- Pipeline verified 7/16: 2-step smoke run, loss 106→81, checkpoint written. ~0.08s/step on MPS after warmup.
- GPU box in the apartment: graduate there for anything bigger than PushT.
