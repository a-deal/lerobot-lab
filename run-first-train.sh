#!/bin/bash
# First fine-tune: ACT on PushT, MPS device. Set up 2026-07-16.
set -euo pipefail
cd "$(dirname "$0")"

# torchcodec supports ffmpeg 4-7; system ffmpeg is 8. Keg-only ffmpeg@7 installed 7/16 for this.
export DYLD_FALLBACK_LIBRARY_PATH=/opt/homebrew/opt/ffmpeg@7/lib

.venv/bin/lerobot-train \
  --dataset.repo_id=lerobot/pusht \
  --policy.type=act \
  --policy.device=mps \
  --policy.push_to_hub=false \
  --batch_size=8 \
  --steps=5000 \
  --log_freq=100 \
  --save_freq=2500 \
  --output_dir=outputs/act_pusht_first
