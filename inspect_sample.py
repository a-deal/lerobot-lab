"""Look at ONE training sample from the PushT dataset, the atom of imitation learning.

Run:  .venv/bin/python inspect_sample.py

What you're looking at when it prints:
- observation.image  -> what the camera saw at one instant (a 96x96 RGB frame)
- observation.state  -> where the robot was (x, y position of the pusher)
- action             -> what the human demonstrator did next (target x, y)

That triple, (what I see, where I am) -> (what to do), repeated 25,650 times
across 206 human demonstrations, is the ENTIRE training signal. The policy is
just a neural net learning this mapping. Everything else is plumbing.
"""
from lerobot.datasets.lerobot_dataset import LeRobotDataset

ds = LeRobotDataset("lerobot/pusht")
print(f"Dataset: {ds.num_episodes} episodes (demonstrations), {ds.num_frames} frames total")
print(f"Recorded at {ds.fps} frames per second\n")

sample = ds[500]  # one instant, mid-demonstration
print("ONE SAMPLE (frame 500):")
for key, val in sample.items():
    if hasattr(val, "shape") and val.numel() > 1:
        print(f"  {key:24s} shape={tuple(val.shape)}  dtype={val.dtype}")
    else:
        print(f"  {key:24s} value={val}")

print("\nThe state vector (where the pusher is):", sample["observation.state"].tolist())
print("The action (where the human moved next):", sample["action"].tolist())
print("\nThat's it. See -> decide -> move, learned from 206 examples of a human doing it.")
