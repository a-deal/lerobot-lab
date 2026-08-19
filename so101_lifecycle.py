"""Own software lifecycle obligations for one SO-101 arm.

Call chain: teleoperation runner -> ArmLifecycle -> LeRobot bus.

This module owns goal-alignment and torque transitions for one arm. It tracks
what cleanup software must attempt without claiming that torque was physically
enabled successfully or that the arm occupies a safe pose. The runner creates
one lifecycle per arm and retains multi-arm orchestration, receipt policy, and
operator interaction.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class ArmLifecycle:
    """Track one arm's software cleanup obligation."""

    name: str
    bus: Any
    joints: tuple[str, ...]

    torque_cleanup_required: bool = field(default=False, init=False)

    def align_goals_and_enable_torque(self, pose: dict[str, float]) -> None:
        """Align goals before attempting torque enable.

        What: Validate the pose, write it as the goal, record the cleanup
        obligation, and then request torque enable.

        Why: A torque-enable call may partially affect hardware before raising.
        The cleanup obligation must therefore exist before that call begins.
        """

        if set(pose) != set(self.joints):
            raise ValueError(
                f"{self.name} torque alignment requires one value for every joint"
            )

        self.bus.sync_write("Goal_Position", pose)
        self.torque_cleanup_required = True
        self.bus.enable_torque(list(self.joints))

    def disable_torque_if_required(self) -> None:
        """Disable torque when software currently owes that cleanup."""

        if not self.torque_cleanup_required:
            return

        self.bus.disable_torque(list(self.joints))
        self.torque_cleanup_required = False
