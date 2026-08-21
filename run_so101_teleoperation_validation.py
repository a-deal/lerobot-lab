"""Run the SO-101 teleoperation validation from the CLI"""

import sys

from so101_teleoperation_validation import run_teleoperation_validation


def main() -> int:
    """Return the exit status produced by one teleoperation-validation run."""

    return run_teleoperation_validation()


if __name__ == "__main__":
    sys.exit(main())
