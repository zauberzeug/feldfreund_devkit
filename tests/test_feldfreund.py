import pytest

from feldfreund_devkit.config import RobotBrainConfiguration


def test_robot_brain_rejects_unsupported_baud_rate() -> None:
    RobotBrainConfiguration(name='rb', serial_baud_rate=921600)
    with pytest.raises(ValueError):
        RobotBrainConfiguration(name='rb', serial_baud_rate=912600)  # type: ignore[arg-type]
