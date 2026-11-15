"""Tracks whether a hardware fault ever occurs."""

from wpilib import Alert, Timer

_has_fault_occurred_alert: Alert | None = None

# Don't track faults from before the robot program has started booting
_robot_program_start_time = -1.0


def robot_program_has_started() -> None:
    """Starts tracking faults."""
    global _robot_program_start_time, _has_fault_occurred_alert
    _robot_program_start_time = Timer.getFPGATimestamp()
    if _has_fault_occurred_alert is None:
        _has_fault_occurred_alert = Alert(
            "A hardware fault has been reported. Check logs.", Alert.AlertType.kInfo
        )
    print(f"Hardware fault tracker has started at {_robot_program_start_time:f}.")


def check_fault(alert: Alert, fault: bool) -> None:
    """If a fault has occurred, sets the given alert and a global persistent alert.

    :param alert: The alert specific to the hardware being checked.
    :param fault: Whether a fault has occurred.
    """
    alert.set(fault)

    # Only latch the global alert once the robot program has been running for 10 seconds
    if (
        fault
        and _has_fault_occurred_alert is not None
        and _robot_program_start_time != -1.0
        and Timer.getFPGATimestamp() - _robot_program_start_time >= 10.0
    ):
        _has_fault_occurred_alert.set(True)
