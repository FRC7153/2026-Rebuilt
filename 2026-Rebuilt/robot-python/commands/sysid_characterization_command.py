"""Runs a SysId routine, but only if the FMS is NOT connected."""

from commands2 import ConditionalCommand, PrintCommand
from commands2.sysid import SysIdRoutine
from wpilib import RobotState


class SysIdCharacterizationCommand(ConditionalCommand):
    def __init__(self, routine: SysIdRoutine, quasistatic: bool, direction: bool) -> None:
        """
        :param routine: The SysId routine to run.
        :param quasistatic: True for quasistatic, False for dynamic.
        :param direction: True for forward, False for backward.
        """
        sysid_direction = SysIdRoutine.Direction.FORWARD if direction else SysIdRoutine.Direction.REVERSE
        super().__init__(
            # FMS is connected, do not characterize:
            PrintCommand("Will not characterize while FMS is connected!"),
            # FMS is not connected, run characterization (quasistatic or dynamic):
            routine.quasistatic(sysid_direction) if quasistatic else routine.dynamic(sysid_direction),
            # Check if FMS is connected
            RobotState.is_fms_attached,
        )
        self.set_name(f"SysID(quasistatic: {quasistatic}, direction: {direction})")
