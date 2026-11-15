"""Runs a SysId routine, but only if the FMS is NOT connected."""

from commands2 import ConditionalCommand, PrintCommand
from commands2.sysid import SysIdRoutine
from wpilib import DriverStation


class SysIdCharacterizationCommand(ConditionalCommand):
    def __init__(self, routine: SysIdRoutine, quasistatic: bool, direction: bool) -> None:
        """
        :param routine: The SysId routine to run.
        :param quasistatic: True for quasistatic, False for dynamic.
        :param direction: True for forward, False for backward.
        """
        sysid_direction = SysIdRoutine.Direction.kForward if direction else SysIdRoutine.Direction.kReverse
        super().__init__(
            # FMS is connected, do not characterize:
            PrintCommand("Will not characterize while FMS is connected!"),
            # FMS is not connected, run characterization (quasistatic or dynamic):
            routine.quasistatic(sysid_direction) if quasistatic else routine.dynamic(sysid_direction),
            # Check if FMS is connected
            DriverStation.isFMSAttached,
        )
        self.setName(f"SysID(quasistatic: {quasistatic}, direction: {direction})")
