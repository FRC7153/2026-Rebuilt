"""Auxiliary Elastic utilities, and a match timer that counts down during real
matches and counts up during practice.
"""

import os

import ntcore
import wpilib
from commands2 import Command, CommandScheduler, InstantCommand
from commands2.button import CommandGenericHID
from wpilib import Alert, DriverStation, SmartDashboard, Timer

from constants import BuildConstants, DashboardConstants


class Dashboard:
    def __init__(self, *controllers: CommandGenericHID) -> None:
        """
        :param controllers: Controllers to check
        """
        self._match_time_pub = (
            ntcore.NetworkTableInstance.getDefault().getDoubleTopic("Elastic/matchTime").publish()
        )
        self._match_timer = Timer()

        self._controllers = controllers
        self._controller_alerts = [
            Alert(f"Controller {c} not connected", Alert.AlertType.kError) for c in range(len(controllers))
        ]

        # Host Elastic configuration file
        if BuildConstants.PUBLISH_EVERYTHING:
            import wpinet

            wpinet.WebServer.getInstance().start(
                DashboardConstants.ELASTIC_SERVER_PORT,
                os.path.join(wpilib.getDeployDirectory(), "Elastic"),
            )
            print("Began hosting Elastic's web server")
        else:
            print("Did NOT host Elastic's web server")

        # Stop the timer (the robot is disabled by default)
        self._match_timer.stop()
        self._match_timer.reset()

        # Begin hosting CommandScheduler for debugging
        if BuildConstants.PUBLISH_EVERYTHING:
            SmartDashboard.putData("Commands", CommandScheduler.getInstance())

    def get_restart_timer_command(self) -> Command:
        """A command to restart the timer. Run when the robot is enabled."""
        return InstantCommand(self._match_timer.restart).ignoringDisable(True)

    def get_stop_timer_command(self) -> Command:
        """A command to stop the timer. Run when the robot is disabled."""
        return InstantCommand(self._match_timer.stop).ignoringDisable(True)

    def set_all_rumble(self, rumble_type: wpilib.interfaces.GenericHID.RumbleType, value: float) -> None:
        """Sets the rumble of all controllers."""
        for controller in self._controllers:
            controller.setRumble(rumble_type, value)

    def update(self) -> None:
        mt = Timer.getMatchTime()

        if mt < 0 and not DriverStation.isFMSAttached():
            # No FMS match time, count up instead of down
            self._match_time_pub.set(self._match_timer.get())
        else:
            # Use real match time
            self._match_time_pub.set(mt)

    def check_hardware(self) -> None:
        for controller, alert in zip(self._controllers, self._controller_alerts):
            alert.set(not controller.isConnected())
