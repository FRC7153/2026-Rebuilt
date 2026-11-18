"""Shooter commands (ports of ShootCommand and DistanceShootCommand)."""

from commands2 import Command
from wpilib import Timer

from subsystems.shooter import Shooter

# Kicker output while the flywheel is still spinning up (holds the fuel back).
# The Java code passed -4000 to a duty-cycle setter, which saturates to full reverse.
KICKER_HOLD_BACK = -1.0
# Kicker output while feeding in DistanceShootCommand (Java passed 5000 -> full forward).
KICKER_FEED = 1.0


class ShootCommand(Command):
    """Sets the shooter, kicker, and live floor speeds until interrupted.

    Also used (with all zeros) as the default command of the shooter, so that the
    shooter stops whenever nothing else is using it.

    :param shooter_velo: 0.0 - 100.0 rps
    :param kicker_speed: 0.0 - 1.0 (scaled to 0 - 10000 RPM closed-loop)
    :param live_floor_speed: -1.0 - 1.0 duty cycle
    """

    VELOCITY_TOLERANCE = 5.0  # rps

    def __init__(self, shooter: Shooter, shooter_velo: float, kicker_speed: float, live_floor_speed: float) -> None:
        super().__init__()
        self._shooter = shooter
        self._shooter_velo = shooter_velo
        self._kicker_speed = kicker_speed
        self._live_floor_speed = live_floor_speed
        self.add_requirements(shooter)

    def execute(self) -> None:
        # Always command the flywheel
        self._shooter.set_shooter_speed(self._shooter_velo)
        self._shooter.set_live_floor_speed(self._live_floor_speed)

        if self._shooter_velo <= 0.0:
            # Stopping: don't feed, and don't run the kicker backwards while the
            # flywheel spins down either.
            self._shooter.stop_kicker()
        elif abs(self._shooter.get_shooter_velocity() - self._shooter_velo) > self.VELOCITY_TOLERANCE:
            # Reverse kicker until shooter is ready
            self._shooter.set_kicker_speed(KICKER_HOLD_BACK)
        else:
            # Shooter ready -> feed
            self._shooter.set_kicker_velo(self._kicker_speed * 10000)

    def is_finished(self) -> bool:
        return False


class DistanceShootCommand(Command):
    """Spins up to the speed for the Limelight-measured distance, then feeds for 0.5 s."""

    FEED_TIME = 0.5  # seconds
    LIVE_FLOOR_SPEED = -0.7

    def __init__(self, shooter: Shooter) -> None:
        super().__init__()
        self._shooter = shooter
        self._feed_timer = Timer()
        self._feeding = False
        self.add_requirements(shooter)

    def initialize(self) -> None:
        self._feeding = False
        self._feed_timer.stop()
        self._feed_timer.reset()

    def execute(self) -> None:
        at_speed = self._shooter.set_velocity_from_limelight()
        self._shooter.set_live_floor_speed(self.LIVE_FLOOR_SPEED)

        if not self._feeding:
            # Hold back the fuel while spinning up
            self._shooter.set_kicker_speed(KICKER_HOLD_BACK)

            if at_speed:
                # Flywheel ready - start feeding
                self._feeding = True
                self._feed_timer.restart()
        else:
            # Feed the fuel through
            self._shooter.set_kicker_speed(KICKER_FEED)

    def is_finished(self) -> bool:
        # Give the fuel enough time to fully exit after feeding starts
        return self._feeding and self._feed_timer.has_elapsed(self.FEED_TIME)

    def end(self, interrupted: bool) -> None:
        self._shooter.stop_shooter_sub()
