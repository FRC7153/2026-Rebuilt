"""Watches the first seconds of teleop for "the robot won't drive" and says why.

If the driver is pushing the stick but the drivetrain is not responding, this
prints ONE line to the driver station console naming the cause it can see:

  * another command still owns the drivetrain (e.g. a leftover auto command)
  * the robot loop is stalled / overrunning (code is too slow or blocked)
  * the roboRIO is browned out (battery sag - motors are disabled by the RIO)
  * none of the above: the code IS commanding the wheels, so the problem is
    downstream of the robot code (CAN bus, motor controllers, power)

After the watch window it always prints a one-line summary, so every match
leaves evidence in the driver station log even when nothing went wrong.
"""

from typing import Callable

from commands2 import Subsystem
from wpilib import RobotController, Timer

from util import console_logger


class HandoffMonitor:
    WATCH_SECONDS = 6.0
    STICK_THRESHOLD = 0.5  # fraction of full stick that counts as "trying to drive"
    RESPONSE_THRESHOLD = 0.25  # m/s of commanded wheel speed that counts as "responding"
    SLOW_LOOP = 0.06  # seconds; three missed 20 ms loops

    def __init__(
        self,
        drivetrain: Subsystem,
        stick_magnitude: Callable[[], float],
        commanded_speed: Callable[[], float],
    ) -> None:
        """
        :param drivetrain: The drive subsystem (used to see which command owns it).
        :param stick_magnitude: Returns how hard the driver is pushing the sticks, 0..1.
        :param commanded_speed: Returns the fastest commanded swerve module speed, m/s.
        """
        self._drivetrain = drivetrain
        self._stick_magnitude = stick_magnitude
        self._commanded_speed = commanded_speed
        self._default_command_name: str | None = None

        self._active = False
        self._start = 0.0
        self._last_loop = 0.0
        self._worst_loop = 0.0
        self._min_voltage = 99.0
        self._browned_out = False
        self._first_stick: float | None = None
        self._first_response: float | None = None
        self._blocker: str | None = None
        self._reported = False

    def start(self) -> None:
        """Call from teleop_init."""
        default = self._drivetrain.get_default_command()
        self._default_command_name = default.get_name() if default is not None else None

        self._active = True
        self._start = Timer.get_timestamp()
        self._last_loop = self._start
        self._worst_loop = 0.0
        self._min_voltage = 99.0
        self._browned_out = False
        self._first_stick = None
        self._first_response = None
        self._blocker = None
        self._reported = False

    def stop(self) -> None:
        """Call from teleop_exit."""
        self._active = False

    def update(self) -> None:
        """Call once per robot loop (cheap; does nothing outside the watch window)."""
        if not self._active:
            return

        now = Timer.get_timestamp()
        elapsed = now - self._start

        self._worst_loop = max(self._worst_loop, now - self._last_loop)
        self._last_loop = now
        self._min_voltage = min(self._min_voltage, RobotController.get_battery_voltage())
        self._browned_out = self._browned_out or RobotController.is_browned_out()

        pushing = self._stick_magnitude() > self.STICK_THRESHOLD
        if pushing and self._first_stick is None:
            self._first_stick = elapsed

        if pushing and self._first_response is None:
            owner = self._drivetrain.get_current_command()
            owner_name = owner.get_name() if owner is not None else None

            if self._commanded_speed() > self.RESPONSE_THRESHOLD:
                self._first_response = elapsed
            elif owner_name != self._default_command_name:
                self._blocker = owner_name or "nothing (default command not running)"

            # Still not responding 0.25 s after the driver started pushing: say why, once
            if self._first_response is None and not self._reported and elapsed - self._first_stick > 0.25:
                self._reported = True
                console_logger.report_warning(f"[handoff] Driver is pushing the stick but {self._reason()}")

        if elapsed >= self.WATCH_SECONDS:
            self._active = False
            print(self._summary())

    def _reason(self) -> str:
        if self._blocker is not None:
            return f"the drivetrain is owned by '{self._blocker}' instead of the drive command."
        if self._browned_out:
            return f"the roboRIO is BROWNED OUT (battery dipped to {self._min_voltage:.1f} V); motors are disabled."
        if self._worst_loop > self.SLOW_LOOP:
            return f"the robot loop stalled for {self._worst_loop * 1000:.0f} ms."
        return "the drive command is running and still commanding ~0 m/s (check deadband/joystick mapping)."

    def _summary(self) -> str:
        if self._first_stick is None:
            response = "driver never pushed the stick"
        elif self._first_response is None:
            response = "DRIVETRAIN NEVER RESPONDED"
        else:
            response = f"drive responded {max(0.0, self._first_response - self._first_stick) * 1000:.0f} ms after the stick"
        return (
            f"[handoff] first {self.WATCH_SECONDS:.0f} s of teleop: {response}; "
            f"worst loop {self._worst_loop * 1000:.0f} ms; "
            f"min battery {self._min_voltage:.1f} V; "
            f"brownout {'YES' if self._browned_out else 'no'}"
            + (f"; drivetrain was held by '{self._blocker}'" if self._blocker else "")
        )
