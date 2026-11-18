"""Intake commands (ports of DeployIntake/HoldIntake/HomeIntake/Intake/IntakePivot commands)."""

from commands2 import Command

from subsystems.intake import Intake


class DeployIntakeCommand(Command):
    """Moves the pivot to a position and runs the rollers until interrupted.

    roller_speed is interpreted by magnitude:
      * |roller_speed| <= 1.0 -> duty cycle (e.g. -0.6 = 60% inwards)
      * otherwise             -> closed-loop velocity in RPM (e.g. -6000)

    (The Java code sent every value to the velocity controller, so the autos that
    asked for "-0.6" were really asking for -0.6 RPM and the rollers never spun.)
    """

    def __init__(self, intake: Intake, pivot_position: float, roller_speed: float) -> None:
        super().__init__()
        self._intake = intake
        self._pivot_position = pivot_position
        self._roller_speed = roller_speed
        self.add_requirements(intake)

    def initialize(self) -> None:
        self._intake.set_and_remember_position(self._pivot_position)

        if self._roller_speed == 0.0:
            self._intake.stop_intake()
        elif abs(self._roller_speed) <= 1.0:
            self._intake.set_intake_speed(self._roller_speed)
        else:
            self._intake.set_intake_velocity(self._roller_speed)

    def end(self, interrupted: bool) -> None:
        self._intake.stop_intake()  # always stop rollers

    def is_finished(self) -> bool:
        return False  # runs until interrupted


class HoldIntakeCommand(Command):
    """Default intake command: keeps holding the last commanded pivot position."""

    def __init__(self, intake: Intake) -> None:
        super().__init__()
        self._intake = intake
        self.add_requirements(intake)

    def execute(self) -> None:
        self._intake.hold_last_position()  # runs every loop

    def is_finished(self) -> bool:
        return False  # runs forever as default


class HomeIntakeCommand(Command):
    """Slowly drives the pivot into its hard stop and zeroes the encoder there."""

    HOMING_OUTPUT = -0.07
    CURRENT_THRESHOLD = 20.0
    CURRENT_SPIKE_LOOPS = 5

    def __init__(self, intake: Intake) -> None:
        super().__init__()
        self._intake = intake
        self._spike_count = 0
        self._has_homed = False
        self.add_requirements(intake)

    def initialize(self) -> None:
        self._spike_count = 0
        self._has_homed = False
        self._intake.set_intake_pivot_voltage(self.HOMING_OUTPUT * 12.0)

    def execute(self) -> None:
        current = abs(self._intake.get_pivot_stator_current())

        if current > self.CURRENT_THRESHOLD:
            self._spike_count += 1
        else:
            self._spike_count = 0

        if self._spike_count >= self.CURRENT_SPIKE_LOOPS and not self._has_homed:
            self._intake.zero_pivot_encoder()
            self._has_homed = True
            print("Homed Intake Pivot")

    def is_finished(self) -> bool:
        return self._has_homed

    def end(self, interrupted: bool) -> None:
        self._intake.set_intake_pivot_voltage(0.0)
        if not interrupted:
            print("Homing complete")


class IntakeCommand(Command):
    """Runs the rollers open-loop (duty cycle) until interrupted."""

    def __init__(self, intake: Intake, speed: float) -> None:
        super().__init__()
        self._intake = intake
        self._speed = speed
        self.add_requirements(intake)

    def initialize(self) -> None:
        self._intake.set_intake_speed(self._speed)

    def end(self, interrupted: bool) -> None:
        self._intake.stop_intake()  # always stop rollers when command ends

    def is_finished(self) -> bool:
        return False  # runs until interrupted


class IntakePivotCommand(Command):
    """Moves the pivot to a position and finishes once it gets there."""

    def __init__(self, intake: Intake, target_position: float) -> None:
        super().__init__()
        self._intake = intake
        self._target_position = target_position
        self.add_requirements(intake)

    def initialize(self) -> None:
        self._intake.set_and_remember_position(self._target_position)

    def is_finished(self) -> bool:
        return abs(self._intake.get_intake_pivot_position() - self._target_position) < 0.05
