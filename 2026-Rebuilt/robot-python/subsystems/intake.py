"""Intake subsystem: Kraken pivot (CANivore) + two NEO Vortex rollers (rio bus)."""

import ntcore
import wpilib
from commands2 import Subsystem
from commands2.sysid import SysIdRoutine
from phoenix6 import SignalLogger, controls, hardware
from rev import PersistMode, ResetMode, SparkFlex, SparkLowLevel
from wpiutil import Alert
from wpilib.sysid import SysIdRoutineLog
from wpilog import DoubleLogEntry

from constants import BuildConstants, HardwareConstants, IntakeConstants
from util import hardware_fault_tracker
from util.setpoint_cache import SetpointCache


class Intake(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self._pivot = hardware.TalonFX(HardwareConstants.INTAKE_PIVOT_CAN, HardwareConstants.CANIVORE)
        self._intake = SparkFlex(HardwareConstants.SYSTEMCORE_CAN_PORT, HardwareConstants.INTAKE_CAN, SparkLowLevel.MotorType.BRUSHLESS)
        self._intake_follower = SparkFlex(HardwareConstants.SYSTEMCORE_CAN_PORT, HardwareConstants.INTAKE_FOLLOWER_CAN, SparkLowLevel.MotorType.BRUSHLESS
        )
        self._intake_encoder = self._intake.get_encoder()
        self._intake_controller = self._intake.get_closed_loop_controller()

        self._static_brake_request = controls.StaticBrake()
        self._voltage_request = controls.VoltageOut(0.0)
        self._pivot_motion_magic_request = (
            controls.MotionMagicVoltage(0.0).with_override_brake_dur_neutral(True).with_slot(0)
        )

        self._pivot_position = self._pivot.get_position()
        self._pivot_stator_current = self._pivot.get_stator_current()

        self._intake_alert = Alert("Intake Alert", "Intake Alert", Alert.Level.HIGH)
        self._intake_follower_alert = Alert("Intake Follower Alert", "Intake Follower Alert", Alert.Level.HIGH)
        self._intake_pivot_alert = Alert("Intake Pivot Alert", "Intake Pivot Alert", Alert.Level.HIGH)

        self._pivot_routine: SysIdRoutine | None = None

        self._last_position = 0.0

        # Only talk to a motor controller when its setpoint actually changes
        self._pivot_cache = SetpointCache()
        self._roller_cache = SetpointCache()

        # Datalog
        log = wpilib.DataLogManager.get_log()
        self._intake_velo_log = DoubleLogEntry(log, "Intake/Velo", "RPM")
        self._intake_setpoint_log = DoubleLogEntry(log, "Intake/Setpoint", "RPM")
        self._intake_pivot_setpoint_log = DoubleLogEntry(log, "Intake/Pivot Setpoint", "rots")
        self._intake_pivot_position_log = DoubleLogEntry(log, "Intake/Pivot Position", "rots")

        # Configure hardware
        self._pivot.configurator.apply(IntakeConstants.INTAKE_PIVOT_CONFIGS)
        self._intake.configure(
            IntakeConstants.intake_config(),
            ResetMode.RESET_SAFE_PARAMETERS,
            PersistMode.NO_PERSIST_PARAMETERS,
        )
        self._intake_follower.configure(
            IntakeConstants.intake_follower_config(),
            ResetMode.RESET_SAFE_PARAMETERS,
            PersistMode.NO_PERSIST_PARAMETERS,
        )

        self._pivot.set_control(self._pivot_motion_magic_request.with_position(0.0))

        # NT logging
        self._intake_velo_pub = None
        self._intake_setpoint_pub = None
        self._intake_pivot_setpoint_pub = None
        self._intake_pivot_position_pub = None
        if BuildConstants.PUBLISH_EVERYTHING:
            nt = ntcore.NetworkTableInstance.get_default().get_table("Intake")
            self._intake_velo_pub = nt.get_double_topic("IntakeVelo").publish()
            self._intake_setpoint_pub = nt.get_double_topic("IntakeSetpoint").publish()
            self._intake_pivot_setpoint_pub = nt.get_double_topic("IntakePivotSetpoint").publish()
            self._intake_pivot_position_pub = nt.get_double_topic("IntakePivotPosition").publish()

    # ---------------------------------------------------------------- rollers

    def set_intake_speed(self, speed: float) -> None:
        """Runs the rollers open-loop. speed is duty cycle, -1.0 to 1.0."""
        speed = max(-1.0, min(1.0, speed))
        if not self._roller_cache.should_send(("duty", speed)):
            return
        self._intake.set_throttle(speed)
        self._intake_setpoint_log.append(speed)
        if self._intake_setpoint_pub is not None:
            self._intake_setpoint_pub.set(speed)

    def set_intake_velocity(self, velocity: float) -> None:
        """Runs the rollers closed-loop. velocity is in RPM."""
        if not self._roller_cache.should_send(("rpm", velocity)):
            return
        self._intake_controller.set_setpoint(velocity, SparkLowLevel.ControlType.VELOCITY)
        self._intake_setpoint_log.append(velocity)
        if self._intake_setpoint_pub is not None:
            self._intake_setpoint_pub.set(velocity)

    def stop_intake(self) -> None:
        if not self._roller_cache.should_send(("stop", 0.0)):
            return
        self._intake.stop_motor()
        self._intake_setpoint_log.append(0.0)
        if self._intake_setpoint_pub is not None:
            self._intake_setpoint_pub.set(0.0)

    def get_intake_velocity(self) -> float:
        """Roller velocity in RPM."""
        return self._intake_encoder.get_velocity().get()

    # ------------------------------------------------------------------ pivot

    def _command_pivot_position(self, rotations: float) -> None:
        if not self._pivot_cache.should_send(rotations):
            return
        self._pivot.set_control(self._pivot_motion_magic_request.with_position(rotations))
        self._intake_pivot_setpoint_log.append(rotations)
        if self._intake_pivot_setpoint_pub is not None:
            self._intake_pivot_setpoint_pub.set(rotations)

    def set_intake_pivot_position(self, position: float) -> None:
        """Moves the pivot to the given position (mechanism rotations) without remembering it."""
        self._command_pivot_position(position)

    def hold_last_position(self) -> None:
        """Holds the last position given to set_and_remember_position."""
        self._command_pivot_position(self._last_position)

    def set_and_remember_position(self, rotations: float) -> None:
        """Moves the pivot to the given position and keeps holding it afterwards."""
        self._last_position = rotations
        self._command_pivot_position(rotations)

    def stop_intake_pivot(self) -> None:
        self._pivot_cache.invalidate()
        self._pivot.set_control(self._static_brake_request)

    def set_intake_pivot_voltage(self, voltage: float) -> None:
        self._pivot_cache.invalidate()
        self._pivot.set_control(self._voltage_request.with_output(voltage))

    def get_intake_pivot_position(self) -> float:
        """Pivot position in mechanism rotations."""
        return self._pivot_position.refresh().value_as_double

    def get_pivot_stator_current(self) -> float:
        """Pivot stator current in amps."""
        return self._pivot_stator_current.refresh().value_as_double

    def zero_pivot_encoder(self) -> None:
        self._pivot_cache.invalidate()
        self._pivot.set_position(0.0)

    # ------------------------------------------------------------------ sysid

    def get_intake_pivot_routine(self) -> SysIdRoutine:
        print("Starting CTRE SignalLogger due to get_intake_pivot_routine")
        SignalLogger.start()

        if self._pivot_routine is None:
            self._pivot_routine = SysIdRoutine(
                SysIdRoutine.Config(
                    ramp_rate=0.2,
                    step_voltage=2.0,
                    record_state=lambda state: SignalLogger.write_string(
                        "SysID_IntakePivot", SysIdRoutineLog.state_enum_to_string(state)
                    ),
                ),
                SysIdRoutine.Mechanism(self.set_intake_pivot_voltage, lambda log: None, self),
            )
        return self._pivot_routine

    # ---------------------------------------------------------------- logging

    def log(self) -> None:
        position = self.get_intake_pivot_position()
        velocity = self._intake_encoder.get_velocity().get()

        self._intake_pivot_position_log.append(position)
        self._intake_velo_log.append(velocity)

        if self._intake_velo_pub is not None:
            self._intake_velo_pub.set(velocity)
        if self._intake_pivot_position_pub is not None:
            self._intake_pivot_position_pub.set(position)

    def check_hardware(self) -> None:
        hardware_fault_tracker.check_fault(
            self._intake_alert, self._intake.has_active_fault().get() or self._intake.has_active_warning().get()
        )
        hardware_fault_tracker.check_fault(
            self._intake_follower_alert,
            self._intake_follower.has_active_fault().get() or self._intake_follower.has_active_warning().get(),
        )
        hardware_fault_tracker.check_fault(self._intake_pivot_alert, not self._pivot.is_connected)
