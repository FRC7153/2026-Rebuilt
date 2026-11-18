"""Shooter subsystem: two Kraken flywheels (CANivore) + kicker and live floor (NEO Vortex)."""

import math

import ntcore
import wpilib
from commands2 import Subsystem
from commands2.sysid import SysIdRoutine
from phoenix6 import SignalLogger, controls, hardware, signals
from rev import PersistMode, ResetMode, SparkFlex, SparkLowLevel
from wpiutil import Alert
from wpilib.sysid import SysIdRoutineLog
from wpilog import DoubleLogEntry

from constants import AprilTagConstants, BuildConstants, HardwareConstants, ShooterConstants
from libs import limelight_helpers
from util import hardware_fault_tracker, shooter_regression
from util.setpoint_cache import SetpointCache


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class Shooter(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self._shooter = hardware.TalonFX(HardwareConstants.SHOOTER_CAN, HardwareConstants.CANIVORE)
        self._shooter2 = hardware.TalonFX(HardwareConstants.SHOOTER_2_CAN, HardwareConstants.CANIVORE)
        self._kicker = SparkFlex(HardwareConstants.SYSTEMCORE_CAN_PORT, HardwareConstants.KICKER_CAN, SparkLowLevel.MotorType.BRUSHLESS)
        self._live_floor = SparkFlex(HardwareConstants.SYSTEMCORE_CAN_PORT, HardwareConstants.LIVEFLOOR_CAN, SparkLowLevel.MotorType.BRUSHLESS)
        self._kicker_encoder = self._kicker.get_encoder()
        self._live_floor_encoder = self._live_floor.get_encoder()
        self._kicker_controller = self._kicker.get_closed_loop_controller()

        self._static_brake_request = controls.StaticBrake()
        self._neutral_request = controls.NeutralOut()
        self._voltage_request = controls.VoltageOut(0.0)
        self._shooter_velo_request = (
            controls.VelocityVoltage(0.0).with_override_brake_dur_neutral(True).with_slot(0)
        )

        self._shooter_velo = self._shooter.get_velocity()

        self._shooter_alert = Alert("Shooter Alert", "Shooter Alert", Alert.Level.HIGH)
        self._shooter2_alert = Alert("Shooter 2 Alert", "Shooter 2 Alert", Alert.Level.HIGH)
        self._kicker_alert = Alert("Kicker Alert", "Kicker Alert", Alert.Level.HIGH)
        self._live_floor_alert = Alert("Live Floor Alert", "Live Floor Alert", Alert.Level.HIGH)

        self._shooter_routine: SysIdRoutine | None = None
        self._kicker_routine: SysIdRoutine | None = None

        # Datalog
        log = wpilib.DataLogManager.get_log()
        self._shooter_velo_log = DoubleLogEntry(log, "Shooter/Velo", "RPS")
        self._kicker_velo_log = DoubleLogEntry(log, "Kicker/Velo", "RPM")
        self._live_floor_velo_log = DoubleLogEntry(log, "LiveFloor/Velo", "RPM")
        self._kicker_setpoint_log = DoubleLogEntry(log, "Kicker/Setpoint", "RPM")
        self._shooter_setpoint_log = DoubleLogEntry(log, "Shooter/Setpoint", "RPS")
        self._live_floor_setpoint_log = DoubleLogEntry(log, "LiveFloor/Setpoint", "RPM")
        self._kicker_voltage_log = DoubleLogEntry(log, "Kicker/Voltage", "Volts")
        self._kicker_position_log = DoubleLogEntry(log, "Kicker/Position", "Rotations")

        self._last_known_distance: float | None = None

        # Only talk to a motor controller when its setpoint actually changes
        self._shooter_cache = SetpointCache()
        self._kicker_cache = SetpointCache()
        self._live_floor_cache = SetpointCache()

        # Configure hardware
        self._shooter.configurator.apply(ShooterConstants.SHOOTER_CONFIG)
        self._kicker.configure(
            ShooterConstants.kicker_config(),
            ResetMode.RESET_SAFE_PARAMETERS,
            PersistMode.NO_PERSIST_PARAMETERS,
        )
        self._live_floor.configure(
            ShooterConstants.livefloor_config(),
            ResetMode.RESET_SAFE_PARAMETERS,
            PersistMode.NO_PERSIST_PARAMETERS,
        )
        self._shooter2.configurator.apply(ShooterConstants.SHOOTER_CONFIG)
        self._shooter2.set_control(
            controls.Follower(HardwareConstants.SHOOTER_CAN, signals.MotorAlignmentValue.OPPOSED)
        )

        # Limelight distance is always published (it is cheap and needed for tuning the regression)
        nt_inst = ntcore.NetworkTableInstance.get_default()
        self._distance_pub = nt_inst.get_table("LLfront").get_double_topic("LL distance").publish()

        # NT logging
        self._kicker_velo_pub = None
        self._shooter_velo_pub = None
        self._live_floor_velo_pub = None
        self._kicker_setpoint_pub = None
        self._shooter_setpoint_pub = None
        self._live_floor_setpoint_pub = None
        if BuildConstants.PUBLISH_EVERYTHING:
            nt = nt_inst.get_table("Shooter")
            self._kicker_velo_pub = nt.get_double_topic("kickerVelo").publish()
            self._shooter_velo_pub = nt.get_double_topic("shooterVelo").publish()
            self._live_floor_velo_pub = nt.get_double_topic("liveFloorVelo").publish()
            self._kicker_setpoint_pub = nt.get_double_topic("kickerSetpoint").publish()
            self._shooter_setpoint_pub = nt.get_double_topic("shooterSetpoint").publish()
            self._live_floor_setpoint_pub = nt.get_double_topic("liveFloorSetpoint").publish()

    # --------------------------------------------------------------- flywheel

    def set_shooter_speed(self, velo: float) -> None:
        """Sets the flywheel velocity, in rotations per second (0 - 100)."""
        velo = _clamp(velo, 0.0, ShooterConstants.SHOOTER_MAX_RPS)
        if not self._shooter_cache.should_send(velo):
            return

        self._shooter.set_control(self._shooter_velo_request.with_velocity(velo).with_slot(0))
        self._shooter_setpoint_log.append(velo)
        if self._shooter_setpoint_pub is not None:
            self._shooter_setpoint_pub.set(velo)

    def get_shooter_velocity(self) -> float:
        """Flywheel velocity in rotations per second."""
        return self._shooter_velo.refresh().value_as_double

    def set_velocity_from_limelight(self) -> bool:
        """Spins the flywheel for the distance measured by the front Limelight.

        Uses the last known distance if the target is not currently visible.

        :returns: True if the flywheel is at speed.
        """
        if limelight_helpers.get_target_count(AprilTagConstants.LL_4_FRONT) >= 1:
            ty = limelight_helpers.get_ty(AprilTagConstants.LL_4_FRONT)
            distance = shooter_regression.get_shooter_distance(ty)
            if math.isfinite(distance):
                self._last_known_distance = distance

        if self._last_known_distance is None:
            # Never seen a target: nothing to aim at
            self._shooter_cache.invalidate()
            self._shooter.set_control(self._neutral_request)
            return False

        self._distance_pub.set(self._last_known_distance)

        target_rps = _clamp(
            shooter_regression.get_velocity_rotations_per_sec(self._last_known_distance),
            0.0,
            ShooterConstants.SHOOTER_MAX_RPS,
        )
        if self._shooter_cache.should_send(target_rps):
            self._shooter.set_control(self._shooter_velo_request.with_velocity(target_rps).with_slot(0))
            self._shooter_setpoint_log.append(target_rps)
            if self._shooter_setpoint_pub is not None:
                self._shooter_setpoint_pub.set(target_rps)

        return abs(self.get_shooter_velocity() - target_rps) < 10.0

    def _set_shooter_voltage(self, voltage: float) -> None:
        self._shooter_cache.invalidate()
        self._shooter.set_control(self._voltage_request.with_output(voltage))

    # ----------------------------------------------------------------- kicker

    def set_kicker_speed(self, speed: float) -> None:
        """Runs the kicker open-loop. speed is duty cycle, -1.0 to 1.0."""
        speed = _clamp(speed, -1.0, 1.0)
        if not self._kicker_cache.should_send(("duty", speed)):
            return
        self._kicker.set_throttle(speed)

        self._kicker_setpoint_log.append(speed)
        if self._kicker_setpoint_pub is not None:
            self._kicker_setpoint_pub.set(speed)

    def set_kicker_velo(self, velo: float) -> None:
        """Runs the kicker closed-loop. velo is in RPM (0 - 6784)."""
        if not self._kicker_cache.should_send(("rpm", velo)):
            return
        self._kicker_controller.set_setpoint(velo, SparkLowLevel.ControlType.VELOCITY)

        self._kicker_setpoint_log.append(velo)
        if self._kicker_setpoint_pub is not None:
            self._kicker_setpoint_pub.set(velo)

    def stop_kicker(self) -> None:
        if not self._kicker_cache.should_send(("stop", 0.0)):
            return
        self._kicker.stop_motor()
        self._kicker_setpoint_log.append(0.0)
        if self._kicker_setpoint_pub is not None:
            self._kicker_setpoint_pub.set(0.0)

    def _set_kicker_voltage(self, voltage: float) -> None:
        self._kicker_cache.invalidate()
        self._kicker.set_voltage(voltage)

    def get_kicker_velocity(self) -> float:
        """Kicker velocity in RPM."""
        return self._kicker_encoder.get_velocity().get()

    # ------------------------------------------------------------- live floor

    def set_live_floor_speed(self, speed: float) -> None:
        """Runs the live floor open-loop. speed is duty cycle, -1.0 to 1.0."""
        speed = _clamp(speed, -1.0, 1.0)
        if not self._live_floor_cache.should_send(speed):
            return
        self._live_floor.set_throttle(speed)

        self._live_floor_setpoint_log.append(speed)
        if self._live_floor_setpoint_pub is not None:
            self._live_floor_setpoint_pub.set(speed)

    # -------------------------------------------------------------- stop/idle

    def stop_shooter_sub(self) -> None:
        """Brakes the flywheel and stops the kicker and live floor."""
        self._shooter_cache.invalidate()
        self._kicker_cache.invalidate()
        self._live_floor_cache.invalidate()
        self._shooter.set_control(self._static_brake_request)
        self._kicker.stop_motor()
        self._live_floor.stop_motor()

    def idle_motors(self) -> None:
        """Lets the flywheel coast and stops the kicker and live floor."""
        self._shooter_cache.invalidate()
        self._kicker_cache.invalidate()
        self._live_floor_cache.invalidate()
        self._shooter.set_control(self._neutral_request)
        self._kicker.stop_motor()
        self._live_floor.stop_motor()

    # ------------------------------------------------------------------ sysid

    def get_shooter_routine(self) -> SysIdRoutine:
        print("Starting CTRE SignalLogger due to get_shooter_routine")
        SignalLogger.start()

        if self._shooter_routine is None:
            self._shooter_routine = SysIdRoutine(
                SysIdRoutine.Config(
                    ramp_rate=0.5,
                    step_voltage=6.0,
                    record_state=lambda state: SignalLogger.write_string(
                        "SysID_Shooter", SysIdRoutineLog.state_enum_to_string(state)
                    ),
                ),
                SysIdRoutine.Mechanism(self._set_shooter_voltage, lambda log: None, self),
            )
        return self._shooter_routine

    def get_kicker_routine(self) -> SysIdRoutine:
        """SysId routine for the kicker (records voltage/position/velocity to the WPILog)."""
        wpilib.DataLogManager.start()

        if self._kicker_routine is None:

            def _log(log: SysIdRoutineLog) -> None:
                (
                    log.motor("kicker")
                    .voltage(self._kicker.get_applied_output().get() * self._kicker.get_bus_voltage().get())
                    .position(self._kicker_encoder.get_position().get())
                    .velocity(self._kicker_encoder.get_velocity().get() / 60.0)
                )

            self._kicker_routine = SysIdRoutine(
                SysIdRoutine.Config(ramp_rate=1.0, step_voltage=6.0, timeout=10.0),
                SysIdRoutine.Mechanism(self._set_kicker_voltage, _log, self),
            )
        return self._kicker_routine

    # ---------------------------------------------------------------- logging

    def log(self) -> None:
        shooter_velo = self.get_shooter_velocity()
        kicker_velo = self._kicker_encoder.get_velocity().get()
        live_floor_velo = self._live_floor_encoder.get_velocity().get()

        self._shooter_velo_log.append(shooter_velo)
        self._kicker_velo_log.append(kicker_velo)
        self._live_floor_velo_log.append(live_floor_velo)

        self._kicker_voltage_log.append(self._kicker.get_applied_output().get() * self._kicker.get_bus_voltage().get())
        self._kicker_position_log.append(self._kicker_encoder.get_position().get())

        if self._shooter_velo_pub is not None:
            self._shooter_velo_pub.set(shooter_velo)
        if self._kicker_velo_pub is not None:
            self._kicker_velo_pub.set(kicker_velo)
        if self._live_floor_velo_pub is not None:
            self._live_floor_velo_pub.set(live_floor_velo)

    def check_hardware(self) -> None:
        hardware_fault_tracker.check_fault(self._shooter_alert, not self._shooter.is_connected)
        hardware_fault_tracker.check_fault(self._shooter2_alert, not self._shooter2.is_connected)
        hardware_fault_tracker.check_fault(
            self._kicker_alert, self._kicker.has_active_fault().get() or self._kicker.has_active_warning().get()
        )
        hardware_fault_tracker.check_fault(
            self._live_floor_alert,
            self._live_floor.has_active_fault().get() or self._live_floor.has_active_warning().get(),
        )
