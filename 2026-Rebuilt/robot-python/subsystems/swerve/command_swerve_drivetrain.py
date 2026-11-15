"""Phoenix 6 swerve drivetrain as a command-based subsystem.

Port of the Tuner X generated CommandSwerveDrivetrain.java, plus PathPlanner
AutoBuilder configuration and Limelight orientation updates.
"""

import math
import traceback
from typing import Callable

from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import PIDConstants, RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController
from phoenix6 import SignalLogger, swerve, utils
from wpilib import DriverStation, Notifier, RobotController
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Pose2d, Rotation2d

from constants import AprilTagConstants
from libs import limelight_helpers


class CommandSwerveDrivetrain(Subsystem, swerve.SwerveDrivetrain):
    """Class that extends the Phoenix 6 SwerveDrivetrain class and implements
    Subsystem so it can easily be used in command-based projects.
    """

    _SIM_LOOP_PERIOD = 0.004  # 4 ms

    # Blue alliance sees forward as 0 degrees (toward red alliance wall)
    _BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0)
    # Red alliance sees forward as 180 degrees (toward blue alliance wall)
    _RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180)

    def __init__(
        self,
        drive_motor_type,
        steer_motor_type,
        encoder_type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        arg0=None,
        arg1=None,
        arg2=None,
        arg3=None,
    ) -> None:
        """Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves.

        Supported argument forms after drivetrain_constants (same as swerve.SwerveDrivetrain):
          (modules)
          (odometry_update_frequency, modules)
          (odometry_update_frequency, odometry_std_devs, vision_std_devs, modules)
        """
        Subsystem.__init__(self)
        swerve.SwerveDrivetrain.__init__(
            self, drive_motor_type, steer_motor_type, encoder_type, drivetrain_constants, arg0, arg1, arg2, arg3
        )

        self._sim_notifier: Notifier | None = None
        self._last_sim_time = 0.0

        # Keep track if we've ever applied the operator perspective before or not
        self._has_applied_operator_perspective = False

        # Swerve requests to apply during SysId characterization
        self._translation_characterization = swerve.requests.SysIdSwerveTranslation()
        self._steer_characterization = swerve.requests.SysIdSwerveSteerGains()
        self._rotation_characterization = swerve.requests.SysIdSwerveRotation()

        # PathPlanner swerve request
        self._path_apply_robot_speeds = swerve.requests.ApplyRobotSpeeds()

        # SysId routine for characterizing translation. This is used to find PID gains for the drive motors.
        self._sys_id_routine_translation = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Reduce dynamic step voltage to 4 V to prevent brownout
                stepVoltage=4.0,
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdTranslation_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(self._translation_characterization.with_volts(output)),
                lambda log: None,
                self,
            ),
        )

        # SysId routine for characterizing steer. This is used to find PID gains for the steer motors.
        self._sys_id_routine_steer = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(self._steer_characterization.with_volts(output)),
                lambda log: None,
                self,
            ),
        )

        # SysId routine for characterizing rotation.
        # This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
        # See the documentation of swerve.requests.SysIdSwerveRotation for info on importing the log to SysId.
        self._sys_id_routine_rotation = SysIdRoutine(
            SysIdRoutine.Config(
                # This is in radians per second², but SysId only supports "volts per second"
                rampRate=math.pi / 6,
                # This is in radians per second, but SysId only supports "volts"
                stepVoltage=math.pi,
                # Use default timeout (10 s)
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdRotation_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: (
                    # output is actually radians per second, but SysId only supports "volts"
                    self.set_control(self._rotation_characterization.with_rotational_rate(output)),
                    # also log the requested output for SysId
                    SignalLogger.write_double("Rotational_Rate", output),
                ),
                lambda log: None,
                self,
            ),
        )

        # The SysId routine to test
        self._sys_id_routine_to_apply = self._sys_id_routine_translation

        self._configure_auto_builder()

        if utils.is_simulation():
            self._start_sim_thread()

    # ------------------------------------------------------------------ commands

    def apply_request(self, request: Callable[[], swerve.requests.SwerveRequest]) -> Command:
        """Returns a command that applies the specified control request to this swerve drivetrain.

        :param request: Function returning the request to apply
        """
        return self.run(lambda: self.set_control(request()))

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        """Runs the SysId Quasistatic test in the given direction for the selected routine."""
        return self._sys_id_routine_to_apply.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        """Runs the SysId Dynamic test in the given direction for the selected routine."""
        return self._sys_id_routine_to_apply.dynamic(direction)

    # ------------------------------------------------------------------ periodic

    def periodic(self) -> None:
        # Periodically try to apply the operator perspective.
        # If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
        # This allows us to correct the perspective in case the robot code restarts mid-match.
        # Otherwise, only check and apply the operator perspective if the DS is disabled.
        # This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
        if not self._has_applied_operator_perspective or DriverStation.isDisabled():
            alliance_color = DriverStation.getAlliance()
            if alliance_color is not None:
                self.set_operator_perspective_forward(
                    self._RED_ALLIANCE_PERSPECTIVE_ROTATION
                    if alliance_color == DriverStation.Alliance.kRed
                    else self._BLUE_ALLIANCE_PERSPECTIVE_ROTATION
                )
                self._has_applied_operator_perspective = True

        # Limelight configuration (MegaTag2 needs the robot heading every loop)
        heading = self.get_state().pose.rotation().degrees()
        limelight_helpers.set_robot_orientation(AprilTagConstants.LL_4_FRONT, heading, flush=False)
        limelight_helpers.set_robot_orientation(AprilTagConstants.LL_3_BACK, heading, flush=True)

        self.update_vision()

    def _start_sim_thread(self) -> None:
        def _sim_periodic() -> None:
            current_time = utils.get_current_time_seconds()
            delta_time = current_time - self._last_sim_time
            self._last_sim_time = current_time

            # use the measured time delta, get battery voltage from WPILib
            self.update_sim_state(delta_time, RobotController.getBatteryVoltage())

        self._last_sim_time = utils.get_current_time_seconds()
        # Run simulation at a faster rate so PID gains behave more reasonably
        self._sim_notifier = Notifier(_sim_periodic)
        self._sim_notifier.startPeriodic(self._SIM_LOOP_PERIOD)

    # -------------------------------------------------------------------- vision

    def add_vision_measurement(
        self,
        vision_robot_pose: Pose2d,
        timestamp: float,
        vision_measurement_std_devs: tuple[float, float, float] | None = None,
    ) -> None:
        """Adds a vision measurement to the Kalman Filter. This will correct the odometry
        pose estimate while still accounting for measurement noise.

        :param vision_robot_pose: The pose of the robot as measured by the vision camera.
        :param timestamp: The timestamp of the vision measurement in seconds (FPGA time).
        :param vision_measurement_std_devs: Standard deviations of the vision pose measurement
            in the form [x, y, theta], with units in meters and radians.
        """
        swerve.SwerveDrivetrain.add_vision_measurement(
            self, vision_robot_pose, utils.fpga_to_current_time(timestamp), vision_measurement_std_devs
        )

    def sample_pose_at(self, timestamp: float) -> Pose2d | None:
        """Return the pose at a given timestamp (FPGA time), if the buffer is not empty."""
        return swerve.SwerveDrivetrain.sample_pose_at(self, utils.fpga_to_current_time(timestamp))

    def update_vision(self) -> None:
        # Vision pose fusion is disabled, exactly as in the Java code. Uncomment to enable.
        # self._add_limelight_measurement(AprilTagConstants.LL_3_BACK)  # TODO
        # self._add_limelight_measurement(AprilTagConstants.LL_4_FRONT)  # TODO
        pass

    def _add_limelight_measurement(self, camera_name: str) -> None:
        estimate = limelight_helpers.get_bot_pose_estimate_wpi_blue_mega_tag2(camera_name)

        if estimate is None:
            return

        if estimate.tag_count < 1:
            return

        if estimate.avg_tag_dist > 5.0:
            return  # TODO

        if abs(self.get_state().speeds.omega) > 4.0:
            return  # Reject vision measurement if robot is rotating too fast

        std_devs = (
            0.9,  # X TODO
            0.9,  # Y TODO
            999999.0,  # Theta TODO
        )

        self.add_vision_measurement(estimate.pose, estimate.timestamp_seconds, std_devs)

    # --------------------------------------------------------------- pathplanner

    def _configure_auto_builder(self) -> None:
        try:
            config = RobotConfig.fromGUISettings()
        except Exception:
            traceback.print_exc()
            DriverStation.reportError(
                "Failed to load PathPlanner robot config (deploy/pathplanner/settings.json); autos are disabled",
                False,
            )
            return

        AutoBuilder.configure(
            lambda: self.get_state().pose,
            self.reset_pose,
            lambda: self.get_state().speeds,
            lambda speeds, feedforwards: self.set_control(self._path_apply_robot_speeds.with_speeds(speeds)),
            PPHolonomicDriveController(
                PIDConstants(2.0, 0.0, 0.2),
                PIDConstants(3.0, 0.0, 0.1),
            ),
            config,
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed,
            self,
        )
