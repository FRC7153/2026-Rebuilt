"""Subsystems, controller bindings and autonomous setup (port of RobotContainer.java)."""

import math

from commands2 import Command, InstantCommand, ParallelCommandGroup, SequentialCommandGroup, WaitCommand
from commands2.button import CommandXboxController, Trigger
from pathplannerlib.auto import NamedCommands
from pathplannerlib.events import EventTrigger
from phoenix6 import swerve
from wpilib import Color, Color8Bit, DriverStation, Mechanism2d, SmartDashboard, Timer

from commands.intake_commands import DeployIntakeCommand, HoldIntakeCommand, HomeIntakeCommand
from commands.point_to_hub_command import PointToHubCommand
from commands.shoot_commands import DistanceShootCommand, ShootCommand
from constants import AprilTagConstants, DashboardConstants, RobotConstants
from libs import elastic
from subsystems.intake import Intake
from subsystems.limelight import Limelight, Version
from subsystems.shooter import Shooter
from subsystems.swerve import tuner_constants
from subsystems.swerve.telemetry import Telemetry
from util import utils
from util.auto_chooser import AutoChooser
from util.dashboard import Dashboard
from util.handoff_monitor import HandoffMonitor


class RobotContainer:
    def __init__(self) -> None:
        # speed_at_12_volts is the desired top speed
        self._max_speed = 1.0 * tuner_constants.speed_at_12_volts
        # 3/4 of a rotation per second max angular velocity
        self._max_angular_rate = 0.75 * 2.0 * math.pi

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(self._max_angular_rate * 0.1)  # Add a 10% deadband
            # Use open-loop control for drive motors
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        )
        self._brake = swerve.requests.SwerveDriveBrake()

        self._logger = Telemetry(self._max_speed)

        self.drivetrain = tuner_constants.create_drivetrain()

        # Controllers
        self._base_controller = CommandXboxController(0)
        self._arms_controller = CommandXboxController(1)

        # Subsystems
        self._shooter = utils.time_instantiation(Shooter)
        self._intake = utils.time_instantiation(Intake)
        self._limelight_front = utils.time_instantiation(
            lambda: Limelight(AprilTagConstants.LL_4_FRONT, Version.LIMELIGHT_4)
        )
        self._limelight_back = utils.time_instantiation(
            lambda: Limelight(AprilTagConstants.LL_3_BACK, Version.LIMELIGHT_3G)
        )

        self._dashboard = Dashboard(self._base_controller, self._arms_controller)
        self._home_intake_command = HomeIntakeCommand(self._intake)

        self._configure_auto_commands()
        self._configure_bindings()
        self._configure_mechanism_display()

        # Explains "the robot won't drive" at the start of teleop (see util/handoff_monitor.py)
        self._drive_state = self.drivetrain.get_state()
        self._handoff_monitor = HandoffMonitor(
            self.drivetrain,
            lambda: max(abs(self._base_controller.getLeftX()), abs(self._base_controller.getLeftY())),
            lambda: max((abs(m.speed) for m in self._drive_state.module_targets), default=0.0),
        )

        # Auto (must be created after the drivetrain, which configures PathPlanner's AutoBuilder)
        self._auto = AutoChooser()

    # ---------------------------------------------------------------------- auto

    def _configure_auto_commands(self) -> None:
        intake = self._intake
        shooter = self._shooter

        NamedCommands.registerCommand(
            "StationaryShoot",
            ParallelCommandGroup(
                SequentialCommandGroup(
                    DeployIntakeCommand(intake, RobotConstants.INTAKE_PIVOT_EXTEND, -0.6).withTimeout(1.5),
                    DeployIntakeCommand(intake, 0.0008, -0.6).withTimeout(1.5),
                    DeployIntakeCommand(intake, 0.25, -0.6).withTimeout(1.5),
                    DeployIntakeCommand(intake, 0.25, -0.6).withTimeout(1.5),
                ),
                ShootCommand(shooter, 24.25, 0.5, -0.7).withTimeout(4),
            ),
        )

        NamedCommands.registerCommand(
            "ExtendIntake",
            DeployIntakeCommand(intake, RobotConstants.INTAKE_PIVOT_EXTEND, -0.5).withTimeout(3.0),
        )

        NamedCommands.registerCommand(
            "RetractIntake",
            DeployIntakeCommand(intake, RobotConstants.INTAKE_PIVOT_STOW, 0.0).withTimeout(2.0),
        )

        NamedCommands.registerCommand(
            "RetractIntakeSpin",
            DeployIntakeCommand(intake, RobotConstants.INTAKE_PIVOT_STOW, -0.5).withTimeout(3.0),
        )

        NamedCommands.registerCommand(
            "BumpAuto",
            SequentialCommandGroup(
                WaitCommand(2.5),
                DeployIntakeCommand(intake, RobotConstants.INTAKE_PIVOT_EXTEND, -0.6).withTimeout(2.75),
                # Was missing a timeout in Java, which made this named command never finish
                DeployIntakeCommand(intake, RobotConstants.INTAKE_PIVOT_STOW, 0.0).withTimeout(2.0),
            ),
        )

        is_during_auto = Trigger(DriverStation.isAutonomousEnabled)

        EventTrigger("ExtendIntake").and_(is_during_auto).onTrue(
            DeployIntakeCommand(intake, RobotConstants.INTAKE_PIVOT_EXTEND, RobotConstants.INTAKE_EXTEND_SPEED)
        )

        EventTrigger("RetractIntake").and_(is_during_auto).onTrue(
            DeployIntakeCommand(intake, RobotConstants.INTAKE_PIVOT_STOW, 0.0)
        )

        EventTrigger("StationaryShoot").and_(is_during_auto).onTrue(
            ParallelCommandGroup(
                ShootCommand(shooter, 15.0, 0.2, -0.2).withTimeout(8.0),
                SequentialCommandGroup(
                    WaitCommand(4.0),
                    DeployIntakeCommand(
                        intake, RobotConstants.INTAKE_PIVOT_EXTEND, RobotConstants.INTAKE_EXTEND_SPEED * -1
                    ).withTimeout(8.0),
                ),
            )
        )

        EventTrigger("StopShooter").onTrue(ShootCommand(shooter, 0.0, 0.0, 0.0))

    # ------------------------------------------------------------------ bindings

    def _configure_bindings(self) -> None:
        base = self._base_controller
        arms = self._arms_controller
        drivetrain = self.drivetrain
        intake = self._intake
        shooter = self._shooter

        stow = RobotConstants.INTAKE_PIVOT_STOW
        extend = RobotConstants.INTAKE_PIVOT_EXTEND
        intake_speed = RobotConstants.INTAKE_EXTEND_SPEED

        # Triggers
        is_enabled_trigger = Trigger(DriverStation.isEnabled)
        is_teleop_trigger = Trigger(DriverStation.isTeleopEnabled)

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(-base.getLeftY() * self._max_speed)  # Drive forward with negative Y
                    .with_velocity_y(-base.getLeftX() * self._max_speed)  # Drive left with negative X
                    .with_rotational_rate(-base.getRightX() * self._max_angular_rate)  # CCW with negative X
                )
            ).withName("TeleopDrive")
        )

        intake.setDefaultCommand(HoldIntakeCommand(intake))

        shooter.setDefaultCommand(ShootCommand(shooter, 0.0, 0.0, 0.0))

        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            drivetrain.apply_request(lambda: idle).ignoringDisable(True).withName("DisabledIdle")
        )

        is_enabled_trigger.onTrue(self._dashboard.get_restart_timer_command()).onFalse(
            self._dashboard.get_stop_timer_command()
        )

        is_teleop_trigger.onTrue(
            InstantCommand(lambda: elastic.select_tab(DashboardConstants.TELEOP_TAB)).ignoringDisable(True)
        )

        # ------------------------------ Base (driver) controller ------------------------------

        # X-brake the wheels.
        # (In Java this was on Y, which is also "point to hub" - both need the drivetrain,
        # so the brake could never run. It now lives on X, which was unused.)
        base.x().whileTrue(drivetrain.apply_request(lambda: self._brake).withName("XBrake"))

        # Point to hub
        base.y().whileTrue(
            PointToHubCommand(
                drivetrain,
                lambda: -base.getLeftY() * self._max_speed,
                lambda: -base.getLeftX() * self._max_speed,
                AprilTagConstants.LL_4_FRONT,
            )
        )

        # Reset gyro
        base.povUp().onTrue(drivetrain.runOnce(drivetrain.seed_field_centric))

        # Aim at the hub and shoot (left stick still translates while aiming)
        base.rightTrigger().whileTrue(
            ParallelCommandGroup(
                PointToHubCommand(
                    drivetrain,
                    lambda: -base.getLeftY() * self._max_speed,
                    lambda: -base.getLeftX() * self._max_speed,
                    AprilTagConstants.LL_4_FRONT,
                ),
                ShootCommand(shooter, 24.25, 0.5, -0.6),
            ).withName("AimAndShoot")
        )

        # Passing shot
        base.leftBumper().whileTrue(ShootCommand(shooter, 15.0, 0.2, -0.2))

        # Intake
        base.leftTrigger().whileTrue(DeployIntakeCommand(intake, extend, intake_speed))
        base.rightBumper().whileTrue(DeployIntakeCommand(intake, stow, 0.0))

        # Reverse intake
        base.b().whileTrue(DeployIntakeCommand(intake, extend, intake_speed * -1))

        # Distance shoot
        base.a().and_(base.rightTrigger()).whileTrue(DistanceShootCommand(shooter))

        # ---------------------------- Arms (operator) controller ----------------------------

        # Partially raise intake
        arms.y().whileTrue(DeployIntakeCommand(intake, 0.18, 0.0))

        # Intake
        arms.leftTrigger().whileTrue(DeployIntakeCommand(intake, extend, intake_speed))
        arms.rightBumper().whileTrue(DeployIntakeCommand(intake, stow, 0.0))
        arms.leftBumper().whileTrue(DeployIntakeCommand(intake, stow, intake_speed))

        # Shoot button
        arms.rightTrigger().whileTrue(ShootCommand(shooter, 24.25, 0.5, -0.6))

        # Distance shoot (operator A while the driver holds the right trigger, as in Java)
        arms.a().and_(base.rightTrigger()).whileTrue(DistanceShootCommand(shooter))

        # Reverse intake
        arms.b().whileTrue(DeployIntakeCommand(intake, extend, intake_speed * -1))

    # ------------------------------------------------------------------ display

    def _configure_mechanism_display(self) -> None:
        """A side view of the intake pivot and shooter for AdvantageScope / Elastic / the sim GUI."""
        self._mechanism = Mechanism2d(1.2, 1.0)
        self._intake_ligament = self._mechanism.getRoot("IntakePivot", 0.75, 0.2).appendLigament(
            "Intake", 0.4, 90, 8, Color8Bit(Color.kOrange)
        )
        self._shooter_ligament = self._mechanism.getRoot("Shooter", 0.3, 0.2).appendLigament(
            "Flywheel", 0.05, 90, 12, Color8Bit(Color.kGreen)
        )
        SmartDashboard.putData("Superstructure", self._mechanism)

    def _update_mechanism_display(self) -> None:
        # Pivot: 0 rotations = stowed (straight up), negative = folding out toward the floor
        self._intake_ligament.setAngle(90.0 + self._intake.get_intake_pivot_position() * 360.0 * 0.3)
        # Flywheel: bar grows with speed (100 rps = 0.6 m)
        self._shooter_ligament.setLength(0.05 + abs(self._shooter.get_shooter_velocity()) * 0.006)

    # ------------------------------------------------------------------ periodic

    def periodic(self) -> None:
        """Runs every robot loop, after the command scheduler."""
        self._drive_state = self.drivetrain.get_state()
        self._logger.telemeterize(self._drive_state)
        self._handoff_monitor.update()

    def teleop_started(self) -> None:
        self._handoff_monitor.start()

    def teleop_ended(self) -> None:
        self._handoff_monitor.stop()

    def check_hardware(self) -> None:
        self._dashboard.check_hardware()
        self._shooter.check_hardware()
        self._intake.check_hardware()
        self._limelight_back.check_hardware()
        self._limelight_front.check_hardware()

    def log(self) -> None:
        self._dashboard.update()
        self._shooter.log()
        self._intake.log()
        self._limelight_back.log()
        self._limelight_front.log()
        self._update_mechanism_display()

    # ---------------------------------------------------------------------- auto

    def get_autonomous_command(self) -> Command:
        return self._auto.get_current_selected_command()

    def is_auto_loaded(self) -> bool:
        return self._auto.is_loaded()

    def pregame_load(self) -> None:
        """Builds the selected auto (does nothing if it is already built)."""
        if self._auto.is_loaded():
            return
        start = Timer.getFPGATimestamp()
        self._auto.load_auto_command()
        print(f"Pregame load took {Timer.getFPGATimestamp() - start:.2f} s")

    def get_home_intake_command(self) -> Command:
        return self._home_intake_command
