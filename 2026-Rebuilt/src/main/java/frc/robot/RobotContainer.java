// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.DeployIntakeCommand;
import frc.robot.Commands.DistanceShootCommand;
import frc.robot.Commands.HoldIntakeCommand;
import frc.robot.Commands.HomeIntakeCommand;
import frc.robot.Commands.ShootCommand;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Libs.Elastic;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Swerve.Telemetry;
import frc.robot.Subsystems.Swerve.TunerConstants;
import frc.robot.Subsystems.Vision.Limelight;
import frc.robot.Subsystems.Vision.Limelight.Version;
import frc.robot.Util.Utils;
import frc.robot.Util.Dashboard.AutoChooser;
import frc.robot.Util.Dashboard.Dashboard;

public class RobotContainer {
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    //private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // Controllers
  private final CommandXboxController baseController = new CommandXboxController(0);
  private final CommandXboxController armsController = new CommandXboxController(1);

  //Subsystems
  private final Shooter shooter = Utils.timeInstantiation(() -> new Shooter());
  private final Intake intake = Utils.timeInstantiation(() -> new Intake());
  private final Limelight limelightFront = Utils.timeInstantiation(() -> new Limelight(AprilTagConstants.LL_4_FRONT, Version.LIMELIGHT_4));
  private final Limelight limelightBack = Utils.timeInstantiation(() -> new Limelight(AprilTagConstants.LL_3_BACK, Version.LIMELIGHT_3G));

  // Auto
  private final AutoChooser auto = new AutoChooser(drivetrain, shooter, intake);
  private final Dashboard dashboard = new Dashboard(baseController, armsController);
  private final Command homeIntakeCommand = new HomeIntakeCommand(intake);

  public RobotContainer() {
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    //SmartDashboard.putData("Home Intake", homeIntakeCommand);

    NamedCommands.registerCommand(
      "StationaryShoot", 
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          new DeployIntakeCommand(intake, 0.25, 0.6).withTimeout(1.5)
          .andThen(new DeployIntakeCommand(intake, 0.0008, 0.6).withTimeout(1.5))
          .andThen(new DeployIntakeCommand(intake, 0.25, 0.6).withTimeout(1.5))
          .andThen(new DeployIntakeCommand(intake, 0.25, 0.6).withTimeout(1.5)), 
          new ShootCommand(shooter, 24.25, 0.5, -0.7).withTimeout(4)
        )
      )
    );

    NamedCommands.registerCommand(
      "ExtendIntake", 
      new DeployIntakeCommand(intake, 0.25, -0.5).withTimeout(3.0)
    );

    NamedCommands.registerCommand(
      "RetractIntake", 
      new DeployIntakeCommand(intake, 0.0008, 0.0).withTimeout(2.0)
    );

    NamedCommands.registerCommand(
      "RetractIntakeSpin", 
      new DeployIntakeCommand(intake, 0.0008, -0.5).withTimeout(3.0)
    );

    NamedCommands.registerCommand("BumpAuto", 
      new SequentialCommandGroup(
        new WaitCommand(2.5),
        new DeployIntakeCommand(intake, RobotConstants.INTAKE_PIVOT_EXTEND, -0.6).withTimeout(2.75),
        new DeployIntakeCommand(intake, RobotConstants.INTAKE_PIVOT_STOW, 0.0)
      )
    );

    configureBindings();
  }

  private void configureBindings() {
    //Triggers 
    final Trigger isEnabledTrigger = new Trigger(DriverStation::isEnabled);
    final Trigger isTeleopTrigger = new Trigger(DriverStation::isTeleopEnabled);

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
    // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() ->
        drive.withVelocityX(-baseController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-baseController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-baseController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
      )
    );
    
    intake.setDefaultCommand(new HoldIntakeCommand(intake));

    shooter.setDefaultCommand(new ShootCommand(shooter, 0.0, 0.0, 0.0));

    baseController.y().whileTrue(drivetrain.applyRequest(() -> brake));
    //baseController.b().whileTrue(drivetrain.applyRequest(() ->
    //    point.withModuleDirection(new Rotation2d(-baseController.getLeftY(), -baseController.getLeftX()))
    //));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

    isEnabledTrigger
      .onTrue(dashboard.getRestartTimerCommand())
      .onFalse(dashboard.getStopTimerCommand());

    isTeleopTrigger
      .onTrue(new InstantCommand(() -> Elastic.selectTab(DashboardConstants.ELASTIC_SERVER_PORT)).ignoringDisable(true));

    baseController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    baseController.rightTrigger().whileTrue(new ShootCommand(shooter, 24.25, 0.5, -0.70));

    baseController.leftTrigger().whileTrue(new DeployIntakeCommand(intake, RobotConstants.INTAKE_PIVOT_EXTEND, RobotConstants.INTAKE_EXTEND_SPEED));
    baseController.rightBumper().whileTrue(new DeployIntakeCommand(intake, RobotConstants.INTAKE_PIVOT_STOW, 0.0));

    // Reverse Intake
    baseController.b().whileTrue(new DeployIntakeCommand(intake, 0.25, 0.8));

    //Distance Shoot Command 
    //baseController.a().and(baseController.rightTrigger()).whileTrue(new DistanceShootCommand(shooter));

    // Toggle Intake 
    armsController.y().whileTrue(new DeployIntakeCommand(intake, 0.18, 0.0));

    armsController.leftTrigger().whileTrue(new DeployIntakeCommand(intake, RobotConstants.INTAKE_PIVOT_EXTEND, RobotConstants.INTAKE_EXTEND_SPEED));
    armsController.rightBumper().whileTrue(new DeployIntakeCommand(intake, RobotConstants.INTAKE_PIVOT_STOW, 0.0));
    armsController.leftBumper().whileTrue(new DeployIntakeCommand(intake, RobotConstants.INTAKE_PIVOT_STOW, -0.4));

    armsController.rightTrigger().whileTrue(new ShootCommand(shooter, -20, -0.4, 0.6));

    // Reverse Intake
    armsController.b().whileTrue(new DeployIntakeCommand(intake, RobotConstants.INTAKE_PIVOT_EXTEND, 0.8));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void checkHardware(){
    dashboard.checkHardware();
    shooter.checkHardware();
    intake.checkHardware();
    limelightBack.checkHardware();
    limelightFront.checkHardware();
  }

  public void log(){
    dashboard.update();
    shooter.log(); 
    intake.log();
    limelightBack.log();
    limelightFront.log();
  }

  public Command getAutonomousCommand() {
    return auto.getCurrentSelectedCommand();
  }

  public void pregameLoad() {
    auto.loadAutoCommand();
  } 

  public Command getHomeIntakeCommand() {
    return homeIntakeCommand;
  }
}