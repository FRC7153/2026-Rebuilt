// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.ShootCommand;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Libs.Elastic;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Swerve.Telemetry;
import frc.robot.Subsystems.Swerve.TunerConstants;
import frc.robot.Util.Utils;
import frc.robot.Util.Dashboard.AutoChooser;
import frc.robot.Util.Dashboard.Dashboard;

public class RobotContainer {
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 1/2 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // Controllers
  private final CommandXboxController baseController = new CommandXboxController(0);
  private final CommandXboxController armsController = new CommandXboxController(1);

  //Subsystems
  private final Shooter shooter = Utils.timeInstantiation(() -> new Shooter());
  private final Climber climber = Utils.timeInstantiation(() -> new Climber());

  // Auto
  private final AutoChooser auto = new AutoChooser(drivetrain, shooter, climber);
  private final Dashboard dashboard = new Dashboard(baseController, armsController);

  public RobotContainer() {
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

    shooter.setDefaultCommand(
      new ShootCommand(shooter, 0.0, 0.0, 0.0)
      );
    
    baseController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    baseController.b().whileTrue(drivetrain.applyRequest(() ->
        point.withModuleDirection(new Rotation2d(-baseController.getLeftY(), -baseController.getLeftX()))
    ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

    isEnabledTrigger
      .onTrue(dashboard.getRestartTimerCommand())
      .onFalse(dashboard.getStopTimerCommand());
    
    //baseController.rightTrigger()
      //.whileTrue(new ShootCommand(shooter));

    isTeleopTrigger
      .onTrue(new InstantCommand(() -> Elastic.selectTab(DashboardConstants.ELASTIC_SERVER_PORT)).ignoringDisable(true));

    baseController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    baseController.leftBumper().and(baseController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    baseController.leftBumper().and(baseController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    baseController.rightBumper().and(baseController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    baseController.rightBumper().and(baseController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    baseController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    baseController.rightTrigger().whileTrue(new ShootCommand(shooter, 25.0, 100, 0.2));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void checkHardware(){
    dashboard.checkHardware();
  }

  public void log(){
    dashboard.update();
    shooter.log(); 
    climber.log();
  }

  public Command getAutonomousCommand() {
        
    return auto.getCurrentSelectedCommand();
  }
}