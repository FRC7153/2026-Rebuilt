// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.PregameCommand;
import frc.robot.Commands.TeleopDriveCommand;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Libs.Elastic;
import frc.robot.Subsystems.Swerve.SwerveDrive;
import frc.robot.Util.Utils;
import frc.robot.Util.Dashboard.AutoChooser;
import frc.robot.Util.Dashboard.Dashboard;

public class RobotContainer {
  // Controllers
  private final CommandXboxController baseController = new CommandXboxController(0);
  private final CommandXboxController armsController = new CommandXboxController(1);

  //Subsystems
  private final SwerveDrive base = Utils.timeInstantiation(() -> new SwerveDrive(baseController::setRumble));

  // Auto
  private final AutoChooser auto = new AutoChooser(base);
  private final Dashboard dashboard = new Dashboard(baseController, armsController);
  private final Command pregameCommand = new PregameCommand(base, auto);

  public RobotContainer() {
    SmartDashboard.putData("Pregame", pregameCommand);
    configureBindings();
  }

  private void configureBindings() {
    //Triggers 
    final Trigger isEnabledTrigger = new Trigger(DriverStation::isEnabled);
    final Trigger isTeleopTrigger = new Trigger(DriverStation::isTeleopEnabled);

    // Inverted/Comined inputs
    final Supplier<Double> baseLeftX = () -> -baseController.getLeftX();
    final Supplier<Double> baseLeftY = () -> -baseController.getLeftY();
    final Supplier<Double> baseRightX = () -> -baseController.getRightX();
    final Trigger fastModeTrigger = baseController.leftTrigger();

    base.setDefaultCommand(
      new TeleopDriveCommand(
        base,
        baseLeftY, 
        baseLeftX, 
        baseRightX, 
        fastModeTrigger, 
        baseController.leftBumper(),
        baseController.rightBumper()
    ));

    isEnabledTrigger
      .onTrue(dashboard.getRestartTimerCommand())
      .onFalse(dashboard.getStopTimerCommand());

    isTeleopTrigger
      .onTrue(new InstantCommand(() -> Elastic.selectTab(DashboardConstants.ELASTIC_SERVER_PORT)).ignoringDisable(true));
  }

  public void checkHardware(){
    base.checkHardware();
    dashboard.checkHardware();
  }

  public Command getPregameCommand(){
    return pregameCommand;
  }

  public void log(){
    base.log();
    dashboard.update();
  }

  public Command getAutonomousCommand() {
    return auto.getCurrentSelectedCommand();
  }
}