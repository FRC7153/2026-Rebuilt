package frc.robot.Util.Dashboard;

import java.util.function.Supplier;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.BuildConstants;
import frc.robot.Commands.SysID.SysIdCharacterizationCommand;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Swerve.CommandSwerveDrivetrain;

import frc.robot.Util.Utils;
import frc.robot.Util.Logging.ConsoleLogger;

public class AutoChooser {
    private static final PrintCommand noOpCommand = new PrintCommand("No-op auto selected.");

    // Each option is a pair of the starting Pose2d and a supplier for the auto command
    private final SendableChooser<Pair<Pose2d, Supplier<Command>>> chooser = new SendableChooser<>();
    private Command currentLoadedCommand = null;

    private final CommandSwerveDrivetrain drive;
    private final Shooter shooter;
    private final Climber climber;

    private final Alert noAutoLoadedAlert = new Alert("No auto loaded yet (run pregame)", AlertType.kInfo);

    public AutoChooser(CommandSwerveDrivetrain drive, Shooter shooter, Climber climber) {
        this.drive = drive;
        this.shooter = shooter;
        this.climber = climber;

    // On change
    chooser.onChange((Pair<Pose2d, Supplier<Command>> newAuto) -> {
      currentLoadedCommand = null;
      noAutoLoadedAlert.set(true);
    });

    // Starting Positions //TODO 

    // Autos that are used for testing
    chooser.addOption("No Auto", Pair.of(null, () -> noOpCommand));

    chooser.addOption("SYSID Shooter Q+", 
      Pair.of(null, () -> new SysIdCharacterizationCommand(Shooter.getShooterRoutine(shooter), true, true)));

    chooser.addOption("SYSID Shooter Q-", 
      Pair.of(null, () -> new SysIdCharacterizationCommand(Shooter.getShooterRoutine(shooter), true, false)));

    chooser.addOption("SYSID Shooter D+", 
      Pair.of(null, () -> new SysIdCharacterizationCommand(Shooter.getShooterRoutine(shooter), false, true)));

    chooser.addOption("SYSID Shooter D-", 
      Pair.of(null, () -> new SysIdCharacterizationCommand(Shooter.getShooterRoutine(shooter), false, false)));

      //Climber SysId 
    chooser.addOption("SYSID Climber Q+", 
      Pair.of(null, () -> new SysIdCharacterizationCommand(Climber.getClimberRoutine(climber), true, true)));
    chooser.addOption("SYSID Climber Q-", 
      Pair.of(null, () -> new SysIdCharacterizationCommand(Climber.getClimberRoutine(climber), true, false)));
    chooser.addOption("SYSID Climber D+", 
      Pair.of(null, () -> new SysIdCharacterizationCommand(Climber.getClimberRoutine(climber), false, true)));
    chooser.addOption("SYSID Climber D-", 
      Pair.of(null, () -> new SysIdCharacterizationCommand(Climber.getClimberRoutine(climber), false, false)));

      
    /**if (BuildConstants.INCLUDE_TEST_AUTOS) {
      // Add Swerve SysId drive autos
      chooser.addOption("SYSID Swerve Drive Q+", 
      Pair.of(null, () -> new SysIdCharacterizationCommand(SwerveSysID.getModuleDriveRoutine(drive), true, true)));
      chooser.addOption("SYSID Swerve Drive Q-", 
      Pair.of(null, () -> new SysIdCharacterizationCommand(SwerveSysID.getModuleDriveRoutine(drive), true, false)));
      chooser.addOption("SYSID Swerve Drive D+", 
      Pair.of(null, () -> new SysIdCharacterizationCommand(SwerveSysID.getModuleDriveRoutine(drive), false, true)));
      chooser.addOption("SYSID Swerve Drive D-", 
      Pair.of(null, () -> new SysIdCharacterizationCommand(SwerveSysID.getModuleDriveRoutine(drive), false, false)));

      // Add Swerve SysId steer autos
      chooser.addOption("SYSID Swerve Steer Q+", 
        Pair.of(null, () -> new SysIdCharacterizationCommand(SwerveSysID.getModuleSteerRoutine(drive), true, true)));
      chooser.addOption("SYSID Swerve Steer Q-", 
        Pair.of(null, () -> new SysIdCharacterizationCommand(SwerveSysID.getModuleSteerRoutine(drive), true, false)));
      chooser.addOption("SYSID Swerve Steer D+", 
        Pair.of(null, () -> new SysIdCharacterizationCommand(SwerveSysID.getModuleSteerRoutine(drive), false, true)));
      chooser.addOption("SYSID Swerve Steer D-", 
        Pair.of(null, () -> new SysIdCharacterizationCommand(SwerveSysID.getModuleSteerRoutine(drive), false, false)));
    } 

    */
    // Add the chooser to the dashboard
    SmartDashboard.putData("Auto", chooser);
    noAutoLoadedAlert.set(true);
  }   

    /**
   * Loads the currently selected command.
   */
    public void loadAutoCommand() {
        Pair<Pose2d, Supplier<Command>> selected = chooser.getSelected();
        currentLoadedCommand = selected.getSecond().get();
        System.out.printf("New auto loaded: %s\n", currentLoadedCommand.getName());
        noAutoLoadedAlert.set(false);

    } 

    public Command getCurrentSelectedCommand(){
        if (currentLoadedCommand == null){
            loadAutoCommand();
        }
        return currentLoadedCommand;
    }
}