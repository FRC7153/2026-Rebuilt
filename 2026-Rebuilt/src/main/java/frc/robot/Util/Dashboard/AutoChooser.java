package frc.robot.Util.Dashboard;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    Pose2d startingCenter = new Pose2d(3.548, 4.449, Rotation2d.k180deg);

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

    chooser.addOption("SYSID Kicker Q+",
      Pair.of(null, () -> new SysIdCharacterizationCommand(Shooter.getKickerRoutine(shooter), true, true)));
    chooser.addOption("SYSID Kicker Q-",
      Pair.of(null, () -> new SysIdCharacterizationCommand(Shooter.getKickerRoutine(shooter), true, false)));
    chooser.addOption("SYSID Kicker D+",
      Pair.of(null, () -> new SysIdCharacterizationCommand(Shooter.getKickerRoutine(shooter), false, true)));
    chooser.addOption("SYSID Kicker D-",
      Pair.of(null, () -> new SysIdCharacterizationCommand(Shooter.getKickerRoutine(shooter), false, false)));

    try {
      PathPlannerPath testPath = PathPlannerPath.fromPathFile("TestAuto");
      Pose2d startingPose = testPath.getStartingDifferentialPose();

      chooser.addOption("Test Auto", 
        Pair.of(startingPose,
          () -> AutoBuilder.buildAuto("TestAuto"))
      );
    } catch (Exception e) {
      e.printStackTrace();
    }

    // Add the chooser to the dashboard
    SmartDashboard.putData("Auto", chooser);
    noAutoLoadedAlert.set(true);
  }   

    /**
   * Loads the currently selected command.
   */
    public void loadAutoCommand() {
      Pair<Pose2d, Supplier<Command>> selected = chooser.getSelected();

      if (selected == null) {
        currentLoadedCommand = noOpCommand;
        return;
      }

      Pose2d startingPose = selected.getFirst();

      // If auto has a starting pose, reset drivetrain
      if (startingPose != null) {
          drive.resetPose(
              DriverStation.getAlliance()
                  .map(a -> a == DriverStation.Alliance.Red
                      ? FlippingUtil.flipFieldPose(startingPose)
                      : startingPose)
                  .orElse(startingPose)
          );
      }

      currentLoadedCommand = selected.getSecond().get();

      System.out.println("New auto loaded: " + currentLoadedCommand.getName());
      noAutoLoadedAlert.set(false);

    } 

    public Command getCurrentSelectedCommand(){
        if (currentLoadedCommand == null){
            loadAutoCommand();
        }
        return currentLoadedCommand;
    }
}