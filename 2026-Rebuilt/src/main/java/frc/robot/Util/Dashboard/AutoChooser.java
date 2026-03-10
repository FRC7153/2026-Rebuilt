package frc.robot.Util.Dashboard;

import java.util.Optional;
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
import frc.robot.Subsystems.Intake;
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
    private final Intake intake;

    private final Alert noAutoLoadedAlert = new Alert("No auto loaded yet (run pregame)", AlertType.kInfo);

    private AutoBuilder autoBuilder = new AutoBuilder(); // For generating path following commands
    public AutoChooser(CommandSwerveDrivetrain drive, Shooter shooter, Climber climber, Intake intake) {
        this.drive = drive;
        this.shooter = shooter;
        this.climber = climber;
        this.intake = intake;

    // On change
    chooser.onChange(newAuto -> {
      currentLoadedCommand = null;
      noAutoLoadedAlert.set(true);
    });

    // Autos that are used for testing
    chooser.setDefaultOption("No Auto", Pair.of(null, () -> noOpCommand));

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

    chooser.addOption("SYSID Intake Pivot Q+",
      Pair.of(null, () -> new SysIdCharacterizationCommand(Intake.getintakePivotRoutine(intake), true, true)));
    chooser.addOption("SYSID Intake Pivot Q-",
      Pair.of(null, () -> new SysIdCharacterizationCommand(Intake.getintakePivotRoutine(intake), true, false)));
    chooser.addOption("SYSID Intake Pivot D+",
      Pair.of(null, () -> new SysIdCharacterizationCommand(Intake.getintakePivotRoutine(intake), false, true)));
    chooser.addOption("SYSID Intake Pivot D-",
      Pair.of(null, () -> new SysIdCharacterizationCommand(Intake.getintakePivotRoutine(intake), false, false)));



    PathPlannerPath testPath = loadPath("TestAuto");
    PathPlannerPath bumpPath = loadPath("BumpAuto");
    
    if(testPath != null){
      chooser.addOption("Test Auto",
        Pair.of(getStartPose(testPath), 
          () -> AutoBuilder.buildAuto("TestAuto")
        )
      ); 
    } 

    if(bumpPath != null) {
      chooser.addOption("BumpAuto", 
        Pair.of(getStartPose(bumpPath), 
        () -> AutoBuilder.buildAuto("BumpAuto")));
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
        noAutoLoadedAlert.set(false);
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

      System.out.println("[AutoChooser] Loaded: " + currentLoadedCommand.getName());
      noAutoLoadedAlert.set(false);

    } 

    public Command getCurrentSelectedCommand(){
        if (currentLoadedCommand == null){
            loadAutoCommand();
        }
        return currentLoadedCommand;
    }

    private PathPlannerPath loadPath(String pathName) {
      try {
        return PathPlannerPath.fromPathFile(pathName);
      } catch (Exception e) {
        System.err.println("[AutoChooser] Could not load path: '" + pathName
          + "'. Check that the file exists in deploy/pathplanner/paths/");
        e.printStackTrace();
        return null;
      }
    }


    private Pose2d getStartPose(PathPlannerPath path) {
        Optional<Pose2d> startPose = path.getStartingHolonomicPose();
        return startPose.orElse(null);
    }
}