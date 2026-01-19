package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Swerve.SwerveConstants;
import frc.robot.Subsystems.Swerve.SwerveDrive;
import frc.robot.Util.Utils;

/**
 * Resets the swerve drive odometry to a default position
 */
public class ResetOdometryToDefaultCommand extends InstantCommand {
  public ResetOdometryToDefaultCommand(SwerveDrive drive) {
    super(() -> {
      if (Utils.isRedAlliance()) {
        // Red alliance default pose
        drive.resetOdometry(SwerveConstants.DEFAULT_RED_POSE);
      } else {
        // Blue alliance default pose
        drive.resetOdometry(SwerveConstants.DEFAULT_BLUE_POSE);
      }
    }, drive);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}