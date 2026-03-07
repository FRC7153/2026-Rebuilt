package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class DistanceShootCommand extends Command {
      private final Shooter shooter;

  public DistanceShootCommand(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  public void execute() {
    // Spin up to regression-predicted velocity each loop
    shooter.setVelocityFromLimelight();
  }

  public boolean isFinished() {
    // isFinished = shooter at speed AND you trigger the feed
    return shooter.setVelocityFromLimelight();
  }

  public void end(boolean interrupted) {
    shooter.stopShooterSub();
  }
}

