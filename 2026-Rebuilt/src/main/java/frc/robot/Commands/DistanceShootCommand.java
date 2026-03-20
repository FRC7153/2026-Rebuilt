package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
      // Shooter ready → feed
      new WaitCommand(1);
      shooter.setKickerSpeed(0.5);
      shooter.setliveFloorSpeed(-0.7);
  }

  public boolean isFinished() {
    // isFinished = shooter at speed AND you trigger the feed
    return shooter.setVelocityFromLimelight();
  }

  public void end(boolean interrupted) {
    shooter.stopShooterSub();
  }
}

