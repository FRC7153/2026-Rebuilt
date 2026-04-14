package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class DistanceShootCommand extends Command {
    private final Shooter shooter;
    private final Timer feedTimer = new Timer();
    private boolean feeding = false;

    public DistanceShootCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        feeding = false;
        feedTimer.reset();
        feedTimer.stop();
    }

    @Override
    public void execute() {
        boolean atSpeed = shooter.setVelocityFromLimelight();

        if (!feeding) {
            // Hold back the note while spinning up
            shooter.setKickerSpeed(-4000);
            shooter.setliveFloorSpeed(-0.7);

            if (atSpeed) {
                // Flywheel ready — start feeding
                feeding = true;
                feedTimer.reset();
                feedTimer.start();
            }
        } else {
            // Feed the note through
            shooter.setKickerSpeed(5000);
            shooter.setliveFloorSpeed(-0.7);
        }
    }

    @Override
    public boolean isFinished() {
        // Give the note enough time to fully exit after feeding starts
        return feeding && feedTimer.hasElapsed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooterSub();
    }
}