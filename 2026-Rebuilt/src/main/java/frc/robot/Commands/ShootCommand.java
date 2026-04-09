package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Shooter;
    /**
     * Creates a command to set the shooter, kicker, and live floor speeds. This command is intended to be used as a default command for the shooter subsystem, so that the shooter can be set to a desired speed whenever it is not being used for other commands.
     * @param shooter
     * @param shooterVelo 0.0 - 100.0 
     * @param kickerSpeed 0-100%
     * @param liveFloorSpeed 0 - 100%
     */
public class ShootCommand extends RunCommand {

    private static final double VELOCITY_TOLERANCE = 5;

    public ShootCommand(Shooter shooter, double shooterVelo, double kickerSpeed, double liveFloorSpeed) {
        super(() -> {
            // Always spin shooter
            shooter.setShooterSpeed(shooterVelo);

            // Kicker & floor logic
            if (Math.abs(shooter.getShooterVelocity() - shooterVelo) > VELOCITY_TOLERANCE) {
                // Reverse kicker until shooter ready
                shooter.setKickerSpeed(-4000);
                shooter.setliveFloorSpeed(liveFloorSpeed);
            } else {
                // Shooter ready → feed
                new WaitCommand(0.01);
                shooter.setKickerVelo(kickerSpeed * 10000);
                shooter.setliveFloorSpeed(liveFloorSpeed);
            }
        }, shooter); // Only requires shooter
    }
}

