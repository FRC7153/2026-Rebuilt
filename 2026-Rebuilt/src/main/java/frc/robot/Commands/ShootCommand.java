package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Shooter;

public class ShootCommand extends RunCommand{

    /**
     * Creates a command to set the shooter, kicker, and live floor speeds. This command is intended to be used as a default command for the shooter subsystem, so that the shooter can be set to a desired speed whenever it is not being used for other commands.
     * @param shooter
     * @param shooterVelo 0.0 - 100.0 
     * @param kickerSpeed rpm
     * @param liveFloorSpeed 0 - 100%
     */
    public ShootCommand(Shooter shooter, double shooterVelo, double kickerSpeed, double liveFloorSpeed){
        super(() -> {
            shooter.setShooterSpeed(shooterVelo);
            shooter.setKickerSpeed(-800);
            new WaitCommand(3.7);
            shooter.setKickerSpeed(kickerSpeed);
            shooter.setliveFloorSpeed(liveFloorSpeed);
        }, shooter);

        addRequirements(shooter);
    }    
}
