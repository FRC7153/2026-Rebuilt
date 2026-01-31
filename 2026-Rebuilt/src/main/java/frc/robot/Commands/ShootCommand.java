package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Shooter;

public class ShootCommand extends SequentialCommandGroup{

    public ShootCommand(Shooter shooter){
        super(
            new InstantCommand(() -> shooter.setShooterSpeed(70)),
            new InstantCommand(() -> shooter.setKickerSpeed(50))
        );
    }

    
}
