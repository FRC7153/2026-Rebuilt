package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class HoldIntakeCommand extends Command {
    private final Intake m_intake;

    public HoldIntakeCommand(Intake intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        m_intake.holdLastPosition(); // runs every loop
    }

    @Override
    public boolean isFinished() {
        return false; // runs forever as default
    }
}
