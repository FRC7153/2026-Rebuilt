package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class IntakeCommand extends Command {

    private final Intake m_intake;
    private final double m_speed;

    public IntakeCommand(Intake intake, double speed) {
        m_intake = intake;
        m_speed = speed;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.setIntakeSpeed(m_speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setIntakeSpeed(0.0); // always stop rollers when command ends
    }

    @Override
    public boolean isFinished() {
        return false; // runs until interrupted
    }
}