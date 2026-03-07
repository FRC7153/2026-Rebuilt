package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class IntakePivotCommand extends Command {

    private final Intake m_intake;
    private final double m_targetPosition;

    public IntakePivotCommand(Intake intake, double targetPosition) {
        m_intake = intake;
        m_targetPosition = targetPosition;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.setAndRememberPosition(m_targetPosition);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_intake.getIntakePivotPosition() - m_targetPosition) < 0.05;
    }
}
