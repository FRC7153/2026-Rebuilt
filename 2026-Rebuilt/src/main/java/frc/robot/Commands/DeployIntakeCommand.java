package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class DeployIntakeCommand extends Command {

    private final Intake m_intake;
    private final double m_pivotPosition;
    private final double m_rollerSpeed;

    public DeployIntakeCommand(Intake intake, double pivotPosition, double rollerSpeed) {
        m_intake = intake;
        m_pivotPosition = pivotPosition;
        m_rollerSpeed = rollerSpeed;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.setAndRememberPosition(m_pivotPosition);
        m_intake.setIntakeSpeed(m_rollerSpeed);
    }


    @Override
    public void end(boolean interrupted) {
        m_intake.setIntakeSpeed(0.0); // always stop rollers
    }

    @Override
    public boolean isFinished() {
        return false; // runs until interrupted
    }
}