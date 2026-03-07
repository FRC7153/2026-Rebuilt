package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants;
import frc.robot.Subsystems.Climber;

public class RetractClimberCommand extends Command {
    private final Climber m_climber;

    public RetractClimberCommand(Climber climber) {
        m_climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        m_climber.retract();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_climber.getClimberPosition() - RobotConstants.CLIMBER_RETRACT) < 0.5;
    }

    @Override
    public void end(boolean interrupted) {
        // position PID holds it — no need to stop
    }
}
