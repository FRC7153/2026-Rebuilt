package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class HomeIntakeCommand extends Command {
    
    private Intake m_intake;

    private static final double kHomingOutput     = -0.07;
    private static final double kCurrentThreshold = 20.0;
    private static final int    kCurrentSpikeLoops = 5;

    private int m_spikeCount = 0;
    private boolean m_hasHomed = false;

    public HomeIntakeCommand(Intake intake) {  
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_spikeCount = 0;
        m_hasHomed = false;
        m_intake.setIntakePivotVoltage(kHomingOutput * 12.0);
    }

    @Override
    public void execute() {
        double current = m_intake.getPivotStatorCurrent();

        if (current > kCurrentThreshold) {
            m_spikeCount++;
        } else {
            m_spikeCount = 0;
        }

        if (m_spikeCount >= kCurrentSpikeLoops && !m_hasHomed) {
            m_intake.zeroPivotEncoder();
            m_hasHomed = true;
            System.out.println("Homed Intake Pivot"); // ✅ only prints once
        }
    }

    @Override
    public boolean isFinished() {
        return m_hasHomed;  // ✅ no print spam
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setIntakePivotVoltage(0.0);
        if (!interrupted) {
            System.out.println("Homing complete");
        }
    }
}