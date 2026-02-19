package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.urcl.URCL;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.HardwareConstants;

public class Climber implements Subsystem {
    private final SparkFlex climber = new SparkFlex(HardwareConstants.CLIMBER_CAN, MotorType.kBrushless);

    private final RelativeEncoder climberEncoder = climber.getEncoder();
    private final SparkClosedLoopController climberController = climber.getClosedLoopController();

    private static SysIdRoutine climberRoutine;

    // NT Logging 
    private final DoublePublisher climberSetpointPub, climberPositionPub;
    
    //Datalog

    private final DoubleLogEntry climberPositionLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Climber/Position", "Rotations");
    
    private final DoubleLogEntry climberSetpointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Climber/Setpoint", "Rotations");

    public Climber() {
        climber.configure(ClimberConstants.CLIMBER_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (BuildConstants.PUBLISH_EVERYTHING) {
            NetworkTable nt = NetworkTableInstance.getDefault().getTable("Climber");
            climberSetpointPub = nt.getDoubleTopic("Setpoint").publish();
            climberPositionPub = nt.getDoubleTopic("Position").publish();
        } else {
            climberSetpointPub = null;
            climberPositionPub = null;
        } 
    } 

    /**
     * Sets the climber to a specific position.
     */
    public void setClimberPosition(double position) {
        climberController.setSetpoint(position, ControlType.kPosition);

        climberSetpointLog.append(position);
        if(BuildConstants.PUBLISH_EVERYTHING) {
            climberSetpointPub.set(position);
        }
    }


    public void stopClimber(){
        climber.stopMotor();
    }

    private void setVoltage(double voltage) {
        climber.setVoltage(voltage);
    }

    public static SysIdRoutine getClimberRoutine(Climber climber) {
        System.out.println("Starting URCL SignalLogger due to getClimberRoutine");
        DataLogManager.start();
        URCL.start();

        if (climberRoutine == null){
            climberRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.of(0.25).per(Second),
                Volts.of(6.0),
                Seconds.of(10)), 
                new SysIdRoutine.Mechanism((voltage) -> {
                    climber.setVoltage(voltage.in(Volts));
                },
                null, climber));
        }

        return climberRoutine;
    }

    public void log() {
        climberEncoder.getVelocity();

        climberPositionLog.append(climberEncoder.getPosition());

        if (BuildConstants.PUBLISH_EVERYTHING) {
            climberPositionPub.set(climberEncoder.getPosition());
        }
    }
}