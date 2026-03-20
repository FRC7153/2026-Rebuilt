package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Util.Dashboard.HardwareFaultTracker;

public class Intake implements Subsystem{
    private final TalonFX intakePivot = new TalonFX(HardwareConstants.INTAKE_PIVOT_CAN, HardwareConstants.CANIVORE);
    private final SparkFlex intake = new SparkFlex(HardwareConstants.INTAKE_CAN, MotorType.kBrushless);
    private final RelativeEncoder intakeEncoder = intake.getEncoder();

    private final StaticBrake staticBrakeRequest = new StaticBrake();
    
    private final StatusSignal<Angle> pivotPosition = intakePivot.getPosition();

    private final PositionVoltage pivotPositionRequest = new PositionVoltage(0.0)
        .withOverrideBrakeDurNeutral(true)
        .withSlot(0);
    
    private final MotionMagicVoltage pivotMotionMagicRequest = new MotionMagicVoltage(0.0)
        .withOverrideBrakeDurNeutral(true)
        .withSlot(0);
    
    private final Alert intakeAlert = new Alert("Intake Alert", Alert.AlertType.kError);
    private final Alert intakePivotAlert = new Alert("Intake Pivot Alert", Alert.AlertType.kError);

    private static SysIdRoutine intakePivotRoutine;

    private double m_lastPosition = 0.0;

    //NT Logging 
    private final DoublePublisher intakeVeloPub, intakeSetpointPub, intakePivotSetpointPub, intakePivotPositionPub;

    //Datalog
    private final DoubleLogEntry intakeVeloLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Intake/Velo", "RPM");
        
    private final DoubleLogEntry intakeSetpointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Intake/Setpoint", "RPM");
        
    private final DoubleLogEntry intakePivotSetpointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Intake/Pivot Setpoint", "rots");
        
    private final DoubleLogEntry intakePivotPositionLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Intake/Pivot Position", "rots");

    public Intake() {
        intakePivot.getConfigurator().apply(IntakeConstants.INTAKE_PIVOT_CONFIGS);
        intake.configure(IntakeConstants.INTAKE_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakePivot.setControl(pivotMotionMagicRequest.withPosition(0.0));
        
        if (BuildConstants.PUBLISH_EVERYTHING) {
            NetworkTable nt = NetworkTableInstance.getDefault().getTable("Shooter");
            intakeVeloPub = nt.getDoubleTopic("IntakVelo").publish();
            intakeSetpointPub = nt.getDoubleTopic("IntakeSetpoint").publish();
            intakePivotSetpointPub = nt.getDoubleTopic("IntakePivotSetpoint").publish();
            intakePivotPositionPub = nt.getDoubleTopic("IntakePivotPosition").publish();
        } else {
            intakeVeloPub = null;
            intakeSetpointPub = null;
            intakePivotSetpointPub = null;
            intakePivotPositionPub = null;
        }
    }

    public void setIntakeSpeed(double speed) {

        intake.set(speed);
        intakeSetpointLog.append(speed);

        if (BuildConstants.PUBLISH_EVERYTHING) {
            intakeSetpointPub.set(speed);
        }
    }

    public void setIntakePivotPosition(double position) {
        position = MathUtil.clamp(position, 0, 2.6);

        intakePivot.setControl(pivotMotionMagicRequest.withPosition(position));
        intakePivotSetpointLog.append(position);

        if (BuildConstants.PUBLISH_EVERYTHING) {
            intakePivotSetpointPub.set(position);
        }
    }

    public void stopIntake() {
        intake.stopMotor();
    }

    public void stopIntakePivot() {
        intakePivot.setControl(staticBrakeRequest);
    }

    public void setIntakePivotVoltage(double voltage) {
        intakePivot.setVoltage(voltage);
    }

    public void holdLastPosition() {
        intakePivot.setControl(pivotMotionMagicRequest.withPosition(m_lastPosition));
    }

    public void setAndRememberPosition(double rotations) {
        m_lastPosition = rotations;
        intakePivot.setControl(pivotMotionMagicRequest.withPosition(rotations));
    }

    public double getIntakePivotPosition() {
        return pivotPosition.getValueAsDouble();
    } 

    public double getIntakeVeloctiy() {
        return intakeEncoder.getVelocity();
    }

    public double getPivotStatorCurrent() {
        return intakePivot.getStatorCurrent().getValueAsDouble();
    }

    public void zeroPivotEncoder() {
        intakePivot.setPosition(0.0);
    }


    public static SysIdRoutine getintakePivotRoutine(Intake intake) {
        System.out.println("Starting Signal due to getIntakePivotRoutine");
        SignalLogger.start();

        if(intakePivotRoutine == null) {
            intakePivotRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.of(0.2).per(Second),
                Volts.of(2
                ), null, state -> SignalLogger.writeString("SysID_IntakePivot", state.toString())
                ), new SysIdRoutine.Mechanism((Voltage v) -> {
                    intake.setIntakePivotVoltage(v.in(Volts));
                }, null, intake)
            );
        }

        return intakePivotRoutine;
    }

    public void log() {
        pivotPosition.refresh();
        intakeEncoder.getVelocity();

        intakePivotPositionLog.append(getIntakePivotPosition());
        intakeVeloLog.append(intakeEncoder.getVelocity());

        if (BuildConstants.PUBLISH_EVERYTHING) {
            intakeVeloPub.set(intakeEncoder.getVelocity());
            intakePivotPositionPub.set(getIntakePivotPosition());
        }
    }

    public void checkHardware() {
        HardwareFaultTracker.checkFault(intakeAlert, intake.hasActiveFault() || intake.hasActiveWarning());
        HardwareFaultTracker.checkFault(intakePivotAlert, !intakePivot.isAlive() || !intakePivot.isConnected());
    }
}