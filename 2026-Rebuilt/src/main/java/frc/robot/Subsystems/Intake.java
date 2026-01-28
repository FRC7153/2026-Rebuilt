package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Util.Dashboard.HardwareFaultTracker;

public class Intake implements Subsystem {
    // Motors
    private final SparkFlex intake = new SparkFlex(HardwareConstants.INTAKE_CAN, MotorType.kBrushless);
    private final SparkFlex intakePivot = new SparkFlex(HardwareConstants.INTAKE_PIVOT_CAN, MotorType.kBrushless);

    // Encoders
    private final RelativeEncoder intakeEncoder = intake.getEncoder();
    private final RelativeEncoder intakePivotEncoder = intakePivot.getEncoder();

    //PID Controllers
    private final SparkClosedLoopController intakePivotController = intakePivot.getClosedLoopController();
    private final SparkClosedLoopController intakeController = intake.getClosedLoopController();

    //Alerts 
    private final Alert intakeAlert = new Alert("Intake Motor Error", AlertType.kError);
    private final Alert intakePivotAlert = new Alert("Intake Pivot Motor Error", AlertType.kError);

    //NT Output 
    private final DoublePublisher intakeVelocityPub;
    private final DoublePublisher intakePivotPositionPub;

    //Datalog Output
    private final DoubleLogEntry intakeVelocityLog =
        new DoubleLogEntry(DataLogManager.getLog(), "Intake/Velocity", "RPM");
    private final DoubleLogEntry intakePivotPositionLog =
        new DoubleLogEntry(DataLogManager.getLog(), "Intake/PivotPosition", "Degrees");
    
    public Intake() {
        intake.configure(IntakeConstants.INTAKE_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        intakePivot.configure(IntakeConstants.INTAKE_PIVOT_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        if (BuildConstants.PUBLISH_EVERYTHING) {
            NetworkTable nt = NetworkTableInstance.getDefault().getTable("manipulator");

            intakeVelocityPub = nt.getDoubleTopic("intakeVelocity").publish();
            intakePivotPositionPub = nt.getDoubleTopic("intakePivotPosition").publish();
        } else {
            intakeVelocityPub = null;
            intakePivotPositionPub = null;
        }
    }

    /**
     * Sets the intake velocity setpoint
     * @param velocity (RPM)
     */
    public void setIntakeVelocity(double velocity) {
        intakeController.setSetpoint(velocity, ControlType.kVelocity);
    }

    /**
     * Sets the intake pivot position setpoint
     * @param position (Degrees)
     */
    public void setIntakePivotPosition(double position) {
        intakePivotController.setSetpoint(position, ControlType.kPosition);
    }

    public void log(){
        intakeVelocityLog.append(intakeEncoder.getVelocity());
        intakePivotPositionLog.append(intakePivotEncoder.getPosition());

        if (BuildConstants.PUBLISH_EVERYTHING) {
            intakeVelocityPub.set(intakeEncoder.getVelocity());
            intakePivotPositionPub.set(intakePivotEncoder.getPosition());
        }
    }

    public void checkHardware(){
        HardwareFaultTracker.checkFault(intakeAlert, intake.hasActiveFault() || intake.hasActiveWarning());
        HardwareFaultTracker.checkFault(intakePivotAlert, intakePivot.hasActiveFault() || intakePivot.hasActiveWarning());
    }

}
