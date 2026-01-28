package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Util.Dashboard.HardwareFaultTracker;

public class SwerveModule {
    private final String name; 

    //Alert 
    private final Alert driveMotorAlert, steerMotorAlert, steerCANCoderAlert, steerNotHomedAlert;

    //Drive Hardware 
    private final TalonFX driveMotor;
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Current> driveCurrent;

    private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0.0)
        .withOverrideBrakeDurNeutral(true)
        .withSlot(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0.0)
        .withOverrideBrakeDurNeutral(true);
    private final DutyCycleOut driveDutyCycleRequest = new DutyCycleOut(0.0)
        .withOverrideBrakeDurNeutral(true);
    private final StaticBrake driveBrakeRequest = new StaticBrake();


    //Steer Hardware
    private final TalonFX steerMotor;
    private boolean hasBuiltInEncoderHomed = false;
    private final StatusSignal<Angle> steerBuiltInAngle;

    private final VoltageOut steerAngleRequest = new VoltageOut(0.0)
        .withOverrideBrakeDurNeutral(true);
    private final PositionVoltage steerPositionRequest = new PositionVoltage(0.0)
        .withOverrideBrakeDurNeutral(true)
        .withSlot(0);
    private final DutyCycleOut steerDutyCycleRequest = new DutyCycleOut(0.0)
        .withOverrideBrakeDurNeutral(true);
        
    //CANCoder
    private final CANcoder steerCANCoder;
    private final StatusSignal<Angle> steerAngle;
    

    /** State (for logging) */
    public final SwerveModuleState state = new SwerveModuleState();
    private SwerveModuleState lastStateRequest = new SwerveModuleState();

    // Logging
    private final DoublePublisher currentPub;
    private final DoubleLogEntry currentLog;

    public SwerveModule(
        String name,
        int driveMotorID,
        int steerMotorID,
        int steerEncoderID,
        double steerCancoderOffsetRots
    ) {
        this.name = name;

        //Alerts 
        driveMotorAlert = new Alert("Swerve Module " + name + " drive motor error", AlertType.kError);
        steerMotorAlert = new Alert("Swerve Module " + name + " steer motor error", AlertType.kError);
        steerCANCoderAlert = new Alert("Swerve Module " + name + " steer CANCoder error", AlertType.kError);
        steerNotHomedAlert = new Alert("Swerve Module " + name + " steer encoder failed to home", AlertType.kError);
        
        //Initialize Drive Motor
        driveMotor = new TalonFX(driveMotorID, HardwareConstants.CANIVORE);

        driveMotor.getConfigurator().apply(SwerveConstants.DRIVE_CONFIG);
        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        driveCurrent = driveMotor.getStatorCurrent();

        //Initialize Steer Encoder 
        steerCANCoder = new CANcoder(steerEncoderID, HardwareConstants.CANIVORE);

        MagnetSensorConfigs cancoderConfig = new MagnetSensorConfigs()
            .withAbsoluteSensorDiscontinuityPoint(SwerveConstants.CANCODER_RANGE)
            .withSensorDirection(SwerveConstants.CANCODER_DIRECTION)
            .withMagnetOffset(steerCancoderOffsetRots);

        steerCANCoder.getConfigurator().apply(cancoderConfig);
        steerAngle = steerCANCoder.getAbsolutePosition();

        //Initialize Steer Motor
        steerMotor = new TalonFX(steerMotorID, HardwareConstants.CANIVORE);

        steerMotor.getConfigurator().apply(SwerveConstants.STEER_CONFIG);
        steerMotor.getVelocity();
        steerMotor.getStatorCurrent();
        steerBuiltInAngle = steerMotor.getPosition();

        //Default State
        setBrakeMode(true);
        setRequest(new SwerveModuleState(), true);

        // Initialize Logging
        currentLog = new DoubleLogEntry(DataLogManager.getLog(), "Swerve/Currents/" + name);

        NetworkTable nt = NetworkTableInstance.getDefault().getTable("Swerve/Currents");
        currentPub = nt.getDoubleTopic(name).publish();
    }


    public StatusSignal<Angle> getDrivePosition() {
        return drivePosition.clone();
    }


    public StatusSignal<Angle> getSteerAngle() {
        return steerAngle.clone();
    }
    /**
     * Homes the steer Kraken's built in relative encoder.
     */
    public void homeEncoder() {
        steerAngle.refresh();
            System.out.printf(
      "SwerveModule %s homed from %f deg to %f deg.\n",
        name,
        steerBuiltInAngle.getValueAsDouble() / SwerveConstants.STEER_GEAR_RATIO * 360.0,
        steerAngle.getValueAsDouble() * 360.0
    );

        StatusCode resp = steerMotor.setPosition(steerAngle.getValueAsDouble() * SwerveConstants.STEER_GEAR_RATIO);
        steerNotHomedAlert.set(!resp.isOK());
        hasBuiltInEncoderHomed = resp.isOK();
    }

    public void setBrakeMode(boolean brake) {
        steerMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        driveMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
    public void setRequest(SwerveModuleState request, boolean closedLoop) {
        //Do not run if Kraken's built in encoder is not homed
        if (!hasBuiltInEncoderHomed) return;

        //Optimize State
        double initialVelocity = request.speedMetersPerSecond;
        steerAngle.refresh();
        Rotation2d currentAngle = Rotation2d.fromRotations(steerAngle.getValueAsDouble());
        request.optimize(currentAngle);
        request.cosineScale(currentAngle);
        lastStateRequest = request;

        if (Math.abs(initialVelocity) < 0.01) {
            // No Movement, apply deadband
            driveMotor.setControl(driveBrakeRequest);
        } else {
            // Set Angle
            steerMotor.setControl(steerPositionRequest.withPosition(
                request.angle.getRotations()
            ));
            if (closedLoop) {
                // Closed Loop Velocity Control (Auto)
                driveMotor.setControl(driveVelocityRequest.withVelocity(
                    request.speedMetersPerSecond / SwerveConstants.WHEEL_CIRCUMFERENCE
                ));
            } else {
                // Open Loop Voltage Control (Teleop)
                driveMotor.setControl(driveDutyCycleRequest.withOutput(
                    request.speedMetersPerSecond / SwerveConstants.MAX_WHEEL_VELOCITY
                ));
            }
        }
    }

    /**
     * Steer heading will default to 0.0 degrees (straight forward)
     * @param voltage Voltage request (for characterization)
     */
    public void setVoltageRequest(double voltage){
        driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
        steerMotor.setControl(steerPositionRequest.withPosition(0.0));
    }

    /**
     * Runs both motors at the given duty cycle (for testing)
     * @param dutyCycle -1.0 to 1.0 (%)
     */
    public void runAllAtDutyCycle(double dutyCycle) {
        driveMotor.setControl(driveDutyCycleRequest.withOutput(dutyCycle));
        steerMotor.setControl(steerDutyCycleRequest.withOutput(dutyCycle));
    }

    /**
     * Sets both motors to brake mode
     */
    public void staticBrakeRequest() {
        driveMotor.setControl(driveBrakeRequest);
        lastStateRequest = new SwerveModuleState();
    }

    /**
     * Sets the steer motor to a voltage output (for characterization)
     * @param voltage Voltage output
     */
    public void setSteerVoltage(double voltage) {
        driveMotor.setControl(driveBrakeRequest);
        steerMotor.setControl(steerAngleRequest.withOutput(voltage));
    }

    /**
     * This will UPDATE the CURRENT STATE {@code this.state} in place, and RETURN the REQUESTED STATE.
     * @return The last requested state via {@code setRequest()}
     */
    public SwerveModuleState getAndUpdateStates(){
        //Refresh Signals
        driveVelocity.refresh();
        steerAngle.refresh();

        //Update State
        state.angle = Rotation2d.fromRotations(steerAngle.getValueAsDouble());
        state.speedMetersPerSecond = driveVelocity.getValueAsDouble() * SwerveConstants.WHEEL_CIRCUMFERENCE;

        // Return last request
        return lastStateRequest;
    }

    public void log(){
        driveCurrent.refresh();
        currentLog.append(driveCurrent.getValueAsDouble());
        currentPub.set(driveCurrent.getValueAsDouble());
    }

    public void checkHardware(){
        //Check Drive Motor
        HardwareFaultTracker.checkFault(driveMotorAlert, !driveMotor.isAlive() || !driveMotor.isConnected());

        // Check Steer Motor Faults
        HardwareFaultTracker.checkFault(steerMotorAlert, !steerMotor.isAlive() || !steerMotor.isConnected());

        // Check Steer CANCoder Faults
        HardwareFaultTracker.checkFault(steerCANCoderAlert, !steerCANCoder.isConnected());
    }


}