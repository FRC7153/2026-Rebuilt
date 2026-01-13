package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.HardwareConstants;

public class SwerveModule {
    private final String name; 

    //Alert 
    private final Alert driveMotorAlert, steerMotorAlert, steerCANCoderAlert, steerNotHomedAlert;

    //Drive Hardware 
    private final TalonFX driveMotor;
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Current> driveCurrent;

    private final VelocityVoltage driveVelocitRequest = new VelocityVoltage(null)
        .withOverrideBrakeDurNeutral(true)
        .withSlot(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0.0)
        .withOverrideBrakeDurNeutral(true);
    private  final DutyCycleOut driveDutyCycleRequest = new DutyCycleOut(0.0)
        .withOverrideBrakeDurNeutral(true);
    private final StaticBrake driveBrakeRequest = new StaticBrake();


    //Steer Hardware
    private final SparkMax steerMotor;
    private final SparkClosedLoopController steerPID;
    private final RelativeEncoder steerBuiltInEncoder;
    private boolean hasBuiltInEncoderHomed = false;

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
        steerMotor = new SparkMax(steerMotorID, MotorType.kBrushless);
        steerMotor.configure(SwerveConstants.STEER_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        steerBuiltInEncoder = steerMotor.getEncoder();

    }
}