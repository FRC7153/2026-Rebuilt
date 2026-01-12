package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class SwerveConstants {
    //Speed Configs 
    public static final double MAX_WHEEL_VELOCITY = 0.0; //TODO

    //Max drive speeds 
    public static final double FAST_TRANLATIONAL_SPEED = 0.0; //TODO m/s
    public static final double FAST_ROTATIONAL_SPEED = 0.0; //TODO rad/s

    public static final double SLOW_TRANSLATIONAL_SPEED = 0.0; //TODO
    public static final double SLOW_ROTATIONAL_SPEED = 0.0; //TODO rad/s

    // CANCODER Offsets (in rotations, CCW+, -0.5 to 0.5 range)
    public static final double FL_CANCODER_OFFSET = 0.0;//TODO
    public static final double FR_CANCODER_OFFSET = 0.0;//TODO
    public static final double RL_CANCODER_OFFSET = 0.0;//TODO
    public static final double RR_CANCODER_OFFSET = 0.0;//TODO

    public static final double CANCODER_RANGE = 0.5;

    public static final SensorDirectionValue CANCODER_DIRECTION =
        SensorDirectionValue.Clockwise_Positive;

    //Drive Kraken x60
    public static final double WHEEL_CIRCUMFERENCE = 0.0; //TODO meters
    public static final double DRIVE_GEAR_RATIO = 0.0; //TODO

    private static final Slot0Configs DRIVE_MOTOR_GAINS = new Slot0Configs()
        .withKP(0.0).withKI(0.0).withKD(0.0).withKS(0.0)
        .withKV(0.0).withKA(0.0);//TODO

    private static final CurrentLimitsConfigs DRIVE_MOTOR_CURRENT_LIMITS =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(45)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true);//TODO

    private static final FeedbackConfigs DRIVE_ENCODER = new FeedbackConfigs()
        .withSensorToMechanismRatio(DRIVE_GEAR_RATIO);

    public static final TalonFXConfiguration DRIVE_CONFIG = new TalonFXConfiguration()
        .withSlot0(DRIVE_MOTOR_GAINS)
        .withCurrentLimits(DRIVE_MOTOR_CURRENT_LIMITS)
        .withFeedback(DRIVE_ENCODER);

    // Steer NEO 
    public static final double STEER_GEAR_RATIO = 0.0; //TODO

    public static final SparkBaseConfig STEER_CONFIG = new SparkFlexConfig()
        .inverted(false)//TODO
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake)
        .apply(new ClosedLoopConfig()
            .pid(0.0, 0.0,0.0, ClosedLoopSlot.kSlot0)
            .positionWrappingInputRange(-0.5 * STEER_GEAR_RATIO, 0.5 * STEER_GEAR_RATIO)
            .positionWrappingEnabled(true));

    //Base size
    public static final Translation2d BASE_DIMENSIONS = 
        new Translation2d(Units.inchesToMeters(27), Units.inchesToMeters(27));

    private static final double EDGE = Units.inchesToMeters(2.625); //Distance edge of module to center of wheel
    public static final double BUMPER_WIDTH = Units.inchesToMeters(0.0); //TODO

    //Module Locations
    /**
     * Positive x values represent moving toward the front of the robot whereas positive y values 
     * represent moving toward the left of the robot.
     */
    public static final Translation2d FL_MODULE_LOCATION = 
        new Translation2d(BASE_DIMENSIONS.getX() / 2.0 - EDGE,
                          BASE_DIMENSIONS.getY() / 2.0 - EDGE);
    public static final Translation2d FR_MODULE_LOCATION = 
        new Translation2d(BASE_DIMENSIONS.getX() / 2.0 - EDGE,
                          BASE_DIMENSIONS.getY() / -2.0 + EDGE);
    public static final Translation2d RL_MODULE_LOCATION =
        new Translation2d(BASE_DIMENSIONS.getX() / -2.0 + EDGE,
                          BASE_DIMENSIONS.getY() / 2.0 - EDGE);
    public static final Translation2d RR_MODULE_LOCATION =
        new Translation2d(BASE_DIMENSIONS.getX() / -2.0 + EDGE,
                          BASE_DIMENSIONS.getY() / -2.0 + EDGE);                
}
