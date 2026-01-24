package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class SwerveConstants {
    //Speed Configs 
    public static final double MAX_WHEEL_VELOCITY = 4.5; 

    //Max drive speeds 
    public static final double FAST_TRANSLATIONAL_SPEED = 4.5; // m/s
    public static final double FAST_ROTATIONAL_SPEED = 8.5; // rad/s

    public static final double SLOW_TRANSLATIONAL_SPEED = FAST_TRANSLATIONAL_SPEED * 0.4; //TODO
    public static final double SLOW_ROTATIONAL_SPEED = 8.5; // rad/s

    // CANCODER Offsets (in rotations, CCW+, -0.5 to 0.5 range)
    public static final double FL_CANCODER_OFFSET = -0.249268;//TODO
    public static final double FR_CANCODER_OFFSET = -0.264160;//TODO
    public static final double RL_CANCODER_OFFSET = -0.260742;//TODO
    public static final double RR_CANCODER_OFFSET = -0.399568;//TODO

    public static final double CANCODER_RANGE = 0.5;

    public static final SensorDirectionValue CANCODER_DIRECTION =
        SensorDirectionValue.Clockwise_Positive;

    //Drive Kraken x60
    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0) * Math.PI; //meters
    public static final double DRIVE_GEAR_RATIO = 8.14; //sds mk4i L1

    private static final Slot0Configs DRIVE_MOTOR_GAINS = new Slot0Configs()
        .withKP(0.3).withKI(0.0).withKD(0.0).withKS(0.0)
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

    // Steer Kraken x60 
    public static final double STEER_GEAR_RATIO = 150.0 / 7.0; //sds mk4i L1

    private static final Slot0Configs STEER_MOTOR_GAINS = new Slot0Configs()
        .withKP(0.3).withKI(0.0).withKD(0.0).withKS(0.0)
        .withKV(0.0).withKA(0.0);//TODO

    private static final CurrentLimitsConfigs STEER_MOTOR_CURRENT_LIMITS =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(20)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(40)
            .withStatorCurrentLimitEnable(true);//TODO

    private static final FeedbackConfigs STEER_ENCODER = new FeedbackConfigs()
        .withSensorToMechanismRatio(STEER_GEAR_RATIO);

    public static final TalonFXConfiguration STEER_CONFIG = new TalonFXConfiguration()
        .withSlot0(STEER_MOTOR_GAINS)
        .withCurrentLimits(STEER_MOTOR_CURRENT_LIMITS)
        .withFeedback(STEER_ENCODER);

    // Odometry
    public static final Matrix<N3, N1> STATE_STD_DEVS = VecBuilder.fill(0.4, 0.4, 0.01); //TODO

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
    public static final Translation2d[] POSITIONS = {
        new Translation2d(BASE_DIMENSIONS.getX() / 2.0 - EDGE, BASE_DIMENSIONS.getY() / 2.0 - EDGE), // FL
        new Translation2d(BASE_DIMENSIONS.getX() / 2.0 - EDGE, BASE_DIMENSIONS.getY() / -2.0 + EDGE), // FR
        new Translation2d(BASE_DIMENSIONS.getX() / -2.0 + EDGE, BASE_DIMENSIONS.getY() / 2.0 - EDGE), // RL
        new Translation2d(BASE_DIMENSIONS.getX() / -2.0 + EDGE, BASE_DIMENSIONS.getY() / -2.0 + EDGE)  // RR
  };               

    // Default positions (against the front reef), if none is set by the auto program by PREGAME
    public static final Pose2d DEFAULT_BLUE_POSE = new Pose2d(3.6 - (BASE_DIMENSIONS.getX() / 2.0) - BUMPER_WIDTH, 4.026, Rotation2d.kZero);//TODO
    public static final Pose2d DEFAULT_RED_POSE = FlippingUtil.flipFieldPose(DEFAULT_BLUE_POSE);//TODO
}
