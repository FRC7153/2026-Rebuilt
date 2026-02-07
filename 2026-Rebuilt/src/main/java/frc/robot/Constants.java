package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class Constants {
    public class BuildConstants {
        public static final boolean PUBLISH_EVERYTHING = true;
        public static final boolean INCLUDE_TEST_AUTOS = true;

        public static final AprilTagFieldLayout FIELD =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);//TODO 
    }

    public class DashboardConstants {
        public static final int ELASTIC_SERVER_PORT = 5800;
    }
    
    public class HardwareConstants {
        // Swerve Hardware CAN IDs
        public final static int FR_DRIVE_CAN = 1;
        public final static int FR_STEER_CAN = 2;
        public final static int FL_DRIVE_CAN = 3;
        public final static int FL_STEER_CAN = 4;
        public final static int RL_DRIVE_CAN = 5;
        public final static int RL_STEER_CAN = 6;
        public final static int RR_DRIVE_CAN = 7;
        public final static int RR_STEER_CAN = 8;

        // CANCoder CAN IDs
        public final static int FR_CANCODER_CAN = 9;
        public final static int FL_CANCODER_CAN = 10;
        public final static int RL_CANCODER_CAN = 11;
        public final static int RR_CANCODER_CAN = 12;

        // Cancoder CAN IDs
        public static final int INTAKE_CAN = 13;
        public static final int INTAKE_PIVOT_CAN = 14;

        //Cancoder CAN IDs 
        public static final int SHOOTER_CAN = 15;
        public static final int KICKER_CAN = 16;
        public static final int SHOOTER_2_CAN = 17;

        // Pigeon 2.0
        public final static int PIGEON_CAN_ID = 20;
        // PDH Can ID
        public static final int PDH_CAN = 21;

        // CAN Busses
        public final static CANBus CANIVORE = new CANBus("CANivore");
        public final static CANBus RIO_CAN = new CANBus("rio");
    }

    public class IntakeConstants {
        // Intake gear ratios
        public static final double INTAKE_GEAR_RATIO = 1.0 / 1.0; //TODO
                
        public static final ClosedLoopConfig INTAKE_PIVOT_PID = new ClosedLoopConfig()
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.0, 0.0, 0.0); //TODO
        
        public static final ClosedLoopConfig INTAKE_PID = new ClosedLoopConfig()
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.0, 0.0, 0.0); //TODO
        
        // Intake Configs Neo vortex pivot 
        public static final SparkBaseConfig INTAKE_PIVOT_CONFIG = new SparkFlexConfig()
            .idleMode(IdleMode.kBrake)
            .inverted(false)//TODO
            .smartCurrentLimit(50)//TODO
            .apply(INTAKE_PIVOT_PID);
        
        public static final SparkBaseConfig INTAKE_CONFIG = new SparkFlexConfig()
            .idleMode(IdleMode.kBrake)
            .inverted(false)//TODO
            .smartCurrentLimit(50)// TODO
            .apply(INTAKE_PID);
    }
    public class ShooterConstants {
    // Shooter gear ratio 
    public static final double SHOOTER_GEAR_RATIO = 1.0 / 1.0;

    public static final Slot0Configs SHOOTER_GAINS = new Slot0Configs()
        .withKP(0.2).withKI(0.0).withKD(0.0).withKS(0.2)
        .withKV(0.12).withKA(0.0);//TODO

    private static final CurrentLimitsConfigs SHOOTER_CURRENT_LIMITS_CONFIGS =
        new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(60)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true);//TODO

    private static final FeedbackConfigs SHOOTER_ENCODER_CONFIGS = new FeedbackConfigs()
        .withSensorToMechanismRatio(SHOOTER_GEAR_RATIO);

    private static final MotorOutputConfigs SHOOTER_OUTPUT_CONFIGS = new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive) //TODO
        .withNeutralMode(NeutralModeValue.Brake);

    public static final TalonFXConfiguration SHOOTER_CONFIG = new TalonFXConfiguration()
        .withSlot0(SHOOTER_GAINS)
        .withCurrentLimits(SHOOTER_CURRENT_LIMITS_CONFIGS)
        .withFeedback(SHOOTER_ENCODER_CONFIGS)
        .withMotorOutput(SHOOTER_OUTPUT_CONFIGS);
    
    private static final Slot0Configs KICKER_GAINS =new Slot0Configs()
        .withKP(0.15).withKI(0.0).withKD(0.0)
        .withKS(0.2).withKV(0.12).withKA(0.0);//TODO

    private static final MotorOutputConfigs KICKER_OUTPUT_CONFIGS = new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive) //TODO
        .withNeutralMode(NeutralModeValue.Brake); 

    public static final TalonFXConfiguration KICKER_CONFIG = new TalonFXConfiguration()
        .withSlot0(KICKER_GAINS)
        .withCurrentLimits(SHOOTER_CURRENT_LIMITS_CONFIGS)
        .withFeedback(SHOOTER_ENCODER_CONFIGS)
        .withMotorOutput(KICKER_OUTPUT_CONFIGS);
    }
}
