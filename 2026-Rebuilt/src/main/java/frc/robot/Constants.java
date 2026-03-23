package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class Constants {
    public class AprilTagConstants {
        public static final String LL_4_FRONT = "limelight-front";
        public static final String LL_3_BACK = "limelight-back";

        public static final double FRONT_CAMERA_HEIGHT = 0.55; //TODO Meters
        public static final double TARGET_HEIGHT = 1.12; //TODO Meters
        public static final double CAMERA_ANGLE_DEG = 0.0; //TODO Degrees

        public static final AprilTagFieldLayout FIELD =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);//TODO 
    }
    public class BuildConstants {
        public static final boolean PUBLISH_EVERYTHING = false;
        public static final boolean INCLUDE_TEST_AUTOS = false;
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

        // Intake Subsystem CAN IDs
        public static final int INTAKE_CAN = 13;
        public static final int INTAKE_PIVOT_CAN = 14;

        //Shooter Subsystem CAN IDs 
        public static final int SHOOTER_CAN = 15;
        public static final int SHOOTER_2_CAN = 16;
        public static final int KICKER_CAN = 17;
        public static final int LIVEFLOOR_CAN = 18;

        // Climber Subsystem CAN IDs
        public static final int CLIMBER_CAN = 19;

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
        public static final double INTAKE_PIVOT_GEAR_RATIO = 15.0 / 1.0; 

        public static final Slot0Configs INTAKE_PIVOT_GAINS = new Slot0Configs()
            .withKP(80.0).withKI(0.0).withKD(4.0)
            .withKS(0.25).withKV(2.5).withKA(0.14118)
            .withKG(0.3).withGravityType(GravityTypeValue.Arm_Cosine); 
        
        public static final MotionMagicConfigs intakePivotMotionMagiv = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(40)
            .withMotionMagicAcceleration(120)
            .withMotionMagicJerk(1000);
        
        public static final CurrentLimitsConfigs INTAKE_PIVOT_CURRENT_LIMITS_CONFIGS =
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(60) //TODO Old was 60
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(80)
                .withStatorCurrentLimitEnable(true);
        
        public static final FeedbackConfigs INTAKE_PIVOT_ENCODER_CONFIGS = new FeedbackConfigs()
            .withSensorToMechanismRatio(INTAKE_PIVOT_GEAR_RATIO);
        
        public static final MotorOutputConfigs INTAKE_PIVOT_OUTPUT_CONFIGS = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive) 
            .withNeutralMode(NeutralModeValue.Brake);
        
        public static final SoftwareLimitSwitchConfigs INTAKE_SOFWARE_CONFIGS = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(false)
            .withForwardSoftLimitThreshold(0.222) //TODO
            .withReverseSoftLimitEnable(false)
            .withReverseSoftLimitThreshold(0.0025); //TODO
        
        // Intake Pivot Configs Kraken x60 pivot 
        public static final TalonFXConfiguration INTAKE_PIVOT_CONFIGS = new TalonFXConfiguration()
            .withSlot0(INTAKE_PIVOT_GAINS)
            .withCurrentLimits(INTAKE_PIVOT_CURRENT_LIMITS_CONFIGS)
            .withFeedback(INTAKE_PIVOT_ENCODER_CONFIGS)
            .withMotorOutput(INTAKE_PIVOT_OUTPUT_CONFIGS)
            .withMotionMagic(intakePivotMotionMagiv)
            .withSoftwareLimitSwitch(INTAKE_SOFWARE_CONFIGS);

        public static final SparkBaseConfig INTAKE_CONFIG = new SparkFlexConfig()
            .idleMode(IdleMode.kBrake)
            .inverted(false)//TODO
            .smartCurrentLimit(30);
    }
    
    public class ShooterConstants {
        // Shooter gear ratio 
        public static final double SHOOTER_GEAR_RATIO = 1.0 / 1.0;

        public static final double LIVEFLOOR_GEAR_RATIO = 25.0 / 1.0;//TODO

        public static final Slot0Configs SHOOTER_GAINS = new Slot0Configs()
            .withKP(0.21896).withKI(0.0).withKD(0.0).withKS(0.29926)
            .withKV(0.13849).withKA(0.068329);//TODO

        private static final CurrentLimitsConfigs SHOOTER_CURRENT_LIMITS_CONFIGS =
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(60)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(40)
                .withSupplyCurrentLowerTime(1)//Seconds
                .withStatorCurrentLimit(80)
                .withStatorCurrentLimitEnable(true);
        
        private static final VoltageConfigs SHOOTER_VOLTAGE_CONFIGS = new VoltageConfigs()
            .withPeakForwardVoltage(12.0) // Volts
            .withPeakReverseVoltage(12.0); // Volts

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
        
        public static final FeedForwardConfig KICKER_FF_CONFIGS = new FeedForwardConfig()
            .sva(0.0074999, 0.0002, 0.00001195, ClosedLoopSlot.kSlot0); //TODO
            
        public static final ClosedLoopConfig KICKER_PID = new ClosedLoopConfig()
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.0004, 0.0, 0.0025)
                .apply(KICKER_FF_CONFIGS); 
        
        // Kicker Configs Neo vortex  
        public static final SparkBaseConfig KICKER_CONFIG = new SparkFlexConfig()
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit(40)
                .apply(KICKER_PID);
        
        public static final ClosedLoopConfig LIVEFLOOR_PID = new ClosedLoopConfig()
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.0, 0.0, 0.0); //TODO
        
        public static final EncoderConfig LIVEFLOOR_ENCODER_CONFIG = new EncoderConfig()
                .velocityConversionFactor(1.0 / LIVEFLOOR_GEAR_RATIO);
                
        public static final SparkBaseConfig LIVEFLOOR_CONFIG = new SparkFlexConfig()
                .idleMode(IdleMode.kBrake)
                .inverted(false)
                .smartCurrentLimit(30)
                .apply(LIVEFLOOR_PID);
    }
    public class RobotConstants {
        // Intake Robot Constants
        public static final double INTAKE_PIVOT_STOW = 0.0008; 
        public static final double INTAKE_PIVOT_EXTEND = 0.245; 
        public static final double INTAKE_EXTEND_SPEED = -0.4;
    }
} 