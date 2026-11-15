"""Robot-wide constants (port of Constants.java)."""

from phoenix6 import CANBus, configs, signals
from rev import FeedbackSensor, SparkBaseConfig, SparkFlexConfig
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout


# Velocity feedforward for a NEO Vortex in RPM velocity mode: 12 V spread over its
# 6784 RPM free speed. Without it the P-only loops only reached ~40% of the setpoint.
VORTEX_KV = 12.0 / 6784.0  # volts per RPM


class AprilTagConstants:
    LL_4_FRONT = "limelight-front"
    LL_3_BACK = "limelight-back"

    FRONT_CAMERA_HEIGHT = 0.73025  # TODO meters
    TARGET_HEIGHT = 1.12  # TODO meters
    CAMERA_ANGLE_DEG = 10.0  # TODO degrees

    FIELD = AprilTagFieldLayout.loadField(AprilTagField.kDefaultField)  # TODO


class BuildConstants:
    PUBLISH_EVERYTHING = False
    INCLUDE_TEST_AUTOS = False


class DashboardConstants:
    ELASTIC_SERVER_PORT = 5800
    # Name of the Elastic tab to switch to when teleop starts
    TELEOP_TAB = "Teleoperated"


class HardwareConstants:
    # Swerve hardware CAN IDs.
    # NOTE: the swerve modules are actually built from the IDs in
    # subsystems/swerve/tuner_constants.py; these are kept for reference only.
    FR_DRIVE_CAN = 1
    FR_STEER_CAN = 2
    FL_DRIVE_CAN = 3
    FL_STEER_CAN = 4
    RL_DRIVE_CAN = 5
    RL_STEER_CAN = 6
    RR_DRIVE_CAN = 7
    RR_STEER_CAN = 8

    # CANcoder CAN IDs
    FR_CANCODER_CAN = 9
    FL_CANCODER_CAN = 10
    RL_CANCODER_CAN = 11
    RR_CANCODER_CAN = 12

    # Intake subsystem CAN IDs
    INTAKE_CAN = 13
    INTAKE_PIVOT_CAN = 14
    INTAKE_FOLLOWER_CAN = 22

    # Shooter subsystem CAN IDs
    SHOOTER_CAN = 15
    SHOOTER_2_CAN = 16
    KICKER_CAN = 17
    LIVEFLOOR_CAN = 18

    # Climber subsystem CAN IDs
    CLIMBER_CAN = 19

    # Pigeon 2.0
    PIGEON_CAN_ID = 20

    # PDH CAN ID
    PDH_CAN = 21

    # CAN busses
    CANIVORE = CANBus("CANivore")
    RIO_CAN = CANBus("rio")


class IntakeConstants:
    # Intake gear ratios
    INTAKE_PIVOT_GEAR_RATIO = 15.0 / 1.0

    INTAKE_PIVOT_GAINS = (
        configs.Slot0Configs()
        .with_k_p(80.0)
        .with_k_i(0.0)
        .with_k_d(4.0)
        .with_k_s(0.3)
        .with_k_v(2.5)
        .with_k_a(0.14118)
        .with_k_g(2.0)
        .with_gravity_type(signals.GravityTypeValue.ARM_COSINE)
    )

    INTAKE_PIVOT_MOTION_MAGIC = (
        configs.MotionMagicConfigs()
        .with_motion_magic_cruise_velocity(5)
        .with_motion_magic_acceleration(6)
        .with_motion_magic_jerk(7)
    )

    INTAKE_PIVOT_CURRENT_LIMITS_CONFIGS = (
        configs.CurrentLimitsConfigs()
        .with_supply_current_limit(60)
        .with_supply_current_limit_enable(True)
        .with_stator_current_limit(80)
        .with_stator_current_limit_enable(True)
    )

    INTAKE_PIVOT_ENCODER_CONFIGS = configs.FeedbackConfigs().with_sensor_to_mechanism_ratio(
        INTAKE_PIVOT_GEAR_RATIO
    )

    INTAKE_PIVOT_OUTPUT_CONFIGS = (
        configs.MotorOutputConfigs()
        .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)
        .with_neutral_mode(signals.NeutralModeValue.BRAKE)
    )

    INTAKE_SOFTWARE_CONFIGS = (
        configs.SoftwareLimitSwitchConfigs()
        .with_forward_soft_limit_enable(False)
        .with_forward_soft_limit_threshold(0.222)  # TODO
        .with_reverse_soft_limit_enable(False)
        .with_reverse_soft_limit_threshold(0.0025)  # TODO
    )

    # Intake pivot configs (Kraken X60)
    INTAKE_PIVOT_CONFIGS = (
        configs.TalonFXConfiguration()
        .with_slot0(INTAKE_PIVOT_GAINS)
        .with_current_limits(INTAKE_PIVOT_CURRENT_LIMITS_CONFIGS)
        .with_feedback(INTAKE_PIVOT_ENCODER_CONFIGS)
        .with_motor_output(INTAKE_PIVOT_OUTPUT_CONFIGS)
        .with_motion_magic(INTAKE_PIVOT_MOTION_MAGIC)
        .with_software_limit_switch(INTAKE_SOFTWARE_CONFIGS)
    )

    INTAKE_P = 0.0001  # TODO

    @staticmethod
    def intake_config() -> SparkFlexConfig:
        """Intake roller leader config (NEO Vortex / SPARK Flex)."""
        cfg = SparkFlexConfig()
        cfg.setIdleMode(SparkBaseConfig.IdleMode.kCoast)
        cfg.inverted(False)
        cfg.smartCurrentLimit(30)
        cfg.closedLoop.setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
        cfg.closedLoop.pid(IntakeConstants.INTAKE_P, 0.0, 0.0)
        cfg.closedLoop.feedForward.kV(VORTEX_KV)
        return cfg

    @staticmethod
    def intake_follower_config() -> SparkFlexConfig:
        """Intake roller follower config; mirrors the leader, inverted."""
        cfg = IntakeConstants.intake_config()
        cfg.follow(HardwareConstants.INTAKE_CAN, True)
        return cfg


class ShooterConstants:
    # Shooter gear ratio
    SHOOTER_GEAR_RATIO = 1.0 / 1.0

    LIVEFLOOR_GEAR_RATIO = 25.0 / 1.0  # TODO

    # Shooter flywheel velocity limits (rotations per second)
    SHOOTER_MAX_RPS = 100.0

    SHOOTER_GAINS = (
        configs.Slot0Configs()
        .with_k_p(0.21896)
        .with_k_i(0.0)
        .with_k_d(0.0)
        .with_k_s(0.29926)
        .with_k_v(0.13849)
        .with_k_a(0.068329)
    )  # TODO

    SHOOTER_CURRENT_LIMITS_CONFIGS = (
        configs.CurrentLimitsConfigs()
        .with_supply_current_limit(60)
        .with_supply_current_limit_enable(True)
        .with_supply_current_lower_limit(40)
        .with_supply_current_lower_time(1)  # seconds
        .with_stator_current_limit(80)
        .with_stator_current_limit_enable(True)
    )

    SHOOTER_ENCODER_CONFIGS = configs.FeedbackConfigs().with_sensor_to_mechanism_ratio(
        SHOOTER_GEAR_RATIO
    )

    SHOOTER_OUTPUT_CONFIGS = (
        configs.MotorOutputConfigs()
        .with_inverted(signals.InvertedValue.CLOCKWISE_POSITIVE)  # TODO
        .with_neutral_mode(signals.NeutralModeValue.COAST)
    )

    SHOOTER_CONFIG = (
        configs.TalonFXConfiguration()
        .with_slot0(SHOOTER_GAINS)
        .with_current_limits(SHOOTER_CURRENT_LIMITS_CONFIGS)
        .with_feedback(SHOOTER_ENCODER_CONFIGS)
        .with_motor_output(SHOOTER_OUTPUT_CONFIGS)
    )

    @staticmethod
    def kicker_config() -> SparkFlexConfig:
        """Kicker config (NEO Vortex / SPARK Flex)."""
        cfg = SparkFlexConfig()
        cfg.setIdleMode(SparkBaseConfig.IdleMode.kCoast)
        cfg.inverted(False)
        cfg.smartCurrentLimit(40)
        cfg.closedLoop.setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
        cfg.closedLoop.pid(0.0001, 0.0, 0.0)
        cfg.closedLoop.feedForward.kV(VORTEX_KV)
        return cfg

    @staticmethod
    def livefloor_config() -> SparkFlexConfig:
        """Live floor config (NEO Vortex / SPARK Flex)."""
        cfg = SparkFlexConfig()
        cfg.setIdleMode(SparkBaseConfig.IdleMode.kCoast)
        cfg.inverted(False)
        cfg.smartCurrentLimit(30)
        cfg.closedLoop.setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
        cfg.closedLoop.pid(0.0, 0.0, 0.0)  # TODO
        return cfg


class RobotConstants:
    # Intake robot constants
    INTAKE_PIVOT_STOW = -0.02
    INTAKE_PIVOT_EXTEND = -0.83
    INTAKE_EXTEND_SPEED = -6000  # RPM, negative is inwards
