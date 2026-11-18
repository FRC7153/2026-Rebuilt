"""Swerve constants (port of the Tuner X generated TunerConstants.java).

https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
"""

from phoenix6 import CANBus, configs, hardware, signals, swerve
from wpimath.units import inches_to_meters

# Both sets of gains need to be tuned to your individual robot.

# The steer motor uses any SwerveModule.SteerRequestType control request with the
# output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
_steer_gains = (
    configs.Slot0Configs()
    .with_static_feedforward_sign(signals.StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN)
    .with_k_p(100)
    .with_k_i(0)
    .with_k_d(0.5)
)

# When using closed-loop control, the drive motor uses the control
# output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
_drive_gains = (
    configs.Slot0Configs()
    .with_k_p(2.2)
    .with_k_i(0)
    .with_k_d(0.04)
    .with_k_s(0.154027)
    .with_k_v(2.5082)
    .with_k_a(0.024792)
)

# The closed-loop output type to use for the steer motors;
# This affects the PID/FF gains for the steer motors
_steer_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE
# The closed-loop output type to use for the drive motors;
# This affects the PID/FF gains for the drive motors
_drive_closed_loop_output = swerve.ClosedLoopOutputType.VOLTAGE

# The type of motor used for the drive motor
_drive_motor_type = swerve.DriveMotorArrangement.TALON_FX_INTEGRATED
# The type of motor used for the steer motor
_steer_motor_type = swerve.SteerMotorArrangement.TALON_FX_INTEGRATED

# The remote sensor feedback type to use for the steer motors;
# When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
_steer_feedback_type = swerve.SteerFeedbackType.FUSED_CANCODER

# The stator current at which the wheels start to slip;
# This needs to be tuned to your individual robot
_slip_current = 120.0  # amps

# Initial configs for the drive and steer motors and the azimuth encoder; these cannot be None.
# Some configs will be overwritten; check the `with_*_initial_configs()` API documentation.
_drive_initial_configs = configs.TalonFXConfiguration().with_current_limits(
    configs.CurrentLimitsConfigs()
    .with_stator_current_limit(80)
    .with_stator_current_limit_enable(True)
    .with_supply_current_limit(40)
    .with_supply_current_lower_limit(30)
    .with_supply_current_lower_time(0.5)
    .with_supply_current_limit_enable(True)
)
_steer_initial_configs = configs.TalonFXConfiguration().with_current_limits(
    configs.CurrentLimitsConfigs()
    # Swerve azimuth does not require much torque output, so we can set a relatively low
    # stator current limit to help avoid brownouts without impacting performance.
    .with_stator_current_limit(40)
    .with_stator_current_limit_enable(True)
    .with_supply_current_limit(30)
    .with_supply_current_limit_enable(True)
)
_encoder_initial_configs = configs.CANcoderConfiguration()
# Configs for the Pigeon 2; leave this None to skip applying Pigeon 2 configs
_pigeon_configs: configs.Pigeon2Configuration | None = None

# CAN bus that the devices are located on;
# All swerve devices must share the same CAN bus
canbus = CANBus("CANivore")

# Theoretical free speed (m/s) at 12 V applied output:
#   Kraken X60 6000 RPM / 8.14 gearing * (2 * pi * 2 in wheel) = 3.92 m/s
# This was 2.5, which made every open-loop speed request (PathPlanner, point-to-hub)
# drive ~50% faster than asked. Teleop is unaffected: full stick is 12 V either way.
# Lower this a little if the real robot can't quite reach it (tune with SysId).
speed_at_12_volts = 3.92  # meters per second

# Every 1 rotation of the azimuth results in _couple_ratio drive motor turns;
# This may need to be tuned to your individual robot
_couple_ratio = 3.5714285714285716

_drive_gear_ratio = 8.142857142857142
_steer_gear_ratio = 21.428571428571427
_wheel_radius = inches_to_meters(2)

_invert_left_side = False
_invert_right_side = False

_pigeon_id = 20

# These are only used for simulation
_steer_inertia = 0.01  # kg m^2
_drive_inertia = 0.01  # kg m^2
# Simulated voltage necessary to overcome friction
_steer_friction_voltage = 0.2
_drive_friction_voltage = 0.2

drivetrain_constants = (
    swerve.SwerveDrivetrainConstants()
    .with_network(canbus)
    .with_pigeon2_id(_pigeon_id)
    .with_pigeon2_configs(_pigeon_configs)
)

_constants_creator: swerve.SwerveModuleConstantsFactory[
    configs.TalonFXConfiguration, configs.TalonFXConfiguration, configs.CANcoderConfiguration
] = (
    swerve.SwerveModuleConstantsFactory()
    .with_drive_motor_gear_ratio(_drive_gear_ratio)
    .with_steer_motor_gear_ratio(_steer_gear_ratio)
    .with_coupling_gear_ratio(_couple_ratio)
    .with_wheel_radius(_wheel_radius)
    .with_steer_motor_gains(_steer_gains)
    .with_drive_motor_gains(_drive_gains)
    .with_steer_motor_closed_loop_output(_steer_closed_loop_output)
    .with_drive_motor_closed_loop_output(_drive_closed_loop_output)
    .with_slip_current(_slip_current)
    .with_speed_at12_volts(speed_at_12_volts)
    .with_drive_motor_type(_drive_motor_type)
    .with_steer_motor_type(_steer_motor_type)
    .with_feedback_source(_steer_feedback_type)
    .with_drive_motor_initial_configs(_drive_initial_configs)
    .with_steer_motor_initial_configs(_steer_initial_configs)
    .with_encoder_initial_configs(_encoder_initial_configs)
    .with_steer_inertia(_steer_inertia)
    .with_drive_inertia(_drive_inertia)
    .with_steer_friction_voltage(_steer_friction_voltage)
    .with_drive_friction_voltage(_drive_friction_voltage)
)

# NOTE: IDs, offsets, positions and inversions below are copied 1:1 from the Java
# TunerConstants that the robot was driving with. The "Back Left" module sits at
# -Y (robot right) and "Back Right" at +Y (robot left) - that is how the Java code
# had it, so it is preserved; only the names are swapped, the geometry is what matters.

# Front Left
_front_left_drive_motor_id = 1
_front_left_steer_motor_id = 2
_front_left_encoder_id = 9
_front_left_encoder_offset = -0.47973628125 + 0.25  # rotations
_front_left_steer_motor_inverted = True
_front_left_encoder_inverted = False

_front_left_x_pos = inches_to_meters(11)
_front_left_y_pos = inches_to_meters(11)

# Front Right
_front_right_drive_motor_id = 3
_front_right_steer_motor_id = 4
_front_right_encoder_id = 10
_front_right_encoder_offset = -0.374755859375  # rotations
_front_right_steer_motor_inverted = True
_front_right_encoder_inverted = False

_front_right_x_pos = inches_to_meters(11)
_front_right_y_pos = inches_to_meters(-11)

# Back Left
_back_left_drive_motor_id = 5
_back_left_steer_motor_id = 6
_back_left_encoder_id = 11
_back_left_encoder_offset = -0.057617 + 0.25  # rotations
_back_left_steer_motor_inverted = True
_back_left_encoder_inverted = False

_back_left_x_pos = inches_to_meters(-11)
_back_left_y_pos = inches_to_meters(-11)

# Back Right
_back_right_drive_motor_id = 7
_back_right_steer_motor_id = 8
_back_right_encoder_id = 12
_back_right_encoder_offset = -0.109130859375  # rotations
_back_right_steer_motor_inverted = True
_back_right_encoder_inverted = False

_back_right_x_pos = inches_to_meters(-11)
_back_right_y_pos = inches_to_meters(11)

front_left = _constants_creator.create_module_constants(
    _front_left_steer_motor_id,
    _front_left_drive_motor_id,
    _front_left_encoder_id,
    _front_left_encoder_offset,
    _front_left_x_pos,
    _front_left_y_pos,
    _invert_left_side,
    _front_left_steer_motor_inverted,
    _front_left_encoder_inverted,
)
front_right = _constants_creator.create_module_constants(
    _front_right_steer_motor_id,
    _front_right_drive_motor_id,
    _front_right_encoder_id,
    _front_right_encoder_offset,
    _front_right_x_pos,
    _front_right_y_pos,
    _invert_right_side,
    _front_right_steer_motor_inverted,
    _front_right_encoder_inverted,
)
back_left = _constants_creator.create_module_constants(
    _back_left_steer_motor_id,
    _back_left_drive_motor_id,
    _back_left_encoder_id,
    _back_left_encoder_offset,
    _back_left_x_pos,
    _back_left_y_pos,
    True,
    _back_left_steer_motor_inverted,
    _back_left_encoder_inverted,
)
back_right = _constants_creator.create_module_constants(
    _back_right_steer_motor_id,
    _back_right_drive_motor_id,
    _back_right_encoder_id,
    _back_right_encoder_offset,
    _back_right_x_pos,
    _back_right_y_pos,
    True,
    _back_right_steer_motor_inverted,
    _back_right_encoder_inverted,
)


def create_drivetrain():
    """Creates a CommandSwerveDrivetrain instance.

    This should only be called once in your robot program.
    """
    from subsystems.swerve.command_swerve_drivetrain import CommandSwerveDrivetrain

    return CommandSwerveDrivetrain(
        hardware.TalonFX,
        hardware.TalonFX,
        hardware.CANcoder,
        drivetrain_constants,
        [front_left, front_right, back_left, back_right],
    )
