"""Simulation physics. Only loaded by `robotpy sim` / `robotpy test`, never on the robot.

The swerve drive simulates itself (Phoenix 6, see CommandSwerveDrivetrain). This
file fills in the rest so the whole robot behaves believably in simulation:

  * intake pivot and shooter flywheel (TalonFX) get simple motor models
  * intake rollers, kicker and live floor (SPARK Flex) get simple motor models
  * both Limelights look "alive", and the front one reports the alliance hub's
    AprilTags when the simulated robot is actually facing them, so point-to-hub
    and distance-shoot work in simulation too
"""

import math

import ntcore
import rev
from phoenix6 import sim as ctre_sim
try:
    from pyfrc.physics.core import PhysicsInterface
except ImportError:  # pyfrc has no working 2027 release yet; this file is unused until it does
    PhysicsInterface = object
from wpilib import Alliance, MatchState, RobotController, RobotState
from wpilib.simulation import DCMotorSim
from wpimath import DCMotor, LinearSystemId

from constants import AprilTagConstants, IntakeConstants, ShooterConstants
from fuel_sim import KICKER_FEED_OUTPUT, PIVOT_EXTENDED_BELOW, ROLLER_INTAKE_RPM, FuelSim

# Rough moments of inertia (kg m^2) - only used to make simulated motion look plausible
PIVOT_MOI = 0.15
FLYWHEEL_MOI = 0.004
SPARK_SPINUP_TIME = 0.12  # seconds for rollers/kicker/live floor to reach speed

# Front Limelight model
LIMELIGHT_HALF_FOV_DEG = 31.0
LIMELIGHT_MAX_RANGE = 6.0  # meters

BLUE_HUB_TAGS = (18, 19, 20, 21, 24, 25, 26, 27)
RED_HUB_TAGS = (2, 3, 4, 5, 8, 9, 10, 11)


class _TalonSim:
    """A TalonFX driving a simple inertia through a gearbox."""

    def __init__(self, talon, gearing: float, moi: float, inverted: bool) -> None:
        self._state = talon.sim_state
        self._gearing = gearing
        gearbox = DCMotor.kraken_x60(1)
        self._model = DCMotorSim(LinearSystemId.DCMotorSystem(gearbox, moi, gearing), gearbox)
        if inverted:
            self._state.orientation = ctre_sim.ChassisReference.CLOCKWISE_POSITIVE

    def update(self, dt: float, battery_voltage: float) -> None:
        self._state.set_supply_voltage(battery_voltage)
        self._model.setInputVoltage(self._state.motor_voltage)
        self._model.update(dt)
        self._state.set_raw_rotor_position(self._model.getAngularPositionRotations() * self._gearing)
        self._state.set_rotor_velocity(self._model.getAngularVelocityRPM() / 60.0 * self._gearing)


class _SparkSim:
    """A SPARK Flex spinning a light roller: speed simply follows the applied output."""

    def __init__(self, spark: rev.SparkFlex) -> None:
        self._motor = DCMotor.neo_vortex(1)
        self._sim = rev.SparkFlexSim(spark, self._motor)
        self._free_speed_rpm = self._motor.free_speed * 60.0 / (2.0 * math.pi)
        self._velocity = 0.0

    @property
    def applied_output(self) -> float:
        return self._sim.get_applied_output()

    def update(self, dt: float, battery_voltage: float) -> None:
        target = self._sim.get_applied_output() * self._free_speed_rpm
        alpha = min(1.0, dt / SPARK_SPINUP_TIME)
        self._velocity += (target - self._velocity) * alpha
        self._sim.iterate(self._velocity, battery_voltage, dt)


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot) -> None:
        self._robot = robot
        self._talons: list[_TalonSim] = []
        self._sparks: list[_SparkSim] = []
        self._ready = False

        self.fuel = FuelSim()
        self._was_auto = False

        nt = ntcore.NetworkTableInstance.get_default()
        self._heartbeat = 0.0
        self._heartbeat_pubs = [
            nt.get_table(name).get_double_topic("hb").publish()
            for name in (AprilTagConstants.LL_4_FRONT, AprilTagConstants.LL_3_BACK)
        ]
        front = nt.get_table(AprilTagConstants.LL_4_FRONT)
        self._t2d_pub = front.get_double_array_topic("t2d").publish()
        self._ty_pub = front.get_double_topic("ty").publish()
        self._tx_pub = front.get_double_topic("tx").publish()
        self._fiducials_pub = front.get_double_array_topic("rawfiducials").publish()

    def _setup(self) -> bool:
        """The robot's hardware only exists once robotInit has run, so set up lazily."""
        container = getattr(self._robot, "_robot_container", None)
        if container is None:
            return False

        intake = container._intake
        shooter = container._shooter
        self._intake = intake
        self._shooter = shooter
        self._drivetrain = container.drivetrain

        self._talons = [
            _TalonSim(intake._pivot, IntakeConstants.INTAKE_PIVOT_GEAR_RATIO, PIVOT_MOI, inverted=True),
            _TalonSim(shooter._shooter, ShooterConstants.SHOOTER_GEAR_RATIO, FLYWHEEL_MOI, inverted=True),
        ]
        self._kicker_sim = _SparkSim(shooter._kicker)
        self._sparks = [
            _SparkSim(intake._intake),
            self._kicker_sim,
            _SparkSim(shooter._live_floor),
        ]
        return True

    def update_sim(self, now: float, tm_diff: float) -> None:
        if not self._ready:
            self._ready = self._setup()
            if not self._ready:
                return

        battery_voltage = RobotController.get_battery_voltage()
        for talon in self._talons:
            talon.update(tm_diff, battery_voltage)
        for spark in self._sparks:
            spark.update(tm_diff, battery_voltage)

        self._update_limelights()
        self._update_fuel(tm_diff)

    # -------------------------------------------------------------------- fuel

    def _update_fuel(self, dt: float) -> None:
        # Fresh field and a preloaded robot every time autonomous starts
        is_auto = RobotState.is_autonomous_enabled()
        if is_auto and not self._was_auto:
            self.fuel.reset()
        self._was_auto = is_auto

        state = self._drivetrain.get_state()
        enabled = RobotState.is_enabled()
        intake_running = (
            enabled
            and self._intake.get_intake_pivot_position() < PIVOT_EXTENDED_BELOW
            and self._intake.get_intake_velocity() < ROLLER_INTAKE_RPM
        )
        kicker_feeding = enabled and self._kicker_sim.applied_output > KICKER_FEED_OUTPUT

        self.fuel.update(
            dt,
            state.pose,
            state.velocity,
            intake_running,
            self._shooter.get_shooter_velocity(),
            kicker_feeding,
        )

    # ------------------------------------------------------------------ vision

    def _update_limelights(self) -> None:
        self._heartbeat += 1.0
        for pub in self._heartbeat_pubs:
            pub.set(self._heartbeat)

        pose = self._drivetrain.get_state().pose
        is_red = MatchState.get_alliance() == Alliance.RED
        heading = pose.rotation().radians()

        fiducials: list[float] = []
        best: tuple[float, float] | None = None  # (abs tx, distance)
        for tag_id in RED_HUB_TAGS if is_red else BLUE_HUB_TAGS:
            tag = AprilTagConstants.FIELD.get_tag_pose(tag_id)
            if tag is None:
                continue

            dx = tag.x - pose.x
            dy = tag.y - pose.y
            distance = math.hypot(dx, dy)
            if distance < 0.3 or distance > LIMELIGHT_MAX_RANGE:
                continue

            # Bearing to the tag relative to where the camera points (CCW positive)
            bearing = math.atan2(dy, dx) - heading
            bearing = math.degrees(math.atan2(math.sin(bearing), math.cos(bearing)))
            if abs(bearing) > LIMELIGHT_HALF_FOV_DEG:
                continue

            # The camera only sees tags that face it
            facing = tag.rotation().to_rotation2d().radians()
            if math.cos(facing) * -dx + math.sin(facing) * -dy <= 0.2 * distance:
                continue

            tx = -bearing  # Limelight: target to the right is positive
            elevation = math.degrees(
                math.atan2(AprilTagConstants.TARGET_HEIGHT - AprilTagConstants.FRONT_CAMERA_HEIGHT, distance)
            )
            ty = elevation - AprilTagConstants.CAMERA_ANGLE_DEG

            # id, txnc, tync, ta, distToCamera, distToRobot, ambiguity
            fiducials += [float(tag_id), tx, ty, 1.0 / (distance * distance), distance, distance, 0.1]
            if best is None or abs(tx) < best[0]:
                best = (abs(tx), distance)
                self._tx_pub.set(tx)
                self._ty_pub.set(ty)

        t2d = [0.0] * 17
        t2d[1] = len(fiducials) // 7
        self._t2d_pub.set(t2d)
        self._fiducials_pub.set(fiducials)
