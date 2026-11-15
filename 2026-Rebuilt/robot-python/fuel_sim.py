"""Simulated FUEL (game pieces) for the simulator. Used only by physics.py.

What it does:
  * puts a grid of fuel in the neutral zone (reset whenever autonomous starts,
    or when /Sim/ResetFuel is set to true)
  * the robot picks fuel up when the intake is extended and the rollers spin inwards
  * the robot launches fuel when the shooter is spinning and the kicker feeds forward
  * launched fuel flies in a ballistic arc; fuel that drops into a hub is scored,
    everything else lands on the floor and can be picked up again

Everything here is an APPROXIMATION meant to make the simulator useful, not a
precise model. The numbers you most likely want to tune are marked TUNE.

AdvantageScope (3D Field tab), drag these on and pick "Game Piece" -> Fuel:
  /Sim/FieldFuel     Translation3d[]  fuel lying on the floor
  /Sim/InFlightFuel  Translation3d[]  fuel in the air
Numbers: /Sim/HeldFuel, /Sim/ShotsFired, /Sim/Score/Blue, /Sim/Score/Red
"""

import math
import random

import ntcore
from wpimath.geometry import Pose2d, Translation3d
from wpimath.kinematics import ChassisSpeeds

FIELD_LENGTH = 16.54
FIELD_WIDTH = 8.07
FUEL_RADIUS = 0.075  # 5.9 in diameter
GRAVITY = 9.81

# Hubs (centers from the AprilTag map) and their top opening
HUBS = {"Blue": (4.625, 4.035), "Red": (11.915, 4.035)}
HUB_TOP_HEIGHT = 1.83  # m (72 in)
HUB_OPENING_RADIUS = 0.53  # m

# Robot geometry (robot frame: +X = front, where the shooter / front Limelight point)
ROBOT_HALF_LENGTH = 0.45  # with bumpers
INTAKE_ON_FRONT = False  # TUNE: set True if the intake is on the same side as the shooter
INTAKE_DEPTH = 0.30  # how far past the bumper the extended intake reaches
INTAKE_HALF_WIDTH = 0.35
HOPPER_CAPACITY = 40  # TUNE
PRELOAD = 8

# Shooter model
LAUNCH_HEIGHT = 0.55  # m
# Chosen so the robot's standard shot (24.25 rps) scores from ~1.5 m, which is
# where ShootAuto shoots from. TUNE both to match the real robot.
LAUNCH_ANGLE_DEG = 70.0  # hood angle above horizontal
EXIT_SPEED_PER_RPS = 0.235  # m/s of ball speed per flywheel rotation per second
FEED_RATE = 8.0  # balls per second while the kicker feeds
MIN_SHOOTER_RPS = 5.0

# Thresholds for "intake is running" / "kicker is feeding"
PIVOT_EXTENDED_BELOW = -0.5  # rotations (stow -0.02, extend -0.83)
ROLLER_INTAKE_RPM = -500.0  # negative is inwards
KICKER_FEED_OUTPUT = 0.2

FIELD_PUBLISH_PERIOD = 0.1  # s


def _neutral_zone_fuel() -> list[list[float]]:
    """Approximate neutral-zone staging: a tight grid at the center of the field."""
    spacing = 2 * FUEL_RADIUS
    fuel = []
    for col in range(12):
        for row in range(30):
            x = FIELD_LENGTH / 2 + (col - 5.5) * spacing
            y = FIELD_WIDTH / 2 + (row - 14.5) * spacing
            fuel.append([x, y])
    return fuel


class FuelSim:
    def __init__(self) -> None:
        self._floor: list[list[float]] = []
        self._in_flight: list[list[float]] = []  # [x, y, z, vx, vy, vz]
        self._held = 0
        self._shots = 0
        self._score = {"Blue": 0, "Red": 0}
        self._feed_timer = 0.0
        self._field_dirty = True
        self._since_field_publish = 0.0

        table = ntcore.NetworkTableInstance.getDefault().getTable("Sim")
        self._field_pub = table.getStructArrayTopic("FieldFuel", Translation3d).publish()
        self._flight_pub = table.getStructArrayTopic("InFlightFuel", Translation3d).publish()
        self._held_pub = table.getIntegerTopic("HeldFuel").publish()
        self._shots_pub = table.getIntegerTopic("ShotsFired").publish()
        self._score_pubs = {
            name: table.getSubTable("Score").getIntegerTopic(name).publish() for name in self._score
        }
        reset_topic = table.getBooleanTopic("ResetFuel")
        self._reset_pub = reset_topic.publish()
        self._reset_pub.set(False)
        self._reset_sub = reset_topic.subscribe(False)

        self.reset()

    # ---------------------------------------------------------------- control

    def reset(self) -> None:
        """Restages the field and preloads the robot."""
        self._floor = _neutral_zone_fuel()
        self._in_flight = []
        self._held = PRELOAD
        self._shots = 0
        self._score = {"Blue": 0, "Red": 0}
        self._feed_timer = 0.0
        self._field_dirty = True

    @property
    def held(self) -> int:
        return self._held

    @property
    def shots(self) -> int:
        return self._shots

    @property
    def score(self) -> dict[str, int]:
        return dict(self._score)

    def place_robot_fuel(self, count: int) -> None:
        """Sets how much fuel the robot is holding (for testing)."""
        self._held = max(0, min(HOPPER_CAPACITY, count))

    # ----------------------------------------------------------------- update

    def update(
        self,
        dt: float,
        pose: Pose2d,
        robot_speeds: ChassisSpeeds,
        intake_running: bool,
        shooter_rps: float,
        kicker_feeding: bool,
    ) -> None:
        if self._reset_sub.get():
            self._reset_pub.set(False)
            self.reset()

        cos_h = math.cos(pose.rotation().radians())
        sin_h = math.sin(pose.rotation().radians())

        if intake_running and self._held < HOPPER_CAPACITY:
            self._run_intake(pose.X(), pose.Y(), cos_h, sin_h)

        if kicker_feeding and shooter_rps > MIN_SHOOTER_RPS and self._held > 0:
            self._feed_timer += dt
            while self._feed_timer >= 1.0 / FEED_RATE and self._held > 0:
                self._feed_timer -= 1.0 / FEED_RATE
                self._launch(pose.X(), pose.Y(), cos_h, sin_h, robot_speeds, shooter_rps)
        else:
            # The first ball comes out as soon as feeding starts
            self._feed_timer = 1.0 / FEED_RATE

        self._step_flight(dt)
        self._publish(dt)

    def _run_intake(self, rx: float, ry: float, cos_h: float, sin_h: float) -> None:
        if INTAKE_ON_FRONT:
            near, far = ROBOT_HALF_LENGTH - 0.05, ROBOT_HALF_LENGTH + INTAKE_DEPTH
        else:
            near, far = -ROBOT_HALF_LENGTH - INTAKE_DEPTH, -ROBOT_HALF_LENGTH + 0.05

        kept = []
        for ball in self._floor:
            dx = ball[0] - rx
            dy = ball[1] - ry
            # Field -> robot frame
            local_x = dx * cos_h + dy * sin_h
            local_y = -dx * sin_h + dy * cos_h
            if (
                self._held < HOPPER_CAPACITY
                and near <= local_x <= far
                and abs(local_y) <= INTAKE_HALF_WIDTH
            ):
                self._held += 1
                self._field_dirty = True
            else:
                kept.append(ball)
        self._floor = kept

    def _launch(
        self, rx: float, ry: float, cos_h: float, sin_h: float, speeds: ChassisSpeeds, shooter_rps: float
    ) -> None:
        self._held -= 1
        self._shots += 1

        exit_speed = shooter_rps * EXIT_SPEED_PER_RPS
        angle = math.radians(LAUNCH_ANGLE_DEG)
        horizontal = exit_speed * math.cos(angle)

        # Robot motion carries over into the shot (robot-relative speeds -> field)
        robot_vx = speeds.vx * cos_h - speeds.vy * sin_h
        robot_vy = speeds.vx * sin_h + speeds.vy * cos_h

        start = 0.1  # launch point slightly ahead of robot center
        self._in_flight.append(
            [
                rx + start * cos_h,
                ry + start * sin_h,
                LAUNCH_HEIGHT,
                horizontal * cos_h + robot_vx,
                horizontal * sin_h + robot_vy,
                exit_speed * math.sin(angle),
            ]
        )

    def _step_flight(self, dt: float) -> None:
        still_flying = []
        for ball in self._in_flight:
            x, y, z, vx, vy, vz = ball
            nx, ny = x + vx * dt, y + vy * dt
            nvz = vz - GRAVITY * dt
            nz = z + (vz + nvz) * 0.5 * dt

            # Falling through a hub's top opening?
            if vz < 0.0 or nvz < 0.0:
                if z >= HUB_TOP_HEIGHT > nz:
                    t = (z - HUB_TOP_HEIGHT) / (z - nz)
                    cx, cy = x + (nx - x) * t, y + (ny - y) * t
                    hub = self._hub_at(cx, cy)
                    if hub is not None:
                        self._score[hub] += 1
                        self._exit_hub(hub)
                        continue

            # Bounce off the field walls
            if not FUEL_RADIUS <= nx <= FIELD_LENGTH - FUEL_RADIUS:
                vx = -vx * 0.4
                nx = min(max(nx, FUEL_RADIUS), FIELD_LENGTH - FUEL_RADIUS)
            if not FUEL_RADIUS <= ny <= FIELD_WIDTH - FUEL_RADIUS:
                vy = -vy * 0.4
                ny = min(max(ny, FUEL_RADIUS), FIELD_WIDTH - FUEL_RADIUS)

            if nz <= FUEL_RADIUS:
                self._floor.append([nx, ny])
                self._field_dirty = True
                continue

            still_flying.append([nx, ny, nz, vx, vy, nvz])
        self._in_flight = still_flying

    @staticmethod
    def _hub_at(x: float, y: float) -> str | None:
        for name, (hx, hy) in HUBS.items():
            if math.hypot(x - hx, y - hy) <= HUB_OPENING_RADIUS:
                return name
        return None

    def _exit_hub(self, hub: str) -> None:
        """Scored fuel comes back out of the hub on its neutral-zone side."""
        hx, hy = HUBS[hub]
        toward_center = 1.0 if hx < FIELD_LENGTH / 2 else -1.0
        self._floor.append([hx + toward_center * random.uniform(0.9, 1.6), hy + random.uniform(-1.0, 1.0)])
        self._field_dirty = True

    # ---------------------------------------------------------------- publish

    def _publish(self, dt: float) -> None:
        self._since_field_publish += dt
        if self._field_dirty and self._since_field_publish >= FIELD_PUBLISH_PERIOD:
            self._field_pub.set([Translation3d(b[0], b[1], FUEL_RADIUS) for b in self._floor])
            self._field_dirty = False
            self._since_field_publish = 0.0

        self._flight_pub.set([Translation3d(b[0], b[1], b[2]) for b in self._in_flight])
        self._held_pub.set(self._held)
        self._shots_pub.set(self._shots)
        for name, pub in self._score_pubs.items():
            pub.set(self._score[name])
