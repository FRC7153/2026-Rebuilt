"""Swerve drive telemetry for AdvantageScope / Elastic.

Based on the Tuner X generated Telemetry.java, with one important difference:
the Java version ran inside the 250 Hz odometry thread. In Python that would
steal most of the roboRIO's CPU (and delay odometry), so this is called from
the main robot loop instead (50 Hz), with the expensive extras at 10 Hz.

Published topics (all visible in AdvantageScope):
  /DriveState/Pose            Pose2d      - drag onto the 2D/3D field as "Robot"
  /DriveState/Speeds          ChassisVelocities
  /DriveState/ModuleStates    SwerveModuleVelocity[]  - Swerve tab "States"
  /DriveState/ModuleTargets   SwerveModuleVelocity[]  - Swerve tab "Targets"
  /DriveState/ModulePositions SwerveModulePosition[]
  /PathPlanner/ActivePath     Pose2d[]    - drag onto the field as "Trajectory"
  /PathPlanner/TargetPose     Pose2d      - drag onto the field as "Ghost"
"""

import telemetry
import ntcore
try:
    from pathplannerlib.logging import PathPlannerLogging
except ImportError:  # no 2027 PathPlannerLib yet
    PathPlannerLogging = None
from phoenix6 import SignalLogger, swerve
from wpilib import Mechanism2d, RobotBase
from wpiutil import Color, Color8Bit
from wpimath import Pose2d
from wpimath import ChassisVelocities, SwerveModulePosition, SwerveModuleVelocity


class Telemetry:
    # The slow extras (SignalLogger copies, Mechanism2d widgets) run every Nth call
    SLOW_DIVIDER = 5

    def __init__(self, max_speed: float) -> None:
        """Construct a telemetry object with the specified max speed of the robot.

        :param max_speed: Maximum speed in meters per second
        """
        self._max_speed = max_speed
        self._counter = 0
        # Only on the real robot: in sim the CTRE .hoot log records every simulated
        # motor signal and grows by ~250 MB per hour (it hit 4 GB overnight).
        if RobotBase.is_real():
            SignalLogger.start()

        # What to publish over networktables for telemetry
        inst = ntcore.NetworkTableInstance.get_default()

        # Robot swerve drive state
        drive_state_table = inst.get_table("DriveState")
        self._drive_pose = drive_state_table.get_struct_topic("Pose", Pose2d).publish()
        self._drive_speeds = drive_state_table.get_struct_topic("Speeds", ChassisVelocities).publish()
        self._drive_module_states = drive_state_table.get_struct_array_topic(
            "ModuleStates", SwerveModuleVelocity
        ).publish()
        self._drive_module_targets = drive_state_table.get_struct_array_topic(
            "ModuleTargets", SwerveModuleVelocity
        ).publish()
        self._drive_module_positions = drive_state_table.get_struct_array_topic(
            "ModulePositions", SwerveModulePosition
        ).publish()
        self._drive_timestamp = drive_state_table.get_double_topic("Timestamp").publish()
        self._drive_odometry_frequency = drive_state_table.get_double_topic("OdometryFrequency").publish()

        # Robot pose for field positioning (Field2d-style, for dashboards without struct support)
        table = inst.get_table("Pose")
        self._field_pub = table.get_double_array_topic("robotPose").publish()
        self._field_type_pub = table.get_string_topic(".type").publish()
        self._field_type_pub.set("Field2d")

        # PathPlanner: the path being followed and the pose it wants the robot at
        pp_table = inst.get_table("PathPlanner")
        self._pp_active_path = pp_table.get_struct_array_topic("ActivePath", Pose2d).publish()
        self._pp_target_pose = pp_table.get_struct_topic("TargetPose", Pose2d).publish()
        self._pp_active_path.set([])
        if PathPlannerLogging is not None:
            PathPlannerLogging.setLogActivePathCallback(lambda poses: self._pp_active_path.set(list(poses)))
            PathPlannerLogging.setLogTargetPoseCallback(self._pp_target_pose.set)

        # Mechanisms to represent the swerve module states
        self._module_mechanisms = [Mechanism2d(1, 1) for _ in range(4)]
        # A direction and length changing ligament for speed representation
        self._module_speeds = [
            mechanism.get_root("RootSpeed", 0.5, 0.5).append_ligament("Speed", 0.5, 0)
            for mechanism in self._module_mechanisms
        ]
        # A direction changing and length constant ligament for module direction
        self._module_directions = [
            mechanism.get_root("RootDirection", 0.5, 0.5).append_ligament(
                "Direction", 0.1, 0, 0, Color8Bit(Color.WHITE)
            )
            for mechanism in self._module_mechanisms
        ]

        # Set up the module state Mechanism2d telemetry
        for i, mechanism in enumerate(self._module_mechanisms):
            telemetry.log(f"Module {i}", mechanism)

    def telemeterize(self, state: swerve.SwerveDrivetrain.SwerveDriveState) -> None:
        """Publishes the swerve drive state. Call once per robot loop."""
        # Telemeterize the swerve drive state
        self._drive_pose.set(state.pose)
        self._drive_speeds.set(state.velocity)
        self._drive_module_states.set(state.module_velocities)
        self._drive_module_targets.set(state.module_targets)

        self._counter += 1
        if self._counter % self.SLOW_DIVIDER != 0:
            return

        self._drive_module_positions.set(state.module_positions)
        self._drive_timestamp.set(state.timestamp)
        if state.odometry_period > 0.0:
            self._drive_odometry_frequency.set(1.0 / state.odometry_period)

        # Also write to the CTRE log file
        SignalLogger.write_struct("DriveState/Pose", Pose2d, state.pose)
        SignalLogger.write_struct("DriveState/Speeds", ChassisVelocities, state.velocity)
        SignalLogger.write_struct_array("DriveState/ModuleStates", SwerveModuleVelocity, state.module_velocities)
        SignalLogger.write_struct_array("DriveState/ModuleTargets", SwerveModuleVelocity, state.module_targets)
        SignalLogger.write_double("DriveState/OdometryPeriod", state.odometry_period, "seconds")

        # Telemeterize the pose to a Field2d
        self._field_pub.set([state.pose.x, state.pose.y, state.pose.rotation().degrees()])

        # Telemeterize each module state to a Mechanism2d
        for i, module_state in enumerate(state.module_velocities[:4]):
            angle = module_state.angle.degrees()
            self._module_speeds[i].set_angle(angle)
            self._module_directions[i].set_angle(angle)
            self._module_speeds[i].set_length(module_state.velocity / (2 * self._max_speed))
