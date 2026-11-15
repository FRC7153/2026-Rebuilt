"""Swerve drive telemetry for AdvantageScope / Elastic.

Based on the Tuner X generated Telemetry.java, with one important difference:
the Java version ran inside the 250 Hz odometry thread. In Python that would
steal most of the roboRIO's CPU (and delay odometry), so this is called from
the main robot loop instead (50 Hz), with the expensive extras at 10 Hz.

Published topics (all visible in AdvantageScope):
  /DriveState/Pose            Pose2d      - drag onto the 2D/3D field as "Robot"
  /DriveState/Speeds          ChassisSpeeds
  /DriveState/ModuleStates    SwerveModuleState[]  - Swerve tab "States"
  /DriveState/ModuleTargets   SwerveModuleState[]  - Swerve tab "Targets"
  /DriveState/ModulePositions SwerveModulePosition[]
  /PathPlanner/ActivePath     Pose2d[]    - drag onto the field as "Trajectory"
  /PathPlanner/TargetPose     Pose2d      - drag onto the field as "Ghost"
"""

import ntcore
from pathplannerlib.logging import PathPlannerLogging
from phoenix6 import SignalLogger, swerve
from wpilib import Color, Color8Bit, Mechanism2d, SmartDashboard
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition, SwerveModuleState


class Telemetry:
    # The slow extras (SignalLogger copies, Mechanism2d widgets) run every Nth call
    SLOW_DIVIDER = 5

    def __init__(self, max_speed: float) -> None:
        """Construct a telemetry object with the specified max speed of the robot.

        :param max_speed: Maximum speed in meters per second
        """
        self._max_speed = max_speed
        self._counter = 0
        SignalLogger.start()

        # What to publish over networktables for telemetry
        inst = ntcore.NetworkTableInstance.getDefault()

        # Robot swerve drive state
        drive_state_table = inst.getTable("DriveState")
        self._drive_pose = drive_state_table.getStructTopic("Pose", Pose2d).publish()
        self._drive_speeds = drive_state_table.getStructTopic("Speeds", ChassisSpeeds).publish()
        self._drive_module_states = drive_state_table.getStructArrayTopic(
            "ModuleStates", SwerveModuleState
        ).publish()
        self._drive_module_targets = drive_state_table.getStructArrayTopic(
            "ModuleTargets", SwerveModuleState
        ).publish()
        self._drive_module_positions = drive_state_table.getStructArrayTopic(
            "ModulePositions", SwerveModulePosition
        ).publish()
        self._drive_timestamp = drive_state_table.getDoubleTopic("Timestamp").publish()
        self._drive_odometry_frequency = drive_state_table.getDoubleTopic("OdometryFrequency").publish()

        # Robot pose for field positioning (Field2d-style, for dashboards without struct support)
        table = inst.getTable("Pose")
        self._field_pub = table.getDoubleArrayTopic("robotPose").publish()
        self._field_type_pub = table.getStringTopic(".type").publish()
        self._field_type_pub.set("Field2d")

        # PathPlanner: the path being followed and the pose it wants the robot at
        pp_table = inst.getTable("PathPlanner")
        self._pp_active_path = pp_table.getStructArrayTopic("ActivePath", Pose2d).publish()
        self._pp_target_pose = pp_table.getStructTopic("TargetPose", Pose2d).publish()
        self._pp_active_path.set([])
        PathPlannerLogging.setLogActivePathCallback(lambda poses: self._pp_active_path.set(list(poses)))
        PathPlannerLogging.setLogTargetPoseCallback(self._pp_target_pose.set)

        # Mechanisms to represent the swerve module states
        self._module_mechanisms = [Mechanism2d(1, 1) for _ in range(4)]
        # A direction and length changing ligament for speed representation
        self._module_speeds = [
            mechanism.getRoot("RootSpeed", 0.5, 0.5).appendLigament("Speed", 0.5, 0)
            for mechanism in self._module_mechanisms
        ]
        # A direction changing and length constant ligament for module direction
        self._module_directions = [
            mechanism.getRoot("RootDirection", 0.5, 0.5).appendLigament(
                "Direction", 0.1, 0, 0, Color8Bit(Color.kWhite)
            )
            for mechanism in self._module_mechanisms
        ]

        # Set up the module state Mechanism2d telemetry
        for i, mechanism in enumerate(self._module_mechanisms):
            SmartDashboard.putData(f"Module {i}", mechanism)

    def telemeterize(self, state: swerve.SwerveDrivetrain.SwerveDriveState) -> None:
        """Publishes the swerve drive state. Call once per robot loop."""
        # Telemeterize the swerve drive state
        self._drive_pose.set(state.pose)
        self._drive_speeds.set(state.speeds)
        self._drive_module_states.set(state.module_states)
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
        SignalLogger.write_struct("DriveState/Speeds", ChassisSpeeds, state.speeds)
        SignalLogger.write_struct_array("DriveState/ModuleStates", SwerveModuleState, state.module_states)
        SignalLogger.write_struct_array("DriveState/ModuleTargets", SwerveModuleState, state.module_targets)
        SignalLogger.write_double("DriveState/OdometryPeriod", state.odometry_period, "seconds")

        # Telemeterize the pose to a Field2d
        self._field_pub.set([state.pose.x, state.pose.y, state.pose.rotation().degrees()])

        # Telemeterize each module state to a Mechanism2d
        for i, module_state in enumerate(state.module_states[:4]):
            angle = module_state.angle.degrees()
            self._module_speeds[i].setAngle(angle)
            self._module_directions[i].setAngle(angle)
            self._module_speeds[i].setLength(module_state.speed / (2 * self._max_speed))
