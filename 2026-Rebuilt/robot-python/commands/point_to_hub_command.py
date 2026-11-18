"""Drives field-centric while automatically rotating to face the alliance hub."""

import math
from typing import Callable

from commands2 import Command
from phoenix6 import swerve
from wpilib import Alliance, MatchState
from wpimath import ProfiledPIDControllerRadians
from wpimath import Pose2d, Translation2d
from wpimath import TrapezoidProfileRadians

from libs import limelight_helpers
from subsystems.swerve.command_swerve_drivetrain import CommandSwerveDrivetrain


class PointToHubCommand(Command):
    # Hub centers in the WPILib blue-origin field frame (meters), taken as the
    # centroid of each hub's AprilTags in the 2026 field map.
    BLUE_HUB = Translation2d(4.625, 4.035)
    RED_HUB = Translation2d(11.915, 4.035)

    # Hub AprilTag IDs per the 2026 field layout
    BLUE_HUB_TAGS = frozenset({18, 19, 20, 21, 24, 25, 26, 27})
    RED_HUB_TAGS = frozenset({2, 3, 4, 5, 8, 9, 10, 11})

    def __init__(
        self,
        drivetrain: CommandSwerveDrivetrain,
        x_supplier: Callable[[], float],
        y_supplier: Callable[[], float],
        limelight_name: str,
    ) -> None:
        super().__init__()
        self._drivetrain = drivetrain
        self._x_supplier = x_supplier
        self._y_supplier = y_supplier
        self._limelight_name = limelight_name

        self._rotation_controller = ProfiledPIDControllerRadians(
            5.0,
            0.0,
            1.5,
            TrapezoidProfileRadians.Constraints(math.radians(540), math.radians(720)),
        )
        self._rotation_controller.enable_continuous_input(-math.pi, math.pi)
        self._rotation_controller.set_tolerance(math.radians(1.5))

        self._drive_request = swerve.requests.FieldCentric().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )
        self._idle_request = swerve.requests.Idle()

        self.add_requirements(drivetrain)

    def initialize(self) -> None:
        state = self._drivetrain.get_state()
        self._rotation_controller.reset(state.pose.rotation().radians(), state.velocity.omega)

    def execute(self) -> None:
        robot_pose = self._drivetrain.get_state().pose

        target_angle = self._get_target_angle(robot_pose)

        rotation_output = self._rotation_controller.calculate(robot_pose.rotation().radians(), target_angle)

        self._drivetrain.set_control(
            self._drive_request.with_velocity_x(self._x_supplier())
            .with_velocity_y(self._y_supplier())
            .with_rotational_rate(rotation_output)
        )

    def _get_target_angle(self, robot_pose: Pose2d) -> float:
        """Returns the field heading (radians) to face.

        Priority: Limelight tx (direct tag offset) -> field pose from MT2 -> odometry fallback
        """
        # --- 1. Try direct tx from Limelight (fastest, lowest latency) ---
        alliance_hub_tags = self._get_alliance_hub_tags()
        best_tx: float | None = None
        for fiducial in limelight_helpers.get_raw_fiducials(self._limelight_name):
            # Find the best hub tag (closest to center of frame)
            if fiducial.id in alliance_hub_tags and (best_tx is None or abs(fiducial.txnc) < abs(best_tx)):
                best_tx = fiducial.txnc  # txnc = raw horizontal offset, degrees

        if best_tx is not None:
            # Rotate by tx offset - negative because tx right = need to turn right (negative CCW)
            return robot_pose.rotation().radians() + math.radians(-best_tx)

        # --- 2. Try MegaTag2 field pose from Limelight ---
        mt2 = limelight_helpers.get_bot_pose_estimate_wpi_blue_mega_tag2(self._limelight_name)
        if mt2 is not None and mt2.tag_count > 0:
            to_hub = self._get_hub_translation() - mt2.pose.translation()
            return math.atan2(to_hub.y, to_hub.x)

        # --- 3. Odometry fallback ---
        to_hub = self._get_hub_translation() - robot_pose.translation()
        return math.atan2(to_hub.y, to_hub.x)

    @staticmethod
    def _is_red() -> bool:
        return MatchState.get_alliance() == Alliance.RED

    def _get_hub_translation(self) -> Translation2d:
        return self.RED_HUB if self._is_red() else self.BLUE_HUB

    def _get_alliance_hub_tags(self) -> frozenset[int]:
        return self.RED_HUB_TAGS if self._is_red() else self.BLUE_HUB_TAGS

    def is_finished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self._drivetrain.set_control(self._idle_request)
