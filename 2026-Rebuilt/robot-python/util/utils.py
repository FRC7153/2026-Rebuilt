"""Misc. helpers (port of Utils.java)."""

import math
import traceback
from typing import Callable, TypeVar

from commands2 import Command
from wpilib import DriverStation, Timer
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds

from constants import AprilTagConstants
from libs import elastic
from util import console_logger

T = TypeVar("T")


def get_current_stack_trace() -> str:
    """The caller of the function that called this, as a string."""
    stack = traceback.extract_stack()
    # [-1] is this function, [-2] its caller, [-3] the caller's caller
    frame = stack[-3] if len(stack) >= 3 else stack[0]
    return f"{frame.filename}:{frame.lineno} in {frame.name}"


def is_red_alliance(warn_if_fails: bool = True) -> bool:
    """
    :param warn_if_fails: If a warning should be reported when the alliance is unknown.
    :returns: If the current alliance is red alliance (False if unknown).
    """
    alliance = DriverStation.getAlliance()
    if alliance is None:
        if warn_if_fails:
            # Attempted to check alliance before it was received!
            trace = get_current_stack_trace()
            console_logger.report_error("Invalid alliance received in " + trace)
            elastic.send_notification(
                elastic.Notification(
                    elastic.NotificationLevel.WARNING,
                    "Invalid alliance received from FMS",
                    trace,
                )
            )
        return False
    return alliance == DriverStation.Alliance.kRed


def apply_deadband(value: float, minimum: float) -> float:
    """0 if abs(value) is less than minimum, else value."""
    return 0.0 if abs(value) < minimum else value


def deadband_chassis_speeds(speeds: ChassisSpeeds, xy_min: float, omega_min: float) -> ChassisSpeeds:
    """Returns the speeds with translational (m/s) and rotational (rad/s) deadbands applied."""
    return ChassisSpeeds(
        apply_deadband(speeds.vx, xy_min),
        apply_deadband(speeds.vy, xy_min),
        apply_deadband(speeds.omega, omega_min),
    )


def is_pose2d_nan(pose: Pose2d) -> bool:
    return math.isnan(pose.X()) or math.isnan(pose.Y()) or math.isnan(pose.rotation().radians())


def get_tag_pose(tag: int) -> Translation2d:
    """The 2d position of the given AprilTag on the field."""
    pose = AprilTagConstants.FIELD.getTagPose(tag)
    if pose is None:
        console_logger.report_warning(f"Unknown tag request (id {tag})")
        return Translation2d()
    return Translation2d(pose.X(), pose.Y())


def time_instantiation(constructor: Callable[[], T]) -> T:
    """Times and prints how long it takes to instantiate an object."""
    start = Timer.getFPGATimestamp()
    obj = constructor()
    elapsed = Timer.getFPGATimestamp() - start
    print(f"{type(obj).__name__} took {elapsed:.4f} seconds to instantiate")
    return obj


def possibly_repeated_command(command: Command, repeated: bool) -> Command:
    """command.repeatedly() if repeated, else just command."""
    return command.repeatedly() if repeated else command


def get_angle_difference_degrees(a: Rotation2d, b: Rotation2d) -> float:
    """Minimum difference between a and b, in degrees."""
    diff = abs(a.degrees() - b.degrees()) % 360.0
    return diff if diff < 180.0 else 360.0 - diff
