"""Minimal Python port of the parts of LimelightHelpers this robot uses.

Everything talks to the Limelight over NetworkTables, exactly like the Java
library does.
"""

from dataclasses import dataclass, field

import ntcore
from wpimath import Pose2d, Rotation2d

_subscribers: dict[tuple[str, str], ntcore.DoubleArraySubscriber] = {}
_orientation_publishers: dict[str, ntcore.DoubleArrayPublisher] = {}


def _sanitize_name(name: str | None) -> str:
    return name if name else "limelight"


def _table(limelight_name: str) -> ntcore.NetworkTable:
    return ntcore.NetworkTableInstance.get_default().get_table(_sanitize_name(limelight_name))


def _array_subscriber(limelight_name: str, entry_name: str) -> ntcore.DoubleArraySubscriber:
    key = (_sanitize_name(limelight_name), entry_name)
    sub = _subscribers.get(key)
    if sub is None:
        sub = _table(limelight_name).get_double_array_topic(entry_name).subscribe([])
        _subscribers[key] = sub
    return sub


@dataclass
class RawFiducial:
    id: int = 0
    txnc: float = 0.0
    tync: float = 0.0
    ta: float = 0.0
    dist_to_camera: float = 0.0
    dist_to_robot: float = 0.0
    ambiguity: float = 0.0


@dataclass
class PoseEstimate:
    pose: Pose2d = field(default_factory=Pose2d)
    timestamp_seconds: float = 0.0
    latency: float = 0.0
    tag_count: int = 0
    tag_span: float = 0.0
    avg_tag_dist: float = 0.0
    avg_tag_area: float = 0.0
    is_mega_tag2: bool = False


def get_ty(limelight_name: str) -> float:
    """Vertical offset from crosshair to target, in degrees."""
    return _table(limelight_name).get_entry("ty").get_double(0.0)


def get_tx(limelight_name: str) -> float:
    """Horizontal offset from crosshair to target, in degrees."""
    return _table(limelight_name).get_entry("tx").get_double(0.0)


def get_target_count(limelight_name: str) -> int:
    """Number of targets currently seen (from the t2d array)."""
    t2d = _array_subscriber(limelight_name, "t2d").get()
    if len(t2d) == 17:
        return int(t2d[1])
    return 0


def set_robot_orientation(
    limelight_name: str,
    yaw: float,
    yaw_rate: float = 0.0,
    pitch: float = 0.0,
    pitch_rate: float = 0.0,
    roll: float = 0.0,
    roll_rate: float = 0.0,
    flush: bool = True,
) -> None:
    """Sends the robot orientation (degrees) used by MegaTag2."""
    key = _sanitize_name(limelight_name)
    pub = _orientation_publishers.get(key)
    if pub is None:
        pub = _table(limelight_name).get_double_array_topic("robot_orientation_set").publish()
        _orientation_publishers[key] = pub
    pub.set([yaw, yaw_rate, pitch, pitch_rate, roll, roll_rate])
    if flush:
        ntcore.NetworkTableInstance.get_default().flush()


def get_raw_fiducials(limelight_name: str) -> list[RawFiducial]:
    """All AprilTags currently seen, with raw (uncorrected) offsets."""
    raw = _array_subscriber(limelight_name, "rawfiducials").get()
    vals_per_entry = 7
    if len(raw) == 0 or len(raw) % vals_per_entry != 0:
        return []

    fiducials = []
    for i in range(len(raw) // vals_per_entry):
        base = i * vals_per_entry
        fiducials.append(
            RawFiducial(
                id=int(raw[base]),
                txnc=raw[base + 1],
                tync=raw[base + 2],
                ta=raw[base + 3],
                dist_to_camera=raw[base + 4],
                dist_to_robot=raw[base + 5],
                ambiguity=raw[base + 6],
            )
        )
    return fiducials


def _get_bot_pose_estimate(limelight_name: str, entry_name: str, is_mega_tag2: bool) -> PoseEstimate | None:
    atomic = _array_subscriber(limelight_name, entry_name).get_atomic()
    pose_array = atomic.value
    if len(pose_array) < 11:
        return None

    pose = Pose2d(pose_array[0], pose_array[1], Rotation2d.from_degrees(pose_array[5]))
    latency = pose_array[6]

    # Convert server timestamp from microseconds to seconds and adjust for latency
    timestamp = (atomic.time / 1_000_000.0) - (latency / 1000.0)

    return PoseEstimate(
        pose=pose,
        timestamp_seconds=timestamp,
        latency=latency,
        tag_count=int(pose_array[7]),
        tag_span=pose_array[8],
        avg_tag_dist=pose_array[9],
        avg_tag_area=pose_array[10],
        is_mega_tag2=is_mega_tag2,
    )


def get_bot_pose_estimate_wpi_blue_mega_tag2(limelight_name: str) -> PoseEstimate | None:
    """MegaTag2 pose estimate in the WPILib blue-origin field frame (or None)."""
    return _get_bot_pose_estimate(limelight_name, "botpose_orb_wpiblue", True)


def get_bot_pose_estimate_wpi_blue(limelight_name: str) -> PoseEstimate | None:
    """MegaTag1 pose estimate in the WPILib blue-origin field frame (or None)."""
    return _get_bot_pose_estimate(limelight_name, "botpose_wpiblue", False)
