"""AprilTag positions for the field.

WPILib 2027 (Python) no longer ships AprilTagFieldLayout, so the tag poses are read
from the Limelight field map in deploy/apriltagfiles. That file uses a field-center
origin; it is converted here to the WPILib origin (blue alliance corner).
"""

import json
import math
import os

from wpimath import Pose3d, Rotation3d, Translation3d

FIELD_LENGTH = 16.541  # m
FIELD_WIDTH = 8.069  # m
FMAP_FILE = os.path.join("apriltagfiles", "FRC2026_ANDYMARK.fmap")


class FieldTags:
    def __init__(self, path: str | None = None) -> None:
        path = path or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "deploy", FMAP_FILE)
        self._tags: dict[int, Pose3d] = {}
        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
        except OSError:
            print(f"[FieldTags] Could not read {path}; no AprilTag poses available")
            return
        for tag in data["fiducials"]:
            t = tag["transform"]  # 4x4 row-major
            yaw = math.atan2(t[4], t[0])
            self._tags[int(tag["id"])] = Pose3d(
                Translation3d(t[3] + FIELD_LENGTH / 2, t[7] + FIELD_WIDTH / 2, t[11]),
                Rotation3d(0.0, 0.0, yaw),
            )

    def get_tag_pose(self, tag_id: int) -> Pose3d | None:
        return self._tags.get(tag_id)
