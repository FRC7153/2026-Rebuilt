"""Limelight health monitoring (heartbeat + hardware stats)."""

from enum import Enum

import ntcore
import wpilib
from wpilib import Alert, Timer
from wpiutil.log import BooleanLogEntry, DoubleLogEntry

from util import hardware_fault_tracker


class Version(Enum):
    """(has integrated IMU, has fans)"""

    LIMELIGHT_3G = (False, False)
    LIMELIGHT_4 = (True, False)

    @property
    def integrated_imu(self) -> bool:
        return self.value[0]

    @property
    def has_fans(self) -> bool:
        return self.value[1]


class Limelight:
    def __init__(self, name: str, version: Version) -> None:
        """
        :param name: NetworkTables name (hostname) of the camera.
        :param version: Hardware version of the camera.
        """
        self.camera_name = name
        self.version = version

        self._not_connected_alert = Alert(f"Limelight {name} is not alive", Alert.AlertType.kWarning)

        # NT topics
        camera_table = ntcore.NetworkTableInstance.getDefault().getTable(name)
        self._heartbeat_sub = camera_table.getDoubleTopic("hb").subscribe(-1.0)
        self._stats_sub = camera_table.getDoubleArrayTopic("hw").subscribe([0.0, 0.0, 0.0, 0.0])
        self._pipeline_pub = camera_table.getDoubleTopic("pipeline").publish()

        # Stats
        self._last_heartbeat = -1.0
        self._last_heartbeat_ts = 0.0
        self._alive = False

        # Logging
        log = wpilib.DataLogManager.getLog()
        log_name = f"Limelight/{name}/"
        self._fps_log = DoubleLogEntry(log, log_name + "fps")
        self._cpu_temp_log = DoubleLogEntry(log, log_name + "cpuTemp", "f")
        self._ram_log = DoubleLogEntry(log, log_name + "ram", "%")
        self._temp_log = DoubleLogEntry(log, log_name + "temp", "f")
        self._is_alive_log = BooleanLogEntry(log, log_name + "isAlive")

    def set_pipeline(self, pipeline: int) -> None:
        self._pipeline_pub.set(pipeline)

    def log(self) -> None:
        """Logs diagnostics data."""
        stats = self._stats_sub.get()

        if self._alive and len(stats) == 4:
            self._fps_log.append(stats[0])
            self._cpu_temp_log.append(stats[1])
            self._ram_log.append(stats[2])
            self._temp_log.append(stats[3])

    def check_hardware(self) -> None:
        """Checks if this Limelight is alive."""
        new_heartbeat = self._heartbeat_sub.get()
        now = Timer.getFPGATimestamp()

        if new_heartbeat == -1.0:
            # No heartbeat
            self._alive = False
        elif new_heartbeat != self._last_heartbeat:
            self._last_heartbeat = new_heartbeat
            self._last_heartbeat_ts = now
            self._alive = True
        elif now - self._last_heartbeat_ts > 1.5:
            # Heartbeat timeout
            self._alive = False

        hardware_fault_tracker.check_fault(self._not_connected_alert, not self._alive)
        self._is_alive_log.append(self._alive)

    def is_alive(self) -> bool:
        """If the Limelight is returning a heartbeat."""
        return self._alive

    def get_version(self) -> Version:
        return self.version
