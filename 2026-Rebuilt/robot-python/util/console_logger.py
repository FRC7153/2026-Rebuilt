"""Reports warnings/errors to the driver station and the WPILog 'messages' entry.

(Plain console output is already captured by DataLogManager.log_console_output.)
"""

import ntcore
import wpilib
from wpilog import StringLogEntry

from constants import BuildConstants

_out_log: StringLogEntry | None = None
_out_pub: ntcore.StringPublisher | None = None


def init() -> None:
    """Starts logging warnings/errors to the WPILog file."""
    global _out_log, _out_pub
    _out_log = StringLogEntry(wpilib.DataLogManager.get_log(), "messages")
    if BuildConstants.PUBLISH_EVERYTHING:
        _out_pub = ntcore.NetworkTableInstance.get_default().get_string_topic("ConsoleOut").publish()


def _record(msg: str) -> None:
    if _out_log is not None:
        _out_log.append(msg)
    if _out_pub is not None:
        _out_pub.set(msg)


def report_warning(msg: str) -> None:
    """Reports a warning to the driver station and the log."""
    _record("[WARNING] " + msg)
    wpilib.report_warning(msg, False)


def report_error(msg: str) -> None:
    """Reports an error to the driver station and the log."""
    _record("[ERROR] " + msg)
    wpilib.report_error(msg, False)
