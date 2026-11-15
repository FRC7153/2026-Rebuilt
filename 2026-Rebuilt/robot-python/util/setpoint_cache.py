"""Avoids re-sending an unchanged setpoint to a motor controller every loop.

Motor controllers hold their last setpoint, so sending the same value 50 times a
second only costs CPU time (which is precious for Python on a roboRIO). The value
is still refreshed periodically in case a controller rebooted and lost it.
"""

from wpilib import Timer


class SetpointCache:
    def __init__(self, refresh_period: float = 0.1) -> None:
        """
        :param refresh_period: Seconds after which an unchanged setpoint is sent again anyway.
        """
        self._refresh_period = refresh_period
        self._key = None
        self._time = 0.0

    def should_send(self, key) -> bool:
        """True if this setpoint differs from the last one sent, or is due for a refresh."""
        now = Timer.getFPGATimestamp()
        if key == self._key and 0.0 <= now - self._time < self._refresh_period:
            return False
        self._key = key
        self._time = now
        return True

    def invalidate(self) -> None:
        """Call after commanding the motor some other way, so the next setpoint is always sent."""
        self._key = None
