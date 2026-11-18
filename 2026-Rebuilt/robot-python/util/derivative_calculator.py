"""Calculates the derivative of a value with respect to time."""

from wpilib import Timer
from wpimath import LinearFilter


class DerivativeCalculator:
    def __init__(self, filter_taps: int):
        """
        :param filter_taps: Number of taps for the moving average that smooths the derivative.
        """
        self._filter = LinearFilter.moving_average(filter_taps)
        self._last_value = 0.0
        self._last_ts = 0.0
        self._first_run = True

    def calculate(self, new_value: float) -> float:
        """
        :param new_value: The next value.
        :returns: The (smoothed) derivative. 0.0 on the first run.
        """
        ts = Timer.get_timestamp()

        if not self._first_run:
            dt = ts - self._last_ts
            if dt > 0.0:
                self._filter.calculate((new_value - self._last_value) / dt)
        else:
            self._first_run = False

        self._last_value = new_value
        self._last_ts = ts

        return self._filter.last_value()

    def last_value(self) -> float:
        """The last derivative calculated."""
        return self._filter.last_value()
