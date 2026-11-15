"""Logs the status of multiple CAN busses."""

import wpilib
from phoenix6 import CANBus
from wpiutil.log import IntegerLogEntry


class CANLogger:
    """Call log() periodically (the robot does so twice a second)."""

    def __init__(self, *busses: CANBus):
        self._busses = busses
        self._logs: list[tuple[IntegerLogEntry, IntegerLogEntry, IntegerLogEntry, IntegerLogEntry]] = []

        log = wpilib.DataLogManager.getLog()
        for bus in busses:
            title = f"CAN/{bus.name}/"
            self._logs.append(
                (
                    IntegerLogEntry(log, title + "Utilization"),
                    IntegerLogEntry(log, title + "TEC"),  # transmit error count
                    IntegerLogEntry(log, title + "REC"),  # receive error count
                    IntegerLogEntry(log, title + "TX_Full"),
                )
            )

    def log(self) -> None:
        for bus, (utilization, tec, rec, tx_full) in zip(self._busses, self._logs):
            status = bus.get_status()
            utilization.append(int(status.bus_utilization * 100))
            tec.append(int(status.tec))
            rec.append(int(status.rec))
            tx_full.append(int(status.tx_full_count))
