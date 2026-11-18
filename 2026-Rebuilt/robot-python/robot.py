"""Robot entry point (port of Robot.java / Main.java).

Deploy with:   py -m robotpy deploy
Simulate with: py -m robotpy sim
"""

import gc
import os

import commands2
import ntcore
import wpilib
from phoenix6 import SignalLogger

from constants import HardwareConstants
from robotcontainer import RobotContainer
from util import console_logger, hardware_fault_tracker, utils
from util.can_logger import CANLogger


class Robot(wpilib.TimedRobot):
    def __init__(self) -> None:
        # 2027: robotInit() is gone, setup happens in the constructor
        super().__init__()
        self._autonomous_command: commands2.Command | None = None

        print(f"Built with WPILib/RobotPy {wpilib.__version__}")

        # Configure CTRE SignalLogger
        SignalLogger.enable_auto_logging(False)
        SignalLogger.stop()
        if wpilib.RobotBase.is_real():
            pass  # SystemCore: keep the CTRE default log location
        else:
            # In simulation, keep all logs in a hidden folder so that they are
            # never uploaded to the roboRIO by `robotpy deploy`.
            sim_log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".simlogs")
            os.makedirs(sim_log_dir, exist_ok=True)
            SignalLogger.set_path(sim_log_dir)
            wpilib.DataLogManager.start(sim_log_dir)

        # Init logging
        wpilib.DataLogManager.log_network_tables(True)
        wpilib.DataLogManager.log_console_output(True)
        console_logger.init()

        wpilib.DriverStation.start_data_log(wpilib.DataLogManager.get_log())
        ntcore.NetworkTableInstance.get_default().start_connection_data_log(
            wpilib.DataLogManager.get_log(), "NTConnections"
        )

        # Init CAN logging
        self._can_logger = CANLogger(HardwareConstants.RIO_CAN, HardwareConstants.CANIVORE)

        # Init robot base
        self._robot_container = utils.time_instantiation(RobotContainer)

        # Add logging periodic
        self.add_periodic(self._robot_container.log, 0.1, 0.001)  # every 100 ms
        self.add_periodic(self._robot_container.check_hardware, 0.5, 0.001)  # every 500 ms
        self.add_periodic(self._can_logger.log, 0.5, 0.003)  # every 500 ms

        # Init hardware fault tracker
        hardware_fault_tracker.robot_program_has_started()

        # Everything created so far lives for the whole match. Moving it out of the
        # garbage collector's reach keeps later collections short (no mid-match hitches).
        gc.collect()
        gc.freeze()

        print("Robot constructor finished")

    def robot_periodic(self) -> None:
        commands2.CommandScheduler.get_instance().run()
        self._robot_container.periodic()

    def disabled_init(self) -> None:
        print("DISABLED mode set")

    def disabled_periodic(self) -> None:
        # Build the selected auto ahead of time. Building parses the path files,
        # which is far too slow to do at the start of the autonomous period.
        if not self._robot_container.is_auto_loaded():
            self._robot_container.pregame_load()
            gc.collect()
            gc.freeze()

    def disabled_exit(self) -> None:
        pass

    def autonomous_init(self) -> None:
        print("AUTONOMOUS mode set")

        # Make sure the auto is loaded (no-op if disabled_periodic already did it)
        self._robot_container.pregame_load()

        # Run auto command
        self._autonomous_command = self._robot_container.get_autonomous_command()

        if self._autonomous_command is not None:
            commands2.CommandScheduler.get_instance().schedule(self._autonomous_command)

    def autonomous_periodic(self) -> None:
        pass

    def autonomous_exit(self) -> None:
        if self._autonomous_command is not None:
            self._autonomous_command.cancel()
            self._autonomous_command = None

    def teleop_init(self) -> None:
        # Make sure nothing from autonomous can keep hold of a subsystem
        if self._autonomous_command is not None:
            self._autonomous_command.cancel()
            self._autonomous_command = None
        commands2.CommandScheduler.get_instance().cancel_all()
        self._robot_container.teleop_started()
        print("TELEOP mode set")

    def teleop_periodic(self) -> None:
        pass

    def teleop_exit(self) -> None:
        self._robot_container.teleop_ended()

    # 2027: "test" mode is now called "utility"
    def utility_init(self) -> None:
        commands2.CommandScheduler.get_instance().cancel_all()

    def utility_periodic(self) -> None:
        pass

    def utility_exit(self) -> None:
        pass
