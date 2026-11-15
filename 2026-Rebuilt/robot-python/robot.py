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
    def robotInit(self) -> None:
        self._autonomous_command: commands2.Command | None = None

        print(f"Built with WPILib/RobotPy {wpilib.__version__}")

        # Configure CTRE SignalLogger
        SignalLogger.enable_auto_logging(False)
        SignalLogger.stop()
        if wpilib.RobotBase.isReal():
            SignalLogger.set_path("/u/CTRE_Signal_Logger")
        else:
            # In simulation, keep all logs in a hidden folder so that they are
            # never uploaded to the roboRIO by `robotpy deploy`.
            sim_log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".simlogs")
            os.makedirs(sim_log_dir, exist_ok=True)
            SignalLogger.set_path(sim_log_dir)
            wpilib.DataLogManager.start(sim_log_dir)

        # Init logging
        wpilib.DataLogManager.logNetworkTables(True)
        wpilib.DataLogManager.logConsoleOutput(True)
        console_logger.init()

        wpilib.DriverStation.startDataLog(wpilib.DataLogManager.getLog(), True)
        ntcore.NetworkTableInstance.getDefault().startConnectionDataLog(
            wpilib.DataLogManager.getLog(), "NTConnections"
        )

        # Init CAN logging
        self._can_logger = CANLogger(HardwareConstants.RIO_CAN, HardwareConstants.CANIVORE)

        # Init robot base
        self._robot_container = utils.time_instantiation(RobotContainer)

        # Add logging periodic
        self.addPeriodic(self._robot_container.log, 0.1, 0.001)  # every 100 ms
        self.addPeriodic(self._robot_container.check_hardware, 0.5, 0.001)  # every 500 ms
        self.addPeriodic(self._can_logger.log, 0.5, 0.003)  # every 500 ms

        # Init hardware fault tracker
        hardware_fault_tracker.robot_program_has_started()

        # Everything created so far lives for the whole match. Moving it out of the
        # garbage collector's reach keeps later collections short (no mid-match hitches).
        gc.collect()
        gc.freeze()

        print("Robot constructor finished")

    def robotPeriodic(self) -> None:
        commands2.CommandScheduler.getInstance().run()
        self._robot_container.periodic()

    def disabledInit(self) -> None:
        print("DISABLED mode set")

    def disabledPeriodic(self) -> None:
        # Build the selected auto ahead of time. Building parses the path files,
        # which is far too slow to do at the start of the autonomous period.
        if not self._robot_container.is_auto_loaded():
            self._robot_container.pregame_load()
            gc.collect()
            gc.freeze()

    def disabledExit(self) -> None:
        pass

    def autonomousInit(self) -> None:
        print("AUTONOMOUS mode set")

        # Make sure the auto is loaded (no-op if disabledPeriodic already did it)
        self._robot_container.pregame_load()

        # Run auto command
        self._autonomous_command = self._robot_container.get_autonomous_command()

        if self._autonomous_command is not None:
            commands2.CommandScheduler.getInstance().schedule(self._autonomous_command)

    def autonomousPeriodic(self) -> None:
        pass

    def autonomousExit(self) -> None:
        if self._autonomous_command is not None:
            self._autonomous_command.cancel()
            self._autonomous_command = None

    def teleopInit(self) -> None:
        # Make sure nothing from autonomous can keep hold of a subsystem
        if self._autonomous_command is not None:
            self._autonomous_command.cancel()
            self._autonomous_command = None
        commands2.CommandScheduler.getInstance().cancelAll()
        self._robot_container.teleop_started()
        print("TELEOP mode set")

    def teleopPeriodic(self) -> None:
        pass

    def teleopExit(self) -> None:
        self._robot_container.teleop_ended()

    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self) -> None:
        pass

    def testExit(self) -> None:
        pass
