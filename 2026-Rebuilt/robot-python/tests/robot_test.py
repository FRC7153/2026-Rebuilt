"""SystemCore (2027) tests.

pyfrc has no 2027 release that installs alongside WPILib 2027 yet, so these tests
drive the robot directly with the WPILib simulation API instead of pyfrc fixtures.
Run with: robotpy test
"""

import os
import sys

import hal
import pytest
import wpilib.simulation as sim

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

DT = 0.02


@pytest.fixture(scope="module")
def robot():
    hal.initialize()
    sim.pause_timing()
    from robot import Robot

    r = Robot()
    sim.DriverStationSim.set_ds_attached(True)
    yield r


@pytest.fixture(scope="module")
def pad():
    p = sim.XboxControllerSim(0)
    p.set_axes_available(6)
    p.set_buttons_available(16)
    p.set_povs_available(1)
    return p


def run(robot, seconds: float) -> None:
    for _ in range(int(seconds / DT)):
        sim.DriverStationSim.notify_new_data()
        sim.step_timing(DT)
        robot._loop_func()


def set_mode(mode, enabled: bool) -> None:
    sim.DriverStationSim.set_robot_mode(mode)
    sim.DriverStationSim.set_enabled(enabled)
    sim.DriverStationSim.notify_new_data()


def test_disabled_and_auto(robot):
    set_mode(hal.RobotMode.TELEOPERATED, False)
    run(robot, 1.0)
    set_mode(hal.RobotMode.AUTONOMOUS, True)
    run(robot, 2.0)


def test_teleop_drives_forward(robot, pad):
    drivetrain = robot._robot_container.drivetrain
    set_mode(hal.RobotMode.TELEOPERATED, True)
    run(robot, 0.5)
    start = drivetrain.get_state().pose.x
    pad.set_left_y(-1.0)
    run(robot, 1.5)
    pad.set_left_y(0.0)
    assert drivetrain.get_state().pose.x - start > 1.0


@pytest.mark.parametrize(
    "button",
    ["a_button", "b_button", "x_button", "y_button", "left_bumper_button", "right_bumper_button", "dpad_up_button"],
)
def test_every_button(robot, pad, button):
    set_mode(hal.RobotMode.TELEOPERATED, True)
    getattr(pad, f"set_{button}")(True)
    run(robot, 0.5)
    getattr(pad, f"set_{button}")(False)
    run(robot, 0.2)


@pytest.mark.parametrize("trigger", ["left_trigger", "right_trigger"])
def test_triggers(robot, pad, trigger):
    set_mode(hal.RobotMode.TELEOPERATED, True)
    getattr(pad, f"set_{trigger}")(1.0)
    run(robot, 1.0)
    getattr(pad, f"set_{trigger}")(0.0)
    run(robot, 0.2)


def test_utility_and_disable(robot):
    set_mode(hal.RobotMode.UTILITY, True)
    run(robot, 0.5)
    set_mode(hal.RobotMode.TELEOPERATED, False)
    run(robot, 0.5)


def test_periodic_logging(robot):
    # These run from add_periodic() on the robot, which the loop stepping above skips
    for _ in range(3):
        robot._robot_container.log()
        robot._robot_container.check_hardware()
        robot._can_logger.log()
        run(robot, 0.1)
